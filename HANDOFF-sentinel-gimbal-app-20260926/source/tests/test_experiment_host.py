"""Offline Profile4/client/CLI regressions. Never opens a device."""
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

HOST = Path(__file__).resolve().parents[1] / 'robots/sentinel/host'
sys.path.insert(0, str(HOST))
import sentinel_client as client
import sentinel_raw as raw
import autotune_dataset as dataset


class Profile4Tests(unittest.TestCase):
    def record(self, kind, payload, stream=6, sequence=0):
        return struct.pack('<4B7I', 1, kind, stream, 0, sequence, 9, 100, 101, 0, 0, 17) + payload

    def test_profile4_lossless_roundtrip_and_unknown(self):
        epoch = 0xf1234567
        state = self.record(6, struct.pack('<I6fI', 0x3101004f, .1, .2, .3, .4, float('nan'), .001, epoch))
        aux = self.record(7, struct.pack('<5I3f', 1 | (1 << 16), 8, 0, 0, 1 | (255 << 8), 70, 2, float('nan')), sequence=1)
        raw.clear_trace_decoders()
        row = raw.decode_record(state)
        self.assertEqual(row['param_epoch_raw'], epoch)
        self.assertIsNone(row['output_normalized'])
        self.assertEqual(row['target_source_id'], 3)
        other = raw.decode_record(aux)
        self.assertEqual(other['provenance'], 1)
        self.assertIsNone(other['dm_state'])
        self.assertIsNone(other['torque_nm'])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'trace.sraw'
            raw.save(path, {'layout_id': raw.LAYOUT, 'run': 17, 'start_us': 100}, [state, aux])
            self.assertTrue(path.read_bytes().endswith(state + aux))
            _, rows = raw.load(path)
            self.assertEqual(len(rows), 2)
            exported = Path(tmp) / 'trace.csv'
            raw.export_csv(path, exported)
            self.assertIn('param_epoch_raw', exported.read_text())
            self.assertIn('torque_nm', exported.read_text())
        self.assertEqual(raw.PROFILES['autotune'], 4)
        for invalid in (state[:-1], self.record(6, state[32:], stream=1), bytes([2])+state[1:], bytes([1, 99])+state[2:]):
            with self.assertRaises(ValueError):
                raw.decode_record(invalid)

    def test_legacy_pitch_kind5_preserved(self):
        record = self.record(5, struct.pack('<6f2I', .1, .2, .3, .4, .5, .001, (1 << 29) | (1 << 30), 0))
        row = raw.decode_record(record)
        self.assertAlmostEqual(row['pitch_feedforward_torque_nm'], .5)
        self.assertEqual(row['units']['pitch_feedforward_torque_nm'], 'N*m')
        self.assertTrue(row['excitation_active'])
        self.assertTrue(row['latched'])


class ClientTests(unittest.TestCase):
    def test_command_framing_and_default_hardware_blocked(self):
        class Fake:
            sent = None
            def exchange(self, wire):
                decoder = client.FrameDecoder()
                request = None
                for byte in wire:
                    value = decoder.push(byte)
                    if value is not None:
                        request = value
                self.sent = request
                kind, sequence, body = request
                response = body[:1] + bytes([client.EXPERIMENT_SNAPSHOT]) + body[2:10]
                response += struct.pack('<8B6I', 1, body[11], 0, 1, 0, 0, 1, 1, 0, 0, 0, 100, 0, 0)
                return client.EXPERIMENT_SNAPSHOT, sequence, response
        transport = Fake()
        connection = client.SentinelClient(transport, session=7)
        command = struct.pack('<4BII', 1, 2, 5, 1, 0, 0)
        out = connection.experiment(command)
        self.assertEqual(transport.sent[0], client.EXPERIMENT)
        self.assertEqual(transport.sent[2], bytes([3, 0x60]) + struct.pack('<II', 7, 1) + command)
        self.assertTrue(out['hardware_blocked'])
        self.assertFalse(out['physical_stop_confirmed'])
        for op in (4, 5):
            with self.assertRaisesRegex(client.ProtocolError, 'hardware_blocked'):
                connection.experiment(struct.pack('<4BII', 1, op, 5, 1, 17, 31))
        with self.assertRaises(ValueError):
            client.encode_experiment(7, 1, b'bad')
        with self.assertRaises(client.ProtocolError):
            client.decode_experiment_snapshot(b'bad')


class CliTests(unittest.TestCase):
    def call(self, *args, env=None):
        return subprocess.run([sys.executable, '-S', str(HOST / 'sentinel_experiment.py'), *map(str, args)],
                              capture_output=True, text=True, timeout=60, env=env)

    def test_status_stop_and_live_rejection(self):
        status = self.call('status')
        self.assertEqual(status.returncode, 0, status.stderr)
        out = json.loads(status.stdout)
        self.assertFalse(out['hardware_enabled'])
        self.assertTrue(out['hardware_blocked'])
        self.assertTrue(out['simulation'])
        stop = self.call('stop', '--session-id', '17')
        self.assertEqual(stop.returncode, 2)
        self.assertEqual(json.loads(stop.stdout)['status'], 'STOP_NOT_CONFIRMED')
        for args in [('run', '--port', '/dev/ttyACM0'), ('run', '--enable'),
                     ('analyze', '--input', '/dev/null', '--output', 'unused', '--read-only')]:
            self.assertNotEqual(self.call(*args).returncode, 0)

    def test_native_simulation_and_read_only_replay(self):
        backend = os.environ.get('SENTINEL_OFFLINE_BACKEND')
        if not backend:
            # Repo policy: a missing native backend is a failure, not a skip
            # (ctest sets SENTINEL_OFFLINE_BACKEND for this contract).
            self.fail('SENTINEL_OFFLINE_BACKEND is not set; build '
                      'ec_sentinel_experiment_backend or run this suite via ctest')
        for axis in ('fire_yaw', 'pitch'):
            for pattern in ('bounded_step', 'trapezoid'):
                with self.subTest(axis=axis, pattern=pattern), tempfile.TemporaryDirectory() as tmp:
                    out = Path(tmp) / 'run'
                    result = self.call('run', '--axis', axis, '--pattern', pattern, '--output', out)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertTrue(json.loads((out / 'manifest.json').read_text())['synthetic'])
                    analysis = json.loads((out / 'analysis.json').read_text())
                    self.assertFalse(analysis['hardware_validated'])
                    self.assertFalse(analysis['candidate_promotable'])
                    self.assertIsNone(analysis['candidate'])
                    if axis == 'pitch':
                        self.assertEqual(analysis['score']['metrics']['U']['status'], 'UNAVAILABLE')
                        self.assertEqual(analysis['score']['metrics']['S']['status'], 'UNAVAILABLE')
                        self.assertIsNone(analysis['score']['L'])
                    replay = Path(tmp) / 'replay'
                    result = self.call('analyze', '--input', out, '--output', replay, '--read-only')
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual((out / 'analysis.json').read_bytes(), (replay / 'analysis.json').read_bytes())
                    self.assertNotEqual(self.call('run', '--axis', axis, '--output', out).returncode, 0)
                    for path in out.glob('*.json'):
                        json.loads(path.read_text(), parse_constant=lambda value: self.fail(value))


if __name__ == '__main__':
    unittest.main()
