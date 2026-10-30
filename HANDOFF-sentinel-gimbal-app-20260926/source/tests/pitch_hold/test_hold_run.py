"""Runner/keyboard/logger tests. Every process is software-only."""
import json
import os
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import Mock, call, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robots/sentinel/host'))
import sentinel_pitch_hold as run

SCRIPT = run.SOURCE / 'robots/sentinel/host/sentinel_pitch_hold.py'
BACKEND = Path(os.environ.get('SENTINEL_PITCH_BACKEND', str(run.SOURCE / 'build/host-tests/ec_sentinel_app_host')))


class RunnerTests(unittest.TestCase):
    def command(self, output, *extra):
        return [sys.executable, str(SCRIPT), '--backend', str(BACKEND), '--output', str(output), '--arm', *extra]

    def test_refuses_unarmed_or_unsafe_serial_before_open(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(run, 'ExclusiveSerial') as transport:
            output = str(Path(folder) / 'not-written.jsonl')
            for args in [ ['--serial', '/not-a-device', '--output', output],
                          ['--serial', '/not-a-device', '--output', output, '--arm'],
                          ['--backend', '/not-a-backend', '--output', output, '--arm', '--duration', 'nan'] ]:
                with self.subTest(args=args), self.assertRaises(SystemExit):
                    run.main(args)
            transport.assert_not_called()

    def finished(self, value):
        future = Future()
        future.set_result(value)
        return future

    def test_failed_preflight_sends_no_mutation(self):
        for failure in ('identity', 'topology'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as folder:
                peer = Mock()
                peer.hello.return_value = self.finished({'identity': 'wrong' if failure == 'identity' else 'sentinel_app',
                                                        'startable': True, 'config_kind': 2})
                peer.diagnostics.return_value = self.finished({'devices': [{'domain': 'pitch', 'bus': 1}]})
                output = Path(folder) / 'preflight.jsonl'
                args = ['--serial', '/not-a-device', '--arm', '--support-ready', '--accept-release',
                        '--physical-low', '-0.3', '--physical-high', '0.4', '--max-temperature', '55',
                        '--max-torque', '2', '--max-speed', '0.6', '--max-error', '0.03',
                        '--duration', '1', '--output', str(output)]
                with patch.object(run, 'ExclusiveSerial'), patch.object(run, 'AsyncSentinelClient', return_value=peer):
                    self.assertEqual(run.main(args), 1)
                for operation in ('authorize', 'publish_targets', 'set_target_source', 'param_write'):
                    getattr(peer, operation).assert_not_called()
                peer.close.assert_called_once()
                last = json.loads(output.read_text().splitlines()[-1])
                self.assertEqual(last['stop_report']['revoke'], 'not_requested')

    def test_invalid_firmware_ttl_never_authorizes(self):
        for target, link in ((None, 200), (0, 200), (10, 200), (100, None)):
            with self.subTest(target=target, link=link), tempfile.TemporaryDirectory() as folder:
                peer = Mock()
                peer.hello.return_value = self.finished({'identity': 'sentinel_app', 'startable': True, 'session': 1})
                peer.diagnostics.return_value = self.finished({'session': 1, 'page': 0,
                    'received_at_monotonic': time.monotonic(), 'target_ttl_ms': target, 'link_ttl_ms': link})
                with patch.object(run.wire, 'ProcessTransport'), patch.object(run, 'AsyncSentinelClient', return_value=peer):
                    result = run.main(['--backend', '/not-a-backend', '--arm', '--output', str(Path(folder) / 'ttl.jsonl')])
                self.assertEqual(result, 1)
                peer.authorize.assert_not_called()
                peer.set_target_source.assert_not_called()
                peer.close.assert_called_once()

    def test_changed_ttl_before_authorization_refuses(self):
        with tempfile.TemporaryDirectory() as folder:
            peer = Mock()
            peer.hello.return_value = self.finished({'identity': 'sentinel_app', 'startable': True, 'session': 1})
            peer.read_all_params.return_value = self.finished([])
            peer.status.return_value = self.finished({'joints': {'pitch': {'angle_rad': 0.0}}})
            peer.diagnostics.side_effect = [self.finished({'session': 1, 'page': 0,
                'received_at_monotonic': time.monotonic(), 'target_ttl_ms': ttl, 'link_ttl_ms': 200})
                for ttl in (100, 80)]
            with patch.object(run.wire, 'ProcessTransport'), patch.object(run, 'AsyncSentinelClient', return_value=peer):
                result = run.main(['--backend', '/not-a-backend', '--arm', '--output', str(Path(folder) / 'ttl.jsonl')])
            self.assertEqual(result, 1)
            peer.authorize.assert_not_called()
            peer.set_target_source.assert_not_called()
            peer.close.assert_called_once()

    def test_uncertain_authorization_attempt_still_revokes(self):
        with tempfile.TemporaryDirectory() as folder:
            peer = Mock()
            peer.hello.return_value = self.finished({'identity': 'sentinel_app', 'startable': True, 'session': 1})
            peer.read_all_params.return_value = self.finished([])
            peer.status.side_effect = [self.finished({'joints': {'pitch': {'angle_rad': 0.0}}}),
                                       self.finished({'domains': [{'execute': False}]})]
            peer.diagnostics.side_effect = [
                self.finished({'session': 1, 'page': 0, 'received_at_monotonic': time.monotonic(),
                               'target_ttl_ms': 100, 'link_ttl_ms': 200}),
                self.finished({'session': 1, 'page': 0, 'received_at_monotonic': time.monotonic(),
                               'target_ttl_ms': 100, 'link_ttl_ms': 200}),
                self.finished({'devices': [{'domain': 'pitch', 'online': True, 'age_ms': 0, 'enabled': False}]})]
            uncertain = Future()
            uncertain.set_exception(TimeoutError('authorization reply lost'))
            peer.authorize.side_effect = [uncertain, self.finished({'result': 0})]
            output = Path(folder) / 'uncertain.jsonl'
            with patch.object(run.wire, 'ProcessTransport'), patch.object(run, 'AsyncSentinelClient', return_value=peer):
                self.assertEqual(run.main(['--backend', '/not-a-backend', '--arm', '--output', str(output)]), 1)
            self.assertEqual(peer.authorize.call_args_list, [call(True, 'maintenance'), call(False, 'maintenance')])
            peer.set_target_source.assert_not_called()
            peer.param_write.assert_not_called()
            peer.close.assert_called_once()
            self.assertEqual(json.loads(output.read_text().splitlines()[-1])['stop_report']['revoke'], 'accepted')

    def test_demo_segments_and_confirmed_stop(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'demo.jsonl'
            result = subprocess.run(self.command(log, '--demo', '--duration', '0.7'), capture_output=True, text=True, timeout=8)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            rows = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual(rows[0]['event'], 'begin')
            samples = [r for r in rows if r['event'] == 'sample']
            self.assertTrue(any(r['state'] == 'HOLDING' for r in samples))
            self.assertEqual({r['segment'] for r in samples if r['state'] == 'HOLDING'}, {1, 2})
            self.assertTrue(rows[-1]['stop_report']['control_inactive'])
            # Synthetic plant hardcodes enabled=True, so don't fabricate a physical disable.
            self.assertFalse(rows[-1]['stop_report']['disabled_feedback'])
            self.assertTrue(all(abs(r['target_rad'] - rows[0]['center_rad']) <= 0.005 for r in samples if r['target_rad'] is not None))

    def test_busy_serial_is_refused_before_open(self):
        with patch.object(run, 'other_serial_owners', return_value={'12345'}), patch.object(run.wire.SerialTransport, '__init__') as opened:
            with self.assertRaisesRegex(RuntimeError, 'already open'):
                run.ExclusiveSerial('/not-a-real-device')
            opened.assert_not_called()

    def test_linux_pty_exclusive_open(self):
        # Kernel TIOCEXCL is exercised on an allocated software pseudo-terminal,
        # never on a connected board/adapter.
        import pty

        import serial
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        os.close(slave)
        owner = None
        try:
            owner = run.ExclusiveSerial(port)
            with self.assertRaises(serial.SerialException):
                serial.Serial(port)
        finally:
            if owner is not None:
                owner.close()
            os.close(master)

    def test_output_never_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'existing.jsonl'
            log.write_text('keep this')
            result = subprocess.run(self.command(log, '--demo'), capture_output=True, text=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(log.read_text(), 'keep this')

    def test_keyboard_quit_and_eof_stop(self):
        for text in ('q', ' ', '\x1b', ''):
            with self.subTest(key=repr(text)), tempfile.TemporaryDirectory() as folder:
                result = subprocess.run(self.command(Path(folder) / 'keys.jsonl', '--duration', '2'),
                                        input=text, capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
                self.assertIn('"control_inactive": true', result.stdout)

    def test_keyboard_steps_segments_and_rejected_overtravel(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'steps.jsonl'
            process = subprocess.Popen(self.command(log, '--duration', '3'), stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            rows = []
            try:
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    if log.exists() and '"state": "HOLDING"' in log.read_text():
                        break
                    time.sleep(0.005)
                else:
                    self.fail('did not reach holding')
                assert process.stdin is not None
                # Exceed each side of the envelope, and change segment without
                # ever sending a stop key between the positive/negative steps.
                process.stdin.write('++++++++++++n------------')
                process.stdin.flush()
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    text = log.read_text()
                    lines = text.splitlines()
                    if not text.endswith('\n'):
                        lines = lines[:-1]
                    rows = [json.loads(line) for line in lines]
                    center = rows[0]['center_rad']
                    if any(r.get('segment') == 1 and r.get('target_rad') is not None
                           and r['target_rad'] < center - 0.0005 for r in rows):
                        break
                    time.sleep(0.005)
                else:
                    self.fail('negative keyboard target/segment not observed')
                process.stdin.write('q')
                process.stdin.flush()
                stdout, stderr = process.communicate(timeout=4)
                self.assertEqual(process.returncode, 0, stdout + stderr)
                self.assertIn('Rejected: step outside', stdout)
                samples = [r for r in rows if r['event'] == 'sample' and r['state'] == 'HOLDING']
                self.assertTrue(any(r['target_rad'] > center + 0.0005 for r in samples))
                self.assertTrue(all(abs(r['target_rad'] - center) <= 0.005 for r in samples))
                self.assertTrue(all(r['state'] == 'HOLDING' for r in rows if r.get('segment') == 1))
            finally:
                if process.poll() is None:
                    process.kill()
                process.communicate()

    def test_sigint_ends_demo_and_flushes_log(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'interrupt.jsonl'
            process = subprocess.Popen(self.command(log, '--demo', '--duration', '4'), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    if log.exists() and '"state": "HOLDING"' in log.read_text():
                        break
                    time.sleep(0.01)
                else:
                    self.fail('did not reach holding')
                process.send_signal(signal.SIGINT)
                stdout, stderr = process.communicate(timeout=4)
                self.assertEqual(process.returncode, 0, stdout + stderr)
                self.assertEqual(json.loads(log.read_text().splitlines()[-1])['event'], 'end')
            finally:
                if process.poll() is None:
                    process.kill()
                process.communicate()

    def test_overflow_and_disk_failure_are_visible(self):
        log = run.AcquisitionLog.__new__(run.AcquisitionLog)
        log.error = ''
        log.queue = queue.Queue(maxsize=1)
        log.append({'a': 1})
        with self.assertRaisesRegex(RuntimeError, 'overflow'):
            log.append({'a': 2})
        log = run.AcquisitionLog.__new__(run.AcquisitionLog)
        log.error = ''
        log.done = threading.Event()
        log.done.set()
        log.queue = queue.Queue()
        log.queue.put('data\n')
        log._files = Mock()
        log.file = Mock()
        log.file.write.side_effect = OSError('disk full')
        log._write()
        self.assertIn('disk full', log.error)
        with self.assertRaises(RuntimeError):
            log.append({'a': 1})


if __name__ == '__main__':
    unittest.main()
