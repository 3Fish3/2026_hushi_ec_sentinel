"""只用原生控制器向量/假串口; 不打开硬件, 不把主机协议测试当作实物验收."""
from pathlib import Path
import json
import math
import queue
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import contextlib
import io

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'boards/sentinel_f407/tools'))
try:
    import motor_bench as bench
except ModuleNotFoundError as exc:
    print(f'SKIP: install project Python dependencies/Tk: {exc}')
    raise SystemExit(77)
codec = sys.argv.pop(1)
VALUES = [.1, 3, .15, 2, .04, .02, 2, .001, .25]


class FakeSerial:
    def __init__(self, body, drop_arm=False, vectors=None, drop_trace=False, delay=0):
        self.body, self.drop_arm, self.vectors = body, drop_arm, vectors
        self.drop_trace, self.delay = drop_trace, delay
        self.buffer = bytearray(); self.writes = []; self.times = []; self.thread_ids = set()
        self.closed = self.active = False
        self.token, self.run, self.ready_at = 0, 0, 0

    def reset_input_buffer(self): self.buffer.clear()

    @property
    def in_waiting(self): return len(self.buffer) if time.monotonic() >= self.ready_at else 0

    def read(self, n):
        self.thread_ids.add(threading.get_ident()); time.sleep(.001)
        if time.monotonic() < self.ready_at: return b''
        data = bytes(self.buffer[:n]); del self.buffer[:n]; return data

    def close(self): self.closed = True

    def write(self, data):
        self.thread_ids.add(threading.get_ident())
        kind, seq, payload = bench.wire.Decoder().feed(data)[0]
        self.writes.append((kind, seq, payload)); self.times.append(time.monotonic())
        if kind == 0x26:
            if self.drop_trace: return len(data)
            _, op, trigger, _, run, arg = struct.unpack('<4BII', payload)
            if op == 1: self.run = run
            if op in (0, 1, 2):
                body = bytearray(self.vectors['data0'][:32])
                struct.pack_into('<H', body, 10, 0)
                struct.pack_into('<I', body, 24, 0)
                body[3] = 1 if op == 1 else 3
                body[28] = trigger
            else:
                body = bytearray(self.vectors[f'meta{arg}' if op == 4 else f'data{arg}'])
            body[1] = op
            struct.pack_into('<I', body, 4, self.run)
            reply = 0xa1
        else:
            if kind == 0x21: self.active = True; self.token = seq
            if kind == 0x23: self.active = False
            body = bytearray(self.body)
            body[8] = int(self.active)
            if not self.active: struct.pack_into('<f', body, 24, 0)
            body[10] = body[11] = 1 if self.active else 0
            struct.pack_into('<H', body, 12, self.token)
            reply = 0xa0
        if not (kind == 0x21 and self.drop_arm):
            self.buffer.extend(bench.wire.encode_frame(reply, seq, body))
            self.ready_at = time.monotonic()+(self.delay if kind == 0x26 else 0)
        return len(data)


class BenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.body = bytes.fromhex(subprocess.check_output([codec, '--vector'], text=True).strip())
        cls.vectors = dict((key, bytes.fromhex(data)) for key, data in
                          (line.split() for line in subprocess.check_output([codec, '--trace-vectors'], text=True).splitlines()))

    def test_native_status(self):
        d = bench.decode_status(self.body)
        self.assertEqual(d['token'], 42); self.assertEqual(d['mode'], 1)
        self.assertTrue(d['feedback_valid']); self.assertTrue(d['position_valid'])
        self.assertAlmostEqual(d['output_a'], .1, places=6)
        self.assertEqual(d['temperature'], 31); self.assertGreater(d['encoder'], 0)
        self.assertEqual(d['rx_id'], 0x204+d['device_id'])
        self.assertEqual(d['slot'], (d['device_id']-1) % 4)
        self.assertEqual(d['control_us'], 110000)
        self.assertEqual(d['feedback_us'], 110000)
        self.assertEqual(d['reference_epoch'], 1)
        for data in (self.body[:74], self.body+b'\0', struct.pack('<I', 0x53354233)+self.body[4:]):
            with self.assertRaises(ValueError): bench.decode_status(data)

    def test_parameters(self):
        self.assertEqual(len(bench.settings_payload(1, VALUES)), 42)
        for index, value in ((0, 1), (0, float('nan')), (1, 3601), (1, 1e-50), (7, -.1), (8, 1), (8, -.1)):
            values = VALUES.copy(); values[index] = value
            with self.assertRaises(ValueError): bench.settings_payload(1, values)
        self.assertEqual(len(bench.settings_payload(1, [2, 0, 3, 60, .766, 0, 7.5, 0, 0])), 42)
        self.assertEqual(len(bench.settings_payload(3, [20, 0, .549, 3.49, .766, 0, 7.5, 0, 0])), 42)

        for tau in (-.01, float('nan'), float('inf')):
            with self.assertRaises(ValueError): bench.settings_payload(1, [*VALUES, tau])

    def test_host_parameters_native_decode(self):
        for mode, target in ((1, .1), (2, .5), (3, -.2)):
            values = [*VALUES, .003, 1]; values[0] = target
            payload = bench.settings_payload(mode, values)
            decoded = [float(v) for v in subprocess.check_output([codec, '--settings', payload.hex()], text=True).split()]
            self.assertEqual(len(decoded), 12)
            for a, b in zip(decoded, [mode, *values]): self.assertAlmostEqual(a, b, places=6)
        forged = struct.pack('<B10fB', 1, float('nan'), *VALUES[1:], 0, 0)
        self.assertNotEqual(subprocess.run([codec, '--settings', forged.hex()], capture_output=True).returncode, 0)

    def test_target_payload(self):
        payload = bench.target_payload(42, -.25, 123456)
        self.assertEqual(struct.unpack('<HfI', payload), (42, -.25, 123456))
        for value, ramp in ((float('nan'), 0), (1, -1), (1, 0x80000000)):
            with self.assertRaises(ValueError): bench.target_payload(42, value, ramp)

    @staticmethod
    def wait_for(condition, timeout=2):
        deadline = time.monotonic()+timeout
        while not condition() and time.monotonic() < deadline: time.sleep(.003)
        if not condition(): raise AssertionError('假串口条件超时')

    def run_session(self, drop=False, arm=False):
        with tempfile.TemporaryDirectory() as folder:
            port = FakeSerial(self.body, drop); events = queue.Queue()
            session = bench.Session('FAKE', Path(folder)/'log.jsonl', events, serial_factory=lambda *a, **kw: port)
            session.start()
            try:
                self.wait_for(lambda: port.writes)
                if arm: session.start_motion(bench.settings_payload(1, VALUES))
                time.sleep(.65 if drop else .28)
                session.stop_motion(); time.sleep(.12)
            finally:
                session.quit_event.set(); session.join(2)
            self.assertFalse(session.is_alive()); self.assertTrue(port.closed)
            self.assertEqual(len(port.thread_ids), 1)
            return port.writes

    def test_no_implicit_arm_and_shutdown_stop(self):
        writes = self.run_session()
        self.assertNotIn(0x21, [x[0] for x in writes]); self.assertNotIn(0x22, [x[0] for x in writes])
        self.assertEqual(writes[-1][0], 0x23)

    def test_arm_once_and_heartbeat(self):
        writes = self.run_session(arm=True)
        arms = [x for x in writes if x[0] == 0x21]; self.assertEqual(len(arms), 1)
        heartbeats = [x for x in writes if x[0] == 0x22]; self.assertTrue(heartbeats)
        self.assertTrue(all(struct.unpack('<H', x[2])[0] == arms[0][1] for x in heartbeats))

    def test_trigger_precedes_queued_start_and_target(self):
        with tempfile.TemporaryDirectory() as folder:
            port = FakeSerial(self.body, vectors=self.vectors)
            session = bench.Session('FAKE', Path(folder)/'log.jsonl', queue.Queue(),
                                    serial_factory=lambda *a, **kw: port)
            # 两个按钮紧接着操作时, 预置触发不能被另一条命令队列反超.
            session.start_capture(6, True)
            session.start_motion(bench.settings_payload(1, VALUES))
            session.start()
            try:
                self.wait_for(lambda: any(w[0] == 0x21 for w in port.writes))
                first_arm = next(i for i, w in enumerate(port.writes) if w[0] == 0x26 and w[2][1] == 1)
                first_start = next(i for i, w in enumerate(port.writes) if w[0] == 0x21)
                self.assertLess(first_arm, first_start)
                self.wait_for(lambda: len(list(Path(folder).glob('*.btrace'))) == 1)
                self.wait_for(lambda: session.capture is None)
                begin = len(port.writes)
                session.start_capture(6, True)
                session.update_target(.12)
                self.wait_for(lambda: any(w[0] == 0x25 for w in port.writes[begin:]))
                writes = port.writes[begin:]
                arm = next(i for i, w in enumerate(writes) if w[0] == 0x26 and w[2][1] == 1)
                target = next(i for i, w in enumerate(writes) if w[0] == 0x25)
                self.assertLess(arm, target)
            finally:
                session.quit_event.set(); session.join(2)
            self.assertFalse(session.is_alive())

    def test_excitation_order_token_and_shutdown(self):
        with tempfile.TemporaryDirectory() as folder:
            port = FakeSerial(self.body, vectors=self.vectors)
            session = bench.Session('FAKE', Path(folder)/'log.jsonl', queue.Queue(),
                                    serial_factory=lambda *a, **kw: port)
            for args in ((0, 0, .1, 10, 100000), (1, 0, .1, 0, 100000),
                         (1, 0, .1, 10, 0), (1, float('nan'), .1, 10, 100000)):
                with self.assertRaises(ValueError): session.excite(*args)
            session.start_motion(bench.settings_payload(1, VALUES))
            session.start()
            try:
                self.wait_for(lambda: session.token is not None)
                session.start_capture(6, True)
                session.excite(1, .05, .02, 10, 500000)
                self.wait_for(lambda: any(w[0] == 0x27 for w in port.writes))
                arm = next(i for i, w in enumerate(port.writes) if w[0] == 0x26 and w[2][1] == 1)
                excite = next(i for i, w in enumerate(port.writes) if w[0] == 0x27)
                self.assertLess(arm, excite)
                token, kind, bias, amp, hz, duration = struct.unpack('<HB3fI', port.writes[excite][2])
                self.assertEqual(token, session.token); self.assertEqual(kind, 1)
                self.assertAlmostEqual(bias, .05); self.assertAlmostEqual(amp, .02)
                self.assertEqual(hz, 10); self.assertEqual(duration, 500000)
                session.stop_motion()
                self.wait_for(lambda: session.token is None)
            finally:
                session.quit_event.set(); session.join(2)
            self.assertFalse(session.is_alive()); self.assertTrue(port.closed)
            self.assertEqual(sum(w[0] == 0x27 for w in port.writes), 1)

    def test_missing_arm_ack_no_retry(self):
        writes = self.run_session(drop=True, arm=True)
        self.assertEqual(sum(x[0] == 0x21 for x in writes), 1)
        self.assertNotIn(0x22, [x[0] for x in writes]); self.assertIn(0x23, [x[0] for x in writes])

    def download_vector(self):
        download = bench.TraceDownload(55, 6, True, {'note': 'native fixture, not hardware'})
        head = bytearray(self.vectors['data0'][:32]); head[1] = 1
        struct.pack_into('<H', head, 10, 0)
        download.accept(1, head)
        self.assertEqual(download.phase, 'start_meta')
        for key in ('meta0', 'meta1', 'data0', 'data3'):
            op, request = download.request()
            self.assertEqual(len(request), 12)
            download.accept(op, self.vectors[key])
        self.assertEqual(download.phase, 'done')
        return download

    def test_experiment_fake_serial_end_to_end(self):
        import motor_bench_experiment as experiment
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);firmware=root/'fixture.elf';firmware.write_bytes(b'not hardware')
            port=FakeSerial(self.body,vectors=self.vectors)
            original=bench.Session
            def factory(*args, **kwargs):
                return original(*args,**kwargs,serial_factory=lambda *a, **kw:port)
            args=SimpleNamespace(output=root/'run',firmware=firmware,port='FAKE',
                id=bench.decode_status(self.body)['device_id'],mode='current',target=.1,seconds=.03,
                current_limit=.2,speed_limit=3,kp=.04,ki=0,position_kp=3,kd=0,alpha=0,derivative_tau=0,encoder_velocity=False,
                compact=False,note='synthetic',wave='step',ramp=0,amplitude=.02,frequency=5)
            with patch.object(bench,'Session',factory),contextlib.redirect_stdout(io.StringIO()):
                experiment.run(args)
            result=json.loads((root/'run'/'result.json').read_text(encoding='utf-8'))
            self.assertFalse(result['final']['active']);self.assertTrue(port.closed)
            self.assertEqual(sum(w[0]==0x21 for w in port.writes),1)
            self.assertTrue(Path(result['record']).is_file())

    def test_ident_native_format_and_roundtrip(self):
        head, raw = bench.trace.decode_reply(self.vectors['ident_data'])
        self.assertEqual(head['version'], 2); self.assertEqual(head['capacity'], 2730)
        _, body = bench.trace.decode_reply(self.vectors['ident_meta'])
        meta = {'layout': bench.trace.IDENT_LAYOUT, 'start': bench.decode_status(body[:bench.STATUS.size]),
                'run': 71, 'build_id': 'native-ident'}
        rows = bench.trace.rows(meta, raw)
        self.assertEqual(len(rows), 8);self.assertEqual(rows[0]['command_raw'], -123)
        self.assertAlmostEqual(rows[-1]['time_s'], .007)
        self.assertTrue(all(r['record_gaps'] == 0 for r in rows))
        self.assertTrue(all(r['p_a'] is None and r['position'] is None for r in rows))
        self.assertIsNone(rows[0]['dt_us']); self.assertEqual(rows[1]['dt_us'], 1000)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'ident.btrace'
            bench.trace.save(path, meta, raw)
            saved_meta, saved_data = bench.trace.load(path)
            self.assertEqual(saved_data, raw);self.assertEqual(saved_meta['record_bytes'], 24)
            outputs = bench.trace.export(path, plot=False)
            self.assertEqual(len(outputs), 2)
        download = bench.TraceDownload(71, 2730, False, {}, compact=True)
        self.assertEqual(download.request()[1][0], 2)
        self.assertEqual(download.request()[1][3], 1)

    def test_trace_native_terms_raw_frame_and_status(self):
        download = self.download_vector()
        rows = bench.trace.rows(download.metadata, download.data)
        self.assertEqual(len(rows), 6)
        self.assertAlmostEqual(rows[0]['p_a'], download.metadata['start']['kp'], places=6)
        self.assertEqual(rows[0]['command_raw'], 3000)
        self.assertTrue(rows[0]['saturated']); self.assertTrue(rows[0]['target_event'])
        self.assertTrue(rows[0]['reference_event'])
        self.assertEqual(rows[0]['submit_latency_us'], 21)
        self.assertTrue(rows[3]['pid_valid']); self.assertFalse(rows[3]['active'])
        self.assertIsNone(rows[3]['command_a'])  # 本地 busy 不等于实际发送了零电流.
        self.assertFalse(rows[4]['pid_valid']); self.assertIsNone(rows[4]['p_a'])
        self.assertEqual(rows[4]['command_raw'], 0)
        self.assertAlmostEqual(rows[-1]['time_s'], .005)
        summary = bench.trace.metrics(rows)
        self.assertEqual(summary['record_gaps'], 0); self.assertEqual(summary['tx_not_accepted'], 1)

    def test_trace_identity_length_and_micro_wrap(self):
        for payload in (b'', self.vectors['data0'][:-1], self.vectors['meta0']+b'0'):
            with self.assertRaises(ValueError): bench.trace.decode_reply(payload)
        download = self.download_vector()
        wrong = bytearray(self.vectors['data0']); struct.pack_into('<I', wrong, 4, 56)
        with self.assertRaises(ValueError): download.accept(3, wrong)
        # 真实 native 记录的字段做时钟平移, 不重算 PID 来伪造闭环证据.
        data = bytearray(download.data)
        for i in range(6):
            for at in (0, 4, 8):
                value = struct.unpack_from('<I', data, i*64+at)[0]
                struct.pack_into('<I', data, i*64+at, (value-101000+0xffffff00) & 0xffffffff)
        rows = bench.trace.rows(download.metadata, data)
        self.assertAlmostEqual(rows[-1]['time_s'], .005)
        self.assertEqual(rows[-1]['submit_latency_us'], 21)
        self.assertTrue(all(row['feedback_age_us'] == 0 for row in rows))

    def test_trace_file_csv_png_and_digest(self):
        download = self.download_vector()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'window.btrace'
            bench.trace.save(path, download.metadata, download.data)
            meta, raw = bench.trace.load(path)
            self.assertEqual(raw, download.data); self.assertEqual(meta['records'], 6)
            outputs = bench.trace.export(path)
            self.assertEqual(len(outputs), 3)
            self.assertTrue(all(Path(p).stat().st_size > 100 for p in outputs))
            self.assertEqual(Path(outputs[-1]).read_bytes()[:8], b'\x89PNG\r\n\x1a\n')
            corrupted = bytearray(path.read_bytes()); corrupted[-1] ^= 1; path.write_bytes(corrupted)
            with self.assertRaises(ValueError): bench.trace.load(path)

    def test_download_keeps_heartbeat_target_and_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            port = FakeSerial(self.body, vectors=self.vectors, delay=.04)
            session = bench.Session('FAKE', Path(directory)/'log.jsonl', queue.Queue(), serial_factory=lambda *a, **kw: port)
            session.start()
            try:
                session.start_motion(bench.settings_payload(1, VALUES))
                self.wait_for(lambda: session.token is not None)
                session.start_capture(6, True, 'fixture')
                self.wait_for(lambda: any(x[0] == 0x26 for x in port.writes))
                session.update_target(-.1, 5000)
                self.wait_for(lambda: any(x[0] == 0x25 for x in port.writes))
                self.wait_for(lambda: list(Path(directory).glob('*.btrace')))
                session.stop_motion()
                self.wait_for(lambda: any(x[0] == 0x23 for x in port.writes))
            finally:
                session.quit_event.set(); session.join(2)
            self.assertFalse(session.is_alive()); self.assertEqual(len(port.thread_ids), 1)
            starts = [x for x in port.writes if x[0] == 0x21]; self.assertEqual(len(starts), 1)
            updates = [x for x in port.writes if x[0] == 0x25]; self.assertEqual(len(updates), 1)
            self.assertEqual(struct.unpack('<H', updates[0][2][:2])[0], starts[0][1])
            renewals = [t for t, (kind, _, _) in zip(port.times, port.writes) if kind in (0x21, 0x22, 0x25)]
            self.assertGreater(len(renewals), 2)
            self.assertLess(max(b-a for a, b in zip(renewals, renewals[1:])), .5)
            meta, data = bench.trace.load(next(Path(directory).glob('*.btrace')))
            self.assertEqual(len(data), 384); self.assertEqual(meta['note'], 'fixture')
            records = [json.loads(line) for line in (Path(directory)/'log.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertTrue(any(r['event'] == 'trace_saved' for r in records))

    def test_trace_timeout_does_not_stop_control(self):
        with tempfile.TemporaryDirectory() as directory:
            port = FakeSerial(self.body, vectors=self.vectors, drop_trace=True)
            events = queue.Queue()
            session = bench.Session('FAKE', Path(directory)/'log.jsonl', events, serial_factory=lambda *a, **kw: port)
            session.start()
            try:
                session.start_motion(bench.settings_payload(1, VALUES))
                self.wait_for(lambda: session.token is not None)
                session.start_capture(6)
                self.wait_for(lambda: any(x[0] == 0x26 for x in port.writes))
                time.sleep(.38)
                self.assertFalse(any(x[0] == 0x23 for x in port.writes))
                self.assertIsNotNone(session.token)
                self.assertTrue(any(x[0] == 0x22 for x in port.writes))
            finally:
                session.quit_event.set(); session.join(2)
            self.assertFalse(session.is_alive())
            self.assertTrue(any(kind == 'error' and '采集响应超时' in text for kind, text in list(events.queue)))


if __name__ == '__main__':
    unittest.main()
