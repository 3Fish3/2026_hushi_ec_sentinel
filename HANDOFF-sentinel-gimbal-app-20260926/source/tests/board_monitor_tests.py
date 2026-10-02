"""只读观察器离线测试, 不打开串口. 使用 C++ 独立编码向量核对字段与 CRC."""
import importlib.util
import contextlib
import io
import json
import time
import types
from unittest.mock import patch
from pathlib import Path
import subprocess
import struct
import sys
import unittest

path = Path(__file__).resolve().parents[1] / 'boards/sentinel_f407/tools/board_monitor.py'
spec = importlib.util.spec_from_file_location('board_monitor', path)
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)
codec = sys.argv.pop(1) if len(sys.argv) > 1 else None


class MonitorTests(unittest.TestCase):
    def fixture(self):
        payload = struct.pack('<10I32s', 16, 6, 0, 0, 123, 10, 0, 0, 0, 0, b'fixture')
        payload += struct.pack('<3I5h3B', 99, 1, 0xfffffff0, -660, 660, -1, 42, 1234, 2, 3, 1)
        payload += struct.pack('<3I4B7f', 123, 0, 16, 0, 0, 1, 0, 1.25, 0, 9.8, 0, 0, 0, 27.25)
        payload += struct.pack('<8If3B', 5000, 0, 0, 0x208, 0, 0, 0, 0, 104.71976, 0, 0, 0)
        payload += struct.pack('<4I14I', 0, 7, 0, 0, 111, *([0]*12), 9)
        wire = monitor.encode_frame(0x81, 0x7e7d, payload)
        frames = monitor.Decoder().feed(wire)
        self.assertEqual(len(frames), 1)
        kind, seq, payload = frames[0]
        self.assertEqual((kind, seq), (0x81, 0x7e7d))
        self.assertEqual(monitor.encode_frame(kind, seq, payload), wire)
        return wire, monitor.decode_snapshot(payload)

    def test_native_vector(self):
        if codec is None:
            self.skipTest('C++ executable not supplied; native cross-check not run')
        native = bytes.fromhex(subprocess.check_output([codec, '--vector'], text=True).strip())
        wire, _ = self.fixture()
        self.assertEqual(native, wire)

    def test_layout(self):
        _, data = self.fixture()
        self.assertEqual(data['header']['build_id'], 'fixture')
        self.assertEqual(data['remote']['ch0'], -660)
        self.assertEqual(data['remote']['ch1'], 660)
        self.assertEqual(data['remote']['ch2'], -1)
        self.assertEqual(data['remote']['ch3'], 42)
        self.assertEqual(data['remote']['ch4'], 1234)
        self.assertEqual(data['remote']['age_ms'], 32)
        self.assertTrue(data['remote']['valid'])
        self.assertEqual(data['imu']['temp_c'], 27.25)
        self.assertEqual(data['can']['last_id'], 0x208)
        self.assertEqual(data['uart3']['noise'], 7)
        self.assertEqual(data['uart6']['rx_bytes'], 111)
        self.assertEqual(data['uart6']['response_drops'], 9)

    def test_chunking_damage_recovery(self):
        wire, _ = self.fixture()
        decoder = monitor.Decoder()
        frames = []
        for b in wire:
            frames.extend(decoder.feed(bytes([b])))
        self.assertEqual(len(frames), 1)
        damaged = bytearray(wire)
        damaged[10] ^= 1
        self.assertFalse(decoder.feed(damaged))
        self.assertEqual(len(decoder.feed(wire + wire)), 2)
        self.assertFalse(decoder.feed(b'\x7e' + b'x'*1000 + b'\x7e'))
        self.assertLessEqual(len(decoder.raw), 264)
        self.assertEqual(len(decoder.feed(wire)), 1)
        self.assertGreaterEqual(decoder.rejected, 2)

    def test_stale_hides_measurements(self):
        _, data = self.fixture()
        self.assertIn('CH raw:', monitor.render(data, 0.1, 'connected'))
        self.assertNotIn('CH raw:', monitor.render(data, 0.5, 'connected'))
        self.assertIn('STALE', monitor.render(data, 0.5, 'connected'))
        data['remote']['valid'] = False
        data['imu']['valid'] = False
        text = monitor.render(data, 0.1, 'connected')
        self.assertIn('unavailable', text)
        self.assertNotIn('CH raw:', text)
        self.assertNotIn('accel m/s2:', text)
        data['header']['build_id'] = '\x1b[2J'
        self.assertNotIn('\x1b', monitor.render(data, 0.1, 'connected'))

    def test_client_readonly_roundtrip_and_cleanup(self):
        wire, _ = self.fixture()
        payload = monitor.Decoder().feed(wire)[0][2]
        class SerialError(Exception):
            pass
        class Port:
            def __init__(self):
                self.buffer = bytearray()
                self.requests = 0
                self.closed = False
            def reset_input_buffer(self):
                self.buffer.clear()
            @property
            def in_waiting(self):
                return len(self.buffer)
            def write(self, data):
                frames = monitor.Decoder().feed(data)
                self_test.assertEqual(len(frames), 1)
                kind, seq, body = frames[0]
                self_test.assertEqual((kind, body), (1, b''))
                self.requests += 1
                self.buffer.extend(monitor.encode_frame(0x81, seq, payload))
                return len(data)
            def read(self, n):
                time.sleep(0.005)
                result = bytes(self.buffer[:n])
                del self.buffer[:n]
                return result
            def close(self):
                self.closed = True
        self_test = self
        port = Port()
        fake = types.SimpleNamespace(Serial=lambda *a, **k: port,
                                     SerialException=SerialError, SerialTimeoutException=SerialError)
        output = io.StringIO()
        with patch.dict(sys.modules, {'serial': fake}), patch.object(sys, 'argv',
                ['board_monitor.py', '--port', 'FAKE', '--headless', '--seconds', '0.25', '--hz', '10']), contextlib.redirect_stdout(output):
            self.assertEqual(monitor.main(), 0)
        self.assertTrue(port.closed)
        self.assertGreaterEqual(port.requests, 2)
        self.assertTrue(all(json.loads(line)['event'] == 'snapshot' for line in output.getvalue().splitlines()))
        # 无返回时严格停止于三次超时, 不自动重连/无限发请求.
        port = Port()
        def discard(data):
            port.requests += 1
            return len(data)
        port.write = discard
        with patch.dict(sys.modules, {'serial': fake}), patch.object(sys, 'argv',
                ['board_monitor.py', '--port', 'FAKE', '--headless', '--seconds', '2']), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(monitor.main(), 2)
        self.assertEqual(port.requests, 3)
        self.assertTrue(port.closed)

    def test_invalid_sizes(self):
        with self.assertRaises(ValueError):
            monitor.decode_snapshot(bytes(249))
        with self.assertRaises(ValueError):
            monitor.encode_frame(1, 1, bytes(257))


unittest.main()
