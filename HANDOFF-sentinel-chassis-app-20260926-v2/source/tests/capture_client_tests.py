"""采集工具离线测试: C++ 字节向量、只读分页、日志标记与 CSV. 不打开真实串口."""
import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'boards/sentinel_f407/tools'))
import sentinel_capture as cap
from board_monitor import Decoder, encode_frame

exe = Path(sys.argv.pop(1))
golden = {int(line.split()[0]): bytes.fromhex(line.split()[1])
          for line in subprocess.check_output([str(exe), '--golden'], text=True).splitlines()}


def summary(now=100):
    values = [0]*24
    values[:2] = [0x43415031, 168000000]
    return struct.pack('<4BI24I32s14I', cap.SCHEMA, 0, 1, 0, now, *values, b'capture-test', *([0]*14))


class Port:
    def __init__(self):
        self.buffer = bytearray()
        self.written = []
    @property
    def in_waiting(self): return len(self.buffer)
    def read(self, n):
        result = bytes(self.buffer[:n]); del self.buffer[:n]; return result
    def write(self, data): self.written.append(data); return len(data)


class Tests(unittest.TestCase):
    def test_native_motor_golden(self):
        page = cap.decode_page(golden[1])
        m, unseen = page['motors']
        self.assertEqual(m['name'], 'LF')
        self.assertEqual(m['encoder'], 1)  # 单圈跨零, 不发明累计圈数或关节零点.
        self.assertEqual(m['rpm'], 120)
        self.assertEqual(m['effort_raw'], -2)
        self.assertAlmostEqual(m['rotor_rad_s'], 4*math.pi)
        self.assertEqual(m['temperature_c'], 43)
        self.assertEqual(m['sum_cycles'], 168000)
        self.assertEqual(m['intervals'], 1)
        self.assertTrue(m['valid'])
        self.assertIsNone(m['cumulative_rotor_rad'])
        self.assertIsNone(m['joint_rad'])
        self.assertFalse(unseen['valid'])
        self.assertIsNone(unseen['encoder'])
        self.assertIsNone(cap.decode_page(golden[5])['motors'][0]['temperature_c'])

    def test_native_dm_golden(self):
        values = cap.decode_page(golden[9])['registers']
        self.assertEqual(values[2]['rid'], 0x50)
        self.assertEqual(values[2]['value'], 1.25)
        self.assertEqual(values[2]['request_ms'], 100)
        self.assertEqual(values[2]['received_ms'], 104)
        self.assertEqual(values[3]['value'], -0.5)
        self.assertTrue(values[3]['valid'])
        self.assertFalse(values[0]['seen'])
        self.assertIsNone(values[0]['value'])

    def test_expiration_and_rollover(self):
        data = bytearray(golden[1]); struct.pack_into('<I', data, 4, 201)
        self.assertFalse(cap.decode_page(data)['motors'][0]['valid'])
        struct.pack_into('<I', data, 4, 3)
        struct.pack_into('<I', data, 8+16, 0xfffffffe)
        self.assertEqual(cap.decode_page(data)['motors'][0]['age_ms'], 5)
        data = bytearray(golden[9]); struct.pack_into('<I', data, 4, 654)
        self.assertFalse(cap.decode_page(data)['registers'][3]['valid'])

    def test_nan_does_not_become_json_nan(self):
        data = bytearray(golden[9]); struct.pack_into('<I', data, 8+2*20+12, 0x7fc00000)
        result = cap.decode_page(data)
        self.assertFalse(result['registers'][2]['valid'])
        self.assertIsNone(result['registers'][2]['value'])
        json.dumps(result, allow_nan=False)

    def test_bad_schema(self):
        for offset, value in ((0, cap.SCHEMA + 1), (1, 20), (2, 0), (3, 1)):
            data = bytearray(golden[1]); data[offset] = value
            with self.assertRaises(ValueError): cap.decode_page(data)
        with self.assertRaises(ValueError): cap.decode_page(golden[1][:-1])
        data = bytearray(summary()); struct.pack_into('<I', data, 8, 0)
        with self.assertRaises(ValueError): cap.decode_page(data)

    def test_sensor_fields(self):
        data = struct.pack('<4BI3I8hH5B3I2B7f4I', cap.SCHEMA, 6, 1, 0, 200,
                           2, 0, 195, 1, 2, 3, 4, 1234, -5, 6, 7, 0x1234, 1, 2, 1, 1, 0,
                           10, 0, 199, 1, 0, 0, 0, 9.81, 0.1, 0.2, 0.3, 30, 0, 0, 0, 0)
        self.assertEqual(len(data), 101)
        result = cap.decode_page(data)
        self.assertEqual(result['remote']['ch4'], 1234)
        self.assertEqual(result['remote']['mouse_x'], -5)
        self.assertEqual(result['remote']['keys'], 0x1234)
        self.assertTrue(result['imu']['valid'])
        self.assertTrue(result['remote']['valid'])
        fresh = cap.sensor_text(result, 0, 'ONLINE')
        aged = cap.sensor_text(result, 200, 'ONLINE')
        self.assertEqual(len(fresh), len(aged))
        self.assertNotIn('SNAPSHOT_OLD', aged)
        self.assertIn('+0.1000', fresh)
        self.assertEqual(aged.count('ONLINE'), 2)
        self.assertIn('    201 ms', aged)  # 年龄持续显示, 不延长 MCU 的有效期.
        result['imu']['state'] = 'LOST'
        self.assertIn('LOST', cap.sensor_text(result, 0, 'ONLINE'))
        self.assertIn('LINK_LOST', cap.sensor_text(result, 0, 'LOST'))

    def make_client(self):
        port, events, now = Port(), [], [0.0]
        return cap.Client(port, 10, events.append, clock=lambda: now[0]), port, events, now

    def feed(self, client, port, payload):
        port.buffer.extend(encode_frame(0xb0, client.pending[0], payload))
        client.tick()

    def test_client_read_only_and_split(self):
        client, port, events, now = self.make_client()
        client.tick()
        self.assertEqual(Decoder().feed(port.written[0]), [(0x30, 1, b'\0')])
        wire = encode_frame(0xb0, 1, summary())
        port.buffer.extend(wire[:10]); client.tick(); self.assertFalse(client.pages)
        port.buffer.extend(wire[10:]); client.tick(); self.assertIn(0, client.pages)
        now[0] = 0.11; client.tick()
        self.assertEqual(Decoder().feed(port.written[-1]), [(0x30, 2, b'\1')])
        self.feed(client, port, golden[1])
        before = len(port.written)
        mark = client.mark('main zero', 'q=0 rad, fixture A')
        self.assertEqual(len(port.written), before)
        self.assertEqual(mark['label'], 'main zero')
        self.assertEqual(mark['pages'][1]['motors'][0]['encoder'], 1)
        now[0] += 1
        mark = client.mark('late', '')
        self.assertFalse(mark['pages'][1]['motors'][0]['valid_at_mark'])

    def test_focus_and_requested_mark(self):
        client, port, events, now = self.make_client()
        client.tick(); self.feed(client, port, summary())
        client.focus_page = 9
        now[0] = 0.11; client.tick()
        self.assertEqual(client.pending[1], 9)
        self.feed(client, port, golden[9])
        client.request_mark(1, 'fixture', '0 rad')
        self.assertFalse(any(e['event'] == 'mark' for e in events))
        now[0] = 0.12; client.tick()
        self.assertEqual(client.pending[1], 1)
        self.feed(client, port, golden[1])
        mark = events[-1]
        self.assertEqual(mark['event'], 'mark')
        self.assertEqual(mark['requested_host_elapsed_s'], 0.11)
        self.assertIsNone(client.mark_request)
        self.assertTrue(all(Decoder().feed(w)[0][0] == 0x30 for w in port.written))

    def test_reboot_clears_pages(self):
        client, port, events, now = self.make_client()
        client.tick(); self.feed(client, port, summary(200))
        now[0] = 0.11; client.tick(); self.feed(client, port, golden[1])  # 160 < 200, 可观察时间倒退.
        self.assertEqual(client.segment, 1)
        self.assertNotIn(0, client.pages)
        now[0] = 0.22; client.tick(); self.assertEqual(client.pending[1], 0)

    def test_client_wrong_page_and_timeout(self):
        client, port, events, now = self.make_client()
        client.tick()
        with self.assertRaises(ValueError): self.feed(client, port, golden[1])
        client, port, events, now = self.make_client()
        client.tick()
        for t in (0.51, 1.02): now[0] = t; client.tick()
        now[0] = 1.53; client.tick()
        self.assertEqual(client.link_state, 'NO_RESPONSE')
        self.assertEqual(len(port.written), 4)
        self.feed(client, port, summary())
        self.assertEqual(client.link_state, 'ONLINE')
        now[0] = 1.64; client.tick()
        now[0] = 2.15; client.tick()
        self.assertEqual(client.link_state, 'LOST')
        self.assertIn(0, client.pages)  # 保留最后数据, 不因失联清空整个面板.
        payload = bytearray(golden[3]); payload[1] = client.pending[1]
        self.feed(client, port, payload)
        self.assertEqual(client.link_state, 'ONLINE')
        self.assertTrue(any(e['event'] == 'link_state' and e['state'] == 'LOST' for e in events))

    def test_partial_devices_and_report_age(self):
        missing, gm = cap.decode_page(golden[3])['motors']
        self.assertEqual(missing['state'], 'NO_FEEDBACK')
        self.assertEqual(gm['state'], 'ONLINE')
        self.assertEqual(cap.display_state(gm, 2000, 100, 'ONLINE'), 'ONLINE / SNAPSHOT_OLD')
        self.assertEqual(cap.display_state(gm, 2000, 100, 'LOST'), 'LINK_LOST (cached)')
        data = bytearray(golden[3]); struct.pack_into('<I', data, 4, 259)
        self.assertEqual(cap.decode_page(data)['motors'][1]['state'], 'LOST')
        self.assertEqual(cap.decode_page(golden[9])['registers'][0]['state'], 'NO_REPLY')
        self.assertEqual(cap.sample_state(0, False, 0, 50, 'INIT_FAILED'), 'INIT_FAILED')

    def test_early_firmware_is_explained_not_guessed(self):
        old = bytearray(summary()); old[0] = 1; del old[96:104]
        with self.assertRaisesRegex(cap.CaptureProtocolError, r'schema=1 .*length=184 .*sentinel_capture.bin'):
            cap.decode_page(old)

    def test_log_mark_export_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            log_path, csv_path = Path(tmp)/'run.jsonl', Path(tmp)/'run.csv'
            log = cap.CaptureLog(log_path, {'hardware': 'fixture'})
            log.emit({'event': 'page', 'sequence': 2, 'segment': 0, 'host_elapsed_s': 1, 'data': cap.decode_page(golden[9])})
            log.emit({'event': 'mark', 'label': 'zero', 'reference': '0 rad'})
            log.close()
            with self.assertRaises(FileExistsError): cap.CaptureLog(log_path, {})
            cap.export_csv(log_path, csv_path)
            text = csv_path.read_text(encoding='utf-8-sig')
            self.assertIn('1.25', text); self.assertIn('zero', text)
            with self.assertRaises(FileExistsError): cap.export_csv(log_path, csv_path)


if __name__ == '__main__': unittest.main()
