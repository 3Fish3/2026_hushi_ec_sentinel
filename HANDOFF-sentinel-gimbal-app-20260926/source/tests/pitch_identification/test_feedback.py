"""Pitch 反馈解码/MCU 时序回归，全部为合成字节，不连接硬件。"""
import csv
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robots/sentinel/host'))
import sentinel_raw as raw
from sentinel_pitch_feedback import analyze_rows


def metadata(origin=0):
    return dict(layout_id=raw.LAYOUT, run=7, start_us=origin, source='SYNTHETIC',
                parameters=[{'id': k, 'value': v} for k, v in
                            [(0x904, 12.5), (0x905, 30.0), (0x906, 10.0), (0x903, 10)]])


def record(kind, stream, seq, time_us, payload, *, source=10, flags=1, lost=0):
    return struct.pack('<4B7I', 1, kind, stream, flags, seq, source, time_us,
                       time_us, lost, 0xffffffff, 7) + payload


def feedback(seq, time_us, *, status=1, node=1, torque=3072, lost=0):
    payload = bytes([(status << 4) | node, 0x80, 0, 0x80, (torque >> 8) & 15, torque & 255, 36, 37])
    return record(4, 9, seq, time_us, struct.pack('<I', 2 | (8 << 8) | (0x011 << 16)) +
                  payload + bytes(20), flags=4, lost=lost)


def control(seq, time_us):
    return record(5, 6, seq, time_us,
                  struct.pack('<6fII', .1, .1, .09, .02, .03, .001,
                              (1 << 16) | (1 << 20), 1))


class FeedbackTests(unittest.TestCase):
    def load(self, records, meta=None):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'pitch.sraw'
            raw.save(path, metadata() if meta is None else meta, records)
            return raw.load(path)

    def test_decode_align_export_and_offline_command(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'pitch.sraw'
            raw.save(path, metadata(), [feedback(0, 100), control(1, 300), control(2, 400)])
            meta, rows = raw.load(path)
            for row in rows[1:]:
                self.assertAlmostEqual(row['pitch_reported_torque_nm'], 3072 * 20 / 4095 - 10)
                self.assertEqual(row['pitch_feedback_received_us'], 100)
                self.assertEqual(row['pitch_feedback_source_sequence'], 10)
                self.assertEqual(row['pitch_mos_temperature_c'], 36)
                self.assertEqual(row['pitch_rotor_temperature_c'], 37)
            self.assertEqual(rows[1]['pitch_feedback_age_us'], 200)
            self.assertEqual(rows[2]['pitch_feedback_age_us'], 300)
            report = analyze_rows(meta, rows)
            self.assertEqual(report['status'], 'complete')
            self.assertEqual(report['unique_feedback_records'], 1)
            self.assertEqual(report['feedback_matched_records'], 2)
            self.assertFalse(report['deployable_gains'])
            raw.export_csv(path, Path(folder) / 'pitch.csv')
            with (Path(folder) / 'pitch.csv').open() as file:
                exported = list(csv.DictReader(file))
            self.assertEqual(exported[1]['pitch_feedback_received_us'], '100')
            script = Path(raw.__file__).with_name('sentinel_pitch_feedback.py')
            command = [sys.executable, '-S', str(script), '--input', str(path),
                       '--output', str(Path(folder) / 'analysis.json')]
            done = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(done.returncode, 0, done.stderr)
            self.assertEqual(json.loads((Path(folder) / 'analysis.json').read_text())['status'], 'complete')
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)

    def test_future_feedback_is_not_backfilled(self):
        _, rows = self.load([control(0, 100), feedback(1, 200)])
        self.assertIsNone(rows[0]['pitch_reported_torque_nm'])
        self.assertEqual(rows[0]['pitch_feedback_status'], 'missing')

    def test_stale_and_invalid_newest_feedback_never_fall_back(self):
        for newest in (feedback(1, 200, node=2), feedback(1, 200, status=8)):
            _, rows = self.load([feedback(0, 100), newest, control(2, 300)])
            self.assertIsNone(rows[2]['pitch_reported_torque_nm'])
            self.assertEqual(rows[2]['pitch_feedback_status'], 'invalid_feedback')
        _, rows = self.load([feedback(0, 100), control(1, 10100)])
        self.assertIsNone(rows[1]['pitch_reported_torque_nm'])
        self.assertEqual(rows[1]['pitch_feedback_status'], 'stale')

    def test_microsecond_wrap_preserves_feedback_age(self):
        _, rows = self.load([feedback(0, 0xfffffff0), control(1, 0x20)], metadata(0xffffff00))
        self.assertEqual(rows[1]['pitch_feedback_age_us'], 48)
        self.assertEqual(rows[1]['pitch_feedback_status'], 'fresh')

    def test_equal_microsecond_orders_by_record_sequence(self):
        _, rows = self.load([control(0, 100), feedback(1, 100)])
        self.assertEqual(rows[0]['pitch_feedback_status'], 'missing')
        _, rows = self.load([feedback(0, 100), control(1, 100)])
        self.assertEqual(rows[1]['pitch_feedback_status'], 'fresh')
        self.assertEqual(rows[1]['pitch_feedback_age_us'], 0)

    def test_invalid_control_validity_does_not_borrow_valid_can(self):
        wire = bytearray(control(1, 300))
        struct.pack_into('<I', wire, 56, 0)
        meta, rows = self.load([feedback(0, 100), bytes(wire)])
        self.assertEqual(rows[1]['pitch_feedback_status'], 'invalid_feedback')
        self.assertIsNone(rows[1]['pitch_reported_torque_nm'])
        self.assertEqual(analyze_rows(meta, rows)['status'], 'incomplete')

    def test_nonfinite_control_values_never_report_complete(self):
        names = ('pitch_requested_rad', 'pitch_adopted_rad', 'pitch_measured_rad',
                 'pitch_measured_velocity_rad_s', 'pitch_feedforward_torque_nm', 'pitch_dt_s')
        for index, name in enumerate(names):
            for value in (float('nan'), float('inf')) + ((0.0, -.001) if name == 'pitch_dt_s' else ()):
                wire = bytearray(control(1, 300))
                struct.pack_into('<f', wire, 32 + 4*index, value)
                meta, rows = self.load([feedback(0, 100), bytes(wire)])
                report = analyze_rows(meta, rows)
                self.assertEqual(report['status'], 'incomplete')
                self.assertIn({'sequence': 1, 'field': name,
                               'reason': 'missing_non_finite_or_invalid_dt'}, report['invalid_fields'])
                json.dumps(report, allow_nan=False)
        meta, rows = self.load([feedback(0, 100), control(1, 300)])
        for name in ('pitch_reported_torque_nm', 'pitch_mos_temperature_c', 'pitch_rotor_temperature_c'):
            rows[1][name] = float('nan')
            report = analyze_rows(meta, rows)
            self.assertEqual(report['status'], 'incomplete')
            json.dumps(report, allow_nan=False)

    def test_old_file_or_invalid_limits_stays_unknown(self):
        for params in ([], [{'id': 0x904, 'value': float('nan')}],
                       metadata()['parameters'][:-1]):
            meta = metadata()
            meta['parameters'] = params
            if params and isinstance(params[0]['value'], float) and params[0]['value'] != params[0]['value']:
                # JSON 本身拒绝 NaN；直接验证离线元数据函数。
                from sentinel_pitch_feedback import config_from_parameters
                self.assertIsNone(config_from_parameters(params))
                continue
            meta, rows = self.load([feedback(0, 100), control(1, 300)], meta)
            self.assertIsNone(rows[1]['pitch_reported_torque_nm'])
            self.assertEqual(rows[1]['pitch_feedback_status'], 'config_missing')
            self.assertEqual(analyze_rows(meta, rows)['status'], 'incomplete')

    def test_record_loss_is_reported(self):
        meta, rows = self.load([feedback(0, 100, lost=1), control(2, 300)])
        self.assertIn('record_loss', analyze_rows(meta, rows)['reasons'])

    def test_incomplete_cli_preserves_report_and_returns_two(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder) / 'old.sraw', Path(folder) / 'analysis.json'
            meta = metadata()
            meta['parameters'] = []
            raw.save(source, meta, [feedback(0, 100), control(1, 300)])
            script = Path(raw.__file__).with_name('sentinel_pitch_feedback.py')
            done = subprocess.run([sys.executable, '-S', str(script), '--input', str(source),
                                   '--output', str(output)], capture_output=True, text=True)
            self.assertEqual(done.returncode, 2, done.stderr)
            report = json.loads(output.read_text(), parse_constant=lambda name: self.fail(name))
            self.assertEqual(report['status'], 'incomplete')
            self.assertIn('missing_mcu_mit_limits', report['reasons'])

    def test_profile4_reported_torque_time_and_old_nan(self):
        for received, valid, torque, expected in [(0xfffffff0, True, .8, .8),
                                                 (0x30, True, .8, None),
                                                 (0xfffffff0, False, .8, None),
                                                 (0xfffffff0, True, float('nan'), None)]:
            wire = bytearray(record(7, 6, 1, 0x20, bytes(32)))
            struct.pack_into('<I', wire, 16, received)
            struct.pack_into('<I', wire, 48, (1 << 16) if valid else 0)
            struct.pack_into('<3f', wire, 52, 70, 2, torque)
            row = raw.decode_record(wire)
            if expected is None:
                self.assertIsNone(row['torque_nm'])
            else:
                self.assertAlmostEqual(row['torque_nm'], expected)
                self.assertEqual(row['feedback_age_us'], 48)
                self.assertEqual(row['torque_source'], 'esc_reported')


if __name__ == '__main__':
    unittest.main()
