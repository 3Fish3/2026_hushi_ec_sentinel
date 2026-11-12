"""分段分析的质量与坐标契约; 测试夹具不作为真实回读或实车验收证据。"""
import copy
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robots/sentinel/host'))
import sentinel_raw as raw
import sentinel_segmented_calibrate as segmented


def fixture(directory, *, lost=False):
    directory = Path(directory)
    groups = {'PMAX': 12.5, 'VMAX': 30., 'TMAX': 10., 'ESC_ID': 1, 'MST_ID': 17, 'Gr': 10.}
    with (directory / 'synthetic-mapping.jsonl').open('w') as stream:
        for count in (1, 2):
            stream.write(json.dumps({'fixture': True, 'data': {'registers': [
                {'name': name, 'value': value, 'valid': True, 'responses': count}
                for name, value in groups.items()]}}) + '\n')
    manifest = {'schema': 'sentinel-observed-calibration/1', 'dataset': '.',
                'mapping_log': 'synthetic-mapping.jsonl', 'sources': {'pitch': {'can_id': 17}}}
    records = []
    for i, sequence in enumerate((0, 1, 3, 4)):
        record = bytearray(64)
        struct.pack_into('<4B7I', record, 0, 1, 1, 0, 1, sequence, i + 1, 1000 * (i + 1),
                         1000 * (i + 1), 0, 0, 7)
        struct.pack_into('<3f', record, 32, .001, .002, .003)
        records.append(bytes(record))
    raw.save(directory / 'synthetic.sraw', {'layout_id': raw.LAYOUT, 'run': 7, 'start_us': 0,
             'profile': 'stationary', 'configuration': 'synthetic_test_fixture',
             'units': {'gyro': 'rad/s', 'accel': 'm/s2', 'temperature': 'degC'}}, records)
    summary = {'complete': False, 'failure': None, 'diagnostics_error': None, 'source_issues': [],
               'final': {'state': 3, 'buffered': 0, 'dropped': int(lost), 'generated': 5, 'run': 7},
               'record_gaps': 1, 'crc_errors': 1, 'received': {'records': 4}}
    (directory / 'synthetic.sraw.summary.json').write_text(json.dumps(summary))
    return manifest


class QualityTests(unittest.TestCase):
    def test_explicit_gap_opt_in_preserves_original_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = fixture(directory)
            path = Path(directory) / 'synthetic.sraw'
            before = path.read_bytes()
            with self.assertRaisesRegex(ValueError, '缺包'):
                segmented.Dataset(manifest, directory).load('synthetic.sraw')
            result = segmented.Dataset(manifest, directory, True).load('synthetic.sraw')
            self.assertFalse(result['summary']['complete'])
            self.assertEqual(result['summary']['record_gaps'], 1)
            self.assertEqual([r['sequence'] for r in result['rows']], [0, 1, 3, 4])
            self.assertEqual([[r['sequence'] for r in p] for p in result['parts']], [[0, 1], [4]])
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(result['rows'][2]['gap'], 1)

    def test_false_complete_is_rejected_even_with_opt_in(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = fixture(directory)
            path = Path(directory) / 'synthetic.sraw.summary.json'
            summary = json.loads(path.read_text())
            summary['complete'] = True
            path.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError, '完整性标记'):
                segmented.Dataset(manifest, directory, True).load('synthetic.sraw')

    def test_wrong_summary_count_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = fixture(directory)
            path = Path(directory) / 'synthetic.sraw.summary.json'
            summary = json.loads(path.read_text())
            summary['record_gaps'] = 0
            path.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError, '缺口不一致'):
                segmented.Dataset(manifest, directory, True).load('synthetic.sraw')

    def test_board_queue_loss_is_not_a_wire_gap(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = fixture(directory, lost=True)
            with self.assertRaisesRegex(ValueError, '记录队列丢弃'):
                segmented.Dataset(manifest, directory, True).load('synthetic.sraw')

    def test_source_changes_across_gap_are_rejected(self):
        rows = [{'kind': 'can', 'stream': 9, 't_s': 0., 'lost': 0, 'gap': 0},
                {'kind': 'can', 'stream': 9, 't_s': 1., 'lost': 1, 'gap': 3}]
        with self.assertRaisesRegex(ValueError, 'CAN队列'):
            segmented.source_quality(rows)
        gyro = [{'kind': 'gyro', 'stream': 0, 't_s': 0., 'lost': 0, 'fifo_overruns': 0, 'flags': 1, 'values': [0, 0, 0]},
                {'kind': 'gyro', 'stream': 0, 't_s': 1., 'lost': 0, 'fifo_overruns': 1, 'flags': 1, 'values': [0, 0, 0]}]
        with self.assertRaisesRegex(ValueError, 'FIFO'):
            segmented.source_quality(gyro)

    def test_clock_regression_across_gap_is_rejected(self):
        rows = [{'kind': 'gyro', 'stream': 0, 't_s': 1., 'lost': 0, 'gap': 0, 'flags': 1, 'values': [0, 0, 0]},
                {'kind': 'gyro', 'stream': 0, 't_s': .5, 'lost': 0, 'gap': 2, 'flags': 1, 'values': [0, 0, 0]}]
        with self.assertRaisesRegex(ValueError, '时间不递增'):
            segmented.source_quality(rows)

    def test_split_does_not_modify_or_renumber_rows(self):
        rows = [{'gap': 0, 'sequence': 0}, {'gap': 0, 'sequence': 1},
                {'gap': 3, 'sequence': 5}, {'gap': 0, 'sequence': 6}]
        before = copy.deepcopy(rows)
        parts = segmented.continuous_parts(rows)
        self.assertEqual(rows, before)
        self.assertEqual([r['sequence'] for p in parts for r in p], [0, 1, 6])

    def test_invalid_imu_cannot_be_silently_dropped(self):
        row = {'kind': 'gyro', 'flags': 1, 'values': [None, 0, 0]}
        with self.assertRaisesRegex(ValueError, '非有限'):
            segmented.source_quality([row])

    def test_derivative_and_interpolation_stay_inside_each_part(self):
        sources = {'main': {'bus': 1, 'can_id': 520}, 'fire': {'bus': 2, 'can_id': 522},
                   'pitch': {'bus': 2, 'can_id': 17}}
        parts = []
        for start, offset in [(0., 0.), (2., 1.5)]:
            part = []
            for i in range(100):
                t = start + i * .01
                part.append(dict(kind='gyro', stream=0, t_s=t, flags=1, gap=0, values=[0, 0, .2]))
                for axis, stream in [('main', 4), ('fire', 5), ('pitch', 9)]:
                    if axis == 'pitch':
                        position = round((.2 / 12.5 + 1) * 65535 / 2)
                        b = bytes([1, position >> 8, position & 255, 128, 0, 0, 25, 25])
                    else:
                        q = .2 * (t - start) + offset if axis == 'main' else .5
                        b = struct.pack('>Hh', round(q * 8192 / (2*np.pi)), 2 if axis == 'main' else 0) + bytes(4)
                    part.append(dict(kind='can', stream=stream, t_s=t, flags=1, gap=0, dlc=8,
                                     data=b.hex(), **sources[axis]))
            parts.append(part)
        class SyntheticDataset:
            manifest = {'sources': sources, 'analysis': {'derivative_half_window': 4}}
            ranges = {'node': 1, 'pmax_rad': 12.5, 'vmax_rad_s': 30.}
            def load(self, name):
                return {'metadata': {'profile': 'rotation', 'start_us': 0}, 'parts': parts}
        windows, omitted = segmented.motion_windows(SyntheticDataset(),
            [{'file': 'synthetic', 'interval_s': [0, 3], 'dominant_axis': 'main'}], np.zeros(3), .06, .02)
        self.assertEqual(len(windows), 2)
        for window in windows:
            for t, v in window['derived']:
                self.assertGreaterEqual(window['t'][0] - .02 + 1e-12, t[0])
                self.assertLessEqual(window['t'][-1] + .02 - 1e-12, t[-1])
            np.testing.assert_allclose(window['derived'][0][1], .2, atol=.02)


class DirectionTests(unittest.TestCase):
    def test_horizontal_forward_uses_raise_sign(self):
        forward = segmented.horizontal_forward(np.array([0., 0., 1.]), np.array([0., -1., 0.]))
        np.testing.assert_allclose(forward, [1, 0, 0], atol=1e-14)
        self.assertGreater(np.cross([0, -1, 0], forward) @ [0, 0, 1], 0)

    def test_parallel_pitch_and_up_are_rejected(self):
        with self.assertRaisesRegex(ValueError, '接近重力'):
            segmented.horizontal_forward(np.array([0., 0., 1.]), np.array([0., 0., 1.]))


if __name__ == '__main__':
    unittest.main()
