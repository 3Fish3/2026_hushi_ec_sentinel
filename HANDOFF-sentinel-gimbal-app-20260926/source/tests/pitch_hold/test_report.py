"""Read-only statistics, including cached-page deduplication and partial files."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robots/sentinel/host'))

from sentinel_pitch_hold_report import summarize


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.summarize = summarize

    def sample(self, sequence=1):
        return {'event': 'sample', 'state': 'HOLDING', 'segment': 1, 'target_rad': 0.1,
                'status': {'0': {'session': 1, 'config_epoch': 2, 'reference_epoch': 3,
                                  'sequence': sequence, 'joints': {'pitch': {'angle_rad': 0.09,
                                  'rate_rad_s': 0.0, 'position_valid': True, 'velocity_valid': True}}}}}

    def test_deduplicated_metrics_and_stop_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'log.jsonl'
            rows = [self.sample(), self.sample(), self.sample(2), {'event': 'end', 'error': None,
                                                                      'stop_report': {'revoke': 'unknown'}}]
            path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
            report = self.summarize(path)
            self.assertTrue(report['complete'])
            self.assertIsNone(report['run_error'])
            self.assertEqual(report['stop_report']['revoke'], 'unknown')
            self.assertEqual(report['segments']['1']['error_rad']['samples'], 2)
            self.assertAlmostEqual(report['segments']['1']['error_rad']['rms'], 0.01)

    def test_live_partial_and_malformed_data(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'log.jsonl'
            path.write_text(json.dumps(self.sample()) + '\n{"partial"')
            with self.assertRaises(ValueError):
                self.summarize(path)
            report = self.summarize(path, live=True)
            self.assertFalse(report['complete'])
            self.assertIsNone(report['stop_report'])
            path.write_text('malformed\n')
            with self.assertRaises(ValueError):
                self.summarize(path, live=True)


if __name__ == '__main__':
    unittest.main()
