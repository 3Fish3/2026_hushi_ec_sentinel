"""Native APP/MIT/recorder/decoder proof, using only an in-process virtual motor.

This is a short-window transport proof, NOT a vehicle experiment or an input to
sentinel_pitch_identify's numerical simulator. It never computes vehicle gains.
"""
from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robots/sentinel/host'))
import sentinel_raw as raw


def capture_fixture(executable: Path, destination: Path) -> dict:
    wire = subprocess.check_output([str(executable), '--raw-pitch-fixture'], text=True, timeout=15)
    header, *encoded = wire.splitlines()
    manifest = json.loads(header)
    if manifest.get('source') != 'APP_MIT_SIMULATION_ONLY' or manifest.get('hardware_ready') is not False:
        raise ValueError('native fixture must explicitly identify offline-only data')
    records = [bytes.fromhex(line) for line in encoded]
    if not records or any(len(record) != raw.SIZE for record in records):
        raise ValueError('native recorder returned incomplete samples')
    destination.mkdir(parents=True, exist_ok=False)
    raw.save(destination / 'samples.sraw', manifest, records)
    metadata, decoded = raw.load(destination / 'samples.sraw')
    exported, run = raw.export_csv(destination / 'samples.sraw', destination / 'samples.csv')
    control = [row for row in decoded if row['kind'] == 'control']
    feedback = [row for row in decoded if row['kind'] == 'can']
    finite_torque = [row['pitch_feedforward_torque_nm'] for row in control
                    if row['pitch_feedforward_torque_nm'] is not None]
    summary = {
        'source': metadata['source'], 'hardware_ready': False, 'run': run,
        'samples': len(decoded), 'csv_rows': exported, 'controls': len(control),
        'can_feedback': len(feedback), 'has_positive_torque': any(t > .02 for t in finite_torque),
        'has_negative_torque': any(t < -.02 for t in finite_torque),
        'finished_holding': bool(control and not control[-1]['excitation_active']
                                 and not control[-1]['latched']),
        'gaps': sum(row.get('gap', 0) for row in decoded),
        'limitations': ['short-window synthetic APP/MIT proof; not measured vehicle data',
                        'no IMU traffic in this fixture; real board window capacity will differ',
                        'no sustained serial recording, vehicle model fit or deployable gains'],
    }
    (destination / 'manifest.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    (destination / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    return summary


class PipelineTests(unittest.TestCase):
    executable: Path

    def test_native_pitch_feedback_and_control_survive_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'capture'
            summary = capture_fixture(self.executable, destination)
            self.assertGreater(summary['controls'], 50)
            self.assertEqual(summary['controls'], summary['can_feedback'])
            self.assertEqual(summary['samples'], summary['csv_rows'])
            self.assertEqual(summary['gaps'], 0)
            self.assertTrue(summary['has_positive_torque'])
            self.assertTrue(summary['has_negative_torque'])
            self.assertTrue(summary['finished_holding'])
            self.assertFalse(summary['hardware_ready'])
            _, rows = raw.load(destination / 'samples.sraw')
            for row in rows:
                self.assertEqual(row['run'], 101)
                if row['kind'] == 'can':
                    self.assertEqual((row['stream'], row['bus'], row['can_id']), (9, 2, 0x011))
                    self.assertEqual(row['dlc'], 8)
                    self.assertTrue(math.isfinite(row['pitch_reported_torque_nm']))
                    self.assertIsInstance(row['pitch_mos_temperature_c'], int)
                else:
                    self.assertEqual((row['kind'], row['stream']), ('control', 6))
                    self.assertFalse(row['latched'])
                    self.assertGreater(row['pitch_dt_s'], 0)
                    self.assertTrue(math.isfinite(row['pitch_measured_rad']))
                    self.assertEqual(row['pitch_feedback_status'], 'fresh')
                    self.assertEqual(row['pitch_feedback_age_us'], 0)
                    self.assertTrue(math.isfinite(row['pitch_reported_torque_nm']))
            with (destination / 'samples.csv').open(newline='') as file:
                self.assertEqual(len(list(csv.DictReader(file))), summary['samples'])
            with self.assertRaises(FileExistsError):
                capture_fixture(self.executable, destination)


if __name__ == '__main__':
    if len(sys.argv) == 4 and sys.argv[1] == '--save-offline-sample':
        print(json.dumps(capture_fixture(Path(sys.argv[2]), Path(sys.argv[3])), indent=2))
    else:
        if len(sys.argv) < 2:
            raise SystemExit('required: native ec_pitch_identification_tests executable')
        PipelineTests.executable = Path(sys.argv.pop(1))
        unittest.main()
