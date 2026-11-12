"""原生后端 + Python 编排的整体回归: 纯软件虚拟电机, 不打开串口.

这条用例把原先只能人工执行的完整流程固定下来: 进程内启动 ec_sentinel_app_host,
由 sentinel_pitch_experiment 走完预检、授权、保持、方波、连续采集、排空与撤销,
最后独立解码产物并核对质量字段.
"""
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2]
SCRIPT = SOURCE / 'robots/sentinel/host/sentinel_pitch_experiment.py'
BACKEND = Path(os.environ.get('SENTINEL_PITCH_BACKEND', str(SOURCE / 'build/host-tests/ec_sentinel_app_host')))
HOST = SOURCE / 'robots/sentinel/host'

sys.path.insert(0, str(HOST))
import sentinel_raw as raw  # noqa: E402


class NativeExperimentEndToEnd(unittest.TestCase):
    def test_mcu_position_waveform_records_actual_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'position'
            command = [sys.executable, str(SCRIPT), '--backend', str(BACKEND),
                       '--output', str(output), '--arm', '--run', '145',
                       '--physical-low', '-0.4', '--physical-high', '0.4', '--excursion', '0.02',
                       '--max-temperature', '60', '--max-torque', '3', '--max-speed', '1.5', '--max-error', '0.03',
                       '--amplitude-nm', '0', '--position-amplitude-rad', '0.01', '--position-speed-rad-s', '0.1',
                       '--half-period-us', '300000', '--baseline-us', '200000', '--cycles', '3',
                       '--final-hold-us', '200000', '--stream-duration-us', '5000000']
            done = subprocess.run(command, capture_output=True, text=True, timeout=120,
                                  env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
            summary = json.loads((output / 'summary.json').read_text())
            metadata, rows = raw.load(output / 'pitch.sraw')
            self.assertTrue(summary['quality']['complete'], summary)
            self.assertEqual(summary['identification']['state_name'], 'complete')
            self.assertTrue(summary['revoked'])
            control = [r for r in rows if r['kind'] == 'control']
            centres = [r['pitch_requested_rad'] for r in control]
            reference = [r['pitch_adopted_rad'] for r in control]
            self.assertLess(max(centres)-min(centres), 1e-6, 'host centre must remain constant')
            self.assertGreater(max(reference)-min(reference), 0.019, 'MCU must generate both sides')
            self.assertLess(max(abs(r-c) for r, c in zip(reference, centres)), 0.01001)
            self.assertAlmostEqual(reference[-1], centres[-1], places=5, msg='normal finish returns to centre')
            self.assertEqual(metadata['recipe']['position_amplitude_rad'], .01)
            self.assertEqual(metadata['recipe']['amplitude_nm'], 0)
            from sentinel_pitch_position_fit import fit_rows
            fitted, *_ = fit_rows(metadata, rows)
            self.assertGreater(fitted['samples'], 120)
            self.assertIsNotNone(fitted['selected'])

    def setUp(self):
        # 后端是 CTest 的必备产物: 缺失必须失败, 不能用 skip 把整条端到端悄悄绕过.
        self.assertTrue(BACKEND.is_file(), f'缺少原生后端 {BACKEND}; 请先构建 ec_sentinel_app_host')

    def test_full_software_run_produces_verified_capture(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'run'
            command = [
                sys.executable, str(SCRIPT),
                '--backend', str(BACKEND), '--output', str(output),
                '--arm', '--run', '144',
                '--physical-low', '-0.4', '--physical-high', '0.4', '--excursion', '0.02',
                '--max-temperature', '60', '--max-torque', '3', '--max-speed', '1.5', '--max-error', '0.03',
                '--amplitude-nm', '0.03', '--half-period-us', '100000',
                '--baseline-us', '200000', '--cycles', '10', '--final-hold-us', '200000',
                '--stream-duration-us', '5000000',
            ]
            done = subprocess.run(command, capture_output=True, text=True, timeout=120,
                                  env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)

            manifest = json.loads((output / 'manifest.json').read_text(encoding='utf-8'))
            summary = json.loads((output / 'summary.json').read_text(encoding='utf-8'))
            metadata, rows = raw.load(output / 'pitch.sraw')

            self.assertEqual(manifest['source'], 'APP_MIT_SIMULATION_ONLY')
            self.assertFalse(manifest['hardware_ready'])
            self.assertFalse(manifest['deployable_gains'])
            self.assertIsNone(summary['error'], summary)
            self.assertEqual(summary['cleanup_errors'], [])
            self.assertTrue(summary['revoked'])
            self.assertEqual(manifest['identification']['state_name'], 'complete')
            self.assertTrue(summary['stream_final']['finished'])
            self.assertTrue(summary['quality']['complete'])
            self.assertEqual(summary['quality']['lost_max'], 0)
            self.assertEqual(summary['quality']['sequence_gaps'], 0)

            status = summary['stream_final']['last_status']
            self.assertEqual(status['dropped'], 0, '固件侧不得有丢样')
            self.assertEqual(status['buffered'], 0, '排空后不得残留缓存')
            self.assertEqual(len(rows), status['generated'], '解码条数必须等于 MCU 生成的条数')
            self.assertTrue(rows)
            self.assertEqual(len(rows), summary['records'])
            self.assertEqual(sum(row['gap'] for row in rows), 0)
            self.assertEqual(max(row['lost'] for row in rows), 0)
            self.assertEqual(metadata['run'], 144)
            self.assertEqual(metadata['pitch_feedback']['source'], 'mcu_parameter_snapshot')
            for param in metadata['parameters']:
                if param['id'] in (0x904, 0x905, 0x906):
                    self.assertFalse(param['writable'])
                    self.assertEqual(param['min'], param['max'])
            controls = [row for row in rows if row['kind'] == 'control']
            matched = [row for row in controls if row['pitch_feedback_status'] == 'fresh']
            self.assertGreater(len(matched), len(controls) * .9)
            self.assertTrue(all(row['pitch_reported_torque_nm'] is not None for row in matched))
            self.assertTrue(all(row['pitch_mos_temperature_c'] is not None for row in matched))
            self.assertTrue(all(row['pitch_feedback_age_us'] >= 0 for row in matched))
            analysis = json.loads((output / 'analysis.json').read_text(encoding='utf-8'))
            self.assertEqual(analysis['feedback_matched_records'], len(matched))
            self.assertFalse(analysis['deployable_gains'])

            # manifest 的自引用必须指向真实文件, 04 那次为空字符串是缺陷.
            for key in ('sraw', 'csv', 'manifest', 'summary', 'analysis'):
                self.assertTrue(manifest['files'][key], f'files.{key} 不得为空')
                self.assertTrue(Path(manifest['files'][key]).is_file(), f'files.{key} 必须存在')
