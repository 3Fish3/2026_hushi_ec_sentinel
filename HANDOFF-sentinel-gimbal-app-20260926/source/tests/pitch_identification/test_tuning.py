"""云台辅助页: 同一会话抬升/保持/采集, 完成后恢复人工调参."""
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE / 'robots/sentinel/host'))

import sentinel_client as wire
from sentinel_console_model import MaintenanceSession
from sentinel_pitch_tuning import capture_view, plan_from_fields

BACKEND = Path(os.environ.get('SENTINEL_PITCH_BACKEND', str(SOURCE / 'build/host-tests/ec_sentinel_app_host')))
FIELDS = {
    'level': '0', 'lift_speed': '0.05', 'lift_timeout': '8', 'settle_error': '0.005',
    'settle_speed': '0.02', 'settle_seconds': '0.12', 'wait_seconds': '8',
    'amplitude': '0.03', 'half_period': '0.1', 'cycles': '3', 'baseline': '0.2',
    'final_hold': '0.2', 'capture_duration': '3', 'post_hold': '0.1', 'hold_timeout': '1',
    'physical_low': '-0.4', 'physical_high': '0.4', 'excursion': '0.02',
    'max_temperature': '60', 'max_torque': '3', 'max_speed': '1.5', 'max_error': '0.03',
}


def wait_for(action, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = action()
        if predicate(value):
            return value
        time.sleep(0.01)
    raise AssertionError(f'等待状态超时: {value}')


class TuningFormTests(unittest.TestCase):
    def test_manual_mode_does_not_require_lift_settings(self):
        values = dict(FIELDS, level='', lift_speed='')
        plan, wait = plan_from_fields(values, lift=False)
        self.assertIsNone(plan.lift)
        self.assertEqual(wait, 8.0)
        self.assertEqual(plan.recipe.half_period_us, 100000)

    def test_nonfinite_values_and_fractional_cycles_are_rejected(self):
        for key, value in [('level', 'nan'), ('cycles', '2.5'), ('wait_seconds', '-1'),
                           ('capture_duration', 'inf'), ('settle_error', '0.1')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                plan_from_fields(dict(FIELDS, **{key: value}), lift=True)


class NativeTuningTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(BACKEND.is_file(), '缺少原生后端, 请先构建')
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.session = MaintenanceSession.connect(
            wire.ProcessTransport(BACKEND, extra_args=['--plant', '--fixture', 'gimbal',
                                                       '--pitch-identification']), 'gimbal')
        self.addCleanup(self.session.close)

    def test_level_then_capture_same_session_and_manual_tuning_still_works(self):
        session = self.session
        identity = session.peer.snapshot()['session']
        plan, wait = plan_from_fields(FIELDS, lift=True)
        with patch.object(session.peer, 'hello', wraps=session.peer.hello) as hello, \
                patch.object(session.peer, 'param_write', wraps=session.peer.param_write) as write:
            session.start_assisted(plan, self.directory / 'first', wait_s=wait)
            ready = wait_for(session.assistance.snapshot, lambda value: value['state'] in ('READY', 'FAILED'))
            self.assertEqual(ready['state'], 'READY', ready)
            self.assertEqual(session.peer.snapshot()['pitch_stream']['records'], 0)
            for action in (lambda: session.submit({'pitch': '0'}),
                           lambda: session.write_parameter(0x901, 12),
                           lambda: session.align('pitch', 0), session.authorize):
                with self.assertRaises(ValueError):
                    action()
            session.assistance.capture()
            wait_for(lambda: session.assistance.active, lambda active: not active)
            done = session.assistance.snapshot()
            self.assertEqual(done['state'], 'COMPLETE', done)
            self.assertTrue(done['report']['lift']['settled'])
            self.assertTrue(done['report']['quality']['complete'])
            self.assertEqual(identity, session.peer.snapshot()['session'])
            hello.assert_not_called()
            write.assert_not_called()
        view = capture_view(self.directory / 'first/pitch.sraw')
        self.assertGreater(view['control_records'], 0)
        self.assertEqual(view['gaps'], 0)
        self.assertTrue(any(abs(x) > 0.01 for x in view['series']['pitch_feedforward_torque_nm']))
        self.assertTrue(all(len(x) == len(view['times']) for x in view['series'].values()))
        first_run = view['metadata']['run']
        session.authorize()
        zero_before = session.read_parameter(0x914)['value']
        result = session.write_parameter(0x901, 11.0)
        self.assertEqual(result['ack']['result'], 0)
        self.assertAlmostEqual(result['readback']['value'], 11.0)
        damping = session.write_parameter(0x902, 0.5)
        self.assertAlmostEqual(damping['readback']['value'], 0.5)
        self.assertEqual(session.read_parameter(0x914)['value'], zero_before,
                         '修改 MIT kp/kd 不应改变关节水平零偏')
        session.submit({'pitch': '0.001'})
        self.assertIsNotNone(session.last_request)
        session.stop()
        # 再次采集使用同一会话递增 run_id, 不需要重新连接或随机碰运气.
        plan, wait = plan_from_fields(FIELDS, lift=False)
        session.start_assisted(plan, self.directory / 'second', capture=True, wait_s=wait)
        wait_for(lambda: session.assistance.active, lambda active: not active)
        self.assertEqual(session.assistance.snapshot()['state'], 'COMPLETE', session.assistance.snapshot())
        metadata = capture_view(self.directory / 'second/pitch.sraw')['metadata']
        self.assertGreater(metadata['run'], first_run)

    def test_stop_while_ready_releases_target_and_finishes_worker(self):
        plan, wait = plan_from_fields(FIELDS, lift=False)
        self.session.start_assisted(plan, self.directory / 'stop', wait_s=wait)
        ready = wait_for(self.session.assistance.snapshot, lambda value: value['state'] in ('READY', 'FAILED'))
        self.assertEqual(ready['state'], 'READY', ready)
        self.session.stop()
        self.session.assistance.wait()
        result = self.session.assistance.snapshot()
        self.assertTrue(result['report']['revoked'], result)
        self.assertIsNone(self.session.peer._source)
        self.assertEqual(self.session.peer.snapshot()['pitch_stream']['records'], 0)
        self.assertFalse(self.session.assistance.active)


class TuningWidgetTests(unittest.TestCase):
    def test_real_widgets_contain_assisted_and_manual_controls(self):
        import dearpygui.dearpygui as dpg
        from sentinel_console import Console
        dpg.create_context()
        try:
            console = Console()
            try:
                for tag in ('parameters_tab', 'assist_tab', 'manual_submit', 'assist_prepare',
                            'assist_capture', 'assist_stop', 'assist_path', 'assist_effort'):
                    self.assertTrue(dpg.does_item_exist(tag), tag)
                self.assertEqual(dpg.get_value('assist_level'), '0')
                self.assertTrue(dpg.get_value('assist_lift'))
                dpg.set_value('part', '底盘')
                console.scope_changed()
                self.assertFalse(dpg.get_item_configuration('assist_tab')['show'])
            finally:
                console.close()
        finally:
            dpg.destroy_context()

    def test_assistance_locks_manual_widgets_and_stop_restores_them(self):
        import dearpygui.dearpygui as dpg
        from sentinel_console import Console
        dpg.create_context()
        try:
            console = Console(backend=BACKEND)
            try:
                console.session = MaintenanceSession.connect(wire.ProcessTransport(
                    BACKEND, extra_args=['--plant', '--fixture', 'gimbal', '--pitch-identification']), 'gimbal')
                console.render_parameters(console.session.parameters)
                plan, wait = plan_from_fields(FIELDS, lift=False)
                with tempfile.TemporaryDirectory() as folder:
                    console.session.start_assisted(plan, Path(folder) / 'widgets', wait_s=wait)
                    wait_for(console.session.assistance.snapshot,
                             lambda value: value['state'] in ('READY', 'FAILED'))
                    console.refresh()
                    self.assertFalse(dpg.get_item_configuration('manual_submit')['enabled'])
                    self.assertFalse(dpg.get_item_configuration('apply_2305')['enabled'])
                    console.stop()
                    console.session.assistance.wait()
                    console.refresh()
                    self.assertTrue(dpg.get_item_configuration('manual_submit')['enabled'])
                    self.assertTrue(dpg.get_item_configuration('apply_2305')['enabled'])
            finally:
                console.close()
        finally:
            dpg.destroy_context()
