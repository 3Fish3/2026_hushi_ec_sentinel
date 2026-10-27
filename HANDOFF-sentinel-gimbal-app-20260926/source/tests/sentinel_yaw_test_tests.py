"""参数化主 Yaw 测试的离线契约；全部设备接口使用替身，不连接硬件。"""
import contextlib
import io
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

HOST = Path(__file__).resolve().parents[1] / 'robots/sentinel/host'
sys.path.insert(0, str(HOST))
import sentinel_yaw_test as yaw


def args(*values):
    return yaw.parser().parse_args(list(values))


def joint(angle=0, rate=0):
    return dict(angle_rad=angle, rate_rad_s=rate, position_valid=True, velocity_valid=True,
                position_age_ms=1, velocity_age_ms=1)


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Transport:
    def __init__(self, *unused):
        self.closed = False

    def close(self):
        self.closed = True


class Sound:
    returncode = 0

    def poll(self):
        return 0

    def communicate(self):
        return ('', '')


class Client:
    def __init__(self, transport):
        self.transport = transport
        self.authority = False
        self.active = False
        self.calls = []
        self.hello_info = dict(config_epoch=1, reference_epoch=0)
        self.params = {0x401: .024, 0x402: .4, 0x403: 0, 0x404: 2.5,
                       0x408: 7.5, 0x409: 0, 0x40a: 0}
        self.targets_sent = []
        self.fail_motion = self.fail_cancel = self.fail_write = False
        self.initial_active = self.wrong_device = self.reject_auth = self.unknown_auth = False
        self.fail_feedback = self.epoch_change = self.high_temp = False

    def hello(self):
        self.calls.append('hello')
        return self.hello_info

    def authorize(self, grant, source):
        self.calls.append(('authorize', grant, source))
        self.authority = grant
        if grant and self.unknown_auth:
            raise TimeoutError('authorize ACK lost')
        return dict(result_name='NOT_PERMITTED' if grant and self.reject_auth else 'ACCEPTED')

    def status(self, page):
        if page == 0:
            j = joint(.1)
            if self.active and self.fail_feedback:
                j['position_valid'] = False
            return {'joints': {'main_yaw': j}}
        return dict(authority=self.authority or self.initial_active,
                    source='maintenance' if self.authority else 'none', manual_stop=False,
                    config_epoch=1, reference_epoch=1 if self.active and self.epoch_change else 0,
                    domains=[dict(domain='main_yaw', execute=self.active or self.initial_active)])

    def diagnostics(self, page):
        return {'devices': [dict(domain='main_yaw', installed=True, online=True, bus=1,
                                 device_id=6 if self.wrong_device else 4, feedback_id=0x208,
                                 protocol=2, temperature_c=60 if self.high_temp and self.active else 26)]}

    def targets(self, values):
        self.calls.append(('targets', bool(values)))
        if values:
            self.targets_sent.append(values)
            if self.fail_motion:
                raise TimeoutError('motion ACK lost')
        elif self.fail_cancel and self.targets_sent:
            raise TimeoutError('cancel ACK lost')
        self.active = bool(values)
        return dict(result_name='ACCEPTED')

    def param_read(self, pid):
        return {'entries': [dict(id=pid, domain='main_yaw', value=self.params[pid], min=0,
                                 max=100, writable=True)]}

    def param_write(self, pid, value):
        self.calls.append(('write', pid, value))
        self.params[pid] = value
        if self.fail_write and value == .32:
            raise TimeoutError('write ACK lost but mutation applied')
        return dict(result_name='ACCEPTED')


class Tests(unittest.TestCase):
    def test_default_is_hardware_free_preview(self):
        with patch.object(yaw, 'BoundedSerial', side_effect=AssertionError('serial opened')), \
             patch.object(yaw, 'identity', side_effect=AssertionError('observer opened')), \
             patch.object(yaw.subprocess, 'Popen', side_effect=AssertionError('process launched')), \
             contextlib.redirect_stdout(io.StringIO()):
            result = yaw.run(args())
        self.assertFalse(result['execute'])
        self.assertAlmostEqual(result['angle_deg'], 5)

    def test_rejects_unsafe_or_nonfinite_inputs(self):
        for name, value in [('--angle-deg', 'nan'), ('--angle-deg', '11'), ('--angle-deg', '0'),
                            ('--ramp-rad-s', '0'), ('--stop-speed', '.6'), ('--margin-deg', '2'),
                            ('--hold-s', 'inf'), ('--settle-s', '10'), ('--sample-s', '.1'),
                            ('--velocity-ki', '-1'), ('--velocity-kp', 'nan'), ('--capture-ms', '400')]:
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                yaw.plan(args(name, value))
        with self.assertRaises(ValueError):
            yaw.plan(args('--angle-deg', '10', '--ramp-rad-s', '.01'))

    def test_single_parameter_and_required_confirmation(self):
        for flags in [('--velocity-ki', '.8', '--velocity-kp', '.08'), ('--execute',),
                      ('--execute', '--port', 'a', '--observer-port', 'b'),
                      ('--execute', '--safety-confirmed', '--port', 'a', '--observer-port', 'a')]:
            with self.subTest(flags=flags), self.assertRaises(ValueError):
                yaw.plan(args(*flags))

    def test_capture_window_validation(self):
        for value in ('-1', '10', 'nan'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                yaw.plan(args('--capture', '--capture-at-s', value))
        self.assertGreater(yaw.plan(args('--capture'))['capture_at_s'], 0)

    def test_ramp_sign_base_and_hold(self):
        a = args('--angle-deg', '5')
        self.assertEqual(yaw.goal_at(1.2, .3, a), 1.2)
        self.assertAlmostEqual(yaw.goal_at(1.2, 1.4, a), 1.24)
        self.assertAlmostEqual(yaw.goal_at(1.2, 10, a), 1.2 + math.radians(5))
        a.direction = 'negative'
        self.assertAlmostEqual(yaw.goal_at(-.2, 10, a), -.2 - math.radians(5))

    def test_safety_both_directions_and_stale_feedback(self):
        a = args()
        self.assertIn('position_bound', yaw.stop_reasons(joint(math.radians(7)), 0, a))
        self.assertIn('position_bound', yaw.stop_reasons(joint(math.radians(-2)), 0, a))
        self.assertIn('speed_bound', yaw.stop_reasons(joint(rate=.6), 0, a))
        j = joint(); j['position_age_ms'] = 21
        self.assertIn('invalid_or_stale_feedback', yaw.stop_reasons(j, 0, a))
        a.direction = 'negative'
        self.assertIn('position_bound', yaw.stop_reasons(joint(math.radians(-7)), 0, a))
        self.assertEqual(yaw.stop_reasons(joint(math.radians(-5)), 0, a), [])

    def simulate(self, configure=lambda client: None, extra=()):
        clock = Clock()
        transport = Transport()
        client = Client(transport)
        configure(client)
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / 'trial'
            a = args('--execute', '--safety-confirmed', '--port', 'mock-maint',
                     '--observer-port', 'mock-observe', '--output', str(output),
                     '--angle-deg', '.02', '--hold-s', '.1', '--settle-s', '.1',
                     '--post-s', '.1', *extra)
            with patch.object(yaw, 'BoundedSerial', return_value=transport), \
                 patch.object(yaw.wire, 'SentinelClient', return_value=client), \
                 patch.object(yaw, 'identity', return_value={'build_id': yaw.DEFAULT_BUILD}), \
                 patch.object(yaw.subprocess, 'Popen', return_value=Sound()), \
                 patch.object(yaw.time, 'monotonic', clock.monotonic), \
                 patch.object(yaw.time, 'sleep', clock.sleep), \
                 contextlib.redirect_stdout(io.StringIO()):
                report = yaw.run(a)
            saved = json.loads((output / 'summary.json').read_text())
            self.assertEqual(report, saved)
            if client.calls:
                self.assertTrue(transport.closed)
            else:
                self.assertFalse(transport.closed)  # 身份失败时从未打开维护串口
        return report, client

    def test_complete_records_and_restores_physical_gain(self):
        report, client = self.simulate(extra=('--velocity-ki', '.8'))
        self.assertTrue(report['completed'])
        self.assertTrue(report['overrides_restored'])
        self.assertTrue(report['final_standby'])
        self.assertIn(('write', 0x402, .32), client.calls)
        self.assertIn(('write', 0x402, .4), client.calls)
        self.assertEqual(client.params[0x402], .4)
        self.assertTrue(all(v[0][0] == 'main_yaw' and v[0][1] == 'position' for v in client.targets_sent))
        self.assertEqual(report['base_rad'], .1)

    def test_motion_timeout_still_cancels_restores_releases(self):
        report, client = self.simulate(lambda c: setattr(c, 'fail_motion', True), ('--velocity-ki', '.8'))
        self.assertFalse(report['completed'])
        self.assertTrue(report['cancel_accepted'])
        self.assertTrue(report['overrides_restored'])
        self.assertTrue(report['release_accepted'])

    def test_cancel_failure_does_not_skip_restore_or_release(self):
        report, client = self.simulate(lambda c: setattr(c, 'fail_cancel', True), ('--velocity-ki', '.8'))
        self.assertFalse(report['completed'])
        self.assertTrue(report['cleanup_errors'])
        self.assertTrue(report['overrides_restored'])
        self.assertTrue(report['release_accepted'])
        self.assertEqual(client.params[0x402], .4)

    def test_unknown_write_outcome_restores_without_motion(self):
        report, client = self.simulate(lambda c: setattr(c, 'fail_write', True), ('--velocity-ki', '.8'))
        self.assertFalse(report['completed'])
        self.assertFalse(client.targets_sent)
        self.assertTrue(report['overrides_restored'])
        self.assertEqual(client.params[0x402], .4)

    def test_rejects_existing_authority_without_touching_targets(self):
        report, client = self.simulate(lambda c: setattr(c, 'initial_active', True))
        self.assertFalse(report['completed'])
        self.assertFalse(any(isinstance(c, tuple) and c[0] in ('targets', 'authorize') for c in client.calls))

    def test_device_mapping_mismatch_no_motion(self):
        report, client = self.simulate(lambda c: setattr(c, 'wrong_device', True))
        self.assertFalse(report['completed'])
        self.assertFalse(client.targets_sent)

    def test_rejected_or_unknown_authorization_cleanup(self):
        for name in ('reject_auth', 'unknown_auth'):
            report, client = self.simulate(lambda c: setattr(c, name, True))
            self.assertFalse(report['completed'])
            self.assertFalse(client.targets_sent)
            self.assertIn(('authorize', False, 'maintenance'), client.calls)

    def test_rejects_unsafe_inherited_parameters(self):
        for pid, value in ((0x401, .8), (0x404, 3), (0x403, .01), (0x408, 20)):
            report, client = self.simulate(lambda c: c.params.update({pid: value}))
            self.assertFalse(report['completed'])
            self.assertFalse(client.targets_sent)

    def test_stale_feedback_epoch_change_temperature_stop(self):
        for name in ('fail_feedback', 'epoch_change', 'high_temp'):
            with self.subTest(name=name):
                report, client = self.simulate(lambda c: setattr(c, name, True))
                self.assertFalse(report['completed'])
                self.assertTrue(report['cancel_accepted'])
                self.assertTrue(report['release_accepted'])

    def test_wrong_firmware_does_not_request_authority(self):
        report, client = self.simulate(extra=('--expected-build', 'unexpected-firmware'))
        self.assertFalse(report['completed'])
        self.assertFalse(client.calls)

    def test_sound_failure_cancels_restores_and_releases(self):
        with patch.object(Sound, 'returncode', 1):
            report, client = self.simulate(extra=('--velocity-ki', '.8'))
        self.assertFalse(report['completed'])
        self.assertFalse(client.targets_sent)
        self.assertTrue(report['overrides_restored'])
        self.assertTrue(report['release_accepted'])

    def test_candidate_rejected_without_motion(self):
        report, client = self.simulate(extra=('--velocity-ki', '3'))
        self.assertFalse(report['completed'])
        self.assertFalse(client.targets_sent)
        self.assertFalse(any(isinstance(c, tuple) and c[0] == 'write' for c in client.calls))

    def test_refuses_existing_output(self):
        with tempfile.TemporaryDirectory() as temp:
            a = args('--execute', '--safety-confirmed', '--port', 'a', '--observer-port', 'b', '--output', temp)
            with patch.object(yaw, 'identity', side_effect=AssertionError('opened')), self.assertRaises(FileExistsError):
                yaw.run(a)

    def test_synchronous_requests_have_bounded_timeout(self):
        transport = object.__new__(yaw.BoundedSerial)
        with patch.object(yaw.wire.SerialTransport, 'exchange', return_value='reply') as method:
            self.assertEqual(transport.exchange(b'frame', 5), 'reply')
        self.assertEqual(method.call_args.args, (b'frame', .1))

    def test_raw_analysis_accepts_only_real_main_yaw_pair(self):
        with tempfile.TemporaryDirectory() as temp:
            for can_id in (0x208, 0x20a):
                path = Path(temp) / (str(can_id) + '.ecraw')
                header = struct.pack('<4B7I', 1, 5, 4, 1, 0, 1, 100, 100, 0, 0, 123)
                control = header + struct.pack('<6f2I', .1, .1, 0, 0, .02, .001, 0, 1)
                header = struct.pack('<4B7I', 1, 4, 5, 4, 1, 1, 100, 100, 0, 0, 123)
                can = header + struct.pack('<I', 1 | (8 << 8) | (can_id << 16)) + bytes.fromhex('0000000000641a00') + bytes(20)
                yaw.raw.save(path, dict(layout_id=yaw.raw.LAYOUT, run=123, start_us=100,
                                        end_us=200, lost=0), [control, can])
                if can_id == 0x208:
                    result = yaw.analyse_raw(path)
                    self.assertTrue(result['mapping_correct'])
                    self.assertEqual(result['effort_raw_range'], [100, 100])
                else:
                    with self.assertRaises(RuntimeError):
                        yaw.analyse_raw(path)

    def test_swd_config_has_no_actuating_commands(self):
        with tempfile.TemporaryDirectory() as temp:
            swd = yaw.SwdObserver(Path(temp))
            swd.ready.set()
            process = Sound()
            process.stdout = io.StringIO('SWD_COMPLETE_NO_HALT_NO_RESET\n')
            process.wait = lambda **kw: 0
            with patch.object(yaw.subprocess, 'Popen', return_value=process):
                swd.start()
                swd.thread.join(2)
            text = (Path(temp) / 'swd.cfg').read_text()
            prohibited = ('halt', 'reset', 'resume', 'mww', 'mwh', 'mwb', 'write_memory', 'flash', 'program')
            self.assertFalse(any(line.strip().split()[0] in prohibited
                                 for line in text.splitlines() if line.strip()))
            self.assertIn('configure -event examine-end {}', text)
            self.assertIn('read_memory', text)
            self.assertTrue(swd.close()['complete'])


if __name__ == '__main__':
    unittest.main()
