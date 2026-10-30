"""单 UART Pitch 辨识编排: 预检顺序, 有界写入, 故障即撤销.

假通道满足监督器所需的全部遥测契约 (状态页/诊断页/目标 ACK), 并补连续流与辨识接口;
时钟与睡眠全部注入, 因此不依赖墙钟或任何串口设备.
"""
import json
import struct
import sys
import time
import unittest
from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robots/sentinel/host'))

import sentinel_async as async_mod  # noqa: E402
import sentinel_pitch_experiment as experiment  # noqa: E402
import sentinel_raw as raw  # noqa: E402
from sentinel_pitch_hold_model import PhysicalBounds, SupervisorLimits  # noqa: E402

RUN = 0x00C0FFEE


def make_record(run, sequence, sampled_us):
    return struct.pack('<4B7I', 1, 5, 6, 0, sequence, 0, sampled_us, sampled_us + 5, 0, 0xffffffff, run) + \
        struct.pack('<6fII', 0.1, 0.1, 0.1, 0.0, 0.04, 0.001, 0, 0)


def make_plan(post_hold_s=0.05, hold_timeout_s=1.0, stream_packet_min=2):
    recipe = experiment.ExperimentRecipe(run=RUN, amplitude_nm=0.04, half_period_us=10000,
                                         baseline_us=5000, cycles=2, final_hold_us=5000)
    return experiment.ExperimentPlan(
        recipe=recipe, stream_duration_us=recipe.excitation_us + int(post_hold_s * 1e6) + 200000,
        physical=PhysicalBounds(-0.377, 0.351), excursion_rad=0.005,
        limits=SupervisorLimits(max_temperature_c=55, max_effort_nm=2, max_speed_rad_s=0.6,
                                max_tracking_error_rad=0.03, max_run_s=30),
        stream_packet_min=stream_packet_min, hold_timeout_s=hold_timeout_s, post_hold_s=post_hold_s)


class ControlledClock:
    def __init__(self, start=1000.0):
        self.now = start

    def __call__(self):
        return self.now

    def advance(self, delay):
        self.now += delay


class FakePeer:
    """假 APP 通道: 预检/授权/连续流/辨识接口 + 可注入故障, 时间来自注入时钟."""

    def __init__(self):
        self.clock = ControlledClock()
        self.session, self.config_epoch, self.reference_epoch = 101, 5, 12
        self.error = ''
        self.pitch_angle = -0.05
        self.pitch_rate = 0.0
        self.follow_targets = False
        self.target_history = []
        self.capture_started_at = None
        self.manual_stop = False
        self.authority = True
        self.control_source = 'maintenance'
        self.status_sequence, self.diagnostics_sequence = 10, 20
        self.ack_sequence = 50
        self.target_rejections = 0
        self.last_target_ack = {'session': self.session, 'sequence': self.ack_sequence, 'result': 0,
                                'result_name': 'ACCEPTED'}
        self.target_source = None
        self.target_source_calls = 0
        self.calls = []
        self.capture = {'run': None, 'active': False, 'packets': 0, 'records': 0, 'fault': None,
                        'confirmed_by': None, 'session': self.session, 'last_ack': None}
        self.sink = None
        self.stream_future = None
        self.ident_state = 'idle'
        self.ident_run = 0
        self.queries = 0
        self.available = True
        self.records = 0
        self.ident_starts = 0
        self.completed_at = None
        self.stop_at = None

    # -- snapshot & pages -------------------------------------------------
    def snapshot(self):
        now = self.clock()
        page0 = {
            'session': self.session, 'sequence': self.status_sequence, 'page': 0,
            'config_epoch': self.config_epoch, 'reference_epoch': self.reference_epoch,
            'received_at_monotonic': now, 'authority': self.authority,
            'manual_stop': self.manual_stop, 'source': self.control_source,
            'joints': {'pitch': {'angle_rad': self.pitch_angle, 'rate_rad_s': self.pitch_rate,
                                 'position_valid': True, 'velocity_valid': True,
                                 'position_age_ms': 10, 'velocity_age_ms': 10}},
        }
        page1 = {
            'session': self.session, 'sequence': self.status_sequence, 'page': 1,
            'config_epoch': self.config_epoch, 'reference_epoch': self.reference_epoch,
            'received_at_monotonic': now, 'authority': self.authority,
            'manual_stop': self.manual_stop, 'source': self.control_source,
            'domains': [{'domain': 'pitch', 'validity': 'valid', 'mode': 'position',
                         'execute': True, 'reason': 'none', 'target': self.pitch_angle,
                         'measurement': self.pitch_angle, 'age_ms': 10, 'velocity_age_ms': 10}],
        }
        return {'connected': True, 'session': self.session, 'config_epoch': self.config_epoch,
                'reference_epoch': self.reference_epoch, 'error': self.error,
                'status': {0: page0, 1: page1}, 'last_target_ack': dict(self.last_target_ack),
                'target_rejections': self.target_rejections,
                'last_target_sent_sequence': self.ack_sequence,
                'pitch_stream': dict(self.capture), 'pitch_stream_error': ''}

    def status(self, page=0):
        future = Future()
        future.set_result(dict(self.snapshot()['status'][page]))
        return future

    def diagnostics(self, page=0):
        future = Future()
        if page == 0:
            future.set_result({'session': self.session, 'page': 0, 'sequence': self.diagnostics_sequence,
                               'config_epoch': self.config_epoch, 'reference_epoch': self.reference_epoch,
                               'target_ttl_ms': 100, 'link_ttl_ms': 200, 'rejected_targets': 0,
                               'frame_rejects': 0, 'received_at_monotonic': self.clock()})
        else:
            future.set_result({'session': self.session, 'page': 1, 'sequence': self.diagnostics_sequence,
                               'devices': [{'slot': 6, 'domain': 'pitch', 'installed': True, 'bus': 2,
                                            'device_id': 1, 'feedback_id': 0x11, 'protocol': 128,
                                            'online': True, 'enabled': True, 'faulted': False,
                                            'age_ms': 10, 'temperature_c': 25, 'rotor_temperature_c': 26,
                                            'effort': 0.1, 'effort_unit': 'Nm'}],
                               'received_at_monotonic': self.clock()})
        return future

    # -- session protocol -------------------------------------------------
    def hello(self):
        self.calls.append(('hello',))
        future = Future()
        future.set_result({'identity': 'sentinel_app', 'startable': True, 'session': self.session,
                           'config_epoch': self.config_epoch, 'reference_epoch': self.reference_epoch,
                           'boot_id': 1, 'config_kind': 1, 'client_nonce': 7})
        return future

    def read_all_params(self):
        self.calls.append(('params',))
        future = Future()
        future.set_result([{'param_id': 0x10, 'value': 0.25}])
        return future

    def authorize(self, grant=True, source='maintenance'):
        self.calls.append(('authorize', bool(grant)))
        future = Future()
        future.set_result({'result': 0, 'result_name': 'ACCEPTED'})
        return future

    def set_target_source(self, source):
        self.calls.append(('source', source is not None))
        self.target_source = source

    def step_target_stream(self):
        """模拟 UART 线程调用目标源, 并推进接受的目标 ACK."""
        if self.target_source is None:
            return None
        target = self.target_source(self.clock(), {'config_epoch': self.config_epoch,
                                                  'reference_epoch': self.reference_epoch})
        if target is not None:
            value = target['setpoints'][0][2]
            self.target_history.append((self.clock(), value))
            if self.follow_targets:
                self.pitch_angle = value
            self.ack_sequence += 1
            self.last_target_ack = {'session': self.session, 'sequence': self.ack_sequence,
                                    'result': 0, 'result_name': 'ACCEPTED',
                                    'received_at_monotonic': self.clock()}
        return target

    # -- capture ----------------------------------------------------------
    def set_pitch_stream_sink(self, callback):
        self.sink = callback

    def start_pitch_stream(self, run, duration_us):
        self.capture_started_at = self.clock()
        self.calls.append(('start_stream', run))
        self.stream_future = Future()
        self.capture.update(run=run, active=True, packets=0, records=0, fault=None, confirmed_by=None)
        return self.stream_future

    def stop_pitch_stream(self, run):
        self.calls.append(('stop_stream', run))
        self.stop_at = self.clock()
        # 与固件快照保持同形: 完整性判定依赖最终 MCU 计数.
        self.capture.update(run=run, active=False, finished=True,
                            last_status={'generated': self.capture['records'], 'dropped': 0, 'buffered': 0})
        future = Future()
        future.set_result({'run': run, 'op': 2})
        return future

    def signal_pitch_capture_fault(self, code, detail):
        self.capture['fault'] = async_mod.CaptureFault(str(code), str(detail), self.capture['run'] or 0,
                                                      self.clock())
        return True

    def pitch_identification(self, op, **kwargs):
        self.calls.append(('ident', op, kwargs.get('run_id')))
        future = Future()
        if op == 'start':
            self.ident_starts += 1
            self.ident_run = kwargs['run_id']
            self.ident_state = 'baseline'
            future.set_result({'result': 0, 'available': self.available, 'run_id': self.ident_run,
                               'state_name': self.ident_state, 'fault_name': 'none'})
        elif op == 'query':
            self.queries += 1
            future.set_result({'result': 0, 'available': self.available, 'run_id': self.ident_run,
                               'state_name': self.ident_state, 'fault_name': 'none'})
        else:
            future.set_result({'result': 0, 'run_id': self.ident_run})
        return future

    # -- injected lane ----------------------------------------------------
    def deliver(self):
        """模拟 UART 线程: 目标续期, 连续流数据, 固件状态推进."""
        self.status_sequence += 1
        self.step_target_stream()
        for _ in range(3):  # 夹具每次推进送出多帧, 与固件 1kHz 轮询同量级
            if not self.capture['active'] or self.sink is None:
                break
            record = make_record(self.capture['run'], self.capture['packets'], int(self.clock() * 1e6))
            try:
                self.sink({'op': 3, 'run': self.capture['run'], 'error': 0}, [record])
            except Exception as error:  # 与真实客户端一致: sink 失败变成可见采集故障.
                self.capture['fault'] = async_mod.CaptureFault('sink', str(error), self.capture['run'],
                                                              self.clock())
                return
            self.capture['packets'] += 1
            self.capture['records'] += 1
            self.records += 1
            if self.stream_future is not None and not self.stream_future.done():
                self.stream_future.set_result({'run': self.capture['run'], 'confirmed_by': 'data'})
        if self.ident_state == 'baseline':
            self.ident_state = 'square'
        elif self.ident_state == 'square' and self.queries >= 2:
            self.ident_state = 'complete'
            self.completed_at = self.clock()


class _FailingFile:
    """头部写入成功后立即失败的磁盘, 用于验证可见写入故障与不完整数据保留."""

    def __init__(self, real):
        self.real = real
        self.writes = 0

    def write(self, blob):
        self.writes += 1
        if self.writes > 1:
            raise OSError('disk full')
        return self.real.write(blob)

    def flush(self):
        self.real.flush()

    def seek(self, *args):
        return self.real.seek(*args)

    def close(self):
        self.real.close()


def failing_opener(target):
    return _FailingFile(target.open('xb'))


class _ZeroWriter:
    """每次返回 0 字节, 模拟最恶劣的短写."""

    def __init__(self, real):
        self.real = real

    def write(self, blob):
        return 0

    def flush(self):
        self.real.flush()

    def seek(self, *args):
        return self.real.seek(*args)

    def close(self):
        self.real.close()


class _SlowRecordWriter:
    """头部很快, 记录写入很慢, 用于制造真正的 close() 超时."""

    def __init__(self, real, delay):
        self.real = real
        self.delay = delay
        self.calls = 0

    def write(self, blob):
        self.calls += 1
        if self.calls > 1:
            time.sleep(self.delay)
        return self.real.write(blob)

    def flush(self):
        self.real.flush()

    def seek(self, *args):
        return self.real.seek(*args)

    def close(self):
        self.real.close()


class _ShortWriter:
    """每次只写一部分并如实返回字节数, 模拟短写."""

    def __init__(self, real):
        self.real = real

    def write(self, blob):
        size = max(1, len(blob) - 1)
        self.real.write(blob[:size])
        return size

    def flush(self):
        self.real.flush()

    def seek(self, *args):
        return self.real.seek(*args)

    def close(self):
        self.real.close()


class _FailingAfterWrites:
    """允许固定次数的写入后失败, 用于区分已落盘条数与仅入队条数."""

    def __init__(self, real, fail_after):
        self.real = real
        self.writes = 0
        self.fail_after = fail_after

    def write(self, blob):
        self.writes += 1
        if self.writes > self.fail_after:
            raise OSError('disk full')
        return self.real.write(blob)

    def flush(self):
        self.real.flush()

    def seek(self, *args):
        return self.real.seek(*args)

    def close(self):
        self.real.close()


class ExperimentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)

    def run_case(self, peer, plan=None, **kwargs):
        plan = plan or make_plan()

        def sleep(delay):
            peer.clock.advance(delay)
            peer.deliver()

        return experiment.run_experiment(peer, plan, self.base / 'run', clock=peer.clock,
                                         sleep=sleep, **kwargs)

    def test_unavailable_firmware_rejects_before_authority_and_targets(self):
        peer = FakePeer()
        peer.available = False
        report = self.run_case(peer)
        self.assertIn('no Pitch identification policy', report['error'])
        self.assertFalse((self.base / 'run').exists(), '被拒绝的预检不得留下半成品目录')
        kinds = [call[0] for call in peer.calls]
        self.assertNotIn('authorize', kinds)
        self.assertNotIn('start_stream', kinds)
        self.assertNotIn(('source', True), [call[:2] for call in peer.calls])
        self.assertFalse(report['holding'])

    def test_existing_output_directory_is_refused_before_authority(self):
        (self.base / 'run').mkdir()
        peer = FakePeer()
        report = self.run_case(peer)
        self.assertIn('FileExistsError', report['error'])
        self.assertNotIn('authorize', [call[0] for call in peer.calls])

    def test_existing_directory_bytes_are_never_overwritten(self):
        target = self.base / 'run'
        target.mkdir()
        sentinel = target / 'summary.json'
        sentinel.write_bytes(b'user-owned data')
        report = self.run_case(FakePeer())
        self.assertIn('FileExistsError', report['error'])
        self.assertEqual(sentinel.read_bytes(), b'user-owned data')
        self.assertEqual(report['files'], {})

    def test_large_realistic_parameter_metadata_survives_writer(self):
        peer = FakePeer()
        def parameters():
            future = Future()
            future.set_result([{'param_id': value, 'value': 0.01} for value in range(64)])
            return future
        peer.read_all_params = parameters
        report = self.run_case(peer)
        self.assertIsNone(report['error'], report)
        metadata, _ = raw.load(self.base / 'run/pitch.sraw')
        self.assertEqual(len(metadata['parameters']), 64)

    def test_missing_final_marker_is_reported_not_silently_dropped(self):
        peer = FakePeer()
        original = peer.stop_pitch_stream
        def stop(run):
            result = original(run)
            peer.capture['finished'] = False
            return result
        peer.stop_pitch_stream = stop
        report = self.run_case(peer)
        self.assertIn('final marker missing', report['error'])
        self.assertFalse(report['quality']['complete'])
        self.assertTrue(report['revoked'])

    def test_normal_completion_keeps_hold_until_bounded_stop(self):
        peer = FakePeer()
        plan = make_plan(post_hold_s=0.05)
        report = self.run_case(peer, plan)
        self.assertIsNone(report['error'], report)
        self.assertTrue(report['holding'])
        self.assertTrue(report['revoked'])
        self.assertEqual(report['identification']['state_name'], 'complete')
        self.assertGreaterEqual(report['records'], 2)
        metadata, rows = raw.load(self.base / 'run/pitch.sraw')
        self.assertEqual(metadata['run'], RUN)
        self.assertEqual(len(rows), report['records'])
        self.assertTrue((self.base / 'run/manifest.json').exists())
        self.assertTrue((self.base / 'run/summary.json').exists())
        self.assertTrue((self.base / 'run/pitch.csv').exists())
        calls = [call[:2] for call in peer.calls]
        self.assertLess(calls.index(('authorize', True)), calls.index(('start_stream', RUN)))
        self.assertLess(calls.index(('start_stream', RUN)), calls.index(('ident', 'start')))
        self.assertEqual(calls[-1], ('authorize', False), '撤销必须是最后一步')
        self.assertNotIn(('ident', 'cancel'), calls, '正常完成不自动取消或重置激励')
        if peer.completed_at is None or peer.stop_at is None:
            self.fail('完成与停止时刻必须被记录')
        self.assertGreaterEqual(peer.stop_at - peer.completed_at, plan.post_hold_s,
                                '完成后必须继续保持直到有界窗口结束')

    def lift_plan(self, **changes):
        lift = experiment.LiftPlan(target_rad=0.0, speed_rad_s=0.05, timeout_s=8.0,
                                   settle_s=0.12, position_tolerance_rad=0.002,
                                   speed_tolerance_rad_s=0.02)
        return replace(make_plan(), lift=replace(lift, **changes))

    def test_lift_from_lower_boundary_settles_before_capture(self):
        peer = FakePeer()
        peer.pitch_angle = make_plan().physical.low
        peer.follow_targets = True
        plan = replace(self.lift_plan(), physical=PhysicalBounds(-0.10, 0.351))
        peer.pitch_angle = plan.physical.low
        report = self.run_case(peer, plan)
        self.assertIsNone(report['error'], report)
        self.assertTrue(report['lift']['settled'])
        self.assertAlmostEqual(report['lift']['initial_rad'], plan.physical.low)
        self.assertGreaterEqual(report['lift']['duration_s'], 2.0 + plan.lift.settle_s)
        self.assertGreaterEqual(peer.capture_started_at, report['lift']['settled_at_monotonic'])
        self.assertAlmostEqual(peer.pitch_angle, 0.0)
        history = peer.target_history
        self.assertAlmostEqual(history[0][1], plan.physical.low)
        for (at, value), (next_at, next_value) in zip(history, history[1:]):
            self.assertLessEqual(abs(next_value - value),
                                 plan.lift.speed_rad_s * (next_at - at) + 1e-9)
        self.assertEqual([call for call in peer.calls if call == ('source', True)], [('source', True)])
        metadata, _ = raw.load(self.base / 'run/pitch.sraw')
        self.assertEqual(metadata['lift']['target_rad'], 0.0)
        self.assertEqual(metadata['envelope_rad'], [-0.005, 0.005])

    def test_lift_stuck_position_times_out_without_capture(self):
        peer = FakePeer()
        peer.pitch_angle = -0.01
        report = self.run_case(peer, self.lift_plan(timeout_s=0.5))
        self.assertIn('lift did not settle', report['error'])
        self.assertNotIn('start_stream', [call[0] for call in peer.calls])
        self.assertFalse(report['lift']['settled'])
        self.assertTrue(report['revoked'])
        self.assertIsNone(peer.target_source)

    def test_lift_rejects_unreachable_timing_before_authority(self):
        peer = FakePeer()
        report = self.run_case(peer, self.lift_plan(timeout_s=0.2))
        self.assertIn('lift timeout', report['error'])
        self.assertNotIn('authorize', [call[0] for call in peer.calls])

    def test_lift_waits_for_real_motion_to_stop(self):
        peer = FakePeer()
        peer.pitch_angle = 0.0
        peer.pitch_rate = 0.03
        report = self.run_case(peer, self.lift_plan(timeout_s=0.5))
        self.assertIn('lift did not settle', report['error'])
        self.assertIsNone(peer.capture_started_at)

    def test_lift_operator_stop_revokes_without_starting_capture(self):
        peer = FakePeer()
        peer.follow_targets = True
        stop = experiment.threading.Event()

        def sleep(delay):
            peer.clock.advance(delay)
            peer.deliver()
            if peer.pitch_angle > -0.045:
                stop.set()

        report = experiment.run_experiment(peer, self.lift_plan(), self.base / 'run',
                                           stop_event=stop, clock=peer.clock, sleep=sleep)
        self.assertIn('operator stop during lift', report['error'])
        self.assertTrue(report['revoked'])
        self.assertIsNone(peer.target_source)
        self.assertIsNone(peer.capture_started_at)

    def test_requested_capture_on_disabled_firmware_rejects_before_authority(self):
        peer = FakePeer()
        peer.available = False
        request = experiment.threading.Event()
        request.set()
        report = self.run_case(peer, self.lift_plan(), capture_request=request)
        self.assertIn('no Pitch identification policy', report['error'])
        self.assertNotIn('authorize', [call[0] for call in peer.calls])

    def test_stale_lift_measurements_cannot_satisfy_settling(self):
        peer = FakePeer()
        peer.pitch_angle = 0.0
        peer.deliver = peer.step_target_stream
        report = self.run_case(peer, self.lift_plan(timeout_s=0.3))
        self.assertIn('lift did not settle', report['error'])
        self.assertIsNone(peer.capture_started_at)

    def test_excitation_uses_only_the_narrowed_envelope(self):
        peer = FakePeer()
        peer.follow_targets = True
        deliver = peer.deliver

        def leave_capture_envelope():
            deliver()
            if peer.ident_starts:
                peer.follow_targets = False
                peer.pitch_angle = -0.02  # 仍在抬升路径内, 已不在最终的 +/-0.005 内.

        peer.deliver = leave_capture_envelope
        report = self.run_case(peer, self.lift_plan())
        self.assertIn('outside software envelope', report['error'])
        self.assertTrue(report['revoked'])

    def test_completed_capture_holds_until_remote_takes_over(self):
        peer = FakePeer()
        plan = replace(make_plan(), remote_handoff_s=1.0)
        phases = []

        def phase(value):
            phases.append(value['state'])
            if value['state'] == 'WAIT_REMOTE':
                # 数据已结束并排空, 此刻仍在保持且没有重写 PID.
                self.assertFalse(peer.capture['active'])
                self.assertIsNotNone(peer.target_source)
                peer.control_source = 'remote'

        report = self.run_case(peer, plan, progress=phase)
        self.assertIsNone(report['error'], report)
        self.assertEqual(report['stop'], 'remote_handoff')
        self.assertTrue(report['quality']['complete'])
        self.assertIsNone(peer.target_source)
        self.assertIn('WAIT_REMOTE', phases)

    def test_remote_handoff_timeout_releases_holding(self):
        peer = FakePeer()
        report = self.run_case(peer, replace(make_plan(), remote_handoff_s=0.08))
        self.assertIn('remote handoff', report['error'])
        self.assertTrue(report['revoked'])
        self.assertIsNone(peer.target_source)

    def test_manifest_self_references_are_backfilled(self):
        """manifest 内嵌 files 必须指向真实存在的自身与 summary, 不能留空字符串."""
        peer = FakePeer()
        report = self.run_case(peer, make_plan(post_hold_s=0.05))
        self.assertIsNone(report['error'], report)
        manifest = json.loads((self.base / 'run/manifest.json').read_text(encoding='utf-8'))
        for key in ('sraw', 'csv', 'manifest', 'summary'):
            self.assertTrue(manifest['files'][key], f'files.{key} 不得为空字符串')
            self.assertTrue(Path(manifest['files'][key]).is_file(), f'files.{key} 指向的文件必须存在')
        self.assertEqual(Path(manifest['files']['manifest']), self.base / 'run/manifest.json')
        self.assertEqual(Path(manifest['files']['summary']), self.base / 'run/summary.json')

    def test_manifest_write_failure_never_leaves_complete_summary(self):
        """manifest 写失败时, 已落盘的 summary 不得继续声称 complete=true."""
        peer = FakePeer()
        target = self.base / 'run'

        def opener(path):
            (target / 'manifest.json').mkdir()  # 占位成目录, 使 manifest 写入必然失败
            return path.open('xb')

        report = self.run_case(peer, make_plan(post_hold_s=0.05), writer_opener=opener)
        self.assertTrue(report['cleanup_errors'], 'manifest 写入失败必须出现在清理错误里')
        summary = json.loads((target / 'summary.json').read_text(encoding='utf-8'))
        self.assertFalse(summary['quality']['complete'], 'summary 不能在报告缺失时声称完整')
        self.assertIsNotNone(summary['error'], 'summary 必须记录失败原因')
        self.assertIn('incomplete_reason', summary['quality'])

    def test_unsealed_writer_skips_parsing_and_reports_incomplete(self):
        """写线程未结束时不得导出/解析一个还在被追加的文件."""
        target = self.base / 'run'
        target.mkdir()
        writer = experiment.SrawWriter(target / 'pitch.sraw', {'layout_id': raw.LAYOUT, 'run': RUN,
                                                              'start_us': 0})
        writer.append(None, [make_record(RUN, 0, 1000)])
        writer.close()
        plan = make_plan()
        report = {'error': None, 'cleanup_errors': [], 'records': 1, 'stream_final': {'finished': True},
                  'identification': {'state_name': 'complete'}, 'source': 'APP_MIT_SIMULATION_ONLY'}
        files = experiment._finalise(target, plan, report, sealed=False)
        self.assertEqual(files['csv'], '', '未封盘时不得声称存在 CSV')
        self.assertFalse((target / 'pitch.csv').exists())
        summary = json.loads((target / 'summary.json').read_text(encoding='utf-8'))
        self.assertFalse(summary['quality']['complete'])
        self.assertIn('still running', summary['error'])

    def test_empty_capture_is_an_error_even_with_a_final_marker(self):
        """空文件不能因收到最终标记而作为成功试验返回."""
        for generated in (0, 1):
            with self.subTest(generated=generated):
                target = self.base / f'empty-{generated}'
                target.mkdir()
                writer = experiment.SrawWriter(
                    target / 'pitch.sraw', {'layout_id': raw.LAYOUT, 'run': RUN, 'start_us': 0})
                writer.close()
                report = {
                    'error': None, 'cleanup_errors': [], 'records': 0,
                    'identification': {'state_name': 'complete'},
                    'stream_final': {'finished': True, 'last_status': {
                        'generated': generated, 'dropped': 0, 'buffered': 0}},
                }
                experiment._finalise(target, make_plan(), report)
                self.assertIsNotNone(report['error'], report)
                self.assertIn('empty capture', report['error'])
                if generated:
                    self.assertIn('generated 1', report['error'])
                self.assertTrue(report['cleanup_errors'])
                for name in ('summary.json', 'manifest.json'):
                    saved = json.loads((target / name).read_text(encoding='utf-8'))
                    self.assertFalse(saved['quality']['complete'], name)

    def test_manifest_and_summary_rewrite_failures_leave_no_success_summary(self):
        """磁盘持续写失败后不能遗留先前发布的成功摘要, 原始记录仍保留."""
        original = experiment._write_json
        summary_writes = 0

        def fail_after_initial_summary(path, payload):
            nonlocal summary_writes
            if path.name == 'manifest.json':
                raise OSError('manifest disk full')
            if path.name == 'summary.json':
                summary_writes += 1
                if summary_writes > 1:
                    raise OSError('summary disk full')
            return original(path, payload)

        with patch.object(experiment, '_write_json', side_effect=fail_after_initial_summary):
            report = self.run_case(FakePeer(), make_plan(post_hold_s=0.05))
        self.assertIn('manifest disk full', report['error'])
        self.assertFalse(report['quality']['complete'])
        self.assertTrue(any('summary disk full' in error for error in report['cleanup_errors']))
        self.assertEqual(report['files']['summary'], '')
        self.assertEqual(report['files']['manifest'], '')
        target = self.base / 'run'
        self.assertFalse((target / 'summary.json').exists(), '不能保留旧的成功摘要')
        self.assertFalse((target / 'manifest.json').exists())
        _, rows = raw.load(target / 'pitch.sraw')
        self.assertGreater(len(rows), 0, '报告失败不能删除原始记录')

    def test_summary_write_failure_marks_manifest_incomplete(self):
        """summary 落不下时, 最后的 manifest 不得声称 complete=true."""
        peer = FakePeer()
        target = self.base / 'run'

        def opener(path):
            (target / 'summary.json').mkdir()  # 占位成目录, 使 summary 写入必然失败
            return path.open('xb')

        report = self.run_case(peer, make_plan(post_hold_s=0.05), writer_opener=opener)
        self.assertTrue(report['cleanup_errors'])
        manifest = json.loads((target / 'manifest.json').read_text(encoding='utf-8'))
        self.assertFalse(manifest['quality']['complete'])
        self.assertIn('incomplete_reason', manifest['quality'])
        self.assertEqual(manifest['files']['summary'], '', '写失败的路径不得留在 files 里')

    def test_close_timeout_skips_csv_export(self):
        """写线程在超时内没结束时, 不得导出/解析仍在被追加的文件."""
        peer = FakePeer()
        target = self.base / 'run'

        def opener(path):
            return _SlowRecordWriter(path.open('xb'), 0.4)

        with patch.object(experiment.SrawWriter, 'JOIN_TIMEOUT_S', 0.05):
            report = self.run_case(peer, make_plan(post_hold_s=0.05), writer_opener=opener)
        self.assertFalse((target / 'pitch.csv').exists(), '未封盘时不得生成 CSV')
        self.assertIn('still running', str(report.get('quality', {}).get('incomplete_reason', ''))
                      + ' ' + ' '.join(report['cleanup_errors']))
        self.assertFalse(report['quality']['complete'])

    def test_guard_fault_cancels_stops_and_revokes_with_partial_data(self):
        peer = FakePeer()

        def sleep(delay):
            peer.clock.advance(delay)
            peer.deliver()
            if peer.ident_state in ('baseline', 'square'):
                peer.manual_stop = True

        report = experiment.run_experiment(peer, make_plan(), self.base / 'run', clock=peer.clock, sleep=sleep)
        self.assertIn('Manual stop', report['error'])
        calls = [call[:2] for call in peer.calls]
        self.assertIn(('ident', 'cancel'), calls)
        self.assertIn(('stop_stream', RUN), calls)
        self.assertEqual(calls[-1], ('authorize', False))
        self.assertTrue((self.base / 'run/pitch.sraw').exists(), '失败时保留已写入的不完整数据')
        self.assertGreaterEqual(report['records'], 1)

    def test_disk_failure_is_visible_and_cleanup_still_revokes(self):
        peer = FakePeer()
        report = self.run_case(peer, writer_opener=failing_opener)
        self.assertIn('disk write failed', report['error'])
        self.assertIn(('stop_stream', RUN), [call[:2] for call in peer.calls])
        self.assertEqual([call[:2] for call in peer.calls][-1], ('authorize', False))
        sraw = self.base / 'run/pitch.sraw'
        self.assertTrue(sraw.exists())
        self.assertEqual(sraw.stat().st_size, len(raw.MAGIC) + 4 + experiment.METADATA_BYTES,
                         '失败时只保留头部, 不追加伪造记录')

    def test_excitation_fault_latches_without_auto_rerun(self):
        peer = FakePeer()
        original = peer.pitch_identification

        def identification(op, **kwargs):
            future = original(op, **kwargs)
            if op == 'query' and peer.queries >= 2:
                peer.ident_state = 'fault'
            return future

        peer.pitch_identification = identification
        report = self.run_case(peer)
        self.assertIn('firmware fault', report['error'])
        self.assertEqual(peer.ident_starts, 1, '不得自动重跑激励')


class SrawWriterTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / 'pitch.sraw'
        self.metadata = {'layout_id': raw.LAYOUT, 'run': RUN, 'start_us': 0}

    def test_queue_overflow_is_visible_and_never_swallowed(self):
        writer = experiment.SrawWriter(self.path, self.metadata, capacity=1)
        writer._done.set()
        writer._thread.join(timeout=2)
        writer._queue.put_nowait(make_record(RUN, 0, 10))
        with self.assertRaises(RuntimeError):
            writer.append(None, [make_record(RUN, 1, 20)])
        self.assertEqual(writer.count, 0)
        writer._queue.get_nowait()
        with self.assertRaises(RuntimeError) as caught:
            writer.close()
        self.assertIn('queue overflow', str(caught.exception))

    def test_short_write_is_rejected(self):
        """短写必须变成可见错误, 不能留下长度不对却自称完整的文件."""
        def opener(target):
            return _ShortWriter(target.open('xb'))

        with self.assertRaises(OSError) as caught:
            experiment.SrawWriter(self.path, self.metadata, opener=opener)
        self.assertIn('short write', str(caught.exception))

    def test_zero_byte_write_is_rejected(self):
        with self.assertRaises(OSError) as caught:
            experiment.SrawWriter(self.path, self.metadata, opener=lambda t: _ZeroWriter(t.open('xb')))
        self.assertIn('short write: 0/', str(caught.exception))

    def test_written_count_is_not_confused_with_queued(self):
        """落盘失败后报出的必须是真正写进文件的条数, 不能用入队条数冒充."""
        def opener(target):
            return _FailingAfterWrites(target.open('xb'), fail_after=2)  # 头部 + 第 1 条

        writer = experiment.SrawWriter(self.path, self.metadata, opener=opener)
        writer.append(None, [make_record(RUN, 0, 1000), make_record(RUN, 1, 2000)])
        with self.assertRaises(RuntimeError) as caught:
            writer.close()
        self.assertIn('written 1/2 queued', str(caught.exception))
        self.assertEqual(writer.count, 2, '入队数应保持 2')
        self.assertEqual(writer.written, 1, '真正落盘的只有第 1 条')

    def test_origin_is_rewritten_for_the_offline_loader(self):
        writer = experiment.SrawWriter(self.path, self.metadata)
        writer.append(None, [make_record(RUN, 0, 1234), make_record(RUN, 1, 2345)])
        writer.close()
        metadata, rows = raw.load(self.path)
        self.assertEqual(metadata['start_us'], 1234)
        self.assertEqual([row['t_s'] for row in rows], [0.0, 0.001111])

    def test_disk_failure_preserves_header_only(self):
        writer = experiment.SrawWriter(self.path, self.metadata, opener=failing_opener)
        writer.append(None, [make_record(RUN, 0, 10)])
        with self.assertRaises(RuntimeError) as caught:
            writer.close()
        self.assertIn('disk write failed', str(caught.exception))
        self.assertEqual(self.path.stat().st_size, len(raw.MAGIC) + 4 + experiment.METADATA_BYTES)


class ParserTests(unittest.TestCase):
    def test_serial_requires_explicit_bounds(self):
        p = experiment.parser()
        args = p.parse_args(['--serial', '/dev/ttyACM0', '--output', 'out', '--arm',
                             '--support-ready', '--accept-release'])
        with self.assertRaises(SystemExit):
            experiment.build_plan(args, p)

    def test_backend_pins_only_virtual_durations(self):
        p = experiment.parser()
        args = p.parse_args(['--backend', '/bin/true', '--output', 'out', '--arm', '--run', str(RUN),
                             '--physical-low', '-0.377', '--physical-high', '0.351', '--excursion', '0.005',
                             '--max-temperature', '55', '--max-torque', '2', '--max-speed', '0.6',
                             '--max-error', '0.03', '--amplitude-nm', '0.04', '--half-period-us', '10000',
                             '--baseline-us', '5000', '--cycles', '2', '--final-hold-us', '5000'])
        plan = experiment.build_plan(args, p)
        self.assertEqual(plan.recipe.run, RUN)
        self.assertGreater(plan.stream_duration_us, plan.recipe.excitation_us)
        self.assertEqual((plan.hold_timeout_s, plan.post_hold_s), (1.0, 0.5))
        self.assertGreater(plan.limits.max_run_s, plan.stream_duration_us / 1e6)

    def test_arm_flag_is_required_before_any_connection(self):
        with self.assertRaises(SystemExit):
            experiment.main(['--backend', '/bin/true', '--output', 'out'])


if __name__ == '__main__':
    unittest.main()
