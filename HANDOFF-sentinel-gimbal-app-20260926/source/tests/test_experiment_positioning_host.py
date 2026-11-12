"""Headless host-only tests: mock transport, no serial import/open, no motion."""
from concurrent.futures import Future
from copy import deepcopy
from pathlib import Path
import struct
import sys
import time
import unittest
from unittest.mock import patch, MagicMock
import importlib.util

HOST = Path(__file__).resolve().parents[1] / 'robots/sentinel/host'
sys.path.insert(0, str(HOST))
import sentinel_client as wire
import sentinel_async as async_mod
from sentinel_experiment_positioning import ExperimentPositioningController, encode_recipe, encode_position_policy


def recipe(axis='fire_yaw'):
    return dict(axis=axis, pattern='bounded_step', experiment_session=17, nonce=31,
        param_epoch=3, guardian_version=2, config_hash=19, safety_reference_id=4,
        center_rad=0., amplitude_rad=.05, min_rad=-.2, max_rad=.2,
        max_velocity_rad_s=.3, max_acceleration_rad_s2=1., max_travel_rad=.5,
        max_output_abs=.1, max_output_integral=.2, move_us=20000, hold_us=10000,
        duration_us=160000, heartbeat_timeout_us=300000, stop_timeout_us=100000, repeats=1)


def policy():
    return dict(human_token=99, position_tolerance=.01, velocity_tolerance=.02,
        positioning_speed_limit=.1, move_us=200000, stable_dwell_us=100000,
        positioning_timeout_us=4000000, hold_timeout_us=15000000, feedback_max_age_us=200000)


def response(op=2, stage=3, result=0, sid=17, nonce=31, axis=5):
    b = bytearray(160)
    struct.pack_into('<8B6I', b, 0, 3, op, result, 0, 2, 1, 0, 1, sid, nonce, 3, 100000, 19, 2)
    flags = 0x3ffff & ~(1 << 3)
    struct.pack_into('<6I', b, 32, 51, flags, 3, 7, 8, 1)
    struct.pack_into('<4B', b, 56, axis, 0, 0, 1)
    struct.pack_into('<I', b, 60, axis)
    b[64:96] = bytes([1])*32
    b[96:128] = bytes([2])*32
    struct.pack_into('<4I3fI', b, 128, stage, 99, 7, 8, 0., .001, .002, 99990)
    return bytes(b)


def decoded(op=2, stage=3, result=0):
    r = wire.decode_experiment_snapshot(bytes((3, wire.EXPERIMENT_SNAPSHOT)) + struct.pack('<II', 12, 9) + response(op, stage, result))
    r['received_at_monotonic'] = 10.
    return r


class Peer:
    def __init__(self):
        self.state = dict(connected=True, session=12, config_epoch=7, reference_epoch=8)
        self.calls = []
    def snapshot(self):
        return deepcopy(self.state)
    def __getattr__(self, name):
        if not name.startswith('experiment_'):
            raise AttributeError(name)
        def invoke(*args, **kwargs):
            f = Future()
            self.calls.append((name, args, kwargs, f))
            return f
        return invoke


class PositioningTests(unittest.TestCase):
    def setUp(self):
        self.peer = Peer()
        self.now = 10.
        self.model = ExperimentPositioningController(self.peer, clock=lambda:self.now)
        self.model.configure(recipe(), policy(), hardware_blocked=False)
        self.model.next_poll = 1000

    def ack(self, op, stage=3, result=0):
        self.peer.calls[-1][3].set_result(decoded(op, stage, result))
        self.model.tick()

    def prepare(self):
        self.model.prepare()
        self.ack(3, 0)

    def waiting(self):
        self.prepare()
        self.model.arm()
        self.ack(4, 0)
        self.model.position()
        self.ack(8, 1)
        self.model.reply = decoded(2, 3)

    def confirmed(self):
        self.waiting()
        self.model.confirm()
        self.ack(9, 4)
        self.assertTrue(self.model.snapshot()['human_confirmed'])

    def test_every_click_is_separate_no_auto_start_or_confirm(self):
        self.waiting()
        self.assertEqual([c[0] for c in self.peer.calls], ['experiment_prepare', 'experiment_arm', 'experiment_position'])
        self.assertFalse(self.model.snapshot()['human_confirmed'])
        self.model.confirm()
        self.assertEqual(self.peer.calls[-1][0], 'experiment_confirm')
        self.assertFalse(self.model.snapshot()['human_confirmed'])
        self.ack(9, 4)
        self.assertTrue(self.model.snapshot()['human_confirmed'])
        self.assertNotIn('experiment_start', [c[0] for c in self.peer.calls])
        with self.assertRaisesRegex(ValueError, 'START unavailable'):
            self.model.start()
        self.assertNotIn('experiment_start', [c[0] for c in self.peer.calls])

    def test_start_only_after_explicit_start_and_capture_ready(self):
        capture_calls = []
        capture = Future()
        self.model.capture_start = lambda *args: capture_calls.append(args) or capture
        self.confirmed()
        self.assertFalse(capture_calls)
        self.model.start()
        self.assertEqual(len(capture_calls), 1)
        self.model.tick()
        self.assertNotEqual(self.peer.calls[-1][0], 'experiment_start')
        capture.set_result({'ready': True, 'dispatch_not_after': self.now + .01})
        self.model.tick()
        self.assertEqual(self.peer.calls[-1][0], 'experiment_start')
        self.ack(5, 5)

    def test_delayed_capture_completion_cannot_start(self):
        capture=Future();self.model.capture_start=lambda *args:capture
        self.confirmed();self.model.start()
        capture.set_result({'ready':True,'dispatch_not_after':self.now-.001})
        self.model.tick()
        self.assertNotIn('experiment_start',[c[0] for c in self.peer.calls])
        self.assertIn('deadline expired',self.model.note)

    def test_completed_fresh_status_precedes_old_reply_expiry(self):
        self.confirmed()
        self.now=10.09
        self.model._submit('status',self.peer.experiment_status(*self.model._ids()))
        fresh=decoded(2,4);fresh['received_at_monotonic']=10.10
        self.peer.calls[-1][3].set_result(fresh)
        self.now=10.101
        self.model.tick()
        self.assertIsNotNone(self.model.confirmed)

    def test_unconfirmed_start_cannot_open_capture(self):
        self.waiting()
        self.model.capture_start = lambda *args:self.fail('capture opened without human ACK')
        with self.assertRaisesRegex(ValueError, 'CONFIRM'):
            self.model.start()

    def test_missing_stale_changed_identity_guard_and_drift_block_confirm(self):
        changes = [dict(body_version=2), dict(received_at_monotonic=1.), dict(session=999),
            dict(experiment_session=18), dict(nonce=32), dict(axis='pitch'), dict(human_token=100),
            dict(positioning_config_epoch=6), dict(positioning_reference_epoch=7), dict(param_epoch=4),
            dict(guardian_version=3), dict(config_hash=20), dict(feedback_sample_us=100001),
            dict(boot_id=99), dict(firmware_sha256='ab'*32), dict(position_actual_rad=.1),
            dict(position_velocity_rad_s=.1), dict(position_target_rad=.02), dict(feedback_fresh=None),
            dict(simulation=True), dict(hardware_capable=False), dict(manual_stop=True),
            dict(guardian_latched=True), dict(positioning_stage=1), dict(position_actual_rad=float('nan'))]
        self.waiting()
        original = deepcopy(self.model.reply)
        for change in changes:
            with self.subTest(change=change):
                self.model.reply = {**original, **change}
                count = len(self.peer.calls)
                with self.assertRaises(ValueError):
                    self.model.confirm()
                self.assertEqual(len(self.peer.calls), count)
                self.assertIsNone(self.model.confirmed)

    def test_missing_each_feedback_identity_field_blocks(self):
        self.waiting()
        original = deepcopy(self.model.reply)
        for field in ('now_us', 'feedback_sample_us', 'boot_id', 'firmware_sha256', 'deployment_sha256',
                      'human_token', 'position_target_rad', 'position_actual_rad', 'position_velocity_rad_s'):
            with self.subTest(field=field):
                self.model.reply = deepcopy(original)
                self.model.reply.pop(field)
                with self.assertRaises((ValueError, TypeError)):
                    self.model.confirm()

    def test_snapshot_reconnect_epochs_and_drift_invalidate_confirmation(self):
        for change in (dict(connected=False), dict(session=13), dict(config_epoch=9), dict(reference_epoch=9)):
            with self.subTest(change=change):
                self.setUp()
                self.confirmed()
                self.peer.state.update(change)
                self.model.tick()
                self.assertIsNone(self.model.confirmed)
        self.setUp()
        self.confirmed()
        self.model.reply['position_actual_rad'] = .1
        self.model.tick()
        self.assertIsNone(self.model.confirmed)

    def test_recipe_axis_policy_change_invalidates_and_cannot_replace_active_hold(self):
        self.confirmed()
        for r, p in (({**recipe(), 'axis':'pitch'}, policy()),
                     ({**recipe(), 'amplitude_rad': .04}, policy()),
                     (recipe(), {**policy(), 'human_token':100})):
            with self.assertRaisesRegex(ValueError, 'Active recipe'):
                self.model.configure(r, p, hardware_blocked=False)
            self.assertIsNone(self.model.confirmed)
            self.assertIsNone(self.model.prepared_binding)

    def test_draft_edit_invalidates_inflight_ack_and_blocks_old_loaded_recipe(self):
        self.model.prepare()
        self.model.draft_changed('JSON changed during PREPARE')
        self.ack(3, 0)
        self.assertIsNone(self.model.prepared_binding)
        self.assertTrue(self.model.draft_dirty)
        with self.assertRaisesRegex(ValueError, 'Draft changed'):
            self.model.position()
        self.model.configure(recipe(), policy(), hardware_blocked=False)
        self.assertFalse(self.model.draft_dirty)
        self.assertIsNone(self.model.prepared_binding)

    def test_default_blocked_and_explicit_policy_validation(self):
        m = ExperimentPositioningController(self.peer)
        m.configure(recipe(), policy())
        with self.assertRaisesRegex(ValueError, 'Hardware blocked'):
            m.prepare()
        self.assertFalse(self.peer.calls)
        for p in ({}, {**policy(), 'position_tolerance':None}, {**policy(), 'velocity_tolerance':float('nan')},
                  {**policy(), 'positioning_speed_limit':0}, {**policy(), 'human_token':False}):
            with self.assertRaises(ValueError):
                encode_position_policy(p)

    def test_backend_failure_and_timeout_never_restore_authorization(self):
        self.waiting()
        self.model.confirm()
        self.ack(9, 4, result=1)
        self.assertIsNone(self.model.confirmed)
        self.model.reply = decoded(2, 3)
        self.model.confirm()
        self.peer.calls[-1][3].set_exception(TimeoutError('ACK timeout'))
        self.model.tick()
        self.assertIsNone(self.model.confirmed)
        self.assertIn('ACK timeout', self.model.note)

    def test_single_bounded_poll_close_and_deferred_stop(self):
        self.prepare()
        self.model.next_poll = 0
        self.model.tick()
        self.assertEqual(self.peer.calls[-1][0], 'experiment_status')
        for _ in range(50):
            self.model.tick()
        self.assertEqual(len(self.peer.calls), 2)
        self.model.stop()
        self.assertEqual(len(self.peer.calls), 2)
        self.ack(2, 3)
        self.assertEqual(self.peer.calls[-1][0], 'experiment_stop')
        self.model.close()
        n = len(self.peer.calls)
        self.model.tick()
        self.assertEqual(len(self.peer.calls), n)
        self.assertIsNone(self.model.confirmed)

    def test_human_hold_confirmation_does_not_require_short_capture_open(self):
        self.waiting()
        self.model.reply['backpressure_ok'] = False
        self.model.confirm()
        r = decoded(9, 4)
        r['backpressure_ok'] = False
        self.peer.calls[-1][3].set_result(r)
        self.model.tick()
        self.assertIsNotNone(self.model.confirmed)
        self.assertNotIn('experiment_start', [c[0] for c in self.peer.calls])

    def test_cancel_is_its_own_command_and_not_stop_or_disable(self):
        self.confirmed()
        self.model.cancel()
        self.assertEqual(self.peer.calls[-1][0], 'experiment_cancel')
        self.ack(10, 7)
        self.assertTrue(self.model.active)
        self.assertIsNone(self.model.confirmed)
        self.assertNotIn('experiment_stop', [c[0] for c in self.peer.calls])

    def test_pitch_backend_block_is_truthful(self):
        self.model.configure(recipe('pitch'), policy(), hardware_blocked=False)
        self.prepare()
        self.model.reply = decoded(2, 3)
        self.model.reply['axis'] = 'pitch'
        with self.assertRaisesRegex(ValueError, 'Pitch hardware blocked'):
            self.model.confirm()


class CodecTests(unittest.TestCase):
    def test_v3_layouts_exact_outer_envelope_unchanged(self):
        command = encode_recipe(recipe())
        self.assertEqual(len(command), 88)
        self.assertEqual(struct.unpack_from('<f', command, 28)[0], 0.)
        body = struct.pack('<4BII', 3, 8, 5, 1, 17, 31) + encode_position_policy(policy())
        self.assertEqual(len(body), 48)
        self.assertEqual(struct.unpack_from('<I', body, 12)[0], 99)
        self.assertEqual(struct.unpack_from('<I', body, 44)[0], 200000)
        self.assertEqual(wire.encode_experiment(12, 9, body), bytes((3, 0x60))+struct.pack('<II', 12, 9)+body)
        for op, length in ((3, 88), (8, 48), (9, 16), (5, 12), (10, 12)):
            with self.assertRaises(ValueError):
                wire.encode_experiment(1, 1, bytes((3, op, 5, 1))+bytes(length-5))
        r = decoded()
        self.assertEqual(r['positioning_stage_name'], 'wait-human')
        self.assertEqual(r['human_token'], 99)
        self.assertAlmostEqual(r['position_actual_rad'], .001)
        self.assertTrue(r['hardware_blocked'], 'legacy authorization unknowns must not become fabricated success')

    def test_invalidated_and_unknown_feedback_are_reported_not_fake_success(self):
        b = bytearray(response(stage=8))
        struct.pack_into('<f', b, 148, float('nan'))
        r = wire.decode_experiment_snapshot(bytes((3, 0xca))+struct.pack('<II', 12, 9)+b)
        self.assertEqual(r['positioning_stage_name'], 'invalidated')
        self.assertIsNone(r['position_actual_rad'])
        self.assertFalse(r['position_feedback_valid'])

    def test_v1_v2_decoding_and_bad_v3_preserved(self):
        for ver, n in ((1, 32), (2, 128)):
            b = bytearray(response()[:n])
            b[0] = ver
            r = wire.decode_experiment_snapshot(bytes((3, 0xca))+struct.pack('<II', 12, 9)+b)
            self.assertEqual(r['body_version'], ver)
            self.assertNotIn('human_token', r)
        for offset, val in ((0, 4), (128, 9)):
            b = bytearray(response())
            b[offset] = val
            with self.assertRaises(wire.ProtocolError):
                wire.decode_experiment_snapshot(bytes((3, 0xca))+struct.pack('<II', 12, 9)+b)


class RecordingTransport:
    def __init__(self):
        self.sent = []
    def send(self, message):
        self.sent.append(message)
    def receive(self, timeout_s=0):
        return None
    def close(self):
        pass


class AsyncTests(unittest.TestCase):
    def setUp(self):
        self.transport = RecordingTransport()
        with patch('threading.Thread.start'):
            self.peer = async_mod.AsyncSentinelClient(self.transport, fast_hz=0, pitch_decoder=lambda *_:None)
        self.peer._session, self.peer._connected = 12, True
        self.peer._state.update(session=12, connected=True, config_epoch=7, reference_epoch=8)

    def begin(self, future):
        request, self.peer._change = self.peer._change, None
        if request is None:
            request = self.peer._reads.popleft()
            self.peer._read_active = True
        self.peer._begin(request, time.monotonic())
        return list(self.peer._pending.values())[-1]

    def ack(self, pending, op, stage=3, sid=17, nonce=31, axis=5):
        body = bytes((3, 0xca))+struct.pack('<II', 12, pending.sequence)+response(op, stage, sid=sid, nonce=nonce, axis=axis)
        self.peer._receive((0xca, pending.frame_sequence, body))

    def test_integrated_separate_methods_correlated_acks_no_targets(self):
        calls = [('prepare', lambda:self.peer.experiment_prepare(encode_recipe(recipe()), hardware_blocked=False), 3),
            ('arm', lambda:self.peer.experiment_arm(5,1,17,31, hardware_blocked=False), 4),
            ('position', lambda:self.peer.experiment_position(5,1,17,31,policy(), hardware_blocked=False), 8),
            ('confirm', lambda:self.peer.experiment_confirm(5,1,17,31,99, hardware_blocked=False), 9),
            ('start', lambda:self.peer.experiment_start(5,1,17,31, hardware_blocked=False), 5),
            ('cancel', lambda:self.peer.experiment_cancel(5,1,17,31), 10),
            ('stop', lambda:self.peer.experiment_stop(5,1,17,31), 6),
            ('status', lambda:self.peer.experiment_status(5,1,17,31), 2)]
        for name, call, op in calls:
            with self.subTest(name=name):
                f = call()
                pending = self.begin(f)
                self.assertFalse(f.done())
                self.ack(pending, op)
                self.assertEqual(f.result()['op'], op)
        with self.assertRaisesRegex(wire.ProtocolError, 'experiment owns reference'):
            self.peer.publish_targets([])
        with self.assertRaisesRegex(wire.ProtocolError, 'experiment owns reference'):
            self.peer.set_target_source(None)
        self.assertIsNone(self.peer._source)
        self.assertIsNone(self.peer._goal)
        for message in self.transport.sent:
            d = wire.FrameDecoder()
            frames = [r for v in message if (r:=d.push(v)) is not None]
            self.assertEqual(frames[0][0], wire.EXPERIMENT)

    def test_default_deny_and_mismatched_ack_waits_then_unknown(self):
        with self.assertRaisesRegex(wire.ProtocolError, 'hardware_blocked'):
            self.peer.experiment_start(5,1,17,31)
        self.assertFalse(self.transport.sent)
        f = self.peer.experiment_confirm(5,1,17,31,99, hardware_blocked=False)
        p = self.begin(f)
        for op, sid, nonce, axis in ((8,17,31,5), (9,18,31,5), (9,17,32,5), (9,17,31,6)):
            self.ack(p, op, sid=sid, nonce=nonce, axis=axis)
            self.assertFalse(f.done())
        self.peer.attempts = 1
        self.peer._expire(p.deadline+1)
        with self.assertRaises(async_mod.OutcomeUnknown):
            f.result()
        self.assertEqual(self.peer.snapshot()['statistics']['invalid_replies'], 4)

    def test_new_hello_releases_reference_owner_only_by_explicit_reset(self):
        self.peer.experiment_prepare(encode_recipe(recipe()), hardware_blocked=False)
        self.assertTrue(self.peer._experiment_owned)
        self.peer._reset_session()
        self.assertFalse(self.peer._experiment_owned)


class GuiCallbackTests(unittest.TestCase):
    """Exercise real DearPyGUI callback methods without creating a GUI or device."""
    def setUp(self):
        fake_dpg = MagicMock()
        package = MagicMock()
        package.dearpygui = fake_dpg
        with patch.dict(sys.modules, {'dearpygui': package, 'dearpygui.dearpygui': fake_dpg}):
            spec = importlib.util.spec_from_file_location('positioning_console_headless', HOST/'sentinel_console.py')
            self.gui = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.gui)
        self.peer = Peer()
        self.model = ExperimentPositioningController(self.peer, clock=lambda:10.)
        self.model.configure(recipe(), policy(), hardware_blocked=False)
        self.model.next_poll = 1000
        self.session = MagicMock()
        self.session.positioning = self.model
        self.session.assistance.active = False
        self.console = object.__new__(self.gui.Console)
        self.console.session = self.session
        self.console.require = lambda:self.session

    def test_real_gui_separate_buttons_do_not_reuse_assist_capture(self):
        for action, op, stage in (('prepare', 3, 0), ('arm', 4, 0), ('position', 8, 3)):
            self.console.positioning_action(action)
            self.assertEqual(self.peer.calls[-1][0], 'experiment_'+action)
            self.peer.calls[-1][3].set_result(decoded(op, stage))
            self.model.tick()
        self.assertIsNone(self.model.confirmed)
        self.console.positioning_action('confirm')
        self.peer.calls[-1][3].set_result(decoded(9, 4))
        self.model.tick()
        self.assertIsNotNone(self.model.confirmed)
        with self.assertRaisesRegex(ValueError, 'START unavailable'):
            self.console.positioning_action('start')
        self.assertNotIn('experiment_start', [c[0] for c in self.peer.calls])
        self.console.positioning_draft_changed()
        self.assertIsNone(self.model.confirmed)
        self.assertIsNone(self.model.prepared_binding)

    def test_old_assistance_cannot_share_positioning_reference(self):
        self.session.assistance.active = True
        with self.assertRaisesRegex(ValueError, 'Pitch assist'):
            self.console.positioning_action('prepare')
        self.assertFalse(self.peer.calls)

    def test_maintenance_session_owns_and_closes_positioning(self):
        from sentinel_console_model import MaintenanceSession
        session = MaintenanceSession(self.peer, 'gimbal')
        session.positioning.configure(recipe(), policy(), hardware_blocked=False)
        session.positioning.active = True
        with self.assertRaisesRegex(ValueError, 'Experiment owns reference'):
            session.require_manual()
        with patch.object(session.assistance, 'close'), patch.object(self.peer, 'close', create=True) as close:
            session.close()
        self.assertTrue(session.positioning.closed)
        close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
