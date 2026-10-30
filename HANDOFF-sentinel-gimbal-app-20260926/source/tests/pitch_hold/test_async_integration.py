"""Software only: real async owner and native APP protocol backend, never serial."""
import os
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robots/sentinel/host'))

import sentinel_client as wire
import sentinel_pitch_hold_model as model
from sentinel_async import AsyncSentinelClient
from sentinel_pitch_hold import shutdown
from sentinel_pitch_hold_model import (
    HoldEnvelope,
    PhysicalBounds,
    PitchHoldSupervisor,
    SupervisorConfig,
    SupervisorState,
    TimingBudget,
)

SOURCE = Path(__file__).resolve().parents[2]


BACKEND = Path(os.environ.get('SENTINEL_PITCH_BACKEND', str(SOURCE / 'build/host-tests/ec_sentinel_app_host')))


class FaultTransport:
    """Inject framed reply loss/rejection above the unmodified native backend."""
    def __init__(self, inner):
        self.inner = inner
        self.mode = ''
        self.block_entered = threading.Event()
        self.block_release = threading.Event()
        self.block_finished = threading.Event()
        self.sent = []

    def send(self, message):
        decoder = wire.FrameDecoder()
        frame = None
        for byte in message:
            decoded = decoder.push(byte)
            if decoded is not None:
                frame = decoded
        position_target = bool(frame and frame[0] == wire.TARGETS and len(frame[2]) > 18 and frame[2][18] > 0)
        blocked = self.mode == 'block-send' and position_target
        if blocked:
            self.mode = ''
            self.block_entered.set()
            if not self.block_release.wait(timeout=1):
                raise TimeoutError('test send block not released')
        self.inner.send(message)
        if position_target and frame is not None:
            self.sent.append((blocked, frame[1]))
        if blocked:
            self.block_finished.set()

    def receive(self, timeout_s=0):
        reply = self.inner.receive(timeout_s)
        if reply and reply[0] == wire.TARGETS_ACK:
            if self.mode == 'drop-target':
                return None
            if self.mode == 'reject-target':
                body = bytearray(reply[2])
                body[10] = wire.RESULT_BY_NAME['REJECTED']
                return reply[0], reply[1], bytes(body)
        if reply and reply[0] == wire.ACK and self.mode == 'drop-change':
            return None
        return reply

    def close(self):
        self.inner.close()


class NativeIntegration(unittest.TestCase):
    def setUp(self):
        self.assertTrue(BACKEND.is_file(), 'build native APP backend first')
        self.transport = FaultTransport(wire.ProcessTransport(BACKEND, extra_args=['--plant', '--fixture', 'gimbal']))
        self.peer = AsyncSentinelClient(self.transport, target_hz=200, fast_hz=50)
        self.guard: PitchHoldSupervisor | None = None
        self.assertEqual(Path(model.__file__ or '').resolve(), SOURCE / 'robots/sentinel/host/sentinel_pitch_hold_model.py')
        self.peer.hello().result(timeout=2)
        result = self.peer.authorize(True, 'maintenance').result(timeout=2)
        self.assertEqual(result['result'], 0)
        # Native MCU time advances with requests, not elapsed wall time.
        for _ in range(10):
            self.peer.status(0).result(timeout=2)
            self.peer.status(1).result(timeout=2)

    def tearDown(self):
        try:
            if self.guard is not None:
                self.guard.stop('test end')
        finally:
            self.peer.close()

    def begin_holding(self) -> PitchHoldSupervisor:
        timing = self.peer.diagnostics(0).result(timeout=1)
        guard = PitchHoldSupervisor(self.peer, SupervisorConfig(
            PhysicalBounds(-0.3, 0.4), HoldEnvelope(-0.1, 0.2),
            TimingBudget(timing['target_ttl_ms'], timing['link_ttl_ms'], self.peer.target_period)))
        self.guard = guard
        guard.start()
        deadline = time.monotonic() + 3
        while guard.state != SupervisorState.HOLDING and time.monotonic() < deadline:
            guard.tick()
            self.assertFalse(guard.is_faulted, guard.fault_reason)
            time.sleep(0.005)
        self.assertTrue(guard.is_holding, guard.fault_reason)
        return guard

    def test_segment_continuity_and_stop(self):
        guard = self.begin_holding()
        for segment in ('first', 'second'):
            actual = guard.actual_pitch_rad
            self.assertIsNotNone(actual)
            assert actual is not None
            guard.set_goal(actual + 0.002, segment)
            for _ in range(25):
                guard.tick()
                self.assertTrue(guard.is_holding, guard.fault_reason)
                time.sleep(0.005)
        guard.stop()
        self.assertEqual(guard.state, SupervisorState.STOPPED)
        deadline = time.monotonic() + 2
        pitch = None
        while time.monotonic() < deadline:
            page = self.peer.status(1).result(timeout=1)
            pitch = next(d for d in page['domains'] if d['domain'] == 'pitch')
            if not pitch['execute']:
                break
        self.assertIsNotNone(pitch)
        assert pitch is not None
        self.assertFalse(pitch['execute'])

    def wait_fault(self, guard: PitchHoldSupervisor):
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not guard.is_faulted:
            guard.tick()
            time.sleep(0.005)
        self.assertTrue(guard.is_faulted, guard.fault_reason)

    def test_explicit_rehello_clears_old_target_fence(self):
        first = self.begin_holding()
        previous = self.peer.snapshot()
        self.assertIsNotNone(previous['last_target_ack'])
        self.assertGreater(previous['last_target_sent_sequence'], 0)
        first.stop('explicit new session')
        hello = self.peer.hello().result(timeout=2)
        snapshot = self.peer.snapshot()
        self.assertNotEqual(hello['session'], previous['session'])
        self.assertIsNone(snapshot['last_target_ack'])
        self.assertEqual(snapshot['last_target_sent_sequence'], 0)
        self.assertEqual(snapshot['target_rejections'], previous['target_rejections'])
        self.assertEqual(self.peer.authorize(True, 'maintenance').result(timeout=2)['result'], 0)
        # Match the runner: fresh status after authorization, not an automatic
        # fast-page reply captured while the new session was still unauthorized.
        self.peer.status(0).result(timeout=2)
        self.peer.status(1).result(timeout=2)
        second = self.begin_holding()
        self.assertTrue(second.is_holding, second.fault_reason)

    def test_blocked_send_then_recovery_does_not_resume(self):
        guard = self.begin_holding()
        self.transport.mode = 'block-send'
        # 单串口拥有者线程只在被驱动时发布目标; 等待期间必须自己 tick,
        # 否则 1s 内是否发出目标取决于后台线程调度 (曾出现偶发失败).
        deadline = time.monotonic() + 3
        while not self.transport.block_entered.is_set() and time.monotonic() < deadline:
            guard.tick()
            time.sleep(0.005)
        self.assertTrue(self.transport.block_entered.is_set(), '目标帧从未尝试发送')
        try:
            self.wait_fault(guard)
            self.assertTrue(guard.is_faulted)
        finally:
            self.transport.block_release.set()
        self.assertTrue(self.transport.block_finished.wait(timeout=1))
        # One previously encoded blocked frame may still be transmitted; only
        # compare new frames after that in-flight send actually completes.
        self.assertEqual(sum(1 for blocked, _ in self.transport.sent if blocked), 1)
        after_inflight = len(self.transport.sent)
        time.sleep(0.03)
        guard.tick()
        self.assertEqual(len(self.transport.sent), after_inflight)
        self.assertTrue(guard.is_faulted)
        self.assertIsNone(guard._goal_source(time.monotonic(),
            {'config_epoch': self.peer.snapshot()['config_epoch'],
             'reference_epoch': self.peer.snapshot()['reference_epoch']}))

    def test_target_ack_loss_latches(self):
        guard = self.begin_holding()
        self.transport.mode = 'drop-target'
        self.wait_fault(guard)
        self.assertIn('ACK stale', guard.fault_reason or '')

    def test_target_ack_rejection_latches(self):
        guard = self.begin_holding()
        self.transport.mode = 'reject-target'
        self.wait_fault(guard)
        self.assertIn('reject', (guard.fault_reason or '').lower())

    def test_disconnected_backend_reports_unknown_stop(self):
        guard = self.begin_holding()
        self.transport.inner._process.kill()
        self.transport.inner._process.wait(timeout=1)
        self.wait_fault(guard)
        report = shutdown(self.peer, guard)
        self.assertEqual(report['revoke'], 'unknown')
        self.assertFalse(report['disabled_feedback'])
        self.assertTrue(report['errors'])
        self.guard = None  # shutdown has already attempted and reported failed cleanup

    def test_lost_stop_ack_does_not_claim_disable(self):
        guard = self.begin_holding()
        self.transport.mode = 'drop-change'
        report = shutdown(self.peer, guard)
        self.assertEqual(report['revoke'], 'unknown')
        self.assertFalse(report['disabled_feedback'])
        self.assertTrue(report['errors'])
        self.guard = None

    def test_real_background_source_watchdog_latches(self):
        guard = self.begin_holding()
        time.sleep(0.20)  # intentional fault injection, not a timing-based success assertion
        guard.tick()
        self.assertTrue(guard.is_faulted)
        self.assertIn('watchdog', guard.fault_reason or '')


if __name__ == '__main__':
    unittest.main()
