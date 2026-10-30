"""复现板端共享慢回包槽: 状态页1/诊断共用每50ms一份的吞吐."""
from collections import deque
from concurrent.futures import Future
import unittest

from test_async_hold import ControlledClock, FakeAsyncSentinelClient
from sentinel_pitch_hold_model import (HoldEnvelope, PhysicalBounds, PitchHoldSupervisor,
                                      SupervisorConfig, SupervisorLimits, TimingBudget)


class SharedSlowPeer(FakeAsyncSentinelClient):
    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self._current_now = clock()
        self.page1 = super().snapshot()['status'][1]
        self.requests = deque()
        self.last_reply = clock()
        self.replies = []

    def snapshot(self):
        result = super().snapshot()
        result['status'][1] = dict(self.page1)
        return result

    def status(self, page=0):
        if page == 0:
            return super().status(page)
        future = Future()
        self.requests.append(('status', page, future))
        return future

    def diagnostics(self, page=0):
        future = Future()
        self.requests.append(('diagnostics', page, future))
        return future

    def pump(self):
        self._current_now = self.clock()
        self.status_seq += 1
        self.step_target_stream(self.clock())
        if self.requests and self.clock() - self.last_reply >= 0.05:
            kind, page, future = self.requests.popleft()
            if kind == 'status':
                self.page1 = super().snapshot()['status'][1]
                payload = self.page1
            else:
                payload = super().diagnostics(page).result()
            future.set_result(payload)
            self.last_reply = self.clock()
            self.replies.append((kind, page, self.clock()))


class SharedSlowScheduleTests(unittest.TestCase):
    def test_shared_50ms_lane_does_not_starve_status_or_diagnostics(self):
        clock = ControlledClock()
        peer = SharedSlowPeer(clock)
        limits = SupervisorLimits()
        guard = PitchHoldSupervisor(peer, SupervisorConfig(
            PhysicalBounds(-0.377, 0.351), HoldEnvelope(-0.2, 0.2),
            TimingBudget(100, 200, 0.02), limits, shared_slow_lane=True), clock_fn=clock)
        guard.start()
        for _ in range(600):
            clock.advance(0.004)
            peer.pump()
            guard.tick()
            self.assertFalse(guard.is_faulted, guard.fault_reason)
        self.assertTrue(guard.is_holding)
        for kind, page, maximum_gap in [('status', 1, 0.12), ('diagnostics', 0, 0.5),
                                        ('diagnostics', 1, 0.5)]:
            stamps = [at for name, number, at in peer.replies if (name, number) == (kind, page)]
            self.assertGreater(len(stamps), 3)
            self.assertLess(max(b-a for a, b in zip(stamps, stamps[1:])), maximum_gap)
        self.assertEqual(guard.config.limits.max_telemetry_age_s, 0.12)
        self.assertEqual(guard.config.timing.target_ttl_ms, 100)
        guard.stop()
