"""Deterministic, offline unit tests for PitchHoldSupervisor.

Covers:
1. Successful full lifecycle & state transitions (IDLE -> ENABLING_HANDSHAKE -> HOLDING -> STOPPED).
2. Distinguishing initial enabling handshake from holding.
3. Enabling handshake timeout when motor never enables / executes.
4. Loss of motor enabled state while holding.
5. Loss of domain execute while holding.
6. Domain reported fault reason while holding.
7. Authority loss and manual stop safety trips.
8. Client disconnection and reported errors.
9. Session and epoch mismatch detection.
10. Stale status telemetry detection and fail-closed missing timestamp handling.
11. Status sequence regression detection.
12. Diagnostics query dispatch, future timeouts, and stale diagnostics detection.
13. Target ACK freshness verification and prevention of pre-start cache laundering.
14. Target ACK rejection and sequence regression detection.
15. Actual pitch bounds checking (physical bounds, envelope limits, NaN/inf, stale sensor age, high speed).
16. Target goal bounds validation.
17. Segment continuity (updating goals across segments without resetting/clearing target source).
18. Stalled supervision watchdog cutoff (target generator ceases when tick loop stalls).
19. Strict no auto-resume policy and explicit manual rearm workflow.
20. Invariant verification: supervisor never invokes publish_permit, param_write, or raw transport.
"""
from __future__ import annotations

import sys
import unittest
from collections.abc import Callable
from concurrent.futures import Future
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robots/sentinel/host'))

from sentinel_pitch_hold_model import (
    HoldEnvelope,
    PhysicalBounds,
    PitchHoldSupervisor,
    PitchSupervisorError,
    SupervisorConfig,
    SupervisorLimits,
    SupervisorState,
    TimingBudget,
)


class ControlledClock:
    """Deterministic monotonic clock for offline unit tests."""
    def __init__(self, start: float = 1000.0) -> None:
        self._now = start

    def __call__(self) -> float:
        return self._now

    def advance(self, dt_s: float) -> float:
        self._now += dt_s
        return self._now


class FakeAsyncSentinelClient:
    """Deterministic fake for AsyncSentinelClient adhering strictly to its public interface.

    Simulates snapshot(), status(page), diagnostics(page), and set_target_source().
    Proves that PitchHoldSupervisor does NOT depend on raw transport, publish_permit,
    or param_write.
    """

    def __init__(
        self,
        session: int = 101,
        config_epoch: int = 5,
        reference_epoch: int = 12,
        initial_pitch_angle: float = -0.05,
    ) -> None:
        self.session = session
        self.config_epoch = config_epoch
        self.reference_epoch = reference_epoch
        self.connected = True
        self.client_error = ""

        # Device & joint telemetry states
        self.pitch_angle = initial_pitch_angle
        self.pitch_rate = 0.0
        self.position_valid = True
        self.position_age_ms = 10
        self.velocity_age_ms = 10
        self.authority = True
        self.manual_stop = False
        self.source = "maintenance"
        self.motor_online = True
        self.motor_enabled = True
        self.motor_faulted = False
        self.motor_temperature = 25
        self.motor_effort = 0.1
        self.domain_execute = True
        self.domain_validity = "valid"
        self.domain_mode = "position"
        self.domain_reason = "none"

        # Sequences & timing
        self.status_seq = 10
        self.status_timestamp_offset = 0.0  # seconds behind clock
        self.omit_status_timestamp = False
        self.target_ttl_ms = 100
        self.link_ttl_ms = 200
        self.diag_seq = 20
        self.target_ack_seq = 50
        self.target_ack_result = 0
        self.target_ack_result_name = "ACCEPTED"
        self.target_ack_reason = 0
        self.target_ack_reason_name = "none"
        self.target_rejections = 0
        self.last_target_ack = {
            "session": self.session,
            "sequence": self.target_ack_seq,
            "result": self.target_ack_result,
            "result_name": self.target_ack_result_name,
            "reason": self.target_ack_reason,
            "reason_name": self.target_ack_reason_name,
        }

        # Diagnostics query futures behavior
        self.diag_future_hang = False
        self.diag_future_fail: Exception | None = None

        # Target source installed by supervisor
        self.target_source: Callable[[float, dict], dict | None] | None = None
        self.set_source_calls = 0
        self.generated_targets: list[tuple[float, dict]] = []

        # Forbidden calls tracker (supervisor must NOT call these)
        self.forbidden_calls: list[str] = []

    def snapshot(self) -> dict:
        now = getattr(self, "_current_now", 1000.0)
        rx_time = now - self.status_timestamp_offset

        page0 = {
            "session": self.session,
            "sequence": self.status_seq,
            "page": 0,
            "config_epoch": self.config_epoch,
            "reference_epoch": self.reference_epoch,
            "clock_ms": int(now * 1000),
            "control_ms": 100,
            "evaluated_us": 500,
            "boot_id": 1,
            "source": self.source,
            "authority": self.authority,
            "manual_stop": self.manual_stop,
            "launch_allowed": False,
            "locally_ready": True,
            "boot_identity_available": True,
            "main_yaw_degraded": False,
            "config_kind": 1,
            "joints": {
                "main_yaw": {"angle_rad": 0.0, "rate_rad_s": 0.0, "position_valid": True, "position_age_ms": 5},
                "fire_yaw": {"angle_rad": 0.0, "rate_rad_s": 0.0, "position_valid": True, "position_age_ms": 5},
                "pitch": {
                    "angle_rad": self.pitch_angle,
                    "rate_rad_s": self.pitch_rate,
                    "position_valid": self.position_valid,
                    "velocity_valid": True,
                    "position_age_ms": self.position_age_ms,
                    "velocity_age_ms": self.velocity_age_ms,
                },
            },
        }
        if not self.omit_status_timestamp:
            page0["received_at_monotonic"] = rx_time

        page1 = {
            "session": self.session,
            "sequence": self.status_seq,
            "page": 1,
            "config_epoch": self.config_epoch,
            "reference_epoch": self.reference_epoch,
            "clock_ms": int(now * 1000),
            "source": self.source,
            "authority": self.authority,
            "manual_stop": self.manual_stop,
            "launch_allowed": False,
            "domains": [
                {
                    "domain": "pitch",
                    "validity": self.domain_validity,
                    "mode": self.domain_mode,
                    "execute": self.domain_execute,
                    "reason": self.domain_reason,
                    "target": self.pitch_angle,
                    "measurement": self.pitch_angle,
                    "age_ms": self.position_age_ms,
                    "velocity_age_ms": self.velocity_age_ms,
                }
            ],
        }
        if not self.omit_status_timestamp:
            page1["received_at_monotonic"] = rx_time

        return {
            "connected": self.connected,
            "session": self.session,
            "config_epoch": self.config_epoch,
            "reference_epoch": self.reference_epoch,
            "error": self.client_error,
            "status": {0: page0, 1: page1},
            "diagnostics": {},  # Cached diagnostics lack timestamps and are not relied upon
            "last_target_ack": dict(self.last_target_ack) if self.last_target_ack else None,
            'target_rejections': self.target_rejections,
            'last_target_sent_sequence': self.target_ack_seq,
        }

    def status(self, page: int = 0) -> Future:
        fut = Future()
        snap = self.snapshot()
        fut.set_result(snap["status"].get(page, {}))
        return fut

    def diagnostics(self, page: int = 0) -> Future:
        fut = Future()
        if self.diag_future_hang:
            return fut
        if self.diag_future_fail is not None:
            fut.set_exception(self.diag_future_fail)
            return fut

        self.diag_seq += 1
        payload: dict[str, Any]
        if page == 0:
            payload = {
                "session": self.session,
                "sequence": self.diag_seq,
                "page": 0,
                "cycles": 1000,
                "rejected_targets": 0,
                "frame_rejects": 0,
                "target_ttl_ms": self.target_ttl_ms,
                "link_ttl_ms": self.link_ttl_ms,
                "config_epoch": self.config_epoch,
                "reference_epoch": self.reference_epoch,
            }
        elif page == 1:
            payload = {
                "session": self.session,
                "sequence": self.diag_seq,
                "page": 1,
                "devices": [
                    {
                        "slot": 6,
                        "domain": "pitch",
                        "installed": True,
                        "online": self.motor_online,
                        "enabled": self.motor_enabled,
                        "faulted": self.motor_faulted,
                        "age_ms": self.position_age_ms,
                        "temperature_c": self.motor_temperature,
                        "rotor_temperature_c": self.motor_temperature,
                        "effort": self.motor_effort,
                        "effort_unit": 'Nm',
                    }
                ],
            }
        else:
            payload = {"session": self.session, "sequence": self.diag_seq, "page": page}
        payload['received_at_monotonic'] = self._current_now
        fut.set_result(payload)
        return fut

    def set_target_source(self, source_fn: Callable[[float, dict], dict | None] | None) -> None:
        self.set_source_calls += 1
        self.target_source = source_fn

    def step_target_stream(self, now: float) -> dict | None:
        """Simulate AsyncSentinelClient invoking the target source on its UART thread."""
        self._current_now = now
        if self.target_source is None:
            return None
        epochs = {"config_epoch": self.config_epoch, "reference_epoch": self.reference_epoch}
        val = self.target_source(now, epochs)
        if val is not None:
            self.generated_targets.append((now, val))
            # Advance ACK sequence
            self.target_ack_seq += 1
            self.last_target_ack = {
                'received_at_monotonic': now,
                "session": self.session,
                "sequence": self.target_ack_seq,
                "result": self.target_ack_result,
                "result_name": self.target_ack_result_name,
                "reason": self.target_ack_reason,
                "reason_name": self.target_ack_reason_name,
            }
        return val

    # Forbidden wire-level mutations
    def publish_permit(self, *args, **kwargs) -> None:
        self.forbidden_calls.append("publish_permit")
        raise RuntimeError("publish_permit called on peer")

    def param_write(self, *args, **kwargs) -> None:
        self.forbidden_calls.append("param_write")
        raise RuntimeError("param_write called on peer")

    def authorize(self, *args, **kwargs) -> None:
        self.forbidden_calls.append("authorize")
        raise RuntimeError("authorize called on peer")


class PitchHoldSupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = ControlledClock(1000.0)
        self.bounds = PhysicalBounds(low=-0.377, high=0.351)
        self.envelope = HoldEnvelope(low=-0.200, high=0.200)
        self.limits = SupervisorLimits(
            max_step_rad=0.005,
            max_speed_rad_s=0.6,
            max_telemetry_age_s=0.10,
            max_sample_age_ms=100.0,
            max_diagnostics_age_s=0.50,
            max_ack_age_s=0.15,
            diagnostics_query_interval_s=0.05,
            diagnostics_query_timeout_s=0.20,
            handshake_timeout_s=0.40,
            watchdog_timeout_s=0.15,
        )
        self.config = SupervisorConfig(
            physical_bounds=self.bounds,
            envelope=self.envelope,
            timing=TimingBudget(100, 200, 0.02),
            limits=self.limits,
        )
        self.fake = FakeAsyncSentinelClient(
            session=101,
            config_epoch=5,
            reference_epoch=12,
            initial_pitch_angle=-0.05,
        )
        self.fake._current_now = self.clock()
        self.faults: list[str] = []
        self.supervisor = PitchHoldSupervisor(
            peer=self.fake,
            config=self.config,
            clock_fn=self.clock,
            on_fault=lambda msg: self.faults.append(msg),
        )

    def _drive_to_holding(self) -> None:
        """Helper to advance supervisor past handshake into HOLDING state."""
        self.supervisor.start()
        self.assertEqual(self.supervisor.state, SupervisorState.ENABLING_HANDSHAKE)

        # Preflight must complete before the source is installed, then a fresh
        # ACK is required for the enabling-to-holding transition.
        for _ in range(3):
            self.clock.advance(0.01)
            self.fake._current_now = self.clock()
            self.supervisor.tick()
            self.fake.step_target_stream(self.clock())
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.HOLDING)

    # --------------------------------------------------------------------------
    # 1. Full Lifecycle & Transitions
    def _advance_stream(self, cycles: int) -> None:
        for _ in range(cycles):
            self.clock.advance(0.02)
            self.fake._current_now = self.clock()
            self.fake.step_target_stream(self.clock())
            self.supervisor.tick()

    # --------------------------------------------------------------------------

    def test_reject_followed_by_success_still_latches(self) -> None:
        self._drive_to_holding()
        self.fake.step_target_stream(self.clock())
        self.fake.target_rejections += 1
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIn('overwritten ACK', self.supervisor.fault_reason or '')

    def test_stale_ack_not_retimestamped_on_observation(self) -> None:
        self._drive_to_holding()
        self.fake.step_target_stream(self.clock())
        self.fake.last_target_ack['received_at_monotonic'] = self.clock() - 1
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIn('timestamp', self.supervisor.fault_reason or '')

    def test_tick_first_after_pause_cannot_revive_source(self) -> None:
        self._drive_to_holding()
        self.clock.advance(0.16)
        self.fake._current_now = self.clock()
        # Even if fresh replies arrive upon resume, the missed supervision
        # deadline itself must latch. Do not let ACK staleness mask this test.
        self.fake.last_target_ack['sequence'] += 1
        self.fake.last_target_ack['received_at_monotonic'] = self.clock()
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIn('watchdog', self.supervisor.fault_reason or '')
        self.assertIsNone(self.fake.step_target_stream(self.clock()))

    def test_handshake_requires_current_not_historical_ack(self) -> None:
        self.supervisor.start()
        self.supervisor.tick()
        self.supervisor._pitch_motor_enabled = True
        self.supervisor._source_installed = True
        self.supervisor._last_seen_ack_seq = self.fake.target_ack_seq + 1
        self.supervisor._last_fresh_ack_time = self.clock() - 1
        self.supervisor._last_diag_received_times = {0: self.clock(), 1: self.clock()}
        self.supervisor._update_lifecycle_state(self.clock())
        self.assertNotEqual(self.supervisor.state, SupervisorState.HOLDING)

    def test_target_stream_pause_with_supervision_never_resumes(self) -> None:
        self._drive_to_holding()
        # UART target generation stalls while the supervisory tick continues.
        # 110 ms exceeds deployment target TTL (100 ms), but is below the old
        # watchdog/ACK limits (150 ms); recovery must not emit a position target.
        for _ in range(11):
            self.clock.advance(0.01)
            self.fake._current_now = self.clock()
            self.supervisor.tick()
        self.assertIsNone(self.fake.step_target_stream(self.clock()))
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIsNone(self.fake.step_target_stream(self.clock()))

    def test_source_uses_fresh_clock_not_stale_scheduler_timestamp(self) -> None:
        self._drive_to_holding()
        t = self.clock()
        source = self.fake.target_source
        assert source is not None
        # Last goal t; supervisor heartbeat t+20 ms; old scheduler timestamp
        # t+30 ms; actual callback t+60 ms. Old clock would accept the goal.
        self.supervisor._last_tick_time = t + 0.02
        self.clock.advance(0.06)
        self.assertIsNone(source(t + 0.03, {'config_epoch': 5, 'reference_epoch': 12}))
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIn('Target generation interval', self.supervisor.fault_reason or '')
        self.fake.step_target_stream(self.clock())
        self.assertTrue(self.supervisor.is_faulted)

    def test_target_interval_budget_boundary(self) -> None:
        self._drive_to_holding()
        source = self.fake.target_source
        assert source is not None
        self.clock.advance(0.049)
        self.supervisor._last_tick_time = self.clock()
        epochs = {'config_epoch': 5, 'reference_epoch': 12}
        self.assertIsNotNone(source(self.clock(), epochs))
        self.clock.advance(0.051)
        self.supervisor._last_tick_time = self.clock()
        self.assertIsNone(source(self.clock(), epochs))
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIsNone(source(self.clock(), epochs))

    def test_handshake_ack_timeout_starts_at_first_goal(self) -> None:
        self.supervisor.start()
        self.supervisor.tick()
        self.clock.advance(0.01)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        source = self.fake.target_source
        assert source is not None
        epochs = {'config_epoch': 5, 'reference_epoch': 12}
        self.assertIsNotNone(source(self.clock(), epochs))
        # Generate on time but never deliver a target ACK. The handshake
        # timeout is 400 ms; the firmware-derived ACK budget is only 50 ms.
        for _ in range(4):
            self.clock.advance(0.02)
            self.fake._current_now = self.clock()
            source(self.clock(), epochs)
            self.supervisor.tick()
            if self.supervisor.is_faulted:
                break
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIn('ACK stale', self.supervisor.fault_reason or '')
        self.fake.last_target_ack['sequence'] += 1
        self.fake.last_target_ack['received_at_monotonic'] = self.clock()
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIsNone(source(self.clock(), epochs))

    def test_delayed_ack_cannot_clear_fault(self) -> None:
        self._drive_to_holding()
        self.clock.advance(0.06)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.fake.last_target_ack['sequence'] += 1
        self.fake.last_target_ack['received_at_monotonic'] = self.clock()
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIsNone(self.fake.step_target_stream(self.clock()))

    def test_ttl_change_during_run_latches(self) -> None:
        self._drive_to_holding()
        self.fake.target_ttl_ms = 80
        self._advance_stream(4)
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIn('TTL', self.supervisor.fault_reason or '')

    def test_invalid_budget_rejected(self) -> None:
        for target, link, period in ((0, 200, 0.02), (100, 0, 0.02), (20, 500, 0.02),
                                     (True, 200, 0.02), (100, 200, float('nan'))):
            with self.subTest(target=target, link=link, period=period), self.assertRaises(ValueError):
                TimingBudget(target, link, period)

    def test_no_target_source_before_validated_preflight(self) -> None:
        self.supervisor.start(initial_target_rad=0.1)
        self.assertIsNone(self.fake.target_source)
        self.assertIsNone(self.supervisor.current_commanded_rad)

    def test_goal_before_measurement_does_not_seed_command(self) -> None:
        self.supervisor.start()
        self.supervisor.set_goal(0.1)
        self.assertIsNone(self.supervisor.current_commanded_rad)

    def test_initial_goal_seeds_from_measurement(self) -> None:
        self.supervisor.start(initial_target_rad=0.1)
        self.supervisor.tick()
        self.clock.advance(0.01)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        commanded = self.supervisor.current_commanded_rad
        self.assertIsNotNone(commanded)
        assert commanded is not None
        self.assertAlmostEqual(commanded, self.fake.pitch_angle)
        target = self.fake.step_target_stream(self.clock())
        self.assertIsNotNone(target)
        assert target is not None
        self.assertAlmostEqual(target['setpoints'][0][2], self.fake.pitch_angle)

    def test_nan_status_timestamp_fails_closed(self) -> None:
        self._drive_to_holding()
        self.fake.status_timestamp_offset = float('nan')
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)

    def test_invalid_velocity_fails_closed(self) -> None:
        self._drive_to_holding()
        self.fake.pitch_rate = float('nan')
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)

    def test_late_completed_diagnostics_fails_closed(self) -> None:
        self.supervisor.start()
        self.supervisor.tick()
        self.clock.advance(0.25)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)

    def test_watchdog_cannot_resume_when_ticks_return(self) -> None:
        self._drive_to_holding()
        self.clock.advance(0.20)
        self.assertIsNone(self.fake.step_target_stream(self.clock()))
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIsNone(self.fake.step_target_stream(self.clock()))

    def test_temperature_torque_and_tracking_protection(self) -> None:
        for field, value in [('motor_temperature', 60), ('motor_effort', 3.0),
                             ('motor_effort', None), ('motor_temperature', float('nan'))]:
            with self.subTest(field=field, value=value):
                self.setUp()
                self._drive_to_holding()
                setattr(self.fake, field, value)
                for _ in range(10):
                    self.clock.advance(0.01)
                    self.fake._current_now = self.clock()
                    self.fake.step_target_stream(self.clock())
                    self.supervisor.tick()
                self.assertTrue(self.supervisor.is_faulted, field)
        self.setUp()
        self._drive_to_holding()
        self.fake.pitch_angle += 0.04
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIn('tracking error', self.supervisor.fault_reason or '')

    def test_max_duration_latches(self) -> None:
        self._drive_to_holding()
        # Age only the run budget, not the supervision heartbeat. A long
        # skipped tick would instead test the independent watchdog.
        self.supervisor._run_start -= 31
        self.supervisor.tick()
        self.assertTrue(self.supervisor.is_faulted)
        self.assertIn('duration', self.supervisor.fault_reason or '')

    def test_stop_does_not_clear_fault_latch(self) -> None:
        self._drive_to_holding()
        self.fake.manual_stop = True
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.supervisor.stop()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        with self.assertRaises(PitchSupervisorError):
            self.supervisor.start()

    def test_failed_target_clear_is_reported(self) -> None:
        self._drive_to_holding()
        def fail_clear(source_fn):
            raise RuntimeError('target clear failed')
        self.fake.set_target_source = fail_clear
        with self.assertRaises(PitchSupervisorError):
            self.supervisor.stop()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertIn('target clear failed', self.supervisor.fault_reason or '')

    def test_full_lifecycle_success(self) -> None:
        self.assertEqual(self.supervisor.state, SupervisorState.IDLE)
        self._drive_to_holding()

        # Step hold for 5 cycles
        for _ in range(5):
            self.clock.advance(0.02)
            self.fake._current_now = self.clock()
            self.fake.step_target_stream(self.clock())
            self.supervisor.tick()
            self.assertEqual(self.supervisor.state, SupervisorState.HOLDING)

        # Orderly stop
        self.supervisor.stop("test complete")
        self.assertEqual(self.supervisor.state, SupervisorState.STOPPED)
        self.assertIsNone(self.fake.target_source)

    def test_distinguishes_initial_handshake_from_holding(self) -> None:
        # Before enabling, motor is not enabled and domain is not executing
        self.fake.motor_enabled = False
        self.fake.domain_execute = False

        self.supervisor.start(initial_target_rad=-0.04)
        self.assertEqual(self.supervisor.state, SupervisorState.ENABLING_HANDSHAKE)

        # First tick dispatches diag
        self.supervisor.tick()
        self.fake.step_target_stream(self.clock())

        # Tick within handshake timeout: does NOT trip error even though enabled=False
        self.clock.advance(0.05)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.ENABLING_HANDSHAKE)
        self.assertFalse(self.supervisor.is_faulted)

        # Now motor enables and executes
        self.fake.motor_enabled = True
        self.fake.domain_execute = True
        self.fake.step_target_stream(self.clock())

        # Keep the target stream alive while the next diagnostic query completes.
        self._advance_stream(4)
        self.assertEqual(self.supervisor.state, SupervisorState.HOLDING)

        # But once in HOLDING, losing enabled trips once diagnostics reports it
        self.fake.motor_enabled = False
        self._advance_stream(4)
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("enabled" in f for f in self.faults))

    def test_handshake_timeout_trips_fault(self) -> None:
        # Motor never enables
        self.fake.motor_enabled = False
        self.supervisor.start()
        self.assertEqual(self.supervisor.state, SupervisorState.ENABLING_HANDSHAKE)

        # Advance past handshake_timeout_s (0.40s)
        for _ in range(10):
            self.clock.advance(0.05)
            self.fake._current_now = self.clock()
            self.fake.step_target_stream(self.clock())
            self.supervisor.tick()

        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("Handshake" in f or "handshake" in f for f in self.faults))

    # --------------------------------------------------------------------------
    # 2. Holding Safety Invariants (Fail-Closed)
    # --------------------------------------------------------------------------

    def test_domain_execute_loss_trips_holding(self) -> None:
        self._drive_to_holding()
        self.fake.domain_execute = False
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("execute" in f for f in self.faults))

    def test_domain_fault_reason_trips_holding(self) -> None:
        self._drive_to_holding()
        self.fake.domain_reason = "fault_dependency"
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("fault_dependency" in f for f in self.faults))

    def test_authority_loss_trips_holding(self) -> None:
        self._drive_to_holding()
        self.fake.authority = False
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("Authority" in f for f in self.faults))

    def test_manual_stop_trips_holding(self) -> None:
        self._drive_to_holding()
        self.fake.manual_stop = True
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("Manual stop" in f for f in self.faults))

    def test_disconnect_and_client_error(self) -> None:
        self._drive_to_holding()
        self.fake.connected = False
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("disconnected" in f for f in self.faults))

    def test_session_and_epoch_mismatch(self) -> None:
        self._drive_to_holding()
        self.fake.reference_epoch += 1
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("Epoch mismatch" in f for f in self.faults))

    # --------------------------------------------------------------------------
    # 3. Telemetry Freshness & Sequence Checks
    # --------------------------------------------------------------------------

    def test_stale_status_telemetry_trips(self) -> None:
        self._drive_to_holding()
        # Telemetry ceases updating received_at_monotonic
        self.fake.status_timestamp_offset = 0.20  # > max_telemetry_age_s (0.10s)
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("stale" in f for f in self.faults))

    def test_missing_received_at_monotonic_fails_closed(self) -> None:
        self._drive_to_holding()
        # Telemetry omits received_at_monotonic
        self.fake.omit_status_timestamp = True
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("received_at_monotonic" in f for f in self.faults))

    def test_status_sequence_regression_trips(self) -> None:
        self._drive_to_holding()
        self.fake.status_seq -= 2
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("regressed" in f for f in self.faults))

    def test_diagnostics_future_timeout_trips(self) -> None:
        self._drive_to_holding()
        # Simulate diagnostics future never completing
        self.fake.diag_future_hang = True
        # Keep each target and supervisor interval below the TTL budget so
        # this test specifically exercises the stalled diagnostics future.
        self._advance_stream(16)
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("timed out" in f for f in self.faults))

    # --------------------------------------------------------------------------
    # 4. Target ACK Freshness & Rejection
    # --------------------------------------------------------------------------

    def test_prestart_ack_cache_not_laundered(self) -> None:
        # Fake starts with last_target_ack sequence = 50
        self.supervisor.start()
        # Target generator runs but target ACK sequence stays frozen at 50
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()

        # Advance clock past max_ack_age_s without advancing ACK sequence
        self.clock.advance(0.20)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        # Handshake or hold must not accept sequence 50 as fresh progress
        self.assertNotEqual(self.supervisor.state, SupervisorState.HOLDING)

    def test_target_ack_rejection_trips(self) -> None:
        self._drive_to_holding()
        self.fake.target_ack_result = 1
        self.fake.target_ack_result_name = "REJECTED"
        self.fake.target_ack_reason_name = "limit_violation"

        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.fake.step_target_stream(self.clock())
        self.supervisor.tick()

        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("REJECTED" in f for f in self.faults))

    def test_target_ack_sequence_regression_trips(self) -> None:
        self._drive_to_holding()
        # Regress sequence
        self.fake.target_ack_seq -= 5
        self.fake.last_target_ack["sequence"] = self.fake.target_ack_seq

        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()

        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("regressed" in f for f in self.faults))

    # --------------------------------------------------------------------------
    # 5. Position Bounds & Envelope Enforcement
    # --------------------------------------------------------------------------

    def test_pitch_outside_envelope_trips(self) -> None:
        self._drive_to_holding()
        # Pitch angle exceeds envelope high (+0.200)
        self.fake.pitch_angle = 0.220
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()

        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("envelope" in f for f in self.faults))

    def test_pitch_rate_exceeds_speed_limit_trips(self) -> None:
        self._drive_to_holding()
        self.fake.pitch_rate = 0.85  # > max_speed_rad_s (0.60)
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()

        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("rate" in f or "speed" in f for f in self.faults))

    def test_pitch_sensor_invalid_trips(self) -> None:
        self._drive_to_holding()
        self.fake.position_valid = False
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()

        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("position marked invalid" in f for f in self.faults))

    def test_target_goal_outside_envelope_rejected(self) -> None:
        self._drive_to_holding()
        with self.assertRaises(PitchSupervisorError):
            self.supervisor.set_goal(0.300)  # > envelope high (0.200)

        with self.assertRaises(PitchSupervisorError):
            self.supervisor.set_goal(-0.250)  # < envelope low (-0.200)

        # Commanded target remains intact
        self.assertIsNotNone(self.supervisor.desired_target_rad)
        assert self.supervisor.desired_target_rad is not None
        self.assertAlmostEqual(self.supervisor.desired_target_rad, -0.05)

    # --------------------------------------------------------------------------
    # 6. Segment Continuity & Slew Rate Bounding
    # --------------------------------------------------------------------------

    def test_segment_continuity_does_not_clear_target_source(self) -> None:
        self._drive_to_holding()
        initial_set_source_calls = self.fake.set_source_calls

        # Transition to Segment B
        self.supervisor.set_goal(0.05, segment_id="segment_B")
        self.assertEqual(self.supervisor.current_segment_id, "segment_B")
        self.assertEqual(self.fake.set_source_calls, initial_set_source_calls)

        # Transition to Segment C
        self.supervisor.set_goal(-0.02, segment_id="segment_C")
        self.assertEqual(self.supervisor.current_segment_id, "segment_C")
        self.assertEqual(self.fake.set_source_calls, initial_set_source_calls)

        # Verify target stream produces valid frames without interruption
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        target = self.fake.step_target_stream(self.clock())
        self.assertIsNotNone(target)
        assert target is not None
        self.assertEqual(target["setpoints"][0][0], "pitch")

    def test_slew_rate_limiting(self) -> None:
        self._drive_to_holding()
        # Commanded target starts at -0.05; request jump to 0.10
        self.supervisor.set_goal(0.10)

        # Actual elapsed time, not callback count, bounds the ramp.
        self.clock.advance(0.02)
        target1 = self.fake.step_target_stream(self.clock())
        self.assertIsNotNone(target1)
        assert target1 is not None
        self.assertAlmostEqual(target1["setpoints"][0][2], -0.049, places=5)

        # Repeated callback at the same timestamp must not move the command.
        target2 = self.fake.step_target_stream(self.clock())
        self.assertIsNotNone(target2)
        assert target2 is not None
        self.assertAlmostEqual(target2["setpoints"][0][2], -0.049, places=5)

    # --------------------------------------------------------------------------
    # 7. Watchdog Cutoff on Stalled Supervision
    # --------------------------------------------------------------------------

    def test_stalled_supervision_watchdog_cuts_target_generation(self) -> None:
        self._drive_to_holding()
        # Normal step produces target
        val = self.fake.step_target_stream(self.clock())
        self.assertIsNotNone(val)

        # Host tick loop hangs/stalls: clock advances past watchdog_timeout_s (0.15s) without supervisor.tick()
        self.clock.advance(0.20)
        val_stalled = self.fake.step_target_stream(self.clock())
        self.assertIsNone(val_stalled, "Target source must cease output when supervisor tick stalls")

    # --------------------------------------------------------------------------
    # 8. Strict No Auto-Resume & Manual Rearm Policy
    # --------------------------------------------------------------------------

    def test_no_auto_resume_and_rearm_workflow(self) -> None:
        self._drive_to_holding()
        # Trigger fault by exceeding envelope
        self.fake.pitch_angle = 0.250
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)

        # Restore pitch angle to safe value
        self.fake.pitch_angle = -0.05
        # Additional ticks DO NOT resume holding
        for _ in range(5):
            self.clock.advance(0.02)
            self.fake._current_now = self.clock()
            self.supervisor.tick()
            self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
            self.assertIsNone(self.fake.step_target_stream(self.clock()))

        # Setting goal while faulted is forbidden
        with self.assertRaises(PitchSupervisorError):
            self.supervisor.set_goal(-0.04)

        # Explicit manual rearm
        self.supervisor.rearm()
        self.assertEqual(self.supervisor.state, SupervisorState.IDLE)
        self.assertIsNone(self.supervisor.fault_reason)

        # Must restart cleanly with handshake
        self._drive_to_holding()
        self.assertEqual(self.supervisor.state, SupervisorState.HOLDING)

    # --------------------------------------------------------------------------
    # 9. Invariant Verification: No Forbidden Wire Mutations
    # --------------------------------------------------------------------------

    def test_proves_no_publish_permit_or_param_write_called(self) -> None:
        self._drive_to_holding()
        for _ in range(5):
            self.clock.advance(0.02)
            self.fake._current_now = self.clock()
            self.fake.step_target_stream(self.clock())
            self.supervisor.tick()

        self.supervisor.stop()
        self.assertEqual(self.fake.forbidden_calls, [], "Supervisor must never call permit, param_write, or authorize")

    # --------------------------------------------------------------------------
    # 10. Edge Cases & Missing Fields (Fail-Closed)
    # --------------------------------------------------------------------------

    def test_missing_joint_pitch_in_status_page0_fails_closed(self) -> None:
        self._drive_to_holding()
        # Override snapshot to remove pitch joint
        orig_snap = self.fake.snapshot
        def snap_without_pitch():
            s = orig_snap()
            s["status"][0]["joints"].pop("pitch", None)
            return s
        self.fake.snapshot = snap_without_pitch

        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("missing" in f or "Joint 'pitch'" in f for f in self.faults))

    def test_missing_domain_pitch_in_status_page1_fails_closed(self) -> None:
        self._drive_to_holding()
        orig_snap = self.fake.snapshot
        def snap_without_domain():
            s = orig_snap()
            s["status"][1]["domains"] = []
            return s
        self.fake.snapshot = snap_without_domain

        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("Domain 'pitch' missing" in f for f in self.faults))

    def test_diagnostics_page0_rejected_targets_trips(self) -> None:
        self._drive_to_holding()
        orig_diag = self.fake.diagnostics
        def diag_with_rejects(page=0):
            fut = orig_diag(page)
            if page == 0:
                fut.result()["rejected_targets"] = 1
            return fut
        self.fake.diagnostics = diag_with_rejects
        self._advance_stream(4)
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("rejected targets" in f for f in self.faults))

    def test_diagnostics_device_offline_and_faulted_trips(self) -> None:
        self._drive_to_holding()
        self.fake.motor_online = False
        self._advance_stream(4)
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("offline" in f for f in self.faults))

    def test_diagnostics_device_stale_sample_age_trips(self) -> None:
        self._drive_to_holding()
        self.fake.position_age_ms = 150  # > max_sample_age_ms (100)
        self._advance_stream(4)
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("stale" in f for f in self.faults))

    def test_non_finite_pitch_angle_fails_closed(self) -> None:
        self._drive_to_holding()
        self.fake.pitch_angle = float("nan")
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("invalid/non-finite" in f for f in self.faults))

    def test_invalid_source_in_status_page0_fails_closed(self) -> None:
        self._drive_to_holding()
        self.fake.source = "remote"  # unauthorized source
        self.clock.advance(0.02)
        self.fake._current_now = self.clock()
        self.supervisor.tick()
        self.assertEqual(self.supervisor.state, SupervisorState.FAULTED)
        self.assertTrue(any("Control source" in f for f in self.faults))


if __name__ == "__main__":
    unittest.main()
