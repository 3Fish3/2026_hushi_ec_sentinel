"""Async-based pitch hold supervisor for Sentinel mechanism control.

This module provides an offline-first, safety-critical supervisor that coordinates
pitch axis holding via the official AsyncSentinelClient interface (sentinel_async.py).

Core Architecture & Invariants:
1. Pure supervisor library: Does NOT open transport, acquire authority, change gains
   (no param_write), send permit frames, or add an unsafe CLI.
2. Single persistent target source: An epoch-bound, nonblocking source is installed
   in AsyncSentinelClient via set_target_source. Segment goal changes update the bounded
   target without clearing or reinstalling the source.
3. Fail-closed latching: Any violation (disconnection, error, sequence regression,
   stale status/diagnostics/ACK, envelope/physical bound violation, authority loss,
   manual stop, loss of domain execute or enabled state) immediately latches into FAULTED
   state, stops target production, and strictly forbids auto-resumption.
4. Handshake vs Hold: Explicitly distinguishes the initial enabling handshake
   (where enabled/execute may transition within a bounded timeout) from holding
   (where any loss of enabled/execute trips immediately).
5. Watchdog cutoff: The nonblocking target source checks local supervisor freshness;
   if the supervisor tick loop stalls, target streaming ceases automatically.
6. Freshness without cache laundering: Diagnostics freshness is verified by dispatching
   read futures and enforcing dispatch-to-completion deadlines. Cached pre-start data
   is never treated as fresh. Status pages require verified received timestamps; missing
   safety-critical timestamps fail closed.
"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field, replace
from enum import StrEnum
from typing import Any


class SupervisorState(StrEnum):
    IDLE = "IDLE"
    ENABLING_HANDSHAKE = "ENABLING_HANDSHAKE"
    HOLDING = "HOLDING"
    FAULTED = "FAULTED"
    STOPPED = "STOPPED"


class PitchSupervisorError(RuntimeError):
    """Base error raised for invalid operations or safety violations."""


class PitchSafetyFault(PitchSupervisorError):
    """Raised or latched when a safety rule, boundary, or freshness deadline trips."""


@dataclass(frozen=True)
class PhysicalBounds:
    """Rigid mechanical hard-stop limits of the pitch axis (in radians)."""
    low: float
    high: float

    def __post_init__(self) -> None:
        if not (isinstance(self.low, (int, float)) and math.isfinite(self.low)):
            raise ValueError(f"physical low bound must be finite, got {self.low}")
        if not (isinstance(self.high, (int, float)) and math.isfinite(self.high)):
            raise ValueError(f"physical high bound must be finite, got {self.high}")
        if self.low >= self.high:
            raise ValueError(f"physical low ({self.low}) must be strictly less than high ({self.high})")


@dataclass(frozen=True)
class HoldEnvelope:
    """Software hold envelope (in radians) within which holding is strictly confined."""
    low: float
    high: float

    def __post_init__(self) -> None:
        if not (isinstance(self.low, (int, float)) and math.isfinite(self.low)):
            raise ValueError(f"envelope low must be finite, got {self.low}")
        if not (isinstance(self.high, (int, float)) and math.isfinite(self.high)):
            raise ValueError(f"envelope high must be finite, got {self.high}")
        if self.low >= self.high:
            raise ValueError(f"envelope low ({self.low}) must be strictly less than high ({self.high})")


@dataclass(frozen=True)
class SupervisorLimits:
    """Timing budgets, slew rate limits, and freshness deadlines."""
    max_temperature_c: float = 55.0      # Test defaults, not calibrated vehicle ratings
    max_effort_nm: float = 2.0
    max_tracking_error_rad: float = 0.03
    max_run_s: float = 30.0
    max_command_speed_rad_s: float = 0.05  # Time-based command slew bound
    max_step_rad: float = 0.005           # Max commanded angle step per target cycle (~50Hz -> 0.25 rad/s)
    max_speed_rad_s: float = 0.6          # Max observed pitch velocity before trip
    max_telemetry_age_s: float = 0.12     # Max host-measured elapsed time without fresh status
    max_sample_age_ms: float = 100.0      # Max sensor sample age reported by MCU
    max_diagnostics_age_s: float = 0.50   # Max elapsed time without fresh completed diagnostics query
    max_ack_age_s: float = 0.15           # Max elapsed time without fresh accepted TARGETS_ACK
    status_query_interval_s: float = 0.04   # Page 1 at <=25 Hz, not each host tick
    diagnostics_query_interval_s: float = 0.10  # Interval between diagnostics queries
    diagnostics_query_timeout_s: float = 0.20   # Timeout waiting for diagnostics future
    handshake_timeout_s: float = 0.50     # Timeout waiting for enabled/execute handshake
    watchdog_timeout_s: float = 0.15      # Max tick interval before nonblocking source self-inhibits

    def __post_init__(self) -> None:
        for name, val in self.__dict__.items():
            if not isinstance(val, (int, float)) or not math.isfinite(val) or val <= 0:
                raise ValueError(f"Limit {name} must be a positive finite number, got {val}")


@dataclass(frozen=True)
class TimingBudget:
    target_ttl_ms: int
    link_ttl_ms: int
    target_period_s: float

    def __post_init__(self) -> None:
        if (type(self.target_ttl_ms) is not int or type(self.link_ttl_ms) is not int
                or self.target_ttl_ms <= 0 or self.link_ttl_ms <= 0
                or not math.isfinite(self.target_period_s) or self.target_period_s <= 0
                or self.target_period_s * 2 > self.budget_s):
            raise ValueError('invalid TTL or target period lacks two-cycle safety margin')

    @property
    def budget_s(self) -> float:
        return min(self.target_ttl_ms, self.link_ttl_ms) / 2000.0


@dataclass(frozen=True)
class SupervisorConfig:
    """Complete configuration combining physical bounds, envelope, and limits."""
    physical_bounds: PhysicalBounds
    envelope: HoldEnvelope
    timing: TimingBudget
    limits: SupervisorLimits = field(default_factory=SupervisorLimits)
    # F407 的状态页1和诊断共用20Hz应答槽; 原生管道可无此物理限速.
    shared_slow_lane: bool = False

    def __post_init__(self) -> None:
        if self.envelope.low < self.physical_bounds.low:
            raise ValueError(
                f"Envelope low ({self.envelope.low}) violates physical low ({self.physical_bounds.low})"
            )
        if self.envelope.high > self.physical_bounds.high:
            raise ValueError(
                f"Envelope high ({self.envelope.high}) violates physical high ({self.physical_bounds.high})"
            )


class PitchHoldSupervisor:
    """Experimental pitch hold supervisor for AsyncSentinelClient.

    Manages safety auditing, status/diagnostics freshness verification,
    epoch pinning, nonblocking target generation with slew rate limiting,
    and fail-closed fault latching.
    """

    def __init__(
        self,
        peer: Any,
        config: SupervisorConfig,
        clock_fn: Callable[[], float] | None = None,
        on_fault: Callable[[str], None] | None = None,
    ) -> None:
        self.peer = peer
        self.config = config
        self.clock_fn = clock_fn or time.monotonic
        self.on_fault = on_fault

        self._lock = threading.Lock()
        self._state = SupervisorState.IDLE
        self._fault_reason: str | None = None

        # Pinned session & epochs (locked on start())
        self._pinned_session: int | None = None
        self._pinned_config_epoch: int | None = None
        self._pinned_reference_epoch: int | None = None

        # Targets & segment tracking
        self._current_commanded_rad: float | None = None
        self._desired_target_rad: float | None = None
        self._current_segment_id: str | None = None
        self._last_tick_time: float = self.clock_fn()
        self._last_source_time: float | None = None
        self._first_source_time: float | None = None
        self._run_start = self.clock_fn()
        self._source_installed = False
        self._status1_future = None
        self._status1_dispatch_time = 0.0
        self._slow_read_turn = 'status'

        # Telemetry observation state
        self._actual_pitch_rad: float | None = None
        self._actual_pitch_rate: float | None = None
        self._last_status_seq: dict[int, int] = {}
        self._last_status_received_monotonic: dict[int, float] = {}

        # Diagnostics query state
        self._diag_counters: dict[str, int] = {}
        self._pending_diag_futures: dict[int, Any] = {}
        self._pending_diag_dispatch_times: dict[int, float] = {}
        self._last_diag_received_times: dict[int, float] = {}
        self._last_diag_query_dispatch_times: dict[int, float] = {}
        self._pitch_motor_online: bool = False
        self._pitch_motor_enabled: bool = False
        self._pitch_motor_faulted: bool = False

        # ACK freshness tracking
        self._target_rejections_baseline = 0
        self._initial_ack_seq: int = -1
        self._last_seen_ack_seq: int = -1
        self._last_fresh_ack_time: float | None = None

        # Handshake timing
        self._handshake_start_time: float | None = None
        self._handshake_criteria_met: bool = False

    @property
    def state(self) -> SupervisorState:
        with self._lock:
            return self._state

    @property
    def fault_reason(self) -> str | None:
        with self._lock:
            return self._fault_reason

    @property
    def current_commanded_rad(self) -> float | None:
        with self._lock:
            return self._current_commanded_rad

    @property
    def desired_target_rad(self) -> float | None:
        with self._lock:
            return self._desired_target_rad

    @property
    def current_segment_id(self) -> str | None:
        with self._lock:
            return self._current_segment_id

    @property
    def actual_pitch_rad(self) -> float | None:
        with self._lock:
            return self._actual_pitch_rad

    @property
    def actual_pitch_rate(self) -> float | None:
        with self._lock:
            return self._actual_pitch_rate

    @property
    def pitch_observation(self) -> tuple[float | None, float | None, int | None, float | None]:
        """同一份已验证反馈的角度、速度、应答序号与主机接收时刻."""
        with self._lock:
            return (self._actual_pitch_rad, self._actual_pitch_rate,
                    self._last_status_seq.get(0), self._last_status_received_monotonic.get(0))

    def narrow_envelope(self, envelope: HoldEnvelope) -> None:
        """在持续保持中收窄行程, 不重装目标源、不撤销力矩."""
        with self._lock:
            if self._state != SupervisorState.HOLDING:
                raise PitchSupervisorError('Cannot narrow envelope outside HOLDING')
            if envelope.low < self.config.envelope.low or envelope.high > self.config.envelope.high:
                raise PitchSupervisorError('New envelope must remain inside the active envelope')
            values = (self._actual_pitch_rad, self._current_commanded_rad, self._desired_target_rad)
            if any(value is None or not envelope.low <= value <= envelope.high for value in values):
                raise PitchSupervisorError('Position or target is outside the new envelope')
            self.config = replace(self.config, envelope=envelope)

    @property
    def is_holding(self) -> bool:
        with self._lock:
            return self._state == SupervisorState.HOLDING

    @property
    def is_faulted(self) -> bool:
        with self._lock:
            return self._state == SupervisorState.FAULTED

    # --------------------------------------------------------------------------
    # Lifecycle & Commands
    # --------------------------------------------------------------------------

    def start(self, initial_target_rad: float | None = None) -> None:
        """Start the supervisor and begin the initial enabling handshake.

        Pins the active session and epochs from AsyncSentinelClient snapshot,
        records initial ACK sequences (to prevent cache laundering), installs
        the persistent nonblocking target generator, and enters ENABLING_HANDSHAKE.
        """
        now = self.clock_fn()
        with self._lock:
            if self._state not in (SupervisorState.IDLE, SupervisorState.STOPPED):
                raise PitchSupervisorError(f"Cannot start supervisor from state {self._state.value}")

            snap = self.peer.snapshot()
            if not snap.get("connected"):
                raise PitchSupervisorError("Cannot start: AsyncSentinelClient is not connected")
            if snap.get("error"):
                raise PitchSupervisorError(f"Cannot start: client reports active error: {snap['error']}")

            session = snap.get("session")
            config_epoch = snap.get("config_epoch")
            reference_epoch = snap.get("reference_epoch")
            if session is None or session == 0 or config_epoch is None or reference_epoch is None:
                raise PitchSupervisorError(
                    f"Cannot start: invalid/uninitialized session or epochs in client snapshot: "
                    f"session={session}, config_epoch={config_epoch}, reference_epoch={reference_epoch}"
                )

            # Pin session and epochs
            self._pinned_session = int(session)
            self._pinned_config_epoch = int(config_epoch)
            self._pinned_reference_epoch = int(reference_epoch)

            # Require receive-time and rejection evidence from the actual client.
            baseline = snap.get('target_rejections')
            fence = snap.get('last_target_sent_sequence')
            if not isinstance(baseline, int) or not isinstance(fence, int):
                raise PitchSupervisorError('Client lacks target rejection/timestamp evidence extension')
            self._target_rejections_baseline = baseline
            # Record baseline ACK state (pre-start ACK is considered stale)
            last_ack = snap.get("last_target_ack")
            if last_ack and last_ack.get("session") == self._pinned_session and "sequence" in last_ack:
                self._initial_ack_seq = int(last_ack["sequence"])
            else:
                self._initial_ack_seq = -1
            self._initial_ack_seq = max(self._initial_ack_seq, fence)
            self._last_seen_ack_seq = self._initial_ack_seq
            self._last_fresh_ack_time = None

            # Reset diagnostics tracking
            self._diag_counters.clear()
            self._pending_diag_futures.clear()
            self._pending_diag_dispatch_times.clear()
            self._last_diag_received_times.clear()
            self._last_diag_query_dispatch_times.clear()
            self._pitch_motor_online = False
            self._pitch_motor_enabled = False
            self._pitch_motor_faulted = False

            # Set initial target
            if initial_target_rad is not None:
                self._validate_target_in_envelope(initial_target_rad)
                self._current_commanded_rad = None
                self._desired_target_rad = initial_target_rad
            else:
                # Target will be seeded from initial valid measured pitch during handshake
                self._current_commanded_rad = None
                self._desired_target_rad = None

            self._state = SupervisorState.ENABLING_HANDSHAKE
            self._fault_reason = None
            self._handshake_start_time = now
            self._handshake_criteria_met = False
            self._last_tick_time = now
            self._source_installed = False
            self._run_start = now
            self._last_source_time = None
            self._first_source_time = None
            self._status1_future = None
            self._slow_read_turn = 'status'
            self._last_status_seq.clear()
            self._last_status_received_monotonic.clear()

        # No target source until both status pages and device diagnostics pass preflight.

    def set_goal(self, target_rad: float, segment_id: str | None = None) -> None:
        """Update the commanded pitch goal without clearing or reinstalling the target source.

        The target is validated against the software envelope and will be slewed
        towards by the persistent nonblocking target generator.
        """
        self._validate_target_in_envelope(target_rad)
        with self._lock:
            if self._state not in (SupervisorState.ENABLING_HANDSHAKE, SupervisorState.HOLDING):
                raise PitchSupervisorError(f"Cannot set goal in state {self._state.value}")
            self._desired_target_rad = target_rad
            if segment_id is not None:
                self._current_segment_id = segment_id

    def stop(self, reason: str = "normal stop") -> None:
        """Orderly stop pitch holding, clear target generator, and transition to STOPPED."""
        with self._lock:
            faulted = self._state == SupervisorState.FAULTED
            if not faulted:
                self._state = SupervisorState.STOPPED
            self._current_commanded_rad = None
            self._desired_target_rad = None
        try:
            self.peer.set_target_source(None)
        except Exception as error:
            with self._lock:
                self._state = SupervisorState.FAULTED
                self._fault_reason = f'{self._fault_reason or reason}; target clear failed: {error}'
            raise PitchSupervisorError(self._fault_reason) from error

    def rearm(self) -> None:
        """Reset a latched FAULTED state back to IDLE.

        Strictly forbids auto-resumption: holding is NOT resumed.
        A full re-start() with handshake is required to resume operations.
        """
        with self._lock:
            if self._state != SupervisorState.FAULTED:
                raise PitchSupervisorError(f"Cannot rearm from non-faulted state {self._state.value}")
        try:
            self.peer.set_target_source(None)
        except Exception as error:
            with self._lock:
                self._fault_reason = f'{self._fault_reason}; target clear failed: {error}'
            raise PitchSupervisorError(self._fault_reason) from error
        with self._lock:
            self._state = SupervisorState.IDLE
            self._fault_reason = None
            self._current_commanded_rad = None
            self._desired_target_rad = None
            self._current_segment_id = None

    # --------------------------------------------------------------------------
    # Periodic Supervisor Tick
    # --------------------------------------------------------------------------

    def tick(self) -> None:
        """Execute one evaluation cycle of supervisor safety audits and queries.

        Must be called periodically by the host application.
        Audits client connection, status pages, diagnostics futures, target ACKs,
        envelope limits, authority, and executes state transitions.
        Any violation immediately latches into FAULTED.
        """
        now = self.clock_fn()
        with self._lock:
            if self._state in (SupervisorState.IDLE, SupervisorState.STOPPED):
                return
            if self._state == SupervisorState.FAULTED:
                # Already faulted; maintain latch
                return

        if (self._source_installed
                and not 0 <= now - self._last_tick_time <= min(
                    self.config.limits.watchdog_timeout_s, self.config.timing.budget_s)):
            self._latch_fault('Supervisor watchdog expired before tick resumed')
            return
        if now - self._run_start > self.config.limits.max_run_s:
            self._latch_fault('Maximum run duration reached')
            return
        # 1. Audit client health and snapshot invariants
        snap = self.peer.snapshot()
        if not snap.get("connected"):
            self._latch_fault(f"Client disconnected: {snap.get('error', 'link down')}")
            return
        if snap.get("error"):
            self._latch_fault(f"Client reported error: {snap['error']}")
            return

        with self._lock:
            pinned_session = self._pinned_session
            pinned_config_epoch = self._pinned_config_epoch
            pinned_ref_epoch = self._pinned_reference_epoch

        client_session = snap.get("session")
        if client_session != pinned_session:
            self._latch_fault(f"Session mismatch: expected {pinned_session}, client has {client_session}")
            return
        client_cfg_epoch = snap.get("config_epoch")
        client_ref_epoch = snap.get("reference_epoch")
        if client_cfg_epoch != pinned_config_epoch or client_ref_epoch != pinned_ref_epoch:
            self._latch_fault(
                f"Epoch mismatch: expected config={pinned_config_epoch}, ref={pinned_ref_epoch}; "
                f"client has config={client_cfg_epoch}, ref={client_ref_epoch}"
            )
            return

        # Page 1 is not automatically polled by AsyncSentinelClient.
        try:
            if self._status1_future is not None:
                if now - self._status1_dispatch_time > self.config.limits.max_telemetry_age_s:
                    self._latch_fault('Status page 1 request timed out')
                    return
                if self._status1_future.done():
                    self._status1_future.result()
                    self._status1_future = None
                    if self.config.shared_slow_lane:
                        self._slow_read_turn = 'diagnostics'
            diagnostics_due = any(now - self._last_diag_query_dispatch_times.get(page, 0.0)
                                  >= self.config.limits.diagnostics_query_interval_s for page in (0, 1))
            status_lane_free = (not self.config.shared_slow_lane or
                                (not self._pending_diag_futures and
                                 (self._slow_read_turn == 'status' or not diagnostics_due)))
            if (status_lane_free and self._status1_future is None and (self._status1_dispatch_time == 0
                    or now - self._status1_dispatch_time >= self.config.limits.status_query_interval_s)):
                self._status1_future = self.peer.status(1)
                self._status1_dispatch_time = now
        except Exception as error:
            self._latch_fault(f'Status page 1 read failed: {error}')
            return

        # 2. Audit Status Pages (fast telemetry)
        if not self._audit_status_pages(snap, now):
            return

        # 3. Audit Diagnostics & Dispatch Queries
        if not self._audit_diagnostics(snap, now):
            return

        # 4. Audit Target ACK Freshness & Results
        if not self._audit_target_acks(snap, now):
            return

        # Install only after all preflight evidence is available. The first source
        # call holds measured position; subsequent calls slew using elapsed time.
        if (not self._source_installed and self._actual_pitch_rad is not None
                and all(page in self._last_diag_received_times for page in (0, 1))):
            try:
                self.peer.set_target_source(self._goal_source)
                self._source_installed = True
            except Exception as error:
                self._latch_fault(f'Target source installation failed: {error}')
                return
        with self._lock:
            self._last_tick_time = now
        # 5. Handshake vs Holding Transitions
        self._update_lifecycle_state(now)

    # --------------------------------------------------------------------------
    # Safety Auditing Details
    # --------------------------------------------------------------------------

    def _audit_status_pages(self, snap: dict, now: float) -> bool:
        """Audit status page 0 (joints/system) and page 1 (domain execution)."""
        status_dict = snap.get("status", {})
        with self._lock:
            state = self._state
            handshake_start = self._handshake_start_time
            pinned_session = self._pinned_session
            pinned_config_epoch = self._pinned_config_epoch
            pinned_ref_epoch = self._pinned_reference_epoch

        if pinned_session is None or pinned_config_epoch is None or pinned_ref_epoch is None:
            self._latch_fault("Supervisor session/epochs not pinned")
            return False

        # In handshake, status pages might not have arrived yet within a brief grace period
        if 0 not in status_dict or 1 not in status_dict:
            if state == SupervisorState.ENABLING_HANDSHAKE:
                if handshake_start is not None and now - handshake_start > self.config.limits.handshake_timeout_s:
                    self._latch_fault("Handshake timeout: status pages 0 and 1 not received")
                    return False
                return True
            else:
                self._latch_fault("Status pages 0 or 1 missing from client snapshot")
                return False

        page0 = status_dict[0]
        page1 = status_dict[1]

        # Audit page 0
        if not self._audit_single_page_header(page0, 0, pinned_session, pinned_config_epoch, pinned_ref_epoch, now):
            return False
        if not self._audit_status_page0_body(page0, now):
            return False

        # Audit page 1
        if not self._audit_single_page_header(page1, 1, pinned_session, pinned_config_epoch, pinned_ref_epoch, now):
            return False
        return self._audit_status_page1_body(page1, now, state)

    def _audit_single_page_header(
        self, page: dict, page_num: int, expected_session: int,
        expected_cfg: int, expected_ref: int, now: float
    ) -> bool:
        if page.get("session") != expected_session:
            self._latch_fault(f"Status page {page_num} session mismatch: {page.get('session')} != {expected_session}")
            return False
        if page.get("config_epoch") != expected_cfg or page.get("reference_epoch") != expected_ref:
            self._latch_fault(
                f"Status page {page_num} epoch mismatch: config={page.get('config_epoch')} (exp {expected_cfg}), "
                f"ref={page.get('reference_epoch')} (exp {expected_ref})"
            )
            return False

        # Check sequence non-regression
        seq = page.get("sequence")
        if seq is None or not isinstance(seq, int):
            self._latch_fault(f"Status page {page_num} lacks valid sequence number")
            return False
        last_seq = self._last_status_seq.get(page_num, -1)
        if seq < last_seq:
            self._latch_fault(f"Status page {page_num} sequence regressed from {last_seq} to {seq}")
            return False
        self._last_status_seq[page_num] = seq

        # Check host received timestamp freshness (Fail-closed if missing)
        recv_time = page.get("received_at_monotonic")
        if recv_time is None or not isinstance(recv_time, (int, float)) or not math.isfinite(recv_time):
            self._latch_fault(f"Status page {page_num} lacks received_at_monotonic timestamp; fail closed")
            return False
        age = now - recv_time
        if age < -0.001 or age > self.config.limits.max_telemetry_age_s:
            self._latch_fault(f"Status page {page_num} telemetry stale (age={age:.3f}s > {self.config.limits.max_telemetry_age_s}s)")
            return False
        self._last_status_received_monotonic[page_num] = recv_time
        return True

    def _audit_status_page0_body(self, page0: dict, now: float) -> bool:
        if any(not isinstance(page0.get(key), bool) for key in ('authority', 'manual_stop')):
            self._latch_fault('Unknown authority/manual stop fields in status page 0')
            return False
        # Authority and Manual Stop
        if not page0.get("authority", False):
            self._latch_fault("Authority lost in status page 0")
            return False
        if page0.get("manual_stop", False):
            self._latch_fault("Manual stop active in status page 0")
            return False

        # Source verification: must be maintenance or host
        source = page0.get("source")
        if source not in ("maintenance", "host"):
            self._latch_fault(f"Control source in status page 0 is '{source}', expected 'maintenance' or 'host'")
            return False

        # Joints inspection for 'pitch'
        joints = page0.get("joints", {})
        if "pitch" not in joints:
            self._latch_fault("Joint 'pitch' missing in status page 0")
            return False
        pj = joints["pitch"]

        if not pj.get("position_valid", False):
            self._latch_fault("Pitch joint position marked invalid in status page 0")
            return False

        pos_age_ms = pj.get("position_age_ms")
        if (not isinstance(pos_age_ms, (int, float)) or not math.isfinite(pos_age_ms)
                or not 0 <= pos_age_ms <= self.config.limits.max_sample_age_ms):
            self._latch_fault(f"Pitch position sensor age stale: {pos_age_ms} ms > {self.config.limits.max_sample_age_ms} ms")
            return False

        angle_rad = pj.get("angle_rad")
        if angle_rad is None or not (isinstance(angle_rad, (int, float)) and math.isfinite(angle_rad)):
            self._latch_fault(f"Pitch angle invalid/non-finite: {angle_rad}")
            return False

        rate_rad_s = pj.get('rate_rad_s')
        velocity_age = pj.get('velocity_age_ms')
        if (not pj.get('velocity_valid') or not isinstance(rate_rad_s, (int, float))
                or not math.isfinite(rate_rad_s) or not isinstance(velocity_age, (int, float))
                or not math.isfinite(velocity_age) or not 0 <= velocity_age <= self.config.limits.max_sample_age_ms):
            self._latch_fault('Pitch velocity invalid or stale')
            return False
        if abs(rate_rad_s) > self.config.limits.max_speed_rad_s:
            self._latch_fault(f'Pitch speed exceeds limit: {rate_rad_s}')
            return False

        # Physical and Envelope Bounds Audit
        if not (self.config.physical_bounds.low <= angle_rad <= self.config.physical_bounds.high):
            self._latch_fault(
                f"Pitch angle {angle_rad:.4f} rad outside physical bounds "
                f"[{self.config.physical_bounds.low}, {self.config.physical_bounds.high}]"
            )
            return False
        if not (self.config.envelope.low <= angle_rad <= self.config.envelope.high):
            self._latch_fault(
                f"Pitch angle {angle_rad:.4f} rad outside software envelope "
                f"[{self.config.envelope.low}, {self.config.envelope.high}]"
            )
            return False

        with self._lock:
            commanded = self._current_commanded_rad
        if commanded is not None and abs(angle_rad - commanded) > self.config.limits.max_tracking_error_rad:
            self._latch_fault('Pitch tracking error exceeds limit')
            return False
        with self._lock:
            self._actual_pitch_rad = float(angle_rad)
            self._actual_pitch_rate = float(rate_rad_s)
            # If no initial target set, seed from current valid pitch
            if self._current_commanded_rad is None:
                self._current_commanded_rad = float(angle_rad)
            if self._desired_target_rad is None:
                self._desired_target_rad = float(angle_rad)

        return True

    def _audit_status_page1_body(self, page1: dict, now: float, state: SupervisorState) -> bool:
        if any(not isinstance(page1.get(key), bool) for key in ('authority', 'manual_stop')):
            self._latch_fault('Unknown authority/manual stop fields in status page 1')
            return False
        if not page1.get("authority", False):
            self._latch_fault("Authority lost in status page 1")
            return False
        if page1.get("manual_stop", False):
            self._latch_fault("Manual stop active in status page 1")
            return False

        domains = page1.get("domains", [])
        pitch_domain = next((d for d in domains if d.get("domain") == "pitch"), None)
        if pitch_domain is None:
            self._latch_fault("Domain 'pitch' missing in status page 1")
            return False

        # Page 1 measurement is mode-dependent: before position mode it can be
        # a velocity, not an angle. Page 0 is the joint-angle source in preflight.
        if pitch_domain.get('mode') == 'position':
            meas = pitch_domain.get('measurement')
            if (not isinstance(meas, (int, float)) or not math.isfinite(meas)
                    or not self.config.envelope.low <= meas <= self.config.envelope.high):
                self._latch_fault(f'Pitch position-domain measurement outside envelope or invalid: {meas}')
                return False
        for domain in domains:
            if domain.get('domain') != 'pitch' and domain.get('execute'):
                self._latch_fault('Unexpected execution on a non-Pitch domain')
                return False

        validity = pitch_domain.get("validity")
        if validity != "valid":
            self._latch_fault(f"Pitch domain validity is '{validity}', expected 'valid'")
            return False

        if not isinstance(pitch_domain.get('execute'), bool):
            self._latch_fault('Unknown Pitch execute flag')
            return False
        execute = pitch_domain['execute']
        reason = pitch_domain.get("reason", "none")
        mode = pitch_domain.get("mode", "none")
        if reason not in ('none', 'no_target', 'standby'):
            self._latch_fault(f'Pitch domain reported fault reason {reason}')
            return False

        if state == SupervisorState.HOLDING:
            if not execute:
                self._latch_fault(f"Pitch domain lost execute state while holding; reason={reason}")
                return False
            if mode != "position":
                self._latch_fault(f"Pitch domain mode is '{mode}', expected 'position' while holding")
                return False
            if reason != "none":
                self._latch_fault(f"Pitch domain reported fault reason '{reason}' while holding")
                return False

        return True

    def _audit_diagnostics(self, snap: dict, now: float) -> bool:
        """Issue and poll diagnostics read futures for freshness verification without cache laundering."""
        with self._lock:
            state = self._state
            pinned_session = self._pinned_session
            pinned_cfg = self._pinned_config_epoch
            pinned_ref = self._pinned_reference_epoch

        if pinned_session is None or pinned_cfg is None or pinned_ref is None:
            self._latch_fault("Supervisor session/epochs not pinned")
            return False

        # Poll in-flight diagnostics futures
        for page_num in (0, 1):
            future = self._pending_diag_futures.get(page_num)
            if future is not None:
                if future.done():
                    dispatch_time = self._pending_diag_dispatch_times[page_num]
                    del self._pending_diag_futures[page_num]
                    del self._pending_diag_dispatch_times[page_num]

                    if now - dispatch_time > self.config.limits.diagnostics_query_timeout_s:
                        self._latch_fault(f'Diagnostics page {page_num} read future timed out')
                        return False
                    exc = future.exception()
                    if exc is not None:
                        self._latch_fault(f"Diagnostics page {page_num} read failed: {exc}")
                        return False

                    diag_payload = future.result()
                    received = diag_payload.get('received_at_monotonic')
                    if (not isinstance(received, (int, float)) or not math.isfinite(received)
                            or not dispatch_time <= received <= now
                            or now - received > self.config.limits.max_diagnostics_age_s):
                        self._latch_fault('Diagnostics timestamp unavailable or stale')
                        return False
                    if not self._process_fresh_diagnostics_page(diag_payload, page_num, pinned_session, pinned_cfg, pinned_ref):
                        return False

                    self._last_diag_received_times[page_num] = received
                    if self.config.shared_slow_lane:
                        self._slow_read_turn = 'status'
                else:
                    # Check future timeout
                    dispatch_time = self._pending_diag_dispatch_times[page_num]
                    if now - dispatch_time > self.config.limits.diagnostics_query_timeout_s:
                        self._latch_fault(f"Diagnostics page {page_num} read future timed out")
                        return False

        # Enforce diagnostics freshness in HOLDING
        if state == SupervisorState.HOLDING:
            for page_num in (0, 1):
                last_rx = self._last_diag_received_times.get(page_num)
                if last_rx is None or (now - last_rx > self.config.limits.max_diagnostics_age_s):
                    age = now - (last_rx or 0)
                    self._latch_fault(f"Diagnostics page {page_num} stale (age={age:.3f}s > {self.config.limits.max_diagnostics_age_s}s)")
                    return False

        # Dispatch new diagnostics queries if interval has elapsed and no query in-flight
        if self.config.shared_slow_lane and (self._status1_future is not None or
                self._pending_diag_futures or self._slow_read_turn != 'diagnostics'):
            return True
        pages = sorted((0, 1), key=lambda page: self._last_diag_query_dispatch_times.get(page, 0.0))
        for page_num in pages:
            if page_num not in self._pending_diag_futures:
                last_dispatch = self._last_diag_query_dispatch_times.get(page_num, 0.0)
                if now - last_dispatch >= self.config.limits.diagnostics_query_interval_s:
                    try:
                        fut = self.peer.diagnostics(page=page_num)
                        self._pending_diag_futures[page_num] = fut
                        self._pending_diag_dispatch_times[page_num] = now
                        self._last_diag_query_dispatch_times[page_num] = now
                        if self.config.shared_slow_lane:
                            break  # 一次诊断之后先刷新状态页1, 不让两份诊断夹住一次执行状态.
                    except Exception as err:
                        self._latch_fault(f"Failed to dispatch diagnostics page {page_num}: {err}")
                        return False

        return True

    def _process_fresh_diagnostics_page(
        self, payload: dict, page_num: int, expected_session: int,
        expected_cfg: int, expected_ref: int
    ) -> bool:
        if payload.get('session') != expected_session or payload.get('page') != page_num:
            self._latch_fault('Diagnostics identity mismatch')
            return False
        if page_num == 0:
            if payload.get("session") != expected_session:
                self._latch_fault(f"Diagnostics page 0 session mismatch: {payload.get('session')} != {expected_session}")
                return False
            if payload.get("config_epoch") != expected_cfg or payload.get("reference_epoch") != expected_ref:
                self._latch_fault(
                    f"Diagnostics page 0 epoch mismatch: config={payload.get('config_epoch')} (exp {expected_cfg}), "
                    f"ref={payload.get('reference_epoch')} (exp {expected_ref})"
                )
                return False
            if any(
                type(payload.get(name)) is not int or payload[name] != expected
                for name, expected in (('target_ttl_ms', self.config.timing.target_ttl_ms),
                                       ('link_ttl_ms', self.config.timing.link_ttl_ms))
            ):
                self._latch_fault('Firmware TTL missing or changed')
                return False
            for name in ('rejected_targets', 'frame_rejects'):
                value = payload.get(name)
                if not isinstance(value, int) or value < 0:
                    self._latch_fault(f'Diagnostics counter unavailable: {name}')
                    return False
                if name not in self._diag_counters:
                    self._diag_counters[name] = value  # read-only preflight, before target source installation
                elif value != self._diag_counters[name]:
                    self._latch_fault(f'Diagnostics page 0 reported new rejected targets/frame rejects: {name}')
                    return False

        elif page_num == 1:
            # Device page: inspect domain 'pitch'
            devices = payload.get("devices", [])
            pitch_dev = next((d for d in devices if d.get("domain") == "pitch"), None)
            if pitch_dev is None:
                self._latch_fault("Pitch device missing in diagnostics page 1")
                return False

            if any(not isinstance(pitch_dev.get(key), bool) for key in ('installed', 'online', 'enabled', 'faulted')):
                self._latch_fault('Pitch device has unknown safety flags')
                return False
            if not pitch_dev.get("installed", False):
                self._latch_fault("Pitch device marked not installed in diagnostics page 1")
                return False
            if not pitch_dev.get("online", False):
                self._latch_fault("Pitch device offline in diagnostics page 1")
                return False
            if pitch_dev.get("faulted", False):
                self._latch_fault("Pitch device faulted in diagnostics page 1")
                return False

            sample_age_ms = pitch_dev.get("age_ms")
            if (not isinstance(sample_age_ms, (int, float)) or not math.isfinite(sample_age_ms)
                    or not 0 <= sample_age_ms <= self.config.limits.max_sample_age_ms):
                self._latch_fault(f"Pitch device sample age stale: {sample_age_ms} ms")
                return False

            for field in ('temperature_c', 'rotor_temperature_c'):
                value = pitch_dev.get(field)
                if (not isinstance(value, (int, float)) or not math.isfinite(value)
                        or value < 0 or value >= self.config.limits.max_temperature_c):
                    self._latch_fault(f'Pitch {field} unavailable or exceeds temperature limit')
                    return False
            effort = pitch_dev.get('effort')
            if (pitch_dev.get('effort_unit') != 'Nm' or not isinstance(effort, (int, float))
                    or not math.isfinite(effort) or abs(effort) >= self.config.limits.max_effort_nm):
                self._latch_fault('Pitch effort unavailable or exceeds torque limit')
                return False
            enabled = pitch_dev.get("enabled", False)
            with self._lock:
                self._pitch_motor_online = True
                self._pitch_motor_enabled = enabled
                self._pitch_motor_faulted = False
                state = self._state

            if state == SupervisorState.HOLDING and not enabled:
                self._latch_fault("Pitch motor lost enabled state while holding")
                return False

        return True

    def _audit_target_acks(self, snap: dict, now: float) -> bool:
        """Audit target ACKs from client snapshot for freshness and acceptance.

        Enforces that ACKs advance beyond the initial fence, checks session and result,
        and ensures continuous freshness once target streaming has begun.
        """
        counter = snap.get('target_rejections')
        if counter != self._target_rejections_baseline:
            self._latch_fault('Target rejection evidence changed (including overwritten ACK)')
            return False
        ack = snap.get("last_target_ack")
        with self._lock:
            state = self._state
            pinned_session = self._pinned_session
            last_seen_seq = self._last_seen_ack_seq
            last_fresh_time = self._last_fresh_ack_time

        if ack is not None:
            ack_session = ack.get("session")
            ack_seq = ack.get("sequence")
            ack_result = ack.get("result")
            ack_result_name = ack.get("result_name")
            ack_reason = ack.get("reason_name")

            if ack_session is not None and ack_session != pinned_session:
                if last_fresh_time is not None:
                    self._latch_fault(f"Target ACK session mismatch: {ack_session} != {pinned_session}")
                    return False
            elif ack_seq is not None:
                if ack_seq < last_seen_seq:
                    self._latch_fault(f"Target ACK sequence regressed: {ack_seq} < {last_seen_seq}")
                    return False
                elif ack_seq > last_seen_seq:
                    # New ACK received! Check rejection
                    if ack_result != 0 or ack_result_name != "ACCEPTED":
                        self._latch_fault(f"Target rejected by MCU: result={ack_result_name} ({ack_result}), reason={ack_reason}")
                        return False
                    received = ack.get('received_at_monotonic')
                    if (not isinstance(received, (int, float)) or not math.isfinite(received)
                            or not 0 <= now - received <= min(
                                self.config.limits.max_ack_age_s, self.config.timing.budget_s)
                            or received < self._run_start):
                        self._latch_fault('Target ACK timestamp unavailable or stale')
                        return False
                    with self._lock:
                        self._last_seen_ack_seq = ack_seq
                        self._last_fresh_ack_time = received

        # Start ACK deadline only when the first target has been generated.
        # Handshake must not conceal a missing ACK until its larger timeout.
        with self._lock:
            current_fresh_time = self._last_fresh_ack_time
            first_source_time = self._first_source_time
        ack_budget = min(self.config.limits.max_ack_age_s, self.config.timing.budget_s)
        since = current_fresh_time if current_fresh_time is not None else first_source_time
        if (state == SupervisorState.HOLDING and since is None) or (since is not None and now - since > ack_budget):
            self._latch_fault(f'Target ACK stale or missing (budget={ack_budget:.3f}s)')
            return False

        return True

    def _update_lifecycle_state(self, now: float) -> None:
        """Drive the state transition from ENABLING_HANDSHAKE to HOLDING."""
        with self._lock:
            if self._state != SupervisorState.ENABLING_HANDSHAKE:
                return
            handshake_start = self._handshake_start_time
            last_seen_ack_seq = self._last_seen_ack_seq
            initial_ack_seq = self._initial_ack_seq
            pitch_enabled = self._pitch_motor_enabled

        # Handshake criteria:
        # 1. Diagnostics confirmed pitch motor enabled == True
        # 2. Status page 1 confirmed pitch domain execute == True
        # 3. Fresh target ACK received and accepted (sequence > initial_ack_seq)
        snap = self.peer.snapshot()
        status_page1 = snap.get("status", {}).get(1, {})
        pitch_dom = next((d for d in status_page1.get("domains", []) if d.get("domain") == "pitch"), {})
        pitch_executing = pitch_dom.get("execute", False)
        ack_progress = (last_seen_ack_seq > initial_ack_seq)

        evidence_fresh = (
            self._last_fresh_ack_time is not None
            and 0 <= now - self._last_fresh_ack_time <= min(
                self.config.limits.max_ack_age_s, self.config.timing.budget_s)
            and all(0 <= now - self._last_diag_received_times.get(page, -math.inf)
                    <= self.config.limits.max_diagnostics_age_s for page in (0, 1))
            and all(0 <= now - self._last_status_received_monotonic.get(page, -math.inf)
                    <= self.config.limits.max_telemetry_age_s for page in (0, 1)))
        if (pitch_enabled and pitch_executing and ack_progress and self._source_installed
                and evidence_fresh):
            with self._lock:
                self._state = SupervisorState.HOLDING
                self._handshake_criteria_met = True
            return

        # Check handshake timeout
        if handshake_start is not None and (now - handshake_start > self.config.limits.handshake_timeout_s):
            missing = []
            if not pitch_enabled:
                missing.append("motor_enabled=False")
            if not pitch_executing:
                missing.append("domain_execute=False")
            if not ack_progress:
                missing.append("target_ack_progress=False")
            self._latch_fault(f"Enabling handshake timed out ({', '.join(missing)})")

    # --------------------------------------------------------------------------
    # Nonblocking Target Source (Called on Client Background Thread)
    # --------------------------------------------------------------------------

    def _goal_source(self, now_s: float, epochs: dict) -> dict | None:
        """Persistent nonblocking target generator invoked by AsyncSentinelClient.

        Operates safely on the client UART thread without blocking or acquiring
        external locks. Checks pinned epochs and host tick watchdog. Slews the
        commanded target towards the desired goal within the software envelope.
        """
        now_s = self.clock_fn()  # The caller's scheduler timestamp can predate a blocking send.
        with self._lock:
            # 1. State check: only generate during handshake or holding
            if self._state not in (SupervisorState.ENABLING_HANDSHAKE, SupervisorState.HOLDING):
                return None

            # 2. Epoch pinning check
            if epochs.get("config_epoch") != self._pinned_config_epoch or epochs.get("reference_epoch") != self._pinned_reference_epoch:
                self._state = SupervisorState.FAULTED
                self._fault_reason = 'Target source epoch mismatch'
                return None

            # 3. Watchdog check: refuse targets if host supervisor tick loop stalled
            if not 0 <= now_s - self._last_tick_time <= min(
                    self.config.limits.watchdog_timeout_s, self.config.timing.budget_s):
                self._state = SupervisorState.FAULTED
                self._fault_reason = 'Supervisor watchdog expired'
                return None

            if now_s - self._run_start > self.config.limits.max_run_s:
                self._state = SupervisorState.FAULTED
                self._fault_reason = 'Maximum run duration reached'
                return None
            if (self._last_source_time is not None
                    and not 0 <= now_s - self._last_source_time <= self.config.timing.budget_s):
                self._state = SupervisorState.FAULTED
                self._fault_reason = 'Target generation interval exceeded firmware TTL safety budget'
                return None
            # 4. If target not initialized yet, withhold output until seeded
            if self._desired_target_rad is None:
                return None

            if self._current_commanded_rad is None:
                return None

            # First output is measured position; repeated calls at one instant
            # cannot move the command. Bound dt after pauses as well as step size.
            dt = 0.0 if self._last_source_time is None else max(0.0, now_s - self._last_source_time)
            self._last_source_time = now_s
            if self._first_source_time is None:
                self._first_source_time = now_s
            step = min(self.config.limits.max_step_rad,
                       self.config.limits.max_command_speed_rad_s * dt)
            diff = self._desired_target_rad - self._current_commanded_rad
            if abs(diff) <= step:
                self._current_commanded_rad = self._desired_target_rad
            elif diff > 0:
                self._current_commanded_rad += step
            else:
                self._current_commanded_rad -= step

            # 6. Contain strictly within envelope
            clamped = max(self.config.envelope.low, min(self.config.envelope.high, self._current_commanded_rad))
            self._current_commanded_rad = clamped

            # Return standard target frame dict matching AsyncSentinelClient requirements
            return dict(epochs, setpoints=[("pitch", "position", clamped, 0)])

    # --------------------------------------------------------------------------
    # Internal Helpers
    # --------------------------------------------------------------------------

    def _validate_target_in_envelope(self, target_rad: float) -> None:
        if not (isinstance(target_rad, (int, float)) and math.isfinite(target_rad)):
            raise PitchSupervisorError(f"Target pitch must be a finite number, got {target_rad}")
        if not (self.config.envelope.low <= target_rad <= self.config.envelope.high):
            raise PitchSupervisorError(
                f"Target pitch {target_rad:.4f} rad exceeds envelope limits "
                f"[{self.config.envelope.low}, {self.config.envelope.high}]"
            )

    def _latch_fault(self, reason: str) -> None:
        """Permanently latch the supervisor into FAULTED state and cut target source."""
        callback = None
        with self._lock:
            if self._state == SupervisorState.FAULTED:
                return
            self._state = SupervisorState.FAULTED
            self._fault_reason = reason
            callback = self.on_fault

        # Attempt to clear target source on client to cease transmission immediately
        try:
            self.peer.set_target_source(None)
        except Exception as error:
            with self._lock:
                self._fault_reason = f'{reason}; target clear failed: {error}'

        if callback is not None:
            try:
                callback(reason)
            except Exception as error:
                with self._lock:
                    self._fault_reason = f'{self._fault_reason}; fault callback failed: {error}'
