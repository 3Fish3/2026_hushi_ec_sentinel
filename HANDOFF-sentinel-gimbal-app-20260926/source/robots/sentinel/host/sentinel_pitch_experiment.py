"""单 UART Pitch 连续保持 + 方波辨识编排: 有界异步写入, 不自动写增益.

一条串口同时承载 v3 业务帧与原始连续流 (0x32/0xB2/0xB3). 编排顺序固定为:
身份/边界/拓扑/TTL/增益快照 -> 授权与保持确认 -> 可选限速抬升与稳定 -> 连续流 -> 激励 ->
监督状态/ACK/流/写入/守卫. 收到 stop 会撤销保持力矩, 机械支撑是前提.

串口路径必须显式给出全部物理与配方边界 (无虚构默认值); --backend 只接软件夹具,
其虚拟时长允许缺省. 不写 PID/增益/TTL, 不产生可部署参数.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import secrets
import signal
import struct
import threading
import time
from collections.abc import Callable
from contextlib import nullcontext
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any

import sentinel_client as wire
import sentinel_raw as raw
from sentinel_async import AsyncSentinelClient
from sentinel_pitch_hold import WARNING, ExclusiveSerial, Keys
from sentinel_pitch_hold_model import (
    HoldEnvelope,
    PhysicalBounds,
    PitchHoldSupervisor,
    SupervisorConfig,
    SupervisorLimits,
    TimingBudget,
)

SOURCE = Path(__file__).resolve().parents[3]
ROOT = SOURCE.parents[1]
METADATA_BYTES = 8192
EXPECTED_TARGET_TTL_MS = 100      # 固件目标有效期契约 (board_app_config.hpp)
MAX_HOLD_BUDGET_S = 0.05          # 监督预算 = min(target, link) TTL / 2
MIN_STOP_MARGIN_RAD = 0.02        # 行程包络与机械止挡之间的最小余量
SUPERVISE_PERIOD_S = 0.004
QUERY_PERIOD_S = 0.1
EXPECTED_PITCH_DEVICE = {'installed': True, 'bus': 2, 'device_id': 1, 'feedback_id': 0x11, 'protocol': 128}


class SrawWriter:
    """有界异步 .sraw 写入器: 队列满/磁盘失败都成为可见错误, 从不阻塞 UART 线程."""

    JOIN_TIMEOUT_S = 2.0

    def __init__(self, path, metadata, *, capacity=2048, opener=None):
        self.path = Path(path)
        self.metadata = dict(metadata)
        self.error = ''
        self.count = 0        # 已入队
        self.written = 0      # 真正写进文件的条数, 落盘故障后不得用入队数冒充
        self.finished = False
        self._first_sample_us = None
        self._queue: queue.Queue = queue.Queue(maxsize=capacity)
        self._done = threading.Event()
        self._file = (opener or (lambda target: target.open('xb')))(self.path)
        try:
            header = self._encode(self.metadata)
            self._write_exact(raw.MAGIC + struct.pack('<I', len(header)) + header)
            self._file.flush()
        except BaseException:
            self._file.close()
            raise
        self._thread = threading.Thread(target=self._drain, daemon=True, name='pitch-sraw')
        self._thread.start()

    def _write_exact(self, blob):
        """短写必须显式失败: 静默截断会让记录长度或元数据悄悄错位."""
        size = self._file.write(blob)
        if size != len(blob):
            raise OSError(f'short write: {size}/{len(blob)} bytes')

    @staticmethod
    def _encode(metadata):
        blob = json.dumps(metadata, ensure_ascii=False, allow_nan=False).encode('utf-8')
        if len(blob) > METADATA_BYTES:
            raise ValueError(f'metadata exceeds {METADATA_BYTES}B: {len(blob)}')
        return blob + b' ' * (METADATA_BYTES - len(blob))

    def append(self, status, records):
        """AsyncSentinelClient 的 sink: 只做入队/边界检查, 不做任何磁盘 I/O."""
        if self.error:
            raise RuntimeError(self.error)
        for record in records:
            if len(record) != raw.SIZE:
                self.error = f'record size {len(record)} != {raw.SIZE}'
                raise RuntimeError(self.error)
            try:
                self._queue.put_nowait(record)
            except queue.Full as error:
                self.error = 'capture queue overflow'
                raise RuntimeError(self.error) from error
            self.count += 1

    def _drain(self):
        try:
            while not self._done.is_set() or not self._queue.empty():
                try:
                    record = self._queue.get(timeout=0.02)
                except queue.Empty:
                    continue
                if self._first_sample_us is None:
                    # 记录布局 '<4B7I': 4B 头之后依次为 seq/source/sampled, 采样时基在偏移 12.
                    self._first_sample_us = struct.unpack_from('<I', record, 12)[0]
                self._write_exact(record)
                self._file.flush()
                self.written += 1
        except Exception as error:
            self.error = f'disk write failed: {error}'
        finally:
            self._rewrite_origin()
            try:
                self._file.close()
            except Exception as error:
                self.error = self.error or f'close failed: {error}'

    def _rewrite_origin(self):
        """用首条记录的回填采样时基重写等长头部, 令 offline loader 的 t_s 有意义."""
        if self._first_sample_us is None:
            return
        try:
            header = self._encode(dict(self.metadata, start_us=self._first_sample_us))
            self._file.seek(len(raw.MAGIC) + 4)
            self._write_exact(header)
            self._file.flush()
        except Exception as error:
            self.error = self.error or f'metadata rewrite failed: {error}'

    def close(self):
        self._done.set()
        self._thread.join(timeout=self.JOIN_TIMEOUT_S)
        self.finished = not self._thread.is_alive()
        if not self.finished:
            self.error = self.error or 'sraw writer did not finish; file may be incomplete'
        if self.error:
            raise RuntimeError(f'{self.error} (written {self.written}/{self.count} queued)')


@dataclass(frozen=True)
class ExperimentRecipe:
    run: int
    amplitude_nm: float
    half_period_us: int
    baseline_us: int
    cycles: int
    final_hold_us: int

    def __post_init__(self):
        if type(self.run) is not int or not 0 < self.run <= 0xffffffff:
            raise ValueError(f'run must be a non-zero uint32, got {self.run!r}')
        if not math.isfinite(self.amplitude_nm) or self.amplitude_nm <= 0:
            raise ValueError(f'amplitude_nm must be positive and finite, got {self.amplitude_nm}')
        for name in ('half_period_us', 'baseline_us', 'cycles', 'final_hold_us'):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f'{name} must be a positive integer, got {value!r}')

    @property
    def excitation_us(self):
        return self.baseline_us + 2 * self.half_period_us * self.cycles + self.final_hold_us


@dataclass(frozen=True)
class LiftPlan:
    """抬升配方; 实车必须显式给出速率、到位阈值与等待时长."""
    target_rad: float
    speed_rad_s: float
    timeout_s: float
    settle_s: float
    position_tolerance_rad: float
    speed_tolerance_rad_s: float

    def __post_init__(self):
        if not math.isfinite(self.target_rad):
            raise ValueError('lift target must be finite')
        for name in ('speed_rad_s', 'timeout_s', 'settle_s', 'position_tolerance_rad',
                     'speed_tolerance_rad_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'lift {name} must be positive and finite')
        if self.settle_s >= self.timeout_s:
            raise ValueError('lift timeout must exceed settling duration')


@dataclass(frozen=True)
class ExperimentPlan:
    recipe: ExperimentRecipe
    stream_duration_us: int
    physical: PhysicalBounds
    excursion_rad: float
    limits: SupervisorLimits
    stream_packet_min: int = 2
    hold_timeout_s: float = 1.0
    post_hold_s: float = 0.5
    lift: LiftPlan | None = None
    remote_handoff_s: float = 0.0

    def __post_init__(self):
        if type(self.stream_duration_us) is not int or self.stream_duration_us <= self.recipe.excitation_us:
            raise ValueError('stream duration must exceed the whole excitation waveform')
        if not math.isfinite(self.excursion_rad) or not 0 < self.excursion_rad <= 0.02:
            raise ValueError(f'excursion must be in (0, 0.02] rad, got {self.excursion_rad}')
        if type(self.stream_packet_min) is not int or self.stream_packet_min < 1:
            raise ValueError('stream_packet_min must be a positive integer')
        for name in ('hold_timeout_s', 'post_hold_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite, got {value}')
        required_us = self.recipe.excitation_us + int(self.post_hold_s * 1e6) + 100000
        if not required_us <= self.stream_duration_us < 0x80000000:
            raise ValueError('capture must cover excitation, post-hold and 100ms sync margin within uint31')
        if self.lift is not None:
            if self.lift.position_tolerance_rad > self.excursion_rad:
                raise ValueError('lift position tolerance must fit the capture excursion')
            if self.lift.speed_rad_s > self.limits.max_speed_rad_s:
                raise ValueError('lift command speed must not exceed the observed speed limit')
            if self.lift.speed_tolerance_rad_s > self.limits.max_speed_rad_s:
                raise ValueError('lift settling speed must not exceed the observed speed limit')
        if not math.isfinite(self.remote_handoff_s) or self.remote_handoff_s < 0:
            raise ValueError('remote handoff wait must be finite and nonnegative')


@dataclass
class _Run:
    """一次辨识的共享状态; clock/sleep 可注入, 便于离线确定性验证."""
    peer: Any
    plan: ExperimentPlan
    stop_event: threading.Event
    keys: Any = None
    clock: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep
    guard: PitchHoldSupervisor | None = None
    writer: SrawWriter | None = None
    progress: Callable[[dict], None] | None = None


def _notify(run, state, **values):
    if run.progress is not None:
        run.progress(dict(state=state, **values))


def _envelopes(plan, initial):
    """抬升路径包含当前位置, 最终辨识范围仍远离机械止点."""
    target = initial if plan.lift is None else plan.lift.target_rad
    capture = HoldEnvelope(target - plan.excursion_rad, target + plan.excursion_rad)
    if (capture.low - plan.physical.low < MIN_STOP_MARGIN_RAD
            or plan.physical.high - capture.high < MIN_STOP_MARGIN_RAD):
        raise RuntimeError('capture posture is too close to a mechanical stop')
    if not plan.physical.low <= initial <= plan.physical.high:
        raise RuntimeError('initial pitch position is outside physical bounds')
    if plan.lift is None:
        return capture, capture
    minimum = abs(target - initial) / plan.lift.speed_rad_s + plan.lift.settle_s
    if minimum >= plan.lift.timeout_s:
        raise RuntimeError('lift timeout cannot cover the requested travel and settling duration')
    travel = HoldEnvelope(max(plan.physical.low, min(initial, capture.low) - plan.excursion_rad),
                          min(plan.physical.high, max(initial, capture.high) + plan.excursion_rad))
    return travel, capture


def _bounded(future, timeout_s, what):
    try:
        return future.result(timeout=timeout_s)
    except TimeoutError as error:
        raise TimeoutError(f'{what}: no reply within {timeout_s}s') from error


def _checked(reply, what):
    if not isinstance(reply, dict) or reply.get('result') != 0:
        raise RuntimeError(f'{what} rejected: {reply}')
    return reply


def _fresh_diagnostics(run):
    """本轮会话内的新鲜计时页; 用固件 TTL 派生 50ms 监督预算."""
    page = _bounded(run.peer.diagnostics(0), 1.0, 'timing diagnostics')
    received = page.get('received_at_monotonic')
    if (page.get('page') != 0 or not isinstance(received, (int, float))
            or not 0 <= run.clock() - received <= 0.12):
        raise RuntimeError('fresh firmware timing diagnostics required')
    if page.get('target_ttl_ms') != EXPECTED_TARGET_TTL_MS:
        raise RuntimeError(f'unexpected firmware target TTL {page.get("target_ttl_ms")} ms; '
                           f'this tool is bound to {EXPECTED_TARGET_TTL_MS} ms')
    return page


def _operator_stop(run):
    if run.stop_event.is_set():
        return True
    return run.keys is not None and run.keys.read() in ('q', ' ', '\x1b', '\x03')


def _supervise(run, what):
    if run.guard is None:
        raise RuntimeError('supervisor is not running')
    run.guard.tick()
    if run.guard.is_faulted:
        raise RuntimeError(run.guard.fault_reason or 'hold supervisor faulted')
    if run.writer is not None and run.writer.error:
        raise RuntimeError(f'capture writer failed: {run.writer.error}')
    snapshot = run.peer.snapshot()
    fault = snapshot.get('pitch_stream', {}).get('fault')
    if fault is not None:
        raise RuntimeError(f'capture fault {fault.code}: {fault.detail}')
    if snapshot.get('error'):
        raise RuntimeError(f'client reported error: {snapshot["error"]} ({what})')


def _await_holding(run):
    deadline = run.clock() + run.plan.hold_timeout_s
    while True:
        if _operator_stop(run):
            raise RuntimeError('operator stop before holding was confirmed')
        _supervise(run, 'holding handshake')
        if run.guard is not None and run.guard.is_holding:
            return
        if run.clock() > deadline:
            raise RuntimeError('holding was not confirmed within the bounded handshake window')
        run.sleep(SUPERVISE_PERIOD_S)


def _await_lift(run, result):
    """目标按既有限速器推进; 只用连续的新反馈判断实际到位与静止."""
    lift = run.plan.lift
    if lift is None or run.guard is None:
        return
    started = run.clock()
    settled_since = None
    previous_sequence = None
    previous_received = None
    run.guard.set_goal(lift.target_rad, 'lift')
    result.update(target_rad=lift.target_rad, settled=False)
    while True:
        if _operator_stop(run):
            raise RuntimeError('operator stop during lift')
        _supervise(run, 'lift')
        now = run.clock()
        if now - started >= lift.timeout_s:
            raise RuntimeError('lift did not settle inside the bounded lift window')
        angle, rate, sequence, received = run.guard.pitch_observation
        if (sequence != previous_sequence and received is not None and received >= started
                and (previous_received is None or received > previous_received)):
            if (previous_received is not None
                    and received - previous_received > run.guard.config.limits.max_telemetry_age_s):
                settled_since = None
            previous_sequence, previous_received = sequence, received
            commanded = run.guard.current_commanded_rad
            reached = (commanded is not None and abs(commanded - lift.target_rad) <= 1e-7
                       and angle is not None and abs(angle - lift.target_rad) <= lift.position_tolerance_rad
                       and rate is not None and abs(rate) <= lift.speed_tolerance_rad_s)
            settled_since = (received if settled_since is None else settled_since) if reached else None
            _notify(run, 'SETTLING' if reached else 'LIFTING', angle_rad=angle, rate_rad_s=rate,
                    target_rad=lift.target_rad, elapsed_s=now - started)
            if settled_since is not None and received - settled_since >= lift.settle_s:
                result.update(settled=True, actual_rad=angle, duration_s=now - started,
                              settled_at_monotonic=now)
                return
        run.sleep(SUPERVISE_PERIOD_S)


def _wait_capture_request(run, request, timeout_s):
    deadline = run.clock() + timeout_s
    while not request.is_set():
        if _operator_stop(run):
            raise RuntimeError('operator stop while holding before capture')
        _supervise(run, 'waiting for capture')
        if run.clock() >= deadline:
            raise RuntimeError('capture was not requested inside the holding window')
        _notify(run, 'READY', remaining_s=max(0, deadline - run.clock()),
                angle_rad=run.guard.actual_pitch_rad, rate_rad_s=run.guard.actual_pitch_rate)
        run.sleep(SUPERVISE_PERIOD_S)


def _supervised_reply(run, future, timeout_s, what):
    deadline = run.clock() + timeout_s
    while not future.done():
        if _operator_stop(run):
            raise RuntimeError(f'operator stop during {what}')
        _supervise(run, what)
        if run.clock() >= deadline:
            raise TimeoutError(f'{what}: reply timed out')
        run.sleep(SUPERVISE_PERIOD_S)
    return future.result()


def _await_stream(run, future):
    deadline = run.clock() + max(run.plan.hold_timeout_s, MAX_HOLD_BUDGET_S)
    while True:
        if _operator_stop(run):
            raise RuntimeError('operator stop before the capture stream was confirmed')
        _supervise(run, 'stream confirmation')
        if future.done():
            result = future.result()
            if result.get('run') != run.plan.recipe.run:
                raise RuntimeError(f'capture stream identity mismatch: {result}')
            packets = run.peer.snapshot().get('pitch_stream', {}).get('packets', 0)
            if packets >= run.plan.stream_packet_min:
                return result
        if run.clock() > deadline:
            raise RuntimeError('capture stream was not confirmed in time')
        run.sleep(SUPERVISE_PERIOD_S)


def _start_excitation(run):
    recipe = run.plan.recipe
    future = run.peer.pitch_identification('start', run_id=recipe.run, amplitude_nm=recipe.amplitude_nm,
                                           half_period_us=recipe.half_period_us,
                                           baseline_us=recipe.baseline_us, cycles=recipe.cycles,
                                           final_hold_us=recipe.final_hold_us)
    deadline = run.clock() + MAX_HOLD_BUDGET_S
    while not future.done():
        if _operator_stop(run):
            raise RuntimeError('operator stop before the excitation start was acknowledged')
        if run.clock() > deadline:
            raise RuntimeError('excitation start was not acknowledged inside one hold budget')
        _supervise(run, 'excitation start')
        run.sleep(SUPERVISE_PERIOD_S)
    reply = future.result()
    if reply.get('result') != 0 or not reply.get('available'):
        raise RuntimeError(f'excitation start rejected: {reply}')
    if reply.get('run_id') != recipe.run:
        raise RuntimeError(f'excitation run mismatch: {reply}')
    return reply


def _supervise_excitation(run):
    """每几毫秒推进监督器并异步查询状态; 保持期间不做任何阻塞等待."""
    deadline = run.clock() + run.plan.stream_duration_us / 1e6
    query, next_query = None, 0.0
    while True:
        if _operator_stop(run):
            raise RuntimeError('operator stop during the excitation window')
        _supervise(run, 'excitation')
        now = run.clock()
        if query is None and now >= next_query:
            query, next_query = run.peer.pitch_identification('query'), now + QUERY_PERIOD_S
        if query is not None and query.done():
            status = query.result()
            query = None
            if status.get('result') != 0:
                raise RuntimeError(f'excitation status request rejected: {status}')
            if status.get('run_id') not in (0, run.plan.recipe.run):
                raise RuntimeError(f'excitation run mismatch: {status}')
            if status.get('state_name') == 'fault':
                raise RuntimeError(f'excitation tripped firmware fault {status.get("fault_name")}')
            if status.get('state_name') == 'complete':
                return status
        if now > deadline:
            raise RuntimeError('excitation did not complete inside the bounded capture window')
        run.sleep(SUPERVISE_PERIOD_S)


def _post_complete_hold(run):
    """波形结束后移除扰动但保留保持, 直到显式停止或有界后置窗口结束; 不自动重跑."""
    deadline = run.clock() + run.plan.post_hold_s
    while run.clock() < deadline:
        if _operator_stop(run):
            return 'operator'
        _supervise(run, 'post-completion hold')
        run.sleep(SUPERVISE_PERIOD_S)
    return 'post_hold_timeout'


def _await_remote_handoff(run):
    """方波已正常结束, 保持原目标直到拨杆明确选中遥控; 不重置或回写增益."""
    deadline = run.clock() + run.plan.remote_handoff_s
    selected = False
    status_query = None
    while True:
        if _operator_stop(run):
            return 'operator'
        snapshot = run.peer.snapshot()
        page = snapshot.get('status', {}).get(0, {})
        domains = snapshot.get('status', {}).get(1, {})
        pitch = next((item for item in domains.get('domains', []) if item.get('domain') == 'pitch'), {})
        now = run.clock()
        remote_selected = (snapshot.get('connected') and not snapshot.get('error') and any(
            item.get('source') == 'remote' and item.get('authority') and not item.get('manual_stop')
            and 0 <= now - item.get('received_at_monotonic', -math.inf) <= 0.12
            and all(item.get(key) == snapshot.get(key) for key in ('session', 'config_epoch', 'reference_epoch'))
            for item in (page, domains)))
        if remote_selected:
            if not selected:
                # MCU 已选中遥控, 撤掉 PC 目标源; 不再以“维护权丢失”误报新遥控流程.
                run.guard.stop('remote takeover selected')
                selected = True
                deadline = min(deadline, now + 0.5)
            if (domains.get('source') == 'remote' and pitch.get('execute')
                    and 0 <= now - domains.get('received_at_monotonic', -math.inf) <= 0.12):
                return 'remote_handoff'
        if selected:
            if status_query is None or status_query.done():
                if status_query is not None:
                    status_query.result()
                status_query = run.peer.status(1)
        else:
            _supervise(run, 'waiting for remote handoff')
        if now >= deadline:
            raise TimeoutError('remote handoff was not confirmed before timeout')
        _notify(run, 'WAIT_REMOTE', remaining_s=deadline - now,
                angle_rad=run.guard.actual_pitch_rad, rate_rad_s=run.guard.actual_pitch_rate)
        run.sleep(SUPERVISE_PERIOD_S)


def _drain_capture_while_holding(run, report):
    _supervised_reply(run, run.peer.stop_pitch_stream(run.plan.recipe.run), 0.5, 'stop capture')
    deadline = run.clock() + 0.5
    while True:
        _supervise(run, 'capture drain before handoff')
        capture = run.peer.snapshot().get('pitch_stream', {})
        if capture.get('finished'):
            report['stream_final'] = capture
            run.peer.set_pitch_stream_sink(None)
            return
        if run.clock() >= deadline:
            raise TimeoutError('capture final marker missing before remote handoff')
        run.sleep(SUPERVISE_PERIOD_S)


def run_experiment(peer, plan, output_dir, *, stop_event=None, keys=None, clock=None, sleep=None,
                   writer_opener=None, reuse_session=False, capture_request=None, capture_wait_s=30.0,
                   progress=None, auto_run_id=False):
    """执行一次有界辨识; 返回结果报告. 任何失败都先撤销保持与授权再返回."""
    directory = Path(output_dir)
    run = _Run(peer=peer, plan=plan, stop_event=stop_event or threading.Event(), keys=keys,
               clock=clock or time.monotonic, sleep=sleep or time.sleep, progress=progress)
    report = {'error': None, 'cleanup_errors': [], 'holding': False, 'stream': {}, 'identification': None,
              'stop': None, 'revoked': False, 'files': {}, 'records': 0}
    metadata = {}
    stream_started = identification_started = identification_complete = False
    stream_drained = False
    authorization_attempted = False
    directory_owned = False
    try:
        if not math.isfinite(capture_wait_s) or capture_wait_s <= 0:
            raise ValueError('capture wait must be positive and finite')
        _notify(run, 'PREFLIGHT')
        hello = peer.snapshot() if reuse_session else _bounded(peer.hello(), 2.0, 'hello')
        if hello.get('identity') != 'sentinel_app' or not hello.get('startable'):
            raise RuntimeError(f'wrong/not-startable APP identity: {hello}')
        timing = _fresh_diagnostics(run)
        timing_budget = TimingBudget(timing['target_ttl_ms'], timing['link_ttl_ms'], 1 / 50)
        if timing_budget.budget_s > MAX_HOLD_BUDGET_S:
            raise RuntimeError(f'hold budget {timing_budget.budget_s}s exceeds {MAX_HOLD_BUDGET_S}s')
        if hello.get('config_kind') == 2:  # 实车角色: 必须先证明是配置好的 CAN2 Pitch 拓扑
            devices = _bounded(peer.diagnostics(1), 1.0, 'device topology')['devices']
            pitch = next(device for device in devices if device['domain'] == 'pitch')
            if any(pitch.get(key) != value for key, value in EXPECTED_PITCH_DEVICE.items()):
                raise RuntimeError(f'not the configured real CAN2 Pitch topology: {pitch}')
        parameters = _bounded(peer.read_all_params(), 3.0, 'gain snapshot')  # 只读快照, 绝不回写
        # 固件没有可信辨识策略时必须在授权/目标之前拒绝.
        available = _bounded(peer.pitch_identification('query'), 1.0, 'identification preflight')
        if auto_run_id:
            # 同一会话连续采集必须使用递增编号, 不用随机数碰运气绕过固件重放检查.
            run_id = available['run_id'] + 1
            plan = replace(plan, recipe=replace(plan.recipe, run=run_id))
            run.plan = plan
        if not available.get('available') and (capture_request is None or capture_request.is_set()):
            raise RuntimeError(f'firmware exposes no Pitch identification policy: {available}')
        page = _bounded(peer.status(0), 1.0, 'status page 0')
        center = page['joints']['pitch']['angle_rad']
        if not math.isfinite(center):
            raise RuntimeError('initial pitch position is not finite')
        travel_envelope, envelope = _envelopes(plan, center)
        limits = plan.limits
        extra_s = (capture_wait_s if capture_request is not None else 0.0) + plan.remote_handoff_s
        if plan.lift is not None:
            extra_s += plan.lift.timeout_s
            limits = replace(limits, max_command_speed_rad_s=plan.lift.speed_rad_s)
        limits = replace(limits, max_run_s=limits.max_run_s + extra_s)
        config = SupervisorConfig(plan.physical, travel_envelope, timing_budget, limits,
                                  shared_slow_lane=hello.get('config_kind') == 2)
        report['lift'] = {'initial_rad': center, 'settled': False} if plan.lift is not None else None
        metadata = {
            'layout_id': raw.LAYOUT, 'tool': 'sentinel_pitch_experiment', 'run': plan.recipe.run,
            'source': 'APP_MIT_SIMULATION_ONLY' if hello.get('config_kind') == 1 else 'APP_MIT_REAL_UNVALIDATED',
            'hardware_ready': False, 'deployable_gains': False,
            'start_us': 0,  # 首条记录到达后就地回填
            'recipe': asdict(plan.recipe), 'stream_duration_us': plan.stream_duration_us,
            'envelope_rad': [envelope.low, envelope.high],
            'travel_envelope_rad': [travel_envelope.low, travel_envelope.high],
            'initial_pitch_rad': center,
            'lift': asdict(plan.lift) if plan.lift is not None else None,
            'remote_handoff_s': plan.remote_handoff_s,
            'physical_rad': [plan.physical.low, plan.physical.high],
            'session': hello.get('session'), 'identity': hello.get('identity'),
            'boot_id': hello.get('boot_id'), 'config_kind': hello.get('config_kind'),
            'target_ttl_ms': timing['target_ttl_ms'], 'link_ttl_ms': timing['link_ttl_ms'],
            'parameters': parameters, 'warning': WARNING,
        }
        report['source'] = metadata['source']
        # 独占新目录: 绝不覆盖既有记录, 且被拒绝的配置不留下半成品目录.
        directory.mkdir(parents=True, exist_ok=False)
        directory_owned = True
        writer = SrawWriter(directory / 'pitch.sraw', metadata, opener=writer_opener)
        run.writer = writer
        peer.set_pitch_stream_sink(writer.append)
        if _operator_stop(run):
            raise RuntimeError('operator stop before authorization')
        authorization_attempted = True
        _checked(_bounded(peer.authorize(True, 'maintenance'), 1.0, 'authorize'), 'authorize')
        _bounded(peer.status(0), 1.0, 'status page 0')
        _bounded(peer.status(1), 1.0, 'status page 1')
        run.guard = PitchHoldSupervisor(peer, config, clock_fn=run.clock)
        run.guard.start(center)
        _notify(run, 'HOLDING')
        _await_holding(run)
        report['holding'] = True
        if plan.lift is not None:
            _await_lift(run, report['lift'])
            run.guard.narrow_envelope(envelope)
        if capture_request is not None:
            _notify(run, 'READY', angle_rad=run.guard.actual_pitch_rad,
                    rate_rad_s=run.guard.actual_pitch_rate)
            _wait_capture_request(run, capture_request, capture_wait_s)
            available = _supervised_reply(run, peer.pitch_identification('query'), 1.0,
                                          'identification preflight')
            if not available.get('available'):
                raise RuntimeError('firmware exposes no Pitch identification policy; capture not started')
        if _operator_stop(run):
            raise RuntimeError('operator stop before capture')
        _notify(run, 'CAPTURING')
        stream_future = peer.start_pitch_stream(plan.recipe.run, plan.stream_duration_us)
        stream_started = True
        report['stream'] = _await_stream(run, stream_future)
        identification_started = True
        report['identification'] = _start_excitation(run)
        report['identification'] = _supervise_excitation(run)
        identification_complete = True
        report['stop'] = _post_complete_hold(run)
        if plan.remote_handoff_s and report['stop'] != 'operator':
            _drain_capture_while_holding(run, report)
            stream_drained = True
            report['stop'] = _await_remote_handoff(run)
    except BaseException as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        _notify(run, 'STOPPING')
        # Transition out of holding BEFORE any potentially blocking cleanup ACK.
        # This is an explicit torque release; physical anti-drop support is mandatory.
        if run.guard is not None:
            try:
                run.guard.stop()
            except Exception as error:
                report['cleanup_errors'].append(f'guard stop: {error}')
        if identification_started and not identification_complete:
            try:
                _bounded(peer.pitch_identification('cancel'), 0.5, 'cancel excitation')
            except Exception as error:
                report['cleanup_errors'].append(f'cancel: {error}')
        if stream_started and not stream_drained:
            try:
                _bounded(peer.stop_pitch_stream(plan.recipe.run), 0.5, 'stop capture')
                drain_deadline = run.clock() + 0.5
                while True:
                    capture = peer.snapshot().get('pitch_stream', {})
                    if capture.get('fault') is not None:
                        raise RuntimeError(f'capture failed while draining: {capture["fault"]}')
                    if capture.get('finished'):
                        report['stream_final'] = capture
                        break
                    if run.clock() >= drain_deadline:
                        raise TimeoutError('capture final marker missing; sample completeness unverified')
                    run.sleep(SUPERVISE_PERIOD_S)
            except Exception as error:
                report['cleanup_errors'].append(f'stop capture: {error}')
            try:
                peer.set_pitch_stream_sink(None)
            except Exception as error:
                report['cleanup_errors'].append(f'clear sink: {error}')
        if authorization_attempted:
            try:
                _checked(_bounded(peer.authorize(False, 'maintenance'), 1.0, 'revoke'), 'revoke')
                report['revoked'] = True
            except Exception as error:
                report['cleanup_errors'].append(f'revoke: {error}')
        if run.writer is not None:
            try:
                run.writer.close()
            except Exception as error:
                report['cleanup_errors'].append(f'writer: {error}')
            # 关闭(排空)之后才是最终条数; 线程未结束时文件仍可能被追加.
            report['records'] = run.writer.written
        if directory_owned:
            sealed = run.writer is None or run.writer.finished
            report['files'] = _finalise(directory, plan, report, sealed=sealed)
    if report['error'] is None and report['cleanup_errors']:
        report['error'] = '; '.join(report['cleanup_errors'])
    return report


def _write_json(path, payload):
    """同目录临时文件 + 原子替换: 中途失败不会留下半截 JSON."""
    text = json.dumps(payload, ensure_ascii=False, allow_nan=False, indent=1) + '\n'
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(text, encoding='utf-8')
    os.replace(temporary, path)


def _finalise(directory, plan, report, *, sealed=True):
    """落盘 manifest/summary/CSV; 不完整数据保留, 失败显式记录而不是静默.

    sealed=False 表示后台写线程在超时内没有结束: 文件仍可能被追加, 此时不导出、
    不解析, 也不就地回填头部, 避免把写一半的文件当成完整记录.
    """
    # manifest 内嵌 files 自身, 两个自引用路径必须在序列化前就确定;
    # 否则 04 那次 manifest 里 files.manifest/files.summary 是空字符串.
    files = {'sraw': str(directory / 'pitch.sraw'), 'csv': '',
             'manifest': str(directory / 'manifest.json'),
             'summary': str(directory / 'summary.json')}
    errors = report['cleanup_errors']
    if sealed:
        try:
            count, identity = raw.export_csv(directory / 'pitch.sraw', directory / 'pitch.csv')
            if identity != plan.recipe.run:
                errors.append(f'csv run mismatch: {identity} != {plan.recipe.run}')
            report['records'] = max(report['records'], count)
            files['csv'] = str(directory / 'pitch.csv')
        except Exception as error:
            errors.append(f'csv export: {error}')
    else:
        errors.append('sraw writer still running: CSV 导出与解析已跳过, 记录视为不完整')
    rows = []
    if sealed and not errors:
        try:
            _, rows = raw.load(directory / 'pitch.sraw')
        except Exception as error:
            errors.append(f'quality validation: {error}')
    status = (report.get('stream_final') or {}).get('last_status') or {}
    gaps = sum(row['gap'] for row in rows)
    lost_max = max((row['lost'] for row in rows), default=0)
    integrity = []
    if sealed and not errors:
        if not rows:
            integrity.append('empty capture: 没有采集记录')
        if not status:
            integrity.append('缺少 MCU 计数 (last_status)')
        else:
            generated = status.get('generated')
            if len(rows) != generated:
                integrity.append(f'解码 {len(rows)} 条 != MCU generated {generated}')
            if status.get('dropped'):
                integrity.append(f'MCU dropped {status["dropped"]}')
            if status.get('buffered'):
                integrity.append(f'MCU buffered {status["buffered"]}')
    if gaps:
        integrity.append(f'序号缺口 {gaps}')
    if lost_max:
        integrity.append(f'最大丢样 {lost_max}')
    if integrity:
        errors.append('capture integrity: ' + '; '.join(integrity))
    # 清理/完整性失败的说明必须出现在落盘报告里, 不能只在返回对象上补.
    if report['error'] is None and errors:
        report['error'] = '; '.join(errors)
    quality = {'records': len(rows), 'sequence_gaps': gaps, 'lost_max': lost_max,
               'span_s': rows[-1]['t_s'] - rows[0]['t_s'] if rows else 0.0,
               'final_marker': bool(report.get('stream_final', {}).get('finished')),
               'complete': (bool(rows) and not errors and report.get('error') is None
                            and report.get('identification', {}).get('state_name') == 'complete'
                            and bool(report.get('stream_final', {}).get('finished')))}
    report['quality'] = quality
    manifest = {'tool': 'sentinel_pitch_experiment', 'run': plan.recipe.run,
                'source': report.get('source', 'UNKNOWN'),
                'hardware_ready': False, 'deployable_gains': False, 'quality': quality,
                'recipe': asdict(plan.recipe), 'stream_duration_us': plan.stream_duration_us,
                'physical_rad': [plan.physical.low, plan.physical.high],
                'excursion_rad': plan.excursion_rad, 'files': files,
                'stream': report.get('stream', {}), 'identification': report.get('identification')}
    manifest['lift'] = report.get('lift')
    summary = {key: value for key, value in report.items() if key not in ('stream', 'files')}
    before = len(errors)
    summary_published = False
    # summary 先落盘, manifest 最后: 后者序列化时看到的已是更新后的 files/quality.
    for name, payload in (('summary.json', summary), ('manifest.json', manifest)):
        try:
            _write_json(directory / name, payload)
            if name == 'summary.json':
                summary_published = True
        except Exception as error:
            errors.append(f'{name}: {error}')
            files[name.split('.')[0]] = ''
            # 立即降级, 让下一份报告与本份回写都看到失败, 而不是事后再纠正.
            report['error'] = report['error'] or '; '.join(errors)
            summary['error'] = report['error']
            quality['complete'] = False
            quality['incomplete_reason'] = list(errors[before:])
    if len(errors) > before and summary_published:
        # 先撤销本轮发布的旧摘要, 回写继续失败时也不能留下 complete=true.
        # 只清理本轮生成的报告; 原始记录与 CSV 保留供排障.
        try:
            (directory / 'summary.json').unlink()
            files['summary'] = ''
            _write_json(directory / 'summary.json', summary)
            files['summary'] = str(directory / 'summary.json')
        except Exception as error:
            errors.append(f'summary.json rewrite: {error}')
    return files


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    link = p.add_mutually_exclusive_group(required=True)
    link.add_argument('--backend', type=Path,
                      help='software fixture; injected as --plant --fixture gimbal --pitch-identification')
    link.add_argument('--serial', type=Path)
    p.add_argument('--output', type=Path, required=True, help='new output directory; existing paths are refused')
    p.add_argument('--arm', action='store_true', help='explicitly authorize this runner to send Pitch targets')
    p.add_argument('--support-ready', action='store_true')
    p.add_argument('--accept-release', action='store_true', help=WARNING)
    p.add_argument('--run', type=float, help='capture/excitation run id; default random non-zero')
    for name in ('physical-low', 'physical-high', 'excursion', 'max-temperature', 'max-torque',
                 'max-speed', 'max-error', 'amplitude-nm', 'half-period-us', 'baseline-us',
                 'final-hold-us', 'stream-duration-us', 'hold-timeout', 'post-hold'):
        p.add_argument('--' + name, type=float)
    p.add_argument('--cycles', type=int)
    p.add_argument('--stream-packets', type=int, default=2)
    p.add_argument('--lift-to-rad', type=float, help='optional calibrated joint target; horizontal is usually 0')
    p.add_argument('--lift-speed', type=float, help='commanded lift speed, rad/s')
    p.add_argument('--lift-timeout', type=float, help='travel and settling deadline, seconds')
    p.add_argument('--settle-seconds', type=float, help='continuous settling duration')
    p.add_argument('--settle-error', type=float, help='position tolerance, rad')
    p.add_argument('--settle-speed', type=float, help='measured settling speed tolerance, rad/s')
    return p


REQUIRED_BOUNDS = ('physical-low', 'physical-high', 'excursion', 'max-temperature', 'max-torque',
                   'max-speed', 'max-error', 'amplitude-nm', 'half-period-us', 'baseline-us', 'cycles',
                   'final-hold-us')


def build_plan(args, p) -> ExperimentPlan:
    lift = None
    lift_fields = ('lift_speed', 'lift_timeout', 'settle_seconds', 'settle_error', 'settle_speed')
    if args.lift_to_rad is not None:
        if any(getattr(args, name) is None for name in lift_fields):
            p.error('lift requires --lift-speed --lift-timeout --settle-seconds --settle-error --settle-speed')
        try:
            lift = LiftPlan(args.lift_to_rad, args.lift_speed, args.lift_timeout, args.settle_seconds,
                            args.settle_error, args.settle_speed)
        except ValueError as error:
            p.error(str(error))
    elif any(getattr(args, name) is not None for name in lift_fields):
        p.error('lift settings require --lift-to-rad')
    for name in REQUIRED_BOUNDS + ('stream-duration-us', 'hold-timeout', 'post-hold'):
        if args.serial is not None and getattr(args, name.replace('-', '_')) is None:
            p.error(f'serial requires explicit --{name}')
    for name, default in (('hold-timeout', 1.0), ('post-hold', 0.5)):  # 仅虚拟时间轴允许缺省
        if getattr(args, name.replace('-', '_')) is None:
            setattr(args, name.replace('-', '_'), default)
    for name in REQUIRED_BOUNDS:
        value = getattr(args, name.replace('-', '_'))
        if value is not None and not math.isfinite(value):
            p.error(f'{name} must be finite')
    if args.run is not None and (not math.isfinite(args.run) or args.run != int(args.run)):
        p.error('run must be a finite integer')
    run = secrets.randbelow(0xfffffffe) + 1 if args.run is None else int(args.run)
    try:
        recipe = ExperimentRecipe(run=run, amplitude_nm=args.amplitude_nm,
                                  half_period_us=int(args.half_period_us), baseline_us=int(args.baseline_us),
                                  cycles=args.cycles, final_hold_us=int(args.final_hold_us))
        duration_us = (int(args.stream_duration_us) if args.stream_duration_us is not None
                       else int(recipe.excitation_us * 1.5 + 1e6))
        limits = SupervisorLimits(max_temperature_c=args.max_temperature, max_effort_nm=args.max_torque,
                                  max_speed_rad_s=args.max_speed, max_tracking_error_rad=args.max_error,
                                  max_run_s=duration_us / 1e6 + plan_margin_s(duration_us),
                                  # 实车状态页 1 约 20 Hz 产出; 按 25 Hz 轮询会让每个请求等满一个
                                  # 出页周期并堆积, 实测 40ms 间隔回复要 51ms, 60ms 只需 10ms.
                                  status_query_interval_s=0.06,
                                  # 实车状态页 1 实测 50ms 产生一页: 120ms 只容一次漏页, 实测会卡在
                                  # 123ms. 0.18s 仍低于固件 200ms 链路 TTL, 超时依旧锁存 fault.
                                  max_telemetry_age_s=0.18)
        return ExperimentPlan(recipe=recipe, stream_duration_us=duration_us,
                              physical=PhysicalBounds(args.physical_low, args.physical_high),
                              excursion_rad=args.excursion, limits=limits,
                              stream_packet_min=args.stream_packets,
                               hold_timeout_s=args.hold_timeout, post_hold_s=args.post_hold, lift=lift)
    except (ValueError, TypeError) as error:
        p.error(str(error))
        raise


def plan_margin_s(duration_us):
    return max(1.0, duration_us / 1e6)


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not args.arm:
        p.error('--arm is required; without it no connection is opened')
    if args.serial and (not args.support_ready or not args.accept_release):
        p.error('serial requires --support-ready --accept-release')
    if args.backend and (args.support_ready or args.accept_release):
        p.error('--support-ready/--accept-release are hardware acknowledgements, not software-plant flags')
    plan = build_plan(args, p)
    transport = (wire.ProcessTransport(args.backend,
                                      extra_args=['--plant', '--fixture', 'gimbal', '--pitch-identification'])
                 if args.backend else ExclusiveSerial(args.serial))
    peer = AsyncSentinelClient(transport, target_hz=200 if args.backend else 50, fast_hz=50)
    stop_event = threading.Event()
    previous = {}
    report = {'error': 'aborted before the experiment started', 'cleanup_errors': []}
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, lambda *_: stop_event.set())
        print(WARNING, flush=True)
        print(f'Run {plan.recipe.run}: excitation {plan.recipe.excitation_us} us, '
              f'stream {plan.stream_duration_us} us, no gain writes.', flush=True)
        # A headless SOFTWARE fixture has no keyboard; stdin EOF is not an
        # operator emergency stop. Physical serial sessions retain Keys/EOF stop.
        with (nullcontext(None) if args.backend else Keys()) as keys:
            report = run_experiment(peer, plan, args.output, stop_event=stop_event, keys=keys)
    except BaseException as error:
        report['error'] = report.get('error') or f'{type(error).__name__}: {error}'
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        try:
            peer.close()
        except Exception as error:
            report['cleanup_errors'].append(f'link close: {error}')
            report['error'] = report.get('error') or str(error)
    print(json.dumps({'error': report.get('error'), 'cleanup_errors': report.get('cleanup_errors'),
                      'records': report.get('records'), 'files': report.get('files')}, ensure_ascii=False),
          flush=True)
    return 0 if report.get('error') is None and not report.get('cleanup_errors') else 1


if __name__ == '__main__':
    raise SystemExit(main())
