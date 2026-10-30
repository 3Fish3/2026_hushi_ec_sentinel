"""业务协议 v3 异步入口: 一个 UART 拥有者, 最新目标邮箱, 一个在途变更.

publish_targets 的每次调用代表新的意图; 尚未发送的旧值被覆盖, 在主机滞留的时间计入源年龄.
固定目标/轨迹可用 set_target_source 提供纯计算生成器 (不做阻塞 I/O); UI 绘制不驱动发送时钟.
生成器必须明确处理参考变化, 不把旧位置数值重新贴上新参考代次. 变更不明只读回确认,
不能用新序号自动重做 align. 原生管道仅验证协议/编排, 不模拟真实 UART 延迟或 MCU 时钟.
"""
from __future__ import annotations

import importlib
import json
import math
import queue
import secrets
import struct
import threading
import time
from collections import OrderedDict, deque
from concurrent.futures import Future
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import sentinel_client as wire

# 单 UART 连续流解码器由 pitch stream 模块拥有; 这里只做分发, 不复制布局解析.
# 缺失属于部署依赖, 由显式采集故障暴露, 不影响既有 v3 路径加载.
def _default_pitch_decoder() -> Any:
    try:
        decoder: Any = importlib.import_module('sentinel_pitch_stream').decode_pitch_stream_packet
    except (ImportError, AttributeError):
        return None
    return decoder if callable(decoder) else None

# 原始连续流请求/应答沿用 sentinel_raw 的外层型式: '<4B2I' 请求, 0xB2 应答, 0xB3 数据.
PITCH_STREAM = 0x32
PITCH_STREAM_ACK = 0xB2
PITCH_STREAM_DATA = 0xB3
PITCH_STREAM_PROFILE = 3
PITCH_STREAM_DOMAIN = 6
PITCH_STREAM_OPS = {'start': 1, 'stop': 2}
PITCH_STREAM_DATA_OP = 3


class OutcomeUnknown(wire.ProtocolError):
    """请求可能已执行, 但当前无法确认; 保留身份供记录, 不宣称成功或未执行."""
    def __init__(self, operation, session, sequence, detail):
        self.operation, self.session, self.sequence = operation, session, sequence
        super().__init__(f'结果未知/已失效: op=0x{operation:02x} session={session} seq={sequence}; {detail}; 请读回后决定新请求')


class _WireLog:
    """字节日志单独写盘; 队列满只丢日志并记间隙, 不阻塞目标流."""
    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.file = Path(path).open('a', encoding='utf-8')
        self.queue = queue.Queue(maxsize=256)
        self.stop = threading.Event()
        self.ordinal = self.dropped = 0
        self.error = ''
        self.thread = threading.Thread(target=self._run, daemon=True, name='sentinel-log')
        self.thread.start()

    def record(self, direction, data):
        self.ordinal += 1
        record = {'ordinal': self.ordinal, 'monotonic_s': time.monotonic(), 'direction': direction,
                  'wire_hex': data.hex()}
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        last = 0
        try:
            self.file.write(json.dumps({'event': 'begin', 'protocol': wire.PROTOCOL_VERSION,
                                        'monotonic_s': time.monotonic()})+ '\n')
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    record = self.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                if record['ordinal'] != last+1:
                    self.file.write(json.dumps({'event': 'gap', 'count': record['ordinal']-last-1})+ '\n')
                self.file.write(json.dumps(record)+'\n')
                last = record['ordinal']
            self.file.write(json.dumps({'event': 'end', 'total': self.ordinal, 'dropped': self.dropped})+ '\n')
        except OSError as error:
            self.error = str(error)
        finally:
            try:
                self.file.close()
            except OSError as error:
                self.error = str(error)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            self.error = '日志写入未在关闭窗口内完成, 文件可能不完整'


@dataclass
class _Request:
    operation: int
    values: dict
    future: Future | None
    session: int
    mutation: bool = False


@dataclass
class _Pending:
    request: _Request
    sequence: int
    frame_sequence: int
    message: bytes
    deadline: float
    attempts: int = 1


@dataclass(frozen=True)
class CaptureFault:
    """不可变采集故障: 一旦记录就保持到下一次显式 start 的新采集身份."""
    code: str
    detail: str
    run: int
    at_monotonic: float


@dataclass
class _StreamRequest:
    op: int
    run: int
    duration_us: int
    future: Future


@dataclass
class _StreamPending:
    request: _StreamRequest
    frame_sequence: int
    message: bytes
    deadline: float


class AsyncSentinelClient:
    def __init__(self, transport, *, target_hz=100.0, fast_hz=100.0,
                 timeout_s=0.15, attempts=3, log_path=None, pitch_decoder=None):
        if not (math.isfinite(target_hz) and target_hz > 0 and math.isfinite(fast_hz) and fast_hz >= 0
                and math.isfinite(timeout_s) and timeout_s > 0 and attempts >= 1):
            raise ValueError('invalid link cadence/timeout')
        self.transport = transport
        self.target_period = 1.0/target_hz
        self.fast_period = 1.0/fast_hz if fast_hz else math.inf
        self.timeout_s, self.attempts = timeout_s, attempts
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._session = self._sequence = self._frame = 0
        self._connected = False
        self._change = None
        self._change_active = False
        self._reads = deque()
        self._read_active = False
        self._pending = OrderedDict()
        self._goal = self._permit = None
        self._source = None
        self._source_generation = 0
        # 连续流身份在首个字节之前绑定, 数据可能先于 start ACK 到达.
        self._pitch_decoder = _default_pitch_decoder() if pitch_decoder is None else pitch_decoder
        self._pitch_requests: deque = deque()
        self._pitch_pending: dict = {}
        self._pitch_sink = None
        self._pitch: dict[str, Any] = {
            'run': None, 'op': 0, 'duration_us': 0, 'active': False, 'session': 0,
            'confirmed_by': None, 'packets': 0, 'records': 0, 'last_status': {},
            'last_packet_monotonic': None, 'started_at_monotonic': None,
            'stopped_at_monotonic': None, 'last_ack': None, 'fault': None,
            'last_sequence': -1, 'last_generated': 0, 'draining': False, 'finished': False}
        self._stats = {'sent': 0, 'replies': 0, 'unmatched': 0, 'invalid_replies': 0, 'retries': 0,
                       'superseded': 0, 'stream_timeouts': 0, 'dropped_epochs': 0}
        self._state: dict[str, Any] = {
            'connected': False, 'session': 0, 'config_epoch': 0, 'reference_epoch': 0,
            'status': {}, 'diagnostics': {}, 'error': '', 'last_change': None, 'last_target_ack': None,
            'target_rejections': 0, 'last_target_rejection': None, 'last_target_sent_sequence': 0}
        self._page_sequences = {}
        try:
            self._log = _WireLog(log_path) if log_path else None
        except OSError:
            self.transport.close()
            raise
        if self._log:
            self.transport.observer = self._log.record
        self._thread = threading.Thread(target=self._run, daemon=True, name='sentinel-uart')
        self._thread.start()

    def snapshot(self):
        with self._lock:
            result = deepcopy(self._state)
            result['statistics'] = dict(self._stats)
            result['change_pending'] = self._change_active
            result['pitch_stream'] = deepcopy(self._pitch)
            result['pitch_stream_error'] = self._pitch_error_locked()
        if self._log:
            result.update(log_dropped=self._log.dropped, log_error=self._log.error)
        return result

    def _post(self, operation, values, mutation=False):
        future = Future()
        with self._lock:
            if self._stop.is_set():
                raise wire.ProtocolError('link owner closed')
            if operation != wire.HELLO and not self._connected:
                raise wire.ProtocolError('hello required')
            request = _Request(operation, values, future, self._session, mutation)
            if mutation or operation == wire.HELLO:
                if self._change_active:
                    raise wire.ProtocolError('one change is already pending')
                self._change, self._change_active = request, True
            else:
                if len(self._reads) >= 8:
                    raise wire.ProtocolError('read request queue full')
                self._reads.append(request)
        self._wake.set()
        return future

    def _pitch_error_locked(self):
        fault = self._pitch['fault']
        return '' if fault is None else f'{fault.code}: {fault.detail}'

    def hello(self, nonce=None):
        return self._post(wire.HELLO, {'nonce': secrets.randbits(32) if nonce is None else nonce})

    def authorize(self, grant=True, source='maintenance'):
        return self._post(wire.AUTHORIZE, {'grant': grant, 'source': source}, True)

    def align(self, domain, value, *, kind='joint_reference', reference_epoch=None):
        if reference_epoch is None:
            reference_epoch = self.snapshot()['reference_epoch']
        return self._post(wire.ALIGN, {'domain': domain, 'value': value, 'kind': kind,
                                       'reference_epoch': reference_epoch}, True)

    def param_write(self, param_id, value):
        return self._post(wire.PARAM_WRITE, {'param_id': param_id, 'value': value}, True)

    def param_read(self, param_id=0xffff, offset=0):
        return self._post(wire.PARAM_READ, {'param_id': param_id, 'offset': offset})

    def status(self, page=0):
        return self._post(wire.STATUS, {'page': page})

    def diagnostics(self, page=0):
        return self._post(wire.DIAGNOSTICS, {'page': page})

    def pitch_identification(self, op, *, run_id=0, amplitude_nm=0.0, half_period_us=0,
                             baseline_us=0, cycles=0, final_hold_us=0,
                             config_epoch=None, reference_epoch=None):
        """复用既有 v3 编解码; query 是只读请求, start/cancel/reset 是变更.

        不发送任意电流、不写增益、不改 TTL; 代次缺省取当前会话快照.
        """
        op_code = wire.PITCH_IDENT_OPS.index(op) if isinstance(op, str) else int(op)
        snap = self.snapshot()
        values = {'op': op_code,
                  'config_epoch': snap['config_epoch'] if config_epoch is None else config_epoch,
                  'reference_epoch': snap['reference_epoch'] if reference_epoch is None else reference_epoch,
                  'run_id': run_id, 'amplitude_nm': amplitude_nm, 'half_period_us': half_period_us,
                  'baseline_us': baseline_us, 'cycles': cycles, 'final_hold_us': final_hold_us}
        return self._post(wire.PITCH_IDENTIFICATION, values, op_code != 0)

    # ------------------------------------------------------------------
    # 单 UART 连续流: 原始 0x32 请求 + 0xB2 应答 + 0xB3 非请求数据.
    # ------------------------------------------------------------------
    def set_pitch_stream_sink(self, callback):
        """安装/清除连续流汇; 回调只在 UART 线程被调用, 必须只做入队."""
        with self._lock:
            self._pitch_sink = callback

    def pitch_stream_status(self):
        """返回已完成的只读快照 Future; 驱动轮询不会阻塞持续保持."""
        future = Future()
        with self._lock:
            future.set_result(deepcopy(dict(self._pitch, error=self._pitch_error_locked())))
        return future

    def start_pitch_stream(self, run, duration_us):
        if type(run) is not int or not 0 < run <= 0xffffffff:
            raise wire.ProtocolError(f'invalid capture run: {run!r}')
        if type(duration_us) is not int or not 0 < duration_us < 0x80000000:
            raise wire.ProtocolError(f'invalid capture duration: {duration_us!r}')
        future = Future()
        with self._lock:
            if self._stop.is_set():
                raise wire.ProtocolError('link owner closed')
            if not self._connected:
                raise wire.ProtocolError('hello required')
            if self._pitch['fault'] is not None:
                raise wire.ProtocolError('capture fault latched; explicit new handshake required')
            previous_run = self._pitch['run']
            final = self._pitch['last_status']
            reusable = (previous_run is not None and self._pitch['finished'] and run > previous_run
                        and final.get('dropped') == 0 and final.get('buffered') == 0)
            if ((previous_run is not None and not reusable) or self._pitch['active']
                    or self._pitch_requests or self._pitch_pending):
                raise wire.ProtocolError('pitch capture identity already bound')
            self._pitch.update(run=run, op=PITCH_STREAM_OPS['start'], duration_us=duration_us,
                               active=True, session=self._session, confirmed_by=None, packets=0,
                               records=0, last_status={}, last_packet_monotonic=None,
                               started_at_monotonic=time.monotonic(), stopped_at_monotonic=None,
                               last_ack=None, fault=None, last_sequence=-1,
                               last_generated=0, draining=False, finished=False)
            self._pitch_requests.append(_StreamRequest(PITCH_STREAM_OPS['start'], run, duration_us, future))
        self._wake.set()
        return future

    def stop_pitch_stream(self, run):
        if type(run) is not int or not 0 < run <= 0xffffffff:
            raise wire.ProtocolError(f'invalid capture run: {run!r}')
        future = Future()
        with self._lock:
            if self._stop.is_set():
                raise wire.ProtocolError('link owner closed')
            if self._pitch['run'] != run:
                raise wire.ProtocolError(f'capture run mismatch: {self._pitch["run"]} != {run}')
            if any(request.op == PITCH_STREAM_OPS['stop'] for request in self._pitch_requests):
                raise wire.ProtocolError('capture stop already pending')
            self._pitch_requests.append(_StreamRequest(PITCH_STREAM_OPS['stop'], run, 0, future))
        self._wake.set()
        return future

    def signal_pitch_capture_fault(self, code, detail):
        """记录汇(队列/磁盘)侧故障; 由持有 writer 的一方调用, UART 线程不被阻塞."""
        return self._capture_fault(str(code), str(detail))

    def _capture_fault(self, code, detail):
        now = time.monotonic()
        with self._lock:
            if self._pitch['fault'] is not None:
                return False
            run = self._pitch['run']
            fault = CaptureFault(code, detail, int(run or 0), now)
            self._pitch.update(fault=fault, active=False, run=None, confirmed_by=None,
                               stopped_at_monotonic=now)
            pendings = list(self._pitch_pending.values())
            self._pitch_pending.clear()
            requests = list(self._pitch_requests)
            self._pitch_requests.clear()
            # 故障后不再续期任何目标/许可: 保持与授权都不得自动恢复.
            self._goal = self._permit = self._source = None
            self._source_generation += 1
            self._state['error'] = f'pitch capture fault: {code}: {detail}'
        error = wire.ProtocolError(f'pitch capture fault: {code}: {detail}')
        for pending in pendings:
            self._settle_stream_future(pending.request, error=error)
        for request in requests:
            self._settle_stream_future(request, error=error)
        return True

    @staticmethod
    def _settle_stream_future(request, value=None, error=None):
        if request.future.done():
            return
        if error is not None:
            request.future.set_exception(error)
        else:
            request.future.set_result(value)

    def _begin_stream(self, request, now):
        profile = PITCH_STREAM_PROFILE if request.op == PITCH_STREAM_OPS['start'] else 0
        domain = PITCH_STREAM_DOMAIN if request.op == PITCH_STREAM_OPS['start'] else 0
        try:
            payload = struct.pack('<4B2I', 1, request.op, profile, domain, request.run, request.duration_us)
            self._frame = (self._frame+1) & 0xffff
            while self._frame in self._pending or self._frame in self._pitch_pending:
                self._frame = (self._frame+1) & 0xffff
            message = wire.encode_frame(PITCH_STREAM, self._frame, payload)
        except (wire.ProtocolError, ValueError, TypeError, OverflowError, struct.error) as error:
            self._settle_stream_future(request, error=error)
            self._capture_fault('encode', str(error))
            return
        self._pitch_pending[self._frame] = _StreamPending(request, self._frame, message, now+self.timeout_s)
        self.transport.send(message)
        self._stats['sent'] += 1

    def _expire_stream(self, now):
        with self._lock:
            active = self._pitch['active'] and self._pitch['confirmed_by'] is not None
            last_packet = self._pitch['last_packet_monotonic'] or self._pitch['started_at_monotonic']
        if active and last_packet is not None and now - last_packet > 0.05:
            self._capture_fault('stream-stale', 'no capture packet within the 50ms holding budget')
            return
        for pending in tuple(self._pitch_pending.values()):
            if now < pending.deadline:
                continue
            del self._pitch_pending[pending.frame_sequence]
            request = pending.request
            if request.op == PITCH_STREAM_OPS['start']:
                # 未确认的采集不得继续被当作可信身份.
                self._settle_stream_future(request, error=TimeoutError('capture start not acknowledged'))
                self._capture_fault('start-timeout', f'run {request.run} not acknowledged')
            else:
                self._settle_stream_future(request, error=TimeoutError('capture stop not acknowledged'))
                with self._lock:
                    self._pitch.update(active=False, run=None, confirmed_by=None,
                                       stopped_at_monotonic=time.monotonic())
                self._stats['stream_timeouts'] += 1

    def _pitch_ack(self, echo, body):
        pending = self._pitch_pending.pop(echo, None)
        if pending is None:
            self._stats['unmatched'] += 1
            return
        request = pending.request
        decoded = self._decode_stream(body, request.run, 'ack')
        if decoded is None:
            self._settle_stream_future(request, error=wire.ProtocolError('pitch capture ACK unusable'))
            return
        status, _ = decoded
        if status.get('op') != request.op or status.get('run') != request.run:
            self._settle_stream_future(request, error=wire.ProtocolError(
                f'capture ACK identity mismatch: {status.get("op")}/{status.get("run")}'))
            self._capture_fault('stale-ack',
                f'op={status.get("op")} run={status.get("run")} expected op={request.op} run={request.run}')
            return
        with self._lock:
            self._pitch['last_ack'] = dict(status)
            if self._pitch['confirmed_by'] is None:
                self._pitch['confirmed_by'] = 'ack'
        if request.op == PITCH_STREAM_OPS['stop']:
            with self._lock:
                # STOP ACK may precede queued data and final marker. Keep identity
                # and sink alive until the explicit state3 packet is validated.
                self._pitch.update(draining=True, stopped_at_monotonic=time.monotonic())
        self._settle_stream_future(request, dict(status, confirmed_by='ack'))

    def _decode_stream(self, body, run, stage):
        decoder: Any = self._pitch_decoder
        if decoder is None:
            self._capture_fault('decoder-unavailable', 'sentinel_pitch_stream.decode_pitch_stream_packet missing')
            return None
        try:
            status, records = decoder(bytes(body), expected_run=run)
        except Exception as error:  # 解码器把截断/身份/拒绝都表达为异常.
            self._capture_fault(f'{stage}-rejected', f'{type(error).__name__}: {error}')
            return None
        if not isinstance(status, dict) or status.get('error'):
            self._capture_fault(f'{stage}-rejected', str(status))
            return None
        return status, list(records or ())

    def _pitch_data(self, body):
        with self._lock:
            run, active = self._pitch['run'], self._pitch['active']
            faulted, sink = self._pitch['fault'] is not None, self._pitch_sink
        if faulted or not active or run is None:
            self._stats['unmatched'] += 1
            return
        decoded = self._decode_stream(body, run, 'data')
        if decoded is None:
            return
        status, records = decoded
        if status.get('op') != PITCH_STREAM_DATA_OP or status.get('run') != run:
            self._capture_fault('foreign-run',
                f'op={status.get("op")} run={status.get("run")} expected run={run}')
            return
        with self._lock:
            expected = self._pitch['last_sequence'] + 1
            generated_before = self._pitch['last_generated']
        generated = status.get('generated')
        if status.get('dropped', 0) or (generated is not None and generated < generated_before):
            self._capture_fault('stream-loss', 'recorder dropped samples or generation counter regressed')
            return
        for record in records:
            sequence = struct.unpack_from('<I', record, 4)[0]
            if sequence != expected:
                self._capture_fault('sequence-gap', f'expected record {expected}, got {sequence}')
                return
            expected += 1
        if status.get('state') == 3 and generated is not None and expected != generated:
            self._capture_fault('incomplete-final', f'collected {expected}, recorder generated {generated}')
            return
        if sink is not None:
            try:
                sink(status, records)
            except Exception as error:  # 汇队列满/已故障: 立即可见, 不阻塞 UART 线程.
                self._capture_fault('sink', f'{type(error).__name__}: {error}')
                return
        with self._lock:
            self._pitch['packets'] += 1
            self._pitch['records'] += len(records)
            self._pitch['last_status'] = dict(status)
            self._pitch['last_sequence'] = expected - 1
            if generated is not None:
                self._pitch['last_generated'] = generated
            if status.get('state') == 3:
                self._pitch.update(finished=True, active=False)
            self._pitch['last_packet_monotonic'] = time.monotonic()
            first = self._pitch['confirmed_by'] is None
            if first:
                self._pitch['confirmed_by'] = 'data'
            starters = [pending for pending in self._pitch_pending.values()
                        if pending.request.op == PITCH_STREAM_OPS['start'] and pending.request.run == run]
        if first:
            for pending in starters:
                self._settle_stream_future(pending.request,
                    dict(status, confirmed_by='data', run=run, session=self._session))

    def read_all_params(self):
        result, entries = Future(), []

        def consume(future):
            if result.cancelled():
                return
            try:
                page = future.result()
                if 'entries' not in page or (page['count'] == 0 and page['total'] > page['offset']):
                    raise wire.ProtocolError(f'parameter read made no progress: {page}')
                entries.extend(page['entries'])
                offset = page['offset'] + page['count']
                if offset >= page['total']:
                    result.set_result(entries)
                else:
                    self.param_read(offset=offset).add_done_callback(consume)
            except Exception as error:
                result.set_exception(error)
        self.param_read().add_done_callback(consume)
        return result

    def _goal_at(self, values, produced_at):
        values = deepcopy(values)
        values.setdefault('config_epoch', self._state['config_epoch'])
        values.setdefault('reference_epoch', self._state['reference_epoch'])
        values.setdefault('setpoints', [])
        return self._session, produced_at, values

    def publish_targets(self, setpoints=(), *, produced_at=None, **values):
        with self._lock:
            if not self._connected:
                raise wire.ProtocolError('hello required')
            self._source_generation += 1
            self._source = None
            if self._goal is not None:
                self._stats['superseded'] += 1
            self._goal = self._goal_at(dict(values, setpoints=setpoints),
                                      time.monotonic() if produced_at is None else produced_at)
        self._wake.set()

    def publish_permit(self, allow, ttl_ms):
        with self._lock:
            if not self._connected:
                raise wire.ProtocolError('hello required')
            self._permit = (self._session, time.monotonic(), bool(allow), int(ttl_ms))
        self._wake.set()

    def set_target_source(self, source):
        """source(now_s, epochs) 返回 publish_targets 的参数, 或 None 结束并撤销目标.

        source 不访问 Tk/串口、不等待 Future. 固定位置测试须锁定下发时参考代次;
        参考改变就返回 None, 不把旧位置数字重贴代次. 速度目标可持续按 SI 生成.
        """
        with self._lock:
            if not self._connected:
                raise wire.ProtocolError('hello required')
            self._source_generation += 1
            self._source = source
            self._goal = None if source else self._goal_at({}, time.monotonic())
        self._wake.set()

    def _make_payload(self, request, sequence):
        op, values = request.operation, request.values
        encoder = {wire.HELLO: wire.encode_hello, wire.AUTHORIZE: wire.encode_authorize,
                   wire.ALIGN: wire.encode_align, wire.PARAM_READ: wire.encode_param_read,
                   wire.PARAM_WRITE: wire.encode_param_write, wire.STATUS: wire.encode_status,
                   wire.DIAGNOSTICS: wire.encode_diagnostics, wire.TARGETS: wire.encode_targets,
                   wire.PERMIT: wire.encode_permit,
                   wire.PITCH_IDENTIFICATION: wire.encode_pitch_identification}[op]
        return encoder(**values) if op == wire.HELLO else encoder(request.session, sequence, **values)

    def _begin(self, request, now):
        future = request.future
        if future is not None and not future.set_running_or_notify_cancel():
            self._clear_lane(request)
            return
        try:
            if request.operation == wire.HELLO:
                self._reset_session()
                sequence = 0
            else:
                if not self._connected or request.session != self._session:
                    raise wire.ProtocolError('request belongs to an inactive session')
                self._sequence += 1
                sequence = self._sequence
            payload = self._make_payload(request, sequence)
            self._frame = (self._frame+1) & 0xffff
            while self._frame in self._pending:
                self._frame = (self._frame+1) & 0xffff
            message = wire.encode_frame(request.operation, self._frame, payload)
        except (wire.ProtocolError, ValueError, TypeError, OverflowError, struct.error) as error:
            self._clear_lane(request)
            if future is not None:
                future.set_exception(error)
            else:
                with self._lock:
                    self._state['error'] = str(error)
            return
        pending = _Pending(request, sequence, self._frame, message, now+self.timeout_s)
        if len(self._pending) >= 128:
            oldest = next(key for key, value in self._pending.items() if value.request.future is None)
            del self._pending[oldest]
            self._stats['stream_timeouts'] += 1
        self._pending[self._frame] = pending
        if request.operation == wire.TARGETS:
            with self._lock:
                self._state['last_target_sent_sequence'] = sequence
        self.transport.send(message)
        self._stats['sent'] += 1

    def _clear_lane(self, request):
        if request.future is None:
            return
        with self._lock:
            if request.mutation or request.operation == wire.HELLO:
                self._change_active = False
            else:
                self._read_active = False

    def _reset_session(self):
        # 新握手会在 MCU 替换会话; 旧读请求/连续目标不迁移到新身份.
        for pending in tuple(self._pending.values()):
            self._finish(pending, error=wire.ProtocolError('session replaced'))
        with self._lock:
            reads = list(self._reads)
            self._reads.clear()
            self._goal = self._permit = self._source = None
            self._source_generation += 1
            self._connected = False
            self._state['connected'] = False
            # 采集身份属于旧会话: 连同未发出的请求一起作废, 旧 run 的迟到应答不得续采.
            pitch_requests = list(self._pitch_requests)
            self._pitch_requests.clear()
            pitch_pendings = list(self._pitch_pending.values())
            self._pitch_pending.clear()
            self._pitch.update(run=None, op=0, duration_us=0, active=False, session=0,
                               confirmed_by=None, packets=0, records=0, last_status={},
                               last_packet_monotonic=None, started_at_monotonic=None,
                               stopped_at_monotonic=None, last_ack=None)
        for request in reads:
            if not request.future.cancelled():
                request.future.set_exception(wire.ProtocolError('session replaced'))
        for pending in pitch_pendings:
            self._settle_stream_future(pending.request, error=wire.ProtocolError('session replaced'))
        for request in pitch_requests:
            self._settle_stream_future(request, error=wire.ProtocolError('session replaced'))
        self._sequence = 0
        self._page_sequences.clear()

    def _finish(self, pending, decoded=None, error=None):
        self._pending.pop(pending.frame_sequence, None)
        request = pending.request
        self._clear_lane(request)
        if request.future is not None:
            if error:
                request.future.set_exception(error)
            else:
                request.future.set_result(decoded)

    def _receive(self, reply):
        rtype, echo, body = reply
        pending = self._pending.get(echo)
        if pending is None:
            self._stats['unmatched'] += 1
            return
        request = pending.request
        if not body or body[0] != wire.PROTOCOL_VERSION:
            error = wire.ProtocolError(f'protocol mismatch: MCU={body[0] if body else None}, client={wire.PROTOCOL_VERSION}')
            self._finish(pending, error=error)
            raise error
        try:
            if len(body) < 2 or body[1] != rtype or rtype not in (wire.RESPONSE_OF[request.operation], wire.ACK):
                raise wire.ProtocolError('unexpected response type')
            decoded = wire.DECODERS[rtype](body)
            if rtype == wire.ACK and wire.RESPONSE_OF[request.operation] != wire.ACK and decoded.get('result') == 0:
                raise wire.ProtocolError('success ACK has the wrong response type')
            if request.operation == wire.HELLO:
                if decoded.get('client_nonce') != request.values['nonce']:
                    raise wire.ProtocolError('hello nonce mismatch')
            elif decoded.get('session') != request.session or decoded.get('sequence') != pending.sequence:
                raise wire.ProtocolError('response identity mismatch')
        except wire.ProtocolError as error:
            self._stats['invalid_replies'] += 1
            with self._lock:
                self._state['error'] = str(error)
            return
        self._stats['replies'] += 1
        decoded['received_at_monotonic'] = time.monotonic()
        with self._lock:
            # Keep every validated rejection, even when a newer successful ACK
            # replaces last_target_ack before the consumer next takes a snapshot.
            if request.operation == wire.TARGETS and decoded.get('result') != 0:
                self._state['target_rejections'] += 1
                self._state['last_target_rejection'] = dict(decoded)
            if request.operation == wire.HELLO:
                self._session = decoded['session']
                self._connected = True
                self._state.update(decoded, connected=True, status={}, diagnostics={}, error='',
                                   last_target_ack=None, last_target_sent_sequence=0)
            else:
                # 缓存重传的代次是原请求完成时的代次, 不能覆盖随后已经观测到的新代次.
                for key in ('config_epoch', 'reference_epoch'):
                    if key in decoded:
                        self._state[key] = max(self._state[key], decoded[key])
            key = request.operation, decoded.get('page', 0)
            if pending.sequence > self._page_sequences.get(key, -1):
                self._page_sequences[key] = pending.sequence
                if request.operation == wire.STATUS and 'page' in decoded:
                    # 只记录主机收包时刻, 供界面判旧; 不更改 MCU 样本时间或线协议.
                    self._state['status'][decoded['page']] = deepcopy(decoded)
                elif request.operation == wire.DIAGNOSTICS and 'page' in decoded:
                    self._state['diagnostics'][decoded['page']] = deepcopy(decoded)
                elif request.operation == wire.TARGETS:
                    self._state['last_target_ack'] = dict(decoded)
            if request.mutation:
                self._state['last_change'] = dict(decoded)
            if decoded.get('result_name') == 'NO_SESSION':
                self._connected = False
                self._state['connected'] = False
                self._state['error'] = 'NO_SESSION: explicit hello required'
                self._goal = self._permit = self._source = None
                self._source_generation += 1
        error = None
        if request.mutation and decoded.get('result_name') in ('NO_SESSION', 'REPLAY'):
            error = OutcomeUnknown(request.operation, request.session, pending.sequence, decoded['result_name'])
        self._finish(pending, decoded, error)

    def _expire(self, now):
        for pending in tuple(self._pending.values()):
            if now < pending.deadline:
                continue
            request = pending.request
            if request.mutation and self._connected and request.session == self._session and pending.attempts < self.attempts:
                # 整个线帧原样重传: 不分配业务新序号, 不重新执行参考变更, 不给旧目标续期.
                self.transport.send(pending.message)
                pending.attempts += 1
                pending.deadline = now+self.timeout_s
                self._stats['retries'] += 1
                continue
            if request.mutation:
                error = OutcomeUnknown(request.operation, request.session, pending.sequence, 'ACK timeout')
                with self._lock:
                    self._state['error'] = str(error)
            else:
                error = TimeoutError('response timeout')
                self._stats['stream_timeouts'] += 1
            # HELLO 总是替换会话, 不能用同 nonce 盲重传并误认延迟的旧会话 ACK.
            self._finish(pending, error=error)

    def _targets(self, now):
        with self._lock:
            source, generation = self._source, self._source_generation
            epochs = {key: self._state[key] for key in ('config_epoch', 'reference_epoch')}
        if source:
            values = source(now, epochs)
            with self._lock:
                if generation == self._source_generation:
                    if values is None:
                        self._source = None
                    self._goal = self._goal_at(values or {}, now)
        with self._lock:
            goal, self._goal = self._goal, None
        if goal is None:
            return
        session, produced, values = goal
        if session != self._session or any(values[key] != self._state[key] for key in ('config_epoch', 'reference_epoch')):
            self._stats['dropped_epochs'] += 1
            return
        age = max(0, math.ceil((time.monotonic()-produced)*1000))
        values['setpoints'] = [(domain, mode, value, min(65535, int(source_age)+age))
                               for domain, mode, value, source_age in values['setpoints']]
        values['chassis_age_ms'] = min(65535, values.get('chassis_age_ms', 0)+age)
        self._begin(_Request(wire.TARGETS, values, None, session), now)

    def _run(self):
        next_target = next_fast = 0.0
        error = None
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                for _ in range(8):
                    reply = self.transport.receive(0)
                    if reply is None:
                        break
                    # 原始连续流有自己的身份(帧回显/op/run), 不参与 v3 业务应答校验.
                    if reply[0] == PITCH_STREAM_DATA:
                        self._pitch_data(reply[2])
                    elif reply[0] == PITCH_STREAM_ACK:
                        self._pitch_ack(reply[1], reply[2])
                    else:
                        self._receive(reply)
                self._expire(now)
                self._expire_stream(now)
                with self._lock:
                    change, self._change = self._change, None
                if change:
                    self._begin(change, now)
                with self._lock:
                    stream_request = (self._pitch_requests.popleft()
                                      if self._pitch_requests and self._connected else None)
                if stream_request:
                    self._begin_stream(stream_request, now)
                if self._connected:
                    if now >= next_target:
                        self._targets(now)
                        next_target = now+self.target_period
                    with self._lock:
                        permit, self._permit = self._permit, None
                    if permit and permit[0] == self._session:
                        _, produced, allow, ttl = permit
                        remaining = ttl-max(0, math.ceil((time.monotonic()-produced)*1000))
                        self._begin(_Request(wire.PERMIT, {'permit': allow and remaining > 0,
                            'ttl_ms': max(1, remaining)}, None, self._session), now)
                with self._lock:
                    read = None
                    if self._reads and not self._read_active:
                        read = self._reads.popleft()
                        self._read_active = True
                if read:
                    self._begin(read, now)
                if self._connected and now >= next_fast and math.isfinite(self.fast_period):
                    # 显式快页查询存在时不以后台快页覆盖它的 MCU 待发送槽.
                    explicit_fast = any(p.request.future is not None and p.request.operation == wire.STATUS
                        and p.request.values.get('page') == 0 for p in self._pending.values())
                    if not explicit_fast:
                        self._begin(_Request(wire.STATUS, {'page': 0}, None, self._session), now)
                    next_fast = now+self.fast_period
                self._wake.wait(0.001)
                self._wake.clear()
        except Exception as caught:
            error = caught
            with self._lock:
                self._state['error'] = str(caught)
        finally:
            self._stop.set()
            for pending in tuple(self._pending.values()):
                request = pending.request
                failure = OutcomeUnknown(request.operation, request.session, pending.sequence,
                    str(error or 'link closed')) if request.mutation else wire.ProtocolError(str(error or 'link closed'))
                self._finish(pending, error=failure)
            with self._lock:
                requests = list(self._reads) + ([self._change] if self._change else [])
                self._reads.clear()
                self._change = None
                stream_requests = list(self._pitch_requests)
                self._pitch_requests.clear()
                stream_pendings = list(self._pitch_pending.values())
                self._pitch_pending.clear()
                self._pitch.update(active=False, run=None, confirmed_by=None)
                self._connected = False
                self._state['connected'] = False
            for request in requests:
                if not request.future.cancelled():
                    request.future.set_exception(wire.ProtocolError(str(error or 'link closed before send')))
            for pending in stream_pendings:
                self._settle_stream_future(pending.request, error=wire.ProtocolError(str(error or 'link closed')))
            for request in stream_requests:
                self._settle_stream_future(request, error=wire.ProtocolError(str(error or 'link closed before send')))

    def close(self):
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=2)
        self.transport.close()
        self._thread.join(timeout=2)
        if self._log:
            self._log.close()
