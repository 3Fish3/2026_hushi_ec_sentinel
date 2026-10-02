"""业务协议 v3 异步入口: 一个 UART 拥有者, 最新目标邮箱, 一个在途变更.

publish_targets 的每次调用代表新的意图; 尚未发送的旧值被覆盖, 在主机滞留的时间计入源年龄.
固定目标/轨迹可用 set_target_source 提供纯计算生成器 (不做阻塞 I/O); UI 绘制不驱动发送时钟.
生成器必须明确处理参考变化, 不把旧位置数值重新贴上新参考代次. 变更不明只读回确认,
不能用新序号自动重做 align. 原生管道仅验证协议/编排, 不模拟真实 UART 延迟或 MCU 时钟.
"""
from __future__ import annotations

from collections import OrderedDict, deque
from concurrent.futures import Future
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import secrets
import struct
import threading
import time

import sentinel_client as wire


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
        record = dict(ordinal=self.ordinal, monotonic_s=time.monotonic(), direction=direction,
                      wire_hex=data.hex())
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        last = 0
        try:
            self.file.write(json.dumps(dict(event='begin', protocol=wire.PROTOCOL_VERSION,
                                           monotonic_s=time.monotonic()))+'\n')
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    record = self.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                if record['ordinal'] != last+1:
                    self.file.write(json.dumps(dict(event='gap', count=record['ordinal']-last-1))+'\n')
                self.file.write(json.dumps(record)+'\n')
                last = record['ordinal']
            self.file.write(json.dumps(dict(event='end', total=self.ordinal, dropped=self.dropped))+'\n')
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


class AsyncSentinelClient:
    def __init__(self, transport, *, target_hz=100.0, fast_hz=100.0,
                 timeout_s=0.15, attempts=3, log_path=None):
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
        self._stats = dict(sent=0, replies=0, unmatched=0, invalid_replies=0, retries=0,
                           superseded=0, stream_timeouts=0, dropped_epochs=0)
        self._state = dict(connected=False, session=0, config_epoch=0, reference_epoch=0,
                           status={}, diagnostics={}, error='', last_change=None, last_target_ack=None)
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

    def hello(self, nonce=None):
        return self._post(wire.HELLO, dict(nonce=secrets.randbits(32) if nonce is None else nonce))

    def authorize(self, grant=True, source='maintenance'):
        return self._post(wire.AUTHORIZE, dict(grant=grant, source=source), True)

    def align(self, domain, value, *, kind='joint_reference', reference_epoch=None):
        if reference_epoch is None:
            reference_epoch = self.snapshot()['reference_epoch']
        return self._post(wire.ALIGN, dict(domain=domain, value=value, kind=kind,
                                           reference_epoch=reference_epoch), True)

    def param_write(self, param_id, value):
        return self._post(wire.PARAM_WRITE, dict(param_id=param_id, value=value), True)

    def param_read(self, param_id=0xffff, offset=0):
        return self._post(wire.PARAM_READ, dict(param_id=param_id, offset=offset))

    def status(self, page=0):
        return self._post(wire.STATUS, dict(page=page))

    def diagnostics(self, page=0):
        return self._post(wire.DIAGNOSTICS, dict(page=page))

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
                   wire.PERMIT: wire.encode_permit}[op]
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
        for request in reads:
            if not request.future.cancelled():
                request.future.set_exception(wire.ProtocolError('session replaced'))
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
        with self._lock:
            if request.operation == wire.HELLO:
                self._session = decoded['session']
                self._connected = True
                self._state.update(decoded, connected=True, status={}, diagnostics={}, error='')
            else:
                # 缓存重传的代次是原请求完成时的代次, 不能覆盖随后已经观测到的新代次.
                for key in ('config_epoch', 'reference_epoch'):
                    if key in decoded:
                        self._state[key] = max(self._state[key], decoded[key])
            key = request.operation, decoded.get('page', 0)
            if pending.sequence > self._page_sequences.get(key, -1):
                self._page_sequences[key] = pending.sequence
                if request.operation == wire.STATUS and 'page' in decoded:
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
                    self._receive(reply)
                self._expire(now)
                with self._lock:
                    change, self._change = self._change, None
                if change:
                    self._begin(change, now)
                if self._connected:
                    if now >= next_target:
                        self._targets(now)
                        next_target = now+self.target_period
                    with self._lock:
                        permit, self._permit = self._permit, None
                    if permit and permit[0] == self._session:
                        _, produced, allow, ttl = permit
                        remaining = ttl-max(0, math.ceil((time.monotonic()-produced)*1000))
                        self._begin(_Request(wire.PERMIT, dict(permit=allow and remaining > 0,
                            ttl_ms=max(1, remaining)), None, self._session), now)
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
                        self._begin(_Request(wire.STATUS, dict(page=0), None, self._session), now)
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
                self._reads.clear(); self._change = None
                self._connected = False
                self._state['connected'] = False
            for request in requests:
                if not request.future.cancelled():
                    request.future.set_exception(wire.ProtocolError(str(error or 'link closed before send')))

    def close(self):
        self._stop.set(); self._wake.set()
        self._thread.join(timeout=2)
        self.transport.close()
        self._thread.join(timeout=2)
        if self._log:
            self._log.close()
