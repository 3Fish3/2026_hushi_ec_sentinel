"""Bounded raw short-window lane, serviced by the existing single UART owner.

No serial object, reader thread, motion command or background capture lives here.
Retries preserve the complete frame/run/op/index; a timeout is not a stop proof.
"""
from concurrent.futures import Future
from dataclasses import dataclass
import math
import threading

import sentinel_client as wire


@dataclass
class _Pending:
    payload: bytes
    session: int
    future: Future
    frame: int | None = None
    message: bytes = b''
    due: float = 0.0
    attempts: int = 0


class RawRequestLane:
    def __init__(self, *, timeout_s=.12, attempts=3):
        if not math.isfinite(timeout_s) or timeout_s <= 0 or attempts not in (1, 2, 3):
            raise ValueError('invalid bounded raw retry policy')
        self.timeout_s, self.attempts = timeout_s, attempts
        self._lock = threading.Lock()
        self._pending = None
        self._closed = False

    def frame_pending(self, frame):
        with self._lock:
            return self._pending is not None and self._pending.frame == frame

    def post(self, payload, session):
        payload = bytes(payload)
        if len(payload) != 12 or payload[0] != 1 or payload[1] > 3 or not session:
            raise ValueError('invalid raw request/session')
        future = Future()
        with self._lock:
            if self._closed:
                raise wire.ProtocolError('raw lane closed')
            if self._pending is not None:
                raise wire.ProtocolError('one raw request already pending')
            self._pending = _Pending(payload, session, future)
        return future

    def tick(self, now, session, allocate_frame, send):
        """UART owner only. allocate_frame returns a globally unused frame id."""
        error = None
        message = None
        with self._lock:
            p = self._pending
            if p is None:
                return
            if p.future.cancelled():
                self._pending = None
                return
            if not session or session != p.session:
                error = wire.ProtocolError('raw request session changed')
            elif p.frame is not None and now < p.due:
                return
            elif p.attempts >= self.attempts:
                error = TimeoutError('raw request outcome unknown; capture/motion stop NOT confirmed')
            else:
                if p.frame is None:
                    p.frame = allocate_frame()
                    p.message = wire.encode_frame(0x31, p.frame, p.payload)
                p.attempts += 1
                p.due = now + self.timeout_s
                message = p.message
            if error is not None:
                self._pending = None
        if error is not None:
            if not p.future.done():
                p.future.set_exception(error)
        elif message is not None:
            try:
                send(message)
            except Exception as exc:
                self.fail(exc)

    def receive(self, reply, session):
        """Return True only for raw ACK frames. Verify op/run before settling."""
        kind, frame, body = reply
        if kind != 0xb1:
            return False
        with self._lock:
            p = self._pending
            if p is None or p.frame != frame:
                return True
            if p.session != session:
                error = wire.ProtocolError('raw ACK session changed')
            elif len(body) < 32 or body[0] != 1 or body[1] != p.payload[1]:
                return True  # malformed/unrelated: await exact ACK, never make success
            elif p.payload[1] and body[4:8] != p.payload[4:8]:
                return True
            else:
                error = None
            self._pending = None
        if not p.future.done():
            if error is not None:
                p.future.set_exception(error)
            else:
                p.future.set_result(bytes(body))  # adapter validates error/layout/rows
        return True

    def fail(self, error):
        with self._lock:
            p, self._pending = self._pending, None
        if p is not None and not p.future.done():
            p.future.set_exception(error)

    def close(self):
        with self._lock:
            self._closed = True
        self.fail(wire.ProtocolError('raw lane closed; stop NOT confirmed'))
