"""Headless contracts for the single-owner raw lane (no UART connection)."""
import pathlib
import struct
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'robots/sentinel/host'))
from sentinel_raw_async import RawRequestLane
import sentinel_client as wire


def request(op=1, run=77):
    return struct.pack('<4B2I', 1, op, 4, 5, run, 125000)


def reply(op=1, run=77):
    return bytes((1, op, 0, 1)) + struct.pack('<I', run) + bytes(24)


class RawLaneTests(unittest.TestCase):
    def setUp(self):
        self.lane = RawRequestLane(timeout_s=.1, attempts=2)
        self.sent = []

    def tick(self, now=0, session=3):
        self.lane.tick(now, session, lambda: 19, self.sent.append)

    def test_no_implicit_capture(self):
        self.tick()
        self.assertEqual(self.sent, [])

    def test_correlated_reply(self):
        f = self.lane.post(request(), 3)
        self.tick()
        self.assertEqual(len(self.sent), 1)
        self.assertFalse(self.lane.receive((0xca, 19, reply()), 3))
        self.lane.receive((0xb1, 20, reply()), 3)
        self.lane.receive((0xb1, 19, reply(run=78)), 3)
        self.lane.receive((0xb1, 19, reply(op=2)), 3)
        self.assertFalse(f.done())
        self.lane.receive((0xb1, 19, reply()), 3)
        self.assertEqual(f.result(), reply())

    def test_bounded_identical_retries(self):
        f = self.lane.post(request(), 3)
        self.tick(0)
        self.tick(.11)
        self.assertEqual(self.sent[0], self.sent[1])
        self.tick(.22)
        with self.assertRaisesRegex(TimeoutError, 'NOT confirmed'):
            f.result()
        self.assertEqual(len(self.sent), 2)

    def test_session_change_never_sends_old_intent(self):
        f = self.lane.post(request(), 3)
        self.tick(session=4)
        with self.assertRaises(wire.ProtocolError):
            f.result()
        self.assertFalse(self.sent)

    def test_one_pending_and_close(self):
        f = self.lane.post(request(), 3)
        with self.assertRaises(wire.ProtocolError):
            self.lane.post(request(), 3)
        self.lane.close()
        with self.assertRaises(wire.ProtocolError):
            f.result()
        with self.assertRaises(wire.ProtocolError):
            self.lane.post(request(), 3)

    def test_cancel_before_send(self):
        f = self.lane.post(request(), 3)
        f.cancel()
        self.tick()
        self.assertFalse(self.sent)

    def test_transport_error(self):
        f = self.lane.post(request(), 3)
        def fail(_):
            raise OSError('offline fixture')
        self.lane.tick(0, 3, lambda:19, fail)
        with self.assertRaises(OSError):
            f.result()


class AsyncOwnerIntegration(unittest.TestCase):
    def peer(self, transport):
        from unittest.mock import patch
        from sentinel_async import AsyncSentinelClient
        with patch('threading.Thread.start'):
            peer = AsyncSentinelClient(transport, fast_hz=0)
        peer._session, peer._connected = 3, True
        peer._state.update(session=3, connected=True)
        return peer

    def test_actual_owner_dispatches_raw_ack_before_business_parser(self):
        class Transport:
            def __init__(self): self.sent=[]; self.replies=[]; self.peer=None
            def send(self, message):
                self.sent.append(message)
                decoder=wire.FrameDecoder()
                for byte in message:
                    frame=decoder.push(byte)
                    if frame is not None:
                        self.assert_raw(frame)
                        self.replies.append((0xb1,frame[1],reply()))
            def assert_raw(self, frame):
                if frame[0] != 0x31: raise AssertionError('unexpected business traffic')
            def receive(self, timeout=0):
                if self.replies:
                    self.peer._stop.set()
                    return self.replies.pop(0)
                return None
            def close(self): pass
        transport=Transport();peer=self.peer(transport);transport.peer=peer
        future=peer.raw_request(request(),expected_session=3)
        peer._run()  # same production owner loop, mock transport only
        self.assertEqual(future.result(timeout=.1), reply())
        self.assertEqual(peer._stats['invalid_replies'],0)
        self.assertEqual(len(transport.sent),1)

    def test_expected_session_is_checked_at_raw_enqueue(self):
        class Transport:
            def close(self): pass
        peer=self.peer(Transport());peer._session=4
        with self.assertRaisesRegex(wire.ProtocolError,'replaced session'):
            peer.raw_request(request(),expected_session=3)
        self.assertFalse(peer._raw_lane.frame_pending(0))

    def test_expired_start_never_reaches_transport(self):
        class Transport:
            def __init__(self):self.sent=[]
            def send(self,message):self.sent.append(message)
            def close(self):pass
        transport=Transport();peer=self.peer(transport)
        future=peer.experiment_start(5,1,17,31,hardware_blocked=False,dispatch_not_after=0.)
        peer._begin(peer._change,0.)
        with self.assertRaisesRegex(wire.ProtocolError,'deadline expired'):
            future.result(timeout=.1)
        self.assertFalse(transport.sent)


if __name__ == '__main__':
    unittest.main()
