"""Actual async receive logic, with no UART thread or external transport."""
import struct
import sys
import time
import unittest
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robots/sentinel/host'))
import sentinel_async as async_mod
import sentinel_client as wire


class RecordingTransport:
    def send(self, message):
        pass


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        with patch('threading.Thread.start'):
            self.peer = async_mod.AsyncSentinelClient(RecordingTransport(), fast_hz=0)
        self.peer._session = 12
        self.peer._connected = True
        self.peer._state.update(session=12, connected=True)

    def target(self):
        future = Future()
        request = async_mod._Request(wire.TARGETS,
            {'config_epoch': 0, 'reference_epoch': 0, 'setpoints': []}, future, 12)
        self.peer._begin(request, time.monotonic())
        return list(self.peer._pending.values())[-1], future

    def reply(self, pending, result=0, session=12):
        body = struct.pack('<BBIIBB', wire.PROTOCOL_VERSION, wire.TARGETS_ACK,
                           session, pending.sequence, result, 0)
        self.peer._receive((wire.TARGETS_ACK, pending.frame_sequence, body))

    def test_reject_then_accept_preserves_evidence_and_timestamps(self):
        first, f1 = self.target()
        self.reply(first, 1)
        second, f2 = self.target()
        self.reply(second, 0)
        snap = self.peer.snapshot()
        self.assertEqual(snap['target_rejections'], 1)
        self.assertEqual(snap['last_target_rejection']['sequence'], first.sequence)
        self.assertEqual(snap['last_target_ack']['result'], 0)
        self.assertEqual(snap['last_target_sent_sequence'], second.sequence)
        self.assertLessEqual(f1.result()['received_at_monotonic'], time.monotonic())
        self.assertIn('received_at_monotonic', f2.result())

    def test_older_matching_rejection_not_discarded(self):
        first, _ = self.target()
        second, _ = self.target()
        self.reply(second)
        self.reply(first, 1)
        snap = self.peer.snapshot()
        self.assertEqual(snap['last_target_ack']['sequence'], second.sequence)
        self.assertEqual(snap['target_rejections'], 1)

    def test_invalid_identity_cannot_increment_evidence(self):
        pending, _ = self.target()
        self.reply(pending, 1, session=13)
        self.assertEqual(self.peer.snapshot()['target_rejections'], 0)
        self.assertEqual(self.peer.snapshot()['statistics']['invalid_replies'], 1)

    def test_unmatched_duplicate_does_not_increment_counter(self):
        pending, _ = self.target()
        self.reply(pending, 1)
        self.reply(pending, 1)
        self.assertEqual(self.peer.snapshot()['target_rejections'], 1)


if __name__ == '__main__':
    unittest.main()
