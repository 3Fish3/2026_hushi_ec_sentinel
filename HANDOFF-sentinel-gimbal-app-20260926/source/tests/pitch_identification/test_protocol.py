"""Protocol wire and client tests for pitch identification (v3 wire contract)."""

from __future__ import annotations

import math
import struct
import unittest

from sentinel_client import (
    ACK,
    PITCH_IDENT_FAULTS,
    PITCH_IDENT_OPS,
    PITCH_IDENT_STATES,
    PITCH_IDENTIFICATION,
    PITCH_IDENTIFICATION_STATUS,
    PROTOCOL_VERSION,
    RESPONSE_OF,
    ProtocolError,
    SentinelClient,
    decode_ack,
    decode_pitch_identification_status,
    encode_pitch_identification,
)


class DummyTransport:
    def __init__(self, reply_payload: bytes, reply_type: int):
        self.reply_payload = reply_payload
        self.reply_type = reply_type
        self.sent_messages = []

    def exchange(self, message: bytes) -> tuple[int, int, bytes]:
        self.sent_messages.append(message)
        return (self.reply_type, 1, self.reply_payload)


class TestPitchIdentificationProtocol(unittest.TestCase):
    def test_constants_and_response_mapping(self):
        self.assertEqual(PITCH_IDENTIFICATION, 0x49)
        self.assertEqual(PITCH_IDENTIFICATION_STATUS, 0xC9)
        self.assertEqual(RESPONSE_OF[PITCH_IDENTIFICATION], PITCH_IDENTIFICATION_STATUS)
        self.assertEqual(PITCH_IDENT_STATES, ("idle", "baseline", "square", "final_hold", "complete", "fault"))
        # 0..9 是与既有固件共享的稳定编码, 新增故障只能追加, 不能让旧编号位移.
        self.assertEqual(
            PITCH_IDENT_FAULTS[:10],
            (
                "none", "authority", "feedback", "isolation", "envelope", "binding",
                "timing", "torque", "transmit", "cancelled",
            ),
        )
        # 固件枚举 PitchIdentificationFault: 10 预留未用, capture 显式固定为 11.
        self.assertEqual(PITCH_IDENT_FAULTS[10], "reserved_10")
        self.assertEqual(PITCH_IDENT_FAULTS[11], "capture")
        self.assertEqual(len(PITCH_IDENT_FAULTS), 12)
        self.assertEqual(PITCH_IDENT_OPS, ("query", "start", "cancel", "reset"))

    def test_encode_query_cancel_reset_lengths(self):
        for op in ("query", "cancel", "reset", 0, 2, 3):
            wire = encode_pitch_identification(
                session=0x12345678,
                sequence=42,
                op=op,
                config_epoch=10,
                reference_epoch=20,
            )
            # Must be exactly 19 bytes
            self.assertEqual(len(wire), 19)
            self.assertEqual(wire[0], PROTOCOL_VERSION)
            self.assertEqual(wire[1], PITCH_IDENTIFICATION)
            session, seq, op_code, c_epoch, r_epoch = struct.unpack_from("<IIBII", wire, 2)
            self.assertEqual(session, 0x12345678)
            self.assertEqual(seq, 42)
            expected_op = PITCH_IDENT_OPS.index(op) if isinstance(op, str) else op
            self.assertEqual(op_code, expected_op)
            self.assertEqual(c_epoch, 10)
            self.assertEqual(r_epoch, 20)

    def test_encode_start_length(self):
        wire = encode_pitch_identification(
            session=0x12345678,
            sequence=100,
            op="start",
            config_epoch=1,
            reference_epoch=2,
            run_id=5,
            amplitude_nm=0.04,
            half_period_us=10000,
            baseline_us=5000,
            cycles=3,
            final_hold_us=5000,
        )
        # Must be exactly 43 bytes
        self.assertEqual(len(wire), 43)
        self.assertEqual(wire[0], PROTOCOL_VERSION)
        self.assertEqual(wire[1], PITCH_IDENTIFICATION)
        session, seq, op_code, c_epoch, r_epoch, run_id, amp, hp, base, cyc, hold = struct.unpack_from(
            "<IIBIIIfIIII", wire, 2
        )
        self.assertEqual(session, 0x12345678)
        self.assertEqual(seq, 100)
        self.assertEqual(op_code, 1)
        self.assertEqual(c_epoch, 1)
        self.assertEqual(r_epoch, 2)
        self.assertEqual(run_id, 5)
        self.assertAlmostEqual(amp, 0.04, places=6)
        self.assertEqual(hp, 10000)
        self.assertEqual(base, 5000)
        self.assertEqual(cyc, 3)
        self.assertEqual(hold, 5000)

    def test_decode_status_success(self):
        raw = bytes([PROTOCOL_VERSION, PITCH_IDENTIFICATION_STATUS]) + struct.pack(
            "<II5BIff",
            0x12345678,  # session
            101,         # sequence
            0,           # result (ACCEPTED)
            0,           # reason
            1,           # available
            2,           # state (square)
            0,           # fault (none)
            7,           # run_id
            0.04,        # requested_nm
            0.0398,      # encoded_nm
        )
        self.assertEqual(len(raw), 27)
        decoded = decode_pitch_identification_status(raw)
        self.assertEqual(decoded["session"], 0x12345678)
        self.assertEqual(decoded["sequence"], 101)
        self.assertEqual(decoded["result"], 0)
        self.assertEqual(decoded["result_name"], "ACCEPTED")
        self.assertTrue(decoded["available"])
        self.assertEqual(decoded["state"], 2)
        self.assertEqual(decoded["state_name"], "square")
        self.assertEqual(decoded["fault"], 0)
        self.assertEqual(decoded["fault_name"], "none")
        self.assertEqual(decoded["run_id"], 7)
        self.assertAlmostEqual(decoded["requested_nm"], 0.04, places=5)
        self.assertAlmostEqual(decoded["encoded_nm"], 0.0398, places=4)

    def test_decode_status_capture_fault(self):
        """采集联锁故障: 固件用 11 表示 capture, 10 保持预留, 解码不得错位."""
        raw = bytes([PROTOCOL_VERSION, PITCH_IDENTIFICATION_STATUS]) + struct.pack(
            "<II5BIff",
            0x12345678,
            104,
            0,           # ACCEPTED
            0,
            0,           # available 已被故障撤销
            5,           # state (fault)
            11,          # fault (capture)
            9,           # run_id
            0.03,
            0.0,
        )
        self.assertEqual(len(raw), 27)
        decoded = decode_pitch_identification_status(raw)
        self.assertEqual(decoded["state_name"], "fault")
        self.assertEqual(decoded["fault"], 11)
        self.assertEqual(decoded["fault_name"], "capture")
        self.assertEqual(PITCH_IDENT_FAULTS[10], "reserved_10")

    def test_decode_status_rejections(self):
        raw = bytes([PROTOCOL_VERSION, PITCH_IDENTIFICATION_STATUS]) + struct.pack(
            "<II5BIff",
            0x12345678,
            102,
            7,           # NOT_PERMITTED
            0,
            0,           # not available
            0,           # idle
            0,           # none
            0,           # run_id 0
            0.0,
            0.0,
        )
        decoded = decode_pitch_identification_status(raw)
        self.assertEqual(decoded["result"], 7)
        self.assertEqual(decoded["result_name"], "NOT_PERMITTED")
        self.assertFalse(decoded["available"])
        self.assertEqual(decoded["state_name"], "idle")

    def test_decode_status_bad_length(self):
        short_payload = bytes([PROTOCOL_VERSION, PITCH_IDENTIFICATION_STATUS]) + b"\x00" * 20
        with self.assertRaises(ProtocolError):
            decode_pitch_identification_status(short_payload)

        long_payload = bytes([PROTOCOL_VERSION, PITCH_IDENTIFICATION_STATUS]) + b"\x00" * 30
        with self.assertRaises(ProtocolError):
            decode_pitch_identification_status(long_payload)

    def test_short_error_ack_decode(self):
        ack_payload = bytes([PROTOCOL_VERSION, ACK]) + struct.pack(
            "<IIBB",
            0x12345678,
            103,
            4,  # NO_SESSION
            0,
        )
        self.assertEqual(len(ack_payload), 12)
        decoded = decode_ack(ack_payload)
        self.assertEqual(decoded["result"], 4)
        self.assertEqual(decoded["result_name"], "NO_SESSION")

    def test_client_exchange_with_status_response(self):
        status_payload = bytes([PROTOCOL_VERSION, PITCH_IDENTIFICATION_STATUS]) + struct.pack(
            "<II5BIff",
            0x11112222,
            1,
            0, 0, 1, 1, 0, 12, 0.0, 0.0,
        )
        transport = DummyTransport(status_payload, PITCH_IDENTIFICATION_STATUS)
        client = SentinelClient(transport, session=0x11112222)
        res = client.pitch_identification("query")
        self.assertEqual(res["result"], 0)
        self.assertEqual(res["result_name"], "ACCEPTED")
        self.assertEqual(res["run_id"], 12)
        self.assertEqual(res["state_name"], "baseline")

    def test_client_exchange_with_short_error_ack(self):
        ack_payload = bytes([PROTOCOL_VERSION, ACK]) + struct.pack(
            "<IIBB",
            0x11112222,
            1,
            7,  # NOT_PERMITTED
            0,
        )
        transport = DummyTransport(ack_payload, ACK)
        client = SentinelClient(transport, session=0x11112222)
        res = client.pitch_identification("start", run_id=1, amplitude_nm=0.04)
        self.assertEqual(res["envelope"], "error")
        self.assertEqual(res["result"], 7)
        self.assertEqual(res["result_name"], "NOT_PERMITTED")

    def test_nonfinite_handling(self):
        wire_nan = encode_pitch_identification(
            session=1, sequence=1, op="start", amplitude_nm=float("nan")
        )
        self.assertEqual(len(wire_nan), 43)
        amp = struct.unpack_from("<f", wire_nan, 2 + 4 + 4 + 1 + 4 + 4 + 4)[0]
        self.assertTrue(math.isnan(amp))

        wire_inf = encode_pitch_identification(
            session=1, sequence=1, op="start", amplitude_nm=float("inf")
        )
        self.assertEqual(len(wire_inf), 43)
        amp_inf = struct.unpack_from("<f", wire_inf, 2 + 4 + 4 + 1 + 4 + 4 + 4)[0]
        self.assertTrue(math.isinf(amp_inf))


if __name__ == "__main__":
    unittest.main()
