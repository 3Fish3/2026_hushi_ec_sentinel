"""单 UART Pitch 连续流: 身份绑定, 交织分发, 失败即闭.

解包器由 sentinel_pitch_stream 拥有, 因此这里注入只做分发的假解码器;
测试证明的是异步层身份/故障/分发约定, 不是线上布局.
"""
import queue
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

RUN = 0x5EED1234
RECORD = (struct.pack('<4B7I', 1, 5, 6, 1, 0, 1, 1000, 1000, 0, 0, RUN)
          + struct.pack('<6fII', 0.1, 0.1, 0.1, 0.0, 0.04, 0.001, 0, 1))


def packet(op, run, error=0, records=()):
    """假连续流报文: 1 字节版本 + op + error + run, 之后是整条 64B 记录."""
    return struct.pack('<BBBI', 1, op, error, run) + b''.join(records)


def fake_decoder(payload, expected_run=None):
    if len(payload) < 7:
        raise ValueError('pitch stream packet truncated')
    version, op, error, run = struct.unpack_from('<BBBI', payload)
    if version != 1:
        raise ValueError('pitch stream version mismatch')
    if expected_run is not None and run != expected_run:
        raise ValueError(f'pitch stream run {run} != {expected_run}')
    if error:
        raise ValueError(f'pitch stream request rejected: {error}')
    blob = payload[7:]
    if len(blob) % 64:
        raise ValueError('pitch stream record truncated')
    return {'op': op, 'run': run, 'error': 0,
            'state': 1 if blob or op != 3 else 3}, [blob[i:i + 64] for i in range(0, len(blob), 64)]


class RecordingTransport:
    def __init__(self):
        self.sent = []

    def send(self, message):
        self.sent.append(bytes(message))

    def receive(self, timeout_s=0.0):
        return None

    def close(self):
        pass

    def frames(self, kind=None):
        decoder, found = wire.FrameDecoder(), []
        for message in self.sent:
            for byte in message:
                frame = decoder.push(byte)
                if frame is not None and (kind is None or frame[0] == kind):
                    found.append(frame)
        return found


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.transport = RecordingTransport()
        with patch('threading.Thread.start'):
            self.peer = async_mod.AsyncSentinelClient(self.transport, fast_hz=0, pitch_decoder=fake_decoder)
        self.peer._session, self.peer._connected = 12, True
        self.peer._state.update(session=12, connected=True, config_epoch=3, reference_epoch=4)
        self.sunk = []
        self.peer.set_pitch_stream_sink(lambda status, records: self.sunk.append((dict(status), list(records))))

    # -- helpers ---------------------------------------------------------
    def start(self, run=RUN, duration_us=1_000_000):
        future = self.peer.start_pitch_stream(run, duration_us)
        request = self.peer._pitch_requests.popleft()
        self.peer._begin_stream(request, time.monotonic())
        return future, list(self.peer._pitch_pending.values())[-1]

    def ack(self, pending, op=1, run=RUN):
        self.peer._pitch_ack(pending.frame_sequence, packet(op, run))

    def snap(self):
        return self.peer.snapshot()['pitch_stream']

    # -- identity & encoding ---------------------------------------------
    def test_start_binds_identity_before_first_byte(self):
        future, pending = self.start()
        self.assertFalse(future.done(), 'ACK 到达前不得宣称已确认')
        snapshot = self.snap()
        self.assertEqual(snapshot['run'], RUN)
        self.assertTrue(snapshot['active'])
        self.assertEqual(snapshot['session'], 12)
        self.assertIsNone(snapshot['confirmed_by'])
        frames = self.transport.frames(async_mod.PITCH_STREAM)
        self.assertEqual(len(frames), 1)
        kind, sequence, payload = frames[0]
        self.assertEqual(sequence, pending.frame_sequence)
        self.assertEqual(struct.unpack('<4B2I', payload),
                         (1, async_mod.PITCH_STREAM_OPS['start'], async_mod.PITCH_STREAM_PROFILE,
                          async_mod.PITCH_STREAM_DOMAIN, RUN, 1_000_000))

    def test_second_run_cannot_rebind_active_identity(self):
        self.start()
        with self.assertRaises(wire.ProtocolError):
            self.peer.start_pitch_stream(RUN + 1, 1_000_000)
        with self.assertRaises(wire.ProtocolError):
            self.peer.stop_pitch_stream(RUN + 1)

    def test_data_before_ack_confirms_capture_then_ack_is_recorded(self):
        future, pending = self.start()
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [RECORD]))
        self.assertTrue(future.done())
        self.assertEqual(future.result()['confirmed_by'], 'data')
        self.assertEqual(len(self.sunk), 1)
        self.assertEqual(self.sunk[0][1], [RECORD])
        self.ack(pending)
        snapshot = self.snap()
        self.assertEqual(snapshot['confirmed_by'], 'data')
        self.assertIsNotNone(snapshot['last_ack'])
        self.assertEqual((snapshot['packets'], snapshot['records']), (1, 1))

    def test_v3_target_ack_still_matches_while_streaming(self):
        self.start()
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [RECORD]))
        future = Future()
        request = async_mod._Request(wire.TARGETS, {'config_epoch': 3, 'reference_epoch': 4,
                                                    'setpoints': []}, future, 12)
        self.peer._begin(request, time.monotonic())
        target = list(self.peer._pending.values())[-1]
        body = struct.pack('<BBIIBB', wire.PROTOCOL_VERSION, wire.TARGETS_ACK,
                           12, target.sequence, 1, 0)
        self.peer._receive((wire.TARGETS_ACK, target.frame_sequence, body))
        snapshot = self.peer.snapshot()
        self.assertEqual(snapshot['target_rejections'], 1)
        self.assertEqual(snapshot['last_target_rejection']['sequence'], target.sequence)
        self.assertEqual(snapshot['pitch_stream']['packets'], 1, 'v3 应答不得被当作流数据')

    # -- fail closed ------------------------------------------------------
    def test_foreign_run_data_latches_immutable_fault(self):
        future, _ = self.start()
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN + 7, 0, [RECORD]))
        self.assertEqual(self.sunk, [])
        snapshot = self.snap()
        self.assertEqual(snapshot['fault'].code, 'data-rejected')
        self.assertFalse(snapshot['active'])
        self.assertIsNone(snapshot['run'])
        self.assertIn('pitch capture fault', self.peer.snapshot()['error'])
        self.assertIn('data-rejected', self.peer.snapshot()['pitch_stream_error'])
        with self.assertRaises(wire.ProtocolError):
            future.result()
        # 后续合法数据不得复活旧采集, 也不得覆盖首个故障.
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [RECORD]))
        self.assertEqual(self.sunk, [])
        self.assertEqual(self.snap()['fault'].code, 'data-rejected')

    def test_truncated_packet_is_visible_fault(self):
        self.start()
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [b'x' * 13]))
        self.assertEqual(self.snap()['fault'].code, 'data-rejected')
        self.assertEqual(self.sunk, [])

    def test_missing_decoder_is_visible_fault(self):
        with patch.object(async_mod, '_default_pitch_decoder', return_value=None), \
                patch('threading.Thread.start'):
            peer = async_mod.AsyncSentinelClient(self.transport, fast_hz=0)
        peer._session, peer._connected = 12, True
        peer._state.update(session=12, connected=True)
        peer.start_pitch_stream(RUN, 1_000_000)
        peer._begin_stream(peer._pitch_requests.popleft(), time.monotonic())
        peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [RECORD]))
        self.assertEqual(peer.snapshot()['pitch_stream']['fault'].code, 'decoder-unavailable')

    def test_sink_queue_failure_latches_fault_and_stops_targets(self):
        self.start()
        self.peer.set_target_source(lambda now, epochs: {'setpoints': []})
        self.peer._targets(time.monotonic())
        before = len(self.transport.frames(wire.TARGETS))

        def full(status, records):
            raise queue.Full('capture queue overflow')

        self.peer.set_pitch_stream_sink(full)
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [RECORD]))
        snapshot = self.snap()
        self.assertEqual(snapshot['fault'].code, 'sink')
        self.assertIn('overflow', snapshot['fault'].detail)
        self.peer._targets(time.monotonic())
        self.assertEqual(len(self.transport.frames(wire.TARGETS)), before, '故障后不得自动续期目标')

    def test_external_fault_signal_is_visible(self):
        future, _ = self.start()
        self.assertTrue(self.peer.signal_pitch_capture_fault('disk', 'writer failed: ENOSPC'))
        self.assertFalse(self.peer.signal_pitch_capture_fault('disk', 'second signal'))
        snapshot = self.snap()
        self.assertEqual((snapshot['fault'].code, snapshot['fault'].run), ('disk', RUN))
        with self.assertRaises(wire.ProtocolError):
            future.result()

    # -- session identity ------------------------------------------------
    def test_reconnect_invalidates_capture_identity_and_late_evidence(self):
        future, pending = self.start()
        self.peer._reset_session()
        snapshot = self.snap()
        self.assertIsNone(snapshot['run'])
        self.assertFalse(snapshot['active'])
        with self.assertRaises(wire.ProtocolError):
            future.result()
        self.peer._pitch_ack(pending.frame_sequence, packet(1, RUN))
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [RECORD]))
        self.assertEqual(self.sunk, [])
        self.assertFalse(self.snap()['active'])
        self.peer._connected = True
        self.peer._session = 13
        self.peer._state.update(connected=True, session=13)
        self.start(run=RUN + 1)
        self.assertEqual(self.snap()['run'], RUN + 1)

    def test_stop_ack_preserves_queued_data_until_final_marker(self):
        self.start()
        stop = self.peer.stop_pitch_stream(RUN)
        request = self.peer._pitch_requests.popleft()
        self.peer._begin_stream(request, time.monotonic())
        self.assertEqual(struct.unpack('<4B2I', self.transport.frames(async_mod.PITCH_STREAM)[-1][2])[1],
                         async_mod.PITCH_STREAM_OPS['stop'])
        pending = list(self.peer._pitch_pending.values())[-1]
        self.ack(pending, op=async_mod.PITCH_STREAM_OPS['stop'])
        self.assertTrue(stop.done())
        self.assertTrue(self.snap()['active'])
        self.assertTrue(self.snap()['draining'])
        self.assertEqual(self.snap()['run'], RUN)
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [RECORD]))
        self.assertEqual(self.sunk[0][1], [RECORD], 'STOP ACK 后仍须接收已排队的样本')
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN))
        self.assertTrue(self.snap()['finished'])
        self.assertFalse(self.snap()['active'])
        packets = len(self.sunk)
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN, 0, [RECORD]))
        self.assertEqual(len(self.sunk), packets, 'final marker 后不能重新开启记录')
        self.assertIsNone(self.snap()['fault'])

    def test_clean_final_marker_allows_only_a_larger_next_run(self):
        original = self.peer._pitch_decoder

        def decoder(payload, expected_run=None):
            status, records = original(payload, expected_run)
            status.update(dropped=0, buffered=0, generated=0)
            return status, records

        self.peer._pitch_decoder = decoder
        _, pending = self.start()
        self.ack(pending)
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN))
        self.assertTrue(self.snap()['finished'])
        for run in (RUN, RUN - 1):
            with self.assertRaises(wire.ProtocolError):
                self.peer.start_pitch_stream(run, 1000000)
        self.start(run=RUN + 1)
        self.assertEqual(self.snap()['run'], RUN + 1)

    def test_final_marker_with_remaining_buffer_does_not_allow_restart(self):
        original = self.peer._pitch_decoder

        def decoder(payload, expected_run=None):
            status, records = original(payload, expected_run)
            status.update(dropped=0, buffered=1, generated=0)
            return status, records

        self.peer._pitch_decoder = decoder
        _, pending = self.start()
        self.ack(pending)
        self.peer._pitch_data(packet(async_mod.PITCH_STREAM_DATA_OP, RUN))
        self.assertTrue(self.snap()['finished'])
        self.assertFalse(self.peer._pitch_pending)
        with self.assertRaises(wire.ProtocolError):
            self.peer.start_pitch_stream(RUN + 1, 1000000)

    def test_sample_gap_clears_renewal_without_auto_resume(self):
        self.start()
        record = bytearray(RECORD)
        struct.pack_into('<I', record, 4, 1)
        self.peer._pitch_data(packet(3, RUN, 0, [bytes(record)]))
        self.assertEqual(self.snap()['fault'].code, 'sequence-gap')
        self.peer._pitch_data(packet(3, RUN, 0, [RECORD]))
        self.assertEqual(self.sunk, [])

    def test_capture_packet_stale_latches_inside_hold_budget(self):
        _, pending = self.start()
        self.ack(pending)
        self.peer._expire_stream(self.snap()['started_at_monotonic'] + 0.051)
        self.assertEqual(self.snap()['fault'].code, 'stream-stale')

    def test_final_generated_count_must_match_received_records(self):
        self.start()
        self.peer._pitch_decoder = lambda body, expected_run: (
            {'op': 3, 'run': expected_run, 'error': 0, 'state': 3,
             'generated': 2, 'dropped': 0}, [RECORD])
        self.peer._pitch_data(b'fixture')
        self.assertEqual(self.snap()['fault'].code, 'incomplete-final')
        self.assertEqual(self.sunk, [])

    def test_start_not_acknowledged_faults(self):
        future, pending = self.start()
        self.peer._expire_stream(pending.deadline + 0.001)
        self.assertIsInstance(future.exception(), TimeoutError)
        self.assertEqual(self.snap()['fault'].code, 'start-timeout')


class IdentificationWiringTests(unittest.TestCase):
    def setUp(self):
        self.transport = RecordingTransport()
        with patch('threading.Thread.start'):
            self.peer = async_mod.AsyncSentinelClient(self.transport, fast_hz=0, pitch_decoder=fake_decoder)
        self.peer._session, self.peer._connected = 12, True
        self.peer._state.update(session=12, connected=True, config_epoch=3, reference_epoch=4)

    def dispatch_read(self):
        self.peer._begin(self.peer._reads.popleft(), time.monotonic())
        return list(self.peer._pending.values())[-1]

    def begin_change(self):
        request = self.peer._change
        if request is None:
            self.fail('变更车道为空')
        self.peer._change = None
        self.peer._change_active = False
        self.peer._begin(request, time.monotonic())
        return list(self.peer._pending.values())[-1]

    def ident_reply(self, pending, state=2, fault=0, run_id=RUN):
        body = bytes([wire.PROTOCOL_VERSION, wire.PITCH_IDENTIFICATION_STATUS]) + struct.pack(
            '<II5BIff', 12, pending.sequence, 0, 0, 1, state, fault, run_id, 0.04, 0.0398)
        self.peer._receive((wire.PITCH_IDENTIFICATION_STATUS, pending.frame_sequence, body))

    def test_query_is_read_and_start_is_mutation(self):
        query = self.peer.pitch_identification('query')
        self.assertTrue(self.peer._reads)
        self.assertIsNone(self.peer._change)
        self.ident_reply(self.dispatch_read())
        self.assertTrue(query.done())
        self.assertEqual(query.result()['state_name'], 'square')
        start = self.peer.pitch_identification('start', run_id=RUN, amplitude_nm=0.04,
                                               half_period_us=10000, baseline_us=5000, cycles=3,
                                               final_hold_us=5000)
        self.begin_change()
        pending = list(self.peer._pending.values())[-1]
        self.ident_reply(pending)
        self.assertEqual(start.result()['run_id'], RUN)
        frames = self.transport.frames(wire.PITCH_IDENTIFICATION)
        self.assertEqual([len(frame[2]) for frame in frames], [19, 43], 'query 19B / start 43B 沿用既有 v3 编码')
        self.assertEqual((frames[0][2][10], frames[1][2][10]), (0, 1))
        self.assertEqual(self.peer.snapshot()['last_change']['run_id'], RUN)

    def test_cancel_and_reset_are_mutations_and_epochs_follow_snapshot(self):
        for op, code in (('cancel', 2), ('reset', 3)):
            self.peer._change = None
            self.peer._change_active = False
            self.peer.pitch_identification(op)
            request = self.peer._change
            if request is None:
                self.fail(f'{op} 必须占变更车道')
            self.assertTrue(request.mutation)
            self.assertEqual((request.values['op'], request.values['config_epoch'],
                              request.values['reference_epoch']), (code, 3, 4))

    def test_real_decoder_is_the_default(self):
        payload = struct.pack('<4B7I', 1, 3, 0, 1, RUN, 1, 0, 0, 1000, 2000, 0x52415701)
        record = struct.pack('<4B7I', 1, 5, 6, 0, 0, 0, 1000, 2000, 0, 0xffffffff, RUN) + \
            struct.pack('<6fII', 0.1, 0.1, 0.1, 0.0, 0.04, 0.001, 0, 0)
        payload += record
        decoder = async_mod._default_pitch_decoder()
        self.assertIsNotNone(decoder, 'sentinel_pitch_stream 应提供默认解码器')
        self.peer._pitch_decoder = decoder
        sunk = []
        self.peer.set_pitch_stream_sink(lambda status, records: sunk.append((dict(status), list(records))))
        future = self.peer.start_pitch_stream(RUN, 1_000_000)
        self.peer._begin_stream(self.peer._pitch_requests.popleft(), time.monotonic())
        self.peer._pitch_data(payload)
        self.assertTrue(future.done())
        self.assertEqual(future.result()['confirmed_by'], 'data')
        self.assertEqual(len(sunk), 1)
        self.assertEqual(len(sunk[0][1]), 1)
        self.assertEqual(len(sunk[0][1][0]), 64)
        self.assertEqual(sunk[0][0]['run'], RUN)


if __name__ == '__main__':
    unittest.main()
