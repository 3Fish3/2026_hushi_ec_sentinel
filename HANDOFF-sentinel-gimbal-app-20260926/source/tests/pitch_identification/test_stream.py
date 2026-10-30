"""Pitch 连续控制流解码器离线契约: 纯字节, 无串口/无发送/无执行器授权."""
import ast
import struct
import subprocess
import sys
import unittest
from pathlib import Path
from typing import Any

import sentinel_pitch_stream as pitch

SOURCE = Path(__file__).resolve().parents[2]


def _codec_path():
    """原生编解码器路径: 优先取命令行实参, 否则回退到默认构建目录."""
    if len(sys.argv) > 1 and Path(sys.argv[1]).is_file():
        return sys.argv.pop(1)
    return str(SOURCE / 'build/host-tests/ec_capture_stream_tests')


CODEC = _codec_path()
KIND_CAN, KIND_CONTROL = 4, 5
PITCH_RUN = 7


def header(op=3, error=0, state=1, run=PITCH_RUN, generated=4, dropped=0, buffered=1,
           start_us=1000, end_us=6000, layout=0x52415701, version=1):
    return struct.pack('<4B7I', version, op, error, state, run, generated, dropped, buffered,
                       start_us, end_us, layout)


def record(kind, stream, run=PITCH_RUN, seq=1, flags=4, data=None, version=1):
    body = struct.pack('<8I', *data) if data is not None else bytes(32)
    return struct.pack('<4B7I', version, kind, stream, flags, seq, seq, 1000, 1000, 0,
                       0xffffffff, run) + body


def can_record(bus=2, dlc=8, can_id=0x011, run=PITCH_RUN, stream=9, kind=KIND_CAN, **kwargs):
    words = [bus | (dlc << 8) | (can_id << 16), 0x11223344, 0x55667788, 0, 0, 0, 0, 0]
    return record(kind, stream, run=run, data=words, **kwargs)[:44]


def control_record(run=PITCH_RUN, stream=6, kind=KIND_CONTROL, word6=0, word7=0, **kwargs):
    values = struct.unpack('<6I', struct.pack('<6f', 0.1, 0.2, 0.3, -0.05, float('nan'), 0.005))
    return record(kind, stream, run=run, data=list(values) + [word6, word7], **kwargs)


class HeaderTests(unittest.TestCase):
    def test_header_only_operations(self):
        for op in (0, 1, 2):
            status, records = pitch.decode_pitch_stream_packet(header(op=op))
            self.assertEqual(status['op'], op)
            self.assertEqual(status['layout_id'], pitch.LAYOUT)
            self.assertEqual(records, [])
            self.assertEqual(set(status), set(pitch.HEADER_KEYS))

    def test_idle_stream_header(self):
        packet = header(op=0, state=0, run=0, generated=0, buffered=0, start_us=0, end_us=0)
        status, records = pitch.decode_pitch_stream_packet(packet)
        self.assertEqual((status['state'], status['run'], records), (0, 0, []))

    def test_reject_header_violations(self):
        cases = [
            header(version=2), header(layout=0x52415702), header(state=4), header(op=4),
            header(error=1), header(error=4), header()[:31],
        ]
        for packet in cases:
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(packet)

    def test_reject_impossible_queue_counters(self):
        for kwargs in ({'buffered': 513}, {'buffered': 5, 'generated': 4},
                       {'state': 3, 'buffered': 1}, {'state': 2, 'buffered': 0}):
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(header(**kwargs))

    def test_active_stream_requires_run_and_ambiguous_time_rejected(self):
        with self.assertRaises(ValueError):
            pitch.decode_pitch_stream_packet(header(state=1, run=0))
        with self.assertRaises(ValueError):
            pitch.decode_pitch_stream_packet(header(start_us=0, end_us=0x80000000))
        # 跨 2^32 回绕但仍在半个回绕窗口内 -> 合法.
        status, _ = pitch.decode_pitch_stream_packet(
            header(op=0, start_us=0xfffff000, end_us=0x00001000))
        self.assertEqual(status['run'], PITCH_RUN)

    def test_trailing_bytes_only_allowed_for_data_frames(self):
        for op in (0, 1, 2):
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(header(op=op) + bytes(4))
        with self.assertRaises(ValueError):
            pitch.decode_pitch_stream_packet(header(op=3))


class RecordTests(unittest.TestCase):
    def test_control_and_can_records_are_rebuilt_to_64_bytes(self):
        packet = header(buffered=0) + can_record() + control_record(word6=0x1f000101, word7=0x99)
        status, records = pitch.decode_pitch_stream_packet(packet, PITCH_RUN)
        self.assertEqual(status['run'], PITCH_RUN)
        self.assertEqual(len(records), 2)
        self.assertTrue(all(len(r) == pitch.RECORD_BYTES for r in records))
        self.assertEqual(records[0][44:], bytes(20))  # 紧凑 CAN 记录尾部按协议零填充
        self.assertEqual(records[0][32:36], struct.pack('<I', 2 | (8 << 8) | (0x011 << 16)))
        self.assertEqual(records[0][1], KIND_CAN)
        self.assertEqual((records[1][1], records[1][2]), (KIND_CONTROL, pitch.PITCH_DOMAIN))
        self.assertEqual(struct.unpack_from('<2I', records[1], 56), (0x1f000101, 0x99))
        self.assertTrue(all(struct.unpack_from('<I', r, 28)[0] == PITCH_RUN for r in records))

    def test_three_record_packet_is_the_224_byte_ceiling(self):
        packet = header(op=3) + control_record() * 3
        self.assertEqual(len(packet), 224)
        self.assertEqual(pitch.MAX_PACKET, 224)
        _, records = pitch.decode_pitch_stream_packet(packet)
        self.assertEqual(len(records), 3)

    def test_fourth_record_exceeds_packet_budget(self):
        with self.assertRaises(ValueError):
            pitch.decode_pitch_stream_packet(header(op=3) + control_record() * 4)

    def test_reject_imu_and_foreign_streams(self):
        for packet in (can_record(kind=1, stream=0), can_record(kind=2, stream=0),
                       can_record(kind=3, stream=0), can_record(stream=4, can_id=0x208),
                       control_record(stream=7), control_record(kind=6),
                       can_record(kind=KIND_CONTROL, stream=9),
                       control_record(kind=KIND_CAN, stream=6)):
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(header(op=3) + packet)

    def test_reject_can_identity(self):
        for kwargs in ({'bus': 1}, {'dlc': 7}, {'can_id': 0x010}, {'can_id': 0x211}):
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(header(op=3) + can_record(**kwargs))

    def test_reject_record_integrity(self):
        cases = [
            header(op=3) + control_record(version=2),
            header(op=3) + control_record(flags=8),
            header(op=3) + control_record(run=8),
            header(op=3) + control_record()[:40],          # 控制记录截断
            header(op=3) + can_record()[:40],              # 紧凑记录截断
            header(op=3) + control_record() + bytes(1),    # 尾部零散字节
        ]
        for packet in cases:
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(packet)

    def test_expected_run_must_match_exactly(self):
        packet = header(op=3) + control_record()
        for expected in (8, 0, -1):
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(packet, expected)
        self.assertEqual(pitch.decode_pitch_stream_packet(packet, PITCH_RUN)[0]['run'], PITCH_RUN)

    def test_reject_non_bytes_payload(self):
        # 运行时类型校验: 显式以 Any 传入非字节对象, 只验证拒绝行为, 不做静态绕行.
        payloads: list[Any] = ['text', None, [1, 2, 3]]
        for payload in payloads:
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(payload)

    def test_final_stopped_header_frame_is_accepted(self):
        status, records = pitch.decode_pitch_stream_packet(header(op=3, state=3, buffered=0))
        self.assertEqual((status['op'], status['state'], status['buffered']), (3, 3, 0))
        self.assertEqual(records, [])
        # 同一空数据帧在记录中或冻结积压时均不得被当成合法数据帧.
        for state, buffered in ((1, 0), (2, 1)):
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(header(op=3, state=state, buffered=buffered))


class GoldenInteropTests(unittest.TestCase):
    """真实原生字节: 由 --pitch-golden 产生, 而非手工同构 mock."""

    @staticmethod
    def frames():
        if not Path(CODEC).is_file():
            raise AssertionError(f'原生编解码器不可用: {CODEC}')
        result = subprocess.run([CODEC, '--pitch-golden'], capture_output=True, text=True, timeout=120)
        if result.returncode != 0:
            raise AssertionError(f'--pitch-golden 失败: {result.returncode} {result.stderr}')
        return [bytes.fromhex(line) for line in result.stdout.split()]

    def test_mixed_packet_and_final_empty_frame(self):
        frames = self.frames()
        # 混合包 = 32B 头 + CAN 44B + control 64B + CAN 44B; 其余 4 帧均为 32B 头部.
        self.assertEqual([len(frame) for frame in frames], [32, 32 + 44 + 64 + 44, 32, 32, 32])

        start, records = pitch.decode_pitch_stream_packet(frames[0], 4242)
        self.assertEqual((start['op'], start['state'], start['error'], start['generated']), (1, 1, 0, 0))
        self.assertEqual(records, [])

        data, records = pitch.decode_pitch_stream_packet(frames[1], 4242)
        self.assertEqual((data['op'], data['state'], data['generated'], data['dropped']), (3, 1, 3, 0))
        self.assertEqual([len(r) for r in records], [64, 64, 64])
        self.assertEqual([(r[1], r[2]) for r in records], [(KIND_CAN, 9), (KIND_CONTROL, 6), (KIND_CAN, 9)])
        self.assertEqual([struct.unpack_from('<I', r, 12)[0] for r in records], [10000, 10000, 15000])
        self.assertEqual(struct.unpack_from('<I', records[0], 32)[0], 2 | (8 << 8) | (0x011 << 16))
        self.assertEqual(struct.unpack_from('<I', records[0], 36)[0], 0x11223344)
        self.assertEqual(struct.unpack_from('<I', records[0], 40)[0], 0x55667788)
        self.assertEqual(records[0][44:], bytes(20))
        self.assertEqual([struct.unpack_from('<I', records[1], 32 + 4 * i)[0] for i in range(8)],
                         [0xa0000000 + i for i in range(8)])

        status, records = pitch.decode_pitch_stream_packet(frames[2], 4242)
        self.assertEqual((status['op'], status['generated'], status['dropped'], status['buffered']), (0, 3, 0, 0))
        self.assertEqual(records, [])

        stop, records = pitch.decode_pitch_stream_packet(frames[3], 4242)
        self.assertEqual((stop['op'], stop['state'], stop['generated'], stop['buffered']), (2, 3, 3, 0))
        self.assertEqual(records, [])

        final, records = pitch.decode_pitch_stream_packet(frames[4], 4242)
        self.assertEqual((final['op'], final['state'], final['buffered'], final['generated']), (3, 3, 0, 3))
        self.assertEqual(records, [])

    def test_real_final_frame_is_never_empty_data_while_active(self):
        final = self.frames()[4]
        self.assertEqual(len(final), 32)
        for state, buffered in ((1, 0), (2, 1), (0, 0)):
            tampered = bytearray(final)
            tampered[3] = state
            struct.pack_into('<I', tampered, 16, buffered)
            with self.assertRaises(ValueError):
                pitch.decode_pitch_stream_packet(bytes(tampered))


class AuthorityTests(unittest.TestCase):
    def test_decoder_imports_only_struct(self):
        module_file = pitch.__file__ or ''
        self.assertTrue(module_file)
        tree = ast.parse(Path(module_file).read_text(encoding='utf-8'))
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split('.')[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split('.')[0])
        self.assertEqual(imported, {'__future__', 'struct'})

    def test_decoder_exposes_no_io_or_actuation_surface(self):
        public = {name for name, value in vars(pitch).items()
                  if not name.startswith('_') and callable(value)}
        self.assertEqual(public, {'decode_pitch_stream_packet'})
        for name in vars(pitch):
            self.assertFalse(name.lower().startswith(
                ('send', 'write', 'open', 'connect', 'serial', 'target')))


if __name__ == '__main__':
    unittest.main()
