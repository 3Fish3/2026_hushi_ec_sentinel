"""USART6 只读板级观察器. 仅发送快照请求, 不含电机控制或参数写入."""
from __future__ import annotations
import argparse
import binascii
import json
import math
from pathlib import Path
import struct
import sys
import time

MAX_PAYLOAD = 256
PAYLOAD_SIZE = 252  # 板级观测布局 v2: remote 段增加 ch4


def encode_frame(kind: int, sequence: int, payload: bytes = b'') -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload exceeds 256 bytes')
    raw = struct.pack('<BBHH', 1, kind, sequence, len(payload)) + payload
    raw += struct.pack('<H', binascii.crc_hqx(raw, 0xffff))
    wire = bytearray(b'\x7e')
    for value in raw:
        if value in (0x7d, 0x7e):
            wire.extend((0x7d, value ^ 0x20))
        else:
            wire.append(value)
    wire.append(0x7e)
    return bytes(wire)


class Decoder:
    def __init__(self):
        self.raw = bytearray()
        self.started = self.escaped = self.oversized = False
        self.rejected = 0

    def feed(self, data: bytes):
        frames = []
        for b in data:
            if b == 0x7e:
                if self.started and (self.raw or self.escaped or self.oversized):
                    valid = False
                    if not self.escaped and not self.oversized and len(self.raw) >= 8:
                        version, kind, seq, size = struct.unpack_from('<BBHH', self.raw)
                        crc, = struct.unpack_from('<H', self.raw, len(self.raw)-2)
                        if (version == 1 and size <= MAX_PAYLOAD and len(self.raw) == size+8
                                and crc == binascii.crc_hqx(self.raw[:-2], 0xffff)):
                            frames.append((kind, seq, bytes(self.raw[6:-2])))
                            valid = True
                    if not valid:
                        self.rejected += 1
                self.raw.clear()
                self.started = True
                self.escaped = self.oversized = False
            elif self.started and not self.oversized:
                if self.escaped:
                    b ^= 0x20
                    self.escaped = False
                elif b == 0x7d:
                    self.escaped = True
                    continue
                if len(self.raw) >= MAX_PAYLOAD + 8:
                    self.oversized = True
                else:
                    self.raw.append(b)
        return frames


def decode_snapshot(payload: bytes) -> dict:
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError(f'unsupported snapshot length: {len(payload)} (layout v2 expects {PAYLOAD_SIZE}); '
                         '客户端与固件布局不一致, 请更新工具而不是忽略长度')
    offset = 0

    def take(fmt):
        nonlocal offset
        values = struct.unpack_from('<'+fmt, payload, offset)
        offset += struct.calcsize('<'+fmt)
        return values

    header = dict(zip(('sample_ms', 'stage', 'fault', 'reset_flags', 'cycles', 'max_period_ms',
                       'skipped', 'stack_free_words', 'heap_free_bytes', 'heap_min_bytes'), take('10I')))
    header['build_id'] = take('32s')[0].split(b'\0', 1)[0].decode('ascii', errors='replace')
    remote = dict(zip(('frames', 'rejected', 'last_ms', 'ch0', 'ch1', 'ch2', 'ch3', 'ch4',
                       'left', 'right', 'online'), take('3I5h3B')))
    imu = dict(zip(('samples', 'errors', 'last_ms', 'acc_id', 'gyro_id', 'ready', 'heater',
                    'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c'), take('3I4B7f')))
    can = dict(zip(('rx_frames', 'dropped', 'ignored', 'last_id', 'dlc', 'last_ms', 'bus', 'online',
                    'speed_rad_s', 'device', 'temp_c', 'temp_valid'), take('8If3B')))
    uart3 = dict(zip(('framing', 'noise', 'overrun', 'overflows'), take('4I')))
    uart6 = dict(zip(('rx_bytes', 'tx_bytes', 'overflows', 'discarded', 'framing', 'noise',
                      'overrun', 'restarts', 'tx_errors', 'busy_rejects', 'last_hal_error', 'bad_frames',
                      'unknown_requests', 'response_drops'), take('14I')))
    assert offset == PAYLOAD_SIZE
    for name in ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c'):
        if not math.isfinite(imu[name]):
            raise ValueError(f'non-finite IMU value: {name}')
    if not math.isfinite(can['speed_rad_s']):
        raise ValueError('non-finite CAN speed')
    remote['age_ms'] = (header['sample_ms'] - remote['last_ms']) & 0xffffffff
    remote['valid'] = bool(remote['online'] and remote['frames'] and remote['age_ms'] < 100)
    imu['age_ms'] = (header['sample_ms'] - imu['last_ms']) & 0xffffffff
    imu['valid'] = bool(imu['ready'] and imu['samples'] and imu['age_ms'] < 100)
    return dict(header=header, remote=remote, imu=imu, can=can, uart3=uart3, uart6=uart6)


def render(snapshot, transport_age: float, status: str) -> str:
    fresh = snapshot is not None and transport_age < 0.5
    lines = ['EC board monitor | READ ONLY | Ctrl+C to close',
             f'Link: {status} | {"LIVE" if fresh else "STALE / NO VALID DATA"} | age={transport_age:.2f}s']
    if not fresh:
        lines.append('Measurements hidden; waiting for a fresh matched response.')
        return '\n'.join(lines)
    h, r, i, c, u = (snapshot[k] for k in ('header', 'remote', 'imu', 'can', 'uart6'))
    lines += [f'Build: {h["build_id"]!r} | uptime={h["sample_ms"]}ms | stage={h["stage"]} fault={h["fault"]} reset=0x{h["reset_flags"]:08x}',
              f'Cycles={h["cycles"]} max={h["max_period_ms"]}ms skipped={h["skipped"]} stack={h["stack_free_words"]} words',
              f'DR16: {"ONLINE" if r["valid"] else "OFFLINE"} frames={r["frames"]} rejected={r["rejected"]} age={r["age_ms"]}ms',
              (f'  CH raw: {r["ch0"]:5} {r["ch1"]:5} {r["ch2"]:5} {r["ch3"]:5} | switches L={r["left"]} R={r["right"]}'
               if r['valid'] else '  CH / switches: unavailable'),
              f'IMU: {"VALID" if i["valid"] else "STALE"} errors={i["errors"]} heater={i["heater"]}',
              (f'  accel m/s2: {i["ax"]:8.3f} {i["ay"]:8.3f} {i["az"]:8.3f} | temp={i["temp_c"]:.2f}C'
               if i['valid'] else '  accel / temperature: unavailable'),
              (f'  gyro rad/s: {i["gx"]:8.4f} {i["gy"]:8.4f} {i["gz"]:8.4f}' if i['valid'] else '  gyro: unavailable'),
              f'CAN: rx={c["rx_frames"]} drop={c["dropped"]} ignored={c["ignored"]} online={c["online"]} last=CAN{c["bus"]}/0x{c["last_id"]:03x}',
              f'UART3 errors: {snapshot["uart3"]}',
              f'UART6: rx={u["rx_bytes"]} tx={u["tx_bytes"]} overflow={u["overflows"]} restart={u["restarts"]} txerr={u["tx_errors"]}',
              f'Protocol: bad={u["bad_frames"]} unknown={u["unknown_requests"]} response_drop={u["response_drops"]}']
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='明确选择 COM 口, 不自动扫描')
    parser.add_argument('--seconds', type=float, default=0, help='0 表示直到 Ctrl+C')
    parser.add_argument('--hz', type=float, default=5, help='请求频率 1..10 Hz')
    parser.add_argument('--log', type=Path, help='JSONL 新文件, 拒绝覆盖')
    parser.add_argument('--headless', action='store_true', help='逐行 JSON, 不刷新终端面板')
    args = parser.parse_args()
    if not math.isfinite(args.hz) or not 1 <= args.hz <= 10 or not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('require 1 <= hz <= 10 and finite seconds >= 0')
    try:
        import serial
    except ImportError:
        parser.error('Install pyserial from requirements-dev.txt in a local virtual environment')
    log = args.log.open('x', encoding='utf-8') if args.log else None
    port = None
    begin = time.monotonic()
    last_rx = float('-inf')
    next_send = next_draw = begin
    seq = int(time.monotonic_ns()) & 0xffff
    pending = None
    snapshot = None
    decoder = Decoder()
    received = 0
    timeouts = 0
    status = 'opening'

    def emit(record):
        if log:
            log.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
            log.flush()
        if args.headless:
            print(json.dumps(record, ensure_ascii=False, allow_nan=False), flush=True)

    try:
        # 不自动重连: 断口先显示失效, 留给现场核实供电, 不反复探测休眠设备.
        port = serial.Serial(args.port, 115200, timeout=0.03, write_timeout=0.5,
                             rtscts=False, dsrdtr=False)
        port.reset_input_buffer()
        status = 'connected'
        while not args.seconds or time.monotonic()-begin < args.seconds:
            now = time.monotonic()
            if pending is not None and now-pending[1] >= 0.5:
                emit({'event': 'response_timeout', 'sequence': pending[0]})
                pending = None
                timeouts += 1
                if timeouts >= 3:
                    raise serial.SerialException('three missing responses; pause and check board power/firmware')
            if pending is None and now >= next_send:
                seq = (seq+1) & 0xffff
                request = encode_frame(1, seq)
                if port.write(request) != len(request):
                    raise serial.SerialTimeoutException('short request write')
                pending = (seq, now)
                next_send = now + 1/args.hz
            for kind, response_seq, payload in decoder.feed(port.read(min(max(port.in_waiting, 1), 4096))):
                if kind != 0x81 or pending is None or response_seq != pending[0]:
                    continue
                try:
                    decoded = decode_snapshot(payload)
                except ValueError as exc:
                    emit({'event': 'invalid_snapshot', 'reason': str(exc)})
                    continue
                # 收到同序号但超时的回复也不使旧数据重新变成有效.
                if time.monotonic()-pending[1] >= 0.5:
                    continue
                snapshot = decoded
                last_rx = time.monotonic()
                pending = None
                received += 1
                timeouts = 0
                emit({'event': 'snapshot', 'host_elapsed_s': last_rx-begin,
                      'sequence': response_seq, 'host_bad_frames': decoder.rejected, 'data': snapshot})
            if time.monotonic() >= next_draw:
                if not args.headless:
                    text = render(snapshot, time.monotonic()-last_rx,
                                  f'{status} / host bad frames={decoder.rejected}')
                    prefix = '\x1b[H\x1b[J' if sys.stdout.isatty() else ''
                    print(prefix+text, flush=True)
                next_draw = time.monotonic()+0.2
    except (serial.SerialException, OSError) as exc:
        emit({'event': 'disconnected', 'reason': str(exc)})
        if not args.headless:
            print(render(None, float('inf'), f'disconnected: {exc}'), flush=True)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        if port is not None:
            port.close()
        if log:
            log.close()
    return 0 if received and time.monotonic()-last_rx < 0.5 else 1


if __name__ == '__main__':
    raise SystemExit(main())
