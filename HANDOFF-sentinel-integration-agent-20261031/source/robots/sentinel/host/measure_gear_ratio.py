#!/usr/bin/env python3
"""方法B：由 C620 转子编码器累计量反推 M3508 减速比。

只读取编码器帧，不使能、不发目标、不写参数。使用前必须断开动力并确认转子能自由转动。

用法（在 Jetson 上，cature 角色固件，观察口 1000000）：
    python3 measure_gear_ratio.py --port /dev/ttyACM1 --output ~/captures/gearratio

输出每轮的原始计数变化与推算减速比；减速比 = 转子圈数 / 输出轴圈数。
前提：操作者用外部标记把输出轴精确转过 --turns 圈（默认 10 圈），并在提示后开始/结束。
"""
from __future__ import annotations
import argparse
import json
import math
import time
from pathlib import Path

import serial

WHEELS = {'LF': 0x203, 'RF': 0x201, 'LB': 0x204, 'RB': 0x202}
ENCODER_LINES = 8192


def decode_uart_frames(buffer: bytearray):
    """最小 UART 帧解析，只用于取出原始记录 payload。"""
    frames = []
    while True:
        start = buffer.find(0x7E)
        if start < 0 or len(buffer) - start < 3:
            break
        end = buffer.find(0x7E, start + 1)
        if end < 0:
            break
        raw = bytearray()
        i = start + 1
        while i < end:
            byte = buffer[i]
            if byte == 0x7D:
                i += 1
                if i >= end:
                    break
                raw.append(buffer[i] ^ 0x20)
            else:
                raw.append(byte)
            i += 1
        del buffer[:end + 1]
        if len(raw) < 8 or raw[0] != 1:
            continue
        ftype = raw[1]
        size = raw[4] | (raw[5] << 8)
        payload = bytes(raw[6:6 + size]) if len(raw) >= 6 + size else b''
        frames.append((ftype, payload))
    return frames


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode_frame(ftype, seq, payload=b''):
    raw = bytes([1, ftype]) + seq.to_bytes(2, 'little') + len(payload).to_bytes(2, 'little') + payload
    raw += crc16(raw).to_bytes(2, 'little')
    out = bytearray(b'\x7e')
    for b in raw:
        if b in (0x7E, 0x7D):
            out += bytes([0x7D, b ^ 0x20])
        else:
            out.append(b)
    out.append(0x7E)
    return bytes(out)


def request(op, run=0, profile=0, arg=0, domain=0):
    return bytes([1, op, profile, domain]) + run.to_bytes(4, 'little') + arg.to_bytes(4, 'little')


class RawLink:
    def __init__(self, port, baud):
        self.port = serial.Serial(port, baud, timeout=0.05)
        self.buf = bytearray()
        self.seq = 0

    def exchange(self, payload, ftype=0x31, expect=0xB1, timeout=1.0):
        self.seq = (self.seq + 1) & 0xFFFF
        self.port.reset_input_buffer()
        self.port.write(encode_frame(ftype, self.seq, payload))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.buf += self.port.read(512)
            frames = decode_uart_frames(self.buf)
            for t, body in frames:
                if t == expect:
                    return body
        raise TimeoutError('no reply from capture role')

    def close(self):
        self.port.close()


def first_encoder(frame: bytes):
    return (frame[0] << 8) | frame[1]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', required=True)
    p.add_argument('--baud', type=int, default=1000000)
    p.add_argument('--output', type=Path)
    p.add_argument('--turns', type=float, default=10.0, help='输出轴计划转过的圈数')
    p.add_argument('--seconds', type=float, default=0.0,
                   help='>0 时按期采集；0 表示手动开始/结束（回车）')
    a = p.parse_args()

    print('!! 先断动力，确认四个轮子可用手自由转动；pitch 需承托。')
    input('确认已断电且能安全手转后，按回车开始记录：')

    link = RawLink(a.port, a.baud)
    run = 20261109
    try:
        status = link.exchange(request(1, run, 1, int(a.seconds * 1e6) if a.seconds else 2000000))
        if status[2] != 0:
            raise RuntimeError('capture rejected start, error=%d' % status[2])
        print('记录已开始。请把输出轴精确转过 %.1f 圈，然后按回车结束。' % a.turns)
        if a.seconds > 0:
            time.sleep(a.seconds)
        else:
            input()
        reply = link.exchange(request(2, run))
        total = reply[10] | (reply[11] << 8)
        print('窗口记录数：%d' % total)
        records = []
        while len(records) < total:
            page = link.exchange(request(3, run, arg=len(records)))
            body = page[32:]
            for i in range(0, len(body) - 63, 64):
                records.append(body[i:i + 64])
            if not body:
                break
    finally:
        link.close()

    if a.output:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        with a.output.open('xb') as f:
            for r in records:
                f.write(r)

    # kind==4(CAN), stream 对应 current_harness 顺序；data[0] 含 bus/dlc/id
    counts = {name: [] for name in WHEELS}
    for r in records:
        kind, stream = r[1], r[2]
        if kind != 4:
            continue
        info = int.from_bytes(r[32:36], 'little')
        can_id = info >> 16
        for name, want in WHEELS.items():
            if can_id == want:
                counts[name].append((r[12] | (r[13] << 8) | (r[14] << 16) | (r[15] << 24),
                                     first_encoder(r[32:])))
    print('\n结果（需人工确认转数一致）：')
    for name in WHEELS:
        rows = counts[name]
        if len(rows) < 3:
            print('  %s: 样本不足(%d)' % (name, len(rows)))
            continue
        enc = [e for _, e in rows]
        unwrapped = [enc[0]]
        for e in enc[1:]:
            d = e - (unwrapped[-1] % ENCODER_LINES)
            while d > ENCODER_LINES / 2:
                d -= ENCODER_LINES
            while d < -ENCODER_LINES / 2:
                d += ENCODER_LINES
            unwrapped.append(unwrapped[-1] + d)
        rotor_turns = (unwrapped[-1] - unwrapped[0]) / ENCODER_LINES
        ratio = abs(rotor_turns) / a.turns
        print('  %s: 转子 %.3f 圈 / 输出轴 %.1f 圈 = %.4f' % (name, rotor_turns, a.turns, ratio))
    print('\n参考：配置当前 chassis_ratio=19；CAD 268/17=15.7647。')
    print('注意：手转速度过低时点数可能不足，重新采集并转快一点。')


if __name__ == '__main__':
    main()
