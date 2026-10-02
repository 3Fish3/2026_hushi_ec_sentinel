"""USART1 字节流对端工具: 发送带序号的二进制块并核对应答. 不是上位机协议, 不连接电机."""
from __future__ import annotations

import argparse
import sys
import time


def xor8(data: bytes) -> int:
    acc = 0
    for byte in data:
        acc ^= byte
    return acc


def frame(seq: int, payload: bytes) -> bytes:
    header = bytes([0xA5, 0x5A, seq & 0xFF, (seq >> 8) & 0xFF, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    body = header + payload
    return body + bytes([xor8(body)])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--seconds', type=float, default=2.0)
    args = parser.parse_args()
    try:
        import serial  # type: ignore
    except ImportError:
        print('pyserial is required on the host PC that talks to USART1', file=sys.stderr)
        return 2
    payloads = [b'\x00\xff\xa5\x5a', b'x' * 64, bytes(range(256)), b'\x00' * 400]
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        sent = bytearray()
        for seq, payload in enumerate(payloads):
            block = frame(seq, payload)
            ser.write(block)
            sent.extend(block)
            time.sleep(0.05)
        deadline = time.monotonic() + args.seconds
        got = bytearray()
        while time.monotonic() < deadline and len(got) < len(sent):
            got.extend(ser.read(len(sent) - len(got)))
        print(f'sent={len(sent)} echoed={len(got)} match={got == sent}')
        return 0 if got == sent else 1


if __name__ == '__main__':
    raise SystemExit(main())
