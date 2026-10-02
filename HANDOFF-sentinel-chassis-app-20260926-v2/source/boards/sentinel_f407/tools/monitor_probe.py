"""USART6 只读协议有限回归. --break-line 额外产生一次 30 ms UART Break 验证恢复.
仅用于获准的无运动台架, 不包含电机控制/任意 CAN 发送. 错误即停, 不自动重试.
"""
import argparse
import json
import time
import serial
import board_monitor as protocol


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--break-line', action='store_true')
    parser.add_argument('--burst-requests', type=int, default=0, help='0..200 个只读请求验证限频')
    args = parser.parse_args()
    if not 0 <= args.burst_requests <= 200:
        parser.error('burst-requests must be 0..200')
    with serial.Serial(args.port, 115200, timeout=0.03, write_timeout=0.5) as port:
        port.reset_input_buffer()
        decoder = protocol.Decoder()
        sequence = 1000
        def receive(seq):
            end = time.monotonic()+0.5
            while time.monotonic() < end:
                for kind, reply_seq, payload in decoder.feed(port.read(min(max(port.in_waiting, 1), 4096))):
                    if kind == 0x81 and reply_seq == seq:
                        data = protocol.decode_snapshot(payload)
                        if data['header']['fault']:
                            raise RuntimeError('MCU fault; stop test')
                        return data
            raise RuntimeError('Response missing; pause for power/firmware investigation')
        def query():
            nonlocal sequence
            time.sleep(0.08)
            sequence += 1
            port.write(protocol.encode_frame(1, sequence))
            return receive(sequence)
        def check(condition, message):
            if not condition:
                raise RuntimeError(message)
        baseline = query()
        print(json.dumps({'case': 'baseline', 'data': baseline}), flush=True)
        broken = bytearray(protocol.encode_frame(1, 2000))
        broken[3] ^= 1
        port.write(broken)
        after_crc = query()
        check(after_crc['uart6']['bad_frames'] > baseline['uart6']['bad_frames'], 'CRC rejection missing')
        port.write(protocol.encode_frame(0x10, 2001, b'unknown'))
        after_unknown = query()
        check(after_unknown['uart6']['unknown_requests'] > after_crc['uart6']['unknown_requests'], 'Unknown command accepted')
        port.write(b'\x7e' + b'x'*300 + b'\x7e')
        after_long = query()
        check(after_long['uart6']['bad_frames'] > after_unknown['uart6']['bad_frames'], 'Oversize rejection missing')
        time.sleep(0.08)
        sequence += 1
        split = protocol.encode_frame(1, sequence)
        for part in (split[:3], split[3:7], split[7:]):
            port.write(part)
            time.sleep(0.02)
        after_split = receive(sequence)
        print(json.dumps({'case': 'crc_unknown_oversize_split', 'data': after_split}), flush=True)
        if args.burst_requests:
            burst = b''.join(protocol.encode_frame(1, 3000+i) for i in range(args.burst_requests))
            port.write(burst)
            # 等待有限 UART 数据发送完成, 不在物理输出尚未结束时清缓冲.
            time.sleep(len(burst)*10/115200 + 0.1)
            after_burst = query()
            check(after_burst['uart6']['overflows'] == after_split['uart6']['overflows'], 'RX overflow during request burst')
            if args.burst_requests > 10:
                check(after_burst['uart6']['response_drops'] > after_split['uart6']['response_drops'], 'Response rate limit not observed')
            print(json.dumps({'case': 'bounded_request_burst', 'data': after_burst}), flush=True)
        if args.break_line:
            port.send_break(duration=0.03)
            after_break = query()
            check(after_break['uart6']['restarts'] > after_split['uart6']['restarts'], 'RX recovery not observed')
            check(after_break['uart6']['framing'] > after_split['uart6']['framing'], 'Break did not produce framing error')
            print(json.dumps({'case': 'break_recovery', 'data': after_break}), flush=True)
        print(json.dumps({'result': 'pass'}), flush=True)


if __name__ == '__main__':
    main()
