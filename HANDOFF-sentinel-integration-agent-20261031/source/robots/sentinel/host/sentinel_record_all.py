#!/usr/bin/env python3
"""手动启动 APP 只读记录：按回车、倒计时，再记录完整 status 0/1/2 页。

不申请控制权、不发目标、不写参数、不操作 SWD。
Ctrl+C 只停止记录，不会让机器人停车；停车需使用遥控撤运动或硬断电。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sentinel_client as wire

PAGES = (0, 1, 2)
DEFAULT_PORT = '/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148384-if02'


def read_status(client, page):
    reply = client.status(page)
    if reply.get('envelope') == 'error' or reply.get('page') != page:
        raise wire.ProtocolError(f'状态页 {page} 无效: {reply}')
    return reply


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=DEFAULT_PORT, help='APP 业务串口，921600 baud')
    parser.add_argument('--seconds', type=float, default=30.0, help='记录秒数，默认30')
    parser.add_argument('--period', type=float, default=0.1, help='每组三页的轮询周期，默认0.1秒')
    parser.add_argument('--countdown', type=int, default=5, help='开始前倒计时秒数，默认5')
    parser.add_argument('--output', type=Path, help='输出文件，默认 ~/captures/app-all-<UTC时间>.jsonl')
    args = parser.parse_args()
    if (not all(math.isfinite(v) and v > 0 for v in (args.seconds, args.period))
            or args.countdown < 0):
        parser.error('seconds/period 必须是有限正数，countdown 不能为负')
    output = args.output or Path.home() / 'captures' / (
        'app-all-' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '.jsonl')
    if output.exists():
        parser.error(f'拒绝覆盖已有文件：{output}')

    print(f'串口：{args.port}\n时长：{args.seconds:g} 秒\n文件：{output}', flush=True)
    print('只读记录，不改变运动。Ctrl+C 停止记录，不负责停车。', flush=True)
    try:
        input('准备好后按回车；连接成功并倒计时结束后才开始计时：')
    except (KeyboardInterrupt, EOFError):
        print('\n已取消，未开始记录。', flush=True)
        return 130

    transport = None
    counts = {page: 0 for page in PAGES}
    errors = 0
    samples = 0
    started = None
    interrupted = False
    try:
        transport = wire.SerialTransport(args.port, 921600)
        client = wire.SentinelClient(transport)
        hello = client.hello()
        for page in PAGES:
            read_status(client, page)
        print('连接成功，三页状态均可读取。', flush=True)
        for remaining in range(args.countdown, 0, -1):
            print(f'倒计时 {remaining} …', flush=True)
            until = time.monotonic() + 1.0
            while time.monotonic() < until:
                # 倒计时期间保持业务会话活跃，不发送运动目标。
                read_status(client, 0)
                time.sleep(min(0.1, max(0.0, until - time.monotonic())))

        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as file:
            started = time.monotonic()
            file.write(json.dumps({'event': 'header', 'port': args.port,
                'started_at': time.time(), 'seconds': args.seconds,
                'period_s': args.period, 'pages': list(PAGES),
                'role': 'readonly_full_snapshot', 'hello': hello}, ensure_ascii=False) + '\n')
            file.flush()
            print(f'\n========== 开始记录：{args.seconds:g} 秒，现在操作 ==========\a', flush=True)
            next_report = 5.0
            try:
                while time.monotonic() - started < args.seconds:
                    loop = time.monotonic()
                    record = {'t_ms': int((loop - started) * 1000), 'wall': time.time()}
                    for page in PAGES:
                        try:
                            record[f'page{page}'] = read_status(client, page)
                            counts[page] += 1
                        except Exception as exc:
                            errors += 1
                            record[f'page{page}_error'] = f'{type(exc).__name__}: {exc}'
                    file.write(json.dumps(record, ensure_ascii=False) + '\n')
                    file.flush()
                    samples += 1
                    elapsed = time.monotonic() - started
                    if elapsed >= next_report:
                        print(f'已录 {elapsed:.1f}/{args.seconds:g} 秒 | {samples} 组 | 错误 {errors}', flush=True)
                        next_report = elapsed + 5.0
                    time.sleep(max(0.0, args.period - (time.monotonic() - loop)))
            except KeyboardInterrupt:
                interrupted = True
            summary = {'event': 'summary', 'elapsed_s': round(time.monotonic() - started, 3),
                'samples': samples, 'page_counts': counts, 'errors': errors,
                'interrupted': interrupted, 'output': str(output)}
            file.write(json.dumps(summary, ensure_ascii=False) + '\n')
        print('\n========== 记录已停止；机器人运动状态未改变 ==========\a', flush=True)
        print(json.dumps(summary, ensure_ascii=False), flush=True)
        return 130 if interrupted else 0 if errors == 0 and min(counts.values()) > 0 else 1
    except KeyboardInterrupt:
        print('\n已取消，未开始正式记录。', flush=True)
        return 130
    except Exception as exc:
        print(f'记录失败：{type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        if started is not None:
            print(f'保留已录部分数据：{output}', file=sys.stderr, flush=True)
        return 1
    finally:
        if transport is not None:
            transport.close()


if __name__ == '__main__':
    sys.exit(main())
