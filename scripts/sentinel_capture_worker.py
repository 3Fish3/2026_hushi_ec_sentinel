"""Headless selected-source collector; SIGTERM requests recorder stop, not motor stop."""
import argparse
import importlib
import json
import hashlib
import math
from pathlib import Path
import queue
import signal
import sys
import threading
import time


def inspect_action(client, log, action):
    """调用原版客户端的方法, 标签与暂停语义与原 Tk 按钮保持一致."""
    kind = action['action']
    if kind == 'focus':
        client.focus_page = action['page']
    elif kind == 'mark':
        if not log.file or not log.enabled:
            return '标签未保存: 请指定日志文件并恢复记录'
        client.request_mark(client.focus_page if client.focus_page is not None else 1,
                            action['label'], action['reference'])
        return '等待该页新快照后记录标签, 不修改设备零位'
    elif kind == 'pause':
        if not log.file:
            return '未指定日志文件'
        if log.enabled:
            log.emit({'event': 'recording_paused', 'host_elapsed_s': client.clock()-client.begin})
            log.enabled = False
        else:
            log.enabled = True
            log.emit({'event': 'recording_resumed', 'host_elapsed_s': client.clock()-client.begin})
        return '记录已恢复' if log.enabled else '记录已暂停'


def inspect_collect(args, stop):
    """复用原 Client/CaptureLog, 仅把 Tk 控件动作换为本进程标准输入."""
    import serial
    capture = importlib.import_module('sentinel_capture')
    baud = args.baud if args.baud is not None else 115200
    log = capture.CaptureLog(args.log, dict(role='sentinel_capture', port=args.port, baud=baud,
        hardware=args.hardware, elf_sha256=hashlib.sha256(args.elf.read_bytes()).hexdigest() if args.elf else None,
        wall_time_ns=time.time_ns(),
        dm_queries='STD 0x7FF DLC4: 01 00 33 RID (whitelist) or 01 00 CC 00 (status)'))
    actions = queue.Queue()
    def receive():
        for line in sys.stdin:
            actions.put(json.loads(line))
    threading.Thread(target=receive, name='capture-ui-actions', daemon=True).start()
    def emit(record):
        log.emit(record)
        print(json.dumps(record, ensure_ascii=False, allow_nan=False), flush=True)
    port = None
    try:
        port = serial.Serial(args.port, baud, timeout=0, write_timeout=0.2, rtscts=False, dsrdtr=False)
        port.reset_input_buffer()
        client = capture.Client(port, args.hz, emit)
        while not stop.is_set() and (not args.seconds or time.monotonic()-client.begin < args.seconds):
            while not actions.empty():
                message = inspect_action(client, log, actions.get_nowait())
                if message:
                    print(json.dumps(dict(event='ui_action', message=message, logging=bool(log.file and log.enabled)),
                                     ensure_ascii=False), flush=True)
            client.tick()
            time.sleep(0.005)
        return 0
    except (OSError, ValueError, serial.SerialException) as error:
        emit(dict(event='protocol_error' if isinstance(error, capture.CaptureProtocolError) else 'disconnected',
                  reason=str(error)))
        return 2
    finally:
        if port is not None:
            port.close()
        log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--port', required=True)
    parser.add_argument('--mode', choices=('stream', 'inspect'), default='stream')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--seconds', type=float, default=0)
    parser.add_argument('--axis', choices=('stationary', 'main', 'fire', 'pitch'))
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--log', type=Path)
    parser.add_argument('--elf', type=Path)
    parser.add_argument('--hardware', default='unconfirmed')
    parser.add_argument('--note', default='')
    parser.add_argument('--baud', type=int, help='配套固件的串口波特率；省略时沿用所选脚本默认值')
    args = parser.parse_args()
    source = args.source.resolve()
    required = 'boards/sentinel_f407/tools/sentinel_capture.py' if args.mode == 'inspect' else 'robots/sentinel/host/sentinel_stream.py'
    if not (source / required).is_file():
        parser.error('所选源码没有连续采集实现；请返回首页显式选择正确版本')
    if args.mode == 'stream' and (args.output is None or args.axis is None):
        parser.error('连续记录需要 --output 和 --axis')
    if args.mode == 'inspect' and (not math.isfinite(args.hz) or not 1 <= args.hz <= 15 or
                                    not math.isfinite(args.seconds) or args.seconds < 0):
        parser.error('require 1 <= hz <= 15 and seconds >= 0')
    stop = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stop.set())
    sys.path[:0] = [str(source / 'robots/sentinel/host'), str(source / 'boards/sentinel_f407/tools')]
    if args.mode == 'inspect':
        return inspect_collect(args, stop)
    stream = importlib.import_module('sentinel_stream')
    if args.baud is not None:
        stream.BAUD = args.baud
    stream.collect(args.port, args.output, args.seconds,
                   'stationary' if args.axis == 'stationary' else 'rotation', args.note,
                   args.axis, stop, lambda event: print(json.dumps(event, ensure_ascii=False), flush=True))


if __name__ == '__main__':
    raise SystemExit(main())
