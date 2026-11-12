"""APP短批次只读记录: 快页0、慢页1/2、倾斜诊断3和有效参数; 不授权、不发目标、不写参数."""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import time
import sentinel_client as wire
from sentinel_async import AsyncSentinelClient


def decode_recording(output: Path):
    """收尾解码完整wire, 保留每帧时刻; 快页不被慢页20Hz出口或UI采样丢掉."""
    decoder = wire.FrameDecoder()
    count = 0
    with (output/'wire.jsonl').open(encoding='utf-8') as source, (output/'samples.jsonl').open('x', encoding='utf-8') as dest:
        for line in source:
            row = json.loads(line)
            if row.get('direction') != 'rx':
                continue
            for byte in bytes.fromhex(row['wire_hex']):
                frame = decoder.push(byte)
                if not frame:
                    continue
                typ, sequence, payload = frame
                if typ not in wire.DECODERS:
                    continue
                decoded = wire.DECODERS[typ](payload)
                kind = 'status'+str(decoded['page']) if typ == wire.STATUS_SNAPSHOT else (
                    'tilt' if typ == wire.DIAGNOSTICS_PAGE and decoded['page'] == 3 else
                    'control'+str(decoded['page']) if typ == wire.DIAGNOSTICS_PAGE and decoded['page'] in (4,5) else 'reply')
                dest.write(json.dumps(dict(monotonic_s=row['monotonic_s'], frame_sequence=sequence,
                    kind=kind, data=decoded), ensure_ascii=False)+'\n')
                count += 1
    for page, domain in ((4,'main_yaw'),(5,'fire_yaw')):
        writer = None
        with (output/f'{domain}-control.csv').open('x',encoding='utf-8',newline='') as dest:
            for line in (output/'samples.jsonl').open(encoding='utf-8'):
                row = json.loads(line)
                if row['kind'] != 'control'+str(page):
                    continue
                for sample in row['data']['samples']:
                    record = dict(received_monotonic_s=row['monotonic_s'], **sample)
                    if writer is None:
                        writer = csv.DictWriter(dest,fieldnames=list(record))
                        writer.writeheader()
                    writer.writerow(record)
    return dict(decoded_frames=count, frame_rejected=decoder.rejected)


def advance_control_cursor(result, cursors, page):
    if result.get('envelope') == 'error' or not result.get('available') or result.get('schema') != 1:
        raise wire.ProtocolError(f'固件没有电流轨迹页{page}; 回报接口/版本, 不重复试动')
    expected = cursors.get(page)
    if expected is not None and result['first_sequence'] != expected:
        raise wire.ProtocolError(f'轨迹页{page}序号缺口: expected={expected}, got={result["first_sequence"]}; 回报已有记录')
    cursors[page] = (result['first_sequence']+len(result['samples'])) & 0xffffffff


def capture(transport, output: Path, seconds: float = 30.0, hz: float = 50.0, *, control_traces=False):
    if not math.isfinite(seconds) or seconds <= 0 or not math.isfinite(hz) or not 0 < hz <= 100:
        raise ValueError('seconds必须为正, 快页hz范围为(0,100]')
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    summary = dict(complete=False, batches=0, started_unix_s=time.time(), started_monotonic_s=time.monotonic())
    # 复用既有异步单串口拥有者: 慢页逐一请求, 后台快页独立轮询, 不改变MCU的20Hz慢出口.
    client = AsyncSentinelClient(transport, fast_hz=hz, timeout_s=.5, attempts=1, log_path=output/'wire.jsonl')
    try:
        summary['hello'] = client.hello().result(timeout=2)
        summary['parameters_before'] = client.read_all_params().result(timeout=10)
        first = client.diagnostics(3).result(timeout=2)
        if first.get('envelope') == 'error' or not first.get('available'):
            raise wire.ProtocolError('当前固件没有倾斜诊断页3; 保留日志并先回报版本/接口问题')
        summary['runtime_before'] = client.diagnostics(2).result(timeout=2)
        cursors = {}
        def trace(page):
            result = client.diagnostics(page, cursor=cursors.get(page)).result(timeout=2)
            advance_control_cursor(result,cursors,page)
            return result
        if control_traces:
            trace(4); trace(5)
            summary['trace_cursor_start'] = dict(cursors)
        begin = time.monotonic()
        summary['sampling_start_monotonic_s'] = begin
        print(f'只读采集中, {seconds:g}s; 动作由遥控操作, Ctrl+C只结束记录.', flush=True)
        while time.monotonic()-begin < seconds:
            for page in (1,2,3):
                if control_traces:
                    trace(4); trace(5)
                result = (client.diagnostics(3) if page == 3 else client.status(page)).result(timeout=2)
                if result.get('envelope') == 'error':
                    raise wire.ProtocolError(f'只读查询失败: {result}')
            summary['batches'] += 1
            # 管道夹具没有MCU慢出口, 不在离线测试中无界忙等; 串口路径通常已用去150ms.
            time.sleep(.001)
        if control_traces:
            trace(4); trace(5)
            summary['trace_cursor_end'] = dict(cursors)
        summary['sampling_end_monotonic_s'] = time.monotonic()
        summary['parameters_after'] = client.read_all_params().result(timeout=10)
        summary['runtime_after'] = client.diagnostics(2).result(timeout=2)
        summary['complete'] = True
    except BaseException as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        client.close()
        state = client.snapshot()
        summary.update(statistics=state['statistics'], log_dropped=state.get('log_dropped',0), log_error=state.get('log_error',''))
        summary['duration_s'] = time.monotonic()-summary['started_monotonic_s']
        try:
            summary.update(decode_recording(output))
            if summary['frame_rejected'] or summary['log_dropped'] or summary['log_error']:
                summary['complete'] = False
        except Exception as error:
            summary['complete'] = False
            summary['decode_error'] = str(error)
        (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    backend = parser.add_mutually_exclusive_group(required=True)
    backend.add_argument('--serial')
    backend.add_argument('--backend', type=Path, help='离线原生host_link, 不访问硬件')
    parser.add_argument('--baud', type=int, default=921600)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--hz', type=float, default=50, help='快页期望Hz; 慢页共享20Hz出口')
    parser.add_argument('--control-traces', action='store_true', help='连续下载main/fire约33Hz电流区间, 无需第二串口')
    args = parser.parse_args()
    transport = wire.SerialTransport(args.serial, args.baud) if args.serial else wire.ProcessTransport(args.backend)
    try:
        if not capture(transport, args.output, args.seconds, args.hz, control_traces=args.control_traces)['complete']:
            raise SystemExit('记录不完整, 请回传本目录summary和已有wire, 不重复操作')
    finally:
        transport.close()


if __name__ == '__main__':
    main()
