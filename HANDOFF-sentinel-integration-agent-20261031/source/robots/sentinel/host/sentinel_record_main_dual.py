#!/usr/bin/env python3
"""双串口只读记录 main 稳向：连续三页状态 + 间歇控制短窗。

只在开始前按一次回车。默认60秒，业务口921600，观察口115200。
默认main惯性航向误差超过5度时触发500ms请求；512条缓存可能提前录满。
例如：sentinel_record_main_dual.py --seconds 120 --raw-ms 500 --trigger-deg 5
不申请控制权、不发目标、不写参数、不使用SWD。
Ctrl+C只停记录，不能停车。短窗缓存和导出耗时决定实际高频覆盖率；
导出期间无法抓另一个高频窗口，结束时仍需等待当前窗口导出。
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import serial
import sentinel_client as wire
import sentinel_raw as raw

BUSINESS_PORT = '/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148384-if02'
OBSERVE_PORT = '/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148353-if02'


def write_line(file, value):
    file.write(json.dumps(value, ensure_ascii=False) + '\n')
    file.flush()


def read_status(client, page):
    value = client.status(page)
    if value.get('envelope') == 'error' or value.get('page') != page:
        raise wire.ProtocolError(f'状态页{page}无效: {value}')
    return value


class TracedRawClient(raw.RawClient):
    """保留失败请求及本客户端启动的run，不猜测超时原因。"""
    def __init__(self, port):
        super().__init__(port)
        self.last_request = None
        self.pending_run = None
        self.wire_retries = 0

    def exchange_wire(self, payload, kind=0x31, response=0xb1):
        # 本机实测观察口完整导出应答可晚于原0.4秒上限；同一请求重传保持序号不变。
        self.sequence = (self.sequence + 1) & 65535
        frame = raw.encode_frame(kind, self.sequence, payload)
        for attempt in range(3):
            if attempt:
                self.wire_retries += 1
            time.sleep(max(0, self.last_send + 0.1 - time.monotonic()))
            if self.port.write(frame) != len(frame):
                raise IOError('观察口请求未完整写入')
            self.last_send = time.monotonic()
            deadline = self.last_send + 1.2
            while time.monotonic() < deadline:
                for rx_type, sequence, body in self.decoder.feed(self.port.read(512)):
                    if rx_type == response and sequence == self.sequence:
                        return body
        raise TimeoutError('观察口同一请求重试3次仍无完整匹配应答（每次等待1.2秒）')

    def exchange(self, payload):
        op = payload[1]
        run = int.from_bytes(payload[4:8], 'little')
        self.last_request = {'op': op, 'run': run,
            'arg': int.from_bytes(payload[8:12], 'little'),
            'phase': {0: '查询状态', 1: '启动窗口', 2: '停止窗口', 3: '导出记录'}.get(op, '未知')}
        if op == 1:
            self.pending_run = run
        return super().exchange(payload)

    def capture(self, *args, **kwargs):
        status, records = super().capture(*args, **kwargs)
        self.pending_run = None
        return status, records


def check_observation(port_path, attempts=5):
    """有限重试观察口查询；退避是恢复措施，不代表已确认超时根因。"""
    last_error = None
    backoff = 0.5
    for attempt in range(1, attempts + 1):
        stream = None
        try:
            stream = serial.Serial(port_path, 115200,
                timeout=0.02, write_timeout=1, exclusive=True)
            stream.reset_input_buffer()
            client = TracedRawClient(stream)
            time.sleep(0.3)
            try:
                wire.decode_status(client.exchange_wire(b'', 0x36, 0xb6))
                print(f'观察口快速快照正常（第{attempt}次检查）', flush=True)
            except Exception as exc:
                print(f'观察口快速快照未响应：{type(exc).__name__}，改查身份', flush=True)
            identity = client.identity()
            return stream, client, identity
        except Exception as exc:
            last_error = exc
            if stream is not None:
                stream.close()
            print(f'观察口第{attempt}/{attempts}次检查失败：'
                  f'{type(exc).__name__}: {exc}；{backoff:.1f}秒后重试', flush=True)
            if attempt < attempts:
                time.sleep(backoff)
                backoff = min(backoff * 2, 4.0)
    raise TimeoutError(f'观察口{port_path}连续{attempts}次无法确认：{last_error}；'
        '尚未开始记录，请保留完整报错以诊断链路。')


def finish_pending(client, results):
    """只停止本客户端仍在记录的run；frozen/full无需清空，可直接开始新run。"""
    run = client.pending_run
    if run is None:
        return
    try:
        status, _ = client.exchange(raw.request(0))
        if status['run'] != run:
            raise RuntimeError(f'当前run={status["run"]}不属于本次请求{run}，不发送停止')
        if status['state'] in (0, 1):
            status, _ = client.exchange(raw.request(2, run))
            if status['run'] != run or status['state'] in (0, 1):
                raise RuntimeError('停止应答未确认当前run已冻结')
            results['flushed_windows'] += 1
            print(f'已停止本次未结束的采集：run={run}', flush=True)
        client.pending_run = None
    except Exception as exc:
        results['flush_errors'] += 1
        print(f'本次采集收尾失败：{type(exc).__name__}: {exc}', flush=True)


def main_error_trigger(record, threshold):
    """按同页、同参考系的已采用目标与测量触发；不把无效反馈当大误差。"""
    page = record.get('page1', {})
    for domain in page.get('domains', []):
        if domain.get('domain') != 'main_yaw':
            continue
        if (not domain.get('execute') or not domain.get('measurement_valid')
                or domain.get('mode') != 'position'
                or domain.get('adopted_frame') != 'local_inertial'
                or domain.get('measured_frame') != 'local_inertial'):
            return None
        target, measured = domain.get('target'), domain.get('measurement')
        if target is None or measured is None or not all(math.isfinite(x) for x in (target, measured)):
            return None
        error = math.degrees(target - measured)
        if abs(error) < threshold:
            return None
        return {'host_trigger_t_s': record['t_ms'] / 1000,
            'error_deg': error, 'target_rad': target, 'measurement_rad': measured,
            'clock_ms': page.get('clock_ms'), 'config_epoch': page.get('config_epoch'),
            'reference_epoch': page.get('reference_epoch')}
    return None


def replace_trigger(triggers, trigger):
    # 只保留最新仍超阈值的状态；导出旧窗口时不排队积压过期触发。
    try:
        triggers.get_nowait()
    except queue.Empty:
        pass
    if trigger is not None:
        triggers.put_nowait(trigger)


def capture_windows(client, directory, started, seconds, raw_ms, stop, results, triggers, trigger_deg):
    with (directory / 'control-windows.jsonl').open('x', encoding='utf-8') as manifest:
        index = 0
        while not stop.is_set() and time.monotonic() - started < seconds:
            trigger = None
            if trigger_deg > 0:
                try:
                    trigger = triggers.get(timeout=0.1)
                except queue.Empty:
                    continue
                if time.monotonic() - started - trigger['host_trigger_t_s'] > 0.5:
                    continue
            if stop.is_set() or time.monotonic() - started >= seconds:
                break
            index += 1
            begin = time.monotonic() - started
            if trigger is not None:
                print(f'触发高频短窗 #{index}：main误差 {trigger["error_deg"]:+.2f}°', flush=True)
            output = directory / f'main-control-{index:03d}.sraw'
            try:
                status, records = client.capture('control', raw_ms * 1000, domain=4)
                metadata = dict(status, schema=raw.VERSION, layout_id=raw.LAYOUT,
                    timebase='TIM5_us32', profile='control', control_domain=4,
                    units={'gyro': 'rad/s', 'accel': 'm/s2', 'temperature': 'degC'},
                    window_requested_us=raw_ms * 1000, trigger=trigger,
                    host_request_t_s=begin, host_saved_t_s=time.monotonic() - started)
                raw.save(output, metadata, records)
                item = {'event': 'window', 'file': output.name, 'trigger': trigger,
                    'host_request_t_s': round(begin, 3),
                    'host_saved_t_s': round(time.monotonic() - started, 3),
                    'records': len(records), 'status': status,
                    'actual_window_us': (status['end_us'] - status['start_us']) & 0xffffffff}
                results['windows'] += 1
                print(f'高频短窗 #{index} 已保存：{len(records)}条，'
                      f'覆盖约 {item["actual_window_us"] / 1000:.1f} ms', flush=True)
            except Exception as exc:
                results['errors'] += 1
                failed_request = client.last_request
                item = {'event': 'window_error', 'index': index, 'trigger': trigger,
                    'host_request_t_s': round(begin, 3),
                    'failed_request': failed_request,
                    'error': f'{type(exc).__name__}: {exc}'}
                phase = failed_request['phase'] if failed_request else '未知步骤'
                print(f'高频短窗 #{index} {phase}失败：{item["error"]}', flush=True)
                # 仅停止仍在记录的本次run，不清空已冻结窗口；失败请求阶段保留供诊断。
                finish_pending(client, results)
                write_line(manifest, item)
                stop.wait(2.0)
                continue
            write_line(manifest, item)
            stop.wait(0.5)
        finish_pending(client, results)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--business-port', default=BUSINESS_PORT)
    parser.add_argument('--observe-port', default=OBSERVE_PORT)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--period', type=float, default=0.1)
    parser.add_argument('--raw-ms', type=int, default=500, help='短窗请求时长；默认500ms，512条缓存满可能提前结束')
    parser.add_argument('--trigger-deg', type=float, default=5.0,
        help='main惯性航向误差触发阈值（度）；默认5，0表示无条件间歇采样')
    parser.add_argument('--countdown', type=int, default=5)
    args = parser.parse_args()
    if (not all(math.isfinite(v) and v > 0 for v in (args.seconds, args.period))
            or not math.isfinite(args.trigger_deg) or args.trigger_deg < 0
            or not 1 <= args.raw_ms <= 2000 or args.countdown < 0):
        parser.error('seconds/period必须为有限正数，raw-ms为1..2000，触发阈值和countdown不能为负')
    if Path(args.business_port).resolve() == Path(args.observe_port).resolve():
        parser.error('两路串口不能是同一个设备')
    output = args.output_dir or Path.home() / 'captures' / (
        'main-dual-' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()))
    if output.exists():
        parser.error(f'拒绝覆盖已有目录：{output}')
    print(f'业务口：{args.business_port}\n观察口：{args.observe_port}\n'
          f'时长：{args.seconds:g}秒\n目录：{output}', flush=True)
    print(f'高频请求：{args.raw_ms}ms，512条缓存满会提前结束；'
          f'触发阈值：{args.trigger_deg:g}°（0为无条件采样）。', flush=True)
    print('记录期间自由操作，不用打标记或碰键盘。请勿同时运行其他串口记录器。', flush=True)
    print('只读，不控制机器人。Ctrl+C或到时只停记录，不停车。', flush=True)
    try:
        input('准备好后按回车；连接检查和倒计时结束后才开始记录：')
    except (KeyboardInterrupt, EOFError):
        print('\n已取消，未开始记录。')
        return 130

    business = observation = worker = None
    stop = threading.Event()
    high_rate = {'windows': 0, 'errors': 0, 'flushed_windows': 0, 'flush_errors': 0}
    triggers = queue.Queue(maxsize=1)
    trigger_samples = 0
    counts = {page: 0 for page in (0, 1, 2)}
    errors = samples = 0
    interrupted = False
    started = None
    try:
        # 观察口检查可能耗时数秒；必须在业务会话建立之前完成，避免会话在等待期间超时。
        observation, raw_client, identity = check_observation(args.observe_port)
        if not identity['build_id'].startswith('app-'):
            raise RuntimeError(f'观察口不是APP固件：{identity}')
        business = wire.SerialTransport(args.business_port, 921600)
        client = wire.SentinelClient(business)
        hello = client.hello()
        for page in counts:
            read_status(client, page)
        parameters = client.read_all_params()
        print('两路连接成功；参数已读回。', flush=True)
        for remaining in range(args.countdown, 0, -1):
            print(f'倒计时 {remaining} …', flush=True)
            until = time.monotonic() + 1
            while time.monotonic() < until:
                read_status(client, 0)
                time.sleep(min(.1, max(0, until - time.monotonic())))

        output.mkdir(parents=True, exist_ok=False)
        with (output / 'status.jsonl').open('x', encoding='utf-8') as file:
            started = time.monotonic()
            write_line(file, {'event': 'header', 'started_at': time.time(),
                'seconds': args.seconds, 'period_s': args.period, 'hello': hello,
                'parameters_at_start': parameters, 'observer_identity': identity,
                'business_port': args.business_port, 'observe_port': args.observe_port,
                'role': 'readonly_main_dual', 'raw_requested_ms': args.raw_ms,
                'trigger_deg': args.trigger_deg, 'trigger_max_age_s': 0.5})
            worker = threading.Thread(target=capture_windows, args=(raw_client,
                output, started, args.seconds, args.raw_ms, stop, high_rate,
                triggers, args.trigger_deg), daemon=True)
            worker.start()
            print(f'\n========== 开始记录：{args.seconds:g}秒，现在自由操作 ==========\a', flush=True)
            next_report = 5
            try:
                while time.monotonic() - started < args.seconds:
                    loop = time.monotonic()
                    record = {'t_ms': int((loop - started) * 1000), 'wall': time.time()}
                    for page in counts:
                        try:
                            record[f'page{page}'] = read_status(client, page)
                            counts[page] += 1
                        except Exception as exc:
                            errors += 1
                            record[f'page{page}_error'] = f'{type(exc).__name__}: {exc}'
                    if args.trigger_deg > 0:
                        trigger = main_error_trigger(record, args.trigger_deg)
                        if trigger is not None:
                            trigger_samples += 1
                            record['main_error_trigger'] = trigger
                        replace_trigger(triggers, trigger)
                    write_line(file, record)
                    samples += 1
                    elapsed = time.monotonic() - started
                    if elapsed >= next_report:
                        print(f'已录 {elapsed:.1f}/{args.seconds:g}秒 | {samples}组 | '
                              f'状态错误 {errors} | 高频短窗 {high_rate["windows"]}份', flush=True)
                        next_report = elapsed + 5
                    time.sleep(max(0, args.period - (time.monotonic() - loop)))
            except KeyboardInterrupt:
                interrupted = True
            finally:
                stop.set()
                elapsed = time.monotonic() - started
                write_line(file, {'event': 'summary', 'elapsed_s': round(elapsed, 3),
                    'samples': samples, 'page_counts': counts, 'errors': errors,
                    'interrupted': interrupted})
        print('状态记录结束，正在收尾最后一份高频短窗；机器人运动状态未改变。', flush=True)
        finish_deadline = time.monotonic() + 120
        while worker.is_alive() and time.monotonic() < finish_deadline:
            worker.join(timeout=5)
            if worker.is_alive():
                print('仍在导出最后一份高频短窗，请等待；机器人运动状态未改变。', flush=True)
        if worker.is_alive():
            raise TimeoutError('高频短窗收尾超过120秒；已保存的数据保留')
        summary = {'event': 'summary', 'output_dir': str(output),
            'samples': samples, 'page_counts': counts, 'status_errors': errors,
            'raw_windows': high_rate['windows'], 'raw_errors': high_rate['errors'],
            'flushed_windows': high_rate['flushed_windows'],
            'flush_errors': high_rate['flush_errors'],
            'trigger_deg': args.trigger_deg, 'trigger_condition_samples': trigger_samples,
            'interrupted': interrupted}
        with (output / 'summary.json').open('x', encoding='utf-8') as file:
            json.dump(summary, file, ensure_ascii=False, indent=2)
        print('\n========== 记录完成；机器人运动状态未改变 ==========\a', flush=True)
        print(json.dumps(summary, ensure_ascii=False), flush=True)
        success = (errors == 0 and high_rate['errors'] == 0 and high_rate['flush_errors'] == 0
            and (high_rate['windows'] > 0 or (args.trigger_deg > 0 and trigger_samples == 0)))
        return 130 if interrupted else 0 if success else 1
    except KeyboardInterrupt:
        print('\n已取消；只停止记录，不负责停车。', flush=True)
        return 130
    except Exception as exc:
        print(f'记录失败：{type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        if started is not None:
            print(f'部分数据保留在：{output}', flush=True)
        return 1
    finally:
        stop.set()
        if worker is not None and worker.is_alive():
            worker.join(timeout=5)
        if observation is not None:
            observation.close()
        if business is not None:
            business.close()


if __name__ == '__main__':
    sys.exit(main())
