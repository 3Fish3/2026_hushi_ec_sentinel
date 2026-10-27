"""主 Yaw 有界试验：默认预览；显式 --execute --safety-confirmed 才连接硬件。

复用 APP 维护协议及原始采集。撤销是零输出，不是主动刹停。
只读 SWD 不暂停/复位 MCU，邮箱读取非原子，不替代 CAN 线上抓包。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import queue
import struct
import subprocess
import threading
import time
import uuid

import sentinel_client as wire
import sentinel_raw as raw
from sentinel_console_model import parameter_scale
import sys

PARAMETERS = {'velocity_kp': 0x0401, 'velocity_ki': 0x0402, 'position_kp': 0x0408}
DEFAULT_BUILD = 'app-gimbal-20261002-yawmap-fix'


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true', help='执行；不指定时只打印计划')
    p.add_argument('--safety-confirmed', action='store_true', help='确认 Pitch 支撑、线束余量、人员离开和物理急停可用')
    p.add_argument('--port', help='UART2 维护串口，921600 baud')
    p.add_argument('--observer-port', help='UART1 观测串口，115200 baud')
    p.add_argument('--expected-build', default=DEFAULT_BUILD)
    p.add_argument('--direction', choices=('positive', 'negative'), default='positive')
    p.add_argument('--angle-deg', type=float, default=5.0, help='相对新读取当前位置；0 < 角度 <= 10°')
    p.add_argument('--ramp-rad-s', type=float, default=.04, help='目标斜坡速度，不是电机速度；0 < 值 <= 0.08')
    p.add_argument('--hold-s', type=float, default=.4, help='斜坡前原地保持')
    p.add_argument('--settle-s', type=float, default=.6, help='斜坡后维持最终目标；总驱动时间 <= 5 s')
    p.add_argument('--post-s', type=float, default=.6, help='撤销后只读观察')
    p.add_argument('--sample-s', type=float, default=.025, help='请求轮询间隔，0.01..0.04 s')
    p.add_argument('--stop-speed', type=float, default=.5, help='速度停止阈值，只允许降低至0.1..0.5 rad/s')
    p.add_argument('--margin-deg', type=float, default=1.0, help='允许超过目标的位移余量，0.1..1°')
    for name in PARAMETERS:
        p.add_argument('--' + name.replace('_', '-'), type=float,
                       help='单变量对比；速度增益使用 GUI 电流口径，结束恢复 MCU 原值')
    p.add_argument('--capture', action='store_true', help='UART1 原始控制短窗及真实0x208反馈')
    p.add_argument('--capture-at-s', type=float, help='从原地保持开始计时；默认驱动结束前0.6 s')
    p.add_argument('--capture-ms', type=int, default=300, help='1..300 ms；缓存满可能提前冻结')
    p.add_argument('--swd', action='store_true', help='只读 CAN1 发送邮箱；CMSIS-DAP c251:f001')
    p.add_argument('--sound-file', type=Path, default=Path('/usr/share/sounds/freedesktop/stereo/bell.oga'))
    p.add_argument('--output', type=Path, help='新的结果目录；已存在则拒绝覆盖')
    return p


def plan(args):
    bounds = {'angle_deg': (0, 10), 'ramp_rad_s': (0, .08), 'hold_s': (.1, 1),
              'settle_s': (.1, 2), 'post_s': (.1, 2), 'sample_s': (.01, .04),
              'stop_speed': (.1, .5), 'margin_deg': (.1, 1)}
    for name, (low, high) in bounds.items():
        value = getattr(args, name)
        if not math.isfinite(value) or not low <= value <= high or (low == 0 and value == 0):
            raise ValueError(f'{name} 超出有限范围 {low}..{high}')
    ramp_s = math.radians(args.angle_deg) / args.ramp_rad_s
    duration = args.hold_s + ramp_s + args.settle_s
    if duration > 5:
        raise ValueError('总驱动时长超过5 s；请调整角度或斜坡，不延长驱动边界')
    overrides = {name: getattr(args, name) for name in PARAMETERS if getattr(args, name) is not None}
    if len(overrides) > 1:
        raise ValueError('每轮只允许修改一项 PID，便于归因和回退')
    if any(not math.isfinite(v) or v < 0 for v in overrides.values()):
        raise ValueError('PID 必须为非负有限值')
    if not 1 <= args.capture_ms <= 300:
        raise ValueError('capture_ms 必须为1..300')
    capture_at = args.capture_at_s if args.capture_at_s is not None else duration - .6
    if args.capture and (not math.isfinite(capture_at) or capture_at < 0
                         or capture_at + args.capture_ms / 1000 > duration):
        raise ValueError('采集短窗必须落在驱动时段内')
    if args.execute and (not args.safety_confirmed or not args.port or not args.observer_port):
        raise ValueError('执行必须指定两个串口，并显式 --safety-confirmed')
    if args.port and args.port == args.observer_port:
        raise ValueError('维护与观测串口不能相同')
    if not args.expected_build:
        raise ValueError('固件身份不能为空')
    return dict(domain='main_yaw', frame='joint', direction=args.direction,
                angle_deg=args.angle_deg, ramp_rad_s=args.ramp_rad_s, ramp_s=ramp_s,
                hold_s=args.hold_s, settle_s=args.settle_s, duration_s=duration,
                post_s=args.post_s, stop_speed=args.stop_speed, margin_deg=args.margin_deg,
                overrides_gui=overrides, capture=args.capture, capture_at_s=capture_at,
                capture_ms=args.capture_ms, swd=args.swd, expected_build=args.expected_build,
                sample_s=args.sample_s, port=args.port, observer_port=args.observer_port,
                sound_file=str(args.sound_file), execute=args.execute,
                parameter_policy='record/readback/restore; current limit unchanged')


def goal_at(base, elapsed, args):
    sign = 1 if args.direction == 'positive' else -1
    return base + sign * min(math.radians(args.angle_deg), max(0, elapsed - args.hold_s) * args.ramp_rad_s)


def feedback_valid(j):
    return (j['position_valid'] and j['velocity_valid']
            and all(j[k] is not None and 0 <= j[k] <= 20 for k in ('position_age_ms', 'velocity_age_ms'))
            and math.isfinite(j['angle_rad']) and math.isfinite(j['rate_rad_s']))


def stop_reasons(j, base, args):
    reasons = []
    if not feedback_valid(j):
        reasons.append('invalid_or_stale_feedback')
    if abs(j['rate_rad_s']) > args.stop_speed:
        reasons.append('speed_bound')
    signed_delta = (j['angle_rad'] - base) * (1 if args.direction == 'positive' else -1)
    if signed_delta > math.radians(args.angle_deg + args.margin_deg) or signed_delta < math.radians(-1):
        reasons.append('position_bound')
    return reasons


def accepted(reply):
    if reply.get('result_name') != 'ACCEPTED':
        raise RuntimeError(str(reply))
    return reply


class BoundedSerial(wire.SerialTransport):
    def exchange(self, message, timeout_s=.1):
        # 同步请求不允许等待5 s后才发现200 ms维护会话已失效.
        return super().exchange(message, min(timeout_s, .1))


def identity(port):
    import serial
    with serial.Serial(port, 115200, timeout=.02, write_timeout=.1) as connection:
        return raw.RawClient(connection).identity()


def analyse_raw(path):
    metadata, rows = raw.load(path)
    controls = [r for r in rows if r['kind'] == 'control']
    can = [r for r in rows if r['kind'] == 'can']
    outputs = [r['values'][4] for r in controls if r['values'][4] is not None]
    efforts = [struct.unpack('>h', bytes.fromhex(r['data'])[4:6])[0] for r in can
               if r['bus'] == 1 and r['can_id'] == 0x208]
    result = dict(path=str(path), control_count=len(controls), can_count=len(can), lost=metadata['lost'],
                  duration_us=raw.delta(metadata['end_us'], metadata['start_us']),
                  mapping_correct=bool(can) and all(r['bus'] == 1 and r['can_id'] == 0x208
                                                    and r['stream'] == 5 for r in can),
                  control_domain_correct=bool(controls) and all(r['stream'] == 4 for r in controls),
                  output_range=[min(outputs), max(outputs)] if outputs else None,
                  effort_raw_range=[min(efforts), max(efforts)] if efforts else None)
    if not result['mapping_correct'] or not result['control_domain_correct'] or result['lost']:
        raise RuntimeError('原始采集错配、缺反馈或有丢失；保留原文件，不判定通过')
    return result


class RawCapture:
    def __init__(self, args, spec, directory, base, build):
        self.stop = threading.Event()
        self.result = {}
        self.args, self.spec, self.directory, self.base, self.build = args, spec, directory, base, build
        self.thread = threading.Thread(target=self._run, name='yaw-raw-capture')

    def _run(self):
        try:
            if self.stop.wait(self.spec['capture_at_s']):
                self.result['error'] = '采集开始前测试已停止'
                return
            import serial
            deadline = time.monotonic() + 18

            class TimedRawClient(raw.RawClient):
                def exchange_wire(self, *values, **options):
                    if time.monotonic() >= deadline:
                        raise TimeoutError('原始采集/导出超过18 s')
                    return super().exchange_wire(*values, **options)

            with serial.Serial(self.args.observer_port, 115200, timeout=.02, write_timeout=.1) as port:
                meta, records = TimedRawClient(port).capture('control', self.args.capture_ms * 1000, 4)
            path = self.directory / 'control.ecraw'
            raw.save(path, dict(meta, control_domain=4, base=self.base,
                                goal=goal_at(self.base, self.spec['duration_s'], self.args),
                                device_identity=self.build), records)
            self.result.update(analyse_raw(path))
        except Exception as error:
            self.result['error'] = repr(error)

    def close(self):
        self.stop.set()
        self.thread.join(20)
        if self.thread.is_alive():
            self.result['error'] = '采集导出超时；结果不完整'
        return self.result


class SwdObserver:
    """固定只读配置；禁止用户注入任意 OpenOCD 命令。"""
    def __init__(self, directory):
        self.directory = directory
        self.ready, self.complete, self.failed = threading.Event(), threading.Event(), threading.Event()
        self.events = queue.SimpleQueue()
        self.process = None
        self.thread = None
        self.go, self.stop = directory / 'swd.go', directory / 'swd.stop'

    def start(self):
        cfg = self.directory / 'swd.cfg'
        cfg.write_text('''source [find interface/cmsis-dap.cfg]
cmsis_dap_vid_pid 0xc251 0xf001
transport select swd
source [find target/stm32f4x.cfg]
stm32f4x.cpu configure -event examine-end {}
adapter speed 200
reset_config none
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
poll
set dhcsr [lindex [read_memory 0xE000EDF0 32 1] 0]
if {($dhcsr & 0x20000) != 0 || [stm32f4x.cpu curstate] ne "running"} { error "CPU not running; no resume" }
echo SWD_READY_NO_HALT
set deadline [expr {[clock milliseconds]+15000}]
while {![file exists {GO}]} {
    if {[file exists {STOP}]} { shutdown; exit }
    if {[clock milliseconds]>$deadline} { error "Start timeout" }
    sleep 10
}
set deadline [expr {[clock milliseconds]+6000}]
set sample 0
while {![file exists {STOP}] && [clock milliseconds]<$deadline} {
    set dhcsr [lindex [read_memory 0xE000EDF0 32 1] 0]
    if {($dhcsr & 0x20000)!=0} { error "CPU halted" }
    set regs [read_memory 0x40006400 32 8]
    set esr [lindex $regs 6]
    echo "CAN_SAMPLE=$sample TSR=[lindex $regs 2] ESR=$esr TEC=[expr {($esr>>16)&255}] REC=[expr {($esr>>24)&255}] LEC=[expr {($esr>>4)&7}]"
    for {set mailbox 0} {$mailbox<3} {incr mailbox} {
        set words [read_memory [expr {0x40006580+16*$mailbox}] 32 4]
        set tir [lindex $words 0]
        set high [lindex $words 3]
        set raw [expr {((($high>>16)&255)<<8)|(($high>>24)&255)}]
        if {$raw>=32768} { set raw [expr {$raw-65536}] }
        echo "CAN_MAILBOX SAMPLE=$sample MAILBOX=$mailbox ID=[format 0x%03x [expr {($tir>>21)&0x7ff}]] TXRQ=[expr {$tir&1}] IDE=[expr {($tir>>2)&1}] DLC=[expr {[lindex $words 1]&15}] TDLR=[lindex $words 2] TDHR=$high FOURTH_SLOT=$raw"
    }
    incr sample
    sleep 30
}
poll
set dhcsr [lindex [read_memory 0xE000EDF0 32 1] 0]
if {($dhcsr & 0x20000)!=0 || [stm32f4x.cpu curstate] ne "running"} { error "CPU not running after observation" }
echo SWD_COMPLETE_NO_HALT_NO_RESET
shutdown
'''.replace('{GO}', '{' + str(self.go) + '}').replace('{STOP}', '{' + str(self.stop) + '}'), encoding='utf-8')
        self.process = subprocess.Popen(['openocd', '-f', str(cfg)], stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.thread = threading.Thread(target=self._read, name='yaw-swd')
        self.thread.start()
        deadline = time.monotonic() + 10
        while not self.ready.wait(.025):
            if self.failed.is_set() or self.process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('SWD observer unavailable; no motion')

    def _read(self):
        try:
            with (self.directory / 'swd.log').open('x', encoding='utf-8') as log:
                for line in self.process.stdout:
                    event = dict(time=time.monotonic(), line=line.strip())
                    log.write(json.dumps(event) + '\n'); log.flush()
                    self.events.put(event)
                    if 'SWD_READY_NO_HALT' in line:
                        self.ready.set()
                    if 'SWD_COMPLETE_NO_HALT_NO_RESET' in line:
                        self.complete.set()
                    if 'Error:' in line or 'Polling target' in line:
                        self.failed.set()
            self.process.wait()
            if self.process.returncode or not self.complete.is_set():
                self.failed.set()
        except Exception:
            self.failed.set()

    def check(self):
        if self.failed.is_set() or self.process.poll() is not None:
            raise RuntimeError('SWD observer stopped; cancel motion')

    def close(self):
        self.stop.touch(exist_ok=True)
        if self.process:
            try:
                self.process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                self.failed.set()
                self.process.terminate()
                self.process.wait(timeout=3)
            if self.thread:
                self.thread.join(3)
            if self.process.stdout:
                self.process.stdout.close()
        return dict(complete=self.complete.is_set(), failed=self.failed.is_set(),
                    reads_nonatomic=True, online_can_capture=False)


def run(args):
    spec = plan(args)
    if not args.execute:
        print(json.dumps(spec, ensure_ascii=False, indent=2))
        return spec
    directory = args.output or Path(__file__).resolve().parents[3] / '.tmp/yaw-tests' / (
        time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8])
    directory = directory.resolve()
    # Tcl路径以花括号引用；拒绝能够突破字面路径边界的名称.
    if any(c in str(directory) for c in '{}\n\r'):
        raise ValueError('输出路径不允许花括号或换行')
    directory.mkdir(parents=True, exist_ok=False)
    report = dict(plan=spec, output=str(directory), motion_started=False, overrides_restored=False)
    transport = client = swd = capture = sound = None
    granted = False
    originals = []
    with (directory / 'events.jsonl').open('x', encoding='utf-8') as log:
        def record(kind, **values):
            log.write(json.dumps(dict(kind=kind, time=time.monotonic(), **values)) + '\n'); log.flush()

        def status_ready(st):
            if not st['authority'] or st['source'] != 'maintenance' or st['manual_stop']:
                raise RuntimeError('Maintenance authority lost')

        try:
            build = identity(args.observer_port)
            record('identity', reply=build)
            if build['build_id'] != args.expected_build:
                raise RuntimeError('固件身份不匹配，不执行运动')
            if args.swd:
                swd = SwdObserver(directory)
                swd.start()
            transport = BoundedSerial(args.port)
            client = wire.SentinelClient(transport)
            record('hello', reply=client.hello())
            initial = client.status(1)
            if initial['authority'] or any(d['execute'] for d in initial['domains']):
                raise RuntimeError('已有控制权或执行目标，拒绝接管')
            device = next(d for d in client.diagnostics(1)['devices'] if d['domain'] == 'main_yaw')
            if not (device['installed'] and device['online'] and device['bus'] == 1
                    and device['device_id'] == 4 and device['feedback_id'] == 0x208 and device['protocol'] == 2):
                raise RuntimeError('主 Yaw 安装、协议或地址不符')
            # 授权ACK丢失也尝试撤销/释放，不把超时当作未执行。
            granted = True
            accepted(client.authorize(True, 'maintenance'))
            status_ready(client.status(1))
            accepted(client.targets([]))
            entries = []
            for pid in (0x401, 0x402, 0x403, 0x404, 0x408, 0x409, 0x40a):
                entry = client.param_read(pid)['entries'][0]
                if entry['id'] != pid or entry['domain'] != 'main_yaw':
                    raise RuntimeError('参数读回域/ID不匹配')
                entries.append(entry)
            record('original_parameters', entries=entries)
            cap = next(e['value'] for e in entries if e['id'] == 0x404)
            if not math.isfinite(cap) or not 0 < cap <= 2.5:
                raise RuntimeError('电流上限超出本测试允许的0..2.5 A')
            # 防止继承之前误填的高增益；本工具只支持P/PI对比，不带未知D项启动。
            limits = {0x401: .12 / cap, 0x402: 2.0 / cap, 0x403: 0,
                      0x408: 15.0, 0x409: 0, 0x40a: 0}
            for entry in entries:
                if entry['id'] in limits and not 0 <= entry['value'] <= limits[entry['id']]:
                    raise RuntimeError('原增益超出本测试范围；拒绝继承高增益或未知D项')
            for name, physical in spec['overrides_gui'].items():
                pid = PARAMETERS[name]
                entry = next(e for e in entries if e['id'] == pid)
                scale = parameter_scale(entry, entries)
                if scale is None:
                    raise RuntimeError('无法换算 GUI 参数口径')
                value = physical / scale
                if not entry['writable'] or not entry['min'] <= value <= entry['max']:
                    raise RuntimeError('参数超出 MCU 范围或不可写')
                if value > limits[pid] or value > max(entry['value'] * 2, 0):
                    raise RuntimeError('单轮增益不允许超过读回原值的2倍')
                # ACK丢失时也尝试恢复，不能假设失败写入未生效.
                originals.append((pid, entry['value']))
                record('parameter_write', id=pid, reply=accepted(client.param_write(pid, value)))
                actual = client.param_read(pid)['entries'][0]['value']
                if not math.isclose(actual, value, rel_tol=1e-5, abs_tol=1e-6):
                    raise RuntimeError('参数写入后读回不匹配')
            angles = []
            for _ in range(5):
                joint = client.status(0)['joints']['main_yaw']
                if not feedback_valid(joint) or abs(joint['rate_rad_s']) > .2:
                    raise RuntimeError('起点反馈无效或电机未静止')
                angles.append(joint['angle_rad']); time.sleep(.025)
            if max(angles) - min(angles) > .002:
                raise RuntimeError('起点仍在变化')
            base = angles[-1]
            report['base_rad'] = base
            report['final_goal_rad'] = goal_at(base, spec['duration_s'], args)
            record('baseline', base=base, goal=report['final_goal_rad'])
            print('BASE', base, 'FINAL_GOAL', report['final_goal_rad'], flush=True)
            sound = subprocess.Popen(['paplay', str(args.sound_file)], stderr=subprocess.PIPE, text=True)
            deadline = time.monotonic() + 3
            while sound.poll() is None:
                joint = client.status(0)['joints']['main_yaw']
                if not feedback_valid(joint) or abs(joint['angle_rad'] - base) > .002 or abs(joint['rate_rad_s']) > .2:
                    raise RuntimeError('提示音期间位置改变')
                if time.monotonic() > deadline:
                    raise RuntimeError('提示音超时')
                if swd:
                    swd.check()
                time.sleep(.025)
            record('sound', code=sound.returncode, error=sound.communicate()[1])
            if sound.returncode:
                raise RuntimeError('提示音失败，不执行运动')
            epochs = (client.hello_info['config_epoch'], client.hello_info['reference_epoch'])
            if args.capture:
                capture = RawCapture(args, spec, directory, base, build)
                capture.thread.start()
            if swd:
                swd.check(); swd.go.touch(exist_ok=False)
            start = time.monotonic()
            next_diag = start
            last_print = 0
            deltas = []
            reason = 'time_limit'
            while time.monotonic() - start < spec['duration_s']:
                if swd:
                    swd.check()
                    while not swd.events.empty():
                        record('swd', observation=swd.events.get())
                if capture and capture.result.get('error'):
                    raise RuntimeError('原始采集失败')
                target = goal_at(base, time.monotonic() - start, args)

                def send():
                    # 刷新目标/维护会话；导出不占此串口.
                    accepted(client.targets([('main_yaw', 'position', target, 0)]))

                report['motion_started'] = True
                send()
                joint = client.status(0)['joints']['main_yaw']
                delta = joint['angle_rad'] - base
                deltas.append(delta)
                record('sample', joint=joint, target=target, delta_deg=math.degrees(delta))
                reasons = stop_reasons(joint, base, args)
                if reasons:
                    reason = ','.join(reasons)
                    record('safety_stop', reasons=reasons, joint=joint)
                    break
                if time.monotonic() - last_print > .25:
                    print('MOTION target_deg=', round(math.degrees(target-base), 3),
                          'delta_deg=', round(math.degrees(delta), 3), 'rate=', joint['rate_rad_s'], flush=True)
                    last_print = time.monotonic()
                if time.monotonic() >= next_diag:
                    send(); st = client.status(1); status_ready(st)
                    record('execution', status=st)
                    if (st['config_epoch'], st['reference_epoch']) != epochs:
                        raise RuntimeError('参考或配置代次改变')
                    own = next(d for d in st['domains'] if d['domain'] == 'main_yaw')
                    if not own['execute'] or any(d['execute'] for d in st['domains'] if d['domain'] != 'main_yaw'):
                        raise RuntimeError('实际执行域改变')
                    send(); device = next(d for d in client.diagnostics(1)['devices'] if d['domain'] == 'main_yaw')
                    record('device', device=device)
                    if not device['online'] or (device['temperature_c'] is not None and device['temperature_c'] >= 50):
                        raise RuntimeError('电机离线或温度达到50°C')
                    next_diag = time.monotonic() + .25
                time.sleep(args.sample_s)
            accepted(client.targets([])); record('cancel')
            if swd:
                swd.stop.touch(exist_ok=True)
            report.update(stop_reason=reason, delta_deg=math.degrees(deltas[-1]),
                          min_delta_deg=math.degrees(min(deltas)), max_delta_deg=math.degrees(max(deltas)))
            if reason != 'time_limit':
                report['error'] = '触发保护边界：' + reason
            end = time.monotonic() + args.post_s
            while time.monotonic() < end:
                joint = client.status(0)['joints']['main_yaw']
                record('post_cancel', joint=joint, delta_deg=math.degrees(joint['angle_rad'] - base))
                if not feedback_valid(joint):
                    raise RuntimeError('撤销后反馈无效')
                time.sleep(.04)
            report.update(post_cancel_deg=math.degrees(joint['angle_rad']-base), final_rate=joint['rate_rad_s'])
        except (Exception, KeyboardInterrupt) as error:
            report['error'] = repr(error)
            record('error', detail=repr(error))
        finally:
            # 每项独立尝试；撤销失败不妨碍恢复/释放，失败必须留证，禁止重连恢复动作.
            cleanup_errors = []
            if granted:
                try:
                    record('final_cancel', reply=accepted(client.targets([])))
                    report['cancel_accepted'] = True
                except Exception as error:
                    cleanup_errors.append('cancel: ' + repr(error))
                restored = True
                for pid, original in reversed(originals):
                    try:
                        record('restore', id=pid, reply=accepted(client.param_write(pid, original)))
                        value = client.param_read(pid)['entries'][0]['value']
                        if not math.isclose(value, original, rel_tol=1e-5, abs_tol=1e-6):
                            raise RuntimeError('恢复读回不匹配')
                    except Exception as error:
                        restored = False; cleanup_errors.append('restore: ' + repr(error))
                report['overrides_restored'] = restored
                try:
                    record('release', reply=accepted(client.authorize(False, 'maintenance')))
                    report['release_accepted'] = True
                    st = client.status(1); record('final_status', status=st)
                    report['final_standby'] = not st['authority'] and not any(d['execute'] for d in st['domains'])
                    if not report['final_standby']:
                        raise RuntimeError('未确认全部域退出执行')
                except Exception as error:
                    cleanup_errors.append('release: ' + repr(error))
            for name, action in (('serial_close', transport.close if transport else None),
                                 ('sound_close', lambda: stop_sound(sound)),
                                 ('swd_close', lambda: report.update(swd=swd.close()) if swd else None),
                                 ('capture_close', lambda: report.update(capture=capture.close()) if capture else None)):
                try:
                    if action:
                        action()
                except Exception as error:
                    cleanup_errors.append(name + ': ' + repr(error))
            report['cleanup_errors'] = cleanup_errors
            report['completed'] = not (report.get('error') or cleanup_errors
                                      or report.get('capture', {}).get('error')
                                      or (args.capture and not report.get('capture', {}).get('mapping_correct'))
                                      or (args.swd and (not report.get('swd', {}).get('complete')
                                                       or report.get('swd', {}).get('failed'))))
            record('summary', report=report)
            (directory / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def stop_sound(sound):
    if sound and sound.poll() is None:
        sound.terminate(); sound.wait(timeout=2)


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        report = run(args)
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2
    return 0 if not args.execute or report['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
