"""GM6020 S5/v6 单电机台架. 连接只查询; 参数按次配置, 波形不改变执行状态."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import board_monitor as wire
import motor_bench_trace as trace

STATUS = struct.Struct('<IIBBBBHHI11fhhBBIBBHHBB5I4fBfHB')
MAGIC = 0x53354236
REASONS = ['空闲', '运行', '手动停止', '时长结束', '命令超时', '反馈失效', '位置参考失效',
           '超速', 'CAN 发送失败', '请求非法', '运行中拒绝重启', '控制周期异常']
MODES = ['电流 A', '速度 rad/s', '相对位置 rad']


def settings_payload(mode, values):
    if len(values) == 9: values = [*values, 0.0]
    if len(values) == 10: values = [*values, 0]
    target, duration, current, speed, kp, ki, pos_kp, kd, alpha, d_tau, encoder_velocity = values
    if not all(math.isfinite(v) for v in values):
        raise ValueError('所有参数必须有限')
    if not (1 <= mode <= 3 and 0 <= duration <= 3600 and 0 < current <= 3 and 0 < speed <= 60
            and min(kp, ki, kd, d_tau) >= 0 and pos_kp > 0 and 0 <= alpha < 1 and encoder_velocity in (0, 1)):
        raise ValueError('时间 0=持续或 <=3600s; 电流 <=3A; 速度目标上限 <=60rad/s; 增益非负; alpha 属于 [0,1)')
    if 0 < duration < 0.001:
        raise ValueError('非零运行时间至少 1 ms')
    if mode < 3 and abs(target) > (current, speed)[mode-1]:
        raise ValueError('目标超过所设电流/速度上限')
    return struct.pack('<B10fB', mode, *values[:10], int(encoder_velocity))


def target_payload(token, value, ramp_us=0):
    if not math.isfinite(value) or not 0 <= ramp_us < 0x80000000:
        raise ValueError('目标须有限; 斜坡时间须在微秒半回绕范围内, 0 为阶跃')
    return struct.pack('<HfI', token, value, ramp_us)


def decode_status(payload):
    if len(payload) != STATUS.size:
        raise ValueError(f'需要 S5/v6 的 {STATUS.size} B 状态, 收到 {len(payload)} B; 不兼容旧固件')
    keys = ('magic', 'ms', 'active', 'mode', 'reason', 'result', 'token', 'flags', 'feedback_ms',
            'target', 'output_a', 'absolute_rad', 'relative_rad', 'speed_rad_s', 'current_limit',
            'speed_limit', 'kp', 'ki', 'position_kp', 'duration_s', 'effort_raw', 'encoder',
            'temperature', 'tx_status', 'tx_errors', 'bus', 'device_id', 'rx_id', 'tx_id', 'slot', 'reserved',
            'control_us', 'feedback_us', 'feedback_sequence', 'reference_epoch', 'settings_epoch',
            'kd', 'feedback_alpha', 'target_final', 'derivative_tau_s',
            'encoder_velocity', 'estimated_velocity_rad_s', 'estimated_velocity_age_ms', 'estimated_velocity_valid')
    data = dict(zip(keys, STATUS.unpack(payload)))
    if (data['magic'] != MAGIC or data['mode'] not in (1, 2, 3) or data['active'] not in (0, 1)
            or data['bus'] != 1 or not 1 <= data['device_id'] <= 7):
        raise ValueError('需要对应 S5/v6 单电机固件; 不发送旧版请求')
    if not all(math.isfinite(data[k]) for k in (*keys[9:20], 'kd', 'feedback_alpha', 'target_final', 'derivative_tau_s', 'estimated_velocity_rad_s')):
        raise ValueError('固件返回非有限控制量')
    data['feedback_age_ms'] = (data['ms']-data['feedback_ms']) & 0xffffffff
    data['feedback_valid'] = bool(data['flags'] & 1 and data['feedback_age_ms'] < 50)
    data['position_valid'] = bool(data['flags'] & 2 and data['feedback_valid'])
    return data


class TraceDownload:
    """冻结窗口按 run/偏移读取. 此状态只属于日志, 失败不撤销电机目标."""
    def __init__(self, run, count, trigger, metadata, compact=False):
        self.compact = compact
        self.record = trace.IDENT_RECORD if compact else trace.RECORD
        self.layout = trace.IDENT_LAYOUT if compact else trace.LAYOUT
        if not 1 <= count <= (trace.IDENT_CAPACITY if compact else trace.CAPACITY):
            raise ValueError('采样数超出所选布局容量, 不分频')
        self.run, self.count, self.trigger = run, count, int(trigger)
        self.phase = 'arm'
        self.metadata = dict(metadata, run=run, nominal_control_hz=1000, decimation=1, layout=self.layout)
        self.header = None
        self.data = bytearray()
        self.next_at = 0.0

    def request(self):
        op, arg = {'arm': (1, self.count), 'wait': (0, 0), 'freeze': (2, 0),
                   'start_meta': (4, 0), 'end_meta': (4, 1), 'data': (3, len(self.data)//self.record.size)}[self.phase]
        return op, trace.request(op, self.run, arg, self.trigger if op == 1 else 0, self.compact)

    def accept(self, op, payload):
        head, body = trace.decode_reply(payload)
        if head['error'] or head['op'] != op or head['run'] != self.run or head['version'] != (2 if self.compact else 1):
            raise ValueError(f'采集请求失败: op={head["op"]}, error={head["error"]}, run={head["run"]}')
        if self.header and any(head[k] != self.header[k] for k in ('run', 'size', 'first_us', 'last_us')):
            raise ValueError('冻结窗口身份/长度/时间改变, 不混接样本')
        if op in (0, 1, 2):
            if head['state'] == 3:
                self.header = head
                self.metadata['window'] = head
                self.phase = 'start_meta' if head['size'] else 'done'
            elif head['state'] in (1, 2):
                self.phase = 'wait'
                self.next_at = time.monotonic()+.05
            else:
                raise ValueError('采集窗口已丢失')
        elif op == 4:
            expected = 0 if self.phase == 'start_meta' else 1
            if head['arg'] != expected or len(body) != STATUS.size+36 or struct.unpack_from('<I', body, STATUS.size)[0] != self.layout:
                raise ValueError('采集元数据布局不匹配')
            self.metadata['start' if expected == 0 else 'end'] = decode_status(body[:STATUS.size])
            self.metadata['build_id'] = body[STATUS.size+4:].split(b'\0', 1)[0].decode('ascii')
            self.phase = 'end_meta' if expected == 0 else 'data'
        elif op == 3:
            at = len(self.data)//self.record.size
            if (head['arg'] != at or not head['count'] or at+head['count'] > self.header['size']
                    or len(body) != head['count']*self.record.size):
                raise ValueError('采集数据偏移/长度不匹配')
            self.data.extend(body)
            if len(self.data)//self.record.size == self.header['size']:
                self.phase = 'done'


class Session(threading.Thread):
    """唯一串口/序号拥有者. 停止优先, 启动不重发, 下载期间仍调度原有心跳."""
    def __init__(self, port_name, log_path, events, serial_factory=serial.Serial, metadata=None):
        super().__init__(daemon=False)
        self.port_name, self.log_path, self.events = port_name, Path(log_path), events
        self.serial_factory, self.metadata = serial_factory, metadata or {}
        self.commands = queue.Queue(maxsize=1)
        self.targets = queue.Queue(maxsize=1)
        self.excitations = queue.Queue(maxsize=1)
        self.trace_commands = queue.Queue(maxsize=2)
        self.quit_event, self.stop_event = threading.Event(), threading.Event()
        self.reference_event, self.silence_event = threading.Event(), threading.Event()
        self.token = None
        self.capture = None
        self.trace_run = time.monotonic_ns() & 0xffffffff

    def start_motion(self, payload):
        self.commands.put_nowait(payload)

    def update_target(self, value, ramp_us=0):
        target_payload(0, value, ramp_us)
        self.targets.put_nowait((value, ramp_us))

    def excite(self, kind, bias, amplitude, frequency_hz, duration_us):
        # 对应 MCU 0x27; 原始请求字节会进入会话日志, 不通过 PC 定时循环产生激励.
        if (kind not in (1, 2) or not all(math.isfinite(v) for v in (bias, amplitude, frequency_hz))
                or amplitude <= 0 or not 0 < frequency_hz < 500 or not 0 < duration_us < 0x80000000):
            raise ValueError('无效辨识激励')
        self.excitations.put_nowait(struct.pack('<B3fI', kind, bias, amplitude, frequency_hz, duration_us))

    def start_capture(self, count=1024, trigger=False, note='', compact=False):
        if not 1 <= count <= (trace.IDENT_CAPACITY if compact else trace.CAPACITY):
            raise ValueError('采样数超出所选布局容量')
        self.trace_commands.put_nowait(('arm', (count, trigger, note, compact)))

    def freeze_capture(self):
        self.trace_commands.put_nowait(('freeze', None))

    def set_reference(self):
        self.reference_event.set()

    def stop_motion(self):
        self.stop_event.set()

    def emit(self, event):
        # 实时显示可丢旧状态, 原始窗口与请求记录单独保存.
        while True:
            try:
                self.events.put_nowait(event)
                return
            except queue.Full:
                try: self.events.get_nowait()
                except queue.Empty: pass

    @staticmethod
    def drain(q):
        while True:
            try: q.get_nowait()
            except queue.Empty: return

    def run(self):
        port = log = None
        decoder = wire.Decoder()
        seq = time.monotonic_ns() & 0xffff
        failures, status_at = 0, 0.0
        connected = False
        try:
            log = self.log_path.open('x', encoding='utf-8')
            port = self.serial_factory(self.port_name, 115200, timeout=.01, write_timeout=.2)
            port.reset_input_buffer()
            self.token = None
            while not self.quit_event.is_set():
                kind, payload, trace_op = 0x20, b'', None
                if self.stop_event.is_set():
                    self.stop_event.clear()
                    self.token = None
                    self.drain(self.commands); self.drain(self.targets); self.drain(self.excitations)
                    kind = 0x23
                elif not connected:
                    pass
                elif self.reference_event.is_set():
                    self.reference_event.clear()
                    kind = 0x24
                elif not self.trace_commands.empty():
                    action, args = self.trace_commands.get_nowait()
                    if action == 'arm':
                        if self.capture is not None:
                            self.emit(('error', '当前窗口尚未导出; 可先结束采样'))
                            continue
                        self.trace_run = ((self.trace_run+1) & 0xffffffff) or 1
                        count, trigger, note, compact = args
                        self.capture = TraceDownload(self.trace_run, count, trigger,
                            dict(self.metadata, note=note, session_log=str(self.log_path), host_started_at=time.time()), compact)
                    elif self.capture is not None and self.capture.phase in ('arm', 'wait'):
                        self.capture.phase = 'freeze'; self.capture.next_at = 0
                    if self.capture is not None:
                        trace_op, payload = self.capture.request()
                    else:
                        # 采集 ACK 丢失后仍可结束原 run, 不影响控制.
                        trace_op, payload = 2, trace.request(2, self.trace_run)
                    # 先按用户请求预置触发, 再下发已排队的启动/新目标, 不漏掉首个 MCU 事件.
                    kind = 0x26
                elif not self.commands.empty():
                    payload = self.commands.get_nowait(); kind = 0x21
                    self.silence_event.clear()
                elif not self.targets.empty():
                    value, ramp = self.targets.get_nowait()
                    if self.token is None:
                        self.emit(('error', '未运行, 不发送同模式目标更新'))
                        continue
                    kind, payload = 0x25, target_payload(self.token, value, ramp)
                elif not self.excitations.empty():
                    wave = self.excitations.get_nowait()
                    if self.token is None:
                        self.emit(('error', '未运行, 不发送辨识激励'))
                        continue
                    kind, payload = 0x27, struct.pack('<H', self.token)+wave
                elif time.monotonic() >= status_at:
                    if self.token is not None and not self.silence_event.is_set():
                        kind, payload = 0x22, struct.pack('<H', self.token)
                else:
                    if self.capture and time.monotonic() >= self.capture.next_at:
                        trace_op, payload = self.capture.request(); kind = 0x26
                    else:
                        self.quit_event.wait(.003)
                        continue
                seq = (seq+1) & 0xffff
                frame = wire.encode_frame(kind, seq, payload)
                if port.write(frame) != len(frame):
                    raise serial.SerialException('写入失败')
                log.write(json.dumps({'event': 'request', 'host_time': time.time(), 'type': kind,
                                      'seq': seq, 'payload_hex': payload.hex()})+'\n')
                body = None
                deadline = time.monotonic()+.3
                while time.monotonic() < deadline:
                    for msg, reply_seq, received in decoder.feed(port.read(min(max(port.in_waiting, 1), 2048))):
                        if msg == (0xa1 if kind == 0x26 else 0xa0) and reply_seq == seq:
                            body = received
                    if body is not None: break
                if kind == 0x26:
                    try:
                        if body is None:
                            raise ValueError('采集响应超时; 控制/心跳继续, 不以日志故障停机')
                        if self.capture:
                            self.capture.accept(trace_op, body)
                            if self.capture.phase == 'done':
                                if self.capture.data:
                                    path = self.log_path.with_suffix(f'.trace-{self.capture.run:08x}.btrace')
                                    trace.save(path, self.capture.metadata, self.capture.data)
                                    log.write(json.dumps({'event': 'trace_saved', 'path': str(path), 'run': self.capture.run})+'\n')
                                    self.emit(('trace_saved', str(path)))
                                else:
                                    self.emit(('note', '采集已结束, 没有触发样本; 未生成空波形'))
                                self.capture = None
                        else:
                            head, _ = trace.decode_reply(body)
                            self.emit(('note', f'采集结束请求: state={head["state"]}, error={head["error"]}'))
                    except (ValueError, OSError) as exc:
                        self.capture = None
                        self.emit(('error', str(exc)))
                elif body is None:
                    failures += 1
                    self.token = None
                    self.stop_event.set()
                    self.emit(('error', '控制响应超时; 请求停止, 不重发启动'))
                    log.write(json.dumps({'event': 'timeout', 'request': kind, 'seq': seq})+'\n')
                    if failures >= 3:
                        raise serial.SerialException('连续超时; 请检查供电及 S5 固件, 不自动重连')
                else:
                    matched = decode_status(body)
                    connected = True; failures = 0
                    if kind == 0x21:
                        self.token = seq if matched['active'] and matched['token'] == seq and matched['result'] == 1 else None
                    if not matched['active']:
                        self.token = None
                    log.write(json.dumps({'event': 'status', 'host_time': time.time(), 'request': kind, 'seq': seq,
                        'host_bad_frames': decoder.rejected, 'data': matched}, ensure_ascii=False, allow_nan=False)+'\n')
                    self.emit(('status', {'request': kind, 'data': matched, 'received_at': time.monotonic()}))
                if kind != 0x26:
                    status_at = time.monotonic()+.1
                log.flush()
        except (OSError, ValueError, serial.SerialException) as exc:
            self.emit(('error', str(exc)))
        finally:
            if port is not None:
                # 原有退出语义: 尽力请求零输出, 不把成功 write 当作机械停止证明.
                try: port.write(wire.encode_frame(0x23, (seq+1)&0xffff))
                except (OSError, serial.SerialException): pass
                try: port.close()
                except (OSError, serial.SerialException) as exc: self.emit(('error', str(exc)))
            if log is not None:
                try: log.close()
                except OSError as exc: self.emit(('error', str(exc)))
            self.emit(('closed', None))


class Panel:
    def __init__(self, root, port_name, log_dir, metadata=None):
        self.root, self.log_dir, self.metadata = root, log_dir, metadata or {}
        self.session = self.exporter = None
        self.events = queue.Queue(maxsize=64)
        self.last, self.last_time = None, float('-inf')
        self.closing = self.start_pending = False
        root.title('GM6020 S5/v6 · 单电机与控制周期波形')
        root.geometry('1000x760')
        frame = ttk.Frame(root, padding=14); frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='可产生转矩 | 零电流不等于刹停 | 紧急处置使用台架物理断电', foreground='red').pack(anchor='w')
        row = ttk.Frame(frame); row.pack(fill='x', pady=8)
        ttk.Label(row, text='串口').pack(side='left')
        self.port = tk.StringVar(value=port_name)
        ttk.Entry(row, textvariable=self.port, width=12).pack(side='left', padx=5)
        self.connect_button = ttk.Button(row, text='连接 (仅查询)', command=self.connect); self.connect_button.pack(side='left')
        ttk.Button(row, text='断开', command=self.disconnect).pack(side='left', padx=5)
        self.mode = ttk.Combobox(row, values=MODES, state='readonly', width=20); self.mode.current(0); self.mode.pack(side='left', padx=12)
        self.mode.bind('<<ComboboxSelected>>', self.change_mode)
        labels = ['目标 (模式单位)', '运行 s (0=持续)', '电流上限 A', '速度目标上限 rad/s',
                  '速度 Kp (A·s/rad)', '速度 Ki (A/rad)', '位置 Kp (1/s)', '速度 Kd (A·s²/rad)', '反馈 alpha (0=原值)', 'D低通 tau s (0=关闭)']
        defaults = ['0.2', '0', f'{3000*3/16384:.9g}', f'{math.radians(200):.9g}',
                    f'{73*180/math.pi*3/16384:.9g}', '0', '7.5', '0', '0', '0']
        self.entries = []
        grid = ttk.Frame(frame); grid.pack(fill='x')
        for i, (label, value) in enumerate(zip(labels, defaults)):
            ttk.Label(grid, text=label).grid(row=i//2, column=2*(i%2), sticky='w', padx=5, pady=4)
            var = tk.StringVar(value=value); self.entries.append(var)
            ttk.Entry(grid, textvariable=var, width=14).grid(row=i//2, column=2*(i%2)+1, padx=8)
        self.encoder_velocity = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text='使用编码器5点因果测速 (默认原生RPM, 参数按次锁定)', variable=self.encoder_velocity).pack(anchor='w')
        ttk.Label(frame, text='旧主 yaw 换算值, 不是稳定参数. 参数按次锁定; 当前模式内只更新目标, 不重置控制器/参考.').pack(anchor='w', pady=6)
        row = ttk.Frame(frame); row.pack(fill='x')
        self.start_button = ttk.Button(row, text='开始本次测试', command=self.start, state='disabled'); self.start_button.pack(side='left')
        ttk.Button(row, text='停止输出 [Space]', command=self.stop).pack(side='left', padx=6)
        ttk.Button(row, text='建立位置参考', command=self.reference).pack(side='left', padx=6)
        ttk.Button(row, text='测试: 停发心跳', command=self.silence).pack(side='left', padx=6)
        root.bind('<space>', lambda event: self.stop())
        row = ttk.Frame(frame); row.pack(fill='x', pady=7)
        ttk.Label(row, text='线性斜坡 s (0=阶跃)').pack(side='left')
        self.ramp_s = tk.StringVar(value='0'); ttk.Entry(row, textvariable=self.ramp_s, width=9).pack(side='left', padx=6)
        self.target_button = ttk.Button(row, text='更新当前模式目标', command=self.target, state='disabled'); self.target_button.pack(side='left')
        row = ttk.Frame(frame); row.pack(fill='x', pady=5)
        ttk.Label(row, text='1 kHz 采样数').pack(side='left')
        self.samples = tk.StringVar(value='1024'); ttk.Entry(row, textvariable=self.samples, width=7).pack(side='left', padx=5)
        self.trigger = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text='等待下一次启动/目标更新触发', variable=self.trigger).pack(side='left')
        ttk.Button(row, text='采集波形', command=self.capture).pack(side='left', padx=6)
        ttk.Button(row, text='结束采样/导出', command=self.freeze).pack(side='left')
        row = ttk.Frame(frame); row.pack(fill='x', pady=5)
        ttk.Label(row, text='工况/供电/安装备注 (可空)').pack(side='left')
        self.condition = tk.StringVar(); ttk.Entry(row, textvariable=self.condition, width=75).pack(side='left', padx=6)
        self.link = tk.StringVar(value='未连接 / 不会自动使能'); ttk.Label(frame, textvariable=self.link).pack(anchor='w', pady=8)
        self.measurements = tk.StringVar(value='无有效数据')
        ttk.Label(frame, textvariable=self.measurements, font=('Consolas', 11), justify='left').pack(anchor='w')
        self.note = tk.StringVar(value='采集/结束只操作日志, 不改变电机状态. 满窗自动冻结, 后台导出原始文件、CSV、PNG.')
        ttk.Label(frame, textvariable=self.note, wraplength=950).pack(anchor='w', pady=8)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(100, self.update)

    def connect(self):
        if self.session and self.session.is_alive(): return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir / f'gm6020-{time.time_ns()}.jsonl'
        self.last = None; self.last_time = float('-inf'); self.start_pending = False
        Session.drain(self.events)
        self.session = Session(self.port.get(), path, self.events, metadata=self.metadata)
        self.session.start(); self.connect_button.configure(state='disabled')
        self.note.set(f'日志: {path}')

    def change_mode(self, event=None):
        self.stop(); self.entries[0].set(('0.2', '1', '0.5')[self.mode.current()])

    def start(self):
        if not self.session or not self.session.is_alive() or time.monotonic()-self.last_time >= .4 or not self.last: return
        if self.start_pending or self.last['active'] or not self.last['feedback_valid']: return
        if self.mode.current() == 2 and not self.last['position_valid']:
            messagebox.showerror('未发送启动', '位置模式需要有效参考; 请先建立位置参考.'); return
        try:
            self.session.start_motion(settings_payload(self.mode.current()+1, [*[float(v.get()) for v in self.entries], int(self.encoder_velocity.get())]))
            self.start_pending = True; self.start_button.configure(state='disabled')
        except (ValueError, OverflowError, queue.Full) as exc:
            messagebox.showerror('未发送启动', str(exc))

    def target(self):
        if not self.session or not self.last or not self.last['active']: return
        try:
            self.session.update_target(float(self.entries[0].get()), int(float(self.ramp_s.get())*1e6))
        except (ValueError, OverflowError, queue.Full) as exc:
            messagebox.showerror('未发送目标', str(exc))

    def capture(self):
        if not self.session or not self.session.is_alive(): return
        try:
            self.session.start_capture(int(self.samples.get()), self.trigger.get(), self.condition.get())
            self.note.set('已请求采集; 等待触发时可点开始或更新目标. 不改变执行状态.')
        except (ValueError, queue.Full) as exc:
            messagebox.showerror('未开始采集', str(exc))

    def freeze(self):
        if self.session and self.session.is_alive():
            try: self.session.freeze_capture()
            except queue.Full: self.note.set('采集请求队列尚未处理, 请稍后再试')

    def stop(self):
        if self.session: self.session.stop_motion()

    def silence(self):
        if self.session:
            self.session.silence_event.set()
            self.note.set('停止心跳但继续查询; 原有 500 ms 命令时效仍生效, 不自动恢复.')

    def reference(self):
        if self.session and self.session.is_alive():
            self.session.set_reference()
            self.note.set('请求以当前角度建立累计角参考; 与启停分离, 不使能, 不改编码器零点.')

    def disconnect(self):
        if self.session:
            self.session.stop_motion(); self.session.quit_event.set()

    def export(self, path):
        if self.exporter and self.exporter.is_alive():
            self.note.set(f'原始文件已保存: {path}; 绘图器忙, 可用 motor_bench_trace.py 单独导出'); return
        def work():
            try:
                result = ('note', '已导出: '+', '.join(trace.export(path)))
            except Exception as exc:
                result = ('error', f'原始文件已保存, 离线绘图失败: {exc}')
            try: self.events.put_nowait(result)
            except queue.Full: pass
        self.exporter = threading.Thread(target=work, name='bench-plot', daemon=False)
        self.exporter.start()

    def update(self):
        while not self.events.empty():
            kind, value = self.events.get_nowait()
            if kind == 'status':
                self.last = value['data']; self.last_time = value['received_at']
                if value['request'] in (0x21, 0x23): self.start_pending = False
            elif kind in ('error', 'note'):
                self.note.set(value)
            elif kind == 'trace_saved':
                self.note.set(f'原始文件已保存: {value}'); self.export(value)
            elif kind == 'closed':
                self.start_pending = False
                self.last_time = float('-inf'); self.connect_button.configure(state='normal')
        fresh = self.last is not None and time.monotonic()-self.last_time < .4
        alive = self.session is not None and self.session.is_alive()
        self.start_button.configure(state='normal' if fresh and alive and not self.start_pending and not self.last['active'] and self.last['feedback_valid'] else 'disabled')
        self.target_button.configure(state='normal' if fresh and alive and self.last['active'] else 'disabled')
        if fresh and alive:
            d = self.last
            reason = REASONS[d['reason']] if d['reason'] < len(REASONS) else str(d['reason'])
            result = REASONS[d['result']] if d['result'] < len(REASONS) else str(d['result'])
            self.link.set(f'CAN{d["bus"]} ID{d["device_id"]} RX 0x{d["rx_id"]:03x} TX 0x{d["tx_id"]:03x} slot{d["slot"]} | '
                          f'{"输出启用" if d["active"] else "输出禁用"} | {reason} | 请求: {result}')
            if d['feedback_valid']:
                rel = f'{d["relative_rad"]:.4f}' if d['position_valid'] else '未建立/失效'
                self.measurements.set(f'编码器 {d["encoder"]:4d} | 绝对角 {d["absolute_rad"]:.4f} rad | 相对角 {rel} rad\n'
                    f'速度 {d["speed_rad_s"]:9.4f} rad/s | 模式 {MODES[d["mode"]-1]} | 当前目标 {d["target"]:.4f} -> {d["target_final"]:.4f}\n'
                    f'命令电流 {d["output_a"]:8.4f} A | 电流反馈 raw {d["effort_raw"]:6d} | 温度 {d["temperature"]} C\n'
                    f'反馈年龄 {d["feedback_age_ms"]} ms | 参考代次 {d["reference_epoch"]} | 参数代次 {d["settings_epoch"]}\n'
                    f'速度 Kp/Ki/Kd {d["kp"]:.6g}/{d["ki"]:.6g}/{d["kd"]:.6g} | alpha {d["feedback_alpha"]:.4g} | D tau {d["derivative_tau_s"]:.4g} s | 编码器测速 {d["encoder_velocity"]}\n'
                    f'估计速度 {d["estimated_velocity_rad_s"]:.3f} rad/s | 中心年龄 {d["estimated_velocity_age_ms"]} ms | 有效 {d["estimated_velocity_valid"]}\n'
                    f'CAN 入队状态 {d["tx_status"]} | TX 失败累计 {d["tx_errors"]} | MCU {d["ms"]} ms')
            else:
                self.measurements.set('电机反馈失效, 隐藏旧测量; 请检查电机电源/CAN.')
        else:
            self.link.set('离线/数据过期 — 输出状态未知, 必要时使用物理断电')
            self.measurements.set('无有效实时测量')
        if self.closing and not alive:
            self.root.destroy(); return
        self.root.after(100, self.update)

    def close(self):
        self.closing = True; self.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM3')
    parser.add_argument('--log-dir', type=Path, default=Path('build/motor-bench-logs'))
    parser.add_argument('--firmware', type=Path, help='可选: 记录本地固件工件摘要, 不冒充 MCU Flash 回读校验')
    args = parser.parse_args()
    metadata = {}
    if args.firmware:
        metadata['host_firmware_artifact'] = {'path': str(args.firmware), 'sha256': hashlib.sha256(args.firmware.read_bytes()).hexdigest()}
    root = tk.Tk(); Panel(root, args.port, args.log_dir, metadata); root.mainloop()


if __name__ == '__main__':
    main()
