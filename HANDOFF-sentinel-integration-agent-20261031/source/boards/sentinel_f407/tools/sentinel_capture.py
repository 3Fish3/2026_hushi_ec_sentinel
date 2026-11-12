"""哨兵无主动驱动采集: DJI 被动反馈 + DM 限定参数读. 人工标签仅写 PC 日志."""
from __future__ import annotations
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time
import uuid

from board_monitor import Decoder, encode_frame

SCHEMA = 3  # 3: sensors 页 remote 段增加 ch4

NAMES = ('LF', 'RF', 'LB', 'RB', 'main_yaw', 'fire_yaw', 'friction_left', 'friction_right', 'feeder', 'pitch_status')
REGISTERS = ((0x07, 'MST_ID', False), (0x08, 'ESC_ID', False), (0x09, 'TIMEOUT', False),
             (0x0a, 'CTRL_MODE', False), (0x0e, 'sw_ver', False), (0x14, 'Gr', True),
             (0x15, 'PMAX', True), (0x16, 'VMAX', True), (0x17, 'TMAX', True),
             (0x24, 'sub_ver', False), (0x50, 'p_m_rad', True), (0x51, 'xout_rad', True))
SUMMARY = ('signature', 'clock_hz', 'reset_flags', 'rx_frames', 'queue_dropped', 'ignored',
           'fifo_overrun_can1', 'fifo_overrun_can2', 'errors_can1', 'errors_can2', 'queue_high_water',
           'max_queue_age_ms', 'max_fast_period_ms', 'max_fast_cost_cycles', 'queries', 'query_busy',
           'query_errors', 'query_timeouts', 'unsolicited', 'unmapped', 'malformed', 'fast_stack_words',
           'unknown_bus', 'unknown_id')
LINK = ('rx_bytes', 'tx_bytes', 'overflows', 'discarded', 'framing', 'noise', 'overrun', 'restarts',
        'tx_errors', 'busy_rejects', 'last_hal_error', 'bad_frames', 'unknown_requests', 'response_drops')


class CaptureProtocolError(ValueError):
    """固件与上位机契约不匹配, 不能按另一版布局猜测解码."""


def sample_state(seen, decoded_valid, age_ms, ttl_ms, unseen='NO_FEEDBACK'):
    if not seen: return unseen
    if age_ms >= ttl_ms: return 'LOST'
    return 'ONLINE' if decoded_valid else 'INVALID'


def display_state(row, extra_ms, ttl_ms, link_state):
    # 设备失联只由 MCU 快照判定; PC 页面老化不能冒充电机停发.
    if link_state != 'ONLINE': return 'LINK_' + link_state + ' (cached)'
    if row['state'] == 'ONLINE' and row['age_ms'] + extra_ms >= ttl_ms:
        return 'ONLINE / SNAPSHOT_OLD'
    return row['state']


def sensor_text(data, extra_ms, link_state):
    # 固定列宽和精度; 状态描述 MCU 快照, 页面老化用年龄表达, 不反复插入状态后缀.
    r, m = data['remote'], data['imu']
    def state(sample):
        return sample['state'] if link_state == 'ONLINE' else 'LINK_' + link_state
    def age(sample, seen):
        return f'{sample["age_ms"] + extra_ms:7.0f}' if seen else '     --'
    channels = ' '.join(f'{r[k]:+5d}' for k in ('ch0', 'ch1', 'ch2', 'ch3', 'ch4')) if r['frames'] else '   -- ' * 5
    gyro = ' '.join(f'{m[k]:+9.4f}' if m[k] is not None else '       --' for k in ('gx', 'gy', 'gz')) if m['samples'] else '       -- ' * 3
    switches = f'{r["left"]}/{r["right"]}' if r['frames'] else '-- '
    keys = f'0x{r["keys"]:04X}' if r['frames'] else '    --'
    return (f'DR16 snapshot={state(r):<16} age={age(r, r["frames"])} ms | ch={channels} '
            f'sw={switches} keys={keys}\n'
            f'IMU  snapshot={state(m):<16} age={age(m, m["samples"])} ms | gyro(rad/s)={gyro} heater={m["heater"]}')


def decode_page(payload: bytes) -> dict:
    if len(payload) < 8:
        raise ValueError('short capture header')
    version, page, count, reserved, now = struct.unpack_from('<4BI', payload)
    lengths = {0: 192, **{p: 112 for p in range(1, 6)}, 6: 101, **{p: 88 for p in range(7, 10)}}
    expected_count = 2 if 1 <= page <= 5 else 4 if 7 <= page <= 9 else 1
    if version != SCHEMA or reserved or page not in lengths or len(payload) != lengths[page] or count != expected_count:
        raise CaptureProtocolError(
            f'capture protocol mismatch: schema={version} (expected {SCHEMA}), page={page}, '
            f'length={len(payload)} (expected {lengths.get(page)}), count={count} (expected {expected_count}); '
            'flash the matching sentinel_capture.bin, not sentinel_s1.bin')
    result = {'schema': version, 'page': page, 'now_ms': now}
    if page == 0:
        result['summary'] = dict(zip(SUMMARY, struct.unpack_from('<24I', payload, 8)))
        if result['summary']['signature'] != 0x43415031 or result['summary']['clock_hz'] != 168000000:
            raise ValueError('not the expected F407 capture role/clock')
        result['build_id'] = payload[104:136].split(b'\0', 1)[0].decode('ascii', errors='strict')
        result['uart6'] = dict(zip(LINK, struct.unpack_from('<14I', payload, 136)))
    elif 1 <= page <= 5:
        motors = []
        for j in range(2):
            row = struct.unpack_from('<BBHBBH7IQ8s', payload, 8+j*52)
            bus, kind, can_id, node, dlc, flags = row[:6]
            if kind not in (1, 2, 3, 4) or flags & ~3 or dlc > 8:
                raise ValueError('invalid motor metadata')
            raw = row[-1]
            m = dict(zip(('received', 'invalid', 'received_ms', 'cycles', 'min_cycles', 'max_cycles', 'intervals', 'sum_cycles'), row[6:-1]))
            m.update(name=NAMES[(page-1)*2+j], bus=bus, kind=kind, can_id=can_id, node=node, dlc=dlc,
                     seen=bool(flags & 1), decoded_valid=bool(flags & 2), raw_hex=raw.hex(),
                     age_ms=(now-m['received_ms']) & 0xffffffff)
            m['valid'] = m['seen'] and m['decoded_valid'] and m['age_ms'] < (500 if kind == 4 else 100)
            m.update(encoder=None, rpm=None, rotor_rad=None, rotor_rad_s=None, effort_raw=None, temperature_c=None,
                     p16=None, v12=None, t12=None, status=None, rotor_temperature_c=None,
                     joint_rad=None, cumulative_rotor_rad=None)
            if m['seen'] and m['decoded_valid']:
                if kind != 4:
                    encoder, rpm, effort = struct.unpack_from('>Hhh', raw)
                    m.update(encoder=encoder, rpm=rpm, rotor_rad=encoder*2*math.pi/8192,
                             rotor_rad_s=rpm*2*math.pi/60, effort_raw=effort,
                             temperature_c=raw[6] if kind != 2 else None)
                else:
                    m.update(p16=(raw[1] << 8) | raw[2], v12=(raw[3] << 4) | (raw[4] >> 4),
                             t12=((raw[4] & 15) << 8) | raw[5], status=raw[0] >> 4,
                             temperature_c=raw[6], rotor_temperature_c=raw[7])
            m['state'] = sample_state(m['seen'], m['decoded_valid'], m['age_ms'], 500 if kind == 4 else 100)
            motors.append(m)
        result['motors'] = motors
    elif page >= 7:
        values = []
        for j in range(4):
            rid, floating, seen, valid, responses, received, bits, requested = struct.unpack_from('<4B4I', payload, 8+j*20)
            spec = REGISTERS[(page-7)*4+j]
            if rid != spec[0] or floating != spec[2] or seen not in (0, 1) or valid not in (0, 1):
                raise ValueError('invalid register metadata')
            value = struct.unpack('<f', struct.pack('<I', bits))[0] if floating else bits
            finite = not floating or math.isfinite(value)
            age = (now-received) & 0xffffffff
            values.append(dict(rid=rid, name=spec[1], seen=bool(seen), decoded_valid=bool(valid and finite),
                               valid=bool(seen and valid and finite and age < (500 if rid >= 0x50 else 5000)),
                               responses=responses, received_ms=received, request_ms=requested, age_ms=age,
                               bits=bits, value=value if seen and valid and finite else None,
                               state=sample_state(seen, valid and finite, age, 500 if rid >= 0x50 else 5000, 'NO_REPLY')))
        result['registers'] = values
    else:
        offset = 8
        r = struct.unpack_from('<3I8hH5B', payload, offset)
        offset += struct.calcsize('<3I8hH5B')
        remote = dict(zip(('frames', 'rejected', 'received_ms', 'ch0', 'ch1', 'ch2', 'ch3', 'ch4',
                           'mouse_x', 'mouse_y', 'mouse_z', 'keys', 'left', 'right', 'online',
                           'mouse_left', 'mouse_right'), r))
        remote['age_ms'] = (now-remote['received_ms']) & 0xffffffff
        remote['valid'] = bool(remote['frames'] and remote['online'] and remote['age_ms'] < 100)
        m = struct.unpack_from('<3I2B7f', payload, offset)
        offset += struct.calcsize('<3I2B7f')
        imu = dict(zip(('samples', 'errors', 'received_ms', 'ready', 'heater', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c'), m))
        finite = all(math.isfinite(imu[k]) for k in ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c'))
        for k in ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c'):
            if not math.isfinite(imu[k]): imu[k] = None
        imu['age_ms'] = (now-imu['received_ms']) & 0xffffffff
        imu['valid'] = bool(imu['samples'] and imu['ready'] and finite and imu['age_ms'] < 50)
        remote['state'] = sample_state(remote['frames'], remote['online'], remote['age_ms'], 100)
        imu['state'] = sample_state(imu['samples'], imu['ready'] and finite, imu['age_ms'], 50,
                                    'INIT_FAILED' if imu['errors'] else 'NO_SAMPLE')
        result.update(remote=remote, imu=imu, uart3=dict(zip(('framing', 'noise', 'overrun', 'overflows'), struct.unpack_from('<4I', payload, offset))))
    return result


class CaptureLog:
    def __init__(self, path: Path | None, metadata: dict):
        self.file = path.open('x', encoding='utf-8') if path else None
        self.enabled = True
        self.emit({'event': 'header', 'schema': 1, 'capture_schema': SCHEMA, 'recording_id': str(uuid.uuid4()), **metadata})

    def emit(self, record):
        if self.file and self.enabled:
            self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
            self.file.flush()

    def close(self):
        if self.file: self.file.close()


class Client:
    """串口唯一拥有者, 仅发分页快照请求; PC 没有 CAN 透传/查询配置/电机控制命令."""
    def __init__(self, port, hz, emit, clock=time.monotonic):
        self.port, self.hz, self.emit, self.clock = port, hz, emit, clock
        self.decoder = Decoder()
        self.begin = clock()
        self.next_send = self.begin
        self.seq = 0
        self.next_page = 0
        self.focus_page = None
        self.mark_request = None
        self.sent_count = 0
        self.pending = None
        self.pages = {}
        self.previous_ms = None
        self.segment = 0
        self.timeouts = 0
        self.last_response = None
        self.link_state = 'WAITING'
        self.status = 'waiting for capture identity'

    def set_link_state(self, state):
        if state != self.link_state:
            self.emit({'event': 'link_state', 'previous': self.link_state, 'state': state,
                       'host_elapsed_s': self.clock()-self.begin})
        self.link_state = state
        self.status = state + ' (read-only polling; no port reopen or device reset)'

    def tick(self):
        now = self.clock()
        if self.pending and now-self.pending[2] >= 0.5:
            self.emit({'event': 'response_timeout', 'sequence': self.pending[0], 'page': self.pending[1]})
            self.pending = None
            self.timeouts += 1
            self.set_link_state('LOST' if self.last_response is not None else 'NO_RESPONSE')
        if self.pending is None and now >= self.next_send:
            self.seq = (self.seq+1) & 0xffff
            # 选中轴时优先刷新该页, 每五次请求仍保留一次后台轮询.
            if 0 not in self.pages:
                page = 0
                self.next_page = 1
            elif self.mark_request:
                page = self.mark_request[0]
            elif self.focus_page is not None and self.sent_count % 5 != 0:
                page = self.focus_page
            else:
                page = self.next_page
                self.next_page = (self.next_page+1) % 10
            self.sent_count += 1
            wire = encode_frame(0x30, self.seq, bytes([page]))
            if self.port.write(wire) != len(wire): raise OSError('short serial request write')
            self.pending = self.seq, page, now
            self.next_send = now+1/self.hz
        data = self.port.read(min(max(self.port.in_waiting, 1), 4096))
        for kind, seq, payload in self.decoder.feed(data):
            now = self.clock()
            if not self.pending or kind != 0xb0 or seq != self.pending[0] or now-self.pending[2] >= 0.5:
                continue
            decoded = decode_page(payload)
            if decoded['page'] != self.pending[1]: raise ValueError('response page mismatch')
            ms = decoded['now_ms']
            if self.previous_ms is not None and (ms-self.previous_ms) & 0xffffffff >= 0x80000000:
                self.pages.clear()
                self.segment += 1
                self.emit({'event': 'mcu_time_discontinuity', 'segment': self.segment, 'now_ms': ms})
            self.previous_ms = ms
            self.last_response = now
            self.set_link_state('ONLINE')
            record = {'event': 'page', 'host_elapsed_s': now-self.begin, 'segment': self.segment,
                      'sequence': seq, 'host_bad_frames': self.decoder.rejected, 'data': decoded}
            self.pages[decoded['page']] = (now, record)
            self.emit(record)
            self.pending = None
            self.timeouts = 0
            if self.mark_request and decoded['page'] == self.mark_request[0]:
                _, label, reference, requested_host = self.mark_request
                self.mark_request = None
                self.mark(label, reference, requested_host)
            self.status = 'ONLINE (pages have independent sample times)'

    def request_mark(self, page, label, reference):
        if not 0 <= page < 10: raise ValueError('invalid marker page')
        self.mark_request = (page, label, reference, self.clock()-self.begin)
        self.next_send = self.clock()

    def mark(self, label, reference, requested_host=None):
        now = self.clock()
        # 附上所有已见页面及其样本身份, 不以点击时间改写采样时刻或标定 MCU.
        samples = []
        for page, (received, record) in sorted(self.pages.items()):
            sample = copy.deepcopy(record['data'])
            sample['response_sequence'] = record['sequence']
            sample['host_page_age_s'] = now-received
            for key in ('motors', 'registers'):
                for row in sample.get(key, []):
                    ttl = (500 if row['kind'] == 4 else 100) if key == 'motors' else 500 if row['rid'] >= 0x50 else 5000
                    row['valid_at_mark'] = row['valid'] and row['age_ms'] + (now-received)*1000 < ttl
            samples.append(sample)
        record = {'event': 'mark', 'label': label, 'reference': reference, 'segment': self.segment,
                  'host_elapsed_s': now-self.begin, 'requested_host_elapsed_s': requested_host, 'pages': samples}
        self.emit(record)
        return record


def export_csv(source: Path, target: Path):
    fields = ('event', 'host_elapsed_s', 'segment', 'sequence', 'page', 'name', 'state', 'valid', 'received_ms',
              'age_ms', 'raw_hex', 'encoder', 'rpm', 'rotor_rad', 'rotor_rad_s', 'effort_raw',
              'temperature_c', 'rid', 'bits', 'value', 'label', 'reference')
    with source.open(encoding='utf-8') as src, target.open('x', newline='', encoding='utf-8-sig') as dst:
        writer = csv.DictWriter(dst, fields, extrasaction='ignore')
        writer.writeheader()
        for line in src:
            record = json.loads(line)
            if record['event'] in ('mark', 'link_state'): writer.writerow(record)
            if record['event'] != 'page': continue
            data = record['data']
            for row in data.get('motors', []) + data.get('registers', []):
                writer.writerow({**record, 'page': data['page'], **row})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--baud',type=int,default=115200,help='旧镜像115200; 连续采集镜像1000000')
    parser.add_argument('--log', type=Path, help='新 JSONL 文件, 不覆盖; 面板可暂停/恢复写日志')
    parser.add_argument('--elf', type=Path, help='对应采集 ELF, 哈希记入日志, 不读取 SWD')
    parser.add_argument('--hardware', default='unconfirmed', help='本次机械/接线版本或标签')
    parser.add_argument('--hz', type=float, default=10, help='总分页请求速率 1..15 Hz, 不是每轴采样率')
    parser.add_argument('--seconds', type=float, default=0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--export', type=Path, help='离线读取 JSONL 导出 CSV, 不打开串口')
    parser.add_argument('--csv', type=Path)
    args = parser.parse_args()
    if args.export:
        if not args.csv: parser.error('--export requires --csv')
        export_csv(args.export, args.csv)
        return 0
    if not args.port: parser.error('--port is required')
    if not math.isfinite(args.hz) or not 1 <= args.hz <= 15 or not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('require 1 <= hz <= 15 and seconds >= 0')
    import serial
    log = CaptureLog(args.log, {'role': 'sentinel_capture', 'port': args.port, 'baud': args.baud,
                               'hardware': args.hardware, 'elf_sha256': hashlib.sha256(args.elf.read_bytes()).hexdigest() if args.elf else None,
                               'wall_time_ns': time.time_ns(), 'dm_queries': 'STD 0x7FF DLC4: 01 00 33 RID (whitelist) or 01 00 CC 00 (status)' })
    port = None
    def emit(record):
        log.emit(record)
        if args.headless: print(json.dumps(record, ensure_ascii=False, allow_nan=False), flush=True)
    try:
        port = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.2, rtscts=False, dsrdtr=False)
        port.reset_input_buffer()
        client = Client(port, args.hz, emit)
        if args.headless:
            while not args.seconds or time.monotonic()-client.begin < args.seconds:
                client.tick()
                time.sleep(0.005)
            return 0 if 0 in client.pages and client.link_state == 'ONLINE' else 1
        import tkinter as tk
        from tkinter import ttk
        root = tk.Tk()
        root.title('Sentinel capture | NO DRIVE | partial-device observation')
        status = tk.StringVar(value=client.status)
        ttk.Label(root, textvariable=status).pack(fill='x')
        tree = ttk.Treeview(root, columns=('id', 'raw', 'value', 'age', 'quality'), show='tree headings', height=22)
        tree.heading('#0', text='来源'); tree.column('#0', width=150)
        for key, title in (('id', 'CAN / RID'), ('raw', '原始编码/位值'), ('value', '角 rad / RPM / DM 状态'), ('age', '样本年龄 ms'), ('quality', '质量')):
            tree.heading(key, text=title); tree.column(key, width=155)
        tree.pack(fill='both', expand=True)
        for name in NAMES + tuple(r[1] for r in REGISTERS): tree.insert('', 'end', iid=name, text=name)
        def select(_event):
            selected = tree.selection()
            if not selected: return
            name = selected[0]
            client.focus_page = 1+NAMES.index(name)//2 if name in NAMES else 7+tuple(r[1] for r in REGISTERS).index(name)//4
        tree.bind('<<TreeviewSelect>>', select)
        views = ttk.Frame(root); views.pack(fill='x')
        def focus(page): client.focus_page = page
        ttk.Button(views, text='全车轮询', command=lambda: focus(None)).pack(side='left')
        ttk.Button(views, text='遥控/IMU 优先', command=lambda: focus(6)).pack(side='left')
        ttk.Label(views, text='选择表中一行可优先读取; 未标定的关节角/多圈角不推算').pack(side='left')
        sensors = tk.StringVar(value='DR16 / IMU: waiting')
        ttk.Label(root, textvariable=sensors, font='TkFixedFont', justify='left', anchor='w').pack(fill='x')
        row = ttk.Frame(root); row.pack(fill='x')
        label, reference = tk.StringVar(), tk.StringVar()
        ttk.Label(row, text='机械标记').pack(side='left')
        ttk.Entry(row, textvariable=label, width=24).pack(side='left')
        ttk.Label(row, text='独立量角/单位/备注').pack(side='left')
        ttk.Entry(row, textvariable=reference, width=35).pack(side='left')
        def mark():
            if not log.file or not log.enabled:
                status.set('标签未保存: 请用 --log 指定文件并恢复记录')
                return
            client.request_mark(client.focus_page if client.focus_page is not None else 1, label.get(), reference.get())
            status.set('等待该页新快照后记录标签, 不修改设备零位')
        ttk.Button(row, text='记录标记', command=mark).pack(side='left')
        toggle = ttk.Button(row)
        def pause():
            if not log.file: return
            if log.enabled:
                log.emit({'event': 'recording_paused', 'host_elapsed_s': time.monotonic()-client.begin})
                log.enabled = False
            else:
                log.enabled = True
                log.emit({'event': 'recording_resumed', 'host_elapsed_s': time.monotonic()-client.begin})
            toggle.configure(text='暂停记录' if log.enabled else '恢复记录')
        toggle.configure(text='暂停记录', command=pause, state='normal' if log.file else 'disabled')
        toggle.pack(side='left')
        failed = False
        visible_segment = -1
        next_sensor_render = 0.0
        def refresh():
            nonlocal failed, visible_segment, next_sensor_render
            try:
                client.tick()
                now = time.monotonic()
                if args.seconds and now-client.begin >= args.seconds:
                    root.destroy(); return
                if visible_segment != client.segment:
                    for name in tree.get_children(): tree.item(name, values=('--', '--', '--', '--', 'UNKNOWN'))
                    sensors.set('DR16 / IMU: waiting for current segment')
                    next_sensor_render = 0.0
                    visible_segment = client.segment
                summary = client.pages.get(0)
                if summary:
                    s = summary[1]['data']['summary']
                    status.set(f'{client.status} | RX={s["rx_frames"]} drop={s["queue_dropped"]} max queue={s["queue_high_water"]} '
                               f'| DM reads={s["queries"]} timeout={s["query_timeouts"]} | ' + ('LOG ON' if log.file and log.enabled else 'LOG OFF'))
                else:
                    status.set(f'{client.status} | 等待当前分段的采集固件身份')
                for received, record in client.pages.values():
                    data = record['data']
                    extra_ms = (now-received)*1000
                    for m in data.get('motors', []):
                        valid = m['seen'] and m['decoded_valid']
                        raw = m['encoder'] if m['kind'] != 4 else m['p16']
                        value = f'{m["rotor_rad"]:.5f}; {m["rpm"]}' if valid and m['rotor_rad'] is not None else '--'
                        if valid and m['kind'] == 4: value = f'state={m["status"]} v12={m["v12"]}'
                        tree.item(m['name'], values=(f'CAN{m["bus"]}/0x{m["can_id"]:03X}', raw if raw is not None else '--', value,
                                  int(m['age_ms']+extra_ms) if m['seen'] else '--',
                                  display_state(m, extra_ms, 500 if m['kind'] == 4 else 100, client.link_state)))
                    for r in data.get('registers', []):
                        valid = r['seen'] and r['decoded_valid']
                        tree.item(r['name'], values=(f'0x{r["rid"]:02X}', f'0x{r["bits"]:08X}' if r['seen'] else '--',
                                  r['value'] if valid else '--', int(r['age_ms']+extra_ms) if r['seen'] else '--',
                                  display_state(r, extra_ms, 500 if r['rid'] >= 0x50 else 5000, client.link_state)))
                    if 'remote' in data and now >= next_sensor_render:
                        sensors.set(sensor_text(data, extra_ms, client.link_state))
                        next_sensor_render = now + 0.2
                root.after(10, refresh)
            except (OSError, ValueError, serial.SerialException) as exc:
                failed = True
                event = 'protocol_error' if isinstance(exc, CaptureProtocolError) else 'disconnected'
                emit({'event': event, 'reason': str(exc)})
                status.set(f'{event.upper()}: {exc}')
                for name in tree.get_children():
                    values = list(tree.item(name, 'values'))
                    if values: values[-1] = event.upper() + ' (cached)'; tree.item(name, values=values)
                sensors.set('DR16 / IMU: LINK_LOST (last values in log)')
                # 保留窗口供查看错误, 不重连、不复位设备.
        root.after(0, refresh)
        root.mainloop()
        return 2 if failed else 0
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, serial.SerialException) as exc:
        emit({'event': 'protocol_error' if isinstance(exc, CaptureProtocolError) else 'disconnected', 'reason': str(exc)})
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        if port is not None: port.close()
        log.close()


if __name__ == '__main__':
    raise SystemExit(main())
