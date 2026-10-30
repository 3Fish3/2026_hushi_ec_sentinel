"""固定 RAM 原始短窗采集/无损保存/CSV 导出. 本工具只发送观测请求, 不控制电机."""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import secrets
import struct
import sys
import time
from typing import Any

TOOLS = Path(__file__).resolve().parents[3] / 'boards/sentinel_f407/tools'
sys.path.insert(0, str(TOOLS))
from board_monitor import Decoder, encode_frame

MAGIC = b'ECRAW1\r\n'
VERSION, LAYOUT, SIZE = 1, 0x52415701, 64
KINDS = {1: 'gyro', 2: 'accel', 3: 'temperature', 4: 'can', 5: 'control'}
PROFILES = {'stationary': 1, 'rotation': 2, 'control': 3}


def delta(value, origin):
    """短窗内的有符号微秒差; 回绕只扩展一次, 不把复位当回绕."""
    return ((value - origin + 0x80000000) & 0xffffffff) - 0x80000000


def decode_record(raw):
    if len(raw) != SIZE:
        raise ValueError('原始记录截断')
    version, kind, stream, flags, seq, source, sampled, arrived, lost, error, run = struct.unpack_from('<4B7I', raw)
    if version != VERSION or kind not in KINDS or flags & ~7:
        raise ValueError('原始记录版本/类型/标志不匹配')
    row: dict[str, Any] = {
        'kind': KINDS[kind], 'stream': stream, 'flags': flags, 'sequence': seq,
        'source_sequence': source, 'sampled_us': sampled, 'arrived_us': arrived,
        'lost': lost, 'error_us': None if error == 0xffffffff else error, 'run': run,
    }
    if kind == 4:
        info = struct.unpack_from('<I', raw, 32)[0]
        row.update(bus=info & 255, dlc=(info >> 8) & 255, can_id=info >> 16, data=raw[36:44].hex())
    else:
        values = struct.unpack_from('<6f' if kind == 5 else '<3f', raw, 32)
        row['values'] = [x if math.isfinite(x) else None for x in values]
        if kind == 1:
            row['fifo_overruns'] = struct.unpack_from('<I', raw, 44)[0]
        if kind == 5:
            row['state_bits'], row['send_result'] = struct.unpack_from('<2I', raw, 56)
            frames = {0: 'joint', 1: 'local_inertial', 2: 'odom'}
            row['requested_frame'] = frames.get((row['state_bits'] >> 25) & 3, 'unknown')
            row['adopted_frame'] = frames.get((row['state_bits'] >> 27) & 3, 'unknown')
            if stream == 6:
                # 域 6 专用于 Pitch 达妙 DM 关节.
                # 6 个 float 物理量依次为:
                # requested joint rad, adopted joint rad, measured joint rad,
                # measured velocity rad/s, encoded MIT feedforward Nm (无指令尝试时为 NaN sentinel), dt.
                # 严禁将 values[4] 标注为归一化电流!
                v = row['values']
                row['pitch_requested_rad'] = v[0]
                row['pitch_adopted_rad'] = v[1]
                row['pitch_measured_rad'] = v[2]
                row['pitch_measured_velocity_rad_s'] = v[3]
                row['pitch_feedforward_torque_nm'] = v[4]
                row['pitch_dt_s'] = v[5]
                row['units'] = {
                    'pitch_requested_rad': 'rad',
                    'pitch_adopted_rad': 'rad',
                    'pitch_measured_rad': 'rad',
                    'pitch_measured_velocity_rad_s': 'rad/s',
                    'pitch_feedforward_torque_nm': 'N*m',
                    'pitch_dt_s': 's',
                }
                # word6 状态字额外解析: bit29 激励激活, bit30 锁存
                row['excitation_active'] = bool(row['state_bits'] & (1 << 29))
                row['latched'] = bool(row['state_bits'] & (1 << 30))
    return row


def save(path, metadata, records):
    for record in records:
        decode_record(record)
    header = json.dumps(metadata, ensure_ascii=False, allow_nan=False).encode('utf-8')
    with Path(path).open('xb') as file:
        file.write(MAGIC + struct.pack('<I', len(header)) + header)
        for record in records:
            file.write(record)


def load(path):
    with Path(path).open('rb') as file:
        if file.read(8) != MAGIC:
            raise ValueError('不是 ECRAW1 文件')
        n = file.read(4)
        if len(n) != 4:
            raise ValueError('文件头截断')
        size = struct.unpack('<I', n)[0]
        if size > 1048576:
            raise ValueError('元数据长度超限')
        try:
            metadata = json.loads(file.read(size))
        except Exception as e:
            raise ValueError(f'元数据解析失败: {e}') from e
        if metadata.get('layout_id') != LAYOUT:
            raise ValueError('原始采样布局不匹配')
        blob = file.read()
    if len(blob) % SIZE:
        raise ValueError('原始记录截断')
    rows = [decode_record(blob[i:i+SIZE]) for i in range(0, len(blob), SIZE)]
    previous = -1
    for row in rows:
        if row['run'] != metadata['run'] or row['sequence'] <= previous:
            raise ValueError('短窗身份混入或记录重排')
        row['gap'] = row['sequence'] - previous - 1
        row['t_s'] = delta(row['sampled_us'], metadata['start_us']) * 1e-6
        row['arrival_s'] = delta(row['arrived_us'], metadata['start_us']) * 1e-6
        previous = row['sequence']
    return metadata, rows


def request(op, run=0, profile=0, arg=0, domain=0):
    return struct.pack('<4B2I', VERSION, op, profile, domain, run, arg)


def decode_reply(payload):
    if len(payload) < 32:
        raise ValueError('原始短窗响应截断')
    v, op, error, state, run, profile, count, total, start, end, lost, layout, capacity = struct.unpack_from('<4BI2BH5I', payload)
    if v != VERSION or layout != LAYOUT or len(payload) != 32 + count * SIZE:
        raise ValueError('原始短窗响应布局不匹配')
    if error:
        raise ValueError(f'原始短窗请求被拒绝: {error}')
    return {'op': op, 'state': state, 'run': run, 'profile': profile, 'total': total, 'start_us': start, 'end_us': end,
            'lost': lost, 'layout_id': layout, 'capacity': capacity}, [payload[i:i+SIZE] for i in range(32, len(payload), SIZE)]


class RawClient:
    def __init__(self, port):
        self.port, self.decoder, self.sequence = port, Decoder(), 0
        self.last_send = 0.0

    def exchange_wire(self, payload, kind=0x31, response=0xb1):
        # 同一个 run/op/index 重传不清空缓存, 不推进机械动作.
        self.sequence = (self.sequence + 1) & 65535
        for _ in range(3):
            time.sleep(max(0.0, self.last_send + 0.065 - time.monotonic()))
            self.port.write(encode_frame(kind, self.sequence, payload))
            self.last_send = time.monotonic()
            end = self.last_send + 0.4
            while time.monotonic() < end:
                for rx_type, rx_sequence, rx_payload in self.decoder.feed(self.port.read(512)):
                    if rx_type == response and rx_sequence == self.sequence:
                        return rx_payload
        raise TimeoutError('原始短窗无响应, 未重开串口或重置设备')

    def exchange(self, payload):
        return decode_reply(self.exchange_wire(payload))

    def identity(self):
        payload = self.exchange_wire(b'', 1, 0x81)
        if len(payload) < 72:
            raise ValueError('观测身份响应截断')
        return {'build_id': payload[40:72].split(b'\0', 1)[0].decode('ascii'),
                'uptime_ms': struct.unpack_from('<I', payload)[0]}

    def capture(self, profile, duration_us, domain=0):
        run = secrets.randbelow(0xffffffff) + 1
        status, _ = self.exchange(request(1, run, PROFILES[profile], duration_us, domain))
        deadline = time.monotonic() + duration_us*1e-6 + 2
        while status['state'] == 1:
            if time.monotonic() > deadline:
                raise TimeoutError('MCU 采集时间未推进, 不重置设备')
            time.sleep(0.065)
            status, _ = self.exchange(request(0))
            if status['run'] != run:
                raise ValueError('采集过程中短窗被替换')
        records = []
        while len(records) < status['total']:
            page, batch = self.exchange(request(3, run, arg=len(records)))
            if page['run'] != run or not batch:
                raise ValueError('冻结窗口已失效')
            for record in batch:
                row = decode_record(record)
                if row['sequence'] != len(records) or row['run'] != run:
                    raise ValueError('导出记录存在缺口或身份不符')
                records.append(record)
        return status, records


def export_csv(input_path, output_path):
    metadata, rows = load(input_path)
    with Path(output_path).open('x', newline='', encoding='utf-8') as file:
        fields = ['kind', 'stream', 't_s', 'arrival_s', 'sequence', 'source_sequence', 'lost', 'error_us', 'gap',
                  'data', 'values', 'requested_frame', 'adopted_frame', 'state_bits', 'send_result',
                  'pitch_requested_rad', 'pitch_adopted_rad', 'pitch_measured_rad',
                  'pitch_measured_velocity_rad_s', 'pitch_feedforward_torque_nm', 'pitch_dt_s',
                  'excitation_active', 'latched']
        writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    return len(rows), metadata.get('run')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    acquire = sub.add_parser('capture')
    acquire.add_argument('--port', required=True)
    acquire.add_argument('--baud', type=int, default=115200)
    acquire.add_argument('--profile', choices=PROFILES, default='stationary')
    acquire.add_argument('--duration-ms', type=int, default=200)
    acquire.add_argument('--domain', type=int, choices=(0, 1, 2, 3, 4, 5, 6, 7, 8, 9), default=4,
                         help='control 轨迹域编号: 0-3 底盘轮, 4 主云台 Yaw (DJI), 5 发射云台 Yaw (DJI), 6 Pitch 达妙 (DM), 7-8 摩擦轮 (DJI), 9 拨弹 (DJI)')
    acquire.add_argument('--metadata', type=Path, help='固件/配置身份及工况标签 JSON, 未知项保留 null')
    acquire.add_argument('--output', type=Path, required=True)
    export = sub.add_parser('export')
    export.add_argument('input', type=Path)
    export.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'capture':
        import serial
        if not 0 < args.duration_ms <= 2000:
            parser.error('duration-ms 范围 1..2000, 缓存满会提前冻结')
        metadata = {}
        if args.metadata:
            try:
                metadata = json.loads(args.metadata.read_text(encoding='utf-8'))
            except Exception as e:
                parser.error(f'无法解析元数据 JSON 文件: {e}')
        with serial.Serial(args.port, args.baud, timeout=0.02) as port:
            client = RawClient(port)
            identity = client.identity()
            status, records = client.capture(args.profile, args.duration_ms * 1000,
                                              args.domain if args.profile == 'control' else 0)
        metadata.update(status, schema=VERSION, timebase='TIM5_us32', units={'gyro': 'rad/s', 'accel': 'm/s2', 'temperature': 'degC'},
                        axes='BMI088 sensor S (unrotated, bias not removed)', firmware=metadata.get('firmware'),
                        configuration=metadata.get('configuration'), device_identity=identity, window_requested_us=args.duration_ms*1000,
                        imu={'gyro_odr_hz': 2000, 'gyro_bw_hz': 230, 'accel_odr_hz': 800, 'accel_range_g': 6,
                             'gyro_range_deg_s': 2000, 'gyro_fifo': 'last of at most 4, other frames counted',
                             'accel_filter': 'normal', 'synchronized': False}, control_domain=args.domain)
        save(args.output, metadata, records)
        print(json.dumps(status, ensure_ascii=False))
    else:
        count, run = export_csv(args.input, args.output)
        print(f'{count} records, run={run}; CSV 是派生数据, 保留原始文件')


if __name__ == '__main__':
    main()
