"""S5/v4 冻结波形的文件、CSV 与离线绘图. 不打开串口、不发送电机请求."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np

LAYOUT = 0x42545231
CAPACITY = 1024
IDENT_LAYOUT = 0x42494431
IDENT_RECORD = struct.Struct('<IIfHhhBBHBB')
IDENT_CAPACITY = 65536//IDENT_RECORD.size
RECORD = struct.Struct('<4I2H3h2B8f2BH')
HEADER = struct.Struct('<4BI4H3I2BH')
FILE_HEADER = struct.Struct('<4sI')
FIELDS = ('control_us', 'feedback_us', 'submit_us', 'dt_us', 'feedback_sequence', 'encoder',
          'rpm', 'effort_raw', 'command_raw', 'flags', 'mode_reason', 'target', 'position',
          'velocity_used', 'velocity_target', 'p_norm', 'i_norm', 'd_norm', 'unconstrained_norm',
          'tx_status', 'axis_status', 'cycle_sequence')


def record_format(metadata):
    return IDENT_RECORD if metadata.get('layout') == IDENT_LAYOUT else RECORD


def request(op, run=0, arg=0, trigger=0, compact=False):
    return struct.pack('<4BII', 2 if compact else 1, op, trigger, int(compact), run, arg)


def decode_reply(payload):
    if len(payload) < HEADER.size:
        raise ValueError('波形响应头截断')
    keys = ('version', 'op', 'error', 'state', 'run', 'size', 'count', 'capacity', 'goal',
            'first_us', 'last_us', 'arg', 'trigger', 'reason', 'meta_size')
    head = dict(zip(keys, HEADER.unpack_from(payload)))
    compact = head['version'] == 2
    capacity = IDENT_CAPACITY if compact else CAPACITY
    if (head['version'] not in (1, 2) or head['capacity'] != capacity or head['state'] > 3
            or head['size'] > capacity or head['count'] > (8 if compact else 3) or head['goal'] > capacity):
        raise ValueError('不兼容的波形布局/容量')
    size = head['count']*(IDENT_RECORD.size if compact else RECORD.size) + head['meta_size']
    if len(payload) != HEADER.size+size:
        raise ValueError('波形响应长度不匹配')
    return head, payload[HEADER.size:]


def save(path, metadata, data):
    record = record_format(metadata)
    if len(data) % record.size or len(data) > 65536:
        raise ValueError('波形数据长度非法')
    meta = dict(metadata, layout=metadata.get('layout', LAYOUT), record_bytes=record.size, records=len(data)//record.size,
                records_sha256=hashlib.sha256(data).hexdigest())
    encoded = json.dumps(meta, ensure_ascii=False, allow_nan=False).encode('utf-8')
    with Path(path).open('xb') as output:
        output.write(FILE_HEADER.pack(b'BTR1', len(encoded)))
        output.write(encoded)
        output.write(data)
    return meta


def load(path):
    raw = Path(path).read_bytes()
    if len(raw) < FILE_HEADER.size:
        raise ValueError('波形文件截断')
    magic, length = FILE_HEADER.unpack_from(raw)
    if magic != b'BTR1' or length > len(raw)-FILE_HEADER.size:
        raise ValueError('波形文件头非法')
    meta = json.loads(raw[FILE_HEADER.size:FILE_HEADER.size+length])
    data = raw[FILE_HEADER.size+length:]
    record = record_format(meta)
    if (meta['layout'] not in (LAYOUT, IDENT_LAYOUT) or meta['record_bytes'] != record.size
            or not 0 <= meta['records'] <= 65536//record.size
            or len(data) != meta['records']*record.size or hashlib.sha256(data).hexdigest() != meta['records_sha256']):
        raise ValueError('波形文件布局/长度/摘要不匹配')
    return meta, data


def ident_rows(metadata, data):
    """长窗不保存PID/位置参考/提交时刻; 缺失字段不填伪造测量, 保留全部控制周期."""
    names = ('control_us', 'feedback_us', 'target', 'encoder', 'rpm', 'command_raw',
             'flags', 'mode_reason', 'cycle_sequence', 'tx_status', 'axis_status')
    result, previous, elapsed = [], None, 0
    for values in IDENT_RECORD.iter_unpack(data):
        r = dict(zip(names, values))
        if not math.isfinite(r['target']): raise ValueError('非有限目标')
        dt = ((r['control_us']-previous['control_us']) & 0xffffffff) if previous else None
        if dt is not None:
            if dt >= 0x80000000: raise ValueError('长窗时间不连续')
            elapsed += dt
        flags = r['flags']
        r.update(time_s=elapsed*1e-6, mode=r['mode_reason'] & 3, reason=r['mode_reason'] >> 4,
                 active=bool(flags & 1), feedback_valid=bool(flags & 2),
                 saturated=bool(flags & 16), target_event=bool(flags & 32),
                 reference_event=bool(flags & 64), tx_attempted=bool(flags & 128),
                 dt_us=dt, pid_valid=False, position_valid=False,
                 outer_saturated=bool(r['mode_reason'] & 4), ramping=bool(r['mode_reason'] & 8))
        r['encoder_rad'] = r['encoder']*2*math.pi/8192 if r['feedback_valid'] else None
        r['raw_velocity_rad_s'] = r['rpm']*2*math.pi/60 if r['feedback_valid'] else None
        r['feedback_age_us'] = (r['control_us']-r['feedback_us']) & 0xffffffff if r['feedback_valid'] else None
        r['command_a'] = r['command_raw']*3/16384 if r['tx_attempted'] and r['tx_status'] == 1 else None
        r['record_gaps'] = ((r['cycle_sequence']-previous['cycle_sequence']-1) & 0xffff) if previous else 0
        for key in ('position', 'velocity_used', 'velocity_target', 'p_a', 'i_a', 'd_a',
                    'unconstrained_a', 'submit_latency_us'):
            r[key] = None
        result.append(r); previous = r
    return result


def rows(metadata, data):
    """保留原始字段; 无效/未提交不是0, 派生量留空, 不跨未知微秒时间拼曲线."""
    if metadata.get('layout') == IDENT_LAYOUT:
        return ident_rows(metadata, data)
    cap = metadata['start']['current_limit']
    result = []
    elapsed = 0
    previous = None
    for values in RECORD.iter_unpack(data):
        row = dict(zip(FIELDS, values))
        if not all(math.isfinite(row[k]) for k in FIELDS[11:19]):
            raise ValueError('波形含非有限控制量')
        if previous is not None:
            gap = (row['control_us']-previous['control_us']) & 0xffffffff
            if gap >= 0x80000000:
                raise ValueError('波形微秒时间不连续, 不能猜测回绕圈数')
            elapsed += gap
        flags = row['flags']
        row.update(time_s=elapsed*1e-6, mode=row['mode_reason'] & 3, reason=row['mode_reason'] >> 4,
                   active=bool(flags & 1), feedback_valid=bool(flags & 2), position_valid=bool(flags & 4),
                   pid_valid=bool(flags & 8), saturated=bool(flags & 16), target_event=bool(flags & 32),
                   reference_event=bool(flags & 64), tx_attempted=bool(flags & 128),
                   outer_saturated=bool(row['mode_reason'] & 4), ramping=bool(row['mode_reason'] & 8))
        row['encoder_rad'] = row['encoder']*2*math.pi/8192 if row['feedback_valid'] else None
        row['raw_velocity_rad_s'] = row['rpm']*2*math.pi/60 if row['feedback_valid'] else None
        row['feedback_age_us'] = (row['control_us']-row['feedback_us']) & 0xffffffff if row['feedback_valid'] else None
        row['submit_latency_us'] = (row['submit_us']-row['control_us']) & 0xffffffff if row['tx_attempted'] else None
        row['command_a'] = row['command_raw']*3/16384 if row['tx_attempted'] and row['tx_status'] == 1 else None
        for name in ('p', 'i', 'd'):
            row[name+'_a'] = row[name+'_norm']*cap if row['pid_valid'] else None
        row['unconstrained_a'] = row['unconstrained_norm']*cap if row['pid_valid'] or row['mode'] == 1 and row['active'] else None
        row['record_gaps'] = ((row['cycle_sequence']-previous['cycle_sequence']-1) & 0xffff) if previous else 0
        result.append(row)
        previous = row
    return result


def metrics(data_rows):
    """窗口统计而非自动验收线; 不把命令电流称为实测电流, 不冒充完整工况 WCET."""
    if not data_rows:
        return {'samples': 0}
    result = dict(samples=len(data_rows), duration_s=data_rows[-1]['time_s'],
                  record_gaps=sum(r['record_gaps'] for r in data_rows),
                  max_dt_us=max((r['dt_us'] for r in data_rows if r['dt_us'] is not None), default=None),
                  tx_not_accepted=sum(r['tx_status'] != 1 for r in data_rows))
    active = [r for r in data_rows if r['active']]
    result['saturation_sample_fraction'] = sum(r['saturated'] for r in active)/len(active) if active else None
    for label, values in (
            ('velocity_error_rad_s', [r['velocity_target']-r['velocity_used'] for r in active if r['pid_valid']]),
            ('position_error_rad', [r['target']-r['position'] for r in active if r['mode'] == 3 and r['position_valid']])):
        if values:
            array = np.asarray(values, dtype=np.float64)
            result[label] = dict(mean=float(np.mean(array)), rms=float(np.sqrt(np.mean(array*array))),
                                 peak_to_peak=float(np.ptp(array)), max_abs=float(np.max(np.abs(array))))
    return result


def export(path, *, plot=True):
    meta, data = load(path)
    decoded = rows(meta, data)
    if not decoded:
        raise ValueError('没有采样记录, 不生成空性能图')
    csv_path = Path(str(path)+'.csv')
    with csv_path.open('w', newline='', encoding='utf-8-sig') as output:
        writer = csv.DictWriter(output, fieldnames=decoded[0].keys())
        writer.writeheader()
        writer.writerows(decoded)
    summary_path = Path(str(path)+'.summary.json')
    summary_path.write_text(json.dumps(metrics(decoded), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    outputs = [str(csv_path), str(summary_path)]
    if plot:
        # 离线 Agg 后端不接管 Tk, 不在串口/心跳线程调用.
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import pyplot as plt
        figure, axes = plt.subplots(5, 1, sharex=True, figsize=(12, 12))
        t = [r['time_s'] for r in decoded]
        def series(key, valid=None):
            return [r[key] if (valid is None or r[valid]) and r[key] is not None else float('nan') for r in decoded]
        axes[0].plot(t, series('encoder_rad'), label='single-turn encoder [rad]', alpha=.7)
        axes[0].plot(t, series('position', 'position_valid'), label='joint position [rad]')
        axes[0].plot(t, [r['target'] if r['mode'] == 3 else float('nan') for r in decoded], label='position target [rad]')
        axes[0].set_ylabel('rad')
        axes[1].plot(t, series('raw_velocity_rad_s'), label='native RPM -> rad/s', alpha=.7)
        axes[1].plot(t, series('velocity_used', 'pid_valid'), label='inner-loop measurement')
        axes[1].plot(t, series('velocity_target', 'pid_valid'), label='inner-loop target')
        axes[1].set_ylabel('rad/s')
        for key, label in [('p_a', 'P'), ('i_a', 'I'), ('d_a', 'D'), ('unconstrained_a', 'before clamp'), ('command_a', 'CAN accepted command')]:
            axes[2].plot(t, series(key), label=label)
        axes[2].set_ylabel('command A')
        for key in ('dt_us', 'feedback_age_us', 'submit_latency_us'):
            axes[3].plot(t, series(key), label=key)
        axes[3].set_ylabel('us')
        for key in ('active', 'saturated', 'outer_saturated', 'reason'):
            axes[4].step(t, series(key), where='post', label=key)
        axes[4].set_ylabel('state / reason code')
        axes[4].set_xlabel('MCU elapsed s (not host arrival)')
        for row in decoded:
            if row['target_event'] or row['reference_event'] or row['tx_status'] != 1:
                for axis in axes:
                    axis.axvline(row['time_s'], color='orange' if row['tx_status'] == 1 else 'red', alpha=.25)
        for axis in axes:
            axis.grid(True, alpha=.25)
            axis.legend(loc='upper right', fontsize=8)
        figure.suptitle(f"S5 ID{meta['start']['device_id']} | run {meta['run']} | {meta.get('build_id', '')}\n"
                       'Command current only; local CAN accepted != physical current or bus completion')
        figure.tight_layout()
        image = Path(str(path)+'.png')
        figure.savefig(image, dpi=140)
        plt.close(figure)
        outputs.append(str(image))
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()
    for output in export(args.trace, plot=not args.no_plot):
        print(output)


if __name__ == '__main__':
    main()
