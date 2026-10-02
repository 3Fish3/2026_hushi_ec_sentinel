"""把原始短窗与标定候选送进原生 SentinelPerception; 不执行串口、烧录或配置写入."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np
from sentinel_raw import load
from sentinel_calibrate import joint_series, quality


def native_input(calibration, metadata, rows, projection, gain, accel_error):
    if calibration['schema'] != 1 or calibration['topology'] != 'main/fire/pitch/IMU':
        raise ValueError('标定版本/拓扑不符')
    if calibration.get('forward_V') is None:
        raise ValueError('主平台前方向未确认, 不能生成有效 heading')
    quality(rows)
    axes = calibration['axes_V']
    # 标定输出的 S 就是原始传感器系; V=S0 是定义, 不再叠加一次安装旋转.
    values = [*axes['fire'], *axes['pitch'], *axes['main'], *calibration['forward_V'],
              *calibration['gyro']['bias_S_rad_s'],
              calibration['reference_joint_rad']['fire'], calibration['reference_joint_rad']['pitch']]
    values += [round(calibration['fits'][f'{axis}_{source}']['lag_s']*1e6)
               for source in ('position', 'velocity') for axis in ('fire', 'pitch')]
    values += [projection, gain, accel_error]
    if not np.all(np.isfinite(values)):
        raise ValueError('配置含非有限值')
    lines = ['SERIAL1 ' + ' '.join(map(str, values))]
    # 借 N2 已验证换算器解码原始 CAN, 保留独立采样时刻. 不从姿态输出反求输入.
    joints = {}
    for axis, stream in (('fire', 5), ('pitch', 9)):
        _, position, velocity = joint_series(rows, axis, calibration['joints'][axis],
                                             metadata.get('joint_reference', {}).get(axis))
        samples = [r for r in rows if r['kind'] == 'can' and r['stream'] == stream]
        for row, p, v in zip(samples, position, velocity):
            joints[row['sequence']] = (axis, p, v)
    # 原始短窗可能先写新 IMU 再倒入较早的 CAN 队列. 按真实到达时刻排序,
    # 原生端逐时刻推进; 不将未来到达的 CAN 提前用于旧 gyro.
    for row in sorted(rows, key=lambda r: r['arrival_s']):
        at = int(row['sampled_us'])
        arrived = int(row['arrived_us'])
        kind = row['kind']
        if kind in ('gyro', 'accel'):
            if not row['flags'] & 1 or any(v is None for v in row['values']):
                continue
            lines.append(f"{kind} {at} {arrived} {row['source_sequence']} " + ' '.join(map(str, row['values'])))
        elif row['sequence'] in joints:
            axis, p, v = joints[row['sequence']]
            lines.append(f'{axis} {at} {arrived} 0 {p} {v}')
    lines.append('end')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calibration', type=Path)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--native', type=Path, required=True, help='ec_sentinel_replay 可执行文件')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--min-projection-squared', type=float, required=True)
    parser.add_argument('--min-heading-gain', type=float, required=True)
    parser.add_argument('--accel-norm-error-mps2', type=float, required=True)
    args = parser.parse_args()
    model = json.loads(args.calibration.read_text(encoding='utf-8'))
    metadata, rows = load(args.recording)
    payload = native_input(model, metadata, rows, args.min_projection_squared,
                           args.min_heading_gain, args.accel_norm_error_mps2)
    result = subprocess.run([str(args.native.resolve())], input=payload, capture_output=True, text=True, check=True)
    with args.output.open('x', encoding='utf-8') as file:
        file.write(result.stdout)


if __name__ == '__main__':
    main()
