"""从原始短窗日志拟合最小旋转模型. 产物是待验证候选, 不自动写电调或运行配置."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from sentinel_raw import load

AXIS_SLOTS = {'main': 4, 'fire': 5, 'pitch': 9}


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1-np.cos(angle)) * skew @ skew


def reference_axis(axis_sensor, moving, axes, fixed_angles):
    """把不同下游固定姿态的 S 系拟合轴变回 V=S0, 不交换 fire/pitch 次序."""
    result = np.asarray(axis_sensor)
    if moving in ('fire', 'main'):
        result = rotation(axes['pitch'], fixed_angles['pitch']) @ result
    if moving == 'main':
        result = rotation(axes['fire'], fixed_angles['fire']) @ result
    return result / np.linalg.norm(result)


def quality(rows):
    if any(r['gap'] for r in rows):
        raise ValueError('日志缺包, 不跨缺口拟合')
    for kind in ('gyro', 'can'):
        for stream in {r['stream'] for r in rows if r['kind'] == kind}:
            selected = [r for r in rows if r['kind'] == kind and r['stream'] == stream]
            if any(b['t_s'] <= a['t_s'] for a, b in zip(selected, selected[1:])):
                raise ValueError('源样本时间不递增')
            if kind == 'can' and len({r['lost'] for r in selected}) > 1:
                raise ValueError('CAN 队列发生丢帧, 不猜编码器历史')
            if kind == 'gyro' and len({r.get('fifo_overruns', 0) for r in selected}) > 1:
                raise ValueError('gyro FIFO 覆盖, 不能输出假完整标定')


def series(rows, kind):
    subset = [r for r in rows if r['kind'] == kind and r['flags'] & 1]
    if not subset or any(any(v is None for v in r['values']) for r in subset):
        raise ValueError(f'{kind} 数据缺失/非有限')
    return np.array([r['t_s'] for r in subset]), np.array([r['values'] for r in subset])


def first_joint_branch(position, ratio, reference, window_reference=None):
    """以首帧的合法机械区间选唯一同余分支; 窗口锚点也表示为含测量误差的区间."""
    bounds = []
    for item in (reference, window_reference):
        if item is None:
            continue
        if item.get('origin') not in ('measured', 'user_confirmed', 'reference_converted', 'synthetic'):
            raise ValueError('首帧机械参考缺少来源')
        low, high = float(item['min_rad']), float(item['max_rad'])
        if not np.all(np.isfinite([low, high])) or low > high:
            raise ValueError('首帧机械参考区间非法')
        bounds.append((low, high))
    if not bounds:
        raise ValueError('缺少首帧机械参考, 不能猜测编码器圈数')
    low, high = max(b[0] for b in bounds), min(b[1] for b in bounds)
    period = 2*np.pi/ratio
    first = np.ceil((low-position)/period)
    last = np.floor((high-position)/period)
    if first != last or low > high:
        raise ValueError('首帧机械参考无唯一同余分支')
    return position + first*period


def joint_series(rows, axis, spec, window_reference=None):
    if spec['origin'] not in ('measured', 'user_confirmed', 'reference_converted', 'synthetic'):
        raise ValueError('缺少传动/量程来源')
    ratio, sign, zero = float(spec['rotor_per_axis']), spec['direction'], float(spec['zero_offset_rad'])
    if not np.all(np.isfinite([ratio, zero])) or ratio <= 0 or sign not in (-1, 1):
        raise ValueError('非法传动配置')
    selected = [r for r in rows if r['kind'] == 'can' and r['stream'] == AXIS_SLOTS[axis]]
    if len(selected) < 5:
        raise ValueError(f'{axis} 反馈不足')
    times, angles, rates = [], [], []
    for r in selected:
        raw = bytes.fromhex(r['data'])
        if r['dlc'] != 8:
            raise ValueError('CAN DLC 错误')
        if axis == 'pitch':
            if raw[0] & 15 != spec['node'] or raw[0] >> 4 not in (0, 1):
                raise ValueError('DM 身份/状态不适用于标定')
            pmax, vmax = float(spec['pmax_rad']), float(spec['vmax_rad_s'])
            angle = ((raw[1] << 8 | raw[2]) / 65535 * 2 - 1) * pmax
            rate = ((raw[3] << 4 | raw[4] >> 4) / 4095 * 2 - 1) * vmax
        else:
            encoder, rpm = struct.unpack_from('>Hh', raw)
            if encoder >= 8192:
                raise ValueError('非法 DJI 编码角')
            angle, rate = encoder * 2*np.pi/8192, rpm * 2*np.pi/60
        times.append(r['t_s']); angles.append(angle); rates.append(rate)
    times = np.array(times)
    if np.any(np.diff(times) <= 0):
        raise ValueError('关节时间重复或倒退')
    if axis != 'pitch':
        if np.max(np.diff(times)) * float(spec['max_rotor_speed_rad_s']) + 4*np.pi/8192 >= np.pi:
            raise ValueError('编码器展开存在多圈歧义')
        angles = np.unwrap(angles)
    # zero_offset 是轴侧机械零位, 不是把第一帧强制设零.
    positions = sign * np.array(angles) / ratio - zero
    if axis != 'pitch':
        first = first_joint_branch(positions[0], ratio, spec.get('reference'), window_reference)
        positions += first - positions[0]
    native = sign * np.array(rates) / ratio
    return times, positions, native


def stationary(windows, limits):
    all_gyro, temperatures, means = [], [], []
    for rows in windows:
        quality(rows)
        _, gyro = series(rows, 'gyro')
        all_gyro.extend(gyro); means.append(gyro.mean(axis=0))
        temperatures.extend(r['values'][0] for r in rows if r['kind'] == 'temperature' and r['values'][0] is not None)
    gyro = np.array(all_gyro)
    if len(gyro) < 20 or np.max(np.std(gyro, axis=0)) > limits['stationary_std_rad_s']:
        raise ValueError('静止激励不满足输入阈值, 不把运动平均掉作为零偏')
    return dict(bias_S_rad_s=gyro.mean(axis=0).tolist(), std_rad_s=gyro.std(axis=0).tolist(),
                window_means_rad_s=np.array(means).tolist(), sample_count=len(gyro),
                temperature_range_c=[min(temperatures), max(temperatures)] if temperatures else None,
                temperature_model=None)


def fit_axis(samples, bias, lag_bound_s, lag_step_s, limits):
    """samples: (gyro_t, gyro_xyz, encoder_t, encoder_rate), 不从姿态估计器反推安装."""
    if lag_bound_s <= 0 or lag_step_s <= 0 or lag_step_s >= lag_bound_s:
        raise ValueError('时移搜索必须有明确范围与分辨率')
    # 固定所有候选的公共区间, 不以少数边缘点降低某个候选的残差.
    aligned = []
    for gt, gyro, et, rate in samples:
        mask = (gt >= et[0] + lag_bound_s) & (gt <= et[-1] - lag_bound_s)
        if np.count_nonzero(mask) < 20:
            raise ValueError('有效变速片段太短, 增加窗口或缩小有依据的搜索范围')
        aligned.append((gt[mask], gyro[mask] - bias, et, rate))
    results = []
    lags = np.linspace(-lag_bound_s, lag_bound_s, int(round(2*lag_bound_s/lag_step_s))+1)
    for lag in lags:
        x = np.concatenate([np.interp(gt-lag, et, rate) for gt, _, et, rate in aligned])
        y = np.concatenate([gyro for _, gyro, _, _ in aligned])
        energy = float(x @ x)
        if energy <= 0:
            raise ValueError('无旋转激励')
        coefficient = x @ y / energy
        residual = y - x[:, None] * coefficient
        results.append((float(np.mean(np.sum(residual**2, axis=1))), coefficient, x))
    best = int(np.argmin([item[0] for item in results]))
    mse, coefficient, rate = results[best]
    scale = float(np.linalg.norm(coefficient))
    if not np.isfinite(scale) or abs(scale-1) > limits['scale_tolerance']:
        raise ValueError(f'轴侧比例异常 ({scale:.6g}), 检查单位/传动/串动, 不自动补偿')
    if np.std(rate) < limits['min_rate_std_rad_s'] or best in (0, len(lags)-1):
        raise ValueError('时移不可辨识或达到搜索边界')
    rms = float(np.sqrt(mse))
    if rms > limits['axis_rms_rad_s']:
        raise ValueError('单轴残差超出输入阈值, 检查父级运动/轴向/比例')
    return dict(axis_S=(coefficient/scale).tolist(), scale=scale, lag_s=float(lags[best]),
                lag_resolution_s=float(lags[1]-lags[0]), rms_rad_s=rms,
                lag_definition='gyro(t) = axis * encoder_rate(t - lag_s) + bias')


def validate_axis(samples, bias, result):
    residuals = []
    for gt, gyro, et, rate in samples:
        query = gt-result['lag_s']
        mask = (query >= et[0]) & (query <= et[-1])
        if np.count_nonzero(mask) < 20:
            raise ValueError('留出数据不足')
        predicted = np.interp(query[mask], et, rate)[:, None] * np.array(result['axis_S']) * result['scale']
        residuals.extend(gyro[mask] - bias - predicted)
    residual = np.array(residuals)
    return dict(rms_rad_s=float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
                peak_rad_s=float(np.max(np.linalg.norm(residual, axis=1))), samples=len(residual))


def run(manifest, base):
    limits = manifest['quality_limits']
    files = {}
    def read_window(name):
        path = (base/name).resolve()
        metadata, rows = load(path)
        if metadata.get('units', {}).get('gyro') != 'rad/s' or metadata.get('units', {}).get('accel') != 'm/s2':
            raise ValueError('单位声明不符')
        quality(rows)
        files[name] = dict(sha256=hashlib.sha256(path.read_bytes()).hexdigest(), metadata=metadata)
        return rows
    stationary_windows = [read_window(name) for name in manifest['stationary']]
    if any(not files[name]['metadata'].get('stationary_confirmed') for name in manifest['stationary']):
        raise ValueError('静止零偏需要明确的静止标签, 不自动吞掉低速转动')
    bias = stationary(stationary_windows, limits)
    axes = {}
    fits = {}
    for axis in ('pitch', 'fire', 'main'):
        entry = manifest['axes'][axis]
        if set(entry['train']) & set(entry['holdout']):
            raise ValueError('训练与留出不能使用同一日志')
        def collect(names, training=False):
            samples = []
            for name in names:
                rows = read_window(name)
                if not files[name]['metadata'].get('parent_fixed_confirmed') or files[name]['metadata'].get('moving_axis') != axis:
                    raise ValueError('缺少现场单轴/父级固定标签, 不能识别平行轴的混合激励')
                gt, gyro = series(rows, 'gyro')
                fixed_angles = {}
                references = files[name]['metadata'].get('joint_reference', {})
                for other in ('main', 'fire', 'pitch'):
                    jt, position, velocity = joint_series(rows, other, manifest['joints'][other], references.get(other))
                    if other != axis and (np.max(np.abs(velocity)) > limits['fixed_rate_rad_s'] or
                                          np.ptp(position) > limits['fixed_position_rad']):
                        raise ValueError('非被测关节串动')
                    fixed_angles[other] = float(np.mean(position)) - manifest['reference_joint_rad'][other]
                transform = np.eye(3)
                if axis in ('fire', 'main'):
                    transform = rotation(axes['pitch'], fixed_angles['pitch'])
                if axis == 'main':
                    transform = rotation(axes['fire'], fixed_angles['fire']) @ transform
                if training and any(abs(fixed_angles[name]) > limits['fixed_position_rad'] for name in
                                    (('pitch',) if axis == 'fire' else ('fire', 'pitch') if axis == 'main' else ())):
                    raise ValueError('训练下游关节未在参考姿态')
                gyro = (gyro - np.array(bias['bias_S_rad_s'])) @ transform.T
                et, position, native = joint_series(rows, axis, manifest['joints'][axis], references.get(axis))
                # 对称小窗差分, 两端裁去; 位置与原生速度各拟合一次, 不假设滤波时延相等.
                if len(et) < 15:
                    raise ValueError('编码角差分窗口不足')
                # 等间隔不是前提: 每点在真实时间的小窗内拟合局部直线, 降低编码量化噪声.
                derivative = np.array([np.polyfit(et[i-4:i+5]-et[i], position[i-4:i+5], 1)[0]
                                       for i in range(4, len(et)-4)])
                samples.append((gt, gyro, et[4:-4], derivative, native[4:-4]))
            return samples
        train, holdout = collect(entry['train'], True), collect(entry['holdout'])
        if not train or not holdout:
            raise ValueError('轴拟合必须提供独立留出窗口')
        for source, index in (('position', 3), ('velocity', 4)):
            samples = [(s[0], s[1], s[2], s[index]) for s in train]
            result = fit_axis(samples, np.zeros(3), entry['lag_bound_s'], entry['lag_step_s'], limits)
            result['holdout'] = validate_axis([(s[0], s[1], s[2], s[index]) for s in holdout], np.zeros(3), result)
            if result['holdout']['rms_rad_s'] > limits['holdout_rms_rad_s']:
                raise ValueError('留出残差超限, 不发布已通过候选')
            result['axis_V'] = result.pop('axis_S')
            fits[f'{axis}_{source}'] = result
        # 首轮训练在可复现参考姿态进行. 不同下游固定姿态由 reference_axis 单独复核.
        fixed = entry['fixed_relative_angles_rad']
        if any(abs(fixed[name]) > 1e-8 for name in (('pitch',) if axis == 'fire' else ('fire', 'pitch') if axis == 'main' else ())):
            raise ValueError('首轮训练需下游处于参考姿态, 非零姿态用于留出换算')
        axes[axis] = fits[f'{axis}_position']['axis_V']
    return dict(schema=1, status='offline_candidate', deployment_ready=False, frame_V='defined as sensor S at calibration pose',
                topology='main/fire/pitch/IMU', gyro=bias, axes_V=axes, fits=fits,
                reference_joint_rad=manifest['reference_joint_rad'], forward_V=manifest.get('forward_V'),
                joints=manifest['joints'], accel_intrinsics=None, limits=limits, input_files=files,
                residual_scope='single-axis holdout; combined dynamics and physical stability are not certified')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = run(json.loads(args.manifest.read_text(encoding='utf-8')), args.manifest.parent)
    with args.output.open('x', encoding='utf-8') as file:
        json.dump(result, file, ensure_ascii=False, allow_nan=False, indent=2)
        file.write('\n')


if __name__ == '__main__':
    main()
