"""保留原始缺包证据的分段离线拟合; 不改变严格标定入口或发布实车验收结论。"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.optimize import least_squares

import sentinel_calibrate as cal
import sentinel_dataset_calibrate as data
import sentinel_dataset_replay as replay
import sentinel_raw as raw

AXES = data.AXES


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def continuous_parts(rows):
    """保持原始记录及序号; 缺口后的首条及两侧时间边界不参与拟合。"""
    parts, current = [], []
    for row in rows:
        if row['gap'] and current:
            parts.append(current)
            current = []
        current.append(row)
    if current:
        parts.append(current)
    return [part[1:] if part[0]['gap'] else part for part in parts]


def source_quality(rows):
    """在切段前检查所有源, 不能借传输缺口掩盖源丢帧、复位或时钟倒退。"""
    for row in rows:
        if row['kind'] in ('gyro', 'accel'):
            if not row['flags'] & 1 or any(v is None for v in row['values']):
                raise ValueError('IMU源样本无效或非有限')
    for kind in ('gyro', 'can'):
        for stream in {r['stream'] for r in rows if r['kind'] == kind}:
            selected = [r for r in rows if r['kind'] == kind and r['stream'] == stream]
            if any(b['t_s'] <= a['t_s'] for a, b in zip(selected, selected[1:])):
                raise ValueError('源样本时间不递增')
            if kind == 'can' and len({r['lost'] for r in selected}) > 1:
                raise ValueError('CAN队列发生丢帧')
            if kind == 'gyro' and len({r.get('fifo_overruns', 0) for r in selected}) > 1:
                raise ValueError('gyro FIFO覆盖')


class Dataset:
    def __init__(self, manifest, root, allow_wire_gaps=False):
        if manifest.get('schema') != 'sentinel-observed-calibration/1':
            raise ValueError('数据清单版本不符')
        self.manifest = manifest
        self.base = Path(root) / manifest['dataset']
        self.allow_wire_gaps = allow_wire_gaps
        self.ranges = data.mapping_ranges(self.base / manifest['mapping_log'])
        if self.ranges['master'] != manifest['sources']['pitch']['can_id']:
            raise ValueError('DM反馈地址与真实回读不符')
        self.files = {}

    def load(self, name):
        if name in self.files:
            return self.files[name]
        path = self.base / name
        metadata, rows = raw.load(path)
        summary_path = Path(str(path) + '.summary.json')
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
        if metadata.get('units') != {'gyro': 'rad/s', 'accel': 'm/s2', 'temperature': 'degC'}:
            raise ValueError('输入单位不符')
        if not rows or summary.get('failure') or summary.get('diagnostics_error') or summary.get('source_issues'):
            raise ValueError('输入为空或板端诊断异常')
        final = summary['final']
        if final['state'] != 3 or final['buffered'] or final['dropped']:
            raise ValueError('板端采集未结束或记录队列丢弃')
        missing = final['generated'] - len(rows)
        counted = sum(r['gap'] for r in rows) + max(0, final['generated'] - rows[-1]['sequence'] - 1)
        if missing < 0 or missing != counted or missing != summary['record_gaps']:
            raise ValueError('记录序号、生成数与摘要缺口不一致')
        if summary['received']['records'] != len(rows) or final['run'] != metadata['run']:
            raise ValueError('摘要条数或run与原始记录不符')
        wire_problem = bool(missing or summary['crc_errors'])
        if summary['complete'] == wire_problem:
            raise ValueError('摘要完整性标记与缺口/CRC不一致')
        if not self.allow_wire_gaps:
            cal.quality(rows)
            if not summary['complete'] or summary['crc_errors']:
                raise ValueError('输入传输不完整; 分段分析必须显式指定--allow-wire-gaps')
        source_quality(rows)
        parts = [part for part in continuous_parts(rows) if part]
        for part in parts:
            cal.quality(part)
        result = dict(metadata=metadata, rows=rows, parts=parts, summary=summary,
                      sha256=digest(path), summary_sha256=digest(summary_path))
        self.files[name] = result
        return result

    def provenance(self):
        return {name: dict(sha256=f['sha256'], summary_sha256=f['summary_sha256'],
                           run=f['metadata']['run'], complete=f['summary']['complete'],
                           record_gaps=f['summary']['record_gaps'], crc_errors=f['summary']['crc_errors'],
                           parts=len(f['parts'])) for name, f in self.files.items()}


def static_stats(dataset, item, guard):
    f = dataset.load(item['file'])
    if f['metadata']['profile'] != 'stationary':
        raise ValueError('静止输入的profile不是stationary')
    samples = []
    for part in f['parts']:
        lo = max(item['interval_s'][0], part[0]['t_s'] + guard)
        hi = min(item['interval_s'][1], part[-1]['t_s'] - guard)
        samples.extend(r['values'] for r in part if r['kind'] == 'gyro' and r['flags'] & 1
                       and lo <= r['t_s'] <= hi and all(v is not None for v in r['values']))
    if len(samples) < 100:
        raise ValueError('连续静止样本不足')
    values = np.asarray(samples)
    return dict(file=item['file'], samples=len(values), mean_rad_s=values.mean(axis=0).tolist(),
                std_rad_s=values.std(axis=0).tolist())


def read_reference(dataset, path):
    reference = json.loads(Path(path).read_text(encoding='utf-8'))
    if reference.get('schema') != 'sentinel-observed-reference-pose/1':
        raise ValueError('对齐参考记录版本不符')
    f = dataset.load(reference['capture_file'])
    if f['sha256'] != reference['capture_sha256']:
        raise ValueError('对齐参考原始数据哈希不符')
    lo, hi = reference['selected_interval_s']
    rows = [r for r in f['rows'] if lo <= r['t_s'] <= hi]
    cal.quality(rows)
    # 三轴必须来自同一真实时间窗, 不能用各自不同次的对齐数值拼接。
    joint = [data.decode_native(rows, a, dataset.manifest['sources'][a], dataset.ranges) for a in AXES]
    native = np.array([np.median(q) for t, q, v in joint])
    for axis, value in zip(AXES, native):
        if abs(value - reference['joints'][axis]['native_median_rad']) > 1e-8:
            raise ValueError('对齐参考角度与原始记录不符')
    t, acc = cal.series(rows, 'accel')
    up = acc.mean(axis=0)
    if np.linalg.norm(up) < 1:
        raise ValueError('对齐参考加速度无有效重力方向')
    return reference, native, up / np.linalg.norm(up)


def motion_windows(dataset, items, bias, guard, lag_bound, stride=1):
    windows, omitted = [], []
    for item in items:
        f = dataset.load(item['file'])
        if f['metadata']['profile'] != 'rotation':
            raise ValueError('运动输入的profile不是rotation')
        count = 0
        for number, part in enumerate(f['parts']):
            if len(part) < 150:
                omitted.append(dict(file=item['file'], part=number, reason='too_few_records'))
                continue
            joints = [data.decode_native(part, a, dataset.manifest['sources'][a], dataset.ranges) for a in AXES]
            derived = [data.derivative(t, q, dataset.manifest['analysis']['derivative_half_window'])
                       for t, q, v in joints]
            lo = max(item['interval_s'][0], part[0]['t_s'] + guard,
                     *(t[0] + lag_bound for t, v in derived))
            hi = min(item['interval_s'][1], part[-1]['t_s'] - guard,
                     *(t[-1] - lag_bound for t, v in derived))
            selected = [r for r in part if r['kind'] == 'gyro' and r['flags'] & 1 and lo <= r['t_s'] <= hi
                        and all(v is not None for v in r['values'])][::stride]
            if len(selected) < 25:
                omitted.append(dict(file=item['file'], part=number, reason='too_few_guarded_gyro_samples'))
                continue
            # 之后prediction的所有t±lag均在当前段的真实导数覆盖内。
            count += 1
            windows.append(dict(name=item['file'] + '#' + str(number), file=item['file'], part=number,
                                axis=item['dominant_axis'], rows=part, joints=joints, derived=derived,
                                start_us=f['metadata']['start_us'],
                                t=np.array([r['t_s'] for r in selected]),
                                gyro=np.array([r['values'] for r in selected]) - bias, interval=[lo, hi]))
        if not count:
            raise ValueError('没有可用连续片段: ' + item['file'])
    return windows, omitted


def aggregate(windows, axes, lags, reference, source='position', position_lags=None):
    result = {}
    for axis in AXES:
        selected = [w for w in windows if w['axis'] == axis]
        error = np.concatenate([data.prediction(w, axes, lags, reference, source, position_lags) - w['gyro']
                                for w in selected])
        actual = np.concatenate([w['gyro'] for w in selected])
        magnitude = np.linalg.norm(error, axis=1)
        energy = np.sum((actual - actual.mean(axis=0)) ** 2)
        if energy <= 0:
            raise ValueError('留出或训练没有可辨识运动: ' + axis)
        result[axis] = dict(samples=len(actual), rms_rad_s=float(np.sqrt(np.mean(magnitude ** 2))),
                            p95_rad_s=float(np.quantile(magnitude, .95)),
                            peak_rad_s=float(magnitude.max()), r2=float(1 - np.sum(error ** 2) / energy))
    return result


def horizontal_forward(up, pitch_axis):
    """粗机械参考: 枪轴水平且垂直pitch铰轴, 正pitch抬头; 不是光学测向。"""
    forward = np.cross(up, pitch_axis)
    if np.linalg.norm(forward) < .1:
        raise ValueError('pitch轴接近重力方向, 无法形成水平前向候选')
    forward /= np.linalg.norm(forward)
    if np.dot(np.cross(pitch_axis, forward), up) <= 0:
        raise ValueError('前向符号不满足正pitch抬头')
    return forward


def initial_axes(windows, reference):
    values = {}
    for axis in ('pitch', 'fire', 'main'):
        i = AXES.index(axis)
        selected = [w for w in windows if w['axis'] == axis]
        x = np.concatenate([np.interp(w['t'], *w['derived'][i]) for w in selected])
        y = np.concatenate([w['gyro'] for w in selected])
        if x @ x < .01:
            raise ValueError('轴激励不足: ' + axis)
        vector = x @ y / (x @ x)
        # 初值只用训练数据, 按各窗中位下游姿态作近似补偿; 最终由完整三轴模型联合求解。
        positions = []
        for j in (1, 2):
            positions.append(float(np.median(np.concatenate([np.interp(w['t'], w['joints'][j][0],
                                                                       w['joints'][j][1]) for w in selected]))))
        if axis in ('fire', 'main'):
            vector = data.rotate(values['pitch'], np.array([positions[1] - reference[2]]), vector)[0]
        if axis == 'main':
            vector = data.rotate(values['fire'], np.array([positions[0] - reference[1]]), vector)[0]
        values[axis] = vector / np.linalg.norm(vector)
    return np.array([values[a] for a in AXES])


def solve(dataset, reference_path, guard=.06):
    manifest = dataset.manifest
    reference_record, reference, up = read_reference(dataset, reference_path)
    static = static_stats(dataset, manifest['stationary_train'], guard)
    bias = np.array(static['mean_rad_s'])
    noise = float(np.linalg.norm(static['std_rad_s']))
    if not np.isfinite(noise) or noise <= 0:
        raise ValueError('静止噪声尺度无效')
    stationary_holdout = [static_stats(dataset, item, guard) for item in manifest['stationary_holdout']]
    bound = manifest['analysis']['lag_bound_s']
    train, skipped_train = motion_windows(dataset, manifest['train'], bias, guard, bound)
    holdout, skipped_holdout = motion_windows(dataset, manifest['holdout'], bias, guard, bound)
    tf, hf = {w['file'] for w in train}, {w['file'] for w in holdout}
    if tf & hf or {dataset.files[f]['sha256'] for f in tf} & {dataset.files[f]['sha256'] for f in hf}:
        raise ValueError('训练与留出必须是不同原始文件和内容')
    initial = initial_axes(train, reference)
    angles = np.concatenate([data.axis_angles(a) for a in initial])

    def axes_of(p):
        return np.array([data.axis_vector(p[2*i:2*i+2]) for i in range(3)])

    fitted = least_squares(lambda p: data.residual(train, axes_of(p), p[6:] * .001, reference, 'position').ravel(),
                           np.r_[angles, [0., 0., 0.]],
                           bounds=(np.r_[np.full(6, -np.inf), np.full(3, -bound*1000)],
                                   np.r_[np.full(6, np.inf), np.full(3, bound*1000)]),
                           loss='soft_l1', f_scale=noise, max_nfev=160, x_scale='jac')
    if not fitted.success:
        raise ValueError('联合拟合未收敛: ' + fitted.message)
    axes, lags = axes_of(fitted.x), fitted.x[6:] * .001
    held = least_squares(lambda p: data.residual(holdout, axes_of(p), lags, reference, 'position').ravel(),
                         fitted.x[:6], loss='soft_l1', f_scale=noise, max_nfev=100)
    if not held.success:
        raise ValueError('独立轴向重复性检查未收敛')
    held_axes = axes_of(held.x)
    forward = horizontal_forward(up, axes[2])
    candidate = dict(schema='sentinel-segmented-site-candidate/1', status='compiled_candidate_not_vehicle_validated',
                     deployment_ready=False, axes_V=dict(zip(AXES, axes.tolist())), forward_V=forward.tolist(),
                     install=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], reference_native_rad=dict(zip(AXES, reference.tolist())),
                     gyro=dict(training=static, holdout=stationary_holdout, bias_S_rad_s=bias.tolist()),
                     sources=manifest['sources'], dm_ranges=dataset.ranges,
                     fits=dict(position=dict(estimated_lags_s=dict(zip(AXES, lags.tolist())),
                                             lags_identified=False, train=aggregate(train, axes, lags, reference),
                                             holdout=aggregate(holdout, axes, lags, reference)),
                               zero_lag=dict(train=aggregate(train, axes, np.zeros(3), reference),
                                             holdout=aggregate(holdout, axes, np.zeros(3), reference))),
                     heldout_axis_difference_deg={a: float(np.degrees(np.arccos(np.clip(x @ y, -1, 1))))
                                                  for a, x, y in zip(AXES, axes, held_axes)},
                     reference_pose=reference_record, reference_pose_sha256=digest(reference_path),
                     provenance=dataset.provenance(), mapping_log=manifest['mapping_log'],
                     mapping_sha256=digest(dataset.base / manifest['mapping_log']),
                     selected_segments={key: [dict(file=w['file'], part=w['part'], interval_s=w['interval'],
                                                  gyro_samples=len(w['t'])) for w in ws]
                                        for key, ws in [('train', train), ('holdout', holdout)]},
                     omitted_segments=skipped_train + skipped_holdout,
                     method=dict(guard_s=guard, lag_bound_s=bound, derivative_half_window=manifest['analysis']['derivative_half_window'],
                                 optimizer='soft_l1 joint body-rate fit', holdout_refit_applied=False,
                                 raw_files_changed=False, old_strict_gate_changed=False),
                     firmware_lag_policy=dict(position_lag_us=[0, 0], velocity_lag_us=[0, 0],
                                              meaning='Explicit zero-lag candidate baseline; not measured hardware zero latency.'),
                     assumptions=['Operator-confirmed main and fire forward alignment and horizontal gun pose is a coarse mechanical reference.',
                                  'Horizontal gun direction is assumed perpendicular to the pitch hinge; positive native pitch raises the gun.',
                                  'The base remained stationary during manual joint recordings; no independent optical truth.',
                                  'install identity defines V=S at the aligned reference pose; raw-sensor bias is subtracted once.'],
                     limitations=['All original CRC/gap evidence is retained; full-file strict calibration remains unchanged.',
                                  'Fitted position lags are not identified hardware delays and are not applied to firmware.',
                                  'No full accelerometer intrinsic, thermal, absolute-heading or actual control-stability acceptance.',
                                  'Raw RPM replay does not reproduce the APP five-point causal GM6020 encoder estimator.'],
                     tool_sha256={Path(p).name: digest(p) for p in (__file__, data.__file__, cal.__file__, raw.__file__)})
    return candidate, train, holdout


def native_replay(candidate, windows, executable, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    axes = np.array([candidate['axes_V'][a] for a in AXES])
    reference = np.array([candidate['reference_native_rad'][a] for a in AXES])
    zero = {a: {'estimated_s': 0.0} for a in ('fire', 'pitch')}
    shim = dict(candidate, fits={'position': {'lags': zero}, 'velocity': {'lags': zero}})
    report = dict(schema='sentinel-segmented-native-replay/1', deployment_ready=False,
                  native_sha256=digest(executable), cases={}, note='Per-segment reset; no heading or real-vehicle claim.')
    for axis in AXES:
        selected = [w for w in windows if w['axis'] == axis]
        report['cases'][axis] = {}
        for variant in ('native_zero', 'derived_diagnostic'):
            for budget in (5000, 10000):
                total, valid, errors, all_rows = 0, 0, [], []
                for window in selected:
                    payload = replay.rate_input(shim, window, variant, budget)
                    # 使用本次粗机械前向候选作几何有效性检查; 输出仍只有角速度。
                    lines = payload.splitlines()
                    header = lines[0].split()
                    header[10:13] = map(str, candidate['forward_V'])
                    lines[0] = ' '.join(header)
                    completed = subprocess.run([str(Path(executable).resolve())], input='\n'.join(lines) + '\n',
                                               capture_output=True, text=True, check=True, timeout=30)
                    rows = list(csv.DictReader(io.StringIO(completed.stdout)))
                    start = window['start_us']
                    times = np.array([raw.delta(int(r['evaluated_us']), start) * 1e-6 for r in rows])
                    lo = max(window['interval'][0], window['rows'][0]['t_s'] + .5)
                    hi = window['interval'][1]
                    use = (times >= lo) & (times <= hi)
                    good = use & np.array([r['validity'] == '3' for r in rows])
                    rates = np.array([[float(r['rate_' + a + '_V_rad_s']) for a in 'xyz'] for r in rows])
                    total += int(use.sum())
                    valid += int(good.sum())
                    expected = np.interp(times[good], *window['derived'][0])[:, None] * axes[0]
                    errors.extend(np.linalg.norm(rates[good] - expected, axis=1).tolist())
                    all_rows.extend(dict(segment=window['part'], **row) for row, keep in zip(rows, use) if keep)
                if not valid:
                    raise ValueError(axis + '/' + variant + ': no valid native outputs')
                key = variant + '_' + str(budget) + 'us'
                report['cases'][axis][key] = dict(outputs=total, valid=valid, valid_fraction=valid / total,
                                                 rms_rad_s=float(np.sqrt(np.mean(np.square(errors)))),
                                                 p95_rad_s=float(np.quantile(errors, .95)))
                with (output / (axis + '-' + key + '.csv')).open('x', newline='', encoding='utf-8') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
                    writer.writeheader()
                    writer.writerows(all_rows)
    with (output / 'summary.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    return report


def plot_holdout(candidate, windows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    axes = np.array([candidate['axes_V'][a] for a in AXES])
    ref = np.array([candidate['reference_native_rad'][a] for a in AXES])
    fig, panels = plt.subplots(3, 2, figsize=(14, 10), layout='constrained')
    for i, axis in enumerate(AXES):
        first = True
        for window in [w for w in windows if w['axis'] == axis]:
            predicted = data.prediction(window, axes, np.zeros(3), ref, 'position')
            panels[i, 0].plot(window['t'], np.linalg.norm(window['gyro'], axis=1), color='black', lw=.6,
                              label='recorded gyro norm' if first else None)
            panels[i, 0].plot(window['t'], np.linalg.norm(predicted, axis=1), color='tab:blue', lw=.7,
                              label='zero-lag model norm' if first else None)
            panels[i, 1].plot(window['t'], np.linalg.norm(predicted - window['gyro'], axis=1), lw=.6)
            first = False
        panels[i, 0].set_ylabel(axis + ' | rad/s')
        panels[i, 1].set_ylabel('vector residual | rad/s')
        for panel in panels[i]:
            panel.grid(alpha=.3)
            panel.set_xlabel('recorded time [s]; gaps kept separate')
        panels[i, 0].legend()
    fig.suptitle('Independent batch-01 holdout | segmented candidate, no hardware validation')
    fig.savefig(output, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-wire-gaps', action='store_true')
    parser.add_argument('--native', type=Path)
    parser.add_argument('--replay-dir', type=Path)
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('输出已存在, 不覆盖候选证据')
    manifest = json.loads(args.manifest.read_text(encoding='utf-8'))
    dataset = Dataset(manifest, args.root, args.allow_wire_gaps)
    candidate, train, holdout = solve(dataset, args.reference)
    candidate['manifest'] = manifest
    candidate['manifest_sha256'] = digest(args.manifest)
    if args.native:
        if not args.replay_dir:
            raise ValueError('--native requires --replay-dir')
        candidate['native_replay'] = native_replay(candidate, holdout, args.native, args.replay_dir)
    if args.plot:
        if args.plot.exists():
            raise ValueError('图像输出已存在')
        plot_holdout(candidate, holdout, args.plot)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(candidate, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(args.output), axes_V=candidate['axes_V'], forward_V=candidate['forward_V'],
                          bias=candidate['gyro']['bias_S_rad_s'], holdout=candidate['fits']['position']['holdout'],
                          repeatability_deg=candidate['heldout_axis_difference_deg'], deployment_ready=False),
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
