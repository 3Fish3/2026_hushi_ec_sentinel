"""Pitch 电调反馈解码、MCU 时间对齐与离线响应统计；不访问设备或写入增益。"""
from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
import statistics


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def config_from_parameters(parameters):
    """量程来自本次 MCU 参数快照；旧固件缺少量程时不套用默认数值。"""
    values = {p['id']: p.get('value') for p in parameters if isinstance(p, dict) and 'id' in p}
    ids = (0x904, 0x905, 0x906, 0x903)
    if not all(_number(values.get(key)) and values[key] > 0 for key in ids):
        return None
    return dict(schema_version=1, source='mcu_parameter_snapshot', bus=2, can_id=0x011, node_id=1,
                position_max_rad=values[0x904], velocity_max_rad_s=values[0x905],
                torque_max_nm=values[0x906], feedback_max_age_us=int(values[0x903] * 1000))


def _configuration(metadata):
    config = config_from_parameters(metadata.get('parameters', []))
    if config is None:
        return None
    if not 0 < config['feedback_max_age_us'] < 0x80000000:
        return None
    return config


def decode_feedback(row, config):
    """与 MCU decode_dm_feedback 一致的 p16/v12/t12 换算；力矩来源为电调上报。"""
    if (row.get('kind'), row.get('stream'), row.get('bus'), row.get('can_id'), row.get('dlc')) != (
            'can', 9, config['bus'], config['can_id'], 8):
        return None
    payload = bytes.fromhex(row['data'])
    if len(payload) != 8:
        raise ValueError('Pitch CAN 反馈长度错误')
    node, status = payload[0] & 15, payload[0] >> 4
    p = (payload[1] << 8) | payload[2]
    v = (payload[3] << 4) | (payload[4] >> 4)
    t = ((payload[4] & 15) << 8) | payload[5]
    return dict(pitch_native_position_rad=p * 2 * config['position_max_rad'] / 65535 - config['position_max_rad'],
                pitch_feedback_velocity_rad_s=v * 2 * config['velocity_max_rad_s'] / 4095 - config['velocity_max_rad_s'],
                pitch_reported_torque_nm=t * 2 * config['torque_max_nm'] / 4095 - config['torque_max_nm'],
                pitch_mos_temperature_c=payload[6], pitch_rotor_temperature_c=payload[7],
                pitch_dm_state=status, pitch_feedback_valid=node == config['node_id'] and status in (0, 1))


def annotate_feedback(metadata, rows):
    """只向后匹配 CAN 接收时刻；复用反馈保留原时间/序号，绝不插值或续期。"""
    from sentinel_raw import delta

    config = _configuration(metadata)
    feedback = []
    for row in rows:
        if config is not None:
            decoded = decode_feedback(row, config)
            if decoded is not None:
                row.update(decoded)
                feedback.append(row)
    origin = metadata.get('start_us')
    if origin is None:
        # Files recorded before start_us existed cannot be time-aligned;
        # decoded feedback fields above still apply, matching is skipped
        # instead of failing the whole load with KeyError.
        return
    feedback.sort(key=lambda row: (delta(row['sampled_us'], origin), row['sequence']))
    keys = [(delta(row['sampled_us'], origin), row['sequence']) for row in feedback]
    for row in rows:
        if (row['kind'], row['stream']) != ('control', 6):
            continue
        row.update(pitch_reported_torque_nm=None, pitch_mos_temperature_c=None,
                   pitch_rotor_temperature_c=None, pitch_feedback_received_us=None,
                   pitch_feedback_source_sequence=None, pitch_feedback_age_us=None,
                   pitch_feedback_status='config_missing' if config is None else 'missing')
        index = bisect.bisect_right(keys, (delta(row['sampled_us'], origin), row['sequence'])) - 1
        if index < 0:
            continue
        sample = feedback[index]
        age = delta(row['sampled_us'], sample['sampled_us'])
        row.update(pitch_feedback_received_us=sample['sampled_us'],
                   pitch_feedback_source_sequence=sample['source_sequence'], pitch_feedback_age_us=age)
        control_valid = ((row['state_bits'] >> 16) & 15) == 1 and ((row['state_bits'] >> 20) & 15) == 1
        if not sample['pitch_feedback_valid'] or not control_valid:
            row['pitch_feedback_status'] = 'invalid_feedback'
        elif age >= config['feedback_max_age_us']:
            row['pitch_feedback_status'] = 'stale'
        else:
            row['pitch_feedback_status'] = 'fresh'
            for name in ('pitch_reported_torque_nm', 'pitch_mos_temperature_c', 'pitch_rotor_temperature_c'):
                row[name] = sample[name]


def _stats(values):
    values = [float(x) for x in values if _number(x)]
    return dict(count=len(values), mean=statistics.fmean(values) if values else None,
                rms=math.sqrt(statistics.fmean(x*x for x in values)) if values else None,
                max_abs=max(map(abs, values)) if values else None)


def analyze_rows(metadata, rows):
    """报告测得的响应与记录质量；不由单次采集自动产生可部署增益。"""
    from sentinel_raw import delta

    control = [r for r in rows if (r['kind'], r['stream']) == ('control', 6)]
    matched = [r for r in control if r.get('pitch_feedback_status') == 'fresh']
    unique = {r['pitch_feedback_source_sequence']: r for r in matched}
    intervals = [delta(b['sampled_us'], a['sampled_us']) for a, b in zip(control, control[1:])]
    errors = [r['pitch_adopted_rad'] - r['pitch_measured_rad'] for r in control
              if _number(r['pitch_adopted_rad']) and _number(r['pitch_measured_rad'])
              and ((r['state_bits'] >> 16) & 15) == 1]
    gaps = sum(r.get('gap', 0) for r in rows)
    lost = max((r['lost'] for r in rows), default=0)
    reasons = []
    invalid_fields = []
    required = ('pitch_requested_rad', 'pitch_adopted_rad', 'pitch_measured_rad',
                'pitch_measured_velocity_rad_s', 'pitch_feedforward_torque_nm', 'pitch_dt_s')
    for row in control:
        fields = required
        if row.get('pitch_feedback_status') == 'fresh':
            fields += ('pitch_reported_torque_nm', 'pitch_mos_temperature_c', 'pitch_rotor_temperature_c')
        for field in fields:
            value = row.get(field)
            if not _number(value) or (field == 'pitch_dt_s' and value <= 0):
                invalid_fields.append(dict(sequence=row['sequence'], field=field,
                                           reason='missing_non_finite_or_invalid_dt'))
    if invalid_fields:
        reasons.append('invalid_required_fields')
    if not control:
        reasons.append('no_pitch_control_records')
    if _configuration(metadata) is None:
        reasons.append('missing_mcu_mit_limits')
    if len(matched) != len(control):
        reasons.append('feedback_missing_stale_or_invalid')
    if gaps or lost:
        reasons.append('record_loss')
    if any(x <= 0 for x in intervals):
        reasons.append('non_monotonic_control_time')
    return dict(schema_version=1, source=metadata.get('source', 'UNKNOWN'),
                status='incomplete' if reasons else 'complete', reasons=reasons,
                invalid_fields=invalid_fields,
                deployable_gains=False, torque_source='esc_reported_using_mcu_mit_limits',
                timing_source='mcu_can_receive_time', control_records=len(control),
                feedback_matched_records=len(matched), unique_feedback_records=len(unique),
                sequence_gaps=gaps, lost_max=lost,
                tracking_error_rad=_stats(errors),
                velocity_rad_s=_stats(r['pitch_measured_velocity_rad_s'] for r in matched),
                reported_torque_nm=_stats(r['pitch_reported_torque_nm'] for r in unique.values()),
                feedforward_command_nm=_stats(r['pitch_feedforward_torque_nm'] for r in control),
                feedback_age_us=_stats(r['pitch_feedback_age_us'] for r in matched),
                max_mos_temperature_c=max((r['pitch_mos_temperature_c'] for r in unique.values()
                                           if _number(r['pitch_mos_temperature_c'])), default=None),
                max_rotor_temperature_c=max((r['pitch_rotor_temperature_c'] for r in unique.values()
                                             if _number(r['pitch_rotor_temperature_c'])), default=None),
                median_control_interval_us=statistics.median(intervals) if intervals else None,
                max_control_interval_us=max(intervals, default=None),
                limitations=['电调上报力矩未用独立传感器标定；不等同于轴端实测力矩',
                             '关联最近已记录 CAN 反馈，限频后未必是本控制周期实际使用的帧；不能据此测精确响应时延',
                             'CAN 接收时间不等于电调内部采样时间；未拼接不连续窗口',
                             '统计报告不证明实车动力学模型或 PID 候选有效'])


def main():
    import sentinel_raw as raw
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    metadata, rows = raw.load(args.input)
    report = analyze_rows(metadata, rows)
    with args.output.open('x', encoding='utf-8') as file:
        json.dump(report, file, ensure_ascii=False, allow_nan=False, indent=2)
        file.write('\n')
    print(json.dumps(dict(status=report['status'], output=str(args.output)), ensure_ascii=False))
    return 0 if report['status'] == 'complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
