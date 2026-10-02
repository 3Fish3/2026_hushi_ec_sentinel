"""GM6020控制台的显示/候选文件模型; 不操作串口或图形上下文."""
from collections import deque
import json
import math
from pathlib import Path
import motor_bench as bench
import motor_bench_trace as trace

# 顺序与S5参数编码器保持一致, 显示单位不更改线协议.
FIELDS = (
    ('target', '目标', ''), ('duration_s', '运行时间', 's · 0=持续'),
    ('current_limit', '电流上限', 'A'), ('speed_limit', '速度目标上限', 'rad/s'),
    ('kp', '速度 Kp', 'A·s/rad'), ('ki', '速度 Ki', 'A/rad'),
    ('position_kp', '位置 Kp', '1/s'), ('kd', '速度 Kd', 'A·s²/rad'),
    ('feedback_alpha', '反馈 alpha', '0=关闭'), ('derivative_tau_s', 'D低通 τ', 's · 0=关闭'))
PROFILE = 'gm6020-console-profile/1'


def status_values(status):
    return [status[key] for key, _, _ in FIELDS] + [int(status['encoder_velocity'])]


def same_settings(mode, values, status):
    return bool(status and mode == status['mode'] and all(
        math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-7)
        for a, b in zip(values, status_values(dict(status, target=status['target_final'])))))


def save_profile(path, mode, values, note):
    bench.settings_payload(mode, values)
    data = dict(schema=PROFILE, mode=mode, values=values, note=note,
                status='manual_candidate', validated=False)
    # 避免一次点击覆盖已有人工候选, 由用户指定新的文件名.
    with Path(path).open('x', encoding='utf-8') as output:
        json.dump(data, output, ensure_ascii=False, indent=2, allow_nan=False)


def load_profile(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('schema') != PROFILE or len(data.get('values', [])) != 11:
        raise ValueError('不是GM6020控制台候选文件')
    bench.settings_payload(data['mode'], data['values'])
    return data


def velocity_shortcut(target, delta, reverse, limit):
    value = -target if reverse else target+delta
    if not all(math.isfinite(v) for v in (target, delta, value, limit)) or abs(value)>limit:
        raise ValueError('快捷目标超过MCU当前速度目标上限或不是有限值; 未发送')
    return value


def position_shortcut(target, delta):
    value = target+delta
    # 位置是相对累计角, 不裁剪为单圈、不按编码器计数取整.
    bench.target_payload(0,value,0)
    return value


class LiveModel:
    def __init__(self):
        self.last = None
        self.received_at = float('-inf')
        self.samples = deque(maxlen=600)  # 低频状态观测, 不是1 kHz波形.
        self.origin = None

    def accept(self, status, received_at):
        if self.origin is None:
            self.origin = received_at
        gap = received_at-self.received_at >= .4
        if gap and self.samples:
            self.samples.append((received_at-self.origin-1e-6, None))
        self.last, self.received_at = status, received_at
        self.samples.append((received_at-self.origin, status.copy()))

    def fresh(self, now):
        return self.last is not None and now-self.received_at < .4

    def average_speed(self, now):
        """最近完整1秒已收状态窗的时间加权均速; 不跨断样/无效反馈补值."""
        if not self.fresh(now) or not self.samples:
            return None
        end = self.samples[-1][0]
        start = end-1.0
        window = []
        for t, d in reversed(self.samples):
            if d is None or not d['feedback_valid']:
                return None
            window.append((t, d['speed_rad_s']))
            if t <= start:
                break
        window.reverse()
        if len(window)<2 or window[0][0]>start:
            return None
        area = 0.0
        for (a, va), (b, vb) in zip(window, window[1:]):
            if b-a >= .4 or b<=a:
                return None
            left = max(a,start)
            vl = va+(vb-va)*(left-a)/(b-a)
            area += (vl+vb)*.5*(b-left)
        return area

    def series(self, key, *, feedback=False, position=False, mode=None):
        x, y = [], []
        for t, d in self.samples:
            valid = d is not None and (not feedback or d['feedback_valid'])
            valid = valid and (not position or d['position_valid']) and (mode is None or d['mode'] == mode)
            if key == 'estimated_velocity_rad_s':
                valid = valid and d['estimated_velocity_valid'] and d['estimated_velocity_age_ms'] < 50
            x.append(t); y.append(d[key] if valid else float('nan'))
        return [x, y]


class Recording:
    def __init__(self, path):
        self.path = Path(path)
        self.metadata, raw = trace.load(path)
        self.rows = trace.rows(self.metadata, raw)
        if not self.rows:
            raise ValueError('波形文件没有样本')

    def series(self, key):
        values = []
        for r in self.rows:
            value = r.get(key)
            if key == 'position' and not r['position_valid']:
                value = None
            if key in ('velocity_used', 'velocity_target') and not r['pid_valid']:
                value = None
            if key == 'speed_target':
                value = r['velocity_target'] if r['pid_valid'] else (r['target'] if r['mode'] == 2 else None)
            if key == 'position_target':
                value = r['target'] if r['mode'] == 3 else None
            values.append(float('nan') if value is None else value)
        return [[r['time_s'] for r in self.rows], values]

    def summary(self, start, end):
        lo, hi = sorted((start, end))
        # DPG浮点编辑框可能舍入窗口端点; 半微秒容差不越过实际采样周期.
        selected = [r for r in self.rows if lo-0.5e-6 <= r['time_s'] <= hi+0.5e-6]
        result = trace.metrics(selected)
        result['duration_s'] = selected[-1]['time_s']-selected[0]['time_s'] if selected else 0
        return result
