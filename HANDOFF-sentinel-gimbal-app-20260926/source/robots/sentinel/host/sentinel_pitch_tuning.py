"""云台辅助调参的数据层: 复用已连接 APP 会话, 不创建第二个串口."""
from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import threading

import sentinel_pitch_experiment as experiment
import sentinel_raw as raw
from sentinel_pitch_feedback import analyze_rows
from sentinel_pitch_hold_model import PhysicalBounds, SupervisorLimits


def plan_from_fields(values, *, lift, remote_handoff=False):
    """界面单位均为 rad / rad/s / N·m / s, 转换后复用同一配方校验."""
    def number(name):
        result = float(values[name])
        if not math.isfinite(result):
            raise ValueError(f'{name} 必须是有限数值')
        return result

    cycles = number('cycles')
    if not cycles.is_integer():
        raise ValueError('方波周期数必须是整数')
    recipe = experiment.ExperimentRecipe(
        run=experiment.secrets.randbelow(0xfffffffe) + 1,
        amplitude_nm=number('amplitude'), half_period_us=int(number('half_period') * 1e6),
        baseline_us=int(number('baseline') * 1e6), cycles=int(cycles),
        final_hold_us=int(number('final_hold') * 1e6))
    duration = int(number('capture_duration') * 1e6)
    limits = SupervisorLimits(max_temperature_c=number('max_temperature'),
                              max_effort_nm=number('max_torque'), max_speed_rad_s=number('max_speed'),
                              max_tracking_error_rad=number('max_error'),
                              max_run_s=duration / 1e6 + experiment.plan_margin_s(duration))
    lifting = (experiment.LiftPlan(number('level'), number('lift_speed'), number('lift_timeout'),
                                  number('settle_seconds'), number('settle_error'), number('settle_speed'))
               if lift else None)
    plan = experiment.ExperimentPlan(
        recipe, duration, PhysicalBounds(number('physical_low'), number('physical_high')),
        number('excursion'), limits, hold_timeout_s=number('hold_timeout'),
        post_hold_s=number('post_hold'), lift=lifting,
        remote_handoff_s=number('remote_wait') if remote_handoff else 0.0)
    wait = number('wait_seconds')
    if wait <= 0:
        raise ValueError('等待采集时长必须大于 0')
    return plan, wait


class PitchTuningController:
    """一次有界的保持/采集任务; GUI 只发请求和读取快照."""

    def __init__(self, peer):
        self.peer = peer
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._capture = threading.Event()
        self._state = {'state': 'IDLE', 'report': None, 'directory': '', 'error': None}

    @property
    def active(self):
        return self._thread is not None and self._thread.is_alive()

    def snapshot(self):
        with self._lock:
            return deepcopy(self._state)

    def _progress(self, value):
        with self._lock:
            self._state.update(value)

    def start(self, plan, directory, *, capture=False, wait_s=30.0):
        if self.active:
            raise ValueError('辅助调参正在运行, 请先停止')
        if not math.isfinite(wait_s) or wait_s <= 0:
            raise ValueError('等待采集的保持时长必须为正数')
        self._stop = threading.Event()
        self._capture = threading.Event()
        if capture:
            self._capture.set()
        with self._lock:
            self._state = {'state': 'PREFLIGHT', 'report': None, 'error': None,
                           'directory': str(directory)}

        def work():
            try:
                report = experiment.run_experiment(
                    self.peer, plan, directory, stop_event=self._stop, reuse_session=True,
                    capture_request=self._capture, capture_wait_s=wait_s, progress=self._progress,
                    auto_run_id=True)
                state = ('FAILED' if report['error'] else
                         'REMOTE_CONTROL' if report['stop'] == 'remote_handoff' else 'COMPLETE')
                self._progress({'state': state, 'report': report, 'error': report['error']})
            except Exception as error:
                self._progress({'state': 'FAILED', 'error': str(error)})

        self._thread = threading.Thread(target=work, name='pitch-assistance', daemon=False)
        self._thread.start()

    def capture(self):
        if not self.active:
            raise ValueError('请先启动保持或开始一次新试验')
        if self.snapshot()['state'] not in ('HOLDING', 'LIFTING', 'SETTLING', 'READY', 'PREFLIGHT'):
            raise ValueError('当前不是等待采集的阶段')
        self._capture.set()

    def stop(self):
        self.request_stop()
        # 先立即撤掉目标源, 不等待磁盘收尾或 GUI 刷新.
        self.peer.set_target_source(None)

    def request_stop(self):
        """页面关闭只设置退出请求; 关闭门禁之后不再自行发起新写入."""
        self._stop.set()

    def wait(self, timeout=5.0):
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            if self.active:
                raise TimeoutError('辅助调参尚未完成撤权与数据收尾')

    def close(self, timeout=5.0):
        if self.active:
            self.stop()
            self.wait(timeout)


def capture_view(path):
    """从实际 .sraw 解码曲线和统计, 不修改记录、不自动写入参数."""
    metadata, rows = raw.load(path)
    control = [row for row in rows if row['kind'] == 'control' and row['stream'] == 6]
    if not control:
        raise ValueError('记录中没有 Pitch control 数据')
    fields = ('pitch_adopted_rad', 'pitch_measured_rad', 'pitch_measured_velocity_rad_s',
              'pitch_feedforward_torque_nm', 'pitch_reported_torque_nm',
              'pitch_mos_temperature_c', 'pitch_rotor_temperature_c')
    times = [row['t_s'] for row in control]
    series = {key: [row.get(key) if row.get(key) is not None else math.nan for row in control]
              for key in fields}
    error = [target - measured for target, measured in
             zip(series['pitch_adopted_rad'], series['pitch_measured_rad'])
             if math.isfinite(target) and math.isfinite(measured)]
    return {'metadata': metadata, 'times': times, 'series': series, 'analysis': analyze_rows(metadata, rows),
            'records': len(rows), 'control_records': len(control),
            'gaps': sum(row['gap'] for row in rows), 'lost_max': max(row['lost'] for row in rows),
            'rms_error_rad': math.sqrt(sum(x*x for x in error) / len(error)) if error else None,
            'max_error_rad': max(map(abs, error)) if error else None,
            'source': metadata.get('source', 'UNKNOWN'), 'path': str(Path(path))}
