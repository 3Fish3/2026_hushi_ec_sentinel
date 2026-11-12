"""Fire-Yaw 小信号扫频: 低于 Fs/4 的正弦频率点, 测幅值比与相位滞后.

只发送 fire_yaw 位置目标, 只读状态页采样, 不改任何增益/参数, 不写固件.
默认预览; 需 --execute --safety-confirmed 才连接硬件. 结束时撤销目标与维护授权.
停止会撤掉 fire_yaw 保持力矩: 必须有机械承接与物理急停.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import sentinel_client as wire
from sentinel_async import AsyncSentinelClient
from sentinel_pitch_hold import ExclusiveSerial

DOMAIN = 'fire_yaw'
DEFAULT_PORT = '/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148384-if02'

# 固件硬编码命令带 (board_app_mechanism_config.hpp)
FIRE_CMD_MIN = -0.930
FIRE_CMD_MAX = 0.479


def _frequencies(text):
    values = []
    for part in text.split(','):
        part = part.strip()
        if not part:
            continue
        value = float(part)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'frequency must be positive finite: {part!r}')
        values.append(value)
    if not values:
        raise ValueError('at least one frequency required')
    return values


def _parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default=DEFAULT_PORT)
    p.add_argument('--execute', action='store_true', help='执行；不指定时只打印计划')
    p.add_argument('--safety-confirmed', action='store_true',
                   help='确认 fire_yaw 可动、线束余量、人员离开、物理急停可用')
    p.add_argument('--frequencies', type=_frequencies, default=_frequencies('0.2,0.5,1,2,3,5'),
                   help='逗号分隔的正弦频率 Hz')
    p.add_argument('--amplitude', type=float, default=0.02, help='正弦幅值 rad, (0, 0.1]')
    p.add_argument('--cycles', type=float, default=6.0, help='每点记录的正弦周期数, >=3')
    p.add_argument('--settle-cycles', type=float, default=2.0, help='每点开始丢弃的周期数, >=1')
    p.add_argument('--headroom', type=float, default=0.03, help='离命令带边界的最小余量 rad')
    p.add_argument('--max-travel', type=float, default=0.25, help='中心偏离初始位置的允许上限 rad')
    p.add_argument('--sample-hz', type=float, default=200.0, help='采样目标频率 Hz')
    p.add_argument('--output', type=Path, help='新的结果目录；已存在则拒绝覆盖')
    return p


def plan(args):
    bounds = {'amplitude': (0.0, 0.1), 'cycles': (3.0, 30.0),
              'settle_cycles': (1.0, 10.0), 'headroom': (0.005, 0.1),
              'max_travel': (0.01, 0.5), 'sample_hz': (100.0, 400.0)}
    for name, (low, high) in bounds.items():
        value = getattr(args, name)
        if not (isinstance(value, (int, float)) and math.isfinite(value) and low <= value <= high):
            raise ValueError(f'{name}={value!r} outside [{low}, {high}]')
    for f in args.frequencies:
        if f > args.sample_hz / 4.0:
            raise ValueError(
                f'frequency {f} Hz exceeds sample_hz/4 = {args.sample_hz / 4.0:.2f} Hz; '
                'raise --sample-hz or lower the frequency')
    return {'frequencies_hz': args.frequencies, 'amplitude_rad': args.amplitude,
            'cycles': args.cycles, 'settle_cycles': args.settle_cycles,
            'sample_hz': args.sample_hz}


def _fit_sine(times, values, freq):
    """最小二乘拟合 A*sin(2*pi*f*t)+B*cos(...)+C, 返回幅值与相位."""
    import numpy as np
    t = np.asarray(times, dtype=float)
    y = np.asarray(values, dtype=float)
    w = 2.0 * math.pi * freq
    design = np.column_stack((np.sin(w * t), np.cos(w * t), np.ones_like(t)))
    a, b, c = np.linalg.lstsq(design, y, rcond=None)[0]
    amp = math.hypot(a, b)
    phase = math.atan2(b, a)
    resid = y - design @ np.array([a, b, c])
    return {'amplitude': float(amp), 'phase_rad': float(phase),
            'offset': float(c), 'residual_rms': float(math.sqrt(float((resid ** 2).mean())))}


def run(args, plan_info, output):
    output.mkdir(parents=True, exist_ok=False)
    report = {'plan': plan_info, 'error': None, 'points': [], 'limits': {}}
    peer = AsyncSentinelClient(ExclusiveSerial(args.port), target_hz=50, fast_hz=200)
    authorized = False
    source_installed = False
    try:
        hello = peer.hello().result(timeout=3)
        if hello.get('identity') != 'sentinel_app':
            raise RuntimeError(f'not a sentinel APP: {hello.get("identity")}')
        page = peer.status(0).result(timeout=2)
        start = page['joints'][DOMAIN]['angle_rad']
        if not math.isfinite(start):
            raise RuntimeError('fire_yaw angle not finite')

        low_limit = FIRE_CMD_MIN + args.headroom
        high_limit = FIRE_CMD_MAX - args.headroom
        center = start

        band_ok = (center - args.amplitude >= low_limit) and (center + args.amplitude <= high_limit)
        if not band_ok:
            raise RuntimeError(
                f'center {center:.4f} ± {args.amplitude} leaves the safe command band '
                f'[{low_limit:.4f}, {high_limit:.4f}]; reposition fire_yaw toward mid-travel')
        report['limits'] = {'start_rad': start, 'center_rad': center,
                            'command_low': FIRE_CMD_MIN, 'command_high': FIRE_CMD_MAX,
                            'safe_low': low_limit, 'safe_high': high_limit}
        report['hello'] = {k: hello.get(k) for k in ('identity', 'config_kind', 'session')}

        def source(now_s, epochs):
            t = now_s - t_start[0]
            value = center + args.amplitude * math.sin(2 * math.pi * current['freq'] * t)
            return {'setpoints': [(DOMAIN, 'position', value, 0)]}

        t_start = [time.monotonic()]
        current = {'freq': args.frequencies[0]}

        if not args.execute:
            report['preview'] = True
            (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
            return report

        peer.authorize(True, 'maintenance').result(timeout=3)
        authorized = True
        peer.status(0).result(timeout=2)
        peer.set_target_source(source)
        source_installed = True

        period = 1.0 / args.sample_hz
        for freq in args.frequencies:
            current['freq'] = freq
            t_start[0] = time.monotonic()
            settle = args.settle_cycles / freq
            measure = args.cycles / freq
            
            # 等待过渡阶段稳定
            settle_end = time.monotonic() + settle
            while time.monotonic() < settle_end:
                time.sleep(0.01)

            times, angles, targets = [], [], []
            nxt = time.monotonic()
            t_meas0 = time.monotonic()
            while time.monotonic() - t_meas0 < measure:
                snap = peer.snapshot()
                if snap.get('error'):
                    raise RuntimeError(f'client error during sweep: {snap["error"]}')
                page0 = snap.get('status', {}).get(0, {}).get('joints', {}).get(DOMAIN, {})
                angle = page0.get('angle_rad')
                now = time.monotonic() - t_meas0
                if isinstance(angle, (int, float)):
                    times.append(now)
                    angles.append(float(angle))
                    targets.append(center + args.amplitude * math.sin(2 * math.pi * freq * (time.monotonic() - t_start[0])))
                nxt += period
                wait = nxt - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                else:
                    nxt = time.monotonic()

            measured = _fit_sine(times, angles, freq)
            commanded = _fit_sine(times, targets, freq)
            gain = measured['amplitude'] / commanded['amplitude'] if commanded['amplitude'] else None
            phase_lag = math.degrees(commanded['phase_rad'] - measured['phase_rad'])
            phase_lag = (phase_lag + 180.0) % 360.0 - 180.0
            point = {'frequency_hz': freq, 'samples': len(times),
                     'commanded_amplitude': commanded['amplitude'],
                     'measured_amplitude': measured['amplitude'],
                     'gain': gain, 'phase_lag_deg': phase_lag,
                     'fit_residual_rms': measured['residual_rms'],
                     'measured_offset_rad': measured['offset']}
            report['points'].append(point)
            print(f"[{freq:4.1f} Hz] Gain: {gain:6.3f} | Phase Lag: {phase_lag:6.1f} deg | ResRMS: {measured['residual_rms']:.5f}", flush=True)

    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        print(f"ERROR: {report['error']}", flush=True)
    finally:
        try:
            if source_installed:
                peer.set_target_source(None)
        except Exception:
            pass
        if authorized:
            try:
                peer.authorize(False, 'maintenance').result(timeout=3)
                report['revoked'] = True
            except Exception as error:
                report['revoked'] = False
                report['revoke_error'] = str(error)
        try:
            peer.close()
        except Exception:
            pass
        (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def main(argv=None):
    args = _parser().parse_args(argv)
    info = plan(args)
    if args.output is None:
        stamp = time.strftime('%Y%m%d-%H%M%S')
        args.output = Path('build/identification/fire-yaw/sweep-' + stamp)
    if args.execute and not args.safety_confirmed:
        raise SystemExit('--execute requires --safety-confirmed: support, harness clearance, '
                         'personnel clear, physical e-stop available')
    print(json.dumps({'plan': info, 'execute': args.execute, 'output': str(args.output)}, ensure_ascii=False))
    if not args.execute:
        return 0
    report = run(args, info, args.output)
    print(json.dumps({k: report[k] for k in ('error', 'revoked') if k in report}, ensure_ascii=False))
    return 0 if report.get('error') is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
