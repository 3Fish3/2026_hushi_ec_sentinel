"""Pitch 自动辨识: 目标阶跃激励 + 离线拟合, 给出 Kp/Kd 建议.

只做两件事: (1) 在受限包络内自动下发一小段目标序列并记录响应;
(2) 离线用记录的 (目标, 实测) 拟合关节惯量/阻尼/重力, 反算候选增益.

本工具不改任何参数, 不写固件. 采集需要显式 --arm 与实车确认参数;
停止或失败都会撤销目标与维护授权, 但撤销力矩本身不防坠.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import secrets
import time
from pathlib import Path

import sentinel_client as wire
from sentinel_async import AsyncSentinelClient
from sentinel_pitch_hold import ExclusiveSerial

WARNING = 'STOP REMOVES HOLDING TORQUE. Support/brake must prevent gravity drop.'
PITCH = 'pitch'
MARGIN_RAD = 0.02


def _seq(t, base, amplitude, dwell, steps):
    """基线保持 dwell 后, 每 dwell 秒交替 +/-amplitude, 结束后回基线."""
    if t < dwell:
        return base
    k = int((t - dwell) // dwell)
    if k >= steps:
        return base
    return base + (amplitude if k % 2 == 0 else -amplitude)


def collect(peer, args, output: Path):
    hello = peer.hello().result(timeout=3)
    if hello.get('identity') != 'sentinel_app' or not hello.get('startable'):
        raise RuntimeError(f'not a startable APP: {hello}')
    diag = peer.diagnostics(0).result(timeout=2)
    ttl = diag.get('target_ttl_ms')
    if not isinstance(ttl, (int, float)) or ttl <= 0:
        raise RuntimeError('firmware did not report a target TTL')
    base = peer.status(0).result(timeout=2)['joints'][PITCH]['angle_rad']
    if not math.isfinite(base):
        raise RuntimeError('pitch angle is not finite')
    if not (base - args.amplitude - MARGIN_RAD >= args.low
            and base + args.amplitude + MARGIN_RAD <= args.high):
        raise RuntimeError(
            f'起始姿态 {base:.4f} rad 加上振幅后离边界不足 {MARGIN_RAD} rad; 请把云台移到中段')

    output.mkdir(parents=True, exist_ok=False)
    samples = []
    fault = None
    started = time.monotonic()
    duration = args.dwell * (args.steps + 1)

    dispatched = {}

    def source(now, epochs):
        t = now - started
        if t > duration + args.dwell:
            return None
        value = _seq(t, base, args.amplitude, args.dwell, args.steps)
        dispatched['target_rad'] = value
        return {'setpoints': [(PITCH, 'position', value, 0)],
                'twist': None, 'main_yaw_frame': 'joint'}

    record = {'base_rad': base, 'amplitude_rad': args.amplitude, 'dwell_s': args.dwell,
              'steps': args.steps, 'target_ttl_ms': ttl, 'run': args.run}
    peer.authorize(True, 'maintenance').result(timeout=3)
    try:
        peer.set_target_source(source)
        next_poll = 0.0
        while True:
            now = time.monotonic() - started
            if now > duration + args.dwell:
                break
            snapshot = peer.snapshot()
            if snapshot.get('error'):
                fault = snapshot['error']
                break
            if now >= next_poll:
                joint = peer.status(0).result(timeout=2)['joints'][PITCH]
                samples.append({'t_s': round(now, 4),
                                'target_rad': dispatched.get('target_rad',
                                    _seq(now, base, args.amplitude, args.dwell, args.steps)),
                                'measured_rad': joint['angle_rad'],
                                'velocity_rad_s': joint.get('velocity_rad_s')})
                next_poll = now + args.period
            time.sleep(0.002)
    finally:
        peer.set_target_source(None)
        try:
            peer.authorize(False, 'maintenance').result(timeout=3)
            record['revoked'] = True
        except Exception as error:
            record['revoked'] = False
            record['revoke_error'] = str(error)

    if samples:
        with (output / 'samples.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
    record.update({'fault': fault, 'samples': len(samples)})
    (output / 'summary.json').write_text(
        json.dumps(record, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    return record


def analyse(directory: Path, kp: float, kd: float, zeta: float, bandwidth: float):
    """用 (target, measured) 拟合 J/b/G, 再按期望带宽与阻尼比反算增益."""
    import numpy as np

    rows = list(csv.DictReader((directory / 'samples.csv').open(encoding='utf-8')))
    if len(rows) < 20:
        raise ValueError('样本太少, 无法拟合')
    t = np.array([float(r['t_s']) for r in rows])
    u = np.array([float(r['target_rad']) for r in rows])
    x = np.array([float(r['measured_rad']) for r in rows])
    dt = float(np.median(np.diff(t)))
    if not dt > 0:
        raise ValueError('时间戳无效')
    v = np.gradient(x, t)
    a = np.gradient(v, t)
    # J*a + (Kd+b)*v - G = Kp*(u-x)
    A = np.column_stack([a, v, -np.ones_like(a)])
    rhs = kp * (u - x)
    (J, kd_plus_b, G), *_ = np.linalg.lstsq(A, rhs, rcond=None)
    viscous = kd_plus_b - kd
    wn = 2.0 * math.pi * bandwidth
    residual = A @ np.array([J, kd_plus_b, G]) - rhs
    fit_rms = float(np.sqrt(np.mean(residual ** 2)))
    rhs_rms = float(np.sqrt(np.mean(rhs ** 2)))
    suggested_kp = float(J * wn * wn)
    suggested_kd = float(2.0 * zeta * J * wn - viscous)
    # Trust gate: an unphysical or poorly-fitting identification must never
    # present its gains as plain suggestions (a negative inertia previously
    # produced negative Kp/Kd with no warning at all).
    problems = []
    if not (math.isfinite(J) and math.isfinite(viscous) and math.isfinite(G)):
        problems.append('non-finite fit parameters')
    if J <= 0:
        problems.append(f'non-positive inertia: {J:.6g}')
    if suggested_kp <= 0 or suggested_kd < 0:
        problems.append(
            f'non-physical suggested gains: kp={suggested_kp:.6g}, kd={suggested_kd:.6g}')
    if rhs_rms <= 0 or fit_rms > 0.25 * rhs_rms:
        problems.append(
            f'poor fit: residual RMS {fit_rms:.6g} vs signal RMS {rhs_rms:.6g}')
    return {'dt_s': round(dt, 5), 'inertia': float(J), 'viscous': float(viscous),
            'gravity_torque': float(G),
            'fit_rms_nm': fit_rms,
            'current_kp': kp, 'current_kd': kd,
            'target_zeta': zeta, 'target_bandwidth_hz': bandwidth,
            'suggested_kp': suggested_kp,
            'suggested_kd': suggested_kd,
            'trusted': not problems, 'trust_problems': problems}


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    link = p.add_mutually_exclusive_group(required=False)
    link.add_argument('--serial', type=Path)
    link.add_argument('--backend', type=Path)
    p.add_argument('--output', type=Path, help='采集输出目录(必须不存在)')
    p.add_argument('--analyse', type=Path, help='分析已有采集目录, 不连接设备')
    p.add_argument('--arm', action='store_true', help='显式授权下发 Pitch 目标')
    p.add_argument('--support-ready', action='store_true')
    p.add_argument('--accept-release', action='store_true')
    p.add_argument('--run', type=int)
    p.add_argument('--low', type=float, help='机械下限 rad')
    p.add_argument('--high', type=float, help='机械上限 rad')
    p.add_argument('--amplitude', type=float, default=0.01, help='阶跃幅度 rad, (0,0.02]')
    p.add_argument('--dwell', type=float, default=0.8, help='每段保持时间 s')
    p.add_argument('--steps', type=int, default=6, help='阶跃段数')
    p.add_argument('--period', type=float, default=0.02, help='采样周期 s')
    p.add_argument('--kp', type=float, help='当前固件 Kp (分析用)')
    p.add_argument('--kd', type=float, help='当前固件 Kd (分析用)')
    p.add_argument('--zeta', type=float, default=1.0)
    p.add_argument('--bandwidth', type=float, default=3.0, help='期望闭环带宽 Hz')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if args.analyse:
        if args.kp is None or args.kd is None:
            p.error('--analyse 需要同时给 --kp 与 --kd')
        print(json.dumps(analyse(args.analyse, args.kp, args.kd, args.zeta, args.bandwidth),
                         ensure_ascii=False, indent=1))
        return 0
    if not args.arm:
        p.error('--arm is required; without it nothing is sent')
    if args.output is None:
        p.error('--output is required')
    if args.serial is None and args.backend is None:
        p.error('one of --serial/--backend is required')
    if not (0 < args.amplitude <= 0.02):
        p.error('--amplitude must be in (0, 0.02]')
    if not (args.dwell > 0):
        p.error('--dwell must be > 0')
    if not (args.steps >= 1):
        p.error('--steps must be >= 1')
    if not (args.period > 0):
        p.error('--period must be > 0')
    if args.low is not None and args.high is not None and not (args.low < args.high):
        p.error('--low must be < --high')
    if args.serial and (not args.support_ready or not args.accept_release
                        or args.low is None or args.high is None):
        p.error('serial requires --support-ready --accept-release --low --high')
    if args.backend and (args.low is None or args.high is None):
        p.error('--backend also requires --low --high')
    args.run = args.run or secrets.randbelow(0xffff) + 1
    transport = (wire.ProcessTransport(args.backend, extra_args=['--plant', '--fixture', 'gimbal'])
                 if args.backend else ExclusiveSerial(args.serial))
    peer = AsyncSentinelClient(transport, target_hz=50, fast_hz=20)
    print(WARNING, flush=True)
    try:
        record = collect(peer, args, args.output)
    finally:
        try:
            peer.close()
        except Exception:
            pass
    print(json.dumps(record, ensure_ascii=False), flush=True)
    return 0 if record.get('fault') is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
