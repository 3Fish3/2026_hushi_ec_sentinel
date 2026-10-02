"""离线 GM6020 等效模型辨识. 不连接串口, 不自动应用参数, 不把命令A当实测电流."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares, lsq_linear
import motor_bench_trace as trace


def dataset(path):
    meta, raw = trace.load(path)
    rows = trace.rows(meta, raw)
    if len(rows) < 32 or any(r['record_gaps'] or not r['feedback_valid'] or r['command_a'] is None for r in rows):
        raise ValueError(f'{path}: 缺帧、反馈失效或未接受命令, 不用于辨识')
    t = np.array([r['time_s'] for r in rows])
    if np.any(np.diff(t) <= 0) or np.max(np.diff(t)) > .01:
        raise ValueError(f'{path}: 时基不连续')
    return dict(path=str(path), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                t=t, u=np.array([r['command_a'] for r in rows]),
                v=np.array([r['raw_velocity_rad_s'] for r in rows]), meta=meta)


def simulate(data, parameters):
    """等效增益a、阻尼b、库仑项c、总延迟L; 不分别宣称测得Kt或J.

    输入按控制周期零阶保持. 在已知停止后启动的记录中, 窗口前命令为零;
    其它窗口须显式给出 u_before, 不假设任意日志都以静止开头.
    """
    a, b, c, delay = parameters
    t, u = data['t'], data['u']
    delayed = np.searchsorted(t, t-delay, side='right')-1
    drive = np.where(delayed >= 0, u[np.maximum(delayed, 0)], data.get('u_before', 0.0))
    out = np.empty(len(t)); out[0] = data['v'][0]
    for i in range(1, len(t)):
        dt = t[i]-t[i-1]
        # 对恒输入粘性阻尼精确积分; 库仑摩擦在零速时用静止约束而不是平滑假运动.
        v = out[i-1]; force = a*drive[i-1]
        if abs(v) < 1e-9 and abs(force) <= c:
            out[i] = 0.0; continue
        sign = np.sign(v) if abs(v) >= 1e-9 else np.sign(force)
        decay = np.exp(-b*dt)
        candidate = v*decay+(force-c*sign)*(-np.expm1(-b*dt)/b if b > 1e-9 else dt)
        if candidate*v < 0 and abs(force) <= c: candidate = 0.0
        out[i] = candidate
    return out


def score(data, parameters):
    predicted = simulate(data, parameters)
    residual = predicted-data['v']
    scale = np.linalg.norm(data['v']-np.mean(data['v']))
    return dict(path=data['path'], sha256=data['sha256'],
                rmse_rad_s=float(np.sqrt(np.mean(residual**2))),
                max_error_rad_s=float(np.max(np.abs(residual))),
                fit_percent=float(100*(1-np.linalg.norm(residual)/scale)) if scale > 1e-9 else None)


def fit(training, validation, delay_grid=None):
    # 对短块积分建线性初值, 避免逐点差分放大整数RPM量化. 延迟通过独立网格确定,
    # 不在零阶保持的分段常数延迟上假装存在可靠梯度.
    candidates = []
    grid=np.linspace(0, .02, 41) if delay_grid is None else np.asarray(delay_grid)
    for delay in grid:
        design, outcome = [], []
        for data in training:
            t, u, v = data['t'], data['u'], data['v']
            index = np.searchsorted(t, t-delay, side='right')-1
            ud = np.where(index >= 0, u[np.maximum(index, 0)], data.get('u_before', 0.0))
            for left in range(0, len(t)-8, 8):
                right=left+8
                if np.min(np.abs(v[left:right+1])) < .5 or v[left]*v[right] <= 0: continue
                dt=np.diff(t[left:right+1])
                design.append([np.sum(ud[left:right]*dt), -np.sum((v[left:right]+v[left+1:right+1])*.5*dt),
                               -np.sum(np.sign(v[left:right])*dt)])
                outcome.append(v[right]-v[left])
        x=np.asarray(design); y=np.asarray(outcome)
        if len(x) < 12 or np.linalg.matrix_rank(x) < 3: continue
        initial=lsq_linear(x,y,bounds=(0,np.inf)).x
        def residual(p):
            return np.concatenate([simulate(d,[*p,delay])-d['v'] for d in training])
        result=least_squares(residual,np.maximum(initial,1e-6),bounds=(0,np.inf),
                             x_scale='jac',max_nfev=60,loss='soft_l1',f_scale=.3)
        candidates.append((float(np.mean(residual(result.x)**2)),[*result.x,float(delay)]))
    if not candidates: raise ValueError('运动/输入激励不足, 无法分离增益、阻尼与摩擦')
    candidates.sort(key=lambda item:item[0]); parameters=candidates[0][1]
    a,b,c,delay=parameters
    return dict(schema=1, kind='effective_current_command_to_native_rpm', deployment_ready=False,
                parameters=dict(a_rad_s2_per_command_a=a,b_per_s=b,c_rad_s2=c,delay_s=delay),
                equivalent_breakaway_command_a=c/a if a>0 else None,
                training=[score(d,parameters) for d in training], validation=[score(d,parameters) for d in validation],
                delay_grid_s=[float(value) for value in grid],
                cautions=['总延迟含电调/测速/传输等未分离动态; 接收年龄不是该延迟',
                          '只适用于实测工况; 不证明真实转矩/惯量, 不自动输出通过结论',
                          '起动项是局部库仑近似, 需留出角度/正反/低输出验证',
                          '窗口前输入默认0, 仅使用已经停止后启动的脉冲记录'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--train',nargs='+',required=True,type=Path)
    parser.add_argument('--validate',nargs='+',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if set(p.resolve() for p in args.train)&set(p.resolve() for p in args.validate):
        parser.error('训练与留出记录必须不同')
    training=[dataset(p) for p in args.train]; validation=[dataset(p) for p in args.validate]
    report=fit(training,validation)
    with args.output.open('x',encoding='utf-8') as f: json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))


if __name__=='__main__': main()
