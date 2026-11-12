"""离线闭环频响/模型/PI设计. V2以独立电流excitation作工具变量, V1记录仍可复核."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from scipy.optimize import least_squares


def read_trial(folder):
    folder=Path(folder)
    summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    root=json.loads((folder.parent/'summary.json').read_text(encoding='utf-8'))
    if not summary['complete'] or not root.get('recording_complete',root['complete']) or root.get('log_dropped') or root.get('log_error'):
        raise ValueError(summary.get('error') or root.get('error') or '记录未完整结束, 不生成控制参数')
    # 撤权ACK不明与记录质量分开: 不谎报试验正常结束, 也不因此丢弃已完整的数据.
    summary['cleanup_error']=root.get('cleanup_error')
    if summary['parameters_before']!=summary['parameters_after']: raise ValueError('参数变化')
    data=np.genfromtxt(folder/'samples.csv',delimiter=',',names=True,dtype=float)
    if data.size<100 or not np.all(np.isfinite(np.column_stack([data[n] for n in data.dtype.names if n!='measuring']))):
        raise ValueError('缺数据或非有限值')
    if np.any(np.diff(data['sequence'])!=1) or np.any((data['flags'].astype(int)&15)!=7):
        raise ValueError('记录缺口、无效或触限, 不辨识')
    if np.any(np.diff(data['time_s'])<=0) or np.max(np.diff(data['time_s']))>.005:
        raise ValueError('控制时基不连续')
    return summary,data


def harmonic(t,x,f):
    # 每个完整周期分离常值和缓慢趋势, 不对量化角度二阶差分.
    phase=2*np.pi*f*t
    basis=np.column_stack([np.ones(len(t)),t-np.mean(t),np.cos(phase),np.sin(phase)])
    coef=np.linalg.lstsq(basis,x,rcond=None)[0]
    residual=x-basis@coef
    return complex(coef[2],-coef[3]),float(np.sqrt(np.mean(residual**2)))


def trial_diagnostics(data):
    t=data['time_s'];u=data['command_a'];v=data['feedback']
    stages=[]
    for stage in sorted(set(data['stage'])):
        mask=data['stage']==stage
        stages.append(dict(stage=int(stage),start_s=float(t[mask][0]),end_s=float(t[mask][-1]),
            speed_mean_rad_s=float(np.mean(v[mask])),speed_std_rad_s=float(np.std(v[mask])),
            command_mean_a=float(np.mean(u[mask])),command_peak_a=float(np.max(abs(u[mask])))))
    return dict(command_unit='A, already converted; do not multiply by 3',command_peak_a=float(np.max(abs(u))),stages=stages)


def frequency_response(plan,data,excitation_start_s=None):
    v2='amplitude_a' in plan
    t=data['time_s'];r=data['excitation_a'] if v2 else data['reference'];u=data['command_a'];y=data['feedback']
    if v2 and excitation_start_s is None: raise ValueError('V2缺少实际扫频起点, 不从固定保持时长猜相位')
    # y按实际使用时刻, 不去掉反馈自身延迟; u按本地CAN接受时刻. 控制器设计再加入提交延迟.
    latency=((data['submit_us']-data['at_us']+2**31)%2**32-2**31)*1e-6
    if np.min(latency)<0 or np.max(latency)>.005: raise ValueError('CAN提交时间与控制时基不相容')
    rows=[];offset=excitation_start_s if v2 else plan['ramp_s']+plan['dwell_s']
    for index,f in enumerate(plan['frequencies']):
        measured=(data['stage']==2+index)&((data['flags'].astype(int)&16)!=0)
        transfers=[];numerator=0j;denominator=0j;input_at_control=0j;reference_power=0.;snrs=[];cycle_counts=[]
        for cycle in range(plan['cycles']):
            lo=offset+(cycle+1)/f;hi=lo+1/f
            take=measured&(t>=lo-1e-7)&(t<hi-1e-7)
            if take.sum()<max(30,.9/f/np.median(np.diff(t))): raise ValueError('频点有效周期不完整')
            R,_=harmonic(t[take],r[take],f);U,_=harmonic(t[take]+latency[take],u[take],f);Y,noise=harmonic(t[take],y[take],f)
            if abs(R)<1e-6 or abs(U)<1e-6: raise ValueError('激励/实际输出不足, 不扩大激励自动重试')
            numerator+=Y*np.conj(R);denominator+=U*np.conj(R);reference_power+=abs(R)**2;transfers.append(Y/U)
            U_at,_=harmonic(t[take],u[take],f);input_at_control+=U_at*np.conj(R)
            snrs.append(abs(Y)/(np.sqrt(2)*max(noise,1e-9)));cycle_counts.append(int(take.sum()))
        if abs(denominator)<1e-12: raise ValueError('总电流相干分量不足, 不输出无穷频响')
        G=numerator/denominator
        # V2的D位于控制输出端: U/D=S, 1-S为当前闭环互补灵敏度T; 输入P仍按CAN提交时刻.
        closed=1-input_at_control/reference_power if v2 else numerator/reference_power
        repeat=float(np.std(transfers)/max(abs(G),1e-12))
        velocity=y[measured]
        # 这是辨识质量筛选, 不是新的运动保护. 信噪比或正反转一致性不够时明确拒绝模型.
        moving=bool(np.mean(velocity)*plan['speed']>0 and np.quantile(abs(velocity),.1)>.05)
        rejected=[]
        if repeat>.25: rejected.append('cycle_variation')
        if np.median(snrs)<2: rejected.append('signal_noise')
        if not moving: rejected.append('noncontinuous_motion')
        useful=not rejected
        rows.append(dict(hz=f,real=float(G.real),imag=float(G.imag),magnitude=float(abs(G)),phase_deg=float(np.angle(G,deg=True)),
            closed_loop_magnitude=float(abs(closed)),closed_loop_phase_deg=float(np.angle(closed,deg=True)),
            actual_speed_mean_rad_s=float(np.mean(velocity)),relative_cycle_std=repeat,median_signal_noise_ratio=float(np.median(snrs)),moving=moving,usable=bool(useful),rejected_by=rejected,cycle_samples=cycle_counts))
        offset+=(plan['cycles']+2)/f
    return rows,float(np.quantile(latency,.95))


def transfer(parameters,f):
    a,b,delay=parameters
    s=2j*np.pi*np.asarray(f)
    return a*np.exp(-s*delay)/(s+b)


def fit_model(rows):
    rows=[r for r in rows if r['usable']]
    if len(rows)<3 or rows[-1]['hz']/rows[0]['hz']<4: raise ValueError('至少三个可信频点且频率跨度4倍, 否则只回传频响')
    f=np.array([r['hz'] for r in rows]);g=np.array([complex(r['real'],r['imag']) for r in rows])
    def residual(p):
        difference=transfer(p,f)/g-1
        return np.r_[difference.real,difference.imag]
    trials=[least_squares(residual,[max(abs(g[0])*2*np.pi*f[0],1e-3),.1,L],
             bounds=([1e-6,0,0],[1e6,1000,.1]),max_nfev=500,x_scale='jac') for L in (0,.01,.03)]
    best=min(trials,key=lambda r:np.linalg.norm(r.fun))
    error=float(np.max(abs(transfer(best.x,f)/g-1)))
    if error>.2 or best.x[2]>.099: raise ValueError(f'低阶模型未解释频响: max_relative_complex_error={error:.3f}')
    return best.x.tolist(),error


def propose(parameters,rows,submit_delay,native):
    # 只在训练数据可信频段内设计, 高频端至少覆盖2倍交越; 不外推未测共振来争带宽.
    good=[r for r in rows if r['usable']];upper=good[-1]['hz']/2;lower=good[0]['hz']
    for fc in np.geomspace(upper,lower,24):
        wc=2*np.pi*fc;zero=wc/5
        P=transfer(parameters,[fc])[0]*np.exp(-1j*wc*submit_delay)
        base=1+zero/(1j*wc);kp=1/abs(base*P);ki=kp*zero
        phase=np.angle(base*P,deg=True);margin=180+phase if phase<0 else phase-180
        if margin<60: continue
        # 数据频点上直接复核相位和高频增益, 不只相信所拟模型.
        fg=np.array([r['hz'] for r in good]);g=np.array([complex(r['real'],r['imag']) for r in good])
        loop=(kp+ki/(2j*np.pi*fg))*g*np.exp(-2j*np.pi*fg*submit_delay)
        if np.any(abs(loop[fg>=2*fc])>=1): continue
        args=[*parameters[:2],parameters[2]+submit_delay,kp,ki,.12,8,0]
        run=subprocess.run([str(native)],input=' '.join(map(str,args))+'\n',text=True,capture_output=True,check=True,timeout=10)
        values=[float(x) for x in run.stdout.split()]
        if len(values)!=5 or not all(np.isfinite(values)): raise ValueError('原生模拟响应非法')
        error,overshoot,settling,peak,saturation=values
        if abs(error)>.012 or overshoot>.15 or saturation>0: continue
        return dict(kp_a_per_rad_s=float(kp),ki_a_per_rad=float(ki),kd=0,
            normalized_at_3a=[float(kp/3),float(ki/3),0],crossover_hz=float(fc),phase_margin_deg=float(margin),
            native_small_signal=dict(final_error=error,overshoot_fraction=overshoot,settling_s=settling,peak_command_a=peak,saturation_fraction=saturation),
            outer_loop='内环候选单独验证, 不直接带旧位置42执行; 外环须按新带宽重新匹配')
    return None


def analyze(training,validation,native=None):
    st,dt=read_trial(training);sv,dv=read_trial(validation)
    if Path(training).resolve()==Path(validation).resolve() or (Path(training)/'samples.csv').read_bytes()==(Path(validation)/'samples.csv').read_bytes():
        raise ValueError('留出记录必须独立, 不能复制同一份数据')
    if st['parameters_before']!=sv['parameters_before']: raise ValueError('训练/留出控制参数不同')
    def start(meta):
        if 'amplitude_a' not in meta['plan']: return None
        if meta.get('schema')!=2 or meta['finish']['state']!=3: raise ValueError('V2未完成, 不拟合')
        return ((meta['finish']['excitation_start_us']-meta['start_us'])&0xffffffff)*1e-6
    v2='amplitude_a' in st['plan']
    if v2!=('amplitude_a' in sv['plan']): raise ValueError('训练和留出激励类型不同')
    train,lt=frequency_response(st['plan'],dt,start(st));valid,lv=frequency_response(sv['plan'],dv,start(sv))
    result=dict(schema=2,deployment_ready=False,cleanup_error=st.get('cleanup_error') or sv.get('cleanup_error'),training=train,validation=valid,
        method='excitation-current IV: YD*/UD*' if v2 else 'reference IV: YR*/UR*',
        closed_loop_definition='1-U_control/D (inferred complementary sensitivity)' if v2 else 'Y/reference',
        diagnostics=dict(training=trial_diagnostics(dt),validation=trial_diagnostics(dv)),
        source_sha256={str(Path(p)/'samples.csv'):hashlib.sha256((Path(p)/'samples.csv').read_bytes()).hexdigest() for p in (training,validation)},
        submit_delay_p95_s=max(lt,lv),status='frequency_response_only',candidate=None)
    try:
        model,error=fit_model(train)
        usable=[r for r in valid if r['usable']]
        if len(usable)<3: raise ValueError('独立留出可信频点不足')
        g=np.array([complex(r['real'],r['imag']) for r in usable]);pred=transfer(model,[r['hz'] for r in usable])
        validation_error=float(np.max(abs(pred/g-1)))
        if validation_error>.25: raise ValueError(f'独立正反向模型不一致: relative_complex_error={validation_error:.3f}')
        result.update(status='validated_local_model',model=dict(a=model[0],b=model[1],delay_s=model[2],training_relative_error=error,
            validation_relative_error=validation_error),boundary='局部带载小信号模型, 不分离Kt/J, 不证明零速摩擦或任意姿态')
        if native:
            result['candidate']=propose(model,train,max(lt,lv),native)
            result['status']='native_candidate' if result['candidate'] else 'no_passing_candidate'
    except ValueError as error: result['reason']=str(error)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--train',type=Path,required=True);p.add_argument('--validate',type=Path,required=True)
    p.add_argument('--native',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    try: result=analyze(a.train,a.validate,a.native)
    except (ValueError,OSError) as error:
        result=dict(status='incomplete_or_invalid_trial',candidate=None,deployment_ready=False,reason=str(error))
    with a.output.open('x',encoding='utf-8') as f: json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
    if 'training' not in result:
        print(json.dumps(result,ensure_ascii=False,indent=2));return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(11,7),sharex=True)
    for key in ('training','validation'):
        rows=result[key];f=[r['hz'] for r in rows]
        axes[0,0].semilogx(f,[20*np.log10(r['magnitude']) for r in rows],'o-',label=key)
        axes[1,0].semilogx(f,[r['phase_deg'] for r in rows],'o-',label=key)
        axes[0,1].semilogx(f,[20*np.log10(r['closed_loop_magnitude']) for r in rows],'o-',label=key)
        axes[1,1].semilogx(f,[r['closed_loop_phase_deg'] for r in rows],'o-',label=key)
    axes[0,0].set_title('current command to adopted velocity');axes[0,1].set_title('existing velocity closed loop')
    axes[0,0].set_ylabel('dB');axes[1,0].set_ylabel('deg')
    for ax in axes.flat: ax.grid(alpha=.3);ax.legend()
    for ax in axes[1]: ax.set_xlabel('Hz')
    fig.tight_layout();fig.savefig(a.output.with_suffix('.png'),dpi=120);plt.close(fig)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__': main()
