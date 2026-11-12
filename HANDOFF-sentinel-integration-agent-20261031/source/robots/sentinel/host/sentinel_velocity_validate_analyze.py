"""15s验证的时域筛查, 不拟合频响或生成下一组PID."""
import argparse
import json
from pathlib import Path
import numpy as np


def reference(t):
    y=np.zeros_like(t)
    for lo,hi,start,end in ((2,2.5,0,.25),(5.5,6,.25,0),(8,8.5,0,-.25),(11.5,12,-.25,0)):
        take=(t>=lo)&(t<hi);y[take]=start+(end-start)*.5*(1-np.cos(np.pi*(t[take]-lo)/(hi-lo)))
    y[(t>=2.5)&(t<5.5)]=.25;y[(t>=8.5)&(t<11.5)]=-.25
    return y


def window(t,data,lo,hi):
    # 实际MCU时刻上的线性插值积分; 只补窗口边缘, 前面先验证采样间隔.
    times=np.r_[lo,t[(t>lo)&(t<hi)],hi];v=np.interp(times,t,data)
    return times,v


def metrics(t,v,lo,hi,target=0):
    x,y=window(t,v,lo,hi);mean=float(np.trapezoid(y,x)/(hi-lo))
    return dict(mean=mean,error=mean-target,rms=float(np.sqrt(np.trapezoid(y*y,x)/(hi-lo))),
        error_rms=float(np.sqrt(np.trapezoid((y-target)**2,x)/(hi-lo))),span=float(np.ptp(y)))


def analyze(folder):
    folder=Path(folder)
    result=dict(schema='main-yaw-zero-bias-15s-analysis/1',verdict='incomplete',deployment_ready=False,reasons=[])
    try:
        root=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
        if not (folder/'trial.json').exists(): raise ValueError(root.get('error','动作尚未开始'))
        trial=json.loads((folder/'trial.json').read_text(encoding='utf-8'))
        result['end_reason']=trial.get('end_reason',trial.get('error',root.get('error','unknown')))
        result['restoration_confirmed']=root['restoration_confirmed'];result['release_confirmed']=root['release_confirmed']
        if not trial.get('recording_complete'): raise ValueError('轨迹/终态未收齐')
        d=np.atleast_1d(np.genfromtxt(folder/'samples.csv',delimiter=',',names=True,dtype=float))
        if len(d)<2: raise ValueError('有效样本不足')
        t=d['time_s'];flags=d['flags'].astype(int);terminal=(flags&32)!=0
        if not np.array_equal(d['sequence'],np.arange(len(d))) or trial['finish']['produced']!=len(d): raise ValueError('序号/数量不符')
        if np.any(np.diff(t)<=0) or np.max(np.diff(t))>.005001: raise ValueError('时基缺口')
        for key in ('time_s','reference','feedback','joint','p_a','i_a','command_a','bias_a','excitation_a'):
            if not np.all(np.isfinite(d[key])): raise ValueError('非有限值: '+key)
        if np.any(d['bias_a']!=0) or np.any(d['excitation_a']!=0): raise ValueError('不是无偏置/无独立激励profile')
        if np.max(abs(d['reference']-reference(t)))>2e-5: raise ValueError('实际波形与15s固定profile不符')
        result.update(samples=len(d),duration_s=float(t[-1]),peak_command_a=float(max(abs(d['command_a']))),
            peak_requested_a=float(max(abs(d['p_a']+d['i_a']))),peak_speed=float(max(abs(d['feedback']))))
        ordinary=~terminal
        if np.any((flags&15)!=7): result['reasons'].append('控制/本地CAN/反馈无效或触限')
        if np.any(abs(d['command_a'][ordinary]-d['p_a'][ordinary]-d['i_a'][ordinary])>1e-4): result['reasons'].append('总电流与P+I不符')
        if not trial.get('motion_complete'):
            result['verdict']='B';result['reasons'].append('MCU提前结束: '+result['end_reason']);return result
        if t[0]>.005 or not 14.999<=t[-1]<=15.006 or not terminal[-1] or np.any(terminal[:-1]) or d['command_raw'][-1]!=0:
            raise ValueError('完成时窗或明确零输出终止样本不符')
        if result['peak_requested_a']>.350001 or result['peak_speed']>.400001: result['reasons'].append('超过本轮诊断停止线')
        plateaus=[]
        for name,lo,hi,target in [('positive',2.5,5.5,.25),('negative',8.5,11.5,-.25)]:
            first=metrics(t,d['feedback'],lo,lo+1,target);last=metrics(t,d['feedback'],hi-1,hi,target)
            xs,ys=window(t,d['i_a'],hi-1,hi)
            integral_slope=float(np.polyfit(xs-xs[0],ys,1)[0])
            encoder_speed=float(np.interp(hi,t,d['joint'])-np.interp(hi-1,t,d['joint']))
            plateaus.append(dict(direction=name,first_second=first,last_second=last,integral_slope_a_s=integral_slope,
                encoder_mean_speed=encoder_speed,velocity_encoder_difference=last['mean']-encoder_speed))
            if abs(last['mean']-encoder_speed)>.025: result['reasons'].append(name+'编码器与采用速度末秒均值差>0.025rad/s')
            if abs(last['error'])>.025: result['reasons'].append(name+'末秒均速误差>0.025rad/s')
        result['plateaus']=plateaus
        zero=metrics(t,d['feedback'],14,15)
        _,joint=window(t,d['joint'],14,15);_,whole=window(t,d['joint'],12,15)
        result['zero_tail']=dict(speed_rms=zero['rms'],last_second_angle_span_rad=float(np.ptp(joint)),
            whole_zero_angle_span_rad=float(np.ptp(whole)),whole_zero_displacement_rad=float(whole[-1]-whole[0]))
        if zero['rms']>.02: result['reasons'].append('末秒速度RMS>0.02rad/s')
        if np.ptp(joint)>3*2*np.pi/8192+1e-6: result['reasons'].append('末秒关节角跨度>3 encoder LSB')
        dt=np.diff(t);gyro_displacement=float(np.trapezoid(d['feedback'],t));encoder_displacement=float(d['joint'][-1]-d['joint'][0])
        result['feedback_check']=dict(velocity_integral_rad=gyro_displacement,encoder_displacement_rad=encoder_displacement,
            effective_hz=float((len(d)-1)/(t[-1]-t[0])),max_interval_us=float(max(dt)*1e6))
        result['observation']='回传一次动作中的平顺/振动观察; 同时报平台误差RMS、跨度及I斜率, 不由均速合格推断无持续振荡.'
        if not root.get('complete'): result['reasons'].append(root.get('error') or root.get('cleanup_error') or '参数恢复/撤权/日志未完整确认')
        result['verdict']='B' if result['reasons'] else 'A_numeric'
    except (OSError,ValueError,KeyError,IndexError) as error:
        result['verdict']='incomplete';result['reasons'].append(str(error))
    return result


def save(folder,result):
    folder=Path(folder)
    (folder/'analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    if 'samples' not in result: return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d=np.atleast_1d(np.genfromtxt(folder/'samples.csv',delimiter=',',names=True,dtype=float));t=d['time_s']
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    axes[0].plot(t,d['reference'],label='reference');axes[0].plot(t,d['feedback'],label='used speed',lw=.6);axes[0].set_ylabel('rad/s')
    for key in ('command_a','p_a','i_a'): axes[1].plot(t,d[key],label=key,lw=.7)
    axes[1].axhline(.35,ls=':',color='gray');axes[1].axhline(-.35,ls=':',color='gray');axes[1].set_ylabel('A')
    axes[2].plot(t,d['joint']-d['joint'][0],label='joint displacement');axes[2].set_ylabel('rad');axes[2].set_xlabel('MCU elapsed s')
    for ax in axes: ax.grid(alpha=.3);ax.legend()
    fig.tight_layout();fig.savefig(folder/'analysis.png',dpi=120);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);a=p.parse_args()
    result=analyze(a.folder);save(a.folder,result);print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__': main()
