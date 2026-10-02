"""离线审计 DJI 位置增量与原生 RPM 积分; 不重拟合安装标定, 不生成运行补偿."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import sentinel_dataset_calibrate as dataset

RPM_PER_RAD_S = 60/(2*np.pi)
ENCODER_STEP_RAD = 2*np.pi/8192


def integral(t, value, at, *, trapezoid=True):
    """样本覆盖区间内的积分; 同时保留梯形与左端保持两种重建用于敏感性检查."""
    t, value, at = np.asarray(t), np.asarray(value), np.asarray(at)
    dt = np.diff(t)
    if len(t)<2 or len(t)!=len(value) or np.any(dt<=0) or np.any(at<t[0]) or np.any(at>t[-1]):
        raise ValueError('积分需要严格递增、覆盖端点的时间轴')
    average = (value[:-1]+value[1:])/2 if trapezoid else value[:-1]
    cumulative = np.r_[0,np.cumsum(average*dt)]
    index = np.clip(np.searchsorted(t,at,side='right')-1,0,len(t)-2)
    h = at-t[index]
    result = cumulative[index]+value[index]*h
    if trapezoid:
        result += .5*(value[index+1]-value[index])/dt[index]*h*h
    return result


def blocks(t, q, v, interval, *, window_s=1., lag_margin_s=.02):
    """位置端点的平均速度不依赖数值求导; 门槛只用于诊断选段, 不是控制参数."""
    t,q,v = np.asarray(t),np.asarray(q),np.asarray(v)
    if window_s<=0 or lag_margin_s<0 or len(t)<2 or len(t)!=len(q) or len(t)!=len(v):
        raise ValueError('无效诊断窗口或数据维度')
    if not all(np.all(np.isfinite(a)) for a in (t,q,v)) or np.any(np.diff(t)<=0):
        raise ValueError('样本必须有限且时间严格递增')
    lo=max(interval[0],t[0]+lag_margin_s)
    hi=min(interval[1],t[-1]-lag_margin_s)
    start=np.arange(lo,hi-window_s+1e-9,window_s)
    end=start+window_s
    delta=np.interp(end,t,q)-np.interp(start,t,q)
    path=np.r_[0,np.cumsum(np.abs(np.diff(q)))]
    travel=np.interp(end,t,path)-np.interp(start,t,path)
    consistency=np.divide(np.abs(delta),travel,out=np.zeros_like(delta),where=travel>0)
    encoder=delta/window_s*RPM_PER_RAD_S
    spans=np.array([np.ptp(np.r_[np.interp(a,t,q),q[(t>a)&(t<b)],np.interp(b,t,q)]) for a,b in zip(start,end)])
    return dict(start=start,end=end,encoder_rpm=encoder,consistency=consistency,
                moving=(np.abs(encoder)>=2)&(consistency>=.9),
                quiet=spans<=2.01*ENCODER_STEP_RAD)


def rpm_average(t,v,window,lag_s=0.,*,trapezoid=True):
    # 正时移表示用较晚报告的速度对照较早的位置区间, 只用于离线诊断.
    return ((integral(t,v,window['end']+lag_s,trapezoid=trapezoid)
             -integral(t,v,window['start']+lag_s,trapezoid=trapezoid))
            /(window['end']-window['start'])*RPM_PER_RAD_S)


def residual_stats(values):
    values=np.asarray(values)
    if not len(values):return dict(count=0,mean=None,rms=None,p95_abs=None)
    return dict(count=int(len(values)),mean=float(np.mean(values)),
                rms=float(np.sqrt(np.mean(values**2))),p95_abs=float(np.percentile(np.abs(values),95)))


def affine_fit(x,y):
    if len(x)<4 or not np.any(x>0) or not np.any(x<0):
        return None
    design=np.c_[x,np.ones(len(x))]
    parameters,_,rank,_=np.linalg.lstsq(design,y,rcond=None)
    if rank!=2:return None
    scale,bias=parameters
    if abs(scale)<np.finfo(float).eps:return None
    return dict(native_scale=float(scale),native_bias_rpm=float(bias),
                residual_rpm=residual_stats(y-design@parameters))


def analyze_axis(training, holdout, *, lag_bound_s=.02, lag_step_s=.001):
    """只在训练段选模型; 留出段不反向选择时移/比例/偏置."""
    if lag_bound_s<0 or lag_step_s<=0:raise ValueError('无效时移网格')
    def collect(entries,lag):
        xs,ys=[],[]
        for entry in entries:
            window=entry['window'];keep=window['moving']
            xs.extend(window['encoder_rpm'][keep])
            ys.extend(rpm_average(entry['t'],entry['v'],window,lag)[keep])
        return np.asarray(xs),np.asarray(ys)
    profile=[]
    for lag in np.linspace(-lag_bound_s,lag_bound_s,round(2*lag_bound_s/lag_step_s)+1):
        x,y=collect(training,lag);fit=affine_fit(x,y)
        if fit is not None:profile.append(dict(lag_s=float(lag),**fit))
    if not profile:return dict(fit=None,lag_profile=[],reason='训练块不足或缺少双向运动')
    best=min(profile,key=lambda row:row['residual_rpm']['rms'])
    def evaluate(entries):
        x,y=collect(entries,0);_,shifted=collect(entries,best['lag_s'])
        return dict(blocks=int(len(x)),positive=residual_stats((y-x)[x>0]),negative=residual_stats((y-x)[x<0]),
            raw=residual_stats(y-x),
            half_bin_hypothesis=residual_stats(y+.5-x),
            frozen_affine_model=residual_stats((shifted-best['native_bias_rpm'])/best['native_scale']-x))
    return dict(fit=best,lag_at_search_boundary=abs(best['lag_s'])>=lag_bound_s-1e-9,
                lag_profile=profile,train=evaluate(training),holdout=evaluate(holdout))


def audit(manifest,root,*,window_s=1.):
    files,hashes,ranges=dataset.load_dataset(manifest,root)
    groups={axis:{key:[] for key in ('train','holdout','stress')} for axis in ('main','fire')}
    records=[];csv_rows=[]
    bound=manifest['analysis']['lag_bound_s']
    for partition in ('train','holdout','stress'):
        for item in manifest.get(partition,[]):
            file=files[item['file']]
            for index,axis in enumerate(('main','fire')):
                t,q,v=file['joints'][index]
                window=blocks(t,q,v,item['interval_s'],window_s=window_s,lag_margin_s=bound)
                entry=dict(t=t,v=v,window=window)
                groups[axis][partition].append(entry)
                measured=rpm_average(t,v,window)
                zoh=rpm_average(t,v,window,trapezoid=False)
                error=measured-window['encoder_rpm'];moving=window['moving'];quiet=window['quiet']
                records.append(dict(file=item['file'],partition=partition,axis=axis,
                    blocks=len(measured),moving_error_rpm=residual_stats(error[moving]),
                    quiet_reported_rpm=residual_stats(measured[quiet]),
                    reconstruction_difference_rpm=residual_stats((zoh-measured)[moving])))
                for i,start in enumerate(window['start']):
                    csv_rows.append(dict(file=item['file'],partition=partition,axis=axis,start_s=float(start),
                        end_s=float(window['end'][i]),encoder_rpm=float(window['encoder_rpm'][i]),
                        native_rpm=float(measured[i]),error_rpm=float(error[i]),
                        zoh_difference_rpm=float(zoh[i]-measured[i]),
                        direction_consistency=float(window['consistency'][i]),
                        moving=bool(moving[i]),quiet=bool(quiet[i])))
    analyses={axis:analyze_axis(group['train'],group['holdout'],lag_bound_s=bound,
              lag_step_s=manifest['analysis']['lag_profile_step_s']) for axis,group in groups.items()}
    result=dict(schema='sentinel-feedback-audit/1',deployment_ready=False,calibration_updated=False,
        input_sha256=hashes,manifest_canonical_sha256=hashlib.sha256(json.dumps(manifest,sort_keys=True).encode()).hexdigest(),
        tool_sha256={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in
                     ('sentinel_feedback_audit.py','sentinel_dataset_calibrate.py','sentinel_calibrate.py','sentinel_raw.py')},
        settings=dict(window_s=window_s,min_moving_rpm=2.,min_direction_consistency=.9,
            quiet_encoder_span_counts=2.01,encoder_step_rad=ENCODER_STEP_RAD,
            native_rpm_step_rad_s=1/RPM_PER_RAD_S,lag_positive='later native velocity vs earlier encoder interval'),
        limitations=['同一CAN帧内的编码位置/RPM自洽性检查, 不是绝对角度或齿比标定',
            '有限区间平均速度与滤波/量化相关, 不等于瞬时速度真值',
            '半格比较仅对选中的移动块; 不构成全速域/静止状态的补偿算法',
            '时移网格最优值不是已验收延迟, 不修改生产插值时效或反馈来源',
            '训练段选择的仿射模型仅在独立留出评价, 不写入配置'],
        axes=analyses,records=records)
    return result,csv_rows


def plot_report(result,rows,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for column,axis in enumerate(('main','fire')):
        for partition in ('train','holdout'):
            selected=[r for r in rows if r['axis']==axis and r['partition']==partition and r['moving']]
            axes[0,column].scatter([r['encoder_rpm'] for r in selected],[r['error_rpm'] for r in selected],s=20,label=partition)
        axes[0,column].axhline(0,color='black',lw=1)
        axes[0,column].axhline(-.5,color='gray',ls='--',label='-0.5 rpm (comparison only)')
        axes[0,column].set(title=f'{axis}: native RPM minus encoder average',xlabel='encoder interval average [rpm]',ylabel='residual [rpm]')
        axes[0,column].legend();axes[0,column].grid(alpha=.25)
        profile=result['axes'][axis]['lag_profile']
        axes[1,column].plot([r['lag_s']*1000 for r in profile],[r['residual_rpm']['rms'] for r in profile])
        axes[1,column].set(title='Training affine-fit residual (not timing acceptance)',xlabel='offline native time shift [ms]',ylabel='RMS [rpm]')
        axes[1,column].grid(alpha=.25)
    fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--window-s',type=float,default=1.)
    parser.add_argument('--plot',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[3]
    manifest=json.loads(args.manifest.read_text(encoding='utf-8'))
    result,rows=audit(manifest,root,window_s=args.window_s)
    result['manifest_file_sha256']=hashlib.sha256(args.manifest.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x',encoding='utf-8') as file:
        json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False);file.write(chr(10))
    with args.output.with_suffix('.csv').open('x',encoding='utf-8-sig',newline='') as file:
        fields=tuple(rows[0]) if rows else ('file','axis')
        writer=csv.DictWriter(file,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    if args.plot:plot_report(result,rows,args.output.with_suffix('.png'))
    print(json.dumps({axis:{k:v for k,v in report.items() if k!='lag_profile'} for axis,report in result['axes'].items()},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
