"""观测候选的原生角速度回放. 测试参考轴不代表车头, 不输出heading或部署参数."""
from __future__ import annotations
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import numpy as np
import sentinel_raw as raw
import sentinel_calibrate as cal
import sentinel_dataset_calibrate as data

def rate_input(candidate, recording, variant, window_us=5000):
    axes=candidate['axes_V'];am=np.array(axes['main'])
    # 只供原生几何有效性检查使用的数学参考轴; 所有导出量均不含heading.
    basis=np.eye(3)[np.argmin(abs(am))];forward=basis-am*(basis@am);forward/=np.linalg.norm(forward)
    position=np.array([candidate['fits']['position']['lags'][a]['estimated_s'] for a in ('fire','pitch')])
    source='position' if variant=='derived_diagnostic' else 'velocity'
    velocity=np.array([candidate['fits'][source]['lags'][a]['estimated_s'] for a in ('fire','pitch')])
    if variant=='native_zero':position*=0;velocity*=0
    values=[*axes['fire'],*axes['pitch'],*axes['main'],*forward,*candidate['gyro']['bias_S_rad_s'],
        candidate['reference_native_rad']['fire'],candidate['reference_native_rad']['pitch'],
        *np.round(position*1e6).astype(int),*np.round(velocity*1e6).astype(int),.01,.02,.8,window_us]
    lines=['SERIAL_RATE1 '+' '.join(map(str,values))];joints={}
    for i,axis in ((1,'fire'),(2,'pitch')):
        rows=cal.joint_records(recording['rows'],axis,{'source':candidate['sources'][axis]})
        t,q,native=recording['joints'][i]
        rates=np.interp(t,*recording['derived'][i]) if variant=='derived_diagnostic' else native
        for row,p,v in zip(rows,q,rates):joints[row['sequence']]=(axis,p,v)
    for row in sorted(recording['rows'],key=lambda r:r['arrival_s']):
        at,arrival=row['sampled_us'],row['arrived_us'];kind=row['kind']
        if kind in ('gyro','accel') and row['flags']&1 and all(v is not None for v in row['values']):
            lines.append(f"{kind} {at} {arrival} {row['source_sequence']} "+' '.join(map(str,row['values'])))
        elif row['sequence'] in joints:
            axis,p,v=joints[row['sequence']];lines.append(f'{axis} {at} {arrival} 0 {p} {v}')
    lines.append('end');return chr(10).join(lines)+chr(10)

def run(candidate, root, native, output, recorded_budget=False):
    manifest=candidate['manifest'];files,hashes,_=data.load_dataset(manifest,root)
    if hashes != candidate['input_sha256']:raise ValueError('输入文件与拟合时的SHA256不符')
    if output.exists():raise ValueError('回放输出目录已存在, 不覆盖证据')
    output.mkdir(parents=True)
    result={'schema':'sentinel-observed-rate-replay/1','deployment_ready':False,
        'native_sha256':hashlib.sha256(native.read_bytes()).hexdigest(),
        'tool_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'heading_claim':None,'rate_reference':'V=S at manifest reference pose; not vehicle axes',
        'native_timing_fixture':{'gyro_age_us':50000,'joint_age_us':20000,'default_extrapolation_us':5000,'max_gyro_step_us':20000, 'recorded_budget':recorded_budget},
        'limitations':['Diagnostic forward basis is not confirmed vehicle forward',
            'Estimated lags include unidentifiable hypotheses; zero-lag comparison retained',
            'derived_diagnostic uses centered future samples and is NOT a causal controller feedback source',
            'Main encoder derivative as reference assumes stationary base; no independent optical truth'],
        'files':{}}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for item in manifest['holdout']+manifest.get('stress',[]):
        name=item['file'];f=files[name];lo,hi=item['interval_s'];am=np.array(candidate['axes_V']['main']);et,ev=f['derived'][0]
        fig,axs=plt.subplots(2,1,figsize=(12,7),sharex=True,layout='constrained')
        gap=max(np.diff(f['gt']).max(),*(np.diff(t).max() for t,_,_ in f['joints']))
        window_us=int(np.ceil(gap*1e6))+1 if recorded_budget else 5000
        result['files'][name]={};expected_mask=(et>=lo)&(et<=hi);axs[0].plot(et[expected_mask],ev[expected_mask],label='main encoder derivative',color='black',linewidth=1.5)
        for variant in ('native_zero','native_estimated','derived_diagnostic'):
            payload=rate_input(candidate,f,variant,window_us)
            completed=subprocess.run([str(native.resolve())],input=payload,capture_output=True,text=True,check=True)
            csv_path=output/(Path(name).stem+'-'+variant+'.csv');csv_path.write_text(completed.stdout,encoding='utf-8')
            rows=list(csv.DictReader(io.StringIO(completed.stdout)))
            times=np.array([raw.delta(int(r['evaluated_us']),f['metadata']['start_us'])*1e-6 for r in rows])
            rates=np.array([[float(r[f'rate_{a}_V_rad_s']) for a in 'xyz'] for r in rows]);valid=np.array([r['validity']=='3' for r in rows]);window=(times>=lo)&(times<=hi);use=valid&window
            if not np.any(use):raise ValueError(f'{name}/{variant}没有原生有效输出')
            expected=np.interp(times[use],et,ev)[:,None]*am;error=rates[use]-expected;magnitude=np.linalg.norm(error,axis=1)
            metric=dict(extrapolation_us=window_us,window_outputs=int(window.sum()),valid_outputs=int(use.sum()),valid_fraction=float(use.sum()/window.sum()),
                rms_vector_error_rad_s=float(np.sqrt(np.mean(magnitude**2))),p95_vector_error_rad_s=float(np.quantile(magnitude,.95)),
                peak_vector_error_rad_s=float(magnitude.max()),mean_error_rad_s=error.mean(axis=0).tolist(),
                mean_main_projection_error_rad_s=float(np.mean(error@am)),local_epochs=sorted({int(r['local_epoch']) for r,t in zip(rows,times) if lo<=t<=hi}))
            if item['dominant_axis']=='main' and variant=='derived_diagnostic':
                # 积分比较不依赖main速度微分, 用于检查幅值差是否仅是微分窗口衰减.
                tt=times[use];vv=rates[use]@am
                integrated=np.r_[0.,np.cumsum(.5*(vv[1:]+vv[:-1])*np.diff(tt))]
                qt,qq,_=f['joints'][0];position=np.interp(tt,qt,qq)
                design=np.column_stack([position-position[0],tt-tt[0],np.ones(len(tt))])
                coefficients,_,rank,_=np.linalg.lstsq(design,integrated,rcond=None)
                metric['integral_scale_check']=dict(rank=int(rank),gyro_angle_per_encoder_angle=float(coefficients[0]),
                    residual_drift_rad_s=float(coefficients[1]),rms_angle_residual_rad=float(np.sqrt(np.mean((design@coefficients-integrated)**2))),
                    interpretation='Observational scale difference, not an identified gear ratio or automatic gyro correction')
            result['files'][name][variant]=metric
            axs[0].plot(times[use][::3],(rates[use]@am)[::3],label=variant,alpha=.7,linewidth=.8)
            axs[1].plot(times[use][::3],(error@am)[::3],label=variant,alpha=.7,linewidth=.8)
        axs[0].set_ylabel('main-axis projection [rad/s]');axs[1].set_ylabel('projection error [rad/s]');axs[1].set_xlabel('MCU sample time [s]')
        for ax in axs:ax.legend();ax.grid()
        fig.suptitle(name+' | production SentinelPerception, no heading claim');fig.savefig(output/(Path(name).stem+'.png'),dpi=130);plt.close(fig)
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8');return result

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('candidate',type=Path);parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[3]);parser.add_argument('--native',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--recorded-budget',action='store_true',help='仅离线: 插值预算取该日志实测最大采样间隔+1us; 不修改固件')
    args=parser.parse_args();candidate=json.loads(args.candidate.read_text(encoding='utf-8'))
    if candidate.get('schema')!='sentinel-observed-kinematics/1':raise ValueError('观测候选版本不符')
    result=run(candidate,args.root,args.native,args.output,args.recorded_budget);print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
