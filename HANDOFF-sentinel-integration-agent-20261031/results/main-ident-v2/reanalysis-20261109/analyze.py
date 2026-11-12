import argparse, csv, hashlib, json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares

p=argparse.ArgumentParser()
p.add_argument('--base',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
rng=np.random.default_rng(20261109)

def harmonic(t,x,f,detrend=True):
    cols=[np.ones(len(t))]
    if detrend: cols.append(t-t.mean())
    cols.extend([np.cos(2*np.pi*f*t),np.sin(2*np.pi*f*t)])
    A=np.column_stack(cols)
    coef=np.linalg.lstsq(A,x,rcond=None)[0]
    return complex(coef[-2],-coef[-1]),float(np.sqrt(np.mean((x-A@coef)**2)))

def cjson(z): return {'real':float(z.real),'imag':float(z.imag),'magnitude':float(abs(z)),'phase_deg':float(np.angle(z,deg=True))}

result={'scope':'offline independent recomputation; no parameter or firmware changes','trials':[],'files_sha256':{}}
allfreq=[]
for run in ('run-20261109T215532','repeat-20261109T221358'):
    for direction in ('positive','negative'):
        folder=a.base/run/direction
        meta=json.loads((folder/'summary.json').read_text())
        raw=folder/'samples.csv'
        result['files_sha256'][str(raw)]=hashlib.sha256(raw.read_bytes()).hexdigest()
        with raw.open() as f: rows=list(csv.DictReader(f))
        arr={k:np.array([float(r[k]) for r in rows]) for k in rows[0] if k!='measuring'}
        t,y,u,d,stage=(arr[k] for k in ('time_s','feedback','command_a','excitation_a','stage'))
        latency=((arr['submit_us']-arr['at_us']+2**31)%2**32-2**31)*1e-6
        flags=arr['flags'].astype(int)
        scan=(stage>=2)&(stage<=6)
        finish=meta['finish']
        start=((finish['excitation_start_us']-meta['start_us'])&0xffffffff)*1e-6
        tspan=t[scan]
        integral_slope=float(np.polyfit(tspan,arr['i_a'][scan],1)[0])
        direction_sign=np.sign(meta['plan']['speed'])
        trial={'run':run,'direction':direction,'complete':meta['complete'],'samples':len(t),
            'parameters_unchanged':meta['parameters_before']==meta['parameters_after'],
            'sequence_gap_count':int(np.sum(np.diff(arr['sequence'])!=1)),
            'bad_flags':int(np.sum((flags&15)!=7)),
            'produced':finish['produced'],'consumed':finish['consumed'],'dropped':finish['dropped'],
            'sample_dt_median_us':float(np.median(np.diff(t))*1e6),
            'sample_dt_max_us':float(np.max(np.diff(t))*1e6),
            'feedback_source_age_max_us':float(arr['source_age_us'].max()),
            'latency_p95_us':float(np.quantile(latency,.95)*1e6),
            'scan_mean_velocity':float(y[scan].mean()),'scan_std_velocity':float(y[scan].std()),
            'scan_directional_velocity_p01':float(np.quantile(y[scan]*direction_sign,.01)),
            'scan_nonpositive_direction_fraction':float(np.mean(y[scan]*direction_sign<=0)),
            'scan_command_mean_a':float(u[scan].mean()),'peak_command_a':float(np.max(abs(u))),
            'scan_i_slope_ma_s':integral_slope*1000,
            'scan_i_change_ma':float((arr['i_a'][scan][-1]-arr['i_a'][scan][0])*1000),
            'joint_net_turns':float((arr['joint'][-1]-arr['joint'][0])/(2*np.pi)),
            'feedback_integral_rad':float(np.trapezoid(y,t)),
            'joint_net_rad':float(arr['joint'][-1]-arr['joint'][0]),
            'working_point_s':start,'frequencies':[]}
        offset=start
        for idx,f in enumerate(meta['plan']['frequencies']):
            cycles=[]
            for n in range(meta['plan']['cycles']):
                lo=offset+(n+1)/f; hi=lo+1/f
                take=(stage==idx+2)&((flags&16)!=0)&(t>=lo-1e-7)&(t<hi-1e-7)
                assert take.sum()>50
                tx=t[take]
                Y,noise=harmonic(tx,y[take],f)
                U,_=harmonic(tx+latency[take],u[take],f)
                D,_=harmonic(tx,d[take],f)
                Y0,_=harmonic(tx,y[take],f,False)
                U0,_=harmonic(tx+latency[take],u[take],f,False)
                D0,_=harmonic(tx,d[take],f,False)
                cycles.append(dict(Y=Y,U=U,D=D,Y0=Y0,U0=U0,D0=D0,noise=noise,samples=int(take.sum()),
                    velocity_mean=float(y[take].mean()),i_mean_a=float(arr['i_a'][take].mean())))
            Y=np.array([c['Y'] for c in cycles]); U=np.array([c['U'] for c in cycles]); D=np.array([c['D'] for c in cycles])
            G=np.sum(Y*D.conj())/np.sum(U*D.conj())
            R=Y/U
            H=Y@D.conj()/(D@D.conj())
            S=U@D.conj()/(D@D.conj())
            Y0=np.array([c['Y0'] for c in cycles]); U0=np.array([c['U0'] for c in cycles]); D0=np.array([c['D0'] for c in cycles])
            G0=np.sum(Y0*D0.conj())/np.sum(U0*D0.conj())
            boot_idx=rng.integers(0,len(cycles),size=(1000,len(cycles)))
            boot=(Y[boot_idx]*D[boot_idx].conj()).sum(axis=1)/(U[boot_idx]*D[boot_idx].conj()).sum(axis=1)
            rep=float(np.std(R)/abs(G))
            snrs=np.array([abs(c['Y'])/(np.sqrt(2)*max(c['noise'],1e-9)) for c in cycles])
            measured=(stage==idx+2)&((flags&16)!=0)
            moving=bool(y[measured].mean()*meta['plan']['speed']>0 and np.quantile(abs(y[measured]),.1)>.05)
            row={'hz':f,'G':cjson(G),'speed_per_excitation':cjson(H),'total_current_per_excitation':cjson(S),
                'K_integrator_from_magnitude':float(2*np.pi*f*abs(G)),
                'relative_cycle_std_Y_over_U':rep,'median_snr':float(np.median(snrs)),
                'cycle_std_Y_over_D_relative':float(np.std(Y/D)/abs(H)),
                'U_cycle_amplitude_min_a':float(min(abs(U))),'U_cycle_amplitude_max_a':float(max(abs(U))),
                'D_cycle_amplitude_min_a':float(min(abs(D))),'D_cycle_amplitude_max_a':float(max(abs(D))),
                'G_without_linear_detrend':cjson(G0),'detrend_relative_complex_difference':float(abs(G0/G-1)),
                'moving':moving,'usable':bool(rep<=.25 and np.median(snrs)>=2 and moving),
                'bootstrap_magnitude_p025_p975_descriptive_only':[float(v) for v in np.quantile(abs(boot),[.025,.975])],
                'bootstrap_relative_complex_std_descriptive_only':float(np.std(boot)/abs(G)),
                'cycles':[{'samples':c['samples'],'Y':cjson(c['Y']),'U':cjson(c['U']),'D':cjson(c['D']),
                    'G':cjson(c['Y']/c['U']),'noise_rms_rad_s':c['noise'],
                    'speed_mean':c['velocity_mean'],'i_mean_a':c['i_mean_a']} for c in cycles]}
            trial['frequencies'].append(row)
            allfreq.append((run,direction,f,G))
            offset+=(meta['plan']['cycles']+2)/f
        result['trials'].append(trial)
# Descriptive fit only: every point is included, including rejected ones. Do not use as controller design.
f=np.array([x[2] for x in allfreq]); g=np.array([x[3] for x in allfreq]); w=2*np.pi*f
K=np.median(w*abs(g))
model=least_squares(lambda x:np.r_[((x[0]/(1j*w))/g-1).real,((x[0]/(1j*w))/g-1).imag], [K],bounds=([0],[1000]))
K=float(model.x[0]); pred=K/(1j*w)
result['descriptive_integrator_fit']={'definition':'G(j*2*pi*f)=K/(j*2*pi*f); f in Hz',
    'K_rad_s2_per_a':K,'median_relative_complex_error':float(np.median(abs(pred/g-1))),
    'max_relative_complex_error':float(max(abs(pred/g-1))),
    'warning':'All 20 estimates, including quality-rejected ones. Descriptive only; not validated, no PID candidate.'}
result['cross_trial_frequency_spread']=[]
for f0 in (.25,.5,1,2,4):
    v=np.array([x[3] for x in allfreq if x[2]==f0])
    result['cross_trial_frequency_spread'].append({'hz':f0,'magnitude_min':float(min(abs(v))),
        'magnitude_max':float(max(abs(v))),'max_min_ratio':float(max(abs(v))/min(abs(v))),
        'phase_min_deg':float(min(np.angle(v,deg=True))),'phase_max_deg':float(max(np.angle(v,deg=True)))})
with (a.output/'independent-analysis.json').open('x') as f: json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
print('OUTPUT',a.output)
for t in result['trials']:
    print(t['run'],t['direction'],'n=',t['samples'],'v=',round(t['scan_mean_velocity'],4),'i_drift_mA=',round(t['scan_i_change_ma'],2),'i_slope=',round(t['scan_i_slope_ma_s'],3),'joint-v difference rad=',round(t['feedback_integral_rad']-t['joint_net_rad'],3))
    for r in t['frequencies']:
        print(' f=',r['hz'],'G=',round(r['G']['magnitude'],3),'phase=',round(r['G']['phase_deg'],1),
            'repeat=',round(r['relative_cycle_std_Y_over_U'],3),'snr=',round(r['median_snr'],2),
            'Umin/max=',[round(r[k],4) for k in ('U_cycle_amplitude_min_a','U_cycle_amplitude_max_a')],
            'Y/D_repeat=',round(r['cycle_std_Y_over_D_relative'],3),'usable=',r['usable'])
print('INTEGRATOR_DESCRIPTIVE',result['descriptive_integrator_fit'])
print('CROSS_TRIAL_SPREAD',result['cross_trial_frequency_spread'])
