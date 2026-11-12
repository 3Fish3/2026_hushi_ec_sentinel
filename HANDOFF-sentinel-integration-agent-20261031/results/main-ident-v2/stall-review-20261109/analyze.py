import csv,json,hashlib,argparse
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
args.output.mkdir(parents=True,exist_ok=False)
base=args.base
runs=['run-20261109T215532','repeat-20261109T221358','amplitude010-20261109T233006']
keys=['time_s','feedback','command_a','command_raw','effort_raw','joint','source_age_us','p_a','i_a','bias_a','excitation_a','stage','flags','submit_us','at_us','heading_gain']
data={};meta={};hashes={}
for run in runs:
    folder=base/run/'negative';meta[run]=json.loads((folder/'summary.json').read_text())
    raw=folder/'samples.csv';hashes[str(raw)]=hashlib.sha256(raw.read_bytes()).hexdigest()
    with raw.open() as f: rows=list(csv.DictReader(f))
    data[run]={k:np.array([float(x[k]) for x in rows]) for k in keys}
    if run==runs[-1]:
        with (args.output/'negative-23to28s.csv').open('x',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader()
            w.writerows(x for x in rows if 23<=float(x['time_s'])<28)
a=data[runs[-1]];t=a['time_s'];mask=(t>=25.13)&(t<=26.60)
summary={'scope':'offline only; original data and eligibility unchanged','source_sha256':hashes,'stall_window':{},'time_windows':[],'angle_passes':[],'cycles':{},'leave_one_cycle_out_diagnostic':[]}
s=summary['stall_window']
s.update(time_start_s=float(t[mask][0]),time_end_s=float(t[mask][-1]),duration_s=float(t[mask][-1]-t[mask][0]),samples=int(mask.sum()),
    mean_feedback_rad_s=float(a['feedback'][mask].mean()),joint_delta_rad=float(a['joint'][mask][-1]-a['joint'][mask][0]),
    joint_rate_from_endpoints_rad_s=float((a['joint'][mask][-1]-a['joint'][mask][0])/(t[mask][-1]-t[mask][0])),
    mean_command_a=float(a['command_a'][mask].mean()),mean_p_a=float(a['p_a'][mask].mean()),mean_i_a=float(a['i_a'][mask].mean()),
    mean_bias_a=float(a['bias_a'][mask].mean()),mean_excitation_a=float(a['excitation_a'][mask].mean()),
    mean_command_raw=float(a['command_raw'][mask].mean()),mean_effort_raw=float(a['effort_raw'][mask].mean()),
    raw_tracking_mean_abs_error=float(np.mean(abs(a['effort_raw'][mask]-a['command_raw'][mask]))),
    raw_same_sign_fraction=float(np.mean(np.sign(a['effort_raw'][mask])==np.sign(a['command_raw'][mask]))),
    max_source_age_us=float(a['source_age_us'][mask].max()),bad_flags=int(np.sum((a['flags'][mask].astype(int)&15)!=7)))
for lo in range(23,28):
    m=(t>=lo)&(t<lo+1)
    summary['time_windows'].append({'start_s':lo,'end_s':lo+1,'speed_mean_rad_s':float(a['feedback'][m].mean()),
        'command_mean_a':float(a['command_a'][m].mean()),'command_raw_mean':float(a['command_raw'][m].mean()),
        'effort_raw_mean':float(a['effort_raw'][m].mean()),'joint_delta_rad':float(a['joint'][m][-1]-a['joint'][m][0])})
for run,d in data.items():
    # Recorded encoder phase comparison; each 2pi-separated pass is retained.
    for angle in 29.625+2*np.pi*np.arange(-8,9):
        m=(d['stage']>=2)&(d['stage']<=6)&(abs(d['joint']-angle)<.05)
        if m.sum()<15:continue
        summary['angle_passes'].append({'run':run,'joint_target_rad':float(angle),'samples':int(m.sum()),
            'time_range_s':[float(d['time_s'][m].min()),float(d['time_s'][m].max())],
            'stage_values':[int(v) for v in np.unique(d['stage'][m])],
            'median_abs_speed_rad_s':float(np.median(abs(d['feedback'][m]))),
            'min_abs_speed_rad_s':float(min(abs(d['feedback'][m]))),'mean_command_a':float(d['command_a'][m].mean())})
    offset=((meta[run]['finish']['excitation_start_us']-meta[run]['start_us'])&0xffffffff)*1e-6
    summary['cycles'][run]=[]
    for c in range(4):
        lo=offset+(c+1)/.25;hi=lo+4;m=(d['time_s']>=lo)&(d['time_s']<hi)&(d['stage']==2)
        summary['cycles'][run].append({'cycle':c+1,'start_s':lo,'end_s':hi,
            'directional_speed_mean_rad_s':float((-d['feedback'][m]).mean()),
            'directional_speed_min_rad_s':float((-d['feedback'][m]).min()),'peak_command_a':float(max(abs(d['command_a'][m]))),
            'joint_start_rad':float(d['joint'][m][0]),'joint_end_rad':float(d['joint'][m][-1])})
def harmonic(t,x,f):
    A=np.column_stack([np.ones(len(t)),t-t.mean(),np.cos(2*np.pi*f*t),np.sin(2*np.pi*f*t)])
    c=np.linalg.lstsq(A,x,rcond=None)[0]
    return complex(c[2],-c[3]),float(np.sqrt(np.mean((x-A@c)**2)))
offset=((meta[runs[-1]]['finish']['excitation_start_us']-meta[runs[-1]]['start_us'])&0xffffffff)*1e-6
lat=((a['submit_us']-a['at_us']+2**31)%2**32-2**31)*1e-6
coefs=[]
for i in range(4):
    lo=offset+(i+1)/.25;hi=lo+4;m=(t>=lo-1e-7)&(t<hi-1e-7)&(a['stage']==2)&((a['flags'].astype(int)&16)!=0)
    Y,n=harmonic(t[m],a['feedback'][m],.25);U,_=harmonic(t[m]+lat[m],a['command_a'][m],.25);D,_=harmonic(t[m],a['excitation_a'][m],.25)
    coefs.append((Y,U,D,n))
for drop in [None,0,1,2,3]:
    c=[x for i,x in enumerate(coefs) if i!=drop]
    Y,U,D,n=[np.array([x[i] for x in c]) for i in range(4)]
    G=np.sum(Y*D.conj())/np.sum(U*D.conj())
    summary['leave_one_cycle_out_diagnostic'].append({'excluded_cycle':None if drop is None else drop+1,
        'magnitude':float(abs(G)),'phase_deg':float(np.angle(G,deg=True)),'repeat':float(np.std(Y/U)/abs(G)),
        'median_snr':float(np.median(abs(Y)/(np.sqrt(2)*n)))})
with (args.output/'stall-analysis.json').open('x') as f:json.dump(summary,f,ensure_ascii=False,indent=2,allow_nan=False)
print(json.dumps({k:summary[k] for k in ('stall_window','time_windows','leave_one_cycle_out_diagnostic')},ensure_ascii=False,indent=2))
