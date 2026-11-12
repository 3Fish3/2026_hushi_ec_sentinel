import argparse,csv,json,hashlib
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
for existing in ('angle-map.json',):
    target=a.output/existing
    if target.exists(): raise SystemExit(f'refusing to overwrite {target}')
runs=['run-20261109T215532','repeat-20261109T221358','amplitude010-20261109T233006']
nb=72;edges=np.linspace(0,2*np.pi,nb+1)
res={'scope':'offline angle-domain analysis; raw CSV, existing analysis.json and eligibility unchanged',
     'bins':nb,'bin_edges_rad':[float(x) for x in edges],'source_sha256':{},'records':[],'slow_zone':[]}
for run in runs:
    for direction in ('positive','negative'):
        raw=a.base/run/direction/'samples.csv'
        res['source_sha256'][str(raw)]=hashlib.sha256(raw.read_bytes()).hexdigest()
        with raw.open() as f:rows=list(csv.DictReader(f))
        col=lambda k:np.array([float(r[k]) for r in rows])
        j=col('joint');v=col('feedback');st=col('stage').astype(int)
        scan=(st>=2)&(st<=6);d=np.sign(np.mean(v[scan]));prof=[];mins=[]
        ang=np.mod(j,2*np.pi)
        for i in range(nb):
            m=scan&(ang>=edges[i])&(ang<edges[i+1])
            prof.append(float(np.mean(v[m]))*d if m.sum()>40 else None)
            mins.append(float(np.min(v[m]))*d if m.sum()>40 else None)
        finite=[(i,x) for i,x in enumerate(prof) if x is not None]
        low=[(i,x) for i,x in finite if x<np.mean([y for _,y in finite])-.05]
        rec={'run':run,'direction':direction,'turns':float((j[-1]-j[0])/(2*np.pi)),
             'mean_speed_rad_s':float(np.mean(v[scan])),'directional_speed_profile':prof,
             'directional_min_profile':mins,
             'slowest_bin_deg':float(np.degrees((edges[finite[int(np.argmin([y for _,y in finite]))][0]]+edges[finite[int(np.argmin([y for _,y in finite]))][0]+1])/2)),
             'slowest_bin_speed_rad_s':float(min(y for _,y in finite))}
        res['records'].append(rec)
# shared slow zone: fixed physical angle where all six profiles are locally depressed
stack=np.array([[np.nan if x is None else x for x in r['directional_speed_profile']] for r in res['records']])
norm=stack/np.nanmean(stack,axis=1,keepdims=True)
avg=np.nanmean(norm,axis=0)
order=np.argsort(avg)
for i in order[:6]:
    res['slow_zone'].append({'center_deg':float(np.degrees((edges[i]+edges[i+1])/2)),
        'normalised_speed':float(avg[i]),'per_record':[None if np.isnan(stack[k,i]) else float(stack[k,i]) for k in range(len(res['records']))]})
# per-revolution depth in the shared slow zone (top slow bin centre)
center=np.radians(res['slow_zone'][0]['center_deg'])
res['per_revolution_depth_pct']=[]
for run in runs:
    for direction in ('positive','negative'):
        raw=a.base/run/direction/'samples.csv'
        with raw.open() as f:rows=list(csv.DictReader(f))
        col=lambda k:np.array([float(r[k]) for r in rows])
        j=col('joint');v=col('feedback');st=col('stage').astype(int)
        scan=(st>=2)&(st<=6);d=np.sign(np.mean(v[scan]));sp=v*d
        ang=np.mod(j,2*np.pi);win=np.abs(np.remainder(ang-center+np.pi,2*np.pi)-np.pi)<np.radians(12)
        span=j[scan][-1]-j[scan][0]
        rev=np.floor((j-j[scan][0])/(2*np.pi)*np.sign(span) if span else 0)
        depths=[]
        for r in np.unique(rev[scan]):
            m=scan&(rev==r);dip=sp[m&win];rest=sp[m&~win]
            if dip.size<25 or rest.size<60:continue
            depths.append({'revolution':int(r),'depth_pct':float(100*(1-dip.mean()/rest.mean())),
                'dip_mean_rad_s':float(dip.mean()),'rest_mean_rad_s':float(rest.mean())})
        res['per_revolution_depth_pct'].append({'run':run,'direction':direction,'depth':depths})
res['interpretation_limits']=['Slow zone depth is descriptive; no mechanical cause is identified.',
    'Vehicles were tested at one working point only; zero-speed friction and other poses are not covered.',
    'Depth per revolution does not increase monotonically, so harness winding is not supported as the dominant cause by this data.']
with (a.output/'angle-map.json').open('x') as f:json.dump(res,f,ensure_ascii=False,indent=2,allow_nan=False)
print(json.dumps({'slow_zone':res['slow_zone'],'slowest_per_record':[{k:r[k] for k in ('run','direction','slowest_bin_deg','slowest_bin_speed_rad_s')} for r in res['records']],
    'per_revolution':res['per_revolution_depth_pct']},ensure_ascii=False,indent=2))
