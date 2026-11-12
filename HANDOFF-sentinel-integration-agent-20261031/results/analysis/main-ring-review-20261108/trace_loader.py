import json,csv,math,statistics as st,bisect
from pathlib import Path
R=Path('<PROJECT_ROOT>/HANDOFF-sentinel-integration-agent-20261031/results')
def delta(a,b):return (int(a)-int(b)+2**31)%2**32-2**31
def deg(x):return math.degrees(x)
def load(name):
 p=R/name;s=json.loads((p/'summary.json').read_text());rs=[json.loads(l) for l in (p/'samples.jsonl').open()]
 p0=[r for r in rs if r['kind']=='status0'];start=s['sampling_start_monotonic_s'];first=min(p0,key=lambda r:abs(r['monotonic_s']-start));origin=first['data']['evaluated_us'];anchor_host=first['monotonic_s']-start
 offsets=[delta(r['data']['evaluated_us'],r['data']['control_ms']*1000) for r in p0];off=st.median(offsets)
 def microtime(us):return anchor_host+delta(us,origin)/1e6
 def millitime(ms):return microtime((int(ms)*1000+int(off))%2**32)
 out=[]
 for r in p0:
  d=r['data'];m=d['main_platform'];j=d['joints']['main_yaw'];w=d['wheel_odom'];tw=w['twist'];out.append(dict(t=microtime(d['evaluated_us']),host=r['monotonic_s']-start,h=deg(m['heading_rad']),a=deg(j['angle_rad']),rate=j['rate_rad_s'],valid=m['validity'],reason=m['reason'],epoch=m['local_epoch'],ref=d['reference_epoch'],source=d['source'],vx=tw[0],vy=tw[1],wz=tw[2],wvalid=w['validity'],gain=m['heading_gain'],plat_rate=m['rate_V_rad_s'],quat=m['orientation_V_to_local_wxyz']))
 ctr=[]
 with (p/'main_yaw-control.csv').open() as f:
  for d in csv.DictReader(f):ctr.append(dict(t=microtime(d['sampled_us']),target=deg(float(d['target'])),cmd=float(d['command_normalized']),cmin=float(d['command_min']),cmax=float(d['command_max']),eff=int(d['effort_raw']),emin=int(d['effort_min']),emax=int(d['effort_max']),execute=d['execute'],failed=d['submit_failed'],missing=d['feedback_missing'],limited=int(d['limited_samples']),n=int(d['control_samples']),seq=int(d['sequence'])))
 slow=[]
 for r in rs:
  if r['kind'] not in ('status1','status2'):continue
  d=r['data'];v=dict(t=millitime(d['clock_ms']),kind=r['kind'],ref=d['reference_epoch'],cfg=d['config_epoch'])
  if r['kind']=='status1':
   m=next(x for x in d['domains'] if x['domain']=='main_yaw');pi=next(x for x in d['domains'] if x['domain']=='pitch');v.update(target=deg(m['target']),measurement=deg(m['measurement']),execute=m['execute'],reason=m['reason'],pitch_execute=pi['execute'],pitch_reason=pi['reason'],source=d['source'])
  else:v.update(twist=d['adopted_twist'])
  slow.append(v)
 return s,out,ctr,slow,off
if __name__=='__main__':
 for name in ['B02/A-221039','B02fix/hold-230307','B02fix/kp28-231137','B02fix/kp28-232336']:
  s,x,c,slow,off=load(name);print('\nRUN',name,'clock us-ms offset median',off)
  ps={p['id']:p['value'] for p in s['parameters_before']};print('PID',ps[1032],ps[1033],ps[1034])
  if name=='B02/A-221039':
   event=[d for d in x if 13<=d['host']<=16];span=max(d['h'] for d in event)-min(d['h'] for d in event)
   assert len(event)>100 and all(d['valid']=='valid' for d in event) and span>20
   print('SYMPTOM CONFIRMED: valid heading span',round(span,4),'deg, samples',len(event))
   print('host vs MCU shifted t delta range ms',min((d['t']-d['host'])*1000 for d in event),max((d['t']-d['host'])*1000 for d in event))
   print('EVENT FAST (0.1s bins nearest): t heading mech mech_rate wheel_vx vy wz')
   for k in range(128,162):
    t=k/10;d=min(x,key=lambda d:abs(d['t']-t));print(round(d['t'],3),*[round(d[q],3) for q in ['h','a','rate','vx','vy','wz']])
   print('EVENT SLOW');print(*[d for d in slow if 12.8<=d['t']<=16.2],sep='\n')
   print('EVENT CONTROL (0.1s nearest): t target command cmin cmax raw_effort execute limited')
   for k in range(128,162):
    t=k/10;d=min(c,key=lambda d:abs(d['t']-t));print(round(d['t'],3),*[round(d[q],4) for q in ['target','cmd','cmin','cmax','eff']],d['execute'],d['limited'])
  print('trace target changes >0.5deg:')
  changes=[(a,b) for a,b in zip(c,c[1:]) if abs(b['target']-a['target'])>.5]
  for a,b in changes[:20]:print(round(b['t'],3),'target',round(a['target'],3),'->',round(b['target'],3),'delta',round(b['target']-a['target'],3),'execution',b['execute'])
  print('count target jumps',len(changes),'failed traces',sum(d['failed']=='True' for d in c),'limited cycles',sum(d['limited'] for d in c),'execute false intervals',sum(d['execute']!='True' for d in c))
