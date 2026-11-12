import sys,os,subprocess,io,csv,json,math,hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(os.environ['CLAUDE_JOB_DIR'])/'tmp'))
from main_ring_review import load
out=Path(os.environ['CLAUDE_JOB_DIR'])/'tmp/cascade-replay';out.mkdir(exist_ok=True)
all_results=[]
for name,event in [('B02/A-221039',(12,16)),('B02fix/hold-230307',(3,18)),('B02fix/kp28-231137',(5,30)),('B02fix/kp28-232336',(3,30))]:
 s,fast,ctrl,slow,off=load(name);params={p['id']:p['value'] for p in s['parameters_before']};targets={p['target'] for p in ctrl};assert len(targets)==1
 header=[params[1032],params[1033],params[1034],params[1025],params[1026],math.radians(next(iter(targets)))];lines=[' '.join(map(str,header))]
 for p in fast:
  assert p['valid']=='valid'
  lines.append(' '.join(str(v) for v in [p['t'],math.radians(p['a']),*p['quat'],*p['plat_rate']]))
 text='\n'.join(lines)+'\n';key=name.replace('/','_');source=out/(key+'.input');source.write_text(text)
 p=subprocess.run([str(Path(os.environ['CLAUDE_JOB_DIR'])/'tmp/cascade_trace_replay')],input=text,text=True,capture_output=True);print('native',name,'exit',p.returncode)
 if p.returncode:print(p.stderr);continue
 dest=out/(key+'.csv');dest.write_text(p.stdout);result=list(csv.DictReader(io.StringIO(p.stdout)));t=np.array([float(r['t_s']) for r in result]);at=np.array([c['t'] for c in ctrl]);pred=np.interp(at,t,[float(r['command_a']) for r in result]);actual=np.array([c['cmd']*3 for c in ctrl]);baseline=(at>=.5)&(at<=3);bias=float(np.median(actual[baseline]-pred[baseline]));pred+=bias;mask=(at>=event[0])&(at<=event[1]);residual=actual[mask]-pred[mask];ix=(t>=event[0])&(t<=event[1]);d={
  'source':name,'position_pid':header[:3],'event_s':event,'unknown_initial_integral_offset_fitted_on_s':[.5,3],'fitted_offset_a':bias,'output_correlation':float(np.corrcoef(actual[mask],pred[mask])[0,1]),'output_difference_rms_a':float(np.sqrt(np.mean(residual**2))),'observed_endpoint_abs_peak_a':float(np.max(np.abs(actual[mask]))),'replayed_abs_peak_a':float(np.max(np.abs(pred[mask]))),'estimated_terms_abs_peak_a':{k:float(np.max(np.abs([float(r[k]) for i,r in enumerate(result) if ix[i]]))) for k in ['position_p_a','rate_p_a','outer_i_a','inner_i_a']},'model_scope':'Native kinematics and PositionLoop/Pid with fixed measured motion interpolated on 1ms grid; 2ms outer update; not closed-loop dynamics or true 1kHz acquisition. Constant integral offset estimated only on quiet 0.5..3s. Logs do not separately measure individual integrator states.'};all_results.append(d);print(json.dumps(d,ensure_ascii=False,indent=2))
(out/'summary.json').write_text(json.dumps(all_results,ensure_ascii=False,indent=2)+'\n')
