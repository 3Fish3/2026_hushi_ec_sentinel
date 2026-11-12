"""Read-only protocol observation. No authorization, targets, parameter writes or reset."""
import argparse,json,time
from pathlib import Path
import sentinel_client as wire

p=argparse.ArgumentParser()
p.add_argument('--serial',required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--seconds',type=float,default=30)
a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
meta={'purpose':'read-only DM feedback-age and query-cadence observation','requested_seconds':a.seconds,
      'authorization_requested':False,'targets_sent':False,'parameters_written':False,'complete':False}
transport=None
n=0
try:
    transport=wire.SerialTransport(a.serial,921600,timeout_s=.5)
    c=wire.SentinelClient(transport)
    meta['hello']=c.hello()
    meta['status_before']=c.status(0)
    meta['diagnostics_before']=c.diagnostics(2)
    meta['devices_before']=c.diagnostics(1)
    meta['pitch_group_before']=c.diagnostics(20)
    begin=time.monotonic();next_diag=5.0
    print('OUTPUT='+str(a.output),flush=True)
    print('SOURCE='+str(meta['status_before'].get('source'))+' AUTHORITY='+str(meta['status_before'].get('authority')),flush=True)
    with (a.output/'observations.jsonl').open('x') as dest:
        while time.monotonic()-begin<a.seconds:
            status=c.status(0)
            elapsed=time.monotonic()-begin
            dest.write(json.dumps({'event':'status','host_elapsed_s':elapsed,'data':status},ensure_ascii=False,allow_nan=False)+'\n')
            dest.flush();n+=1
            if elapsed>=next_diag:
                for name,page in [('diagnostics',2),('devices',1),('pitch_group',20)]:
                    value=c.diagnostics(page)
                    dest.write(json.dumps({'event':name,'host_elapsed_s':time.monotonic()-begin,'data':value},ensure_ascii=False,allow_nan=False)+'\n')
                    dest.flush()
                next_diag+=5.0
            time.sleep(.02)
    meta['actual_seconds']=time.monotonic()-begin
    meta['diagnostics_after']=c.diagnostics(2)
    meta['devices_after']=c.diagnostics(1)
    meta['pitch_group_after']=c.diagnostics(20)
    meta['status_after']=c.status(0)
    meta['complete']=True
except BaseException as error:
    meta['error']=type(error).__name__+': '+str(error)
    raise
finally:
    if transport is not None:transport.close()
    meta['status_samples']=n
    with (a.output/'summary.json').open('x') as dest:
        json.dump(meta,dest,ensure_ascii=False,indent=2,allow_nan=False)
    print('COMPLETE='+str(meta['complete'])+' SAMPLES='+str(n),flush=True)
