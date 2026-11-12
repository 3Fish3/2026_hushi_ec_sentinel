import argparse,json,csv,hashlib,collections
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);a=p.parse_args()
b=a.base
obsdir=b/'readonly-20261111T004019'
s=json.loads((obsdir/'summary.json').read_text())
records=[json.loads(l) for l in (obsdir/'observations.jsonl').open()]
rows=[x['data'] for x in records if x['event']=='status']
assert s['complete'] and not s['authorization_requested'] and not s['targets_sent'] and not s['parameters_written']
d0=s['diagnostics_before'];d1=s['diagnostics_after']
keys=['cycles','dm_query_sent','dm_command_sent','dm_enable_sent','dm_disable_sent','dm_tx_error','group_tx_error','group_refused','reconstruction_missing','imu_invalid','imu_long_gap','imu_invalidations']
delta={k:(d1[k]-d0[k])%2**32 for k in keys}
result={'scope':'read-only observation; no motion authorization, controller changes, reset or halt',
 'observation':{'folder':obsdir.name,'seconds':s['actual_seconds'],'samples':len(rows),
 'source_counts':dict(collections.Counter(x['source'] for x in rows)),
 'authority_counts':{str(k):v for k,v in collections.Counter(x['authority'] for x in rows).items()},
 'platform_states':dict(collections.Counter(x['main_platform']['validity'] for x in rows)),
 'platform_reasons':dict(collections.Counter(x['main_platform']['reason'] for x in rows)),
 'counter_delta':delta,'query_submit_hz_approx':delta['dm_query_sent']/s['actual_seconds'],
 'pitch_device_before':next(x for x in s['devices_before']['devices'] if x['domain']=='pitch'),
 'pitch_device_after':next(x for x in s['devices_after']['devices'] if x['domain']=='pitch')},
 'pitch_age_bins':[],'raw_receive_windows':[],'can_mailbox':{},'previous_active_trials':[],'input_sha256':{}}
for lo,hi in [(0,5),(5,10),(10,20),(20,50),(50,100),(100,1000)]:
 rr=[x for x in rows if x['joints']['pitch']['position_age_ms'] is not None and lo<=x['joints']['pitch']['position_age_ms']<hi]
 result['pitch_age_bins'].append({'age_ms_ge':lo,'age_ms_lt':hi,'count':len(rr),
  'platform_states':dict(collections.Counter(x['main_platform']['validity'] for x in rr))})
rawdir=b/'rawpassive-20261111T005214'
for path in sorted(rawdir.glob('burst-*-can.json')):
 rr=[x for x in json.loads(path.read_text()) if x['bus']==2 and x['can_id']==17]
 times=np.array([x['sampled_us'] for x in rr],dtype=np.int64)
 gaps=np.diff(times)/1000;seqdiff=np.diff(np.array([x['source_sequence'] for x in rr]))
 result['raw_receive_windows'].append({'file':path.relative_to(b).as_posix(),'recorded_pitch_frames':len(rr),
  'node_nibbles':dict(collections.Counter(str(int(x['data'][:2],16)&15) for x in rr)),
  'status_nibbles':dict(collections.Counter(str(int(x['data'][:2],16)>>4) for x in rr)),
  'recorded_gap_ms':gaps.tolist(),'source_sequence_deltas':seqdiff.tolist(),
  'uninterrupted_source_sequence_gaps_ms':[float(gap) for gap,n in zip(gaps,seqdiff) if n==1 and gap>5],
  'lost_values':sorted({x['lost'] for x in rr}),
  'note':'Recorder CAN rate limit is 2ms; source sequence deltas identify whether an intermediate matched RX frame existed. arrived_us is set equal to sampled_us in recorder and is not a queue latency measurement.'})
log=(b/'mailbox-20261111T005906/openocd.log').read_text()
assert 'CPU_STATE_BEFORE running' in log and 'CPU_STATE_AFTER running' in log
snap=[];mcr=None
for line in log.splitlines():
 if line.startswith('CAN2_MCR '):mcr=int(line.split()[1],0)
 if not line.startswith('MAILBOX2_SAMPLE '):continue
 _,i,sa,tir,dlc,low,sb=line.split();sa,tir,dlc,low,sb=[int(v,0) for v in (sa,tir,dlc,low,sb)]
 stable=sa==sb and not(tir&1)
 query=((tir>>21)&0x7ff)==0x7ff and (dlc&15)==4 and low==0xcc0001
 evidence=stable and query and bool(sa&(1<<16)) and bool(sa&(1<<18)) and not bool(sa&(1<<17))
 snap.append({'index':int(i),'tsr_before':hex(sa),'tsr_after':hex(sb),'tir2':hex(tir),'dlc2':dlc&15,
  'data_low2':hex(low),'query_id':(tir>>21)&0x7ff,'stable':stable,'matches_pitch_query':query,'completed_arbitration_loss':evidence})
assert mcr is not None and mcr&(1<<4)
result['can_mailbox']={'cpu_before':'running','cpu_after':'running','can2_mcr':hex(mcr),
 'NART_set':True,'snapshots':snap,'stable_query_loss_snapshots':sum(x['completed_arbitration_loss'] for x in snap),
 'note':'Sticky status snapshots are not counts or an estimate of lost-frame probability. Multiple snapshots may refer to the same completion.'}
assert result['can_mailbox']['stable_query_loss_snapshots']>0
for path in sorted((b.parent/'velocity15').glob('*/trial.json')):
 m=json.loads(path.read_text());before=m.get('runtime_before');after=m.get('runtime_after');raw=path.parent/'samples.csv'
 if not before or not after or not raw.exists():continue
 with raw.open() as f:rr=list(csv.DictReader(f))
 if not rr:continue
 age=np.array([float(x['source_age_us']) for x in rr]);tt=np.array([float(x['time_s']) for x in rr])
 result['previous_active_trials'].append({'run':path.parent.name,'duration_s':float(tt[-1]-tt[0]),
 'max_source_age_us':float(age.max()),'p99_source_age_us':float(np.quantile(age,.99)),
 'counter_delta':{k:(after[k]-before[k])%2**32 for k in ('cycles','dm_query_sent','dm_command_sent','dm_enable_sent','dm_disable_sent','dm_tx_error','reconstruction_missing')}})
for directory in [obsdir,rawdir,b/'canregs-20261111T005746',b/'mailbox-20261111T005906']:
 for f in sorted(directory.rglob('*')):
  if f.is_file():result['input_sha256'][f.relative_to(b).as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
with (b/'diagnosis.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
print('Saved diagnosis.json')
print('Query submission Hz',result['observation']['query_submit_hz_approx'])
print('Platform states',result['observation']['platform_states'])
print('Stable actual-query arbitration-loss snapshots',result['can_mailbox']['stable_query_loss_snapshots'],'/',len(snap))
print('Active trial comparisons',len(result['previous_active_trials']))
