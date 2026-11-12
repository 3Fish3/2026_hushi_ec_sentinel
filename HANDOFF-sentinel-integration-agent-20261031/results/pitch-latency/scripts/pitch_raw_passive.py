"""Passive 200 ms RAM recorder bursts on observation UART; no motor commands or authorization.
Existing frozen data is exported before any new recorder start. No reset/halt/flash.
"""
import argparse,json,struct,time,secrets
from pathlib import Path
import sentinel_client as wire

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
port='/dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148353-if02'
seq=0;t=None
summary={'purpose':'passive raw CAN receive timing; existing APP rotation recorder profile only',
 'motor_authorization_requested':False,'targets_sent':False,'parameter_writes':False,'reset_or_halt':False,
 'duration_us_each':200000,'bursts':[],'complete':False}

def request(op,run=0,profile=0,arg=0):
 global seq
 seq=(seq+1)&65535
 payload=struct.pack('<4B2I',1,op,profile,0,run,arg)
 t.send(wire.encode_frame(0x31,seq,payload))
 deadline=time.monotonic()+2
 while time.monotonic()<deadline:
  fr=t.receive(.02)
  if fr is None:continue
  kind,rseq,b=fr
  if kind!=0xb1 or rseq!=seq:continue
  if len(b)<32:raise ValueError('raw header short')
  v,rop,error,state,rid,prof,count,total,start,end,lost,layout,capacity=struct.unpack_from('<4BI2BH5I',b)
  if v!=1 or layout!=0x52415701 or len(b)!=32+count*64 or error:raise ValueError('raw reply invalid: '+str((v,layout,error,len(b),count)))
  return dict(op=rop,state=state,run=rid,profile=prof,total=total,start_us=start,end_us=end,lost=lost,layout_id=layout,capacity=capacity),[b[i:i+64] for i in range(32,len(b),64)]
 raise TimeoutError('observation recorder did not respond; no automatic reconnect or reset')

def export(status,label):
 records=[]
 while len(records)<status['total']:
  time.sleep(.065)
  page,batch=request(3,status['run'],arg=len(records))
  if page['run']!=status['run'] or not batch:raise ValueError('raw buffer changed')
  for rec in batch:
   fields=struct.unpack_from('<4B7I',rec)
   if fields[4]!=len(records) or fields[10]!=status['run']:raise ValueError('raw sequence/run mismatch')
   records.append(rec)
 metadata=dict(status,passive_only=True,label=label)
 blob=json.dumps(metadata,ensure_ascii=False).encode()
 with (a.output/(label+'.raw')).open('xb') as out:out.write(b'ECRAW1\r\n'+struct.pack('<I',len(blob))+blob+b''.join(records))
 cans=[]
 for rec in records:
  v,kind,stream,flags,rseq,source,sampled,arrived,lost,error,run=struct.unpack_from('<4B7I',rec)
  if kind!=4:continue
  info=struct.unpack_from('<I',rec,32)[0]
  cans.append(dict(stream=stream,sequence=rseq,source_sequence=source,sampled_us=sampled,arrived_us=arrived,
   lost=lost,bus=info&255,dlc=(info>>8)&255,can_id=info>>16,data=rec[36:44].hex()))
 with (a.output/(label+'-can.json')).open('x') as out:json.dump(cans,out,indent=2)
 return dict(status,label=label,records=len(records),can_records=len(cans),pitch_can_records=sum(x['bus']==2 and x['can_id']==17 for x in cans))

try:
 t=wire.SerialTransport(port,115200,timeout_s=.5)
 status,_=request(0);summary['initial_status']=status
 if status['state']==1:raise RuntimeError('an existing raw recording is active; leave it untouched')
 if status['total']:
  summary['preserved_prior_buffer']=export(status,'prior-buffer')
 for index in range(2):
  run=secrets.randbelow(0xffffffff)+1
  time.sleep(.065)
  status,_=request(1,run,profile=2,arg=200000)
  deadline=time.monotonic()+3
  while status['state']==1:
   if time.monotonic()>deadline:raise TimeoutError('raw recording did not finish; no reset')
   time.sleep(.075);status,_=request(0)
   if status['run']!=run:raise ValueError('raw run replaced')
  info=export(status,'burst-'+str(index+1));summary['bursts'].append(info)
  print('BURST',index+1,info,flush=True)
  if status['state']==3 or status['lost']:break
 summary['complete']=len(summary['bursts'])==2
except BaseException as error:
 summary['error']=type(error).__name__+': '+str(error)
 raise
finally:
 if t is not None:t.close()
 with (a.output/'summary.json').open('x') as f:json.dump(summary,f,ensure_ascii=False,indent=2)
 print('OUTPUT='+str(a.output)+' COMPLETE='+str(summary['complete']),flush=True)
