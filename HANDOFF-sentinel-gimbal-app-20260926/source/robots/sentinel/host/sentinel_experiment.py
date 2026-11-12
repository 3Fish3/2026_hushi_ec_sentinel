#!/usr/bin/env python3
"""Gate A-C OFFLINE ONLY. No serial import, enable, PID write or live run option.
The native backend uses the same bounded MCU generator with synthetic guards.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

from experiment_offline import offline_path
import os
import autotune_dataset as ds
import autotune_identify as identify
import autotune_score as scoring
import sentinel_raw as raw

WEIGHTS={'E':.30,'T':.15,'O':.10,'V':.20,'U':.10,'S':.15}
AXES={'fire_yaw':5,'pitch':6}
_BACKEND_NAMES=('ec_sentinel_experiment_backend',)

def find_default_backend():
    """Locate the native offline generator so the CLI works without --backend.

    Search order: SENTINEL_OFFLINE_BACKEND env, then build/* trees next to this
    package. Returns None if nothing is found (caller must then require an
    explicit --backend). This never opens a serial/CAN device.
    """
    override=os.environ.get('SENTINEL_OFFLINE_BACKEND')
    if override:
        p=Path(override).expanduser()
        if p.is_file(): return p
    here=Path(__file__).resolve()
    repo=here.parents[3]  # .../source
    candidates=[]
    for name in _BACKEND_NAMES:
        candidates.append(repo/'build'/name)
        build=repo/'build'
        if build.is_dir():
            candidates.extend(sorted(build.glob('*/'+name)))
    for c in candidates:
        if c.is_file() and os.access(c,os.X_OK): return c
    return None

def safe(value):
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,dict): return {str(k):safe(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)): return [safe(v) for v in value]
    return value

def write_json(path,value):
    with path.open('x',encoding='utf-8') as f: f.write(json.dumps(safe(value),ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n')

def analyze_dataset(dataset):
    # Revalidate independently: caller-supplied quality flags cannot authorize scoring.
    q=ds.validate_samples(dataset['manifest'],dataset['samples'])
    metrics={k:{'status':'UNAVAILABLE','value':None,'reason':'data_quality_rejected'} for k in WEIGHTS}
    score_raw=None; score_reason=None
    if q['valid']:
        try:
            score_raw=scoring.score_dataset({**dataset,'quality':q})
            for k in WEIGHTS:
                t=score_raw['overall']['terms'][k]
                metrics[k]={'status':'AVAILABLE' if t['status']=='ok' else 'UNAVAILABLE',
                    'value':t['raw'],'normalized':t['normalized'],'reason':t.get('note') or t['status']}
        except (ValueError,TypeError,KeyError) as exc: score_reason=str(exc)
    axis=dataset['manifest']['axis']
    model=None; rejection='data_quality_rejected'
    if q['valid']:
        adapted,err=identify.adapt_m1_samples(dataset['samples'])
        if adapted is not None:
            model=identify.identify_model(adapted,dt_s=dataset['samples'][0]['dt_s'])
            rejection=model.reason if not model.valid else 'closed_loop_reference_diagnostic_only'
        else: rejection=err
    # No physical object input/torque calibration is supplied by these experiments.
    # Thus even a good reference->response fit is NOT a hardware PID candidate.
    return {'schema_version':'excitation-analysis-v1','axis':axis,'offline_only':True,
        'hardware_validated':False,'quality':q,'model':identify.model_to_dict(model) if model else None,
        'model_input':'adopted_reference_rad','model_output':'angle_rad,velocity_rad_s',
        'model_scope':'closed_loop_reference diagnostic, not actuator dynamics',
        'rejection_reason':rejection,'candidate':None,'candidate_promotable':False,
        'parameter_ids':({'velocity_kp':1281,'velocity_ki':1282,'velocity_kd':1283,
            'position_kp':1288,'position_ki':1289,'position_kd':1290} if axis=='fire_yaw' else {'mit_kp':2305,'mit_kd':2306}),
        'score':{'weights':WEIGHTS,'metrics':metrics,'L':score_raw['overall']['L'] if score_raw else None,
            'raw':score_raw,'reason':score_reason},
        'pitch_output':'UNAVAILABLE: MIT command is not measured torque' if axis=='pitch' else 'normalized synthetic output proxy, not calibrated current'}

def report(out,analysis):
    write_json(out/'analysis.json',analysis)
    (out/'report.md').write_text('# Excitation offline report\n\nSYNTHETIC when declared / OFFLINE_ONLY / NOT_HARDWARE_VALIDATED\n\n'
        f"Axis: {analysis['axis']}\n\nModel: {analysis['rejection_reason']}\n\n"
        'Balanced weights E/T/O/V/U/S = 30/15/10/20/10/15. Missing metrics remain UNAVAILABLE; L is null if incomplete.\n\n'
        'No candidate is promoted or written. Pitch torque is not measured. STOP is not physically confirmed.\n',encoding='utf-8')

def emit_demo(axis,pattern,backend,out):
    backend=offline_path(backend).resolve()
    if not backend.is_file(): raise ValueError('native simulator executable required')
    r=subprocess.run([str(backend),axis,pattern],capture_output=True,text=True,timeout=10,check=True)
    rows=list(csv.DictReader(io.StringIO(r.stdout)))
    if not 1<=len(rows)<=256: raise ValueError('native short-window length invalid')
    out.mkdir(parents=True,exist_ok=False)
    records=[]; events=[]; refs=[]; pos=.138; vel=0.0
    domain=AXES[axis]; nan=float('nan')
    for i,row in enumerate(rows):
        t=int(row['sampled_us']); ref=float(row['generated_reference_rad'])
        if not all(math.isfinite(float(row[k])) for k in ('generated_reference_rad','generated_velocity_rad_s','generated_acceleration_rad_s2')): raise ValueError('nonfinite native reference')
        # Explicit synthetic response, not a claim about the real motor. Pre-update state.
        output=max(-1.0,min(1.0,20.0*(ref-pos)-.5*vel))
        flags=(1|2|4|8|32|64|(1<<8)|(1<<16)|(1<<20)|(1<<24)|(1<<26)|(3<<28))
        if axis=='fire_yaw': flags|=16
        head=lambda kind,seq:struct.pack('<4B7I',1,kind,domain,0,seq,i,t,t,0,0,17)
        state=head(6,2*i)+struct.pack('<I6fI',flags,ref,ref,pos,vel,output if axis=='fire_yaw' else nan,.001,7)
        provenance=1 if axis=='pitch' else 2
        frame_id=1 if axis=='pitch' else 0x2ff
        aux=head(7,2*i+1)+struct.pack('<5I3f',frame_id|(provenance<<16),8,0,0,1|(1<<8)|(1<<16),
            70.0 if axis=='pitch' else 0.0,2.0 if axis=='pitch' else 0.0,nan)
        assert len(state)==len(aux)==64
        records.extend((state,aux))
        refs.append({'session_id':17,'axis':axis,'sampled_us':t,'source_sequence':i,'param_epoch':7,
            'generated_reference_rad':ref,'simulated_adopted_reference_rad':ref,
            'adoption_evidence':'SYNTHETIC; native FSM, not deployed app actuation',
            'velocity_reference_rad_s':float(row['generated_velocity_rad_s']),
            'acceleration_reference_rad_s2':float(row['generated_acceleration_rad_s2'])})
        acceleration=100.0*output-5.0*vel
        vel+=acceleration*.001;pos+=vel*.001
    gains={'velocity_kp':.17,'velocity_ki':3.0,'velocity_kd':0.0,'position_kp':28.0,'position_ki':0.0,'position_kd':.1} if axis=='fire_yaw' else {'mit_kp':70.0,'mit_kd':2.0}
    manifest={'schema_version':1,'run_id':'run-17','axis':axis,'axis_domain':domain,
        'synthetic':True,'boot_id':'synthetic-only','firmware':{'id':'native-offline-excitation','validated':False},
        'units':dict(scoring.CANONICAL_UNITS),'param_set':{'param_epoch':7,'values':gains,'source':'synthetic-fixture'},
        'target_source_map':{'3':'maintenance'},'scale':scoring.frozen_scale(axis),
        'scenarios':[{'name':'bounded_slew_not_instant_step','kind':'continuous','i0':0,'i1':len(rows)}],
        'context':{'scene':'native-synthetic-short-session','load':'synthetic','temperature':'unknown','session':'17'},
        'safety_envelope':{'approved':False,'provenance':'SYNTHETIC_NOT_HARDWARE'},
        'flags':{'autotune_enabled':False,'hardware_actuation':False,'param_autowrite':False,'shooting_enabled':False},
        'session_id':17,'nonce':31,'pattern':pattern,'files':{'raw':['trace.sraw']}}
    ds.install_trace_decoders()
    raw.save(out/'trace.sraw',{'layout_id':raw.LAYOUT,'schema':1,'run':17,'start_us':0},records)
    write_json(out/'manifest.json',manifest)
    dataset=ds.load_dataset(out)
    ds.write_samples(out/'samples.csv',dataset['samples'])
    # Keep raw as canonical input; CSV is a derivative, never silently preferred over raw evidence.
    write_json(out/'quality.json',dataset['quality'])
    events=[{'event':'SIMULATED_PREPARE_ARM_START','session_id':17,'sampled_us':0},
        {'event':'SYNTHETIC_REFERENCE_GENERATION_FINISHED','session_id':17,
         'stop_confirmed':False,'not_hardware_evidence':True}]+dataset['events']
    (out/'events.jsonl').write_text(''.join(json.dumps(safe(e),sort_keys=True,allow_nan=False)+'\n' for e in events),encoding='utf-8')
    write_json(out/'excitation.json',{'synthetic':True,'session_id':17,'nonce':31,'pattern':pattern,
        'approved_hardware_config':None,'center_rad':.138,'amplitude_rad':.02,'duration_us':120000,'references':refs})
    write_json(out/'parameters.json',{'synthetic':True,'start':{'param_epoch':7,'values':gains},'end':{'param_epoch':7,'values':gains}})
    report(out,analyze_dataset(dataset))
    return {'status':'OFFLINE_ONLY','samples':len(rows),'synthetic':True,'hardware_blocked':True,'hardware_validated':False}

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__); sp=p.add_subparsers(dest='command',required=True)
    sp.add_parser('status',help='offline capability; never opens a port')
    run=sp.add_parser('run',help='dry-run simulation ONLY')
    run.add_argument('--axis',choices=AXES,required=True);run.add_argument('--pattern',choices=('bounded_step','trapezoid'),default='bounded_step')
    run.add_argument('--backend',type=offline_path,default=None,help='native simulator executable; auto-discovered if omitted')
    run.add_argument('--output',type=offline_path,required=True)
    stop=sp.add_parser('stop',help='no live STOP transport is enabled at Gate A-C');stop.add_argument('--session-id',type=int,required=True)
    a=sp.add_parser('analyze',help='disk files only');a.add_argument('--input',type=offline_path,required=True)
    a.add_argument('--output',type=offline_path,required=True);a.add_argument('--objective',choices=('balanced',),default='balanced');a.add_argument('--read-only',action='store_true',required=True)
    args=p.parse_args(argv)
    try:
        if args.command=='status': result={'hardware_enabled':False,'hardware_blocked':True,'pitch_hardware_blocked':True,'simulation':True,'hardware_validated':False,'gate':'A-C_OFFLINE_ONLY','live_transport':'DISABLED_IN_THIS_CLI'}
        elif args.command=='stop':
            print(json.dumps({'status':'STOP_NOT_CONFIRMED','physical_estop_required_if_live':True}));return 2
        elif args.command=='run':
            backend=args.backend or find_default_backend()
            if backend is None:
                raise ValueError('native simulator executable required (pass --backend or set SENTINEL_OFFLINE_BACKEND; build ec_sentinel_experiment_backend)')
            result=emit_demo(args.axis,args.pattern,backend,args.output)
        else:
            dataset=ds.load_dataset(args.input);analysis=analyze_dataset(dataset)
            args.output.mkdir(parents=True,exist_ok=False);report(args.output,analysis)
            result={'status':'READ_ONLY_OFFLINE','quality_valid':analysis['quality']['valid']}
        print(json.dumps(safe(result),sort_keys=True,allow_nan=False));return 0
    except (ValueError,OSError,KeyError,TypeError,subprocess.SubprocessError) as exc:
        print(json.dumps({'status':'REJECTED','reason':str(exc)},ensure_ascii=False),file=sys.stderr);return 2
if __name__=='__main__': raise SystemExit(main())
