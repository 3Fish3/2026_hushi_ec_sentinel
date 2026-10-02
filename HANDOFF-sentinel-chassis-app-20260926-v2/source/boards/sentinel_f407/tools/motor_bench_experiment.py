"""单颗 GM6020 辨识/验证入口: 参数、有限目标、原始记录. 不透传CAN、不自动扫描端口."""
import argparse
import hashlib
import json
from pathlib import Path
import queue
import time
import motor_bench as bench


def run(args):
    folder=args.output
    folder.mkdir(parents=True,exist_ok=False)
    options=vars(args).copy()
    options={k:str(v) if isinstance(v,Path) else v for k,v in options.items()}
    metadata={'experiment':options,'firmware_artifact_sha256':hashlib.sha256(args.firmware.read_bytes()).hexdigest(),
              'firmware_identity_boundary':'主机工件身份; 实际烧录对应关系须由烧录校验日志证明'}
    events=queue.Queue()
    session=bench.Session(args.port,folder/'session.jsonl',events,metadata=metadata)
    statuses=[];saved=None;started=time.monotonic()
    session.start()
    def receive(timeout=.5):
        kind,value=events.get(timeout=timeout)
        if kind in ('error','closed'): raise RuntimeError(f'{kind}: {value}')
        if kind=='status': statuses.append(value['data'])
        return kind,value
    def await_status(request, predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            kind,value=receive()
            if kind=='status' and (request is None or value['request']==request):
                if not predicate(value['data']): raise RuntimeError(f'请求 {request} 未满足: {value["data"]}')
                return value['data']
        raise RuntimeError(f'请求 {request} 超时')
    try:
        await_status(None,lambda s: s['device_id']==args.id and s['bus']==1 and s['feedback_valid'] and not s['active'])
        if args.mode=='position':
            session.set_reference();await_status(0x24,lambda s:s['position_valid'])
        mode={'current':1,'velocity':2,'position':3}[args.mode]
        # 从零目标启动, 同模式目标/激励才触发窗口, 启动本身不重用旧积分.
        values=[0,args.seconds+.5,args.current_limit,args.speed_limit,args.kp,args.ki,args.position_kp,args.kd,args.alpha,args.derivative_tau,int(args.encoder_velocity)]
        session.start_motion(bench.settings_payload(mode,values))
        await_status(0x21,lambda s:s['active'] and s['result']==1)
        session.start_capture(2730 if args.compact else 1024,True,args.note,compact=args.compact)
        if args.wave=='step': session.update_target(args.target,round(args.ramp*1e6))
        else: session.excite(1 if args.wave=='sine' else 2,args.target,args.amplitude,args.frequency,round(args.seconds*1e6))
        await_status(0x25 if args.wave=='step' else 0x27,lambda s:s['active'] and s['result']==1)
        stop_at=time.monotonic()+args.seconds;stop_requested=False
        deadline=time.monotonic()+max(60,args.seconds+5)
        while time.monotonic()<deadline:
            if not stop_requested and time.monotonic()>=stop_at:
                session.stop_motion();stop_requested=True
            try: kind,value=receive(.05)
            except queue.Empty: continue
            if kind=='trace_saved': saved=Path(value)
            if saved is not None and stop_requested: break
        if saved is None: raise RuntimeError('没有取得完整冻结窗口')
        session.stop_motion()
        final=await_status(0x23,lambda s:not s['active'] and s['output_a']==0)
    finally:
        session.stop_motion();session.quit_event.set();session.join(2)
    if session.is_alive(): raise RuntimeError('串口线程未退出')
    outputs=bench.trace.export(saved)
    meta,data=bench.trace.load(saved);rows=bench.trace.rows(meta,data)
    result=dict(record=str(saved),outputs=outputs,metrics=bench.trace.metrics(rows),final=final,
                elapsed_host_s=time.monotonic()-started)
    with (folder/'result.json').open('x',encoding='utf-8') as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True);p.add_argument('--id',type=int,default=4)
    p.add_argument('--firmware',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=['current','velocity','position'],required=True)
    p.add_argument('--target',type=float,required=True);p.add_argument('--seconds',type=float,default=2)
    p.add_argument('--wave',choices=['step','sine','square'],default='step')
    p.add_argument('--amplitude',type=float,default=.02);p.add_argument('--frequency',type=float,default=5)
    p.add_argument('--ramp',type=float,default=0);p.add_argument('--compact',action='store_true')
    p.add_argument('--current-limit',type=float,default=3000*3/16384)
    p.add_argument('--speed-limit',type=float,default=3.4906585)
    p.add_argument('--kp',type=float,required=True);p.add_argument('--ki',type=float,default=0)
    p.add_argument('--kd',type=float,default=0);p.add_argument('--alpha',type=float,default=0)
    p.add_argument('--derivative-tau',type=float,default=0,help='独立D低通时间常数s, 0保持原算法')
    p.add_argument('--encoder-velocity',action='store_true',help='显式使用编码器5点因果测速, 默认原生RPM')
    p.add_argument('--position-kp',type=float,default=7.5);p.add_argument('--note',default='')
    args=p.parse_args()
    if not 0<args.seconds<=3600-.5 or not 0<=args.ramp<2147: p.error('试验时间/斜坡超出接口范围')
    # 在打开串口前完成两端目标及参数合法性检查, 沿用既有模式量程.
    mode={'current':1,'velocity':2,'position':3}[args.mode]
    for target in ([args.target] if args.wave=='step' else [args.target-args.amplitude,args.target+args.amplitude]):
        bench.settings_payload(mode,[target,args.seconds+.5,args.current_limit,args.speed_limit,args.kp,args.ki,args.position_kp,args.kd,args.alpha,args.derivative_tau,int(args.encoder_velocity)])
    if args.wave!='step':
        # 预检不启动线程、不打开串口, 沿用请求编码器的合法性约束.
        bench.Session(args.port,args.output/'unused',queue.Queue()).excite(
            1 if args.wave=='sine' else 2,args.target,args.amplitude,args.frequency,round(args.seconds*1e6))
    if not args.firmware.is_file(): p.error('固件工件不存在')
    run(args)


if __name__=='__main__':main()
