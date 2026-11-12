"""main速度闭环辨识: 显式执行有限轨迹, MCU生成目标; 不写PID、不重连、不自动增加激励."""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import queue
import time
import sentinel_client as wire
from sentinel_async import AsyncSentinelClient

DEFAULT_PLAN = dict(speed=1., amplitude_a=.06, bias_a=.18, acceleration=.8, ramp_s=5.,
                    settle_timeout_s=15., zero_s=3.,
                    frequencies=[.25,.5,1.,2.,4.], cycles=4)


def duration(plan):
    return 2*plan['ramp_s']+plan['settle_timeout_s']+plan['zero_s']+sum((plan['cycles']+2)/f for f in plan['frequencies'])


def validate_plan(p):
    values=[p[k] for k in ('speed','amplitude_a','bias_a','acceleration','ramp_s','settle_timeout_s','zero_s')]+p['frequencies']
    if (not all(math.isfinite(x) for x in values) or p['speed']==0 or p['amplitude_a']<=0 or p['bias_a']<0 or
        p['bias_a']+p['amplitude_a']>=3 or p['settle_timeout_s']<2 or
        min(p['acceleration'],p['ramp_s'],p['zero_s'])<=0 or not 1<=len(p['frequencies'])<=6 or
        not all(0<f<=20 for f in p['frequencies']) or
        any(b<=a for a,b in zip(p['frequencies'],p['frequencies'][1:])) or
        not 3<=p['cycles']<=16 or duration(p)>=300 or
        1.875*abs(p['speed'])/p['ramp_s']>p['acceleration']):
        raise ValueError('试验计划非法: 检查单向速度/幅度、加速度、频点及有限时长')


def accepted(value):
    if 'result' in value and value['result'] != 0:
        raise wire.ProtocolError(f'请求被拒绝: {value}')
    return value


def wait_execution(client, indices, enabled, timeout=2):
    deadline=time.monotonic()+timeout
    while True:
        page=accepted(client.status(1).result(2))
        if all(page['domains'][i]['execute']==enabled for i in indices): return
        if time.monotonic()>=deadline: raise wire.ProtocolError('目标执行状态未建立/撤销; 不启动激励')


def collect(client, incoming, folder, plan, parameters, hello, progress=print):
    folder.mkdir(parents=True,exist_ok=False)
    summary=dict(complete=False,schema=2,plan=plan,hello=hello,parameters_before=parameters,samples=0,
                 current_unit='A; command_a已经换算, 不再乘3',instrument='excitation_a')
    try:
        summary['runtime_before']=client.diagnostics(2).result(2)
        started=accepted(client.identification(1,plan).result(2));summary['start']=started
        expected=0;last=None;writer=None;deadline=time.monotonic()+duration(plan)+5;next_status=0
        progress(f"开始有限试验 {plan['speed']:+g}rad/s, 最长{duration(plan):.1f}s; 电流扰动{plan['amplitude_a']:.3f}A, 不调整参数或重试")
        with (folder/'samples.csv').open('x',newline='',encoding='utf-8') as dest:
            while True:
                try: packet=incoming.get(timeout=.01)
                except queue.Empty: packet=None
                if packet:
                    if packet.get('schema')!=2 or packet['run_id']!=started['run_id'] or packet['first_sequence']!=expected or packet['dropped']:
                        raise wire.ProtocolError('高频记录身份/序号缺口; 已有文件保留, 不重做动作')
                    summary['start_us']=packet['start_us']
                    for sample in packet['samples']:
                        if sample['flags']&15 != 7:
                            raise wire.ProtocolError(f"控制/发送/反馈无效或触限: sequence={sample['sequence']}, flags={sample['flags']}")
                        if last is not None and not 0<((sample['at_us']-last)&0xffffffff)<=5000:
                            raise wire.ProtocolError('控制记录时基异常')
                        last=sample['at_us']
                        row=dict(**sample,time_s=((sample['at_us']-packet['start_us'])&0xffffffff)*1e-6,
                                 command_a=sample['command_raw']*started['raw_to_axis_amp'])
                        if writer is None:
                            writer=csv.DictWriter(dest,fieldnames=list(row));writer.writeheader()
                        writer.writerow(row);expected+=1;summary['samples']=expected
                now=time.monotonic()
                if now>=next_status:
                    status=accepted(client.identification(2).result(2));summary['finish']=status;next_status=now+.2
                finish=summary.get('finish',{})
                if finish.get('state') in (3,4) and expected==finish['produced']:
                    if finish['state']==4:
                        reason='工作点在限时内未建立, 未开始频点, 返回现有记录' if finish['reason']==6 else f"MCU试验中止, reason={finish['reason']}"
                        raise wire.ProtocolError(reason)
                    break
                if now>deadline: raise TimeoutError('有限试验/记录未按期结束; 不自动重连')
                state=client.snapshot()
                if state['error'] or state.get('log_dropped') or state.get('log_error'):
                    raise wire.ProtocolError(f"链路或记录错误: {state['error']} {state.get('log_error','')}")
        summary['runtime_after']=client.diagnostics(2).result(2)
        summary['parameters_after']=client.read_all_params().result(10)
        if summary['parameters_before']!=summary['parameters_after']: raise wire.ProtocolError('试验中参数发生变化')
        summary['complete']=True
    except BaseException as error:
        summary['error']=f'{type(error).__name__}: {error}'
        raise
    finally:
        (folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return summary


def run(transport, output, plan, signs=(1,-1), *, progress=print):
    validate_plan(plan)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    incoming=queue.Queue(maxsize=512)
    result=dict(complete=False,started_unix_s=time.time(),plan=plan,trials=[])
    client=AsyncSentinelClient(transport,target_hz=50,fast_hz=10,timeout_s=.5,attempts=1,
        log_path=output/'wire.jsonl',identification_sink=incoming.put_nowait)
    active=False
    try:
        hello=client.hello().result(2);parameters=client.read_all_params().result(10)
        capability=accepted(client.identification(2).result(2))
        if capability.get('schema')!=2:
            raise wire.ProtocolError('需要V2电流激励固件, 当前是旧速度参考试验; 尚未授权或发目标')
        fast=accepted(client.status(0).result(2))
        if fast['main_platform']['validity']!='valid' or any(not fast['joints'][d]['position_valid'] for d in ('main_yaw','fire_yaw','pitch')):
            raise wire.ProtocolError('main惯性或关节参考不可用; 回报现状, 不自动对齐')
        accepted(client.authorize(True,'maintenance').result(2));active=True
        deadline=time.monotonic()+2
        while True:
            fast=client.status(0).result(2)
            if fast.get('source')=='maintenance' and fast.get('authority'): break
            if time.monotonic()>=deadline: raise wire.ProtocolError('维护未持权; 检查既有遥控档, 不伪造DR16')
        epochs={k:fast[k] for k in ('config_epoch','reference_epoch')}
        holds=[('fire_yaw','position',fast['joints']['fire_yaw']['angle_rad'],0),
               ('pitch','position',fast['joints']['pitch']['angle_rad'],0)]
        def source(points):
            return lambda now,current: (dict(setpoints=points,twist=(0.,0.,0.),main_yaw_frame='local_inertial',**epochs)
                                        if current==epochs else None)
        for sign in signs:
            # 正反向之间不释放fire/pitch, 只撤main旧意图, 显式clear后用当前heading建立新意图.
            client.set_target_source(source(holds));wait_execution(client,(4,),False)
            accepted(client.identification(3).result(2))
            fast=client.status(0).result(2)
            client.set_target_source(source(holds+[('main_yaw','position',fast['main_platform']['heading_rad'],0)]))
            wait_execution(client,(4,5,6),True)
            name='positive' if sign>0 else 'negative'
            collect(client,incoming,output/name,dict(plan,speed=sign*abs(plan['speed'])),parameters,hello,progress)
            result['trials'].append(name)
        result['recording_complete']=True
        result['complete']=True
    except BaseException as error:
        result['error']=f'{type(error).__name__}: {error}'
        raise
    finally:
        if active:
            try:
                client.set_target_source(None)
                accepted(client.identification(0).result(2))
                accepted(client.authorize(False,'maintenance').result(2))
            except Exception as error: result['cleanup_error']=str(error)
        client.close();state=client.snapshot()
        result.update(statistics=state['statistics'],log_dropped=state.get('log_dropped',0),log_error=state.get('log_error',''))
        if result.get('cleanup_error') or result['log_dropped'] or result['log_error']: result['complete']=False
        (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--serial',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--speed',type=float,default=1.,help='试验基准速度, 非已实测控制参数')
    p.add_argument('--current-amplitude',type=float,default=.06,help='独立电流正弦幅值A')
    p.add_argument('--bias-current',type=float,default=.18,help='仅试验的固定负载偏置幅值A')
    p.add_argument('--acceleration',type=float,default=.8,help='只约束起停速度曲线, 不约束电流正弦')
    p.add_argument('--directions',choices=['both','positive','negative'],default='both')
    p.add_argument('--frequencies',default=None,help='逗号分隔频点Hz; 默认用内置列表')
    p.add_argument('--cycles',type=int,default=None,help='每频点测量周期数; 默认用内置值')
    p.add_argument('--execute',action='store_true',help='显式执行; 不指定只打印计划, 不打开串口')
    a=p.parse_args();plan=dict(DEFAULT_PLAN,speed=abs(a.speed),amplitude_a=a.current_amplitude,bias_a=a.bias_current,acceleration=a.acceleration)
    if a.frequencies is not None: plan['frequencies']=[float(x) for x in a.frequencies.split(',') if x.strip()]
    if a.cycles is not None: plan['cycles']=a.cycles
    validate_plan(plan)
    print(json.dumps(dict(plan=plan,maximum_duration_per_direction_s=duration(plan),directions=a.directions),ensure_ascii=False,indent=2))
    if not a.execute: return
    signs=(1,-1) if a.directions=='both' else (1,) if a.directions=='positive' else (-1,)
    port=wire.SerialTransport(a.serial,921600)
    try:
        if not run(port,a.output,plan,signs)['complete']: raise SystemExit('采集或撤权未完整确认, 返回summary与已有材料')
    finally: port.close()
    print('有限试验结束, 已撤目标/权限, PID未改. 使用sentinel_identify_analyze.py离线分析并回传.')


if __name__=='__main__': main()
