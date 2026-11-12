"""固定候选的15s无偏置速度验证; 单次start、连续积分、成套RAM保存和恢复."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import queue
import time
import sentinel_client as wire
from sentinel_async import AsyncSentinelClient
from sentinel_identify import accepted, wait_execution

PROFILE='main-yaw-zero-bias-15s/1'
CANDIDATE={0x408:6.3,0x409:.45,0x40a:0.,0x401:.4,0x402:.35,0x403:0.}
REASONS={0:'complete',1:'cancelled',2:'execution_lost',3:'controller',4:'reference_changed',
         5:'tx_failure',6:'working_point',7:'current_stop',8:'speed_stop',9:'parameters_changed'}


def values(parameters): return {p['id']:p['value'] for p in parameters}


def same(actual,expected):
    return all(k in actual and math.isclose(actual[k],v,rel_tol=2e-6,abs_tol=1e-8) for k,v in expected.items())


def write_tuple(client,target):
    for key,value in target.items(): accepted(client.param_write(key,value).result(2))
    current=client.read_all_params().result(10)
    if not same(values(current),target): raise wire.ProtocolError('整组参数读回不符, 不开始/恢复运动')
    return current


def collect(client,incoming,folder,parameters):
    summary=dict(profile=PROFILE,recording_complete=False,motion_complete=False,samples=0,parameters_before=parameters)
    try:
        summary['runtime_before']=client.diagnostics(2).result(2)
        start=accepted(client.identification(5).result(2));summary['start']=start
        expected=0;last=None;writer=None;deadline=time.monotonic()+20;next_status=0
        with (folder/'samples.csv').open('x',newline='',encoding='utf-8') as dest:
            while True:
                try: packet=incoming.get(timeout=.01)
                except queue.Empty: packet=None
                if packet:
                    if packet['schema']!=2 or packet['run_id']!=start['run_id'] or packet['first_sequence']!=expected or packet['dropped']:
                        raise wire.ProtocolError('记录身份/序号/缓冲覆盖异常, 保留已有数据')
                    summary['start_us']=packet['start_us']
                    for sample in packet['samples']:
                        row=dict(**sample,time_s=((sample['at_us']-packet['start_us'])&0xffffffff)*1e-6,
                            command_a=sample['command_raw']*start['raw_to_axis_amp'])
                        if writer is None: writer=csv.DictWriter(dest,fieldnames=list(row));writer.writeheader()
                        writer.writerow(row);expected+=1;summary['samples']=expected
                        # 停止/发送失败样本仍写入, 不因flags非正常值而丢掉真正的触发证据.
                        if last is not None and not 0<((sample['at_us']-last)&0xffffffff)<=5000:
                            raise wire.ProtocolError('记录时基异常')
                        last=sample['at_us']
                now=time.monotonic()
                if now>=next_status:
                    summary['finish']=accepted(client.identification(2).result(2));next_status=now+.2
                finish=summary.get('finish',{})
                if finish.get('state') in (3,4) and expected==finish['produced']:
                    summary['recording_complete']=True;summary['motion_complete']=finish['state']==3
                    summary['end_reason']=REASONS.get(finish['reason'],str(finish['reason']));break
                if now>deadline: raise TimeoutError('15s动作/流未按期结束, 不自动重试')
                state=client.snapshot()
                if state['error'] or state.get('log_dropped') or state.get('log_error'):
                    raise wire.ProtocolError(f"链路/日志错误: {state['error']} {state.get('log_error','')}")
        summary['metadata_after']=accepted(client.identification(4).result(2))
        summary['parameters_after']=client.read_all_params().result(10)
        summary['runtime_after']=client.diagnostics(2).result(2)
        if values(summary['parameters_after'])!=values(parameters):
            raise wire.ProtocolError('试验中有效参数发生变化')
    except BaseException as error:
        summary['error']=f'{type(error).__name__}: {error}'
        raise
    finally:
        (folder/'trial.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return summary


def run(transport,output,firmware=None,*,progress=print):
    folder=Path(output);folder.mkdir(parents=True,exist_ok=False)
    incoming=queue.Queue(maxsize=512)
    report=dict(schema=PROFILE,complete=False,recording_complete=False,restoration_confirmed=False,
        release_confirmed=False,firmware=firmware or {},candidate_normalized=CANDIDATE,
        candidate_physical_velocity=[1.2,1.05,0],candidate_position=[6.3,.45,0])
    client=AsyncSentinelClient(transport,target_hz=50,fast_hz=10,timeout_s=.5,attempts=1,
        log_path=folder/'wire.jsonl',identification_sink=incoming.put_nowait)
    active=False;touched=False;saved={};holds_source=None
    try:
        report['hello']=client.hello().result(2)
        report['metadata_before']=accepted(client.identification(4).result(2))
        capability=report['metadata_before'].get('validation')
        if not capability or capability['version']!=1:
            raise wire.ProtocolError('固件不支持15s固定验证; 尚未授权/写参数/发目标')
        if not math.isclose(capability['current_limit_a'],3.,abs_tol=1e-6) or capability['feedback_alpha']!=0:
            raise wire.ProtocolError('实际量程/反馈滤波与固定候选不同, 返回元数据而非自动改配置')
        fast=accepted(client.status(0).result(2))
        if fast['main_platform']['validity']!='valid' or any(not fast['joints'][d]['position_valid'] for d in ('main_yaw','fire_yaw','pitch')):
            raise wire.ProtocolError('惯性/关节参考无效, 不自动重新对齐')
        accepted(client.authorize(True,'maintenance').result(2));active=True
        deadline=time.monotonic()+2
        while True:
            fast=accepted(client.status(0).result(2))
            if fast.get('source')=='maintenance' and fast.get('authority'): break
            if time.monotonic()>=deadline: raise wire.ProtocolError('维护未持权, 沿用既有遥控首启/维护档')
        epochs={k:fast[k] for k in ('config_epoch','reference_epoch')}
        holds=[(d,'position',fast['joints'][d]['angle_rad'],0) for d in ('fire_yaw','pitch')]
        def source(points):
            return lambda now,current: dict(setpoints=points,twist=(0.,0.,0.),main_yaw_frame='local_inertial',**epochs) if current==epochs else None
        holds_source=source(holds);client.set_target_source(holds_source);wait_execution(client,(4,),False)
        accepted(client.identification(3).result(2))
        before=client.read_all_params().result(10);report['parameters_before']=before
        old=values(before);saved={k:old[k] for k in CANDIDATE}
        touched=True # 第一条写请求即使ACK不明, 清理仍须核对并恢复实际旧值.
        applied=write_tuple(client,CANDIDATE);report['parameters_applied']=applied
        report['metadata_applied']=accepted(client.identification(4).result(2))
        fast=accepted(client.status(0).result(2))
        client.set_target_source(source(holds+[('main_yaw','position',fast['main_platform']['heading_rad'],0)]))
        wait_execution(client,(4,5,6),True)
        progress('执行一次15s验证: 0 → +0.25 → 0 → -0.25 → 0rad/s; bias/D=0, 中间不清I')
        trial=collect(client,incoming,folder,applied)
        report.update(recording_complete=trial['recording_complete'],motion_complete=trial['motion_complete'],end_reason=trial['end_reason'])
    except BaseException as error:
        report['error']=f'{type(error).__name__}: {error}'
    finally:
        errors=[]
        if active:
            try:
                client.set_target_source(holds_source)
                status=accepted(client.identification(2).result(2))
                if status['state'] in (1,2): accepted(client.identification(0).result(2))
                wait_execution(client,(4,),False)
                if touched:
                    actual=values(client.read_all_params().result(10))
                    # 不重发未确认的候选写入; 此处是已撤执行后的明确旧值恢复事务.
                    changed={k:v for k,v in saved.items() if not same(actual,{k:v})}
                    report['parameters_restored']=write_tuple(client,changed)
                    report['restoration_confirmed']=same(values(report['parameters_restored']),saved)
                else: report['restoration_confirmed']=True
            except Exception as error: errors.append('restore: '+str(error))
            finally:
                client.set_target_source(None)
                try:
                    accepted(client.authorize(False,'maintenance').result(2));report['release_confirmed']=True
                except Exception as error: errors.append('release: '+str(error))
        client.close();state=client.snapshot()
        report.update(statistics=state['statistics'],log_dropped=state.get('log_dropped',0),log_error=state.get('log_error',''))
        if errors: report['cleanup_error']='; '.join(errors)
        report['complete']=bool(report.get('motion_complete') and report['recording_complete'] and report['restoration_confirmed'] and
            report['release_confirmed'] and not report.get('error') and not errors and not report['log_dropped'] and not report['log_error'])
        (folder/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--serial',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--firmware-bin',type=Path);p.add_argument('--firmware-elf',type=Path)
    p.add_argument('--execute',action='store_true')
    a=p.parse_args()
    print(json.dumps(dict(profile=PROFILE,duration_s=15,bias_a=0,excitation_a=0,speed_rad_s=.25,
        stop_current_a=.35,stop_speed_rad_s=.4,velocity_physical=[1.2,1.05,0],position=[6.3,.45,0]),ensure_ascii=False,indent=2))
    if not a.execute: return
    if not a.firmware_bin or not a.firmware_elf: p.error('执行时提供本次实际构建的--firmware-bin与--firmware-elf, 自动记录哈希')
    firmware={}
    for kind,path in [('bin',a.firmware_bin),('elf',a.firmware_elf)]:
        data=path.read_bytes();firmware[kind]=dict(name=path.name,size=len(data),sha256=hashlib.sha256(data).hexdigest())
    if a.output.exists(): p.error('输出目录已存在, 不覆盖之前记录')
    port=wire.SerialTransport(a.serial,921600)
    try: result=run(port,a.output,firmware)
    finally: port.close()
    from sentinel_velocity_validate_analyze import analyze, save
    verdict=analyze(a.output);save(a.output,verdict)
    print(json.dumps(verdict,ensure_ascii=False,indent=2))
    if not result['complete'] or verdict['verdict']!='A_numeric': raise SystemExit('返回现有记录/结束原因, 不加幅或自动重试')
    print('数值筛查A, 已恢复旧增益并撤权限. 连同动作观察回传, 不自动进入位置/小陀螺试验.')


if __name__=='__main__': main()
