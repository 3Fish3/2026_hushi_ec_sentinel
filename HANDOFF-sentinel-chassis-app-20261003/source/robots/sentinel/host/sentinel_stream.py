"""只读连续采集客户端: 1Mbps, 接收电机/IMU同一MCU时间轴, 不发送执行器命令."""
from __future__ import annotations
import argparse
from collections import Counter
import json
from pathlib import Path
import secrets
import struct
import threading
import time
import sentinel_raw as raw
from board_monitor import Decoder, encode_frame
from sentinel_capture import decode_page

BAUD=1000000
BUILD='capture-cap-stream-1m-v1'
HEADER=struct.Struct('<4B7I')


def decode_packet(payload):
    if len(payload)<32:
        raise ValueError('连续帧截断')
    keys=('version','op','error','state','run','generated','dropped','buffered','start_us','end_us','layout_id')
    status=dict(zip(keys,HEADER.unpack_from(payload)))
    if status['version']!=1 or status['op']>3 or status['state']>3 or status['layout_id']!=raw.LAYOUT:
        raise ValueError('连续帧版本/布局错误')
    if status['error']:
        raise ValueError(f'采集请求拒绝: {status["error"]}')
    records=[];offset=32
    while offset<len(payload):
        if status['op']!=3 or offset+4>len(payload):
            raise ValueError('连续数据类别错误')
        kind=payload[offset+1]
        if kind not in (1,2,3,4):
            raise ValueError('采集角色不接受控制记录/未知类型')
        size=48 if kind==1 else 44
        if offset+size>len(payload):
            raise ValueError('紧凑记录截断')
        record=payload[offset:offset+size]+bytes(64-size)
        row=raw.decode_record(record)
        if row['run']!=status['run']:
            raise ValueError('帧/记录run不一致')
        records.append(record);offset+=size
    if len(records)>3:
        raise ValueError('超过连续帧容量')
    return status,records


class StreamClient:
    def __init__(self,port):
        self.port=port;self.decoder=Decoder();self.seq=0
        self.on_data=lambda status,records:None
        self.ack={}

    def poll(self):
        data=self.port.read(min(max(self.port.in_waiting,1),16384))
        for kind,seq,payload in self.decoder.feed(data):
            if kind in (0xb2,0xb3):
                status,records=decode_packet(payload)
                if kind==0xb3:
                    self.on_data(status,records)
                else:
                    self.ack[seq]=status

    def command(self,op,run=0,profile=0,duration=0):
        self.seq=(self.seq+1)&65535
        seq=self.seq;self.ack.clear()
        frame=encode_frame(0x32,seq,raw.request(op,run,profile,duration))
        for _ in range(3):
            if self.port.write(frame)!=len(frame):
                raise OSError('串口写入不完整')
            end=time.monotonic()+.5
            while time.monotonic()<end:
                self.poll()
                if seq in self.ack:
                    result=self.ack.pop(seq)
                    if result['op']!=op:raise ValueError('连续采集应答操作不匹配')
                    return result
        raise TimeoutError('连续采集无应答; 不重启MCU')


def collect(port_name,output,seconds=30,profile='rotation',note='',axis='main',stop=None,progress=None):
    import serial
    if profile not in ('rotation','stationary') or not 1<=seconds<=1800:
        raise ValueError('请选择静止/运动, 时长1..1800秒')
    path=Path(output);path.parent.mkdir(parents=True,exist_ok=True)
    stop=stop or threading.Event();progress=progress or (lambda event:None)
    run=secrets.randbelow(0xffffffff)+1
    counters=Counter();first_source={};last_source={}
    final=None;latest=None;last_sequence=-1;gaps=0;complete=False;failure=None
    started=False;client=None;start=None
    with serial.Serial(port_name,BAUD,timeout=.02,write_timeout=.5) as port:
        reader=raw.RawClient(port)
        identity=reader.identity()
        if identity['build_id']!=BUILD:
            raise ValueError(f'不是本包只读连续采集镜像: {identity["build_id"]}')
        client=StreamClient(port)
        previous=client.command(0)
        if previous['state'] in (1,2):
            # 新连接开始记录前结束遗留采集, 仅操作记录器, 不发电机失能/置零.
            ended=False
            def discard_old(status,records):
                nonlocal ended
                if status['run']==previous['run'] and status['state']==3:ended=True
            client.on_data=discard_old
            client.command(2,previous['run'])
            end=time.monotonic()+2
            while not ended and time.monotonic()<end:client.poll()
            if not ended and client.command(0)['state']!=3:
                raise TimeoutError('旧记录队列未排空')
        before=decode_page(reader.exchange_wire(b'\x00',0x30,0xb0))
        client.decoder.rejected=0
        # 文件先以独占方式打开, 避免覆盖错误发生后才发现已经启动记录.
        with path.open('xb') as file:
            pending=[]
            client.on_data=lambda status,records:pending.append((status,records))
            try:
                started=True  # 启动ACK丢失也会在finally按本run请求停止采集.
                start=client.command(1,run,raw.PROFILES[profile],round(seconds*1e6))
                if start['run']!=run:raise ValueError('开始应答身份不匹配')
                metadata=dict(schema=1,layout_id=raw.LAYOUT,run=run,start_us=start['start_us'],
                    profile=profile,streamed=True,window_requested_us=round(seconds*1e6),
                    selected_rate_limits_hz={'gyro':250 if profile=='rotation' else 1000,
                                            'accel':100 if profile=='rotation' else 200,
                                            'each_gimbal_can':250 if profile=='rotation' else 0},
                    device_identity=identity,board_before=before,host_started_at=time.time(),baud=BAUD,axis=axis,note=note,
                    timebase='TIM5_us32',units={'gyro':'rad/s','accel':'m/s2','temperature':'degC'},
                    axes='BMI088 sensor S (unrotated, bias not removed)',
                    imu={'gyro_odr_hz':2000,'gyro_bw_hz':230,'accel_odr_hz':800,'synchronized':False},
                    completion_summary=path.name+'.summary.json')
                encoded=json.dumps(metadata,ensure_ascii=False,allow_nan=False).encode('utf-8')
                file.write(raw.MAGIC+struct.pack('<I',len(encoded))+encoded);file.flush()
                def receive(status,records):
                    nonlocal final,latest,last_sequence,gaps
                    if status['run']!=run:raise ValueError('连续采集中run被替换')
                    latest=status
                    for record in records:
                        r=raw.decode_record(record)
                        if r['sequence']<=last_sequence:raise ValueError('记录倒序/重复')
                        gaps+=r['sequence']-last_sequence-1;last_sequence=r['sequence']
                        key=f'{r["kind"]}:{r["stream"]}'
                        counters[key]+=1;first_source.setdefault(key,r['sampled_us']);last_source[key]=r['sampled_us']
                        counters['records']+=1
                        lost_key='can_queue_dropped_max' if r['kind']=='can' else 'gyro_discarded_max' if r['kind']=='gyro' else 'other_lost_max'
                        counters[lost_key]=max(counters[lost_key],r['lost'])
                        counters['gyro_fifo_overruns_max']=max(counters['gyro_fifo_overruns_max'],r.get('fifo_overruns',0))
                        file.write(record)
                    if status['state']==3:final=status
                client.on_data=receive
                for status,records in pending:receive(status,records)
                pending.clear()
                progress(dict(event='recording',path=str(path),seconds=seconds,run=run))
                now=time.monotonic();deadline=now+seconds+3;last_progress=last_flush=last_packet=now
                previous_count=counters['records'];sent_stop=False
                while final is None:
                    if stop.is_set() and not sent_stop:
                        ack=client.command(2,run);sent_stop=True;deadline=min(deadline,time.monotonic()+3)
                        if ack['state']==3 and ack['generated']==counters['records']+ack['dropped']:
                            final=ack
                    client.poll();now=time.monotonic()
                    if counters['records']!=previous_count:
                        previous_count=counters['records'];last_packet=now
                    if now-last_packet>.8 and final is None:
                        status=client.command(0)
                        if status['run']!=run:raise ValueError('采集身份改变')
                        if status['state']==3:final=status
                        last_packet=now
                    if now-last_flush>1:file.flush();last_flush=now
                    if now-last_progress>.2:
                        progress(dict(event='progress',records=counters['records'],dropped=(latest or start)['dropped'],
                                      gaps=gaps,crc_errors=client.decoder.rejected))
                        last_progress=now
                    if now>deadline:raise TimeoutError('采集结束未确认, 已保留接收数据')
                gaps+=max(0,final['generated']-last_sequence-1)
                complete=(final['dropped']==0 and gaps==0 and client.decoder.rejected==0 and counters['records']==final['generated'])
            except Exception as exc:
                failure=str(exc)
            finally:
                # 即便文件写入失败也仅停止采集, 不发送任何执行器帧.
                if started and final is None:
                    client.on_data=lambda status,records:None
                    try:client.command(2,run)
                    except (OSError,ValueError,TimeoutError):pass
                file.flush()
        after=None;diagnostics_error=None
        try:after=decode_page(reader.exchange_wire(b'\x00',0x30,0xb0))
        except (OSError,ValueError,TimeoutError) as exc:diagnostics_error=str(exc)
        source_issues=[]
        if after:
            for key in ('queue_dropped','fifo_overrun_can1','fifo_overrun_can2','errors_can1','errors_can2','malformed'):
                if after['summary'][key]!=before['summary'][key]:source_issues.append(key)
        summary=dict(run=run,profile=profile,axis=axis,complete=complete,failure=failure,
                     board_after=after,diagnostics_error=diagnostics_error,source_issues=source_issues,
                     final=final,received=dict(counters),
                     record_gaps=gaps,crc_errors=client.decoder.rejected,
                     first_sample_us=first_source,last_sample_us=last_source)
        path.with_name(path.name+'.summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        progress(dict(event='finished',path=str(path),summary=summary))
        if failure:raise RuntimeError(failure)
        return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=30);parser.add_argument('--profile',choices=['stationary','rotation'],default='rotation')
    parser.add_argument('--axis',default='main');parser.add_argument('--note',default='')
    args=parser.parse_args()
    result=collect(args.port,args.output,args.seconds,args.profile,args.note,args.axis,
                   progress=lambda event:print(json.dumps(event,ensure_ascii=False),flush=True))
    if not result['complete']:raise SystemExit(2)


if __name__=='__main__':main()
