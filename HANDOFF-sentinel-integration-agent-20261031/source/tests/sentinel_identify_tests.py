"""辨识的协议交叉验证、编排、合成频响恢复与拒绝路径; 不访问串口或硬件."""
import csv
from concurrent.futures import Future
import json
import queue
import threading
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'robots/sentinel/host'))
import sentinel_client as wire
import sentinel_identify as capture
import sentinel_identify_analyze as analysis
import sentinel_velocity_validate as validation
import sentinel_velocity_validate_analyze as validation_analysis
NATIVE=Path(sys.argv.pop(1));INTEGRATION=Path(sys.argv.pop(1))


def synthetic(folder,seed=1,sign=1,gain=1):
    folder.mkdir(parents=True)
    p=dict(capture.DEFAULT_PLAN,speed=sign*.8)
    offset=p['ramp_s']+2.;excitation_start=offset
    t=np.arange(0,capture.duration(p),.001);r=np.full(len(t),p['speed']);y=r.copy();u=r*.3/8;excitation=np.zeros(len(t))
    stage=np.zeros(len(t),dtype=int);flags=np.full(len(t),7,dtype=int)
    noise=np.random.default_rng(seed).normal(0,.0001,len(t))
    for i,f in enumerate(p['frequencies']):
        lo=offset+1/f;hi=offset+(p['cycles']+1)/f;take=(t>=lo-1e-7)&(t<hi-1e-7);s=2j*np.pi*f
        G=analysis.transfer([8,.3,.006],[f])[0];C=.18+.005/s;D=-1j*p['amplitude_a']
        U=D/(1+C*G);Y=G*U
        phase=np.exp(2j*np.pi*f*(t[take]-offset))
        excitation[take]=np.real(D*phase);y[take]+=gain*np.real(Y*phase)+noise[take];u[take]+=np.real(U*phase)-.18*noise[take]
        stage[take]=2+i;flags[take]|=16;offset+=(p['cycles']+2)/f
    with (folder/'samples.csv').open('w',newline='') as file:
        w=csv.writer(file);w.writerow(['sequence','time_s','at_us','submit_us','reference','feedback','command_a','stage','flags','excitation_a'])
        w.writerows(zip(range(len(t)),t,np.rint(t*1e6),np.rint(t*1e6),r,y,u,stage,flags,excitation))
    meta=dict(complete=True,schema=2,plan=p,start_us=0,finish=dict(state=3,excitation_start_us=int(excitation_start*1e6)),parameters_before=[dict(id=0x401,value=.06)],parameters_after=[dict(id=0x401,value=.06)])
    (folder/'summary.json').write_text(json.dumps(meta));(folder.parent/'summary.json').write_text(json.dumps(dict(complete=True)))


class IdentifyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.root=Path(cls.tmp.name)
        output=cls.root/'native.wire'
        subprocess.run([str(INTEGRATION),'--wire',str(output)],check=True,capture_output=True)
        decoder=wire.FrameDecoder();cls.packets=[];cls.statuses=[];cls.frames=[]
        for byte in output.read_bytes():
            frame=decoder.push(byte)
            if frame: cls.frames.append(frame)
            if frame and frame[0]==wire.IDENTIFICATION_STREAM: cls.packets.append(wire.decode_identification_stream(frame[2]))
            if frame and frame[0]==wire.IDENTIFICATION_STATUS: cls.statuses.append(wire.decode_identification_status(frame[2]))
        assert decoder.rejected==0 and cls.packets and cls.statuses
        output=cls.root/'validation.wire'
        subprocess.run([str(INTEGRATION),'--validation-wire',str(output)],check=True,capture_output=True)
        decoder=wire.FrameDecoder();cls.validation_packets=[];cls.validation_status=[]
        for byte in output.read_bytes():
            frame=decoder.push(byte)
            if frame and frame[0]==wire.IDENTIFICATION_STREAM: cls.validation_packets.append(wire.decode_identification_stream(frame[2]))
            if frame and frame[0]==wire.IDENTIFICATION_STATUS: cls.validation_status.append(wire.decode_identification_status(frame[2]))
        assert decoder.rejected==0 and sum(len(p['samples']) for p in cls.validation_packets)==15001

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def test_native_cross_language_layout_and_continuity(self):
        points=[p for packet in self.packets for p in packet['samples']]
        self.assertEqual([p['sequence'] for p in points],list(range(len(points))))
        self.assertGreater(len(points),1200)
        self.assertTrue(all(p['flags']&7==7 for p in points))
        self.assertTrue(any(p['measuring'] for p in points))
        self.assertTrue(any(s['state']==3 for s in self.statuses))
        self.assertEqual(len(wire.encode_identification(1,2,1,capture.DEFAULT_PLAN)),66)
        self.assertTrue(all(p['schema']==2 for p in self.packets))
        self.assertTrue(any(abs(p['excitation_a'])>.001 for p in points))
        legacy=next(f[2][:40] for f in self.frames if f[0]==wire.IDENTIFICATION_STATUS)
        self.assertEqual(wire.decode_identification_status(legacy)['schema'],1)
        for bad in (b'',bytes(23),bytes(240)):
            with self.assertRaises(wire.ProtocolError): wire.decode_identification_stream(bad)

    def test_actual_async_owner_receives_native_high_rate_stream(self):
        from sentinel_async import AsyncSentinelClient
        frames=self.frames
        hello=next(f[2] for f in frames if f[0]==wire.HELLO_ACK)
        status=next(f[2] for f in frames if f[0]==wire.IDENTIFICATION_STATUS)
        stream=[f[2] for f in frames if f[0]==wire.IDENTIFICATION_STREAM]
        class Backend(wire._FrameTransport):
            def __init__(self):
                super().__init__();self.q=queue.Queue(maxsize=4096);self.requests=wire.FrameDecoder();self.producers=[];self.send_lock=threading.Lock()
            def put(self,kind,seq,body):
                frame=wire.encode_frame(kind,seq,body)
                with self.send_lock:
                    for start in range(0,len(frame),97): self.q.put_nowait(frame[start:start+97])
            def send(self,message):
                if self.observer: self.observer('tx_attempt',message)
                for b in message:
                    request=self.requests.push(b)
                    if request is None: continue
                    kind,seq,body=request
                    if kind==wire.HELLO:
                        reply=bytearray(hello);reply[22:26]=body[2:6];self.put(wire.HELLO_ACK,seq,reply)
                    elif kind==wire.IDENTIFICATION:
                        # 正常应答优先, 不能用预塞1.3秒数据再应答的假后端制造超时.
                        for packet in stream[:2]: self.put(wire.IDENTIFICATION_STREAM,0,packet)
                        reply=bytearray(status);reply[2:10]=body[2:10];self.put(wire.IDENTIFICATION_STATUS,seq,reply)
                        # 发送请求不能同步生成数秒响应再返回, 否则模拟器CPU耗时被误计为应答超时.
                        def produce():
                            for packet in stream[2:]: self.put(wire.IDENTIFICATION_STREAM,0,packet)
                        task=threading.Thread(target=produce);self.producers.append(task);task.start()
                    else: self.put(wire.ACK,seq,bytes([wire.PROTOCOL_VERSION,wire.ACK])+body[2:10]+bytes([15,0]))
            def _read_chunk(self,timeout_s):
                try: return self.q.get(timeout=timeout_s)
                except queue.Empty: return None
            def close(self):
                for task in self.producers:task.join(1)
        observed=[];done=threading.Event()
        def receive(packet):
            observed.append(packet)
            if len(observed)==len(stream): done.set()
        peer=AsyncSentinelClient(Backend(),fast_hz=1,identification_sink=receive)
        try:
            peer.hello().result(2);result=peer.identification(2).result(3)
            self.assertIn('produced',result)
            self.assertTrue(done.wait(6))
            self.assertEqual(sum(len(p['samples']) for p in observed),sum(len(p['samples']) for p in self.packets))
            self.assertEqual(peer.snapshot()['error'],'')
        finally: peer.close()

    def test_plan_and_no_parameter_writes(self):
        capture.validate_plan(capture.DEFAULT_PLAN)
        with self.assertRaises(ValueError): capture.validate_plan(dict(capture.DEFAULT_PLAN,amplitude_a=3))
        with self.assertRaises(ValueError): capture.validate_plan(dict(capture.DEFAULT_PLAN,ramp_s=.001))
        self.assertAlmostEqual(capture.duration(capture.DEFAULT_PLAN),74.5)
        self._run_capture(False)

    def test_loss_retains_partial_and_does_not_restart(self): self._run_capture(True)

    def test_unconfirmed_release_is_not_reported_complete(self): self._run_capture(False,True)

    def test_legacy_firmware_rejected_before_authority(self): self._run_capture(False,legacy=True)

    def test_working_point_timeout_returns_partial_without_second_direction(self): self._run_capture(False,aborted=True)

    def _run_capture(self,gap,release_error=False,legacy=False,aborted=False):
        from copy import deepcopy
        # 编排假后端同步投递短样本; 完整多秒流由上方原生/实际异步接收测试覆盖.
        calls=[];packets=deepcopy(self.packets[:128]);sample_count=sum(len(p['samples']) for p in packets)
        def ready(x): f=Future();f.set_result(x);return f
        class Client:
            def __init__(self,transport,**kwargs):
                self.sink=kwargs['identification_sink'];self.run_id=0;self.source=None;self.executing=False;self.count=0
                kwargs['log_path'].write_text('')
            def hello(self): return ready(dict(session=1))
            def read_all_params(self): return ready([])
            def authorize(self,enabled,role):
                calls.append(('authorize',enabled))
                if release_error and not enabled: raise TimeoutError('release unknown')
                return ready(dict(result=0))
            def diagnostics(self,page): return ready({})
            def identification(self,action=2,plan=None):
                calls.append(('identification',action))
                if action==1:
                    self.run_id+=1;self.count=sample_count
                    for index,p in enumerate(deepcopy(packets)):
                        p['run_id']=self.run_id
                        if gap and index==2: p['first_sequence']+=1
                        self.sink(p)
                elif action==3: self.count=0
                return ready(dict(schema=1 if legacy else 2,state=(4 if aborted else 3) if self.count else 0,reason=6 if aborted else 0,
                    run_id=self.run_id,produced=self.count,raw_to_axis_amp=3/16384,excitation_start_us=0))
            def set_target_source(self,source):
                values=source(0,dict(config_epoch=1,reference_epoch=1)) if source else {}
                wire.encode_targets(1,1,**dict(config_epoch=1,reference_epoch=1,setpoints=[],**{})|values)
                if source:
                    self.executing=any(p[0]=='main_yaw' for p in values.get('setpoints',[]))
                    self.assert_holds=all(any(p[0]==d for p in values.get('setpoints',[])) for d in ('fire_yaw','pitch'))
                    if not self.assert_holds: raise AssertionError('保持下游轴不能在正反段之间撤掉')
                else: self.executing=False
            def status(self,page):
                if page==0: return ready(dict(config_epoch=1,reference_epoch=1,source='maintenance',authority=True,
                    main_platform=dict(validity='valid',heading_rad=.3),joints={d:dict(position_valid=True,angle_rad=0) for d in ('main_yaw','fire_yaw','pitch')}))
                return ready(dict(domains=[dict(execute=self.executing if i==4 else True) for i in range(10)]))
            def snapshot(self): return dict(error='',statistics={},log_dropped=0,log_error='')
            def close(self): pass
        class Port: pass
        with tempfile.TemporaryDirectory() as folder,patch.object(capture,'AsyncSentinelClient',Client):
            output=Path(folder)/'run'
            if gap or legacy or aborted:
                with self.assertRaises(wire.ProtocolError): capture.run(Port(),output,capture.DEFAULT_PLAN,progress=lambda _:None)
                if not legacy:
                    s=json.loads((output/'positive/summary.json').read_text());self.assertFalse(s['complete'])
                    if aborted:
                        self.assertEqual(s['samples'],sample_count);self.assertIn('工作点',s['error'])
                self.assertFalse((output/'negative').exists())
            else:
                report=capture.run(Port(),output,capture.DEFAULT_PLAN,progress=lambda _:None)
                self.assertEqual(report['complete'],not release_error);self.assertEqual(report['trials'],['positive','negative'])
                if release_error:
                    self.assertIn('release unknown',report['cleanup_error'])
                    self.assertTrue(report['recording_complete'])
                self.assertEqual(json.loads((output/'negative/summary.json').read_text())['samples'],sample_count)
        self.assertEqual(calls.count(('identification',1)),0 if legacy else 1 if gap or aborted else 2)
        if legacy: self.assertFalse(any(x[0]=='authorize' for x in calls))
        else: self.assertEqual(calls[-1],('authorize',False))

    def test_validation_native_profile_metadata_and_zero_bias(self):
        packets=self.validation_packets;points=[s for p in packets for s in p['samples']]
        self.assertEqual([s['sequence'] for s in points],list(range(15001)))
        t=np.array([((s['at_us']-packets[0]['start_us'])&0xffffffff)*1e-6 for s in points])
        self.assertLess(max(abs(np.array([s['reference'] for s in points])-validation_analysis.reference(t))),2e-5)
        self.assertTrue(all(s['bias_a']==0 and s['excitation_a']==0 for s in points))
        self.assertEqual(points[-1]['command_raw'],0);self.assertTrue(points[-1]['flags']&32)
        info=next(s['validation'] for s in reversed(self.validation_status) if s.get('validation'))
        self.assertEqual(info['profile'],1);self.assertEqual(info['inner_period_us'],1000);self.assertEqual(info['outer_period_us'],2000)
        self.assertAlmostEqual(info['velocity_pid'][0],.4,places=6)
        self.assertAlmostEqual(info['outer_integral_limit'],15)
        self.assertEqual(len(wire.encode_identification(1,2,5)),11)

    def test_validation_lifecycle_restore_and_no_midpoint_reset(self): self._validation_run()
    def test_validation_old_capability_changes_nothing(self): self._validation_run('legacy')
    def test_validation_unknown_parameter_write_is_rolled_back(self): self._validation_run('write_unknown')
    def test_validation_restore_unknown_still_releases(self): self._validation_run('restore_unknown')
    def test_validation_early_stop_retains_trigger(self): self._validation_run('current_stop')
    def test_validation_gap_restores_without_retry(self): self._validation_run('gap')

    def _validation_run(self,fault=''):
        from copy import deepcopy
        original={0x401:.06,0x402:.005/3,0x403:0.,0x408:42.,0x409:3.,0x40a:0.}
        actual=dict(original);calls=[];threads=[];started=[False];writes=[0]
        packets=deepcopy(self.validation_packets if not fault else self.validation_packets[:32])
        if fault=='current_stop':
            last=packets[-1]['samples'][-1];last['p_a']=.36;last['i_a']=0;last['command_raw']=0;last['flags']|=32
        count=sum(len(p['samples']) for p in packets)
        def ready(value): f=Future();f.set_result(value);return f
        def fail(error): f=Future();f.set_exception(error);return f
        class Client:
            def __init__(self,transport,**kw):
                self.sink=kw['identification_sink'];self.executing=False;self.state=0;self.release=False
                kw['log_path'].write_text('')
            def hello(self): return ready(dict(session=1))
            def identification(self,action=2,plan=None):
                calls.append(('id',action))
                if action==5:
                    if not validation.same(actual,validation.CANDIDATE): raise AssertionError('候选未成套生效')
                    started[0]=True;self.state=4 if fault=='current_stop' else 3
                    def send():
                        for i,p in enumerate(packets):
                            if fault=='gap' and i==3:p['first_sequence']+=1
                            self.sink.__self__.put(p,timeout=5)
                    thread=threading.Thread(target=send);threads.append(thread);thread.start()
                if action==3:self.state=0
                if action==0 and self.state in (1,2):self.state=4
                result=dict(schema=2,state=self.state,reason=7 if fault=='current_stop' else 0,run_id=packets[0]['run_id'],
                    produced=count if started[0] else 0,raw_to_axis_amp=3/16384)
                if action==4 and fault!='legacy':result['validation']=dict(version=1,profile=int(started[0]),current_limit_a=3.,feedback_alpha=0.)
                return ready(result)
            def read_all_params(self):return ready([dict(id=k,value=v) for k,v in actual.items()])
            def param_write(self,k,v):
                calls.append(('write',k,v));writes[0]+=1
                if self.executing:raise AssertionError('禁止执行中写半套或恢复')
                actual[k]=v
                if (fault=='write_unknown' and writes[0]==2) or (fault=='restore_unknown' and started[0]):return fail(TimeoutError('write outcome unknown'))
                return ready(dict(result=0))
            def authorize(self,enabled,role):calls.append(('authorize',enabled));return ready(dict(result=0))
            def set_target_source(self,source):
                data=source(0,dict(config_epoch=1,reference_epoch=1)) if source else {}
                self.executing=any(p[0]=='main_yaw' for p in data.get('setpoints',[]))
                if self.executing and not validation.same(actual,validation.CANDIDATE):raise AssertionError('新内环不能带旧外环启动')
                if source:
                    assert all(any(p[0]==axis for p in data['setpoints']) for axis in ('fire_yaw','pitch'))
            def status(self,page):
                if page==0:return ready(dict(config_epoch=1,reference_epoch=1,source='maintenance',authority=True,
                    main_platform=dict(validity='valid',heading_rad=.3),joints={d:dict(position_valid=True,angle_rad=0) for d in ('main_yaw','fire_yaw','pitch')}))
                return ready(dict(domains=[dict(execute=self.executing if i==4 else True) for i in range(10)]))
            def diagnostics(self,page):return ready({})
            def snapshot(self):return dict(error='',statistics={},log_dropped=0,log_error='')
            def close(self):
                for thread in threads:thread.join(2)
        with tempfile.TemporaryDirectory() as folder,patch.object(validation,'AsyncSentinelClient',Client):
            root=Path(folder)/'run';report=validation.run(object(),root,progress=lambda _:None)
            if fault=='legacy':
                self.assertFalse(any(c[0] in ('authorize','write') for c in calls));self.assertFalse(started[0])
                self.assertIn('固件',validation_analysis.analyze(root)['reasons'][0])
            else:
                self.assertTrue(report['release_confirmed']);self.assertEqual(calls[-1],('authorize',False))
                if fault=='restore_unknown':self.assertFalse(report['restoration_confirmed']);self.assertFalse(report['complete'])
                else:self.assertTrue(validation.same(actual,original));self.assertTrue(report['restoration_confirmed'])
            self.assertEqual(calls.count(('id',5)),0 if fault in ('legacy','write_unknown') else 1)
            self.assertEqual(calls.count(('id',3)),0 if fault=='legacy' else 1)
            if not fault:
                self.assertTrue(report['complete']);self.assertEqual(report['end_reason'],'complete')
                result=validation_analysis.analyze(root);self.assertNotEqual(result['verdict'],'incomplete',result)
                self.assertEqual(len(result['plateaus']),2)
            elif fault=='current_stop':
                self.assertTrue(report['recording_complete']);self.assertEqual(report['end_reason'],'current_stop')
                result=validation_analysis.analyze(root);self.assertEqual(result['verdict'],'B');self.assertGreater(result['peak_requested_a'],.35)
            else:self.assertFalse(report['complete'])

    def test_validation_numeric_A_and_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);t=np.arange(15001)*.001;speed=validation_analysis.reference(t)
            q=np.r_[0,np.cumsum((speed[1:]+speed[:-1])*.0005)]
            def write_data(v,angle):
                with (root/'samples.csv').open('w',newline='') as f:
                    w=csv.writer(f);w.writerow(['sequence','time_s','reference','feedback','joint','p_a','i_a','command_a','command_raw','bias_a','excitation_a','flags'])
                    flags=np.full(len(t),7);flags[-1]|=32
                    w.writerows(zip(range(len(t)),t,speed,v,angle,np.zeros(len(t)),np.zeros(len(t)),np.zeros(len(t)),np.zeros(len(t)),np.zeros(len(t)),np.zeros(len(t)),flags))
            (root/'trial.json').write_text(json.dumps(dict(recording_complete=True,motion_complete=True,end_reason='complete',finish=dict(produced=len(t)))))
            (root/'summary.json').write_text(json.dumps(dict(complete=True,restoration_confirmed=True,release_confirmed=True)))
            write_data(speed,q);self.assertEqual(validation_analysis.analyze(root)['verdict'],'A_numeric')
            drift=q.copy();drift[t>=12]+=.005*(t[t>=12]-12);write_data(speed,drift)
            result=validation_analysis.analyze(root);self.assertEqual(result['verdict'],'B');self.assertTrue(any('encoder' in r for r in result['reasons']))
            write_data(speed,q*1.5);result=validation_analysis.analyze(root)
            self.assertEqual(result['verdict'],'B');self.assertTrue(any('编码器与采用速度' in r for r in result['reasons']))

    def test_reference_instrumented_model_and_native_candidate(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);synthetic(root/'positive');synthetic(root/'negative',seed=2,sign=-1)
            (root/'summary.json').write_text(json.dumps(dict(complete=False,recording_complete=True,cleanup_error='release unknown')))
            result=analysis.analyze(root/'positive',root/'negative',NATIVE)
            self.assertEqual(result['status'],'native_candidate',result)
            self.assertIn('excitation-current',result['method'])
            for row in result['training']:
                s=2j*np.pi*row['hz'];P=analysis.transfer([8,.3,.006],[row['hz']])[0];C=.18+.005/s
                T=C*P/(1+C*P)
                self.assertAlmostEqual(row['closed_loop_magnitude'],abs(T),delta=.02)
            self.assertAlmostEqual(result['model']['a'],8,delta=.15)
            self.assertAlmostEqual(result['model']['delay_s'],.006,delta=.002)
            self.assertLess(result['model']['validation_relative_error'],.03)
            self.assertFalse(result['deployment_ready'])
            self.assertEqual(result['cleanup_error'],'release unknown')

    def test_independent_bad_validation_produces_no_pid(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);synthetic(root/'positive');synthetic(root/'negative',sign=-1,gain=1.7)
            result=analysis.analyze(root/'positive',root/'negative',NATIVE)
            self.assertEqual(result['status'],'frequency_response_only');self.assertIsNone(result['candidate'])
            self.assertIn('不一致',result['reason'])


if __name__=='__main__': unittest.main()
