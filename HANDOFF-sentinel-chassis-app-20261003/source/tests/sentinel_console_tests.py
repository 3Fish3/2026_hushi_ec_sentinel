"""正式 APP GUI 数据层契约; 使用原生主机后端, 不访问物理串口."""
from concurrent.futures import Future
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'robots/sentinel/host'))
import sentinel_client as wire
from sentinel_async import OutcomeUnknown
from sentinel_console_model import (MaintenanceSession, LiveHistory, target_packet, page_fresh,
    parameter_label, parameter_scale, physical_parameter)

BACKEND=Path(sys.argv.pop(1)) if len(sys.argv)>1 and not sys.argv[1].startswith('-') else ROOT/'build/sentinel-app-host/Release/ec_sentinel_app_host.exe'


def wait_for(action, predicate, timeout=3):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        value=action()
        if predicate(value):return value
        time.sleep(.015)
    raise AssertionError(f'condition timed out, last={value}')


class ModelTests(unittest.TestCase):
    def test_atomic_target_scope_and_units(self):
        packet=target_packet('gimbal',{'main_yaw':'0.12','pitch':'-.1'},frame='local_inertial')
        self.assertEqual(packet['setpoints'],[('main_yaw','position',.12,0),('pitch','position',-.1,0)])
        self.assertEqual(packet['main_yaw_frame'],'local_inertial')
        self.assertEqual(target_packet('chassis',{},twist=('0.2','0','-.1'))['twist'],(.2,0,-.1))
        for args,kwargs in ((('gimbal',{'trigger':'1'}),{}),(('gimbal',{'pitch':'nan'}),{}),
                            (('gimbal',{}),{'twist':(1,2,3)}),
                            (('chassis',{'chassis_lf':'1'}),{'twist':(0,0,0)}),
                            (('gimbal',{}),{'frame':'guess'})):
            with self.assertRaises(ValueError):target_packet(*args,**kwargs)

    def test_parameter_labels_follow_slot_not_domain_number(self):
        self.assertEqual(parameter_label({'id':0x901}),('MIT Kp','1'))
        self.assertEqual(parameter_label({'id':0xa01}),('惯性对齐偏移','rad'))
        self.assertEqual(parameter_label({'id':0x408}),('位置环 Kp','1/s'))

    def test_current_gain_units_and_missing_scale(self):
        entry=dict(id=0x401,value=.024,min=-100,max=100)
        cap=dict(id=0x404,value=2.5)
        self.assertAlmostEqual(physical_parameter(entry,[entry,cap])['value'],.06)
        self.assertAlmostEqual(physical_parameter(dict(entry,id=0x402,value=.4),[cap])['value'],1.0)
        self.assertIsNone(physical_parameter(entry,[entry]))
        self.assertIsNone(parameter_scale(entry,[dict(cap,value=0)]))
        self.assertEqual(parameter_scale(dict(entry,id=0x408),[]),1)

    def test_idle_joint_plot_uses_angle_not_velocity(self):
        history=LiveHistory()
        entry=dict(domain='main_yaw',mode='none',execute=False,target=0,measurement=-25.65,
            measurement_valid=True,requested_frame=None,adopted_frame='joint',measured_frame='joint')
        page=dict(session=1,sequence=1,reference_epoch=1,received_at_monotonic=10,domains=[entry])
        fast=dict(received_at_monotonic=10,joints={'main_yaw':dict(angle_rad=-.0084,
            rate_rad_s=-25.65,position_valid=True,velocity_valid=True)})
        history.accept(dict(connected=True,status={0:fast,1:page}))
        _,target,measured,error=history.series('main_yaw')
        self.assertAlmostEqual(measured[-1],-.0084)
        self.assertTrue(math.isnan(target[-1]));self.assertTrue(math.isnan(error[-1]))
        self.assertEqual(history.domain('main_yaw',10.1)['measurement_unit'],'rad')
        self.assertAlmostEqual(history.joint('main_yaw',10.1)['angle_rad'],-.0084)
        later=deepcopy(page);later.update(sequence=2,received_at_monotonic=11)
        history.accept(dict(connected=True,status={0:fast,1:later}))
        self.assertTrue(math.isnan(history.series('main_yaw')[2][-1]))
        self.assertIsNone(history.joint('main_yaw',11))

    def test_history_stale_frames_and_reference_break(self):
        history=LiveHistory(capacity=4)
        entry=dict(domain='main_yaw',mode='position',target=.2,measurement=.1,measurement_valid=True,
                   requested_frame='joint',adopted_frame='joint',measured_frame='joint',execute=True)
        page=dict(session=1,sequence=1,reference_epoch=1,received_at_monotonic=10,domains=[entry])
        history.accept(dict(connected=True,status={1:page}))
        history.accept(dict(connected=True,status={1:page}))
        self.assertEqual(len(history.samples),1)
        self.assertIsNotNone(history.domain('main_yaw',10.2))
        self.assertIsNone(history.domain('main_yaw',11))
        self.assertFalse(page_fresh(page,9))
        next_page=deepcopy(page);next_page.update(sequence=2,reference_epoch=2,received_at_monotonic=10.1)
        next_page['domains'][0]['measured_frame']='odom'
        history.accept(dict(connected=True,status={1:next_page}))
        self.assertTrue(math.isnan(history.series('main_yaw')[-1][-1]))
        self.assertEqual(len(history.series('main_yaw')[0]),3)
        history.accept(dict(connected=False,status={1:next_page}))
        self.assertIsNone(history.domain('main_yaw',10.2))


class NativeTests(unittest.TestCase):
    def setUp(self):
        self.sessions=[]

    def tearDown(self):
        for session in self.sessions:session.close()

    def session(self,mechanism='gimbal'):
        session=MaintenanceSession.connect(wire.ProcessTransport(BACKEND,extra_args=['--plant','--fixture',mechanism]),mechanism)
        self.sessions.append(session)
        return session

    def status(self,session):
        return session.peer.status(1).result(timeout=2)

    def test_connect_only_reads_and_parameters_are_scoped(self):
        session=self.session()
        self.assertFalse(session.requested_authority)
        self.assertTrue(session.parameters)
        self.assertTrue(all(e['domain'] in session.domains and not e['whole_vehicle'] for e in session.parameters))
        status=self.status(session)
        self.assertTrue(page_fresh(status,time.monotonic()))
        self.assertFalse(any(e['execute'] for e in status['domains']))
        self.assertIsNone(session.peer.snapshot()['last_target_ack'])
        with self.assertRaises(ValueError):session.submit({'main_yaw':'.1'})
        with self.assertRaises(ValueError):session.write_parameter(0x701,1)

    def test_target_stream_and_hot_parameter_readback(self):
        session=self.session();session.authorize();session.submit({'main_yaw':'.2','fire_yaw':'.1'})
        status=wait_for(lambda:self.status(session),lambda s:all(e['execute'] for e in s['domains'] if e['domain'] in ('main_yaw','fire_yaw')))
        self.assertFalse(any(e['execute'] for e in status['domains'] if e['domain'] not in session.domains))
        parameter=next(e for e in session.parameters if e['domain']=='main_yaw' and (e['id'] & 255)==1)
        result=session.write_parameter(parameter['id'],parameter['value']+.001)
        self.assertEqual(result['ack']['result_name'],'ACCEPTED')
        self.assertEqual(result['readback']['value'],result['ack']['effective'])
        self.assertIsNone(result['unknown'])
        self.assertIsNotNone(session.last_request)
        session.stop()
        wait_for(lambda:self.status(session),lambda s:not any(e['execute'] for e in s['domains']))
        self.assertTrue(session.peer.snapshot()['connected'])
        self.assertTrue(session.requested_authority)
        session.release();self.assertFalse(session.requested_authority)

    def test_physical_gains_roundtrip_and_uninstalled_rejection(self):
        session=self.session()
        cap=session.read_parameter(0x404)['value']
        result=session.write_physical_parameter(0x401,.06)
        self.assertAlmostEqual(result['readback']['value'],.06/cap,places=6)
        result=session.write_physical_parameter(0x402,1.0)
        self.assertAlmostEqual(result['readback']['value'],1.0/cap,places=6)
        self.assertAlmostEqual(physical_parameter(result['readback'],session.parameters)['value'],1.0,places=6)
        session.installed_domains=frozenset({'main_yaw'})
        session.authorize()
        with self.assertRaisesRegex(ValueError,'未装配'):session.submit({'fire_yaw':'.1'})
        self.assertIsNone(session.last_request)

    def test_reference_epoch_stops_old_position_source(self):
        session=self.session();session.authorize();session.submit({'main_yaw':'.2'})
        wait_for(lambda:self.status(session),lambda s:s['domains'][4]['execute'])
        zero=next(e for e in session.parameters if e['domain']=='fire_yaw' and (e['id'] & 255)==20)
        result=session.write_parameter(zero['id'],zero['value']+.01)
        self.assertIsNone(result['unknown'])
        wait_for(lambda:session.note,lambda note:'参考代次改变' in note)
        wait_for(lambda:self.status(session),lambda s:not any(e['execute'] for e in s['domains']))

    def test_profile_load_is_draft_only_and_rejects_fixture_on_real(self):
        session=self.session()
        parameter=next(e for e in session.parameters if e['writable'] and e['kind']=='hot')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'profile.json';session.save_readback(path)
            profile=json.loads(path.read_text(encoding='utf-8'))
            next(e for e in profile['params'] if e['id']==parameter['id'])['value']=parameter['value']+.003
            path.write_text(json.dumps(profile),encoding='utf-8')
            drafts=session.load_drafts(path)
            self.assertAlmostEqual(drafts[parameter['id']],parameter['value']+.003)
            self.assertEqual(session.read_parameter(parameter['id'])['value'],parameter['value'])
            self.assertIsNone(session.peer.snapshot()['last_change'])
            actual=session.peer.snapshot()
            with patch.object(session.peer,'snapshot',return_value=dict(actual,config_kind=2)):
                with self.assertRaises(wire.ProtocolError):session.load_drafts(path)
            profile['params'].append(dict(id=0x701,value=1))
            path.write_text(json.dumps(profile),encoding='utf-8')
            with self.assertRaises(ValueError):session.load_drafts(path)

    def test_unknown_transaction_reads_back_without_reissuing(self):
        session=self.session()
        parameter=next(e for e in session.parameters if e['writable'] and e['kind']=='hot')
        future=Future();future.set_exception(OutcomeUnknown(wire.PARAM_WRITE,1,1,'test lost ACK'))
        with patch.object(session.peer,'param_write',return_value=future) as write:
            result=session.write_parameter(parameter['id'],parameter['value']+.001)
            self.assertEqual(write.call_count,1)
        self.assertIsNotNone(result['unknown'])
        self.assertIsNone(result['ack'])
        self.assertEqual(result['readback']['value'],parameter['value'])

    def test_four_motor_bench_workflow(self):
        session=self.session('bench')
        self.assertEqual(session.installed_domains,{'main_yaw','pitch','friction_left','trigger'})
        session.authorize();session.align('main_yaw',0)
        session.submit({'main_yaw':'.05','pitch':'.02','friction_left':'.2','trigger':'.2'})
        status=wait_for(lambda:self.status(session),lambda s:all(e['execute'] for e in s['domains'] if e['domain'] in session.domains))
        self.assertFalse(status['launch_allowed'])
        result=session.write_physical_parameter(0x601,2.0)
        self.assertAlmostEqual(physical_parameter(result['readback'],session.parameters)['value'],2.0,places=5)
        session.stop()
        wait_for(lambda:self.status(session),lambda s:not any(e['execute'] for e in s['domains']))
        session.submit({'main_yaw':'.05','friction_left':'.2','trigger':'.2'})
        wait_for(lambda:self.status(session),lambda s:s['domains'][4]['execute'])
        session.write_parameter(0x914,.01)
        wait_for(lambda:session.note,lambda note:'参考代次改变' in note)
        wait_for(lambda:self.status(session),lambda s:not any(e['execute'] for e in s['domains']))

    def test_chassis_and_shooter_are_independent(self):
        chassis=self.session('chassis');chassis.authorize();chassis.submit({},twist=(.1,0,0))
        status=wait_for(lambda:self.status(chassis),lambda s:all(e['execute'] for e in s['domains'][:4]))
        self.assertFalse(any(e['execute'] for e in status['domains'][4:]))
        shooter=self.session('shooter');shooter.authorize()
        shooter.submit({'friction_left':'2','friction_right':'-2'})
        status=wait_for(lambda:self.status(shooter),lambda s:all(e['execute'] for e in s['domains'][7:9]))
        self.assertFalse(status['domains'][9]['execute'])
        self.assertFalse(any(e['execute'] for e in status['domains'][:7]))


if __name__=='__main__':unittest.main()
