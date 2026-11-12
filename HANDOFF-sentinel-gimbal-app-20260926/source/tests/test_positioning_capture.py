"""Capture owner tests use futures only, never serial or real motion."""
from concurrent.futures import Future
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import patch
import tempfile
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robots/sentinel/host'))
from sentinel_positioning_capture import PositioningCaptureOwner

class Peer:
    def __init__(self): self.session=12;self.calls=[]
    def snapshot(self): return {'session':self.session}
    def raw_request(self,b,*,expected_session=None):
        if expected_session != self.session:raise ValueError("session changed atomically")
        self.calls.append(b);return Future()

class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.peer=Peer();self.owner=PositioningCaptureOwner(self.peer)
        self.settings={'run':7,'scheduling_margin_us':20000,'output_path':self.temp.name+'/data.sraw'}
        self.identity=dict(session=12,boot_id=2,firmware_sha256='ab'*32,deployment_sha256='cd'*32,
            config_epoch=3,reference_epoch=4,param_epoch=5,config_hash=6,guardian_version=7,
            experiment_session=8,nonce=9,axis='fire_yaw')
    def test_load_does_not_capture(self):
        self.owner.configure(self.settings);self.assertFalse(self.peer.calls)
        with self.assertRaises(ValueError):self.owner.finish()
    def test_explicit_settings_and_output_exclusivity(self):
        for k,v in [('run',0),('run',True),('scheduling_margin_us',0),('output_path',self.temp.name+'/data.csv')]:
            with self.assertRaises(ValueError):self.owner.configure({**self.settings,k:v})
        Path(self.settings['output_path']).touch()
        with self.assertRaises(ValueError):self.owner.configure(self.settings)
    def test_session_bound_one_attempt_and_result(self):
        self.owner.configure(self.settings)
        @dataclass
        class Receipt:ready:bool=True
        with patch('sentinel_positioning_capture.ShortCaptureAdapter') as factory:
            pending=Future();factory.return_value.prepare_start.return_value=pending
            f=self.owner.start({'axis':'fire_yaw','duration_us':120000},self.identity)
            self.assertFalse(f.done());factory.return_value.prepare_start.assert_called_once_with(5,7,120000)
            exchange=factory.call_args.args[0]
            self.peer.session=13
            with self.assertRaises(ValueError):exchange(b'test')
            self.assertFalse(self.peer.calls)
            pending.set_result(Receipt());self.assertTrue(f.result()['ready'])
            with self.assertRaises(ValueError):self.owner.start({'axis':'fire_yaw','duration_us':120000},self.identity)
            self.owner.finish();factory.return_value.finish.assert_called_once_with(self.settings['output_path'])
            self.owner.close();factory.return_value.close.assert_called_once_with(wait=False)
    def test_missing_identity_no_attempt(self):
        self.owner.configure(self.settings)
        with patch('sentinel_positioning_capture.ShortCaptureAdapter') as factory:
            for key in self.identity:
                identity=dict(self.identity);identity.pop(key)
                with self.assertRaises(ValueError):self.owner.start({'axis':'fire_yaw','duration_us':120000},identity)
            factory.assert_not_called()

if __name__=='__main__':unittest.main()
