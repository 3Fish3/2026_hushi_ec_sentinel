"""连续采集的原生字节互操作及假串口; 不连接真实板卡."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robots/sentinel/host'))
import sentinel_stream as stream
import sentinel_raw as raw
from board_monitor import Decoder,encode_frame
codec=sys.argv.pop(1)


class FakePort:
    def __init__(self,golden,corrupt=False,wait_stop=False,build=stream.BUILD):
        self.golden=golden;self.corrupt=corrupt;self.wait_stop=wait_stop;self.build=build
        self.buffer=bytearray();self.writes=[];self.run=0;self.state=0;self.generated=0
    def __enter__(self):return self
    def __exit__(self,*args):pass
    @property
    def in_waiting(self):return len(self.buffer)
    def read(self,n):
        if not self.buffer:time.sleep(.001)
        result=bytes(self.buffer[:min(n,73)]);del self.buffer[:len(result)];return result
    def status(self,op):
        return stream.HEADER.pack(1,op,0,self.state,self.run,self.generated,0,0,0,1000,raw.LAYOUT)
    def write(self,data):
        kind,seq,payload=Decoder().feed(data)[0];self.writes.append((kind,payload))
        if kind==1:
            body=bytearray(72);body[40:40+len(self.build)]=self.build.encode()
            self.buffer.extend(encode_frame(0x81,seq,body))
        elif kind==0x30:
            body=bytearray(192);struct.pack_into('<4BI',body,0,3,0,1,0,1000)
            struct.pack_into('<II',body,8,0x43415031,168000000)
            body[104:104+len(self.build)]=self.build.encode()
            self.buffer.extend(encode_frame(0xb0,seq,body))
        elif kind==0x32:
            v,op,profile,domain,run,duration=struct.unpack('<4BII',payload)
            if op==1:
                self.run=run;self.state=1
            elif op==2:self.state=3
            self.buffer.extend(encode_frame(0xb2,seq,self.status(op)))
            if op==1:
                status,records=stream.decode_packet(self.golden)
                compact=bytearray()
                for record in records:
                    record=bytearray(record);struct.pack_into('<I',record,28,run)
                    compact.extend(record[:48 if record[1]==1 else 44])
                self.generated=len(records);self.state=1 if self.wait_stop else 3
                wire=bytearray(encode_frame(0xb3,7,self.status(3)+compact))
                if self.corrupt:wire[10]^=1
                self.buffer.extend(wire)
        else:raise AssertionError('unexpected command')
        return len(data)


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.golden=bytes.fromhex(subprocess.check_output([codec,'--golden'],text=True).strip())
    def test_native_compact_restore(self):
        status,records=stream.decode_packet(self.golden)
        self.assertEqual(status['generated'],3)
        self.assertEqual([raw.decode_record(r)['kind'] for r in records],['gyro','accel','can'])
        self.assertTrue(all(len(r)==64 for r in records))
        with self.assertRaises(ValueError):stream.decode_packet(self.golden[:-1])
        wrong=bytearray(self.golden);wrong[33]=5
        with self.assertRaises(ValueError):stream.decode_packet(wrong)
    def run_capture(self,corrupt=False,wait_stop=False,build=stream.BUILD):
        fake=FakePort(self.golden,corrupt,wait_stop,build)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'test.sraw'
            event=threading.Event()
            if wait_stop:event.set()
            with patch('serial.Serial',return_value=fake):
                result=stream.collect('FAKE',path,1,stop=event)
            meta,rows=raw.load(path)
            summary=json.loads(Path(str(path)+'.summary.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['run'],result['run']);self.assertEqual(summary,result)
            self.assertEqual(meta['device_identity']['build_id'],build)
            self.assertEqual(meta['harness'],'10310221' if build.startswith('capture-current-') else 'legacy')
        self.assertTrue(all(kind in (1,0x30,0x32) for kind,_ in fake.writes))
        return result,rows,fake
    def test_file_roundtrip(self):
        result,rows,_=self.run_capture()
        self.assertTrue(result['complete']);self.assertEqual(len(rows),3)
        self.assertEqual(result['record_gaps'],0)
    def test_current_harness_identity(self):
        result,rows,_=self.run_capture(build='capture-current-20261031-032021')
        self.assertTrue(result['complete']);self.assertEqual(len(rows),3)
        fake=FakePort(self.golden,build='app-integration-20261031-032014')
        with tempfile.TemporaryDirectory() as folder,patch('serial.Serial',return_value=fake):
            with self.assertRaises(ValueError):stream.collect('FAKE',Path(folder)/'bad.sraw',1)
        self.assertTrue(all(kind==1 for kind,_ in fake.writes))

    def test_corrupt_frame_not_silent_success(self):
        result,rows,_=self.run_capture(corrupt=True)
        self.assertFalse(result['complete']);self.assertEqual(len(rows),0)
        self.assertEqual(result['record_gaps'],3);self.assertEqual(result['crc_errors'],1)
    def test_stop_only_stops_recorder(self):
        result,rows,fake=self.run_capture(wait_stop=True)
        self.assertTrue(result['complete'])
        self.assertTrue(any(kind==0x32 and payload[1]==2 for kind,payload in fake.writes))


if __name__=='__main__':unittest.main()
