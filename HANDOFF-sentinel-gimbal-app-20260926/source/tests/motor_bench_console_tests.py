"""控制台数据契约; 不创建图形上下文、不打开硬件串口."""
from pathlib import Path
import json
import math
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'boards/sentinel_f407/tools'))
import motor_bench as bench
import motor_bench_trace as trace
from motor_bench_console_model import LiveModel, Recording, FIELDS, status_values, same_settings, save_profile, load_profile, velocity_shortcut, position_shortcut
codec=sys.argv.pop(1)


class ConsoleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.status=bench.decode_status(bytes.fromhex(subprocess.check_output([codec,'--vector'],text=True).strip()))
        cls.vectors=dict((key,bytes.fromhex(value)) for key,value in
                        (line.split() for line in subprocess.check_output([codec,'--trace-vectors'],text=True).splitlines()))

    def test_profile_no_implicit_parameter_change(self):
        original=self.status.copy();values=status_values(original)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'candidate.json'
            save_profile(path,original['mode'],values,'人工候选')
            data=load_profile(path)
            self.assertEqual(data['values'],values)
            self.assertFalse(data['validated'])
            with self.assertRaises(FileExistsError):save_profile(path,original['mode'],values,'')
            path.write_text(json.dumps(dict(data,schema='vehicle')),encoding='utf-8')
            with self.assertRaises(ValueError):load_profile(path)
        self.assertEqual(original,self.status)

    def test_float_readback_and_target_final(self):
        values=status_values(self.status)
        self.assertTrue(same_settings(self.status['mode'],values,self.status))
        values[4]+=0.1
        self.assertFalse(same_settings(self.status['mode'],values,self.status))
        self.assertFalse(same_settings(3,status_values(self.status),self.status))
        d=dict(self.status,target=0.01,target_final=.1)
        values=status_values(d);values[0]=.1
        self.assertTrue(same_settings(d['mode'],values,d))

    def test_stale_and_discontinuous_data(self):
        model=LiveModel();model.accept(self.status,10)
        self.assertTrue(model.fresh(10.2));self.assertFalse(model.fresh(10.5))
        model.accept(dict(self.status,feedback_valid=False,position_valid=False),10.6)
        x,y=model.series('speed_rad_s',feedback=True)
        self.assertEqual(len(x),3)
        self.assertTrue(math.isnan(y[1]));self.assertTrue(math.isnan(y[2]))
        self.assertEqual(model.series('output_a')[1][-1],self.status['output_a'])
        model.accept(dict(self.status,estimated_velocity_valid=0),10.7)
        self.assertTrue(math.isnan(model.series('estimated_velocity_rad_s',feedback=True)[1][-1]))

    def test_mode_separation_and_bounded_history(self):
        model=LiveModel()
        for n in range(1000):model.accept(dict(self.status,mode=1),n*.1)
        self.assertLessEqual(len(model.samples),600)
        self.assertTrue(all(math.isnan(v) for v in model.series('target',mode=2)[1]))

    def test_recording_native_layout_and_selection(self):
        header,body=trace.decode_reply(self.vectors['data0'])
        meta=dict(start=self.status,layout=trace.LAYOUT)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'sample.btrace'
            trace.save(path,meta,body)
            record=Recording(path)
            self.assertEqual(len(record.rows),header['count'])
            end=record.rows[-1]['time_s']
            self.assertEqual(record.summary(0,end-1e-8)['samples'],len(record.rows))
            self.assertEqual(record.summary(10,20)['samples'],0)
            self.assertEqual(len(record.series('command_a')[0]),len(record.rows))
            # 位置无参考时不能把原始占位零当作机械位置.
            record.rows[0]['position_valid']=False
            self.assertTrue(math.isnan(record.series('position')[1][0]))
            record.rows[0]['pid_valid']=False
            self.assertTrue(math.isnan(record.series('velocity_used')[1][0]))

    def test_speed_shortcuts(self):
        self.assertEqual(velocity_shortcut(1,.5,False,15),1.5)
        self.assertEqual(velocity_shortcut(1,-2,False,15),-1)
        self.assertEqual(velocity_shortcut(-3,0,True,15),3)
        for target,delta in [(15,.5),(float('nan'),0)]:
            with self.assertRaises(ValueError):velocity_shortcut(target,delta,False,15)

    def test_position_shortcuts(self):
        self.assertAlmostEqual(position_shortcut(.1,.01),.11)
        self.assertAlmostEqual(position_shortcut(0,-.1),-.1)
        self.assertAlmostEqual(position_shortcut(7,.1),7.1)
        self.assertAlmostEqual(position_shortcut(0,.0001),.0001)
        for value in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):position_shortcut(value,.01)

    def test_one_second_time_weighted_speed(self):
        model=LiveModel()
        # 不规则到达, v(t)=2t; 最近1秒[.2,1.2]的均值为1.4, 不取样本算术平均.
        for t in [0,.13,.27,.4,.62,.9,1.2]:
            model.accept(dict(self.status,speed_rad_s=2*t),t)
        self.assertAlmostEqual(model.average_speed(1.2),1.4)
        self.assertIsNone(model.average_speed(1.7))
        model.accept(dict(self.status,feedback_valid=False),1.3)
        self.assertIsNone(model.average_speed(1.3))
        for i in range(12):
            t=1.4+i*.1
            model.accept(dict(self.status,speed_rad_s=-2),t)
        self.assertAlmostEqual(model.average_speed(2.5),-2)
        model.accept(self.status,3.1)
        self.assertIsNone(model.average_speed(3.1))
        short=LiveModel();short.accept(self.status,0);short.accept(self.status,.1)
        self.assertIsNone(short.average_speed(.1))

    def test_bad_parameters_not_saved(self):
        with tempfile.TemporaryDirectory() as folder:
            values=status_values(self.status);values[2]=float('nan')
            path=Path(folder)/'bad.json'
            with self.assertRaises(ValueError):save_profile(path,1,values,'')
            self.assertFalse(path.exists())


if __name__=='__main__':unittest.main()
