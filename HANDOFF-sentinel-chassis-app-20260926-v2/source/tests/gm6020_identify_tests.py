"""辨识和原生闭环模拟的离线契约; 不打开硬件, 不用合成性能替代实测."""
from pathlib import Path
import subprocess
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'boards/sentinel_f407/tools'))
import motor_bench_identify as ident
import motor_bench_tune as tune
native=sys.argv.pop(1)


class IdentifyTests(unittest.TestCase):
    def sample(self, sign, seed):
        rng=np.random.default_rng(seed)
        t=np.arange(0,1.5,.002)
        levels=np.repeat(rng.uniform(.1,.4,15),50)*sign
        d=dict(t=t,u=levels,v=np.zeros(len(t)),path=f'synthetic-{seed}',sha256='synthetic')
        d['v']=ident.simulate(d,[100,3,2,.004])
        return d

    def test_known_model_and_independent_validation(self):
        training=[self.sample(1,17),self.sample(-1,18)]
        report=ident.fit(training,[self.sample(1,19)],delay_grid=[.002,.004,.006])
        p=report['parameters']
        self.assertAlmostEqual(p['a_rad_s2_per_command_a'],100,delta=1)
        self.assertAlmostEqual(p['b_per_s'],3,delta=.1)
        self.assertAlmostEqual(p['c_rad_s2'],2,delta=.1)
        self.assertEqual(p['delay_s'],.004)
        self.assertLess(report['validation'][0]['rmse_rad_s'],.01)
        self.assertFalse(report['deployment_ready'])

    def test_reject_unexcited_data_and_bad_model(self):
        d=dict(t=np.arange(100)*.001,u=np.zeros(100),v=np.zeros(100),path='zero',sha256='synthetic')
        with self.assertRaises(ValueError):ident.fit([d],[d],delay_grid=[0])
        model={'training':[],'validation':[{'rmse_rad_s':2,'fit_percent':-1}]}
        result=tune.suggest(model,'not-executed')
        self.assertEqual(result['status'],'model_not_validated');self.assertFalse(result['deployment_ready'])

    def test_native_actual_controller_closed_loop(self):
        rows=['100 3 2 .004 .1 .5 3 2 1 4 0',
              '100 3 2 .004 .1 .5 3 2 -1 4 0',
              '100 3 2 .004 .1 .5 3 3 .3 4 0',
              '100 3 2 .004 0 0 3 2 1 4 0',
              '100 3 2 .004 .1 .5 3 2 1 4 0 .001 .003',
              '100 3 2 .004 .1 .5 3 2 1 4 0 .001 .003 1']
        output=subprocess.check_output([native],input='\n'.join(rows)+'\n',text=True)
        results=[[float(v) for v in line.split()] for line in output.splitlines()]
        self.assertEqual(len(results),6)
        for r in results[:3]+results[4:]:
            self.assertEqual(len(r),7);self.assertEqual(r[6],0);self.assertLess(abs(r[0]),.05)
        self.assertAlmostEqual(results[0][0],-results[1][0],delta=.01)
        self.assertEqual(results[3][0],-1) # 无输出不允许冒充跟踪通过.
        bad=subprocess.run([native],input='100 3 2 .2 .1 .5 3 2 1 4 0\n',text=True,capture_output=True)
        self.assertNotEqual(bad.returncode,0)


if __name__=='__main__':unittest.main()
