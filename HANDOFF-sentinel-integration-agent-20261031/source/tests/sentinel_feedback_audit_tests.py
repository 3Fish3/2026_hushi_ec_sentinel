"""反馈审计的积分/留出/量化契约; 合成数据不作为实车参数依据."""
from pathlib import Path
import struct
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'robots/sentinel/host'))
import sentinel_feedback_audit as audit
from sentinel_dataset_calibrate import decode_native


def signal(frequency=.27, amplitude=12., *, quantized=False, scale=1., bias=0., lag=.008):
    t=np.arange(0,60,.004)
    def speed(t):
        return amplitude*np.sin(2*np.pi*frequency*t)+3*np.sin(2*np.pi*.083*t)
    q=(-amplitude*np.cos(2*np.pi*frequency*t)/(2*np.pi*frequency)
       -3*np.cos(2*np.pi*.083*t)/(2*np.pi*.083))/audit.RPM_PER_RAD_S
    rpm=scale*speed(t-lag)+bias
    if quantized:
        q=np.floor(q/audit.ENCODER_STEP_RAD)*audit.ENCODER_STEP_RAD
        rpm=np.floor(rpm)
    v=rpm/audit.RPM_PER_RAD_S
    return dict(t=t,v=v,window=audit.blocks(t,q,v,(2,58)))


class FeedbackAuditTests(unittest.TestCase):
    def test_exact_linear_integral_at_fractional_times(self):
        t=np.array([0,.2,.6,1.3,2.]);v=2*t+3
        at=np.array([0,.1,.4,1.6,2.])
        np.testing.assert_allclose(audit.integral(t,v,at),at**2+3*at,atol=1e-14)
        self.assertAlmostEqual(audit.integral([0,1,2],[1,3,4],[.5,1.5],trapezoid=False)[1],2.5)
        with self.assertRaises(ValueError):audit.integral(t,v,[-.01])
        with self.assertRaises(ValueError):audit.integral([0,1,1],[0,0,0],[.5])

    def test_known_affine_delay_and_independent_holdout(self):
        train=signal(scale=.97,bias=-.45)
        holdout=signal(.31,17,scale=.97,bias=-.45)
        result=audit.analyze_axis([train],[holdout])
        self.assertAlmostEqual(result['fit']['lag_s'],.008,delta=.001)
        self.assertAlmostEqual(result['fit']['native_scale'],.97,delta=.0001)
        self.assertAlmostEqual(result['fit']['native_bias_rpm'],-.45,delta=.0001)
        self.assertLess(result['holdout']['frozen_affine_model']['rms'],.0005)
        altered=dict(holdout,v=holdout['v']*4)
        other=audit.analyze_axis([train],[altered])
        self.assertEqual(result['fit'],other['fit'])
        self.assertGreater(other['holdout']['frozen_affine_model']['rms'],1)

    def test_floor_quantization_diagnostic_not_stationary_compensation(self):
        result=audit.analyze_axis([signal(quantized=True,lag=0)],[signal(.31,17,quantized=True,lag=0)])
        for partition in ('train','holdout'):
            self.assertLess(result[partition]['positive']['mean'],-.25)
            self.assertLess(result[partition]['negative']['mean'],-.25)
            self.assertLess(result[partition]['half_bin_hypothesis']['rms'],result[partition]['raw']['rms'])
        t=np.arange(0,10,.005);q=np.ones_like(t);v=np.zeros_like(t)
        window=audit.blocks(t,q,v,(2,8))
        self.assertTrue(np.all(window['quiet']));self.assertFalse(np.any(window['moving']))
        np.testing.assert_array_equal(audit.rpm_average(t,v,window),np.zeros(6))
        self.assertIsNone(audit.analyze_axis([dict(t=t,v=v,window=window)],[])['fit'])

    def test_raw_signed_words_have_no_implicit_bias(self):
        # 仅验证字段, 不把这些极端速度构造成可驱动请求. 旧 stream 标签不能覆盖 CAN 身份.
        rpms=[-32768,-32767,-257,-256,-255,-128,-1,0,1,127,128,255,256,32766,32767]
        rows=[dict(kind='can',bus=1,can_id=0x208,stream=99,dlc=8,t_s=i*.004,
                   data=struct.pack('>HhHBB',4096,rpm,0,30,0).hex()) for i,rpm in enumerate(rpms)]
        rows.append(dict(rows[0],bus=2,data=struct.pack('>HhHBB',0,123,0,30,0).hex()))
        _,q,v=decode_native(rows,'main',{'bus':1,'can_id':0x208},{})
        np.testing.assert_allclose(q,np.pi,rtol=0,atol=1e-14)
        np.testing.assert_allclose(v,np.array(rpms)*2*np.pi/60,rtol=0,atol=1e-12)


if __name__=='__main__':unittest.main()
