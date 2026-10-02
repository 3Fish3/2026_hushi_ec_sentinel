"""原生记录互操作与独立合成标定轨迹; 不连接硬件."""
import copy
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robots/sentinel/host'))
import sentinel_raw as raw
import sentinel_calibrate as cal
import sentinel_dataset_calibrate as observed
from unittest.mock import patch
import sentinel_replay as replay
import csv
import io
from board_monitor import Decoder, encode_frame

EXE = sys.argv.pop(1) if len(sys.argv) > 1 else None
REPLAY = sys.argv.pop(1) if len(sys.argv) > 1 else None
LIMITS = dict(stationary_std_rad_s=0.01, scale_tolerance=0.12, min_rate_std_rad_s=0.05,
              axis_rms_rad_s=0.08, holdout_rms_rad_s=0.09, fixed_rate_rad_s=0.04, fixed_position_rad=0.015)


class RawTests(unittest.TestCase):
    def test_native_roundtrip(self):
        self.assertIsNotNone(EXE)
        wire = bytes.fromhex(subprocess.check_output([EXE, '--raw-golden'], text=True).strip())
        record = raw.decode_record(wire)
        self.assertEqual(record['values'], [1.25, -0.5, 0.0])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'sample.sraw'
            raw.save(path, dict(layout_id=raw.LAYOUT, run=9, start_us=0xffffff00), [wire])
            metadata, records = raw.load(path)
            self.assertAlmostEqual(records[0]['t_s'], 240e-6)
            self.assertAlmostEqual(records[0]['arrival_s'], 276e-6)
            response = subprocess.check_output([EXE, '--raw-replay', str(path)], text=True).split()
            self.assertEqual(response, ['9', '0', '4294967280', '1.250000'])
            with path.open('ab') as file:
                file.write(b'\0')
            with self.assertRaises(ValueError):
                raw.load(path)
        with self.assertRaises(ValueError):
            raw.decode_record(b'\x02'+wire[1:])
        corrupt = bytearray(wire)
        struct.pack_into('<I', corrupt, 4, 2)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'gap.sraw'
            raw.save(path, dict(layout_id=raw.LAYOUT, run=9, start_us=0xffffff00), [bytes(corrupt)])
            _, rows = raw.load(path)
            self.assertEqual(rows[0]['gap'], 2)
            with self.assertRaises(ValueError): cal.quality(rows)

    def test_control_reference_tags(self):
        wire = bytearray(64)
        struct.pack_into('<4B7I', wire, 0, 1, 5, 4, 1, 0, 1, 1000, 1000, 0, 0, 1)
        struct.pack_into('<6f', wire, 32, 1.1, 0.2, 0.2, 0.0, 0.0, 0.001)
        struct.pack_into('<2I', wire, 56, (2 << 25) | (0 << 27), 0)
        row = raw.decode_record(wire)
        self.assertEqual(row['requested_frame'], 'odom')
        self.assertEqual(row['adopted_frame'], 'joint')

    def test_decoder_budget(self):
        # UART 最坏字节转义上界: 3*64+32 载荷, 65 ms 一页在 115200 8N1 下留余量.
        payload = bytes([0x7e])*224
        wire = encode_frame(0xb1, 0x7e7d, payload)
        self.assertLessEqual(len(wire), 2*(224+8)+2)
        self.assertLess((2*(224+8)+2)*10/115200, 0.065)
        self.assertEqual(Decoder().feed(wire), [(0xb1, 0x7e7d, payload)])
        header = struct.pack('<4BI2BH5I', 1, 0, 0, 2, 5, 2, 0, 0, 10, 20, 0, raw.LAYOUT, 512)
        self.assertEqual(raw.decode_reply(header)[0]['run'], 5)
        with self.assertRaises(ValueError): raw.decode_reply(header+b'\0')

    def test_client_window_and_retry(self):
        from unittest.mock import patch
        golden = bytes.fromhex(subprocess.check_output([EXE, '--raw-golden'], text=True).strip())
        class Port:
            def __init__(self):
                self.decoder, self.output, self.dropped = Decoder(), b'', False
                self.run = 0
            def write(self, wire):
                for kind, seq, payload in self.decoder.feed(wire):
                    self.assert_kind = kind
                    _, op, profile, domain, run, arg = struct.unpack('<4B2I', payload)
                    if op == 1:
                        self.run = run
                        if not self.dropped:
                            self.dropped = True
                            continue  # 首个开始 ACK 丢失, 重传同 run
                    record = bytearray(golden)
                    struct.pack_into('<I', record, 28, self.run)
                    header = struct.pack('<4BI2BH5I', 1, op, 0, 2, self.run, 1, 1 if op == 3 else 0,
                                         1, 0xffffff00, 1000, 0, raw.LAYOUT, 512)
                    self.output += encode_frame(0xb1, seq, header + (record if op == 3 else b''))
            def read(self, _):
                value, self.output = self.output, b''
                return value
        port = Port()
        # 只加速软件时钟, 不打开物理串口.
        ticks = iter(np.arange(0, 1000, 0.1))
        with patch.object(raw.time, 'sleep'), patch.object(raw.time, 'monotonic', side_effect=lambda: float(next(ticks))):
            status, records = raw.RawClient(port).capture('stationary', 200000)
        self.assertEqual(port.assert_kind, 0x31)
        self.assertEqual(len(records), 1)
        self.assertTrue(port.dropped)
        self.assertEqual(raw.decode_record(records[0])['run'], status['run'])

    def test_signed_axis_delay_and_holdout(self):
        axis = np.array([-0.2, 0.9, 0.4]); axis /= np.linalg.norm(axis)
        bias = np.array([0.03, -0.04, 0.02])
        tau = 0.0035
        et = np.arange(0, 0.5, 0.001)
        gt = np.arange(0.03, 0.45, 0.002)
        signal = lambda t: 2*np.sin(23*t) + 0.5*np.sin(49*t)
        gyro = signal(gt-tau)[:, None]*axis+bias
        result = cal.fit_axis([(gt, gyro, et, signal(et))], bias, 0.01, 0.00025, LIMITS)
        self.assertAlmostEqual(result['lag_s'], tau, places=4)
        np.testing.assert_allclose(result['axis_S'], axis, atol=1e-5)
        held = signal(gt-tau+0.4)[:, None]*axis+bias
        validation = cal.validate_axis([(gt, held, et, signal(et+0.4))], bias, result)
        self.assertLess(validation['rms_rad_s'], 0.001)
        with self.assertRaises(ValueError):
            cal.fit_axis([(gt, gyro, et, signal(et)*57.2958)], bias, 0.01, 0.00025, LIMITS)
        with self.assertRaises(ValueError):
            cal.fit_axis([(gt, np.ones((len(gt), 3))*axis+bias, et, np.ones(len(et)))], bias, 0.01, 0.00025, LIMITS)
        # 未测父级转动不能被一个方向拟合吸收.
        bad = gyro + np.cos(gt*77)[:, None]*np.array([1, 0, 0])
        with self.assertRaises(ValueError):
            cal.fit_axis([(gt, bad, et, signal(et))], bias, 0.01, 0.00025, LIMITS)

    def test_reference_pose(self):
        af = np.array([0.2, 0.1, 0.974679434]); af /= np.linalg.norm(af)
        ap = np.array([-0.1, 0.995, 0]); ap /= np.linalg.norm(ap)
        am = np.array([0, 0, 1.0])
        qf, qp = 0.5, -0.7
        k = cal.rotation(af, qf) @ cal.rotation(ap, qp)
        np.testing.assert_allclose(cal.reference_axis(k.T@am, 'main', dict(fire=af, pitch=ap), dict(fire=qf, pitch=qp)), am, atol=1e-8)
        np.testing.assert_allclose(cal.reference_axis(cal.rotation(ap, qp).T@af, 'fire', dict(pitch=ap), dict(pitch=qp)), af, atol=1e-8)

    def test_joint_first_branch(self):
        # 独立从机械真值生成编码器, 每个短窗都跨零; 同余周期是 2π/N, 不是固定 2π.
        for ratio in (1.0, 1.5):
            for sign in (-1, 1):
                for zero in (0.0, 0.17, -0.17):
                    spec = dict(origin='synthetic', rotor_per_axis=ratio, direction=sign,
                                zero_offset_rad=zero, max_rotor_speed_rad_s=30,
                                reference=dict(origin='synthetic', min_rad=-1, max_rad=1))
                    for start in (-0.05, 0.05):
                        truth = np.array([start, start/2, 0, -start/2, -start, start/2])
                        counts = np.round((truth+zero)*ratio/sign*8192/(2*np.pi)).astype(int) % 8192
                        rows = [dict(kind='can', stream=5, dlc=8, t_s=i*0.001,
                                     data=struct.pack('>HhI', int(e), 0, 0).hex()) for i, e in enumerate(counts)]
                        _, positions, _ = cal.joint_series(rows, 'fire', spec)
                        np.testing.assert_allclose(positions, truth, atol=2*np.pi/8192/ratio)
                        first = sign*counts[0]*2*np.pi/8192/ratio-zero
                        candidates = [first+k*2*np.pi/ratio for k in range(-4, 5)
                                      if -1 <= first+k*2*np.pi/ratio <= 1]
                        self.assertEqual(len(candidates), 1)
                        self.assertAlmostEqual(positions[0], candidates[0])
                        bad = copy.deepcopy(spec)
                        for ref in (None, dict(origin='synthetic', min_rad=-8, max_rad=8),
                                    dict(origin='synthetic', min_rad=0.4, max_rad=0.5),
                                    dict(origin='unknown', min_rad=-1, max_rad=1)):
                            bad['reference'] = ref
                            with self.assertRaises(ValueError): cal.joint_series(rows, 'fire', bad)
                        bad['reference'] = dict(origin='synthetic', min_rad=-8, max_rad=8)
                        anchor = dict(origin='user_confirmed', min_rad=start-0.01, max_rad=start+0.01)
                        _, anchored, _ = cal.joint_series(rows, 'fire', bad, anchor)
                        np.testing.assert_allclose(anchored, positions, atol=1e-10)

    def test_observed_serial_body_rate(self):
        axes = np.array([[0.2, -0.1, 1.0], [-0.1, 0.3, 1.0], [0.3, -1.0, 0.2]])
        axes /= np.linalg.norm(axes, axis=1)[:, None]
        reference = np.array([0.0, 0.6, -0.2])
        angles = np.array([[0.4, 0.8, 0.3], [-0.3, -0.2, 0.7]])
        rates = np.array([[0.8, -0.5, 0.3], [-0.7, 0.4, -0.2]])
        for q, v, estimated in zip(angles, rates, observed.body_rate(axes, angles, rates, reference)):
            def pose(x):
                return cal.rotation(axes[0], x[0]) @ cal.rotation(axes[1], x[1]-reference[1]) @ cal.rotation(axes[2], x[2]-reference[2])
            h = 1e-6; r = pose(q); skew = r.T @ ((pose(q+h*v)-pose(q-h*v))/(2*h))
            np.testing.assert_allclose(estimated, [skew[2,1], skew[0,2], skew[1,0]], atol=1e-8)
        for axis in axes:
            np.testing.assert_allclose(observed.axis_vector(observed.axis_angles(axis)), axis, atol=1e-12)

    def test_observed_fit_known_truth(self):
        axes = np.array([[.18, -.06, 1.], [-.16, .02, 1.], [.35, -.93, .05]])
        axes /= np.linalg.norm(axes, axis=1)[:,None]
        bias = np.array([.002, -.003, .004]); tau = .003
        reference = np.array([.2, .7, -.1]); t = np.arange(0, 2, .004)
        files = {}; rng = np.random.default_rng(11)
        for i, axis in enumerate(observed.AXES):
            for hold in (False, True):
                fixed = reference.copy()
                if hold:
                    fixed[1] += .2; fixed[2] += .25
                phase = .7 if hold else 0.; frequency = 12 if hold else 9
                def trajectory(time):
                    q = np.tile(fixed, (len(time),1)); v = np.zeros_like(q)
                    q[:,i] += .12*np.sin(frequency*time+phase); v[:,i] = .12*frequency*np.cos(frequency*time+phase)
                    return q,v
                q,v = trajectory(t); delayed_q,delayed_v = trajectory(t-tau)
                # 用旋转矩阵及独立串联雅可比产生测量, 不调用被测body_rate生成真值.
                g = []
                for a,rate in zip(delayed_q,delayed_v):
                    ef = cal.rotation(axes[1],a[1]-reference[1]); ep = cal.rotation(axes[2],a[2]-reference[2])
                    g.append((ef@ep).T@axes[0]*rate[0]+ep.T@axes[1]*rate[1]+axes[2]*rate[2]+bias)
                name=axis+('-held' if hold else '')
                joints=[(t,q[:,j],v[:,j]) for j in range(3)]
                files[name]=dict(gt=t,g=np.array(g)+rng.normal(0,.0003,(len(t),3)),joints=joints,derived=[observed.derivative(jt,jq,4) for jt,jq,_ in joints])
        files['static']=dict(gt=t,g=bias+rng.normal(0,.0003,(len(t),3)))
        manifest=dict(schema='sentinel-observed-calibration/1',sources={},stationary_train=dict(file='static',interval_s=[.1,1.8]),stationary_holdout=[],
                      train=[dict(file=a,dominant_axis=a,interval_s=[.1,1.8]) for a in observed.AXES],
                      holdout=[dict(file=a+'-held',dominant_axis=a,interval_s=[.1,1.8]) for a in observed.AXES],
                      reference=dict(file='main',at_s=0.),analysis=dict(lag_bound_s=.01,lag_profile_step_s=.001),unconfirmed=['forward'],hypotheses=['synthetic known truth'])
        with patch.object(observed,'load_dataset',return_value=(files,{},{})):
            result=observed.run(manifest,Path('.'))
        self.assertFalse(result['deployment_ready']);self.assertIsNone(result['forward_V'])
        for name,truth in zip(observed.AXES,axes):
            np.testing.assert_allclose(result['axes_V'][name],truth,atol=.006)
            self.assertLess(abs(result['fits']['position']['lags'][name]['estimated_s']-tau),.0015)
        # 九点线性微分对12 rad/s正弦有可计算的幅值衰减, 不能当成精确解析导数.
        offsets = np.arange(-4,5)*.004
        derivative_gain = (offsets@np.sin(12*offsets))/(12*(offsets@offsets))
        error_bound = 1.44*abs(1-derivative_gain)+1.44*12**2*.004**2/8+3*np.sqrt(3)*.0003
        self.assertLess(max(m['rms_rad_s'] for m in result['fits']['position']['holdout'].values()),error_bound)

    def test_raw_calibration_pipeline(self):
        axes = dict(pitch=np.array([0., 1., 0.]), fire=np.array([0.2, 0., np.sqrt(0.96)]), main=np.array([0., 0., 1.]))
        bias = np.array([0.02, -0.03, 0.01]); tau = 0.003
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            specs = {name: dict(origin='synthetic', rotor_per_axis=1, direction=1, zero_offset_rad=0,
                                max_rotor_speed_rad_s=30, node=1, pmax_rad=12.5, vmax_rad_s=30,
                                reference=dict(origin='synthetic', min_rad=-2, max_rad=2)) for name in axes}
            specs['fire'].update(rotor_per_axis=1.5, direction=-1, zero_offset_rad=0.17)
            specs['main']['zero_offset_rad'] = 0.03
            def make(name, moving=None, phase=0.0, shifted=False):
                records = []
                run = 42
                fixed = dict(main=0.8, fire=-0.05, pitch=0.0)
                if shifted:
                    if moving == 'main': fixed['fire'] = 0.35
                    if moving in ('main', 'fire'): fixed['pitch'] = 0.4
                def angle(t): return 0.8 + 0.12*np.sin(20*t+phase)
                def rate(t): return 2.4*np.cos(20*t+phase)
                k = cal.rotation(axes['fire'], fixed['fire']+0.05) @ cal.rotation(axes['pitch'], fixed['pitch'])
                axis_s = k.T@axes[moving] if moving == 'main' else cal.rotation(axes['pitch'], fixed['pitch']).T@axes['fire'] if moving == 'fire' else axes['pitch']
                def add(kind, stream, t, data, source):
                    sampled = int(t*1e6)
                    records.append(struct.pack('<4B7I', 1, kind, stream, 1 if kind != 4 else 4,
                        len(records), source, sampled, sampled+20, 0, 500 if kind == 1 else 0xffffffff, run) + data.ljust(32, b'\0'))
                for i in range(400):
                    t = i/1000
                    g = bias if moving is None else bias+axis_s*rate(t-tau)
                    add(1, 0, t, struct.pack('<3f', *g), i)
                    if i % 5 == 0:
                        qf = angle(t-tau) if moving == 'fire' else fixed['fire']
                        qp = angle(t-tau) if moving == 'pitch' else fixed['pitch']
                        pose = cal.rotation(axes['fire'], qf+0.05) @ cal.rotation(axes['pitch'], qp)
                        accel = pose.T @ np.array([0, 0, 9.81])
                        add(2, 0, t, struct.pack('<3f', *accel), i//5)
                    for axis, slot in cal.AXIS_SLOTS.items():
                        p = angle(t) if axis == moving else fixed[axis]
                        v = rate(t) if axis == moving else 0
                        if axis == 'pitch':
                            p16 = round((p+12.5)/25*65535); v12 = round((v+30)/60*4095)
                            data = bytes([1, p16 >> 8, p16 & 255, v12 >> 4, (v12 & 15) << 4, 0, 25, 25])
                            can_id = 0
                        else:
                            spec = specs[axis]
                            rotor = (p+spec['zero_offset_rad'])*spec['rotor_per_axis']/spec['direction']
                            rpm = v*spec['rotor_per_axis']/spec['direction']*60/(2*np.pi)
                            data = struct.pack('>HhhBB', round(rotor*8192/(2*np.pi)) % 8192, round(rpm), 0, 25, 0)
                            can_id = 0x20a if axis == 'main' else 0x208
                        add(4, slot, t, struct.pack('<I', 1 | 8 << 8 | can_id << 16)+data, i)
                meta = dict(layout_id=raw.LAYOUT, run=run, start_us=0, units={'gyro': 'rad/s', 'accel': 'm/s2'},
                            stationary_confirmed=moving is None, moving_axis=moving, parent_fixed_confirmed=True,
                            configuration='synthetic', firmware='native synthetic raw fixture')
                raw.save(base/name, meta, records)
            make('static.sraw')
            manifest = dict(stationary=['static.sraw'], quality_limits=LIMITS, reference_joint_rad=dict(main=0.8, fire=-0.05, pitch=0.0), joints=specs, axes={}, forward_V=[1, 0, 0])
            for axis in axes:
                make(axis+'.sraw', axis)
                make(axis+'-held.sraw', axis, 0.9, True)
                manifest['axes'][axis] = dict(train=[axis+'.sraw'], holdout=[axis+'-held.sraw'], lag_bound_s=0.01, lag_step_s=0.0005,
                                             fixed_relative_angles_rad=dict(fire=0, pitch=0))
            result = cal.run(manifest, base)
            self.assertFalse(result['deployment_ready'])
            for axis in axes:
                np.testing.assert_allclose(result['axes_V'][axis], axes[axis], atol=0.01)
                self.assertLess(abs(result['fits'][axis+'_position']['lag_s']-tau), 0.0015)
            # 真正从保存的原始字节/独立留出/标定候选进入同一 C++ 感知路径.
            metadata, rows = raw.load(base/'pitch-held.sraw')
            text = replay.native_input(result, metadata, rows, 0.01, 0.02, 0.8)
            self.assertIsNotNone(REPLAY)
            output = subprocess.run([REPLAY], input=text, text=True, capture_output=True, check=True)
            data = list(csv.DictReader(io.StringIO(output.stdout)))
            valid = [r for r in data if r['validity'] == '3']
            self.assertGreater(len(valid), 300)
            self.assertLess(np.ptp([float(r['heading_rad']) for r in valid]), 0.015)
            self.assertLess(max(abs(float(r['heading_rate_rad_s'])) for r in valid), 0.04)
            self.assertEqual({r['local_epoch'] for r in valid}, {'1'})
            # 现场main/fire地址互换后, 标定与回放都按同一显式地址选择.
            mapped = copy.deepcopy(result); mapped_rows = copy.deepcopy(rows)
            for axis, identifier in [('main', 0x208), ('fire', 0x20a), ('pitch', 0)]:
                mapped['joints'][axis]['source'] = dict(bus=1, can_id=identifier)
            for row in mapped_rows:
                if row['kind'] == 'can' and row['can_id'] in (0x208, 0x20a):
                    row['can_id'] = 0x20a if row['can_id'] == 0x208 else 0x208
                    row['stream'] = 4 if row['stream'] == 5 else 5
            self.assertEqual(replay.native_input(mapped, metadata, mapped_rows, 0.01, 0.02, 0.8), text)
            rate_lines = text.splitlines()
            rate_lines[0] = rate_lines[0].replace('SERIAL1 ', 'SERIAL_RATE1 ', 1)+' 5000'
            rate_output = subprocess.run([REPLAY], input=chr(10).join(rate_lines)+chr(10), text=True, capture_output=True, check=True)
            rate_rows = list(csv.DictReader(io.StringIO(rate_output.stdout)))
            self.assertNotIn('heading_rad', rate_rows[0])
            rate_valid = [r for r in rate_rows if r['validity'] == '3']
            self.assertEqual(len(rate_valid), len(valid))
            self.assertLess(max(np.linalg.norm([float(r[f'rate_{a}_V_rad_s']) for a in 'xyz']) for r in rate_valid), 0.05)
            # 窗口自己的首帧锚区间也进入相同换算/原生入口; 无锚的多解模型拒绝.
            ambiguous = copy.deepcopy(result)
            ambiguous['joints']['fire']['reference'].update(min_rad=-8, max_rad=8)
            with self.assertRaises(ValueError): replay.native_input(ambiguous, metadata, rows, 0.01, 0.02, 0.8)
            anchored = dict(metadata, joint_reference={'fire': dict(origin='user_confirmed', min_rad=-0.06, max_rad=-0.04)})
            self.assertEqual(replay.native_input(ambiguous, anchored, rows, 0.01, 0.02, 0.8), text)
            bad = copy.deepcopy(manifest)
            bad['axes']['fire']['holdout'] = bad['axes']['fire']['train']
            with self.assertRaises(ValueError): cal.run(bad, base)
            bad = copy.deepcopy(manifest)
            bad['joints']['main']['rotor_per_axis'] = 19
            with self.assertRaises(ValueError): cal.run(bad, base)
            # 平行轴不能靠混合标签自动认领; 缺少轴标签或真实非被测关节运动均拒绝.
            bad = copy.deepcopy(manifest)
            bad['axes']['main']['train'] = ['fire.sraw']
            with self.assertRaises(ValueError): cal.run(bad, base)


if __name__ == '__main__':
    unittest.main()
