"""实测日志的观测式串联轴向候选; 无机械零位/朝前确认时不生成部署配置."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from scipy.optimize import least_squares
import sentinel_calibrate as cal
import sentinel_raw as raw

AXES = ('main', 'fire', 'pitch')

def rotate(axis, angle, vector):
    """批量Rodrigues旋转, 向量在最后一维, 不改变串联次序."""
    vector = np.broadcast_to(vector, (len(angle), 3))
    c, s = np.cos(angle)[:, None], np.sin(angle)[:, None]
    return c*vector+s*np.cross(axis, vector)+(1-c)*(vector@axis)[:, None]*axis

def axis_vector(pair):
    theta, phi = pair
    return np.array([np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi), np.cos(theta)])

def axis_angles(vector):
    v = vector/np.linalg.norm(vector)
    return [np.arccos(np.clip(v[2], -1, 1)), np.arctan2(v[1], v[0])]

def body_rate(axes, angles, rates, reference):
    am, af, ap = axes
    fire, pitch = angles[:, 1]-reference[1], angles[:, 2]-reference[2]
    main = rotate(ap, -pitch, rotate(af, -fire, am))*rates[:, 0, None]
    downstream = rotate(ap, -pitch, af)*rates[:, 1, None]+ap*rates[:, 2, None]
    return main+downstream

def decode_native(rows, axis, source, ranges):
    """只解码原生坐标, DJI首圈为日志坐标约定, 不是机械参考恢复."""
    selected = cal.joint_records(rows, axis, {'source': source})
    if len(selected) < 15:
        raise ValueError(f'{axis}原始反馈不足')
    t, q, v = [], [], []
    for r in selected:
        if r['dlc'] != 8:
            raise ValueError('CAN DLC不符')
        b = bytes.fromhex(r['data'])
        if axis == 'pitch':
            if b[0]&15 != ranges['node'] or b[0]>>4 not in (0, 1):
                raise ValueError('DM节点/状态不适用于拟合')
            position = (((b[1]<<8)|b[2])/65535*2-1)*ranges['pmax_rad']
            velocity = (((b[3]<<4)|(b[4]>>4))/4095*2-1)*ranges['vmax_rad_s']
        else:
            encoder, rpm = struct.unpack_from('>Hh', b)
            if encoder >= 8192:
                raise ValueError('DJI编码器超出13位')
            position, velocity = encoder*2*np.pi/8192, rpm*2*np.pi/60
        t.append(r['t_s']); q.append(position); v.append(velocity)
    t, q, v = np.array(t), np.array(q), np.array(v)
    if np.any(np.diff(t) <= 0):
        raise ValueError('原生样本时刻非递增')
    if axis != 'pitch':
        q = np.unwrap(q)
    return t, q, v

def derivative(t, q, half):
    if half < 1 or len(t) < 2*half+1:
        raise ValueError('导数窗口不足')
    rate = [np.polyfit(t[i-half:i+half+1]-t[i], q[i-half:i+half+1], 1)[0]
            for i in range(half, len(t)-half)]
    return t[half:-half], np.array(rate)

def mapping_ranges(path):
    groups = {key: [] for key in ('PMAX', 'VMAX', 'TMAX', 'ESC_ID', 'MST_ID', 'Gr')}
    for line in path.read_text(encoding='utf-8').splitlines():
        row = json.loads(line)
        for reg in row.get('data', {}).get('registers', []):
            if reg['name'] in groups:
                groups[reg['name']].append(reg)
    values = {}
    for name, records in groups.items():
        if not records or not all(r['valid'] for r in records):
            raise ValueError(f'{name}缺少有效回读')
        unique = {r['value'] for r in records}
        if len(unique) != 1 or records[-1]['responses'] <= records[0]['responses']:
            raise ValueError(f'{name}不一致或未更新')
        values[name] = unique.pop()
    return dict(pmax_rad=values['PMAX'], vmax_rad_s=values['VMAX'], tmax_nm=values['TMAX'],
                node=values['ESC_ID'], master=values['MST_ID'], gr=values['Gr'])

def load_dataset(manifest, root):
    base = root/manifest['dataset']; ranges = mapping_ranges(base/manifest['mapping_log'])
    if manifest['sources']['pitch']['can_id'] != ranges['master']:
        raise ValueError('数据清单的DM反馈地址与设备回读不符')
    windows = [manifest['stationary_train'], *manifest['stationary_holdout'],
               *manifest['train'], *manifest['holdout'], *manifest.get('stress', [])]
    train_files = {s['file'] for s in manifest['train']}
    if train_files & {s['file'] for s in manifest['holdout']}:
        raise ValueError('训练与留出必须来自不同文件')
    files, hashes = {}, {}
    for item in windows:
        name = item['file']
        if name in files: continue
        path = base/name; metadata, rows = raw.load(path); cal.quality(rows)
        if metadata.get('units') != {'gyro': 'rad/s', 'accel': 'm/s2', 'temperature': 'degC'}:
            raise ValueError('输入单位不符')
        summary = json.loads(Path(str(path)+'.summary.json').read_text(encoding='utf-8'))
        if not summary['complete'] or summary['crc_errors'] or summary['final']['dropped']:
            raise ValueError('输入传输不完整')
        gt, g = cal.series(rows, 'gyro'); files[name] = dict(metadata=metadata, rows=rows, gt=gt, g=g)
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        if metadata['profile'] == 'rotation':
            joints = [decode_native(rows, axis, manifest['sources'][axis], ranges) for axis in AXES]
            files[name]['joints'] = joints
            files[name]['derived'] = [derivative(t, q, manifest['analysis']['derivative_half_window']) for t, q, _ in joints]
    hashes[manifest['mapping_log']] = hashlib.sha256((base/manifest['mapping_log']).read_bytes()).hexdigest()
    if {hashes[s['file']] for s in manifest['train']} & {hashes[s['file']] for s in manifest['holdout']}:
        raise ValueError('训练与留出内容哈希相同, 重命名不能构成独立验证')
    return files, hashes, ranges

def select_window(item, files, bias, margin):
    f = files[item['file']]; lo, hi = item['interval_s']
    lo = max(lo, *(t[0]+margin for t, _ in f['derived']))
    hi = min(hi, *(t[-1]-margin for t, _ in f['derived']))
    mask = (f['gt'] >= lo) & (f['gt'] <= hi)
    return dict(name=item['file'], t=f['gt'][mask], gyro=f['g'][mask]-bias,
                joints=f['joints'], derived=f['derived'], interval=[lo, hi])

def prediction(window, axes, lags, reference, source, position_lags=None):
    t = window['t']; angles, rates = [], []
    for i, (jt, q, native) in enumerate(window['joints']):
        query = t-lags[i]
        angle_query = t-(lags[i] if position_lags is None else position_lags[i])
        angles.append(np.interp(angle_query, jt, q))
        vt, v = window['derived'][i] if source == 'position' else (jt, native)
        rates.append(np.interp(query, vt, v))
    return body_rate(axes, np.array(angles).T, np.array(rates).T, reference)

def residual(windows, axes, lags, reference, source, position_lags=None):
    return np.concatenate([prediction(w, axes, lags, reference, source, position_lags)-w['gyro'] for w in windows])

def metrics(windows, axes, lags, reference, source, position_lags=None):
    result = {}
    for w in windows:
        predicted = prediction(w, axes, lags, reference, source, position_lags); error = predicted-w['gyro']
        magnitude = np.linalg.norm(error, axis=1); energy = np.sum((w['gyro']-w['gyro'].mean(axis=0))**2)
        result[w['name']] = dict(samples=len(error), interval_s=w['interval'], rms_rad_s=float(np.sqrt(np.mean(magnitude**2))),
            p95_rad_s=float(np.quantile(magnitude, .95)), peak_rad_s=float(magnitude.max()),
            r2=float(1-np.sum(error**2)/energy), residual_bias_rad_s=error.mean(axis=0).tolist())
    return result

def run(manifest, root):
    if manifest['schema'] != 'sentinel-observed-calibration/1': raise ValueError('数据清单版本不符')
    files, hashes, ranges = load_dataset(manifest, root)
    def static_stats(item):
        f = files[item['file']]; lo, hi = item['interval_s']; g = f['g'][(f['gt']>=lo)&(f['gt']<=hi)]
        return dict(file=item['file'], interval_s=item['interval_s'], samples=len(g), mean_rad_s=g.mean(axis=0).tolist(), std_rad_s=g.std(axis=0).tolist())
    static = static_stats(manifest['stationary_train']); bias = np.array(static['mean_rad_s']); noise = np.linalg.norm(static['std_rad_s'])
    if static['samples'] < 20 or not np.isfinite(noise) or noise <= 0:
        raise ValueError('静止训练窗不足或无可用噪声尺度')
    bound = manifest['analysis']['lag_bound_s']
    windows = {key: [select_window(s, files, bias, bound) for s in manifest.get(key, [])] for key in ('train','holdout','stress')}
    r = manifest['reference']; reference = np.array([np.interp(r['at_s'], t, q) for t,q,_ in files[r['file']]['joints']])
    initial = []
    for i, name in enumerate(AXES):
        w = next(w for w,s in zip(windows['train'],manifest['train']) if s['dominant_axis']==name)
        t,v = w['derived'][i]; x = np.interp(w['t'],t,v); initial.extend(axis_angles(x@w['gyro']/(x@x)))
    def axes_of(p): return np.array([axis_vector(p[2*i:2*i+2]) for i in range(3)])
    # 稳定静止噪声给出稳健损失尺度; 不用留出数据调整它. 时移变量使用毫秒改善数值尺度.
    def fun(p): return residual(windows['train'], axes_of(p), p[6:]*.001, reference, 'position').ravel()
    fitted = least_squares(fun, np.r_[initial,[0.,0.,0.]], bounds=(np.r_[np.full(6,-np.inf),np.full(3,-bound*1000)],np.r_[np.full(6,np.inf),np.full(3,bound*1000)]),
                           loss='soft_l1', f_scale=noise, max_nfev=160, x_scale='jac')
    if not fitted.success: raise ValueError('轴向拟合未收敛: '+fitted.message)
    axes = axes_of(fitted.x); fits = {}
    for source in ('position','velocity'):
        position_lags = None if source == 'position' else fitted.x[6:]*.001
        if source == 'position': lags = fitted.x[6:]*.001
        else:
            solved = least_squares(lambda p: residual(windows['train'],axes,p*.001,reference,source,position_lags).ravel(), np.zeros(3), bounds=(-bound*1000,bound*1000),loss='soft_l1',f_scale=noise,x_scale='jac')
            if not solved.success: raise ValueError('原生速度时移拟合未收敛')
            lags = solved.x*.001
        profiles = {}
        grid = np.linspace(-bound,bound,round(2*bound/manifest['analysis']['lag_profile_step_s'])+1)
        for i,name in enumerate(AXES):
            values = {}
            for part in ('train','holdout'):
                costs=[]
                for candidate in grid:
                    test=lags.copy();test[i]=candidate;e=residual(windows[part],axes,test,reference,source,position_lags)
                    costs.append(float(np.mean(2*noise**2*(np.sqrt(1+(e/noise)**2)-1))))
                best=int(np.argmin(costs));good=grid[np.array(costs)<=min(costs)*1.01]
                values[part]=dict(best_s=float(grid[best]),edge=best in (0,len(grid)-1),within_one_percent_s=[float(good.min()),float(good.max())],costs=costs)
            # 只是条件代价曲线的分辨率检查, 不是统计置信区间或硬件时延校准.
            sample_step=max(float(np.median(np.diff(w['joints'][i][0]))) for w in windows['train'])
            qualified=not any(v['edge'] for v in values.values()) and abs(values['train']['best_s']-values['holdout']['best_s'])<=sample_step and all(v['within_one_percent_s'][1]-v['within_one_percent_s'][0]<=2*sample_step for v in values.values())
            profiles[name]=dict(estimated_s=float(lags[i]),identified_at_log_resolution=qualified,candidate_s=float(lags[i]) if qualified else None,grid_s=grid.tolist(),conditional_profiles=values)
        fits[source]=dict(lags=profiles,**{key:metrics(w,axes,lags,reference,source,position_lags) for key,w in windows.items()})
    # 留出仅用于独立复核方向; 不把这次求解结果回写训练候选.
    fixed_lags = fitted.x[6:]*.001
    check = least_squares(lambda p: residual(windows['holdout'],axes_of(p),fixed_lags,reference,'position').ravel(),
                          fitted.x[:6],loss='soft_l1',f_scale=noise,max_nfev=100)
    if not check.success: raise ValueError('留出几何重复性检查未收敛')
    held_axes = axes_of(check.x)
    repeatability = {name:float(np.degrees(np.arccos(np.clip(a@b,-1,1)))) for name,a,b in zip(AXES,axes,held_axes)}
    scale_check = {}
    for part in ('train','holdout'):
        blocks = []
        for w in windows[part]:
            columns = []
            for i in range(3):
                component = dict(w,derived=[(t,v if j==i else np.zeros_like(v)) for j,(t,v) in enumerate(w['derived'])])
                columns.append(prediction(component,axes,fixed_lags,reference,'position'))
            blocks.append(np.stack(columns,axis=-1).reshape(-1,3))
        design = np.concatenate(blocks); measured = np.concatenate([w['gyro'] for w in windows[part]]).ravel()
        coefficients,_,rank,_ = np.linalg.lstsq(design,measured,rcond=None)
        scale_check[part] = dict(rank=int(rank),rate_multipliers=dict(zip(AXES,coefficients.tolist())),applied=False)
    return dict(schema='sentinel-observed-kinematics/1',status='offline_candidate',deployment_ready=False,
        frame_V=manifest['reference'],reference_native_rad=dict(zip(AXES,reference.tolist())),axes_V={name:axis.tolist() for name,axis in zip(AXES,axes)},
        sources=manifest['sources'],dm_ranges=ranges,gyro=dict(training=static,holdout=[static_stats(s) for s in manifest['stationary_holdout']],bias_S_rad_s=bias.tolist()),
        fits=fits,heldout_axis_difference_deg=repeatability,native_unit_scale_check=scale_check,
        input_sha256=hashes,manifest=manifest,forward_V=None,mechanical_reference=None,
        tool_sha256={Path(p).name:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in (__file__,cal.__file__,raw.__file__)},
        limitations=manifest['unconfirmed']+manifest['hypotheses']+['No acceptance of full accelerometer intrinsics or temperature model','Conditional lag curves are not confidence intervals','Recorded gyro/rates are downselected and cannot certify full-rate control stability'])

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('manifest',type=Path);parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[3]);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();manifest=json.loads(args.manifest.read_text(encoding='utf-8'));result=run(manifest,args.root)
    with args.output.open('x',encoding='utf-8') as file:json.dump(result,file,ensure_ascii=False,indent=2,allow_nan=False);file.write(chr(10))
    print(json.dumps({'axes_V':result['axes_V'],'gyro_bias':result['gyro']['bias_S_rad_s'],'fits':{source:{'lags':{axis:{k:v for k,v in details.items() if k in ('estimated_s','candidate_s','identified_at_log_resolution')} for axis,details in fit['lags'].items()},'holdout':fit['holdout']} for source,fit in result['fits'].items()}},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
