import argparse,csv,hashlib,json,sys
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser()
p.add_argument('run',type=Path)
p.add_argument('--source',type=Path,required=True)
p.add_argument('--native',type=Path,required=True)
p.add_argument('--base',type=Path,required=True)
a=p.parse_args()
sys.path.insert(0,str(a.source/'robots/sentinel/host'))
import sentinel_identify_analyze as tool
root=json.loads((a.run/'summary.json').read_text())
if root.get('recording_complete',root.get('complete')):
    result=tool.analyze(a.run/'positive',a.run/'negative',a.native)
else:
    result={'status':'incomplete_or_invalid_trial','candidate':None,'deployment_ready':False,'reason':root.get('error')}
with (a.run/'analysis.json').open('x') as f: json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
comparisons=[]
for run in (a.base/'run-20261109T215532',a.base/'repeat-20261109T221358',a.run):
    rp=json.loads((run/'summary.json').read_text())
    prior_analysis=a.base/'analysis.json' if run.name=='run-20261109T215532' else run/'analysis.json'
    an=json.loads(prior_analysis.read_text())
    entry={'run':run.name,'complete':rp.get('complete'),'statistics':rp.get('statistics'),'plan':rp['plan'],'directions':{}}
    for direction,key in [('positive','training'),('negative','validation')]:
        folder=run/direction
        if not (folder/'summary.json').exists(): continue
        s=json.loads((folder/'summary.json').read_text())
        m={'complete':s.get('complete'),'error':s.get('error'),'samples':s.get('samples'),
           'parameters_unchanged':s.get('parameters_before')==s.get('parameters_after'),
           'frequency_response':an.get(key,[])}
        reference=json.loads((a.base/'repeat-20261109T221358'/direction/'summary.json').read_text())
        m['parameters_match_006_reference']=s.get('parameters_before')==reference.get('parameters_before')
        m['plan_differences_vs_006_reference']={k:[reference['plan'].get(k),v] for k,v in s['plan'].items() if reference['plan'].get(k)!=v}
        if (folder/'samples.csv').exists():
            with (folder/'samples.csv').open() as f: rows=list(csv.DictReader(f))
            if rows:
                ar=lambda k:np.array([float(r[k]) for r in rows])
                t=ar('time_s');stage=ar('stage');scan=(stage>=2)&(stage<=6);v=ar('feedback');u=ar('command_a')
                flags=ar('flags').astype(int);finish=s.get('finish',{})
                m.update(samples_actual=len(rows),produced=finish.get('produced'),consumed=finish.get('consumed'),dropped=finish.get('dropped'),
                    sequence_gaps=int(np.sum(np.diff(ar('sequence'))!=1)),bad_flags=int(np.sum((flags&15)!=7)),
                    duration_s=float(t[-1]-t[0]),peak_command_a=float(max(abs(u))),
                    net_joint_turns=float((ar('joint')[-1]-ar('joint')[0])/(2*np.pi)),
                    sample_dt_median_us=float(np.median(np.diff(t))*1e6))
                if finish.get('excitation_start_us'):
                    m['working_point_s']=((finish['excitation_start_us']-s['start_us'])&0xffffffff)*1e-6
                if scan.any():
                    m.update(scan_velocity_mean_rad_s=float(v[scan].mean()),scan_velocity_std_rad_s=float(v[scan].std()),
                        scan_nonpositive_direction_fraction=float(np.mean(v[scan]*np.sign(s['plan']['speed'])<=0)),
                        scan_bias_mean_a=float(ar('bias_a')[scan].mean()),excitation_peak_a=float(max(abs(ar('excitation_a')[scan]))))
        entry['directions'][direction]=m
    comparisons.append(entry)
with (a.run/'comparison.json').open('x') as f: json.dump(comparisons,f,ensure_ascii=False,indent=2,allow_nan=False)
lines=['# 0.10 A 激励单变量对照报告','',
    '- 原 0.06 A 对照：首轮及第二轮 V2；新一轮只将独立正弦幅值改为 0.10 A。',
    '- 目标 ±1 rad/s、偏置 ±0.18 A、0.25/0.5/1/2/4 Hz、每点 4 周期保持。未修改 PID 或烧录固件，未冻结积分。',
    f'- 本轮完成：`{root.get("complete")}`；分析状态：`{result["status"]}`。',
    f'- 分析说明：{result.get("reason","见 analysis.json")}。候选是否生成：`{result.get("candidate") is not None}`；未应用任何候选。','',
    '## 采集与工作点','',
    '| 轮次 | 方向 | 样本 | 扫频均速 rad/s | 峰值命令 A | 净转角/圈 | 丢样 | 与0.06A对照参数一致 |',
    '|---|---|---:|---:|---:|---:|---:|---|']
for e in comparisons:
    for direction,m in e['directions'].items():
        if 'scan_velocity_mean_rad_s' not in m: continue
        lines.append(f'| {e["run"]} | {direction} | {m["samples_actual"]} | {m["scan_velocity_mean_rad_s"]:.4f} | {m["peak_command_a"]:.4f} | {m["net_joint_turns"]:.3f} | {m["dropped"]} | {m["parameters_match_006_reference"]} |')
lines+=['','## 各频点质量对比','',
    '| 方向 | Hz | 首轮0.06A repeat/SNR | 第二轮0.06A repeat/SNR | 本轮0.10A repeat/SNR | 本轮usable | 本轮拒绝原因 |',
    '|---|---:|---|---|---|---|---|']
for direction,key in [('positive','training'),('negative','validation')]:
    new=comparisons[-1]['directions'].get(direction,{}).get('frequency_response',[])
    for idx,r in enumerate(new):
        vals=[]
        for e in comparisons:
            fr=e['directions'][direction]['frequency_response'][idx]
            vals.append(f'{fr["relative_cycle_std"]:.3f} / {fr["median_signal_noise_ratio"]:.2f}')
        lines.append(f'| {direction} | {r["hz"]:g} | '+ ' | '.join(vals)+f' | {r["usable"]} | {", ".join(r.get("rejected_by",[]))} |')
lines+=['','## 解释边界','',
    '逐周期波动、SNR 和模型资格分别核对，不把残差直接命名为机械漂移或传感器量化噪声。',
    '如果本轮改善，只能报告该工况下的观测；既有两轮0.06A本来就有变化，单轮对照不足以确定一般性的因果规律。',
    'command_a 已是 A，扫频均速仅统计 stage 2–6；全部数值和参数对照见 comparison.json。','']
with (a.run/'REPORT.md').open('x') as f: f.write('\n'.join(lines))
with (a.run/'SHA256SUMS.txt').open('x') as f:
    for path in sorted(a.run.rglob('*')):
        if path.is_file() and path.name!='SHA256SUMS.txt':
            f.write(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+path.relative_to(a.run).as_posix()+'\n')
new=comparisons[-1]
print(json.dumps({'run':new,'analysis_status':result['status'],'reason':result.get('reason'),
    'usable':{k:sum(r['usable'] for r in result.get(k,[])) for k in ('training','validation')},
    'report':str(a.run/'REPORT.md')},ensure_ascii=False,indent=2))
