"""离线PI候选筛选: 必须通过原生C++控制链, 不连接串口、不自动部署."""
import argparse
import itertools
import json
import math
from pathlib import Path
import subprocess


def model_usable(model):
    # 这是模型选用条件, 不是固件停机门禁; 近静止记录不使用易失真的百分比fit.
    records=model['training']+model['validation']
    return bool(model['validation']) and all(r['rmse_rad_s'] <= .15 or
        (r['fit_percent'] is not None and r['fit_percent'] >= 80) for r in records)


def suggest(model, executable):
    if not model_usable(model):
        return dict(schema=1,status='model_not_validated',deployment_ready=False,candidates=[],
                    reason='留出/训练记录未满足模型选用条件, 不从失配模型生成实机参数')
    p=model['parameters'];a,b,c,delay=[p[k] for k in
        ('a_rad_s2_per_command_a','b_per_s','c_rad_s2','delay_s')]
    cases=[]
    # 用二阶闭环极点关系形成候选, 不是在硬件上盲扫. 延迟/量化/饱和交原生仿真检验.
    for wn,zeta in itertools.product((5,10,15,20,30,40,60),(0.7,1.0,1.3)):
        kp=(2*zeta*wn-b)/a;ki=wn*wn/a
        if kp<=0: continue
        for scale in (.5,1,1.5):cases.append((kp,ki*scale,wn,zeta))
    requests=[];identities=[]
    for idx,(kp,ki,wn,zeta) in enumerate(cases):
        # +/-20%仅为明确的敏感性场景, 不假称实测参数置信区间.
        for factor,target in itertools.product((.8,1,1.2),(-1,1)):
            requests.append(f'{a*factor} {b} {c} {delay} {kp} {ki} 7.5 2 {target} 4 0')
            identities.append(idx)
    if not requests: return dict(schema=1,status='no_candidates',deployment_ready=False,candidates=[])
    run=subprocess.run([str(executable)],input='\n'.join(requests)+'\n',text=True,capture_output=True,check=True,timeout=120)
    lines=run.stdout.splitlines()
    if len(lines)!=len(requests): raise ValueError('原生模拟响应数不匹配')
    groups={i:[] for i in range(len(cases))}
    for identity,line in zip(identities,lines):
        values=[float(v) for v in line.split()]
        if len(values)!=7 or not all(math.isfinite(v) for v in values): raise ValueError('原生模拟指标非法')
        groups[identity].append(values)
    ranked=[]
    for idx,rows in groups.items():
        kp,ki,wn,zeta=cases[idx]
        # 无增长/持续饱和是基本可用性; 不用压低cap让电机不动来优化分数.
        if any(row[6] or row[4]>.01 or abs(row[0])>.1 or row[1]>.2 or row[3]>1.5 for row in rows):continue
        objective=max(r[1]+.05*r[2]+.001*r[5] for r in rows)
        ranked.append(dict(kp=kp,ki=ki,kd=0,feedback_alpha=0,score=objective,
                           design_wn_rad_s=wn,design_zeta=zeta,simulation=rows))
    ranked.sort(key=lambda r:r['score'])
    return dict(schema=1,status='simulation_candidates' if ranked else 'no_passing_candidates',deployment_ready=False,
                candidates=ranked[:5],boundary='仅仿真候选, 实机留出工况验收后才允许作为空载基线; 非实车参数')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True);p.add_argument('--native',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();model=json.loads(args.model.read_text(encoding='utf-8'))
    report=suggest(model,args.native)
    with args.output.open('x',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
