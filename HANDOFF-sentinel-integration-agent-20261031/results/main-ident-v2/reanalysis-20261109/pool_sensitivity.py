"""Descriptive pooling sensitivity only; does not change trial usability or create PID candidates."""
from pathlib import Path
import json
import numpy as np

base=Path(__file__).resolve().parent
r=json.loads((base/'independent-analysis.json').read_text())
z=lambda x:complex(x['real'],x['imag'])
def block(tr,index):
    c=tr['frequencies'][index]['cycles']
    Y=np.array([z(x['Y']) for x in c]); U=np.array([z(x['U']) for x in c]); D=np.array([z(x['D']) for x in c])
    return np.sum(Y*D.conj()),np.sum(U*D.conj())
def pooled(blocks):
    return sum(x[0] for x in blocks)/sum(x[1] for x in blocks)
rows=[]
for index,f in enumerate((.25,.5,1,2,4)):
    blocks=[block(t,index) for t in r['trials']]
    g=pooled(blocks)
    loo=[pooled([b for j,b in enumerate(blocks) if j!=i]) for i in range(4)]
    runs=[]
    for run in ('run-20261109T215532','repeat-20261109T221358'):
        runs.append(pooled([block(t,index) for t in r['trials'] if t['run']==run]))
    rows.append({'hz':f,'pooled_magnitude':float(abs(g)),'pooled_phase_deg':float(np.angle(g,deg=True)),
        'max_leave_one_trial_out_relative_complex_change':float(max(abs(np.array(loo)/g-1))),
        'run1_pooled_magnitude':float(abs(runs[0])),'run2_pooled_magnitude':float(abs(runs[1])),
        'run2_vs_run1_relative_complex_change':float(abs(runs[1]/runs[0]-1))})
result={'assumption':'All directions/runs share a local plant. Includes points rejected by per-trial quality rules.',
    'meaning':'Descriptive sensitivity, not holdout model validation; no eligibility criteria were relaxed.',
    'frequencies':rows}
with (base/'pooled-sensitivity.json').open('x') as f:
    json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
print(json.dumps(result,ensure_ascii=False,indent=2))
