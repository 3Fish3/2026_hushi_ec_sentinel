import csv, json, math, sys, collections
from pathlib import Path

D = Path(sys.argv[1])
S = json.loads((D / 'summary.json').read_text())
start = S['sampling_start_monotonic_s']
end = S['sampling_end_monotonic_s']
rows = [json.loads(l) for l in (D / 'samples.jsonl').open()]
fast = [r for r in rows if r['kind'] == 'status0' and start <= r['monotonic_s'] <= end]
valid = [r for r in fast if r['data']['main_platform']['validity'] == 'valid']

print('== %s ==' % D.name)
print('duration %.2fs, fast %d, valid %d (%.0f%%)' % (
    end - start, len(fast), len(valid), 100.0 * len(valid) / max(1, len(fast))))
print('params identical:', S['parameters_before'] == S['parameters_after'])

def load(name):
    with (D / name).open() as f:
        return list(csv.DictReader(f))

def fnum(x):
    try:
        return float(x)
    except Exception:
        return None

for axis in ('main_yaw', 'fire_yaw'):
    c = load('%s-control.csv' % axis)
    if not c:
        print(axis, 'no rows'); continue
    lim = sum(1 for r in c if int(r['limited_samples'] or 0) > 0)
    samp = sum(int(r['control_samples'] or 0) for r in c)
    lims = sum(int(r['limited_samples'] or 0) for r in c)
    subf = sum(1 for r in c if r['submit_failed'] == 'True')
    cmiss = sum(1 for r in c if r['command_missing'] == 'True')
    fmiss = sum(1 for r in c if r['feedback_missing'] == 'True')
    eff = [abs(fnum(r['effort_raw'])) for r in c if fnum(r['effort_raw']) is not None]
    cmds = [abs(fnum(r['command_normalized'])) for r in c if fnum(r['command_normalized']) is not None]
    print('%s: rows=%d control_samples=%d limited_samples=%d(%.2f%%) limited_frames=%d/%d' % (
        axis, len(c), samp, lims, 100.0 * lims / max(1, samp), lim, len(c)))
    print('   peak|cmd|=%.4f  peak|effort|=%s  submit_failed=%d command_missing=%d feedback_missing=%d' % (
        max(cmds) if cmds else float('nan'), max(eff) if eff else None, subf, cmiss, fmiss))

# heading bins (5s)
h = []
for r in valid:
    v = r['data']['main_platform']['heading_rad']
    h.append(v if not h else h[-1] + math.remainder(v - h[-1], 2 * math.pi))
print('\n5s bins: span_deg  net_deg  wheel|v| mean')
for lo in range(0, int(end - start) + 1, 5):
    hi = lo + 5
    idx = [i for i, r in enumerate(valid) if lo <= r['monotonic_s'] - start < hi]
    if len(idx) < 2:
        continue
    seg = [h[i] for i in idx]
    speeds = []
    for i in idx:
        w = valid[i]['data'].get('wheel_odom') or {}
        tw = w.get('twist')
        if w.get('validity') == 'valid' and tw:
            speeds.append(math.hypot(tw[0], tw[1]))
    print('%2d-%2ds span=%7.3f net=%+7.3f wheel=%.3f n=%d' % (
        lo, hi, math.degrees(max(seg) - min(seg)), math.degrees(seg[-1] - seg[0]),
        sum(speeds) / len(speeds) if speeds else 0.0, len(idx)))

# main tracking error from page1
errs = []
for r in rows:
    if r['kind'] != 'status1' or not (start <= r['monotonic_s'] <= end):
        continue
    for d in r['data'].get('domains', []):
        if d['domain'] != 'main_yaw' or not d.get('measurement_valid'):
            continue
        if d.get('target') is None or d.get('measurement') is None:
            continue
        errs.append(abs(math.degrees(math.remainder(d['target'] - d['measurement'], 2 * math.pi))))
if errs:
    errs.sort()
    print('\nmain tracking |err|: n=%d peak=%.2f p95=%.2f median=%.2f deg' % (
        len(errs), errs[-1], errs[int(0.95 * (len(errs) - 1))], errs[len(errs) // 2]))
