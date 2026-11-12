"""Offline closed-loop Pitch reference-to-position fit; never writes gains or opens serial."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import sentinel_raw as raw


def score(actual, predicted):
    residual = actual - predicted
    total = float(np.sum((actual - np.mean(actual)) ** 2))
    return dict(rmse_rad=float(np.sqrt(np.mean(residual ** 2))),
                r2=1 - float(np.sum(residual ** 2)) / total if total > 0 else None)


def windows(u, y, t, order, delay, dt):
    start = max(order, delay + order - 1)
    result = []
    for k in range(start, len(y)):
        # Exclude a complete lag window around every missing/irregular sample.
        gaps = np.diff(t[k-start:k+1])
        if np.any(gaps <= 0) or np.any(gaps > 1.5 * dt):
            continue
        row = [y[k-i] for i in range(1, order+1)]
        row += [u[k-delay-j] for j in range(order)]
        result.append((k, row))
    return result


def simulate(u, y, t, coefficient, order, delay, dt):
    start = max(order, delay + order - 1)
    predicted = np.array(y, copy=True)
    mask = np.zeros(len(y), dtype=bool)
    warm_until = start
    for k in range(start, len(y)):
        if not 0 < t[k]-t[k-1] <= 1.5*dt:
            warm_until = k+start
        if k < warm_until:
            continue
        row = [predicted[k-i] for i in range(1, order+1)]
        row += [u[k-delay-j] for j in range(order)]
        predicted[k] = float(np.dot(row, coefficient))
        mask[k] = True
    return predicted, mask


def fit_rows(metadata, rows):
    if not metadata.get('recipe', {}).get('position_amplitude_rad', 0):
        raise ValueError('requires an MCU position-waveform recording')
    samples = []
    excluded = 0
    for row in rows:
        if row.get('kind') != 'control' or row.get('stream') != 6:
            continue
        values = [row.get(k) for k in ('t_s', 'pitch_adopted_rad', 'pitch_measured_rad')]
        if any(not isinstance(v, (float, int)) or not math.isfinite(v) for v in values):
            excluded += 1
            continue
        if row.get('latched') or row.get('send_result') != 1:
            excluded += 1
            continue
        if ((row.get('state_bits', 0) >> 16) & 15) != 1:
            excluded += 1
            continue
        samples.append(values)
    if len(samples) < 120:
        raise ValueError('need at least 120 valid position samples')
    t, reference, measured = np.asarray(samples, dtype=float).T
    if np.any(np.diff(t) <= 0):
        raise ValueError('non-increasing MCU sample times')
    dt = float(np.median(np.diff(t)))
    split = int(len(t)*.6)
    # Centres are learned from the training portion only.
    u0, y0 = float(np.mean(reference[:split])), float(np.mean(measured[:split]))
    u, y = reference-u0, measured-y0
    report = dict(schema_version=1, model_scope='closed_loop_position_reference_to_position',
                  source=metadata.get('source'), run=metadata.get('run'), deployable_gains=False,
                  samples=len(t), excluded_samples=excluded, nominal_dt_s=dt,
                  recording_gaps=int(np.sum(np.diff(t) > 1.5*dt)),
                  validation='first 60 percent train; last 40 percent free simulation',
                  reference_span_rad=float(np.ptp(reference)), position_span_rad=float(np.ptp(measured)),
                  candidates=[])
    chosen = None
    for order in (1, 2):
        for delay in (0, 1, 2, 3):
            design = windows(u[:split], y[:split], t[:split], order, delay, dt)
            index = np.array([k for k, _ in design], dtype=int)
            x = np.array([r for _, r in design])
            coefficient, _, rank, _ = np.linalg.lstsq(x, y[index], rcond=None)
            if rank < 2*order:
                continue
            poles = np.roots(np.r_[1., -coefficient[:order]])
            if np.any(np.abs(poles) >= 1):
                continue
            yp, mask = simulate(u[split:], y[split:], t[split:], coefficient, order, delay, dt)
            if np.sum(mask) < 40:
                continue
            validation = score(y[split:][mask], yp[mask])
            a, b = coefficient[:order], coefficient[order:]
            denominator = 1-float(np.sum(a))
            result = dict(order=order, input_delay_samples=delay,
                          output_history_coefficients=a.tolist(), input_coefficients=b.tolist(),
                          input_centre_rad=u0, output_centre_rad=y0,
                          dc_gain=float(np.sum(b))/denominator if denominator else None,
                          poles=[[float(v.real), float(v.imag)] for v in poles],
                          training_one_step=score(y[index], x@coefficient),
                          validation_free_run=validation)
            report['candidates'].append(result)
            if chosen is None or validation['rmse_rad'] < chosen['validation_free_run']['rmse_rad']:
                chosen = result
    report['selected'] = chosen
    report['status'] = ('validated_closed_loop_fit' if chosen and
                        chosen['validation_free_run']['r2'] is not None and
                        chosen['validation_free_run']['r2'] >= .7 else 'insufficient_fit')
    report['limitations'] = ['empirical closed-loop model at the recorded gains and posture',
                             'no bare inertia/damping estimate or automatic gain deployment',
                             'nominal-period discrete model; lag windows crossing gaps are excluded']
    return report, t, reference, measured, split


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True, help='new JSON file')
    p.add_argument('--plot', type=Path, help='new PNG file')
    args = p.parse_args()
    if args.output.exists() or (args.plot and args.plot.exists()):
        p.error('output already exists')
    metadata, rows = raw.load(args.input)
    report, t, reference, measured, split = fit_rows(metadata, rows)
    with args.output.open('x', encoding='utf-8') as file:
        json.dump(report, file, ensure_ascii=False, allow_nan=False, indent=2)
        file.write('\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axis = plt.subplots(figsize=(10, 4))
        axis.plot(t, reference, label='MCU reference', linewidth=1.1)
        axis.plot(t, measured, label='measured', linewidth=1)
        axis.axvline(t[split], color='grey', linestyle='--', label='held-out validation begins')
        chosen = report['selected']
        if chosen:
            c = np.r_[chosen['output_history_coefficients'], chosen['input_coefficients']]
            yp, mask = simulate(reference[split:]-chosen['input_centre_rad'],
                                measured[split:]-chosen['output_centre_rad'], t[split:], c,
                                chosen['order'], chosen['input_delay_samples'], report['nominal_dt_s'])
            axis.plot(t[split:][mask], yp[mask]+chosen['output_centre_rad'], label='free simulation')
        axis.set(xlabel='MCU time [s]', ylabel='joint angle [rad]', title=report['status'])
        axis.legend(); axis.grid(alpha=.2); fig.tight_layout()
        fig.savefig(args.plot, dpi=160); plt.close(fig)
    print(json.dumps(dict(status=report['status'], selected=report['selected']), ensure_ascii=False))
    return 0 if report['status'] == 'validated_closed_loop_fit' else 2


if __name__ == '__main__':
    raise SystemExit(main())
