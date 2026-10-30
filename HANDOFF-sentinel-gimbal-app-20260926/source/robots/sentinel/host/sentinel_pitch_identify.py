"""SIMULATION ONLY: fixed-period square-torque identification, no serial access.

This first-stage experiment tests the recording/analysis method, NOT a physical
motor. All default plant parameters and excitation limits are fictional. No
output from this module is authorised for use as vehicle gains or excitation.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np
from scipy import linalg, optimize, signal

SOURCE = 'SIMULATION_ONLY'
SCHEMA = 1


@dataclass(frozen=True)
class Experiment:
    sample_hz: int = 500
    baseline_s: float = 1.0
    period_s: float = .6
    cycles: int = 6
    final_hold_s: float = 1.0
    amplitude_nm: float = .08
    polarity: int = 1
    target_rad: float = .1
    kp: float = 12.0
    kd: float = .4
    bias_nm: float = .35
    travel_rad: float = .08
    max_velocity_rad_s: float = .8
    max_torque_nm: float = 1.0

    def __post_init__(self):
        values = asdict(self)
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError('experiment values must be finite')
        positive = ('sample_hz', 'baseline_s', 'period_s', 'cycles', 'final_hold_s',
                    'amplitude_nm', 'travel_rad', 'max_velocity_rad_s', 'max_torque_nm', 'kp')
        if any(values[key] <= 0 for key in positive) or self.kd < 0:
            raise ValueError('invalid experiment limits')
        if self.polarity not in (-1, 1) or not isinstance(self.cycles, int):
            raise ValueError('invalid polarity/cycle count')
        if not isinstance(self.sample_hz, int) or not 100 <= self.sample_hz <= 1000:
            raise ValueError('sample_hz must be an integer from 100 to 1000')
        if self.period_s * self.sample_hz < 20 or self.duration_s > 30:
            raise ValueError('insufficient samples per period or excessive duration')
        if self.kp > 500 or self.kd > 5:
            raise ValueError('outside MIT protocol gain ranges')

    @property
    def duration_s(self):
        return self.baseline_s + self.cycles * self.period_s + self.final_hold_s


@dataclass(frozen=True)
class Plant:
    """Fictional local mechanical model, independent of vehicle configuration."""
    inertia_kg_m2: float = .025
    viscous_nm_s_rad: float = .08
    gravity_slope_nm_rad: float = .4
    gravity_bias_nm: float = .35
    delay_samples: int = 2
    angle_noise_rad: float = .00008
    velocity_noise_rad_s: float = .002

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('plant values must be finite')
        if self.inertia_kg_m2 <= 0 or self.viscous_nm_s_rad < 0:
            raise ValueError('invalid inertia/damping')
        if not isinstance(self.delay_samples, int) or not 0 <= self.delay_samples <= 5:
            raise ValueError('simulation delay must be 0..5 samples')
        if self.angle_noise_rad < 0 or self.velocity_noise_rad_s < 0:
            raise ValueError('noise must be nonnegative')


def square_wave(times, plan):
    # MCU timing is simulated here. Fixed frequency, not a chirp/sweep.
    phase = times - plan.baseline_s
    active = (phase >= 0) & (phase < plan.cycles * plan.period_s)
    halves = np.floor((phase + 1e-10) / (plan.period_s / 2)).astype(np.int64)
    return np.where(active, plan.polarity * plan.amplitude_nm * np.where(halves % 2 == 0, 1., -1.), 0.)


def validation_plan(plan):
    # Different input and noise, never used for fitting or delay selection.
    return replace(plan, amplitude_nm=plan.amplitude_nm * .7, polarity=-plan.polarity)


def delayed(values, count):
    return np.concatenate((np.zeros(count), values[:-count])) if count else values.copy()


def simulate(plan, plant, *, seed):
    dt = 1. / plan.sample_hz
    times = np.arange(round(plan.duration_s * plan.sample_hz) + 1) * dt
    perturbation = square_wave(times, plan)
    applied = delayed(perturbation, plant.delay_samples)
    stiffness = plan.kp + plant.gravity_slope_nm_rad
    if stiffness <= 0:
        raise ValueError('unstable simulated holding configuration')
    equilibrium = plan.target_rad + (plan.bias_nm - plant.gravity_bias_nm) / stiffness
    position, velocity = equilibrium, 0.
    angles, velocities, torques = [], [], []

    def dynamics(state, disturbance):
        q, v = state
        gravity = plant.gravity_bias_nm + plant.gravity_slope_nm_rad * (q - plan.target_rad)
        torque = plan.kp * (plan.target_rad - q) - plan.kd * v + plan.bias_nm + disturbance
        if abs(torque) > plan.max_torque_nm:
            raise ValueError('simulated torque guard tripped')
        return np.array((v, (torque - gravity - plant.viscous_nm_s_rad * v) / plant.inertia_kg_m2))

    # RK4 plant, not the discrete transfer-function implementation used to fit it.
    for disturbance in applied:
        if abs(position - plan.target_rad) > plan.travel_rad:
            raise ValueError('simulated travel guard tripped')
        if abs(velocity) > plan.max_velocity_rad_s:
            raise ValueError('simulated velocity guard tripped')
        state = np.array((position, velocity))
        angles.append(position)
        velocities.append(velocity)
        torques.append(plan.kp * (plan.target_rad - position) - plan.kd * velocity + plan.bias_nm + disturbance)
        k1 = dynamics(state, disturbance)
        k2 = dynamics(state + dt * k1 / 2, disturbance)
        k3 = dynamics(state + dt * k2 / 2, disturbance)
        k4 = dynamics(state + dt * k3, disturbance)
        position, velocity = state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
    random = np.random.default_rng(seed)
    size = len(times)
    data = {
        'sequence': np.arange(size), 'time_us': np.rint(times * 1e6).astype(np.int64),
        'perturbation_nm': perturbation, 'target_rad': np.full(size, plan.target_rad),
        'kp': np.full(size, plan.kp), 'kd': np.full(size, plan.kd), 'bias_nm': np.full(size, plan.bias_nm),
        'angle_rad': np.array(angles) + random.normal(0, plant.angle_noise_rad, size),
        'velocity_rad_s': np.array(velocities) + random.normal(0, plant.velocity_noise_rad_s, size),
        'simulated_applied_torque_nm': np.array(torques),
        'enabled': np.ones(size), 'faulted': np.zeros(size), 'saturated': np.zeros(size), 'lost': np.zeros(size),
    }
    validate_data(data, plan)
    return data


def validate_data(data, plan):
    required = ('sequence', 'time_us', 'perturbation_nm', 'target_rad', 'kp', 'kd', 'bias_nm',
                'angle_rad', 'velocity_rad_s', 'enabled', 'faulted', 'saturated', 'lost')
    if any(key not in data for key in required):
        raise ValueError('missing sample fields')
    size = len(data['time_us'])
    expected = round(plan.duration_s * plan.sample_hz) + 1
    if size != expected or any(len(data[key]) != size for key in required):
        raise ValueError('incomplete experiment: baseline, all cycles and final hold are required')
    if data['time_us'][0] != 0:
        raise ValueError('experiment time must start at zero')
    if any(not np.all(np.isfinite(data[key])) for key in required):
        raise ValueError('non-finite samples')
    if not np.array_equal(data['sequence'], np.arange(size)):
        raise ValueError('missing/reordered sample sequence')
    intervals = np.diff(data['time_us']) / 1e6
    if not np.allclose(intervals, 1. / plan.sample_hz, rtol=.001, atol=1e-6):
        raise ValueError('irregular sample timing; do not fill missing points with cached values')
    for key, value in (('kp', plan.kp), ('kd', plan.kd), ('target_rad', plan.target_rad), ('bias_nm', plan.bias_nm)):
        if not np.allclose(data[key], value, rtol=0, atol=1e-9):
            raise ValueError('holding configuration changed: ' + key)
    for key, value in (('enabled', 1), ('faulted', 0), ('saturated', 0), ('lost', 0)):
        if not np.all(data[key] == value):
            raise ValueError('invalid state: ' + key)
    if np.max(np.abs(data['angle_rad'] - plan.target_rad)) > plan.travel_rad:
        raise ValueError('travel envelope violated')
    if np.max(np.abs(data['velocity_rad_s'])) > plan.max_velocity_rad_s:
        raise ValueError('velocity envelope violated')
    times = (data['time_us'] - data['time_us'][0]) / 1e6
    if not np.allclose(data['perturbation_nm'], square_wave(times, plan), rtol=0, atol=1e-9):
        raise ValueError('input does not match fixed-period square experiment')
    if np.count_nonzero(np.diff(data['perturbation_nm'])) < 4:
        raise ValueError('insufficient excitation')


def predict(parameters, data, plan, lag):
    omega, zeta, gain = np.exp(parameters[:3])
    equilibrium = parameters[3]
    matrix = np.array(((0., 1.), (-omega**2, -2*zeta*omega)))
    augmented = np.zeros((3, 3))
    augmented[:2, :2] = matrix
    augmented[1, 2] = gain
    transition = np.asarray(linalg.expm(augmented / plan.sample_hz), dtype=float)
    ad, bd = transition[:2, :2], transition[:2, 2:]
    # Exact ZOH recurrence evaluated by compiled lfilter; no noisy double differentiation.
    denominator = [1., -np.trace(ad), np.linalg.det(ad)]
    numerator = [0., bd[0, 0], ad[0, 1]*bd[1, 0] - ad[1, 1]*bd[0, 0]]
    q0 = data['angle_rad'][0] - equilibrium
    initial = [q0, -ad[1, 1]*q0]  # simulator starts at rest
    response, _ = signal.lfilter(numerator, denominator, delayed(data['perturbation_nm'], lag), zi=initial)
    return response + equilibrium


def identify(data, plan):
    validate_data(data, plan)
    baseline = float(np.median(data['angle_rad'][:max(1, round(plan.baseline_s * plan.sample_hz))]))
    best = None
    for lag in range(6):
        fit = optimize.least_squares(
            lambda parameters, delay=lag: predict(parameters, data, plan, delay) - data['angle_rad'],
            np.r_[np.log([20., .6, 40.]), baseline],
            bounds=(np.r_[np.log([.5, .02, .02]), baseline - .02],
                    np.r_[np.log([200., 5., 1000.]), baseline + .02]),
            max_nfev=100, ftol=1e-9, xtol=1e-9, gtol=1e-10)
        if fit.success and (best is None or fit.cost < best[0].cost):
            best = fit, lag
    if best is None:
        raise ValueError('model optimisation did not converge')
    fit, lag = best
    omega, zeta, gain = np.exp(fit.x[:3])
    inertia = 1. / gain
    return {
        'parameters': fit.x.tolist(), 'delay_samples': lag, 'delay_ms': lag * 1000 / plan.sample_hz,
        'natural_frequency_rad_s': omega, 'damping_ratio': zeta,
        'inertia_kg_m2': inertia, 'viscous_nm_s_rad': 2*zeta*omega*inertia - plan.kd,
        'gravity_slope_nm_rad': omega**2*inertia - plan.kp,
        'train_rmse_rad': float(np.sqrt(np.mean(fit.fun**2))),
    }


def validate_model(model, data, plan):
    validate_data(data, plan)
    prediction = predict(np.array(model['parameters']), data, plan, model['delay_samples'])
    residual = prediction - data['angle_rad']
    active = np.abs(data['perturbation_nm']) > 0
    response = data['angle_rad'][active]
    rmse = float(np.sqrt(np.mean(residual[active]**2)))
    span = float(np.ptp(response))
    if span <= 1e-9:
        raise ValueError('response too small to validate')
    r2 = float(1 - np.sum(residual[active]**2) / np.sum((response - np.mean(response))**2))
    nrmse = rmse / span
    return {'rmse_rad': rmse, 'nrmse': nrmse, 'r2': r2, 'passed': bool(nrmse < .1 and r2 > .9)}, prediction


def write_csv(path, data):
    with path.open('x', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(data)
        writer.writerows(zip(*data.values(), strict=True))


def read_csv(path):
    with path.open(newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        rows = list(reader)
        return {key: np.array([float(row[key]) for row in rows]) for key in reader.fieldnames or ()}


def create_run(path, plan, plant, *, seed):
    if path.exists():
        raise FileExistsError(path)
    train = simulate(plan, plant, seed=seed)
    test_plan = validation_plan(plan)
    validation = simulate(test_plan, plant, seed=seed+1)
    path.mkdir(parents=True, exist_ok=False)
    manifest = {
        'schema': SCHEMA, 'source': SOURCE, 'hardware_ready': False, 'seed': seed,
        'experiment': asdict(plan), 'validation_experiment': asdict(test_plan), 'simulated_truth': asdict(plant),
        'assumptions': ['fictional local linear plant; no hardware access',
                        'fixed known PD and calibrated torque scale', 'initially at rest; no saturation',
                        'only additive perturbation is delayed in this simulator'],
    }
    (path / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    write_csv(path / 'train.csv', train)
    write_csv(path / 'validation.csv', validation)


def analyze_run(path):
    try:
        manifest = json.loads((path / 'manifest.json').read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError('unreadable or invalid manifest') from error
    ready = manifest.get('hardware_ready')
    if (manifest.get('schema') != SCHEMA or manifest.get('source') != SOURCE
            or not isinstance(ready, bool) or ready):
        raise ValueError('this analyzer only accepts SIMULATION_ONLY runs; not a hardware tool')
    plan = Experiment(**manifest['experiment'])
    test_plan = Experiment(**manifest['validation_experiment'])
    if test_plan != validation_plan(plan):
        raise ValueError('unexpected validation experiment')
    train, validation = read_csv(path / 'train.csv'), read_csv(path / 'validation.csv')
    model = identify(train, plan)
    score, prediction = validate_model(model, validation, test_plan)
    physically_plausible = model['inertia_kg_m2'] > 0 and model['viscous_nm_s_rad'] >= 0
    return {'source': SOURCE, 'hardware_ready': False, 'model': model, 'validation': score,
            'passed': bool(score['passed'] and physically_plausible),
            'prediction_rad': prediction.tolist(),
            'limitations': ['simulation result, NOT measured vehicle dynamics or deployable gains',
                            'local output-error fit with known holding gains; no full-travel gravity identification',
                            'no confidence intervals or motor inner-loop model in this first-stage proof']}


def save_report(path, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # Analysis does not alter or overwrite the original samples/manifest.
    with (path / 'report.json').open('x', encoding='utf-8') as file:
        json.dump(report, file, indent=2)
    validation = read_csv(path / 'validation.csv')
    times = validation['time_us'] / 1e6
    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].step(times, validation['perturbation_nm'], where='post')
    axes[0].set_ylabel('Added torque [Nm]')
    axes[1].plot(times, validation['angle_rad'], label='Simulated encoder')
    axes[1].plot(times, report['prediction_rad'], '--', label='Independent-run prediction')
    axes[1].set_ylabel('Angle [rad]')
    axes[1].legend()
    axes[2].plot(times, np.array(report['prediction_rad']) - validation['angle_rad'])
    axes[2].set_ylabel('Prediction error [rad]')
    axes[2].set_xlabel('Simulation time [s]')
    figure.suptitle('SIMULATION ONLY - not a vehicle experiment')
    figure.tight_layout()
    figure.savefig(path / 'response.png')
    plt.close(figure)
    text = '# Pitch square-torque identification — SIMULATION ONLY\n\n'
    text += 'No serial port, firmware flashing, motor enabling or vehicle experiment was performed.\n\n'
    text += f'Independent validation passed: {report["passed"]}\n\n'
    text += '```json\n' + json.dumps({k: v for k, v in report.items() if k != 'prediction_rad'}, indent=2) + '\n```\n'
    with (path / 'report.md').open('x', encoding='utf-8') as file:
        file.write(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='new output directory, never overwritten')
    parser.add_argument('--seed', type=int, default=27)
    args = parser.parse_args()
    create_run(args.output, Experiment(), Plant(), seed=args.seed)
    report = analyze_run(args.output)
    save_report(args.output, report)
    print(json.dumps({k: v for k, v in report.items() if k != 'prediction_rad'}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
