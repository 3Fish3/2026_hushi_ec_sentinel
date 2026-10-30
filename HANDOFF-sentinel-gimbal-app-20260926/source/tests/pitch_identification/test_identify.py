"""Offline identification contracts; no serial port or firmware access."""
import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import sentinel_pitch_identify as ident


class ExperimentTests(unittest.TestCase):
    def test_fixed_period_baseline_and_return_to_hold(self):
        plan = ident.Experiment()
        t = np.arange(0, plan.duration_s, 1 / plan.sample_hz)
        wave = ident.square_wave(t, plan)
        self.assertTrue(np.all(wave[t < plan.baseline_s] == 0))
        self.assertTrue(np.all(wave[t >= plan.baseline_s + plan.cycles * plan.period_s] == 0))
        self.assertEqual(set(np.unique(wave)), {-plan.amplitude_nm, 0., plan.amplitude_nm})
        changes = t[np.flatnonzero(np.diff(wave) != 0) + 1]
        np.testing.assert_allclose(np.diff(changes[1:-1]), plan.period_s / 2, atol=1 / plan.sample_hz)

    def test_reject_invalid_plan(self):
        cases = [lambda: ident.Experiment(amplitude_nm=-1), lambda: ident.Experiment(sample_hz=0),
                 lambda: ident.Experiment(period_s=float('nan')), lambda: ident.Experiment(cycles=0),
                 lambda: ident.Experiment(kp=-1), lambda: ident.Experiment(kd=-1),
                 lambda: ident.Experiment(max_torque_nm=0)]
        for index, case in enumerate(cases):
            with self.subTest(index=index), self.assertRaises(ValueError):
                case()

    def test_simulation_is_reproducible_and_stops_at_limits(self):
        plan = ident.Experiment()
        first = ident.simulate(plan, ident.Plant(), seed=5)
        second = ident.simulate(plan, ident.Plant(), seed=5)
        np.testing.assert_array_equal(first['angle_rad'], second['angle_rad'])
        with self.assertRaisesRegex(ValueError, 'torque'):
            ident.simulate(ident.Experiment(max_torque_nm=.1), ident.Plant(), seed=5)
        with self.assertRaisesRegex(ValueError, 'travel'):
            ident.simulate(ident.Experiment(travel_rad=.0001), ident.Plant(), seed=5)

    def test_quality_gate_rejects_missing_invalid_and_changing_data(self):
        plan = ident.Experiment()
        data = ident.simulate(plan, ident.Plant(), seed=6)
        for field, value in (('time_us', 0), ('sequence', 999), ('angle_rad', np.nan),
                             ('enabled', 0), ('faulted', 1), ('saturated', 1), ('lost', 1), ('kp', 1)):
            with self.subTest(field=field):
                broken = {k: v.copy() for k, v in data.items()}
                broken[field][100] = value
                with self.assertRaises(ValueError):
                    ident.validate_data(broken, plan)

    def test_reject_truncated_shifted_and_duplicate_timestamps(self):
        plan = ident.Experiment()
        data = ident.simulate(plan, ident.Plant(), seed=12)
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            ident.validate_data({k: v[:-1] for k, v in data.items()}, plan)
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            ident.validate_data({k: v[:-plan.sample_hz] for k, v in data.items()}, plan)
        shifted = {k: v.copy() for k, v in data.items()}
        shifted['time_us'] += 2000
        with self.assertRaisesRegex(ValueError, 'zero'):
            ident.validate_data(shifted, plan)
        duplicated = {k: v.copy() for k, v in data.items()}
        duplicated['time_us'][100] = duplicated['time_us'][99]
        with self.assertRaisesRegex(ValueError, 'timing'):
            ident.validate_data(duplicated, plan)

    def test_identification_recovers_known_plant_and_predicts_independent_run(self):
        plan, plant = ident.Experiment(), ident.Plant()
        train = ident.simulate(plan, plant, seed=7)
        test_plan = ident.validation_plan(plan)
        validation = ident.simulate(test_plan, plant, seed=19)
        result = ident.identify(train, plan)
        self.assertAlmostEqual(result['inertia_kg_m2'], plant.inertia_kg_m2, delta=.002)
        self.assertAlmostEqual(result['viscous_nm_s_rad'], plant.viscous_nm_s_rad, delta=.03)
        self.assertAlmostEqual(result['gravity_slope_nm_rad'], plant.gravity_slope_nm_rad, delta=.15)
        self.assertLessEqual(abs(result['delay_samples'] - plant.delay_samples), 1)
        score, prediction = ident.validate_model(result, validation, test_plan)
        self.assertTrue(score['passed'])
        self.assertGreater(score['r2'], .95)
        self.assertLess(score['nrmse'], .1)
        self.assertEqual(len(prediction), len(validation['angle_rad']))
        self.assertFalse(np.array_equal(train['perturbation_nm'], validation['perturbation_nm']))

    def test_independent_validation_rejects_wrong_mechanics(self):
        plan = ident.Experiment()
        model = ident.identify(ident.simulate(plan, ident.Plant(), seed=17), plan)
        test_plan = ident.validation_plan(plan)
        other = ident.Plant(inertia_kg_m2=.09, viscous_nm_s_rad=.3)
        score, _ = ident.validate_model(model, ident.simulate(test_plan, other, seed=18), test_plan)
        self.assertFalse(score['passed'])

    def test_different_inertias_and_delays_are_not_hardcoded(self):
        plan = ident.Experiment()
        for plant in (ident.Plant(inertia_kg_m2=.015, viscous_nm_s_rad=.04, delay_samples=0),
                      ident.Plant(inertia_kg_m2=.04, viscous_nm_s_rad=.15, delay_samples=4)):
            with self.subTest(plant=plant):
                model = ident.identify(ident.simulate(plan, plant, seed=22), plan)
                self.assertAlmostEqual(model['inertia_kg_m2'], plant.inertia_kg_m2, delta=.002)
                self.assertAlmostEqual(model['viscous_nm_s_rad'], plant.viscous_nm_s_rad, delta=.03)
                self.assertEqual(model['delay_samples'], plant.delay_samples)

    def test_artifacts_are_marked_simulation_and_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'run'
            ident.create_run(path, ident.Experiment(), ident.Plant(), seed=8)
            manifest = json.loads((path / 'manifest.json').read_text())
            self.assertEqual(manifest['source'], 'SIMULATION_ONLY')
            self.assertFalse(manifest['hardware_ready'])
            with (path / 'train.csv').open() as file:
                self.assertGreater(len(list(csv.DictReader(file))), 1000)
            with self.assertRaises(FileExistsError):
                ident.create_run(path, ident.Experiment(), ident.Plant(), seed=8)
            report = ident.analyze_run(path)
            self.assertEqual(report['source'], 'SIMULATION_ONLY')
            self.assertIn('validation', report)
            manifest['simulated_truth'] = {'inertia_kg_m2': 9999, 'delay_samples': 99}
            (path / 'manifest.json').write_text(json.dumps(manifest))
            self.assertEqual(ident.analyze_run(path)['model'], report['model'],
                             'fitter must not copy simulated truth from the manifest')
            manifest['source'] = 'HARDWARE'
            (path / 'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'SIMULATION_ONLY'):
                ident.analyze_run(path)


if __name__ == '__main__':
    unittest.main()
