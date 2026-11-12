"""Closed-loop fitting must predict held-out data without bridging missing samples."""
import unittest

import numpy as np

from sentinel_pitch_position_fit import fit_rows, simulate, windows


class PositionFitTests(unittest.TestCase):
    def test_held_out_free_simulation_recovers_known_closed_loop(self):
        t = np.arange(700)*.006
        u = .138 + .01*np.where((np.arange(700)//50)%2, 1., -1.)
        y = np.zeros(len(t)); y[0] = u[0]-.002
        for k in range(1, len(t)):
            y[k] = .8*y[k-1] + .2*(u[k-1]-.002)
        rows = [dict(kind='control', stream=6, t_s=float(a), pitch_adopted_rad=float(b),
                     pitch_measured_rad=float(c), send_result=1, state_bits=1 << 16, latched=False)
                for a, b, c in zip(t, u, y)]
        report, *_ = fit_rows({'recipe': {'position_amplitude_rad': .01}}, rows)
        self.assertEqual(report['status'], 'validated_closed_loop_fit')
        self.assertGreater(report['selected']['validation_free_run']['r2'], .99)
        self.assertAlmostEqual(report['selected']['dc_gain'], 1., delta=.02)
        self.assertFalse(report['deployable_gains'])

    def test_gap_breaks_lag_windows_and_free_simulation(self):
        t = np.r_[np.arange(100)*.006, 1.+np.arange(100)*.006]
        u = np.sin(t); y = np.cos(t)
        valid = {k for k, _ in windows(u, y, t, 2, 1, .006)}
        self.assertNotIn(100, valid); self.assertNotIn(101, valid)
        predicted, mask = simulate(u, y, t, np.array([.5, .2, .1, .1]), 2, 1, .006)
        self.assertFalse(mask[100]); self.assertFalse(mask[101]); self.assertTrue(mask[102])
        self.assertEqual(predicted[100], y[100])

    def test_torque_only_recording_is_rejected(self):
        with self.assertRaises(ValueError):
            fit_rows({'recipe': {'amplitude_nm': .05}}, [])


if __name__ == '__main__':
    unittest.main()
