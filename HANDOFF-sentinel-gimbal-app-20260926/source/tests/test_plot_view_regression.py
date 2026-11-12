"""Offline regressions for native viewport policy and disconnected samples."""
import math
import unittest
from unittest.mock import MagicMock, patch

import sentinel_console as ui


class PlotViewRegression(unittest.TestCase):
    def console(self, series):
        console = ui.Console.__new__(ui.Console)
        console.job = console.session = None
        console.param_ids = []
        console.assist_fields = {}
        console.focus = 'pitch'
        console.domain_label = MagicMock(return_value='pitch')
        console.history = MagicMock()
        console.history.joint.return_value = None
        console.history.domain.return_value = None
        console.history.series.return_value = series
        console.plot_following = True
        return console

    def dpg(self):
        dpg = MagicMock()
        dpg.get_value.return_value = True
        dpg.get_item_rect_size.return_value = (800, 600)
        dpg.get_axis_limits.return_value = (0., 1.)
        dpg.is_item_hovered.return_value = False
        return dpg

    def limits(self, dpg, tag):
        return [call.args[1:] for call in dpg.set_axis_limits.call_args_list if call.args[0] == tag][-1]

    def test_constant_feedback_centered_without_fake_zero_target(self):
        console = self.console(([4., 5.], [math.nan]*2, [-.262]*2, [math.nan]*2))
        dpg = self.dpg()
        with patch.object(ui, 'dpg', dpg):
            console.refresh()
        low, high = self.limits(dpg, 'tracking_y')
        self.assertAlmostEqual((low + high) / 2, -.262)
        self.assertLess(high - low, .05)

    def test_old_extreme_leaving_window_allows_shrink_and_both_lines_fit(self):
        console = self.console(([0., 30., 31.], [1., .2, .2], [-1., -.26, -.26], [2., .46, .46]))
        dpg = self.dpg()
        with patch.object(ui, 'dpg', dpg):
            console.refresh()
        low, high = self.limits(dpg, 'tracking_y')
        self.assertLess(low, -.26)
        self.assertGreater(high, .2)
        self.assertGreater(low, -.4)
        self.assertLess(high, .4)

    def test_isolated_points_are_visible_without_joining_stale_gaps(self):
        console = self.console(([0., 1., 1., 2., 2.], [.2, math.nan, .2, math.nan, .2],
                                [-.26, math.nan, -.26, math.nan, -.26], [math.nan]*5))
        dpg = self.dpg()
        with patch.object(ui, 'dpg', dpg):
            console.refresh()
        writes = {call.args[0]: call.args[1] for call in dpg.set_value.call_args_list}
        self.assertIn('measured_points', writes)
        self.assertEqual(writes['measured_points'], [[0., 1., 2.], [-.26]*3])
        self.assertEqual(writes['measured'], [[], []], 'must not connect across stale gaps')

    def test_mouse_interaction_releases_axes_and_refresh_does_not_reclaim(self):
        console = self.console(([4., 5.], [.2]*2, [-.26]*2, [.46]*2))
        dpg = self.dpg()
        dpg.is_item_hovered.side_effect = lambda tag: tag == 'tracking'
        with patch.object(ui, 'dpg', dpg):
            console.plot_interaction()
            console.refresh()
        self.assertFalse(console.plot_following)
        self.assertEqual(dpg.set_axis_limits_auto.call_count, 4)
        dpg.set_axis_limits.assert_not_called()
        with patch.object(ui, 'dpg', dpg):
            console.fit_plots()
            console.refresh()
        self.assertTrue(console.plot_following)
        dpg.set_axis_limits.assert_called()

    def test_empty_then_single_value_initializes_on_first_finite_value(self):
        console = self.console(([0.], [math.nan], [math.nan], [math.nan]))
        dpg = self.dpg()
        with patch.object(ui, 'dpg', dpg):
            console.refresh()
        dpg.set_axis_limits.assert_not_called()
        series = console.history.series
        assert isinstance(series, MagicMock)
        series.return_value = ([0.], [math.nan], [-.262], [math.nan])
        with patch.object(ui, 'dpg', dpg):
            console.refresh()
        self.assertLess(self.limits(dpg, 'tracking_y')[0], -.262)
        self.assertGreater(self.limits(dpg, 'tracking_x')[1], 0.)


if __name__ == '__main__':
    unittest.main()
