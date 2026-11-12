"""双口记录器的离线回归测试，不打开串口。"""
import importlib.util
import math
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robots/sentinel/host'))
SPEC = importlib.util.spec_from_file_location('record_main_dual',
    ROOT / 'robots/sentinel/host/sentinel_record_main_dual.py')
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


def status(error_deg, **changes):
    domain = dict(domain='main_yaw', execute=True, measurement_valid=True,
        mode='position', adopted_frame='local_inertial', measured_frame='local_inertial',
        target=math.radians(error_deg), measurement=0)
    domain.update(changes)
    return {'t_ms': 100, 'page1': {'clock_ms': 123, 'config_epoch': 1,
        'reference_epoch': 0, 'domains': [domain]}}


def counters():
    return {'windows': 0, 'errors': 0, 'flushed_windows': 0, 'flush_errors': 0}


class TriggerTests(unittest.TestCase):
    def test_threshold_and_both_signs(self):
        self.assertIsNone(recorder.main_error_trigger(status(4.9), 5))
        for value in (5, -5, 10, -10):
            hit = recorder.main_error_trigger(status(value), 5)
            self.assertAlmostEqual(hit['error_deg'], value)
            self.assertEqual(hit['reference_epoch'], 0)

    def test_invalid_or_different_frame_not_triggered(self):
        for changes in ({'execute': False}, {'measurement_valid': False},
                {'mode': 'none'}, {'measured_frame': 'joint'},
                {'measurement': None}, {'target': float('nan')}):
            self.assertIsNone(recorder.main_error_trigger(status(20, **changes), 5))

    def test_only_latest_trigger_and_recovery_clears_it(self):
        pending = queue.Queue(maxsize=1)
        recorder.replace_trigger(pending, {'host_trigger_t_s': 1})
        recorder.replace_trigger(pending, {'host_trigger_t_s': 2})
        self.assertEqual(pending.qsize(), 1)
        self.assertEqual(pending.get_nowait()['host_trigger_t_s'], 2)
        recorder.replace_trigger(pending, {'host_trigger_t_s': 3})
        recorder.replace_trigger(pending, None)
        self.assertTrue(pending.empty())

    def test_stale_trigger_does_not_start_capture(self):
        pending = queue.Queue(maxsize=1)
        pending.put({'host_trigger_t_s': -2})
        client = Mock(pending_run=None)
        with tempfile.TemporaryDirectory() as folder:
            recorder.capture_windows(client, Path(folder), time.monotonic(), .02,
                500, threading.Event(), counters(), pending, 5)
        client.capture.assert_not_called()

    def test_trigger_captures_requested_larger_window(self):
        pending = queue.Queue(maxsize=1)
        trigger = {'host_trigger_t_s': 0, 'error_deg': 9}
        pending.put(trigger)
        stop = threading.Event()
        client = Mock(pending_run=None)
        # Stop after the first trigger; no hardware or raw file is required.
        def fail(*args, **kwargs):
            stop.set()
            raise TimeoutError('mock transport failure')
        client.capture.side_effect = fail
        client.last_request = {'phase': '启动窗口'}
        results = counters()
        with tempfile.TemporaryDirectory() as folder:
            recorder.capture_windows(client, Path(folder), time.monotonic(), 1,
                500, stop, results, pending, 5)
            text = (Path(folder) / 'control-windows.jsonl').read_text()
            self.assertIn('"error_deg": 9', text)
        client.capture.assert_called_once_with('control', 500000, domain=4)
        self.assertEqual(results['errors'], 1)

    def test_stop_uses_owned_run_not_zero(self):
        client = Mock(pending_run=123)
        client.exchange.side_effect = [({'run': 123, 'state': 1}, []),
            ({'run': 123, 'state': 2}, [])]
        results = counters()
        recorder.finish_pending(client, results)
        self.assertEqual(client.exchange.call_args.args[0], recorder.raw.request(2, 123))
        self.assertEqual(results['flush_errors'], 0)

    def test_frozen_and_full_not_drained(self):
        for state in (2, 3):
            client = Mock(pending_run=123)
            client.exchange.return_value = ({'run': 123, 'state': state}, [])
            recorder.finish_pending(client, counters())
            self.assertEqual(client.exchange.call_count, 1)
            self.assertIsNone(client.pending_run)


if __name__ == '__main__':
    unittest.main()
