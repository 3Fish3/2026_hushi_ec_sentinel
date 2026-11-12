"""Offline mock-only tests; no hardware/serial transport."""
from concurrent.futures import Future
from pathlib import Path
import struct
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robots/sentinel/host'))
import sentinel_raw as raw
from sentinel_short_capture import ShortCaptureAdapter


def record(seq, kind, *, axis=6, run=42, source=7, sampled=1001000, lost=0):
    header = struct.pack('<4B7I', 1, kind, axis, 1, seq, source, sampled,
                         sampled + 9, lost, 0xffffffff, run)
    data = bytearray(32)
    if kind == 6:
        struct.pack_into('<I5ffI', data, 0, 0, 1., 2., 3., 4., float('nan'), .001, 9)
    return header + data


class Transport:
    def __init__(self):
        self.calls = []
        self.threads = []
        self.opened = False
        self.window = 0
        self.now = 0.0
        self.rows = [record(0, 6), record(1, 7)]
        self.preflight = {}
        self.ack = {}
        self.final = {}
        self.page = {}
        self.open_delay = 0
        self.future = False
        self.wire = False
        self.failure = None

    def __call__(self, payload):
        self.threads.append(threading.get_ident())
        v, op, profile, axis, run, arg = struct.unpack('<4B2I', payload)
        self.calls.append((op, run, profile, arg, axis))
        status = dict(op=op, state=0, run=0, profile=1, total=0,
                      start_us=0, end_us=0, lost=0, layout_id=raw.LAYOUT, capacity=512)
        batch = []
        if op == 1:
            self.opened = True
            self.window = arg
            self.now += self.open_delay
            status.update(state=1, run=run, profile=profile, start_us=1000000,
                          end_us=1000000)
            status.update(self.ack)
        elif self.opened:
            status.update(state=2, run=42, profile=4, total=len(self.rows),
                          start_us=1000000, end_us=1000000 + self.window)
            status.update(self.final)
            if op == 3:
                batch = self.rows[arg:arg+3]
                status.update(self.page)
        else:
            status.update(self.preflight)
        result = (status, batch)
        if self.wire:
            result = struct.pack('<4BI2BH5I', 1, op, status.get('error', 0), status['state'],
                                 status['run'], status['profile'], len(batch), status['total'],
                                 status['start_us'], status['end_us'], status['lost'],
                                 status['layout_id'], status['capacity']) + b''.join(batch)
        if self.future:
            future = Future()
            if self.failure:
                future.set_exception(self.failure)
            else:
                future.set_result(result)
            return future
        return result


class ShortCaptureTests(unittest.TestCase):
    def adapter(self, transport=None, **kwargs):
        transport = transport or Transport()
        adapter = ShortCaptureAdapter(transport, scheduling_margin_us=kwargs.pop('margin', 10000),
                                      identities={'boot_id': 'observed-boot', 'firmware_id': 'observed-build'},
                                      clock=lambda: transport.now, sleep=lambda _: None,
                                      max_status_polls=3, **kwargs)
        self.addCleanup(adapter.close)
        return adapter, transport

    def start(self, adapter):
        return adapter.prepare_start(6, 42, 180000).result(timeout=2)

    def test_no_open_during_construction_or_hold(self):
        adapter, transport = self.adapter()
        self.assertEqual(transport.calls, [])
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(RuntimeError):
                adapter.finish(Path(directory) / 'x.sraw').result()
        self.assertEqual(transport.calls, [])
        receipt = self.start(adapter)
        self.assertEqual([c[0] for c in transport.calls], [0, 1])
        self.assertEqual(transport.calls[1], (1, 42, 4, 190000, 6))
        self.assertTrue(receipt.ready)
        self.assertEqual(receipt.row_budget, 380)
        self.assertEqual(receipt.identities['boot_id'], 'observed-boot')
        self.assertNotIn(threading.get_ident(), transport.threads)

    def test_invalid_inputs_do_not_exchange(self):
        for axis, run, duration in [(4,42,1), (7,42,1), (True,42,1),
                                     (6,0,1), (6,-1,1), (6,2**32,1),
                                     (6,42,0), (6,42,180001), (6,42,1.2)]:
            with self.subTest(axis=axis, run=run, duration=duration):
                adapter, transport = self.adapter()
                with self.assertRaises(ValueError):
                    adapter.prepare_start(axis, run, duration).result()
                self.assertEqual(transport.calls, [])

    def test_margin_capacity_ceiling_and_strict_window(self):
        adapter, _ = self.adapter(margin=75999)
        receipt = self.start(adapter)
        self.assertEqual((receipt.window_us, receipt.row_budget), (255999, 512))
        adapter, transport = self.adapter(margin=76000)
        with self.assertRaises(ValueError):
            self.start(adapter)
        self.assertEqual(transport.calls, [])
        adapter, _ = self.adapter(margin=1)
        self.assertEqual(self.start(adapter).row_budget, 362)
        for margin in (0, -1, True, 1.5, 256000):
            with self.assertRaises(ValueError):
                self.adapter(margin=margin)

    def test_existing_active_frozen_runs_never_replaced(self):
        for preflight in ({'state': 1}, {'state': 2}, {'state': 3}, {'total': 2}, {'run': 1}):
            adapter, transport = self.adapter()
            transport.preflight = preflight
            with self.assertRaises(RuntimeError):
                self.start(adapter)
            self.assertEqual([c[0] for c in transport.calls], [0])

    def test_ack_rejection_and_mismatches(self):
        for patch in ({'error': 1}, {'run': 8}, {'profile': 3}, {'op': 3}, {'lost': 1},
                      {'state': 2}, {'capacity': 1024}, {'layout_id': 0},
                      {'decoder_errors': 1}, {'drops': 1}, {'total': 379},
                      {'end_us': 1010001}):
            with self.subTest(patch=patch):
                adapter, transport = self.adapter()
                transport.ack = patch
                with self.assertRaises(ValueError):
                    self.start(adapter)
                with self.assertRaises(RuntimeError):
                    self.start(adapter)
                self.assertEqual([c[0] for c in transport.calls], [0, 1])

    def test_late_ack(self):
        adapter, transport = self.adapter()
        transport.open_delay = .010001
        with self.assertRaisesRegex(ValueError, 'late'):
            self.start(adapter)

    def test_future_and_wire_export_preserves_nan_bytes(self):
        for wire in (False, True):
            adapter, transport = self.adapter()
            transport.future, transport.wire = True, wire
            self.start(adapter)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'capture.sraw'
                metadata = adapter.finish(path).result(timeout=2)
                self.assertFalse(metadata['synthetic'])
                self.assertEqual(metadata['provenance'], 'REAL_OFFLINE')
                loaded, rows = raw.load(path)
                self.assertEqual(loaded['run'], 42)
                self.assertEqual(len(rows), 2)
                blob = path.read_bytes()
                header_size = struct.unpack_from('<I', blob, 8)[0]
                self.assertEqual(blob[12+header_size:], b''.join(transport.rows))
                with self.assertRaises(FileExistsError):
                    adapter.finish(path).result(timeout=2)
            self.assertTrue(all(c[0] in (0,1,3) for c in transport.calls))

    def test_future_exception_and_timeout(self):
        adapter, transport = self.adapter()
        transport.future, transport.failure = True, RuntimeError('transport failed')
        with self.assertRaisesRegex(RuntimeError, 'transport failed'):
            self.start(adapter)
        pending = Future()
        adapter, _ = self.adapter(exchange_timeout_s=.001)
        adapter.exchange = lambda _: pending
        with self.assertRaises(TimeoutError):
            self.start(adapter)

    def test_bounded_polling_no_completion(self):
        adapter, transport = self.adapter()
        self.start(adapter)
        transport.final = {'state': 1}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'x.sraw'
            with self.assertRaises(TimeoutError):
                adapter.finish(path).result(timeout=2)
            self.assertFalse(path.exists())
        self.assertEqual([c[0] for c in transport.calls], [0,1,0,0,0])

    def test_stop_logging_is_only_op2_not_motor_stop(self):
        adapter, transport = self.adapter()
        self.start(adapter)
        self.assertEqual(adapter.stop_logging().result()['state'], 2)
        self.assertEqual(transport.calls[-1], (2,42,0,0,0))
        adapter.close()
        self.assertEqual([c[0] for c in transport.calls], [0,1,2])

    def test_completion_rejections(self):
        for patch in ({'state': 0}, {'state': 3}, {'run': 8}, {'profile': 3},
                      {'start_us': 1}, {'end_us': 1000010}, {'lost': 1},
                      {'total': 0}, {'total': 3}, {'total': 382}):
            with self.subTest(patch=patch):
                adapter, transport = self.adapter()
                self.start(adapter)
                transport.final = patch
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / 'x.sraw'
                    with self.assertRaises(ValueError):
                        adapter.finish(path).result(timeout=2)
                    self.assertFalse(path.exists())

    def test_full_rows_sequence_pairs_identity_and_decoder_checks(self):
        invalid = [
            [record(1,6), record(2,7)],
            [record(0,6,run=41), record(1,7)],
            [record(0,6,axis=5), record(1,7)],
            [record(0,6,lost=1), record(1,7)],
            [record(0,6), record(1,7,source=8)],
            [record(0,6), record(1,7,sampled=1001001)],
            [record(0,7), record(1,6)],
            [record(0,6)[:-1], record(1,7)],
            [bytes([2])+record(0,6)[1:], record(1,7)],
            [record(0,6,sampled=999999), record(1,7,sampled=999999)],
            [record(0,6), record(1,7), record(2,6), record(3,7)],
        ]
        for rows in invalid:
            with self.subTest(rows=rows):
                adapter, transport = self.adapter()
                self.start(adapter)
                transport.rows = rows
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / 'x.sraw'
                    with self.assertRaises(ValueError):
                        adapter.finish(path).result(timeout=2)
                    self.assertFalse(path.exists())

    def test_valid_multi_page_and_empty_page(self):
        adapter, transport = self.adapter()
        self.start(adapter)
        transport.rows += [record(2,6,source=8,sampled=1002000),
                           record(3,7,source=8,sampled=1002000)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'multi.sraw'
            self.assertEqual(adapter.finish(path).result()['total'], 4)
            self.assertEqual([call[3] for call in transport.calls if call[0] == 3], [0,3])
        adapter, transport = self.adapter()
        self.start(adapter)
        original = transport.__call__
        def empty_page(payload):
            status, rows = original(payload)
            return status, [] if payload[1] == 3 else rows
        adapter.exchange = empty_page
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'empty.sraw'
            with self.assertRaisesRegex(ValueError, 'empty'):
                adapter.finish(path).result()
            self.assertFalse(path.exists())

    def test_wire_refusal_decoder_errors(self):
        adapter, transport = self.adapter()
        transport.wire, transport.ack = True, {'error': 3}
        with self.assertRaises(ValueError):
            self.start(adapter)
        adapter, _ = self.adapter()
        adapter.exchange = lambda _: b'bad'
        with self.assertRaises(ValueError):
            self.start(adapter)

    def test_empty_or_changed_readback(self):
        for patch in ({'total': 4}, {'state': 1}, {'end_us': 1190001}):
            adapter, transport = self.adapter()
            self.start(adapter)
            transport.page = patch
            with tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(ValueError):
                    adapter.finish(Path(directory) / 'x.sraw').result()


if __name__ == '__main__':
    unittest.main()
