"""Finite Profile4 observation adapter; no serial ownership and no motor commands.

Call prepare_start ONLY from HUMAN-confirmed START orchestration, never during
human-hold. All public operations return concurrent.futures.Future; exchange and
Future waits run on our private worker, not the shared UART reader. The supplied
exchange must be nonblocking/bounded itself and return wire reply bytes or
(status_dict, full_64_byte_records), directly or via concurrent.futures.Future.
The parent owns transport serialization, identity acquisition and motor START.
Readiness here is recorder evidence, NOT safety approval or motion admission.
"""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

import sentinel_raw as raw


@dataclass(frozen=True)
class CaptureReceipt:
    axis: int
    run: int
    profile: int
    start_us: int
    duration_us: int
    window_us: int
    scheduling_margin_us: int
    row_budget: int
    ready: bool
    observed_state: int
    observed: dict
    identities: dict
    dispatch_not_after: float


def _integer(name, value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in [{low}, {high}]')
    return value


class ShortCaptureAdapter:
    """One finite run per adapter; construction performs no exchange.

    scheduling_margin_us is explicitly chosen by the orchestrator; it covers
    ACK/START scheduling, not a physical safety threshold. Duration + margin
    must be strictly <256ms and ceil(window/1ms)*2 must fit 512 records.
    identities is caller-observed provenance (e.g. boot_id/firmware_id), not
    synthesized identity. No automatic retries, reset, replacement or clear.
    close() shuts down only this worker; it NEVER stops logging or a motor.
    """
    def __init__(self, exchange, *, scheduling_margin_us, identities,
                 exchange_timeout_s=0.5, poll_interval_s=0.01,
                 max_status_polls=64, clock=time.monotonic, sleep=time.sleep):
        if not callable(exchange):
            raise TypeError('exchange must be callable')
        self.margin = _integer('scheduling_margin_us', scheduling_margin_us, 1, 255999)
        if not isinstance(identities, dict) or not identities:
            raise ValueError('explicit observed identities required')
        # Detach provenance from caller mutations and reject NaN metadata now.
        self.identities = json.loads(json.dumps(identities, allow_nan=False))
        for name, value in [('exchange_timeout_s', exchange_timeout_s),
                            ('poll_interval_s', poll_interval_s)]:
            if not isinstance(value, (int, float)) or not 0 < value < float('inf'):
                raise ValueError(f'{name} must be positive and finite')
        self.exchange_timeout = exchange_timeout_s
        self.poll_interval = poll_interval_s
        self.max_polls = _integer('max_status_polls', max_status_polls, 1, 10000)
        self.exchange, self.clock, self.sleep = exchange, clock, sleep
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='short-capture')
        self._attempted = False
        self._receipt = None
        self._open_identity = None

    def close(self, *, wait=True):
        self._worker.shutdown(wait=wait, cancel_futures=True)

    def prepare_start(self, axis, run, duration_us) -> Future:
        """Explicit START-only logging open. Resolves to CaptureReceipt, no motion."""
        return self._worker.submit(self._prepare_start, axis, run, duration_us)

    def finish(self, path) -> Future:
        """Poll finite completion, validate paired full rows, exclusively save .sraw.

        Resolves to metadata. On any failure no fabricated/partial export is
        written. A failed/late open is never retried or silently replaced.
        """
        return self._worker.submit(self._finish, Path(path))

    def stop_logging(self) -> Future:
        """Only raw op2 for this run. This is NOT an emergency/motor stop."""
        return self._worker.submit(self._stop_logging)

    def _call(self, op, run=0, profile=0, arg=0, axis=0):
        began = self.clock()
        result = self.exchange(raw.request(op, run, profile, arg, axis))
        if isinstance(result, Future):
            result = result.result(timeout=self.exchange_timeout)
        if self.clock() - began > self.exchange_timeout:
            raise TimeoutError('late raw response')
        if isinstance(result, (bytes, bytearray)):
            result = raw.decode_reply(result)
        if not isinstance(result, (tuple, list)) or len(result) != 2:
            raise ValueError('exchange must return reply bytes or (status, records)')
        status, records = result
        if not isinstance(status, dict) or not isinstance(records, (tuple, list)):
            raise ValueError('invalid decoded reply')
        for key in ('op', 'state', 'run', 'profile', 'total', 'start_us',
                    'end_us', 'lost', 'layout_id', 'capacity'):
            _integer(key, status.get(key), 0, 0xffffffff)
        if status.get('error', 0) or status.get('decoder_errors', 0) or status.get('drops', 0):
            raise ValueError('raw decoder/request error or drops')
        if status['op'] != op or status['layout_id'] != raw.LAYOUT or status['capacity'] != 512:
            raise ValueError('raw reply op/layout/capacity mismatch')
        if status['total'] > 512 or status['state'] not in (0, 1, 2, 3):
            raise ValueError('invalid recorder state/count')
        if status['lost']:
            raise ValueError('raw recorder lost rows')
        if op != 3 and records:
            raise ValueError('unexpected rows outside readback')
        return dict(status), list(records)

    def _match(self, status):
        run, start = self._open_identity
        if status['run'] != run or status['profile'] != 4 or status['start_us'] != start:
            raise ValueError('run/profile/start identity mismatch')

    def _prepare_start(self, axis, run, duration_us):
        _integer('axis', axis, 5, 6)
        _integer('run', run, 1, 0xffffffff)
        _integer('duration_us', duration_us, 1, 180000)
        window = duration_us + self.margin
        budget = 2 * ((window + 999) // 1000)
        if window >= 256000 or budget > 512:
            raise ValueError('finite window/margin exceeds 512 rows at 2 rows/ms')
        if self._attempted:
            raise RuntimeError('adapter already attempted a run; no implicit overwrite/retry')
        status, _ = self._call(0)
        if status['state'] != 0 or status['total'] != 0 or status['run'] != 0:
            raise RuntimeError('existing active/frozen run must not be overwritten')
        self._attempted = True  # Even a timeout can mean the device opened the run.
        began = self.clock()
        status, _ = self._call(1, run, 4, window, axis)
        self._open_identity = (run, status['start_us'])
        self._match(status)
        age = raw.delta(status['end_us'], status['start_us'])
        elapsed = (self.clock() - began) * 1e6
        if (status['state'] != 1 or age < 0 or age > self.margin or
                elapsed > self.margin or status['total'] > budget - 2):
            raise ValueError('late/not-ready finite recorder ACK')
        self._receipt = CaptureReceipt(axis, run, 4, status['start_us'], duration_us,
                                       window, self.margin, budget, True,
                                       status['state'], status, dict(self.identities), began + self.margin / 1e6)
        return self._receipt

    def _stop_logging(self):
        if self._open_identity is None:
            raise RuntimeError('no acknowledged run to stop')
        status, _ = self._call(2, self._open_identity[0])
        self._match(status)
        if status['state'] not in (2, 3):
            raise ValueError('logging stop not acknowledged')
        return status

    def _finish(self, path):
        receipt = self._receipt
        if receipt is None:
            raise RuntimeError('no ready capture receipt')
        if path.suffix != '.sraw':
            raise ValueError('output must be .sraw')
        status = None
        for index in range(self.max_polls):
            status, _ = self._call(0)
            self._match(status)
            if status['state'] == 2:
                break
            if status['state'] != 1:
                raise ValueError('finite capture not completed normally (idle/full)')
            if index + 1 < self.max_polls:
                self.sleep(self.poll_interval)
        else:
            raise TimeoutError('no finite capture completion within poll bound')
        elapsed = raw.delta(status['end_us'], receipt.start_us)
        if not receipt.window_us <= elapsed < 256000:
            raise ValueError('incomplete or invalid finite window')
        total = status['total']
        if not total or total % 2 or total > receipt.row_budget:
            raise ValueError('empty/unpaired/over-budget finite capture')
        records = []
        while len(records) < total:
            page, batch = self._call(3, receipt.run, arg=len(records))
            self._match(page)
            if any(page[k] != status[k] for k in ('state', 'total', 'end_us')):
                raise ValueError('frozen readback changed')
            if not 1 <= len(batch) <= min(3, total - len(records)):
                raise ValueError('empty/oversized readback page')
            records.extend(batch)
        pairs = set()
        for index in range(0, total, 2):
            decoded = []
            for seq in (index, index + 1):
                record = records[seq]
                if not isinstance(record, bytes) or len(record) != 64:
                    raise ValueError('readback requires full 64B records')
                row = raw.decode_record(record)  # Fail closed on decoder errors.
                if (row['sequence'] != seq or row['run'] != receipt.run or
                        row['stream'] != receipt.axis or row['lost'] != 0):
                    raise ValueError('row sequence/run/axis/drop mismatch')
                if not 0 <= raw.delta(row['sampled_us'], receipt.start_us) < receipt.window_us:
                    raise ValueError('row sample time outside finite window')
                decoded.append(row)
            if records[index][1] != 6 or records[index + 1][1] != 7:
                raise ValueError('expected paired kinds6+7')
            keys = [tuple(row[k] for k in ('stream', 'source_sequence', 'sampled_us', 'run'))
                    for row in decoded]
            if keys[0] != keys[1] or keys[0] in pairs:
                raise ValueError('missing/duplicate/mismatched Profile4 pair')
            pairs.add(keys[0])
        metadata = dict(status)
        metadata.update(axis=receipt.axis, duration_us=receipt.duration_us,
                        window_us=receipt.window_us, scheduling_margin_us=self.margin,
                        row_budget=receipt.row_budget, identities=receipt.identities,
                        receipt=asdict(receipt), synthetic=False, provenance='REAL_OFFLINE',
                        label='NOT_HARDWARE_VALIDATED', safety_approval=False)
        # save uses exclusive xb; original payloads (including NaN sentinels) are
        # preserved byte-for-byte, never replaced with made-up output numbers.
        raw.save(path, metadata, records)
        return metadata
