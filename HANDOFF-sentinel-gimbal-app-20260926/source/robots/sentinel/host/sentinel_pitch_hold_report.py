"""Read-only segment statistics; can run separately while holding continues."""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Moments:
    n: int = 0
    mean: float = 0
    m2: float = 0
    sum_sq: float = 0
    peak: float = 0

    def add(self, value):
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        self.m2 += delta * (value - self.mean)
        self.sum_sq += value * value
        self.peak = max(self.peak, abs(value))

    def summary(self):
        if not self.n:
            return {'samples': 0}
        return {'samples': self.n, 'mean': self.mean, 'std': math.sqrt(max(0, self.m2 / self.n)),
                'rms': math.sqrt(self.sum_sq / self.n), 'peak_abs': self.peak}


def summarize(path, *, live=False):
    segments = {}
    last = {}
    complete = False
    stop_report = None
    failure = None
    with Path(path).open(encoding='utf-8') as stream:
        for ordinal, line in enumerate(stream, 1):
            if not line.endswith('\n'):
                if live:
                    break
                raise ValueError(f'incomplete line {ordinal}; use --live only for an active writer')
            try:
                row = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f'invalid JSON at line {ordinal}: {error}') from error
            if not isinstance(row, dict):
                raise ValueError(f'expected record object at line {ordinal}')
            if row.get('event') == 'end':
                complete = True
                stop_report = row.get('stop_report')
                failure = row.get('error')
            if row.get('event') != 'sample' or row.get('state') != 'HOLDING':
                continue
            segment = str(row['segment'])
            page = row.get('status', {}).get('0', {})
            joint = page.get('joints', {}).get('pitch', {})
            identity = (page.get('session'), page.get('config_epoch'), page.get('reference_epoch'), page.get('sequence'))
            if any(x is None for x in identity):
                raise ValueError(f'missing sample identity at line {ordinal}')
            if last.get(segment) == identity:
                continue  # repeated reads of one cached page must not bias statistics
            last[segment] = identity
            actual = joint.get('angle_rad')
            target = row.get('target_rad')
            rate = joint.get('rate_rad_s')
            if (not joint.get('position_valid') or not joint.get('velocity_valid')
                    or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (target, actual, rate))):
                raise ValueError(f'invalid holding sample at line {ordinal}')
            if segment not in segments:
                segments[segment] = {'error_rad': Moments(), 'angle_rad': Moments(), 'rate_rad_s': Moments()}
            stats = segments[segment]
            stats['error_rad'].add(target - actual)
            stats['angle_rad'].add(actual)
            stats['rate_rad_s'].add(rate)
    if not complete and not live:
        raise ValueError('log has no end record; stop status unknown (or use --live for an active run)')
    return {'complete': complete, 'run_error': failure, 'stop_report': stop_report,
            'segments': {name: {metric: value.summary() for metric, value in stats.items()}
                         for name, stats in segments.items()},
            'note': 'Protocol/software measurements only; not a stability certificate or gain recommendation.'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args(argv)
    print(json.dumps(summarize(args.log, live=args.live), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
