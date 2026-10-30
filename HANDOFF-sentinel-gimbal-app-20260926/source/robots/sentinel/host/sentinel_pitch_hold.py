"""Pitch hold runner: bounded acquisition, no gain writes or automatic tuning.

Only --backend uses the software plant. Serial operation requires explicit safety
acknowledgments and limits. Stopping removes torque: mechanical support is essential.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import select
import signal
import sys
import threading
import time
from contextlib import ExitStack
from dataclasses import asdict
from pathlib import Path

import sentinel_client as wire
from sentinel_async import AsyncSentinelClient
from sentinel_pitch_hold_model import (
    HoldEnvelope,
    PhysicalBounds,
    PitchHoldSupervisor,
    SupervisorConfig,
    SupervisorLimits,
    TimingBudget,
)

SOURCE = Path(__file__).resolve().parents[3]
ROOT = SOURCE.parents[1]

WARNING = 'STOP REMOVES HOLDING TORQUE. Support/brake must prevent gravity drop.'


class AcquisitionLog:
    """Bounded async writer; queue or disk failure becomes a visible stop reason."""
    def __init__(self, path):
        with ExitStack() as acquisition:
            self.file = acquisition.enter_context(Path(path).open('x', encoding='utf-8'))
            self._files = acquisition.pop_all()  # Transfer ownership to the writer's finally.
        self.queue = queue.Queue(maxsize=1024)
        self.error = ''
        self.done = threading.Event()
        self.worker = threading.Thread(target=self._write, daemon=True)
        try:
            self.worker.start()
        except BaseException:
            self._files.close()
            raise

    def append(self, record):
        if self.error:
            raise RuntimeError(self.error)
        # Serialize on the supervising thread, never the UART target thread.
        line = json.dumps(record, allow_nan=False) + '\n'
        try:
            self.queue.put_nowait(line)
        except queue.Full as error:
            self.error = 'acquisition queue overflow'
            raise RuntimeError(self.error) from error

    def _write(self):
        try:
            while not self.done.is_set() or not self.queue.empty():
                try:
                    line = self.queue.get(timeout=0.02)
                except queue.Empty:
                    continue
                self.file.write(line)
                self.file.flush()
        except Exception as error:
            self.error = f'log write failed: {error}'
        finally:
            try:
                self._files.close()
            except Exception as error:
                self.error = f'log close failed: {error}'

    def close(self):
        self.done.set()
        self.worker.join(timeout=1)
        if self.worker.is_alive():
            self.error = 'log writer did not finish; log incomplete'
        if self.error:
            raise RuntimeError(self.error)


def other_serial_owners(port):
    import subprocess
    result = subprocess.run(['fuser', str(port)], capture_output=True, text=True, timeout=2)
    if result.returncode not in (0, 1) or (result.returncode == 1 and result.stderr.strip()):
        raise RuntimeError(f'cannot establish serial ownership: {result.stderr.strip()}')
    return set(result.stdout.split()) - {str(os.getpid())}


class ExclusiveSerial(wire.SerialTransport):
    """Linux TIOCEXCL plus pre/post-open busy check, not just an advisory lock."""
    def __init__(self, port):
        import fcntl
        import termios
        if other_serial_owners(port):
            raise RuntimeError('serial is already open; close the existing console first')
        super().__init__(str(port), baudrate=921600)
        try:
            fcntl.ioctl(self._serial.fileno(), termios.TIOCEXCL)
            if other_serial_owners(port):
                raise RuntimeError('another process acquired serial during open')
        except BaseException:
            self.close()
            raise


class Keys:
    """Nonblocking single keys on a TTY; stdin EOF is a stop, never unattended hold."""
    def __enter__(self):
        self.saved = None
        if sys.stdin.isatty():
            import termios
            import tty
            self.saved = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        return self

    def read(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            char = os.read(sys.stdin.fileno(), 1).decode('ascii', errors='replace')
            return char if char else 'q'
        return ''

    def __exit__(self, exc_type, exc_value, traceback):
        if self.saved is not None:
            import termios
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self.saved)


def checked_result(future):
    result = future.result(timeout=1)
    if result.get('result') != 0:
        raise RuntimeError(f'request rejected: {result}')
    return result


def shutdown(peer, guard):
    """Report revocation ACK separately from fresh disabled telemetry. Never claim
    physical standstill from an ACK. Peer.close is always attempted by caller.
    """
    errors = []
    if guard is not None:
        try:
            guard.stop()
        except Exception as error:
            errors.append(str(error))
    report = {'revoke': 'unknown', 'disabled_feedback': False, 'control_inactive': False, 'errors': errors}
    try:
        checked_result(peer.authorize(False, 'maintenance'))
        report['revoke'] = 'accepted'
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            status = peer.status(1).result(timeout=0.15)
            report['control_inactive'] = all(not d['execute'] for d in status['domains'])
            page = peer.diagnostics(1).result(timeout=0.15)
            device = next(d for d in page['devices'] if d['domain'] == 'pitch')
            if (device.get('online') and device.get('age_ms') is not None
                    and 0 <= device['age_ms'] <= 100 and 'enabled' in device and not device['enabled']):
                report['disabled_feedback'] = True
            if report['disabled_feedback'] and report['control_inactive']:
                break
    except Exception as error:
        errors.append(str(error))
    return report


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    link = p.add_mutually_exclusive_group(required=True)
    link.add_argument('--backend', type=Path)
    link.add_argument('--serial', type=Path)
    p.add_argument('--arm', action='store_true', help='explicitly authorize this runner to send Pitch position targets')
    p.add_argument('--support-ready', action='store_true')
    p.add_argument('--accept-release', action='store_true', help=WARNING)
    p.add_argument('--demo', action='store_true', help='software plant only; scripted segments')
    p.add_argument('--output', type=Path, required=True, help='new JSONL file; existing files are never overwritten')
    for name in ('physical-low', 'physical-high', 'max-temperature', 'max-torque', 'max-speed', 'max-error', 'duration'):
        p.add_argument('--' + name, type=float)
    p.add_argument('--excursion', type=float, default=0.005, help='maximum absolute deviation from initial measured position, <=0.01 rad')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not args.arm:
        p.error('--arm is required; without it no connection is opened')
    if args.serial and (not args.support_ready or not args.accept_release or args.demo):
        p.error('serial requires --support-ready --accept-release, and forbids --demo')
    defaults = {'physical_low': -0.3, 'physical_high': 0.4, 'max_temperature': 55, 'max_torque': 2,
                'max_speed': 0.6, 'max_error': 0.03, 'duration': 2}
    for name, default in defaults.items():
        if getattr(args, name) is None:
            if args.serial:
                p.error(f'serial requires explicit --{name.replace("_", "-")}')
            setattr(args, name, default)
        if not math.isfinite(getattr(args, name)):
            p.error(f'{name} must be finite')
    if not math.isfinite(args.excursion) or not 0 < args.excursion <= 0.01:
        p.error('excursion must be finite and in (0, 0.01] rad')
    try:
        limits = SupervisorLimits(max_temperature_c=args.max_temperature, max_effort_nm=args.max_torque,
            max_speed_rad_s=args.max_speed, max_tracking_error_rad=args.max_error, max_run_s=args.duration)
        physical = PhysicalBounds(args.physical_low, args.physical_high)
    except ValueError as error:
        p.error(str(error))
    # Request ordinary shutdown before the independent hard run watchdog. This
    # reserves time, rather than extending/ignoring the watchdog at normal expiry.
    normal_run_s = args.duration - min(0.05, args.duration / 4)
    peer = guard = log = None
    stop_requested = threading.Event()
    previous_signals = {}
    report = None
    failure = None
    authorization_attempted = False
    ever_holding = False
    operator_stop = False
    try:
        log = AcquisitionLog(args.output)
        transport = (wire.ProcessTransport(args.backend, extra_args=['--plant', '--fixture', 'gimbal'])
                     if args.backend else ExclusiveSerial(args.serial))
        target_hz = 200 if args.backend else 50
        peer = AsyncSentinelClient(transport, target_hz=target_hz, fast_hz=50)
        hello = peer.hello().result(timeout=1)
        if hello.get('identity') != 'sentinel_app' or not hello.get('startable'):
            raise RuntimeError(f'wrong/not-startable APP identity: {hello}')
        if args.serial:
            devices = peer.diagnostics(1).result(timeout=1)['devices']
            pitch = next(d for d in devices if d['domain'] == 'pitch')
            expected = {'installed': True, 'bus': 2, 'device_id': 1, 'feedback_id': 0x11, 'protocol': 128}
            if hello.get('config_kind') != 2 or any(pitch.get(k) != v for k, v in expected.items()):
                raise RuntimeError(f'not the configured real CAN2 Pitch topology: {pitch}')
        # Fresh response in this session, before authorization or any target.
        timing_page = peer.diagnostics(0).result(timeout=1)
        if (timing_page.get('session') != hello.get('session') or timing_page.get('page') != 0
                or not isinstance(timing_page.get('received_at_monotonic'), (int, float))
                or not 0 <= time.monotonic() - timing_page['received_at_monotonic'] <= 0.12):
            raise RuntimeError('fresh firmware timing diagnostics required')
        timing = TimingBudget(timing_page.get('target_ttl_ms'), timing_page.get('link_ttl_ms'), 1 / target_hz)
        baseline_parameters = peer.read_all_params().result(timeout=3)
        page = peer.status(0).result(timeout=1)
        center = page['joints']['pitch']['angle_rad']
        if not math.isfinite(center):
            raise RuntimeError('initial position is not finite')
        envelope = HoldEnvelope(center - args.excursion, center + args.excursion)
        if envelope.low - physical.low < 0.02 or physical.high - envelope.high < 0.02:
            raise RuntimeError('initial posture too close to a mechanical stop; reposition while supported')
        config = SupervisorConfig(physical, envelope, timing, limits,
                                  shared_slow_lane=hello.get('config_kind') == 2)
        log.append({'event': 'begin', 'hello': hello, 'config': asdict(config),
                    'parameters': baseline_parameters, 'center_rad': center, 'warning': WARNING,
                    'timing_budget_s': timing.budget_s, 'normal_run_s': normal_run_s})
        print(WARNING, flush=True)
        print(f'Run budget: {args.duration:g}s, excursion: +/-{args.excursion:g} rad. No gain writes.', flush=True)
        print('Keys: + / - = 0.001 rad step; n = new segment; SPACE / Esc / q / Ctrl-C = stop (torque off).', flush=True)
        # Re-read immediately before authorization: parameter capture may have
        # taken long enough for the original evidence to expire or TTL to change.
        verify_timing = peer.diagnostics(0).result(timeout=1)
        if (verify_timing.get('session') != hello.get('session') or verify_timing.get('page') != 0
                or verify_timing.get('target_ttl_ms') != timing.target_ttl_ms
                or verify_timing.get('link_ttl_ms') != timing.link_ttl_ms
                or not isinstance(verify_timing.get('received_at_monotonic'), (int, float))
                or not 0 <= time.monotonic() - verify_timing['received_at_monotonic'] <= 0.12):
            raise RuntimeError('firmware timing changed or became stale before authorization')
        # Explicit --arm plus physical DR16 maintenance authorization are both required.
        authorization_attempted = True  # A lost ACK may still mean it was granted.
        checked_result(peer.authorize(True, 'maintenance'))
        peer.status(0).result(timeout=1)
        peer.status(1).result(timeout=1)
        guard = PitchHoldSupervisor(peer, config)
        start = time.monotonic()
        guard.start()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_signals[signum] = signal.getsignal(signum)
            signal.signal(signum, lambda *_: stop_requested.set())
        segment = 0
        goal = center
        with Keys() as keys:
            while not stop_requested.is_set():
                now = time.monotonic()
                if now - start >= normal_run_s:
                    break
                guard.tick()
                if guard.is_faulted:
                    raise RuntimeError(guard.fault_reason)
                key = '' if args.demo else keys.read()
                if key in ('q', ' ', '\x1b', '\x03'):
                    operator_stop = True
                    break
                if guard.is_holding:
                    if not ever_holding:
                        print(f'HOLDING confirmed at {guard.actual_pitch_rad:.6f} rad', flush=True)
                    ever_holding = True
                    if args.demo and segment == 0:
                        segment = 1
                    elif args.demo and segment == 1 and now - start > args.duration / 2:
                        segment = 2
                        goal = center + min(0.002, args.excursion / 2)
                        guard.set_goal(goal, str(segment))
                    elif key in ('+', '-'):
                        proposed = goal + (0.001 if key == '+' else -0.001)
                        if envelope.low <= proposed <= envelope.high:
                            goal = proposed
                            guard.set_goal(goal, str(segment))
                        else:
                            print('Rejected: step outside initial-position excursion envelope', flush=True)
                    elif key == 'n':
                        segment += 1
                        guard.set_goal(goal, str(segment))
                snap = peer.snapshot()
                log.append({'event': 'sample', 'monotonic_s': now, 'segment': segment, 'state': guard.state.value,
                    'target_rad': guard.current_commanded_rad, 'actual_rad': guard.actual_pitch_rad,
                    'velocity_rad_s': guard.actual_pitch_rate, 'status': snap['status'], 'diagnostics': snap['diagnostics'],
                    'last_target_ack': snap['last_target_ack'], 'target_rejections': snap.get('target_rejections')})
                time.sleep(0.005)
        if not ever_holding and not operator_stop and not stop_requested.is_set():
            raise RuntimeError('Holding was not confirmed before the run deadline')
    except BaseException as error:
        failure = f'{type(error).__name__}: {error}'
        print(f'FAILED: {failure}', file=sys.stderr, flush=True)
    finally:
        if guard is not None and guard.is_faulted:
            failure = failure or guard.fault_reason or 'supervisor faulted'
        if peer is not None:
            try:
                if authorization_attempted:
                    report = shutdown(peer, guard)
                else:
                    # Wrong/unvalidated devices must receive no control mutations.
                    report = {'revoke': 'not_requested', 'disabled_feedback': False,
                              'control_inactive': False, 'errors': []}
                report['holding_established'] = ever_holding
                report['software_backend'] = bool(args.backend)
            except Exception as error:
                failure = failure or f'shutdown failed: {error}'
            finally:
                try:
                    peer.close()
                except Exception as error:
                    failure = failure or f'link close failed: {error}'
        if log is not None:
            try:
                log.append({'event': 'end', 'error': failure, 'stop_report': report})
            except Exception as error:
                failure = failure or str(error)
            finally:
                try:
                    log.close()
                except Exception as error:
                    failure = failure or str(error)
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)
    print(json.dumps({'error': failure, 'stop_report': report}), flush=True)
    # The native plant intentionally reports enabled constantly. Its app-level
    # nonexecution can be verified; hardware disable feedback cannot be fabricated.
    stopped = report and report['control_inactive'] and (bool(args.backend) or report['disabled_feedback'])
    return 0 if not failure and stopped and report is not None and not report['errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
