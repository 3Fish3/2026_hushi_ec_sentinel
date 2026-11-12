"""Headless, explicit-click experiment positioning. No device imports or capture auto-start.
Backend evidence is authoritative. A host acknowledgement is not physical safety evidence.
"""
from __future__ import annotations
from copy import deepcopy
import math
import struct
import time

AXES = {'fire_yaw': 5, 'pitch': 6}
PATTERNS = {'bounded_step': 1, 'trapezoid': 2}
RECIPE_UINTS = ('param_epoch', 'guardian_version', 'config_hash', 'safety_reference_id')
RECIPE_FLOATS = ('center_rad', 'amplitude_rad', 'min_rad', 'max_rad', 'max_velocity_rad_s',
    'max_acceleration_rad_s2', 'max_travel_rad', 'max_output_abs', 'max_output_integral')
RECIPE_TIMES = ('move_us', 'hold_us', 'duration_us', 'heartbeat_timeout_us', 'stop_timeout_us')
POLICY_FLOATS = ('position_tolerance', 'velocity_tolerance', 'positioning_speed_limit')
POLICY_TIMES = ('move_us', 'stable_dwell_us', 'positioning_timeout_us', 'hold_timeout_us', 'feedback_max_age_us')


def uint(value, name):
    if type(value) is not int or not 0 < value <= 0xffffffff:
        raise ValueError(name + ': explicit positive uint32 required')
    return value


def f32(value, name, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(name + ': explicit finite value required')
    try:
        result = struct.unpack('<f', struct.pack('<f', value))[0]
    except (OverflowError, struct.error) as exc:
        raise ValueError(name + ': float32 overflow') from exc
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(name + ': invalid float32')
    return result


def encode_position_policy(policy):
    required = {'human_token', *POLICY_FLOATS, *POLICY_TIMES}
    if set(policy) != required:
        raise ValueError('policy requires exactly: ' + ', '.join(sorted(required)))
    token = uint(policy['human_token'], 'human_token')
    floats = [f32(policy[k], k, True) for k in POLICY_FLOATS]
    times = [uint(policy[k], k) for k in POLICY_TIMES]
    if any(t >= 0x80000000 for t in times):
        raise ValueError('policy times must be below half MCU clock range')
    if policy['stable_dwell_us'] >= policy['positioning_timeout_us']:
        raise ValueError('stable dwell must fit positioning timeout')
    return struct.pack('<I3f5I', token, *floats, *times)


def encode_recipe(recipe):
    required = {'axis', 'pattern', 'experiment_session', 'nonce', 'repeats',
                *RECIPE_UINTS, *RECIPE_FLOATS, *RECIPE_TIMES}
    if set(recipe) != required:
        raise ValueError('recipe requires exactly: ' + ', '.join(sorted(required)))
    axis, pattern = AXES[recipe['axis']], PATTERNS[recipe['pattern']]
    ids = [uint(recipe[k], k) for k in ('experiment_session', 'nonce', *RECIPE_UINTS)]
    floats = [f32(recipe[k], k, k != 'center_rad' and k not in ('min_rad', 'max_rad')) for k in RECIPE_FLOATS]
    if not recipe['min_rad'] < recipe['center_rad'] < recipe['max_rad']:
        raise ValueError('center must be strictly inside supplied recipe limits (not physical evidence)')
    if recipe['amplitude_rad'] > min(recipe['center_rad']-recipe['min_rad'], recipe['max_rad']-recipe['center_rad']):
        raise ValueError('amplitude exceeds supplied recipe limits')
    times = [uint(recipe[k], k) for k in RECIPE_TIMES]
    if any(t >= 0x80000000 for t in times):
        raise ValueError('recipe times must be below half MCU clock range')
    repeats = uint(recipe['repeats'], 'repeats')
    if repeats > 255:
        raise ValueError('repeats exceeds uint8')
    return struct.pack('<4B6I9f5IB3x', 3, 3, axis, pattern, *ids, *floats, *times, repeats)


class ExperimentPositioningController:
    """Single outstanding async request/poll, serviced by the console frame tick.

    capture_start, when supplied by a separately prepared capture owner, must open
    actual raw capture immediately before START and return a Future. No capture
    provider is installed by default: the real GUI truthfully shows unavailable.
    """
    def __init__(self, peer, *, clock=time.monotonic, capture_start=None):
        self.peer, self.clock, self.capture_start = peer, clock, capture_start
        self.recipe = self.policy = self.command = None
        self.hardware_blocked = True
        self.pending = None
        self.pending_kind = None
        self.next_poll = 0
        self.reply = None
        self.binding = None
        self.confirmed = None
        self.prepared_binding = None
        self.prepared_identity = None
        self.pending_binding = None
        self.revision = 0
        self.pending_revision = None
        self.draft_dirty = False
        self.active = False
        self.closed = False
        self.note = 'Hardware blocked; START unavailable without prepared raw capture provider'
        self.requested = None
        self.authorization_invalidated_reason = 'No explicit human confirmation ACK'
        self.capture_pending = None
        self.deferred = None

    def invalidate(self, reason):
        self.confirmed = None
        self.authorization_invalidated_reason = str(reason)
        self.note = str(reason)

    def draft_changed(self, reason):
        self.revision += 1
        self.draft_dirty = True
        self.prepared_binding = None
        self.invalidate(reason)

    def configure(self, recipe, policy, *, hardware_blocked=True):
        command = encode_recipe(recipe)
        encode_position_policy(policy)
        identity = (command, encode_position_policy(policy), bool(hardware_blocked))
        if identity != self.binding:
            self.prepared_binding = None
            if self.active:
                self.invalidate('Active recipe cannot be replaced; STOP and reconnect first')
                raise ValueError(self.note)
            self.invalidate('Recipe/axis/policy changed; PREPARE and human confirmation required')
            self.reply = None
        self.recipe, self.policy, self.command = deepcopy(recipe), deepcopy(policy), command
        self.hardware_blocked = bool(hardware_blocked)
        self.binding = identity
        self.draft_dirty = False

    def _ids(self):
        if self.recipe is None:
            raise ValueError('Load explicit recipe and policy JSON first')
        return (AXES[self.recipe['axis']], PATTERNS[self.recipe['pattern']],
                self.recipe['experiment_session'], self.recipe['nonce'])

    def _ready(self, motion=False):
        if self.closed or self.pending is not None or self.capture_pending is not None:
            raise ValueError('Closed or one experiment operation/poll is pending')
        snap = self.peer.snapshot()
        if not snap.get('connected'):
            self.invalidate('Disconnected; confirmation invalid')
            raise ValueError(self.note)
        self._ids()
        if motion and self.draft_dirty:
            raise ValueError('Draft changed; explicit validation/reload required before any motion command')
        if motion and self.hardware_blocked:
            raise ValueError('Hardware blocked (default); host opt-out is NOT safety evidence')
        return snap

    def _submit(self, kind, future):
        if kind != 'status':
            self.requested = kind
        self.pending_binding = self.binding
        self.pending_revision = self.revision
        self.pending_kind, self.pending = kind, future
        self.note = kind + ' requested; waiting for authoritative backend ACK'
        return future

    def prepare(self):
        self._ready(True)
        self.invalidate('New PREPARE invalidates human confirmation')
        self.active = True
        try:
            return self._submit('prepare', self.peer.experiment_prepare(self.command, hardware_blocked=self.hardware_blocked))
        except Exception:
            self.active = False
            raise

    def arm(self):
        self._ready(True)
        self.invalidate('ARM is not position/confirmation/start')
        return self._submit('arm', self.peer.experiment_arm(*self._ids(), hardware_blocked=self.hardware_blocked))

    def position(self):
        self._ready(True)
        self.invalidate('Position requested; human confirmation remains absent')
        return self._submit('position', self.peer.experiment_position(*self._ids(), self.policy, hardware_blocked=self.hardware_blocked))

    def _evidence(self, stages):
        snap = self.peer.snapshot()
        r = self.reply or {}
        age = self.clock()-r.get('received_at_monotonic', -math.inf)
        max_age = self.policy['feedback_max_age_us']
        if any(type(r.get(k)) is not int or not 0 <= r[k] <= 0xffffffff for k in ('now_us', 'feedback_sample_us')):
            raise ValueError('Missing exact MCU feedback sample clock')
        sample_age = (r['now_us']-r['feedback_sample_us']) & 0xffffffff
        expected = (self.recipe['experiment_session'], self.recipe['nonce'], self.recipe['axis'])
        if (self.draft_dirty or self.prepared_binding != self.binding or not snap.get('connected') or r.get('body_version') != 3 or r.get('result') != 0
            or r.get('reason') != 'none' or r.get('positioning_stage') not in stages
            or (r.get('experiment_session'), r.get('nonce'), r.get('axis')) != expected
            or r.get('session') != snap.get('session')
            or r.get('config_epoch') != snap.get('config_epoch')
            or r.get('reference_epoch') != snap.get('reference_epoch')
            or r.get('positioning_config_epoch') != snap.get('config_epoch')
            or r.get('positioning_reference_epoch') != snap.get('reference_epoch')
            or r.get('human_token') != self.policy['human_token']
            or r.get('param_epoch') != self.recipe['param_epoch']
            or r.get('config_hash') != self.recipe['config_hash']
            or r.get('guardian_version') != self.recipe['guardian_version']
            or not 0 <= age <= min(.1, self.recipe['heartbeat_timeout_us']/3e6) or sample_age > max_age):
            raise ValueError('Missing/stale/changed backend positioning identity or feedback')
        for k in ('feedback_fresh', 'reference_valid', 'limits_valid', 'actuator_ready',
                  'other_axes_excluded', 'link_valid', 'observation_valid', 'stop_policy_verified',
                  'maintenance_authorized', 'operator_approved', 'lease_active', 'guardian_approved',
                  'remote_online', 'hardware_capable_flag'):
            if r.get(k) is not True:
                raise ValueError('Backend guard unavailable/rejected: ' + k)
        if r.get('manual_stop') is not False or r.get('guardian_latched') is not False:
            raise ValueError('Backend stop/guardian blocks confirmation')
        if r.get('simulation') is not False or r.get('hardware_capable') is not True:
            raise ValueError('Backend not hardware capable; simulation is not safety evidence')
        if self.recipe['axis'] == 'pitch' and r.get('pitch_hardware_blocked') is not False:
            raise ValueError('Pitch hardware blocked by backend')
        for k in ('position_target_rad', 'position_actual_rad', 'position_velocity_rad_s'):
            if type(r.get(k)) not in (float, int) or not math.isfinite(r[k]):
                raise ValueError('Missing exact MCU feedback: ' + k)
        if (r['position_target_rad'] != f32(self.recipe['center_rad'], 'center_rad')
            or abs(r['position_actual_rad']-r['position_target_rad']) > self.policy['position_tolerance']
            or abs(r['position_velocity_rad_s']) > self.policy['velocity_tolerance']):
            raise ValueError('Backend target changed or feedback drifted')
        device_identity = (r.get('boot_id'), r.get('firmware_sha256'), r.get('deployment_sha256'))
        hashes_known = all(isinstance(h, str) and len(h) == 64 and h != '0'*64 for h in device_identity[1:])
        if device_identity != self.prepared_identity or not device_identity[0] or not hashes_known:
            raise ValueError('Missing/changed boot or firmware/deployment identity')
        return (self.binding, r.get('boot_id'), r.get('firmware_sha256'), r.get('deployment_sha256'),
                r['session'], r['config_epoch'], r['reference_epoch'], r['human_token'])

    def confirm(self):
        self._ready(True)
        try:
            self._evidence((3,))
            return self._submit('confirm', self.peer.experiment_confirm(*self._ids(), self.policy['human_token'], hardware_blocked=self.hardware_blocked))
        except Exception as exc:
            self.invalidate(exc)
            raise

    def start(self):
        self._ready(True)
        try:
            if self.confirmed is None or self._evidence((4,)) != self.confirmed:
                raise ValueError('Separate human CONFIRM ACK required')
            if self.capture_start is None:
                raise ValueError('START unavailable: no prepared raw Profile4 capture orchestration')
            # Never open a short capture while the operator checks positioning.
            self.capture_pending = self.capture_start(deepcopy(self.recipe), deepcopy(self.reply))
            if not hasattr(self.capture_pending, 'done'):
                self.capture_pending = None
                raise ValueError('Capture owner must return an acknowledged Future')
            self.requested = 'capture-before-explicit-start'
        except Exception as exc:
            self.invalidate(exc)
            raise

    def _withdraw(self, kind):
        self.invalidate(kind.upper() + ' requested; CANCEL may hold, STOP is not physical e-stop evidence')
        self._ids()
        if self.closed:
            raise ValueError('Controller closed')
        if self.capture_pending is not None:
            self.capture_pending.cancel()
            self.capture_pending = None
        if self.pending is not None:
            # Bounded explicit-click intent, not a second poll or duplicate command.
            if self.deferred != 'stop':
                self.deferred = kind
            return self.pending
        self._ready()
        return self._submit(kind, getattr(self.peer, 'experiment_' + kind)(*self._ids()))

    def cancel(self):
        return self._withdraw('cancel')

    def stop(self):
        return self._withdraw('stop')

    def tick(self):
        if self.closed:
            return
        if self.capture_pending is not None and self.capture_pending.done():
            future, self.capture_pending = self.capture_pending, None
            try:
                evidence = future.result()
                if not isinstance(evidence, dict) or evidence.get('ready') is not True:
                    raise ValueError('Raw capture not authoritatively ready')
                if self.confirmed is None or self._evidence((4,)) != self.confirmed:
                    raise ValueError('Positioning authorization changed during capture open')
                deadline = evidence.get('dispatch_not_after')
                if type(deadline) not in (int, float) or not math.isfinite(deadline) or self.clock() >= deadline:
                    raise ValueError('Finite capture START dispatch deadline expired')
                self._submit('start', self.peer.experiment_start(*self._ids(), hardware_blocked=self.hardware_blocked,
                    dispatch_not_after=deadline))
            except Exception as exc:
                self.invalidate(exc)
        if self.pending is not None and self.pending.done():
            future, kind = self.pending, self.pending_kind
            self.pending = self.pending_kind = None
            try:
                r = future.result()
                if self.pending_binding != self.binding or self.pending_revision != self.revision:
                    raise ValueError('Recipe changed while command was in flight')
                if r.get('result') != 0 or r.get('body_version') != 3:
                    raise ValueError('Backend rejected/unsupported ' + kind + ': ' + str(r))
                self.reply = deepcopy(r)
                if kind == 'prepare':
                    self.prepared_binding = self.binding
                    self.prepared_identity = (r.get('boot_id'), r.get('firmware_sha256'), r.get('deployment_sha256'))
                self.note = 'Backend ACK: ' + kind + '; stage=' + str(r.get('positioning_stage_name'))
                if kind == 'confirm':
                    self.confirmed = self._evidence((4,))
                    self.authorization_invalidated_reason = None
                if kind == 'stop' and r.get('physical_stop_confirmed') is True:
                    self.active = False
                if r.get('positioning_stage') not in (4, 5):
                    self.confirmed = None
            except Exception as exc:
                if kind == 'prepare':
                    # A rejected PREPARE leaves no live session; without this
                    # the poll loop would keep polling a dead session forever.
                    self.active = False
                self.invalidate(exc)
        if self.confirmed is not None:
            try:
                if self._evidence((4, 5)) != self.confirmed:
                    self.invalidate('Backend identity changed')
            except Exception as exc:
                self.invalidate(exc)
        if self.deferred is not None and self.pending is None:
            kind, self.deferred = self.deferred, None
            try:
                self._withdraw(kind)
            except Exception as exc:
                self.invalidate(exc)
        if self.active and self.pending is None and self.capture_pending is None and self.clock() >= self.next_poll:
            try:
                self._ready()
                self._submit('status', self.peer.experiment_status(*self._ids()))
                self.next_poll = self.clock() + min(.05, self.recipe['heartbeat_timeout_us']/6e6)
            except Exception as exc:
                self.invalidate(exc)

    def snapshot(self):
        return deepcopy({'requested': self.requested, 'backend': self.reply,
            'human_confirmed': self.confirmed is not None, 'active': self.active,
            'authorization_invalidated_reason': self.authorization_invalidated_reason,
            'hardware_blocked': self.hardware_blocked, 'draft_dirty': self.draft_dirty, 'capture_available': self.capture_start is not None,
            'recipe': self.recipe, 'position_policy': self.policy,
            'pending': self.pending_kind, 'note': self.note})

    def close(self):
        self.closed = True
        self.deferred = None
        self.invalidate('Closed; no authorization survives reconnect; MCU hold timeout remains authoritative')
        if self.pending is not None:
            self.pending.cancel()
        if self.capture_pending is not None:
            self.capture_pending.cancel()
        self.pending = self.capture_pending = None
