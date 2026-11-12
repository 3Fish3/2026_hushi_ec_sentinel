"""Sentinel AutoTune M1 offline balanced scoring — pure standard library.

Scope: OFFLINE, SYNTHETIC / NOT_HARDWARE_VALIDATED only. This module never
connects to hardware, never writes parameters and never generates writable
candidates. It consumes the frozen host dataset dict
``{schema_version, manifest, samples, quality, events}`` (see
``docs/autotune/IMPLEMENTATION_CONTRACT.md``) and produces per-scenario and
overall six-term balanced losses.

Six normalized losses (lower is better):

    L = 0.30 E + 0.15 T + 0.10 O + 0.20 V + 0.10 U + 0.15 S

E tracks tracking error (real-dt weighted RMSE / e_ref).
T is settling time for legal steps, or explicit delay/phase for continuous
  motion — a hold/continuous segment is NEVER reported as settling time.
O is overshoot fraction on legal step/reverse segments.
V is non-commanded residual/ringing energy (high-passed error), not raw speed.
U is normalized output RMS plus saturation ratio.
S is output derivative/reversal activity with a disclosed fixed normalizer.

Missing or legally-inapplicable T/O is ``null`` with status
``insufficient_scenario`` — never zeroed to fake a finite total. Unknown facts
carry ``null`` plus a reason. No NaN is ever emitted.

S is a DISCLOSED PROXY, not a frequency-domain attribution: it is the RMS of
the discrete time-derivative of ``output_normalized``. The sign of that
derivative is used only to count output *reversals*, emitted as a diagnostic
(``terms.S.diagnostics.reversals``); the scalar S does not pretend to model
frequency content.

Input quality is never trusted: ``score_dataset`` rejects datasets whose
declared quality is absent, ``valid != True`` or carries errors, AND
independently re-runs ``autotune_dataset.validate_samples`` so a forged
``{'valid': True}`` cannot hide bad rows. Individual sample signal-validity
(rc_valid / feedback_valid / faults / stop / non-executing mode / CAN tx /
axis or param_epoch drift) is enforced directly and can never be constant-true.

A missing ``output_normalized`` is NOT a rejection when it is an explicit null
with a documented non-empty ``missing_fields`` reason (e.g. a real pitch trace
with no internal controller-output channel): U/S become ``unknown`` (null), L is
incomplete/null, and the run is non-promotable — unknown is preserved and
reported, never treated as a known zero. A finite output that simultaneously
claims to be missing (self-contradictory bundle) and NaN/Inf are still rejected.

A genuine large commanded step that the mechanism failed to follow (stuck / low
Kp / bad control) is a REAL experiment with poor response — it is evaluated
(T = ``not_settled``, E reflects the large error), NOT rejected as a mislabelled
scenario. Only a *target-event* inconsistency (declared pre/post/amplitude/
direction not realised by the samples' targets) rejects a step.
"""
from __future__ import annotations
import hashlib
import json
import math

SCHEMA_VERSION = 1

TERM_WEIGHTS = {'E': 0.30, 'T': 0.15, 'O': 0.10, 'V': 0.20, 'U': 0.10, 'S': 0.15}
TERM_ORDER = ['E', 'T', 'O', 'V', 'U', 'S']

CANONICAL_UNITS = {
    'angle': 'rad',
    'velocity': 'rad/s',
    'time': 's',
    'timestamp': 'us',
    'output': 'normalized',
}

VALID_AXES = ('fire_yaw', 'pitch')

# --------------------------------------------------------------------------- #
# Frozen per-axis normalization scales.
#
# These are ENGINEERING placeholders frozen for reproducibility, NOT measured
# hardware limits and NOT safety envelopes. They live here (owned by the
# scoring agent) and are recorded into each dataset manifest with a sha256
# fingerprint so a score can never be silently recomputed against a different
# ruler. Changing them requires re-freezing and invalidates comparability.
# --------------------------------------------------------------------------- #
_FROZEN_SCALE_VALUES = {
    'fire_yaw': {
        'e_ref': 0.02,          # rad, reference tracking tolerance
        't_ref': 0.20,          # s, reference settling/delay time
        'o_ref': 0.10,          # overshoot fraction reference
        'v_ref': 0.02,          # rad, residual ringing amplitude reference
        'u_ref': 0.50,          # normalized output burden reference
        's_ref': 50.0,          # normalized-output per second derivative reference
        'band_frac': 0.02,      # settling band as fraction of step amplitude
        'sat_threshold': 0.98,  # |output| saturation threshold
        'v_hp_hz': 5.0,         # residual high-pass cutoff for V
    },
    'pitch': {
        'e_ref': 0.02,
        't_ref': 0.25,
        'o_ref': 0.10,
        'v_ref': 0.02,
        'u_ref': 0.50,
        's_ref': 50.0,
        'band_frac': 0.02,
        'sat_threshold': 0.98,
        'v_hp_hz': 5.0,
    },
}


def _canonical(obj):
    return json.dumps(obj, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True, allow_nan=False)


def _fingerprint(values):
    return hashlib.sha256(_canonical(values).encode('utf-8')).hexdigest()


def freeze_scale_dict(values):
    """Return a copy of ``values`` with a fresh deterministic fingerprint."""
    body = {k: values[k] for k in sorted(values) if k != 'fingerprint'}
    for key, v in body.items():
        if not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(float(v)):
            raise ValueError(f'scale value {key!r} must be a finite number')
    out = dict(body)
    out['fingerprint'] = _fingerprint(body)
    return out


# Reference normalizers that MUST be strictly positive (a zero/negative ruler
# would let any signal score arbitrarily well or invert sign).
_POSITIVE_REFS = ('e_ref', 't_ref', 'o_ref', 'v_ref', 'u_ref', 's_ref')
# band_frac is a fraction of step amplitude in (0, 1]; sat_threshold in (0, 1].
_FRACTION_KEYS = ('band_frac', 'sat_threshold')


def validate_scale(scale):
    """Reject a scale whose reference normalizers are non-positive/out-of-range.

    Prevents e.g. a negative ``e_ref`` or a zero ``u_ref`` from silently
    bypassing normalization (which would produce nonsensical or inverted scores).
    """
    if not isinstance(scale, dict):
        raise ValueError('scale must be a dict')
    for key in _POSITIVE_REFS:
        if key not in scale:
            raise ValueError(f'scale missing required reference {key!r}')
        v = scale[key]
        if not _is_num(v) or not math.isfinite(float(v)) or float(v) <= 0:
            raise ValueError(f'scale.{key} must be finite and > 0, got {v!r}')
    for key in _FRACTION_KEYS:
        if key in scale:
            v = scale[key]
            if not _is_num(v) or not (0.0 < float(v) <= 1.0):
                raise ValueError(f'scale.{key} must be in (0, 1], got {v!r}')
    if 'v_hp_hz' in scale:
        v = scale['v_hp_hz']
        if not _is_num(v) or not math.isfinite(float(v)) or float(v) <= 0:
            raise ValueError(f'scale.v_hp_hz must be finite and > 0, got {v!r}')


def frozen_scale(axis):
    """Return the frozen normalizer dict (with fingerprint) for an axis."""
    if axis not in _FROZEN_SCALE_VALUES:
        raise ValueError(f'unknown axis {axis!r}')
    return freeze_scale_dict(_FROZEN_SCALE_VALUES[axis])


# --------------------------------------------------------------------------- #
# Small numeric helpers (standard library only).
# --------------------------------------------------------------------------- #
def _is_num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def _require_finite(name, value):
    if value is None:
        return
    if not _is_num(value) or not math.isfinite(float(value)):
        raise ValueError(f'non-finite or non-numeric value in {name}: {value!r}')


def _dt_weighted_rms(items):
    """items: iterable of (value, dt). Returns sqrt(sum v^2 dt / sum dt)."""
    num = den = 0.0
    for v, dt in items:
        num += v * v * dt
        den += dt
    if den <= 0:
        raise ValueError('zero total dt')
    return math.sqrt(num / den)


def _term(term, raw, normalized, weight, count, status='ok', note=None,
          proxy=None, diagnostics=None):
    contribution = None
    if status == 'ok' and normalized is not None:
        contribution = weight * normalized
    return {
        'term': term,
        'raw': raw,
        'normalized': normalized,
        'weight': weight,
        'contribution': contribution,
        'count': count,
        'status': status,
        'note': note,
        'proxy': proxy,
        'diagnostics': diagnostics,
    }


# --------------------------------------------------------------------------- #
# Validation (do not trust the declared quality gate).
# --------------------------------------------------------------------------- #
def _unwrap_times(samples):
    """Strictly-increasing unwrapped microsecond times, accumulating u32 wraps.

    Raw ``t_mcu_us`` is a 32-bit counter. Each time a raw value drops below the
    previous raw value by a large margin we treat it as a genuine 32-bit wrap and
    add another ``2**32`` to the accumulated offset, so *every* subsequent small
    raw timestamp stays ordered correctly. A duplicate (exact equal) timestamp is
    always rejected. A same-magnitude backward jump that is NOT a legal wrap is
    rejected as non-monotonic (restart/identity change must not masquerade as a
    wrap). Raw samples must lie in the u32 range ``[0, 2**32)``.
    """
    U32 = 0x100000000
    HALF = 0x80000000
    out = []
    prev_raw = None
    offset = 0
    for idx, s in enumerate(samples):
        v = s.get('t_mcu_us')
        if not isinstance(v, int) or isinstance(v, bool):
            raise ValueError(f'sample {idx}: t_mcu_us must be an int, got {v!r}')
        if v < 0 or v >= U32:
            raise ValueError(f'sample {idx}: t_mcu_us={v} outside u32 range [0,2**32)')
        if prev_raw is not None:
            if v == prev_raw:
                raise ValueError(f'sample {idx}: duplicate t_mcu_us={v}')
            if v < prev_raw:
                # A wrap is only legal if the drop is more than half the u32 range
                # (i.e. the counter rolled over), not a small backward step.
                if (prev_raw - v) > HALF:
                    offset += U32
                else:
                    raise ValueError(
                        f'sample {idx}: non-monotonic t_mcu_us={v} after {prev_raw} '
                        f'(not a legal 32-bit wrap)')
        out.append(v + offset)
        prev_raw = v
    return out


def _check_quality_gate(dataset, samples):
    """Enforce the dataset quality gate; do NOT trust a bare ``valid=True``.

    Accepts either the loader's full validation output
    (``{valid, errors, warnings, ...}``) or a wrapped ``{'gate': {...}}``.
    STRICT semantics: ``valid`` must be exactly True AND ``errors`` must be empty;
    ``quality.valid=False`` or *any* error code is always rejected. There is no
    per-error-code exemption — a genuine ``missing_bundle`` (or any other error)
    is never tolerated, even if that row also happens to have a documented-null
    output. A documented-null output is deliberately NOT an error in the dataset
    validator, so it passes naturally; if it ever appears as an error it means
    something else is wrong and must be rejected.

    Independently re-derives structural validity from the samples via the dataset
    validator (deferred import) so a fabricated ``{'valid': True}`` cannot hide
    bad rows.
    """
    quality = dataset.get('quality')
    if not isinstance(quality, dict):
        raise ValueError('dataset.quality missing or not a dict (quality not proven)')
    gate = quality.get('gate') if isinstance(quality.get('gate'), dict) else None
    # Merge gate view; prefer explicit top-level fields when present.
    valid = quality.get('valid', gate.get('valid') if gate else None)
    errors = quality.get('errors')
    if errors is None and gate is not None:
        errors = gate.get('errors')
    if errors is None:
        errors = []
    if not isinstance(errors, list):
        raise ValueError('dataset.quality.errors must be a list')
    if valid is not True:
        raise ValueError(f'dataset.quality.valid must be true, got {valid!r}')
    if errors:
        codes = [e.get('code') if isinstance(e, dict) else str(e) for e in errors[:5]]
        raise ValueError(f'dataset.quality reports errors: {codes}')

    # Independent re-validation: never let a forged quality dict bypass structure.
    try:
        import autotune_dataset as _validator  # deferred: avoid import cycle
    except Exception:
        _validator = None
    if _validator is not None:
        manifest = dataset.get('manifest') or {}
        try:
            result = _validator.validate_samples(manifest, samples)
        except Exception as exc:
            raise ValueError(f'dataset failed independent re-validation: {exc}')
        if not result.get('valid'):
            codes = [e.get('code') for e in result.get('errors', [])][:5]
            raise ValueError(
                f'dataset failed independent re-validation (quality.valid lied): {codes}')
        if result.get('errors'):
            raise ValueError(
                f'dataset re-validation returned errors despite valid=True: '
                f'{[e.get("code") for e in result["errors"]][:5]}')


def _validate_sample_row(idx, s, axis):
    """Reject rows that are individually unusable for scoring."""
    if not isinstance(s, dict):
        raise ValueError(f'sample {idx} is not a dict')

    # axis must match manifest (per-row silent axis drift is not tolerated)
    s_axis = s.get('axis')
    if s_axis is not None and s_axis != axis:
        raise ValueError(f'sample {idx}: axis {s_axis!r} != manifest axis {axis!r}')

    # unknown parameter epoch cannot be attributed to a single version
    if s.get('param_epoch') is None:
        raise ValueError(f'sample {idx}: param_epoch is unknown (null) - version not attributable')

    dt = s.get('dt_s')
    if not _is_num(dt) or not math.isfinite(float(dt)) or dt <= 0:
        raise ValueError(f'sample {idx}: dt_s must be a finite positive number, got {dt!r}')

    for field in ('angle_rad', 'adopted_target_rad', 'request_target_rad',
                  'velocity_rad_s', 'output_normalized'):
        if field in s:
            _require_finite(f'sample {idx}.{field}', s[field])

    # required signals for scoring
    if s.get('angle_rad') is None:
        raise ValueError(f'sample {idx}: angle_rad is required for scoring (null not allowed)')
    if s.get('adopted_target_rad') is None:
        raise ValueError(f'sample {idx}: adopted_target_rad missing (no executed target)')

    # Output may be legitimately absent on real traces (e.g. a pitch trace that
    # simply has no internal controller-output channel). An explicit null with a
    # missing_fields reason is ALLOWED and carried forward as *unknown* (U/S/L
    # become null, the run is non-promotable) — never silently treated as zero.
    # What we DO reject: a finite value that simultaneously claims to be missing
    # (self-contradictory bundle), and NaN/Inf (handled by _require_finite above).
    out = s.get('output_normalized')
    missing = s.get('missing_fields') if isinstance(s.get('missing_fields'), dict) else {}
    if out is None:
        if 'output_normalized' not in missing or not missing.get('output_normalized'):
            raise ValueError(
                f'sample {idx}: output_normalized is null but no missing_fields '
                f'reason given (unknown must be documented, not silent)')
        # documented absence -> allowed (U/S unknown, non-promotable)
        missing_reason = missing.get('output_normalized')
        if not isinstance(missing_reason, str) or not missing_reason.strip():
            raise ValueError(
                f'sample {idx}: output_normalized missing reason must be a '
                f'non-empty string, got {missing_reason!r}')
    else:
        if 'output_normalized' in missing:
            raise ValueError(
                f'sample {idx}: output_normalized is a finite value but declared '
                f'missing="{missing["output_normalized"]}" (self-contradictory bundle)')

    # signal-validity flags: any false flag disqualifies the whole run's quality
    if s.get('rc_valid') is False:
        raise ValueError(f'sample {idx}: rc_valid=false (RC input invalid)')
    if s.get('feedback_valid') is False:
        raise ValueError(f'sample {idx}: feedback_valid=false (feedback invalid)')

    # faults / stop / non-executing mode cannot be scored as valid tracking
    faults = s.get('fault_bits')
    if faults not in (0, None):
        raise ValueError(f'sample {idx}: fault_bits={faults} (fault present)')
    src = s.get('target_source')
    if src in ('stop', 'manual_stop'):
        raise ValueError(f'sample {idx}: target_source={src!r} (stop / not commanding)')
    if s.get('controller_mode') == 'none':
        raise ValueError(f'sample {idx}: controller_mode="none" (not executing)')

    # CAN transmit result must be a clean accept when reported
    tx = s.get('can_tx_result')
    if tx is not None and tx not in ('accepted',):
        raise ValueError(f'sample {idx}: can_tx_result={tx!r} (not accepted)')


def _validate(dataset):
    if not isinstance(dataset, dict):
        raise ValueError('dataset must be a dict')
    if dataset.get('schema_version', SCHEMA_VERSION) != SCHEMA_VERSION:
        raise ValueError('dataset schema_version must be 1')
    manifest = dataset.get('manifest')
    samples = dataset.get('samples')
    if not isinstance(manifest, dict) or not isinstance(samples, list):
        raise ValueError('dataset missing manifest or samples')
    if not samples:
        raise ValueError('dataset has no samples')

    axis = manifest.get('axis')
    if axis not in VALID_AXES:
        raise ValueError(f'manifest.axis must be one of {VALID_AXES}, got {axis!r}')

    units = manifest.get('units')
    if not isinstance(units, dict):
        raise ValueError('manifest.units missing')
    for key, want in CANONICAL_UNITS.items():
        if units.get(key) != want:
            raise ValueError(f'unit mismatch for {key!r}: expected {want!r}, got {units.get(key)!r}')

    flags = manifest.get('flags')
    if not isinstance(flags, dict):
        raise ValueError('manifest.flags missing')
    for key in ('autotune_enabled', 'hardware_actuation', 'param_autowrite', 'shooting_enabled'):
        if flags.get(key) is not False:
            raise ValueError(f'manifest.flags.{key} must be false in M1 (got {flags.get(key)!r})')

    scale = manifest.get('scale')
    if not isinstance(scale, dict) or 'fingerprint' not in scale:
        raise ValueError('manifest.scale must be frozen (missing fingerprint)')
    recomputed = freeze_scale_dict(scale)
    if recomputed['fingerprint'] != scale.get('fingerprint'):
        raise ValueError('manifest.scale fingerprint mismatch (tampered or unfrozen normalizer)')
    # Reject non-positive / out-of-range reference normalizers (no bypass).
    validate_scale(scale)

    # Per-sample signal validity (independent of the declared quality flag).
    for idx, s in enumerate(samples):
        _validate_sample_row(idx, s, axis)

    # Duplicate sequence / explicit time-gap rejection across the run.
    seq_seen = set()
    prev_dt = None
    max_dt = manifest.get('max_dt_s')
    bound_known = _is_num(max_dt) and math.isfinite(float(max_dt))
    for idx, s in enumerate(samples):
        seq = s.get('sequence')
        if seq is not None:
            if seq in seq_seen:
                raise ValueError(f'sample {idx}: duplicate sequence={seq!r}')
            seq_seen.add(seq)
        dt = float(s['dt_s'])
        if bound_known and dt > float(max_dt) + 1e-12:
            raise ValueError(
                f'sample {idx}: dt_s={dt} exceeds declared max_dt_s={max_dt} (explicit time gap)')

    times = _unwrap_times(samples)

    scenarios = manifest.get('scenarios')
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError('manifest.scenarios must be a non-empty list')
    n = len(samples)
    for sidx, sc in enumerate(scenarios):
        i0, i1 = sc.get('i0'), sc.get('i1')
        if not isinstance(i0, int) or not isinstance(i1, int) or not (0 <= i0 < i1 <= n):
            raise ValueError(f'scenario {sidx}: invalid half-open range i0={i0!r} i1={i1!r} (n={n})')
        # A step/reverse must be a real, internally consistent step.
        _validate_step_definition(sc, samples, i0, i1, sidx, axis)

    # Enforce the dataset quality gate last (independent re-validation).
    _check_quality_gate(dataset, samples)

    return manifest, samples, times, scenarios


def _scenario_epoch(samples, i0, i1):
    epochs = []
    for i in range(i0, i1):
        e = samples[i].get('param_epoch')
        epochs.append(e)
    distinct = []
    for e in epochs:
        if e not in distinct:
            distinct.append(e)
    if len(distinct) > 1:
        raise ValueError(f'mixed param_epoch inside scoring window: {distinct}')
    return distinct[0] if distinct else None


# --------------------------------------------------------------------------- #
# Term computations.
# --------------------------------------------------------------------------- #
def _error(sample):
    # Prefer adopted target (what the controller actually tracked).
    tgt = sample.get('adopted_target_rad')
    if tgt is None:
        tgt = sample.get('request_target_rad')
    return float(tgt) - float(sample['angle_rad'])


def _compute_E(samples, i0, i1, scale):
    vals = []
    for i in range(i0, i1):
        s = samples[i]
        vals.append((_error(s), float(s['dt_s'])))
    raw = _dt_weighted_rms(vals)
    return _term('E', raw, raw / scale['e_ref'], TERM_WEIGHTS['E'], i1 - i0)


def _compute_U(samples, i0, i1, scale):
    items = []
    sat_num = sat_den = 0.0
    for i in range(i0, i1):
        s = samples[i]
        out = s.get('output_normalized')
        dt = float(s['dt_s'])
        if out is None:
            return _term('U', None, None, TERM_WEIGHTS['U'], i1 - i0,
                         status='unknown', note='output_normalized unavailable')
        o = float(out)
        items.append((o, dt))
        if abs(o) >= scale['sat_threshold']:
            sat_num += dt
        sat_den += dt
    rms = _dt_weighted_rms(items)
    sat_ratio = sat_num / sat_den if sat_den > 0 else 0.0
    raw = 0.7 * rms + 0.3 * sat_ratio
    return _term('U', raw, raw / scale['u_ref'], TERM_WEIGHTS['U'], i1 - i0)


def _compute_S(samples, i0, i1, scale):
    """S = RMS of the output derivative (smoothed-command activity proxy).

    DISCLOSED PROXY: S is built from the discrete derivative of
    ``output_normalized`` (normalized-output per second), NOT a frequency-domain
    attribution. The sign of the derivative is used only to count output
    reversals, which is emitted as an extra *diagnostic* (``diagnostics.
    reversals``) so a reviewer can see oscillation activity; it is not folded
    into the scalar S in a way that pretends to model frequency.
    """
    items = []
    reversals = 0
    prev_out = None
    prev_dir = 0
    for i in range(i0, i1):
        s = samples[i]
        out = s.get('output_normalized')
        dt = float(s['dt_s'])
        if out is None:
            return _term('S', None, None, TERM_WEIGHTS['S'], i1 - i0,
                         status='unknown', note='output_normalized unavailable',
                         proxy='output_derivative',
                         diagnostics={'reversals': None})
        o = float(out)
        if prev_out is not None:
            d = o - prev_out
            items.append((d / dt, dt))
            cur_dir = (d > 0) - (d < 0)
            if cur_dir != 0 and prev_dir != 0 and cur_dir != prev_dir:
                reversals += 1
            if cur_dir != 0:
                prev_dir = cur_dir
        prev_out = o
    diagnostics = {'reversals': reversals}
    if not items:
        return _term('S', 0.0, 0.0, TERM_WEIGHTS['S'], i1 - i0,
                     proxy='output_derivative', diagnostics=diagnostics)
    raw = _dt_weighted_rms(items)  # units: normalized-output per second
    return _term('S', raw, raw / scale['s_ref'], TERM_WEIGHTS['S'], i1 - i0,
                 proxy='output_derivative', diagnostics=diagnostics)


def _compute_V(samples, i0, i1, scale, times):
    # High-pass the tracking error to isolate non-commanded residual/ringing.
    fc = float(scale['v_hp_hz'])
    filtered = []
    e_lp = None
    prev_t = None
    for i in range(i0, i1):
        s = samples[i]
        dt = float(s['dt_s'])
        e = _error(s)
        if e_lp is None:
            e_lp = e
            hp = 0.0
        else:
            rc = 1.0 / (2.0 * math.pi * fc)
            alpha = dt / (rc + dt)
            e_lp = e_lp + alpha * (e - e_lp)
            hp = e - e_lp
        filtered.append((hp, dt))
        prev_t = dt
    raw = _dt_weighted_rms(filtered)
    return _term('V', raw, raw / scale['v_ref'], TERM_WEIGHTS['V'], i1 - i0)


def _step_params(sc):
    step_index = sc.get('step_index')
    if step_index is None or not isinstance(step_index, int):
        return None
    post = sc.get('post_target_rad')
    pre = sc.get('pre_target_rad')
    amp = sc.get('amplitude_rad')
    if post is None or amp is None:
        return None
    post = float(post)
    amp = float(amp)
    if not math.isfinite(post) or not math.isfinite(amp) or amp <= 0:
        return None
    direction = sc.get('direction', '+')
    return step_index, post, amp, direction, (None if pre is None else float(pre))


def _validate_step_definition(sc, samples, i0, i1, sidx, axis):
    """Ensure a step/reverse scenario's declared targets match the real samples.

    Rejects a mislabelled step: direction must agree with sign(post-pre); the
    declared amplitude must equal |post-pre|; pre/post targets must be realised
    by the samples' adopted/request target before/after the step index; and the
    actual motion must be non-trivial (a tiny dither cannot masquerade as a step).
    """
    if sc.get('kind') not in ('step', 'reverse'):
        return
    step_index = sc.get('step_index')
    post = sc.get('post_target_rad')
    pre = sc.get('pre_target_rad')
    amp = sc.get('amplitude_rad')
    direction = sc.get('direction')
    if not isinstance(step_index, int):
        raise ValueError(f'scenario {sidx}: step kind requires an int step_index')
    if not (i0 <= step_index < i1):
        raise ValueError(f'scenario {sidx}: step_index {step_index} outside [{i0},{i1})')
    if post is None or amp is None or pre is None:
        raise ValueError(f'scenario {sidx}: step requires pre_target_rad/post_target_rad/amplitude_rad')
    post, pre, amp = float(post), float(pre), float(amp)
    if not all(math.isfinite(x) for x in (post, pre, amp)):
        raise ValueError(f'scenario {sidx}: step targets must be finite')
    if amp <= 0:
        raise ValueError(f'scenario {sidx}: step amplitude must be > 0')
    delta = post - pre
    if delta == 0:
        raise ValueError(f'scenario {sidx}: step pre==post (no move)')
    if abs(abs(delta) - amp) > 1e-6 + 1e-6 * amp:
        raise ValueError(
            f'scenario {sidx}: declared amplitude {amp} != |post-pre| {abs(delta)}')
    expect_dir = '+' if delta > 0 else '-'
    if direction not in ('+', '-'):
        raise ValueError(f'scenario {sidx}: direction must be "+" or "-", got {direction!r}')
    if direction != expect_dir:
        raise ValueError(
            f'scenario {sidx}: direction {direction!r} contradicts post-pre sign ({expect_dir!r})')
    # samples must realise the declared pre target roughly before the step and
    # the post target roughly after it (bound by amplitude).
    tol = max(0.25 * amp, 1e-6)

    def _tgt(i):
        t = samples[i].get('adopted_target_rad')
        if t is None:
            t = samples[i].get('request_target_rad')
        return None if t is None else float(t)

    pre_ok = False
    for i in range(i0, step_index):
        t = _tgt(i)
        if t is not None and abs(t - pre) <= tol:
            pre_ok = True
            break
    post_ok = False
    for i in range(step_index, i1):
        t = _tgt(i)
        if t is not None and abs(t - post) <= tol:
            post_ok = True
            break
    if not pre_ok or not post_ok:
        raise ValueError(
            f'scenario {sidx}: samples do not realise declared step targets '
            f'pre={pre} post={post} (mislabelled step: target event inconsistent)')
    # NOTE: we deliberately do NOT require the *actual angle* to move by any
    # fraction of the amplitude. A genuine large commanded step that the
    # mechanism failed to follow (stuck / low Kp / bad control) is exactly what
    # must be evaluated — it is a real step with poor response (T=not_settled,
    # L over available terms), NOT a mislabelled/tiny-motion scenario. Only a
    # *target-event* inconsistency (pre/post/amp/direction vs samples) rejects.


def _compute_T(samples, times, i0, i1, scale, sc):
    kind = sc.get('kind')
    if kind in ('step', 'reverse'):
        params = _step_params(sc)
        if params is None:
            return _term('T', None, None, TERM_WEIGHTS['T'], i1 - i0,
                         status='insufficient_scenario',
                         note='step scenario lacks explicit step_index/post_target/amplitude')
        step_index, post, amp, _, _pre = params
        if not (i0 <= step_index < i1):
            return _term('T', None, None, TERM_WEIGHTS['T'], i1 - i0,
                         status='insufficient_scenario', note='step_index outside scenario')
        band = max(scale['band_frac'] * amp, 1e-9)
        t_step = times[step_index] * 1e-6
        last_bad = None
        for i in range(step_index, i1):
            if abs(float(samples[i]['angle_rad']) - post) > band:
                last_bad = i
        if last_bad is None:
            raw = 0.0
        else:
            settle_idx = last_bad + 1
            if settle_idx >= i1:
                return _term('T', None, None, TERM_WEIGHTS['T'], i1 - i0,
                             status='not_settled',
                             note='error never entered settling band within window')
            raw = times[settle_idx] * 1e-6 - t_step
        raw = max(raw, 0.0)
        return _term('T', raw, raw / scale['t_ref'], TERM_WEIGHTS['T'], i1 - i0)
    if kind == 'continuous':
        # Explicit delay estimate (NOT settling time). Lag via correlation peak.
        raw = _estimate_delay(samples, i0, i1)
        if raw is None:
            return _term('T', None, None, TERM_WEIGHTS['T'], i1 - i0,
                         status='insufficient_scenario',
                         note='continuous delay not estimable')
        return _term('T', raw, raw / scale['t_ref'], TERM_WEIGHTS['T'], i1 - i0,
                     note='continuous motion: delay/phase lag, not settling time')
    # hold, static, or unknown kind: T is legally inapplicable.
    return _term('T', None, None, TERM_WEIGHTS['T'], i1 - i0,
                 status='insufficient_scenario',
                 note=f'{kind!r} segment has no step/transition for settling or delay')


def _estimate_delay(samples, i0, i1):
    """Discrete correlation-peak delay between target and angle (seconds)."""
    tgt = []
    ang = []
    for i in range(i0, i1):
        s = samples[i]
        t = s.get('adopted_target_rad')
        if t is None:
            t = s.get('request_target_rad')
        tgt.append(float(t))
        ang.append(float(s['angle_rad']))
    n = len(tgt)
    if n < 8:
        return None
    mt = sum(tgt) / n
    ma = sum(ang) / n
    dtv = [tgt[i] - mt for i in range(n)]
    dav = [ang[i] - ma for i in range(n)]
    denom = math.sqrt(sum(x * x for x in dtv) * sum(x * x for x in dav))
    if denom <= 0:
        return None
    max_lag = max(1, n // 8)
    best_lag, best_corr = 0, -2.0
    for lag in range(0, max_lag + 1):
        c = 0.0
        for i in range(0, n - lag):
            c += dtv[i] * dav[i + lag]
        c /= denom
        if c > best_corr:
            best_corr, best_lag = c, lag
    # correct global sign convention: positive lag means angle lags target
    dts = [float(samples[i]['dt_s']) for i in range(i0, i1)]
    avg_dt = sum(dts) / len(dts)
    return best_lag * avg_dt


def _compute_O(samples, i0, i1, scale, sc):
    kind = sc.get('kind')
    if kind not in ('step', 'reverse'):
        return _term('O', None, None, TERM_WEIGHTS['O'], i1 - i0,
                     status='insufficient_scenario',
                     note=f'{kind!r} segment has no step/reverse for overshoot')
    params = _step_params(sc)
    if params is None:
        return _term('O', None, None, TERM_WEIGHTS['O'], i1 - i0,
                     status='insufficient_scenario',
                     note='step scenario lacks explicit step targets/amplitude')
    step_index, post, amp, direction, _pre = params
    if not (i0 <= step_index < i1):
        return _term('O', None, None, TERM_WEIGHTS['O'], i1 - i0,
                     status='insufficient_scenario', note='step_index outside scenario')
    if direction == '-':
        peak_dev = max(0.0, post - min(float(samples[i]['angle_rad']) for i in range(step_index, i1)))
    else:
        peak_dev = max(0.0, max(float(samples[i]['angle_rad']) for i in range(step_index, i1)) - post)
    raw = peak_dev / amp
    return _term('O', raw, raw / scale['o_ref'], TERM_WEIGHTS['O'], i1 - i0)


# --------------------------------------------------------------------------- #
# Scoring entry point.
# --------------------------------------------------------------------------- #
def score_dataset(dataset):
    """Score an offline dataset dict. Raises ValueError on invalid input."""
    manifest, samples, times, scenarios = _validate(dataset)
    axis = manifest['axis']
    scale = manifest['scale']

    scenario_results = []
    for sc in scenarios:
        i0, i1 = sc['i0'], sc['i1']
        epoch = _scenario_epoch(samples, i0, i1)
        terms = {
            'E': _compute_E(samples, i0, i1, scale),
            'T': _compute_T(samples, times, i0, i1, scale, sc),
            'O': _compute_O(samples, i0, i1, scale, sc),
            'V': _compute_V(samples, i0, i1, scale, times),
            'U': _compute_U(samples, i0, i1, scale),
            'S': _compute_S(samples, i0, i1, scale),
        }
        complete = all(terms[k]['status'] == 'ok' for k in TERM_ORDER)
        if complete:
            l_norm = sum(terms[k]['contribution'] for k in TERM_ORDER)
            l_term = {
                'term': 'L', 'raw': l_norm, 'normalized': l_norm,
                'weight': sum(TERM_WEIGHTS.values()), 'contribution': l_norm,
                'count': i1 - i0, 'status': 'ok', 'note': None,
            }
        else:
            missing = [k for k in TERM_ORDER if terms[k]['status'] != 'ok']
            l_term = {
                'term': 'L', 'raw': None, 'normalized': None,
                'weight': sum(TERM_WEIGHTS.values()), 'contribution': None,
                'count': i1 - i0, 'status': 'insufficient_terms',
                'note': 'missing/inapplicable: ' + ','.join(missing),
            }
        terms['L'] = l_term
        scenario_results.append({
            'name': sc.get('name'),
            'kind': sc.get('kind'),
            'axis': axis,
            'i0': i0, 'i1': i1,
            # Step/reverse definition carried through so compare_runs can demand
            # structurally identical experiments (amplitude/direction/step pos).
            'step_index': sc.get('step_index'),
            'pre_target_rad': sc.get('pre_target_rad'),
            'post_target_rad': sc.get('post_target_rad'),
            'amplitude_rad': sc.get('amplitude_rad'),
            'direction': sc.get('direction'),
            'param_epoch': epoch,
            'count': i1 - i0,
            'valid_for_scoring': True,
            'terms': terms,
        })

    overall = _aggregate_overall(scenario_results, manifest, dataset.get('quality'))

    # Collect param epochs actually seen (per-scenario single version enforced).
    epochs = []
    for r in scenario_results:
        if r['param_epoch'] is not None and r['param_epoch'] not in epochs:
            epochs.append(r['param_epoch'])
    epochs.sort(key=lambda e: (str(type(e)), e))

    input_sha256 = _scientific_identity(manifest, samples, scenarios)

    # Return a deterministic, path/provenance-free view so that the score bytes
    # are identical for the same scientific content regardless of where the
    # dataset was exported (files/provenance paths, timestamps are excluded).
    return {
        'schema_version': SCHEMA_VERSION,
        'axis': axis,
        'synthetic': bool(manifest.get('synthetic', False)),
        'context': manifest.get('context'),
        'scale_fingerprint': scale.get('fingerprint'),
        'input_sha256': input_sha256,
        'scenarios': scenario_results,
        'overall': overall,
        'param_epochs': epochs,
    }


# Fields that belong to scientific identity (excluded: files, provenance,
# timestamps, absolute paths, run/boot identifiers).
_IDENTITY_SAMPLE_FIELDS = (
    'axis', 'param_epoch', 'sample_index', 'sequence', 'source_sequence',
    't_mcu_us', 'dt_s', 'request_target_rad', 'adopted_target_rad',
    'angle_rad', 'velocity_rad_s', 'output_normalized',
)
_IDENTITY_SCENARIO_FIELDS = (
    'name', 'kind', 'i0', 'i1', 'step_index', 'pre_target_rad',
    'post_target_rad', 'amplitude_rad', 'direction',
)


def _canonical_float(x):
    if x is None:
        return None
    if isinstance(x, bool):
        return x
    if isinstance(x, (int, float)):
        f = float(x)
        if not math.isfinite(f):
            return None
        # normalise -0.0 and integral floats for stable hashing
        if f == 0.0:
            return 0.0
        return f
    return x


def _scientific_identity(manifest, samples, scenarios):
    """sha256 over canonical samples + effective scale/scenario/context/params.

    Deliberately excludes file/provenance artifact paths and timestamps so the
    fingerprint tracks *science*, not packaging. ``files`` paths, run_id and
    boot_id never enter the hash.
    """
    canonical_samples = []
    for s in samples:
        canonical_samples.append({k: _canonical_float(s.get(k))
                                  for k in _IDENTITY_SAMPLE_FIELDS})
    canonical_scenarios = []
    for sc in scenarios:
        canonical_scenarios.append({k: _canonical_float(sc.get(k))
                                    for k in _IDENTITY_SCENARIO_FIELDS})
    scale = manifest.get('scale') or {}
    scale_body = {k: _canonical_float(scale[k]) for k in sorted(scale)
                  if k != 'fingerprint'}
    # Parameter identity: the *actual* tuning values, not just the epoch label.
    # Two runs labelled epoch=1 but with different gains are NOT the same
    # scientific identity and must hash differently.
    param_set = manifest.get('param_set') if isinstance(manifest.get('param_set'), dict) else {}
    param_identity = {
        'param_epoch': param_set.get('param_epoch'),
        'values': _canonical(param_set.get('values') or {}),
        'source': param_set.get('source'),
    }
    # Firmware identity (which controller/firmware produced this science).
    firmware = manifest.get('firmware') if isinstance(manifest.get('firmware'), dict) else {}
    firmware_identity = {
        'id': firmware.get('id', firmware.get('firmware_id')),
        'version': firmware.get('version', firmware.get('build_id')),
    }
    payload = {
        'axis': manifest.get('axis'),
        'synthetic': bool(manifest.get('synthetic', False)),
        'units': manifest.get('units'),
        'context': manifest.get('context'),
        'scale': {'values': scale_body, 'fingerprint': scale.get('fingerprint')},
        'param_identity': param_identity,
        'firmware_identity': firmware_identity,
        'scenarios': canonical_scenarios,
        'samples': canonical_samples,
    }
    return hashlib.sha256(_canonical(payload).encode('utf-8')).hexdigest()


def _aggregate_term(scenario_results, term):
    acc_num = acc_den = 0.0
    count = 0
    raw_acc = raw_den = 0.0
    ok = 0
    for r in scenario_results:
        t = r['terms'][term]
        if t['status'] != 'ok' or t['normalized'] is None:
            continue
        w = max(int(t['count']), 1)
        acc_num += t['normalized'] * w
        acc_den += w
        if t['raw'] is not None:
            raw_acc += t['raw'] * w
            raw_den += w
        count += t['count']
        ok += 1
    if acc_den <= 0:
        return _term(term, None, None, TERM_WEIGHTS[term], count,
                     status='insufficient_scenario',
                     note='no scenario provided a valid value for this term')
    normalized = acc_num / acc_den
    raw = raw_acc / raw_den if raw_den > 0 else None
    return _term(term, raw, normalized, TERM_WEIGHTS[term], count)


# Terms that MUST be present (status 'ok') for a scenario of a given kind to count
# toward a complete six-term loss. A segment that commands motion and fails to
# follow (e.g. T=not_settled, or O unavailable) leaves the required views
# incomplete; skipping it must NOT be masked by another segment that succeeded.
# 'hold' (and static/unknown) legally have no step/transition, so they carry no
# required T/O and never, by their mere presence, make a step+hold run incomplete.
_REQUIRED_TERMS_BY_KIND = {
    'step': ('T', 'O'),
    'reverse': ('T', 'O'),
    'continuous': ('T',),
}


def _incomplete_required_scenarios(scenario_results):
    """Return human-readable list of scenarios missing a REQUIRED term.

    Only motion-commanding kinds (step/reverse/continuous) impose requirements;
    a required term whose status is not 'ok' (e.g. 'not_settled' T, 'unknown' O)
    marks the run's six-term loss incomplete — a bad segment is never dropped or
    faked to 0 to pretend completeness.
    """
    incomplete = []
    for r in scenario_results:
        kind = r.get('kind')
        required = _REQUIRED_TERMS_BY_KIND.get(kind, ())
        if not required:
            continue
        missing = [t for t in required
                   if r['terms'].get(t, {}).get('status') != 'ok'
                   or r['terms'].get(t, {}).get('normalized') is None]
        if missing:
            incomplete.append({
                'scenario': r.get('name'),
                'kind': kind,
                'missing_terms': missing,
            })
    return incomplete


def _aggregate_overall(scenario_results, manifest, quality=None):
    terms = {k: _aggregate_term(scenario_results, k) for k in TERM_ORDER}
    complete = all(terms[k]['status'] == 'ok' for k in TERM_ORDER)
    # Required-scenario completeness: any real motion segment that did not
    # produce its required T/O (e.g. a legal step the mechanism never followed)
    # makes the aggregate incomplete even if other segments scored fine.
    incomplete_required = _incomplete_required_scenarios(scenario_results)
    required_complete = not incomplete_required
    complete = complete and required_complete
    if complete:
        l_norm = sum(terms[k]['contribution'] for k in TERM_ORDER)
        l_term = {'term': 'L', 'raw': l_norm, 'normalized': l_norm,
                  'weight': sum(TERM_WEIGHTS.values()), 'contribution': l_norm,
                  'count': sum(r['count'] for r in scenario_results),
                  'status': 'ok', 'note': None}
    elif not required_complete:
        l_term = {'term': 'L', 'raw': None, 'normalized': None,
                  'weight': sum(TERM_WEIGHTS.values()), 'contribution': None,
                  'count': sum(r['count'] for r in scenario_results),
                  'status': 'incomplete_required_scenarios',
                  'note': 'required T/O incomplete in: ' + '; '.join(
                      f"{x['scenario']}({x['kind']}:{','.join(x['missing_terms'])})"
                      for x in incomplete_required)}
    else:
        missing = [k for k in TERM_ORDER if terms[k]['status'] != 'ok']
        l_term = {'term': 'L', 'raw': None, 'normalized': None,
                  'weight': sum(TERM_WEIGHTS.values()), 'contribution': None,
                  'count': sum(r['count'] for r in scenario_results),
                  'status': 'insufficient_terms',
                  'note': 'missing/inapplicable: ' + ','.join(missing)}
    terms['L'] = l_term

    safety_ok = _has_explicit_safety(manifest)
    quality_ok = _quality_ok(manifest, quality)
    reasons = []
    if not quality_ok:
        reasons.append('dataset quality not proven good (valid!=true or errors present or absent)')
    if not required_complete:
        reasons.append('incomplete_required_scenarios: ' + '; '.join(
            f"{x['scenario']}({x['kind']}:{','.join(x['missing_terms'])})"
            for x in incomplete_required))
    if not complete and required_complete:
        reasons.append('incomplete six-term loss (missing/inapplicable terms)')
    if not safety_ok:
        reasons.append('missing explicit safety_envelope provenance/approval')
    reasons.append('no repeated trials / uncertainty quantification in M1')
    reasons.append('M1 never produces writable or promotable candidates')
    gate = {
        'promotable': False,
        'quality_pass': quality_ok,
        'safety_pass': safety_ok,
        'reasons': reasons,
    }
    overall_status = 'complete' if complete else l_term['status']
    return {
        'terms': terms,
        'L': l_term['normalized'],
        'status': overall_status,
        'incomplete_required_scenarios': incomplete_required,
        'count': sum(r['count'] for r in scenario_results),
        'gate': gate,
    }


def _has_explicit_safety(manifest):
    env = manifest.get('safety_envelope')
    if not isinstance(env, dict) or not env:
        return False
    has_prov = bool(env.get('provenance') or env.get('source'))
    has_appr = bool(env.get('approved') or env.get('approval') or env.get('approval_record'))
    return has_prov and has_appr


def _quality_ok(manifest, quality):
    """Whether the dataset's declared quality is actually good (never constant).

    Quality must be present, be a dict, report ``valid is True`` and carry an
    empty ``errors`` list (or a nested ``gate`` with ``valid is True``/no
    errors). Absent/ambiguous quality is treated as NOT proven good.
    """
    if not isinstance(quality, dict):
        return False
    gate = quality.get('gate') if isinstance(quality.get('gate'), dict) else None
    valid = quality.get('valid', gate.get('valid') if gate else None)
    errors = quality.get('errors')
    if errors is None and gate is not None:
        errors = gate.get('errors')
    if errors is None:
        errors = []
    return valid is True and isinstance(errors, list) and not errors


# --------------------------------------------------------------------------- #
# Candidate comparison.
# --------------------------------------------------------------------------- #
def _scenario_signature(scenarios):
    """Structural signature: name/kind + the full step/window definition.

    Two runs whose scenarios merely share a name+kind but differ in amplitude,
    direction, step position or half-open window are NOT comparable — a bigger
    step or a different direction is a different experiment.
    """
    sig = []
    for s in scenarios:
        sig.append((
            str(s.get('name')), str(s.get('kind')),
            s.get('i0'), s.get('i1'), s.get('step_index'),
            _canonical_float(s.get('pre_target_rad')),
            _canonical_float(s.get('post_target_rad')),
            _canonical_float(s.get('amplitude_rad')),
            str(s.get('direction')),
        ))
    return sorted(sig)


def compare_runs(baseline, candidate):
    """Compare two scored runs for comparability and promotion gating.

    Never silently compares different axis / scenarios / context / frozen
    scales. Baseline and candidate param_epochs may legitimately differ; each
    run must be internally single-version. M1 never promotes.
    """
    if not isinstance(baseline, dict) or not isinstance(candidate, dict):
        raise ValueError('baseline and candidate must be scored run dicts')

    reasons = []
    comparable = True

    def fail(msg):
        nonlocal comparable
        comparable = False
        reasons.append(msg)

    if baseline.get('axis') != candidate.get('axis'):
        fail(f"axis differs: {baseline.get('axis')!r} vs {candidate.get('axis')!r}")

    if baseline.get('scale_fingerprint') != candidate.get('scale_fingerprint'):
        fail('frozen scale fingerprint differs')

    if (baseline.get('context') or {}) != (candidate.get('context') or {}):
        fail('experiment context differs (scene/load/temperature/session)')

    if _scenario_signature(baseline.get('scenarios', [])) != _scenario_signature(candidate.get('scenarios', [])):
        fail('scenario set differs (name/kind/window/amplitude/direction/step definition)')

    # Each run must be internally single-version per scenario (enforced at score
    # time); disclose the parameter identities.
    b_epochs = baseline.get('param_epochs', [])
    c_epochs = candidate.get('param_epochs', [])
    if len(b_epochs) > 1 or len(c_epochs) > 1:
        fail('a run mixes multiple param_epochs')

    b_over = baseline.get('overall', {})
    c_over = candidate.get('overall', {})

    # Deltas / improvement are only meaningful for comparable runs. When not
    # comparable we emit null (never a misleading difference).
    deltas = {}
    for k in TERM_ORDER + ['L']:
        if not comparable:
            deltas[k] = None
            continue
        bt = b_over.get('terms', {}).get(k, {})
        ct = c_over.get('terms', {}).get(k, {})
        if bt.get('normalized') is not None and ct.get('normalized') is not None:
            deltas[k] = ct['normalized'] - bt['normalized']
        else:
            deltas[k] = None

    bl = b_over.get('L')
    cl = c_over.get('L')
    if not comparable:
        bl_cmp = cl_cmp = None
        meets_5pct = None
    else:
        bl_cmp, cl_cmp = bl, cl
        meets_5pct = bool(bl is not None and cl is not None and cl <= 0.95 * bl)

    # Promotion gate: needs comparability + quality + explicit safety envelope +
    # repeated meaningful improvement with uncertainty. M1 has no repeats.
    gate_reasons = []
    if not comparable:
        gate_reasons.append('runs are not comparable: ' + '; '.join(reasons))
    # M1 score objects are path/provenance-free and carry no approved
    # safety_envelope, so safety is never proven -> never promote. Promotion in
    # later milestones requires a dataset-level approved envelope + repeats.
    safety_ok = False
    gate_reasons.append('missing explicit safety_envelope provenance/approval '
                        '(not proven at score level)')
    gate_reasons.append('no repeated trials / uncertainty quantification in M1')
    if comparable:
        if not meets_5pct:
            gate_reasons.append('no measurable improvement meeting L_new <= 0.95*L_baseline')
        else:
            gate_reasons.append('improvement present but unconfirmed without repeats/uncertainty')
    gate_reasons.append('M1 never produces promotable or writable candidates')
    promotable = False  # M1: never promote, never writable.

    return {
        'schema_version': SCHEMA_VERSION,
        'comparable': comparable,
        'reasons': reasons,
        'baseline': {'param_epochs': b_epochs, 'L': bl},
        'candidate': {'param_epochs': c_epochs, 'L': cl},
        'deltas': deltas,
        'improvement': {'L_baseline': bl_cmp, 'L_candidate': cl_cmp, 'meets_5pct': meets_5pct},
        'gate': {
            'promotable': promotable,
            'quality_pass': bool(b_over.get('gate', {}).get('quality_pass')) and bool(c_over.get('gate', {}).get('quality_pass')),
            'safety_pass': safety_ok,
            'reasons': gate_reasons,
        },
    }


if __name__ == '__main__':  # pragma: no cover - manual smoke only
    import sys
    if len(sys.argv) > 1:
        with open(sys.argv[1], encoding='utf-8') as fh:
            data = json.load(fh)
        print(json.dumps(score_dataset(data), ensure_ascii=False, indent=2))
    else:
        print('usage: autotune_score.py <dataset.json>')
