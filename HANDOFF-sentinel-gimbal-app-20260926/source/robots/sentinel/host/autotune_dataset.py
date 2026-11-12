"""Autotune M0/M1 offline dataset loader and validator.

SYNTHETIC / NOT_HARDWARE_VALIDATED.  This module only reads offline CSV/JSONL/ECRAW1
files.  It never opens a serial port, probe or device path, never writes hardware
parameters and never acts on motors.  Data validity computed here is NOT scenario
sufficiency and NOT hardware safety approval.

Frozen API (see source/docs/autotune/IMPLEMENTATION_CONTRACT.md):
    load_dataset(path) -> {schema_version, manifest, samples, quality, events}
    validate_samples(manifest, samples) -> {schema_version, valid, errors,
        warnings, segments, fields, counts, time}

Standard library only.
"""
from __future__ import annotations

import csv
import json
import math
import struct
from pathlib import Path

import sentinel_raw as raw

SCHEMA_VERSION = 1
LABEL = 'NOT_HARDWARE_VALIDATED'
SYNTHETIC_LABEL = 'SYNTHETIC'
REAL_OFFLINE_LABEL = 'REAL_OFFLINE'

AXES = ('fire_yaw', 'pitch')

# Canonical sample field order (frozen).  Aliases are documented in DATA_SCHEMA.md;
# units are never silently changed.
SAMPLE_FIELDS = [
    'schema_version', 'run_id', 'boot_id', 'firmware_id', 'axis',
    'param_epoch', 'sample_index', 'sequence', 'source_sequence',
    't_mcu_us', 'arrived_us', 'dt_s', 'request_target_rad',
    'adopted_target_rad', 'angle_rad', 'velocity_rad_s',
    'output_normalized', 'current_a', 'pid_p', 'pid_i', 'pid_d',
    'mit_kp', 'mit_kd', 'dm_state', 'torque_nm', 'controller_mode',
    'target_source', 'rc_valid', 'feedback_valid', 'can_tx_result',
    'fault_bits', 'sample_quality', 'missing_fields',
]
COLUMNS = list(SAMPLE_FIELDS)

# Extended columns written by write_samples (documented additions, superset of COLUMNS).
EXTRA_COLUMNS = ['requested_frame', 'adopted_frame', 'limited',
                 'feedback_received_us', 'feedback_age_us', 'torque_source']

_INT_FIELDS = {'schema_version', 'param_epoch', 'sample_index', 'sequence',
               'source_sequence', 't_mcu_us', 'arrived_us', 'fault_bits'}
_FLOAT_FIELDS = {'dt_s', 'request_target_rad', 'adopted_target_rad', 'angle_rad',
                 'velocity_rad_s', 'output_normalized', 'current_a', 'pid_p',
                 'pid_i', 'pid_d', 'mit_kp', 'mit_kd', 'torque_nm'}
_BOOL_FIELDS = {'rc_valid', 'feedback_valid'}
_STR_FIELDS = {'run_id', 'boot_id', 'firmware_id', 'axis', 'controller_mode',
               'target_source', 'can_tx_result', 'sample_quality'}
_JSON_FIELDS = {'missing_fields'}

CONTROL_MODES = {0: 'none', 1: 'current', 2: 'velocity', 3: 'position'}
# DjiGroupStatus -> text (source/include/ec/drivers/dji_device.hpp:58)
GROUP_STATUS = {0: 'not_sent', 1: 'accepted', 2: 'invalid_config',
                3: 'missing_output', 4: 'stale_output', 5: 'invalid_output',
                6: 'busy', 7: 'error'}
STOP_SOURCES = {'manual_stop', 'stop', 'none'}

# Frozen AutoTune Kind6 data0 flag bits (firmware authoritative layout).  Bits not
# listed here are undefined at this revision and are NOT interpreted as truth.
KIND6_SAMPLE_FLAG_BITS = {
    0: 'request_valid', 1: 'adopted_valid', 2: 'pos_valid_bit',
    3: 'vel_valid_bit', 4: 'output_valid', 5: 'enabled', 6: 'feedback_valid',
    7: 'timing_error',
    # 8-9 controller_mode (enum), 10-15 fault_mask, 16-19 posValidity,
    # 20-23 velValidity, 24 rc_valid, 25 manual_stop, 26 executing, 27 fault_present,
    # 28-31 target_source (not derivable from mode).
}
KIND6_CFG_FAULT_MASK = {0: 'executor_rejected', 1: 'unreachable',
                        2: 'feedback_lost', 3: 'ref_lost', 4: 'device_fault',
                        5: 'restarted'}
# target_source carried directly in bits 28-31 (do NOT infer from controller_mode).
KIND6_TARGET_SOURCE_BITS = (28, 31)
# Kind7 data0 provenance: which physical attempt the aux frame actually describes.
KIND7_FRAME_PROVENANCE = {1: 'pitch_actual_attempt', 2: 'fire_reconstructed_nonactual'}

# How samples are paired across Kind6 (sample) and Kind7 (control/aux) records.
# The key NEVER includes the local ECRAW1 ``sequence`` -- that is the per-record
# uniqueness counter (state N, aux N+1) owned by recorder framing; pairing identity
# is the physical origin (stream + source_sequence + sampled_us + run).  The time
# component is ``sampled_us``; state.arrived is the real commit time and aux.arrived
# is merely route arrival, so neither arrival time is used as a measurement instant.
PAIR_KEY_NAMES = ('stream', 'source_sequence', 'sampled_us', 'run')

_DEVICE_HINTS = ('/dev/', 'serial://', 'serial:', 'com1', 'com2', 'com3', 'com4',
                 'ttyusb', 'ttyacm', '\\\\\\.\\')


# --------------------------------------------------------------------- utils
def _reject_device(path):
    text = str(path).lower().replace('\\', '/')
    if any(hint in text for hint in ('/dev/', 'serial://', 'serial:', 'ttyusb',
                                     'ttyacm')):
        raise ValueError(f'拒绝设备路径, 仅支持离线文件: {path}')
    if text.startswith('com') and text[3:4].isdigit():
        raise ValueError(f'拒绝串口设备名, 仅支持离线文件: {path}')


def _guard_file(path, role='file'):
    """Safety gate for every file this module will actually open.

    The offline manifest is untrusted input: its ``files.samples`` / ``files.raw``
    entries (and any ECRAW1 trailer references) could point -- directly or via a
    symlink -- at a character device, FIFO, socket or block device under ``/dev`` or
    elsewhere, which would turn a pure-offline loader into a device reader (or hang
    on a FIFO).  So for the *resolved real* path we reject device URIs / ``/dev/``
    and require a regular file: FIFO / socket / char / block are all refused.  A
    regular file reached through a symlink into a normal directory is still allowed.
    """
    raw = Path(path)
    # Reject on both the literal and the symlink-resolved text so a symlink whose
    # *name* looks innocent but whose target is /dev/... is also caught.
    _reject_device(path)
    try:
        resolved = raw.resolve(strict=False)
    except (OSError, RuntimeError) as exc:  # pragma: no cover - defensive
        raise ValueError(f'{role} 路径无法解析, 拒绝打开: {path} ({exc})')
    _reject_device(resolved)
    try:
        st = resolved.stat()
    except FileNotFoundError:
        raise ValueError(f'{role} 文件不存在: {path}')
    except OSError as exc:
        raise ValueError(f'{role} 路径不可访问, 拒绝打开: {path} ({exc})')
    import stat as _stat
    if not _stat.S_ISREG(st.st_mode):
        kind = ('FIFO' if _stat.S_ISFIFO(st.st_mode) else
                'socket' if _stat.S_ISSOCK(st.st_mode) else
                '字符/块设备' if (_stat.S_ISCHR(st.st_mode) or
                                 _stat.S_ISBLK(st.st_mode)) else '非普通文件')
        raise ValueError(f'{role} 必须是普通文件, 拒绝打开 ({kind}): {path}')
    return resolved


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _finite_or_none(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, float) and math.isnan(value):
        return None
    return value


def _coerce_field(name, value, where, non_finite):
    """Strict conversion for one canonical field.  Returns (value, error_or_None)."""
    if value is None:
        return None, None
    if name in _JSON_FIELDS:
        if isinstance(value, dict):
            return value, None
        try:
            parsed = json.loads(value) if isinstance(value, str) else value
        except (ValueError, TypeError) as exc:
            return None, f'{name}: 无法解析 JSON ({exc})'
        return (parsed if isinstance(parsed, dict) else None), None
    if name in _BOOL_FIELDS:
        if isinstance(value, bool):
            return value, None
        if isinstance(value, int) and value in (0, 1):
            return bool(value), None
        text = str(value).strip().lower()
        if text in ('true', '1', 'yes', 'y', 'on'):
            return True, None
        if text in ('false', '0', 'no', 'n', 'off'):
            return False, None
        return None, f'{name}: 无法把 {value!r} 严格转换为 bool'
    if name in _INT_FIELDS:
        if isinstance(value, bool):
            return None, f'{name}: bool 不能当整数'
        if isinstance(value, int):
            return value, None
        if isinstance(value, float):
            if not math.isfinite(value) or value != int(value):
                return None, f'{name}: 非整数值 {value!r}'
            return int(value), None
        text = str(value).strip()
        if text == '':
            return None, None
        try:
            return int(text, 0), None
        except ValueError:
            try:
                f = float(text)
            except ValueError:
                return None, f'{name}: 无法解析整数 {value!r}'
            if not math.isfinite(f) or f != int(f):
                return None, f'{name}: 非整数值 {text!r}'
            return int(f), None
    if name in _FLOAT_FIELDS:
        if isinstance(value, bool):
            return None, f'{name}: bool 不能当浮点'
        if _is_number(value):
            val = float(value)
        else:
            text = str(value).strip()
            if text == '':
                return None, None
            try:
                val = float(text)
            except ValueError:
                return None, f'{name}: 无法解析浮点 {value!r}'
        if not math.isfinite(val):
            non_finite.append({'field': name, 'where': where})
            return None, None
        return val, None
    # strings and passthrough (dm_state)
    if isinstance(value, (dict, list)):
        return value, None
    return value, None


def _normalize_sample(raw_row, where, non_finite):
    """Return (canonical_sample, errors).  Missing fields get a reason in missing_fields."""
    errors = []
    if not isinstance(raw_row, dict):
        return None, [f'{where}: 样本不是对象']
    sample = {}
    for field in SAMPLE_FIELDS:
        value, err = _coerce_field(field, raw_row.get(field), where, non_finite)
        if err:
            errors.append(err)
        sample[field] = value
    # Preserve declared missing_fields exactly (row-level reasons).  Nulls without a
    # declared reason still get one in validate_samples' per-field tally, so we do not
    # mutate the row here (keeps CSV export->reload byte-stable).
    declared = sample.get('missing_fields')
    sample['missing_fields'] = dict(declared) if isinstance(declared, dict) else {}
    # keep harmless documented extras when present
    for extra in EXTRA_COLUMNS:
        if extra in raw_row:
            sample[extra] = raw_row[extra]
    return sample, errors


# -------------------------------------------------------------------- ECRAW1
# ----------------------------------------------------- AutoTune trace decoders
# Frozen Kind6/Kind7 byte layouts.  These are injected into ``sentinel_raw`` so the
# base decoder does not hard-code unfinished formats; the record header (version/kind/
# stream/flags/sequence/source/sampled/arrived/lost/error/run) is decoded generically
# by ``sentinel_raw.decode_record``.  Values 1..5 keep their historical layout.

def _pack_word(raw, offset):
    return struct.unpack_from('<I', raw, offset)[0]


def decode_autotune_sample(raw):
    """Kind6 ``autotune_sample`` (firmware authoritative layout).

    data0 (word at offset 32) = sample flags:
      bits0-7  request_valid/adopted_valid/pos_valid_bit/vel_valid_bit/output_valid/
               enabled/feedback_valid/timing_error
      bits8-9  controller_mode (enum)
      bits10-15 fault_mask
      bits16-19 posValidity; bits20-23 velValidity
      bit24 rc_valid; bit25 manual_stop; bit26 executing; bit27 fault_present
      bits28-31 target_source (carried directly; NOT inferred from mode)
    data1 (offset 36) = request target (float32, request_target_rad).
    float words (offset 40..60) = adopted, angle, rate, output, dt, param_epoch_words.
    data7 (offset 60) = full 32-bit param_epoch (firmware writes out.state.data[7]).
    NaN float32 -> ``None`` (evidence), never 0.  All 32 bits of the epoch preserved.
    """
    flags_word = struct.unpack_from('<I', raw, 32)[0]
    epoch_raw = struct.unpack_from('<I', raw, 60)[0]
    values = struct.unpack_from('<5f', raw, 36)
    tf_lo, tf_hi = KIND6_TARGET_SOURCE_BITS
    tf_width = tf_hi - tf_lo + 1

    def _f(v):
        return None if not math.isfinite(v) else v

    return {
        'sample_flags': flags_word,
        'request_valid': bool((flags_word >> 0) & 1),
        'adopted_valid': bool((flags_word >> 1) & 1),
        'pos_validity_bit': (flags_word >> 2) & 1,
        'vel_validity_bit': (flags_word >> 3) & 1,
        'output_valid': bool((flags_word >> 4) & 1),
        'enabled': bool((flags_word >> 5) & 1),
        'feedback_valid': bool((flags_word >> 6) & 1),
        'timing_error': bool((flags_word >> 7) & 1),
        'controller_mode': (flags_word >> 8) & 0x3,
        'fault_mask': (flags_word >> 10) & 0x3f,
        'pos_validity': (flags_word >> 16) & 0xf,
        'vel_validity': (flags_word >> 20) & 0xf,
        'rc_valid': bool((flags_word >> 24) & 1),
        'manual_stop': bool((flags_word >> 25) & 1),
        'executing': bool((flags_word >> 26) & 1),
        'fault_present': bool((flags_word >> 27) & 1),
        'target_source_id': (flags_word >> tf_lo) & ((1 << tf_width) - 1),
        'param_epoch_raw': epoch_raw,
        # data1=request target(36); data2..data6 = adopted/angle/rate/output/dt (40..56); data7=epoch(60).
        'request_target_rad': _f(values[0]),
        'adopted_target_rad': _f(values[1]),
        'angle_rad': _f(values[2]),
        'velocity_rad_s': _f(values[3]),
        'output_normalized': _f(values[4]),
        'dt_s': _f(struct.unpack_from('<f', raw, 56)[0]),
    }


def decode_autotune_aux(raw):
    """Kind7 ``autotune_control`` aux (firmware authoritative layout).

    data0 = frame.id (bits0-15) | provenance<<16 (1 Pitch actual attempt,
            2 Fire reconstructed non-actual)
    data1 = size (byte0) | frame_kind<<8
    data2 = byte (offset40 low byte region)
    data3 = byte
    data4 = transport(b0) | dm_state(b1, 0xFF unknown) | feedback_valid<<16 |
            timing_error<<17
    data5/6/7 = Fire/Pitch kp, kd, torque (float32).
    ``arrived`` on aux is merely route arrival, not a measurement instant.
    """
    data0 = struct.unpack_from('<I', raw, 32)[0]
    data1 = struct.unpack_from('<I', raw, 36)[0]
    data2 = struct.unpack_from('<I', raw, 40)[0]
    data3 = struct.unpack_from('<I', raw, 44)[0]
    data4 = struct.unpack_from('<I', raw, 48)[0]
    floats = struct.unpack_from('<3f', raw, 52)

    def _b(word, idx):
        return (word >> (8 * idx)) & 0xff

    def _f(v):
        return None if not math.isfinite(v) else v

    dm_state_byte = _b(data4, 1)
    torque = _f(floats[2])
    feedback_age = (struct.unpack_from('<I', raw, 12)[0] -
                    struct.unpack_from('<I', raw, 16)[0]) & 0xffffffff
    if raw[2] == 6 and (not (data4 & (1 << 16)) or feedback_age >= 0x80000000):
        torque = None
    return {
        'frame_id': data0 & 0xffff,
        'provenance': (data0 >> 16) & 0xffff,
        'frame_size': data1 & 0xff,
        'frame_kind': (data1 >> 8) & 0xff,
        'data2': data2,
        'data3': data3,
        'transport': _b(data4, 0),
        'dm_state': None if dm_state_byte == 0xff else dm_state_byte,
        'feedback_valid': bool((data4 >> 16) & 1),
        'timing_error': bool((data4 >> 17) & 1),
        'mit_kp': _f(floats[0]),
        'mit_kd': _f(floats[1]),
        'torque_nm': torque,
        'feedback_received_us': struct.unpack_from('<I', raw, 16)[0] if torque is not None and raw[2] == 6 else None,
        'feedback_age_us': feedback_age if torque is not None and raw[2] == 6 else None,
        'torque_source': 'esc_reported' if torque is not None and raw[2] == 6 else None,
    }


def install_trace_decoders():
    """Register the frozen Kind6/7 decoders with ``sentinel_raw`` (idempotent)."""
    raw.register_trace_decoder(6, decode_autotune_sample)
    raw.register_trace_decoder(7, decode_autotune_aux)


def _fault_reasons(fault_mask):
    reasons = []
    bit = 0
    while fault_mask:
        if fault_mask & 1:
            reasons.append(KIND6_CFG_FAULT_MASK.get(bit, f'bit{bit}'))
        fault_mask >>= 1
        bit += 1
    return reasons


_AUTOTUNE_OPTIONAL_NULL = ('current_a', 'pid_p', 'pid_i', 'pid_d', 'mit_kp',
                           'mit_kd', 'dm_state', 'torque_nm')

def _pair_traces(records, events, file_name):
    """Pair Kind6 state with Kind7 aux by physical identity.

    PAIR key = (stream, source_sequence, sampled_us, run) -- NEVER the local ECRAW1
    ``sequence``.  Each physical sample spans two records whose own ``sequence`` must
    be an adjacent strictly-increasing pair (state N, aux N+1); a broken N/N+1 adjacency
    or a missing/ambiguous aux record keeps the sample with an explicit error event
    (no synthetic pair, no sort that would hide the gap).
    """
    samples6 = [r for r in records if r.get('kind') == 'autotune_sample']
    aux7 = [r for r in records if r.get('kind') == 'autotune_control']
    index7 = {}
    for a in aux7:
        key = (a['stream'], a['source_sequence'], a['sampled_us'], a['run'])
        index7.setdefault(key, []).append(a)
    paired = []
    duplicates = 0
    missing = 0
    nonadjacent = 0
    for s in samples6:
        key = (s['stream'], s['source_sequence'], s['sampled_us'], s['run'])
        matches = index7.get(key)
        if matches is None:
            missing += 1
            paired.append((s, None))
            continue
        if len(matches) > 1:
            duplicates += 1
            events.append(dict(code='duplicate_pair', severity='error',
                               detail=f'{file_name}: 身份 {key} 有 {len(matches)} 条 Kind7 记录, 无法唯一配对',
                               identity=list(key)))
        aux = matches[0]
        # sequence adjacency: aux.sequence must be state.sequence + 1
        if aux['sequence'] != s['sequence'] + 1:
            nonadjacent += 1
            events.append(dict(code='pair_sequence_not_adjacent', severity='warning',
                               detail=f'{file_name}: state#{s["sequence"]} 与 aux#{aux["sequence"]} '
                                      f'非 N/N+1 相邻 (按物理身份仍配对, 但记录疑似被中间记录打断)',
                               state_sequence=s['sequence'], aux_sequence=aux['sequence']))
        paired.append((s, aux))
    if missing:
        events.append(dict(code='missing_pair_record', severity='error',
                           detail=f'{file_name}: {missing} 条 Kind6 样本缺 Kind7 aux 记录, 保留样本并标错',
                           file=file_name, missing=missing))
        # A genuinely absent control record IS the missing bundle (not a null output).
        events.append(dict(code='missing_bundle', severity='error',
                           detail=f'{file_name}: {missing} 条样本缺控制记录, 视为 missing bundle',
                           file=file_name, missing=missing))
    if duplicates:
        events.append(dict(code='ambiguous_pair_record', severity='error',
                           detail=f'{file_name}: {duplicates} 条 Kind6 样本 Kind7 配对不唯一',
                           file=file_name, duplicates=duplicates))
    if nonadjacent:
        events.append(dict(code='nonadjacent_pairs', severity='warning',
                           detail=f'{file_name}: {nonadjacent} 对记录 sequence 不连续',
                           file=file_name, nonadjacent=nonadjacent))
    return paired, missing, duplicates


def install_trace_decoders_if_needed():  # pragma: no cover - thin alias
    install_trace_decoders()


def _samples_from_raw(path, manifest, quality, events):
    install_trace_decoders()
    metadata, rows = raw.load(path)
    file_name = path.name
    axis = manifest.get('axis')
    param_epoch = (manifest.get('param_set') or {}).get('param_epoch')
    fw = manifest.get('firmware') or {}
    firmware_id = fw.get('firmware_id') or fw.get('build_id') or fw.get('id')
    boot_id = manifest.get('boot_id')
    run_id = manifest.get('run_id') or f'run-{metadata.get("run")}'

    # New trace (Kind6/7) takes precedence when present; legacy Kind5 'control' is the
    # fallback so an old physical log is never mis-synthesized into new traces.
    has_kind6 = any(r.get('kind') == 'autotune_sample' for r in rows)
    samples = []
    control_seen = 0
    ignored = 0

    if has_kind6:
        paired, missing, duplicates = _pair_traces(rows, events, file_name)
        for seq6, aux7 in paired:
            control_seen += 1
            samples.append(_build_autotune_sample(
                seq6, aux7, manifest, metadata, events, file_name, axis, run_id,
                boot_id, firmware_id, param_epoch, len(samples), quality))
    else:
        for row in rows:
            if row['kind'] != 'control':
                ignored += 1
                continue
            control_seen += 1
            samples.append(_legacy_kind5_sample(row, path, manifest, axis, run_id,
                                               boot_id, firmware_id, param_epoch,
                                               len(samples), quality))

    if control_seen == 0:
        events.append(dict(code='no_control_records', severity='error',
                           detail=f'{file_name}: 物理日志无 control 记录, 不合成控制样本',
                           file=file_name, ignored_records=ignored))
    else:
        events.append(dict(code='control_records_loaded', severity='info',
                           detail=f'{file_name}: {control_seen} 条控制样本',
                           file=file_name, ignored_records=ignored))
    return samples


def _build_autotune_sample(seq6, aux7, manifest, metadata, events, file_name, axis,
                           run_id, boot_id, firmware_id, param_epoch, idx, quality):
    missing = {}
    for name in _AUTOTUNE_OPTIONAL_NULL:
        missing[name] = 'not_in_autotune_sample'

    # controller_mode from bits8-9 of the sample flags; unknown if it maps to nothing.
    controller_mode = CONTROL_MODES.get(seq6.get('controller_mode'))
    if controller_mode is None:
        missing['controller_mode'] = 'mode_unknown'

    # target_source comes directly from bits28-31 (never inferred from mode).
    ts_id = seq6.get('target_source_id')
    target_source = manifest.get('target_source_map', {}).get(ts_id, f'ts_{ts_id}')
    if ts_id is None:
        missing['target_source'] = 'target_source_bits_absent'

    # Fault: fault_present bit OR nonzero fault_mask.  Map mask bits to reasons.
    fault_mask = seq6.get('fault_mask', 0)
    fault_reasons = _fault_reasons(fault_mask)
    fault_present = bool(seq6.get('fault_present')) or bool(fault_mask)
    fault_bits = 0
    for name in fault_reasons:
        fault_bits |= (1 << list(KIND6_CFG_FAULT_MASK.keys())[
            list(KIND6_CFG_FAULT_MASK.values()).index(name)])
    if fault_present and not fault_mask:
        fault_bits = 1  # fault_present set but no named mask bits

    rc_valid = bool(seq6.get('rc_valid'))
    manual_stop = bool(seq6.get('manual_stop'))
    executing = bool(seq6.get('executing'))

    # epoch: prefer firmware per-sample full 32-bit epoch; 0xffffffff means unknown.
    epoch = seq6.get('param_epoch_raw')
    if epoch in (None, 0xffffffff):
        epoch = param_epoch
        if epoch is None:
            missing['param_epoch'] = 'unknown'

    # aux-derived fields (Kind7); only fill when truly carried (not sentinel).
    if aux7 is not None:
        if aux7.get('mit_kp') is not None:
            missing.pop('mit_kp', None)
        if aux7.get('mit_kd') is not None:
            missing.pop('mit_kd', None)
        if aux7.get('torque_nm') is not None:
            missing.pop('torque_nm', None)
        if aux7.get('dm_state') is not None:
            missing.pop('dm_state', None)
    feedback_valid = seq6.get('feedback_valid')
    if aux7 is not None and aux7.get('feedback_valid') is not None:
        feedback_valid = bool(aux7['feedback_valid'])

    adopted = seq6.get('adopted_target_rad')
    if adopted is None:
        missing['adopted_target_rad'] = 'no_target'
    if seq6.get('angle_rad') is None:
        missing.setdefault('angle_rad', 'non_finite_or_missing')
    if seq6.get('output_normalized') is None:
        missing.setdefault('output_normalized', 'non_finite_or_missing')
    if boot_id is None:
        missing['boot_id'] = 'unknown'
    # Only an actually unknown per-record epoch (or a missing manifest fallback)
    # may be declared missing.  A finite firmware epoch must never be marked unknown.
    if epoch in (None, 0xffffffff):
        missing['param_epoch'] = 'unknown'

    return dict(
        schema_version=SCHEMA_VERSION,
        run_id=run_id,
        boot_id=boot_id,
        firmware_id=firmware_id,
        axis=axis,
        param_epoch=epoch,
        sample_index=idx,
        sequence=seq6['sequence'],
        source_sequence=seq6['source_sequence'],
        t_mcu_us=seq6['sampled_us'],
        arrived_us=seq6['arrived_us'],
        dt_s=seq6.get('dt_s'),
        request_target_rad=seq6.get('request_target_rad'),
        adopted_target_rad=adopted,
        angle_rad=seq6.get('angle_rad'),
        velocity_rad_s=seq6.get('velocity_rad_s'),
        output_normalized=seq6.get('output_normalized'),
        current_a=None,
        pid_p=None, pid_i=None, pid_d=None,
        mit_kp=(aux7 or {}).get('mit_kp'),
        mit_kd=(aux7 or {}).get('mit_kd'),
        dm_state=(aux7 or {}).get('dm_state'),
        torque_nm=(aux7 or {}).get('torque_nm'),
        controller_mode=controller_mode,
        target_source=target_source,
        rc_valid=rc_valid,
        feedback_valid=feedback_valid,
        can_tx_result=None,
        fault_bits=fault_bits,
        sample_quality='ok' if not missing else 'partial',
        missing_fields=missing,
        requested_frame=None,
        adopted_frame=None,
        limited=None,
        fault_reasons=fault_reasons,
        manual_stop=manual_stop,
        executing=executing,
        frame_provenance=KIND7_FRAME_PROVENANCE.get((aux7 or {}).get('provenance')),
        record_sequence_kind7=(aux7['sequence'] if aux7 else None),
        feedback_received_us=(aux7 or {}).get('feedback_received_us'),
        feedback_age_us=(aux7 or {}).get('feedback_age_us'),
        torque_source=(aux7 or {}).get('torque_source'),
    )

def _legacy_kind5_sample(row, path, manifest, axis, run_id, boot_id, firmware_id,
                         param_epoch, idx, quality):
    values = row['values']
    non_finite = []
    if any(v is None for v in values):
        for name, v in zip(('request_target_rad', 'adopted_target_rad',
                            'angle_rad', 'velocity_rad_s',
                            'output_normalized', 'dt_s'), values):
            if v is None:
                non_finite.append({'field': name, 'where': f'raw:{path.name}#{row["sequence"]}'})
        quality.setdefault('non_finite', []).extend(non_finite)
    state_bits = row.get('state_bits', 0)
    mode = CONTROL_MODES.get(state_bits & 0xff)
    pos_validity = (state_bits >> 16) & 0xf
    feedback_valid = True if pos_validity == 1 else (None if pos_validity == 0 else False)
    send_result = row.get('send_result')
    missing = {}
    for name in _AUTOTUNE_OPTIONAL_NULL:
        missing[name] = 'not_in_control_record'
    if values[0] is None:
        missing['request_target_rad'] = 'non_finite_or_missing'
    if values[1] is None:
        missing['adopted_target_rad'] = 'no_target'
    if values[4] is None:
        missing['output_normalized'] = 'non_finite_or_missing'
    if row.get('error_us') is None:
        missing['error_us'] = 'timing_error_unknown'
    if boot_id is None:
        missing['boot_id'] = 'unknown'
    return dict(
        schema_version=SCHEMA_VERSION, run_id=run_id, boot_id=boot_id,
        firmware_id=firmware_id, axis=axis, param_epoch=param_epoch,
        sample_index=idx, sequence=row['sequence'],
        source_sequence=row['source_sequence'], t_mcu_us=row['sampled_us'],
        arrived_us=row['arrived_us'], dt_s=_finite_or_none(values[5]),
        request_target_rad=values[0], adopted_target_rad=values[1],
        angle_rad=values[2], velocity_rad_s=values[3], output_normalized=values[4],
        current_a=None, pid_p=None, pid_i=None, pid_d=None,
        mit_kp=None, mit_kd=None, dm_state=None, torque_nm=None,
        controller_mode=mode, target_source=manifest.get('target_source', 'unknown'),
        rc_valid=None, feedback_valid=feedback_valid,
        can_tx_result=GROUP_STATUS.get(send_result), fault_bits=0,
        sample_quality='ok', missing_fields=missing,
        requested_frame=row.get('requested_frame'),
        adopted_frame=row.get('adopted_frame'),
        limited=bool(state_bits & (1 << 24)),
    )



def _read_jsonl(path):
    rows = []
    with Path(path).open('r', encoding='utf-8') as handle:
        for lineno, line in enumerate(handle, 1):
            text = line.strip()
            if not text:
                continue
            rows.append(json.loads(text))
    return rows


def _read_csv(path, non_finite):
    rows = []
    errors = []
    with Path(path).open('r', encoding='utf-8', newline='') as handle:
        reader = csv.DictReader(handle)
        for record in reader:
            rows.append(record)
    return rows, errors


def _load_samples_file(path, non_finite, events):
    suffix = Path(path).suffix.lower()
    if suffix == '.jsonl' or suffix == '.ndjson':
        raw_rows = _read_jsonl(path)
    elif suffix in ('.csv',):
        raw_rows, _ = _read_csv(path, non_finite)
    else:
        raise ValueError(f'不支持的样本文件类型: {path}')
    samples = []
    errors = []
    for i, row in enumerate(raw_rows):
        sample, errs = _normalize_sample(row, f'{Path(path).name}:{i}', non_finite)
        errors.extend(errs)
        if sample is not None:
            samples.append(sample)
    if errors:
        events.append(dict(code='sample_coercion_errors', severity='error',
                           detail=f'{len(errors)} 条字段转换失败', errors=errors))
    return samples


def load_dataset(path):
    """Load an offline run directory or manifest.json. Never touches a device."""
    path = Path(path)
    _reject_device(path)
    manifest = {}
    samples = []
    # Labels are decided from the *declared* data provenance, never assumed.  A
    # missing/unknown ``synthetic`` flag must NOT be silently branded SYNTHETIC -- that
    # would fabricate identity for an unknown (possibly real) log.  The offline /
    # not-hardware-validated label is always true for this module regardless.
    quality = {'labels': [LABEL], 'non_finite': [], 'fields': {}}
    events = []

    if path.is_dir():
        manifest_path = path / 'manifest.json'
        if not manifest_path.exists():
            raise ValueError(f'运行目录缺少 manifest.json: {path}')
        base = path
    elif path.exists() and path.suffix.lower() == '.json':
        manifest_path = path
        base = path.parent
    elif path.exists() and path.suffix.lower() in ('.csv', '.jsonl', '.ndjson'):
        base = path.parent
        manifest_path = None
    else:
        raise FileNotFoundError(f'路径不存在: {path}')

    if manifest_path is not None:
        safe_manifest = _guard_file(manifest_path, role='manifest.json')
        manifest = json.loads(safe_manifest.read_text(encoding='utf-8'))
        if not isinstance(manifest, dict):
            raise ValueError('manifest.json 顶层必须是对象')
        manifest.setdefault('schema_version', SCHEMA_VERSION)
        # Do NOT default synthetic to True: an undeclared provenance is unknown.

    # Resolve the data-provenance labels from what the manifest actually declares.
    #   synthetic is True  -> SYNTHETIC + NOT_HARDWARE_VALIDATED
    #   synthetic is False -> REAL_OFFLINE + NOT_HARDWARE_VALIDATED (decoded as real,
    #                         not falsely branded synthetic)
    #   missing / unknown  -> NOT_HARDWARE_VALIDATED only; identity stays unknown and a
    #                         warning is emitted.  Never fabricate a synthetic identity.
    declared_synthetic = manifest.get('synthetic')
    if declared_synthetic is True:
        quality['labels'] = [SYNTHETIC_LABEL, LABEL]
    elif declared_synthetic is False:
        quality['labels'] = [REAL_OFFLINE_LABEL, LABEL]
    else:
        quality['labels'] = [LABEL]
        events.append(dict(code='synthetic_flag_unknown', severity='warning',
                           detail='manifest.synthetic 缺失或未知, 数据来源身份未知, 不标 SYNTHETIC'))

    files = manifest.get('files') or {}
    if isinstance(files, str):
        files = {'samples': files}

    non_finite = quality['non_finite']
    if manifest_path is None:  # single samples file given directly
        samples.extend(_load_samples_file(_guard_file(path, role='samples'),
                                          non_finite, events))
    else:
        # Prefer an exported samples file; raw ECRAW1 is only a fallback.  The parent
        # exports both samples.csv and raw copies, and raw/ may also carry manifest or
        # JSON side-copies that are NOT ECRAW1 - loading both would duplicate samples.
        samples_name = files.get('samples')
        if samples_name:
            samples.extend(_load_samples_file(_guard_file(base / samples_name,
                                                          role='samples'),
                                              non_finite, events))
        else:
            for raw_name in files.get('raw', []) or []:
                raw_path = base / raw_name
                if Path(raw_name).suffix.lower() in ('.json', '.jsonl', '.csv'):
                    events.append(dict(code='skipped_non_ecraw1', severity='info',
                                       detail=f'{raw_name}: 非 ECRAW1, 已跳过', file=raw_name))
                    continue
                samples.extend(_samples_from_raw(_guard_file(raw_path, role='raw'),
                                                 manifest, quality, events))
        if not files:
            events.append(dict(code='no_files', severity='warning',
                               detail='manifest.files 为空, 无样本'))

    quality['counts'] = {'samples': len(samples)}
    # The frozen API exposes data quality on the load result too: embed the full
    # validation output so a consumer that only calls load_dataset (e.g. the CLI
    # analyze path) still sees ``valid``/``errors``/``segments``.  Extra evidence
    # gathered during loading (labels, non_finite) is preserved alongside.
    validation = validate_samples(manifest, samples)
    merged = dict(validation)
    merged['labels'] = quality.get('labels', [LABEL])
    if quality.get('non_finite'):
        merged['non_finite'] = quality['non_finite']
    return dict(schema_version=SCHEMA_VERSION, manifest=manifest, samples=samples,
                quality=merged, events=events)


# ----------------------------------------------------------------- validate
def _epoch_wrap(prev, cur):
    """True if prev->cur is a legitimate u32 wrap, not a version change."""
    return prev is not None and cur is not None and prev >= 0xfffffff0 and cur <= 0x00000010


def _check_units(manifest, errors, warnings):
    units = manifest.get('units')
    if units is None:
        warnings.append(dict(code='units_unknown', detail='manifest.units 缺失, 单位未知'))
        return
    # 与评分模块 CANONICAL_UNITS 完全一致的键与取值 (评分代理冻结). 未知多余键不报错,
    # 但缺键或取值不符即单位 schema 错误, 避免单位被静默改写.
    expected = {'angle': 'rad', 'velocity': 'rad/s', 'time': 's',
                'timestamp': 'us', 'output': 'normalized'}
    for key, want in expected.items():
        if key not in units:
            errors.append(dict(code='unit_schema_error',
                               detail=f'units 缺少 {key!r} (期望 {want!r})'))
        elif units[key] != want:
            errors.append(dict(code='unit_schema_error',
                               detail=f'units[{key!r}]={units[key]!r}, 期望 {want!r}'))


def validate_samples(manifest, samples):
    errors, warnings, segments = [], [], []
    fields = {}
    manifest = manifest or {}
    samples = samples or []

    n = len(samples)
    _check_units(manifest, errors, warnings)
    axis = manifest.get('axis')
    if axis not in AXES:
        warnings.append(dict(code='axis_unknown', detail=f'axis={axis!r} 未知'))

    max_dt = manifest.get('max_dt_s')
    bound_known = isinstance(max_dt, (int, float)) and math.isfinite(max_dt)

    if n == 0:
        errors.append(dict(code='empty', detail='无样本, 视为空/不足'))
        errors.append(dict(code='insufficient', detail='空样本不能评分'))
        return _finalize(manifest, errors, warnings, segments, fields, {}, {},
                         bound_known, max_dt, n)

    for field in SAMPLE_FIELDS:
        fields[field] = {'null': 0, 'reasons': {}}

    seen_seq = set()
    seen_t = {}
    prev_t = None
    prev_epoch = None
    prev_boot = None
    prev_source_seq = None
    epoch_known = True
    boot_known = True
    break_points = {0, n}
    scenario_breaks = set()
    for sc in manifest.get('scenarios') or []:
        i0 = int(sc.get('i0', 0))
        i1 = int(sc.get('i1', 0))
        if 0 < i0 < n:
            scenario_breaks.add(i0)
        if 0 < i1 < n:
            scenario_breaks.add(i1)

    # Identity references: run/axis/firmware are established from the FIRST SAMPLE, not
    # the manifest label.  A manifest run_id is a packaging label ("r0") and may legitimately
    # differ from per-sample run tags ("r"); what matters for corruption is that samples do
    # not MIX different run/axis/firmware identities among themselves.
    ref_run = None
    ref_axis = axis
    ref_fw = None
    ref_boot = None  # boot identity comes from samples; manifest boot_id is a label

    for i, s in enumerate(samples):
        # --- identity consistency: run / axis / firmware / boot must not mix ---
        run = s.get('run_id')
        if ref_run is None:
            ref_run = run
        elif run is not None and run != ref_run:
            errors.append(dict(code='mixed_run',
                               detail=f'run_id {run!r} 与首个样本 {ref_run!r} 混用', index=i))
            break_points.add(i)
        elif ref_run is None:
            ref_run = run
        ax = s.get('axis')
        if ref_axis is None:
            ref_axis = ax
        elif ax is not None and ax != ref_axis:
            errors.append(dict(code='mixed_axis',
                               detail=f'axis {ax!r} 与 {ref_axis!r} 混用', index=i))
            break_points.add(i)
        fw = s.get('firmware_id')
        if ref_fw is None:
            ref_fw = fw
        elif fw is not None and fw != ref_fw:
            errors.append(dict(code='mixed_firmware',
                               detail=f'firmware_id {fw!r} 与 {ref_fw!r} 混用', index=i))
            break_points.add(i)

        boot = s.get('boot_id')
        if boot is None:
            boot_known = False
            if prev_boot is not None or i == 0:
                pass  # unknown boot is retained, never defaulted to a known run
        elif ref_boot is None:
            ref_boot = boot
        elif boot != ref_boot:
            errors.append(dict(code='boot_restart',
                               detail=f'boot {ref_boot}->{boot}', index=i))
            break_points.add(i)
            seen_seq.clear()
            seen_t.clear()
            prev_t = None
            prev_epoch = None
            prev_source_seq = None
            ref_boot = boot
        prev_boot = boot

        # --- epoch: ANY change is mixed (v2 revision is not the same version);
        #     a u32 wrap is still a different revision and must NOT be excused. ---
        epoch = s.get('param_epoch')
        if epoch is None:
            epoch_known = False
        elif prev_epoch is not None and epoch != prev_epoch:
            errors.append(dict(code='mixed_epoch',
                               detail=f'param_epoch {prev_epoch}->{epoch} 变化(非同版本)', index=i))
            break_points.add(i)
        prev_epoch = epoch if epoch is not None else prev_epoch

        # --- sequence: unique per stream; a duplicate is an error ---
        seq = s.get('sequence')
        if seq is not None:
            key = (s.get('stream'), seq)
            if key in seen_seq:
                errors.append(dict(code='duplicate_sequence', detail=f'sequence {seq} 重复', index=i))
            seen_seq.add(key)

        # --- time: strictly increasing per run; duplicate ANYWHERE (not just adjacent)
        #     and regression are errors. u32 wrap only when identity matches. ---
        t = s.get('t_mcu_us')
        if t is not None:
            if t in seen_t:
                errors.append(dict(code='duplicate_time',
                                   detail=f't_mcu_us {t} 与非相邻样本重复(首次 index {seen_t[t]})',
                                   index=i))
            else:
                seen_t[t] = i
            if prev_t is not None:
                if t < prev_t and not (prev_t > 0xffff0000 and t < 0x10000):
                    errors.append(dict(code='time_regression',
                                       detail=f't_mcu_us {prev_t}->{t} 倒退', index=i))
            prev_t = t

        # --- source_sequence jump (new state record advances by 2; a real gap is an error),
        #     explicit lost / error are rejections ---
        sseq = s.get('source_sequence')
        if sseq is not None and prev_source_seq is not None:
            step = s.get('source_sequence_step', 1)
            expected_next = prev_source_seq + step
            if sseq != expected_next and sseq != prev_source_seq:
                errors.append(dict(code='source_sequence_jump',
                                   detail=f'source_sequence {prev_source_seq}->{sseq} 跳变(期望 {expected_next})',
                                   index=i))
        if sseq is not None:
            prev_source_seq = sseq
        lost = s.get('lost')
        if isinstance(lost, int) and lost > 0:
            errors.append(dict(code='lost', detail=f'lost={lost} 存在丢包', index=i))
        err_us = s.get('error_us')
        if isinstance(err_us, int) and err_us not in (0, 0xffffffff) and err_us > 0:
            errors.append(dict(code='timing_error', detail=f'error_us={err_us}', index=i))

        # --- non-finite anywhere (dt NaN included) ---
        for field in _FLOAT_FIELDS:
            val = s.get(field)
            if isinstance(val, float) and not math.isfinite(val):
                errors.append(dict(code='non_finite', detail=f'{field} 非有限', index=i, field=field))

        # --- dt: non-positive or over explicit max => ERROR (hard, not advisory) ---
        dt = s.get('dt_s')
        if isinstance(dt, float) and math.isfinite(dt):
            if dt <= 0.0:
                errors.append(dict(code='dt_invalid', detail=f'dt_s={dt} 非正', index=i))
            elif bound_known and dt > max_dt + 1e-12:
                errors.append(dict(code='dt_exceeds_bound',
                                   detail=f'dt_s={dt} > max_dt_s={max_dt}', index=i))
                break_points.add(i)

        # --- feedback / fault / stop / rc / manual_stop / executing / can ---
        fv = s.get('feedback_valid')
        if fv is False:
            errors.append(dict(code='feedback_invalid', detail='feedback_valid=false', index=i))
        faults = s.get('fault_bits')
        if faults not in (0, None):
            errors.append(dict(code='fault', detail=f'fault_bits={faults}', index=i))
            errors.extend([dict(code='fault_reason', detail=r, index=i)
                           for r in (s.get('fault_reasons') or [])])
        src = s.get('target_source')
        mode = s.get('controller_mode')
        if src in STOP_SOURCES or mode == 'none':
            errors.append(dict(code='stop', detail=f'target_source={src!r} mode={mode!r}', index=i))
            break_points.add(i)
        if s.get('rc_valid') is False:
            errors.append(dict(code='rc_invalid', detail='rc_valid=false(遥控失联)', index=i))
        if s.get('manual_stop') is True:
            errors.append(dict(code='manual_stop', detail='manual_stop=true(人工停机)', index=i))
            break_points.add(i)
        if s.get('executing') is False:
            errors.append(dict(code='not_executing', detail='executing=false(未在执行)', index=i))
        can = s.get('can_tx_result')
        if can == 'busy':
            errors.append(dict(code='can_busy', detail='can_tx_result=busy', index=i))
        elif can in ('error', 'invalid_config', 'missing_output', 'stale_output',
                     'invalid_output'):
            errors.append(dict(code='can_error', detail=f'can_tx_result={can}', index=i))

        # --- no-execute: adopted target genuinely absent (not merely output) ---
        adopted = s.get('adopted_target_rad')
        if adopted is None:
            errors.append(dict(code='no_execute', detail='adopted_target_rad 缺失(未执行/无目标)', index=i))

        # --- output_normalized: a NULL output is NOT a missing bundle (real Pitch has no
        #     internal output).  A genuinely absent CONTROL record is the missing bundle,
        #     which is detected at pairing/loading time, not here.  A finite output that is
        #     yet declared missing is a contradiction. ---
        missing = s.get('missing_fields') or {}
        for fname in SAMPLE_FIELDS:
            if fname == 'missing_fields':
                continue
            declared_missing = fname in missing
            val = s.get(fname)
            if declared_missing and val is not None:
                errors.append(dict(code='field_contradiction',
                                   detail=f'{fname} 声明缺失但存在有限值 {val!r}', index=i))

        # --- field null tally ---
        for field in SAMPLE_FIELDS:
            if field == 'missing_fields':
                continue
            if s.get(field) is None:
                fields[field]['null'] += 1
                reason = missing.get(field, 'null')
                fields[field]['reasons'][reason] = fields[field]['reasons'].get(reason, 0) + 1

    if not bound_known:
        warnings.append(dict(code='dt_bound_unknown',
                             detail='manifest.max_dt_s 未声明, 不能声称时间连续可靠'))
    if not epoch_known:
        warnings.append(dict(code='epoch_unknown', detail='param_epoch 存在未知, 不能声称单版本'))
    if not boot_known:
        warnings.append(dict(code='boot_unknown', detail='boot_id 存在未知, 不得当作已知 boot 的干净运行'))

    all_breaks = sorted(break_points | scenario_breaks)
    error_index = {}
    for err in errors:
        idx = err.get('index')
        if isinstance(idx, int):
            error_index.setdefault(idx, []).append(err['code'])

    seg_axis = ref_axis
    for j in range(len(all_breaks) - 1):
        i0, i1 = all_breaks[j], all_breaks[j + 1]
        if i1 <= i0:
            continue
        seg_errors = []
        for k in range(i0, i1):
            seg_errors.extend(error_index.get(k, []))
        kind = 'run'
        for sc in manifest.get('scenarios') or []:
            if int(sc.get('i0', 0)) <= i0 and i1 <= int(sc.get('i1', 0)):
                kind = sc.get('kind') or sc.get('name') or 'scenario'
                break
        seg_epoch = None
        for k in range(i0, i1):
            if samples[k].get('param_epoch') is not None:
                seg_epoch = samples[k]['param_epoch']
                break
        status = 'valid' if not seg_errors else 'invalid'
        segments.append(dict(i0=i0, i1=i1, kind=kind, axis=seg_axis, status=status,
                             valid_for_scoring=(status == 'valid' and bound_known),
                             param_epoch=seg_epoch))

    time_info = _time_summary(samples, bound_known, max_dt)

    missing_reasons = {}
    for field, info in fields.items():
        for reason, cnt in info['reasons'].items():
            missing_reasons[reason] = missing_reasons.get(reason, 0) + cnt

    valid = not errors
    # M1 NEVER produces a promotable candidate, regardless of an approval string:
    # an 'approved' label alone is not a licence, and M1 has no repeated trials.
    promotable = False
    counts = {'samples': n, 'segments': len(segments), 'errors': len(errors),
              'warnings': len(warnings), 'promotable': promotable,
              'missing_reasons': missing_reasons}
    return dict(schema_version=SCHEMA_VERSION, valid=valid, errors=errors,
                warnings=warnings, segments=segments, fields=fields,
                counts=counts, time=time_info)
def _time_summary(samples, bound_known, max_dt):
    ts = [s.get('t_mcu_us') for s in samples if isinstance(s.get('t_mcu_us'), int)]
    if not ts:
        return {'t0_us': None, 't1_us': None, 'span_s': None, 'dt_min_s': None,
                'dt_max_s': None, 'bound_s': max_dt if bound_known else None,
                'bound_known': bound_known}
    dts = [s.get('dt_s') for s in samples if isinstance(s.get('dt_s'), (int, float))
           and math.isfinite(s.get('dt_s', math.nan))]
    return {'t0_us': min(ts), 't1_us': max(ts), 'span_s': (max(ts) - min(ts)) * 1e-6,
            'dt_min_s': min(dts) if dts else None,
            'dt_max_s': max(dts) if dts else None,
            'bound_s': max_dt if bound_known else None, 'bound_known': bound_known}


def _finalize(manifest, errors, warnings, segments, fields, counts, time_info,
              bound_known, max_dt, n):
    safety_ok = bool((manifest.get('safety_envelope') or {}).get('approved'))
    counts = {'samples': n, 'segments': 0, 'errors': len(errors),
              'warnings': len(warnings), 'promotable': False, 'missing_reasons': {}}
    for field in SAMPLE_FIELDS:
        fields[field] = {'null': 0, 'reasons': {}}
    time_info = {'t0_us': None, 't1_us': None, 'span_s': None, 'dt_min_s': None,
                 'dt_max_s': None, 'bound_s': max_dt if bound_known else None,
                 'bound_known': bound_known}
    return dict(schema_version=SCHEMA_VERSION, valid=False, errors=errors,
                warnings=warnings, segments=segments, fields=fields,
                counts=counts, time=time_info)


# -------------------------------------------------------------------- write
def _cell(value):
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, allow_nan=False)
    if isinstance(value, float) and not math.isfinite(value):
        return ''
    return value


def write_samples(path, samples):
    """Write canonical samples to CSV.  Refuses to overwrite an existing file."""
    path = Path(path)
    if path.exists():
        raise FileExistsError(f'拒绝覆盖已有样本文件: {path}')
    header = COLUMNS + EXTRA_COLUMNS
    with path.open('x', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for s in samples:
            writer.writerow([_cell(s.get(col)) for col in header])
    return len(samples)


def write_run(dataset, out_dir):
    """Persist a dataset to a fresh directory (manifest/samples/quality/events)."""
    out = Path(out_dir)
    if out.exists():
        raise FileExistsError(f'拒绝覆盖已有运行目录: {out}')
    out.mkdir(parents=True)
    (out / 'manifest.json').write_text(
        json.dumps(dataset.get('manifest', {}), ensure_ascii=False, allow_nan=False,
                   indent=2) + '\n', encoding='utf-8')
    write_samples(out / 'samples.csv', dataset.get('samples', []))
    (out / 'quality.json').write_text(
        json.dumps(dataset.get('quality', {}), ensure_ascii=False, allow_nan=False,
                   indent=2) + '\n', encoding='utf-8')
    with (out / 'events.jsonl').open('x', encoding='utf-8') as handle:
        for event in dataset.get('events', []):
            handle.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
    return out
