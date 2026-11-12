"""哨兵正式 APP 业务协议主机客户端 (O5).

同一份协议同时用于主机软件后端 (离线) 与 F407 正式 APP: 这里只做编码/解码与会话维护,
不包含控制策略, 也不提供任意注入/内存/CAN 写入能力.

传输方式:
  * ProcessTransport: 启动 examples/sentinel_app_host_link.cpp 的软件后端, 走 stdin/stdout,
    不需要真实 COM 或板卡; 设备反馈由后端的 --inject 文件注入 (测试入口, 不属于协议).
  * SerialTransport: 真实串口 (pyserial 可选), 协议字节流完全相同.

最小 CLI:
  python robots/sentinel/host/sentinel_client.py --help
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

PROTOCOL_VERSION = 3
FRAME_START = 0x7E
FRAME_ESCAPE = 0x7D
ESCAPE_XOR = 0x20
PAYLOAD_MAX = 240

HELLO = 0x40
AUTHORIZE = 0x41
TARGETS = 0x42
ALIGN = 0x43
PARAM_READ = 0x44
PARAM_WRITE = 0x45
STATUS = 0x46
DIAGNOSTICS = 0x47
PERMIT = 0x48
IDENTIFICATION = 0x49
IDENTIFICATION_STATUS = 0xC9
IDENTIFICATION_STREAM = 0xCA
HELLO_ACK = 0xC0
ACK = 0xC1
TARGETS_ACK = 0xC2
PARAM_VALUE = 0xC4
STATUS_SNAPSHOT = 0xC6
DIAGNOSTICS_PAGE = 0xC7

RESPONSE_OF = {
    HELLO: HELLO_ACK,
    AUTHORIZE: ACK,
    TARGETS: TARGETS_ACK,
    ALIGN: ACK,
    PARAM_READ: PARAM_VALUE,
    PARAM_WRITE: ACK,
    PERMIT: ACK,
    STATUS: STATUS_SNAPSHOT,
    DIAGNOSTICS: DIAGNOSTICS_PAGE,
    IDENTIFICATION: IDENTIFICATION_STATUS,
}

RESULT_NAMES = {
    0: "ACCEPTED",
    1: "REJECTED",
    2: "BAD_VERSION",
    3: "BAD_LENGTH",
    4: "NO_SESSION",
    5: "REPLAY",
    6: "EPOCH_MISMATCH",
    7: "NOT_PERMITTED",
    8: "UNKNOWN_REQUEST",
    9: "UNKNOWN_PARAM",
    10: "INVALID_VALUE",
    11: "REFERENCE_IN_USE",
    12: "NO_TARGET",
    13: "ALIGN_FAILED",
    14: "UNSUPPORTED",
}
RESULT_BY_NAME = {name: code for code, name in RESULT_NAMES.items()}

# 对齐类型: 机械关节参考与惯性/odom 对齐偏移是两件不同的事, 不能共用一个数值.
ALIGN_KINDS = ("joint_reference", "inertial_alignment")

DOMAINS = (
    "chassis_lf",
    "chassis_rf",
    "chassis_lb",
    "chassis_rb",
    "main_yaw",
    "fire_yaw",
    "pitch",
    "friction_left",
    "friction_right",
    "trigger",
)
MODE_NAMES = ("none", "current", "velocity", "position")
YAW_FRAMES = ("joint", "local_inertial", "odom")
VALIDITY_NAMES = ("unknown", "unavailable", "incomplete", "valid", "stale", "invalid")
REASON_NAMES = (
    "none", "not_configured", "incomplete_config", "no_reference", "no_feedback",
    "stale_feedback", "invalid_feedback", "no_target", "stale_target", "invalid_target",
    "no_authority", "source_mismatch", "locally_not_ready", "manual_stop", "fault_dependency",
    "limit_violation", "controller_error", "output_rejected", "unsupported", "awaiting_remote",
    "standby", "restarted", "degraded", "not_permitted",
)
SOURCE_NAMES = ("none", "remote", "host", "maintenance")
PARAM_KIND_NAMES = ("hot", "reference")
UNAVAILABLE_AGE = 0xFFFFFFFF


OBSERVE_REQUEST, OBSERVE_RESPONSE = 0x36, 0xB6


class ProtocolError(RuntimeError):
    """协议层错误: 版本/长度/CRC 不符或响应类型不匹配."""


def crc16(data: bytes) -> int:
    """与固件 uart_crc16 一致 (CRC-16/CCITT-FALSE)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    if len(payload) > PAYLOAD_MAX:
        raise ProtocolError(f"payload too long: {len(payload)}")
    raw = bytes([1, message_type, sequence & 0xFF, (sequence >> 8) & 0xFF,
                 len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    crc = crc16(raw)
    raw += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    out = bytearray([FRAME_START])
    for byte in raw:
        if byte in (FRAME_START, FRAME_ESCAPE):
            out += bytes([FRAME_ESCAPE, byte ^ ESCAPE_XOR])
        else:
            out.append(byte)
    out.append(FRAME_START)
    return bytes(out)


class FrameDecoder:
    """逐字节解码; 遇到非法帧丢弃并重新同步, 与固件解码器语义一致."""

    def __init__(self) -> None:
        self._raw = bytearray()
        self._escape = False
        self._started = False
        self.rejected = 0

    def push(self, byte: int):
        if byte == FRAME_START:
            if self._started and not self._escape:
                self._started = True
                self._raw = bytearray()
                return None
            self._started = True
            self._raw = bytearray()
            self._escape = False
            return None
        if not self._started:
            return None
        if self._escape:
            self._raw.append(byte ^ ESCAPE_XOR)
            self._escape = False
        elif byte == FRAME_ESCAPE:
            self._escape = True
        else:
            self._raw.append(byte)
        return self._finish()

    def _finish(self):
        if len(self._raw) < 8:
            return None
        version = self._raw[0]
        size = self._raw[4] | (self._raw[5] << 8)
        if size > PAYLOAD_MAX:
            self.rejected += 1
            self._started = False
            self._raw.clear()
            return None
        if len(self._raw) != size + 8:
            if len(self._raw) > size + 8:
                self.rejected += 1
                self._started = False
            return None
        body = bytes(self._raw[: 6 + size])
        expect = self._raw[6 + size] | (self._raw[7 + size] << 8)
        if crc16(body) != expect:
            self.rejected += 1
            self._started = False
            return None
        self._started = False
        if version != 1:
            self.rejected += 1
            return None
        return (self._raw[1], self._raw[2] | (self._raw[3] << 8), bytes(self._raw[6 : 6 + size]))


def _pack(message_type: int, body: bytes) -> bytes:
    return bytes([PROTOCOL_VERSION, message_type]) + body


def _require(payload: bytes, size: int, what: str) -> None:
    """长度校验先于任何字段解码: 错误应答不能按成功布局解码."""
    if len(payload) < size:
        raise ProtocolError(f"{what}: short payload {len(payload)} < {size}")


def encode_hello(nonce: int = 0x11223344) -> bytes:
    return _pack(HELLO, struct.pack("<I", nonce))


def encode_authorize(session: int, sequence: int, grant: bool, source: str) -> bytes:
    return _pack(AUTHORIZE, struct.pack("<IIBB", session, sequence, 1 if grant else 0,
                                        SOURCE_NAMES.index(source)))


def encode_targets(session: int, sequence: int, config_epoch: int, reference_epoch: int,
                   setpoints, twist=None, chassis_age_ms: int = 0,
                   main_yaw_frame: str = "joint") -> bytes:
    """四元组 (domain, mode, value, age_ms); 主 yaw 参考系独立指定, 线格式始终带标签."""
    body = struct.pack("<IIIIB", session, sequence, config_epoch, reference_epoch, len(setpoints))
    for domain, mode, value, age_ms in setpoints:
        frame = YAW_FRAMES.index(main_yaw_frame) if domain == "main_yaw" else 0
        body += struct.pack("<BBBfH", DOMAINS.index(domain), MODE_NAMES.index(mode), frame,
                            float(value), int(age_ms))
    if twist is None:
        body += struct.pack("<B", 0)
    else:
        body += struct.pack("<BfffH", 1, *[float(v) for v in twist], int(chassis_age_ms))
    return _pack(TARGETS, body)


def encode_align(session: int, sequence: int, domain: str, value: float,
                 reference_epoch: int, kind: str = "joint_reference") -> bytes:
    return _pack(ALIGN, struct.pack("<IIBBfI", session, sequence, DOMAINS.index(domain),
                                    ALIGN_KINDS.index(kind), float(value), reference_epoch))


def encode_permit(session: int, sequence: int, permit: bool, ttl_ms: int) -> bytes:
    return _pack(PERMIT, struct.pack("<IIBH", session, sequence, 1 if permit else 0, int(ttl_ms)))


def encode_param_read(session: int, sequence: int, param_id: int, offset: int = 0) -> bytes:
    return _pack(PARAM_READ, struct.pack("<IIHH", session, sequence, offset, param_id))


def encode_param_write(session: int, sequence: int, param_id: int, value: float) -> bytes:
    return _pack(PARAM_WRITE, struct.pack("<IIHf", session, sequence, param_id, float(value)))


def encode_status(session: int, sequence: int, page: int = 1) -> bytes:
    return _pack(STATUS, struct.pack("<IIB", session, sequence, page))


def encode_diagnostics(session: int, sequence: int, page: int = 0, cursor=None) -> bytes:
    if cursor is not None and page not in (4, 5):
        raise ValueError("cursor只适用于电流轨迹页4/5")
    return _pack(DIAGNOSTICS, struct.pack("<IIB", session, sequence, page) +
                 (struct.pack("<I", cursor) if cursor is not None else b""))


def decode_hello_ack(payload: bytes) -> dict:
    if len(payload) != 47:
        raise ProtocolError(f'hello_ack: expected 47 bytes, got {len(payload)}')
    session, config_epoch, reference_epoch, clock, boot_id, nonce = \
        struct.unpack_from("<IIIIII", payload, 2)
    role, complete, startable, boot_identity = struct.unpack_from("<4B", payload, 26)
    identity = payload[30:46].split(b"\0", 1)[0].decode("ascii", "replace")
    return dict(session=session, config_epoch=config_epoch, reference_epoch=reference_epoch,
                clock_ms=clock, boot_id=boot_id, client_nonce=nonce, role=role,
                config_complete=bool(complete), startable=bool(startable),
                boot_identity_available=bool(boot_identity), identity=identity, config_kind=payload[46])


def decode_ack(payload: bytes) -> dict:
    """ACK 有三种长度, 按精确长度分派:
      12 = 纯结果码与原因;
      20 = +参考/配置代次 (align 成功应答);
      24 = +实际生效值与代次 (param_write).
    不能因为"长度够"就把 20 B 对齐包按成功效布局解码.
    """
    _require(payload, 12, "ack")
    if len(payload) not in (12, 20, 24):
        # 只接受声明的三种长度: 其它长度不能因为"至少 12 字节"就当作合法 ACK 解码.
        raise ProtocolError(f"ack: undeclared payload length {len(payload)}")
    session, sequence = struct.unpack_from("<II", payload, 2)
    result, reason = struct.unpack_from("<BB", payload, 10)
    out = dict(session=session, sequence=sequence, result=result,
               result_name=RESULT_NAMES.get(result, f"?{result}"), reason=reason,
               reason_name=REASON_NAMES[reason] if reason < len(REASON_NAMES) else f"?{reason}")
    if len(payload) == 24:
        out["effective"] = struct.unpack_from("<f", payload, 12)[0]
        out["reference_epoch"], out["config_epoch"] = struct.unpack_from("<II", payload, 16)
    elif len(payload) == 20:
        out["reference_epoch"], out["config_epoch"] = struct.unpack_from("<II", payload, 12)
    return out


def decode_param_value(payload: bytes) -> dict:
    _require(payload, 16, "param_value")
    session, sequence, count, offset, total = struct.unpack_from("<IIHHH", payload, 2)
    _require(payload, 16 + count * 16, "param_value entries")
    entries = []
    for i in range(count):
        base = 16 + i * 16
        param_id, kind, overridden, value, low, high = struct.unpack_from("<HBBfff", payload, base)
        entries.append(dict(id=param_id, kind=PARAM_KIND_NAMES[kind] if kind < 2 else "?",
                            overridden=bool(overridden & 1), value=value, min=low, max=high,
                            domain=_name(DOMAINS, (overridden >> 1) & 15),
                            whole_vehicle=bool(overridden & 32), writable=bool(overridden & 64)))
    return dict(session=session, sequence=sequence, count=count, offset=offset, total=total,
                entries=entries)


class _Cursor:
    def __init__(self, payload: bytes, offset: int = 0):
        self.payload, self.offset = payload, offset

    def take(self, fmt: str):
        size = struct.calcsize('<' + fmt)
        _require(self.payload, self.offset + size, 'page')
        values = struct.unpack_from('<' + fmt, self.payload, self.offset)
        self.offset += size
        return values

    def finish(self):
        if self.offset != len(self.payload):
            raise ProtocolError(f'page: unexpected length {len(self.payload)} != {self.offset}')


def _name(names, index):
    return names[index] if index < len(names) else f'?{index}'


def _age(value):
    return None if value == UNAVAILABLE_AGE else value


def decode_status(payload: bytes) -> dict:
    r = _Cursor(payload, 2)
    session, sequence, page, config_epoch, reference_epoch, clock = r.take('IIBIII')
    out = dict(session=session, sequence=sequence, page=page, config_epoch=config_epoch,
               reference_epoch=reference_epoch, clock_ms=clock)
    if page == 0:
        out['control_ms'], out['evaluated_us'], out['boot_id'] = r.take('III')
        source, flags, config_kind = r.take('BBB')
        out.update(source=_name(SOURCE_NAMES, source), authority=bool(flags & 1),
                   manual_stop=bool(flags & 2), launch_allowed=bool(flags & 4),
                   locally_ready=bool(flags & 8), boot_identity_available=bool(flags & 16),
                   main_yaw_degraded=bool(flags & 32), config_kind=config_kind)
        validity, reason, references, local_epoch, alignment_epoch = r.take('BBBII')
        main = dict(validity=_name(VALIDITY_NAMES, validity), reason=_name(REASON_NAMES, reason),
                    local_reference=bool(references & 1), odom_aligned=bool(references & 2),
                    local_epoch=local_epoch, alignment_epoch=alignment_epoch)
        out['joints'] = {}
        for domain in ('main_yaw', 'fire_yaw', 'pitch'):
            angle, rate, pa, va, valid = r.take('ffIIB')
            out['joints'][domain] = dict(angle_rad=angle, rate_rad_s=rate,
                position_valid=bool(valid & 1), velocity_valid=bool(valid & 2),
                position_age_ms=_age(pa), velocity_age_ms=_age(va))
        main['orientation_V_to_local_wxyz'] = r.take('ffff')
        main['rate_V_rad_s'] = r.take('fff')
        main['heading_rad'], main['odom_offset_rad'], main['heading_gain'] = r.take('fff')
        main['gyro_sample_us'], = r.take('I')
        main['ages_us'] = tuple(_age(x) for x in r.take('III'))
        main['time_error_us'] = tuple(_age(x) for x in r.take('III'))
        out['main_platform'] = main
        validity, flags = r.take('BB')
        wheel = dict(validity=_name(VALIDITY_NAMES, validity), rank_ok=bool(flags & 1),
                     pose_valid=bool(flags & 2), continuous=bool(flags & 4))
        wheel['twist'] = r.take('fff')
        wheel['pose_xy_yaw'] = r.take('fff')
        age, wheel['skew_us'], wheel['residual_rad_s'], wheel['gaps'] = r.take('IIfI')
        wheel['age_ms'] = _age(age)
        out['wheel_odom'] = wheel
        out['twist_scale'], = r.take('f')
    elif page == 1:
        source, authority, manual_stop, launch, count = r.take('BBBBB')
        if count != len(DOMAINS):
            raise ProtocolError(f'status: unexpected domain count {count}')
        out.update(source=_name(SOURCE_NAMES, source), authority=bool(authority),
                   manual_stop=bool(manual_stop), launch_allowed=bool(launch), domains=[])
        for domain in DOMAINS:
            validity, mode, execute, reason, frames, target, measurement, pa, va = r.take('BBBBBffII')
            out['domains'].append(dict(domain=domain, validity=_name(VALIDITY_NAMES, validity),
                mode=_name(MODE_NAMES, mode), execute=bool(execute), reason=_name(REASON_NAMES, reason),
                target=target, measurement=measurement, age_ms=_age(pa), velocity_age_ms=_age(va),
                requested_frame=_name(YAW_FRAMES, frames & 3) if frames & 64 else None,
                adopted_frame=_name(YAW_FRAMES, (frames >> 2) & 3),
                measured_frame=_name(YAW_FRAMES, (frames >> 4) & 3), measurement_valid=bool(frames & 128)))
    elif page == 2:
        out['requests'] = []
        for domain in DOMAINS:
            present, mode, frame, value, age, ttl = r.take('BBBfII')
            out['requests'].append(dict(domain=domain, present=bool(present), mode=_name(MODE_NAMES, mode),
                frame=_name(YAW_FRAMES, frame), value=value, age_ms=_age(age), ttl_ms=ttl))
        valid, x, y, yaw, at, integrated, rejected = r.take('BfffIII')
        out['command_prediction'] = dict(valid=bool(valid), pose_xy_yaw=(x, y, yaw),
            at_ms=at, integrated_ms=integrated, rejected_steps=rejected)
        flags, = r.take('B')
        requested, adopted = r.take('fff'), r.take('fff')
        out['requested_twist'] = requested if flags & 1 else None
        out['adopted_twist'] = adopted if flags & 2 else None
        out['twist_scale'], = r.take('f')
    else:
        raise ProtocolError(f'unknown status page {page}')
    r.finish()
    return out


def decode_diagnostics(payload: bytes) -> dict:
    r = _Cursor(payload, 2)
    session, sequence, page, count = r.take('IIBB')
    out = dict(session=session, sequence=sequence, page=page)
    if page in (0, 2):
        if page == 0:
            keys = ('cycles', 'rejected_targets', 'target_invalidations', 'link_losses', 'source_changes',
                'replay_rejects', 'accepted_commands', 'session_id', 'config_epoch', 'reference_epoch',
                'config_issues', 'config_complete', 'frame_rejects', 'duplicate_requests', 'expired_sessions',
                'boot_id', 'target_ttl_ms', 'link_ttl_ms', 'tx_transaction', 'tx_ack', 'tx_fast', 'tx_slow',
                'replaced_transaction', 'replaced_ack', 'replaced_fast', 'replaced_slow', 'tx_busy', 'tx_pending')
        else:
            keys = ('cycles', 'enable_failures', 'target_rejections', 'unreachable_targets', 'group_sent',
                'group_refused', 'group_tx_error', 'dm_enable_sent', 'dm_disable_sent', 'dm_command_sent',
                'dm_tx_error', 'dm_query_sent', 'skipped_groups', 'timing_observed', 'control_last_us',
                'control_max_us', 'tx_last_us', 'tx_max_us', 'imu_accepted', 'imu_invalid', 'imu_duplicate',
                'imu_regression', 'imu_too_old', 'imu_long_gap', 'imu_reprime', 'imu_extrapolations',
                'imu_invalidations', 'alignment_changes', 'accel_updates', 'accel_rejected',
                'reconstruction_missing', 'projection_rejected', 'wheel_updates', 'wheel_gaps',
                'wheel_integrated_us', 'wheel_continuous')
        if count != len(keys) + (8 if page == 2 else 0):
            raise ProtocolError(f'diagnostics page {page}: wrong count {count}')
        out.update(zip(keys, r.take('I' * len(keys))))
        if page == 2:
            out['wheel_sample_us'] = r.take('IIII')
            out['wheel_age_us'] = tuple(_age(value) for value in r.take('IIII'))
    elif page == 1:
        if count != 10:
            raise ProtocolError(f'device page: wrong count {count}')
        out['devices'] = []
        for slot in range(count):
            domain, flags, bus, protocol, device_id, feedback_id, age, velocity, effort, temp1, temp2 = r.take('BBBBHHIffBB')
            out['devices'].append(dict(slot=slot, domain=_name(DOMAINS, domain), installed=bool(flags & 1),
                online=bool(flags & 2), enabled=bool(flags & 4), faulted=bool(flags & 8),
                assembled=bool(flags & 128), bus=bus, protocol=protocol, device_id=device_id,
                feedback_id=feedback_id, age_ms=_age(age), velocity_rad_s=velocity,
                effort=effort if flags & 64 else None, effort_unit='Nm' if protocol == 128 else 'raw',
                temperature_c=temp1 if flags & 16 else None, rotor_temperature_c=temp2 if flags & 32 else None))
    elif page == 3:
        available, result, mask, schema = r.take('BBBB')
        if count != 1 or schema != 1:
            raise ProtocolError('tilt diagnostics: unsupported snapshot schema')
        names = ('applied', 'no_change', 'invalid_input', 'invalid_dt', 'rejected_norm',
                 'rejected_projection', 'rejected_antipodal', 'missing_anchor', 'unaligned')
        out.update(available=bool(available), result='not_attempted' if result == 255 else _name(names, result),
                   extrapolated_mask=mask, schema=schema)
        out['local_epoch'], out['gyro_us'], out['accel_us'], out['evaluated_us'] = r.take('IIII')
        out['joint_query_us'] = r.take('II')
        out['joint_source_age_us'] = tuple(_age(value) for value in r.take('II'))
        out['counts'] = dict(zip(names, r.take('I' * len(names))))
        (out['innovation_rad'], out['tilt_step_rad'], out['heading_injection_rad'],
         out['max_abs_heading_injection_rad']) = r.take('ffff')
    elif page in (4, 5):
        schema, available, period_ms, first, following = r.take('BBHII')
        if schema != 1 or count > 6 or (not available and count):
            raise ProtocolError('control trace: unsupported schema/count')
        out.update(schema=schema, available=bool(available), period_ms=period_ms,
                   first_sequence=first, next_sequence=following, samples=[])
        for i in range(count):
            at, target, command, low, high, effort, emin, emax, flags, samples, limited = r.take('IffffhhhHHH')
            out['samples'].append(dict(sequence=(first+i)&0xffffffff, sampled_us=at,
                target=target if math.isfinite(target) else None,
                command_normalized=command if flags&1 else None,
                command_min=None if flags&64 else low, command_max=None if flags&64 else high,
                effort_raw=effort if flags&2 else None, effort_min=None if flags&128 else emin,
                effort_max=None if flags&128 else emax, flags=flags, control_samples=samples,
                limited_samples=limited, execute=bool(flags&4), submit_failed=bool(flags&8),
                command_missing=bool(flags&16), feedback_missing=bool(flags&32), frame=_name(YAW_FRAMES,(flags>>8)&3)))
    elif 10 <= page <= 17 or page == 20:
        if count not in (0, 1):
            raise ProtocolError(f'group page: wrong count {count}')
        out['group'] = None
        if count:
            bus, can_id, result, timing, computed, submitted, failed, invalid, dt, max_dt, last_compute, last_submit = r.take('BHBBIIIIffII')
            out['group'] = dict(bus=bus, can_id=can_id, result=result, timing_observed=bool(timing & 1), stop_only=bool(timing & 2),
                computations=computed, submitted=submitted, tx_failed=failed, invalid_steps=invalid,
                last_dt_s=dt, max_dt_s=max_dt, last_computed_us=last_compute, last_submitted_us=last_submit)
    else:
        raise ProtocolError(f'unknown diagnostics page {page}')
    r.finish()
    return out


def encode_identification(session, sequence, action=2, plan=None):
    data = struct.pack('<IIB', session, sequence, action)
    if action == 1:
        if plan is None or not 1 <= len(plan['frequencies']) <= 6:
            raise ValueError('identification frequency count')
        frequencies = plan['frequencies']+[0.]*(6-len(plan['frequencies']))
        data += struct.pack('<B7fBB6f', 2,plan['speed'],plan['amplitude_a'],plan['bias_a'],plan['ramp_s'],
                            plan['settle_timeout_s'],plan['zero_s'],plan['acceleration'],
                            len(plan['frequencies']),plan['cycles'],*frequencies)
    return _pack(IDENTIFICATION,data)


def decode_identification_status(payload):
    r = _Cursor(payload)
    version, kind, session, sequence, state, reason, run, start, produced, consumed, dropped, duration, scale = r.take('BBIIBBIIIIIff')
    schema,excitation_start = (1,0) if len(payload)==40 else r.take('BI')
    validation=None
    if len(payload)>45:
        extension,profile=r.take('BB')
        if extension!=1: raise ProtocolError('validation capability version')
        values=r.take('14f');inner_us,outer_us,max_step_us=r.take('III')
        validation=dict(version=extension,profile=profile,duration_s=values[0],stop_current_a=values[1],stop_speed=values[2],
            velocity_pid=list(values[3:6]),position_pid=list(values[6:9]),inner_integral_limit=values[9],outer_integral_limit=values[10],
            current_limit_a=values[11],max_dt_s=values[12],feedback_alpha=values[13],inner_period_us=inner_us,outer_period_us=outer_us,max_step_us=max_step_us)
    r.finish()
    if version != PROTOCOL_VERSION or kind != IDENTIFICATION_STATUS or schema not in (1,2):
        raise ProtocolError('identification status version/type/schema')
    return dict(session=session,sequence=sequence,state=state,reason=reason,run_id=run,start_us=start,
                produced=produced,consumed=consumed,dropped=dropped,duration_s=duration,raw_to_axis_amp=scale,
                schema=schema,excitation_start_us=excitation_start,validation=validation)


def decode_identification_stream(payload):
    r = _Cursor(payload)
    version, kind, session, run, start, first, count, state, dropped = r.take('BBIIIIBBI')
    if version != PROTOCOL_VERSION or kind != IDENTIFICATION_STREAM or not 1 <= count <= 5:
        raise ProtocolError('identification stream version/type/count')
    # V1原始记录仍可离线读取; 执行脚本会在授权前要求V2能力.
    schema=1 if len(payload)==24+40*count else r.take('B')[0]
    if schema!=1 and (schema!=2 or count>4): raise ProtocolError('identification stream schema')
    samples = []
    for n in range(count):
        at, submit, ref, feedback, joint, gain, p, i, command, effort, age, flags = r.take('II6fhhHH')
        samples.append(dict(sequence=(first+n)&0xffffffff,at_us=at,submit_us=submit,reference=ref,feedback=feedback,
            joint=joint,heading_gain=gain,p_a=p,i_a=i,command_raw=command,effort_raw=effort,source_age_us=age,flags=flags,
            stage=flags>>8,measuring=bool(flags&16)))
        if schema==2:
            bias,excitation=r.take('ff')
            samples[-1].update(bias_a=bias,excitation_a=excitation)
    r.finish()
    return dict(session=session,run_id=run,start_us=start,first_sequence=first,state=state,dropped=dropped,schema=schema,samples=samples)


DECODERS = {
    HELLO_ACK: decode_hello_ack,
    ACK: decode_ack,
    TARGETS_ACK: decode_ack,
    PARAM_VALUE: decode_param_value,
    STATUS_SNAPSHOT: decode_status,
    DIAGNOSTICS_PAGE: decode_diagnostics,
    IDENTIFICATION_STATUS: decode_identification_status,
    IDENTIFICATION_STREAM: decode_identification_stream,
}

# 短应答与成功布局混用会导致 struct.error 而不是明确的协议失败: 服务端在错误路径上
# 统一回 ACK, 客户端先按类型/长度/身份确认, 再按对应布局解码.
SHORT_ERROR_TYPES = (ACK,)


class _FrameTransport:
    """同步单次请求与异步拥有者共用字节传输; 只在调用线程解析帧."""

    def __init__(self):
        self._decoder = FrameDecoder()
        self._frames = deque()
        self.observer = None

    def receive(self, timeout_s: float = 0.0):
        if self._frames:
            return self._frames.popleft()
        chunk = self._read_chunk(timeout_s)
        if chunk is None:
            return None
        if not chunk:
            raise ProtocolError('backend closed the link')
        if self.observer:
            self.observer('rx', chunk)
        for byte in chunk:
            frame = self._decoder.push(byte)
            if frame is not None:
                self._frames.append(frame)
        return self._frames.popleft() if self._frames else None

    def exchange(self, message: bytes, timeout_s: float = 5.0) -> tuple:
        self.send(message)
        deadline = time.monotonic() + timeout_s
        while (remaining := deadline - time.monotonic()) > 0:
            reply = self.receive(min(remaining, 0.01))
            if reply is not None:
                return reply
        raise TimeoutError('no response from link')


class ProcessTransport(_FrameTransport):
    """离线软件后端: 不需要真实 COM 或板卡; 管道读取队列固定上限."""

    def __init__(self, executable, inject_file: str | None = None,
                 extra_args: list | None = None) -> None:
        command = [str(executable)]
        if inject_file:
            command += ["--inject", str(inject_file)]
        if extra_args:
            command += [str(arg) for arg in extra_args]
        super().__init__()
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self._incoming: queue.Queue = queue.Queue(maxsize=64)
        self._closing = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self) -> None:
        assert self._process.stdout is not None
        while not self._closing.is_set():
            chunk = self._process.stdout.read1(512)
            while not self._closing.is_set():
                try:
                    self._incoming.put(chunk, timeout=0.05)
                    break
                except queue.Full:
                    continue
            if not chunk:
                return

    def send(self, message: bytes) -> None:
        if self.observer:
            self.observer('tx_attempt', message)
        assert self._process.stdin is not None
        self._process.stdin.write(message)
        self._process.stdin.flush()

    def _read_chunk(self, timeout_s: float):
        try:
            return self._incoming.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._closing.set()
        if self._process.stdin:
            try:
                self._process.stdin.close()
            except BrokenPipeError:
                pass
        try:
            self._process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=5)
        self._reader.join(timeout=1)
        if self._process.stdout:
            self._process.stdout.close()


class SerialTransport(_FrameTransport):
    """真实 APP 串口; 无自动换速或重连. USART6 只读观测须显式指定 115200."""

    def __init__(self, port: str, baudrate: int = 921600, timeout_s: float = 0.05) -> None:
        import serial  # 延迟导入: 离线路径不要求安装 pyserial

        super().__init__()
        self._serial = serial.Serial(port, baudrate, timeout=0, write_timeout=timeout_s)

    def send(self, message: bytes) -> None:
        if self.observer:
            self.observer('tx_attempt', message)
        if self._serial.write(message) != len(message):
            raise ProtocolError('serial write was incomplete')

    def _read_chunk(self, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while True:
            count = min(512, self._serial.in_waiting)
            if count:
                return self._serial.read(count)
            if time.monotonic() >= deadline:
                return None
            time.sleep(min(0.001, max(0, deadline-time.monotonic())))

    def close(self) -> None:
        self._serial.close()


@dataclass
class SentinelClient:
    """会话客户端: 维护 session/序号, 并把协议结果翻译成可读状态.

    每个应答都必须能证明它属于本次请求: 外层帧序号回显 + body 里的原请求身份都要匹配,
    CRC 正确不等于应答属于本次请求.
    """

    transport: object
    session: int = 0
    sequence: int = 0
    frame_sequence: int = 0
    hello_info: dict = field(default_factory=dict)
    log: list = field(default_factory=list)

    def _next_sequence(self) -> int:
        self.sequence += 1
        return self.sequence

    def _request(self, payload: bytes, message_type: int, expect: int, session: int | None = None,
                 sequence: int | None = None, nonce: int | None = None) -> dict:
        self.frame_sequence = (self.frame_sequence + 1) & 0xFFFF
        sent_frame = self.frame_sequence
        reply = self.transport.exchange(encode_frame(message_type, sent_frame, payload))
        rtype, echo, body = reply
        if not body or body[0] != PROTOCOL_VERSION:
            raise ProtocolError(f"protocol mismatch: MCU={body[0] if body else None}, client={PROTOCOL_VERSION}")
        if len(body) < 2 or body[1] != rtype:
            raise ProtocolError('frame/body response type mismatch')
        if rtype not in (expect, ACK):
            raise ProtocolError(f"unexpected response type 0x{rtype:02x}, expected 0x{expect:02x}")
        if echo != sent_frame:
            # 不匹配说明这不是本次请求的应答.
            raise ProtocolError(f"response frame sequence {echo} != request {sent_frame}")
        decoded = DECODERS[rtype](body)
        if rtype == ACK and expect != ACK:
            decoded["envelope"] = "error"
        if nonce is not None:
            # 握手应答用客户端 nonce 配对 (此时还没有会话身份).
            if decoded.get("client_nonce") != nonce:
                raise ProtocolError("hello_ack does not carry this client's nonce")
        elif session is not None:
            if decoded.get("session") != session or decoded.get("sequence") != sequence:
                raise ProtocolError(
                    f"response identity session={decoded.get('session')} "
                    f"seq={decoded.get('sequence')} does not match request "
                    f"session={session} seq={sequence}")
        elif decoded.get("session") not in (0, None):
            # 没有会话的请求 (hello) 不应收到带会话身份的应答.
            raise ProtocolError("response carries a session but the request had none")
        self.log.append((RESPONSE_OF[message_type], decoded))
        return decoded

    def hello(self, nonce: int = 0x11223344) -> dict:
        info = self._request(encode_hello(nonce), HELLO, HELLO_ACK, nonce=nonce)
        self.session = info["session"]
        self.sequence = 0
        # 保存副本: 后续 align/param_write 会就地刷新代次, 不应改变调用方已持有的握手快照.
        self.hello_info = dict(info)
        return info

    def authorize(self, grant: bool = True, source: str = "host") -> dict:
        sequence = self._next_sequence()
        return self._request(encode_authorize(self.session, sequence, grant, source),
                             AUTHORIZE, ACK, self.session, sequence)

    def targets(self, setpoints, twist=None, config_epoch=None, reference_epoch=None,
                chassis_age_ms: int = 0, main_yaw_frame: str = "joint") -> dict:
        info = self.hello_info
        sequence = self._next_sequence()
        return self._request(encode_targets(
            self.session, sequence,
            info.get("config_epoch", 0) if config_epoch is None else config_epoch,
            info.get("reference_epoch", 0) if reference_epoch is None else reference_epoch,
            setpoints, twist, chassis_age_ms, main_yaw_frame), TARGETS, TARGETS_ACK, self.session, sequence)

    def align(self, domain: str, value: float, reference_epoch=None,
              kind: str = "joint_reference") -> dict:
        epoch = self.hello_info.get("reference_epoch", 0) if reference_epoch is None \
            else reference_epoch
        sequence = self._next_sequence()
        ack = self._request(encode_align(self.session, sequence, domain, value, epoch, kind),
                            ALIGN, ACK, self.session, sequence)
        # 对齐会推进参考代次: 不把 hello 时的旧代次继续用在下一次目标下发上.
        if "reference_epoch" in ack:
            self.hello_info["reference_epoch"] = ack["reference_epoch"]
        return ack

    def permit(self, allow: bool, ttl_ms: int) -> dict:
        sequence = self._next_sequence()
        return self._request(encode_permit(self.session, sequence, allow, ttl_ms), PERMIT, ACK,
                             self.session, sequence)

    def param_read(self, param_id: int, offset: int = 0) -> dict:
        sequence = self._next_sequence()
        return self._request(
            encode_param_read(self.session, sequence, param_id, offset),
            PARAM_READ, PARAM_VALUE, self.session, sequence)

    def param_write(self, param_id: int, value: float) -> dict:
        sequence = self._next_sequence()
        ack = self._request(encode_param_write(self.session, sequence, param_id, value),
                             PARAM_WRITE, ACK, self.session, sequence)
        # 参考类参数成功后会推进参考代次: 立即采用回包里的新代次, 否则下一次目标仍用旧代次.
        if "reference_epoch" in ack:
            self.hello_info["reference_epoch"] = ack["reference_epoch"]
            self.hello_info["config_epoch"] = ack["config_epoch"]
        return ack

    def read_all_params(self) -> list:
        """按页读回全部参数: 单页放不下时继续请求, 不静默截断."""
        entries, offset, total = [], 0, None
        while total is None or offset < total:
            page = self.param_read(0xFFFF, offset)
            if page.get("count", 0) == 0 and page.get("total", 0) > offset:
                raise ProtocolError(f"param read made no progress: {page}")
            entries.extend(page["entries"])
            offset = page["offset"] + page["count"]
            total = page["total"]
            if page["count"] == 0:
                break
        return entries

    def status(self, page: int = 1) -> dict:
        sequence = self._next_sequence()
        return self._request(encode_status(self.session, sequence, page), STATUS, STATUS_SNAPSHOT,
                             self.session, sequence)

    def diagnostics(self, page: int = 0, cursor=None) -> dict:
        sequence = self._next_sequence()
        return self._request(encode_diagnostics(self.session, sequence, page, cursor), DIAGNOSTICS,
                             DIAGNOSTICS_PAGE, self.session, sequence)


def observe_fast(transport, sequence: int) -> dict:
    """USART6 独立一致快照; 不握手、不刷新 USART1 业务会话、无执行器请求."""
    kind, echo, body = transport.exchange(encode_frame(OBSERVE_REQUEST, sequence, b''))
    if kind != OBSERVE_RESPONSE or echo != (sequence & 0xffff):
        raise ProtocolError('observation response identity mismatch')
    if len(body) < 2 or body[:2] != bytes([PROTOCOL_VERSION, STATUS_SNAPSHOT]):
        raise ProtocolError('APP observation role/version mismatch')
    return decode_status(body)


def format_status(status: dict) -> str:
    lines = [f"source={status['source']} authority={status['authority']} "
             f"manual_stop={status['manual_stop']} launch_allowed={status['launch_allowed']} "
             f"config_epoch={status['config_epoch']} reference_epoch={status['reference_epoch']} "
             f"clock={status['clock_ms']}ms"]
    for entry in status["domains"]:
        sample_age = entry['age_ms'] if entry['mode'] == 'position' else entry['velocity_age_ms']
        age = '-' if sample_age is None else f'{sample_age}ms'
        adopted = f"{entry['target']:+.4f}" if entry['execute'] else '--'
        measured = f"{entry['measurement']:+.4f}" if entry['measurement_valid'] else '--'
        unit = 'rad' if entry['mode'] == 'position' else 'rad/s'
        lines.append(f"  {entry['domain']:<15} {entry['validity']:<12} {entry['mode']:<9} "
                     f"execute={int(entry['execute'])} reason={entry['reason']:<18} "
                     f"adopted={adopted} measured={measured} {unit} age={age} "
                     f"frames(req/use/meas)={entry['requested_frame']}/{entry['adopted_frame']}/{entry['measured_frame']}")
    return "\n".join(lines)


def save_config(path: Path, entries: list, config_kind: int = 0) -> None:
    """主机侧配置文件: 只保存参数值, 不写 MCU Flash, 也不改电调保存设置."""
    path.write_text(json.dumps(
        {"protocol_version": PROTOCOL_VERSION, "config_kind": config_kind,
         "origin": "ram_readback_candidate", "validated_on_vehicle": False,
         "params": [{key: e[key] for key in ('id', 'kind', 'value', 'domain', 'whole_vehicle', 'writable') if key in e}
                    for e in entries]},
        indent=2, ensure_ascii=False), encoding="utf-8")


def load_config(path: Path, target_config_kind: int = 0) -> list:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("protocol_version") != PROTOCOL_VERSION:
        raise ProtocolError("host config file was written for another protocol version")
    if data.get('config_kind') == 1 and target_config_kind == 2:
        raise ProtocolError('synthetic fixture profile cannot be loaded onto a real deployment')
    return data.get("params", [])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="哨兵正式 APP 最小操作端 (业务协议)")
    parser.add_argument("--host-backend", type=Path,
                        default=Path(__file__).resolve().parents[3] / "build/sentinel-app-host"
                        / "Release/ec_sentinel_app_host.exe",
                        help="离线软件后端可执行文件")
    parser.add_argument("--serial", help="真实串口设备 (与 --host-backend 二选一)")
    parser.add_argument("--baud", type=int, default=921600, help="APP USART1 波特率, 不自动探测/换速")
    parser.add_argument("--inject", type=Path, help="离线反馈注入文件 (测试入口, 不属于协议)")
    parser.add_argument("--plant", action="store_true",
                        help="离线后端用合成负载闭合回路 (主机侧夹具, 不是实车模型)")
    parser.add_argument("--authorize", action="store_true", help="握手后请求上位机来源")
    parser.add_argument("--status", action="store_true", help="打印常态状态")
    parser.add_argument("--diagnostics", action="store_true", help="打印诊断计数")
    parser.add_argument("--status-page", type=int, choices=(0, 1, 2), default=1)
    parser.add_argument("--diag-page", type=int, choices=(0, 1, 2, 10, 11, 12, 13, 14, 15, 16, 17, 20), default=0)
    parser.add_argument("--yaw-frame", choices=YAW_FRAMES, default="joint", help="主 yaw 目标参考系")
    parser.add_argument("--params", action="store_true", help="打印参数读回")
    parser.add_argument("--param-write", nargs=2, metavar=("ID", "VALUE"),
                        help="写一个参数 (ID 为十进制或 0x 前缀)")
    parser.add_argument("--target", nargs="+", metavar="FIELD",
                        help="下发单个机构目标; 其余域保持无目标 (可选第 4 项: 源年龄 ms)")
    parser.add_argument("--align", nargs="+", metavar=("DOMAIN", "RAD"),
                        help="建立参考 (可选第 3 项: joint_reference|inertial_alignment)")
    parser.add_argument("--permit", type=int, metavar="TTL_MS",
                        help="授予发射许可, 有效期 TTL_MS 毫秒 (0 = 撤销)")
    parser.add_argument("--save-config", type=Path, help="把当前参数值保存到主机配置文件")
    parser.add_argument("--load-config", type=Path, help="从主机配置文件写回参数")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    transport = SerialTransport(args.serial, args.baud) if args.serial else ProcessTransport(
        args.host_backend, args.inject, ["--plant"] if args.plant else None)
    client = SentinelClient(transport)
    # 进程返回 0 只表示本次命令行全部成功: 任何被拒绝的请求都给出非零结果,
    # 批量加载也不会因为中途拒绝而假报成功.
    failed = 0

    def report(label: str, ack: dict) -> None:
        nonlocal failed
        ok = ack.get("result_name") == "ACCEPTED"
        if not ok:
            failed += 1
        print(f"{label}: {ack['result_name']} ({ack['reason_name']})")

    try:
        info = client.hello()
        print(f"identity={info['identity']} session={info['session']} "
              f"boot_id={info['boot_id']} boot_identity_available="
              f"{int(info['boot_identity_available'])} "
              f"config_epoch={info['config_epoch']} reference_epoch={info['reference_epoch']} "
              f"startable={info['startable']} complete={info['config_complete']} config_kind={info['config_kind']}")
        if args.authorize:
            report("authorize", client.authorize(True))
        if args.align:
            kind = args.align[2] if len(args.align) > 2 else "joint_reference"
            report(f"align {kind}", client.align(args.align[0], float(args.align[1]), kind=kind))
        if args.permit is not None:
            report("permit", client.permit(args.permit > 0, max(1, args.permit)))
        if args.target:
            age_ms = int(args.target[3]) if len(args.target) > 3 else 0
            report("targets", client.targets(
                [(args.target[0], args.target[1], float(args.target[2]), age_ms)], main_yaw_frame=args.yaw_frame))
        if args.param_write:
            param_id = int(args.param_write[0], 0)
            ack = client.param_write(param_id, float(args.param_write[1]))
            report(f"param_write {param_id}", ack)
            print(f"  effective={ack.get('effective')}")
        if args.load_config:
            for entry in load_config(args.load_config, info['config_kind']):
                if not entry.get('writable', True):
                    print(f"readonly: id={entry['id']} (not applied)")
                    continue
                report(f"load {entry['id']}", client.param_write(entry["id"], entry["value"]))
        if args.params or args.save_config:
            entries = client.read_all_params()
            if args.save_config:
                save_config(args.save_config, entries, info['config_kind'])
                print(f"saved {len(entries)} parameters to {args.save_config}")
            else:
                for entry in entries:
                    print(f"  id={entry['id']:<6} {entry['kind']:<9} "
                          f"{'override' if entry['overridden'] else 'default ':<9} "
                          f"{entry['value']:+.5f} [{entry['min']:+.3f}, {entry['max']:+.3f}]")
        if args.status:
            status = client.status(args.status_page)
            print(format_status(status) if args.status_page == 1 else json.dumps(status, ensure_ascii=False))
        if args.diagnostics:
            for key, value in client.diagnostics(args.diag_page).items():
                print(f"  {key}={value}")
    finally:
        transport.close()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
