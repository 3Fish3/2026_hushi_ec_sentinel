"""Read-only, fail-closed view of device-reported experiment state.
Missing fields are None/UNKNOWN, never inferred from a host checkbox or M3 lease.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict, fields, replace
import math
import struct
import time
from sentinel_client import ProtocolError

STATES=('disabled','idle','armed','running','finishing','complete','fault_latched')
REASONS=('none','hardware_blocked','bad_state','invalid_config','authorization','manual_stop','guardian','feedback','reference','limits','actuator','other_axes','pitch_blocked','timing','link','backpressure','observation','identity','position_limit','output_limit','travel_limit','heartbeat_timeout','stop_timeout')

@dataclass(frozen=True)
class DeviceState:
    connected: bool = False
    received_monotonic: float | None = None
    firmware_identity: str | None = None
    boot_id: int | None = None
    app_session: int | None = None
    experiment_session: int | None = None
    nonce: int | None = None
    param_epoch: int | None = None
    config_epoch: int | None = None
    reference_epoch: int | None = None
    config_hash: int | None = None
    guardian_version: int | None = None
    hardware_capable: bool | None = None
    simulation: bool | None = None
    pitch_hardware_blocked: bool | None = None
    axis: str | None = None
    state: str = 'UNKNOWN'
    reason: str = 'UNKNOWN'
    result: int | None = None
    op: int | None = None
    now_us: int | None = None
    manual_stop: bool | None = None
    guardian_latched: bool | None = None
    feedback_fresh: bool | None = None
    reference_valid: bool | None = None
    limits_valid: bool | None = None
    actuator_ready: bool | None = None
    other_axes_excluded: bool | None = None
    link_valid: bool | None = None
    backpressure_ok: bool | None = None
    observation_valid: bool | None = None
    output_zero: bool | None = None
    stop_confirmed: bool | None = None
    stop_phase: str | None = None
    position_rad: float | None = None
    velocity_rad_s: float | None = None
    output_abs: float | None = None
    blocked_reason: str | None = None
    hardware_capable_flag: bool | None = None
    guardian_approved: bool | None = None
    remote_online: bool | None = None
    maintenance_authorized: bool | None = None
    operator_approved: bool | None = None
    stop_policy_verified: bool | None = None
    lease_active: bool | None = None
    stop_policy_executed: bool | None = None
    approval_record_version: int | None = None
    axis_identity: int | None = None
    firmware_sha256: str | None = None
    deployment_sha256: str | None = None
    guardian_state: int | None = None
    provisioning_state: int | None = None
    stale_after_s: float = 1.0

    @property
    def fresh(self):
        return self.connected and self.received_monotonic is not None and time.monotonic()-self.received_monotonic <= self.stale_after_s

    @property
    def unknown_fields(self):
        required=('firmware_identity','boot_id','param_epoch','config_hash','guardian_version',
            'hardware_capable','simulation','manual_stop','guardian_latched','feedback_fresh',
            'reference_valid','limits_valid','actuator_ready','other_axes_excluded','link_valid',
            'backpressure_ok','observation_valid')
        return [name for name in required if getattr(self,name) is None]

    @property
    def motion_allowed(self):
        positives=('hardware_capable','feedback_fresh','reference_valid','limits_valid','actuator_ready',
                   'other_axes_excluded','link_valid','backpressure_ok','observation_valid')
        return (self.fresh and not self.unknown_fields and self.axis in ('fire_yaw','pitch')
            and self.simulation is False and self.manual_stop is False and self.guardian_latched is False
            and self.state in ('idle','armed') and self.result==0 and self.reason=='none'
            and all(getattr(self,k) is True for k in positives)
            and (self.axis!='pitch' or self.pitch_hardware_blocked is False))

    @property
    def physical_stop_confirmed(self):
        return (self.fresh and self.state in ('complete','idle','fault_latched')
                and self.stop_confirmed is True and self.feedback_fresh is True
                and self.output_zero is True and self.stop_phase in ('feedback_confirmed','complete'))

    def to_dict(self):
        return {**asdict(self),'fresh':self.fresh,'motion_allowed':self.motion_allowed,
                'unknown_fields':self.unknown_fields,'physical_stop_confirmed':self.physical_stop_confirmed}

    def disconnected(self, reason):
        return replace(self,connected=False,blocked_reason=str(reason))


def decode_experiment_response(payload: bytes, base: DeviceState | None = None) -> DeviceState:
    if not ((len(payload)==32 and payload[0]==1) or (len(payload)==128 and payload[0]==2)):
        raise ProtocolError('experiment response: unsupported schema/version/length')
    version,op,result,reason,state,capable,simulation,pitch_blocked,session,nonce,epoch,now,config,guardian=struct.unpack('<8B6I',payload[:32])
    if op not in range(1,8) or result>4 or reason>=len(REASONS) or state>=len(STATES) or max(capable,simulation,pitch_blocked)>1:
        raise ProtocolError('experiment response: invalid enum or boolean')
    decoded = replace(base or DeviceState(),connected=True,received_monotonic=time.monotonic(),
        op=op,result=result,reason=REASONS[reason],state=STATES[state],hardware_capable=bool(capable),
        simulation=bool(simulation),pitch_hardware_blocked=bool(pitch_blocked),experiment_session=session,
        nonce=nonce,param_epoch=epoch,now_us=now,config_hash=config,guardian_version=guardian)
    if version==1:
        return decoded
    boot,flags,live_epoch,config_epoch,ref_epoch,approval=struct.unpack_from('<6I',payload,32)
    axis,stop,guardian_state,provisioning=struct.unpack_from('<4B',payload,56)
    if axis not in (0,5,6) or stop>6 or provisioning>2 or flags & ~0x3ffff:
        raise ProtocolError('experiment v2: invalid flags or state')
    old_boot=base.boot_id if base else None
    if old_boot is not None and boot and old_boot!=boot:
        raise ProtocolError('experiment v2: boot identity changed; reconnect required')
    bit=lambda n: bool(flags & (1<<n))
    return replace(decoded,boot_id=boot or old_boot,param_epoch=live_epoch,config_epoch=config_epoch,
        reference_epoch=ref_epoch,approval_record_version=approval,
        axis={0:None,5:'fire_yaw',6:'pitch'}[axis],
        stop_phase=('none','requested','withdrawn','policy_executing','feedback_confirmed','complete','unknown')[stop],
        guardian_state=guardian_state,provisioning_state=provisioning,
        axis_identity=struct.unpack_from('<I',payload,60)[0],
        firmware_sha256=payload[64:96].hex(),deployment_sha256=payload[96:128].hex(),
        hardware_capable_flag=bit(0),guardian_approved=bit(1),guardian_latched=not bit(2),
        manual_stop=bit(3),remote_online=bit(4),maintenance_authorized=bit(5),
        operator_approved=bit(6),feedback_fresh=bit(7),stop_policy_verified=bit(8),
        lease_active=bit(9),stop_policy_executed=bit(10),
        reference_valid=bit(11),limits_valid=bit(12),actuator_ready=bit(13),other_axes_excluded=bit(14),
        link_valid=bit(15),backpressure_ok=bit(16),observation_valid=bit(17),
        stop_confirmed=stop in (4,5),output_zero=bit(10) and stop in (4,5))
