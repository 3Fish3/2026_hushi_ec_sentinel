"""正式 APP 维护控制台的数据层; 不依赖图形库, 不预置实车控制参数."""
from __future__ import annotations

import math
from collections import deque
from copy import deepcopy
from pathlib import Path

import sentinel_client as wire
from sentinel_async import AsyncSentinelClient, OutcomeUnknown
from sentinel_test import PARTS as BASE_PARTS
from sentinel_test import accepted
from sentinel_pitch_tuning import PitchTuningController
from sentinel_experiment_positioning import ExperimentPositioningController
from sentinel_positioning_capture import PositioningCaptureOwner

PARTS = {**BASE_PARTS, 'bench': ('main_yaw', 'pitch', 'friction_left', 'trigger')}

DOMAIN_LABELS = dict(zip(wire.DOMAINS, ('左前轮', '右前轮', '左后轮', '右后轮',
    '主 yaw', 'fire yaw', 'pitch', '左摩擦轮', '右摩擦轮', '拨弹'), strict=False))
PART_LABELS = {'gimbal': '云台', 'chassis': '底盘', 'shooter': '发射机构', 'bench': '四电机台架'}
DJI_FIELDS = {1: ('速度环 Kp', 'A/(rad/s)'), 2: ('速度环 Ki', 'A/rad'), 3: ('速度环 Kd', 'A/(rad/s²)'),
    4: ('电流上限', 'A'), 5: ('协议输出上限', 'raw'), 6: ('轴侧速度上限', 'rad/s'),
    7: ('反馈时效', 'ms'), 8: ('位置环 Kp', '1/s'), 9: ('位置环 Ki', '1/s²'),
    10: ('位置环 Kd', '1'), 20: ('关节零偏', 'rad'), 21: ('关节下界', 'rad'),
    22: ('关节上界', 'rad')}
DM_FIELDS = {1: ('MIT Kp', '1'), 2: ('MIT Kd', '1'), 3: ('反馈时效', 'ms'),
    **{k: DJI_FIELDS[k] for k in (20, 21, 22)}}


def parameter_label(entry):
    slot, local = entry['id'] >> 8, entry['id'] & 255
    if slot == 10 and local == 1:
        return '惯性对齐偏移', 'rad'
    return (DM_FIELDS if slot == 9 else DJI_FIELDS).get(local, (f'参数 0x{entry["id"]:04x}', '?'))


def parameter_scale(entry, entries):
    """速度PID在界面用电流口径; 协议与候选文件仍保存MCU原生数值."""
    slot, local = entry['id'] >> 8, entry['id'] & 255
    if slot < 9 and local in (1, 2, 3):
        cap = next((e['value'] for e in entries if e['id'] == (slot << 8 | 4)), None)
        if cap is None or not math.isfinite(cap) or cap <= 0:
            return None
        return cap
    return 1.0


def physical_parameter(entry, entries):
    scale = parameter_scale(entry, entries)
    if scale is None:
        return None
    return {**entry, **{key: entry[key]*scale for key in ('value', 'min', 'max')}}


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('请输入有限数值')
    return number


def target_packet(mechanism, drafts, *, twist=None, frame='joint'):
    if frame not in wire.YAW_FRAMES:
        raise ValueError('未知主 yaw 参考系')
    points = []
    for domain, text in drafts.items():
        if domain not in PARTS[mechanism]:
            raise ValueError('目标超出所选机构')
        mode = 'position' if domain in PARTS['gimbal'] else 'velocity'
        points.append((domain, mode, finite(text), 0))
    if twist is not None:
        if mechanism != 'chassis' or points or len(twist) != 3:
            raise ValueError('底盘 twist 与逐轮目标二选一')
        twist = tuple(finite(value) for value in twist)
    return {'setpoints': points, 'twist': twist, 'main_yaw_frame': frame}


def page_fresh(page, now, lifetime=.6):
    return bool(page and 0 <= now-page.get('received_at_monotonic', -math.inf) <= lifetime)


class LiveHistory:
    def __init__(self, capacity=1200):
        self.samples = deque(maxlen=capacity)
        self.origin = None
        self.identity = None
        self.pages = {}
        self.connected = False

    def accept(self, snapshot):
        self.connected = snapshot.get('connected', False)
        self.pages = snapshot.get('status', {}) if self.connected else {}
        page = self.pages.get(1)
        if not page:
            return
        key = (page['session'], page['sequence'])
        if key == self.identity:
            return
        if self.identity and key[0] != self.identity[0]:
            self.samples.clear()
            self.origin = None
        self.identity = key
        stamp = page['received_at_monotonic']
        if self.origin is None:
            self.origin = stamp
        self.samples.append((stamp-self.origin, self.plot_page(page)))

    def joint(self, domain, now):
        page = self.pages.get(0)
        if not self.connected or page is None or not page_fresh(page, now):
            return None
        return page.get('joints', {}).get(domain)

    def plot_page(self, page):
        out = deepcopy(page)
        fast = self.pages.get(0)
        latest = max(page['received_at_monotonic'], fast.get('received_at_monotonic', 0) if fast else 0)
        fresh = page_fresh(fast, latest)
        for entry in out['domains']:
            entry['measurement_unit'] = 'rad' if entry['domain'] in PARTS['gimbal'] else 'rad/s'
            # 域页在停止模式下携带速度, 不把它连到位置曲线上.
            # 机械角统一取快页; 惯性位置执行仍使用域页的同坐标系测量.
            if entry['domain'] in PARTS['gimbal'] and not (
                    entry['mode'] == 'position' and entry['measured_frame'] != 'joint'):
                joint = fast.get('joints', {}).get(entry['domain']) if fresh and fast is not None else None
                entry['measurement_valid'] = bool(joint and joint['position_valid'])
                entry['measurement'] = joint['angle_rad'] if joint is not None and entry['measurement_valid'] else math.nan
                entry['measured_frame'] = 'joint'
        return out

    def domain(self, domain, now):
        page = self.pages.get(1)
        if not self.connected or not page_fresh(page, now):
            return None
        return next((entry for entry in self.plot_page(page)['domains'] if entry['domain'] == domain), None)

    def series(self, domain):
        times, targets, measurements, errors = [], [], [], []
        last = None
        for stamp, page in self.samples:
            entry = next(e for e in page['domains'] if e['domain'] == domain)
            signature = (entry['mode'], entry['adopted_frame'], page['reference_epoch'])
            if last is not None and (signature != last[1] or stamp-last[0] > .6):
                times.append(stamp)
                targets.append(math.nan)
                measurements.append(math.nan)
                errors.append(math.nan)
            target = entry['target'] if entry['execute'] and entry['requested_frame'] is not None else math.nan
            measured = entry['measurement'] if entry['measurement_valid'] else math.nan
            error = target-measured if entry['adopted_frame'] == entry['measured_frame'] else math.nan
            times.append(stamp)
            targets.append(target)
            measurements.append(measured)
            errors.append(error)
            last = stamp, signature
        return times, targets, measurements, errors


class MaintenanceSession:
    """GUI 的阻塞事务由工作线程调用, 目标源由串口拥有者线程持续执行."""
    def __init__(self, peer, mechanism):
        self.peer, self.mechanism = peer, mechanism
        self.domains = PARTS[mechanism]
        self.parameters = []
        self.installed_domains = frozenset()
        self.requested_authority = False
        self.last_request = None
        self.note = ''
        self.assistance = PitchTuningController(peer)
        self.positioning_capture = PositioningCaptureOwner(peer)
        self.positioning = ExperimentPositioningController(peer, capture_start=self.positioning_capture.start)

    def require_manual(self):
        if self.positioning.active:
            raise ValueError('Experiment owns reference; STOP and reconnect before manual/assist control')
        if self.assistance.active:
            raise ValueError('辅助调参占用目标控制, 请先停止后再人工修改')

    def start_assisted(self, plan, directory, *, capture=False, wait_s=30.0):
        self.require_manual()
        if self.mechanism != 'gimbal' or 'pitch' not in self.installed_domains:
            raise ValueError('辅助调参仅支持已装配 Pitch 的云台 APP')
        self.stop()
        self.assistance.start(plan, directory, capture=capture, wait_s=wait_s)
        self.last_request = None
        self.note = '辅助调参接管 Pitch; 结束或停止会释放保持力矩'

    @classmethod
    def connect(cls, transport, mechanism, log_path=None):
        # 快页用于显示, 保留 20 Hz 作为显示策略; 串口性能须另行验证.
        peer = AsyncSentinelClient(transport, target_hz=50, fast_hz=20, log_path=log_path)
        session = cls(peer, mechanism)
        try:
            peer.hello().result(timeout=2)
            session.refresh_parameters()
            devices = peer.diagnostics(1).result(timeout=2)['devices']
            session.installed_domains = frozenset(e['domain'] for e in devices if e['installed'])
            return session
        except Exception:
            peer.close()
            raise

    def refresh_parameters(self):
        entries = self.peer.read_all_params().result(timeout=5)
        self.parameters = [entry for entry in entries
                           if not entry['whole_vehicle'] and entry['domain'] in self.domains]
        return deepcopy(self.parameters)

    def authorize(self):
        self.require_manual()
        reply = accepted(self.peer.authorize(True, 'maintenance').result(timeout=2))
        self.requested_authority = True
        return reply

    def submit(self, drafts, *, twist=None, frame='joint', permit=False):
        self.require_manual()
        if any(domain not in self.installed_domains for domain in drafts):
            raise ValueError('目标包含未装配电机; 请只选择已装配域')
        packet = target_packet(self.mechanism, drafts, twist=twist, frame=frame)
        if permit and self.mechanism != 'shooter':
            raise ValueError('发射许可仅属于发射机构')
        if not self.requested_authority:
            raise ValueError('请先显式申请维护控制; ACK 不代表执行')
        reference = self.peer.snapshot()['reference_epoch']
        last_permit = -math.inf
        self.last_request = deepcopy(packet)
        self.note = '持续发送中; 未勾选域撤销, 实际执行以 MCU 状态为准'

        def source(now, epochs):
            nonlocal last_permit
            if packet['setpoints'] and self.mechanism in ('gimbal', 'bench') and epochs['reference_epoch'] != reference:
                self.note = '参考代次改变, 已撤销旧位置目标; 请核对后重新下发'
                return None
            if self.mechanism == 'shooter' and now-last_permit >= .1:
                self.peer.publish_permit(permit, 200)
                last_permit = now
            return dict(epochs, **packet)
        self.peer.set_target_source(source)

    def stop(self):
        if self.positioning.active:
            result = self.positioning.stop()
            self.note = 'Experiment STOP requested; backend stop stage is authoritative, NOT physical e-stop'
            return result
        if self.assistance.active:
            self.assistance.stop()
        self.peer.set_target_source(None)
        if self.mechanism == 'shooter':
            self.peer.publish_permit(False, 1)
        self.last_request = None
        self.note = '已请求撤销目标; 不是硬件急停, 请看 MCU 执行状态'

    def release(self):
        self.stop()
        self.assistance.close()
        reply = accepted(self.peer.authorize(False, 'maintenance').result(timeout=2))
        self.requested_authority = False
        return reply

    def align(self, domain, value):
        self.require_manual()
        if domain not in self.domains or domain not in PARTS['gimbal']:
            raise ValueError('只能确认所选云台关节的已知角度')
        return accepted(self.peer.align(domain, finite(value), kind='joint_reference').result(timeout=2))

    def read_parameter(self, param_id):
        page = self.peer.param_read(param_id).result(timeout=2)
        entries = page.get('entries', [])
        if len(entries) != 1 or entries[0]['whole_vehicle'] or entries[0]['domain'] not in self.domains:
            raise ValueError('参数不存在或超出所选机构')
        entry = entries[0]
        self.parameters = [entry if old['id'] == param_id else old for old in self.parameters]
        return deepcopy(entry)

    def write_physical_parameter(self, param_id, value):
        return self.write_parameter(param_id, value, physical=True)

    def write_parameter(self, param_id, value, *, physical=False):
        self.require_manual()
        entry = self.read_parameter(param_id)
        if physical:
            slot, local = param_id >> 8, param_id & 255
            if slot < 9 and local in (1, 2, 3):
                self.read_parameter(slot << 8 | 4)
            scale = parameter_scale(entry, self.parameters)
            if scale is None:
                raise ValueError('缺少有效电流上限, 无法换算速度PID')
            value = finite(value)/scale
        number = finite(value)
        if not entry['writable']:
            raise ValueError('该参数没有在线生效路径')
        if not entry['min'] <= number <= entry['max']:
            raise ValueError(f'参数应在 [{entry["min"]}, {entry["max"]}] 内')
        unknown = None
        reply = None
        try:
            reply = accepted(self.peer.param_write(param_id, number).result(timeout=2))
        except OutcomeUnknown as error:
            unknown = str(error)
        # 读回展示与事务确认分开: 值相同也不能证明丢失的 ACK 已收到.
        try:
            actual = self.read_parameter(param_id)
        except Exception as error:
            return {'ack': reply, 'readback': None, 'unknown': unknown, 'read_error': str(error)}
        return {'ack': reply, 'readback': actual, 'unknown': unknown, 'read_error': None}

    def save_readback(self, path):
        entries = self.refresh_parameters()
        wire.save_config(Path(path), entries, self.peer.snapshot()['config_kind'])

    def load_drafts(self, path):
        entries = wire.load_config(Path(path), self.peer.snapshot()['config_kind'])
        live = {entry['id']: entry for entry in self.parameters}
        if any(entry['id'] not in live for entry in entries):
            raise ValueError('候选包含其它机构或不存在的参数')
        drafts = {}
        for entry in entries:
            value = finite(entry['value'])
            spec = live[entry['id']]
            if not spec['min'] <= value <= spec['max']:
                raise ValueError(f'参数 0x{entry["id"]:04x} 超出 MCU 范围')
            drafts[entry['id']] = value
        return drafts

    def close(self):
        try:
            self.positioning.close()
            self.positioning_capture.close()
            self.assistance.close()
            if self.requested_authority and self.peer.snapshot()['connected']:
                if self.positioning.active:
                    accepted(self.peer.authorize(False, 'maintenance').result(timeout=2))
                    self.requested_authority = False
                else:
                    self.release()
        finally:
            self.peer.close()
