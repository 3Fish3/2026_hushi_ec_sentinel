"""分机构最小测试入口: 目标/参数/状态/字节日志; 不修改安装声明, 不透传 CAN.

所有运动值由操作人给出 (SI), 无猜测的实车增益或运动默认值. --fixture 仅属合成后端.
USART1 业务默认 921600; 并行只读观测使用 --observe 的 USART6/115200, 不握手抢会话.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shlex
import time

from sentinel_async import AsyncSentinelClient, OutcomeUnknown
import sentinel_client as wire

PARTS = {'chassis': wire.DOMAINS[:4], 'gimbal': wire.DOMAINS[4:7], 'shooter': wire.DOMAINS[7:]}


def accepted(reply):
    if reply.get('result_name') != 'ACCEPTED':
        raise wire.ProtocolError(str(reply))
    return reply


class MechanismTest:
    def __init__(self, peer, mechanism):
        self.peer, self.mechanism = peer, mechanism
        self.domains = PARTS[mechanism]
        self.note = ''
        self.permit = False
        self.requested_authority = False

    def parameters(self):
        return [e for e in self.peer.read_all_params().result(timeout=3)
                if not e['whole_vehicle'] and e['domain'] in self.domains]

    def set_parameter(self, param_id, value):
        entries = self.peer.param_read(param_id).result(timeout=2).get('entries', [])
        if not entries or entries[0]['whole_vehicle'] or entries[0]['domain'] not in self.domains:
            raise wire.ProtocolError('parameter is outside the selected mechanism')
        if not entries[0]['writable']:
            raise wire.ProtocolError('parameter has no online apply path')
        try:
            reply = accepted(self.peer.param_write(param_id, value).result(timeout=2))
            print(f'id=0x{param_id:04x} effective={reply.get("effective")} '
                  f'config={reply.get("config_epoch")} reference={reply.get("reference_epoch")}')
            return reply
        except OutcomeUnknown:
            # 只读实际值, 不把一次不确定事务换成新身份重做.
            print('变更结果未知; 当前读回:', self.peer.param_read(param_id).result(timeout=2))
            raise

    def start(self, setpoints=(), twist=None, frame='joint', duration=None):
        points = [(d, m, float(v), 0) for d, m, v in setpoints]
        if any(d not in self.domains or m != ('position' if d in wire.DOMAINS[4:7] else 'velocity')
               or not math.isfinite(v) for d, m, v, _ in points):
            raise ValueError('target domain/mode/value does not match the selected mechanism')
        if len({d for d, _, _, _ in points}) != len(points):
            raise ValueError('duplicate target domain')
        if twist is not None and (self.mechanism != 'chassis' or len(twist) != 3
                                  or not all(math.isfinite(v) for v in twist)):
            raise ValueError('base_link twist requires the chassis mechanism and finite SI values')
        if twist is not None and points:
            raise ValueError('use a chassis twist or individual wheel targets, not both')
        if frame not in wire.YAW_FRAMES:
            raise ValueError('unknown main yaw frame')
        accepted(self.peer.authorize(True, 'maintenance').result(timeout=2))
        self.requested_authority = True
        reference = self.peer.snapshot()['reference_epoch']
        started = time.monotonic()
        last_permit = started-1
        self.note = '发送意图中; 是否采用/执行以 MCU 状态为准'

        def source(now, epochs):
            nonlocal last_permit
            if duration is not None and now-started >= duration:
                self.note = '目标发送已结束'
                return None
            if any(mode == 'position' for _, mode, _, _ in points) and epochs['reference_epoch'] != reference:
                self.note = '参考已改变, 旧位置意图已撤销; 需按新参考下发'
                return None
            if now-last_permit >= 0.1:
                self.peer.publish_permit(self.permit, 200)
                last_permit = now
            return dict(epochs, setpoints=points, twist=twist, main_yaw_frame=frame)
        self.peer.set_target_source(source)

    def stop(self):
        self.peer.set_target_source(None)  # 空原子包撤销意图; 不是切入人工整车停止档.
        self.peer.publish_permit(False, 1)
        self.note = '已请求撤销目标, 保持维护连接'

    def status(self):
        status = self.peer.status(1).result(timeout=2)
        if 'domains' not in status:
            raise wire.ProtocolError(str(status))
        view = dict(status, domains=[e for e in status['domains'] if e['domain'] in self.domains])
        return wire.format_status(view)

    def save(self, path):
        wire.save_config(path, self.parameters(), self.peer.snapshot()['config_kind'])
        print('已保存当前参数候选; 不是实车验收或 MCU Flash 保存:', path)

    def load(self, path):
        entries = wire.load_config(path, self.peer.snapshot()['config_kind'])
        live = {e['id']: e for e in self.parameters()}
        # 先核对整份文件的作用域, 不能执行一半才发现越界; 各写入本身仍为单参数事务.
        if any(e['id'] not in live for e in entries):
            raise wire.ProtocolError('profile includes unknown parameters or another mechanism')
        for entry in entries:
            if not live[entry['id']]['writable']:
                print(f'readonly id=0x{entry["id"]:04x}: not applied')
                continue
            self.set_parameter(entry['id'], float(entry['value']))

    def release(self):
        self.stop()
        reply = accepted(self.peer.authorize(False, 'maintenance').result(timeout=2))
        self.requested_authority = False
        return reply

    def command(self, line):
        args = shlex.split(line)
        if not args:
            return True
        op, *values = args
        if op == 'quit':
            return False
        if op == 'status':
            print(self.status())
        elif op == 'params':
            print(json.dumps(self.parameters(), ensure_ascii=False, indent=2))
        elif op == 'set' and len(values) == 2:
            self.set_parameter(int(values[0], 0), float(values[1]))
        elif op == 'target' and len(values) in (3, 4):
            self.start([values[:3]], frame=values[3] if len(values) == 4 else 'joint')
        elif op == 'twist' and len(values) == 3:
            self.start(twist=tuple(float(value) for value in values))
        elif op == 'permit' and values in (['0'], ['1']):
            self.permit = values == ['1']
            self.peer.publish_permit(self.permit, 200)
        elif op == 'stop':
            self.stop()
        elif op == 'release':
            self.release()
        elif op == 'save' and len(values) == 1:
            self.save(Path(values[0]))
        elif op == 'load' and len(values) == 1:
            self.load(Path(values[0]))
        else:
            raise ValueError('status | params | set ID VALUE | target DOMAIN MODE VALUE [FRAME] | '
                             'twist VX VY WZ | permit 0/1 | stop | release | save/load PATH | quit')
        return True


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    link = result.add_mutually_exclusive_group(required=True)
    link.add_argument('--host-backend', type=Path)
    link.add_argument('--serial', help='APP 业务 USART1 (C 板丝印 UART2)')
    link.add_argument('--observe', help='只读 USART6 (丝印 UART1), 不建立业务会话')
    result.add_argument('--baud', type=int, help='显式波特率; 业务默认 921600, 只读默认 115200')
    result.add_argument('--mechanism', choices=PARTS, default='chassis')
    result.add_argument('--fixture', choices=PARTS, help='仅合成后端: 未试机构参数/反馈缺失场景')
    result.add_argument('--plant', action='store_true', help='仅合成后端, 不是实车模型')
    result.add_argument('--target', nargs=3, action='append', metavar=('DOMAIN', 'MODE', 'VALUE'))
    result.add_argument('--twist', nargs=3, type=float, metavar=('VX', 'VY', 'WZ'))
    result.add_argument('--yaw-frame', choices=wire.YAW_FRAMES, default='joint')
    result.add_argument('--permit-forward', action='store_true', help='显式请求正向发射许可; 不自动判断摩擦轮到速')
    result.add_argument('--duration', type=float, default=2.0, help='有限测试的主机时长 s, 默认只观察')
    result.add_argument('--params', action='store_true')
    result.add_argument('--set', nargs=2, action='append', metavar=('ID', 'VALUE'))
    result.add_argument('--save-profile', type=Path)
    result.add_argument('--load-profile', type=Path)
    result.add_argument('--log', type=Path, help='原始字节 JSONL: 发送尝试与所有接收字节, 另含丢日志计数')
    result.add_argument('--interactive', action='store_true', help='简单命令输入; 输入/打印等待不暂停后台目标流')
    return result


def main(argv=None):
    argument_parser = parser()
    args = argument_parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        argument_parser.error('duration must be positive and finite')
    if not args.host_backend and (args.fixture or args.plant):
        argument_parser.error('fixture/plant are host-only')
    if args.observe:
        if any((args.target, args.twist, args.permit_forward, args.params, args.set,
                args.save_profile, args.load_profile, args.interactive, args.log)):
            argument_parser.error('observation only accepts port, baud and duration')
        transport = wire.SerialTransport(args.observe, args.baud or 115200)
        try:
            stop, sequence = time.monotonic()+args.duration, 0
            while time.monotonic() < stop:
                sequence += 1
                print(json.dumps(wire.observe_fast(transport, sequence), ensure_ascii=False))
                time.sleep(0.2)
        finally:
            transport.close()
        return 0
    extra = ['--plant'] if args.plant else []
    if args.fixture:
        extra += ['--fixture', args.fixture]
    transport = wire.ProcessTransport(args.host_backend, extra_args=extra) if args.host_backend else \
        wire.SerialTransport(args.serial, args.baud or 921600)
    peer = AsyncSentinelClient(transport, log_path=args.log)
    test = MechanismTest(peer, args.mechanism)
    result = 0
    try:
        info = peer.hello().result(timeout=2)
        print(f'protocol={wire.PROTOCOL_VERSION} config_kind={info["config_kind"]} '
              f'complete={info["config_complete"]}; fixture != vehicle acceptance')
        if args.load_profile:
            test.load(args.load_profile)
        test.permit = args.permit_forward
        if args.target or args.twist:
            test.start(args.target or (), args.twist, args.yaw_frame, None if args.interactive else args.duration)
        for key, value in args.set or ():
            test.set_parameter(int(key, 0), float(value))
        if args.params:
            print(json.dumps(test.parameters(), ensure_ascii=False, indent=2))
        if args.save_profile:
            test.save(args.save_profile)
        if args.interactive:
            print('status | params | set ID VALUE | target DOMAIN MODE VALUE [FRAME] | twist VX VY WZ | '
                  'permit 0/1 | stop | release | save/load PATH | quit')
            while True:
                try:
                    if not test.command(input('> ')):
                        break
                except (ValueError, wire.ProtocolError, TimeoutError) as error:
                    print(error)
        else:
            stop = time.monotonic()+args.duration
            while time.monotonic() < stop:
                print(test.status())
                time.sleep(0.2)
        print(test.note)
    except (wire.ProtocolError, ValueError, TimeoutError) as error:
        print(error)
        result = 1
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if test.requested_authority and peer.snapshot()['connected']:
            try:
                test.release()
            except (wire.ProtocolError, TimeoutError) as error:
                print('目标/授权撤销未确认:', error)
        peer.close()
        print('link:', peer.snapshot().get('statistics'))
        if peer.snapshot().get('log_error'):
            print('log:', peer.snapshot()['log_error'])
    return result


if __name__ == '__main__':
    raise SystemExit(main())
