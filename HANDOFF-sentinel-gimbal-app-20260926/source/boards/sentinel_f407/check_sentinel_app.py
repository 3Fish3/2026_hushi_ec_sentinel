"""只读核对正式 APP 镜像的角色身份、链接路径与静态预算, 不连接调试器或硬件.

与 check_firmware.py (默认 S1-S4 非运动审计) 和 check_capture.py (只读采集) 分开:
本脚本描述的是**会驱动执行器**的角色, 因此它核对的是"APP 路径确实被链接进来了",
而不是"镜像里没有发送路径". 三个角色的审计规则各自独立, 不互相放宽.
"""
import argparse
import hashlib
import json
import math
import re
import struct
import subprocess
from pathlib import Path

# 部署摘要布局 (board_app_config.hpp 的 DeploymentSummary): 11 个统计 uint32 +
# 9 个 DJI 槽地址 + 1 个 DM 地址, 共 21 个 uint32.
SUMMARY_FIELDS = (('magic', 'layout', 'kind', 'dji_capacity', 'dm_capacity', 'installed_dji',
                   'installed_dm', 'synthetic_values', 'unknown_origin_installed',
                   'harness_dji', 'harness_dm') +
                  tuple(f'dji_address_{slot}' for slot in range(9)) + ('dm_address',))
SUMMARY_MAGIC = 0x45434150
SUMMARY_LAYOUT = 2
CONFIG_KIND_REAL_DEPLOYMENT = 2

# 必须存在的 APP 路径符号: 缺任何一个都说明"只编了库/只链了假 sink", 不是 APP 接入.
REQUIRED_SYMBOLS = (
    'ec_app_init',                     # 启动前构造配置与应用对象
    'ec_app_control_tick',             # 1 kHz 控制周期体
    'ec_app_control_task_entry',       # 控制任务入口 (freertos.c 调用)
    'ec_app_route',                    # CAN 收帧路由到应用执行层
    'ec_app_link_bytes',               # USART1 业务协议字节入口
    'ec_board_app_control',            # APP 强钩子: 控制
    'ec_board_app_commit',             # APP 强钩子: 提交
    'ec_registry_publish_app',         # 应用状态发布
    'ec_app_deployment_summary',       # 部署配置静态摘要 (供本脚本读取)
)
# 真实控制与发送路径: 只出现"应用层函数"而没有编解码/发送组/发送适配器不算接入完成.
REQUIRED_PATH_PATTERNS = ('encode_dji', 'DjiTxGroup', 'HAL_CAN_AddTxMessage')
# 其它角色专属符号: 正式 APP 镜像里不得出现.
FORBIDDEN_PATTERNS = ('ec_capture_init', 'ec_capture_snapshot', 'g_ec_s5_signature')
# 允许的固件标识前缀: 与构建时 EC_FIRMWARE_ID="app-<label>" 一致.
FIRMWARE_ID_PATTERN = re.compile(r'^app-[A-Za-z0-9._-]+$')


def require(condition, message):
    if not condition:
        raise SystemExit(message)


def load_symbols(nm, elf):
    """读全部已定义符号 (含局部): 控制任务栈是 static, 只在局部符号里出现."""
    output = subprocess.check_output([str(nm), '-C', '--defined-only', str(elf)], text=True)
    entries = []
    for line in output.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3:
            entries.append((int(parts[0], 16), parts[1], parts[2]))
    return entries


def find_symbol(entries, name):
    """按符号名定位: 支持 C 名与 C++ 命名空间函数 (ec::sentinel::ec_app_init())."""
    for address, kind, text in entries:
        if text == name or text.startswith(name + '(') or f'::{name}(' in text:
            return address, kind
    return None


def has_prefix(entries, pattern):
    return any(pattern in text for _, _, text in entries)


def read_summary(image, address):
    offset = address - 0x08000000
    require(0 <= offset <= len(image) - 4 * len(SUMMARY_FIELDS),
            'Deployment summary symbol is outside the flashed image')
    values = struct.unpack_from(f'<{len(SUMMARY_FIELDS)}I', image, offset)
    return dict(zip(SUMMARY_FIELDS, values, strict=True))


def read_pitch_summary(image, address, enabled):
    """Read the separately versioned opt-in descriptor, never infer policy from symbols."""
    fields = ('magic', 'layout', 'enabled', 'require_capture', 'min_rad', 'max_rad',
              'max_amplitude_nm', 'max_expected_torque_nm', 'max_velocity_rad_s',
              'max_tracking_error_rad', 'min_half_period_us', 'max_duration_us',
              'max_feedback_age_us', 'max_step_us', 'zero_offset_rad', 'soft_min_rad',
              'soft_max_rad', 'dm_address', 'target_ttl_ms', 'link_ttl_ms', 'record_bytes')
    offset = address - 0x08000000
    require(0 <= offset <= len(image) - 4 * len(fields), 'Pitch summary outside image')
    encoded: dict[str, int] = dict(zip(fields, struct.unpack_from('<21I', image, offset), strict=True))
    summary = dict[str, int | float](encoded)
    require(summary['magic'] == 0x50494431 and summary['layout'] == 1, 'Pitch summary identity mismatch')
    require(summary['enabled'] == int(enabled), 'Pitch identification opt-in and audit selection differ')
    require(summary['record_bytes'] == 64, 'Pitch recording must retain complete 64-byte records')
    for name in fields[4:10] + fields[14:17]:
        summary[name] = struct.unpack('<f', struct.pack('<I', encoded[name]))[0]
        require(math.isfinite(summary[name]), f'Pitch {name} must be finite')
    require(summary['target_ttl_ms'] == 100 and summary['link_ttl_ms'] == 200,
            'Pitch identification must not relax target/link TTLs')
    if not enabled:
        require(summary['require_capture'] == 0 and all(summary[key] == 0 for key in fields[4:14]),
                'Ordinary APP must have no latent identification policy')
        return summary
    require(summary['require_capture'] == 1, 'Pitch identification must require synchronized recording')
    require(summary['dm_address'] == 0x00110102, 'Pitch identification needs CAN2 node1/master0x011')
    require(summary['soft_min_rad'] <= summary['min_rad'] < summary['max_rad'] <= summary['soft_max_rad'],
            'Pitch policy outside joint limits')
    require(0 < summary['max_amplitude_nm'] <= summary['max_expected_torque_nm'] <= 10,
            'Pitch torque policy outside confirmed MIT encoding range')
    require(summary['max_velocity_rad_s'] > 0 and summary['max_tracking_error_rad'] > 0,
            'Pitch velocity/tracking guards must be positive')
    require(0 < summary['min_half_period_us'] <= summary['max_duration_us'] < 0x80000000 and
            0 < summary['max_feedback_age_us'] < 0x80000000 and 0 < summary['max_step_us'] <= 5000,
            'Pitch timing guards invalid')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', required=True, type=Path)
    parser.add_argument('--nm', required=True, type=Path)
    parser.add_argument('--gm6020-bench', action='store_true', help='核对显式单GM6020 ID4台架镜像, 不放宽整车默认审计')
    parser.add_argument('--four-motor-bench', action='store_true', help='核对显式四电机CAN1台架镜像')
    parser.add_argument('--chassis-commissioning', action='store_true', help='核对实车底盘调试APP, 仅四轮可执行')
    parser.add_argument('--gimbal-commissioning', action='store_true', help='核对三关节独立机械调试镜像')
    parser.add_argument('--shooter-commissioning', action='store_true', help='核对双摩擦轮与单拨弹调试镜像')
    parser.add_argument('--pitch-identification', action='store_true',
                        help='Require explicitly enabled bounded Pitch experiment descriptor (gimbal only)')
    args = parser.parse_args()
    require(not args.pitch_identification or args.gimbal_commissioning,
            'Pitch identification audit requires --gimbal-commissioning')
    commissioning = args.chassis_commissioning or args.gimbal_commissioning or args.shooter_commissioning
    require(sum((args.gm6020_bench, args.four_motor_bench, args.chassis_commissioning,
                 args.gimbal_commissioning, args.shooter_commissioning)) <= 1,
            'Select only one APP role audit')
    elf = args.elf.read_bytes()
    image = args.elf.with_suffix('.bin').read_bytes()
    require(elf[:6] == b'\x7fELF\x01\x01', 'Expected a little-endian ELF32 image')
    require(struct.unpack_from('<H', elf, 18)[0] == 40, 'Expected ARM machine type')
    require(struct.unpack_from('<I', elf, 36)[0] & 0x400, 'Expected hard-float ABI')
    entries = load_symbols(args.nm, args.elf)
    require(not subprocess.check_output([str(args.nm), '-u', str(args.elf)], text=True).strip(),
            'Unresolved firmware symbols')
    for name in REQUIRED_SYMBOLS:
        require(find_symbol(entries, name) is not None, f'Missing production APP symbol: {name}')
    # APP 钩子必须是强定义: 弱定义说明这个镜像其实还在用"不下发任何输出"的默认钩子.
    for hook in ('ec_board_app_control', 'ec_board_app_commit'):
        found = find_symbol(entries, hook)
        require(found is not None and found[1] == 'T',
                f'{hook} must be strongly defined by the APP role')
    for pattern in REQUIRED_PATH_PATTERNS:
        require(has_prefix(entries, pattern),
                f'Missing real control/transmit path symbol: {pattern}')
    for pattern in FORBIDDEN_PATTERNS:
        require(not has_prefix(entries, pattern),
                f'Other role symbol present in the production APP image: {pattern}')
    # 控制实时路径不得依赖动态分配/异常.
    require(not has_prefix(entries, 'malloc') and
            not any(text.startswith('_Zn') and text != '_ZnwjPv' for _, _, text in entries),
            'Unexpected libc/C++ allocation dependency')
    require(not any(text in {'_read', '_write', '_close', '_lseek', '_sbrk'}
                    for _, _, text in entries),
            'Unused nosys I/O or heap stubs remain in the firmware')
    build_id = find_symbol(entries, 'g_ec_s1_build_id')
    if build_id is None:
        raise SystemExit('Missing firmware identity symbol')
    offset = build_id[0] - 0x08000000
    label = image[offset:offset + 64].split(b'\0', 1)[0].decode('ascii', 'replace')
    require(FIRMWARE_ID_PATTERN.match(label),
            f'Production APP firmware label must start with "app-": {label!r}')
    # 部署摘要: 核对镜像里没有臆测配置, 且声明的容量覆盖应用槽位.
    summary_symbol = find_symbol(entries, 'ec_app_deployment_summary')
    if summary_symbol is None:
        raise SystemExit('Missing deployment summary symbol')
    summary = read_summary(image, summary_symbol[0])
    pitch_symbol = find_symbol(entries, 'ec_pitch_identification_summary')
    if pitch_symbol is None:
        raise SystemExit('Missing explicit Pitch identification opt-in descriptor')
    pitch_summary = read_pitch_summary(image, pitch_symbol[0], args.pitch_identification)
    require(summary['magic'] == SUMMARY_MAGIC, 'Deployment summary magic mismatch')
    require(summary['layout'] == SUMMARY_LAYOUT, 'Deployment summary layout mismatch')
    require(summary['kind'] == CONFIG_KIND_REAL_DEPLOYMENT,
            'Deployment summary must declare a real deployment configuration')
    require(summary['synthetic_values'] == 0, 'Deployment image must not contain synthetic values')
    require(label.startswith('app-gm6020-id4-') == args.gm6020_bench,
            'Bench firmware label and explicit audit selection must agree')
    require(label.startswith('app-four-motor-') == args.four_motor_bench,
            'Four-motor firmware label and explicit audit selection must agree')
    require(label.startswith('app-chassis-') == args.chassis_commissioning,
            'Chassis commissioning label and explicit audit selection must agree')
    require(label.startswith('app-gimbal-') == args.gimbal_commissioning,
            'Gimbal commissioning label and explicit audit selection must agree')
    require(label.startswith('app-shooter-') == args.shooter_commissioning,
            'Shooter commissioning label and explicit audit selection must agree')
    require(summary['installed_dji'] == (9 if commissioning else 3 if args.four_motor_bench else 1 if args.gm6020_bench else 0) and
            summary['installed_dm'] == (1 if args.four_motor_bench or commissioning else 0),
            'Assembled motor count must match the explicit image role')
    require(summary['unknown_origin_installed'] == 0,
            'Deployment image must not declare assembled devices without a known origin')
    require(summary['dji_capacity'] == 9 and summary['dm_capacity'] == 1,
            'Deployment image must reserve nine DJI slots and one DM slot')
    require(summary['harness_dji'] == 9 and summary['harness_dm'] == 1,
            'Deployment image must carry the verified harness list (9 DJI + 1 DM)')

    # 地址级核对: 凡只看装配数量时, "底盘挂错总线"或"yaw/pitch 挂反总线"的镜像照样能通过;
    # 这里按符号读出的每槽总线与设备 ID 逐项比对, 与主机测试里的地址契约互为镜像级证据.
    def encode_dji(bus, device_id):
        return (bus & 0xFF) | ((device_id & 0xFF) << 8)

    # 实车 2026-10-27 只读采集确认: 底盘四轮与主 yaw 在 CAN1, 发射 yaw/两摩擦轮/拨弹在 CAN2.
    vehicle_slots = ((1, 1), (1, 2), (1, 3), (1, 4), (1, 4), (2, 6), (2, 5), (2, 6), (2, 7))
    if commissioning:
        expected_dji, expected_dm = dict(enumerate(vehicle_slots)), (2, 0x01, 0x011)
    elif args.four_motor_bench:
        expected_dji, expected_dm = {4: (1, 4), 6: (1, 2), 8: (1, 3)}, (2, 0x01, 0x011)
    elif args.gm6020_bench:
        expected_dji, expected_dm = {4: (1, 4)}, None
    else:
        expected_dji, expected_dm = {}, None
    for slot in range(9):
        want = expected_dji.get(slot)
        got = summary[f'dji_address_{slot}']
        if want is None:
            require(got == 0,
                    f'DJI slot {slot} must stay unassembled in this role, image reports {got:#06x}')
        else:
            require(got == encode_dji(*want),
                    f'DJI slot {slot} address mismatch: image {got:#06x} != expected '
                    f'{encode_dji(*want):#06x} (bus {want[0]}, device {want[1]})')
    if expected_dm is None:
        require(summary['dm_address'] == 0,
                'DM slot must stay unassembled in this role')
    else:
        dm_bus, dm_can_id, dm_master_id = expected_dm
        want_dm = (dm_bus & 0xFF) | ((dm_can_id & 0xFF) << 8) | ((dm_master_id & 0xFFFF) << 16)
        require(summary['dm_address'] == want_dm,
                f'DM address mismatch: image {summary["dm_address"]:#08x} != expected {want_dm:#08x} '
                f'(bus {dm_bus}, can_id {dm_can_id:#04x}, master_id {dm_master_id:#05x})')
    # 控制任务必须存在且使用静态栈 (不占用 FreeRTOS heap).
    for name in ('appControlStack', 'appControlTaskBuffer'):
        require(find_symbol(entries, name) is not None,
                f'Missing static control task storage: {name}')
    require(len(image) <= 1024 * 1024, 'Image exceeds flash capacity')
    print(json.dumps({'result': 'pass', 'elf': str(args.elf), 'image_bytes': len(image),
                      'pitch_identification': pitch_summary,
                      'elf_sha256': hashlib.sha256(elf).hexdigest(),
                      'bin_sha256': hashlib.sha256(image).hexdigest(),
                      'image_role': ('ACTUATING gimbal commissioning; source-declared zero/limits, hardware validation required' if args.gimbal_commissioning else
                                     'ACTUATING shooter commissioning, friction pair and single feeder' if args.shooter_commissioning else
                                     'ACTUATING vehicle chassis commissioning, remote startup, four C620 wheels' if args.chassis_commissioning else
                                     'ACTUATING four-motor CAN1 APP bench, maintenance-only' if args.four_motor_bench else
                                     'ACTUATING single GM6020 ID4 APP bench, maintenance-only'
                                     if args.gm6020_bench else
                                     'production APP (9 DJI + 1 DM, 1 kHz, business protocol)'),
                      'firmware_label': label,
                      'deployment_summary': summary,
                      'scope': 'offline image and source configuration checks only; '
                               'no hardware execution, no probe, no flashing'}, indent=2))


if __name__ == '__main__':
    main()
