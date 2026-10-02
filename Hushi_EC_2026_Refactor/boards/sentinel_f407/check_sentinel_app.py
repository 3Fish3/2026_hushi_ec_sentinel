"""只读核对正式 APP 镜像的角色身份、链接路径与静态预算, 不连接调试器或硬件.

与 check_firmware.py (默认 S1-S4 非运动审计) 和 check_capture.py (只读采集) 分开:
本脚本描述的是**会驱动执行器**的角色, 因此它核对的是"APP 路径确实被链接进来了",
而不是"镜像里没有发送路径". 三个角色的审计规则各自独立, 不互相放宽.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess

# 部署摘要布局 (board_app_config.hpp 的 DeploymentSummary): 11 个 uint32.
SUMMARY_FIELDS = ('magic', 'layout', 'kind', 'dji_capacity', 'dm_capacity', 'installed_dji',
                  'installed_dm', 'synthetic_values', 'unknown_origin_installed',
                  'harness_dji', 'harness_dm')
SUMMARY_MAGIC = 0x45434150
SUMMARY_LAYOUT = 1
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
    values = struct.unpack_from('<%dI' % len(SUMMARY_FIELDS), image, offset)
    return dict(zip(SUMMARY_FIELDS, values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', required=True, type=Path)
    parser.add_argument('--nm', required=True, type=Path)
    args = parser.parse_args()
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
    require(build_id is not None, 'Missing firmware identity symbol')
    offset = build_id[0] - 0x08000000
    label = image[offset:offset + 64].split(b'\0', 1)[0].decode('ascii', 'replace')
    require(FIRMWARE_ID_PATTERN.match(label),
            f'Production APP firmware label must start with "app-": {label!r}')
    # 部署摘要: 核对镜像里没有臆测配置, 且声明的容量覆盖应用槽位.
    summary_symbol = find_symbol(entries, 'ec_app_deployment_summary')
    require(summary_symbol is not None, 'Missing deployment summary symbol')
    summary = read_summary(image, summary_symbol[0])
    require(summary['magic'] == SUMMARY_MAGIC, 'Deployment summary magic mismatch')
    require(summary['layout'] == SUMMARY_LAYOUT, 'Deployment summary layout mismatch')
    require(summary['kind'] == CONFIG_KIND_REAL_DEPLOYMENT,
            'Deployment summary must declare a real deployment configuration')
    require(summary['synthetic_values'] == 0, 'Deployment image must not contain synthetic values')
    require(summary['installed_dji'] == 0 and summary['installed_dm'] == 0,
            'No verified vehicle parameters yet: the deployment table must not declare devices')
    require(summary['unknown_origin_installed'] == 0,
            'Deployment image must not declare assembled devices without a known origin')
    require(summary['dji_capacity'] == 9 and summary['dm_capacity'] == 1,
            'Deployment image must reserve nine DJI slots and one DM slot')
    require(summary['harness_dji'] == 9 and summary['harness_dm'] == 1,
            'Deployment image must carry the verified harness list (9 DJI + 1 DM)')
    # 控制任务必须存在且使用静态栈 (不占用 FreeRTOS heap).
    for name in ('appControlStack', 'appControlTaskBuffer'):
        require(find_symbol(entries, name) is not None,
                f'Missing static control task storage: {name}')
    require(len(image) <= 1024 * 1024, 'Image exceeds flash capacity')
    print(json.dumps({'result': 'pass', 'elf': str(args.elf), 'image_bytes': len(image),
                      'elf_sha256': hashlib.sha256(elf).hexdigest(),
                      'bin_sha256': hashlib.sha256(image).hexdigest(),
                      'image_role': 'production APP (9 DJI + 1 DM, 1 kHz, business protocol)',
                      'firmware_label': label,
                      'deployment_summary': summary,
                      'scope': 'offline image and source configuration checks only; '
                               'no hardware execution, no probe, no flashing'}, indent=2))


if __name__ == '__main__':
    main()
