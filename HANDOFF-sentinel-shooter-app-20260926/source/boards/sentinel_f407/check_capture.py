"""离线核对独立采集镜像; 允许限定 DM 参数/状态读, 不允许电机驱动/设置写入路径."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def check_image(elf_path, nm):
    elf = elf_path.read_bytes()
    image = elf_path.with_suffix('.bin').read_bytes()
    def require(ok, message):
        if not ok:
            raise ValueError(message)
    require(elf[:6] == b'\x7fELF\x01\x01', 'expected little-endian ELF32')
    require(struct.unpack_from('<H', elf, 18)[0] == 40, 'expected ARM')
    require(struct.unpack_from('<I', elf, 36)[0] & 0x400, 'expected hard-float ABI')
    require(struct.unpack_from('<I', image)[0] == 0x20020000, 'unexpected initial MSP')
    symbols = subprocess.check_output([str(nm), '-C', '--defined-only', str(elf_path)], text=True)
    raw = subprocess.check_output([str(nm), '--defined-only', str(elf_path)], text=True)
    require(not subprocess.check_output([str(nm), '-u', str(elf_path)], text=True).strip(), 'unresolved symbols')
    table = {}
    for line in raw.splitlines():
        fields = line.split()
        if len(fields) == 3:
            table[fields[2]] = (int(fields[0], 16), fields[1])
    for name in ('g_ec_capture_signature', 'HAL_CAN_AddTxMessage', 'HAL_CAN_ErrorCallback', 'g_ec_s1_build_id'):
        require(name in table, f'missing capture symbol: {name}')
    for name in ('allowed_read', 'ReadOnlyTx', 'Collector::receive', 'Collector::poll'):
        require(name in symbols, f'missing acquisition path: {name}')
    forbidden = ('encode_dji', 'encode_mit', 'encode_dm_enable', 'encode_dm_disable', 'DjiTxGroup',
                 'DjiAxis', 'DmDevice', 'HalCanTx', 'g_ec_s5_signature', 'HAL_TIM_PWM_Init',
                 'HAL_TIM_PWM_Start', 'HAL_TIM_PWM_ConfigChannel', 'MX_USB_')
    require(not any(name in symbols for name in forbidden), 'actuation/allocation symbol in capture image')
    require('malloc' not in table and not any(name.startswith('_Zn') and name != '_ZnwjPv' for name in table),
            'unexpected libc/C++ allocation')
    for index, name in {1: 'Reset_Handler', 15: 'SysTick_Handler', 36: 'CAN1_RX0_IRQHandler',
                        80: 'CAN2_RX0_IRQHandler', 38: 'CAN1_SCE_IRQHandler', 82: 'CAN2_SCE_IRQHandler', 87: 'USART6_IRQHandler'}.items():
        require(name in table, f'missing vector {name}')
        address, kind = table[name]
        require(struct.unpack_from('<I', image, index*4)[0] == address | 1, f'wrong vector {name}')
        require(0x08000000 <= address < 0x08000000 + len(image), f'vector outside image: {name}')
        require(name == 'Reset_Handler' or kind == 'T', f'default vector {name}')
    require(len(image) <= 1024*1024, 'Flash overflow')
    return {'result': 'pass', 'role': 'capture-dm-read-only', 'elf': str(elf_path),
            'elf_sha256': hashlib.sha256(elf).hexdigest(), 'image_bytes': len(image),
            'hardware_verified': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', type=Path, required=True)
    parser.add_argument('--nm', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(check_image(args.elf, args.nm)))
