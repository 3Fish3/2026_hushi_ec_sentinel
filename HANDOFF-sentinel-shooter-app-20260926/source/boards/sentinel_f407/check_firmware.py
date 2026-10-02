"""只读核对 S1-S4 非运动固件身份、向量和外设范围, 不连接调试器或硬件."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess


def require(condition, message):
    if not condition:
        raise SystemExit(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', required=True, type=Path)
    parser.add_argument('--nm', required=True, type=Path)
    parser.add_argument('--allow-gm6020-bench', action='store_true',
                        help='Explicitly inspect an actuating S5 image, never label it non-actuating')
    args = parser.parse_args()
    elf = args.elf.read_bytes()
    image = args.elf.with_suffix('.bin').read_bytes()
    require(elf[:6] == b'\x7fELF\x01\x01', 'Expected a little-endian ELF32 image')
    require(struct.unpack_from('<H', elf, 18)[0] == 40, 'Expected ARM machine type')
    require(struct.unpack_from('<I', elf, 36)[0] & 0x400, 'Expected hard-float ABI')
    output = subprocess.check_output([str(args.nm), '-g', '--defined-only', str(args.elf)], text=True)
    symbols = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 3:
            symbols[parts[2]] = (int(parts[0], 16), parts[1])
    require(not subprocess.check_output([str(args.nm), '-u', str(args.elf)], text=True).strip(),
            'Unresolved firmware symbols')
    vectors = {1: 'Reset_Handler', 2: 'NMI_Handler', 3: 'HardFault_Handler',
               4: 'MemManage_Handler', 5: 'BusFault_Handler', 6: 'UsageFault_Handler',
               11: 'SVC_Handler', 14: 'PendSV_Handler', 15: 'SysTick_Handler',
               28: 'DMA1_Stream1_IRQHandler', 36: 'CAN1_RX0_IRQHandler',
               53: 'USART1_IRQHandler', 55: 'USART3_IRQHandler',
               61: 'TIM8_TRG_COM_TIM14_IRQHandler', 80: 'CAN2_RX0_IRQHandler',
               73: 'DMA2_Stream1_IRQHandler', 84: 'DMA2_Stream5_IRQHandler',
               85: 'DMA2_Stream6_IRQHandler', 86: 'DMA2_Stream7_IRQHandler', 87: 'USART6_IRQHandler'}
    require(struct.unpack_from('<I', image)[0] == 0x20020000, 'Unexpected initial MSP')
    for index, name in vectors.items():
        require(name in symbols, f'Missing vector implementation: {name}')
        address, kind = symbols[name]
        require(struct.unpack_from('<I', image, index * 4)[0] == (address | 1),
                f'Wrong vector entry: {name}')
        require(0x08000000 <= address < 0x08000000 + len(image), f'Handler outside image: {name}')
        if name != 'Reset_Handler':
            require(kind == 'T', f'Weak/default handler still selected: {name}')
    require(struct.unpack_from('<I', elf, 24)[0] == (symbols['Reset_Handler'][0] | 1), 'Wrong ELF entry')
    for name in ('g_ec_s1', 'g_ec_s1_build_id', 'g_ec_io', 'ec_s1_task', 'ec_s1_fail',
                 'ec_board_io_init', 'vApplicationMallocFailedHook', 'vApplicationStackOverflowHook'):
        require(name in symbols, f'Missing diagnostic or protection symbol: {name}')
    # TIM14 共用的 HAL IRQ 分发会引用空的 PWM 回调, 它不等于配置或启动 PWM.
    forbidden_prefix = ('HAL_TIM_PWM_Init', 'HAL_TIM_PWM_Start', 'HAL_TIM_PWM_ConfigChannel', 'MX_USB_')
    require(not any(name.startswith(forbidden_prefix) for name in symbols), 'Unexpected actuator/USB symbol')
    if args.allow_gm6020_bench:
        require('g_ec_s5_signature' in symbols, 'Expected the explicit S5 bench marker')
    else:
        require('g_ec_s5_signature' not in symbols, 'Actuating S5 image; not an S1-S4 artifact')
        require(not any('encode_dji' in name or 'HalCanTx' in name for name in symbols),
                'S2-S4 image must not link motor command encoding or CAN TX adapter')
        # 默认镜像的装配层必须存在且钩子可被 APP 强定义覆盖, 同时不得携带任何发送路径.
        for hook in ('ec_board_app_control', 'ec_board_app_commit'):
            require(symbols.get(hook, (0, None))[1] == 'W',
                    f'{hook} must be a weak APP override point')
        require(not any('DjiTxGroup' in name or 'HAL_CAN_AddTxMessage' in name for name in symbols),
                'Default image must not contain any actuator transmit path')
    # ARM size_t 为 unsigned int; optional 在 Debug 可能保留 placement new(size_t, void*).
    # _ZnwjPv 仅返回调用者提供的地址, 不分配内存; 普通 new/new[] 仍禁止.
    require('malloc' not in symbols and not any(name.startswith('_Zn') and name != '_ZnwjPv' for name in symbols),
            'Unexpected libc/C++ allocation dependency')
    require(not {'_read', '_write', '_close', '_lseek', '_sbrk'} & symbols.keys(),
            'Unused nosys I/O or heap stubs remain in the firmware')

    root = Path(__file__).resolve().parent / 'cubemx' / 'sentinel_s1'
    ioc = dict(line.split('=', 1) for line in (root / 'sentinel_s1.ioc').read_text(encoding='utf-8').splitlines()
               if '=' in line and not line.startswith('#'))
    expected = {'Mcu.CPN': 'STM32F407IGH6', 'RCC.HSE_VALUE': '12000000',
                'RCC.PLLSourceVirtual': 'RCC_PLLSOURCE_HSE', 'RCC.SYSCLKFreq_VALUE': '168000000',
                'RCC.APB1Freq_Value': '42000000', 'RCC.APB2Freq_Value': '84000000',
                'ProjectManager.FirmwarePackage': 'STM32Cube FW_F4 V1.28.3',
                'PF6.PinState': 'GPIO_PIN_RESET', 'PA4.PinState': 'GPIO_PIN_SET',
                'PB0.PinState': 'GPIO_PIN_SET', 'PF6.Signal': 'GPIO_Output',
                'PA4.Signal': 'GPIO_Output', 'PB0.Signal': 'GPIO_Output',
                'USART1.BaudRate': '115200', 'USART3.BaudRate': '100000',
                'USART6.BaudRate': '115200', 'PG9.Signal': 'USART6_RX', 'PG14.Signal': 'USART6_TX',
                'Dma.USART6_RX.3.Mode': 'DMA_CIRCULAR', 'Dma.USART6_RX.3.Instance': 'DMA2_Stream1',
                'Dma.USART6_TX.4.Instance': 'DMA2_Stream6',
                'USART3.Parity': 'PARITY_EVEN', 'USART3.WordLength': 'WORDLENGTH_9B',
                'Dma.USART1_RX.0.Mode': 'DMA_CIRCULAR', 'Dma.USART3_RX.2.Mode': 'DMA_CIRCULAR',
                'PA9.Signal': 'USART1_TX', 'PB7.Signal': 'USART1_RX',
                'PC11.Signal': 'USART3_RX', 'PD0.Signal': 'CAN1_RX', 'PB5.Signal': 'CAN2_RX',
                'PB3.Signal': 'SPI1_SCK',
                'SPI1.BaudRatePrescaler': 'SPI_BAUDRATEPRESCALER_16',
                'SPI1.CalculateBaudRate': '5.25 MBits/s'}
    for key, value in expected.items():
        require(ioc.get(key) == value, f'Unexpected IOC configuration: {key}')
    uart_source = (root / 'Core/Src/usart.c').read_text(encoding='utf-8')
    irq_source = (root / 'Core/Src/stm32f4xx_it.c').read_text(encoding='utf-8')
    require('huart6.Init.BaudRate = 115200;' in uart_source and
            'GPIO_AF8_USART6' in uart_source and 'ec_board_on_uart_irq(&huart6);' in irq_source,
            'Missing USART6 configuration or IDLE hook')
    spi_source = (root / 'Core/Src/spi.c').read_text(encoding='utf-8')
    require(re.search(r'hspi1\.Init\.BaudRatePrescaler\s*=\s*SPI_BAUDRATEPRESCALER_16\s*;', spi_source),
            'Generated SPI1 prescaler must match IOC and stay below BMI088 10 MHz limit')
    ips = {value for key, value in ioc.items() if re.fullmatch(r'Mcu\.IP\d+', key)}
    require({'FREERTOS', 'NVIC', 'RCC', 'SYS', 'DMA', 'USART1', 'USART3', 'USART6', 'CAN1', 'CAN2', 'SPI1'} <= ips,
            'Board IOC missing S2-S4 peripherals')
    require('USB' not in ips and 'TIM1' not in ips, 'Unexpected motion/USB IP in non-actuating image')
    require(len(image) <= 1024 * 1024, 'Image exceeds flash capacity')
    print(json.dumps({'result': 'pass', 'elf': str(args.elf), 'image_bytes': len(image),
                      'elf_sha256': hashlib.sha256(elf).hexdigest(),
                      'bin_sha256': hashlib.sha256(image).hexdigest(),
                      'image_role': 'S5 ACTUATING GM6020 bench' if args.allow_gm6020_bench else 'S1-S4 non-actuating',
                      'scope': 'offline image and source configuration checks only; no hardware execution'}, indent=2))


if __name__ == '__main__':
    main()
