"""把 S2–S4 外设写入 CubeMX .ioc, 保留已验证的时钟树. 不连接硬件."""
from pathlib import Path
import re

IOC = Path(__file__).resolve().parent / 'cubemx/sentinel_s1/sentinel_s1.ioc'

EXTRA = {
    'PG9.Locked': 'true',
    'PG9.Mode': 'Asynchronous',
    'PG9.Signal': 'USART6_RX',
    'PG14.Locked': 'true',
    'PG14.Mode': 'Asynchronous',
    'PG14.Signal': 'USART6_TX',
    'USART6.BaudRate': '115200',
    'USART6.VirtualMode': 'VM_ASYNC',
    'USART6.IPParameters': 'VirtualMode,BaudRate',
    'PA9.Locked': 'true',
    'PA9.Mode': 'Asynchronous',
    'PA9.Signal': 'USART1_TX',
    'PB7.Locked': 'true',
    'PB7.Mode': 'Asynchronous',
    'PB7.Signal': 'USART1_RX',
    'PC10.Locked': 'true',
    'PC10.Mode': 'Asynchronous',
    'PC10.Signal': 'USART3_TX',
    'PC11.Locked': 'true',
    'PC11.Mode': 'Asynchronous',
    'PC11.Signal': 'USART3_RX',
    'PD0.Locked': 'true',
    'PD0.Mode': 'CAN_Activate',
    'PD0.Signal': 'CAN1_RX',
    'PD1.Locked': 'true',
    'PD1.Mode': 'CAN_Activate',
    'PD1.Signal': 'CAN1_TX',
    'PB5.Locked': 'true',
    'PB5.Mode': 'CAN_Activate',
    'PB5.Signal': 'CAN2_RX',
    'PB6.Locked': 'true',
    'PB6.Mode': 'CAN_Activate',
    'PB6.Signal': 'CAN2_TX',
    'PB3.Locked': 'true',
    'PB3.Mode': 'Full_Duplex_Master',
    'PB3.Signal': 'SPI1_SCK',
    'PB4.Locked': 'true',
    'PB4.Mode': 'Full_Duplex_Master',
    'PB4.Signal': 'SPI1_MISO',
    'PA7.GPIOParameters': 'GPIO_PuPd',
    'PA7.GPIO_PuPd': 'GPIO_PULLUP',
    'PA7.Locked': 'true',
    'PA7.Mode': 'Full_Duplex_Master',
    'PA7.Signal': 'SPI1_MOSI',
    'USART1.BaudRate': '115200',
    'USART1.VirtualMode': 'VM_ASYNC',
    'USART1.IPParameters': 'VirtualMode,BaudRate',
    'USART3.BaudRate': '100000',
    'USART3.Parity': 'PARITY_EVEN',
    'USART3.WordLength': 'WORDLENGTH_9B',
    'USART3.VirtualMode': 'VM_ASYNC',
    'USART3.IPParameters': 'VirtualMode,BaudRate,Parity,WordLength',
    'CAN1.CalculateBaudRate': '1000000',
    'CAN1.CalculateTimeQuantum': '71.42857142857143',
    'CAN1.BS1': 'CAN_BS1_10TQ',
    'CAN1.BS2': 'CAN_BS2_3TQ',
    'CAN1.Prescaler': '3',
    'CAN1.TXFP': 'ENABLE',
    'CAN1.ABOM': 'ENABLE',
    'CAN1.NART': 'DISABLE',
    'CAN1.IPParameters': 'CalculateTimeQuantum,BS1,BS2,Prescaler,CalculateBaudRate,TXFP,NART,ABOM',
    'CAN2.CalculateBaudRate': '1000000',
    'CAN2.CalculateTimeQuantum': '71.42857142857143',
    'CAN2.BS1': 'CAN_BS1_10TQ',
    'CAN2.BS2': 'CAN_BS2_3TQ',
    'CAN2.Prescaler': '3',
    'CAN2.TXFP': 'ENABLE',
    'CAN2.ABOM': 'ENABLE',
    'CAN2.NART': 'DISABLE',
    'CAN2.IPParameters': 'CalculateTimeQuantum,BS1,BS2,Prescaler,CalculateBaudRate,TXFP,NART,ABOM',
    'SPI1.VirtualType': 'VM_MASTER',
    'SPI1.Mode': 'SPI_MODE_MASTER',
    'SPI1.Direction': 'SPI_DIRECTION_2LINES',
    'SPI1.CalculateBaudRate': '5.25 MBits/s',
    'SPI1.BaudRatePrescaler': 'SPI_BAUDRATEPRESCALER_16',
    'SPI1.CLKPolarity': 'SPI_POLARITY_HIGH',
    'SPI1.CLKPhase': 'SPI_PHASE_2EDGE',
    'SPI1.IPParameters': 'VirtualType,Mode,Direction,CalculateBaudRate,BaudRatePrescaler,CLKPolarity,CLKPhase',
    'Dma.Request0': 'USART1_RX',
    'Dma.Request1': 'USART1_TX',
    'Dma.Request2': 'USART3_RX',
    'Dma.Request3': 'USART6_RX',
    'Dma.Request4': 'USART6_TX',
    'Dma.RequestsNb': '5',
    'Dma.USART1_RX.0.Direction': 'DMA_PERIPH_TO_MEMORY',
    'Dma.USART1_RX.0.FIFOMode': 'DMA_FIFOMODE_DISABLE',
    'Dma.USART1_RX.0.Instance': 'DMA2_Stream5',
    'Dma.USART1_RX.0.MemDataAlignment': 'DMA_MDATAALIGN_BYTE',
    'Dma.USART1_RX.0.MemInc': 'DMA_MINC_ENABLE',
    'Dma.USART1_RX.0.Mode': 'DMA_CIRCULAR',
    'Dma.USART1_RX.0.PeriphDataAlignment': 'DMA_PDATAALIGN_BYTE',
    'Dma.USART1_RX.0.PeriphInc': 'DMA_PINC_DISABLE',
    'Dma.USART1_RX.0.Priority': 'DMA_PRIORITY_HIGH',
    'Dma.USART1_RX.0.RequestParameters':
        'Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode',
    'Dma.USART1_TX.1.Direction': 'DMA_MEMORY_TO_PERIPH',
    'Dma.USART1_TX.1.FIFOMode': 'DMA_FIFOMODE_DISABLE',
    'Dma.USART1_TX.1.Instance': 'DMA2_Stream7',
    'Dma.USART1_TX.1.MemDataAlignment': 'DMA_MDATAALIGN_BYTE',
    'Dma.USART1_TX.1.MemInc': 'DMA_MINC_ENABLE',
    'Dma.USART1_TX.1.Mode': 'DMA_NORMAL',
    'Dma.USART1_TX.1.PeriphDataAlignment': 'DMA_PDATAALIGN_BYTE',
    'Dma.USART1_TX.1.PeriphInc': 'DMA_PINC_DISABLE',
    'Dma.USART1_TX.1.Priority': 'DMA_PRIORITY_HIGH',
    'Dma.USART1_TX.1.RequestParameters':
        'Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode',
    'Dma.USART3_RX.2.Direction': 'DMA_PERIPH_TO_MEMORY',
    'Dma.USART3_RX.2.FIFOMode': 'DMA_FIFOMODE_DISABLE',
    'Dma.USART3_RX.2.Instance': 'DMA1_Stream1',
    'Dma.USART3_RX.2.MemDataAlignment': 'DMA_MDATAALIGN_BYTE',
    'Dma.USART3_RX.2.MemInc': 'DMA_MINC_ENABLE',
    'Dma.USART3_RX.2.Mode': 'DMA_CIRCULAR',
    'Dma.USART3_RX.2.PeriphDataAlignment': 'DMA_PDATAALIGN_BYTE',
    'Dma.USART3_RX.2.PeriphInc': 'DMA_PINC_DISABLE',
    'Dma.USART3_RX.2.Priority': 'DMA_PRIORITY_HIGH',
    'Dma.USART3_RX.2.RequestParameters':
        'Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode',
    'NVIC.USART6_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.DMA2_Stream1_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.DMA2_Stream6_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.USART1_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.USART3_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.DMA1_Stream1_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.DMA2_Stream5_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.DMA2_Stream7_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.CAN1_RX0_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'NVIC.CAN2_RX0_IRQn': r'true\:6\:0\:true\:false\:true\:true\:true\:true\:true',
    'ProjectManager.AskForMigrate': 'false',
    'ProjectManager.functionlistsort':
        '1-SystemClock_Config-RCC-false-HAL-false,'
        '2-MX_GPIO_Init-GPIO-false-HAL-true,'
        '3-MX_DMA_Init-DMA-false-HAL-true,'
        '4-MX_USART1_UART_Init-USART1-false-HAL-true,'
        '5-MX_USART3_UART_Init-USART3-false-HAL-true,'
        '6-MX_CAN1_Init-CAN1-false-HAL-true,'
        '7-MX_CAN2_Init-CAN2-false-HAL-true,'
        '8-MX_SPI1_Init-SPI1-false-HAL-true,'
        '9-MX_USART6_UART_Init-USART6-false-HAL-true',
}

# CubeMX 会重新排序编号, 必须重建完整列表, 不能按旧 S1 的编号增量覆盖.
ips = ['CAN1', 'CAN2', 'DMA', 'FREERTOS', 'NVIC', 'RCC', 'SPI1', 'SYS', 'USART1', 'USART3', 'USART6']
pins = ['PB5', 'PB4', 'PB3', 'PA14', 'PA13', 'PB7', 'PB6', 'PD0', 'PC11', 'PC10',
        'PD1', 'PA9', 'PH0-OSC_IN', 'PH1-OSC_OUT', 'PF6', 'PA4', 'PA7', 'PB0',
        'VP_FREERTOS_VS_CMSIS_V2', 'VP_SYS_VS_tim14', 'PG9', 'PG14']
EXTRA.update({f'Mcu.IP{i}': name for i, name in enumerate(ips)})
EXTRA.update({f'Mcu.Pin{i}': name for i, name in enumerate(pins)})
EXTRA.update({'Mcu.IPNb': str(len(ips)), 'Mcu.PinsNb': str(len(pins))})

# USART6 使用独立 DMA stream, 不占 USART1/USART3 的资源.
for request, index, stream, mode, direction in [
    ('USART6_RX', 3, 'DMA2_Stream1', 'DMA_CIRCULAR', 'DMA_PERIPH_TO_MEMORY'),
    ('USART6_TX', 4, 'DMA2_Stream6', 'DMA_NORMAL', 'DMA_MEMORY_TO_PERIPH'),
]:
    values = {
        'Instance': stream, 'Direction': direction, 'PeriphInc': 'DMA_PINC_DISABLE',
        'MemInc': 'DMA_MINC_ENABLE', 'PeriphDataAlignment': 'DMA_PDATAALIGN_BYTE',
        'MemDataAlignment': 'DMA_MDATAALIGN_BYTE', 'Mode': mode, 'Priority': 'DMA_PRIORITY_HIGH',
        'FIFOMode': 'DMA_FIFOMODE_DISABLE',
        'RequestParameters': 'Instance,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode',
    }
    for key, value in values.items():
        EXTRA[f'Dma.{request}.{index}.{key}'] = value


def main() -> None:
    text = IOC.read_text(encoding='utf-8')
    lines = text.splitlines()
    keys = {}
    order = []
    for line in lines:
        if line.startswith('#') or '=' not in line:
            order.append(('raw', line))
            continue
        key, value = line.split('=', 1)
        if re.fullmatch(r'Mcu\.(?:IP|Pin)\d+', key):
            continue
        keys[key] = value
        order.append(('kv', key))
    keys.update(EXTRA)
    out = []
    seen = set()
    for kind, payload in order:
        if kind == 'raw':
            out.append(payload)
            continue
        out.append(f'{payload}={keys[payload]}')
        seen.add(payload)
    for key in EXTRA:
        if key not in seen:
            out.append(f'{key}={keys[key]}')
    IOC.write_text('\n'.join(out) + '\n', encoding='utf-8')
    print(f'patched {IOC}')


if __name__ == '__main__':
    main()
