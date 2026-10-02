#ifndef EC_BOARD_MONITOR_H
#define EC_BOARD_MONITOR_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// USART6 唯一所有者; 与 S1 任务、同优先级 UART/DMA IRQ 配合使用.
void ec_monitor_init(void);
void ec_monitor_poll(void);
void ec_monitor_rx_event(void);
void ec_monitor_tx_complete(void);
void ec_monitor_uart_error(uint32_t error);
#ifdef __cplusplus
}
#endif
#endif
