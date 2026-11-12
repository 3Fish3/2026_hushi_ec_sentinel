#ifndef EC_BOARD_IO_H
#define EC_BOARD_IO_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

struct EcUartDiag {
    uint32_t rx_bytes, tx_bytes, tx_dropped, overflows, discarded;
    uint32_t framing, noise, overrun, idle;
};
struct EcCanDiag {
    uint32_t rx_frames, rx_dropped, ignored, last_id, last_dlc, last_ms, last_bus;
    uint32_t online_count;
    float last_speed_rad_s;
    uint8_t last_device_id, last_temp_c, last_temp_valid;
};
struct EcRemoteDiag {
    uint32_t frames, rejected, last_ms;
    int16_t ch0, ch1, ch2, ch3, ch4;
    // 完整 DR16 样本的其余字段: 键鼠与按键. typed 交接见 board_inputs.hpp.
    int16_t mouse_x, mouse_y, mouse_z;
    uint16_t keys;
    uint8_t switch_left, switch_right, online, mouse_left, mouse_right;
};
struct EcImuDiag {
    uint32_t samples, errors, last_ms;
    uint8_t acc_id, gyro_id, ready, heater_enabled;
    float ax, ay, az, gx, gy, gz, temp_c;
};
struct EcBoardIo {
    uint32_t magic, abi;
    struct EcUartDiag usart1, usart3;
    struct EcCanDiag can;
    struct EcRemoteDiag remote;
    struct EcImuDiag imu;
};
// 停核读取; 不是运行中一致快照 (控制任务要一致样本请用 board_inputs.hpp 的 typed 交接).
// 加热默认关闭, 不发送电机控制帧.
extern volatile struct EcBoardIo g_ec_io;
void ec_board_io_init(void);
// TIM5 自由运行微秒时钟; 毫秒会话时间继续使用 HAL_GetTick.
uint32_t ec_board_micros(void);
void ec_board_io_poll(uint32_t now_ms);
void ec_board_on_uart_irq(void* huart);

// 角色相关的有界入口. 单周期搬运/解析量有硬上界, 不做无界 drain:
//  - 默认/S5/采集角色由 10 ms 任务调用 (采集角色不调用 CAN 收帧);
//  - 正式 APP 角色由 1 kHz 控制任务调用 CAN 收帧/IMU 采样/USART1 搬运, 10 ms 任务不重复读.
void ec_board_drain_can(uint32_t now_ms, size_t budget);
void ec_board_poll_uart1(void);
void ec_board_poll_imu(uint32_t now_ms);
// USART1 有界发送 (正式 APP 的业务协议应答): 队列满时拒绝并计数, 不阻塞调用者.
int ec_board_uart1_try_send(const uint8_t* data, size_t size);
uint32_t ec_board_uart1_overflows(void);

#ifdef __cplusplus
}
#endif
#endif
