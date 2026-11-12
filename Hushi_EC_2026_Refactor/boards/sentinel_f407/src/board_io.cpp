#include "board_io.h"
#include "board_app.hpp"
#include "board_capture.hpp"
#include "board_monitor.h"
#include "board_assembly.hpp"
#include "board_bench.hpp"
#include "board_raw.hpp"
#include "board_inputs.hpp"
#include "board_can_queue.hpp"
#include "board_spi_transfer.hpp"
#include <limits>
#include "board_observation.hpp"
#include "board_registry.h"
#include "ec/channel.hpp"
#include "ec/drivers/bmi088.hpp"
#include "ec/drivers/bmi088_device.hpp"
#include "ec/drivers/device_watch.hpp"
#include "ec/drivers/dji.hpp"
#include "ec/drivers/dr16.hpp"
#include "ec/platform/stm32f407.hpp"
#include "ec/transport/byte_pipe.hpp"
#include "can.h"
#include "spi.h"
#include "usart.h"

extern "C" {
volatile EcBoardIo g_ec_io{};
}

namespace {
constexpr std::size_t kUart1Rx = 512;
constexpr std::size_t kUart1Tx = 512;
constexpr std::size_t kUart3Rx = 64;
constexpr ec::Lifetime kCanTtl{50};
using Pipe1 = ec::transport::DmaBytePipe<kUart1Rx>;
using Pipe3 = ec::transport::DmaBytePipe<kUart3Rx>;
using Tx1 = ec::transport::ByteTxQueue<kUart1Tx>;

Pipe1 usart1_rx{};
Pipe3 usart3_rx{};
Tx1 usart1_tx{};
ec::drivers::Dr16Stream dr16{};
ec::stm32f407::IrqCriticalSection uart_lock{};
bool remote_valid{false};
#if !EC_SENTINEL_CAPTURE
// 队列容量与单周期解析上限是角色相关的编译期常量 (推导见 board_role.hpp):
// 正式 APP 为 1 kHz 控制周期与九 DJI + 一 DM 的 10 帧/ms 输入率设计.
ec::stm32f407::IrqCriticalSection can_lock{};
#if EC_SENTINEL_APP
ec::sentinel::board::CanInbox<ec::sentinel::can_queue_capacity / 2,
    ec::stm32f407::IrqCriticalSection> can_rx{can_lock};
#else
ec::EventQueue<ec::drivers::StampedCanFrame, ec::sentinel::can_queue_capacity,
    ec::stm32f407::IrqCriticalSection> can_rx{can_lock};
#endif
ec::drivers::DeviceWatch<ec::sentinel::can_watch_capacity> devices{};
#endif
bool tx_busy{false};
std::size_t tx_inflight{0};
bool imu_ready{false};
void bump(volatile std::uint32_t& value) noexcept { value = value + 1; }
std::optional<ec::drivers::Bmi088Device> imu{};

void publish_uart(UART_HandleTypeDef* huart, bool idle = false) {
    if (huart->hdmarx == nullptr) { return; }
    if (huart->Instance == USART6) { ec_monitor_rx_event(); return; }
    const auto ndtr = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (huart->Instance == USART1) {
        usart1_rx.on_dma_write((Pipe1::capacity() - ndtr) % Pipe1::capacity());
        if (idle) { bump(g_ec_io.usart1.idle); }
    } else if (huart->Instance == USART3) {
        // USART3 与其 DMA IRQ 同优先级, 共同独占分帧器, 不会互相抢占.
        const auto overflows = usart3_rx.overflows();
        usart3_rx.on_dma_write((Pipe3::capacity() - ndtr) % Pipe3::capacity());
        if (usart3_rx.overflows() != overflows) {
            dr16.reset();
            remote_valid = false;
            ec::sentinel::input_remote_lost();
            g_ec_io.remote.online = 0;
        }
        std::uint8_t tmp[kUart3Rx]{};
        const auto n = usart3_rx.read(tmp);
        dr16.push({tmp, n});
        if (!idle) { return; }
        bump(g_ec_io.usart3.idle);
        const auto sample = dr16.on_idle();
        g_ec_io.remote.frames = dr16.accepted;
        g_ec_io.remote.rejected = dr16.rejected;
        if (sample) {
            const ec::Tick at{HAL_GetTick()};
            // 完整样本先交给 typed 交接(含鼠标/按键), 控制任务不必再解析协议.
            ec::sentinel::input_publish_remote(*sample, at);
            g_ec_io.remote.ch0 = sample->ch0;
            g_ec_io.remote.ch1 = sample->ch1;
            g_ec_io.remote.ch2 = sample->ch2;
            g_ec_io.remote.ch3 = sample->ch3;
            g_ec_io.remote.ch4 = sample->ch4;
            g_ec_io.remote.switch_left = static_cast<std::uint8_t>(sample->switch_left);
            g_ec_io.remote.switch_right = static_cast<std::uint8_t>(sample->switch_right);
            g_ec_io.remote.mouse_x = sample->mouse_x;
            g_ec_io.remote.mouse_y = sample->mouse_y;
            g_ec_io.remote.mouse_z = sample->mouse_z;
            g_ec_io.remote.mouse_left = sample->mouse_left ? 1 : 0;
            g_ec_io.remote.mouse_right = sample->mouse_right ? 1 : 0;
            g_ec_io.remote.keys = sample->keys;
            g_ec_io.remote.last_ms = at.ms;
            remote_valid = true;
            g_ec_io.remote.online = 1;
        }
    }
}

void note_pipe_error(UART_HandleTypeDef* huart, ec::transport::UartError error) {
    if (huart->Instance == USART1) { usart1_rx.note_error(error); }
    else { usart3_rx.note_error(error); }
}

void note_uart_error(UART_HandleTypeDef* huart, std::uint32_t code) {
    auto* diag = huart->Instance == USART1 ? &g_ec_io.usart1 : &g_ec_io.usart3;
    if (code & HAL_UART_ERROR_FE) {
        note_pipe_error(huart, ec::transport::UartError::framing);
        bump(diag->framing);
    }
    if (code & HAL_UART_ERROR_NE) {
        note_pipe_error(huart, ec::transport::UartError::noise);
        bump(diag->noise);
    }
    if (code & HAL_UART_ERROR_ORE) {
        note_pipe_error(huart, ec::transport::UartError::overrun);
        bump(diag->overrun);
    }
}

void start_rx(UART_HandleTypeDef* huart, std::uint8_t* buf, std::uint16_t n) {
    // 启动前或 HAL 完成 RX 中止后, 硬件/软件游标必须一起回到零.
    if (huart->Instance == USART1) { usart1_rx.reset(); }
    else {
        usart3_rx.reset();
        dr16.reset();
        remote_valid = false;
        ec::sentinel::input_remote_lost();
        g_ec_io.remote.online = 0;
    }
    if (HAL_UART_Receive_DMA(huart, buf, n) != HAL_OK) { Error_Handler(); }
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void kick_tx() {
    if (tx_busy) { return; }
    const auto chunk = usart1_tx.peek();
    if (chunk.empty()) { return; }
    tx_inflight = chunk.size();
    tx_busy = true;
    if (HAL_UART_Transmit_DMA(&huart1, const_cast<std::uint8_t*>(chunk.data()),
                              static_cast<std::uint16_t>(tx_inflight)) != HAL_OK) {
        tx_busy = false;
        tx_inflight = 0;
        bump(g_ec_io.usart1.tx_dropped);
    }
}

void copy_uart_stats() {
    g_ec_io.usart1.rx_bytes = usart1_rx.received();
    g_ec_io.usart1.overflows = usart1_rx.overflows();
    g_ec_io.usart1.discarded = usart1_rx.discarded();
    g_ec_io.usart3.rx_bytes = usart3_rx.received();
    g_ec_io.usart3.overflows = usart3_rx.overflows();
    g_ec_io.usart3.discarded = usart3_rx.discarded();
}

bool spi_xfer(std::uint8_t* tx, std::uint8_t* rx, std::uint16_t n) {
    // 单任务独占 SPI1, 8 bit 全双工; 100 us 是事务截止, 不是 HAL 毫秒超时.
    struct Io {
        SPI_TypeDef* spi;
        ec::MicroTick now() const noexcept { return {ec_board_micros()}; }
        bool tx_ready() const noexcept { return spi->SR & SPI_SR_TXE; }
        bool rx_ready() const noexcept { return spi->SR & SPI_SR_RXNE; }
        bool busy() const noexcept { return spi->SR & SPI_SR_BSY; }
        bool error() const noexcept { return spi->SR & (SPI_SR_OVR | SPI_SR_MODF); }
        void write(std::uint8_t value) noexcept {
            *reinterpret_cast<volatile std::uint8_t*>(&spi->DR) = value;
        }
        std::uint8_t read() const noexcept {
            return *reinterpret_cast<volatile std::uint8_t*>(&spi->DR);
        }
    } io{hspi1.Instance};
    __HAL_SPI_ENABLE(&hspi1);
    const bool ok = ec::sentinel::board::transfer_spi(io, {tx, n}, {rx, n}, 100);
    if (!ok) {
        __HAL_SPI_DISABLE(&hspi1);
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    }
    return ok;
}

bool accel_read(std::uint8_t reg, std::uint8_t* data, std::uint16_t n) {
    std::uint8_t tx[8]{}, rx[8]{};
    if (static_cast<std::size_t>(n) + 2 > sizeof(tx)) { return false; }
    tx[0] = static_cast<std::uint8_t>(reg | ec::drivers::bmi088_read_bit);
    HAL_GPIO_WritePin(IMU_ACCEL_CS_GPIO_Port, IMU_ACCEL_CS_Pin, GPIO_PIN_RESET);
    const bool ok = spi_xfer(tx, rx, static_cast<std::uint16_t>(n + 2));
    HAL_GPIO_WritePin(IMU_ACCEL_CS_GPIO_Port, IMU_ACCEL_CS_Pin, GPIO_PIN_SET);
    return ok && ec::drivers::parse_bmi088_accel_spi_read({rx, static_cast<std::size_t>(n + 2)}, {data, n});
}

bool gyro_read(std::uint8_t reg, std::uint8_t* data, std::uint16_t n) {
    std::uint8_t tx[8]{}, rx[8]{};
    if (static_cast<std::size_t>(n) + 1 > sizeof(tx)) { return false; }
    tx[0] = static_cast<std::uint8_t>(reg | ec::drivers::bmi088_read_bit);
    HAL_GPIO_WritePin(IMU_GYRO_CS_GPIO_Port, IMU_GYRO_CS_Pin, GPIO_PIN_RESET);
    const bool ok = spi_xfer(tx, rx, static_cast<std::uint16_t>(n + 1));
    HAL_GPIO_WritePin(IMU_GYRO_CS_GPIO_Port, IMU_GYRO_CS_Pin, GPIO_PIN_SET);
    return ok && ec::drivers::parse_bmi088_gyro_spi_read({rx, static_cast<std::size_t>(n + 1)}, {data, n});
}

bool accel_write(std::uint8_t reg, std::uint8_t value) {
    std::uint8_t tx[2]{reg, value}, rx[2]{};
    HAL_GPIO_WritePin(IMU_ACCEL_CS_GPIO_Port, IMU_ACCEL_CS_Pin, GPIO_PIN_RESET);
    const bool ok = spi_xfer(tx, rx, 2);
    HAL_GPIO_WritePin(IMU_ACCEL_CS_GPIO_Port, IMU_ACCEL_CS_Pin, GPIO_PIN_SET);
    return ok;
}

bool gyro_write(std::uint8_t reg, std::uint8_t value) {
    std::uint8_t tx[2]{reg, value}, rx[2]{};
    HAL_GPIO_WritePin(IMU_GYRO_CS_GPIO_Port, IMU_GYRO_CS_Pin, GPIO_PIN_RESET);
    const bool ok = spi_xfer(tx, rx, 2);
    HAL_GPIO_WritePin(IMU_GYRO_CS_GPIO_Port, IMU_GYRO_CS_Pin, GPIO_PIN_SET);
    return ok;
}

// 板侧 SPI 适配: 片选、SPI 时序与加速度计的虚读字节都按已验证方式处理,
// 寄存器语义、量程与读取次序交给通用设备层. 默认固件不接任何加热输出.
struct BoardImuBus final : ec::drivers::Bmi088Bus {
    [[nodiscard]] bool read(bool accel, std::uint8_t reg,
        std::span<std::uint8_t> out) noexcept override {
        return accel ? accel_read(reg, out.data(), static_cast<std::uint16_t>(out.size()))
                     : gyro_read(reg, out.data(), static_cast<std::uint16_t>(out.size()));
    }
    [[nodiscard]] bool write(bool accel, std::uint8_t reg, std::uint8_t value) noexcept override {
        return accel ? accel_write(reg, value) : gyro_write(reg, value);
    }
    void delay_ms(std::uint32_t ms) noexcept override { HAL_Delay(ms); }
    [[nodiscard]] ec::MicroTick micros() const noexcept override { return {ec_board_micros()}; }
};

BoardImuBus imu_bus{};

void init_can(CAN_HandleTypeDef* handle, std::uint32_t bank) {
    CAN_FilterTypeDef filter{};
    filter.FilterBank = bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(handle, &filter) != HAL_OK) { Error_Handler(); }
    if (HAL_CAN_Start(handle) != HAL_OK) { Error_Handler(); }
    std::uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING;
#if EC_SENTINEL_CAPTURE
    notifications |= CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_ERROR | CAN_IT_BUSOFF |
        CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_LAST_ERROR_CODE;
#endif
    if (HAL_CAN_ActivateNotification(handle, notifications) != HAL_OK) { Error_Handler(); }
}

void init_imu() {
    g_ec_io.imu.heater_enabled = 0;
    imu.emplace(imu_bus, ec::sentinel::kImuTtl);
    const auto result = imu->init(EC_SENTINEL_APP || EC_SENTINEL_CAPTURE);
    g_ec_io.imu.acc_id = imu->accel_chip_id();
    g_ec_io.imu.gyro_id = imu->gyro_chip_id();
    imu_ready = result == ec::drivers::Bmi088Result::ok;
    g_ec_io.imu.ready = imu_ready ? 1 : 0;
    if (!imu_ready) { bump(g_ec_io.imu.errors); }
}

void poll_uart1() {
    std::uint8_t tmp[ec::sentinel::uart1_copy_chunk]{};
    for (std::size_t remaining = ec::sentinel::uart1_poll_budget; remaining != 0;) {
        std::size_t n{};
#if EC_SENTINEL_APP
        // 正式 APP: 只搬运字节, 分帧/会话/应答在 APP 侧 (本层不解析业务载荷).
        {
            ec::ScopedCritical guard(uart_lock);
            n = usart1_rx.read({tmp, remaining < sizeof(tmp) ? remaining : sizeof(tmp)});
        }
        if (n != 0) { ec::sentinel::ec_app_link_bytes({tmp, n}); }
#else
        {
            ec::ScopedCritical guard(uart_lock);
            // TX 未释放时保留 RX 数据, 不把短时背压误计成发送丢弃.
            const auto room = usart1_tx.free();
            const auto limit = room < sizeof(tmp) ? room : sizeof(tmp);
            n = usart1_rx.read({tmp, limit < remaining ? limit : remaining});
            if (n != 0 && !usart1_tx.try_push({tmp, n})) {
                g_ec_io.usart1.tx_dropped = g_ec_io.usart1.tx_dropped + n;
            }
            kick_tx();
        }
#endif
        if (n == 0) { break; }
        remaining -= n;
    }
}

void poll_uart3() {
    ec::ScopedCritical guard(uart_lock);
    // 在同一临界区取当前时间, 避免 ISR 的新时间戳晚于任务入口的 now_ms.
    g_ec_io.remote.online = remote_valid &&
        ec::fresh({HAL_GetTick()}, {g_ec_io.remote.last_ms}, ec::sentinel::kRemoteTtl) ? 1 : 0;
}

#if !EC_SENTINEL_CAPTURE
// 收帧: 逐帧路由, 单周期解析量有硬上界; 剩余帧留到下一周期.
// 不用无界 drain —— 收帧中断持续入队时无界循环会拖长控制周期.
void drain_can(std::uint32_t now_ms, std::size_t budget) {
    for (std::size_t processed = 0; processed < budget; ++processed) {
        const auto item = can_rx.try_pop();
        if (!item) { break; }
        ec_bench_receive(item->bus,item->frame,item->received_at,item->received_us);
        // 路由目标按角色确定: 正式 APP 的设备属于应用执行层, 其余角色属于板级静态装配层.
#if EC_SENTINEL_APP
        ec::sentinel::ec_app_route(item->bus, item->frame, item->received_at, item->received_us);
#else
        ec::sentinel::assembly_route(item->bus, item->frame, item->received_at);
#endif
        if (!devices.note(item->bus, item->frame.id, item->received_at)) {
            bump(g_ec_io.can.rx_dropped);
        }
        g_ec_io.can.last_id = item->frame.id;
        g_ec_io.can.last_dlc = item->frame.size;
        g_ec_io.can.last_ms = item->received_at.ms;
        g_ec_io.can.last_bus = item->bus;
        const auto mode = ec::sentinel::dji_mode(item->bus, item->frame.id);
        const auto decoded = mode ? ec::drivers::decode_dji(*mode, item->frame) : std::nullopt;
        if (decoded) {
            g_ec_io.can.last_device_id = decoded->device_id;
            g_ec_io.can.last_speed_rad_s = decoded->rotor_speed_rad_s;
            g_ec_io.can.last_temp_valid = decoded->temperature_c.has_value() ? 1 : 0;
            g_ec_io.can.last_temp_c = decoded->temperature_c.value_or(0);
        }
    }
    std::uint32_t online = 0;
    for (const auto& slot : devices.slots()) {
        if (slot.used && ec::fresh({now_ms}, slot.last_seen, kCanTtl)) { ++online; }
    }
    g_ec_io.can.online_count = online;
}
#endif

void poll_imu(std::uint32_t now_ms) {
    if (!imu || !imu_ready) { return; }
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
    const auto result = imu->read_timed({now_ms});
    if (result == ec::drivers::Bmi088Result::transport_error) { bump(g_ec_io.imu.errors); }
    const auto& reading = imu->last_reading();
    if (!imu->ready()) { return; }
    // 即使没有新向量也交接 FIFO 缺口计数, 保留旧样本时刻, 不刷新数据年龄.
    ec::sentinel::input_publish_imu_reading(reading);
    if (!(reading.times.gyro_updated || reading.times.accel_updated ||
            reading.times.temperature_updated)) { return; }
#else
    if (imu->read({now_ms}) != ec::drivers::Bmi088Result::ok) { bump(g_ec_io.imu.errors); return; }
    const auto& reading = imu->last_reading();
    ec::sentinel::input_publish_imu(reading.sample, reading.sampled_at);
#endif
    g_ec_io.imu.ax = reading.sample.acc_mps2[0];
    g_ec_io.imu.ay = reading.sample.acc_mps2[1];
    g_ec_io.imu.az = reading.sample.acc_mps2[2];
    g_ec_io.imu.gx = reading.sample.gyro_rad_s[0];
    g_ec_io.imu.gy = reading.sample.gyro_rad_s[1];
    g_ec_io.imu.gz = reading.sample.gyro_rad_s[2];
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
    ec_raw_imu(reading);
    g_ec_io.imu.temp_c = reading.times.temperature_valid
        ? reading.sample.temp_c : std::numeric_limits<float>::quiet_NaN();
#else
    g_ec_io.imu.temp_c = reading.sample.temp_c;
#endif
    g_ec_io.imu.last_ms = reading.sampled_at.ms;
    bump(g_ec_io.imu.samples);
}
} // 板级 IO 内部

extern "C" std::uint32_t ec_board_micros(void) { return TIM5->CNT; }

extern "C" void ec_board_io_init(void) {
    // TIM5 在当前板级未分配, 32 bit 自由运行; 不占用 RTOS/HAL tick 定时器.
    __HAL_RCC_TIM5_CLK_ENABLE();
    const auto pclk = HAL_RCC_GetPCLK1Freq();
    const auto timer_hz = pclk * ((RCC->CFGR & RCC_CFGR_PPRE1) ? 2u : 1u);
    TIM5->CR1 = 0;
    TIM5->PSC = timer_hz / 1000000u - 1u;
    TIM5->ARR = 0xffffffffu;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CNT = 0;
    TIM5->CR1 = TIM_CR1_CEN;
    g_ec_io.magic = 0x45434f49u;
    g_ec_io.abi = 3; // 3: EcRemoteDiag 增加 ch4
#if !EC_SENTINEL_CAPTURE
    ec::sentinel::assembly_init();
#endif
#if EC_SENTINEL_APP
    // DMA 启动前明确设置 APP 的业务波特率; 其它角色仍采用 CubeMX 配置.
    huart1.Init.BaudRate = ec::sentinel::app_uart_baud;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
    // 正式 APP: 调度器启动前完成配置校验与全部对象构造 (运行期不再分配).
    ec::sentinel::ec_app_init();
#endif
    ec::sentinel::input_reset();
    ec_registry_init();
    start_rx(&huart1, usart1_rx.dma_buffer(), static_cast<std::uint16_t>(Pipe1::capacity()));
    start_rx(&huart3, usart3_rx.dma_buffer(), static_cast<std::uint16_t>(Pipe3::capacity()));
    // 所有角色都在 CAN 接收启用前完成传感器阻塞初始化.
    init_imu();
#if EC_SENTINEL_CAPTURE
    ec_capture_init();
#endif
    init_can(&hcan1, 0);
    init_can(&hcan2, 14);
    ec_monitor_init();
}

extern "C" void ec_board_io_poll(uint32_t now_ms) {
#if !EC_SENTINEL_APP
    // 正式 APP 由 1 kHz 控制任务搬运 USART1 (业务协议), 10 ms 任务不重复读.
    poll_uart1();
#else
    // 正式 APP 的 CAN 收帧/IMU 采样/控制周期全部由 1 kHz 控制任务驱动.
    (void)now_ms;
#endif
    poll_uart3();
#if !EC_SENTINEL_CAPTURE && !EC_SENTINEL_APP
    // 正式 APP 由 1 kHz 控制任务按 can_frames_per_cycle 收帧.
    drain_can(now_ms, ec::sentinel::can_frames_per_cycle);
#endif
#if EC_S5_GM6020_BENCH
    // 电机周期为 1 ms; SPI 观测与串口统计仍为 10 ms, 不盲目加倍外设采样负载.
    static std::uint32_t last_observation{};
    if (now_ms-last_observation>=10) {
        last_observation=now_ms;
        poll_imu(now_ms);
        copy_uart_stats();
    }
#else
#if !EC_SENTINEL_APP && !EC_SENTINEL_CAPTURE
    poll_imu(now_ms);
#endif
    copy_uart_stats();
    // 采集角色不调用控制/提交钩子; 正式 APP 由 1 kHz 控制任务调用 assembly_step.
#if !EC_SENTINEL_CAPTURE && !EC_SENTINEL_APP
    ec::sentinel::assembly_step(now_ms);
#endif
#endif
    ec_monitor_poll();
    ec_bench_tick();
    // 监控注册表只做固定字段更新; 串口/SWD 的编码只在被请求时发生.
    // 正式 APP 由 1 kHz 控制任务每 10 个周期发布一次 (同一任务写, 不需要额外同步).
#if !EC_SENTINEL_APP
    ec_registry_publish(now_ms);
#endif
}

extern "C" void ec_board_drain_can(uint32_t now_ms, size_t budget) {
#if !EC_SENTINEL_CAPTURE
    drain_can(now_ms, budget);
#else
    (void)now_ms;
    (void)budget;
#endif
}

extern "C" void ec_board_poll_uart1(void) { poll_uart1(); }

extern "C" void ec_board_poll_imu(uint32_t now_ms) { poll_imu(now_ms); }

extern "C" int ec_board_uart1_try_send(const uint8_t* data, size_t size) {
    if (data == nullptr || size == 0) { return 0; }
    ec::ScopedCritical guard(uart_lock);
    if (!usart1_tx.try_push({data, size})) {
        g_ec_io.usart1.tx_dropped = g_ec_io.usart1.tx_dropped + size;
        return 0;
    }
    kick_tx();
    return 1;
}

extern "C" uint32_t ec_board_uart1_overflows(void) { return usart1_rx.overflows(); }

extern "C" void ec_board_on_uart_irq(void* handle) {
    auto* huart = static_cast<UART_HandleTypeDef*>(handle);
    const auto status = huart->Instance->SR;
    // 先交给 HAL 中止错误接收; 清 IDLE 的 SR/DR 序列也会清掉错误标志.
    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) { return; }
    if (status & USART_SR_IDLE) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        publish_uart(huart, true);
    }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) { publish_uart(huart); }
extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* huart) { publish_uart(huart); }

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART6) { ec_monitor_tx_complete(); return; }
    if (huart->Instance != USART1) { return; }
    usart1_tx.pop(tx_inflight);
    g_ec_io.usart1.tx_bytes += tx_inflight;
    tx_inflight = 0;
    tx_busy = false;
    kick_tx();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART6) { ec_monitor_uart_error(huart->ErrorCode); return; }
    note_uart_error(huart, huart->ErrorCode);
    // TX DMA 错误也会进入此回调, 不得重置仍在运行的 RX 管线.
    if (huart->RxState != HAL_UART_STATE_READY) { return; }
    if (huart->Instance == USART1) {
        start_rx(huart, usart1_rx.dma_buffer(), static_cast<std::uint16_t>(Pipe1::capacity()));
    } else if (huart->Instance == USART3) {
        start_rx(huart, usart3_rx.dma_buffer(), static_cast<std::uint16_t>(Pipe3::capacity()));
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle) {
    CAN_RxHeaderTypeDef header{};
    std::array<std::uint8_t, 8> data{};
    if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, data.data()) != HAL_OK) { return; }
    bump(g_ec_io.can.rx_frames);
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8) {
        bump(g_ec_io.can.ignored);
        return;
    }
    ec::drivers::StampedCanFrame item{};
    item.bus = handle->Instance == CAN1 ? 1 : 2;
    item.received_at = {HAL_GetTick()};
    item.received_us = {ec_board_micros()};
    item.frame.id = static_cast<std::uint16_t>(header.StdId);
    item.frame.size = static_cast<std::uint8_t>(header.DLC);
    item.frame.data = data;
#if EC_SENTINEL_CAPTURE
    ec_capture_receive(item.bus, item.frame);
#else
    if (can_rx.try_push(item) != ec::PushResult::accepted) { bump(g_ec_io.can.rx_dropped); }
#endif
}
