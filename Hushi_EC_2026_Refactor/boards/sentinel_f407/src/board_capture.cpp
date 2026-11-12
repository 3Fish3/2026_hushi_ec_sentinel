#include "board_capture.hpp"
#include "capture.hpp"
#include "board_io.h"
#include "board_raw.hpp"
#include "s1_diagnostics.h"
#include "ec/channel.hpp"
#include "ec/platform/stm32f407.hpp"
#include "can.h"
#include <bit>

static_assert(EC_SENTINEL_CAPTURE && !EC_S5_GM6020_BENCH);
extern "C" { extern const volatile std::uint32_t g_ec_capture_signature = 0x43415031; }
namespace {
namespace cap = ec::sentinel::capture;
struct Received {
    ec::transport::CanFrame frame{};
    std::uint32_t ms{}, cycles{}, us{};
    std::uint8_t bus{};
};
ec::stm32f407::IrqCriticalSection lock{};
ec::EventQueue<Received, 128, ec::stm32f407::IrqCriticalSection> queue{lock};
cap::Collector collector{};
std::uint32_t queued{}, high_water{}, dropped{}, overrun[2]{}, errors[2]{};
std::uint32_t max_age{}, max_gap{}, max_cost{}, stack_words{};
// 只提供固定参数/状态读取能力; 不借用可接任意控制帧的 HalCanTx.
static_assert(cap::dm_bus == 1 || cap::dm_bus == 2, "DM bus must be CAN1 or CAN2");
CAN_HandleTypeDef* dm_can_handle() noexcept { return cap::dm_bus == 1 ? &hcan1 : &hcan2; }
class ReadOnlyTx final : public ec::transport::CanTx {
public:
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        using ec::transport::SendResult;
        if (!cap::allowed_read(frame)) { return SendResult::invalid; }
        auto* handle = dm_can_handle();
        if (HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0) { return SendResult::busy; }
        CAN_TxHeaderTypeDef header{};
        header.StdId = 0x7ff; header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 4;
        auto data = frame.data;
        std::uint32_t mailbox{};
        const auto result = HAL_CAN_AddTxMessage(handle, &header, data.data(), &mailbox);
        return result == HAL_OK ? SendResult::accepted : result == HAL_BUSY ? SendResult::busy : SendResult::error;
    }
};
ReadOnlyTx tx{};
void capture_task(void*) {
    ec::stm32f407::PeriodicWait wait(pdMS_TO_TICKS(1));
    auto last = HAL_GetTick();
    auto last_stack = last;
    for (;;) {
        if (!wait.wait_next()) { ec_s1_fail(EC_S1_PERIOD_ERROR, __FILE__, __LINE__); }
        const auto now = HAL_GetTick();
        const auto start = DWT->CYCCNT;
        if (now - last > max_gap) { max_gap = now - last; }
        last = now;
        // 快任务独占解码、查询和有界 IMU 读取; 不执行串口/文件操作.
        // 128 是 CAN 每周期明确上界, SPI 不在其它任务重复读取.
        for (unsigned budget = 0; budget < 128; ++budget) {
            std::optional<Received> item;
            {
                ec::ScopedCritical guard(lock);
                item = queue.try_pop();
                if (item) { --queued; }
            }
            if (!item) { break; }
            const auto age = HAL_GetTick() - item->ms;
            if (age > max_age) { max_age = age; }
            collector.receive(item->bus, item->frame, item->ms, item->cycles);
            ec_raw_can(item->bus, item->frame, {item->us}, dropped);
        }
        collector.poll(tx, HAL_GetTick(), ec_raw_fast_dm());
        ec_board_poll_imu(HAL_GetTick());
        const auto cost = DWT->CYCCNT - start;
        if (cost > max_cost) { max_cost = cost; }
        if (now - last_stack >= 1000) {
            stack_words = uxTaskGetStackHighWaterMark(nullptr);
            last_stack = now;
        }
    }
}
} // 板级采集内部

void ec_capture_init() noexcept {
    if (!ec::stm32f407::CycleTimer::enable()) { Error_Handler(); }
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 6, 0); HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 6, 0); HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
}
void ec_capture_start_task() noexcept {
    if (xTaskCreate(capture_task, "captureRx", 512, nullptr, uxTaskPriorityGet(nullptr) + 1, nullptr) != pdPASS) {
        ec_s1_fail(EC_S1_TASK_CREATE, __FILE__, __LINE__);
    }
}
void ec_capture_receive(std::uint8_t bus, const ec::transport::CanFrame& frame) noexcept {
    const Received item{frame, HAL_GetTick(), DWT->CYCCNT, ec_board_micros(), bus};
    ec::ScopedCritical guard(lock);
    if (queue.try_push(item) == ec::PushResult::accepted) {
        ++queued;
        if (queued > high_water) { high_water = queued; }
    } else {
        ++dropped;
        g_ec_io.can.rx_dropped = g_ec_io.can.rx_dropped + 1;
    }
}
extern "C" void CAN1_SCE_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
extern "C" void CAN2_SCE_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan2); }
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* handle) {
    const auto index = handle->Instance == CAN1 ? 0u : 1u;
    if (handle->ErrorCode & HAL_CAN_ERROR_RX_FOV0) { ++overrun[index]; }
    if (handle->ErrorCode & ~HAL_CAN_ERROR_RX_FOV0) { ++errors[index]; }
}
std::size_t ec_capture_snapshot(std::uint8_t page, std::span<std::uint8_t> out) noexcept {
    if (page >= cap::page_count) { return 0; }
    cap::Writer w(out);
    if (page >= 1 && page <= 5) {
        const auto start = static_cast<std::size_t>((page - 1) * 2);
        std::array<cap::Motor, 2> copy{};
        std::uint32_t now{};
        {
            ec::ScopedCritical guard(lock);
            copy = {collector.motors()[start], collector.motors()[start + 1]};
            now = uwTick;
        }
        w.header(page, 2, now);
        for (std::size_t i = 0; i < copy.size(); ++i) { cap::encode_motor(w, cap::installs[start + i], copy[i]); }
    } else if (page >= 7) {
        const auto start = static_cast<std::size_t>((page - 7) * 4);
        std::array<cap::Register, 4> copy{};
        std::uint32_t now{};
        {
            ec::ScopedCritical guard(lock);
            for (std::size_t i = 0; i < copy.size(); ++i) { copy[i] = collector.values()[start + i]; }
            now = uwTick;
        }
        w.header(page, 4, now);
        for (std::size_t i = 0; i < copy.size(); ++i) { cap::encode_register(w, cap::registers[start + i], copy[i]); }
    } else if (page == 0) {
        std::array<std::uint32_t, 24> values{};
        std::uint32_t now{};
        {
            ec::ScopedCritical guard(lock);
            const auto& d = collector.diagnostics();
            values = {g_ec_capture_signature, SystemCoreClock, g_ec_s1.reset_flags,
                g_ec_io.can.rx_frames, dropped, g_ec_io.can.ignored,
                overrun[0], overrun[1], errors[0], errors[1], high_water, max_age, max_gap, max_cost,
                d.queries, d.tx_busy, d.tx_error, d.timeouts, d.unsolicited, d.unknown, d.malformed, stack_words,
                d.unknown_bus, d.unknown_id};
            now = uwTick;
        }
        w.header(page, 1, now);
        for (auto value : values) { w.u32(value); }
        bool ended = false;
        for (std::size_t i = 0; i < 32; ++i) {
            const auto c = ended ? 0 : static_cast<std::uint8_t>(g_ec_s1_build_id[i]);
            w.u8(c); ended = ended || c == 0;
        }
    } else {
        // IMU 来自快任务, DR16 来自 ISR; 锁定固定字段复制/编码以保持一致.
        ec::ScopedCritical guard(lock);
        w.header(page, 1, uwTick);
        const auto& r = g_ec_io.remote;
        w.u32(r.frames); w.u32(r.rejected); w.u32(r.last_ms);
        w.u16(static_cast<std::uint16_t>(r.ch0)); w.u16(static_cast<std::uint16_t>(r.ch1));
        w.u16(static_cast<std::uint16_t>(r.ch2)); w.u16(static_cast<std::uint16_t>(r.ch3));
        w.u16(static_cast<std::uint16_t>(r.ch4));
        w.u16(static_cast<std::uint16_t>(r.mouse_x)); w.u16(static_cast<std::uint16_t>(r.mouse_y));
        w.u16(static_cast<std::uint16_t>(r.mouse_z)); w.u16(r.keys);
        w.u8(r.switch_left); w.u8(r.switch_right); w.u8(r.online); w.u8(r.mouse_left); w.u8(r.mouse_right);
        const auto& m = g_ec_io.imu;
        w.u32(m.samples); w.u32(m.errors); w.u32(m.last_ms); w.u8(m.ready); w.u8(m.heater_enabled);
        for (float value : {m.ax, m.ay, m.az, m.gx, m.gy, m.gz, m.temp_c}) { w.u32(std::bit_cast<std::uint32_t>(value)); }
        w.u32(g_ec_io.usart3.framing); w.u32(g_ec_io.usart3.noise); w.u32(g_ec_io.usart3.overrun);
        w.u32(g_ec_io.usart3.overflows);
    }
    return w.size();
}
