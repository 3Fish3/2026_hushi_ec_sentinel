#include "ec/platform/stm32f407.hpp"
#include "ec/periodic.hpp"

namespace ec::stm32f407 {
static_assert(sizeof(TickType_t) == sizeof(std::uint32_t), "32-bit FreeRTOS ticks required");
static_assert(configTICK_RATE_HZ > 0);
Tick now() noexcept { return Tick{HAL_GetTick()}; }

PeriodicWait::PeriodicWait(TickType_t period_ticks) noexcept
    : period_(period_ticks), previous_(xTaskGetTickCount()) {}

std::optional<std::uint32_t> PeriodicWait::wait_next() noexcept {
    const auto plan = plan_periodic(previous_, xTaskGetTickCount(), period_);
    if (!plan) { return std::nullopt; }
    previous_ = plan->anchor_tick;
    vTaskDelayUntil(&previous_, period_);
    return plan->skipped;
}

bool CycleTimer::enable() noexcept {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return available();
}

bool CycleTimer::available() noexcept {
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0;
}

void CycleTimer::reset() noexcept {
    start_ = DWT->CYCCNT;
}

std::uint32_t CycleTimer::elapsed_cycles() const noexcept {
    return DWT->CYCCNT - start_; // 回绕安全, 只适用于远小于 2^32 周期的时间差
}

std::uint32_t CycleTimer::elapsed_us() const noexcept {
    const auto per_us = SystemCoreClock / 1000000u;
    if (per_us == 0) { return 0; }
    return elapsed_cycles() / per_us;
}

#if defined(HAL_CAN_MODULE_ENABLED)
transport::SendResult HalCanTx::try_send(const transport::CanFrame& frame) noexcept {
    using transport::SendResult;
    if (!frame.valid()) { return SendResult::invalid; }
    if (handle_.Instance == nullptr || HAL_CAN_GetState(&handle_) != HAL_CAN_STATE_LISTENING) {
        return SendResult::error; // 不在未启动或停止状态缓存可能过期的控制帧.
    }
    // 该句柄仅由一个任务发送, 不允许其他任务或 ISR 并发调用 HAL 发送接口.
    if (HAL_CAN_GetTxMailboxesFreeLevel(&handle_) == 0) { return SendResult::busy; }
    CAN_TxHeaderTypeDef header{};
    header.StdId = frame.id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame.size;
    header.TransmitGlobalTime = DISABLE;
    auto bytes = frame.data; // HAL 接口参数非 const, 提供独立的有界副本.
    std::uint32_t mailbox{};
    switch (HAL_CAN_AddTxMessage(&handle_, &header, bytes.data(), &mailbox)) {
    case HAL_OK: return SendResult::accepted;
    case HAL_BUSY: return SendResult::busy;
    default: return SendResult::error;
    }
}
#endif
} // 命名空间 ec::stm32f407
