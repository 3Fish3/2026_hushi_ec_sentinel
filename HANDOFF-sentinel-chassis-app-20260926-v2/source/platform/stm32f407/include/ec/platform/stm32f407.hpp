#pragma once
#include "ec/time.hpp"
#include "ec/transport/can.hpp"
#include <cstdint>
#include <optional>
extern "C" {
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
}

namespace ec::stm32f407 {
// 仅支持单核 Cortex-M4. 恢复原 PRIMASK, 可用于普通 ISR 和任务, 不适用于 NMI/HardFault.
// 临界区内不能调用 RTOS、HAL 或用户回调; 仅供有界通道复制使用.
class IrqCriticalSection {
public:
    [[nodiscard]] std::uint32_t enter() noexcept {
        const auto saved = __get_PRIMASK();
        __disable_irq();
        __DMB();
        return saved;
    }
    void exit(std::uint32_t saved) noexcept {
        __DMB();
        __set_PRIMASK(saved);
    }
};
// HAL 时基必须由板级代码维持单调毫秒计数, 不能以会暂停的时基实现保护计时.
[[nodiscard]] Tick now() noexcept;

// 在任务入口创建, 使用原生 RTOS tick 而不是硬编码毫秒频率; 模块不自行创建任务.
class PeriodicWait {
public:
    explicit PeriodicWait(TickType_t period_ticks) noexcept;
    // 返回跳过的旧周期数量; 非法周期或超出半回绕观察范围时返回空值.
    // 任务超期时跳过积压周期, 不连续补跑旧控制目标. 禁止从 ISR 调用.
    [[nodiscard]] std::optional<std::uint32_t> wait_next() noexcept;
private:
    TickType_t period_;
    TickType_t previous_;
};

// DWT 周期计数: 只用于同一测量窗口内的短时间差(例如一次控制周期的执行耗时),
// 不是跨机同步时钟, 也不用于长期计时. 差值按 2^32 周期回绕处理, 因此测量窗口
// 必须远小于一个回绕周期 (168 MHz 下约 25.6 s); 跨窗口的差值无意义.
// 本机只有单核 Cortex-M4, 不涉及多核校准.
class CycleTimer {
public:
    // 使能 DWT CYCCNT; 返回 false 表示本环境不支持, 调用方不得改用其他计时源冒充.
    [[nodiscard]] static bool enable() noexcept;
    [[nodiscard]] static bool available() noexcept;
    void reset() noexcept;
    [[nodiscard]] std::uint32_t elapsed_cycles() const noexcept;
    // 用 SystemCoreClock 换算到微秒; 不足 1 us 的差值会得到 0, 不四舍五入.
    [[nodiscard]] std::uint32_t elapsed_us() const noexcept;
private:
    std::uint32_t start_{};
};

#if defined(HAL_CAN_MODULE_ENABLED)
class HalCanTx final : public transport::CanTx {
public:
    explicit HalCanTx(CAN_HandleTypeDef& handle) noexcept : handle_(handle) {}
    [[nodiscard]] transport::SendResult try_send(const transport::CanFrame& frame) noexcept override;
private:
    CAN_HandleTypeDef& handle_;
};
#endif
} // 命名空间 ec::stm32f407
