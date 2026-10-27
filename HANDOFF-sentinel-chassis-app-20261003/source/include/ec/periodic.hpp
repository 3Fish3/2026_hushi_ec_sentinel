#pragma once
#include <cstdint>
#include <optional>

namespace ec {
struct PeriodicPlan {
    std::uint32_t anchor_tick{}; // 更新后的相位基点; 下一截止点为 anchor_tick + period.
    std::uint32_t skipped{};
};
// 参数统一使用调用者时钟的原生 tick, 不混用 HAL 毫秒和 FreeRTOS tick.
// 当前时刻正好到截止点时允许立即执行; 已经错过的周期不积压补跑.
// 和 freshness 一样, 要求两次观察相隔不足半个回绕周期.
[[nodiscard]] constexpr std::optional<PeriodicPlan> plan_periodic(
    std::uint32_t previous, std::uint32_t now, std::uint32_t period) noexcept {
    if (period == 0 || period >= 0x80000000u) { return std::nullopt; }
    const std::uint32_t elapsed = now - previous;
    if (elapsed >= 0x80000000u) { return std::nullopt; }
    const std::uint32_t skipped = elapsed > period ? (elapsed - 1) / period : 0;
    return PeriodicPlan{previous + skipped * period, skipped};
}
} // 命名空间 ec
