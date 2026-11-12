#pragma once
#include <cmath>
#include <cstdint>
#include <numbers>

namespace ec::control {
// 角度归一化到 (-pi, pi]. 只做数值归一, 不隐含路径选择, 也不改变累计圈数信息.
// 边界采用 float 表示的 ±pi: float 的 pi 略大于双精度 pi, 混用会在该边界上翻到负侧.
// 端点 (±pi) 允许 1 ulp 级差异, 不改变后续控制语义.
[[nodiscard]] inline float wrap_to_pi(float rad) noexcept {
    constexpr float tau = 2.0f * std::numbers::pi_v<float>;
    const float wrapped = std::fmod(rad, tau);
    // 非有限输入经 fmod 后保持非有限, 不在此处伪造有效角度.
    if (wrapped > std::numbers::pi_v<float>) { return wrapped - tau; }
    if (wrapped <= -std::numbers::pi_v<float>) { return wrapped + tau; }
    return wrapped;
}

// 单圈最短弧误差, 结果落在 (-pi, pi]. 由调用方显式使用, 不由框架自动施加到所有关节.
[[nodiscard]] inline float shortest_arc_error(float target_rad, float measured_rad) noexcept {
    return wrap_to_pi(target_rad - measured_rad);
}

// continuous 直接作差, 保留多圈累计语义; shortest 只在单圈内取最短路径.
enum class ArcMode : std::uint8_t { continuous, shortest };

[[nodiscard]] inline float position_error(ArcMode mode, float target_rad, float measured_rad) noexcept {
    return mode == ArcMode::shortest ? shortest_arc_error(target_rad, measured_rad)
                                     : target_rad - measured_rad;
}
} // 命名空间 ec::control
