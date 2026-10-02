#pragma once
#include <cmath>

namespace ec::control {
// 电流模式换算基准: 协议满量程 raw 对应的轴侧满量程电流. 只描述协议比例, 不含限幅策略.
// 归一化执行量 ±1 对应"配置的 raw 编码上限"(由组装层一次确定), 不是协议满量程.
struct CurrentScale {
    float amp_full_scale{}; // 协议满量程电流, 必须 > 0. 例: C620 ±16384 = ±20 A.
    float raw_full_scale{}; // 协议满量程 raw, 必须 > 0.
};

[[nodiscard]] inline bool current_scale_valid(CurrentScale scale) noexcept {
    return std::isfinite(scale.amp_full_scale) && scale.amp_full_scale > 0 &&
           std::isfinite(scale.raw_full_scale) && scale.raw_full_scale > 0;
}

// A → 协议 raw. 不检查协议范围, 协议范围由设备编码边界负责.
[[nodiscard]] inline float amps_to_raw(CurrentScale scale, float amps) noexcept {
    return amps * scale.raw_full_scale / scale.amp_full_scale;
}

[[nodiscard]] inline float raw_to_amps(CurrentScale scale, float raw) noexcept {
    return raw * scale.amp_full_scale / scale.raw_full_scale;
}

// 把 A 域增益换算到归一化域. raw_output_limit 是归一化 ±1 对应的 raw 编码上限.
// 例 (GM6020 旧主 yaw 速度环): 满量程 ±16384 = ±3 A, 编码上限 3000,
// Kp = 73 raw/(deg/s) → 0.765855 A/(rad/s) → 1.3941973 normalized/(rad/s).
[[nodiscard]] inline float gain_amp_to_normalized(CurrentScale scale, float raw_output_limit,
                                                 float gain_amp) noexcept {
    return gain_amp * scale.raw_full_scale / (scale.amp_full_scale * raw_output_limit);
}
} // 命名空间 ec::control
