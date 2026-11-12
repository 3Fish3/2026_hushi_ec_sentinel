#pragma once
#include "ec/control/pid.hpp"
#include "ec/time.hpp"
#include <cstdint>

namespace ec::control {
// IMU 温控计算组件: 温度 → 占空比. 只算输出, PWM 由平台层写到具体定时器;
// 默认关闭, 关闭或温度失效时输出 0, 不做任何"HAL 输出默认接通".
// 参数按旧仓库 TIM10 (ARR=4999, 预分频 167, 约 50 Hz 级 PWM) 的尺度换算:
// 旧的 MaxOutput 2000 → 0.4 占空比, IntegralLimit 300 → 0.06, Kp 1000/°C → 0.2/°C,
// Ki 20 → 0.004/°C/s. 目标 40 °C 来自旧配置 RefTemp, 不是实测最优温度.
struct ThermalConfig {
    float target_c{40.0f};
    float deadband_c{0.1f};   // 旧 PID Deadband: 误差在带内时不重算输出
    float kp{0.2f};
    float ki{0.004f};
    float kd{0.0f};
    float duty_limit{0.4f};
    float integral_limit{0.06f};
    Lifetime temperature_ttl{500};
};

class ThermalController {
public:
    enum class Status : std::uint8_t {
        off, active, temperature_stale, invalid_input, invalid_dt,
    };
    explicit ThermalController(const ThermalConfig& config) noexcept;
    // 使能状态变化时清控制器历史, 避免关闭期间积累的积分在重新使能瞬间释放.
    void set_enabled(bool enabled) noexcept;
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }
    [[nodiscard]] const ThermalConfig& config() const noexcept { return config_; }
    // 温度、采集时刻与当前时刻由调用方给出; 返回占空比, 取值为 [0, duty_limit].
    // 温度超时、非法输入或非法 dt 时输出 0, 而不是沿用上一次输出.
    [[nodiscard]] float update(float temperature_c, Tick temperature_at, Tick now, float dt_s) noexcept;
    [[nodiscard]] Status status() const noexcept { return status_; }
    [[nodiscard]] float duty() const noexcept { return duty_; }
private:
    ThermalConfig config_;
    Pid pid_;
    float duty_{};
    bool enabled_{};
    bool has_previous_{};
    Status status_{Status::off};
};
} // 命名空间 ec::control
