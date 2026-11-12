#include "ec/control/thermal.hpp"
#include <cmath>

namespace ec::control {
ThermalController::ThermalController(const ThermalConfig& config) noexcept
    : config_(config),
      pid_({config.kp, config.ki, config.kd, config.integral_limit, 0.0f, config.duty_limit}) {}

void ThermalController::set_enabled(bool enabled) noexcept {
    if (enabled_ == enabled) { return; }
    enabled_ = enabled;
    // 使能变化即重置: 关闭期间不保留积分, 重新使能从零开始.
    pid_.reset();
    has_previous_ = false;
    duty_ = 0.0f;
    status_ = enabled ? Status::active : Status::off;
}

float ThermalController::update(float temperature_c, Tick temperature_at, Tick now, float dt_s) noexcept {
    if (!enabled_) {
        duty_ = 0.0f;
        status_ = Status::off;
        return duty_;
    }
    if (!std::isfinite(temperature_c)) {
        duty_ = 0.0f;
        status_ = Status::invalid_input;
        return duty_;
    }
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) {
        duty_ = 0.0f;
        status_ = Status::invalid_dt;
        return duty_;
    }
    if (!fresh(now, temperature_at, config_.temperature_ttl)) {
        duty_ = 0.0f;
        status_ = Status::temperature_stale;
        return duty_;
    }
    const float error = config_.target_c - temperature_c;
    // 旧 PID 在死区内不重算, 因此这里保持上一次输出; 把 deadband_c 设为 0 可关闭该行为.
    if (std::fabs(error) <= config_.deadband_c && has_previous_) {
        status_ = Status::active;
        return duty_;
    }
    const auto output = pid_.step(config_.target_c, temperature_c, dt_s);
    if (!output) {
        duty_ = 0.0f;
        status_ = Status::invalid_input;
        return duty_;
    }
    duty_ = *output;
    has_previous_ = true;
    status_ = Status::active;
    return duty_;
}
} // 命名空间 ec::control
