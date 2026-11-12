#include "ec/control/pid.hpp"
#include <algorithm>
#include <cmath>

namespace ec::control {
Pid::Pid(PidConfig config) noexcept : config_(config) {
    configured_ = std::isfinite(config.kp) && std::isfinite(config.ki) &&
        std::isfinite(config.kd) && std::isfinite(config.integral_limit) &&
        std::isfinite(config.output_min) && std::isfinite(config.output_max) &&
        config.integral_limit >= 0.0f && config.output_min < config.output_max &&
        std::isfinite(config.derivative_tau_s) && config.derivative_tau_s >= 0.0f;
}

bool Pid::set_config(const PidConfig& config) noexcept {
    const bool ok = std::isfinite(config.kp) && std::isfinite(config.ki) &&
        std::isfinite(config.kd) && std::isfinite(config.integral_limit) &&
        std::isfinite(config.output_min) && std::isfinite(config.output_max) &&
        config.integral_limit >= 0.0f && config.output_min < config.output_max &&
        std::isfinite(config.derivative_tau_s) && config.derivative_tau_s >= 0.0f;
    if (!ok) { return false; }
    config_ = config;
    configured_ = true;
    return true;
}

void Pid::reset() noexcept {
    diagnostics_ = {};
    integral_ = 0.0f;
    previous_measured_ = 0.0f;
    derivative_rate_ = 0.0f;
    has_previous_ = false;
}

std::optional<float> Pid::step(float target, float measured, float dt_s) noexcept {
    if (!configured_ || !std::isfinite(target) || !std::isfinite(measured) ||
        !std::isfinite(dt_s) || dt_s <= 0.0f) {
        reset();
        return std::nullopt;
    }
    const float error = target - measured;
    const float candidate = integral_ + config_.ki * error * dt_s;
    float derivative{};
    if (has_previous_ && config_.kd != 0.0f) {
        if (config_.derivative_tau_s > 0.0f) {
            // 后向Euler: 用实际dt更新微分低通, P/I仍直接使用本次measured.
            const float weight = dt_s / (config_.derivative_tau_s + dt_s);
            derivative_rate_ = (1.0f-weight)*derivative_rate_ +
                (measured-previous_measured_)/(config_.derivative_tau_s+dt_s);
            derivative = -config_.kd*derivative_rate_;
        } else {
            // 保持关闭滤波时既有浮点运算顺序和数值行为.
            derivative = -config_.kd*(measured-previous_measured_)/dt_s;
            derivative_rate_ = 0.0f;
        }
    } else { derivative_rate_ = 0.0f; }
    if (!std::isfinite(error) || !std::isfinite(candidate) || !std::isfinite(derivative)) {
        reset();
        return std::nullopt;
    }
    const float bounded = std::clamp(candidate, -config_.integral_limit, config_.integral_limit);
    const float proportional = config_.kp * error;
    const float output = proportional + bounded + derivative;
    if (!std::isfinite(output)) {
        reset();
        return std::nullopt;
    }
    integral_ = bounded;
    previous_measured_ = measured;
    has_previous_ = true;
    const float limited = std::clamp(output, config_.output_min, config_.output_max);
    diagnostics_ = {target, measured, proportional, bounded, derivative, output, limited, true};
    return limited;
}
} // 命名空间 ec::control
