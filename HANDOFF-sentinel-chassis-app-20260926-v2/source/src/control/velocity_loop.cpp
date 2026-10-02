#include "ec/control/velocity_loop.hpp"
#include <cmath>

namespace ec::control {
VelocityLoop::VelocityLoop(VelocityLoopConfig config) noexcept : config_(config), pid_(config.pid) {
    configured_ = pid_.configured() && config.feedback_ttl.valid() &&
        std::isfinite(config.max_target_rad_s) && config.max_target_rad_s > 0 &&
        std::isfinite(config.max_dt_s) && config.max_dt_s > 0 &&
        std::isfinite(config.feedback_alpha) && config.feedback_alpha >= 0 &&
        config.feedback_alpha < 1 &&
        config.pid.output_min >= -1 && config.pid.output_max <= 1;
}

std::optional<Grant> VelocityLoop::arm(CommandSource source, bool authorized, bool ready) noexcept {
    pid_.reset();
    filter_primed_ = false;
    // ready 必须来自应用的本地自检, 不应直接采用对端声明.
    return gate_.arm(source, authorized, ready && configured_);
}

void VelocityLoop::revoke() noexcept {
    gate_.invalidate();
    reset_controller();
}

void VelocityLoop::reset_controller() noexcept {
    pid_.reset();
    filter_primed_ = false;
}

VelocityResult VelocityLoop::reject(VelocityStatus status) noexcept {
    revoke();
    return {status, std::nullopt};
}

VelocityResult VelocityLoop::step(Tick now, float dt_s,
    const std::optional<Command<float>>& target,
    const std::optional<Sample<float>>& feedback) noexcept {
    if (!configured_) { return reject(VelocityStatus::invalid_config); }
    if (!target) { return reject(VelocityStatus::no_command); }
    if (!fresh(now, target->received_at, target->ttl)) { return reject(VelocityStatus::stale_command); }
    if (!gate_.accepts(target->grant)) { return reject(VelocityStatus::unauthorized); }
    if (!feedback || feedback->validity != Validity::valid) { return reject(VelocityStatus::no_feedback); }
    if (!feedback->fresh_at(now, config_.feedback_ttl)) { return reject(VelocityStatus::stale_feedback); }
    if (!std::isfinite(target->value) || std::fabs(target->value) > config_.max_target_rad_s) {
        return reject(VelocityStatus::invalid_target);
    }
    if (!std::isfinite(dt_s) || dt_s <= 0 || dt_s > config_.max_dt_s) {
        return reject(VelocityStatus::invalid_dt);
    }
    // 非有限测量不由这里单独拦截: Pid 会拒绝并返回空, 由 controller_error 统一上报.
    if (config_.feedback_alpha > 0.0f) {
        // 首次样本直接采用测量值, 不用伪造的零历史污染闭环.
        filtered_feedback_ = filter_primed_
            ? config_.feedback_alpha * filtered_feedback_ + (1.0f - config_.feedback_alpha) * feedback->value
            : feedback->value;
        filter_primed_ = true;
    }
    const float measured = config_.feedback_alpha > 0.0f ? filtered_feedback_ : feedback->value;
    const auto output = pid_.step(target->value, measured, dt_s);
    if (!output) { return reject(VelocityStatus::controller_error); }
    return {VelocityStatus::active, output};
}
} // 命名空间 ec::control
