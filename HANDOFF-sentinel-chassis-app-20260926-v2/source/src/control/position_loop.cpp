#include "ec/control/position_loop.hpp"
#include <cmath>
#include <algorithm>

namespace ec::control {
PositionLoop::PositionLoop(PositionLoopConfig config) noexcept
    : config_(config), position_pid_(config.position_pid), velocity_(config.velocity) {
    // 外环限幅必须落在内环目标范围内, 否则位置环会生成内环必然拒绝的目标.
    configured_ = velocity_.configured() && position_pid_.configured() &&
        config.position_pid.output_min >= -config.velocity.max_target_rad_s &&
        config.position_pid.output_max <= config.velocity.max_target_rad_s;
}

std::optional<Grant> PositionLoop::arm(CommandSource source, bool authorized, bool ready) noexcept {
    position_pid_.reset();
    held_velocity_.reset();
    return velocity_.arm(source, authorized, ready && configured_);
}

void PositionLoop::revoke() noexcept {
    position_pid_.reset();
    held_velocity_.reset();
    velocity_.revoke();
}

void PositionLoop::reset_controller() noexcept {
    position_pid_.reset();
    held_velocity_.reset();
    velocity_.reset_controller();
}

PositionResult PositionLoop::reject(PositionStatus status) noexcept {
    revoke();
    return {status, std::nullopt, std::nullopt, std::nullopt};
}

PositionResult PositionLoop::step(Tick now, float dt_s,
    const std::optional<Command<float>>& target,
    const std::optional<Sample<float>>& position,
    const std::optional<Sample<float>>& velocity) noexcept {
    return step_split(now, dt_s, dt_s, target, position, velocity, 1.0f);
}

PositionResult PositionLoop::step_split(Tick now, float dt_s, std::optional<float> outer_dt_s,
    const std::optional<Command<float>>& target, const std::optional<Sample<float>>& position,
    const std::optional<Sample<float>>& velocity, float rate_per_axis) noexcept {
    if (!configured_) { return reject(PositionStatus::invalid_config); }
    if (!target) { return reject(PositionStatus::no_command); }
    if (!fresh(now, target->received_at, target->ttl)) { return reject(PositionStatus::stale_command); }
    if (!position || position->validity != Validity::valid) { return reject(PositionStatus::no_feedback); }
    if (!position->fresh_at(now, config_.velocity.feedback_ttl)) {
        return reject(PositionStatus::stale_feedback);
    }
    if (!std::isfinite(target->value) || !std::isfinite(position->value)) {
        return reject(PositionStatus::invalid_target);
    }
    // 内环步长由环路检验, 外环的最大真实步长由分频调度者控制; 空值只是保持外环输出.
    if (!std::isfinite(dt_s) || dt_s <= 0 || dt_s > config_.velocity.max_dt_s ||
        (outer_dt_s && (!std::isfinite(*outer_dt_s) || *outer_dt_s <= 0))) {
        return reject(PositionStatus::invalid_dt);
    }
    if (!std::isfinite(rate_per_axis) || rate_per_axis == 0) { return reject(PositionStatus::invalid_target); }
    if (outer_dt_s) {
        const float error = position_error(config_.arc, target->value, position->value);
        if (!std::isfinite(error)) { return reject(PositionStatus::invalid_target); }
        held_velocity_ = position_pid_.step(position->value + error, position->value, *outer_dt_s);
    }
    if (!held_velocity_) { return reject(PositionStatus::controller_error); }
    const std::optional<float> velocity_target{std::clamp(*held_velocity_/rate_per_axis,
        -velocity_.max_target(), velocity_.max_target())};
    auto mapped_velocity = velocity;
    if (mapped_velocity) { mapped_velocity->value /= rate_per_axis; }
    // 内环继承外环命令的时间与有效期, 不用 now 刷新旧意图期限.
    const Command<float> inner{*velocity_target, target->received_at, target->ttl, target->grant};
    const auto inner_result = velocity_.step(now, dt_s, inner, mapped_velocity);
    if (!inner_result.output) {
        // 内环已撤销授权并清自己的历史; 外环只清自己的控制器, 不重复撤销.
        position_pid_.reset();
        held_velocity_.reset();
        return {PositionStatus::inner_rejected, std::nullopt, velocity_target, inner_result.status};
    }
    return {PositionStatus::active, inner_result.output, velocity_target, inner_result.status};
}
} // 命名空间 ec::control
