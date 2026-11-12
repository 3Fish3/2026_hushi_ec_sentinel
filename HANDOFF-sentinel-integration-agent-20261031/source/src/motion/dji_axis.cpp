#include "ec/motion/dji_axis.hpp"
#include <algorithm>
#include <cmath>

namespace ec::motion {
namespace {
bool finite_positive(float value) noexcept {
    return std::isfinite(value) && value > 0.0f;
}

// 环路的拒绝原因显式映射到轴级枚举; 不用枚举序号互转, 避免两侧新增取值后静默错位.
AxisStatus axis_status(control::VelocityStatus status) noexcept {
    using control::VelocityStatus;
    switch (status) {
    case VelocityStatus::active: return AxisStatus::active;
    case VelocityStatus::invalid_config: return AxisStatus::invalid_config;
    case VelocityStatus::no_command: return AxisStatus::no_target;
    case VelocityStatus::stale_command: return AxisStatus::stale_target;
    case VelocityStatus::unauthorized: return AxisStatus::unauthorized;
    case VelocityStatus::no_feedback: return AxisStatus::no_feedback;
    case VelocityStatus::stale_feedback: return AxisStatus::stale_feedback;
    case VelocityStatus::invalid_target: return AxisStatus::invalid_target;
    case VelocityStatus::invalid_dt: return AxisStatus::invalid_dt;
    case VelocityStatus::controller_error: return AxisStatus::controller_error;
    }
    return AxisStatus::controller_error;
}

AxisStatus axis_status(control::PositionStatus status) noexcept {
    using control::PositionStatus;
    switch (status) {
    case PositionStatus::active: return AxisStatus::active;
    case PositionStatus::invalid_config: return AxisStatus::invalid_config;
    case PositionStatus::no_command: return AxisStatus::no_target;
    case PositionStatus::stale_command: return AxisStatus::stale_target;
    case PositionStatus::unauthorized: return AxisStatus::unauthorized;
    case PositionStatus::no_feedback: return AxisStatus::no_feedback;
    case PositionStatus::stale_feedback: return AxisStatus::stale_feedback;
    case PositionStatus::invalid_target: return AxisStatus::invalid_target;
    case PositionStatus::invalid_dt: return AxisStatus::invalid_dt;
    case PositionStatus::controller_error: return AxisStatus::controller_error;
    case PositionStatus::inner_rejected: return AxisStatus::controller_error; // 由内环状态决定
    }
    return AxisStatus::controller_error;
}
} // 内部辅助函数

DjiAxis::DjiAxis(DjiAxisConfig config, drivers::DjiDevice& device) noexcept
    : config_(config), device_(device),
      velocity_{{config.velocity_pid, device.config().feedback_ttl, config.max_velocity_rad_s,
                 config.max_dt_s, config.feedback_alpha}},
      position_{{config.position_pid,
                 {config.velocity_pid, device.config().feedback_ttl, config.max_velocity_rad_s,
                  config.max_dt_s, config.feedback_alpha},
                 config.arc}} {
    const auto mode_full_scale = drivers::dji_current_amp_full_scale(device.config().mode);
    const auto protocol = drivers::dji_protocol(device.config().mode);
    const auto raw_full_scale = protocol ? static_cast<float>(protocol->limit) : 0.0f;
    configured_ = device.configured() && mode_full_scale.has_value() && protocol.has_value() &&
        finite_positive(config.current_limit_a) && config.command_ttl.valid() && velocity_.configured() &&
        device.config().output_limit_raw > 0 &&
        static_cast<float>(device.config().output_limit_raw) <= raw_full_scale;
    if (!configured_) { return; }
    // 归一化基准用设备实际使用的 raw 编码上限, 不假定它等于协议满量程.
    amp_full_scale_ =
        *mode_full_scale * static_cast<float>(device.config().output_limit_raw) / raw_full_scale;
    current_ratio_ = config.current_limit_a / amp_full_scale_;
    // 编码上限必须能表示轴侧电流上限, 否则归一化输出会越过设备编码边界.
    if (!finite_positive(amp_full_scale_) || current_ratio_ > 1.0f) {
        configured_ = false;
    }
}

std::optional<Grant> DjiAxis::enable(control::ControlMode mode,
    CommandSource source, bool authorized, bool ready) noexcept {
    disable();
    if (!configured_ || !ready) { return std::nullopt; }
    std::optional<Grant> grant;
    switch (mode) {
    case control::ControlMode::current:
        // 开环电流模式不需要闭环, 但仍需本地授权门; 复用速度环持有的门, 不另建一份授权状态.
        grant = velocity_.arm(source, authorized, true);
        break;
    case control::ControlMode::velocity:
        grant = velocity_.arm(source, authorized, true);
        break;
    case control::ControlMode::position:
        // 位置环内部持有自己的速度环, 它的门就是位置模式的唯一授权门.
        if (!position_.configured()) { return std::nullopt; }
        grant = position_.arm(source, authorized, true);
        break;
    default:
        return std::nullopt;
    }
    if (!grant) { return std::nullopt; }
    grant_ = grant;
    mode_ = mode;
    enabled_ = true;
    return grant;
}

void DjiAxis::disable() noexcept {
    velocity_.revoke();
    position_.revoke(); // 位置环内部的门与控制器一起撤销
    enabled_ = false;
    grant_.reset();
    target_.reset();
    output_.reset();
    status_ = AxisStatus::idle;
}

bool DjiAxis::set_current(float amps, Tick now) noexcept {
    if (!enabled_ || mode_ != control::ControlMode::current) { return false; }
    if (!std::isfinite(amps) || std::fabs(amps) > config_.current_limit_a) { return false; }
    target_ = Sample<float>{amps, now, Validity::valid};
    return true;
}

bool DjiAxis::set_velocity(float rad_s, Tick now) noexcept {
    if (!enabled_ || mode_ != control::ControlMode::velocity) { return false; }
    if (!std::isfinite(rad_s) || (config_.max_velocity_rad_s > 0 && std::fabs(rad_s) > config_.max_velocity_rad_s)) { return false; }
    target_ = Sample<float>{rad_s, now, Validity::valid};
    return true;
}

bool DjiAxis::set_position(float rad, Tick now) noexcept {
    // 位置范围属于机构限位, 由上层提供; 轴只拒绝非有限值.
    if (!enabled_ || mode_ != control::ControlMode::position || !std::isfinite(rad)) { return false; }
    target_ = Sample<float>{rad, now, Validity::valid};
    return true;
}

bool DjiAxis::reference_position(float axis_rad, Tick now) noexcept {
    if (!device_.reference_position(axis_rad, now)) { return false; }
    // 测量跳变会经 D 项放大, 重新建立参考后必须清控制器历史.
    velocity_.reset_controller();
    position_.reset_controller();
    return true;
}

DjiAxisResult DjiAxis::fail(AxisStatus status) noexcept {
    if (status != AxisStatus::idle) { disable(); } // 任一拒绝都退出使能, 旧目标与旧授权同时失效.
    status_ = status;
    return {status, std::nullopt, std::nullopt};
}

DjiAxisResult DjiAxis::tick(Tick now, float dt_s) noexcept {
    const auto device_state = device_.state(now);
    return tick_impl(now, dt_s, device_state.position_rad, device_state.velocity_rad_s, dt_s, 1.0f);
}

DjiAxisResult DjiAxis::tick_joint_velocity(Tick now,float dt_s,const Sample<float>& velocity) noexcept {
    if (mode_!=control::ControlMode::velocity && mode_!=control::ControlMode::position) {
        return fail(AxisStatus::invalid_target);
    }
    return tick_impl(now,dt_s,device_.state(now).position_rad,velocity,dt_s,1.0f);
}

DjiAxisResult DjiAxis::tick_external_position(Tick now, float dt_s, const Sample<float>& position,
    const Sample<float>& velocity, std::optional<float> outer_dt_s, float rate_per_axis) noexcept {
    if (mode_ != control::ControlMode::position) { return fail(AxisStatus::invalid_target); }
    return tick_impl(now, dt_s, position, velocity, outer_dt_s, rate_per_axis);
}

DjiAxisResult DjiAxis::tick_identification(Tick now, float dt_s, float target,
    const Sample<float>& velocity, float feedforward_a) noexcept {
    if (mode_ != control::ControlMode::position || !std::isfinite(feedforward_a)) { return fail(AxisStatus::invalid_target); }
    return tick_impl(now,dt_s,{},velocity,std::nullopt,1.0f,target,feedforward_a);
}

DjiAxisResult DjiAxis::tick_impl(Tick now, float dt_s, const Sample<float>& position,
    const Sample<float>& velocity, std::optional<float> outer_dt_s, float rate_per_axis,
    std::optional<float> rate_override, float feedforward_a) noexcept {
    if (!configured_) { return fail(AxisStatus::invalid_config); }
    if (!enabled_ || mode_ == control::ControlMode::none) { return fail(AxisStatus::idle); }
    if (!target_) { return fail(AxisStatus::no_target); }
    const Command<float> command{target_->value, target_->sampled_at, config_.command_ttl, *grant_};
    const auto accept = [&](float normalized) {
        output_ = drivers::DjiOutput{normalized, now};
        status_ = AxisStatus::active;
        return DjiAxisResult{AxisStatus::active, target_->value, normalized};
    };
    switch (mode_) {
    case control::ControlMode::current:
        if (!fresh(now, command.received_at, command.ttl)) { return fail(AxisStatus::stale_target); }
        return accept(target_->value / amp_full_scale_);
    case control::ControlMode::velocity: {
        const auto result = velocity_.step(now, dt_s, command, velocity);
        if (!result.output) { return fail(axis_status(result.status)); }
        return accept(*result.output * current_ratio_);
    }
    case control::ControlMode::position: {
        if (rate_override) {
            const Command<float> target{*rate_override,command.received_at,command.ttl,command.grant};
            const auto result=position_.step_inner(now,dt_s,target,velocity);
            if (!result.output) { return fail(axis_status(result.status)); }
            // 仅诊断入口叠加轴向电流, 总输出仍按既有轴/电调范围编码.
            return accept(std::clamp(*result.output*current_ratio_+feedforward_a/amp_full_scale_,
                -current_ratio_,current_ratio_));
        }
        const auto result =
            position_.step_split(now, dt_s, outer_dt_s, command, position, velocity, rate_per_axis);
        if (!result.output) {
            const auto inner = result.status == control::PositionStatus::inner_rejected && result.velocity_status
                ? axis_status(*result.velocity_status)
                : axis_status(result.status);
            return fail(inner);
        }
        return accept(*result.output * current_ratio_);
    }
    default:
        return fail(AxisStatus::idle);
    }
}

DjiAxisState DjiAxis::state(Tick now) const noexcept {
    DjiAxisState out{};
    out.mode = mode_;
    out.enabled = enabled_;
    if (target_) {
        out.target = *target_;
        if (!fresh(now, target_->sampled_at, config_.command_ttl)) { out.target.validity = Validity::invalid; }
    }
    const auto device_state = device_.state(now);
    out.rotor = device_state.rotor;
    out.velocity_rad_s = device_state.velocity_rad_s;
    out.position_rad = device_state.position_rad;
    out.output = output_;
    out.status = status_;
    return out;
}
} // 命名空间 ec::motion
