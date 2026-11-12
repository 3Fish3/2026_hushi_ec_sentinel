#include "sentinel_remote.hpp"
#include <cmath>

namespace ec::sentinel::app {
namespace {
[[nodiscard]] float clampf(float value, float limit) noexcept {
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}
[[nodiscard]] std::uint32_t index(Domain domain) noexcept { return domain_index(domain); }
// 用途索引: 只用于取该用途的目标角与回中状态.
constexpr std::size_t purpose_index(AxisPurpose purpose) noexcept {
    return static_cast<std::size_t>(purpose);
}
constexpr AxisPurpose angle_purpose(Domain domain) noexcept {
    switch (domain) {
    case Domain::main_yaw: return AxisPurpose::main_yaw_direction;
    case Domain::fire_yaw: return AxisPurpose::fire_yaw_direction;
    case Domain::pitch: return AxisPurpose::pitch_angle;
    default: return AxisPurpose::none;
    }
}
} // 内部辅助

RemoteMapper::RemoteMapper(RemoteMappingConfig config, Lifetime remote_ttl) noexcept
    : config_(config), ttl_(remote_ttl) {}

// 参考语义改变: 该关节的已保持角属于旧坐标, 不能作为新参考下的目标继续执行.
// 只丢弃保持角, 不清来源意图/拨杆层/摩擦开关状态.
void RemoteMapper::drop_held_angle(Domain domain) noexcept {
    const auto purpose = angle_purpose(domain);
    if (purpose == AxisPurpose::none) { return; }
    auto& state = purposes_[purpose_index(purpose)];
    state.target_valid = false;
    state.target_rad = 0.0f;
}

std::optional<float> RemoteMapper::held_angle(Domain domain) const noexcept {
    const auto purpose = angle_purpose(domain);
    const auto& state = purposes_[purpose_index(purpose)];
    return purpose != AxisPurpose::none && state.target_valid ? std::optional{state.target_rad} : std::nullopt;
}
void RemoteMapper::limit_held_angle(Domain domain, float value) noexcept {
    const auto purpose = angle_purpose(domain);
    auto& state = purposes_[purpose_index(purpose)];
    if (purpose == AxisPurpose::none || !state.target_valid) { return; }
    if (state.target_rad != value) { ++diagnostics_.angle_limits; }
    state.target_rad = value;
}
void RemoteMapper::reset() noexcept {
    layer_ = Ch4Layer::unknown;
    intent_ = SourceIntent::none;
    purposes_ = {};
    x_purpose_ = AxisPurpose::none;
    y_purpose_ = AxisPurpose::none;
    friction_on_ = false;
    manual_stop_ = false;
    started_ = false;
    chassis_centred_ = false;
}

// 用途索引: 只用于取该用途的目标角与回中状态.
Ch4Layer RemoteMapper::classify(std::int16_t ch4) const noexcept {
    const auto& c = config_.ch4;
    // 滞回: 只有越过带外阈值才换层, 避免中位附近抖动导致反复切层.
    const auto high = [&](std::int16_t value) { return value > c.upper; };
    const auto low = [&](std::int16_t value) { return value < c.lower; };
    Ch4Layer side = Ch4Layer::unknown;
    if (high(ch4)) { side = c.low_is_launcher ? Ch4Layer::fire_yaw : Ch4Layer::launcher; }
    else if (low(ch4)) { side = c.low_is_launcher ? Ch4Layer::launcher : Ch4Layer::fire_yaw; }
    else { side = Ch4Layer::main_gimbal; }
    if (layer_ == Ch4Layer::unknown || side == layer_) { return side; }
    // 换层方向与当前层相反时, 需要越过带外阈值 (滞回带内保持当前层).
    if (side == Ch4Layer::main_gimbal) { return Ch4Layer::main_gimbal; }
    const bool moving_up = ch4 > c.mid;
    const bool inside_band = (moving_up && ch4 < c.upper + c.hysteresis) ||
        (!moving_up && ch4 > c.lower - c.hysteresis);
    return inside_band ? layer_ : side;
}

bool RemoteMapper::stick_active(float axis) const noexcept {
    return std::fabs(axis) > config_.stick_deadzone;
}

RemoteCommand RemoteMapper::step(const std::optional<input::Dr16Snapshot>& sample, Tick now,
    float dt_s, const std::array<std::optional<float>, domain_count>& actual_angle_rad) noexcept {
    RemoteCommand out{};
    static_cast<void>(now); // 时效由 Dr16Input 判定, 这里只处理已经一致的快照
    if (!sample) {
        // 失联/从未收到: 清除本地摩擦与拨弹意图, 不自动恢复; 已生效的人工停止不被解除.
        diagnostics_.link_lost = true;
        chassis_centred_ = false;
        friction_on_ = false;
        purposes_ = {};
        x_purpose_ = AxisPurpose::none;
        y_purpose_ = AxisPurpose::none;
        out.manual_stop = manual_stop_;
        return out;
    }
    diagnostics_.link_lost = false;
    ++diagnostics_.frames;
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) { dt_s = 0.0f; }

    // 左拨杆: 上=本地, 中=待机/释放正常来源, 下=Jetson. 遥控自身不产生 Jetson 目标.
    const auto previous_intent = intent_;
    switch (remote_source_selection(config_.layout, sample->switch_left, sample->switch_right)) {
    case drivers::Dr16Switch::up: intent_ = SourceIntent::local; break;
    case drivers::Dr16Switch::mid: intent_ = SourceIntent::standby; break;
    case drivers::Dr16Switch::down: intent_ = SourceIntent::host; break;
    case drivers::Dr16Switch::unknown: intent_ = SourceIntent::none; break;
    }
    if (intent_ != previous_intent) {
        ++diagnostics_.source_changes;
        chassis_centred_ = false;
        // 换源清掉人工摩擦/拨弹意图与角度通道, 不自动恢复.
        friction_on_ = false;
        purposes_ = {};
        x_purpose_ = AxisPurpose::none;
        y_purpose_ = AxisPurpose::none;
        if (config_.layout == RemoteLayout::gimbal_validation && intent_ == SourceIntent::local) {
            // 回中等待期间也保持接管角, 避免先撤力矩再等待摇杆.
            for (const auto domain : {Domain::main_yaw, Domain::fire_yaw, Domain::pitch}) {
                const auto angle = actual_angle_rad[index(domain)];
                if (!angle || !std::isfinite(*angle)) { continue; }
                auto& held = purposes_[purpose_index(angle_purpose(domain))];
                held.target_rad = *angle;
                held.target_valid = true;
            }
        }
    }
    // 右拨杆: 下=人工整车停止 (锁存到拨回运行档), 中=普通运行, 上=本地底盘旋转操作.
    manual_stop_ = sample->switch_right == drivers::Dr16Switch::down;
    out.manual_stop = manual_stop_;
    out.intent = manual_stop_ ? SourceIntent::standby : intent_;
    out.rotation_gear = sample->switch_right == drivers::Dr16Switch::up;

    if (config_.layout == RemoteLayout::mechanism_commissioning) {
        // 云台/发射独立包由维护台给目标, 遥控仅提供首启、维护档和人工停止.
        out.rotation_gear = false;
        return out;
    }
    if (config_.layout == RemoteLayout::chassis_commissioning) {
        // 此角色只接管底盘, 不读取拨轮、不生成任何云台或发射目标.
        out.rotation_gear = false;
        if (out.intent != SourceIntent::local || manual_stop_) {
            chassis_centred_ = false;
            return out;
        }
        const bool active = stick_active(sample->axis[0]) || stick_active(sample->axis[2]) ||
            stick_active(sample->axis[3]);
        if (!chassis_centred_) {
            if (active) { return out; }
            chassis_centred_ = true;
        }
        const auto stick = [&](float value) {
            return stick_active(value) ? clampf(value, 1.0f) : 0.0f;
        };
        // DR16横杆正值为右拨: 车体+Y向左, +wz为俯视逆时针.
        out.targets.chassis = motion::PlanarTwist{
            stick(sample->axis[3]) * config_.chassis_vx_max_mps,
            -stick(sample->axis[2]) * config_.chassis_vy_max_mps,
            -stick(sample->axis[0]) * config_.chassis_wz_max_rad_s};
        out.chassis_active = true;
        return out;
    }

    const auto previous_layer = layer_;
    if (config_.layout == RemoteLayout::gimbal_validation) {
        // 验证档不依赖已失效的拨轮: 右中操纵主 yaw, 右上操纵发射 yaw; pitch 始终共用右 Y.
        out.rotation_gear = false;
        layer_ = sample->switch_right == drivers::Dr16Switch::up ? Ch4Layer::fire_yaw : Ch4Layer::main_gimbal;
    } else {
        layer_ = classify(sample->ch4);
    }
    if (layer_ != previous_layer) { ++diagnostics_.layer_changes; }
    out.layer = layer_;

    const bool local_active = out.intent == SourceIntent::local && !manual_stop_;

    // 本周期两个输入通道的用途. 用途改变 → 该用途必须先看到一次回中才接管.
    AxisPurpose x_purpose = AxisPurpose::none;
    AxisPurpose y_purpose = AxisPurpose::none;
    if (local_active) {
        if (out.rotation_gear) { x_purpose = AxisPurpose::rotation_wz; }
        else if (layer_ == Ch4Layer::main_gimbal) { x_purpose = AxisPurpose::main_yaw_direction; }
        else if (layer_ == Ch4Layer::fire_yaw) { x_purpose = AxisPurpose::fire_yaw_direction; }
        else if (layer_ == Ch4Layer::launcher) { x_purpose = AxisPurpose::friction_switch; }
        if (layer_ == Ch4Layer::main_gimbal || layer_ == Ch4Layer::fire_yaw) {
            y_purpose = AxisPurpose::pitch_angle;
        } else if (layer_ == Ch4Layer::launcher) {
            y_purpose = AxisPurpose::trigger;
        }
    }
    const auto purpose_changed = [&](AxisPurpose previous, AxisPurpose current) {
        if (previous == current || current == AxisPurpose::none) { return false; }
        auto& state = purposes_[purpose_index(current)];
        // 任何"改用途"都要求先看到一次回中: 角度用途、开关类用途与旋转档覆盖用同一条规则.
        // 不因为"首次进入该用途"就接受旧偏转 (C06).
        state.awaiting_recentre = true;
        ++diagnostics_.recentre_required;
        return true;
    };
    static_cast<void>(purpose_changed(x_purpose_, x_purpose));
    static_cast<void>(purpose_changed(y_purpose_, y_purpose));
    x_purpose_ = x_purpose;
    y_purpose_ = y_purpose;

    // 回中一次后才允许该用途接管; 未操纵的角度用途保持目标角.
    const auto may_act = [&](AxisPurpose purpose, bool active) {
        auto& state = purposes_[purpose_index(purpose)];
        if (state.awaiting_recentre) {
            if (active) { return false; }
            state.awaiting_recentre = false;
        }
        return true;
    };

    // 左杆前后/左右始终给底盘 vx/vy; 旋转档的 wz 由右 X 经同一回中接管后覆盖 (见下).
    // 未标定的映射配置 (量程为 0) 不得生成"有效零目标": 零值不等于已标定的零运动意图.
    const bool chassis_configured = config_.chassis_vx_max_mps > 0.0f ||
        config_.chassis_vy_max_mps > 0.0f || config_.chassis_wz_max_rad_s > 0.0f;
    if (local_active && chassis_configured) {
        out.targets.chassis = motion::PlanarTwist{
            clampf(sample->axis[3], 1.0f) * config_.chassis_vx_max_mps,
            clampf(sample->axis[2], 1.0f) * config_.chassis_vy_max_mps, 0.0f};
        out.chassis_active = true;
    }

    const float right_x = sample->axis[0];
    const float right_y = sample->axis[1];

    // 已保持的角度目标每周期都要出现在完整快照里: 切层只改变"当前操纵哪个通道",
    // 不能让另一个轴的保持角从快照里消失 —— 完整快照会把缺省域当作撤销, 于是"没操纵的轴
    // 保持"会变成"被撤输出".
    const auto emit_held_angle = [&](AxisPurpose purpose, Domain domain) {
        const auto& state = purposes_[purpose_index(purpose)];
        if (!state.target_valid) { return; }
        out.targets.domains[index(domain)] =
            DomainSetpoint{control::ControlMode::position, state.target_rad, 0,
                domain == Domain::main_yaw && config_.layout != RemoteLayout::gimbal_validation
                    ? YawFrame::local_inertial : YawFrame::joint};
    };

    // 角度用途: 目标角速度 × dt 积分成角度目标, 回中保持目标角.
    const auto apply_angle = [&](AxisPurpose purpose, float axis, Domain domain, float rate_max) {
        const bool active = stick_active(axis);
        if (!may_act(purpose, active)) { return; }
        auto& state = purposes_[purpose_index(purpose)];
        if (!state.target_valid) {
            // 初次接管取有效实际姿态建立新目标; 没有有效实际姿态就不给目标.
            const auto actual = actual_angle_rad[index(domain)];
            if (!actual) { return; }
            state.target_rad = *actual;
            state.target_valid = true;
        }
        if (active && dt_s > 0.0f) { state.target_rad += clampf(axis, 1.0f) * rate_max * dt_s; }
        out.targets.domains[index(domain)] =
            DomainSetpoint{control::ControlMode::position, state.target_rad, 0,
                domain == Domain::main_yaw && config_.layout != RemoteLayout::gimbal_validation
                    ? YawFrame::local_inertial : YawFrame::joint};
    };

    switch (x_purpose) {
    case AxisPurpose::main_yaw_direction:
        if (config_.yaw_rate_max_rad_s > 0.0f) {
            apply_angle(x_purpose, right_x, Domain::main_yaw, config_.yaw_rate_max_rad_s);
        }
        break;
    case AxisPurpose::fire_yaw_direction:
        if (config_.yaw_rate_max_rad_s > 0.0f) {
            apply_angle(x_purpose, right_x, Domain::fire_yaw, config_.yaw_rate_max_rad_s);
        }
        break;
    case AxisPurpose::friction_switch: {
        // 右 X 正向开/负向关/中位保持; 不锁存停止, 跨层保留.
        const bool active = stick_active(right_x);
        if (may_act(x_purpose, active)) {
            if (right_x > config_.stick_deadzone) { friction_on_ = true; }
            else if (right_x < -config_.stick_deadzone) { friction_on_ = false; }
        }
        break;
    }
    case AxisPurpose::rotation_wz: {
        // 旋转档覆盖同一轴的用途: 与角度/开关用途走同一条回中接管, 不直接使用轴值.
        const bool active = stick_active(right_x);
        if (may_act(x_purpose, active) && active && out.chassis_active) {
            out.targets.chassis = motion::PlanarTwist{out.targets.chassis->vx_mps,
                out.targets.chassis->vy_mps,
                clampf(right_x, 1.0f) * config_.chassis_wz_max_rad_s};
        }
        break;
    }
    case AxisPurpose::pitch_angle:
    case AxisPurpose::trigger:
    case AxisPurpose::none:
        break;
    }

    if (y_purpose == AxisPurpose::pitch_angle) {
        // 未标定的 pitch 速率 (0) 不生成位置保持目标 (那会是"有效零目标").
        if (config_.pitch_rate_max_rad_s > 0.0f) {
            apply_angle(y_purpose, right_y, Domain::pitch, config_.pitch_rate_max_rad_s);
        }
    } else if (y_purpose == AxisPurpose::trigger) {
        // 拨弹不锁存: 中位立即取消, 负向人工反转.
        const bool active = stick_active(right_y);
        if (config_.trigger_rad_s > 0.0f && may_act(y_purpose, active) && active) {
            out.targets.domains[index(Domain::trigger)] = DomainSetpoint{
                control::ControlMode::velocity,
                right_y > 0.0f ? config_.trigger_rad_s : -config_.trigger_rad_s};
        }
    }
    // 已保持的角度目标每周期都要出现在完整快照里: 切层与"等待回中"只限制新的操纵
    // 增量, 不取消已有保持; 切回已保持的用途时 apply_angle 会提前返回, 因此本轴的保持
    // 也必须在这里补上, 否则完整快照的缺省语义会把它当成撤销.
    if (local_active) {
        emit_held_angle(AxisPurpose::main_yaw_direction, Domain::main_yaw);
        emit_held_angle(AxisPurpose::fire_yaw_direction, Domain::fire_yaw);
        emit_held_angle(AxisPurpose::pitch_angle, Domain::pitch);
    }

    // 摩擦轮: 跨层保留人工运行意图, 由新鲜遥控帧持续生成本地目标.
    // 未标定的目标转速 (0) 不生成"有效零目标".
    out.friction_on = friction_on_;
    if (local_active && friction_on_ && config_.friction_rad_s > 0.0f) {
        out.targets.domains[index(Domain::friction_left)] =
            DomainSetpoint{control::ControlMode::velocity, config_.friction_rad_s};
        out.targets.domains[index(Domain::friction_right)] =
            DomainSetpoint{control::ControlMode::velocity, -config_.friction_rad_s};
    }
    if (!local_active) {
        // 非本地来源或人工停止: 不给本地目标, 并清掉人工摩擦/拨弹意图.
        friction_on_ = false;
        purposes_ = {};
        x_purpose_ = AxisPurpose::none;
        y_purpose_ = AxisPurpose::none;
    }
    started_ = true;
    return out;
}
} // 命名空间 ec::sentinel::app
