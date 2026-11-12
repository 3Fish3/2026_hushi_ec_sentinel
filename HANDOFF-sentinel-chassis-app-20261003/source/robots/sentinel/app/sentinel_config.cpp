#include "sentinel_config.hpp"
#include <cmath>
#include <numbers>

namespace ec::sentinel::app {
namespace {
constexpr float pi = std::numbers::pi_v<float>;

struct Validator {
    ConfigReport report{};
    void add(ConfigIssue issue) noexcept {
        for (std::size_t i = 0; i < report.issue_count; ++i) {
            if (report.issues[i] == issue) { return; }
        }
        if (report.issue_count < report.issues.size()) {
            report.issues[report.issue_count++] = issue;
        }
    }
};

[[nodiscard]] bool pid_ok(const control::PidConfig& gains) noexcept {
    return std::isfinite(gains.kp) && std::isfinite(gains.ki) && std::isfinite(gains.kd) &&
        std::isfinite(gains.integral_limit) && gains.integral_limit >= 0.0f &&
        std::isfinite(gains.output_min) && std::isfinite(gains.output_max) &&
        gains.output_min < gains.output_max;
}

// 两个需求条件取合取: 任一方不可用即不可用, 任一方缺项即缺项.
[[nodiscard]] ConfigState combine(ConfigState a, ConfigState b) noexcept {
    if (a == ConfigState::unavailable || b == ConfigState::unavailable) {
        return ConfigState::unavailable;
    }
    if (a == ConfigState::complete && b == ConfigState::complete) {
        return ConfigState::complete;
    }
    return ConfigState::incomplete;
}

// 部署用途下检查单项来源: 合成值或未标来源都不能进入正式部署配置.
void check_origin(Origin origin, ConfigAudience audience, Validator& v, bool& startable) noexcept {
    if (audience != ConfigAudience::deployment) { return; }
    if (origin == Origin::synthetic) {
        v.add(ConfigIssue::synthetic_value_in_deployment);
        startable = false;
    } else if (origin == Origin::unspecified) {
        v.add(ConfigIssue::unknown_origin_in_deployment);
        startable = false;
    }
}

[[nodiscard]] ConfigState check_dji_slot(const DjiSlot& slot, ConfigAudience audience,
    bool needs_position, Validator& v, bool& startable) noexcept {
    if (!slot.installed) { return ConfigState::unavailable; }
    check_origin(slot.origin, audience, v, startable);
    bool complete = true;
    const auto protocol = drivers::dji_protocol(slot.mode);
    const auto amp_full_scale = drivers::dji_current_amp_full_scale(slot.mode);
    // 电压模式没有电流上限语义, DjiAxis 的归一化基准无从建立.
    if (!protocol || !amp_full_scale) {
        v.add(ConfigIssue::unsupported_mode);
        complete = false;
    }
    if (slot.bus == 0 || slot.bus > 2) {
        v.add(ConfigIssue::invalid_bus);
        complete = false;
    }
    if (!protocol || slot.device_id == 0 || slot.device_id > protocol->count) {
        v.add(ConfigIssue::invalid_device_id);
        complete = false;
    }
    if (!slot.rotor_per_axis) {
        v.add(ConfigIssue::missing_rotor_ratio);
        complete = false;
    } else if (!std::isfinite(*slot.rotor_per_axis) || *slot.rotor_per_axis <= 0.0f) {
        v.add(ConfigIssue::invalid_rotor_ratio);
        complete = false;
    }
    if (!slot.direction) {
        v.add(ConfigIssue::missing_direction);
        complete = false;
    } else if (*slot.direction != 1 && *slot.direction != -1) {
        v.add(ConfigIssue::invalid_direction);
        complete = false;
    }
    if (!slot.feedback_ttl.valid()) {
        v.add(ConfigIssue::missing_feedback_ttl);
        complete = false;
    }
    if (!slot.current_limit_a) {
        v.add(ConfigIssue::missing_current_limit);
        complete = false;
    } else if (!std::isfinite(*slot.current_limit_a) || *slot.current_limit_a <= 0.0f ||
        (amp_full_scale && *slot.current_limit_a > *amp_full_scale)) {
        v.add(ConfigIssue::invalid_current_limit);
        complete = false;
    }
    if (!slot.output_limit_raw) {
        v.add(ConfigIssue::missing_output_limit);
        complete = false;
    } else if (*slot.output_limit_raw <= 0 ||
        (protocol && *slot.output_limit_raw > protocol->limit)) {
        v.add(ConfigIssue::invalid_output_limit);
        complete = false;
    }
    if (!slot.max_rotor_speed_rad_s) {
        v.add(ConfigIssue::missing_speed_bound);
        complete = false;
    } else if (!std::isfinite(*slot.max_rotor_speed_rad_s) || *slot.max_rotor_speed_rad_s <= 0.0f) {
        v.add(ConfigIssue::invalid_speed_bound);
        complete = false;
    }
    if (!slot.max_axis_velocity_rad_s) {
        v.add(ConfigIssue::missing_axis_speed_limit);
        complete = false;
    } else if (!std::isfinite(*slot.max_axis_velocity_rad_s) ||
        *slot.max_axis_velocity_rad_s <= 0.0f) {
        v.add(ConfigIssue::invalid_axis_speed_limit);
        complete = false;
    }
    if (!slot.velocity_pid) {
        v.add(ConfigIssue::missing_gains);
        complete = false;
    } else if (!pid_ok(*slot.velocity_pid)) {
        v.add(ConfigIssue::invalid_gains);
        complete = false;
    }
    if (needs_position) {
        if (!slot.position_pid) {
            v.add(ConfigIssue::missing_gains);
            complete = false;
        } else if (!pid_ok(*slot.position_pid)) {
            v.add(ConfigIssue::invalid_gains);
            complete = false;
        }
    }
    return complete ? ConfigState::complete : ConfigState::incomplete;
}

[[nodiscard]] ConfigState check_dm_slot(const DmSlot& slot, ConfigAudience audience, Validator& v,
    bool& startable) noexcept {
    if (!slot.installed) { return ConfigState::unavailable; }
    check_origin(slot.origin, audience, v, startable);
    bool complete = true;
    if (slot.bus == 0 || slot.bus > 2) {
        v.add(ConfigIssue::invalid_bus);
        complete = false;
    }
    if (!slot.can_id) {
        v.add(ConfigIssue::missing_dm_can_id);
        complete = false;
    }
    if (!slot.master_id) {
        v.add(ConfigIssue::missing_dm_master_id);
        complete = false;
    }
    if ((slot.can_id && *slot.can_id > 0x7ff) || (slot.master_id && *slot.master_id > 0x7ff)) {
        v.add(ConfigIssue::missing_dm_can_id);
        complete = false;
    }
    if (!slot.limits) {
        v.add(ConfigIssue::missing_dm_limits);
        complete = false;
    } else {
        const auto& limits = *slot.limits;
        const bool valid = std::isfinite(limits.position_max_rad) && limits.position_max_rad > 0.0f &&
            std::isfinite(limits.velocity_max_rad_s) && limits.velocity_max_rad_s > 0.0f &&
            std::isfinite(limits.torque_max_nm) && limits.torque_max_nm > 0.0f;
        if (!valid) {
            v.add(ConfigIssue::invalid_dm_limits);
            complete = false;
        }
    }
    if (!slot.feedback_ttl.valid()) {
        v.add(ConfigIssue::missing_feedback_ttl);
        complete = false;
    }
    const bool kp_ok = slot.mit_kp && std::isfinite(*slot.mit_kp) && *slot.mit_kp >= 0.0f &&
        *slot.mit_kp <= drivers::dm_kp_max;
    const bool kd_ok = slot.mit_kd && std::isfinite(*slot.mit_kd) && *slot.mit_kd >= 0.0f &&
        *slot.mit_kd <= drivers::dm_kd_max;
    if (!slot.mit_kp || !slot.mit_kd) {
        v.add(ConfigIssue::missing_dm_gains);
        complete = false;
    } else if (!kp_ok || !kd_ok) {
        v.add(ConfigIssue::invalid_dm_gains);
        complete = false;
    }
    return complete ? ConfigState::complete : ConfigState::incomplete;
}

[[nodiscard]] ConfigState check_wheels(const SentinelConfig& config, Validator& v) noexcept {
    bool complete = true;
    for (const auto& wheel : config.wheels) {
        const bool present = wheel.x_m && wheel.y_m && wheel.roll_dir_rad && wheel.radius_m &&
            wheel.gain;
        if (!present) {
            v.add(ConfigIssue::missing_wheel_geometry);
            complete = false;
            continue;
        }
        const bool valid = std::isfinite(*wheel.x_m) && std::isfinite(*wheel.y_m) &&
            std::isfinite(*wheel.roll_dir_rad) && std::isfinite(*wheel.radius_m) &&
            *wheel.radius_m > 0.0f && std::isfinite(*wheel.gain) &&
            (*wheel.gain == 1.0f || *wheel.gain == -1.0f);
        if (!valid) {
            v.add(ConfigIssue::invalid_wheel_geometry);
            complete = false;
        }
    }
    if (complete) {
        const auto drive = omni_drive(config);
        if (!drive || !motion::omni_well_posed(*drive)) {
            v.add(ConfigIssue::degenerate_wheel_layout);
            complete = false;
        }
    }
    return complete ? ConfigState::complete : ConfigState::incomplete;
}

// 关节几何: 返回每个关节表的配置状态; 未确认零位/范围都算缺项.
[[nodiscard]] std::array<ConfigState, joint_slot_count> check_joints(const SentinelConfig& config,
    Validator& v) noexcept {
    std::array<ConfigState, joint_slot_count> states{};
    std::array<bool, joint_slot_count> seen{};
    for (const auto& joint : config.joints) {
        if (!is_joint(joint.domain)) {
            v.add(ConfigIssue::invalid_joint_range);
            continue;
        }
        const auto index = joint_index(joint.domain);
        if (seen[index]) {
            v.add(ConfigIssue::domain_duplicated);
            continue;
        }
        seen[index] = true;
        if (!joint.zero_offset_rad || (!joint.manual_continuous && !joint.commissioning_unbounded && (!joint.min_rad || !joint.max_rad))) {
            v.add(ConfigIssue::missing_joint_reference);
            states[index] = ConfigState::incomplete;
            continue;
        }
        bool valid = std::isfinite(*joint.zero_offset_rad) && (joint.commissioning_unbounded
            ? !joint.manual_continuous && !joint.min_rad && !joint.max_rad
            : joint.manual_continuous ? joint.domain != Domain::pitch && !joint.min_rad && !joint.max_rad
            : std::isfinite(*joint.min_rad) && std::isfinite(*joint.max_rad) && *joint.min_rad < *joint.max_rad);
        if (joint.commissioning_unbounded && joint.domain != Domain::pitch) {
            for (const auto& slot : config.dji) {
                if (slot.installed && slot.domain == joint.domain) {
                    valid = valid && slot.rotor_per_axis && *slot.rotor_per_axis == 1.0f;
                }
            }
        }
        if (!valid) {
            v.add(ConfigIssue::invalid_joint_range);
            states[index] = ConfigState::incomplete;
            continue;
        }
        states[index] = ConfigState::complete;
    }
    for (std::size_t i = 0; i < joint_slot_count; ++i) {
        if (!seen[i]) {
            v.add(ConfigIssue::missing_joint_reference);
            states[i] = ConfigState::incomplete;
        }
    }
    return states;
}

[[nodiscard]] bool check_timing(const TimingConfig& timing, Validator& v) noexcept {
    bool ok = true;
    if (!(timing.control_dt_s > 0.0f) || !std::isfinite(timing.control_dt_s)) {
        v.add(ConfigIssue::missing_timing);
        ok = false;
    }
    for (const auto ttl : {timing.target_ttl, timing.remote_ttl, timing.imu_ttl,
             timing.host_link_ttl}) {
        if (!ttl.valid()) {
            v.add(ConfigIssue::missing_timing);
            ok = false;
        }
    }
    if (!timing.max_dt_s) {
        v.add(ConfigIssue::missing_timing);
        ok = false;
    } else if (!std::isfinite(*timing.max_dt_s) || *timing.max_dt_s < timing.control_dt_s) {
        v.add(ConfigIssue::invalid_timing);
        ok = false;
    }
    return ok;
}
} // 内部辅助

bool ConfigReport::has(ConfigIssue issue) const noexcept { return count(issue) > 0; }
std::size_t ConfigReport::count(ConfigIssue issue) const noexcept {
    std::size_t total = 0;
    for (std::size_t i = 0; i < issue_count; ++i) {
        if (issues[i] == issue) { ++total; }
    }
    return total;
}

ConfigReport validate(const SentinelConfig& config, ConfigAudience audience) noexcept {
    Validator v{};
    v.report.kind = config.kind;
    v.report.audience = audience;
    bool startable = true;
    const bool deployment = audience == ConfigAudience::deployment;
    if (config.kind == ConfigKind::unconfigured) {
        v.add(ConfigIssue::unconfigured_kind);
        startable = false;
    }
    if (deployment && config.kind == ConfigKind::synthetic_fixture) {
        v.add(ConfigIssue::synthetic_value_in_deployment);
        startable = false;
    }

    if (config.serial) {
        check_origin(config.serial_origin, audience, v, startable);
        if (!config.serial->valid()) { v.add(ConfigIssue::invalid_serial_geometry); startable = false; }
    }

    // 域身份唯一性: 每颗设备槽显式声明域, 不允许两个槽认领同一域.
    std::array<const DjiSlot*, domain_count> dji_by_domain{};
    std::array<const DmSlot*, domain_count> dm_by_domain{};
    std::array<unsigned, domain_count> claimed{};
    for (const auto& slot : config.dji) {
        if (!slot.installed) { continue; }
        const auto index = domain_index(slot.domain);
        if (claimed[index]++ == 0) {
            dji_by_domain[index] = &slot;
        } else {
            v.add(ConfigIssue::domain_duplicated);
        }
    }
    for (const auto& slot : config.dm) {
        if (!slot.installed) { continue; }
        const auto index = domain_index(slot.domain);
        if (claimed[index]++ == 0) {
            dm_by_domain[index] = &slot;
        } else {
            v.add(ConfigIssue::domain_duplicated);
        }
    }

    // 电流模式GM6020仍识别对应的电压命令ID; C610/C620高编号组不能与它混用.
    const auto gm_voltage = *drivers::dji_protocol(drivers::DjiMode::gm6020_voltage);
    for (const auto& gm : config.dji) {
        if (!gm.installed || gm.mode != drivers::DjiMode::gm6020_current) { continue; }
        const auto voltage_id = gm.device_id <= 4 ? gm_voltage.first_tx : gm_voltage.second_tx;
        for (const auto& other : config.dji) {
            if (!other.installed || other.bus != gm.bus) { continue; }
            const auto protocol = drivers::dji_protocol(other.mode);
            if (!protocol) { continue; }
            const auto command_id = other.device_id <= 4 ? protocol->first_tx : protocol->second_tx;
            if (command_id == voltage_id) {
                v.add(ConfigIssue::dji_command_mode_conflict);
                startable = false;
            }
        }
    }
    // 达妙问询式接收按低8位匹配, 不能只检查11位CAN ID是否相等.
    // DJI反馈也是总线上的实际报文, MCU接收过滤不能阻止电调直接接收它.
    for (const auto& dm : config.dm) {
        if (!dm.installed || !dm.can_id) { continue; }
        for (const auto& dji : config.dji) {
            if (!dji.installed || dji.bus != dm.bus) { continue; }
            const auto protocol = drivers::dji_protocol(dji.mode);
            if (protocol && ((protocol->rx_base + dji.device_id) & 0xffu) == (*dm.can_id & 0xffu)) {
                v.add(ConfigIssue::dm_feedback_id_alias);
                startable = false;
            }
        }
    }
    const auto wheel_state = check_wheels(config, v);
    const auto joint_states = check_joints(config, v);
    const bool groups_ok = config.runtime.gm.valid() && config.runtime.chassis.valid() &&
        config.runtime.shooter.valid() && config.runtime.pitch.valid() &&
        config.runtime.direction_period_us > 0 &&
        config.runtime.direction_period_us <= config.runtime.direction_max_step_us &&
        config.runtime.direction_max_step_us < 0x80000000u;
    if (!groups_ok) { v.add(ConfigIssue::invalid_timing); }
    const bool timing_ok = check_timing(config.timing, v) && groups_ok;

    bool all_complete = timing_ok;
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        ConfigState state = ConfigState::unavailable;
        if (dji_by_domain[i]) {
            const bool needs_position =
                domain == Domain::main_yaw || domain == Domain::fire_yaw;
            state = check_dji_slot(*dji_by_domain[i], audience, needs_position, v, startable);
        } else if (dm_by_domain[i]) {
            state = check_dm_slot(*dm_by_domain[i], audience, v, startable);
        }
        if (state != ConfigState::unavailable) {
            if (is_chassis(domain)) { state = combine(state, wheel_state); }
            if (is_joint(domain)) { state = combine(state, joint_states[joint_index(domain)]); }
        }
        v.report.domains[i] = state;
        if (state != ConfigState::complete) { all_complete = false; }
    }
    v.report.complete = all_complete;
    v.report.startable = startable;
    return v.report;
}

std::optional<drivers::DjiDeviceConfig> device_config(const DjiSlot& slot) noexcept {
    if (!slot.installed) { return std::nullopt; }
    const auto protocol = drivers::dji_protocol(slot.mode);
    if (!protocol || slot.bus == 0 || slot.bus > 2 || slot.device_id == 0 ||
        slot.device_id > protocol->count) {
        return std::nullopt;
    }
    if (!slot.rotor_per_axis || !std::isfinite(*slot.rotor_per_axis) || *slot.rotor_per_axis <= 0.0f) {
        return std::nullopt;
    }
    if (!slot.direction || (*slot.direction != 1 && *slot.direction != -1)) { return std::nullopt; }
    if (!slot.feedback_ttl.valid()) { return std::nullopt; }
    if (!slot.max_rotor_speed_rad_s || !std::isfinite(*slot.max_rotor_speed_rad_s) ||
        *slot.max_rotor_speed_rad_s <= 0.0f) {
        return std::nullopt;
    }
    if (!slot.output_limit_raw || *slot.output_limit_raw <= 0 ||
        *slot.output_limit_raw > protocol->limit) {
        return std::nullopt;
    }
    return drivers::DjiDeviceConfig{slot.bus, slot.mode, slot.device_id, *slot.rotor_per_axis,
        *slot.direction, slot.feedback_ttl, *slot.max_rotor_speed_rad_s, *slot.output_limit_raw};
}

std::optional<drivers::DmDeviceConfig> device_config(const DmSlot& slot) noexcept {
    if (!slot.installed || slot.bus == 0 || slot.bus > 2) { return std::nullopt; }
    if (!slot.can_id || !slot.master_id || *slot.can_id > 0x7ff || *slot.master_id > 0x7ff) {
        return std::nullopt;
    }
    if (!slot.limits || !slot.feedback_ttl.valid()) { return std::nullopt; }
    const auto& limits = *slot.limits;
    if (!(std::isfinite(limits.position_max_rad) && limits.position_max_rad > 0.0f) ||
        !(std::isfinite(limits.velocity_max_rad_s) && limits.velocity_max_rad_s > 0.0f) ||
        !(std::isfinite(limits.torque_max_nm) && limits.torque_max_nm > 0.0f)) {
        return std::nullopt;
    }
    return drivers::DmDeviceConfig{slot.bus, *slot.can_id, *slot.master_id, limits,
        slot.feedback_ttl};
}

std::optional<motion::OmniDrive> omni_drive(const SentinelConfig& config) noexcept {
    motion::OmniDrive drive{};
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        const auto& wheel = config.wheels[i];
        if (!wheel.x_m || !wheel.y_m || !wheel.roll_dir_rad || !wheel.radius_m || !wheel.gain) {
            return std::nullopt;
        }
        if (!std::isfinite(*wheel.x_m) || !std::isfinite(*wheel.y_m) ||
            !std::isfinite(*wheel.roll_dir_rad) || !std::isfinite(*wheel.radius_m) ||
            *wheel.radius_m <= 0.0f || (*wheel.gain != 1.0f && *wheel.gain != -1.0f)) {
            return std::nullopt;
        }
        drive.wheels[i] = motion::OmniWheel{*wheel.x_m, *wheel.y_m, *wheel.roll_dir_rad,
            *wheel.radius_m, *wheel.gain};
    }
    return drive;
}

SentinelConfig synthetic_config() noexcept {
    SentinelConfig config{};
    config.kind = ConfigKind::synthetic_fixture;
    const auto slot = [](Domain domain, std::uint8_t bus, drivers::DjiMode mode,
                          std::uint8_t device_id, float rotor_per_axis, int direction,
                          float current_limit_a, std::int16_t output_limit_raw,
                          float axis_velocity_limit_rad_s, bool needs_position) {
        DjiSlot out{};
        out.domain = domain;
        out.installed = true;
        out.origin = Origin::synthetic;
        out.bus = bus;
        out.mode = mode;
        out.device_id = device_id;
        out.rotor_per_axis = rotor_per_axis;
        out.direction = direction;
        out.current_limit_a = current_limit_a;
        out.output_limit_raw = output_limit_raw;
        out.max_rotor_speed_rad_s = 100.0f;
        // 轴侧量程按机构类型给出: 夹具值, 不代表任何实车量程, 但至少自洽
        // (轮/云台/摩擦轮/拨弹的量级不同, 用同一个值会让夹具测试失去意义).
        out.max_axis_velocity_rad_s = axis_velocity_limit_rad_s;
        out.feedback_ttl = Lifetime{20};
        // 夹具增益: 量纲是"满量程分数", 数值本身不对应任何实车调参结果.
        out.velocity_pid = control::PidConfig{1.2f, 0.05f, 0.0f, 0.0f, -1.0f, 1.0f};
        if (needs_position) {
            out.position_pid = control::PidConfig{2.0f, 0.0f, 0.02f, 0.5f, -1.0f, 1.0f};
        }
        return out;
    };
    config.dji[0] = slot(Domain::chassis_lf, 2, drivers::DjiMode::c620_current, 1, 19.0f, 1,
        3.0f, 16384, 50.0f, false);
    config.dji[1] = slot(Domain::chassis_rf, 2, drivers::DjiMode::c620_current, 2, 19.0f, 1,
        3.0f, 16384, 50.0f, false);
    config.dji[2] = slot(Domain::chassis_lb, 2, drivers::DjiMode::c620_current, 3, 19.0f, 1,
        3.0f, 16384, 50.0f, false);
    config.dji[3] = slot(Domain::chassis_rb, 2, drivers::DjiMode::c620_current, 4, 19.0f, 1,
        3.0f, 16384, 50.0f, false);
    config.dji[4] = slot(Domain::main_yaw, 1, drivers::DjiMode::gm6020_current, 6, 1.0f, -1,
        3.0f, 16384, 20.0f, true);
    config.dji[5] = slot(Domain::fire_yaw, 1, drivers::DjiMode::gm6020_current, 4, 1.0f, 1, 3.0f,
        16384, 20.0f, true);
    config.dji[6] = slot(Domain::friction_left, 2, drivers::DjiMode::c620_current, 5, 19.0f, -1,
        5.0f, 16384, 30.0f, false);
    config.dji[7] = slot(Domain::friction_right, 2, drivers::DjiMode::c620_current, 6, 19.0f, 1,
        5.0f, 16384, 30.0f, false);
    config.dji[8] = slot(Domain::trigger, 2, drivers::DjiMode::c610_current, 7, 36.0f, -1, 5.0f,
        10000, 12.0f, false);

    DmSlot pitch{};
    pitch.domain = Domain::pitch;
    pitch.installed = true;
    pitch.origin = Origin::synthetic;
    pitch.bus = 1;
    pitch.can_id = 0x01;
    pitch.master_id = 0x00;
    pitch.limits = drivers::DmMitLimits{12.5f, 30.0f, 10.0f};
    pitch.feedback_ttl = Lifetime{20};
    pitch.mit_kp = 20.0f;
    pitch.mit_kd = 1.0f;
    config.dm[0] = pitch;

    // 夹具几何: 轮心 ±0.2 m 的正方形布局, 滚动方向切向 (半径方向 +90°), 半径 0.06 m.
    const std::array<std::array<float, 2>, chassis_wheel_count> positions{{
        {0.2f, 0.2f}, {0.2f, -0.2f}, {-0.2f, 0.2f}, {-0.2f, -0.2f}}};
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        config.wheels[i].x_m = positions[i][0];
        config.wheels[i].y_m = positions[i][1];
        config.wheels[i].roll_dir_rad = std::atan2(positions[i][1], positions[i][0]) + pi * 0.5f;
        config.wheels[i].radius_m = 0.06f;
        config.wheels[i].gain = 1.0f;
    }
    // 夹具关节: 主 yaw 是机械受限的有限转角 (< 一圈, 否则单圈编码器无法唯一恢复参考).
    config.joints[0] = JointGeometryConfig{Domain::main_yaw, 0.0f, -2.6f, 2.6f};
    config.joints[1] = JointGeometryConfig{Domain::fire_yaw, 0.0f, -1.5f, 1.5f};
    config.joints[2] = JointGeometryConfig{Domain::pitch, 0.0f, -1.0f, 0.5f};

    config.timing.control_dt_s = 0.001f;
    config.timing.target_ttl = Lifetime{20};
    config.timing.remote_ttl = Lifetime{100};
    config.timing.imu_ttl = Lifetime{50};
    config.timing.host_link_ttl = Lifetime{500};
    config.timing.max_dt_s = 0.02f;
    // 历史夹具显式使用同频组, 多速率用例另行选择 RuntimeTiming 候选.
    const GroupTiming fixture{1000, 0, {2000}, 20000};
    config.runtime.gm = fixture;
    config.runtime.chassis = fixture;
    config.runtime.shooter = fixture;
    config.runtime.pitch = fixture;
    config.perception = PerceptionTiming{{50000}, {20000}, {5000}, 20000};
    config.serial = SerialGeometry{{0,0,1}, {0,1,0}, {0,0,1}, {1,0,0}, {},
        estimation::identity_rotation(), 0, 0, {}, {}, 0.01f, 0.02f, 0.8f};
    config.serial_origin = Origin::synthetic;
    return config;
}

SentinelConfig real_config_placeholder() noexcept {
    // 只声明配置类别, 不填任何猜测值: 所有域保持"未装配/未提供", 由持有者逐项补齐.
    SentinelConfig config{};
    config.kind = ConfigKind::real_deployment;
    return config;
}
} // 命名空间 ec::sentinel::app
