#include "sentinel_param_apply.hpp"
#include <cmath>

namespace ec::sentinel::app {
namespace {
// 增益类参数在参数表中的局部序号 (与 ParamStore 的构造保持一致).
constexpr std::uint16_t local_velocity_kp = 1;
constexpr std::uint16_t local_velocity_ki = 2;
constexpr std::uint16_t local_velocity_kd = 3;
constexpr std::uint16_t local_current_limit = 4;
constexpr std::uint16_t local_axis_velocity = 6;
constexpr std::uint16_t local_position_kp = 8;
constexpr std::uint16_t local_position_ki = 9;
constexpr std::uint16_t local_position_kd = 10;
constexpr std::uint16_t local_joint_zero = 20;
constexpr std::uint16_t local_joint_min = 21;
constexpr std::uint16_t local_joint_max = 22;
// 整车槽的局部序号.
constexpr std::uint16_t local_vehicle_yaw_alignment = 1;
constexpr std::uint16_t local_vehicle_target_ttl = 2;

// 从一份 PidConfig 里按局部序号取/放增益, 避免三处重复的 if 链.
[[nodiscard]] float gain_of(const control::PidConfig& pid, std::uint16_t local) noexcept {
    switch (local) {
    case local_velocity_kp:
    case local_position_kp: return pid.kp;
    case local_velocity_ki:
    case local_position_ki: return pid.ki;
    default: return pid.kd;
    }
}

void set_gain(control::PidConfig& pid, std::uint16_t local, float value) noexcept {
    switch (local) {
    case local_velocity_kp:
    case local_position_kp: pid.kp = value; return;
    case local_velocity_ki:
    case local_position_ki: pid.ki = value; return;
    default: pid.kd = value; return;
    }
}

// 关节几何: 参数只给一个端点时, 另一端沿用当前生效值, 保证区间始终成对有效.
[[nodiscard]] bool merge_joint_geometry(const JointGeometryConfig& current, std::uint16_t local,
    float value, JointGeometryConfig& out) noexcept {
    out = current;
    switch (local) {
    case local_joint_zero: out.zero_offset_rad = value; return true;
    case local_joint_min: out.min_rad = value; break;
    case local_joint_max: out.max_rad = value; break;
    default: return false;
    }
    if (out.min_rad && out.max_rad && !(*out.min_rad < *out.max_rad)) { return false; }
    return true;
}

[[nodiscard]] bool is_joint_geometry_param(std::uint16_t local) noexcept {
    return local == local_joint_zero || local == local_joint_min || local == local_joint_max;
}

// 本版是否实现了该参数的在线生效路径. 未实现的参数一律先报 unsupported:
// 让调用方为了一个根本不会生效的参数去停止机构是没有意义的.
[[nodiscard]] bool has_apply_path(const SentinelApp& app, std::size_t slot,
    std::uint16_t local) noexcept {
    if (slot < dji_slot_count) {
        const auto& dji = app.config().dji[slot];
        if (!dji.installed) { return false; }
        if (local == local_velocity_kp || local == local_velocity_ki || local == local_velocity_kd) {
            return dji.velocity_pid.has_value();
        }
        if (local == local_position_kp || local == local_position_ki || local == local_position_kd) {
            return dji.position_pid.has_value();
        }
        return local == local_current_limit || is_joint_geometry_param(local);
    }
    if (slot == dji_slot_count) {
        const auto& dm = app.config().dm[0];
        if (!dm.installed) { return false; }
        if (local == 1 || local == 2) { return dm.mit_kp.has_value() && dm.mit_kd.has_value(); }
        return is_joint_geometry_param(local);
    }
    if (slot == domain_count) {
        return local == local_vehicle_yaw_alignment || local == local_vehicle_target_ttl;
    }
    return false;
}
} // 内部辅助

bool param_has_apply_path(const SentinelApp& app, std::uint16_t id) noexcept {
    return has_apply_path(app, static_cast<std::size_t>(id >> 8), static_cast<std::uint16_t>(id & 0xffu));
}

ParamWriteResult apply_param(ParamStore& params, SentinelApp& app, SentinelExecutor* executor,
    SentinelPerception* perception, std::uint16_t id, float value, bool idle) noexcept {
    const auto* spec = params.find(id);
    if (spec == nullptr) { return ParamWriteResult::unknown_param; }
    if (!std::isfinite(value) || value < spec->min_value || value > spec->max_value) {
        return ParamWriteResult::invalid_value;
    }
    // 先判断本版是否实现了在线生效路径, 再要求机构空闲: 顺序反过来会把
    // "未实现"报成"参考在用", 使调用方白白停一次机构.
    if (!param_has_apply_path(app, id)) {
        return ParamWriteResult::unsupported;
    }
    // 参考类参数改变参考语义: 受影响机构必须先退出执行. 全车参数由调用方按所有域判定.
    if (spec->kind == ParamKind::reference && !idle) {
        return ParamWriteResult::reference_in_use;
    }
    const auto slot = static_cast<std::size_t>(id >> 8);
    const auto local = static_cast<std::uint16_t>(id & 0xFFu);
    // 实际生效值: 成功路径一律从运行对象读回, 不用写入值直接充当读回值.
    float actual = value;

    if (slot < dji_slot_count) {
        const auto& dji = app.config().dji[slot];
        if (!dji.installed) { return ParamWriteResult::not_configured; }
        auto* axis = executor != nullptr ? executor->axis(dji.domain) : nullptr;
        if (local == local_velocity_kp || local == local_velocity_ki || local == local_velocity_kd) {
            if (axis == nullptr || !dji.velocity_pid) { return ParamWriteResult::unsupported; }
            auto pid = axis->active_velocity_pid();
            set_gain(pid, local, value);
            if (!axis->set_velocity_gains(pid)) { return ParamWriteResult::invalid_value; }
            actual = gain_of(axis->active_velocity_pid(), local);
        } else if (local == local_position_kp || local == local_position_ki ||
                   local == local_position_kd) {
            if (axis == nullptr || !dji.position_pid) { return ParamWriteResult::unsupported; }
            auto pid = axis->position_pid();
            set_gain(pid, local, value);
            if (!axis->set_position_gains(pid)) { return ParamWriteResult::invalid_value; }
            actual = gain_of(axis->position_pid(), local);
        } else if (local == local_current_limit) {
            if (axis == nullptr) { return ParamWriteResult::unsupported; }
            if (!axis->set_current_limit(value)) { return ParamWriteResult::invalid_value; }
            actual = axis->current_limit();
        } else if (local == local_joint_zero || local == local_joint_min ||
                   local == local_joint_max) {
            JointGeometryConfig next{};
            if (!merge_joint_geometry(app.joint_geometry(dji.domain), local, value, next)) {
                return ParamWriteResult::invalid_value;
            }
            if (executor != nullptr && !executor->set_joint_geometry(dji.domain, next)) {
                return ParamWriteResult::unsupported;
            }
            const auto old_zero = app.joint_geometry(dji.domain).zero_offset_rad;
            if (!app.set_joint_geometry(dji.domain, next)) {
                return ParamWriteResult::invalid_value;
            }
            if (perception && old_zero && next.zero_offset_rad) {
                perception->shift_joint_zero(dji.domain, *old_zero-*next.zero_offset_rad);
            }
            // 读回取自应用层这一份权威配置, 不是刚才构造的局部量.
            const auto& live = app.joint_geometry(dji.domain);
            actual = local == local_joint_zero ? live.zero_offset_rad.value_or(value)
                                               : (local == local_joint_min
                                                         ? live.min_rad.value_or(value)
                                                         : live.max_rad.value_or(value));
        } else {
            // 电流/输出编码上限、反馈时限与轴侧量程上限参与设备与轴的构造期换算,
            // 尚未实现不重建对象的生效路径: 明确上报, 不谎报 effective.
            return ParamWriteResult::unsupported;
        }
    } else if (slot == dji_slot_count) {
        const auto& dm = app.config().dm[0];
        if (!dm.installed) { return ParamWriteResult::not_configured; }
        if (local == 1 || local == 2) {
            if (executor == nullptr || !dm.mit_kp || !dm.mit_kd) {
                return ParamWriteResult::unsupported;
            }
            const float kp = local == 1 ? value : executor->dm_mit_kp();
            const float kd = local == 2 ? value : executor->dm_mit_kd();
            if (!executor->set_dm_mit_gains(kp, kd)) { return ParamWriteResult::invalid_value; }
            actual = local == 1 ? executor->dm_mit_kp() : executor->dm_mit_kd();
        } else if (local == local_joint_zero || local == local_joint_min ||
                   local == local_joint_max) {
            JointGeometryConfig next{};
            if (!merge_joint_geometry(app.joint_geometry(dm.domain), local, value, next)) {
                return ParamWriteResult::invalid_value;
            }
            if (executor != nullptr && !executor->set_joint_geometry(dm.domain, next)) {
                return ParamWriteResult::unsupported;
            }
            const auto old_zero = app.joint_geometry(dm.domain).zero_offset_rad;
            if (!app.set_joint_geometry(dm.domain, next)) {
                return ParamWriteResult::invalid_value;
            }
            if (perception && old_zero && next.zero_offset_rad) {
                perception->shift_joint_zero(dm.domain, *old_zero-*next.zero_offset_rad);
            }
            const auto& live = app.joint_geometry(dm.domain);
            actual = local == local_joint_zero ? live.zero_offset_rad.value_or(value)
                                               : (local == local_joint_min
                                                         ? live.min_rad.value_or(value)
                                                         : live.max_rad.value_or(value));
        } else {
            return ParamWriteResult::unsupported; // DM 反馈时限: 同上
        }
    } else if (slot == domain_count) {
        if (local == local_vehicle_yaw_alignment) {
            if (perception == nullptr) { return ParamWriteResult::unsupported; }
            // 惯性对齐偏移与机械零位是两件事: 这里只改惯性参考与 odom 的偏移.
            // 对齐语义只影响主 yaw: 只作废该域目标/保持并上推该域与全局代次,
            // 不把底盘等其它合法目标顺带停掉.
            perception->set_yaw_alignment(value, app.reference_epoch() + 1);
            app.invalidate_domain_reference(Domain::main_yaw);
            actual = perception->yaw_alignment_rad();
        } else if (local == local_vehicle_target_ttl) {
            const auto ttl_ms = static_cast<std::uint32_t>(value);
            if (!app.set_target_ttl(Lifetime{ttl_ms})) { return ParamWriteResult::invalid_value; }
            actual = static_cast<float>(app.target_ttl().ms);
        } else {
            return ParamWriteResult::unsupported;
        }
    } else {
        return ParamWriteResult::unknown_param;
    }

    // 落到运行对象之后才写参数存储: 失败路径不留下"新数值已写入"的假象.
    params.set_effective(id, actual);
    return params.write(id, actual, true);
}
} // 命名空间 ec::sentinel::app
