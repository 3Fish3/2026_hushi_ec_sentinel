#include "sentinel_params.hpp"
#include <cmath>

namespace ec::sentinel::app {
namespace {
// 域槽位: 底盘四轮 / 两 yaw / pitch / 摩擦两轮 / 拨弹 = 10, 另有整车槽 10.
constexpr std::size_t vehicle_slot = domain_count;

[[nodiscard]] bool finite_range(float low, float high) noexcept {
    return std::isfinite(low) && std::isfinite(high) && low <= high;
}
} // 内部辅助

ParamStore::ParamStore(const SentinelConfig& config) noexcept {
    const auto add = [&](std::size_t slot, ParamKind kind, Domain domain, bool whole,
                          std::uint16_t local, float value, float low, float high,
                          const char* name, const char* unit) {
        if (count_ >= entries_.size() || !finite_range(low, high)) { return; }
        auto& entry = entries_[count_++];
        entry.spec = ParamSpec{param_id(slot, local), kind, domain, whole, value, low, high, name,
            unit};
        entry.value = value;
        entry.overridden = false;
    };
    // 每个 DJI 域的热调参数: 速度环增益 + 电流/输出限幅 + 轴侧速度上限.
    const auto add_dji = [&](std::size_t slot, Domain domain, bool has_position) {
        const auto& dji = config.dji[slot];
        const float kp = dji.velocity_pid ? dji.velocity_pid->kp : 0.0f;
        const float ki = dji.velocity_pid ? dji.velocity_pid->ki : 0.0f;
        const float kd = dji.velocity_pid ? dji.velocity_pid->kd : 0.0f;
        // 未配置不是数值零, 不向调参端注册伪造的参数读回.
        if (dji.velocity_pid) {
            add(slot, ParamKind::hot, domain, false, 1, kp, -100.0f, 100.0f, "velocity_kp", "1");
            add(slot, ParamKind::hot, domain, false, 2, ki, -100.0f, 100.0f, "velocity_ki", "1");
            add(slot, ParamKind::hot, domain, false, 3, kd, -100.0f, 100.0f, "velocity_kd", "1");
        }
        // 电流/输出限幅与时限参与设备与轴的构造期换算, 不能在执行中改变 → 参考类.
        if (dji.current_limit_a) {
            add(slot, ParamKind::reference, domain, false, 4, *dji.current_limit_a, 0.0f,
                30.0f, "current_limit_a", "A");
        }
        if (dji.output_limit_raw) {
            add(slot, ParamKind::reference, domain, false, 5,
                static_cast<float>(*dji.output_limit_raw), 0.0f, 25000.0f, "output_limit_raw", "raw");
        }
        if (dji.max_axis_velocity_rad_s) {
            add(slot, ParamKind::reference, domain, false, 6,
                *dji.max_axis_velocity_rad_s, 0.0f, 200.0f, "max_axis_velocity_rad_s", "rad/s");
        }
        add(slot, ParamKind::reference, domain, false, 7, static_cast<float>(dji.feedback_ttl.ms),
            1.0f, 1000.0f, "feedback_ttl_ms", "ms");
        if (has_position && dji.position_pid) {
            add(slot, ParamKind::hot, domain, false, 8, dji.position_pid->kp, -100.0f, 100.0f,
                "position_kp", "1");
            add(slot, ParamKind::hot, domain, false, 9, dji.position_pid->ki, -100.0f, 100.0f,
                "position_ki", "1");
            add(slot, ParamKind::hot, domain, false, 10, dji.position_pid->kd, -100.0f, 100.0f,
                "position_kd", "1");
        }
        // 参考参数: 关节零位与机械范围 (只有关节域有).
        for (std::size_t i = 0; i < joint_slot_count; ++i) {
            if (config.joints[i].domain != domain) { continue; }
            const auto& joint = config.joints[i];
            if (!joint.zero_offset_rad) { continue; }
            add(slot, ParamKind::reference, domain, false, 20, *joint.zero_offset_rad, -3.2f, 3.2f,
                "zero_offset_rad", "rad");
            if (joint.min_rad && joint.max_rad) {
                add(slot, ParamKind::reference, domain, false, 21, *joint.min_rad, -3.2f, 3.2f,
                    "min_rad", "rad");
                add(slot, ParamKind::reference, domain, false, 22, *joint.max_rad, -3.2f, 3.2f,
                    "max_rad", "rad");
            }
        }
    };
    for (std::size_t i = 0; i < dji_slot_count; ++i) {
        if (!config.dji[i].installed) { continue; }
        add_dji(i, config.dji[i].domain, config.dji[i].position_pid.has_value());
    }
    if (config.dm[0].installed) {
        const auto slot = dji_slot_count; // DM 使用独立槽位, 避免与 DJI 槽冲突
        if (config.dm[0].mit_kp) {
            add(slot, ParamKind::hot, config.dm[0].domain, false, 1, *config.dm[0].mit_kp,
                0.0f, 500.0f, "mit_kp", "1");
        }
        if (config.dm[0].mit_kd) {
            add(slot, ParamKind::hot, config.dm[0].domain, false, 2, *config.dm[0].mit_kd,
                0.0f, 5.0f, "mit_kd", "1");
        }
        add(slot, ParamKind::reference, config.dm[0].domain, false, 3,
            static_cast<float>(config.dm[0].feedback_ttl.ms), 1.0f, 1000.0f, "feedback_ttl_ms",
            "ms");
        for (std::size_t i = 0; i < joint_slot_count; ++i) {
            if (config.joints[i].domain != config.dm[0].domain) { continue; }
            const auto& joint = config.joints[i];
            if (joint.zero_offset_rad) {
                add(slot, ParamKind::reference, config.dm[0].domain, false, 20, *joint.zero_offset_rad,
                    -3.2f, 3.2f, "zero_offset_rad", "rad");
            }
            if (joint.min_rad && joint.max_rad) {
                add(slot, ParamKind::reference, config.dm[0].domain, false, 21, *joint.min_rad, -3.2f,
                    3.2f, "min_rad", "rad");
                add(slot, ParamKind::reference, config.dm[0].domain, false, 22, *joint.max_rad, -3.2f,
                    3.2f, "max_rad", "rad");
            }
        }
    }
    // 保持原 ID, 对齐偏移的真实影响域仅为主 yaw, 不要求无关机构退出.
    add(vehicle_slot, ParamKind::reference, Domain::main_yaw, false, 1, 0.0f, -3.2f, 3.2f,
        "yaw_alignment_rad", "rad");
    add(vehicle_slot, ParamKind::reference, Domain::chassis_lf, true, 2,
        config.timing.target_ttl.ms ? static_cast<float>(config.timing.target_ttl.ms) : 0.0f, 1.0f,
        1000.0f, "target_ttl_ms", "ms");
}

const ParamSpec* ParamStore::find(std::uint16_t id) const noexcept {
    for (std::size_t i = 0; i < count_; ++i) {
        if (entries_[i].spec.id == id) { return &entries_[i].spec; }
    }
    return nullptr;
}

const ParamEntry* ParamStore::entry(std::uint16_t id) const noexcept {
    for (std::size_t i = 0; i < count_; ++i) {
        if (entries_[i].spec.id == id) { return &entries_[i]; }
    }
    return nullptr;
}

ParamEntry* ParamStore::find_mutable(std::uint16_t id) noexcept {
    for (std::size_t i = 0; i < count_; ++i) {
        if (entries_[i].spec.id == id) { return &entries_[i]; }
    }
    return nullptr;
}

ParamWriteResult ParamStore::write_entry(ParamEntry& entry, float value,
    bool domain_idle) noexcept {
    if (!std::isfinite(value) || value < entry.spec.min_value || value > entry.spec.max_value) {
        return ParamWriteResult::invalid_value;
    }
    // 参考类参数改变零位/机械范围语义: 受影响机构必须已退出执行.
    if (entry.spec.kind == ParamKind::reference && !domain_idle) {
        return ParamWriteResult::reference_in_use;
    }
    entry.value = value;
    entry.overridden = value != entry.spec.default_value;
    return ParamWriteResult::accepted;
}

ParamWriteResult ParamStore::write(std::uint16_t id, float value, bool domain_idle) noexcept {
    auto* entry = find_mutable(id);
    if (entry == nullptr) { return ParamWriteResult::unknown_param; }
    return write_entry(*entry, value, domain_idle);
}

ParamWriteResult ParamStore::reset_to_default(std::uint16_t id, bool domain_idle) noexcept {
    auto* entry = find_mutable(id);
    if (entry == nullptr) { return ParamWriteResult::unknown_param; }
    return write_entry(*entry, entry->spec.default_value, domain_idle);
}

void ParamStore::set_effective(std::uint16_t id, float value) noexcept {
    auto* entry = find_mutable(id);
    if (entry != nullptr) { entry->value = value; }
}
} // 命名空间 ec::sentinel::app
