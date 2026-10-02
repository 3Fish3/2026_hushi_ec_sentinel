#pragma once
// 仅主机离线缺项夹具, 不是实车配置切换开关. 所有已装槽与总线地址保持不变;
// 未试机构缺少运动参数且不产生反馈, 仍须按实际共享命令组明确撤输出.
#include "sentinel_config.hpp"
#include <optional>
#include <string_view>

namespace ec::sentinel::host {
enum class Mechanism { all, chassis, gimbal, shooter };
[[nodiscard]] inline std::optional<Mechanism> mechanism(std::string_view name) noexcept {
    if (name == "all") { return Mechanism::all; }
    if (name == "chassis") { return Mechanism::chassis; }
    if (name == "gimbal") { return Mechanism::gimbal; }
    if (name == "shooter") { return Mechanism::shooter; }
    return std::nullopt;
}
[[nodiscard]] inline bool selected(Mechanism part, app::Domain domain) noexcept {
    return part == Mechanism::all ||
        (part == Mechanism::chassis && app::is_chassis(domain)) ||
        (part == Mechanism::gimbal && app::is_joint(domain)) ||
        (part == Mechanism::shooter && (domain == app::Domain::friction_left ||
            domain == app::Domain::friction_right || domain == app::Domain::trigger));
}
[[nodiscard]] inline app::SentinelConfig partial_fixture_config(Mechanism part) noexcept {
    auto config = app::synthetic_config();
    if (part == Mechanism::all) { return config; }
    for (auto& slot : config.dji) {
        if (selected(part, slot.domain)) { continue; }
        slot.rotor_per_axis.reset();
        slot.velocity_pid.reset(); slot.position_pid.reset();
    }
    if (part != Mechanism::chassis) { config.wheels = {}; }
    if (part != Mechanism::gimbal) {
        config.joints = {}; config.serial.reset();
        config.dm[0].limits.reset(); config.dm[0].mit_kp.reset(); config.dm[0].mit_kd.reset();
    }
    return config;
}
[[nodiscard]] inline bool selected_feedback(Mechanism part, const app::SentinelConfig& config,
    std::uint8_t bus, std::uint16_t id) noexcept {
    for (const auto& slot : config.dji) {
        const auto protocol = drivers::dji_protocol(slot.mode);
        if (slot.installed && protocol && bus == slot.bus && id == protocol->rx_base+slot.device_id) {
            return selected(part, slot.domain);
        }
    }
    for (const auto& slot : config.dm) {
        if (slot.installed && bus == slot.bus && slot.master_id && id == *slot.master_id) {
            return selected(part, slot.domain);
        }
    }
    return false;
}
} // 命名空间 ec::sentinel::host
