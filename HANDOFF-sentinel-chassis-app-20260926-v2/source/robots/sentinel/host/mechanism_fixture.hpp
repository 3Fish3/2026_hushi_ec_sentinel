#pragma once
// 仅主机离线夹具, 不是实车配置切换开关. 常规机构分支保留既有装配/地址,
// 未试机构缺参且无反馈; bench分支单独复用已确认四电机装配, 仍标为合成负载.
#include "sentinel_config.hpp"
#include "../../../boards/sentinel_f407/include/board_app_bench_config.hpp"
#include <optional>
#include <string_view>

namespace ec::sentinel::host {
enum class Mechanism { all, chassis, gimbal, shooter, bench };
[[nodiscard]] inline std::optional<Mechanism> mechanism(std::string_view name) noexcept {
    if (name == "bench") { return Mechanism::bench; }
    if (name == "all") { return Mechanism::all; }
    if (name == "chassis") { return Mechanism::chassis; }
    if (name == "gimbal") { return Mechanism::gimbal; }
    if (name == "shooter") { return Mechanism::shooter; }
    return std::nullopt;
}
[[nodiscard]] inline bool selected(Mechanism part, app::Domain domain) noexcept {
    return part == Mechanism::all ||
        (part == Mechanism::bench && (domain == app::Domain::main_yaw || domain == app::Domain::pitch ||
            domain == app::Domain::friction_left || domain == app::Domain::trigger)) ||
        (part == Mechanism::chassis && app::is_chassis(domain)) ||
        (part == Mechanism::gimbal && app::is_joint(domain)) ||
        (part == Mechanism::shooter && (domain == app::Domain::friction_left ||
            domain == app::Domain::friction_right || domain == app::Domain::trigger));
}
[[nodiscard]] inline app::SentinelConfig partial_fixture_config(Mechanism part) noexcept {
    auto config = app::synthetic_config();
    if (part == Mechanism::all) { return config; }
    if (part == Mechanism::bench) {
        for (auto& slot : config.dji) { slot.installed = false; }
        for (auto& slot : config.dm) { slot.installed = false; }
        config.serial.reset();
        config = board::four_motor_bench_config(config);
        // 只复用装配/控制链, 合成负载不构成实测参数, 候选不能写入物理部署.
        for (auto& slot : config.dji) { slot.origin = app::Origin::synthetic; }
        for (auto& slot : config.dm) { slot.origin = app::Origin::synthetic; }
        return config;
    }
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
