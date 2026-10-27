#pragma once
// 实车底盘调试专用配置; CAD几何/用户确认传动与台架控制初值分别注明来源.
#include "sentinel_config.hpp"

namespace ec::sentinel::board {
// 2026-10-03操作员确认原生正转推车方向LF↙/RF↖/LB↘/RB↗: 左侧反向换算.
// direction同时作用于反馈与电流指令, 不改变PID符号或车体系几何.
inline constexpr std::array<int, 4> chassis_motor_directions{{-1, 1, -1, 1}};
inline constexpr float chassis_radius_m = 0.0819702695f;
inline constexpr float chassis_ratio = 19.0f; // 用户2026-09-26明确确认, 不取CAD引用名268:17
// 首轮操作量程是本调试版本策略, 不是实测极限或比赛性能承诺.
inline constexpr float chassis_remote_translation_mps = 0.4f;
inline constexpr float chassis_remote_rotation_rad_s = 0.8f;

[[nodiscard]] constexpr app::SentinelConfig chassis_commissioning_config(
    app::SentinelConfig config) noexcept {
    // 保留整车已安装地址, 未调机构不创建控制器, 由执行器持续提交明确停止帧.
    constexpr std::array<std::uint8_t, 9> buses{{1, 1, 1, 1, 1, 2, 2, 2, 2}};
    constexpr std::array<std::uint8_t, 9> ids{{1, 2, 3, 4, 4, 6, 5, 6, 7}};
    for (std::size_t i = 0; i < config.dji.size(); ++i) {
        auto& slot = config.dji[i];
        slot.domain = static_cast<app::Domain>(i < 6 ? i : i + 1);
        slot.installed = true; slot.origin = app::Origin::user_confirmed;
        slot.bus = buses[i]; slot.device_id = ids[i];
        slot.mode = (i == 4 || i == 5) ? drivers::DjiMode::gm6020_current
            : i == 8 ? drivers::DjiMode::c610_current : drivers::DjiMode::c620_current;
    }
    constexpr std::array<std::array<float, 2>, 4> centers{{
        {{0.1525495102f, 0.1530217415f}}, {{0.1531000333f, -0.1523929265f}},
        {{-0.1531783251f, 0.1524712183f}}, {{-0.1524712183f, -0.1531000333f}}}};
    constexpr std::array<float, 4> angles{{-0.7853981634f, 0.7853981634f,
        0.7853981634f, -0.7853981634f}};
    for (std::size_t i = 0; i < 4; ++i) {
        auto& slot = config.dji[i];
        slot.origin = app::Origin::reference_converted;
        slot.rotor_per_axis = chassis_ratio; slot.direction = chassis_motor_directions[i];
        // 2026-10-27 架空单轮阶跃实测: 纯P的稳态误差恒为0.202 rad/s(齿轮箱静摩擦, 与目标无关),
        // 加积分后3/6 rad/s静差降到0.6%/0.06%; 上升时间受电流上限限制, 不靠加大P提速.
        slot.current_limit_a = 6000.0f * 20.0f / 16384.0f;
        slot.output_limit_raw = 16384;
        slot.max_rotor_speed_rad_s = 1000.0f;
        slot.max_axis_velocity_rad_s = 10.0f;
        slot.feedback_ttl = Lifetime{50};
        slot.velocity_pid = control::PidConfig{3.5f / *slot.current_limit_a,
            10.0f / *slot.current_limit_a, 0.0f, 1.0f, -1.0f, 1.0f};
        config.wheels[i] = app::WheelGeometryConfig{centers[i][0], centers[i][1],
            angles[i], chassis_radius_m, 1.0f};
    }
    auto& pitch = config.dm[0];
    pitch.domain = app::Domain::pitch; pitch.installed = true;
    pitch.origin = app::Origin::user_confirmed; pitch.bus = 2;
    pitch.can_id = 1; pitch.master_id = 0x011;
    pitch.limits = drivers::DmMitLimits{12.5f, 30.0f, 10.0f};
    pitch.feedback_ttl = Lifetime{100};
    // 不提供MIT增益或机械参考, 不授予pitch驱动资格; 调试上电保持既有失能/只读策略.
    return config;
}
} // 命名空间 ec::sentinel::board
