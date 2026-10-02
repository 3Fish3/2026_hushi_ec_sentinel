#pragma once
// 云台/发射独立调试: 地址与撤输出责任保留整车, 只配齐本次测试机构.
#include "sentinel_config.hpp"

namespace ec::sentinel::board {
[[nodiscard]] constexpr app::SentinelConfig mechanism_commissioning_config(
    app::SentinelConfig config, bool gimbal) noexcept {
    constexpr std::array<std::uint8_t, 9> buses{{2,2,2,2,1,1,2,2,2}};
    constexpr std::array<std::uint8_t, 9> ids{{1,2,3,4,4,6,5,6,7}};
    for (std::size_t i=0; i<config.dji.size(); ++i) {
        auto& slot=config.dji[i];
        slot.domain=static_cast<app::Domain>(i<6 ? i : i+1);
        slot.installed=true; slot.origin=app::Origin::user_confirmed;
        slot.bus=buses[i]; slot.device_id=ids[i];
        slot.mode=(i==4 || i==5) ? drivers::DjiMode::gm6020_current :
            i==8 ? drivers::DjiMode::c610_current : drivers::DjiMode::c620_current;
    }
    auto& pitch=config.dm[0];
    pitch.domain=app::Domain::pitch; pitch.installed=true; pitch.origin=app::Origin::user_confirmed;
    pitch.bus=1; pitch.can_id=1; pitch.master_id=0;
    pitch.limits=drivers::DmMitLimits{12.5f,30.0f,10.0f}; pitch.feedback_ttl=Lifetime{100};
    if (gimbal) {
        for (std::size_t i=4; i<=5; ++i) {
            auto& slot=config.dji[i];
            slot.rotor_per_axis=1.0f; slot.direction=1;
            slot.current_limit_a=2.5f; slot.output_limit_raw=16384;
            slot.max_rotor_speed_rad_s=60.0f; slot.max_axis_velocity_rad_s=15.0f;
            slot.feedback_ttl=Lifetime{50};
            // 当前GM6020台架已通过初值, 仅作带载待调起点; 两yaw使用同类五点测速.
            slot.velocity_pid=control::PidConfig{0.06f/2.5f,1.0f/2.5f,0,1,-1,1};
            slot.position_pid=control::PidConfig{7.5f,0,0,15,-15,15};
        }
        constexpr float tau=6.2831853071795864769f;
        config.joints[0]={app::Domain::main_yaw,1027.0f*tau/8192.0f,{},{},false,true};
        config.joints[1]={app::Domain::fire_yaw,2017.0f*tau/8192.0f,{},{},false,true};
        config.joints[2]={app::Domain::pitch,34282.0f*25.0f/65535.0f-12.5f,{},{},false,true};
        // 使用已通过四电机台架的MIT初值, 不是重力负载调参完成的承诺.
        pitch.mit_kp=35.0f/4095.0f*500.0f; pitch.mit_kd=0.0f;
    } else {
        config.shooter_commissioning=true;
        for (std::size_t i=6; i<9; ++i) {
            auto& slot=config.dji[i];
            slot.direction=1; slot.feedback_ttl=Lifetime{50};
            slot.rotor_per_axis=i==8 ? 36.0f : 1.0f;
            slot.current_limit_a=i==8 ? 8.0f : 6000.0f*20.0f/16384.0f;
            slot.output_limit_raw=i==8 ? 10000 : 16384;
            slot.max_rotor_speed_rad_s=1000.0f;
            slot.max_axis_velocity_rad_s=1000.0f/ *slot.rotor_per_axis;
            // M3508用本次已通过台架的19:1轴侧增益换算到直驱转子侧, 不抄旧摩擦轮D项.
            const float kp=i==8 ? 0.618794f : 2.657763f/19.0f;
            slot.velocity_pid=control::PidConfig{kp/ *slot.current_limit_a,0,0,1,-1,1};
        }
    }
    return config;
}
} // 命名空间 ec::sentinel::board
