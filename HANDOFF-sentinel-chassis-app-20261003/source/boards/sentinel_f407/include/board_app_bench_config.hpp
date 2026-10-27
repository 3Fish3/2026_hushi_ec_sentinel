#pragma once
// 正式APP裸电机台架配置, 不代表实车部署或机械限位.
#include "sentinel_config.hpp"

namespace ec::sentinel::board {
[[nodiscard]] constexpr app::SentinelConfig single_gm6020_bench_config(app::SentinelConfig config) noexcept {
    auto& slot = config.dji[4];
    slot.domain = app::Domain::main_yaw;
    slot.installed = true;
    slot.origin = app::Origin::reference_converted;
    slot.bus = 1;
    slot.mode = drivers::DjiMode::gm6020_current;
    slot.device_id = 4;
    slot.rotor_per_axis = 1.0f;
    slot.direction = 1;
    // 用户2026-09-26确认的S5基线: 0.06/1.0/0 A口径, 外环7.5.
    // 保留2.5A、15rad/s与同一五点因果测速; 这是空载起点, 不是APP实机稳定性验收.
    slot.current_limit_a = 2.5f;
    slot.output_limit_raw = 16384;
    slot.max_rotor_speed_rad_s = 60.0f; // 与原S5设备解圈契约一致
    slot.max_axis_velocity_rad_s = 15.0f;
    slot.feedback_ttl = Lifetime{50};
    slot.velocity_pid = control::PidConfig{0.06f/2.5f, 1.0f/2.5f, 0.0f, 1.0f, -1.0f, 1.0f};
    slot.position_pid = control::PidConfig{7.5f, 0.0f, 0.0f, 0.0f, -15.0f, 15.0f};
    // 自由台架没有0/2π机械挡; 仅由用户显式align建立连续位置参考, 不自动捕获上电零点.
    config.joints[app::joint_index(app::Domain::main_yaw)] =
        app::JointGeometryConfig{app::Domain::main_yaw, 0.0f, {}, {}, true};
    return config;
}
// 用户确认全部CAN1: GM6020 ID4、M3508 ID2、M2006 ID3; 达妙 pitch 在 CAN2 控制ID1/反馈ID0x011.
// M2006不用ID7: 0x1FF电流组会被同总线GM6020识别为电压命令.
// M3508不用ID1: 0x201低8位会命中达妙控制ID1, 不能靠MCU收帧过滤解决.
[[nodiscard]] constexpr app::SentinelConfig four_motor_bench_config(app::SentinelConfig config) noexcept {
    config = single_gm6020_bench_config(config);
    config.independent_motor_bench = true;
    // M3508复用速度域, 并非真的摩擦轮; 不创建缺三轮的假底盘.
    auto& m3508 = config.dji[6];
    m3508.domain = app::Domain::friction_left;
    m3508.installed = true; m3508.origin = app::Origin::reference_converted;
    m3508.bus = 1; m3508.device_id = 2; m3508.mode = drivers::DjiMode::c620_current;
    m3508.rotor_per_axis = 19.0f; m3508.direction = 1;
    m3508.current_limit_a = 6000.0f*20.0f/16384.0f;
    m3508.output_limit_raw = 16384;
    // 1000rad/s是解圈工程上界、10rad/s是本台架操作量程, 均非实测性能结论.
    m3508.max_rotor_speed_rad_s = 1000.0f; m3508.max_axis_velocity_rad_s = 10.0f;
    m3508.feedback_ttl = Lifetime{50};
    // 旧底盘速度P=2 raw/(deg/s), 携带N=19和原6000raw限幅换算; 裸机仍待人工调参.
    m3508.velocity_pid = control::PidConfig{2.657763f / *m3508.current_limit_a, 0, 0, 1, -1, 1};
    auto& m2006 = config.dji[8];
    m2006.domain = app::Domain::trigger;
    m2006.installed = true; m2006.origin = app::Origin::reference_converted;
    m2006.bus = 1; m2006.device_id = 3; m2006.mode = drivers::DjiMode::c610_current;
    m2006.rotor_per_axis = 36.0f; m2006.direction = 1;
    m2006.current_limit_a = 8.0f; m2006.output_limit_raw = 10000;
    m2006.max_rotor_speed_rad_s = 1000.0f; m2006.max_axis_velocity_rad_s = 10.0f;
    m2006.feedback_ttl = Lifetime{50};
    m2006.velocity_pid = control::PidConfig{(0.017188734f*36.0f)/8.0f, 0, 0, 1, -1, 1};
    // 19/36仅按原装减速箱与旧实现换算, 不包含外部传动; 安装后人工核对方向与速度.
    auto& dm = config.dm[0];
    dm.domain = app::Domain::pitch; dm.installed = true; dm.origin = app::Origin::reference_converted;
    dm.bus = 2; dm.can_id = 1; dm.master_id = 0x011;
    dm.limits = drivers::DmMitLimits{12.5f, 30.0f, 10.0f};
    dm.feedback_ttl = Lifetime{100};
    // 旧有效MIT编码Kp=35、Kd=0换回物理增益, 不是直接将35当物理Kp; 本裸机待调.
    dm.mit_kp = 35.0f/4095.0f*500.0f; dm.mit_kd = 0.0f;
    // 自由DM使用原生绝对位置, 这里的上下界仅是已确认协议量程, 不是机构硬限位.
    config.joints[app::joint_index(app::Domain::pitch)] =
        app::JointGeometryConfig{app::Domain::pitch, 0.0f, -12.5f, 12.5f};
    return config;
}
} // 命名空间 ec::sentinel::board
