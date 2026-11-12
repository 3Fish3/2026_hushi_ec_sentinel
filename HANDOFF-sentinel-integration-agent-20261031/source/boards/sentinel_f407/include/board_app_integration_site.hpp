#pragma once
// 2026-11-03现场候选: 按11-03重录数据(对齐参考+训练/留出)重新拟合的C板几何。
// 输入比例仅作单位换算; 不修改已冻结PID、接线或协议输出范围。
#include "sentinel_remote.hpp"

namespace ec::sentinel::board {
// 用户授权采用保守初值; 单位分别为m/s、rad/s, 不是执行器运动限幅。
// 2026-11-03 用户要求底盘量程拉高: 按轮面线速度2.0m/s折算.
//   平移 2.0/cos45 = 2.828427 m/s; 转向/自旋 2.0/0.21607 = 9.256184 rad/s (约530 deg/s).
// 依据M3508 P19输出空载约49.1rad/s(轮面4.03m/s)的物理上限, 取约一半留出打滑与电流裕量;
// 这不是实测极限, 也不是执行器限幅. 云台三项main/fire/pitch速率保持0.8/0.7/0.5未变.
// 2026-11-04 现场实测: 指令2.130m/s 对应实际轮面1.589m/s(达成率约75%);
// 用户要求调低到1.0m/s, 满杆预计实际约0.75m/s.
inline constexpr float integration_translation_mps = 1.0f;
// 2026-11-04 现场要求普通转向与固定自旋都降到3 rad/s(约172 deg/s);
// 对应轮面切向速度 3*0.21607=0.648 m/s. 原值turn9.256184/spin6.0作废.
inline constexpr float integration_turn_rad_s = 3.0f;
inline constexpr float integration_spin_rad_s = 3.0f;
inline constexpr float integration_main_rate_rad_s = 0.8f;
inline constexpr float integration_fire_rate_rad_s = 0.7f;
inline constexpr float integration_pitch_rate_rad_s = 0.5f;

// V=S为2026-11-03T18:21:44Z对齐参考姿态的传感器系; 原始偏置在install前扣一次。
// 2026-11-03 C板更换后按新板重录(对齐×2/静止×2/三轴各×2)重新拟合, 旧板几何作废:
// 新旧main/fire轴的X分量符号相反, 说明新板IMU安装方向与旧板不同, 不可复用旧值.
// forward结合用户确认的水平/朝前姿态、静止加速度和正pitch抬头符号构造;
// 采用枪轴垂直pitch铰轴的粗机械假设, 不是独立光学测向或实车验收结果。
// fire/pitch参考已减去本版软件零偏; 不修改关节软件零点或电调硬件零点。
// 位置/速度时移选明确的零时移候选基线, 不把未辨识的优化时移当作实测延迟。
inline constexpr std::optional<app::SerialGeometry> integration_serial_geometry{
    app::SerialGeometry{
        {-0.086802847f, 0.031076549f, 0.995740686f},
        {0.014223269f, -0.999415767f, 0.031077689f},
        {-0.097089624f, 0.027570401f, 0.994893702f},
        {0.997511309f, 0.016329022f, 0.068589730f},
        {0.001770788f, -0.000808838f, 0.000463396f},
        estimation::identity_rotation(),
        -0.099708751f, -0.052262315f,
        {0, 0}, {0, 0},
        0.01f, 0.02f, 0.8f
    }
};

[[nodiscard]] constexpr app::RemoteMappingConfig integration_mapping() noexcept {
    app::RemoteMappingConfig out{};
    out.layout = app::RemoteLayout::chassis_gimbal_integration;
    out.stick_deadzone = 0.05f;
    out.chassis_vx_max_mps = out.chassis_vy_max_mps = integration_translation_mps;
    out.chassis_wz_max_rad_s = integration_turn_rad_s;
    out.spin_wz_rad_s = integration_spin_rad_s;
    out.yaw_rate_max_rad_s = integration_main_rate_rad_s;
    out.fire_yaw_rate_rad_s = integration_fire_rate_rad_s;
    out.pitch_rate_max_rad_s = integration_pitch_rate_rad_s;
    return out;
}
static_assert(integration_translation_mps >= 0 && integration_turn_rad_s >= 0 &&
    integration_main_rate_rad_s >= 0 && integration_fire_rate_rad_s >= 0 && integration_pitch_rate_rad_s >= 0);
} // 命名空间 ec::sentinel::board
