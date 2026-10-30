#pragma once
// 实验车 Pitch 方波辨识的可信策略 (仅在 -DEC_APP_PITCH_IDENTIFICATION=ON 时编入).
//
// 这些是"允许的上限", 不是推荐运行值: 主机每次试验仍须在以下范围内显式给出幅度、
// 半周期、次数与保护阈值. 上下限取自 board_app_mechanism_config.hpp 的实车标定
// (pitch_min=-0.2727550, pitch_max=+0.4362448), 并进一步向内收窄, 避免辨识激励
// 把云台推到机械止挡附近.
// 力矩、速度、时长等数值是待验证的试验草稿, 尚未实车测定, 不作为已标定额定值.
//
// 实验车可承受损坏, 但这里的力矩/速度/误差上限仍然生效: 一旦超限固件会锁存 fault
// 并撤销输出, 不会因为"允许损坏"而放宽编码范围.

namespace ec::sentinel::board {

inline constexpr ec::sentinel::app::PitchIdentificationPolicy trusted_pitch_identification_policy{
    // 收进实车操作带[command_min=-0.132, command_max=+0.235], 激励目标也不得越出摩擦轮可用范围.
    .min_rad = -0.132f,
    .max_rad = 0.235f,
    .max_amplitude_nm = 0.05f,
    .max_expected_torque_nm = 3.0f,
    .max_velocity_rad_s = 1.5f,
    .max_tracking_error_rad = 0.03f,
    .min_half_period_us = 20000u,
    .max_duration_us = 20000000u,
    .max_feedback_age_us = 20000u,
    .max_step_us = 3000u,
    .require_capture = true,
    .deployment_opt_in = true,
};

} // namespace ec::sentinel::board
