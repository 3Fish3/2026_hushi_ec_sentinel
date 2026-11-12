#pragma once
// 正式 APP 的部署配置表 (O6).
// 本文件只使用 ec::sentinel::app 的配置类型, 不含 HAL/板级类型, 因此同一份表既能编进
// F407 固件, 也能被主机测试与离线检查脚本直接核对, 避免"测试用的表"和"烧录的表"分叉.
//
// 当前状态: **默认实车执行配置仍未回填**. 已有实车CAN身份、DM量程与人工粗零位记录,
// 不再把这些全部列为未知; 但关节方向/实际限位、轮几何和可执行参数尚未组成完整配置.
// 默认deployment_config()因此只声明配置类别与软件时序策略, 不装配设备槽.
// 台架通过不使默认实车自动具备执行资格; 缺项按当次机构闭合, 不要求无关机构先验收.
// 合成夹具 (app::synthetic_config()) 只用于主机离线测试, 绝不作为本镜像的默认配置.
#include "sentinel_config.hpp"
#include "sentinel_remote.hpp"
#include "board_app_bench_config.hpp" // IWYU pragma: keep (used in bench builds)
#include "board_app_chassis_config.hpp" // IWYU pragma: keep (used in chassis builds)
#include "board_app_mechanism_config.hpp"
#ifndef EC_APP_GIMBAL_COMMISSIONING
#define EC_APP_GIMBAL_COMMISSIONING 0
#endif
#ifndef EC_APP_GIMBAL_REMOTE_VERIFY
#define EC_APP_GIMBAL_REMOTE_VERIFY 0
#endif
#ifndef EC_APP_SHOOTER_COMMISSIONING
#define EC_APP_SHOOTER_COMMISSIONING 0
#endif
#ifndef EC_APP_CHASSIS_COMMISSIONING
#define EC_APP_CHASSIS_COMMISSIONING 0
#endif
#ifndef EC_APP_FOUR_MOTOR_BENCH
#define EC_APP_FOUR_MOTOR_BENCH 0
#endif
#ifndef EC_APP_GM6020_BENCH
#define EC_APP_GM6020_BENCH 0
#endif
#ifndef EC_APP_PITCH_IDENTIFICATION
#define EC_APP_PITCH_IDENTIFICATION 0
#endif
#if EC_APP_PITCH_IDENTIFICATION
#if !EC_APP_GIMBAL_COMMISSIONING
#error "Pitch identification is only supported in the gimbal APP role"
#endif
#ifndef EC_APP_PITCH_POLICY_HEADER
#error "Supply a reviewed physical policy header; there are no deployment defaults"
#endif
#include EC_APP_PITCH_POLICY_HEADER
#endif
#include <array>
#include <bit>
#include <limits>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ec::sentinel::board {
// 已确认接线 (2026-10-27 实车只读采集与用户接线确认, 与 board_observation.hpp 的 dji_mode
// 一致): CAN1 四轮 C620 (反馈 0x201..0x204) 与主 yaw GM6020 (反馈 0x208); CAN2 发射 yaw
// GM6020 (0x20a)、两摩擦轮 C620 (0x205/0x206) 与 ID7 的 C610 (0x207).
// 这里只记录已验证的总线/协议/设备 ID, 不包含机械参考和控制参数.
// CAN2 的轮序/方向等仍需冻结; 四电机裸台架改用的CAN1 M2006 ID3不替代实车CAN2 ID7.
struct HarnessEntry {
    std::uint8_t bus{};
    drivers::DjiMode mode{drivers::DjiMode::c620_current};
    std::uint8_t device_id{};
};
inline constexpr std::array<HarnessEntry, 9> known_dji_harness{{
    {1, drivers::DjiMode::c620_current, 1},    // 反馈 0x201
    {1, drivers::DjiMode::c620_current, 2},    // 反馈 0x202
    {1, drivers::DjiMode::c620_current, 3},    // 反馈 0x203
    {1, drivers::DjiMode::c620_current, 4},    // 反馈 0x204
    {1, drivers::DjiMode::gm6020_current, 4},  // 反馈 0x208
    {2, drivers::DjiMode::gm6020_current, 6},  // 反馈 0x20a
    {2, drivers::DjiMode::c620_current, 5},    // 反馈 0x205
    {2, drivers::DjiMode::c620_current, 6},    // 反馈 0x206
    {2, drivers::DjiMode::c610_current, 7},    // 反馈 0x207
}};
// 云台测速复用已确认接线表，避免测速与设备接收总线分别硬编码。
[[nodiscard]] constexpr std::optional<std::size_t> gimbal_velocity_slot(
    std::uint8_t bus, std::uint32_t id, std::uint8_t size) noexcept {
    if (size != 8) { return std::nullopt; }
    for (std::size_t i = 0; i < 2; ++i) {
        const auto& entry = known_dji_harness[i + 4];
        if (bus == entry.bus && id == 0x204u + entry.device_id) { return i; }
    }
    return std::nullopt;
}
// 实车记录已读回DM CAN2控制ID1/反馈ID0x011, MIT量程12.5rad/30rad/s/10N·m.
// 这里仍只声明接线存在; 协议量程不等于pitch机械行程或可直接部署的控制参数.
inline constexpr bool known_dm_harness = true;

// 软件时序策略 (不是实测机械/电气参数): 控制周期由板级 1 kHz 任务决定, 遥控/IMU 时效
// 沿用板级既有口径 (board_inputs.hpp), 目标与上位机链路时效是首版策略默认值, 待与
// 上位机回传率核对后确认.
inline constexpr float app_control_dt_s = 0.001f;
// 单次控制步长的上限 (ms): 超过它按超期计数; 真实 dt 仍照传给轴/环, 由它们既有的
// max_dt 拒绝路径处理, 不用标称周期冒充正常采样.
inline constexpr std::uint32_t app_max_dt_ms = 5;

// 单个控制周期的真实步长决策 (纯函数, 便于用受控时钟覆盖边界).
//  - 首次周期: 用明确的初始化步长 (不把合法 Tick=0 当哨兵);
//  - 重复时刻 (elapsed==0) 或时间回退 (半个回绕窗口外的倒退): dt_s = 0,
//    交给轴/环已有的 dt 拒绝/不推进路径, 不伪装成标称周期;
//  - 否则用真实毫秒差, 并单独给出超期标记供统计.
struct AppControlStep {
    float dt_s{app_control_dt_s};
    bool overrun{};
};
[[nodiscard]] constexpr AppControlStep app_control_step(
    std::uint32_t now_ms, std::uint32_t last_ms, bool have_last) noexcept {
    AppControlStep out{};
    if (!have_last) { return out; }
    const std::uint32_t elapsed_ms = now_ms - last_ms;
    if (elapsed_ms == 0u || elapsed_ms >= 0x80000000u) {
        out.dt_s = 0.0f;
        return out;
    }
    out.overrun = elapsed_ms > app_max_dt_ms;
    out.dt_s = static_cast<float>(elapsed_ms) * 0.001f;
    return out;
}

// 正式 APP 控制使用微秒差; 毫秒版本保留给已有观测/离线入口.
[[nodiscard]] constexpr AppControlStep app_control_step_us(
    std::uint32_t now_us, std::uint32_t last_us, bool have_last) noexcept {
    if (!have_last) { return {}; }
    const auto elapsed = now_us - last_us;
    if (elapsed == 0 || elapsed >= 0x80000000u) { return {0.0f, false}; }
    return {static_cast<float>(elapsed) * 1e-6f, elapsed > app_max_dt_ms * 1000u};
}

// 本地操纵量程当前为0, 表示未配置, 不是已标定的保守值. 用户已确认拨轮不可用,
// 新组合拨杆映射与小陀螺测试尚待实现/冻结; 不继续把旧ch4三层实测作为上车前提.
// 操纵速度、方向和具体档位须按新方案回填, 不在这里用任意值冒充实测基线.
[[nodiscard]] constexpr app::RemoteMappingConfig deployment_mapping() noexcept {
    app::RemoteMappingConfig mapping{};
#if EC_APP_GIMBAL_REMOTE_VERIFY
    mapping.layout = app::RemoteLayout::gimbal_validation;
    // 遥控验证的待调速率策略, 不是机械实测额定值; 不改 PID/零位/反馈量程.
    mapping.stick_deadzone = 0.05f;
    mapping.yaw_rate_max_rad_s = 0.5f;
    // 2026-10-29 pitch满杆目标速率由0.1提到0.6rad/s, 为原目标速率6倍.
    // 旧速度测试平均值含到位后的保持时间, 不可当作恒速能力或控制带宽.
    mapping.pitch_rate_max_rad_s = 0.6f;
#elif EC_APP_GIMBAL_COMMISSIONING || EC_APP_SHOOTER_COMMISSIONING
    mapping.layout = app::RemoteLayout::mechanism_commissioning;
#elif EC_APP_CHASSIS_COMMISSIONING
    mapping.layout = app::RemoteLayout::chassis_commissioning;
    mapping.stick_deadzone = 0.05f; // 初始操纵死区策略, 不是实测手柄标定值
    mapping.chassis_vx_max_mps = chassis_remote_translation_mps;
    mapping.chassis_vy_max_mps = chassis_remote_translation_mps;
    mapping.chassis_wz_max_rad_s = chassis_remote_rotation_rad_s;
#endif
    return mapping;
}
inline constexpr std::uint32_t app_target_ttl_ms = 100;
inline constexpr std::uint32_t app_host_link_ttl_ms = 200;
inline constexpr float app_max_dt_s = 0.005f;

[[nodiscard]] constexpr app::SentinelConfig with_pitch_identification(app::SentinelConfig config) noexcept {
#if EC_APP_PITCH_IDENTIFICATION
    constexpr auto policy = trusted_pitch_identification_policy;
    static_assert(policy.require_capture && policy.deployment_opt_in,
        "Deployment identification must explicitly require capture and opt in");
    static_assert(policy.min_rad < policy.max_rad && policy.max_amplitude_nm > 0 &&
        policy.max_expected_torque_nm >= policy.max_amplitude_nm && policy.max_velocity_rad_s > 0 &&
        policy.max_tracking_error_rad > 0 && policy.min_half_period_us > 0 &&
        policy.max_duration_us > 0 && policy.max_duration_us < 0x80000000u &&
        policy.max_feedback_age_us > 0 && policy.max_feedback_age_us < 0x80000000u &&
        policy.max_step_us > 0 && policy.max_step_us <= app_max_dt_ms * 1000u,
        "Invalid trusted Pitch policy");
    static_assert(policy.min_rad >= -std::numeric_limits<float>::max() &&
        policy.max_rad <= std::numeric_limits<float>::max() &&
        policy.max_expected_torque_nm <= std::numeric_limits<float>::max() &&
        policy.max_velocity_rad_s <= std::numeric_limits<float>::max() &&
        policy.max_tracking_error_rad <= std::numeric_limits<float>::max(),
        "Pitch policy bounds must be finite");
    config.pitch_identification = policy;
#endif
    return config;
}

// 部署配置: 默认只声明类别/时序, 机构角色按独立配置补齐装配槽.
[[nodiscard]] constexpr app::SentinelConfig deployment_config() noexcept {
    app::SentinelConfig config{};
    config.kind = app::ConfigKind::real_deployment;
    config.timing.control_dt_s = app_control_dt_s;
    config.timing.target_ttl = Lifetime{app_target_ttl_ms};
    config.timing.remote_ttl = Lifetime{100};
    config.timing.imu_ttl = Lifetime{50};
    config.timing.host_link_ttl = Lifetime{app_host_link_ttl_ms};
    config.timing.max_dt_s = app_max_dt_s;
#if EC_APP_GIMBAL_COMMISSIONING || EC_APP_SHOOTER_COMMISSIONING
    return with_pitch_identification(mechanism_commissioning_config(config, EC_APP_GIMBAL_COMMISSIONING != 0));
#elif EC_APP_CHASSIS_COMMISSIONING
    return chassis_commissioning_config(config);
#elif EC_APP_FOUR_MOTOR_BENCH
    return four_motor_bench_config(config);
#elif EC_APP_GM6020_BENCH
    return single_gm6020_bench_config(config);
#else
    return config;
#endif
}

// 部署配置的静态摘要: 固定布局、无指针, 供离线检查脚本从未压缩镜像按符号读取,
// 核对镜像里不是合成配置、也不含"装配了但来源未知"的槽位.
inline constexpr std::uint32_t deployment_summary_dji_slots = 9;
struct DeploymentSummary {
    std::uint32_t magic;
    std::uint32_t layout;
    std::uint32_t kind;              // app::ConfigKind 的数值
    std::uint32_t dji_capacity;      // 配置可声明的 DJI 槽数
    std::uint32_t dm_capacity;
    std::uint32_t installed_dji;     // 已声明装配的 DJI 槽数
    std::uint32_t installed_dm;
    std::uint32_t synthetic_values;  // 部署配置里的合成来源值 (必须为 0)
    std::uint32_t unknown_origin_installed; // 装配但来源未知 (部署用途拒绝)
    std::uint32_t harness_dji;       // 已验证接线条目数
    std::uint32_t harness_dm;
    // 每槽实际总线/设备 ID: bus | device_id<<8, 0 表示该槽未装配.
    // 只报装配数量时, "底盘挂错总线"这类镜像照样能通过审计; 地址进摘要才能离线核对.
    std::uint32_t dji_address[deployment_summary_dji_slots];
    std::uint32_t dm_address;        // bus | can_id<<8 | master_id<<16
};
inline constexpr std::uint32_t deployment_summary_magic = 0x45434150u; // "ECAP"
inline constexpr std::uint32_t deployment_summary_layout = 2; // 2: 增补每槽地址表

[[nodiscard]] constexpr DeploymentSummary summarize(const app::SentinelConfig& config) noexcept {
    DeploymentSummary summary{deployment_summary_magic, deployment_summary_layout,
        static_cast<std::uint32_t>(config.kind),
        static_cast<std::uint32_t>(config.dji.size()),
        static_cast<std::uint32_t>(config.dm.size()), 0, 0, 0, 0,
        static_cast<std::uint32_t>(known_dji_harness.size()), known_dm_harness ? 1u : 0u,
        {}, 0}; // 地址表与 DM 地址先清零, 下面按实际装配填写
    std::uint32_t slot_index = 0;
    for (const auto& slot : config.dji) {
        const std::uint32_t index = slot_index++;
        if (!slot.installed) { continue; }
        ++summary.installed_dji;
        if (slot.origin == app::Origin::synthetic) { ++summary.synthetic_values; }
        if (slot.origin == app::Origin::unspecified) { ++summary.unknown_origin_installed; }
        if (index < deployment_summary_dji_slots) {
            summary.dji_address[index] = static_cast<std::uint32_t>(slot.bus) |
                (static_cast<std::uint32_t>(slot.device_id) << 8);
        }
    }
    for (const auto& slot : config.dm) {
        if (!slot.installed) { continue; }
        ++summary.installed_dm;
        if (slot.origin == app::Origin::synthetic) { ++summary.synthetic_values; }
        if (slot.origin == app::Origin::unspecified) { ++summary.unknown_origin_installed; }
    }
    if (!config.dm.empty() && config.dm[0].installed) {
        summary.dm_address = static_cast<std::uint32_t>(config.dm[0].bus) |
            (static_cast<std::uint32_t>(config.dm[0].can_id.value_or(0)) << 8) |
            (static_cast<std::uint32_t>(config.dm[0].master_id.value_or(0)) << 16);
    }
    return summary;
}

// Separate fixed-layout descriptor: do not silently change DeploymentSummary v2.
struct PitchDeploymentSummary {
    std::uint32_t magic, layout, enabled, require_capture;
    std::uint32_t min_rad, max_rad, max_amplitude_nm, max_expected_torque_nm;
    std::uint32_t max_velocity_rad_s, max_tracking_error_rad;
    std::uint32_t min_half_period_us, max_duration_us, max_feedback_age_us, max_step_us;
    std::uint32_t zero_offset_rad, soft_min_rad, soft_max_rad, dm_address;
    std::uint32_t target_ttl_ms, link_ttl_ms, record_bytes;
};
[[nodiscard]] constexpr PitchDeploymentSummary summarize_pitch(const app::SentinelConfig& config) noexcept {
    const auto p = config.pitch_identification.value_or(app::PitchIdentificationPolicy{});
    const auto& joint = config.joints[2];
    return {0x50494431u, 1u, p.deployment_opt_in ? 1u : 0u, p.require_capture ? 1u : 0u,
        std::bit_cast<std::uint32_t>(p.min_rad), std::bit_cast<std::uint32_t>(p.max_rad),
        std::bit_cast<std::uint32_t>(p.max_amplitude_nm), std::bit_cast<std::uint32_t>(p.max_expected_torque_nm),
        std::bit_cast<std::uint32_t>(p.max_velocity_rad_s), std::bit_cast<std::uint32_t>(p.max_tracking_error_rad),
        p.min_half_period_us, p.max_duration_us, p.max_feedback_age_us, p.max_step_us,
        std::bit_cast<std::uint32_t>(joint.zero_offset_rad.value_or(0.0f)),
        std::bit_cast<std::uint32_t>(joint.min_rad.value_or(0.0f)),
        std::bit_cast<std::uint32_t>(joint.max_rad.value_or(0.0f)), summarize(config).dm_address,
        config.timing.target_ttl.ms, config.timing.host_link_ttl.ms, 64u};
}
inline constexpr PitchDeploymentSummary pitch_deployment_summary = summarize_pitch(deployment_config());
static_assert(sizeof(PitchDeploymentSummary) == 21u * sizeof(std::uint32_t));
#if EC_APP_PITCH_IDENTIFICATION
static_assert(pitch_deployment_summary.enabled == 1 && pitch_deployment_summary.require_capture == 1);
static_assert(deployment_config().joints[2].min_rad && deployment_config().joints[2].max_rad &&
    trusted_pitch_identification_policy.min_rad >= *deployment_config().joints[2].min_rad &&
    trusted_pitch_identification_policy.max_rad <= *deployment_config().joints[2].max_rad,
    "Identification bounds must remain inside calibrated joint limits");
static_assert(trusted_pitch_identification_policy.max_expected_torque_nm <=
    deployment_config().dm[0].limits.value().torque_max_nm, "Pitch policy exceeds MIT torque encoding range");
#endif
inline constexpr DeploymentSummary deployment_summary = summarize(deployment_config());
static_assert(deployment_summary.magic == deployment_summary_magic);
static_assert(deployment_summary.kind == static_cast<std::uint32_t>(app::ConfigKind::real_deployment),
    "部署配置不得声明为合成夹具或未配置");
static_assert(deployment_summary.synthetic_values == 0, "部署配置不得含合成来源的值");
static_assert(deployment_summary.installed_dji == ((EC_APP_CHASSIS_COMMISSIONING || EC_APP_GIMBAL_COMMISSIONING || EC_APP_SHOOTER_COMMISSIONING) ? 9u : EC_APP_FOUR_MOTOR_BENCH ? 3u : EC_APP_GM6020_BENCH ? 1u : 0u) &&
    deployment_summary.installed_dm == ((EC_APP_CHASSIS_COMMISSIONING || EC_APP_GIMBAL_COMMISSIONING || EC_APP_SHOOTER_COMMISSIONING || EC_APP_FOUR_MOTOR_BENCH) ? 1u : 0u),
    "整车默认无装配槽; 台架装配数须与显式构建角色一致");
static_assert(deployment_summary.dji_capacity >= known_dji_harness.size(),
    "已验证接线条目数不得超过配置可声明的 DJI 槽数");
} // 命名空间 ec::sentinel::board

extern "C" {
// 部署摘要符号: 单一定义在包含本头文件的板级 TU (固件: board_app.cpp; 主机: 部署配置测试).
// 外部链接 + used 保证它被发出; 读取方必须经 volatile 指针读, 否则 Release 下引用会被
// 常量折叠, 链接器就会把符号 GC 掉 (镜像里就没有可读的部署摘要了).
// 因此本头文件不得被同一二进制里的多个 TU 同时包含.
extern const ec::sentinel::board::PitchDeploymentSummary ec_pitch_identification_summary;
[[gnu::used]] const ec::sentinel::board::PitchDeploymentSummary ec_pitch_identification_summary =
    ec::sentinel::board::pitch_deployment_summary;
extern const ec::sentinel::board::DeploymentSummary ec_app_deployment_summary;
[[gnu::used]] const ec::sentinel::board::DeploymentSummary ec_app_deployment_summary =
    ec::sentinel::board::deployment_summary;
}
