#pragma once
// 哨兵正式 APP 的配置边界 (O0):
//  - 安装声明 (总线/协议/设备 ID) 与机械/电气换算参数分开, 缺项用 nullopt 表达;
//  - 合成测试夹具与真实部署配置是两种显式类别, 不得静默混用 (见 validate/config_audience);
//  - 只做配置校验与类型转换, 不构造设备对象、不发送任何帧.
// 旧仓库有效参数到新域的换算依据和仍然缺项的内容集中记录在
// docs/robots/sentinel/应用配置与换算基线.md, 本文件不重复维护第二份数值表.
#include "ec/control/pid.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/drivers/dm_device.hpp"
#include "ec/motion/omni_kinematics.hpp"
#include "sentinel_types.hpp"
#include "sentinel_runtime.hpp"
#include "sentinel_kinematics.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ec::sentinel::app {
// 配置类别. unconfigured 表示还没有任何有效配置, 不是"空车可用".
enum class ConfigKind : std::uint8_t { unconfigured = 0, synthetic_fixture, real_deployment };
// 配置用途: 离线测试接受合成夹具; 部署用途只接受真实配置, 且拒绝任何合成来源的值.
enum class ConfigAudience : std::uint8_t { offline_test = 0, deployment };
// 单项参数来源. unspecified 表示尚未提供, 不得用零或任意保守值顶替.
enum class Origin : std::uint8_t { unspecified = 0, synthetic, reference_converted, measured, user_confirmed };
// 用户确认直接耦合; 不意味着摩擦轮其它参数已经齐备.
inline constexpr float confirmed_friction_rotor_per_axis = 1.0f;
// 配置齐备程度 (与运行期反馈有效性无关).
enum class ConfigState : std::uint8_t { unavailable = 0, incomplete, complete };

[[nodiscard]] constexpr const char* origin_name(Origin origin) noexcept {
    switch (origin) {
    case Origin::unspecified: return "unspecified";
    case Origin::synthetic: return "synthetic";
    case Origin::reference_converted: return "reference_converted";
    case Origin::measured: return "measured";
    case Origin::user_confirmed: return "user_confirmed";
    }
    return "unspecified";
}

// 一个 DJI 设备槽. installed=false 表示该域没有装配设备 (与"装配了但参数未知"不同).
struct DjiSlot {
    Domain domain{Domain::chassis_lf};
    bool installed{};
    Origin origin{Origin::unspecified};
    std::uint8_t bus{};
    drivers::DjiMode mode{drivers::DjiMode::c620_current};
    std::uint8_t device_id{};
    // 机械/电气换算: 未确认时保持 nullopt, 不填猜测值.
    std::optional<float> rotor_per_axis{};       // 转子角 / 轴侧角 (无量纲)
    std::optional<int> direction{};              // 只允许 +1 / -1
    std::optional<float> current_limit_a{};      // 轴侧电流上限 (A), 用于环路归一化
    std::optional<std::int16_t> output_limit_raw{}; // 归一化输出 ±1 对应的协议量
    std::optional<float> max_rotor_speed_rad_s{};   // 采样间隔内的已知速度上界
    std::optional<float> max_axis_velocity_rad_s{}; // 轴侧目标量程 (rad/s), 也是环路与缩放的边界
    Lifetime feedback_ttl{};                     // 反馈有效期; 无效表示尚未配置
    std::optional<control::PidConfig> velocity_pid{};
    std::optional<control::PidConfig> position_pid{}; // 连续/位置语义轴才需要
};

// 一个 DM 设备槽. MIT 映射量程 (PMAX/VMAX/TMAX) 没有通用默认值, 必须显式给出.
struct DmSlot {
    Domain domain{Domain::pitch};
    bool installed{};
    Origin origin{Origin::unspecified};
    std::uint8_t bus{};
    std::optional<std::uint16_t> can_id{};    // MIT 命令帧 ID
    std::optional<std::uint16_t> master_id{}; // 反馈帧 ID
    std::optional<drivers::DmMitLimits> limits{};
    Lifetime feedback_ttl{};
    std::optional<float> mit_kp{};
    std::optional<float> mit_kd{};
};

// 底盘几何: 每项都来自机械记录, 缺项时保持 nullopt.
struct WheelGeometryConfig {
    std::optional<float> x_m{};
    std::optional<float> y_m{};
    std::optional<float> roll_dir_rad{};
    std::optional<float> radius_m{};
    std::optional<float> gain{}; // 电机到该轮的方向/传动符号
};

// 关节几何与参考配置. 零位偏移是参考类配置: 未确认时不取得对应执行资格.
struct JointGeometryConfig {
    Domain domain{Domain::main_yaw};
    std::optional<float> zero_offset_rad{};
    std::optional<float> min_rad{};
    std::optional<float> max_rad{};
    // 明确无机械限位的DJI台架轴: 不猜上电圈数, 必须通过align显式建立连续参考.
    bool manual_continuous{false};
    // 独立机构调试尚未测机械端点: 保留标记零位, 不伪造限位.
    // DJI仅支持1:1单圈绝对参考, 启动选相对标记最近的一圈后连续跟踪; DM沿用原生绝对角.
    bool commissioning_unbounded{false};
};

// 时序与时效配置. 默认值只是"尚未配置"的占位, 不代表已实测的周期或超时.
struct TimingConfig {
    float control_dt_s{};
    Lifetime target_ttl{};    // 域目标默认有效期
    Lifetime remote_ttl{};    // 遥控样本有效期
    Lifetime imu_ttl{};       // IMU 样本有效期
    Lifetime host_link_ttl{}; // 上位机会话链路存活期
    std::optional<float> max_dt_s{};
};

// 整车配置根: 九 DJI + 一 DM + 四轮几何 + 三关节几何 + 时序.
struct SentinelConfig {
    ConfigKind kind{ConfigKind::unconfigured};
    // 独立裸电机台架不构成底盘/发射机构, 不应用跨电机故障联动与发射许可.
    bool independent_motor_bench{false};
    // 发射机构空载独立调试不依赖未调云台, 仍保留摩擦轮联动故障与显式正向拨弹许可.
    bool shooter_commissioning{false};
    std::array<DjiSlot, dji_slot_count> dji{};
    std::array<DmSlot, dm_slot_count> dm{};
    std::array<WheelGeometryConfig, chassis_wheel_count> wheels{};
    std::array<JointGeometryConfig, joint_slot_count> joints{{
        {Domain::main_yaw, {}, {}, {}},
        {Domain::fire_yaw, {}, {}, {}},
        {Domain::pitch, {}, {}, {}},
    }};
    TimingConfig timing{};
    RuntimeTiming runtime{};
    PerceptionTiming perception{};
    std::optional<SerialGeometry> serial{};
    Origin serial_origin{Origin::unspecified};
};

enum class ConfigIssue : std::uint8_t {
    none = 0,
    unconfigured_kind,
    domain_duplicated,
    unsupported_mode, // 该协议模式没有电流上限语义 (GM6020 电压模式)
    invalid_bus,
    invalid_device_id,
    missing_rotor_ratio,
    invalid_rotor_ratio,
    missing_direction,
    invalid_direction,
    missing_feedback_ttl,
    missing_current_limit,
    invalid_current_limit,
    missing_output_limit,
    invalid_output_limit,
    missing_speed_bound,
    invalid_speed_bound,
    missing_axis_speed_limit,
    invalid_axis_speed_limit,
    missing_gains,
    invalid_gains,
    missing_joint_reference,
    invalid_joint_range,
    missing_dm_can_id,
    missing_dm_master_id,
    missing_dm_limits,
    invalid_dm_limits,
    missing_dm_gains,
    invalid_dm_gains,
    missing_wheel_geometry,
    invalid_wheel_geometry,
    degenerate_wheel_layout,
    missing_timing,
    invalid_timing,
    synthetic_value_in_deployment,
    unknown_origin_in_deployment,
    invalid_serial_geometry,
    dm_feedback_id_alias,
    dji_command_mode_conflict,
};

// 配置校验结论. domains 只表达"配置是否齐备", 不代表当前反馈可用.
struct ConfigReport {
    ConfigKind kind{ConfigKind::unconfigured};
    ConfigAudience audience{ConfigAudience::offline_test};
    bool startable{}; // 允许按该用途构造应用
    bool complete{};  // 每个域配置都齐备
    std::size_t issue_count{};
    std::array<ConfigIssue, 64> issues{};
    std::array<ConfigState, domain_count> domains{};

    [[nodiscard]] bool has(ConfigIssue issue) const noexcept;
    [[nodiscard]] std::size_t count(ConfigIssue issue) const noexcept;
};

// 校验配置: 单位/范围/域身份唯一性/来源混用. 不修改配置, 不构造设备.
[[nodiscard]] ConfigReport validate(const SentinelConfig& config, ConfigAudience audience) noexcept;

// 设备配置转换: 只有该槽参数齐备且合法时才返回值.
[[nodiscard]] std::optional<drivers::DjiDeviceConfig> device_config(const DjiSlot& slot) noexcept;
[[nodiscard]] std::optional<drivers::DmDeviceConfig> device_config(const DmSlot& slot) noexcept;
// 四轮几何转换: 任一轮缺项时返回空, 不补默认值.
[[nodiscard]] std::optional<motion::OmniDrive> omni_drive(const SentinelConfig& config) noexcept;

// 主机合成夹具: 九 DJI + 一 DM 全部字段显式填写并标记 Origin::synthetic, 只用于离线测试.
// 数值是夹具值, 不代表任何实车机械/传动/电气参数.
[[nodiscard]] SentinelConfig synthetic_config() noexcept;
// 真实部署配置入口: 只声明"尚未配置", 不填任何猜测值; 由持有者逐项补齐后再校验.
[[nodiscard]] SentinelConfig real_config_placeholder() noexcept;
} // 命名空间 ec::sentinel::app
