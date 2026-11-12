#pragma once
// 哨兵正式 APP 的数据边界 (O0):
//   "配置" / "原始设备样本" / "关节与惯性反馈" / "各域目标" / "来源与权限" /
//   "执行结果" / "状态快照" 分开表达. 本文件只有数据类型与取值域, 不含执行算法、
//   协议编解码、设备对象所有权或任务调度; 后者分别属于 motion/drivers/board 层.
// 关键约定:
//  - 控制模式 (目标语义) 与权限/故障状态是两个正交维度, 不合并成单一 mode 枚举;
//  - 未配置 / 未建参考 / 从未收到反馈, 必须由明确的 Reason 表达, 不用 0 值或"健康"顶替;
//  - 单位统一 SI (m, rad, m/s, rad/s, A); 原始协议字节只出现在 RawCanSample 中,
//    浮点反算量不得冒充原始编码.
#include "ec/control/control_mode.hpp"
#include "ec/control/execution_context.hpp"
#include "ec/execution_gate.hpp"
#include "ec/time.hpp"
#include "ec/motion/omni_kinematics.hpp"
#include "sentinel_runtime.hpp"
#include "sentinel_kinematics.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ec::sentinel::app {
// 执行域: 每个域是一个可独立接受目标并撤输出的机构单元.
// 域与设备的对应关系由配置层显式声明, 不从总线 ID 或电机型号反推.
enum class Domain : std::uint8_t {
    chassis_lf = 0,
    chassis_rf,
    chassis_lb,
    chassis_rb,
    main_yaw,
    fire_yaw,
    pitch,
    friction_left,
    friction_right,
    trigger,
};
inline constexpr std::size_t domain_count = 10;
inline constexpr std::size_t chassis_wheel_count = 4;
// 正式 APP 的设备容量: 九 DJI (四轮 + 两 yaw + 两摩擦轮 + 拨弹) + 一 DM (pitch).
inline constexpr std::size_t dji_slot_count = 9;
inline constexpr std::size_t dm_slot_count = 1;
// 三个关节域的设备既可能是 DJI (两 yaw) 也可能是 DM (pitch); 索引用于关节几何表.
inline constexpr std::size_t joint_slot_count = 3;

[[nodiscard]] constexpr std::size_t domain_index(Domain domain) noexcept {
    return static_cast<std::size_t>(domain);
}
[[nodiscard]] constexpr bool is_chassis(Domain domain) noexcept {
    return domain_index(domain) < chassis_wheel_count;
}
[[nodiscard]] constexpr bool is_joint(Domain domain) noexcept {
    return domain == Domain::main_yaw || domain == Domain::fire_yaw || domain == Domain::pitch;
}
// 模式—域白名单: 位置域只接受位置意图, 速度域只接受速度意图.
// 未实现的组合 (对关节下发速度、对速度域下发位置、任何 current) 明确拒绝,
// 不允许"接受后再在别处按另一种语义执行". 协议层与应用层共用这一个定义.
[[nodiscard]] constexpr bool mode_allowed_for(Domain domain, control::ControlMode mode) noexcept {
    if (mode == control::ControlMode::none) { return false; }
    if (is_joint(domain)) { return mode == control::ControlMode::position; }
    return mode == control::ControlMode::velocity;
}
// 关节几何表索引 (main_yaw=0, fire_yaw=1, pitch=2); 非关节域返回 nullopt 语义的哨兵值.
[[nodiscard]] constexpr std::size_t joint_index(Domain domain) noexcept {
    switch (domain) {
    case Domain::main_yaw: return 0;
    case Domain::fire_yaw: return 1;
    case Domain::pitch: return 2;
    default: return joint_slot_count;
    }
}

[[nodiscard]] constexpr const char* domain_name(Domain domain) noexcept {
    switch (domain) {
    case Domain::chassis_lf: return "chassis_lf";
    case Domain::chassis_rf: return "chassis_rf";
    case Domain::chassis_lb: return "chassis_lb";
    case Domain::chassis_rb: return "chassis_rb";
    case Domain::main_yaw: return "main_yaw";
    case Domain::fire_yaw: return "fire_yaw";
    case Domain::pitch: return "pitch";
    case Domain::friction_left: return "friction_left";
    case Domain::friction_right: return "friction_right";
    case Domain::trigger: return "trigger";
    }
    return "unknown";
}

// 运行期域有效性. 只有"配置完整且当前反馈/参考可用"才是 valid;
// 从未收到样本是 unknown, 不是健康也不是故障.
enum class DomainValidity : std::uint8_t {
    unknown = 0,
    unavailable, // 该域未装配设备
    incomplete,  // 已声明装配但配置缺项, 不取得执行资格
    valid,
    stale,   // 曾经可用, 当前超出时效
    invalid, // 反馈或配置被判定无效
};

// 明确原因码. 未配置/未建参考不得用零值或"正常"表达.
enum class Reason : std::uint8_t {
    none = 0,
    not_configured,
    incomplete_config,
    no_reference,
    no_feedback,
    stale_feedback,
    invalid_feedback,
    no_target,
    stale_target,
    invalid_target,
    no_authority,
    source_mismatch,
    locally_not_ready,
    manual_stop,
    fault_dependency,
    limit_violation,
    controller_error,
    output_rejected,
    unsupported,
    awaiting_remote, // 首启尚未看到有效遥控运行状态
    standby,        // 来源选择处于待机位, 已释放正常来源
    restarted,      // 执行器重新上电, 需重新建立反馈/参考与新目标
    degraded,       // 惯性失效后的编码器降级保持 (G07), 不是设备故障
    not_permitted,  // 外部许可 (发射授权/裁判) 缺失或过期, 不是设备故障
};

// 故障状态: 只由已判定的设备/参考故障置位; 与"正常无目标""通信时效"分开.
enum class FaultState : std::uint8_t {
    none = 0,
    feedback_lost,
    reference_lost,
    limit_violation,
    device_fault,
    dependency_fault,
    degraded, // 惯性反馈失效但编码器降级仍有效 (G07), 不是设备故障
};

// 控制来源. host = Jetson 应用链; maintenance = PC 直连互斥维护会话.
// 来源仲裁与人工停止覆盖规则不在这里实现, 这里只表达结果.
enum class Source : std::uint8_t { none = 0, remote, host, maintenance };

[[nodiscard]] constexpr const char* validity_name(DomainValidity validity) noexcept {
    switch (validity) {
    case DomainValidity::unknown: return "unknown";
    case DomainValidity::unavailable: return "unavailable";
    case DomainValidity::incomplete: return "incomplete";
    case DomainValidity::valid: return "valid";
    case DomainValidity::stale: return "stale";
    case DomainValidity::invalid: return "invalid";
    }
    return "unknown";
}
[[nodiscard]] constexpr const char* reason_name(Reason reason) noexcept {
    switch (reason) {
    case Reason::none: return "none";
    case Reason::not_configured: return "not_configured";
    case Reason::incomplete_config: return "incomplete_config";
    case Reason::no_reference: return "no_reference";
    case Reason::no_feedback: return "no_feedback";
    case Reason::stale_feedback: return "stale_feedback";
    case Reason::invalid_feedback: return "invalid_feedback";
    case Reason::no_target: return "no_target";
    case Reason::stale_target: return "stale_target";
    case Reason::invalid_target: return "invalid_target";
    case Reason::no_authority: return "no_authority";
    case Reason::source_mismatch: return "source_mismatch";
    case Reason::locally_not_ready: return "locally_not_ready";
    case Reason::manual_stop: return "manual_stop";
    case Reason::fault_dependency: return "fault_dependency";
    case Reason::limit_violation: return "limit_violation";
    case Reason::controller_error: return "controller_error";
    case Reason::output_rejected: return "output_rejected";
    case Reason::unsupported: return "unsupported";
    case Reason::awaiting_remote: return "awaiting_remote";
    case Reason::standby: return "standby";
    case Reason::restarted: return "restarted";
    case Reason::degraded: return "degraded";
    case Reason::not_permitted: return "not_permitted";
    }
    return "none";
}
[[nodiscard]] constexpr const char* fault_name(FaultState fault) noexcept {
    switch (fault) {
    case FaultState::none: return "none";
    case FaultState::feedback_lost: return "feedback_lost";
    case FaultState::reference_lost: return "reference_lost";
    case FaultState::limit_violation: return "limit_violation";
    case FaultState::device_fault: return "device_fault";
    case FaultState::dependency_fault: return "dependency_fault";
    case FaultState::degraded: return "degraded";
    }
    return "none";
}
[[nodiscard]] constexpr const char* source_name(Source source) noexcept {
    switch (source) {
    case Source::none: return "none";
    case Source::remote: return "remote";
    case Source::host: return "host";
    case Source::maintenance: return "maintenance";
    }
    return "none";
}

// 原始接收样本: 保留字节与接收时刻. 型号由配置声明, 不靠 CAN ID 猜测.
struct RawCanSample {
    std::uint8_t bus{};
    std::uint16_t id{};
    std::uint8_t size{};
    std::array<std::uint8_t, 8> data{};
    Tick received_at{};
};

// 关节反馈 (轴侧 SI). 无效来源保持 Validity::unknown, 不填 0.
// temperature_c 的 nullopt 表示"该设备无此字段"(例如 C610 温度保留位), 不是 0 °C.
struct JointFeedback {
    Sample<float> position_rad{};
    Sample<float> velocity_rad_s{};
    Sample<float> current_a{};
    std::optional<float> temperature_c{};
    std::optional<RawCanSample> raw{};
};

// 惯性反馈: 角速度与比力是传感器量, 姿态是估计量, 分开表达.
// reference_valid 表示是否已建立可用的参考原点/对齐关系; 未建立时不冒充有效姿态.
// 惯性反馈快照. 明确区分传感器样本时刻与本地摄入时刻: 只按摄入时刻判新鲜会把
// 采样间隔抖动/乱序静默算成"新数据".
struct InertialFeedback {
    DomainValidity validity{DomainValidity::unavailable};
    Reason reason{Reason::not_configured};
    Sample<std::array<float, 3>> gyro_rad_s{};   // 去偏后的机体系角速度
    Sample<std::array<float, 3>> accel_mps2{};   // 安装变换后的最新原始比力, 与重力融合质量分开
    Sample<std::array<float, 3>> roll_pitch_yaw_rad{};
    // 重力方向: 参考系 +Z 在机体系中的单位向量 (加速度计观测方向).
    Sample<std::array<float, 3>> gravity_direction_body{};
    Tick sampled_at{};  // 传感器/链路样本时刻
    Tick ingested_at{}; // 本地摄入时刻
    std::uint32_t age_ms{};   // 快照相对样本时刻的年龄
    bool extrapolated{};      // 快照由最近角速度外推得到, 不是直接测量
    bool reference_valid{};
    Tick reference_established_at{};
};

// 主平台的本地惯性反馈, 不与末端传感器在线状态混用.
struct PlatformFeedback {
    PlatformKinematics value{};
    DomainValidity validity{DomainValidity::unavailable};
    Reason reason{Reason::incomplete_config};
    float continuous_heading{};
    MicroTick evaluated_at{}, gyro_at{};
    std::array<std::uint32_t, 3> ages_us{0xffffffffu, 0xffffffffu, 0xffffffffu};
    std::array<std::uint32_t, 3> time_error_us{0xffffffffu, 0xffffffffu, 0xffffffffu};
    std::uint32_t local_epoch{}, alignment_epoch{};
    bool local_reference{};
    bool odom_aligned{};
    float odom_offset_rad{};
};

// 外部约束与许可 (功率/热量/发射/裁判). 缺失或过期一律不得当作许可;
// 本版只对"发射许可"做门禁 (A11), 功率/热量仅作为可注入观测上报, 阈值依据仍缺.
struct AuxState {
    bool launch_permit_valid{};
    bool launch_permit{};
    Tick launch_sampled_at{};
    Lifetime launch_ttl{};
    bool power_limited{};
    bool heat_limited{};

    [[nodiscard]] bool launch_allowed(Tick now) const noexcept {
        return launch_permit_valid && launch_permit && fresh(now, launch_sampled_at, launch_ttl);
    }
};

// 一份域目标 = 完整意图. 有效零值 (value == 0) 与"没有目标"是两件事:
// present 为假时该域执行无目标动作, 不能把 0 目标当成它的替代.
struct DomainTarget {
    control::ControlMode mode{control::ControlMode::none};
    float value{};
    Tick accepted_at{};
    Lifetime ttl{};
    // 接受该目标时的授权身份: 换来源/换会话后, 旧目标不因"当前有人持权"而复活.
    Grant grant{};
    YawFrame frame{YawFrame::joint};
    [[nodiscard]] bool present() const noexcept {
        return mode != control::ControlMode::none;
    }
    [[nodiscard]] bool fresh_at(Tick now) const noexcept {
        return present() && fresh(now, accepted_at, ttl);
    }
};

// 权限/自检状态: 与各域控制模式正交, 整车一份.
// manual_stop 只由人工整车停止挡触发; 软件异常不得自动切入该状态.
struct PermissionState {
    Source source{Source::none};
    bool authority{};
    bool manual_stop{};
    bool locally_ready{};
    std::uint32_t generation{}; // 授权代次: 换来源/撤销/换会话时递增
};

// 输入链路状态: 与权限分开, 因为"链路在线"不等于"已取得控制权".
struct InputState {
    bool remote_online{};
    bool host_link_alive{};
    Tick remote_sampled_at{};
    Tick host_sampled_at{};
};

// 单个域的执行结论: 由应用层在每周期依据权限/时效/故障得出, 交给执行层落实.
// applied_output 是归一化执行量 (不是电流/电压/角度); 未计算时保持空.
// stop 是本周期显式选择的撤输出动作, 未配置时为 unspecified (必须报告而不是静默归零).
struct DomainExecution {
    Reason reason{Reason::none};
    bool execute{};
    control::ControlMode mode{control::ControlMode::none};
    control::StopAction stop{control::StopAction::unspecified};
    std::optional<float> adopted_target{};
    std::optional<float> applied_output{};
    Grant token{};
    Tick at{};
    YawFrame frame{YawFrame::joint};
};

// 单个域的一致状态快照.
struct DomainStatus {
    Domain domain{Domain::chassis_lf};
    bool configured{};
    DomainValidity validity{DomainValidity::unknown};
    Reason reason{Reason::not_configured};
    FaultState fault{FaultState::none};
    control::ControlMode mode{control::ControlMode::none};
    bool enabled{};
    std::optional<DomainTarget> target{};
    JointFeedback feedback{};
    std::optional<float> output{};
};

// 整车状态快照. 只表达"当前观察到什么、为什么不可用", 不代替实机验收结论.
struct StatusSnapshot {
    Tick at{};
    PermissionState permission{};
    InputState input{};
    std::array<DomainStatus, domain_count> domains{};
    // 与观测状态分开的执行结论 (O1 起): 权限/时效/故障/无目标动作.
    std::array<DomainExecution, domain_count> executions{};
    InertialFeedback inertial{};
    PlatformFeedback main_yaw{};
    AuxState aux{};
    bool launch_allowed{};
    std::optional<motion::PlanarTwist> requested_twist{}, adopted_twist{};
    float twist_scale{1.0f};
    std::uint32_t config_epoch{};
    std::uint32_t reference_epoch{};

    [[nodiscard]] const DomainStatus& domain(Domain id) const noexcept {
        return domains[domain_index(id)];
    }
};
} // 命名空间 ec::sentinel::app
