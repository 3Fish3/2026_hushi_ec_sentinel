#pragma once
// 执行与编组层 (O2): 把应用层的每域执行结论落实到真实设备, 并按 (总线, 命令 CAN ID)
// 提交输出. 拥有九 DJI 设备 + 一个 DM 设备、轴对象与发送组; 不解析协议、不做权限判定、
// 不替应用决定停止动作. 正常与故障路径都走真实 codec/设备/发送组, 传输 sink 可以是主机替身.
#include "ec/drivers/dji_device.hpp"
#include "ec/drivers/dm_device.hpp"
#include "ec/motion/dji_axis.hpp"
#include "ec/transport/can.hpp"
#include "sentinel_app.hpp"
#include "sentinel_config.hpp"
#include "sentinel_params.hpp"
#include "raw_record.hpp"
#include <array>
#include <cstddef>
#include <optional>

namespace ec::sentinel::app {
// 单圈轴侧角 + 已配置范围能唯一确定关节角时返回该角; 不能唯一确定时返回空,
// 由调用方保持"参考未知直到显式对齐", 不猜圈数、不自动寻零.
// alias_period_rad 是轴侧量的混叠周期: 转子单圈编码器经 N:1 传动后在轴侧只看得到
// 2π/N 的周期, 用 2π 会把不同圈数当成同一读数.
[[nodiscard]] std::optional<float> recover_single_turn_joint(float min_rad, float max_rad,
    float zero_offset_rad, float axis_single_turn_rad, float alias_period_rad) noexcept;
// 在 [min_rad, max_rad] 内选择与请求方向 2π 等价且最接近当前位置的分支;
// 无可达解返回空 (不自动调底盘或另一轴补偿). 候选数量固定, 工作量不随请求数值增长.
[[nodiscard]] std::optional<float> select_equivalent_branch(
    float min_rad, float max_rad, float current_rad, float requested_rad) noexcept;

struct JointReference {
    bool valid{};
    float joint_rad{};
};

struct ExecutorStats {
    std::uint32_t cycles{};
    std::uint32_t enable_failures{};
    std::uint32_t target_rejections{};
    std::uint32_t unreachable_targets{};
    std::uint32_t group_sent{};
    std::uint32_t group_refused{};
    std::uint32_t group_tx_error{}; // 队列满/发送错误: 不报告为已到达电调
    std::uint32_t dm_enable_sent{};
    std::uint32_t dm_disable_sent{};
    std::uint32_t dm_command_sent{};
    std::uint32_t dm_tx_error{};
    std::uint32_t dm_query_sent{};
    std::uint32_t skipped_groups{};
    bool timing_observed{};
    std::uint32_t control_last_us{}, control_max_us{}, tx_last_us{}, tx_max_us{};
};

struct MainYawControlStats {
    std::uint32_t outer_steps{}, inner_steps{}, feedback_switches{};
    float outer_dt_s{}, local_target_rad{}, resolved_joint_rad{}, equivalent_rate_rad_s{};
    YawFrame frame{YawFrame::joint};
    bool mechanical_limit{};
};

struct ExecutorGroupStats {
    std::uint8_t bus{};
    std::uint16_t id{};
    std::uint32_t computations{}, submitted{}, tx_failed{}, invalid_steps{};
    float last_dt_s{}, max_dt_s{};
    MicroTick last_computed{}, last_submitted{};
    bool timing_observed{}, stop_only{};
    drivers::DjiGroupStatus last_result{drivers::DjiGroupStatus::not_sent};
};

class SentinelExecutor {
public:
    SentinelExecutor(const SentinelConfig& config, ConfigAudience audience,
        transport::CanTx& bus1, transport::CanTx& bus2,
        MicroTick (*clock)() noexcept = nullptr) noexcept;
    SentinelExecutor(const SentinelExecutor&) = delete;
    SentinelExecutor& operator=(const SentinelExecutor&) = delete;

    // 总线输出拥有者已构造不等于每轴可执行; 缺项命令组只允许显式零输出.
    [[nodiscard]] bool assembled() const noexcept { return assembled_; }
    [[nodiscard]] const ConfigReport& report() const noexcept { return report_; }
    [[nodiscard]] drivers::DjiDevice* dji_device(std::size_t slot) noexcept;
    [[nodiscard]] drivers::DmDevice* dm_device() noexcept {
        return dm_devices_[0] ? &*dm_devices_[0] : nullptr;
    }
    [[nodiscard]] motion::DjiAxis* axis(Domain domain) noexcept;
    // 显式选择已换算为关节坐标的测速样本; 无样本时不暗中回退另一反馈源.
    void set_joint_velocity_feedback(Domain domain, Sample<float> feedback) noexcept {
        joint_velocity_feedback_[domain_index(domain)] = feedback;
    }
    // 收帧路由: 板级/主机把总线上的帧逐帧投递进来; 设备自行按 ID 过滤.
    void route(std::uint8_t bus, const transport::CanFrame& frame, Tick at,
        std::optional<MicroTick> at_us = std::nullopt) noexcept;
    [[nodiscard]] std::optional<MicroTick> feedback_time(Domain domain) const noexcept {
        return feedback_times_[domain_index(domain)];
    }

    // 关节参考: 显式对齐 (人工/载荷标定) 或按单圈反馈自动唯一恢复.
    [[nodiscard]] bool align_reference(Domain domain, float joint_rad, Tick now) noexcept;
    [[nodiscard]] const JointReference& reference(Domain domain) const noexcept {
        return references_[domain_index(domain)];
    }
    // 关节侧反馈视图 (位置已减零位偏移, 与健康判定/参考同一坐标). 调用方 (板级/主机)
    // 应当用这个而不是设备原生量, 否则非零零位时两处不是同一个角.
    [[nodiscard]] DomainFeedback joint_feedback(Domain domain, Tick now) const noexcept;

    // 从真实设备反馈推导应用层健康/故障输入. 会推进自动参考恢复, 但从不虚构反馈.
    [[nodiscard]] FaultInputs health(Tick now) noexcept;
    [[nodiscard]] const FaultInputs& last_health() const noexcept { return health_; }

    // 一个执行周期: 按 app 的执行结论使能/设目标/撤输出, 再按组提交并处理 pitch.
    void step(Tick now, float dt_s, const SentinelApp& app,
        std::optional<MicroTick> now_us = std::nullopt) noexcept;
    [[nodiscard]] const ExecutorStats& stats() const noexcept { return stats_; }
    [[nodiscard]] const MainYawControlStats& main_yaw_stats() const noexcept { return main_yaw_stats_; }
    [[nodiscard]] std::optional<recording::Record> trace(Domain domain, Tick now, const SentinelApp& app) const noexcept;
    [[nodiscard]] const ExecutorGroupStats& pitch_stats() const noexcept { return pitch_stats_; }
    [[nodiscard]] std::optional<ExecutorGroupStats> group_stats(std::size_t index) const noexcept {
        return index < group_count_ ? std::optional{groups_[index].stats} : std::nullopt;
    }
    // 实际生效的速度环增益 (0=Kp, 1=Ki, 2=Kd): 供参数读回与测试核对, 不改变行为.
    // 读的是当前模式真正在用的环路, 不是另一份 config 缓存.
    [[nodiscard]] float velocity_gain(std::size_t dji_slot, int which) const noexcept;    // 参数生效事务专用: 把参考类配置原位落到运行对象上, 不重建对象、不重置控制器状态.
    // 关节几何同时被应用层限幅与执行层分支/健康使用, 因此事务必须两边一起更新;
    // 这里是执行层侧的入口, 读回仍以应用层配置为准 (单一权威).
    [[nodiscard]] bool set_joint_geometry(Domain domain, const JointGeometryConfig& joint) noexcept;
    // DM MIT 增益存在执行层配置里 (设备本身不存增益): 参数事务的生效与读回入口.
    [[nodiscard]] bool set_dm_mit_gains(float kp, float kd) noexcept;
    [[nodiscard]] float dm_mit_kp() const noexcept { return config_.dm[0].mit_kp.value_or(0.0f); }
    [[nodiscard]] float dm_mit_kd() const noexcept { return config_.dm[0].mit_kd.value_or(0.0f); }
    // pitch 的使能/失能请求状态: 用于观测"请求已发"与"设备已确认"的区别.
    [[nodiscard]] bool pitch_enable_requested() const noexcept { return pitch_enable_requested_; }
    [[nodiscard]] bool pitch_disable_requested() const noexcept { return pitch_disable_requested_; }
    [[nodiscard]] bool pitch_enable_confirmed() const noexcept { return pitch_enable_confirmed_; }

private:
    struct TxGroupSlot {
        std::uint8_t bus{};
        std::uint16_t tx_id{};
        unsigned group{};
        std::array<std::optional<std::size_t>, 4> indices{}; // dji_devices_ 下标
        std::optional<drivers::DjiTxGroup> sender{};
        std::optional<transport::CanFrame> zero_frame{}; // 有已装设备未形成 device 时, 整组只能发已知地址的零命令
        GroupTiming timing{};
        GroupClock clock{};
        ScheduledStep step{};
        ExecutorGroupStats stats{};
        bool compute_started{};
    };
    struct DomainRuntime {
        std::size_t dji_slot{};
        std::size_t group{};
        std::size_t slot{};
        drivers::DjiDevice* device{};
        bool ever_enabled{};
    };

    [[nodiscard]] transport::CanTx& bus_for(std::uint8_t bus_id) noexcept;
    [[nodiscard]] bool feedback_ready(const DomainRuntime& runtime, Tick now) const noexcept;
    [[nodiscard]] std::optional<float> branch_for(
        Domain domain, float requested_rad, Tick now) noexcept;
    void step_pitch(Tick now, const SentinelApp& app, bool force_stop = false) noexcept;
    void query_pitch() noexcept;
    [[nodiscard]] bool send_pitch_frame(const transport::CanFrame& frame) noexcept;
    [[nodiscard]] bool pitch_output_fresh() noexcept;
    void send_groups(Tick now) noexcept;

    SentinelConfig config_{};
    ConfigReport report_{};
    bool assembled_{};
    transport::CanTx* bus1_{};
    transport::CanTx* bus2_{};
    std::array<std::optional<drivers::DjiDevice>, dji_slot_count> dji_devices_{};
    std::array<std::optional<drivers::DmDevice>, dm_slot_count> dm_devices_{};
    std::array<std::optional<motion::DjiAxis>, domain_count> axes_{};
    std::array<TxGroupSlot, 8> groups_{};
    std::size_t group_count_{};
    std::array<std::optional<DomainRuntime>, domain_count> runtime_{};
    std::array<JointReference, domain_count> references_{};
    std::array<std::optional<drivers::DjiOutput>, dji_slot_count> outputs_{};
    std::array<bool, domain_count> unreachable_{};
    std::array<bool, domain_count> rejected_{};
    FaultInputs health_{};
    std::array<std::optional<MicroTick>, domain_count> feedback_times_{};
    std::array<std::optional<Sample<float>>, domain_count> joint_velocity_feedback_{};
    ExecutorStats stats_{};
    MicroTick (*clock_)() noexcept{};
    std::array<bool, domain_count> was_executing_{};
    GroupClock pitch_clock_{};
    GroupClock direction_clock_{};
    MainYawControlStats main_yaw_stats_{};
    bool main_inertial_active_{};
    ExecutorGroupStats pitch_stats_{};
    bool pitch_enable_requested_{};
    bool pitch_enable_confirmed_{};
    bool pitch_disable_requested_{};
    // 有界重试退避: 撤输出期间至多每个重试间隔发一次 disable, 不因历史失败永久停止尝试.
    bool pitch_disable_attempted_{};
    Tick pitch_last_disable_at_{};
};
} // 命名空间 ec::sentinel::app
