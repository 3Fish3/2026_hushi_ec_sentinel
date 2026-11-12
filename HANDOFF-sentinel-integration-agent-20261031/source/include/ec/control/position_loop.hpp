#pragma once
#include "ec/control/angle.hpp"
#include "ec/control/velocity_loop.hpp"

namespace ec::control {
struct PositionLoopConfig {
    // 位置误差 rad → 速度目标 rad/s. output_min/output_max 就是速度限幅, 应落在
    // ±velocity.max_target_rad_s 之内 (构造时检查).
    PidConfig position_pid{};
    VelocityLoopConfig velocity{}; // 内环, 复用正式速度环, 不复制 PID
    ArcMode arc{ArcMode::continuous};
};

// 只表达本次计算可用性, 不是控制模式或整车安全状态机.
enum class PositionStatus {
    active, invalid_config, no_command, stale_command, unauthorized, no_feedback,
    stale_feedback, invalid_target, invalid_dt, controller_error, inner_rejected
};
struct PositionResult {
    PositionStatus status{PositionStatus::no_command};
    std::optional<float> output{};          // 归一化执行量, 由执行器适配显式换算
    std::optional<float> velocity_target{}; // 外环产生的内环目标, rad/s
    std::optional<VelocityStatus> velocity_status{}; // 内环原始结果, 仅用于诊断
};

// 单任务位置链: 位置误差 (rad) → 速度目标 (rad/s) → 归一化执行量.
// 授权门只保留在内环, 外环不复制第二份授权状态; 任一拒绝即撤销授权并清控制器历史.
// 位置反馈与速度反馈必须属于同一根已定义轴; 反馈注入就是传入 Sample 本身.
class PositionLoop {
public:
    explicit PositionLoop(PositionLoopConfig config) noexcept;
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] std::optional<Grant> arm(CommandSource source, bool authorized, bool ready) noexcept;
    void revoke() noexcept;
    void reset_controller() noexcept;
    // 在线更新位置环增益 (内环速度环保持不变), 不重置控制器状态.
    [[nodiscard]] bool set_pid_config(const PidConfig& pid) noexcept {
        return position_pid_.set_config(pid);
    }
    [[nodiscard]] PidConfig pid_config() const noexcept { return position_pid_.config(); }
    // 位置串级的内环速度环: 位置模式实际使用的就是这一份, 与独立速度环是两个不同实例.
    // "速度环增益"这一参数语义在两种模式下一致, 因此两条路径都必须能被更新与读回,
    // 否则位置模式下的读回会来自一个根本没有参与控制的环路.
    [[nodiscard]] bool set_inner_pid_config(const PidConfig& pid) noexcept {
        return velocity_.set_pid_config(pid);
    }
    [[nodiscard]] PidConfig inner_pid_config() const noexcept { return velocity_.pid_config(); }
    [[nodiscard]] const PidDiagnostics& inner_diagnostics() const noexcept { return velocity_.diagnostics(); }
    [[nodiscard]] const PidDiagnostics& outer_diagnostics() const noexcept { return position_pid_.diagnostics(); }
    // 内环目标量程 (rad/s): 位置模式的速度目标限幅随机构量程配置更新, 不重建对象.
    [[nodiscard]] bool set_inner_max_target(float rad_s) noexcept {
        if (!velocity_.set_max_target(rad_s)) { return false; }
        auto pid=position_pid_.config();
        pid.limit_output=rad_s>0.0f;
        if (pid.limit_output) { pid.output_min=-rad_s; pid.output_max=rad_s; }
        return position_pid_.set_config(pid);
    }
    [[nodiscard]] float inner_max_target() const noexcept { return velocity_.max_target(); }
    // 专用辨识入口: 复用本位置串级内环及同一授权, 不计算/更新位置外环.
    [[nodiscard]] VelocityResult step_inner(Tick now, float dt_s, const Command<float>& target,
        const Sample<float>& velocity) noexcept { return velocity_.step(now,dt_s,target,velocity); }
    [[nodiscard]] PositionResult step(Tick now, float dt_s,
        const std::optional<Command<float>>& target_rad,
        const std::optional<Sample<float>>& position_rad,
        const std::optional<Sample<float>>& velocity_rad_s) noexcept;
    // 分频外环: nullopt 表示本周期只运行内环, 不是 dt=0. rate_per_axis 把输入参考系
    // 的角速度换算到执行轴等效速度; 机械位置环取 1, 惯性方向链由运动学提供有符号增益.
    [[nodiscard]] PositionResult step_split(Tick now, float inner_dt_s, std::optional<float> outer_dt_s,
        const std::optional<Command<float>>& target_rad,
        const std::optional<Sample<float>>& position_rad,
        const std::optional<Sample<float>>& velocity_rad_s, float rate_per_axis) noexcept;
private:
    [[nodiscard]] PositionResult reject(PositionStatus status) noexcept;
    PositionLoopConfig config_;
    Pid position_pid_;
    VelocityLoop velocity_;
    bool configured_{};
    std::optional<float> held_velocity_{};
};
} // 命名空间 ec::control
