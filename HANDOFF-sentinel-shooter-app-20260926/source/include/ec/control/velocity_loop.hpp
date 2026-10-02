#pragma once
#include "ec/channel.hpp"
#include "ec/control/pid.hpp"
#include <cmath>

namespace ec::control {
struct VelocityLoopConfig {
    PidConfig pid{};
    Lifetime feedback_ttl{};
    float max_target_rad_s{};
    float max_dt_s{};
    // 反馈一阶低通: filtered = alpha * 上次滤波值 + (1 - alpha) * 本次测量.
    // 每个成功周期只应用一次, 与反馈到达频率无关; 0 表示不滤波.
    // 复位后首个样本直接采用测量值. 旧驱动按"每帧"从 0 起滤波 (0.1*old + 0.9*new),
    // 首帧只有 90% 测量值, 此处有意不继承该首帧行为.
    float feedback_alpha{};
};

// 只表达本次计算可用性, 不是控制模式或整车安全状态机.
enum class VelocityStatus {
    active, invalid_config, no_command, stale_command, unauthorized,
    no_feedback, stale_feedback, invalid_target, invalid_dt, controller_error
};
struct VelocityResult {
    VelocityStatus status{VelocityStatus::no_command};
    std::optional<float> output{};
};

// 单任务速度控制链: 目标和反馈必须属于同一根已定义轴, 单位均为 rad/s.
// 输出是 [-1, 1] 内的归一化执行量, 必须由执行器适配显式换算, 不能直接视作 A/N·m.
// 无输出只表示禁止执行; 减速、保持、零电流或失能等动作仍由机构组装层决定.
class VelocityLoop {
public:
    explicit VelocityLoop(VelocityLoopConfig config) noexcept;
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] std::optional<Grant> arm(CommandSource source, bool authorized, bool ready) noexcept;
    void revoke() noexcept;
    // 只清控制器与滤波历史, 不改变授权状态; 重新建立位置参考后必须调用, 否则
    // 测量跳变会经 D 项 (作用于测量值) 产生一次冲击.
    void reset_controller() noexcept;
    // 在线更新环路增益; 只校验增益与归一化输出范围, 不重置控制器/滤波/授权状态.
    [[nodiscard]] bool set_pid_config(const PidConfig& pid) noexcept {
        if (pid.output_min < -1 || pid.output_max > 1) { return false; }
        return pid_.set_config(pid);
    }
    [[nodiscard]] PidConfig pid_config() const noexcept { return pid_.config(); }
    [[nodiscard]] const PidDiagnostics& diagnostics() const noexcept { return pid_.diagnostics(); }
    // 目标量程 (rad/s) 的原位更新: 与反馈时限一样属于机构量程配置, 不重建对象.
    [[nodiscard]] bool set_max_target(float rad_s) noexcept {
        if (!(rad_s > 0.0f) || !std::isfinite(rad_s)) { return false; }
        config_.max_target_rad_s = rad_s;
        return true;
    }
    [[nodiscard]] float max_target() const noexcept { return config_.max_target_rad_s; }
    [[nodiscard]] VelocityResult step(Tick now, float dt_s,
        const std::optional<Command<float>>& target,
        const std::optional<Sample<float>>& feedback) noexcept;
private:
    [[nodiscard]] VelocityResult reject(VelocityStatus status) noexcept;
    VelocityLoopConfig config_;
    Pid pid_;
    ExecutionGate gate_;
    bool filter_primed_{};
    float filtered_feedback_{};
    bool configured_{};
};
} // 命名空间 ec::control
