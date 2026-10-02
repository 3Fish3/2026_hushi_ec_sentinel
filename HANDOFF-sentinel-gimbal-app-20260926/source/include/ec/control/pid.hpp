#pragma once
#include <optional>

namespace ec::control {
struct PidConfig {
    float kp{};
    float ki{};
    float kd{};
    float integral_limit{};
    float output_min{};
    float output_max{};
    // 测量侧微分独立一阶低通时间常数, 单位s; 0保持原始微分, 不延迟P/I反馈.
    float derivative_tau_s{};
};

// 最近一次真实计算的只读诊断; 单位与本 PID 输出一致, 不在观测端重算积分/微分.
struct PidDiagnostics {
    float target{}, measured{}, p{}, i{}, d{}, unconstrained{}, output{};
    bool valid{};
};

// 单任务控制组件, dt 由调用者按实际经过的秒数传入.
// 微分作用于测量值, 积分采用有界钳位; 可选D独立低通, 不含前馈或机构安全动作.
class Pid {
public:
    explicit Pid(PidConfig config) noexcept;
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] std::optional<float> step(float target, float measured, float dt_s) noexcept;
    // 在线更新增益/限幅: 校验与构造一致, 但**不**重置积分与上一次测量.
    // 因此这不是无扰切换, 调用方须自行决定是否先 reset().
    [[nodiscard]] bool set_config(const PidConfig& config) noexcept;
    [[nodiscard]] const PidConfig& config() const noexcept { return config_; }
    void reset() noexcept;
    [[nodiscard]] const PidDiagnostics& diagnostics() const noexcept { return diagnostics_; }
private:
    PidConfig config_;
    bool configured_{};
    bool has_previous_{};
    float integral_{};
    float previous_measured_{};
    float derivative_rate_{};
    PidDiagnostics diagnostics_{};
};
} // 命名空间 ec::control
