#pragma once
#include "ec/control/control_mode.hpp"
#include "ec/control/position_loop.hpp"
#include "ec/control/velocity_loop.hpp"
#include "ec/drivers/dji_device.hpp"

namespace ec::motion {
// 轴侧电流上限与环路归一化的关系:
//   环路归一化输出 ±1 = 轴侧电流上限 current_limit_a; 轴在输出处乘一次
//   current_limit_a / (设备 raw 编码上限对应的电流), 换算到设备的编码基准.
//   current 模式的目标本身就是 A, 直接除以同一个满量程电流, 不再乘该比例.
// 传入的 PidConfig 因此以"电流上限的分数"为单位, 电流上限由 current_limit_a 唯一表达.
struct DjiAxisConfig {
    float current_limit_a{}; // 轴侧电流上限, 必须 > 0 且不超过设备编码上限对应电流
    control::PidConfig velocity_pid{};
    control::PidConfig position_pid{};
    control::ArcMode arc{control::ArcMode::continuous};
    Lifetime command_ttl{}; // 目标有效期; 目标不刷新即失效并使本轴退出使能
    float max_velocity_rad_s{};
    float max_dt_s{};
    float feedback_alpha{}; // 透传给速度环的反馈低通
};

// 只表达本次计算可用性, 不是控制模式或整车安全状态机.
enum class AxisStatus : std::uint8_t {
    idle, active, invalid_config, no_target, stale_target, unauthorized, invalid_target,
    no_feedback, stale_feedback, invalid_dt, controller_error
};

struct DjiAxisResult {
    AxisStatus status{AxisStatus::idle};
    std::optional<float> target{};            // 本周期采用的目标, 单位随模式
    std::optional<float> normalized_output{}; // 归一化执行量, 交给组发送器编码
};

struct DjiAxisState {
    control::ControlMode mode{control::ControlMode::none};
    bool enabled{};
    Sample<float> target{}; // 最近一次被接受的目标, 单位随模式
    Sample<drivers::DjiFeedback> rotor{};
    Sample<float> velocity_rad_s{};
    Sample<float> position_rad{};
    std::optional<drivers::DjiOutput> output{};
    AxisStatus status{AxisStatus::idle};
};

// 单任务轴门面: 组合既有 DjiDevice、正式速度/位置环与协议换算, 不复制编解码或 PID.
// 设备由组装层持有且生命周期长于本对象; 轴不拥有设备, 传动比/方向/编码上限只来自设备配置,
// 避免两份不同来源的传动与限幅同时生效. 仅支持有电流上限语义的协议 (C610/C620/GM6020 电流);
// GM6020 电压模式没有电流上限可比, 需另建电压域配置.
// 默认不使能. enable(mode, ...) 是显式状态转换, 也是唯一重置控制器与授权的时机;
// 每周期 set_* 只更新目标. 模式切换 = disable 后重新 enable, 不提供隐式热切换.
// 任一被拒绝的周期都会退出使能, 旧目标连同旧授权一起失效, 恢复必须重新 enable.
class DjiAxis {
public:
    DjiAxis(DjiAxisConfig config, drivers::DjiDevice& device) noexcept;
    DjiAxis(const DjiAxis&) = delete;
    DjiAxis& operator=(const DjiAxis&) = delete;
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] const DjiAxisConfig& config() const noexcept { return config_; }
    [[nodiscard]] control::ControlMode mode() const noexcept { return mode_; }
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }

    [[nodiscard]] std::optional<Grant> enable(control::ControlMode mode,
        CommandSource source, bool authorized, bool ready) noexcept;
    void disable() noexcept;

    // 只有与当前模式匹配的设置会被接受; 不匹配或越界返回 false 且不改变目标.
    [[nodiscard]] bool set_current(float amps, Tick now) noexcept;
    [[nodiscard]] bool set_velocity(float rad_s, Tick now) noexcept;
    [[nodiscard]] bool set_position(float rad, Tick now) noexcept;
    // 位置参考与使能/停用无关; 反馈不新鲜时失败, 也不会伪造掉电前的多圈值.
    [[nodiscard]] bool reference_position(float axis_rad, Tick now) noexcept;

    // 在线更新环路增益: 只改增益, 不改变模式/使能/目标/参考, 也不重置控制器状态.
    // 调用方须在机构退出执行后再更新参考类参数; 本接口不承担该判定.
    // 位置模式实际使用的是位置串级的内环速度环, 因此两条路径必须同时更新,
    // 否则位置模式调参看似生效而内环不变 (两种模式下该参数的语义是同一个).
    [[nodiscard]] bool set_velocity_gains(const control::PidConfig& pid) noexcept {
        // 先更新位置串级内环: 位置模式下真正生效的就是它. 未配置位置环的轴
        // (只有速度语义) 没有内环, 不算失败.
        if (position_.configured() && !position_.set_inner_pid_config(pid)) { return false; }
        return velocity_.set_pid_config(pid);
    }
    [[nodiscard]] bool set_position_gains(const control::PidConfig& pid) noexcept {
        return position_.set_pid_config(pid);
    }
    // 参数读回入口: 必须来自当前模式真正在用的那个环路, 不用另一份 config 缓存证明成功.
    [[nodiscard]] control::PidConfig active_velocity_pid() const noexcept {
        return mode_ == control::ControlMode::position ? position_.inner_pid_config()
                                                      : velocity_.pid_config();
    }
    [[nodiscard]] control::PidConfig position_pid() const noexcept { return position_.pid_config(); }
    // 参考类参数的原位生效: 只改轴侧电流上限的换算比例, 不改变模式/使能/目标/参考.
    // 设备编码上限对应的电流没有变, 因此重新归一化即可; 超出设备编码能力时拒绝.
    [[nodiscard]] bool set_current_limit(float amps) noexcept {
        if (!configured_ || !std::isfinite(amps) || amps <= 0.0f || amps > amp_full_scale_) {
            return false;
        }
        config_.current_limit_a = amps;
        current_ratio_ = amps / amp_full_scale_;
        return true;
    }
    [[nodiscard]] float current_limit() const noexcept { return config_.current_limit_a; }
    // 目标量程原位更新: 同时更新独立速度环与位置串级内环的限幅, 不重建对象.
    [[nodiscard]] bool set_max_velocity(float rad_s) noexcept {
        if (!configured_ || !velocity_.set_max_target(rad_s) ||
            !position_.set_inner_max_target(rad_s)) {
            return false;
        }
        config_.max_velocity_rad_s = rad_s;
        return true;
    }
    [[nodiscard]] float max_velocity() const noexcept { return config_.max_velocity_rad_s; }
    // 读当前模式真正参与控制的内环, 不把另一实例的历史作为当前分项.
    [[nodiscard]] const control::PidDiagnostics& velocity_diagnostics() const noexcept {
        return mode_ == control::ControlMode::position ? position_.inner_diagnostics() : velocity_.diagnostics();
    }
    [[nodiscard]] const control::PidDiagnostics& position_diagnostics() const noexcept {
        return position_.outer_diagnostics();
    }

    [[nodiscard]] DjiAxisResult tick(Tick now, float dt_s) noexcept;
    // 显式替代关节速率测量, 位置仍来自设备机械参考; 仅速度/位置模式可用.
    // 缺失/超时不回退到设备RPM, 沿用原环路时效与授权契约.
    [[nodiscard]] DjiAxisResult tick_joint_velocity(Tick now,float dt_s,const Sample<float>& velocity) noexcept;
    // 显式外部位置/速率反馈 (同一轴的惯性参考), 不会在缺失时偷换成编码器反馈.
    // 仍由本轴唯一的 PositionLoop/VelocityLoop 和设备电流换算产生输出.
    [[nodiscard]] DjiAxisResult tick_external_position(Tick now, float dt_s,
        const Sample<float>& position, const Sample<float>& velocity,
        std::optional<float> outer_dt_s, float rate_per_axis) noexcept;
    [[nodiscard]] const std::optional<drivers::DjiOutput>& output() const noexcept { return output_; }
    [[nodiscard]] DjiAxisState state(Tick now) const noexcept;
private:
    [[nodiscard]] DjiAxisResult tick_impl(Tick now, float dt_s, const Sample<float>& position,
        const Sample<float>& velocity, std::optional<float> outer_dt_s, float rate_per_axis) noexcept;
    [[nodiscard]] DjiAxisResult fail(AxisStatus status) noexcept;
    [[nodiscard]] bool store(float value, Tick now, float magnitude_limit) noexcept;

    DjiAxisConfig config_;
    drivers::DjiDevice& device_; // 建立位置参考需要修改设备; 设备仍由组装层持有, 轴不拥有它
    control::VelocityLoop velocity_;
    control::PositionLoop position_;
    float amp_full_scale_{};       // 设备 raw 编码上限对应的轴侧电流
    float current_ratio_{};        // current_limit_a / amp_full_scale, 落在 (0, 1]
    bool configured_{};
    control::ControlMode mode_{control::ControlMode::none};
    bool enabled_{};
    std::optional<Grant> grant_{};
    std::optional<Sample<float>> target_{};
    std::optional<drivers::DjiOutput> output_{};
    AxisStatus status_{AxisStatus::idle};
};
} // 命名空间 ec::motion
