#pragma once
#include "ec/control/current_scale.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/motion/dji_axis.hpp"
#include "gm6020_bench_excitation.hpp"
#include "gm6020_bench_velocity.hpp"
#include <cmath>
#include <cstdint>
#include <optional>

namespace ec::sentinel {
enum class BenchMode : std::uint8_t { current=1, velocity=2, position=3 };
enum class BenchReason : std::uint8_t { idle, running, stopped, duration, command_timeout,
    feedback_lost, position_lost, overspeed, tx_failed, invalid_request, busy, timing };
struct BenchSettings {
    BenchMode mode{BenchMode::current};
    // 旧主 yaw: 73 raw/(deg/s), 3000 raw, 200 deg/s; 按 ±16384 = ±3 A 换算.
    float target{}, duration_s{0}, current_limit{3000.0f*3/16384}, speed_limit{3.4906585f};
    float velocity_kp{73.0f*57.2957795f*3/16384}, velocity_ki{0}, position_kp{7.5f};
    float velocity_kd{}, feedback_alpha{}, derivative_tau_s{};
    bool encoder_velocity{};
};

// GM6020 单电机台架: 只负责命令/观测装配、运行时长与心跳有效期; 控制、限幅与授权
// 全部交给正式 DjiAxis, 台架内不再有第二套 PID 或位置→速度换算.
// 电流单位 A, 速度 rad/s. 位置是相对显式参考的累计角, 不再每次开始自动归零:
// 运行前必须由操作者下发建立参考的请求, 停止再启动不会重置已有参考.
// 独立单电机夹具, 不代表整车安全参数.
class Gm6020Bench {
public:
    static constexpr float amp_full_scale = 3.0f;
    static constexpr std::uint16_t raw_full_scale = 16384;
    static constexpr ec::Lifetime feedback_ttl{50};
    static constexpr ec::Lifetime heartbeat_ttl{500};
    static constexpr float max_dt_s = 0.03f;

    explicit Gm6020Bench(std::uint8_t device_id=1) noexcept
        : device_({1, ec::drivers::DjiMode::gm6020_current, device_id, 1, 1, feedback_ttl, 60, raw_full_scale}) {}

    void receive(std::uint8_t bus, const ec::transport::CanFrame& frame, ec::Tick at, ec::MicroTick at_us) noexcept {
        if (device_.receive(bus, frame, at) == ec::drivers::DjiReceiveResult::accepted) {
            encoder_ = static_cast<std::uint16_t>((static_cast<unsigned>(frame.data[0]) << 8) | frame.data[1]);
            rpm_ = static_cast<std::int16_t>((static_cast<unsigned>(frame.data[2]) << 8) | frame.data[3]);
            feedback_us_ = at_us; ++feedback_sequence_;
            encoder_velocity_.receive(encoder_,at,at_us);
        }
    }
    // 三种模式都要求启动瞬间已有新鲜反馈; 位置模式还要求已有有效参考.
    bool start(BenchSettings cfg, std::uint16_t token, ec::Tick now, ec::MicroTick now_us) noexcept;
    // 同模式目标更新不重建轴/参考/积分; ramp_us=0 为阶跃, 否则是一次有限线性斜坡.
    bool update_target(std::uint16_t token, float value, std::uint32_t ramp_us,
        ec::Tick now, ec::MicroTick now_us) noexcept;
    bool excite(std::uint16_t token, BenchExcitation wave, ec::Tick now, ec::MicroTick now_us) noexcept;
    // 心跳只是刷新目标有效期: 过期后不能靠下一次心跳复活.
    bool heartbeat(std::uint16_t token, ec::Tick now) noexcept;
    // 以当前位置为零点建立累计角参考; 与启停分离, 也不清除或伪造多圈历史.
    bool set_reference(ec::Tick now) noexcept;
    void stop(BenchReason reason=BenchReason::stopped) noexcept;
    void invalid_request() noexcept { request_result_ = BenchReason::invalid_request; }
    // 返回本次命令电流 (A); 未运行时为零, 由板级装配显式提交给组发送器.
    float step(ec::Tick now, ec::MicroTick now_us) noexcept;
    static bool valid(const BenchSettings& c) noexcept;

    [[nodiscard]] std::uint16_t encoder() const noexcept { return encoder_; }
    [[nodiscard]] const ec::drivers::DjiDevice& device() const noexcept { return device_; }
    [[nodiscard]] bool active() const noexcept { return active_; }
    [[nodiscard]] BenchReason reason() const noexcept { return reason_; }
    [[nodiscard]] BenchReason request_result() const noexcept { return request_result_; }
    [[nodiscard]] std::uint16_t token() const noexcept { return token_; }
    [[nodiscard]] float current() const noexcept { return current_; }
    [[nodiscard]] const BenchSettings& settings() const noexcept { return cfg_; }
    [[nodiscard]] float applied_target() const noexcept { return applied_target_; }
    [[nodiscard]] ec::MicroTick control_us() const noexcept { return previous_us_; }
    [[nodiscard]] ec::MicroTick feedback_us() const noexcept { return feedback_us_; }
    [[nodiscard]] std::uint32_t dt_us() const noexcept { return dt_us_; }
    [[nodiscard]] std::uint32_t feedback_sequence() const noexcept { return feedback_sequence_; }
    [[nodiscard]] std::uint32_t cycle_sequence() const noexcept { return cycle_sequence_; }
    [[nodiscard]] std::uint32_t reference_epoch() const noexcept { return reference_epoch_; }
    [[nodiscard]] std::uint32_t settings_epoch() const noexcept { return settings_epoch_; }
    [[nodiscard]] std::int16_t rpm() const noexcept { return rpm_; }
    [[nodiscard]] ec::Sample<float> estimated_velocity() const noexcept { return encoder_velocity_.value(); }
    [[nodiscard]] std::uint8_t events() const noexcept { return events_; }
    [[nodiscard]] bool ramping() const noexcept { return ramp_us_ != 0; }
    [[nodiscard]] ec::motion::AxisStatus axis_status() const noexcept { return axis_status_; }
    [[nodiscard]] const ec::control::PidDiagnostics* velocity_diagnostics() const noexcept {
        return computed_ && axis_ ? &axis_->velocity_diagnostics() : nullptr;
    }
    [[nodiscard]] const ec::control::PidDiagnostics* position_diagnostics() const noexcept {
        return computed_ && axis_ ? &axis_->position_diagnostics() : nullptr;
    }
    [[nodiscard]] unsigned group_number() const noexcept { return (device_.config().device_id-1u)/4u+1u; }
    [[nodiscard]] unsigned slot() const noexcept { return (device_.config().device_id-1u)%4u; }
    [[nodiscard]] std::array<const ec::drivers::DjiDevice*,4> group_devices() const noexcept {
        std::array<const ec::drivers::DjiDevice*,4> slots{}; slots[slot()]=&device_; return slots;
    }

private:
    [[nodiscard]] ec::motion::DjiAxisConfig axis_config() const noexcept;
    [[nodiscard]] bool apply_target(ec::Tick now) noexcept;
    [[nodiscard]] BenchReason reason_for(ec::motion::AxisStatus status, ec::Tick now) const noexcept;
    ec::drivers::DjiDevice device_;
    BenchEncoderVelocity encoder_velocity_;
    std::optional<ec::motion::DjiAxis> axis_;
    BenchSettings cfg_{};
    std::optional<BenchExcitation> excitation_;
    ec::MicroTick excitation_started_{};
    bool active_{};
    BenchReason reason_{BenchReason::idle},request_result_{BenchReason::idle};
    std::uint16_t token_{},encoder_{};
    ec::Tick started_{},last_command_{};
    ec::MicroTick previous_us_{},feedback_us_{},ramp_started_{};
    std::uint32_t dt_us_{},feedback_sequence_{},cycle_sequence_{},reference_epoch_{},settings_epoch_{},ramp_us_{};
    std::int16_t rpm_{};
    std::uint8_t pending_events_{},events_{}; // bit0 目标请求, bit1 显式参考; 每周期消费一次.
    ec::motion::AxisStatus axis_status_{ec::motion::AxisStatus::idle};
    bool computed_{};
    float current_{},applied_target_{},ramp_from_{};
};

// 台架按本次运行的参数重新组装一次轴对象: 增益与限幅是构造期配置,
// 设备则一直保留, 因此位置参考和累计角不会被重新开始测试清掉.
inline ec::motion::DjiAxisConfig Gm6020Bench::axis_config() const noexcept {
    const ec::control::CurrentScale scale{amp_full_scale, static_cast<float>(raw_full_scale)};
    const float raw_limit = ec::control::amps_to_raw(scale, cfg_.current_limit);
    // 旧参数以 A/(rad/s) 给出, 轴的环路归一化以电流上限为基准, 因此在此换算一次.
    return {cfg_.current_limit,
        {ec::control::gain_amp_to_normalized(scale, raw_limit, cfg_.velocity_kp),
         ec::control::gain_amp_to_normalized(scale, raw_limit, cfg_.velocity_ki),
         ec::control::gain_amp_to_normalized(scale, raw_limit, cfg_.velocity_kd), 1.0f, -1.0f, 1.0f, cfg_.derivative_tau_s},
        {cfg_.position_kp, 0.0f, 0.0f, 0.0f, -cfg_.speed_limit, cfg_.speed_limit},
        ec::control::ArcMode::continuous, heartbeat_ttl, cfg_.speed_limit, max_dt_s, cfg_.feedback_alpha};
}

inline bool Gm6020Bench::apply_target(ec::Tick now) noexcept {
    if (!axis_) { return false; }
    switch (cfg_.mode) {
    case BenchMode::current: return axis_->set_current(applied_target_, now);
    case BenchMode::velocity: return axis_->set_velocity(applied_target_, now);
    case BenchMode::position: return axis_->set_position(applied_target_, now);
    }
    return false;
}

inline bool Gm6020Bench::start(BenchSettings cfg, std::uint16_t token, ec::Tick now, ec::MicroTick now_us) noexcept {
    if (active_) { request_result_=BenchReason::busy; return false; }
    if (!valid(cfg)) { request_result_=BenchReason::invalid_request; return false; }
    cfg_=cfg;
    const auto state=device_.state(now);
    if (!state.velocity_rad_s.fresh_at(now,feedback_ttl)) { request_result_=BenchReason::feedback_lost; return false; }
    if (cfg_.encoder_velocity && cfg_.mode!=BenchMode::current &&
        !encoder_velocity_.value().fresh_at(now,feedback_ttl)) {
        request_result_=BenchReason::feedback_lost;return false;
    }
    // 位置参考不再由 start 自动建立: 没有参考就不允许进入位置模式.
    if (cfg_.mode==BenchMode::position && !state.position_rad.fresh_at(now,feedback_ttl)) {
        request_result_=BenchReason::position_lost; return false;
    }
    axis_.reset();
    axis_.emplace(axis_config(),device_);
    if (!axis_->configured()) { request_result_=BenchReason::invalid_request; return false; }
    const auto mode=cfg_.mode==BenchMode::current ? ec::control::ControlMode::current
        : cfg_.mode==BenchMode::velocity ? ec::control::ControlMode::velocity
                                         : ec::control::ControlMode::position;
    if (!axis_->enable(mode,ec::CommandSource::remote,true,true)) {
        request_result_=BenchReason::invalid_request; return false;
    }
    applied_target_=cfg_.target;
    if (!apply_target(now)) { axis_->disable(); request_result_=BenchReason::invalid_request; return false; }
    token_=token; started_=last_command_=now; previous_us_=now_us; ramp_us_=0; excitation_.reset();
    ++settings_epoch_; pending_events_|=1;
    current_=0; computed_=false; active_=true; reason_=request_result_=BenchReason::running;
    return true;
}

inline bool Gm6020Bench::heartbeat(std::uint16_t token, ec::Tick now) noexcept {
    if (!active_ || !axis_ || token!=token_) { request_result_=BenchReason::invalid_request; return false; }
    // 有效期已过时不得靠这次心跳抢在控制任务前复活.
    if (!ec::fresh(now,last_command_,heartbeat_ttl)) { stop(BenchReason::command_timeout); return false; }
    if (!apply_target(now)) { stop(BenchReason::invalid_request); return false; }
    last_command_=now; request_result_=BenchReason::running; return true;
}

inline bool Gm6020Bench::update_target(std::uint16_t token, float value, std::uint32_t ramp_us,
    ec::Tick now, ec::MicroTick now_us) noexcept {
    if (!active_ || token!=token_) { request_result_=BenchReason::invalid_request; return false; }
    if (!ec::fresh(now,last_command_,heartbeat_ttl)) { stop(BenchReason::command_timeout); return false; }
    auto next=cfg_; next.target=value;
    if (!valid(next) || ramp_us>=0x80000000u) { request_result_=BenchReason::invalid_request; return false; }
    excitation_.reset();
    ramp_from_=applied_target_; ramp_started_=now_us; ramp_us_=ramp_us; cfg_.target=value;
    if (!ramp_us_) { applied_target_=value; }
    if (!apply_target(now)) { stop(BenchReason::invalid_request); return false; }
    pending_events_|=1; last_command_=now; request_result_=BenchReason::running; return true;
}

inline bool Gm6020Bench::excite(std::uint16_t token, BenchExcitation wave, ec::Tick now,
    ec::MicroTick now_us) noexcept {
    if (!active_ || token!=token_ || !wave.valid()) { invalid_request(); return false; }
    if (!ec::fresh(now,last_command_,heartbeat_ttl)) { stop(BenchReason::command_timeout); return false; }
    auto lower=cfg_,upper=cfg_;
    lower.target=wave.bias-wave.amplitude; upper.target=wave.bias+wave.amplitude;
    if (!valid(lower) || !valid(upper)) { invalid_request(); return false; }
    excitation_=wave; excitation_started_=now_us; ramp_us_=0; cfg_.target=wave.bias;
    applied_target_=wave.value(0); pending_events_|=1;
    if (!apply_target(now)) { stop(BenchReason::invalid_request); return false; }
    last_command_=now; request_result_=BenchReason::running; return true;
}

inline bool Gm6020Bench::set_reference(ec::Tick now) noexcept {
    // 运行中建立参考会同时清控制器历史, 避免测量跳变经 D 项产生冲击.
    const bool accepted=axis_ ? axis_->reference_position(0.0f,now) : device_.reference_position(0.0f,now);
    if (accepted) { ++reference_epoch_; pending_events_|=2; }
    return accepted;
}

inline void Gm6020Bench::stop(BenchReason reason) noexcept {
    if (axis_) { axis_->disable(); }
    active_=false; current_=0; computed_=false; ramp_us_=0; excitation_.reset(); reason_=request_result_=reason;
}

inline BenchReason Gm6020Bench::reason_for(ec::motion::AxisStatus status, ec::Tick now) const noexcept {
    using ec::motion::AxisStatus;
    switch (status) {
    case AxisStatus::stale_target: return BenchReason::command_timeout;
    case AxisStatus::invalid_target: return BenchReason::invalid_request;
    case AxisStatus::invalid_dt: return BenchReason::timing;
    case AxisStatus::no_feedback:
    case AxisStatus::stale_feedback:
        // 位置模式缺参考与速度反馈丢失是不同故障原因, 状态回传需要区分.
        return cfg_.mode==BenchMode::position && !device_.state(now).position_rad.fresh_at(now,feedback_ttl)
            ? BenchReason::position_lost : BenchReason::feedback_lost;
    case AxisStatus::idle: return BenchReason::stopped;
    default: return BenchReason::invalid_request;
    }
}

inline float Gm6020Bench::step(ec::Tick now, ec::MicroTick now_us) noexcept {
    dt_us_=now_us.us-previous_us_.us; previous_us_=now_us;
    events_=pending_events_; pending_events_=0; ++cycle_sequence_; computed_=false;
    axis_status_=ec::motion::AxisStatus::idle;
    if (!active_ || !axis_) { return current_=0.0f; }
    if (!ec::fresh(now,last_command_,heartbeat_ttl)) { stop(BenchReason::command_timeout); return current_=0.0f; }
    if (cfg_.duration_s>0 && now.ms-started_.ms>=static_cast<std::uint32_t>(cfg_.duration_s*1000)) {
        stop(BenchReason::duration); return current_=0.0f;
    }
    // 台架级联机检查: 电流模式不闭环, 但电机不再回传时不得继续推进电流.
    if (!device_.state(now).velocity_rad_s.fresh_at(now,feedback_ttl)) {
        stop(BenchReason::feedback_lost); return current_=0.0f;
    }
    // 同一微秒重复调用不发旧命令; 30 ms 是既有时序契约, 不是新加的台架门禁.
    if (dt_us_==0) { return current_=0.0f; }
    if (dt_us_>30000u) {
        axis_status_=ec::motion::AxisStatus::invalid_dt; stop(BenchReason::timing); return current_=0.0f;
    }
    if (excitation_) {
        const auto elapsed=now_us.us-excitation_started_.us;
        applied_target_=excitation_->value(elapsed);
        if (elapsed>=excitation_->duration_us) { excitation_.reset(); pending_events_|=1; }
        if (!apply_target(now)) { stop(BenchReason::invalid_request); return current_=0.0f; }
    }
    if (ramp_us_) {
        const auto elapsed=now_us.us-ramp_started_.us;
        const float t=elapsed>=ramp_us_ ? 1.0f : static_cast<float>(elapsed)/static_cast<float>(ramp_us_);
        applied_target_=(1.0f-t)*ramp_from_+t*cfg_.target;
        if (elapsed>=ramp_us_) { ramp_us_=0; }
        if (!apply_target(now)) { stop(BenchReason::invalid_request); return current_=0.0f; }
    }
    const float dt_s=static_cast<float>(dt_us_)*1e-6f;
    const auto result=cfg_.encoder_velocity && cfg_.mode!=BenchMode::current
        ? axis_->tick_joint_velocity(now,dt_s,encoder_velocity_.value()) : axis_->tick(now,dt_s);
    axis_status_=result.status;
    if (result.status!=ec::motion::AxisStatus::active) {
        stop(reason_for(result.status,now));
        return current_=0.0f;
    }
    computed_=true;
    return current_=*result.normalized_output*amp_full_scale;
}

inline bool Gm6020Bench::valid(const BenchSettings& c) noexcept {
    for (float v:{c.target,c.duration_s,c.current_limit,c.speed_limit,c.velocity_kp,c.velocity_ki,c.position_kp,
        c.velocity_kd,c.feedback_alpha,c.derivative_tau_s}) {
        if (!std::isfinite(v)) { return false; }
    }
    if (c.duration_s<0 || c.duration_s>3600 || c.current_limit<=0 || c.current_limit>amp_full_scale ||
        c.speed_limit<=0 || c.speed_limit>60 || c.velocity_kp<0 ||
        c.velocity_ki<0 || c.position_kp<=0 || c.velocity_kd<0 || c.feedback_alpha<0 || c.feedback_alpha>=1 || c.derivative_tau_s<0) {
        return false;
    }
    switch(c.mode) {
    case BenchMode::current: return std::abs(c.target)<=c.current_limit;
    case BenchMode::velocity: return std::abs(c.target)<=c.speed_limit;
    case BenchMode::position: return true; // 相对累计角, 不限制为单圈.
    default: return false;
    }
}
} // 命名空间 ec::sentinel
