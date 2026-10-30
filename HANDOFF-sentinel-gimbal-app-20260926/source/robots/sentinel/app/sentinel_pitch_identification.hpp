#pragma once
// Additive Pitch MIT square-torque experiment. No authority grant or target renewal.
// The trusted policy is absent in every deployed configuration by default.
#include "ec/drivers/dm.hpp"
#include "ec/execution_gate.hpp"
#include "ec/time.hpp"
#include <cmath>
#include <cstdint>

namespace ec::sentinel::app {
struct PitchIdentificationPolicy {
    float min_rad{}, max_rad{}, max_amplitude_nm{}, max_expected_torque_nm{};
    float max_velocity_rad_s{}, max_tracking_error_rad{};
    std::uint32_t min_half_period_us{}, max_duration_us{}, max_feedback_age_us{}, max_step_us{};
    // 采集门槛: 为真时启动前必须已有同 run 的连续 control 域 6 采集流. 默认关闭,
    // 既有聚合初始化因此保持有效; 不在此处引入任何物理默认值.
    bool require_capture{false};
    // 实车部署显式开关: 只有配置持有者显式 opted-in 才参与执行器可用性判定. 默认关闭.
    bool deployment_opt_in{false};
    [[nodiscard]] bool valid() const noexcept {
        return std::isfinite(min_rad) && std::isfinite(max_rad) && min_rad < max_rad &&
            std::isfinite(max_amplitude_nm) && max_amplitude_nm > 0 &&
            std::isfinite(max_expected_torque_nm) && max_expected_torque_nm >= max_amplitude_nm &&
            std::isfinite(max_velocity_rad_s) && max_velocity_rad_s > 0 &&
            std::isfinite(max_tracking_error_rad) && max_tracking_error_rad > 0 &&
            min_half_period_us > 0 && max_duration_us > 0 && max_duration_us < 0x80000000u &&
            max_feedback_age_us > 0 && max_feedback_age_us < 0x80000000u &&
            max_step_us > 0 && max_step_us < 0x80000000u;
    }
};
struct PitchIdentificationRecipe {
    std::uint32_t run_id{};
    float amplitude_nm{};
    std::uint32_t half_period_us{}, baseline_us{}, cycles{}, final_hold_us{};
    [[nodiscard]] std::uint64_t duration_us() const noexcept {
        return baseline_us + 2ull*half_period_us*cycles + final_hold_us;
    }
};
enum class PitchIdentificationState : std::uint8_t { idle, baseline, square, final_hold, complete, fault };
// 数值即协议状态字节: 0..10 的既有编码保持不变, 新增采集故障固定为 11 (10 预留未用).
enum class PitchIdentificationFault : std::uint8_t { none, authority, feedback, isolation, envelope, binding, timing, torque, transmit, cancelled, capture = 11 };
struct PitchIdentificationEvidence {
    bool maintenance{}, executing{}, enabled{}, feedback_valid{}, isolated{};
    Grant grant{};
    std::uint32_t config_epoch{}, reference_epoch{}, feedback_age_us{};
    float target_rad{}, position_rad{}, velocity_rad_s{}, kp{}, kd{};
    float native_target_rad{}, native_position_rad{}, encoded_kp{}, encoded_kd{}, encoded_velocity{};
    drivers::DmMitLimits limits{};
};

class PitchIdentification {
public:
    [[nodiscard]] bool start(const PitchIdentificationPolicy& policy, PitchIdentificationRecipe recipe,
        const PitchIdentificationEvidence& evidence, MicroTick now) noexcept {
        if (active() || latched() || !policy.valid() || !recipe.run_id || recipe.run_id <= last_run_id_ ||
            !std::isfinite(recipe.amplitude_nm) || recipe.amplitude_nm <= 0 ||
            recipe.amplitude_nm > policy.max_amplitude_nm || !recipe.cycles ||
            recipe.half_period_us < policy.min_half_period_us || recipe.half_period_us > policy.max_duration_us ||
            recipe.cycles > policy.max_duration_us / 2u / recipe.half_period_us ||
            recipe.baseline_us < policy.min_half_period_us || recipe.final_hold_us < policy.min_half_period_us ||
            recipe.duration_us() > policy.max_duration_us) { return false; }
        // Failed startup must not disturb the existing holding target.
        if (check(evidence, policy) != PitchIdentificationFault::none) { return false; }
        policy_ = policy; recipe_ = recipe; binding_ = evidence;
        start_ = last_ = now; stepped_ = false; last_run_id_ = recipe.run_id;
        state_ = PitchIdentificationState::baseline; fault_ = PitchIdentificationFault::none;
        requested_nm_ = encoded_nm_ = 0;
        return true;
    }
    [[nodiscard]] float tick(const PitchIdentificationEvidence& evidence, MicroTick now) noexcept {
        if (!active()) { return 0; }
        const auto fail = check(evidence, policy_);
        if (fail != PitchIdentificationFault::none) { trip(fail); return 0; }
        if (evidence.grant != binding_.grant || evidence.config_epoch != binding_.config_epoch ||
            evidence.reference_epoch != binding_.reference_epoch || evidence.target_rad != binding_.target_rad ||
            evidence.kp != binding_.kp || evidence.kd != binding_.kd ||
            evidence.native_target_rad != binding_.native_target_rad) {
            trip(PitchIdentificationFault::binding); return 0;
        }
        const auto gap = now.us-last_.us;
        if (gap > policy_.max_step_us || (stepped_ && gap == 0)) {
            trip(PitchIdentificationFault::timing); return 0;
        }
        last_ = now; stepped_ = true;
        const auto elapsed = now.us-start_.us;
        if (elapsed >= 0x80000000u) { trip(PitchIdentificationFault::timing); return 0; }
        requested_nm_ = 0;
        if (elapsed < recipe_.baseline_us) { state_ = PitchIdentificationState::baseline; }
        else if (elapsed < recipe_.baseline_us + 2ull*recipe_.half_period_us*recipe_.cycles) {
            state_ = PitchIdentificationState::square;
            requested_nm_ = ((elapsed-recipe_.baseline_us)/recipe_.half_period_us)%2 == 0 ?
                recipe_.amplitude_nm : -recipe_.amplitude_nm;
        } else { state_ = PitchIdentificationState::final_hold; }
        const auto raw = drivers::dm_encode_torque(evidence.limits, requested_nm_);
        const auto encoded = raw ? drivers::dm_decode_torque(evidence.limits, *raw) : std::nullopt;
        if (!encoded) { trip(PitchIdentificationFault::torque); return 0; }
        encoded_nm_ = *encoded;
        const float expected = evidence.encoded_kp*(evidence.native_target_rad-evidence.native_position_rad) +
            evidence.encoded_kd*(evidence.encoded_velocity-evidence.velocity_rad_s) + encoded_nm_;
        if (!std::isfinite(expected) || std::fabs(expected) > policy_.max_expected_torque_nm) {
            trip(PitchIdentificationFault::torque); return 0;
        }
        if (elapsed >= recipe_.duration_us()) { state_ = PitchIdentificationState::complete; }
        return requested_nm_;
    }
    void trip(PitchIdentificationFault fault) noexcept {
        if (state_ == PitchIdentificationState::idle || latched()) { return; }
        state_ = PitchIdentificationState::fault; fault_ = fault;
        requested_nm_ = encoded_nm_ = 0;
    }
    // Caller must confirm both old target removal and fresh disabled feedback.
    [[nodiscard]] bool reset(bool no_target, bool disabled_confirmed) noexcept {
        if (!latched() || !no_target || !disabled_confirmed) { return false; }
        state_ = PitchIdentificationState::idle; fault_ = PitchIdentificationFault::none;
        requested_nm_ = encoded_nm_ = 0;
        return true; // last_run_id_ is deliberately not reset
    }
    [[nodiscard]] bool active() const noexcept {
        return state_ == PitchIdentificationState::baseline || state_ == PitchIdentificationState::square ||
            state_ == PitchIdentificationState::final_hold;
    }
    [[nodiscard]] bool latched() const noexcept { return state_ == PitchIdentificationState::fault; }
    [[nodiscard]] PitchIdentificationState state() const noexcept { return state_; }
    [[nodiscard]] PitchIdentificationFault fault() const noexcept { return fault_; }
    [[nodiscard]] std::uint32_t run_id() const noexcept { return recipe_.run_id; }
    [[nodiscard]] float requested_nm() const noexcept { return requested_nm_; }
    [[nodiscard]] float encoded_nm() const noexcept { return encoded_nm_; }
    [[nodiscard]] const PitchIdentificationPolicy& policy() const noexcept { return policy_; }
private:
    [[nodiscard]] static PitchIdentificationFault check(const PitchIdentificationEvidence& e,
        const PitchIdentificationPolicy& p) noexcept {
        if (!e.maintenance || e.grant.source != CommandSource::maintenance || !e.executing) { return PitchIdentificationFault::authority; }
        if (!e.feedback_valid || !e.enabled || e.feedback_age_us > p.max_feedback_age_us) { return PitchIdentificationFault::feedback; }
        if (!e.isolated) { return PitchIdentificationFault::isolation; }
        if (!std::isfinite(e.target_rad) || !std::isfinite(e.position_rad) || !std::isfinite(e.velocity_rad_s) ||
            !std::isfinite(e.kp) || !std::isfinite(e.kd) || e.kp < 0 || e.kd < 0 ||
            !std::isfinite(e.native_target_rad) || !std::isfinite(e.native_position_rad) ||
            !std::isfinite(e.encoded_kp) || !std::isfinite(e.encoded_kd) || !std::isfinite(e.encoded_velocity) ||
            e.target_rad <= p.min_rad || e.target_rad >= p.max_rad ||
            e.position_rad <= p.min_rad || e.position_rad >= p.max_rad ||
            std::fabs(e.velocity_rad_s) > p.max_velocity_rad_s ||
            std::fabs(e.target_rad-e.position_rad) > p.max_tracking_error_rad) { return PitchIdentificationFault::envelope; }
        return PitchIdentificationFault::none;
    }
    PitchIdentificationPolicy policy_{};
    PitchIdentificationRecipe recipe_{};
    PitchIdentificationEvidence binding_{};
    MicroTick start_{}, last_{};
    std::uint32_t last_run_id_{};
    bool stepped_{};
    PitchIdentificationState state_{PitchIdentificationState::idle};
    PitchIdentificationFault fault_{PitchIdentificationFault::none};
    float requested_nm_{}, encoded_nm_{};
};
} // namespace ec::sentinel::app
