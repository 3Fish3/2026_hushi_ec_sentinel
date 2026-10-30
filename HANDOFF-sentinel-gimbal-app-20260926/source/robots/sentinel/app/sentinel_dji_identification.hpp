#pragma once
// Additive DJI (C610/C620/GM6020 电流模式) 方波激励, 用于系统辨识.
// 与 PitchIdentification 结构相同但作用在"电流域": 在位置/速度环输出之上叠加一个已知
// 方波电流分量, 交给执行器按轴电流上限硬限幅, 因此激励永远无法让指令电流超过 current_limit.
// 默认不参与任何部署; 只有显式 policy 且 maintenance 持有、目标新鲜、隔离时才可启动.
#include "ec/execution_gate.hpp"
#include "ec/time.hpp"
#include <cmath>
#include <cstdint>

namespace ec::sentinel::app {
struct DjiIdentificationPolicy {
    float min_rad{}, max_rad{};
    float max_amplitude_a{}, max_expected_current_a{};
    float max_velocity_rad_s{}, max_tracking_error_rad{};
    std::uint32_t min_half_period_us{}, max_duration_us{}, max_feedback_age_us{}, max_step_us{};
    bool require_capture{false};
    bool deployment_opt_in{false};
    [[nodiscard]] bool valid() const noexcept {
        return std::isfinite(min_rad) && std::isfinite(max_rad) && min_rad < max_rad &&
            std::isfinite(max_amplitude_a) && max_amplitude_a > 0 &&
            std::isfinite(max_expected_current_a) && max_expected_current_a >= max_amplitude_a &&
            std::isfinite(max_velocity_rad_s) && max_velocity_rad_s > 0 &&
            std::isfinite(max_tracking_error_rad) && max_tracking_error_rad > 0 &&
            min_half_period_us > 0 && max_duration_us > 0 && max_duration_us < 0x80000000u &&
            max_feedback_age_us > 0 && max_feedback_age_us < 0x80000000u &&
            max_step_us > 0 && max_step_us < 0x80000000u;
    }
};
struct DjiIdentificationRecipe {
    std::uint32_t run_id{};
    float amplitude_a{};          // 叠加电流方波幅度 (轴侧安培)
    std::uint32_t half_period_us{}, baseline_us{}, cycles{}, final_hold_us{};
    [[nodiscard]] std::uint64_t duration_us() const noexcept {
        return baseline_us + 2ull*half_period_us*cycles + final_hold_us;
    }
};
enum class DjiIdentificationState : std::uint8_t { idle, baseline, square, final_hold, complete, fault };
// 数值即协议状态字节; 采集故障固定 11, 与 pitch 对齐.
enum class DjiIdentificationFault : std::uint8_t {
    none, authority, feedback, isolation, envelope, binding, timing, current, transmit, cancelled, capture = 11 };
struct DjiIdentificationEvidence {
    bool maintenance{}, executing{}, enabled{}, feedback_valid{}, isolated{};
    Grant grant{};
    std::uint32_t config_epoch{}, reference_epoch{}, feedback_age_us{};
    float target_rad{}, position_rad{}, velocity_rad_s{};
    float current_limit_a{};   // 轴侧电流上限 (安培); 激励叠加后被它硬限幅
    float loop_current_a{};    // 本周期环路自身请求的电流 (安培), 用于期望上限核对
};

class DjiIdentification {
public:
    [[nodiscard]] bool start(const DjiIdentificationPolicy& policy, DjiIdentificationRecipe recipe,
        const DjiIdentificationEvidence& evidence, MicroTick now) noexcept {
        if (active() || latched() || !policy.valid() || !recipe.run_id || recipe.run_id <= last_run_id_ ||
            !std::isfinite(recipe.amplitude_a) || recipe.amplitude_a <= 0 ||
            recipe.amplitude_a > policy.max_amplitude_a || !recipe.cycles ||
            recipe.half_period_us < policy.min_half_period_us || recipe.half_period_us > policy.max_duration_us ||
            recipe.cycles > policy.max_duration_us / 2u / recipe.half_period_us ||
            recipe.baseline_us < policy.min_half_period_us || recipe.final_hold_us < policy.min_half_period_us ||
            recipe.duration_us() > policy.max_duration_us) { return false; }
        if (check(evidence, policy) != DjiIdentificationFault::none) { return false; }
        policy_ = policy; recipe_ = recipe; binding_ = evidence;
        start_ = last_ = now; stepped_ = false; last_run_id_ = recipe.run_id;
        state_ = DjiIdentificationState::baseline; fault_ = DjiIdentificationFault::none;
        requested_a_ = 0;
        return true;
    }
    [[nodiscard]] float tick(const DjiIdentificationEvidence& evidence, MicroTick now) noexcept {
        if (!active()) { return 0; }
        const auto fail = check(evidence, policy_);
        if (fail != DjiIdentificationFault::none) { trip(fail); return 0; }
        if (evidence.grant != binding_.grant || evidence.config_epoch != binding_.config_epoch ||
            evidence.reference_epoch != binding_.reference_epoch || evidence.target_rad != binding_.target_rad ||
            evidence.current_limit_a != binding_.current_limit_a) {
            trip(DjiIdentificationFault::binding); return 0;
        }
        const auto gap = now.us-last_.us;
        if (gap > policy_.max_step_us || (stepped_ && gap == 0)) {
            trip(DjiIdentificationFault::timing); return 0;
        }
        last_ = now; stepped_ = true;
        const auto elapsed = now.us-start_.us;
        if (elapsed >= 0x80000000u) { trip(DjiIdentificationFault::timing); return 0; }
        requested_a_ = 0;
        if (elapsed < recipe_.baseline_us) { state_ = DjiIdentificationState::baseline; }
        else if (elapsed < recipe_.baseline_us + 2ull*recipe_.half_period_us*recipe_.cycles) {
            state_ = DjiIdentificationState::square;
            requested_a_ = ((elapsed-recipe_.baseline_us)/recipe_.half_period_us)%2 == 0 ?
                recipe_.amplitude_a : -recipe_.amplitude_a;
        } else { state_ = DjiIdentificationState::final_hold; }
        // 期望电流上限核对: 环路请求 + 激励, 不得超过策略上限 (部署时取轴电流上限).
        const float expected = evidence.loop_current_a + requested_a_;
        if (!std::isfinite(expected) || std::fabs(expected) > policy_.max_expected_current_a) {
            trip(DjiIdentificationFault::current); return 0;
        }
        if (elapsed >= recipe_.duration_us()) { state_ = DjiIdentificationState::complete; }
        return requested_a_;
    }
    void trip(DjiIdentificationFault fault) noexcept {
        if (state_ == DjiIdentificationState::idle || latched()) { return; }
        state_ = DjiIdentificationState::fault; fault_ = fault;
        requested_a_ = 0;
    }
    [[nodiscard]] bool reset(bool no_target, bool disabled_confirmed) noexcept {
        if (!latched() || !no_target || !disabled_confirmed) { return false; }
        state_ = DjiIdentificationState::idle; fault_ = DjiIdentificationFault::none;
        requested_a_ = 0;
        return true;
    }
    [[nodiscard]] bool active() const noexcept {
        return state_ == DjiIdentificationState::baseline || state_ == DjiIdentificationState::square ||
            state_ == DjiIdentificationState::final_hold;
    }
    [[nodiscard]] bool latched() const noexcept { return state_ == DjiIdentificationState::fault; }
    [[nodiscard]] DjiIdentificationState state() const noexcept { return state_; }
    [[nodiscard]] DjiIdentificationFault fault() const noexcept { return fault_; }
    [[nodiscard]] std::uint32_t run_id() const noexcept { return recipe_.run_id; }
    [[nodiscard]] float requested_a() const noexcept { return requested_a_; }
    [[nodiscard]] const DjiIdentificationPolicy& policy() const noexcept { return policy_; }
private:
    [[nodiscard]] static DjiIdentificationFault check(const DjiIdentificationEvidence& e,
        const DjiIdentificationPolicy& p) noexcept {
        if (!e.maintenance || e.grant.source != CommandSource::maintenance || !e.executing) {
            return DjiIdentificationFault::authority; }
        if (!e.feedback_valid || !e.enabled || e.feedback_age_us > p.max_feedback_age_us) {
            return DjiIdentificationFault::feedback; }
        if (!e.isolated) { return DjiIdentificationFault::isolation; }
        if (!std::isfinite(e.target_rad) || !std::isfinite(e.position_rad) || !std::isfinite(e.velocity_rad_s) ||
            !std::isfinite(e.current_limit_a) || e.current_limit_a <= 0 ||
            e.target_rad <= p.min_rad || e.target_rad >= p.max_rad ||
            e.position_rad <= p.min_rad || e.position_rad >= p.max_rad ||
            std::fabs(e.velocity_rad_s) > p.max_velocity_rad_s ||
            std::fabs(e.target_rad-e.position_rad) > p.max_tracking_error_rad) {
            return DjiIdentificationFault::envelope; }
        return DjiIdentificationFault::none;
    }
    DjiIdentificationPolicy policy_{};
    DjiIdentificationRecipe recipe_{};
    DjiIdentificationEvidence binding_{};
    MicroTick start_{}, last_{};
    std::uint32_t last_run_id_{};
    bool stepped_{};
    DjiIdentificationState state_{DjiIdentificationState::idle};
    DjiIdentificationFault fault_{DjiIdentificationFault::none};
    float requested_a_{};
};
} // namespace ec::sentinel::app
