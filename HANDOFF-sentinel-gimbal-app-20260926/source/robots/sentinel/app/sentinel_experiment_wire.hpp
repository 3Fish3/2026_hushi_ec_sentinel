#pragma once
#include "sentinel_experiment.hpp"
#include <cstddef>
#include <cstdint>
#include <span>
namespace ec::sentinel::app::excitation {
// Body is carried by existing v3 session framing/CRC16, NOT by Profile4.
// No wire command can choose simulation or approve deployment gates.
// Body v1/v2 layouts are unchanged. Body v3 replies are 160 bytes, first 128
// identical to v2 except version. Common request header (12 bytes, LE):
// u8 version/op/axis/pattern; u32 experiment_session/nonce.
// PREPARE(3), ARM(4), HEARTBEAT(2/7), START(5), STOP(6) keep their layouts.
// POSITION(8), 48 bytes: u32 human_token@12; f32 position_tolerance_rad@16,
// velocity_tolerance_rad_s@20, speed_limit_rad_s@24; u32 move_us@28,
// stable_dwell_us@32, positioning_timeout_us@36, hold_timeout_us@40,
// feedback_max_age_us@44. Target/bounds/acceleration/travel/output limits are
// immutable PREPARE fields; target is center_rad, NEVER a fabricated center.
// CONFIRM(9), 16 bytes: explicit HUMAN-action assertion token@12.
// CANCEL(10), 12 bytes: terminal cancel-to-hold once no longer moving;
// mid-move cancellation is rejected (use explicit STOP, not torque-off disguised
// as cancel). No command refreshes/extends hold_timeout_us: it starts at stable
// wait-human and covers confirmation, generation and nominal completion hold.
// New reply suffix: u8 PositionStage@128; zero reserved@129..131;
// u32 remaining human_token@132, captured config_epoch@136/reference_epoch@140;
// f32 target@144/actual@148/velocity@152; u32 feedback_sample_us@156.
// Stage: none0,moving1,settling2,wait_human3,confirmed4,running5,
// holding_complete6,cancelled_hold7,invalidated8. Generation completion keeps
// legacy State::armed while holding; stage6 is NOT legacy stop completion.
// START consumes token and is separate from CONFIRM; hardware never bypasses
// positioning via old START. Ordinary wait/cancel/completion retains reference;
// safety faults, explicit STOP and timeouts use the existing verified stop path.
// HUMAN token is a host assertion, not cryptographic proof of human presence.
class Wire {
public:
    explicit Wire(bool simulation=false) noexcept : simulation_controller_(simulation), use_simulation_(simulation) {}
    Wire(const TrustedDeploymentSafetyConfig& d,const TrustedDeploymentRoot& r) noexcept : hardware_controller_(d,r) {}
    std::size_t handle(std::span<const std::uint8_t> request, const Guards& guards,
        std::uint32_t now_us, std::span<std::uint8_t> response) noexcept;
    const Controller& controller() const noexcept { return use_simulation_ ? simulation_controller_ : hardware_controller_; }
    Controller& controller_for_cycle() noexcept { return use_simulation_ ? simulation_controller_ : hardware_controller_; }
private:
    Controller simulation_controller_{};
    HardwareExperimentController hardware_controller_{};
    bool use_simulation_{};
};
} // namespace ec::sentinel::app::excitation
