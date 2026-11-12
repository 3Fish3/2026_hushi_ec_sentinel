#pragma once
#include <cstdint>
#include <array>
#include <optional>

namespace ec::sentinel::app::excitation {
enum class Axis : std::uint8_t { fire_yaw=5, pitch=6 };
enum class Pattern : std::uint8_t { bounded_step=1, trapezoid=2 };
enum class State : std::uint8_t { disabled, idle, armed, running, finishing, complete, fault_latched };
enum class Reason : std::uint8_t { none, hardware_blocked, bad_state, invalid_config,
    authorization, manual_stop, guardian, feedback, reference, limits, actuator,
    other_axes, pitch_blocked, timing, link, backpressure, observation, identity,
    position_limit, output_limit, travel_limit, heartbeat_timeout, stop_timeout };
struct Config {
    Axis axis{Axis::fire_yaw}; Pattern pattern{Pattern::bounded_step};
    std::uint32_t session_id{}, nonce{}, param_epoch{}, guardian_version{}, config_hash{}, safety_reference_id{};
    float center_rad{}, amplitude_rad{}, min_rad{}, max_rad{};
    float max_velocity_rad_s{}, max_acceleration_rad_s2{}, max_travel_rad{};
    float max_output_abs{}, max_output_integral{};
    std::uint32_t move_us{}, hold_us{}, duration_us{}, heartbeat_timeout_us{}, stop_timeout_us{};
    std::uint8_t repeats{};
};
struct Guards {
    bool maintenance_authorized{}, operator_approved{}, remote_online{}, manual_stop{};
    bool guardian_approved{}, guardian_clear{}, feedback_fresh{}, reference_valid{}, limits_valid{};
    bool actuator_healthy{}, other_axes_idle{}, pitch_fall_arrest_verified{}, stop_strategy_verified{};
    bool timing_valid{}, link_alive{}, backpressure_ok{}, output_observation_valid{}, stop_confirmed{};
    std::uint32_t param_epoch{}, guardian_version{}, config_hash{}, safety_reference_id{};
    float actual_position_rad{}, observed_output{}, actual_velocity_rad_s{};
    std::uint32_t feedback_sample_us{}, stop_policy_sample_us{}, config_epoch{}, reference_epoch{}, axis_identity{};
    std::uint32_t capture_remaining_us{}, capture_sample_us{};
    bool stop_policy_executed{}, lease_active{};
    std::uint8_t guardian_state{};
};
struct Reference { float position_rad{}, velocity_rad_s{}, acceleration_rad_s2{}; };
// Bounds below are software simulation admission limits, NEVER physical approvals.
[[nodiscard]] bool valid_config(const Config& c) noexcept;
[[nodiscard]] std::optional<Reference> generate(const Config& c, std::uint32_t elapsed_us) noexcept;
enum class StopPolicy : std::uint8_t { unverified, zero_output };
enum class StopStage : std::uint8_t { none, stop_requested, reference_withdrawn, policy_executing, feedback_confirmed, complete, stop_unknown };
// Controlled-build records only. Neither object is writable through the wire API.
// SHA256 detects record changes against an independently pinned local build root;
// it is NOT a remote signature scheme and does not turn PREPARE into approval.
struct TrustedDeploymentSafetyConfig {
    std::uint32_t axis_identity{}, config_epoch{}, reference_epoch{}, guardian_version{}, config_hash{}, safety_reference_id{}, approval_record_version{};
    std::array<std::uint8_t,32> firmware_sha256{}, deployment_sha256{};
    float min_rad{},max_rad{},max_velocity_rad_s{},max_acceleration_rad_s2{},max_travel_rad{},max_output_abs{},max_output_integral{};
    std::uint32_t max_duration_us{},max_heartbeat_timeout_us{},max_stop_timeout_us{};
    float stop_velocity_rad_s{},stop_position_delta_rad{};
    std::uint32_t stop_dwell_us{};
    StopPolicy stop_policy{StopPolicy::unverified};
};
struct TrustedDeploymentRoot {
    std::uint32_t boot_id{},axis_identity{};
    std::array<std::uint8_t,32> firmware_sha256{},deployment_sha256{};
};
std::array<std::uint8_t,32> deployment_digest(const TrustedDeploymentSafetyConfig&) noexcept;
// Explicit measured policy: zero/default values cannot authorize positioning.
enum class PositionStage : std::uint8_t { none, moving, settling, wait_human, confirmed, running, holding_complete, cancelled_hold, invalidated };
struct PositionPolicy {
    std::uint32_t human_token{};
    float position_tolerance_rad{}, velocity_tolerance_rad_s{}, speed_limit_rad_s{};
    std::uint32_t move_us{}, stable_dwell_us{}, positioning_timeout_us{}, hold_timeout_us{}, feedback_max_age_us{};
};
class Controller {
public:
    explicit Controller(bool simulation=false) noexcept : simulation_(simulation),
        state_(simulation ? State::idle : State::disabled), reason_(simulation ? Reason::none : Reason::hardware_blocked) {}
    bool prepare(const Config&, const Guards&, std::uint32_t now_us, bool positioned_session=false) noexcept;
    bool arm(const Guards&, std::uint32_t now_us) noexcept;
    bool start(const Guards&, std::uint32_t now_us) noexcept;
    bool heartbeat(std::uint32_t now_us) noexcept;
    bool request_position(const PositionPolicy&, const Guards&, std::uint32_t now_us) noexcept;
    bool confirm_position(std::uint32_t human_token, const Guards&, std::uint32_t now_us) noexcept;
    bool cancel_position(const Guards&, std::uint32_t now_us) noexcept;
    PositionStage position_stage() const noexcept { return position_stage_; }
    const PositionPolicy& position_policy() const noexcept { return position_policy_; }
    std::uint32_t position_config_epoch() const noexcept { return position_config_epoch_; }
    std::uint32_t position_reference_epoch() const noexcept { return position_reference_epoch_; }
    void stop(std::uint32_t now_us) noexcept;
    std::optional<Reference> tick(const Guards&, std::uint32_t now_us) noexcept;
    State state() const noexcept { return state_; }
    Reason reason() const noexcept { return reason_; }
    bool simulation() const noexcept { return simulation_; }
    const Config& config() const noexcept { return config_; }
    bool hardware_capable() const noexcept { return hardware_; }
    std::uint8_t provisioning_state() const noexcept { return provisioning_state_; }
    const TrustedDeploymentSafetyConfig& deployment() const noexcept { return deployment_; }
    const TrustedDeploymentRoot& root() const noexcept { return root_; }
    void publish_cycle_evidence(bool valid,std::uint32_t now_ms) noexcept { cycle_valid_=valid;cycle_ms_=now_ms; }
    bool cycle_evidence_fresh(std::uint32_t now_ms) const noexcept { return cycle_valid_ && now_ms-cycle_ms_<5; }
    StopStage stop_stage() const noexcept { return stop_stage_; }
    void reference_withdrawn(std::uint32_t now) noexcept;
    void observe_stop(const Guards&,std::uint32_t now) noexcept;
protected:
    Controller(const TrustedDeploymentSafetyConfig&,const TrustedDeploymentRoot&) noexcept;
private:
    Reason check(const Guards&, bool require_capture=true) const noexcept;
    bool position_feedback_valid(const Guards&, std::uint32_t now) const noexcept;
    bool position_at_target(const Guards&) const noexcept;
    std::optional<Reference> position_tick(const Guards&, std::uint32_t now) noexcept;
    PositionStage position_stage_{PositionStage::none};
    PositionPolicy position_policy_{};
    std::uint32_t position_start_us_{}, hold_start_us_{}, stable_start_us_{}, position_sample_us_{};
    std::uint32_t position_config_epoch_{}, position_reference_epoch_{}, position_axis_identity_{};
    float position_origin_rad_{};
    bool stable_started_{}, hold_started_{}, positioned_session_{};
    void fault(Reason r) noexcept { state_=State::fault_latched; reason_=r; if(position_stage_!=PositionStage::none) { position_stage_=PositionStage::invalidated; position_policy_.human_token=0; } }
    void finish(std::uint32_t now) noexcept { state_=State::finishing; finish_us_=now; if(hardware_) stop_stage_=StopStage::stop_requested; }
    bool enabled() const noexcept { return simulation_ || hardware_; }
    bool within_deployment(const Config&) const noexcept;
    const bool simulation_;
    bool hardware_{},cycle_valid_{}; std::uint32_t cycle_ms_{};
    std::uint8_t provisioning_state_{};
    TrustedDeploymentSafetyConfig deployment_{}; TrustedDeploymentRoot root_{};
    StopStage stop_stage_{StopStage::none};
    std::uint32_t withdrawn_us_{}, dwell_start_us_{}, stop_sample_us_{}, stop_policy_start_us_{};
    bool dwell_started_{}; float stop_position_{};
    State state_; Reason reason_; Config config_{};
    bool prepared_{}, first_tick_{true}, previous_stop_confirmed_{};
    std::uint32_t start_us_{}, last_us_{}, heartbeat_us_{}, finish_us_{};
    double travel_{}, output_integral_{};
    float last_position_{};
};
class HardwareExperimentController final : public Controller {
public:
    HardwareExperimentController() noexcept : Controller(false) {}
    HardwareExperimentController(const TrustedDeploymentSafetyConfig& d,const TrustedDeploymentRoot& r) noexcept : Controller(d,r) {}
};
} // namespace ec::sentinel::app::excitation
