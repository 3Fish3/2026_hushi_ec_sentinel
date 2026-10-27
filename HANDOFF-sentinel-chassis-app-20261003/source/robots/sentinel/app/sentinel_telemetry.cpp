#include "sentinel_telemetry.hpp"
#include "sentinel_protocol.hpp"

namespace ec::sentinel::app {
namespace {
std::uint32_t age(const Sample<float>& value, Tick now) noexcept {
    return value.validity == Validity::valid ? now.ms-value.sampled_at.ms : 0xffffffffu;
}
void twist(PayloadWriter& w, const motion::PlanarTwist& value) noexcept {
    w.f32(value.vx_mps); w.f32(value.vy_mps); w.f32(value.wz_rad_s);
}
} // 编码辅助

DomainMeasurement domain_measurement(const SentinelApp& app, Domain domain, Tick now) noexcept {
    const auto i = domain_index(domain);
    const auto& snapshot = app.snapshot();
    const auto& status = snapshot.domains[i];
    const auto& execution = snapshot.executions[i];
    DomainMeasurement out;
    const bool position = execution.mode == control::ControlMode::position;
    const auto& selected = position ? status.feedback.position_rad : status.feedback.velocity_rad_s;
    out.value = selected.value;
    out.position_age_ms = age(status.feedback.position_rad, now);
    out.velocity_age_ms = age(status.feedback.velocity_rad_s, now);
    bool valid = selected.validity == Validity::valid;
    YawFrame measured_frame = YawFrame::joint;
    if (domain == Domain::main_yaw && position && execution.frame != YawFrame::joint) {
        const auto& platform = snapshot.main_yaw;
        measured_frame = execution.frame;
        out.value = platform.continuous_heading +
            (measured_frame == YawFrame::odom ? platform.odom_offset_rad : 0.0f);
        valid = platform.validity == DomainValidity::valid && platform.local_reference &&
            (measured_frame != YawFrame::odom || platform.odom_aligned);
        out.position_age_ms = out.velocity_age_ms = valid ?
            platform.ages_us[0]/1000u + now.ms-snapshot.at.ms : 0xffffffffu;
    }
    out.frames = static_cast<std::uint8_t>((status.target ? static_cast<unsigned>(status.target->frame) : 0u) |
        (static_cast<unsigned>(execution.frame)<<2) | (static_cast<unsigned>(measured_frame)<<4) |
        (status.target ? 64u : 0u) | (valid ? 128u : 0u));
    return out;
}

std::size_t encode_fast_status(const SentinelApp& app, const SentinelPerception* perception,
    Tick now, std::uint32_t session, std::uint32_t sequence, std::uint32_t boot_id,
    std::span<std::uint8_t> out) noexcept {
    static_assert(fast_status_size <= protocol_payload_max);
    const auto& state = app.snapshot();
    const auto& platform = state.main_yaw;
    const WheelOdometry empty{};
    const auto& wheel = perception ? perception->wheel_odom() : empty;
    PayloadWriter w{out};
    w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(MessageType::status_snapshot));
    w.u32(session); w.u32(sequence); w.u8(static_cast<std::uint8_t>(StatusPage::fast));
    w.u32(state.config_epoch); w.u32(state.reference_epoch); w.u32(now.ms);
    w.u32(state.at.ms); w.u32(platform.evaluated_at.us); w.u32(boot_id);
    w.u8(static_cast<std::uint8_t>(state.permission.source));
    w.u8(static_cast<std::uint8_t>((state.permission.authority ? 1u : 0u) |
        (state.permission.manual_stop ? 2u : 0u) | (state.launch_allowed ? 4u : 0u) |
        (state.permission.locally_ready ? 8u : 0u) | (boot_id ? 16u : 0u) |
        (app.main_yaw_degraded() ? 32u : 0u)));
    w.u8(static_cast<std::uint8_t>(app.config().kind));
    w.u8(static_cast<std::uint8_t>(platform.validity)); w.u8(static_cast<std::uint8_t>(platform.reason));
    w.u8(static_cast<std::uint8_t>((platform.local_reference ? 1u : 0u) | (platform.odom_aligned ? 2u : 0u)));
    w.u32(platform.local_epoch); w.u32(platform.alignment_epoch);
    for (const auto domain : {Domain::main_yaw, Domain::fire_yaw, Domain::pitch}) {
        const auto& feedback = state.domains[domain_index(domain)].feedback;
        w.f32(feedback.position_rad.value); w.f32(feedback.velocity_rad_s.value);
        w.u32(age(feedback.position_rad, now)); w.u32(age(feedback.velocity_rad_s, now));
        w.u8(static_cast<std::uint8_t>((feedback.position_rad.validity == Validity::valid ? 1u : 0u) |
            (feedback.velocity_rad_s.validity == Validity::valid ? 2u : 0u)));
    }
    const auto& q = platform.value.orientation;
    w.f32(q.w); w.f32(q.x); w.f32(q.y); w.f32(q.z);
    for (const auto value : platform.value.rate_V) { w.f32(value); }
    w.f32(platform.continuous_heading); w.f32(platform.odom_offset_rad); w.f32(platform.value.heading_gain);
    w.u32(platform.gyro_at.us);
    for (const auto value : platform.ages_us) { w.u32(value); }
    for (const auto value : platform.time_error_us) { w.u32(value); }
    w.u8(static_cast<std::uint8_t>(wheel.validity));
    w.u8(static_cast<std::uint8_t>((wheel.rank_ok ? 1u : 0u) | (wheel.pose.valid ? 2u : 0u) |
        (wheel.updates && wheel.continuous ? 4u : 0u)));
    twist(w, wheel.twist);
    w.f32(wheel.pose.x_m); w.f32(wheel.pose.y_m); w.f32(wheel.pose.yaw_rad);
    w.u32(wheel.updates ? now.ms-wheel.pose.at.ms : 0xffffffffu);
    w.u32(wheel.skew_us); w.f32(wheel.residual_rad_s); w.u32(wheel.gaps);
    w.f32(state.twist_scale);
    return w.overflow || w.size() != fast_status_size ? 0 : w.size();
}
} // 命名空间 ec::sentinel::app
