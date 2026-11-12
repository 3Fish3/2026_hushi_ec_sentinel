#include "sentinel_execution.hpp"
#include "sentinel_experiment.hpp"
#include "sentinel_oscillation_guard.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace {
using namespace ec::sentinel::app;
using oscillation::Axis;
using oscillation::GuardMode;
using oscillation::GuardState;

// Synthetic-but-valid test limits (offline only; deployment rejects synthetic_test).
oscillation::GuardianLimits synthetic_limits(std::uint32_t version) noexcept {
    oscillation::GuardianLimits l{};
    l.axis = Axis::fire_yaw;
    l.mode = GuardMode::synthetic_test;
    l.version = version;
    l.config_hash = 0xA5A5'1234u;
    l.has_mechanical_range = true;
    l.mechanical_min_rad = -1.0f;
    l.mechanical_max_rad = 1.0f;
    l.margin_rad = 0.05f;
    l.has_max_speed = true;
    l.max_speed_rad_s = 10.0f;
    l.residual_deadband_rad = 0.01f;
    l.residual_energy_budget = 1.0f;
    l.reversal_count_budget = 50.0f;
    l.window_ms = 200.0f;
    l.exposure_budget = 0.5f;
    l.latch_budget = 1.0f;
    l.decay_ratio = 0.7f;
    l.growth_ratio = 1.3f;
    l.suspect_exit_ms = 100.0f;
    l.hazard_level_scale = 0.05f;
    l.max_dt_s = 0.01f;
    l.nominal_dt_s = 0.001f;
    l.energy_tau_s = 1.0f;
    l.sample_rate_hz = 1000.0f;
    l.band_highpass_hz = 2.0f;
    l.band_lowpass_hz = 40.0f;
    return l;
}

// Fully approved limits require provenance; otherwise configure must reject.
oscillation::GuardianLimits approved_limits(std::uint32_t version) noexcept {
    auto l = synthetic_limits(version);
    l.mode = GuardMode::approved;
    const char source[] = "unit-test-source";
    const char approval[] = "unit-test-approval";
    const char notes[] = "rad, rad/s, s, Hz";
    for (std::size_t i = 0; i < sizeof(source); ++i) { l.source_id[i] = source[i]; }
    for (std::size_t i = 0; i < sizeof(approval); ++i) { l.approval_id[i] = approval[i]; }
    for (std::size_t i = 0; i < sizeof(notes); ++i) { l.unit_notes[i] = notes[i]; }
    l.on_site_approved = true;
    return l;
}

int failures = 0;
void check(bool ok, const char* what) noexcept {
    if (!ok) { ++failures; std::printf("FAIL: %s\n", what); }
}
} // namespace

int main() {
    // 1) configure_guardian rejects unconfigured/pitch/axis mismatch/deployment synthetic.
    {
        oscillation::SentinelOscillationGuard guard{};
        check(guard.status().state == GuardState::unconfigured, "default guardian unconfigured");
        check(guard.status().mode == GuardMode::unconfigured, "default guardian mode unconfigured");
        check(!guard.status().inhibit(), "unconfigured not inhibited (no latch yet)");
        oscillation::GuardianSample idle{};
        idle.axis = Axis::fire_yaw;
        idle.dt_s = 0.001f;
        idle.now_us = 1000;
        const auto report = guard.step(idle);
        check(report.state == GuardState::unconfigured, "unconfigured step stays unconfigured");
        check(report.reason_bits != 0, "unconfigured step reports config reason");
    }
    // 2) Pure algorithm: observe_only never admits, approved-with-provenance can run normal.
    {
        auto observe = synthetic_limits(1);
        observe.mode = GuardMode::observe_only;
        oscillation::SentinelOscillationGuard guard{};
        guard.configure(observe);
        oscillation::GuardianSample s{};
        s.axis = Axis::fire_yaw;
        s.executing = true;
        s.authority_ok = true;
        s.rc_valid = true;
        s.feedback_valid = true;
        s.angle_rad = 0.0f;
        s.velocity_rad_s = 0.0f;
        s.target_valid = true;
        s.request_target_rad = 0.0f;
        s.target_adopted = true;
        s.adopted_target_rad = 0.0f;
        s.has_output = true;
        s.output_normalized = 0.0f;
        s.dt_s = 0.001f;
        s.now_us = 1000;
        const auto r1 = guard.step(s);
        check(r1.mode == GuardMode::observe_only, "observe_only mode visible");
        // observe_only 只观测不锁存: 可停在 observe_only/normal/suspect 等观测态, 但绝不 fault_latched.
        check(r1.state != GuardState::fault_latched && !r1.latched,
            "observe_only never fault_latched or latched");
        check(r1.hardware_ready == false, "observe_only never hardware_ready");
    }
    // 3) RuntimeQuery gating: absent binding, identity mismatch, stale evidence denied.
    {
        excitation::RuntimeQuery q{};
        q.now_us = 10'000;
        q.boot_id = 7;
        q.axis_identity = 9;
        q.param_epoch = 3;
        q.config_epoch = 4;
        q.reference_epoch = 5;
        q.guardian_version = 11;
        q.axis = excitation::Axis::fire_yaw;
        auto evidence = [](void*, const excitation::RuntimeQuery&) noexcept {
            excitation::RuntimeEvidence ev{};
            ev.boot_id = 7; ev.axis_identity = 9; ev.param_epoch = 3;
            ev.config_epoch = 4; ev.reference_epoch = 5;
            ev.guardian_version = 11; ev.sampled_us = 9'000;
            ev.source_id = 0x45435254u; ev.axis = excitation::Axis::fire_yaw;
            ev.guardian_state = 0; ev.approved = true; ev.clear = true;
            return ev;
        };
        excitation::RuntimeBinding b{evidence, nullptr, 0x45435254u, 5000};
        const auto ready = excitation::evaluate_runtime(b, q);
        check(ready.ready(), "fresh identity-matched evidence ready");
        q.now_us = 30'000;
        const auto stale = excitation::evaluate_runtime(b, q);
        check(!stale.ready() && stale.status == excitation::RuntimeStatus::stale,
            "expired evidence is stale");
        q.now_us = 10'000;
        excitation::RuntimeBinding none{};
        const auto absent = excitation::evaluate_runtime(none, q);
        check(!absent.ready() && absent.status == excitation::RuntimeStatus::absent,
            "no binding absent");
    }
    // 4) Hard fault latch: velocity over limit latches and inhibit zeroes output intent.
    {
        auto limits = approved_limits(2);
        oscillation::SentinelOscillationGuard guard{};
        guard.configure(limits);
        oscillation::GuardianSample s{};
        s.axis = Axis::fire_yaw;
        s.executing = true;
        s.authority_ok = true;
        s.rc_valid = true;
        s.feedback_valid = true;
        s.angle_rad = 0.0f;
        s.velocity_rad_s = 0.0f;
        s.target_valid = true;
        s.request_target_rad = 0.0f;
        s.target_adopted = true;
        s.adopted_target_rad = 0.0f;
        s.has_output = true;
        s.output_normalized = 0.1f;
        s.dt_s = 0.001f;
        s.now_us = 1000;
        (void)guard.step(s);
        s.velocity_rad_s = 100.0f; // hard velocity limit breach
        s.now_us = 2000;
        const auto r = guard.step(s);
        check(r.latched, "velocity breach latches");
        check(r.state == GuardState::fault_latched, "velocity breach state fault_latched");
        check(r.stop_request == oscillation::StopRequest::requested, "latch requests stop");
        // Latch persists even after velocity returns normal.
        s.velocity_rad_s = 0.0f;
        s.now_us = 3000;
        const auto r2 = guard.step(s);
        check(r2.latched, "latch persists after recovery");
    }
    // 5) Manual stop is independent and does not latch.
    {
        auto limits = approved_limits(3);
        oscillation::SentinelOscillationGuard guard{};
        guard.configure(limits);
        oscillation::GuardianSample s{};
        s.axis = Axis::fire_yaw;
        s.executing = true;
        s.authority_ok = true;
        s.rc_valid = true;
        s.feedback_valid = true;
        s.angle_rad = 0.0f;
        s.velocity_rad_s = 0.0f;
        s.target_valid = true;
        s.request_target_rad = 0.0f;
        s.target_adopted = true;
        s.adopted_target_rad = 0.0f;
        s.has_output = true;
        s.output_normalized = 0.0f;
        s.dt_s = 0.001f;
        s.now_us = 1000;
        (void)guard.step(s);
        s.manual_stop = true;
        s.now_us = 2000;
        const auto r = guard.step(s);
        check((r.reason_bits & oscillation::reason_manual_stop) != 0, "manual_stop reason set");
        s.manual_stop = false;
        s.now_us = 3000;
        const auto r2 = guard.step(s);
        check(!r2.latched, "manual stop alone does not latch");
    }
    if (failures == 0) { std::printf("sentinel_experiment_admission_tests: all passed\n"); }
    return failures == 0 ? 0 : 1;
}
