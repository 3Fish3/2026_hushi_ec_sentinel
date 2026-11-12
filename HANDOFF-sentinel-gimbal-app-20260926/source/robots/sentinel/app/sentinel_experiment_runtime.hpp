#pragma once

#include "sentinel_experiment.hpp"
#include <cstdint>

namespace ec::sentinel::app::excitation {

// Software FireYaw admission only: ready is NOT a physical safety approval.
// All identities and TTL policy must come from independently provisioned local
// state. Query fields express expectations, never approval or measured evidence.
struct RuntimeQuery {
    std::uint32_t now_us{}, boot_id{}, axis_identity{}, param_epoch{};
    std::uint32_t config_epoch{}, reference_epoch{}, guardian_version{};
    Axis axis{}; // Zero is invalid; callers must explicitly select fire_yaw.
};

// Wire/evidence encoding for RuntimeEvidence::guardian_state. This encoding is
// deliberately independent from oscillation::GuardState (whose normal == 2):
// evidence producers must map by explicit comparison, never by static_cast of
// the other enum's underlying value.
enum class RuntimeGuardianState : std::uint8_t { normal = 0, not_normal = 0xFF };

struct RuntimeEvidence {
    std::uint32_t boot_id{}, axis_identity{}, param_epoch{}, config_epoch{};
    std::uint32_t reference_epoch{}, guardian_version{}, sampled_us{}, source_id{};
    Axis axis{};
    std::uint8_t guardian_state{};
    bool approved{}, clear{}, inhibited{}, latched{};
    bool lease_active{}; // Parameter-tuning telemetry only: never grants motion.
};

struct RuntimeBinding {
    // LOCAL TRUSTED SOURCE ONLY. The callback must read an independently owned,
    // coherent snapshot; it must NOT echo query fields to manufacture evidence.
    // The owner guarantees context lifetime and synchronization during evaluation.
    // A null context is permitted for a stateless source. This interface provides
    // no remote trust, signature verification, or signed-attestation guarantee.
    RuntimeEvidence (*evidence)(void*, const RuntimeQuery&) noexcept{};
    void* context{};
    std::uint32_t source_id{}; // Independently pinned, nonzero local source.
    std::uint32_t ttl_us{};    // Explicit policy: 0 < ttl_us < 2^31; no default.
};

enum class RuntimeStatus : std::uint8_t {
    absent, invalid_binding, identity, stale, not_approved, inhibited, no_lease, ready
};

struct RuntimeReport {
    RuntimeStatus status{RuntimeStatus::absent};
    RuntimeEvidence evidence{};
    [[nodiscard]] constexpr bool ready() const noexcept {
        return status == RuntimeStatus::ready;
    }
};

// Each evaluation re-reads evidence: a prior ready report is not a retained lease.
// Denial precedence: missing source, binding, identities, age, inhibition,
// approval/clear. A parameter lease is telemetry, not excitation authority.
// Reports retain returned evidence even on denial.
[[nodiscard]] inline RuntimeReport evaluate_runtime(
    const RuntimeBinding& binding, const RuntimeQuery& query) noexcept {
    if (!binding.evidence) return {};
    constexpr std::uint32_t half_range = std::uint32_t{1} << 31;
    if (!binding.source_id || !binding.ttl_us || binding.ttl_us >= half_range)
        return {RuntimeStatus::invalid_binding, {}};

    const auto evidence = binding.evidence(binding.context, query);
    if (query.axis != Axis::fire_yaw || evidence.axis != Axis::fire_yaw ||
        !query.boot_id || !query.axis_identity || !query.param_epoch ||
        !query.config_epoch || !query.reference_epoch || !query.guardian_version ||
        evidence.boot_id != query.boot_id ||
        evidence.axis_identity != query.axis_identity ||
        evidence.param_epoch != query.param_epoch ||
        evidence.config_epoch != query.config_epoch ||
        evidence.reference_epoch != query.reference_epoch ||
        evidence.guardian_version != query.guardian_version ||
        evidence.source_id != binding.source_id)
        return {RuntimeStatus::identity, evidence};

    // Unsigned modulo subtraction admits rollover but rejects future/ambiguous
    // timestamps (half range or more). Exact TTL is expired, not inclusive.
    const std::uint32_t age = query.now_us - evidence.sampled_us;
    if (age >= half_range || age >= binding.ttl_us)
        return {RuntimeStatus::stale, evidence};
    if (evidence.guardian_state != static_cast<std::uint8_t>(RuntimeGuardianState::normal) ||
        evidence.inhibited || evidence.latched)
        return {RuntimeStatus::inhibited, evidence};
    if (!evidence.approved || !evidence.clear)
        return {RuntimeStatus::not_approved, evidence};
    return {RuntimeStatus::ready, evidence};
}

} // namespace ec::sentinel::app::excitation
