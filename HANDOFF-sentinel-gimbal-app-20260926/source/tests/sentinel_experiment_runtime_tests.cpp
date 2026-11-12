#include "sentinel_experiment_runtime.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <type_traits>
#include <utility>

using namespace ec::sentinel::app::excitation;
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "line %d: %s\n", __LINE__, #x); std::abort(); } } while (false)

namespace {
constexpr std::uint32_t half_range = std::uint32_t{1} << 31;
static_assert(static_cast<std::uint8_t>(RuntimeGuardianState::normal) == 0);
static_assert(static_cast<std::uint8_t>(RuntimeGuardianState::not_normal) != 0);
// Encoding contract: oscillation::GuardState uses an independent encoding
// (normal == 2). RuntimeEvidence::guardian_state must carry RuntimeGuardianState
// codes only; board_app maps by explicit comparison (see board_app.cpp).
static_assert(noexcept(evaluate_runtime(std::declval<const RuntimeBinding&>(),
                                       std::declval<const RuntimeQuery&>())));
static_assert(noexcept(RuntimeReport{}.ready()));
static_assert(std::is_trivially_copyable_v<RuntimeQuery>);
static_assert(std::is_trivially_copyable_v<RuntimeEvidence>);
static_assert(std::is_trivially_copyable_v<RuntimeBinding>);
static_assert(std::is_trivially_copyable_v<RuntimeReport>);
static_assert(!RuntimeReport{}.ready());

RuntimeQuery query_fixture() noexcept {
    RuntimeQuery q{};
    q.now_us = 1000; q.boot_id = 11; q.axis_identity = 21;
    q.param_epoch = 31; q.config_epoch = 41; q.reference_epoch = 51;
    q.guardian_version = 61; q.axis = Axis::fire_yaw;
    return q;
}

// Deliberately independent fixture: NEVER derive this snapshot from the query.
RuntimeEvidence evidence_fixture() noexcept {
    RuntimeEvidence e{};
    e.boot_id = 11; e.axis_identity = 21; e.param_epoch = 31;
    e.config_epoch = 41; e.reference_epoch = 51; e.guardian_version = 61;
    e.sampled_us = 995; e.source_id = 71; e.axis = Axis::fire_yaw;
    e.guardian_state = static_cast<std::uint8_t>(RuntimeGuardianState::normal);
    e.approved = true; e.clear = true; e.lease_active = true;
    return e;
}
struct Source {
    RuntimeEvidence snapshot{evidence_fixture()};
    unsigned calls{};
};
RuntimeEvidence snapshot(void* context, const RuntimeQuery&) noexcept {
    auto& source = *static_cast<Source*>(context);
    ++source.calls;
    return source.snapshot;
}
RuntimeEvidence empty_snapshot(void*, const RuntimeQuery&) noexcept { return {}; }
RuntimeBinding binding_fixture(Source& source) noexcept {
    // Synthetic test policy only, not a physical timing recommendation.
    return {snapshot, &source, 71, 10};
}
void expect(const RuntimeBinding& binding, const RuntimeQuery& query, RuntimeStatus status) {
    const auto report = evaluate_runtime(binding, query);
    CHECK(report.status == status);
    CHECK(report.ready() == (status == RuntimeStatus::ready));
}

void absent_and_binding_tests() {
    Source source{};
    auto b = binding_fixture(source);
    const auto q = query_fixture();
    expect({}, q, RuntimeStatus::absent);
    expect({}, {}, RuntimeStatus::absent);
    b.evidence = nullptr;
    expect(b, q, RuntimeStatus::absent);
    CHECK(source.calls == 0);
    b = binding_fixture(source); b.source_id = 0;
    expect(b, q, RuntimeStatus::invalid_binding);
    for (const auto ttl : {0u, half_range, half_range + 1, std::numeric_limits<std::uint32_t>::max()}) {
        b = binding_fixture(source); b.ttl_us = ttl;
        expect(b, q, RuntimeStatus::invalid_binding);
    }
    CHECK(source.calls == 0); // Invalid configuration never invokes a source.
    b = {empty_snapshot, nullptr, 71, 10};
    expect(b, q, RuntimeStatus::identity); // Missing snapshot is not ready.
    b = binding_fixture(source);
    expect(b, {}, RuntimeStatus::identity); // Missing expectations also fail.
    expect(b, q, RuntimeStatus::ready);
    const auto report = evaluate_runtime(b, q);
    CHECK(report.evidence.sampled_us == 995);
    CHECK(report.evidence.source_id == 71);
}

void identity_tests() {
    Source source{};
    const auto b = binding_fixture(source);
    constexpr std::array query_members{
        &RuntimeQuery::boot_id, &RuntimeQuery::axis_identity, &RuntimeQuery::param_epoch,
        &RuntimeQuery::config_epoch, &RuntimeQuery::reference_epoch, &RuntimeQuery::guardian_version};
    constexpr std::array evidence_members{
        &RuntimeEvidence::boot_id, &RuntimeEvidence::axis_identity, &RuntimeEvidence::param_epoch,
        &RuntimeEvidence::config_epoch, &RuntimeEvidence::reference_epoch, &RuntimeEvidence::guardian_version};
    for (std::size_t i = 0; i < query_members.size(); ++i) {
        auto q = query_fixture(); q.*query_members[i] = 0;
        expect(b, q, RuntimeStatus::identity);
        // Matching zero identities must never authorize a snapshot either.
        source.snapshot.*evidence_members[i] = 0;
        expect(b, q, RuntimeStatus::identity);
        source.snapshot = evidence_fixture();
        q = query_fixture(); ++(q.*query_members[i]);
        expect(b, q, RuntimeStatus::identity);
        q = query_fixture(); source.snapshot.*evidence_members[i] = 0;
        expect(b, q, RuntimeStatus::identity);
        source.snapshot = evidence_fixture(); ++(source.snapshot.*evidence_members[i]);
        expect(b, q, RuntimeStatus::identity);
        source.snapshot = evidence_fixture();
    }
    for (const auto id : {0u, 70u, 72u}) {
        source.snapshot.source_id = id;
        expect(b, query_fixture(), RuntimeStatus::identity);
        CHECK(evaluate_runtime(b, query_fixture()).evidence.source_id == id);
    }
    source.snapshot = evidence_fixture();
    auto wrong_pin = b; wrong_pin.source_id = 72;
    expect(wrong_pin, query_fixture(), RuntimeStatus::identity);
    // Every byte other than FireYaw is rejected, even a matching pitch domain.
    for (unsigned axis = 0; axis < 256; ++axis) {
        if (axis == static_cast<unsigned>(Axis::fire_yaw)) continue;
        auto q = query_fixture(); q.axis = static_cast<Axis>(axis);
        expect(b, q, RuntimeStatus::identity);
        source.snapshot.axis = q.axis;
        expect(b, q, RuntimeStatus::identity);
        q = query_fixture();
        expect(b, q, RuntimeStatus::identity);
        source.snapshot = evidence_fixture();
    }
}

void age_tests() {
    Source source{};
    auto b = binding_fixture(source);
    auto q = query_fixture();
    for (const auto age : {0u, 1u, 9u, 10u, 11u, half_range - 1, half_range,
                           half_range + 1, std::numeric_limits<std::uint32_t>::max()}) {
        source.snapshot.sampled_us = q.now_us - age;
        expect(b, q, age < b.ttl_us ? RuntimeStatus::ready : RuntimeStatus::stale);
    }
    source.snapshot.sampled_us = q.now_us + 1;
    expect(b, q, RuntimeStatus::stale); // Future by one microsecond.
    b.ttl_us = 1;
    source.snapshot.sampled_us = q.now_us;
    expect(b, q, RuntimeStatus::ready);
    --source.snapshot.sampled_us;
    expect(b, q, RuntimeStatus::stale);
    b.ttl_us = half_range - 1;
    source.snapshot.sampled_us = q.now_us - (half_range - 2);
    expect(b, q, RuntimeStatus::ready);
    --source.snapshot.sampled_us;
    expect(b, q, RuntimeStatus::stale);
    b = binding_fixture(source); q.now_us = 3;
    source.snapshot.sampled_us = std::numeric_limits<std::uint32_t>::max() - 2;
    expect(b, q, RuntimeStatus::ready); // Rollover age is six, not negative.
    q.now_us = 7;
    expect(b, q, RuntimeStatus::stale); // Rollover exact TTL is expired.
    q.now_us = std::numeric_limits<std::uint32_t>::max();
    source.snapshot.sampled_us = 0;
    expect(b, q, RuntimeStatus::stale); // Future across rollover.
    q.now_us = 0; source.snapshot.sampled_us = 0;
    expect(b, q, RuntimeStatus::ready); // Timestamp zero itself is valid.
}

void guardian_and_revocation_tests() {
    Source source{};
    const auto b = binding_fixture(source);
    const auto q = query_fixture();
    for (unsigned state = 1; state < 256; ++state) {
        source.snapshot.guardian_state = static_cast<std::uint8_t>(state);
        expect(b, q, RuntimeStatus::inhibited);
    }
    // The board's explicit not-normal mapping must never admit.
    source.snapshot.guardian_state =
        static_cast<std::uint8_t>(RuntimeGuardianState::not_normal);
    expect(b, q, RuntimeStatus::inhibited);
    // All boolean combinations: none may bypass a missing condition.
    for (unsigned bits = 0; bits < 32; ++bits) {
        source.snapshot = evidence_fixture();
        auto& e = source.snapshot;
        e.approved = (bits & 1) != 0; e.clear = (bits & 2) != 0;
        e.inhibited = (bits & 4) != 0; e.latched = (bits & 8) != 0;
        e.lease_active = (bits & 16) != 0;
        const auto status = e.inhibited || e.latched ? RuntimeStatus::inhibited :
            !e.approved || !e.clear ? RuntimeStatus::not_approved :
            RuntimeStatus::ready; // Parameter lease never authorizes or blocks motion.
        expect(b, q, status);
    }
    source.snapshot = evidence_fixture();
    const auto prior = evaluate_runtime(b, q);
    CHECK(prior.ready());
    source.snapshot.approved = false;
    expect(b, q, RuntimeStatus::not_approved);
    CHECK(!evaluate_runtime(b, q).evidence.approved);
    source.snapshot = evidence_fixture(); source.snapshot.lease_active = false;
    expect(b, q, RuntimeStatus::ready); // Absent parameter lease is not absent motion authority.
    source.snapshot = evidence_fixture(); source.snapshot.latched = true;
    expect(b, q, RuntimeStatus::inhibited);
    source.snapshot = {}; // Provider lost its local snapshot after being ready.
    expect(b, q, RuntimeStatus::identity);
    CHECK(source.calls > 32); // Every evaluation observes the current source.
}
} // namespace

int main() {
    absent_and_binding_tests();
    identity_tests();
    age_tests();
    guardian_and_revocation_tests();
    std::puts("sentinel_experiment_runtime_tests: PASS");
}
