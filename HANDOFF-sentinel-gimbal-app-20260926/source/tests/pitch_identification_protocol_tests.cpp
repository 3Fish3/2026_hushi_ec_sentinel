#include "sentinel_app.hpp"
#include "sentinel_config.hpp"
#include "sentinel_execution.hpp"
#include "sentinel_params.hpp"
#include "sentinel_perception.hpp"
#include "sentinel_protocol.hpp"
#include "sentinel_pitch_identification.hpp"
#include "sim_plant.hpp"
#include <array>
#include <cstdio>
#include <limits>
#include <span>
#include <vector>

using namespace ec;
using namespace ec::sentinel::app;
using ec::transport::CanFrame;
using ec::sentinel::host::SoftwareBus;

namespace {
int failures = 0;
void check(bool ok, int line) {
    if (!ok) {
        std::fprintf(stderr, "pitch_identification_protocol_tests line %d: contract failed\n", line);
        ++failures;
    }
}
void check_code(ResultCode actual, ResultCode expected, int line) {
    if (actual != expected) {
        std::fprintf(stderr, "line %d: expected code %d, got %d\n", line, static_cast<int>(expected), static_cast<int>(actual));
        ++failures;
    }
}
#define CHECK(x) check((x), __LINE__)
#define CHECK_CODE(act, exp) check_code((act), (exp), __LINE__)

PitchIdentificationPolicy default_policy() {
    return {-0.4f, 0.4f, 0.1f, 1.0f, 0.5f, 0.05f, 10000, 2000000, 10000, 6000};
}

PitchIdentificationRecipe default_recipe(std::uint32_t run_id = 1) {
    return {run_id, 0.04f, 10000, 10000, 2, 10000};
}

SentinelConfig make_test_config(bool available = false) {
    auto config = synthetic_config();
    config.independent_motor_bench = true;
    for (auto& motor : config.dji) { motor.installed = false; }
    config.serial.reset();
    config.timing.target_ttl = {100};
    config.dm[0].bus = 2;
    config.dm[0].mit_kp = 12.0f;
    config.dm[0].mit_kd = 0.4f;
    if (available) {
        config.pitch_identification = default_policy();
    }
    return config;
}

struct DecodedStatus {
    ResultCode result{ResultCode::rejected};
    std::uint8_t reason{};
    bool available{};
    PitchIdentificationState state{PitchIdentificationState::idle};
    PitchIdentificationFault fault{PitchIdentificationFault::none};
    std::uint32_t run_id{};
    float requested_nm{};
    float encoded_nm{};
};

DecodedStatus decode_status_payload(std::span<const std::uint8_t> payload) {
    DecodedStatus out{};
    if (payload.size() < 12) { return out; }
    out.result = static_cast<ResultCode>(payload[10]);
    out.reason = payload[11];
    if (payload.size() == 27) {
        out.available = payload[12] != 0;
        out.state = static_cast<PitchIdentificationState>(payload[13]);
        out.fault = static_cast<PitchIdentificationFault>(payload[14]);
        PayloadReader r{payload};
        for (int i = 0; i < 15; ++i) { static_cast<void>(r.u8()); }
        out.run_id = r.u32();
        out.requested_nm = r.f32();
        out.encoded_nm = r.f32();
    }
    return out;
}

struct TestRig {
    SentinelConfig config;
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    SentinelApp app;
    SentinelExecutor exec;
    SentinelPerception perception;
    ParamStore params;
    SentinelHostSession session;
    ec::sentinel::host::SimPlant plant{};
    AppInputs inputs{};
    Tick now{1};
    std::uint32_t command_seq{};

    explicit TestRig(bool available = false, Lifetime link_ttl = {})
        : config(make_test_config(available)),
          app(config, ConfigAudience::offline_test, StartupPolicy::maintenance_only),
          exec(config, ConfigAudience::offline_test, bus1, bus2),
          perception(config, ConfigAudience::offline_test),
          params(config),
          session(app, params, &exec, &perception, link_ttl) {
        inputs.link.locally_ready = true;
        inputs.link.remote_online = true;
        inputs.link.switch_left = drivers::Dr16Switch::mid;
        inputs.link.switch_right = drivers::Dr16Switch::up;
        inputs.link.remote_sampled_at = now;
        inputs.link.maintenance_link_alive = true;
        inputs.link.maintenance_request = true;
        inputs.link.maintenance_session_epoch = 1;
        plant.dm.position_rad = 0.1f;
        plant.dm_reports_enabled = false;
    }

    void prime_motor() {
        plant.dm.position_rad = 0.1f;
        plant.dm_reports_enabled = false;
        step(now.ms + 1);
        target();
        step(now.ms + 1);
        plant.dm_reports_enabled = true;
        step(now.ms + 1);
        step(now.ms + 1);
        CHECK(exec.pitch_enable_confirmed());
    }

    void target(float value = 0.1f) {
        TargetSnapshot packet;
        packet.domains[domain_index(Domain::pitch)] = DomainSetpoint{control::ControlMode::position, value};
        const auto res = app.accept_targets(packet, now, Source::maintenance);
        CHECK(res == AcceptResult::accepted);
    }

    void step(std::uint32_t ms) {
        now = {ms}; bus1.clear(); bus2.clear();
        const auto lstate = session.link_state(now);
        if (session.role_source() == Source::maintenance) {
            inputs.link.maintenance_link_alive = lstate.alive;
            inputs.link.maintenance_request = lstate.request;
            inputs.link.maintenance_session_epoch = lstate.epoch;
        } else if (session.role_source() == Source::host) {
            inputs.link.host_link_alive = lstate.alive;
            inputs.link.host_request = lstate.request;
            inputs.link.host_session_epoch = lstate.epoch;
        } else {
            inputs.link.maintenance_request = false;
            inputs.link.host_request = false;
        }
        plant.emit_feedback(config, [&](std::uint8_t bus, const CanFrame& frame, Tick at) {
            exec.route(bus, frame, at, MicroTick{at.ms * 1000u});
        }, now);
        inputs.feedback[domain_index(Domain::pitch)] = exec.joint_feedback(Domain::pitch, now);
        inputs.faults = exec.health(now);
        inputs.epochs = Epochs{app.config_epoch(), app.reference_epoch()};
        app.update(now, inputs);
        exec.step(now, 0.001f, app, MicroTick{ms * 1000u});
        plant.integrate(config, bus1, bus2, 0.001f);
    }

    std::uint32_t next_seq() { return ++command_seq; }

    std::vector<std::uint8_t> handle_raw(std::span<const std::uint8_t> request) {
        std::array<std::uint8_t, protocol_payload_max> response{};
        const auto size = session.handle(request, now, response);
        if (size == 0) { return {}; }
        return {response.data(), response.data() + size};
    }

    std::vector<std::uint8_t> hello() {
        std::array<std::uint8_t, 6> req{};
        PayloadWriter w{req};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::hello));
        w.u32(0x11223344);
        auto rep = handle_raw({req.data(), w.size()});
        step(now.ms + 1);
        return rep;
    }

    std::vector<std::uint8_t> authorize(bool grant, Source source, std::uint32_t seq) {
        std::array<std::uint8_t, 12> req{};
        PayloadWriter w{req};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::authorize));
        w.u32(session.session_id());
        w.u32(seq);
        w.u8(grant ? 1 : 0);
        w.u8(static_cast<std::uint8_t>(source));
        auto rep = handle_raw({req.data(), w.size()});
        step(now.ms + 1);
        return rep;
    }

    std::vector<std::uint8_t> make_pitch_req(std::uint32_t seq, std::uint8_t op,
        std::uint32_t c_epoch, std::uint32_t r_epoch,
        const PitchIdentificationRecipe* recipe = nullptr) {
        std::vector<std::uint8_t> req(op == 1 ? 43 : 19);
        PayloadWriter w{req};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::pitch_identification));
        w.u32(session.session_id());
        w.u32(seq);
        w.u8(op);
        w.u32(c_epoch);
        w.u32(r_epoch);
        if (op == 1 && recipe != nullptr) {
            w.u32(recipe->run_id);
            w.f32(recipe->amplitude_nm);
            w.u32(recipe->half_period_us);
            w.u32(recipe->baseline_us);
            w.u32(recipe->cycles);
            w.u32(recipe->final_hold_us);
        }
        return req;
    }

    std::vector<std::uint8_t> send_pitch(std::uint32_t seq, std::uint8_t op,
        std::uint32_t c_epoch, std::uint32_t r_epoch,
        const PitchIdentificationRecipe* recipe = nullptr) {
        const auto req = make_pitch_req(seq, op, c_epoch, r_epoch, recipe);
        return handle_raw(req);
    }
};

void test_available_disabled_default() {
    TestRig rig(false); // default fixture, available = false
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));
    CHECK(!rig.exec.pitch_identification_available());

    // Query: available is 0, result accepted
    const auto q_reply = rig.send_pitch(rig.next_seq(), 0, rig.app.config_epoch(), rig.app.reference_epoch());
    CHECK(q_reply.size() == 27);
    const auto q_status = decode_status_payload(q_reply);
    CHECK(q_status.result == ResultCode::accepted);
    CHECK(!q_status.available);
    CHECK(q_status.state == PitchIdentificationState::idle);

    // Start: rejected because unavailable
    const auto recipe = default_recipe(1);
    const auto s_reply = rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &recipe);
    CHECK(s_reply.size() == 27);
    const auto s_status = decode_status_payload(s_reply);
    CHECK_CODE(s_status.result, ResultCode::rejected);
    CHECK(!s_status.available);
}

void test_request_lengths_and_invalid_op() {
    TestRig rig(true);
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));

    // Truncated header (< 10 bytes)
    std::vector<std::uint8_t> short_hdr{protocol_version, static_cast<std::uint8_t>(MessageType::pitch_identification), 1, 2, 3};
    auto reply = rig.handle_raw(short_hdr);
    CHECK(reply.size() == 12);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::bad_length);

    // Truncated body (15 bytes, < 19)
    std::vector<std::uint8_t> short_body(15);
    PayloadWriter w{short_body};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::pitch_identification));
    w.u32(rig.session.session_id());
    w.u32(rig.next_seq());
    w.u8(0);
    reply = rig.handle_raw(short_body);
    CHECK(reply.size() == 12);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::bad_length);

    // Query with extra trailing bytes (20 bytes != 19)
    std::vector<std::uint8_t> long_query(20, 0);
    PayloadWriter wq{long_query};
    wq.u8(protocol_version);
    wq.u8(static_cast<std::uint8_t>(MessageType::pitch_identification));
    wq.u32(rig.session.session_id());
    wq.u32(rig.next_seq());
    wq.u8(0);
    wq.u32(rig.app.config_epoch());
    wq.u32(rig.app.reference_epoch());
    reply = rig.handle_raw(long_query);
    CHECK(reply.size() == 12);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::bad_length);

    // Start with 19 bytes (missing recipe != 43)
    auto short_start = rig.make_pitch_req(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch());
    short_start.resize(19);
    reply = rig.handle_raw(short_start);
    CHECK(reply.size() == 12);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::bad_length);

    // Start with 44 bytes (extra byte)
    const auto r = default_recipe(1);
    auto long_start = rig.make_pitch_req(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &r);
    long_start.push_back(0);
    reply = rig.handle_raw(long_start);
    CHECK(reply.size() == 12);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::bad_length);

    // Start with NaN amplitude
    auto nan_recipe = default_recipe(2);
    nan_recipe.amplitude_nm = std::numeric_limits<float>::quiet_NaN();
    reply = rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &nan_recipe);
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::invalid_value);

    // Invalid op (e.g. 4)
    auto bad_op = rig.make_pitch_req(rig.next_seq(), 4, rig.app.config_epoch(), rig.app.reference_epoch());
    reply = rig.handle_raw(bad_op);
    CHECK(reply.size() == 12);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::invalid_value);
}

void test_ownership_and_epochs() {
    TestRig rig(true);
    static_cast<void>(rig.hello());

    // Without authorization (no ownership):
    // Query succeeds
    auto reply = rig.send_pitch(rig.next_seq(), 0, 999, 999);
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::accepted);

    // Start rejected: not_permitted
    const auto r = default_recipe(1);
    reply = rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &r);
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::not_permitted);

    // Reset rejected: not_permitted
    reply = rig.send_pitch(rig.next_seq(), 3, rig.app.config_epoch(), rig.app.reference_epoch());
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::not_permitted);

    // Cancel rejected: not_permitted
    reply = rig.send_pitch(rig.next_seq(), 2, rig.app.config_epoch(), rig.app.reference_epoch());
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::not_permitted);

    // Authorize with ordinary Source::host:
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    reply = rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &r);
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::not_permitted);

    // Authorize with Source::maintenance:
    static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));
    // Epoch mismatch: config_epoch wrong
    reply = rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch() + 1, rig.app.reference_epoch(), &r);
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::epoch_mismatch);

    // Epoch mismatch: reference_epoch wrong
    reply = rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch() + 1, &r);
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::epoch_mismatch);

    // Reset epoch mismatch
    reply = rig.send_pitch(rig.next_seq(), 3, rig.app.config_epoch() + 1, rig.app.reference_epoch());
    CHECK(reply.size() == 27);
    CHECK(static_cast<ResultCode>(reply[10]) == ResultCode::epoch_mismatch);
}

void test_synthesised_start_success() {
    TestRig rig(true);
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));
    rig.prime_motor();

    // Start with matching epochs and motor primed
    const auto r = default_recipe(1);
    auto reply = rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &r);
    CHECK(reply.size() == 27);
    auto status = decode_status_payload(reply);
    CHECK(status.result == ResultCode::accepted);
    CHECK(status.available);
    CHECK(status.run_id == 1);
    CHECK(status.state == PitchIdentificationState::baseline);

    // Query reflects active experiment
    reply = rig.send_pitch(rig.next_seq(), 0, 0, 0);
    CHECK(reply.size() == 27);
    status = decode_status_payload(reply);
    CHECK(status.result == ResultCode::accepted);
    CHECK(status.state == PitchIdentificationState::baseline);

    // Cancel transitions state to fault/cancelled
    reply = rig.send_pitch(rig.next_seq(), 2, 0, 0);
    CHECK(reply.size() == 27);
    status = decode_status_payload(reply);
    CHECK(status.result == ResultCode::accepted);
    CHECK(status.state == PitchIdentificationState::fault);
    CHECK(status.fault == PitchIdentificationFault::cancelled);
}

void test_idempotence_and_intervening_traffic() {
    TestRig rig(true);
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));
    rig.prime_motor();

    // Step 1: Start with allocated sequence
    const auto start_seq = rig.next_seq();
    const auto r = default_recipe(10);
    const auto seq_start_req = rig.make_pitch_req(start_seq, 1, rig.app.config_epoch(), rig.app.reference_epoch(), &r);
    auto reply1 = rig.handle_raw(seq_start_req);
    CHECK(reply1.size() == 27);
    auto status1 = decode_status_payload(reply1);
    CHECK(status1.result == ResultCode::accepted);
    CHECK(status1.run_id == 10);

    // Step 2: Intervening targets with new sequence
    const auto target_seq = rig.next_seq();
    std::vector<std::uint8_t> target_req(29);
    PayloadWriter wt{target_req};
    wt.u8(protocol_version);
    wt.u8(static_cast<std::uint8_t>(MessageType::targets));
    wt.u32(rig.session.session_id());
    wt.u32(target_seq);
    wt.u32(rig.app.config_epoch());
    wt.u32(rig.app.reference_epoch());
    wt.u8(1); // count = 1
    wt.u8(static_cast<std::uint8_t>(Domain::pitch));
    wt.u8(static_cast<std::uint8_t>(control::ControlMode::position));
    wt.u8(0); // frame
    wt.f32(0.1f); // value
    wt.u16(0); // age_ms
    wt.u8(0); // twist_present = 0
    CHECK(!wt.overflow);
    auto target_reply = rig.handle_raw(target_req);
    CHECK(!target_reply.empty());
    CHECK(static_cast<ResultCode>(target_reply[10]) == ResultCode::accepted);

    // Step 3: Intervening query with new sequence
    const auto query_seq = rig.next_seq();
    auto q_reply = rig.send_pitch(query_seq, 0, 0, 0);
    CHECK(q_reply.size() == 27);

    // Step 4: Retransmit identical start
    auto retransmit_reply = rig.handle_raw(seq_start_req);
    CHECK(retransmit_reply.size() == 27);
    auto retransmit_status = decode_status_payload(retransmit_reply);
    CHECK(retransmit_status.result == ResultCode::accepted);
    CHECK(retransmit_status.run_id == 10);
    CHECK(rig.session.duplicate_requests() > 0);

    // Step 5: Seq collision: altered start with start_seq -> REPLAY reject
    auto altered_recipe = default_recipe(11);
    const auto collision_req = rig.make_pitch_req(start_seq, 1, rig.app.config_epoch(), rig.app.reference_epoch(), &altered_recipe);
    auto collision_reply = rig.handle_raw(collision_req);
    CHECK(collision_reply.size() == 12);
    CHECK(static_cast<ResultCode>(collision_reply[10]) == ResultCode::replay);
}

void test_session_replacement_and_expiry() {
    // 1. Replacing HELLO aborts active experiment
    {
        TestRig rig(true);
        static_cast<void>(rig.hello());
        static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));
        rig.prime_motor();
        const auto r = default_recipe(20);
        static_cast<void>(rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &r));
        CHECK(rig.exec.pitch_identification().active());

        // Replace HELLO
        static_cast<void>(rig.hello());
        CHECK(!rig.exec.pitch_identification().active());
        CHECK(rig.exec.pitch_identification().fault() == PitchIdentificationFault::cancelled);
    }

    // 2. Revoking authorization (grant = 0) aborts active experiment
    {
        TestRig rig(true);
        static_cast<void>(rig.hello());
        static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));
        rig.prime_motor();
        const auto r = default_recipe(21);
        static_cast<void>(rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &r));
        CHECK(rig.exec.pitch_identification().active());

        // Revoke authorization
        static_cast<void>(rig.authorize(false, Source::maintenance, rig.next_seq()));
        CHECK(!rig.exec.pitch_identification().active());
        CHECK(rig.exec.pitch_identification().fault() == PitchIdentificationFault::cancelled);
    }

    // 3. Expiry of controlling session aborts active experiment
    {
        TestRig rig(true, Lifetime{50}); // link_ttl = 50ms
        static_cast<void>(rig.hello());
        static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));
        rig.prime_motor();
        const auto r = default_recipe(22);
        static_cast<void>(rig.send_pitch(rig.next_seq(), 1, rig.app.config_epoch(), rig.app.reference_epoch(), &r));
        CHECK(rig.exec.pitch_identification().active());

        // Fast forward clock past link_ttl
        rig.now.ms += 60;
        static_cast<void>(rig.session.link_state(rig.now));
        CHECK(!rig.exec.pitch_identification().active());
        CHECK(rig.exec.pitch_identification().fault() == PitchIdentificationFault::cancelled);
    }

    // 4. Idle or complete experiment is not tripped by session replacement
    {
        TestRig rig(true);
        static_cast<void>(rig.hello());
        static_cast<void>(rig.authorize(true, Source::maintenance, rig.next_seq()));
        CHECK(rig.exec.pitch_identification().state() == PitchIdentificationState::idle);
        static_cast<void>(rig.hello());
        CHECK(rig.exec.pitch_identification().state() == PitchIdentificationState::idle);
        CHECK(rig.exec.pitch_identification().fault() == PitchIdentificationFault::none);
    }
}
} // namespace

int main() {
    test_available_disabled_default();
    test_request_lengths_and_invalid_op();
    test_ownership_and_epochs();
    test_synthesised_start_success();
    test_idempotence_and_intervening_traffic();
    test_session_replacement_and_expiry();

    if (failures != 0) {
        std::fprintf(stderr, "FAILED: %d contract assertion(s) failed\n", failures);
        return 1;
    }
    std::puts("All pitch identification protocol tests passed.");
    return 0;
}
