// Profile4 is an observer of the existing scheduler/control/transport paths.
#include "sentinel_execution.hpp"
#include "sentinel_param_apply.hpp"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace {
using namespace ec;
using namespace ec::sentinel::app;
namespace recording = ec::sentinel::recording;
void check(bool ok, const char* expr, int line) {
    if (!ok) { std::fprintf(stderr, "line %d: %s\n", line, expr); std::exit(3); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)
struct Bus final : transport::CanTx {
    std::array<transport::CanFrame, 256> frames{};
    std::size_t count{};
    transport::SendResult result{transport::SendResult::accepted};
    transport::SendResult try_send(const transport::CanFrame& f) noexcept override {
        CHECK(count < frames.size()); frames[count++] = f; return result;
    }
};
SentinelConfig relocated_config() {
    auto c = synthetic_config();
    c.runtime.gm = GroupTiming{2000, 0, {3000}, 6000};
    c.runtime.pitch = GroupTiming{2000, 0, {3000}, 6000};
    // Do not assume Fire happens to occupy reference package slot 5.
    const auto fire = [&] { for (std::size_t n = 0; n < c.dji.size(); ++n) {
        if (c.dji[n].domain == Domain::fire_yaw) { return n; }
    } return c.dji.size(); }();
    CHECK(fire < c.dji.size());
    std::swap(c.dji[0], c.dji[fire]);
    return c;
}
struct Rig {
    SentinelConfig config{relocated_config()};
    SentinelApp app{config, ConfigAudience::offline_test};
    Bus bus1{}, bus2{};
    SentinelExecutor exec{config, ConfigAudience::offline_test, bus1, bus2};
    void step(std::uint32_t ms, float dt = 0.001f) {
        AppInputs in{}; in.link.locally_ready = true;
        in.faults = exec.health(Tick{ms});
        app.update(Tick{ms}, in);
        exec.step(Tick{ms}, dt, app, MicroTick{ms * 1000u + 23u});
    }
};
float f32(std::uint32_t bits) { return std::bit_cast<float>(bits); }
void pair_contract(const AutotuneBundle& b, Domain d, const ExecutorGroupStats& stats) {
    CHECK(b.state.kind == recording::Kind::autotune_state);
    CHECK(b.aux.kind == recording::Kind::autotune_aux);
    CHECK(b.state.stream == domain_index(d));
    CHECK(b.state.stream == b.aux.stream);
    CHECK(b.state.source_sequence == stats.computations);
    CHECK(b.state.source_sequence == b.aux.source_sequence);
    CHECK(b.state.sampled_us == stats.last_computed.us);
    CHECK(b.state.sampled_us == b.aux.sampled_us);
    CHECK(b.state.run == b.aux.run);
    CHECK(b.state.sequence == 0 && b.aux.sequence == 0); // Assigned by recorder, not experiment ID.
    CHECK(b.state.error_us == 1000u); // Millisecond arrival fallback is not a measured microclock.
    CHECK(f32(b.state.data[6]) == stats.last_dt_s);
}
void profile4_computed_ticks_and_epoch() {
    Rig r;
    CHECK(r.exec.assembled());
    CHECK(r.exec.dji_slot_for(Domain::fire_yaw) == 0);
    CHECK(!r.exec.autotune_trace(Domain::pitch, Tick{0}, r.app));
    CHECK(!r.exec.autotune_trace(Domain::fire_yaw, Tick{0}, r.app));
    CHECK(!r.exec.autotune_trace(Domain::main_yaw, Tick{0}, r.app));
    std::uint32_t previous_pitch = 0, previous_fire = 0, pitch_bundles = 0, fire_bundles = 0;
    for (std::uint32_t ms = 1; ms <= 12; ++ms) {
        r.step(ms);
        const auto pitch = r.exec.autotune_trace(Domain::pitch, Tick{ms}, r.app);
        CHECK(pitch.has_value() == (r.exec.pitch_stats().computations != previous_pitch));
        if (pitch) {
            ++pitch_bundles;
            pair_contract(*pitch, Domain::pitch, r.exec.pitch_stats());
            CHECK(std::isnan(f32(pitch->state.data[5]))); // MIT is not measured normalized output.
            CHECK(!(pitch->state.data[0] & (1u << 4)));
            CHECK(pitch->state.data[7] == r.exec.param_epoch());
            if (ms == 1) {
                CHECK((pitch->aux.data[0] >> 16) == 1); // Exact attempted Pitch bytes.
                CHECK((pitch->aux.data[1] >> 8) == 3); // Disable, never MIT.
                const auto& bus = r.config.dm[0].bus == 1 ? r.bus1 : r.bus2;
                CHECK(bus.count > 0);
                const auto& frame = bus.frames[bus.count - 1];
                CHECK((pitch->aux.data[0] & 0xffffu) == frame.id);
                for (std::size_t n = 0; n < 8; ++n) {
                    CHECK(((pitch->aux.data[2 + n / 4] >> ((n % 4) * 8)) & 0xffu) == frame.data[n]);
                }
            } else {
                // During disable retry backoff the existing path sends a query, not the old disable.
                CHECK((pitch->aux.data[0] & 0xffffu) == 0x7ffu);
                CHECK((pitch->aux.data[1] >> 8) == 4);
            }
        }
        previous_pitch = r.exec.pitch_stats().computations;
        const auto fire = r.exec.autotune_trace(Domain::fire_yaw, Tick{ms}, r.app);
        if (fire) {
            ++fire_bundles;
            CHECK(fire->state.source_sequence > previous_fire);
            CHECK(fire->state.sampled_us == ms * 1000u + 23u);
            CHECK(fire->state.data[7] == r.exec.param_epoch());
            previous_fire = fire->state.source_sequence;
        }
        CHECK(r.exec.param_epoch() == 1);
        const auto guards = r.exec.experiment_guards(Domain::pitch, Tick{ms});
        CHECK(guards.param_epoch == r.exec.param_epoch());
        CHECK(!guards.guardian_approved);
    }
    CHECK(pitch_bundles == r.exec.pitch_stats().computations);
    CHECK(fire_bundles == previous_fire);
    CHECK(pitch_bundles > 0 && pitch_bundles < 12);
    CHECK(fire_bundles > 0 && fire_bundles < 12);
    auto* axis = r.exec.axis(Domain::fire_yaw);
    CHECK(axis);
    auto position = axis->position_pid();
    position.ki += 0.125f;
    CHECK(axis->set_position_gains(position));
    CHECK(r.exec.param_epoch() == 1); // Observation is at the step boundary, not a setter side effect.
    r.step(13); CHECK(r.exec.param_epoch() == 2);
    r.step(14); CHECK(r.exec.param_epoch() == 2);
    auto velocity = axis->active_velocity_pid(); velocity.kd += 0.0625f;
    CHECK(axis->set_velocity_gains(velocity));
    r.step(15); CHECK(r.exec.param_epoch() == 3);
    CHECK(r.exec.set_dm_mit_gains(r.exec.dm_mit_kp() + 1.0f, r.exec.dm_mit_kd()));
    r.step(16); CHECK(r.exec.param_epoch() == 4);
    CHECK(!r.exec.set_dm_mit_gains(-1.0f, 0.0f));
    r.step(17); CHECK(r.exec.param_epoch() == 4);
    // An invalid base dt forces actual Pitch computation even between scheduled ticks.
    r.step(18, 0.0f);
    const auto bad = r.exec.autotune_trace(Domain::pitch, Tick{18}, r.app);
    CHECK(bad && (bad->state.data[0] & (1u << 7)) && (bad->aux.data[4] & (1u << 17)));
    r.step(19);
    const auto next = r.exec.autotune_trace(Domain::pitch, Tick{19}, r.app);
    if (next) { CHECK(!(next->state.data[0] & (1u << 7))); }
}
std::uint32_t measured_us = 0;
MicroTick measured_clock() noexcept { return MicroTick{measured_us}; }
void measured_timing_and_transport_failure() {
    auto config = relocated_config();
    SentinelApp app{config, ConfigAudience::offline_test};
    Bus bus1, bus2;
    bus1.result = transport::SendResult::busy;
    bus2.result = transport::SendResult::busy;
    SentinelExecutor exec{config, ConfigAudience::offline_test, bus1, bus2, measured_clock};
    measured_us = 1027;
    exec.step(Tick{1}, 0.001f, app, MicroTick{1023});
    const auto b = exec.autotune_trace(Domain::pitch, Tick{1}, app);
    CHECK(b);
    CHECK(b->state.sampled_us == measured_us);
    CHECK(b->state.arrived_us == measured_us);
    CHECK(b->state.error_us == 0);
    CHECK((b->aux.data[4] & 0xffu) == static_cast<std::uint32_t>(drivers::DjiGroupStatus::busy));
    CHECK(exec.pitch_stats().submitted == 0);
    CHECK((b->aux.data[0] >> 16) == 1); // Attempted frame is retained even when transport refuses.
    CHECK(std::isnan(f32(b->state.data[5])));
}
void pitch_reported_torque_and_read_only_ranges() {
    Rig r;
    const auto limits = *r.config.dm[0].limits;
    const auto p = drivers::dm_encode_position(limits, r.config.joints[2].zero_offset_rad.value_or(0.0f) + 0.1f);
    const auto v = drivers::dm_encode_velocity(limits, 0.0f);
    const auto t = drivers::dm_encode_torque(limits, 0.8f);
    CHECK(p && v && t);
    transport::CanFrame frame{};
    frame.id = *r.config.dm[0].master_id; frame.size = 8;
    frame.data = {static_cast<std::uint8_t>(0x10u | *r.config.dm[0].can_id),
        static_cast<std::uint8_t>(*p >> 8), static_cast<std::uint8_t>(*p),
        static_cast<std::uint8_t>(*v >> 4),
        static_cast<std::uint8_t>(((*v & 15u) << 4) | (*t >> 8)),
        static_cast<std::uint8_t>(*t), 32, 33};
    r.exec.route(r.config.dm[0].bus, frame, {1}, MicroTick{900});
    r.step(1);
    auto b = r.exec.autotune_trace(Domain::pitch, {1}, r.app);
    CHECK(b && (b->aux.data[4] & (1u << 16)));
    CHECK(std::fabs(f32(b->aux.data[7]) - *drivers::dm_decode_torque(limits, *t)) < 1e-6f);
    CHECK(b->aux.arrived_us == 900 && b->aux.sampled_us == 1023);
    CHECK(std::isnan(f32(b->state.data[5]))); // 上报力矩不能伪装成归一化输出。
    r.step(200);
    b = r.exec.autotune_trace(Domain::pitch, {200}, r.app);
    CHECK(b && std::isnan(f32(b->aux.data[7]))); // 旧反馈不会被控制周期续期。
    Rig future;
    future.exec.route(future.config.dm[0].bus, frame, {1}, MicroTick{2000});
    future.step(1);
    b = future.exec.autotune_trace(Domain::pitch, {1}, future.app);
    CHECK(b && std::isnan(f32(b->aux.data[7]))); // 不能把未来反馈配给过去的计算。
    ParamStore params{r.config};
    const std::array expected{limits.position_max_rad, limits.velocity_max_rad_s, limits.torque_max_nm};
    for (std::size_t n = 0; n < expected.size(); ++n) {
        const auto id = param_id(dji_slot_count, static_cast<std::uint16_t>(4 + n));
        CHECK(params.entry(id) && params.entry(id)->value == expected[n]);
        CHECK(!param_has_apply_path(r.app, id));
        CHECK(apply_param(params, r.app, &r.exec, nullptr, id, expected[n] + 1.0f, true) == ParamWriteResult::invalid_value);
        CHECK(apply_param(params, r.app, &r.exec, nullptr, id, expected[n], true) == ParamWriteResult::unsupported);
        CHECK(params.entry(id)->value == expected[n]);
    }
}
void recorder_admission_is_not_output_evidence() {
    Rig r;
    recording::Recorder rec;
    CHECK(!r.exec.experiment_guards(Domain::fire_yaw, {1}).backpressure_ok);
    r.exec.bind_pitch_recorder(&rec);
    std::array<std::uint8_t, 12> request{1, 1, 4, 5};
    recording::put32(request, 4, 42);
    recording::put32(request, 8, 100000);
    std::array<std::uint8_t, 224> response{};
    CHECK(rec.request(request, response, {1000}) == 32 && response[2] == 0);
    CHECK(r.exec.experiment_guards(Domain::fire_yaw, {1}).backpressure_ok);
    CHECK(!r.exec.experiment_guards(Domain::pitch, {1}).backpressure_ok);
    CHECK(!r.exec.experiment_guards(static_cast<Domain>(255), {1}).backpressure_ok);
    r.bus1.result = r.bus2.result = transport::SendResult::busy;
    r.step(1);
    CHECK(!r.exec.experiment_guards(Domain::fire_yaw, {1}).stop_policy_executed);
    CHECK(!r.exec.experiment_guards(Domain::fire_yaw, {101}).backpressure_ok);
    for (std::uint32_t i = 0; i < 256; ++i) {
        recording::Record state{};
        state.kind = recording::Kind::autotune_state;
        state.stream = 5; state.source_sequence = i + 1; state.sampled_us = 1001 + i;
        auto aux = state; aux.kind = recording::Kind::autotune_aux;
        rec.push_bundle(state, aux, {1001});
    }
    CHECK(rec.size() == recording::capacity);
    CHECK(!r.exec.experiment_guards(Domain::fire_yaw, {2}).backpressure_ok);
}
void same_bus(const Bus& a, const Bus& b) {
    CHECK(a.count == b.count);
    for (std::size_t n = 0; n < a.count; ++n) {
        CHECK(a.frames[n].id == b.frames[n].id);
        CHECK(a.frames[n].size == b.frames[n].size);
        CHECK(a.frames[n].data == b.frames[n].data);
    }
}
void observation_does_not_modify_control() {
    Rig observed, baseline;
    for (std::uint32_t ms = 1; ms <= 20; ++ms) {
        observed.step(ms); baseline.step(ms);
        const auto before = observed.exec.stats();
        const auto epoch = observed.exec.param_epoch();
        for (int repeat = 0; repeat < 3; ++repeat) {
            for (auto d : {Domain::fire_yaw, Domain::pitch}) {
                (void)observed.exec.autotune_trace(d, Tick{ms}, observed.app);
                (void)observed.exec.experiment_guards(d, Tick{ms});
            }
        }
        CHECK(observed.exec.stats().cycles == before.cycles);
        CHECK(observed.exec.stats().dm_disable_sent == before.dm_disable_sent);
        CHECK(observed.exec.param_epoch() == epoch);
        same_bus(observed.bus1, baseline.bus1); same_bus(observed.bus2, baseline.bus2);
        for (auto d : {Domain::fire_yaw, Domain::main_yaw}) {
            CHECK(observed.exec.axis(d)->enabled() == baseline.exec.axis(d)->enabled());
            CHECK(observed.exec.axis(d)->mode() == baseline.exec.axis(d)->mode());
            CHECK(observed.exec.axis(d)->position_pid().kp == baseline.exec.axis(d)->position_pid().kp);
        }
    }
}
} // namespace
int main() {
    profile4_computed_ticks_and_epoch();
    observation_does_not_modify_control();
    measured_timing_and_transport_failure();
    pitch_reported_torque_and_read_only_ranges();
    recorder_admission_is_not_output_evidence();
    std::puts("sentinel autotune observation tests passed");
}
