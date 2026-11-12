// Synthetic, offline recorder contracts adapted from integration autotune trace tests.
// NOT_HARDWARE_VALIDATED: payloads below are explicit fixtures, not actuator evidence.
#include "raw_record.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace raw = ec::sentinel::recording;
namespace {
void check(bool ok, const char* expression, int line) {
    if (!ok) { std::fprintf(stderr, "line %d: %s\n", line, expression); std::exit(1); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)
using Request = std::array<std::uint8_t, 12>;
using Reply = std::array<std::uint8_t, raw::stream_packet_bytes>;
Request request(std::uint8_t op, std::uint8_t profile, std::uint8_t domain,
    std::uint32_t run = 42, std::uint32_t arg = 100000) {
    Request req{1, op, profile, domain}; raw::put32(req, 4, run); raw::put32(req, 8, arg); return req;
}
raw::Record sample(raw::Kind kind, std::uint8_t axis = 5, std::uint32_t seq = 1,
    std::uint32_t at = 1000) {
    return {kind, axis, 1, 999, seq, at, at + 7, 3, 4, 999};
}
void bundle(raw::Recorder& rec, std::uint32_t seq, std::uint8_t axis = 5) {
    auto s = sample(raw::Kind::autotune_state, axis, seq, seq);
    auto a = s; a.kind = raw::Kind::autotune_aux;
    rec.push_bundle(s, a, {1});
}
raw::Record decoded(const Reply& reply, std::size_t at) {
    auto value = raw::decode(std::span<const std::uint8_t>(reply).subspan(at, raw::record_bytes));
    CHECK(value); return *value;
}
void admission() {
    for (bool continuous : {false, true}) {
        for (std::uint8_t axis = 0; axis < 11; ++axis) {
            raw::Recorder rec; Reply out{};
            auto req = request(1, 4, axis);
            const auto n = continuous ? rec.stream_request(req, out, {0}) : rec.request(req, out, {0});
            CHECK(n == raw::reply_header);
            CHECK(out[2] == ((axis == 5 || axis == 6) ? 0 : 1));
            CHECK(rec.autotune_axis() == ((axis == 5 || axis == 6) ? axis : -1));
            if (out[2]) { continue; }
            CHECK(rec.control_domain() == -1 && !rec.fast_dm());
            CHECK((continuous ? rec.stream_request(req, out, {1}) : rec.request(req, out, {1})) == 32);
            CHECK(out[2] == 0); // Exact retry is idempotent.
            req[3] = axis == 5 ? 6 : 5;
            CHECK((continuous ? rec.stream_request(req, out, {1}) : rec.request(req, out, {1})) == 32);
            CHECK(out[2] == 2); // Same run cannot change selected axis.
            req = request(1, 4, axis, 43);
            CHECK((continuous ? rec.stream_request(req, out, {1}) : rec.request(req, out, {1})) == 32);
            CHECK(out[2] == 3); // Active run cannot be replaced.
        }
        for (auto arg : {0u, continuous ? 1800000001u : 2000001u}) {
            raw::Recorder rec; Reply out{}; auto req = request(1, 4, 5, 42, arg);
            CHECK((continuous ? rec.stream_request(req, out, {0}) : rec.request(req, out, {0})) == 32);
            CHECK(out[2] == 1 && rec.state() == raw::State::idle);
        }
    }
}
void legacy_admission_and_deadline() {
    for (bool continuous : {false, true}) {
        for (std::uint8_t profile = 0; profile <= 5; ++profile) {
            for (std::uint8_t domain = 0; domain < 11; ++domain) {
                raw::Recorder rec; Reply out{};
                const bool admitted = profile == 4 ? (domain == 5 || domain == 6)
                    : profile == 3 ? (continuous ? domain == 6 : domain <= 9)
                    : (profile == 1 || profile == 2) && domain == 0;
                auto req = request(1, profile, domain);
                CHECK((continuous ? rec.stream_request(req, out, {0}) : rec.request(req, out, {0})) == 32);
                CHECK(out[2] == (admitted ? 0 : 1));
            }
        }
        raw::Recorder rec; Reply out{};
        const ec::MicroTick start{0xfffffff0u};
        auto req = request(1, 4, 5, 42, 32);
        CHECK((continuous ? rec.stream_request(req, out, start) : rec.request(req, out, start)) == 32);
        auto s = sample(raw::Kind::autotune_state); auto a = s; a.kind = raw::Kind::autotune_aux;
        rec.push_bundle(s, a, {0x0fu}); CHECK(rec.size() == 2);
        ++s.source_sequence; ++a.source_sequence;
        rec.push_bundle(s, a, {0x10u});
        CHECK(rec.size() == 2 && rec.state() == raw::State::frozen && rec.autotune_axis() == -1);
    }
}
void pair_contracts() {
    raw::Recorder rec; Reply out{};
    CHECK(rec.request(request(1, 4, 5), out, {0}) == 32 && out[2] == 0);
    for (unsigned kind = 0; kind <= 9; ++kind) {
        rec.push(sample(static_cast<raw::Kind>(kind)), {1});
    }
    CHECK(rec.size() == 0); // No legacy, unknown or standalone bundle records.
    auto s = sample(raw::Kind::autotune_state); auto a = s; a.kind = raw::Kind::autotune_aux;
    for (unsigned mismatch = 0; mismatch < 5; ++mismatch) {
        auto bad = a;
        if (mismatch == 0) { ++bad.stream; }
        if (mismatch == 1) { ++bad.source_sequence; }
        if (mismatch == 2) { ++bad.sampled_us; }
        if (mismatch == 3) { bad.kind = raw::Kind::control; }
        if (mismatch == 4) { bad.kind = raw::Kind::autotune_state; }
        rec.push_bundle(s, bad, {1}); CHECK(rec.size() == 0);
    }
    auto wrong_axis = s; wrong_axis.stream = 6;
    auto wrong_aux = a; wrong_aux.stream = 6;
    rec.push_bundle(wrong_axis, wrong_aux, {1}); CHECK(rec.size() == 0);
    // Preserve every data word, including unavailable output NaNs and opaque aux evidence.
    s.data = {0x00000101, 0x3f800000, 0x40000000, 0x40400000, 0x40800000,
        0x7fc12345, 0x3a83126f, 0x12345678};
    a.data = {0x87654321, 2, 3, 4, 5, 6, 0x7fc54321, 0xffffffff};
    rec.push_bundle(s, a, {1}); CHECK(rec.size() == 2);
    rec.push_bundle(s, a, {2}); CHECK(rec.size() == 2); // Duplicate source sequence.
    ++s.source_sequence; ++a.source_sequence;
    rec.push_bundle(s, a, {3}); CHECK(rec.size() == 4); // No sampled-time throttling.
    CHECK(rec.request(request(2, 4, 5), out, {4}) == 32 && rec.autotune_axis() == -1);
    CHECK(rec.request(request(3, 4, 5, 42, 0), out, {5}) == 224);
    const auto first = decoded(out, 32), second = decoded(out, 96), third = decoded(out, 160);
    CHECK(first.kind == raw::Kind::autotune_state && second.kind == raw::Kind::autotune_aux);
    CHECK(first.sequence == 0 && second.sequence == 1 && third.sequence == 2);
    CHECK(first.run == 42 && second.run == 42 && first.source_sequence == second.source_sequence);
    CHECK(first.stream == second.stream && first.sampled_us == second.sampled_us);
    CHECK(first.arrived_us == 1007 && first.lost == 3 && first.error_us == 4);
    CHECK(first.data == s.data && second.data == a.data);
    CHECK(std::isnan(std::bit_cast<float>(first.data[5])));
    CHECK(std::isnan(std::bit_cast<float>(second.data[6])));
    rec.push_bundle(s, a, {6}); CHECK(rec.size() == 4); // Frozen rejects further writes.
}
void capacity_and_streaming() {
    raw::Recorder short_rec; Reply out{};
    CHECK(short_rec.request(request(1, 4, 6), out, {0}) == 32);
    for (std::uint32_t i = 0; i < 300; ++i) { bundle(short_rec, i, 6); }
    CHECK(short_rec.size() == raw::capacity && short_rec.state() == raw::State::full);
    CHECK(short_rec.request(request(3, 4, 6, 42, 510), out, {2}) == 160);
    CHECK(decoded(out, 32).kind == raw::Kind::autotune_state);
    CHECK(decoded(out, 96).kind == raw::Kind::autotune_aux);
    raw::Recorder rec;
    CHECK(rec.stream_request(request(1, 4, 5), out, {0}) == 32 && out[2] == 0);
    for (std::uint32_t i = 0; i < 256; ++i) { bundle(rec, i); }
    CHECK(rec.size() == raw::capacity && rec.state() == raw::State::recording);
    bundle(rec, 256); CHECK(rec.size() == raw::capacity);
    CHECK(rec.stream_request(request(0, 0, 0), out, {1}) == 32);
    CHECK(raw::get32(out, 12) == 2 && raw::get32(out, 8) == 512);
    rec.stream_consume(1); // Simulate one transmitted record: only one free slot.
    bundle(rec, 256); CHECK(rec.size() == 511); // Neither half admitted.
    CHECK(rec.stream_request(request(0, 0, 0), out, {1}) == 32);
    CHECK(raw::get32(out, 12) == 4 && raw::get32(out, 8) == 512);
    rec.stream_consume(1); bundle(rec, 256); CHECK(rec.size() == 512); // Retry not poisoned by failed capacity admission.
    std::size_t count{};
    CHECK(rec.stream_packet(out, {1}, count) == 224 && count == 3);
    CHECK(decoded(out, 32).sequence == 2 && decoded(out, 96).sequence == 3);
    CHECK(decoded(out, 160).sequence == 4); // Three complete 64-byte records, no truncation.
    CHECK(rec.size() == 512); // Packet peek does not consume.
    std::array<std::uint8_t, 95> tiny{};
    CHECK(rec.stream_packet(tiny, {1}, count) == 32 && count == 0);
    CHECK(rec.request(request(1, 4, 5, 43), out, {1}) == 0); // Cannot reset active stream via short-window API.
    CHECK(rec.stream_request(request(2, 4, 5), out, {2}) == 32);
    rec.stream_consume(510);
    CHECK(rec.stream_packet(out, {2}, count) == 160 && count == 2);
    CHECK(decoded(out, 32).sequence == 512 && decoded(out, 96).sequence == 513);
    rec.stream_consume(count);
    CHECK(rec.stream_packet(out, {2}, count) == 0); // Drained stop terminal notification remains legacy behavior.
}
void abi_and_legacy() {
    static_assert(raw::record_bytes == 64 && raw::reply_header == 32 && raw::capacity == 512);
    static_assert(raw::layout_id == 0x52415701 && raw::version == 1);
    for (unsigned kind = 1; kind <= 7; ++kind) {
        auto r = sample(static_cast<raw::Kind>(kind));
        r.data[7] = 0x87654321;
        std::array<std::uint8_t, 64> bytes{}; CHECK(raw::encode(r, bytes));
        CHECK(bytes[0] == 1 && bytes[1] == kind && bytes[2] == 5 && bytes[3] == 1);
        CHECK(raw::get32(bytes, 4) == 999 && raw::get32(bytes, 8) == 1);
        CHECK(raw::get32(bytes, 28) == 999 && raw::get32(bytes, 60) == 0x87654321);
        const auto back = raw::decode(bytes); CHECK(back && back->data == r.data);
        bytes[1] = 8; CHECK(!raw::decode(bytes)); bytes[1] = 0; CHECK(!raw::decode(bytes));
        bytes[1] = 1; bytes[3] = 8; CHECK(!raw::decode(bytes));
    }
    for (std::uint8_t profile = 1; profile <= 3; ++profile) {
        raw::Recorder rec; Reply out{};
        const auto domain = static_cast<std::uint8_t>(profile == 3 ? 6 : 0);
        CHECK(rec.request(request(1, profile, domain), out, {0}) == 32 && out[2] == 0);
        bundle(rec, 1); CHECK(rec.size() == 0);
        // Malformed bundle must never fall back to a valid legacy single-record write.
        rec.push_bundle(sample(raw::Kind::gyro, 0), sample(raw::Kind::control), {1});
        CHECK(rec.size() == 0);
        rec.push(sample(raw::Kind::gyro, 0, 1, 1000), {1}); CHECK(rec.size() == 1);
        rec.push(sample(raw::Kind::gyro, 0, 2, 2000), {2});
        CHECK(rec.size() == (profile == 2 ? 1u : 2u));
    }
    raw::Recorder pitch; Reply out{};
    CHECK(pitch.stream_request(request(1, 3, 6), out, {0}) == 32 && out[2] == 0);
    pitch.push(sample(raw::Kind::gyro, 0), {1}); CHECK(pitch.size() == 0);
    auto ctrl = sample(raw::Kind::control, 6, 1, 1000); ctrl.data[7] = 0xaabbccdd;
    pitch.push(ctrl, {1}); ++ctrl.source_sequence; ctrl.sampled_us = 5999;
    pitch.push(ctrl, {2}); CHECK(pitch.size() == 1);
    ++ctrl.source_sequence; ctrl.sampled_us = 6000; pitch.push(ctrl, {3}); CHECK(pitch.size() == 2);
    ec::transport::CanFrame frame{}; frame.id = 0x011; frame.size = 8;
    pitch.can(1, frame, {1000}, {3}, 0); CHECK(pitch.size() == 2); // Wrong physical bus.
    pitch.can(2, frame, {1000}, {3}, 0); CHECK(pitch.size() == 3);
    std::size_t count{};
    CHECK(pitch.stream_packet(out, {3}, count) == 32 + 64 + 64 + 44 && count == 3);
    CHECK(decoded(out, 32).data[7] == 0xaabbccdd);
    CHECK(decoded(out, 96).data[7] == 0xaabbccdd);
    CHECK(out[160 + 1] == 4 && out[160 + 2] == 9); // Existing canonical CAN2/0x011 maps stream9.
    CHECK(pitch.pitch_stream_healthy(42, {3}) && pitch.pitch_stream_ready(42, 99997, {3}));
}
} // namespace
void readiness() {
    raw::Recorder rec; Reply out{};
    CHECK(!rec.autotune_ready(5,{0}) && rec.autotune_run()==0);
    CHECK(rec.request(request(1,4,5),out,{100})==32 && out[2]==0);
    CHECK(rec.autotune_ready(5,{100}) && rec.autotune_run()==42);
    CHECK(!rec.autotune_ready(6,{100}));
    CHECK(!rec.autotune_ready(5,{100100}));
    CHECK(!rec.autotune_ready(5,{99}));
    for (unsigned i=0;i<256;++i) {
        auto state=sample(raw::Kind::autotune_state,5,i+1,101+i);
        auto aux=state; aux.kind=raw::Kind::autotune_aux;
        rec.push_bundle(state,aux,{101});
    }
    CHECK(!rec.autotune_ready(5,{101}));
    raw::Recorder stream;
    CHECK(stream.stream_request(request(1,4,5),out,{0})==32 && out[2]==0);
    CHECK(!stream.autotune_ready(5,{0})); // No unproven live streaming admission.
}
int main() {
    readiness(); admission(); legacy_admission_and_deadline(); pair_contracts(); capacity_and_streaming(); abi_and_legacy();
    std::puts("offline recording contracts passed (NOT_HARDWARE_VALIDATED)");
}
