// Fixed-memory APP raw stream dispatch, shared by board and native backend.
#include "sentinel_pitch_stream_link.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>

namespace {
using namespace ec;
using namespace ec::sentinel;
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "pitch stream link line %d\n", __LINE__); std::exit(2); } } while (false)
transport::UartFrame request(std::uint8_t op, std::uint32_t run = 17) {
    transport::UartFrame frame{};
    frame.type = 0x32; frame.sequence = 9; frame.size = 12;
    frame.payload[0] = 1; frame.payload[1] = op;
    if (op == 1) { frame.payload[2] = 3; frame.payload[3] = 6; }
    recording::put32(frame.payload, 4, run);
    recording::put32(frame.payload, 8, 3000000);
    return frame;
}
void dispatch_and_retry() {
    recording::Recorder recorder;
    app::PitchStreamLink link{recorder};
    std::array<std::uint8_t, 224> response{};
    auto start = request(1);
    CHECK(link.request(start, response, {1000}, false) == 32 && response[2] == 1);
    CHECK(!recorder.pitch_stream_healthy(17, {1000}));
    CHECK(link.request(start, response, {1000}, true) == 32 && response[2] == 0);
    CHECK(recorder.pitch_stream_ready(17, 2999999, {1001}));
    const auto push = [&](unsigned time, unsigned sequence) {
        recording::Record record{recording::Kind::control, 6, 1};
        record.source_sequence = sequence; record.sampled_us = time; record.arrived_us = time;
        record.data[7] = 0x12345678;
        recorder.push(record, {time});
    };
    push(1000, 1); push(6000, 2);
    std::array<std::uint8_t, transport::uart_wire_max> first{};
    std::size_t first_size{};
    CHECK(!link.service({6000}, [&](std::span<const std::uint8_t> wire) {
        first_size = wire.size();
        std::copy(wire.begin(), wire.end(), first.begin());
        return false;
    }));
    CHECK(recorder.size() == 2); // Submission failure does not eat queue entries.
    CHECK(link.service({6000}, [&](std::span<const std::uint8_t> wire) {
        CHECK(wire.size() == first_size && std::equal(wire.begin(), wire.end(), first.begin()));
        transport::UartFrameDecoder decoder;
        std::optional<transport::UartFrame> decoded;
        for (auto byte : wire) { if (auto frame = decoder.push(byte)) { decoded = frame; } }
        CHECK(decoded && decoded->type == 0xb3 && decoded->size == 160);
        CHECK(recording::get32(decoded->payload, 32 + 60) == 0x12345678);
        CHECK(decoded->sequence == 0);
        return true;
    }));
    CHECK(recorder.size() == 0);
    CHECK(link.request(start, response, {7000}, true) == 32 && response[2] == 0);
    CHECK(recording::get32(response, 20) == 1000); // Retransmit must not rebase time.
    auto stop = request(2);
    CHECK(link.request(stop, response, {8000}, false) == 32 && response[2] == 0);
    CHECK(!recorder.pitch_stream_healthy(17, {8000}));
    CHECK(link.service({8000}, [](std::span<const std::uint8_t> wire) {
        transport::UartFrameDecoder decoder;
        std::optional<transport::UartFrame> decoded;
        for (auto byte : wire) { if (auto frame = decoder.push(byte)) { decoded = frame; } }
        CHECK(decoded && decoded->size == 32 && decoded->payload[3] == 3);
        return true;
    }));
    CHECK(!link.service({9000}, [](auto) { CHECK(false); return false; }));
    start.payload[2] = 2; start.payload[3] = 0;
    CHECK(link.request(start, response, {10000}, true) == 32 && response[2] == 1);
    start.size = 11;
    CHECK(link.request(start, response, {10000}, true) == 0);
}
}
int main() { dispatch_and_retry(); std::puts("pitch stream link contracts passed"); }
