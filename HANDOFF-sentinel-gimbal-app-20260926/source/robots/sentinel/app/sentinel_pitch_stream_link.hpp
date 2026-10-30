#pragma once
// Same bounded stream dispatch for the APP board and native test backend.
// The caller owns this object and Recorder on ONE control thread. No UART waits.
#include "raw_record.hpp"
#include "ec/transport/uart_frame.hpp"
#include <algorithm>
#include <array>
#include <span>

namespace ec::sentinel::app {
class PitchStreamLink {
public:
    static constexpr std::uint8_t request_type = 0x32;
    static constexpr std::uint8_t response_type = 0xb2;
    static constexpr std::uint8_t data_type = 0xb3;
    explicit PitchStreamLink(recording::Recorder& recorder) noexcept : recorder_(&recorder) {}

    [[nodiscard]] std::size_t request(const transport::UartFrame& frame,
        std::span<std::uint8_t> output, MicroTick now, bool maintenance_holds) noexcept {
        if (frame.type != request_type || frame.size != 12) { return 0; }
        const auto input = std::span<const std::uint8_t>{frame.payload.data(), frame.size};
        if (input[1] == 1 && (!maintenance_holds || input[2] != 3 || input[3] != 6)) {
            // Read-only collection is not an authority grant. In the APP, only
            // an already-held maintenance session can start Pitch collection.
            std::array<std::uint8_t, 12> invalid{};
            std::copy(input.begin(), input.end(), invalid.begin());
            invalid[2] = 0;
            return recorder_->stream_request(invalid, output, now);
        }
        return recorder_->stream_request(input, output, now);
    }

    template<class Send>
    bool service(MicroTick now, Send&& send) noexcept {
        std::array<std::uint8_t, recording::stream_packet_bytes> payload{};
        std::array<std::uint8_t, transport::uart_wire_max> wire{};
        std::size_t count{};
        const auto size = recorder_->stream_packet(payload, now, count);
        if (!size) { return false; }
        const auto encoded = transport::encode_uart_frame(data_type, sequence_,
            {payload.data(), size}, wire);
        if (!encoded || !send(std::span<const std::uint8_t>{wire.data(), encoded})) {
            return false; // Retry the SAME records; failed submission never consumes.
        }
        recorder_->stream_consume(count);
        ++sequence_;
        return true;
    }
private:
    recording::Recorder* recorder_;
    std::uint16_t sequence_{};
};
} // namespace ec::sentinel::app
