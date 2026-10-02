#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ec::transport {
// 有界字节流分帧, 不依赖 UART IDLE. 不提供鉴权、可靠交付或命令有效期.
inline constexpr std::size_t uart_payload_max = 256;
inline constexpr std::size_t uart_wire_max = 2 * (6 + uart_payload_max + 2) + 2;
struct UartFrame {
    std::uint8_t type{};
    std::uint16_t sequence{}, size{};
    std::array<std::uint8_t, uart_payload_max> payload{};
};
inline std::uint16_t uart_crc16(std::span<const std::uint8_t> bytes) noexcept {
    std::uint16_t crc = 0xffff;
    for (auto b : bytes) {
        crc ^= static_cast<std::uint16_t>(b) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = static_cast<std::uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
        }
    }
    return crc;
}
// 输出容量须容纳该载荷的最坏转义长度. 返回 0 表示未编码.
inline std::size_t encode_uart_frame(std::uint8_t type, std::uint16_t sequence,
    std::span<const std::uint8_t> payload, std::span<std::uint8_t> output) noexcept {
    if (payload.size() > uart_payload_max || output.size() < 2 * (payload.size() + 8) + 2) { return 0; }
    std::array<std::uint8_t, uart_payload_max + 8> raw{};
    raw[0] = 1; raw[1] = type;
    raw[2] = static_cast<std::uint8_t>(sequence); raw[3] = static_cast<std::uint8_t>(sequence >> 8);
    raw[4] = static_cast<std::uint8_t>(payload.size()); raw[5] = static_cast<std::uint8_t>(payload.size() >> 8);
    for (std::size_t i = 0; i < payload.size(); ++i) { raw[6 + i] = payload[i]; }
    const auto crc = uart_crc16({raw.data(), 6 + payload.size()});
    raw[6 + payload.size()] = static_cast<std::uint8_t>(crc);
    raw[7 + payload.size()] = static_cast<std::uint8_t>(crc >> 8);
    std::size_t n = 0;
    output[n++] = 0x7e;
    for (std::size_t i = 0; i < payload.size() + 8; ++i) {
        if (raw[i] == 0x7e || raw[i] == 0x7d) {
            output[n++] = 0x7d; output[n++] = raw[i] ^ 0x20;
        } else { output[n++] = raw[i]; }
    }
    output[n++] = 0x7e;
    return n;
}

// 单一消费者拥有; 遇到丢字节/传输重启须 reset, 统计保留.
class UartFrameDecoder {
public:
    void reset() noexcept { used_ = 0; started_ = escaped_ = oversized_ = false; }
    [[nodiscard]] std::uint32_t rejected() const noexcept { return rejected_; }
    [[nodiscard]] std::optional<UartFrame> push(std::uint8_t b) noexcept {
        if (b == 0x7e) {
            std::optional<UartFrame> frame;
            if (started_ && (used_ || oversized_ || escaped_)) {
                if (!oversized_ && !escaped_ && used_ >= 8 && raw_[0] == 1) {
                    const auto size = static_cast<std::uint16_t>(raw_[4] | (raw_[5] << 8));
                    const auto crc = static_cast<std::uint16_t>(raw_[used_-2] | (raw_[used_-1] << 8));
                    if (size <= uart_payload_max && used_ == size + 8u &&
                        uart_crc16({raw_.data(), used_-2}) == crc) {
                        frame.emplace();
                        frame->type = raw_[1];
                        frame->sequence = static_cast<std::uint16_t>(raw_[2] | (raw_[3] << 8));
                        frame->size = size;
                        for (std::size_t i = 0; i < size; ++i) { frame->payload[i] = raw_[6+i]; }
                    }
                }
                if (!frame) { ++rejected_; }
            }
            used_ = 0; started_ = true; escaped_ = oversized_ = false;
            return frame;
        }
        if (!started_ || oversized_) { return std::nullopt; }
        if (escaped_) { b ^= 0x20; escaped_ = false; }
        else if (b == 0x7d) { escaped_ = true; return std::nullopt; }
        if (used_ == raw_.size()) { oversized_ = true; }
        else { raw_[used_++] = b; }
        return std::nullopt;
    }
private:
    std::array<std::uint8_t, uart_payload_max + 8> raw_{};
    std::size_t used_{};
    bool started_{}, escaped_{}, oversized_{};
    std::uint32_t rejected_{};
};
} // 命名空间 ec::transport
