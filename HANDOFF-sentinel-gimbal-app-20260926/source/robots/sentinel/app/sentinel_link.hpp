#pragma once
// 固定四槽 UART 应答出口: 变更结果、目标 ACK、快状态、低频页.
// 已在 DMA 中的帧不能抢占; 只替换尚未发送的同类应答, 始终保留最新帧与原请求身份.
#include "sentinel_protocol.hpp"
#include "ec/transport/uart_frame.hpp"
#include <array>

namespace ec::sentinel::app {
class ReplyMux {
public:
    // 管道后端可显式用 0 (无物理带宽限制); 串口低频页默认不超过 20 Hz.
    explicit ReplyMux(std::uint32_t slow_period_ms = 50) noexcept : slow_period_ms_(slow_period_ms) {}
    bool enqueue(const transport::UartFrame& request, std::span<const std::uint8_t> payload) noexcept {
        const auto lane = classify(request);
        auto& slot = slots_[lane];
        if (slot.size) { ++replaced_[lane]; }
        slot.size = transport::encode_uart_frame(declared_response_type(payload), request.sequence, payload, slot.wire);
        return slot.size != 0;
    }
    template<class Send> bool service(Tick now, Send&& send) noexcept {
        for (std::size_t i=0; i<slots_.size(); ++i) {
            auto& slot = slots_[i];
            if (!slot.size || (i == 3 && slow_sent_ && now.ms-last_slow_.ms < slow_period_ms_)) { continue; }
            if (!send(std::span<const std::uint8_t>{slot.wire.data(), slot.size})) { ++busy_; return false; }
            slot.size = 0; ++sent_[i];
            if (i == 3) { last_slow_ = now; slow_sent_ = true; }
            return true; // 每次调用最多交付一帧, 不用积压数量决定控制周期工作量.
        }
        return false;
    }
    [[nodiscard]] std::array<std::uint32_t, 10> diagnostics() const noexcept {
        std::array<std::uint32_t, 10> out{};
        for (std::size_t i=0; i<4; ++i) { out[i]=sent_[i]; out[4+i]=replaced_[i]; out[9]+=slots_[i].size ? 1u : 0u; }
        out[8]=busy_; return out;
    }
private:
    static std::size_t classify(const transport::UartFrame& request) noexcept {
        switch (static_cast<MessageType>(request.type)) {
        case MessageType::hello: case MessageType::authorize: case MessageType::align: case MessageType::param_write:
            return 0;
        case MessageType::targets: case MessageType::permit: return 1;
        case MessageType::status: return request.size == 11 && request.payload[10] == 0 ? 2 : 3;
        default: return 3;
        }
    }
    struct Slot { std::array<std::uint8_t, transport::uart_wire_max> wire{}; std::size_t size{}; };
    std::array<Slot, 4> slots_{};
    std::array<std::uint32_t, 4> sent_{}, replaced_{};
    std::uint32_t busy_{}, slow_period_ms_{};
    Tick last_slow_{};
    bool slow_sent_{};
};
} // 命名空间 ec::sentinel::app
