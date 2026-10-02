#pragma once
#include "ec/drivers/dji.hpp"
#include "ec/platform/host.hpp"
#include "ec/time.hpp"
#include "ec/transport/can.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>

namespace ec::sil {
// 仅用于主机验证, 不连接硬件、不创建线程, 不使用墙上时钟等待.
class Clock {
public:
    explicit Clock(std::uint32_t initial_ms = 0) : initial_ms_(initial_ms) {}
    void advance(std::uint32_t ms) noexcept { elapsed_ms_ += ms; }
    [[nodiscard]] Tick now() const noexcept { return {initial_ms_ + static_cast<std::uint32_t>(elapsed_ms_)}; }
    [[nodiscard]] std::uint64_t elapsed_ms() const noexcept { return elapsed_ms_; }
private:
    std::uint32_t initial_ms_{};
    std::uint64_t elapsed_ms_{};
};

enum class CanFault { none, drop, busy, error };
// 有界延迟传输: accepted 后也可丢包. 不模拟 CAN 位仲裁、重发或电气故障.
class CanLink final : public transport::CanTx {
public:
    explicit CanLink(const Clock& clock) : clock_(clock) {}
    CanFault fault{CanFault::none};
    std::uint32_t delay_ms{0};
    std::uint32_t dropped{0};

    transport::SendResult try_send(const transport::CanFrame& frame) noexcept override {
        if (!frame.valid()) { return transport::SendResult::invalid; }
        if (fault == CanFault::busy) { return transport::SendResult::busy; }
        if (fault == CanFault::error) { return transport::SendResult::error; }
        if (fault == CanFault::drop) { ++dropped; return transport::SendResult::accepted; }
        if (count_ == pending_.size()) { return transport::SendResult::busy; }
        pending_[write_] = {frame, clock_.elapsed_ms() + delay_ms};
        write_ = (write_ + 1) % pending_.size();
        ++count_;
        return transport::SendResult::accepted;
    }

    [[nodiscard]] std::optional<transport::CanFrame> receive() noexcept {
        if (count_ == 0 || pending_[read_].due_ms > clock_.elapsed_ms()) { return std::nullopt; }
        const auto result = pending_[read_].frame;
        read_ = (read_ + 1) % pending_.size();
        --count_;
        return result;
    }
private:
    struct Pending { transport::CanFrame frame{}; std::uint64_t due_ms{}; };
    const Clock& clock_;
    std::array<Pending, 8> pending_{};
    std::size_t read_{}, write_{}, count_{};
};

// 纯虚构 C620 台架: 一阶转子模型, 无减速箱/真实惯量/热模型.
// 归一化输出到原始协议数的映射、阻尼和看门狗都是测试夹具参数, 不能部署到实车.
class Motor {
public:
    static constexpr int raw_limit = 10000;
    static constexpr Lifetime watchdog{25};
    explicit Motor(std::uint8_t device_id = 1) : device_id_(device_id) {}
    bool powered{true};
    bool foreign_slot_command{false};
    float speed_rad_s{};
    float angle_rad{};
    float applied{};

    void power(bool enabled) noexcept {
        powered = enabled;
        applied = 0;
        last_command_.reset();
    }

    void receive(const transport::CanFrame& frame, Tick now) noexcept {
        if (!powered || device_id_ < 1 || device_id_ > 8 || frame.size != 8 ||
            frame.id != (device_id_ <= 4 ? 0x200 : 0x1ff)) { return; }
        const std::size_t offset = ((device_id_ - 1u) % 4u) * 2u;
        // 仅供单电机夹具检查其他槽位; 多电机测试允许其他槽非零.
        for (std::size_t i = 0; i < frame.data.size(); ++i) {
            if (i != offset && i != offset + 1) {
                foreign_slot_command = foreign_slot_command || frame.data[i] != 0;
            }
        }
        const unsigned bits = (static_cast<unsigned>(frame.data[offset]) << 8) | frame.data[offset + 1];
        const int raw = bits < 0x8000 ? static_cast<int>(bits) : static_cast<int>(bits) - 65536;
        applied = std::clamp(static_cast<float>(raw) / raw_limit, -1.0f, 1.0f);
        last_command_ = now;
    }

    void step(Tick now, float dt_s) noexcept {
        if (!powered || !last_command_ || !fresh(now, *last_command_, watchdog)) { applied = 0; }
        // 解析的一阶离散化, 避免大步长导致数值爆炸; 不据此证明真实调参稳定性.
        speed_rad_s += (80.0f * applied - speed_rad_s) * (1.0f - std::exp(-dt_s / 0.08f));
        angle_rad = std::fmod(angle_rad + speed_rad_s * dt_s, 2.0f * std::numbers::pi_v<float>);
        if (angle_rad < 0) { angle_rad += 2.0f * std::numbers::pi_v<float>; }
    }

    [[nodiscard]] transport::CanFrame feedback() const noexcept {
        transport::CanFrame frame{static_cast<std::uint16_t>(0x200 + device_id_), 8, {}};
        const auto encoder = static_cast<std::uint16_t>(angle_rad * (8192.0f / (2.0f * std::numbers::pi_v<float>)));
        const int rpm = static_cast<int>(std::lround(speed_rad_s * (60.0f / (2.0f * std::numbers::pi_v<float>))));
        const auto speed = static_cast<std::uint16_t>(std::clamp(rpm, -32768, 32767));
        frame.data[0] = static_cast<std::uint8_t>(encoder >> 8);
        frame.data[1] = static_cast<std::uint8_t>(encoder);
        frame.data[2] = static_cast<std::uint8_t>(speed >> 8);
        frame.data[3] = static_cast<std::uint8_t>(speed);
        frame.data[6] = 25;
        return frame;
    }
private:
    std::uint8_t device_id_;
    std::optional<Tick> last_command_{};
};
} // 命名空间 ec::sil
