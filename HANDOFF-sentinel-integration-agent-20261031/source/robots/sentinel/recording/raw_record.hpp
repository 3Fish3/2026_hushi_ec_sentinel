#pragma once
#include "ec/time.hpp"
#include "ec/transport/can.hpp"
#include "ec/drivers/bmi088_device.hpp"
#include <array>
#include <bit>
#include <cstdint>
#include <optional>
#include <span>

namespace ec::sentinel::recording {
inline constexpr std::uint8_t version = 1, request_type = 0x31, response_type = 0xb1;
inline constexpr std::size_t record_bytes = 64, capacity = 512, reply_header = 32;
// 标识单位/流布局与当前 BMI088 寄存器方案, 不是机器的物理标定身份.
inline constexpr std::uint32_t layout_id = 0x52415701;
enum class Profile : std::uint8_t { stationary = 1, rotation = 2, control = 3 };
enum class Kind : std::uint8_t { gyro = 1, accel = 2, temperature = 3, can = 4, control = 5 };
enum class State : std::uint8_t { idle, recording, frozen, full };
struct Record {
    Kind kind{};
    std::uint8_t stream{}, flags{}; // bit0 有效, bit1 时刻为估计, bit2 为接收时刻
    std::uint32_t sequence{}, source_sequence{}, sampled_us{}, arrived_us{}, lost{}, error_us{}, run{};
    std::array<std::uint32_t, 8> data{};
};
inline void put32(std::span<std::uint8_t> out, std::size_t at, std::uint32_t v) noexcept {
    for (unsigned i = 0; i < 4; ++i) { out[at + i] = static_cast<std::uint8_t>(v >> (8 * i)); }
}
inline std::uint32_t get32(std::span<const std::uint8_t> in, std::size_t at) noexcept {
    std::uint32_t v{};
    for (unsigned i = 0; i < 4; ++i) { v |= static_cast<std::uint32_t>(in[at + i]) << (8 * i); }
    return v;
}
inline bool encode(const Record& r, std::span<std::uint8_t> out) noexcept {
    if (out.size() < record_bytes) { return false; }
    out[0] = version; out[1] = static_cast<std::uint8_t>(r.kind); out[2] = r.stream; out[3] = r.flags;
    const std::array header{r.sequence, r.source_sequence, r.sampled_us, r.arrived_us, r.lost, r.error_us, r.run};
    for (std::size_t i = 0; i < header.size(); ++i) { put32(out, 4 + i * 4, header[i]); }
    for (std::size_t i = 0; i < r.data.size(); ++i) { put32(out, 32 + i * 4, r.data[i]); }
    return true;
}
inline std::optional<Record> decode(std::span<const std::uint8_t> in) noexcept {
    if (in.size() != record_bytes || in[0] != version || in[1] < 1 || in[1] > 5 || (in[3] & ~7u)) {
        return std::nullopt;
    }
    Record r{static_cast<Kind>(in[1]), in[2], in[3], get32(in, 4), get32(in, 8), get32(in, 12),
        get32(in, 16), get32(in, 20), get32(in, 24), get32(in, 28)};
    for (std::size_t i = 0; i < r.data.size(); ++i) { r.data[i] = get32(in, 32 + i * 4); }
    return r;
}
struct CanAddress { std::uint8_t bus; std::uint16_t id; };
// 默认保留历史流号契约; 新接线由板级显式传入, 原始CAN地址仍写入每条记录.
inline constexpr std::array<CanAddress,10> legacy_can_addresses{{
    {2,0x201},{2,0x202},{2,0x203},{2,0x204},{1,0x20a},{1,0x208},
    {2,0x205},{2,0x206},{2,0x207},{1,0}}};
// 固定短窗, 冻结后才读取. 不覆盖, 不为慢 PC 无限积压; 一个控制/采集任务拥有写入.
class Recorder {
public:
    explicit constexpr Recorder(std::array<CanAddress,10> addresses=legacy_can_addresses) noexcept : addresses_(addresses) {}
    void advance(MicroTick now) noexcept {
        if (state_ == State::recording && now.us - start_.us >= duration_) { state_ = State::frozen; end_ = now; }
    }
    void push(Record r, MicroTick now) noexcept;
    void imu(const drivers::Bmi088Reading& sample, MicroTick now) noexcept;
    void can(std::uint8_t bus, const transport::CanFrame& frame, MicroTick at, MicroTick now,
        std::uint32_t lost) noexcept;
    // 一个有界请求 -> 一个有界响应; 调用方负责与写任务的短临界区互斥.
    [[nodiscard]] std::size_t request(std::span<const std::uint8_t> in,
        std::span<std::uint8_t> out, MicroTick now) noexcept;
    // 连续采集复用固定512条RAM为队列, 保留源时刻/全局序号; 满队列丢新条并计数.
    [[nodiscard]] std::size_t stream_request(std::span<const std::uint8_t> in,
        std::span<std::uint8_t> out, MicroTick now) noexcept;
    [[nodiscard]] std::size_t stream_packet(std::span<std::uint8_t> out, MicroTick now,
        std::size_t& count) noexcept;
    void stream_consume(std::size_t count) noexcept;
    [[nodiscard]] bool fast_dm() const noexcept { return state_ == State::recording && profile_ == Profile::rotation; }
    [[nodiscard]] int control_domain() const noexcept { return state_ == State::recording && profile_ == Profile::control ? selected_domain_ : -1; }
    [[nodiscard]] State state() const noexcept { return state_; }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
private:
    std::array<CanAddress,10> addresses_;
    std::array<Record, capacity> records_{};
    std::array<MicroTick, 32> last_{};
    std::array<bool, 32> seen_{};
    std::array<std::uint32_t, 32> sources_{};
    std::array<std::uint32_t, 10> can_sequences_{};
    State state_{};
    Profile profile_{Profile::stationary};
    std::uint8_t selected_domain_{};
    MicroTick start_{}, end_{};
    std::uint32_t run_{}, duration_{}, lost_{}, sequence_{};
    std::size_t size_{}, head_{};
    bool continuous_{}, stream_finished_{};
    void stream_header(std::span<std::uint8_t> out, std::uint8_t op, std::uint8_t error,
        MicroTick now, std::size_t remaining) const noexcept;
};
} // 命名空间 ec::sentinel::recording
