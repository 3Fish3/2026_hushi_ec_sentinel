#pragma once
#include "ec/time.hpp"
#include "ec/transport/can.hpp"
#include "ec/drivers/bmi088_device.hpp"
#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace ec::sentinel::recording {
inline constexpr std::uint8_t version = 1, request_type = 0x31, response_type = 0xb1;
inline constexpr std::size_t record_bytes = 64, capacity = 512, reply_header = 32;
// Pitch 连续控制流: 每条流各自限速 200 Hz, 单包至多 3 条完整 64B 记录 (32B 头 + 192B = 224B).
inline constexpr std::uint32_t pitch_stream_period_us = 5000;
inline constexpr std::size_t stream_packet_bytes = reply_header + 3 * record_bytes;
// 标识单位/流布局与当前 BMI088 寄存器方案, 不是机器的物理标定身份.
inline constexpr std::uint32_t layout_id = 0x52415701;
// 采集短窗配置方案:
// stationary(1): 仅 IMU
// rotation(2): IMU + 主云台/发射云台/俯仰轴 CAN 反馈 (stream 4, 5, 9)
// control(3): 短窗 (512 条固定 RAM, 64B/条, layout_id 0x52415701), 记录选定控制域的 CAN 反馈与 trace.
//   连续流 (stream_request op1) 只开放 Profile 3 + 域 6 (Pitch 达妙 DM 关节), 约束如下:
//     - 记录集严格限于 CAN 流 9 (CAN2 0x011) 与 control 域 6; 全部 IMU 记录在入口剔除.
//     - 每条流各自限速 200 Hz (5000us); 短窗历史速率 1000us 保持不变.
//     - 各记录按自身类型定长承载: gyro 48B, CAN 44B, control 完整 64B; 混合单包至多 3 条, 上限 224B.
//   物理带宽约束: UART 921600 8N1 理论带宽 ~92 KB/s (未计入转义与协议帧开销); 200 Hz x 2 流 x 64B 仍在预算内,
//   多轴 1kHz 连续外发仍不可行.
//
// control 域 6 (Pitch 达妙 DM 关节) 语义:
//   stream: 6 (控制域编号, 对应 CAN 流 9 达妙反馈 0x011)
//   data[0]: requested joint rad (float)
//   data[1]: adopted joint rad (float)
//   data[2]: measured joint rad (float)
//   data[3]: measured velocity rad/s (float)
//   data[4]: encoded MIT feedforward Nm (float; 无指令尝试时为 NaN sentinel, 严禁标为归一化电流)
//   data[5]: dt (float)
//   data[6]: word6 状态字 (mode[0:7], reason[8:15], pos_valid[16:19], vel_valid[20:23], limited[24],
//            req_frame[25:26], adopt_frame[27:28], bit29 excitation-active, bit30 excitation-latch)
//   data[7]: word7 本地 DjiGroupStatus (提交结果, 非 ESC 到达确认)
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
// 固定短窗, 冻结后才读取. 不覆盖, 不为慢 PC 无限积压; 一个控制/采集任务拥有写入.
class Recorder {
public:
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
    // Pitch 连续流就绪/健康查询. 纯只读: 不推进状态机, 不消费队列, 不产生任何副作用.
    // healthy: 活动中的连续 control 域 6 + 精确非零 run + 无丢条 + 32 位半回绕窗口内未过截止.
    // ready: healthy 全部条件之外, 剩余时长还必须 >= required_us.
    // run 不匹配视为已被替换或已停止; 队列满或累计丢条均判定为不健康.
    [[nodiscard]] bool pitch_stream_ready(std::uint32_t run, std::uint64_t required_us,
        MicroTick now) const noexcept;
    [[nodiscard]] bool pitch_stream_healthy(std::uint32_t run, MicroTick now) const noexcept;
    [[nodiscard]] State state() const noexcept { return state_; }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
private:
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
