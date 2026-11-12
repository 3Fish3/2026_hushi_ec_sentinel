#pragma once
#include "ec/transport/can.hpp"
#include <cmath>
#include <cstdint>
#include <optional>

namespace ec::drivers {
// 达妙 DM-J4310-2EC V1.1 (手册 V1.2) MIT 协议. 所有量都是设备输出轴上的物理量;
// 不在此层模拟编码器计数, 也不做旧代码的 1.18/6.25 线性位置变换.
// 位置/速度/力矩的映射范围 (PMAX/VMAX/TMAX) 必须由每个设备显式配置: 它们是设备侧
// 可写的映射参数, 不是所有型号共用的常量.
struct DmMitLimits {
    float position_max_rad{};
    float velocity_max_rad_s{};
    float torque_max_nm{};
};

// Kp/Kd 的协议范围由手册固定, 与 PMAX/VMAX/TMAX 不同, 不随设备配置变化.
inline constexpr float dm_kp_max = 500.0f; // 12 bit, [0, 500]
inline constexpr float dm_kd_max = 5.0f;   // 12 bit, [0, 5]

// 反馈状态半字节: 0 为掉电默认的 Disabled, 1 为 Enabled, 8..E 为故障.
enum class DmStatus : std::uint8_t {
    disabled = 0x0, enabled = 0x1,
    over_voltage = 0x8, under_voltage = 0x9, over_current = 0xa,
    mos_over_temperature = 0xb, coil_over_temperature = 0xc,
    communication_lost = 0xd, overload = 0xe,
    unknown = 0xff, // 手册未定义的取值 (2..7, 0xF)
};

[[nodiscard]] inline bool dm_status_valid(std::uint8_t nibble) noexcept {
    return nibble == 0x0 || nibble == 0x1 || (nibble >= 0x8 && nibble <= 0xe);
}
[[nodiscard]] inline DmStatus dm_status(std::uint8_t nibble) noexcept {
    return dm_status_valid(nibble) ? static_cast<DmStatus>(nibble) : DmStatus::unknown;
}
// 故障态与 Disabled 都不代表设备在使能状态下工作; 两者都不自动清除.
[[nodiscard]] inline bool dm_status_enabled(DmStatus status) noexcept {
    return status == DmStatus::enabled;
}
[[nodiscard]] inline bool dm_status_fault(DmStatus status) noexcept {
    return status >= DmStatus::over_voltage && status <= DmStatus::overload;
}

struct DmMitCommand {
    float position_rad{};   // ±position_max_rad
    float velocity_rad_s{}; // ±velocity_max_rad_s
    float kp{};             // [0, 500]
    float kd{};             // [0, 5]
    float torque_nm{};      // ±torque_max_nm
};

struct DmFeedback {
    std::uint8_t node_id{}; // 反馈载荷中的控制器 ID (低 4 位)
    DmStatus status{DmStatus::unknown};
    float position_rad{};
    float velocity_rad_s{};
    float torque_nm{};
    std::uint8_t mos_temperature_c{};
    std::uint8_t rotor_temperature_c{};
};

// 物理量 ↔ 协议 raw. 编码按手册的线性映射并在 0..(2^bits-1) 内截断取整; 越界与非有限
// 输入返回空, 不静默钳位. 分开暴露 raw 与物理量, 以便核对"旧 raw 35 ≈ 4.2735"这类换算.
[[nodiscard]] inline std::optional<std::uint16_t> dm_encode_position(
    const DmMitLimits& limits, float position_rad) noexcept {
    if (!std::isfinite(position_rad) || !std::isfinite(limits.position_max_rad) ||
        limits.position_max_rad <= 0 || std::fabs(position_rad) > limits.position_max_rad) {
        return std::nullopt;
    }
    constexpr double span = 65535.0;
    const double raw = (static_cast<double>(position_rad) + limits.position_max_rad) * span /
                       (2.0 * limits.position_max_rad);
    return static_cast<std::uint16_t>(raw);
}
[[nodiscard]] inline std::optional<float> dm_decode_position(
    const DmMitLimits& limits, std::uint16_t raw) noexcept {
    if (!std::isfinite(limits.position_max_rad) || limits.position_max_rad <= 0) { return std::nullopt; }
    return static_cast<float>(static_cast<double>(raw) * 2.0 * limits.position_max_rad / 65535.0 -
                              limits.position_max_rad);
}
[[nodiscard]] inline std::optional<std::uint16_t> dm_encode_velocity(
    const DmMitLimits& limits, float velocity_rad_s) noexcept {
    if (!std::isfinite(velocity_rad_s) || !std::isfinite(limits.velocity_max_rad_s) ||
        limits.velocity_max_rad_s <= 0 || std::fabs(velocity_rad_s) > limits.velocity_max_rad_s) {
        return std::nullopt;
    }
    const double raw = (static_cast<double>(velocity_rad_s) + limits.velocity_max_rad_s) * 4095.0 /
                       (2.0 * limits.velocity_max_rad_s);
    return static_cast<std::uint16_t>(raw);
}
[[nodiscard]] inline std::optional<float> dm_decode_velocity(
    const DmMitLimits& limits, std::uint16_t raw) noexcept {
    if (!std::isfinite(limits.velocity_max_rad_s) || limits.velocity_max_rad_s <= 0) { return std::nullopt; }
    return static_cast<float>(static_cast<double>(raw) * 2.0 * limits.velocity_max_rad_s / 4095.0 -
                              limits.velocity_max_rad_s);
}
[[nodiscard]] inline std::optional<std::uint16_t> dm_encode_torque(
    const DmMitLimits& limits, float torque_nm) noexcept {
    if (!std::isfinite(torque_nm) || !std::isfinite(limits.torque_max_nm) ||
        limits.torque_max_nm <= 0 || std::fabs(torque_nm) > limits.torque_max_nm) {
        return std::nullopt;
    }
    const double raw = (static_cast<double>(torque_nm) + limits.torque_max_nm) * 4095.0 /
                       (2.0 * limits.torque_max_nm);
    return static_cast<std::uint16_t>(raw);
}
[[nodiscard]] inline std::optional<float> dm_decode_torque(
    const DmMitLimits& limits, std::uint16_t raw) noexcept {
    if (!std::isfinite(limits.torque_max_nm) || limits.torque_max_nm <= 0) { return std::nullopt; }
    return static_cast<float>(static_cast<double>(raw) * 2.0 * limits.torque_max_nm / 4095.0 -
                              limits.torque_max_nm);
}
[[nodiscard]] inline std::optional<std::uint16_t> dm_encode_kp(float kp) noexcept {
    if (!std::isfinite(kp) || kp < 0.0f || kp > dm_kp_max) { return std::nullopt; }
    return static_cast<std::uint16_t>(static_cast<double>(kp) * 4095.0 / dm_kp_max);
}
[[nodiscard]] inline std::optional<float> dm_decode_kp(std::uint16_t raw) noexcept {
    return static_cast<float>(static_cast<double>(raw) * dm_kp_max / 4095.0);
}
[[nodiscard]] inline std::optional<std::uint16_t> dm_encode_kd(float kd) noexcept {
    if (!std::isfinite(kd) || kd < 0.0f || kd > dm_kd_max) { return std::nullopt; }
    return static_cast<std::uint16_t>(static_cast<double>(kd) * 4095.0 / dm_kd_max);
}
[[nodiscard]] inline std::optional<float> dm_decode_kd(std::uint16_t raw) noexcept {
    return static_cast<float>(static_cast<double>(raw) * dm_kd_max / 4095.0);
}

// MIT 命令帧: ID 等于设备配置的 CAN ID; DLC 固定 8, 大端半字节打包.
// void 语义的使能/失能命令用同一 ID, 数据为 7 字节 0xFF 加最后一个命令字节.
enum class DmAction : std::uint8_t { disable = 0xfd, enable = 0xfc };

[[nodiscard]] inline std::optional<transport::CanFrame> encode_dm_mit(
    std::uint16_t can_id, const DmMitLimits& limits, const DmMitCommand& command) noexcept {
    if (can_id > 0x7ff) { return std::nullopt; }
    const auto position = dm_encode_position(limits, command.position_rad);
    const auto velocity = dm_encode_velocity(limits, command.velocity_rad_s);
    const auto kp = dm_encode_kp(command.kp);
    const auto kd = dm_encode_kd(command.kd);
    const auto torque = dm_encode_torque(limits, command.torque_nm);
    if (!position || !velocity || !kp || !kd || !torque) { return std::nullopt; }
    transport::CanFrame frame{can_id, 8, {}};
    frame.data[0] = static_cast<std::uint8_t>(*position >> 8);
    frame.data[1] = static_cast<std::uint8_t>(*position & 0xff);
    frame.data[2] = static_cast<std::uint8_t>(*velocity >> 4);
    frame.data[3] = static_cast<std::uint8_t>(((*velocity & 0x0f) << 4) | (*kp >> 8));
    frame.data[4] = static_cast<std::uint8_t>(*kp & 0xff);
    frame.data[5] = static_cast<std::uint8_t>(*kd >> 4);
    frame.data[6] = static_cast<std::uint8_t>(((*kd & 0x0f) << 4) | (*torque >> 8));
    frame.data[7] = static_cast<std::uint8_t>(*torque & 0xff);
    return frame;
}

[[nodiscard]] inline std::optional<transport::CanFrame> encode_dm_action(
    std::uint16_t can_id, DmAction action) noexcept {
    if (can_id > 0x7ff) { return std::nullopt; }
    transport::CanFrame frame{can_id, 8, {}};
    frame.data.fill(0xff);
    frame.data[7] = static_cast<std::uint8_t>(action);
    return frame;
}

// 反馈帧: ID 为 Master ID, D[0] = 节点 ID | 状态半字节 << 4. 这里不做节点过滤,
// 由设备层按配置的节点字段判定归属.
[[nodiscard]] inline std::optional<DmFeedback> decode_dm_feedback(
    const transport::CanFrame& frame, const DmMitLimits& limits) noexcept {
    if (!frame.valid() || frame.size != 8) { return std::nullopt; }
    const auto position = static_cast<std::uint16_t>(
        (static_cast<unsigned>(frame.data[1]) << 8) | frame.data[2]);
    const auto velocity = static_cast<std::uint16_t>(
        (static_cast<unsigned>(frame.data[3]) << 4) | (frame.data[4] >> 4));
    const auto torque = static_cast<std::uint16_t>(
        ((static_cast<unsigned>(frame.data[4]) & 0x0fu) << 8) | frame.data[5]);
    const auto decoded_position = dm_decode_position(limits, position);
    const auto decoded_velocity = dm_decode_velocity(limits, velocity);
    const auto decoded_torque = dm_decode_torque(limits, torque);
    if (!decoded_position || !decoded_velocity || !decoded_torque) { return std::nullopt; }
    DmFeedback result{};
    result.node_id = static_cast<std::uint8_t>(frame.data[0] & 0x0fu);
    result.status = dm_status(static_cast<std::uint8_t>(frame.data[0] >> 4));
    result.position_rad = *decoded_position;
    result.velocity_rad_s = *decoded_velocity;
    result.torque_nm = *decoded_torque;
    result.mos_temperature_c = frame.data[6];
    result.rotor_temperature_c = frame.data[7];
    return result;
}
} // 命名空间 ec::drivers
