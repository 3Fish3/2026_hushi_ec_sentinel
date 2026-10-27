#pragma once
#include "ec/drivers/dji.hpp"
#include <cstddef>

namespace ec::sentinel {
// 10 ms 观察周期内最多搬运一圈, 分成短临界区, 足以覆盖 115200 8N1 的持续输入.
constexpr std::size_t uart1_poll_budget = 512;
constexpr std::size_t uart1_copy_chunk = 64;

// 装机映射来自 2026-10-27 实车只读采集确认: CAN1 四轮 C620 (0x201..0x204) 与主 yaw
// GM6020 (0x208); CAN2 发射 yaw GM6020 (0x20a)、两摩擦轮 C620 (0x205/0x206) 与 C610 (0x207).
// 未配置的 ID 仅统计在线, 不尝试用互相重叠的协议猜测设备类型.
[[nodiscard]] constexpr std::optional<drivers::DjiMode> dji_mode(
    std::uint8_t bus, std::uint16_t id) noexcept {
    if (bus == 1 && id >= 0x201 && id <= 0x204) { return drivers::DjiMode::c620_current; }
    if (bus == 1 && id == 0x208) { return drivers::DjiMode::gm6020_current; }
    if (bus == 2 && id == 0x20a) { return drivers::DjiMode::gm6020_current; }
    if (bus == 2 && id >= 0x205 && id <= 0x206) { return drivers::DjiMode::c620_current; }
    if (bus == 2 && id == 0x207) { return drivers::DjiMode::c610_current; }
    return std::nullopt;
}
} // 命名空间 ec::sentinel
