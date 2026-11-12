#pragma once
#include "ec/drivers/dji.hpp"
#include "board_current_harness.hpp"
#include <cstddef>

namespace ec::sentinel {
// 10 ms 观察周期内最多搬运一圈, 分成短临界区, 足以覆盖 115200 8N1 的持续输入.
constexpr std::size_t uart1_poll_budget = 512;
constexpr std::size_t uart1_copy_chunk = 64;

// 装机映射来自硬件 Checklist: CAN1 双 yaw, CAN2 六个 C620 和 ID7 的 C610.
// 未配置的 ID 仅统计在线, 不尝试用互相重叠的协议猜测设备类型.
[[nodiscard]] constexpr std::optional<drivers::DjiMode> dji_mode(
    std::uint8_t bus, std::uint16_t id) noexcept {
#if EC_APP_CHASSIS_GIMBAL_INTEGRATION || EC_CAPTURE_CURRENT_HARNESS
    for (const auto& slot : board::current_harness) {
        if (slot.bus!=bus || slot.rx_id!=id || slot.kind==capture::Kind::dm) { continue; }
        return slot.kind==capture::Kind::gm6020 ? drivers::DjiMode::gm6020_current :
            slot.kind==capture::Kind::c610 ? drivers::DjiMode::c610_current : drivers::DjiMode::c620_current;
    }
#else
    if (bus == 1 && (id == 0x208 || id == 0x20a)) { return drivers::DjiMode::gm6020_current; }
    if (bus == 2 && id >= 0x201 && id <= 0x206) { return drivers::DjiMode::c620_current; }
    if (bus == 2 && id == 0x207) { return drivers::DjiMode::c610_current; }
#endif
    return std::nullopt;
}
} // 命名空间 ec::sentinel
