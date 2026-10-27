#pragma once
#include "ec/drivers/dji_device.hpp"
#include "ec/drivers/dm_device.hpp"
#include <span>

namespace ec::drivers {
// 混合总线静态核对: 同一条总线上一个 11 位 CAN ID 只应有一种含义.
// 检查 DM 反馈/命令 ID 与 DJI 反馈区间、DJI 命令 ID 是否重叠; 这只能发现配置冲突,
// 不能代替实测抓包, 也不校验设备侧实际写入的 ID.
[[nodiscard]] inline bool valid_mixed_bus(std::span<const DmDeviceConfig> dm,
                                          std::span<const DjiDeviceConfig> dji) noexcept {
    if (dm.empty() && dji.empty()) { return false; }
    for (const auto& device : dm) {
        if (device.bus == 0 || device.can_id > 0x7ff || device.master_id > 0x7ff) { return false; }
        for (const auto& dji_device : dji) {
            if (dji_device.bus != device.bus) { continue; }
            const auto protocol = dji_protocol(dji_device.mode);
            if (!protocol) { return false; }
            const auto first_feedback = protocol->rx_base + 1;
            const auto last_feedback = protocol->rx_base + protocol->count;
            // MCU 接收的 DJI 反馈 ID 集合不能与 DM 反馈 ID 重叠, 否则同帧会被两种协议认领.
            if (device.master_id >= first_feedback && device.master_id <= last_feedback) { return false; }
            // DJI 命令 ID 与 DM 反馈 ID 相同会让总线上一个 ID 同时表示命令和反馈.
            if (device.master_id == protocol->first_tx || device.master_id == protocol->second_tx) {
                return false;
            }
            // DM 命令 ID 落在 DJI 反馈区间或等于 DJI 命令 ID 同样冲突.
            if (device.can_id >= first_feedback && device.can_id <= last_feedback) { return false; }
            if (device.can_id == protocol->first_tx || device.can_id == protocol->second_tx) {
                return false;
            }
        }
    }
    return true;
}
} // 命名空间 ec::drivers
