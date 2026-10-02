#include "ec/drivers/dm_device.hpp"
#include <cmath>

namespace ec::drivers {
namespace {
bool valid_limits(const DmMitLimits& limits) noexcept {
    return std::isfinite(limits.position_max_rad) && limits.position_max_rad > 0 &&
        std::isfinite(limits.velocity_max_rad_s) && limits.velocity_max_rad_s > 0 &&
        std::isfinite(limits.torque_max_nm) && limits.torque_max_nm > 0;
}
bool valid_config(const DmDeviceConfig& config) noexcept {
    // 手册建议小于 16, 但命令帧 ID 本身是标准 11 位; 真正的约束是共用 Master ID
    // 时的节点低 4 位唯一性, 由 valid_dm_layout 检查.
    if (config.can_id == config.master_id) { return false; } // 自己的反馈 ID 不能就是自己的命令 ID
    return config.bus != 0 && config.can_id <= 0x7ff && config.master_id <= 0x7ff &&
        valid_limits(config.limits) && config.feedback_ttl.valid();
}
std::uint8_t node_of(std::uint16_t can_id) noexcept { return static_cast<std::uint8_t>(can_id & 0x0fu); }
} // 内部辅助函数

bool valid_dm_addresses(std::span<const DmDeviceConfig> devices) noexcept {
    if (devices.empty()) { return false; }
    for (std::size_t i = 0; i < devices.size(); ++i) {
        const auto& device = devices[i];
        if (!device.bus || device.can_id > 0x7ff || device.master_id > 0x7ff ||
            device.can_id == device.master_id) { return false; }
        for (std::size_t j = 0; j < i; ++j) {
            if (devices[i].bus != devices[j].bus) { continue; }
            // 发送 ID 相同会互相覆盖命令; 反馈节点相同则无法归属.
            if (devices[i].can_id == devices[j].can_id) { return false; }
            if (devices[i].master_id == devices[j].master_id &&
                node_of(devices[i].can_id) == node_of(devices[j].can_id)) {
                return false;
            }
            // 同一总线上一个 11 位 ID 只能有一种含义: 一台设备的反馈 ID 不得等于另一台的命令 ID,
            // 否则一方的反馈帧会被另一方（含其电调）当成命令. 共享 Master ID 仍按载荷节点区分.
            if (devices[i].master_id == devices[j].can_id || devices[i].can_id == devices[j].master_id) {
                return false;
            }
        }
    }
    return true;
}

bool valid_dm_layout(std::span<const DmDeviceConfig> devices) noexcept {
    if (!valid_dm_addresses(devices)) { return false; }
    for (const auto& device : devices) { if (!valid_config(device)) { return false; } }
    return true;
}

DmDevice::DmDevice(DmDeviceConfig config) noexcept
    : config_(config), configured_(valid_config(config)) {}

DmReceiveResult DmDevice::receive(std::uint8_t bus,
    const transport::CanFrame& frame, Tick received_at) noexcept {
    if (!configured_) { return DmReceiveResult::invalid_config; }
    if (bus != config_.bus || frame.id != config_.master_id) { return DmReceiveResult::ignored; }
    if (!frame.valid() || frame.size != 8) { return DmReceiveResult::invalid_frame; }
    // 共用 Master ID 时按载荷节点字段路由; 其他节点的帧既不接受也不刷新本设备时效.
    if (static_cast<std::uint8_t>(frame.data[0] & 0x0fu) != node_of(config_.can_id)) {
        return DmReceiveResult::invalid_node;
    }
    if (has_received_ && tick_regressed(received_at, last_received_)) {
        return DmReceiveResult::invalid_time;
    }
    const auto decoded = decode_dm_feedback(frame, config_.limits);
    if (!decoded) { return DmReceiveResult::invalid_frame; }
    last_received_ = received_at;
    has_received_ = true;
    state_.feedback = {*decoded, received_at, Validity::valid};
    if (dm_status_enabled(decoded->status)) { state_.enable_confirmed = true; }
    return DmReceiveResult::accepted;
}

DmRequestResult DmDevice::request_result(Tick now) const noexcept {
    if (state_.pending_request == DmRequest::none) { return DmRequestResult::unknown; }
    if (!online(now)) { return DmRequestResult::unknown; }
    const auto status = state_.feedback.value.status;
    // 故障优先于请求方向: 使能请求遇到故障状态时不算被接受.
    if (dm_status_fault(status)) { return DmRequestResult::faulted; }
    if (state_.pending_request == DmRequest::enable && dm_status_enabled(status)) {
        return DmRequestResult::accepted;
    }
    if (state_.pending_request == DmRequest::disable && status == DmStatus::disabled) {
        return DmRequestResult::accepted;
    }
    return DmRequestResult::pending;
}

std::optional<transport::CanFrame> DmDevice::encode_mit(const DmMitCommand& command) const noexcept {
    if (!configured_) { return std::nullopt; }
    return encode_dm_mit(config_.can_id, config_.limits, command);
}

std::optional<transport::CanFrame> DmDevice::encode_enable() noexcept {
    if (!configured_) { return std::nullopt; }
    const auto frame = encode_dm_action(config_.can_id, DmAction::enable);
    if (frame) { state_.pending_request = DmRequest::enable; }
    return frame;
}

std::optional<transport::CanFrame> DmDevice::encode_disable() noexcept {
    if (!configured_) { return std::nullopt; }
    const auto frame = encode_dm_action(config_.can_id, DmAction::disable);
    if (frame) { state_.pending_request = DmRequest::disable; }
    return frame;
}

void DmDevice::reset() noexcept {
    state_ = {};
    has_received_ = false;
}
} // 命名空间 ec::drivers
