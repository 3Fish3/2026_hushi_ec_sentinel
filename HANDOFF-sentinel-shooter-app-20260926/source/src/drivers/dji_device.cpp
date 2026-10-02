#include "ec/drivers/dji_device.hpp"
#include <cmath>
#include <limits>
#include <numbers>

namespace ec::drivers {
namespace {
constexpr double tau = 2 * std::numbers::pi_v<double>;
constexpr double encoder_step = tau / 8192;

bool valid_config(const DjiDeviceConfig& c) noexcept {
    const auto p = dji_protocol(c.mode);
    return p && c.bus != 0 && c.device_id > 0 && c.device_id <= p->count &&
        std::isfinite(c.rotor_per_axis) && c.rotor_per_axis > 0 &&
        (c.direction == 1 || c.direction == -1) && c.feedback_ttl.valid() &&
        std::isfinite(c.max_rotor_speed_rad_s) && c.max_rotor_speed_rad_s > 0 &&
        c.output_limit_raw > 0 && c.output_limit_raw <= p->limit;
}
unsigned group_of(const DjiDeviceConfig& c) noexcept { return (c.device_id - 1u) / 4u + 1u; }
std::uint16_t tx_id(const DjiDeviceConfig& c, const DjiProtocol& p) noexcept {
    return group_of(c) == 1 ? p.first_tx : p.second_tx;
}
bool fits_float(double value) noexcept {
    return std::isfinite(value) && std::fabs(value) <= std::numeric_limits<float>::max();
}
} // 内部辅助函数

bool valid_dji_addresses(std::span<const DjiDeviceConfig> devices) noexcept {
    if (devices.empty()) { return false; }
    for (std::size_t i = 0; i < devices.size(); ++i) {
        const auto& a = devices[i];
        const auto protocol = dji_protocol(a.mode);
        if (!protocol || a.bus == 0 || a.device_id == 0 || a.device_id > protocol->count) { return false; }
        const auto pa = *protocol;
        for (std::size_t j = 0; j < i; ++j) {
            const auto& b = devices[j];
            if (a.bus != b.bus) { continue; }
            const auto pb = *dji_protocol(b.mode);
            if (pa.rx_base + a.device_id == pb.rx_base + b.device_id) { return false; }
            // C610/C620 可同组合并; 不支持跨协议族/组号共享同一命令 ID.
            if (tx_id(a, pa) == tx_id(b, pb) &&
                (pa.rx_base != pb.rx_base || group_of(a) != group_of(b))) { return false; }
        }
    }
    return true;
}

bool valid_dji_layout(std::span<const DjiDeviceConfig> devices) noexcept {
    if (!valid_dji_addresses(devices)) { return false; }
    for (const auto& device : devices) { if (!valid_config(device)) { return false; } }
    return true;
}

DjiDevice::DjiDevice(DjiDeviceConfig config) noexcept
    : config_(config), configured_(valid_config(config)) {}

void DjiDevice::lose_position() noexcept {
    state_.position_rad.validity = Validity::invalid;
}
void DjiDevice::reset() noexcept {
    state_ = {};
    previous_encoder_ = 0;
    position_rad_ = 0;
}

DjiReceiveResult DjiDevice::receive(std::uint8_t bus,
    const transport::CanFrame& frame, Tick received_at) noexcept {
    if (!configured_) { return DjiReceiveResult::invalid_config; }
    const auto p = *dji_protocol(config_.mode);
    if (bus != config_.bus || frame.id != p.rx_base + config_.device_id) {
        return DjiReceiveResult::ignored;
    }
    const auto decoded = decode_dji(config_.mode, frame);
    if (!decoded || std::fabs(decoded->rotor_speed_rad_s) > config_.max_rotor_speed_rad_s) {
        lose_position();
        return DjiReceiveResult::invalid_frame; // 不刷新最后有效速度的时间戳.
    }
    const double velocity = static_cast<double>(decoded->rotor_speed_rad_s) *
        config_.direction / config_.rotor_per_axis;
    if (!fits_float(velocity)) {
        lose_position();
        return DjiReceiveResult::invalid_frame;
    }
    const auto encoder = static_cast<std::uint16_t>((static_cast<unsigned>(frame.data[0]) << 8) | frame.data[1]);
    if (state_.rotor.validity == Validity::valid) {
        const std::uint32_t gap = received_at.ms - state_.rotor.sampled_at.ms;
        if (gap >= 0x80000000u) {
            lose_position();
            return DjiReceiveResult::invalid_time;
        }
        // 本地毫秒戳有量化误差, 保守增加 1 ms 和两格编码器误差.
        // 无序号/源端戳时不能检测所有延迟、重复帧或短暂设备重启.
        const double max_delta = config_.max_rotor_speed_rad_s *
            ((static_cast<double>(gap) + 1) * 0.001) + 2 * encoder_step;
        // 用原始计数判定跨圈与精确半圈, 避免浮点角度舍入改变歧义边界.
        int delta_counts = static_cast<int>(encoder) - previous_encoder_;
        if (delta_counts > 4096) { delta_counts -= 8192; }
        if (delta_counts < -4096) { delta_counts += 8192; }
        const double delta = delta_counts * encoder_step;
        if (!fresh(received_at, state_.rotor.sampled_at, config_.feedback_ttl) ||
            std::abs(delta_counts) == 4096 || max_delta >= std::numbers::pi_v<double> || std::fabs(delta) > max_delta) {
            lose_position();
        } else if (state_.position_rad.validity == Validity::valid) {
            const double next = position_rad_ + config_.direction * delta / config_.rotor_per_axis;
            if (!fits_float(next)) { lose_position(); }
            else {
                position_rad_ = next;
                state_.position_rad = {static_cast<float>(next), received_at, Validity::valid};
            }
        }
    }
    previous_encoder_ = encoder;
    state_.rotor = {*decoded, received_at, Validity::valid};
    state_.velocity_rad_s = {static_cast<float>(velocity), received_at, Validity::valid};
    return DjiReceiveResult::accepted;
}

DjiDeviceState DjiDevice::state(Tick now) const noexcept {
    auto result = state_;
    if (state_.rotor.validity == Validity::valid &&
        !fresh(now, state_.rotor.sampled_at, config_.feedback_ttl)) {
        result.rotor.validity = result.velocity_rad_s.validity = Validity::invalid;
        if (result.position_rad.validity == Validity::valid) { result.position_rad.validity = Validity::invalid; }
    }
    return result;
}

bool DjiDevice::reference_position(float axis_rad, Tick now) noexcept {
    lose_position();
    if (!configured_ || !std::isfinite(axis_rad) || !state_.rotor.fresh_at(now, config_.feedback_ttl)) {
        return false;
    }
    position_rad_ = axis_rad;
    state_.position_rad = {axis_rad, state_.rotor.sampled_at, Validity::valid};
    return true;
}

std::optional<std::int16_t> DjiDevice::encode_output(float normalized) const noexcept {
    if (!configured_ || !std::isfinite(normalized) || std::fabs(normalized) > 1) { return std::nullopt; }
    // 先校验有限性与范围再转换, 最大协议量 25000 可安全放入 int16_t.
    return static_cast<std::int16_t>(std::lround(normalized * config_.direction * config_.output_limit_raw));
}

DjiTxGroup::DjiTxGroup(transport::CanTx& bus, std::uint8_t bus_id, unsigned group,
    Lifetime output_ttl, std::array<const DjiDevice*, 4> devices) noexcept
    : bus_(bus), group_(group), output_ttl_(output_ttl), devices_(devices) {
    if (!output_ttl.valid() || (group != 1 && group != 2) || bus_id == 0) { return; }
    std::array<DjiDeviceConfig, 4> layout{};
    std::size_t count = 0;
    int encoding_limit = 0;
    std::optional<std::uint16_t> command_id{};
    for (std::size_t i = 0; i < devices_.size(); ++i) {
        const auto* device = devices_[i];
        if (!device) { continue; }
        const auto& c = device->config();
        if (!device->configured() || c.bus != bus_id || c.device_id != (group - 1) * 4 + i + 1) { return; }
        const auto p = *dji_protocol(c.mode);
        const auto id = tx_id(c, p);
        if (command_id && *command_id != id) { return; }
        command_id = id;
        layout[count++] = c;
        // 同帧 C610/C620 每槽先按自己的限值检查, 最终用较大编码范围封装.
        if (p.limit > encoding_limit) { encoding_limit = p.limit; encoding_mode_ = c.mode; }
    }
    configured_ = valid_dji_layout({layout.data(), count});
}

DjiGroupResult DjiTxGroup::send(Tick now,
    const std::array<std::optional<DjiOutput>, 4>& outputs) noexcept {
    if (!configured_) { return {DjiGroupStatus::invalid_config, std::nullopt}; }
    std::array<std::int16_t, 4> raw{};
    for (std::size_t i = 0; i < devices_.size(); ++i) {
        if (!devices_[i]) {
            if (outputs[i]) { return {DjiGroupStatus::invalid_output, std::nullopt}; }
            continue;
        }
        if (!outputs[i]) { return {DjiGroupStatus::missing_output, std::nullopt}; }
        if (!fresh(now, outputs[i]->computed_at, output_ttl_)) {
            return {DjiGroupStatus::stale_output, std::nullopt};
        }
        const auto value = devices_[i]->encode_output(outputs[i]->normalized);
        if (!value) { return {DjiGroupStatus::invalid_output, std::nullopt}; }
        raw[i] = *value;
    }
    const auto frame = encode_dji(encoding_mode_, group_, raw);
    if (!frame) { return {DjiGroupStatus::invalid_output, std::nullopt}; }
    const auto result = bus_.try_send(*frame);
    switch (result) {
    case transport::SendResult::accepted: return {DjiGroupStatus::accepted, result};
    case transport::SendResult::busy: return {DjiGroupStatus::busy, result};
    default: return {DjiGroupStatus::error, result};
    }
}
} // 命名空间 ec::drivers
