#pragma once
#include "ec/time.hpp"
#include "ec/transport/can.hpp"
#include <array>
#include <cstdint>
#include <optional>

namespace ec::drivers {
struct WatchedDevice {
    std::uint8_t bus{};
    std::uint16_t id{};
    Tick last_seen{};
    std::uint32_t frames{};
    bool used{};
};

// 按 (bus, 标准 ID) 记录最近出现时间. 满员时拒绝新 ID, 不覆盖已有设备.
template<std::size_t Capacity>
class DeviceWatch {
    static_assert(Capacity > 0 && Capacity <= 64);
public:
    [[nodiscard]] bool note(std::uint8_t bus, std::uint16_t id, Tick now) noexcept {
        if (bus == 0 || id > 0x7ff) { return false; }
        if (auto* slot = find(bus, id)) {
            slot->last_seen = now;
            ++slot->frames;
            return true;
        }
        for (auto& slot : slots_) {
            if (slot.used) { continue; }
            slot = WatchedDevice{bus, id, now, 1, true};
            return true;
        }
        ++dropped_ids_;
        return false;
    }

    [[nodiscard]] bool online(std::uint8_t bus, std::uint16_t id, Tick now, Lifetime ttl) const noexcept {
        const auto* slot = find(bus, id);
        return slot && fresh(now, slot->last_seen, ttl);
    }

    [[nodiscard]] std::optional<WatchedDevice> get(std::uint8_t bus, std::uint16_t id) const noexcept {
        const auto* slot = find(bus, id);
        if (!slot) { return std::nullopt; }
        return *slot;
    }

    [[nodiscard]] std::uint32_t dropped_ids() const noexcept { return dropped_ids_; }
    [[nodiscard]] const std::array<WatchedDevice, Capacity>& slots() const noexcept { return slots_; }

private:
    [[nodiscard]] WatchedDevice* find(std::uint8_t bus, std::uint16_t id) noexcept {
        for (auto& slot : slots_) {
            if (slot.used && slot.bus == bus && slot.id == id) { return &slot; }
        }
        return nullptr;
    }
    [[nodiscard]] const WatchedDevice* find(std::uint8_t bus, std::uint16_t id) const noexcept {
        for (auto& slot : slots_) {
            if (slot.used && slot.bus == bus && slot.id == id) { return &slot; }
        }
        return nullptr;
    }

    std::array<WatchedDevice, Capacity> slots_{};
    std::uint32_t dropped_ids_{0};
};

struct StampedCanFrame {
    transport::CanFrame frame{};
    Tick received_at{};
    std::uint8_t bus{};
    MicroTick received_us{};
};
} // 命名空间 ec::drivers
