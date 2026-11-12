#pragma once
#include "ec/drivers/dr16.hpp"
#include "ec/time.hpp"
#include <array>
#include <cstdint>
#include <optional>

namespace ec::input {
// 一帧解码结果的边沿: 只在前后两帧都可信时才有意义.
// 启动、掉线恢复或人为 reset 后的第一帧不产生任何边沿, 避免把"一直按住的键"误报成新按下.
struct Dr16Edges {
    std::uint16_t keys_pressed{};
    std::uint16_t keys_released{};
    bool mouse_left_pressed{};
    bool mouse_left_released{};
    bool mouse_right_pressed{};
    bool mouse_right_released{};
};

struct Dr16Snapshot {
    drivers::Dr16Sample raw{};      // 与该帧完全一致的原始字段
    std::array<float, 4> axis{};    // dr16_axis(ch0..ch3), 归一化到 [-1, 1]
    std::int16_t ch4{};             // 第 5 通道原始值; 无依据的量程不做归一化
    drivers::Dr16Switch switch_left{drivers::Dr16Switch::unknown};
    drivers::Dr16Switch switch_right{drivers::Dr16Switch::unknown};
    std::uint16_t keys{};           // 键位位图, 具体键义由上层映射
    bool mouse_left{};
    bool mouse_right{};
    Dr16Edges edges{};
    Tick sampled_at{};
};

// 薄输入服务: 只维护最新样本、时效与边沿, 不定义键位含义、连击或控制权策略.
// 由单一任务拥有, 不加锁; 掉线判断用帧时间戳与 Lifetime, 不在这里做接管或仲裁.
class Dr16Input {
public:
    explicit Dr16Input(Lifetime sample_ttl) noexcept : ttl_(sample_ttl) {}
    Dr16Input(const Dr16Input&) = delete;
    Dr16Input& operator=(const Dr16Input&) = delete;

    // 启动或人工重建: 丢弃样本, 下一帧只用于恢复基准, 不产生边沿.
    void reset() noexcept {
        sample_ = {};
        has_sample_ = false;
        last_at_ = {};
    }

    // 时间戳倒退的帧被拒绝(半个回绕周期内才算倒退), 也不刷新时效.
    [[nodiscard]] bool accept(const drivers::Dr16Sample& sample, Tick received_at) noexcept {
        if (has_sample_ && (received_at.ms - last_at_.ms) >= 0x80000000u) {
            ++rejected_;
            return false;
        }
        // 上一帧是否仍算在线, 决定本帧能否作为边沿基准.
        const bool had_sample = has_sample_;
        const bool continuous = had_sample && fresh(received_at, last_at_, ttl_);
        const auto previous = sample_;
        sample_ = sample;
        last_at_ = received_at;
        has_sample_ = true;
        ++accepted_;
        if (continuous) {
            edges_ = diff(previous, sample);
        } else {
            edges_ = {};
            if (had_sample) { ++gaps_; }
        }
        return true;
    }

    [[nodiscard]] bool online(Tick now) const noexcept {
        return has_sample_ && fresh(now, last_at_, ttl_);
    }
    [[nodiscard]] Lifetime ttl() const noexcept { return ttl_; }
    [[nodiscard]] std::uint32_t accepted() const noexcept { return accepted_; }
    [[nodiscard]] std::uint32_t rejected() const noexcept { return rejected_; }
    // 掉线或时间倒退导致不连续基准的次数, 用于观测链路质量.
    [[nodiscard]] std::uint32_t gaps() const noexcept { return gaps_; }

    // 最近一次接受的边沿, 与 accept 同步更新; 不清零, 由调用方按需读取.
    [[nodiscard]] const Dr16Edges& edges() const noexcept { return edges_; }

    // 一致的整帧快照: 所有字段来自同一帧, 不会出现轴与按键不同源的情况.
    [[nodiscard]] std::optional<Dr16Snapshot> snapshot(Tick now) const noexcept {
        if (!online(now)) { return std::nullopt; }
        Dr16Snapshot out{};
        out.raw = sample_;
        out.axis = {drivers::dr16_axis(sample_.ch0), drivers::dr16_axis(sample_.ch1),
            drivers::dr16_axis(sample_.ch2), drivers::dr16_axis(sample_.ch3)};
        out.ch4 = sample_.ch4;
        out.switch_left = sample_.switch_left;
        out.switch_right = sample_.switch_right;
        out.keys = sample_.keys;
        out.mouse_left = sample_.mouse_left;
        out.mouse_right = sample_.mouse_right;
        out.edges = edges_;
        out.sampled_at = last_at_;
        return out;
    }

private:
    static Dr16Edges diff(const drivers::Dr16Sample& previous, const drivers::Dr16Sample& current) noexcept {
        Dr16Edges out{};
        out.keys_pressed = static_cast<std::uint16_t>(current.keys & ~previous.keys);
        out.keys_released = static_cast<std::uint16_t>(~current.keys & previous.keys);
        out.mouse_left_pressed = current.mouse_left && !previous.mouse_left;
        out.mouse_left_released = !current.mouse_left && previous.mouse_left;
        out.mouse_right_pressed = current.mouse_right && !previous.mouse_right;
        out.mouse_right_released = !current.mouse_right && previous.mouse_right;
        return out;
    }

    Lifetime ttl_{};
    drivers::Dr16Sample sample_{};
    Dr16Edges edges_{};
    Tick last_at_{};
    bool has_sample_{};
    std::uint32_t accepted_{};
    std::uint32_t rejected_{};
    std::uint32_t gaps_{};
};
} // 命名空间 ec::input
