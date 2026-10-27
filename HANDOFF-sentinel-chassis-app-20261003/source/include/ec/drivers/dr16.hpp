#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace ec::drivers {
enum class Dr16Switch : std::uint8_t { unknown = 0, up = 1, down = 2, mid = 3 };

struct Dr16Sample {
    std::int16_t ch0{}; // 右摇杆 X, -660..660
    std::int16_t ch1{}; // 右摇杆 Y
    std::int16_t ch2{}; // 左摇杆 X
    std::int16_t ch3{}; // 左摇杆 Y
    // 第 5 通道: byte16..17 小端, 与官方 C 板示例一致减 1024 (示例本身标注 //NULL,
    // 即该字段含义未被文档化). 因此不套用 ch0–ch3 的 ±660 归一化, 只提供原始值;
    // 有效量程/中位/方向由上层配置显式给出, 实测前不当作可信操纵量.
    std::int16_t ch4{};
    Dr16Switch switch_left{Dr16Switch::unknown};
    Dr16Switch switch_right{Dr16Switch::unknown};
    std::int16_t mouse_x{};
    std::int16_t mouse_y{};
    std::int16_t mouse_z{};
    bool mouse_left{};
    bool mouse_right{};
    std::uint16_t keys{};
};

// DR16 18 字节固定帧, 与现有运行解码字段一致. 通道超范围或拨杆取值非法则拒绝.
// 不在此层做掉线超时; 调用方用 Sample 时间戳与 Lifetime 判断陈旧.
[[nodiscard]] inline std::optional<Dr16Sample> decode_dr16(
    std::span<const std::uint8_t, 18> buf) noexcept {
    const auto ch = [](unsigned value) {
        return static_cast<std::int16_t>(static_cast<int>(value) - 1024);
    };
    Dr16Sample out{};
    out.ch0 = ch((buf[0] | (static_cast<unsigned>(buf[1]) << 8)) & 0x07ffu);
    out.ch1 = ch(((buf[1] >> 3) | (static_cast<unsigned>(buf[2]) << 5)) & 0x07ffu);
    out.ch2 = ch(((buf[2] >> 6) | (static_cast<unsigned>(buf[3]) << 2) |
                  (static_cast<unsigned>(buf[4]) << 10)) & 0x07ffu);
    out.ch3 = ch(((buf[4] >> 1) | (static_cast<unsigned>(buf[5]) << 7)) & 0x07ffu);
    const unsigned switches = buf[5] >> 4;
    // 位域与官方示例注释的左右命名相反 (官方: bit5:4=left, bit7:6=right);
    // 已运行的接管操作不因示例注释翻转, 物理对应留实测.
    out.ch4 = static_cast<std::int16_t>(
        static_cast<std::uint16_t>(buf[16] | (static_cast<unsigned>(buf[17]) << 8)) - 1024u);
    out.switch_left = static_cast<Dr16Switch>((switches & 0x0cu) >> 2);
    out.switch_right = static_cast<Dr16Switch>(switches & 0x03u);
    const auto i16 = [](std::uint8_t lo, std::uint8_t hi) {
        const auto raw = static_cast<std::uint16_t>(lo | (static_cast<unsigned>(hi) << 8));
        return static_cast<std::int16_t>(raw);
    };
    out.mouse_x = i16(buf[6], buf[7]);
    out.mouse_y = i16(buf[8], buf[9]);
    out.mouse_z = i16(buf[10], buf[11]);
    if (buf[12] > 1 || buf[13] > 1) { return std::nullopt; }
    out.mouse_left = buf[12] != 0;
    out.mouse_right = buf[13] != 0;
    out.keys = static_cast<std::uint16_t>(buf[14] | (static_cast<unsigned>(buf[15]) << 8));
    const auto in_range = [](std::int16_t v) { return v >= -660 && v <= 660; };
    const auto switch_ok = [](Dr16Switch s) {
        return s == Dr16Switch::up || s == Dr16Switch::mid || s == Dr16Switch::down;
    };
    if (!in_range(out.ch0) || !in_range(out.ch1) || !in_range(out.ch2) || !in_range(out.ch3) ||
        !switch_ok(out.switch_left) || !switch_ok(out.switch_right)) {
        return std::nullopt;
    }
    return out;
}

[[nodiscard]] constexpr float dr16_axis(std::int16_t raw) noexcept {
    return static_cast<float>(raw) / 660.0f;
}

// DR16 专用分帧: DMA 块可以拆分帧, 仅真实 UART IDLE 结束一帧.
// 启动/丢字节后先等待一个 IDLE 建立边界; 不用字段碰巧合法来猜测帧头.
struct Dr16Stream {
    std::array<std::uint8_t, 18> pending{};
    std::size_t filled{0};
    std::uint32_t accepted{};
    std::uint32_t rejected{};
    bool synchronized{false};
    bool oversized{false};

    void push(std::span<const std::uint8_t> bytes) noexcept {
        for (auto byte : bytes) {
            if (filled == pending.size()) { oversized = true; }
            else { pending[filled++] = byte; }
        }
    }

    // 丢弃错误前的半帧, 保留累计诊断; 后续首个 IDLE 只用于重新同步.
    void reset() noexcept {
        if (filled != 0 || oversized) { ++rejected; }
        filled = 0;
        oversized = false;
        synchronized = false;
    }

    std::optional<Dr16Sample> on_idle() noexcept {
        std::optional<Dr16Sample> sample{};
        if (synchronized && !oversized && filled == pending.size()) {
            sample = decode_dr16(pending);
        }
        if (sample) { ++accepted; }
        else if (filled != 0 || oversized) { ++rejected; }
        filled = 0;
        oversized = false;
        synchronized = true;
        return sample;
    }
};
} // 命名空间 ec::drivers
