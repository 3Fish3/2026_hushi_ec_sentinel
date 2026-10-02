#pragma once
#include <cstdint>

namespace ec {
// 本地单调毫秒时钟, 按 2^32 回绕; 不是日历时间或线协议时间戳.
// 必须在半个回绕周期内 (~24.8 天) 读取或使数据失效, 才能无歧义比较.
struct Tick { std::uint32_t ms{}; };
struct Lifetime {
    std::uint32_t ms{};
    [[nodiscard]] constexpr bool valid() const noexcept {
        return ms > 0 && ms < 0x80000000u;
    }
};
[[nodiscard]] constexpr bool fresh(Tick now, Tick received, Lifetime ttl) noexcept {
    // 到达截止时刻即过期; 半个回绕周期内的未来时间戳也视为无效.
    return ttl.valid() && (now.ms - received.ms) < ttl.ms;
}

// 半回绕窗口内的先后判定: 时间戳会按 2^32 回绕, 因此不能用 `<` 直接比较毫秒值.
// 无符号差值落在半个回绕周期内表示 later 在 earlier 之后; 超过则视为倒退或时间歧义.
[[nodiscard]] constexpr bool tick_before(Tick later, Tick earlier) noexcept {
    return (later.ms - earlier.ms) != 0u && (later.ms - earlier.ms) < 0x80000000u;
}

// 时间戳倒退或处于半回绕歧义区时为真; 相同时间戳不算倒退 (同一毫秒内的多帧按接口约定处理).
[[nodiscard]] constexpr bool tick_regressed(Tick now, Tick before) noexcept {
    const std::uint32_t gap = now.ms - before.ms;
    return gap != 0u && gap >= 0x80000000u;
}

// 采样/控制用微秒时基, 与协议毫秒 Tick 不可隐式互换.
// 比较范围小于半个 32 位回绕周期 (~35.8 min), 长期日志按流序号扩展.
struct MicroTick { std::uint32_t us{}; };
struct MicroLifetime {
    std::uint32_t us{};
    [[nodiscard]] constexpr bool valid() const noexcept {
        return us > 0 && us < 0x80000000u;
    }
};
[[nodiscard]] constexpr bool micro_fresh(MicroTick now, MicroTick sampled, MicroLifetime ttl) noexcept {
    return ttl.valid() && now.us - sampled.us < ttl.us;
}
[[nodiscard]] constexpr bool micro_regressed(MicroTick now, MicroTick previous) noexcept {
    return now.us - previous.us >= 0x80000000u;
}

enum class Validity : std::uint8_t { unknown, valid, invalid };
template<class T> struct Sample {
    T value{};
    Tick sampled_at{};
    Validity validity{Validity::unknown};
    [[nodiscard]] bool fresh_at(Tick now, Lifetime max_age) const noexcept {
        return validity == Validity::valid && fresh(now, sampled_at, max_age);
    }
};
} // 命名空间 ec
