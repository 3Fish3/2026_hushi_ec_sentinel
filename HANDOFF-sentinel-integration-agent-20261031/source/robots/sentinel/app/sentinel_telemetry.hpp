#pragma once
// 协议与板级只读快照共用的逐字段遥测编码. 只取应用单写者已经发布的事实, 不采样硬件.
#include "sentinel_app.hpp"
#include "sentinel_perception.hpp"
#include <cstdint>
#include <span>

namespace ec::sentinel::app {
struct DomainMeasurement {
    float value{};
    std::uint32_t position_age_ms{0xffffffffu}, velocity_age_ms{0xffffffffu};
    // bit0..1 请求参考, bit2..3 执行参考, bit4..5 测量参考, bit6 有请求, bit7 测量有效.
    std::uint8_t frames{};
};
[[nodiscard]] DomainMeasurement domain_measurement(const SentinelApp& app, Domain domain, Tick now) noexcept;
// 响应总长 214 B, 含 v3/0xc6/session/seq/page=0. USART6 只读观测直接复制相同载荷,
// 不维护另一份含义近似的状态结构. session/seq 只是响应关联, 不是传感器样本身份.
inline constexpr std::size_t fast_status_size = 214;
[[nodiscard]] std::size_t encode_fast_status(const SentinelApp& app, const SentinelPerception* perception,
    Tick now, std::uint32_t session, std::uint32_t sequence, std::uint32_t boot_id,
    std::span<std::uint8_t> out) noexcept;
} // 命名空间 ec::sentinel::app
