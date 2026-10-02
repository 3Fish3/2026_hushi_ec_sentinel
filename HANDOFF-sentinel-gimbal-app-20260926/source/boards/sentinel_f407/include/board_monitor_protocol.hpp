#pragma once
#include "board_io.h"
#include "s1_diagnostics.h"
#include <array>
#include <bit>
#include <cstdint>
#include <span>

namespace ec::sentinel {
inline constexpr std::uint8_t monitor_request = 0x01, monitor_response = 0x81;
// 注册表分页: 请求载荷是 2 字节小端起始序号, 响应载荷是显式小端编码的一页记录.
inline constexpr std::uint8_t registry_page_request = 0x02, registry_page_response = 0x82;
// 板级观测布局版本: 载荷长度就是判别依据, 客户端必须先核对长度再解析.
// v2 = 在 remote 段 ch3 之后增加 ch4.
inline constexpr std::uint32_t monitor_layout_version = 2;
inline constexpr std::size_t monitor_payload_size = 252;
// 不是整车上位机控制协议. 逐字段小端编码, 不传输 C 结构体布局/指针.
// 调用者须冻结共享字段, 此函数只复制/编码, 不调用 HAL/RTOS 或分配内存.
inline std::size_t encode_monitor_snapshot(const volatile EcS1Diagnostics& s,
    const volatile EcBoardIo& io, std::uint32_t now, const std::array<std::uint32_t, 14>& link,
    std::span<std::uint8_t> out) noexcept {
    if (out.size() < monitor_payload_size) { return 0; }
    std::size_t n = 0;
    const auto u8 = [&](std::uint8_t v) { out[n++] = v; };
    const auto u16 = [&](std::uint16_t v) { u8(static_cast<std::uint8_t>(v)); u8(static_cast<std::uint8_t>(v >> 8)); };
    const auto u32 = [&](std::uint32_t v) { u16(static_cast<std::uint16_t>(v)); u16(static_cast<std::uint16_t>(v >> 16)); };
    const auto f32 = [&](float v) { u32(std::bit_cast<std::uint32_t>(v)); };
    u32(now); u32(s.stage); u32(s.fault); u32(s.reset_flags); u32(s.cycles);
    u32(s.max_elapsed_ms); u32(s.skipped_periods); u32(s.stack_free_words);
    u32(s.heap_free_bytes); u32(s.heap_min_free_bytes);
    const char* label = s.build_id;
    bool ended = label == nullptr;
    for (std::size_t i = 0; i < 32; ++i) {
        const auto c = ended ? 0 : static_cast<std::uint8_t>(label[i]);
        u8(c); ended = ended || c == 0;
    }
    const auto& r = io.remote;
    u32(r.frames); u32(r.rejected); u32(r.last_ms);
    u16(static_cast<std::uint16_t>(r.ch0)); u16(static_cast<std::uint16_t>(r.ch1));
    u16(static_cast<std::uint16_t>(r.ch2)); u16(static_cast<std::uint16_t>(r.ch3));
    u16(static_cast<std::uint16_t>(r.ch4));
    u8(r.switch_left); u8(r.switch_right); u8(r.online);
    const auto& i = io.imu;
    u32(i.samples); u32(i.errors); u32(i.last_ms);
    u8(i.acc_id); u8(i.gyro_id); u8(i.ready); u8(i.heater_enabled);
    f32(i.ax); f32(i.ay); f32(i.az); f32(i.gx); f32(i.gy); f32(i.gz); f32(i.temp_c);
    const auto& c = io.can;
    u32(c.rx_frames); u32(c.rx_dropped); u32(c.ignored); u32(c.last_id); u32(c.last_dlc);
    u32(c.last_ms); u32(c.last_bus); u32(c.online_count);
    f32(c.last_speed_rad_s); u8(c.last_device_id); u8(c.last_temp_c); u8(c.last_temp_valid);
    u32(io.usart3.framing); u32(io.usart3.noise); u32(io.usart3.overrun); u32(io.usart3.overflows);
    for (auto v : link) { u32(v); }
    return n;
}
} // 命名空间 ec::sentinel
