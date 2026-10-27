#pragma once
#include "gm6020_bench.hpp"
#include <bit>
#include <span>

namespace ec::sentinel {
// S5/v6: 显式选择原生RPM或编码器5点测速, 并报告估计值/中心年龄/有效位.
inline constexpr std::uint32_t bench_magic=0x53354236;
inline constexpr std::size_t bench_status_size=126, bench_settings_size=42;
namespace bench_wire {
inline void u16(std::span<std::uint8_t> out,std::size_t at,std::uint16_t value) noexcept {
    out[at]=static_cast<std::uint8_t>(value); out[at+1]=static_cast<std::uint8_t>(value>>8);
}
inline void u32(std::span<std::uint8_t> out,std::size_t at,std::uint32_t value) noexcept {
    u16(out,at,static_cast<std::uint16_t>(value)); u16(out,at+2,static_cast<std::uint16_t>(value>>16));
}
inline void f32(std::span<std::uint8_t> out,std::size_t at,float value) noexcept { u32(out,at,std::bit_cast<std::uint32_t>(value)); }
inline std::uint16_t get16(std::span<const std::uint8_t> in,std::size_t at) noexcept {
    return static_cast<std::uint16_t>(in[at]|(static_cast<std::uint16_t>(in[at+1])<<8));
}
inline std::uint32_t get32(std::span<const std::uint8_t> in,std::size_t at) noexcept {
    return get16(in,at)|(static_cast<std::uint32_t>(get16(in,at+2))<<16);
}
inline float get_float(std::span<const std::uint8_t> in,std::size_t at) noexcept { return std::bit_cast<float>(get32(in,at)); }
}
inline std::optional<BenchSettings> decode_bench_settings(std::span<const std::uint8_t> bytes) noexcept {
    if(bytes.size()!=bench_settings_size || bytes[41]>1){return std::nullopt;}
    using bench_wire::get_float;
    BenchSettings cfg{static_cast<BenchMode>(bytes[0]),get_float(bytes,1),get_float(bytes,5),get_float(bytes,9),
        get_float(bytes,13),get_float(bytes,17),get_float(bytes,21),get_float(bytes,25),get_float(bytes,29),get_float(bytes,33),get_float(bytes,37),bytes[41]!=0};
    return Gm6020Bench::valid(cfg)?std::optional{cfg}:std::nullopt;
}
// 0x27: token(u16), kind(u8), bias/amplitude/frequency(3*f32), duration_us(u32).
inline bool apply_bench_excitation(Gm6020Bench& bench,std::span<const std::uint8_t> bytes,
    ec::Tick now,ec::MicroTick now_us) noexcept {
    if (bytes.size()!=19) { bench.invalid_request(); return false; }
    using namespace bench_wire;
    return bench.excite(get16(bytes,0), {static_cast<BenchExcitation::Kind>(bytes[2]),
        get_float(bytes,3),get_float(bytes,7),get_float(bytes,11),get32(bytes,15)},now,now_us);
}
// 显式小端字段编码, 不传目标 ABI/结构体填充. 电流反馈保持厂商 raw 单位.
inline std::size_t encode_bench_status(const Gm6020Bench& bench,ec::Tick now,
    std::uint8_t tx_status,std::uint32_t tx_errors,std::span<std::uint8_t> out,
    std::uint32_t signature=bench_magic) noexcept {
    if(out.size()<bench_status_size){return 0;}
    std::size_t n=0;
    const auto b=[&](std::uint8_t v){out[n++]=v;};
    const auto h=[&](std::uint16_t v){bench_wire::u16(out,n,v);n+=2;};
    const auto w=[&](std::uint32_t v){bench_wire::u32(out,n,v);n+=4;};
    const auto f=[&](float v){bench_wire::f32(out,n,v);n+=4;};
    const auto s=bench.device().state(now);const auto& c=bench.settings();
    const auto& device=bench.device().config();
    const auto protocol=*ec::drivers::dji_protocol(device.mode);
    w(signature);w(now.ms);b(bench.active());b(static_cast<std::uint8_t>(c.mode));
    b(static_cast<std::uint8_t>(bench.reason()));b(static_cast<std::uint8_t>(bench.request_result()));h(bench.token());
    h((s.velocity_rad_s.fresh_at(now,{50})?1:0)|(s.position_rad.fresh_at(now,{50})?2:0));w(s.rotor.sampled_at.ms);
    for(float v:{bench.applied_target(),bench.current(),s.rotor.value.rotor_angle_rad,s.position_rad.value,s.velocity_rad_s.value,
                 c.current_limit,c.speed_limit,c.velocity_kp,c.velocity_ki,c.position_kp,c.duration_s}){f(v);}
    h(static_cast<std::uint16_t>(s.rotor.value.effort_raw));h(bench.encoder());
    b(s.rotor.value.temperature_c.value_or(0));b(tx_status);w(tx_errors);
    b(device.bus);b(device.device_id);h(protocol.rx_base+device.device_id);
    h(bench.group_number()==1 ? protocol.first_tx : protocol.second_tx);b(static_cast<std::uint8_t>(bench.slot()));b(0);
    w(bench.control_us().us);w(bench.feedback_us().us);w(bench.feedback_sequence());w(bench.reference_epoch());w(bench.settings_epoch());
    f(c.velocity_kd);f(c.feedback_alpha);f(c.target);f(c.derivative_tau_s);
    const auto estimate=bench.estimated_velocity();b(c.encoder_velocity);f(estimate.value);
    const auto age=now.ms-estimate.sampled_at.ms;
    h(static_cast<std::uint16_t>(age>65535?65535:age));b(estimate.fresh_at(now,Gm6020Bench::feedback_ttl));
    return n;
}
} // 命名空间 ec::sentinel
