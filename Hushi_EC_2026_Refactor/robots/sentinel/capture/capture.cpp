#include "capture.hpp"
#include "ec/drivers/dji.hpp"
#include "ec/drivers/dm.hpp"
#include <bit>
#include <cmath>

namespace ec::sentinel::capture {
namespace {
std::optional<std::size_t> register_index(std::uint8_t rid) noexcept {
    for (std::size_t i = 0; i < registers.size(); ++i) {
        if (registers[i].rid == rid) { return i; }
    }
    return std::nullopt;
}
} // 内部辅助

std::optional<ec::transport::CanFrame> read_request(std::uint8_t rid) noexcept {
    if (!register_index(rid)) { return std::nullopt; }
    return ec::transport::CanFrame{0x7ff, 4,
        {static_cast<std::uint8_t>(dm_can_id), static_cast<std::uint8_t>(dm_can_id >> 8), 0x33, rid}};
}
ec::transport::CanFrame status_request() noexcept {
    return {0x7ff, 4, {static_cast<std::uint8_t>(dm_can_id), static_cast<std::uint8_t>(dm_can_id >> 8), 0xcc, 0}};
}
bool allowed_read(const ec::transport::CanFrame& f) noexcept {
    const auto status = status_request();
    if (f.id == status.id && f.size == status.size && f.data == status.data) { return true; }
    const auto expected = read_request(f.data[3]);
    return expected && f.id == expected->id && f.size == expected->size && f.data == expected->data;
}
void Collector::receive(std::uint8_t bus, const ec::transport::CanFrame& f,
    std::uint32_t ms, std::uint32_t cycles) noexcept {
    if (!f.valid()) { ++diagnostics_.malformed; return; }
    // 参数响应与普通 MIT 状态共用 Master ID. 优先识别参数响应, 不把它解成角度/温度.
    if (bus == dm_bus && f.id == dm_master_id && f.size == 8 &&
        f.data[0] == (dm_can_id & 0xff) && f.data[1] == (dm_can_id >> 8) && f.data[2] == 0x33) {
        const auto index = register_index(f.data[3]);
        if (!index || !pending_ || *index != pending_index_ || ms - requested_at_ >= dm_timeout_ms) {
            ++diagnostics_.unsolicited;
            return;
        }
        auto& value = values_[*index];
        value.bits = static_cast<std::uint32_t>(f.data[4]) |
            (static_cast<std::uint32_t>(f.data[5]) << 8) |
            (static_cast<std::uint32_t>(f.data[6]) << 16) |
            (static_cast<std::uint32_t>(f.data[7]) << 24);
        value.request_ms = requested_at_;
        value.received_ms = ms;
        value.seen = true;
        value.valid = !registers[*index].floating || std::isfinite(std::bit_cast<float>(value.bits));
        ++value.responses;
        pending_ = false;
        return;
    }
    for (std::size_t i = 0; i < installs.size(); ++i) {
        const auto& config = installs[i];
        if (config.bus != bus || config.rx_id != f.id) { continue; }
        auto& m = motors_[i];
        if (m.seen && ms - m.last_ms < 1000) {
            // CYCCNT 只作短区间差值. 1 s 内不可能发生两次 168 MHz 计数回绕.
            const auto gap = cycles - m.cycles;
            if (m.intervals == 0 || gap < m.min_cycles) { m.min_cycles = gap; }
            if (gap > m.max_cycles) { m.max_cycles = gap; }
            ++m.intervals;
            m.sum_cycles += gap;
        }
        m.raw = f; m.last_ms = ms; m.cycles = cycles; m.seen = true; ++m.received;
        if (config.kind == Kind::dm) {
            m.valid = f.size == 8 && (f.data[0] & 0x0f) == config.node &&
                ec::drivers::dm_status_valid(f.data[0] >> 4);
            if (m.valid && pending_ && pending_index_ == 255 && ms - requested_at_ < dm_timeout_ms) {
                pending_ = false;
            }
        } else {
            const auto mode = config.kind == Kind::c620 ? ec::drivers::DjiMode::c620_current :
                config.kind == Kind::c610 ? ec::drivers::DjiMode::c610_current :
                ec::drivers::DjiMode::gm6020_current;
            const auto decoded = ec::drivers::decode_dji(mode, f);
            m.valid = decoded && decoded->device_id == config.node;
        }
        if (!m.valid) { ++m.invalid; }
        return;
    }
    ++diagnostics_.unknown;
    diagnostics_.unknown_bus = bus;
    diagnostics_.unknown_id = f.id;
}
void Collector::poll(ec::transport::CanTx& tx, std::uint32_t now, bool fast_status) noexcept {
    if (fast_status != fast_status_) { pending_ = false; fast_status_ = fast_status; }
    if (fast_status) {
        // 原始旋转短窗期间只询问状态, 500 Hz; 不等待上一回复、不补发积压查询.
        if (attempted_ && now - last_attempt_ < 2) { return; }
        attempted_ = true; last_attempt_ = now;
        switch (tx.try_send(status_request())) {
        case ec::transport::SendResult::accepted: ++diagnostics_.queries; break;
        case ec::transport::SendResult::busy: ++diagnostics_.tx_busy; break;
        default: ++diagnostics_.tx_error; break;
        }
        return;
    }
    if (pending_) {
        if (now - requested_at_ < dm_timeout_ms) { return; }
        ++diagnostics_.timeouts;
        pending_ = false;
    }
    if (attempted_ && now - last_attempt_ < dm_period_ms) { return; }
    attempted_ = true;
    last_attempt_ = now;
    // 两次位置读取 + 一次状态读取 + 一次配置读取. 未响应也不回退到控制帧.
    const std::uint8_t index = phase_ == 0 ? 10 : phase_ == 1 ? 11 : phase_ == 2 ? 255 : config_index_;
    if (phase_ == 3) { config_index_ = static_cast<std::uint8_t>((config_index_ + 1) % 10); }
    phase_ = static_cast<std::uint8_t>((phase_ + 1) % 4);
    const auto request = index == 255 ? std::optional{status_request()} : read_request(registers[index].rid);
    if (!request) { return; }
    switch (tx.try_send(*request)) {
    case ec::transport::SendResult::accepted:
        ++diagnostics_.queries;
        requested_at_ = now; pending_index_ = index; pending_ = true;
        break;
    case ec::transport::SendResult::busy: ++diagnostics_.tx_busy; break;
    default: ++diagnostics_.tx_error; break;
    }
}
void Writer::u8(std::uint8_t v) noexcept {
    if (size_ >= out_.size()) { failed_ = true; return; }
    out_[size_++] = v;
}
void Writer::u16(std::uint16_t v) noexcept { u8(static_cast<std::uint8_t>(v)); u8(static_cast<std::uint8_t>(v >> 8)); }
void Writer::u32(std::uint32_t v) noexcept { u16(static_cast<std::uint16_t>(v)); u16(static_cast<std::uint16_t>(v >> 16)); }
void Writer::u64(std::uint64_t v) noexcept { u32(static_cast<std::uint32_t>(v)); u32(static_cast<std::uint32_t>(v >> 32)); }
void Writer::bytes(std::span<const std::uint8_t> v) noexcept { for (auto b : v) { u8(b); } }
void Writer::header(std::uint8_t page, std::uint8_t count, std::uint32_t now) noexcept {
    u8(schema); u8(page); u8(count); u8(0); u32(now);
}
void encode_motor(Writer& w, const Install& i, const Motor& m) noexcept {
    w.u8(i.bus); w.u8(static_cast<std::uint8_t>(i.kind)); w.u16(i.rx_id);
    w.u8(i.node); w.u8(m.raw.size); w.u16(static_cast<std::uint16_t>((m.seen ? 1 : 0) | (m.valid ? 2 : 0)));
    w.u32(m.received); w.u32(m.invalid); w.u32(m.last_ms); w.u32(m.cycles);
    w.u32(m.min_cycles); w.u32(m.max_cycles); w.u32(m.intervals); w.u64(m.sum_cycles);
    w.bytes(m.raw.data);
}
void encode_register(Writer& w, RegisterSpec spec, const Register& value) noexcept {
    w.u8(spec.rid); w.u8(spec.floating ? 1 : 0); w.u8(value.seen ? 1 : 0); w.u8(value.valid ? 1 : 0);
    w.u32(value.responses); w.u32(value.received_ms); w.u32(value.bits); w.u32(value.request_ms);
}
std::size_t encode_page(const Collector& c, std::uint8_t page, std::uint32_t now,
    std::span<std::uint8_t> out) noexcept {
    Writer w(out);
    if (page >= 1 && page <= 5) {
        w.header(page, 2, now);
        const auto start = static_cast<std::size_t>((page - 1) * 2);
        for (std::size_t i = start; i < start + 2; ++i) { encode_motor(w, installs[i], c.motors()[i]); }
    } else if (page >= 7 && page <= 9) {
        w.header(page, 4, now);
        const auto start = static_cast<std::size_t>((page - 7) * 4);
        for (std::size_t i = start; i < start + 4; ++i) { encode_register(w, registers[i], c.values()[i]); }
    } else { return 0; }
    return w.size();
}
} // 命名空间 ec::sentinel::capture
