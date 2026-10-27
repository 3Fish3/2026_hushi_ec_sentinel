#include "ec/monitor/registry.hpp"
#include <bit>
#include <cstdint>

namespace ec::monitor {
namespace {
void put_u16(std::span<std::uint8_t> out, std::size_t& n, std::uint16_t value) noexcept {
    out[n++] = static_cast<std::uint8_t>(value);
    out[n++] = static_cast<std::uint8_t>(value >> 8);
}
void put_u32(std::span<std::uint8_t> out, std::size_t& n, std::uint32_t value) noexcept {
    put_u16(out, n, static_cast<std::uint16_t>(value));
    put_u16(out, n, static_cast<std::uint16_t>(value >> 16));
}
[[nodiscard]] std::uint16_t get_u16(std::span<const std::uint8_t> in, std::size_t& n) noexcept {
    const auto value = static_cast<std::uint16_t>(in[n] | (in[n + 1] << 8));
    n += 2;
    return value;
}
[[nodiscard]] std::uint32_t get_u32(std::span<const std::uint8_t> in, std::size_t& n) noexcept {
    const auto low = get_u16(in, n);
    return static_cast<std::uint32_t>(low) | (static_cast<std::uint32_t>(get_u16(in, n)) << 16);
}
} // 线协议内部

std::optional<std::uint16_t> Registry::claim(RecordKind kind, std::uint16_t id) noexcept {
    if (kind == RecordKind::unassigned || used_ >= record_capacity) { return std::nullopt; }
    if (find(kind, id) != nullptr) { return std::nullopt; }
    auto& slot = records_[used_];
    slot = {};
    slot.id = id;
    slot.kind = kind;
    slot.validity = Validity::unknown;
    // 注册发生在初始化阶段: 暂存与发布区同时写入, 读者不必等到第一次 end_update.
    published_[used_] = slot;
    used_count_ = static_cast<std::uint32_t>(used_ + 1);
    return static_cast<std::uint16_t>(used_++);
}

Record* Registry::find(RecordKind kind, std::uint16_t id) noexcept {
    for (std::size_t i = 0; i < used_; ++i) {
        if (records_[i].kind == kind && records_[i].id == id) { return &records_[i]; }
    }
    return nullptr;
}

std::span<Record> Registry::records() noexcept { return {records_.data(), used_}; }

void Registry::begin_update() noexcept {
    // 只标记窗口开始; 待发布区间不清零 —— 窗口外(初始化阶段)的写入也要在下一次发布生效.
    updating_ = true;
}

void Registry::end_update(Tick now) noexcept {
    // 发布协议: 代次在复制**前**先变成奇数(发布中), 复制完成后才变成偶数(可读).
    // 读者只接受“偶数且两次读取相同”的代次: 若两次读取都落在复制窗口内, 读到的都是同一个
    // 奇数代次, 因此不会被误认为一致快照 —— 单靠“复制后才 ++generation”做不到这一点.
    //
    // clock_ / used_count_ 与 records_ 同属一份快照, 必须全部落在奇偶括号**内部**:
    // 在括号外先写时钟的话, 读者可能在“偶数代次”下读到新时钟配旧记录, 而两次代次读取仍相同.
    ++generation_; // → 奇数: 发布中, 读者必须重试
    clock_ = now;
    for (std::size_t i = 0; i < used_; ++i) { published_[i] = records_[i]; }
    used_count_ = static_cast<std::uint32_t>(used_);
    updating_ = false;
    ++generation_; // → 偶数: 本次发布已完成
}

void Registry::describe(Descriptor& out, const char* build_id) noexcept {
    out = {};
    // 描述符指向发布视图: SWD 读者看到的是最近一次已提交的一致快照.
    out.records_address = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(published_.data()));
    out.generation_address = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&generation_));
    out.clock_address = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&clock_));
    out.used_address = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&used_count_));
    out.build_id_address = build_id == nullptr
        ? 0u
        : static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(build_id));
    // flags 是描述符里的静态属性(bit0: 监控已启用), 不是发布状态快照;
    // 发布状态由 generation 的奇偶携带, 因为外部读取者读的是 RAM 里的实时值.
    out.flags = 1u;
}

Page Registry::read_page(std::uint16_t first_index) const noexcept {
    Page page{};
    page.header.layout = layout_version;
    page.header.generation = generation_;
    page.header.clock_ms = clock_.ms;
    page.header.flags = 1u; // 静态属性: 本页来自已启用监控的注册表
    page.header.used = static_cast<std::uint16_t>(used_);
    page.first_index = first_index;
    page.total = static_cast<std::uint16_t>(used_);
    if (first_index >= used_) { return page; }
    const auto remaining = used_ - first_index;
    page.count = static_cast<std::uint16_t>(remaining < page_capacity ? remaining : page_capacity);
    for (std::size_t i = 0; i < page.count; ++i) { page.records[i] = published_[first_index + i]; }
    return page;
}

std::size_t encode_page(const Page& page, std::span<std::uint8_t> out) noexcept {
    const auto needed = page_wire_size(page.count);
    if (out.size() < needed || page.count > page_capacity) { return 0; }
    std::size_t n = 0;
    put_u32(out, n, page.header.layout);
    put_u32(out, n, page.header.generation);
    put_u32(out, n, page.header.clock_ms);
    put_u32(out, n, page.header.flags);
    put_u16(out, n, page.header.used);
    put_u16(out, n, page.first_index);
    put_u16(out, n, page.count);
    put_u16(out, n, page.total);
    for (std::size_t i = 0; i < page.count; ++i) {
        const auto& r = page.records[i];
        put_u16(out, n, r.id);
        out[n++] = static_cast<std::uint8_t>(r.kind);
        out[n++] = static_cast<std::uint8_t>(r.validity);
        out[n++] = r.mode;
        out[n++] = r.enabled;
        out[n++] = static_cast<std::uint8_t>(r.reject);
        out[n++] = r.detail;
        put_u32(out, n, r.sampled_at.ms);
        put_u32(out, n, r.published_at.ms);
        put_u32(out, n, std::bit_cast<std::uint32_t>(r.target));
        put_u32(out, n, std::bit_cast<std::uint32_t>(r.measurement));
    }
    return n;
}

std::optional<Page> decode_page(std::span<const std::uint8_t> in) noexcept {
    if (in.size() < 24) { return std::nullopt; }
    std::size_t n = 0;
    Page page{};
    page.header.layout = get_u32(in, n);
    page.header.generation = get_u32(in, n);
    page.header.clock_ms = get_u32(in, n);
    page.header.flags = get_u32(in, n);
    page.header.used = get_u16(in, n);
    page.first_index = get_u16(in, n);
    page.count = get_u16(in, n);
    page.total = get_u16(in, n);
    if (page.header.layout != layout_version || page.count > page_capacity ||
        page.total > record_capacity || page.first_index > page.total) {
        return std::nullopt;
    }
    if (in.size() != page_wire_size(page.count)) { return std::nullopt; }
    for (std::size_t i = 0; i < page.count; ++i) {
        auto& r = page.records[i];
        r.id = get_u16(in, n);
        r.kind = static_cast<RecordKind>(in[n++]);
        r.validity = static_cast<Validity>(in[n++]);
        r.mode = in[n++];
        r.enabled = in[n++];
        r.reject = static_cast<RejectReason>(in[n++]);
        r.detail = in[n++];
        r.sampled_at = Tick{get_u32(in, n)};
        r.published_at = Tick{get_u32(in, n)};
        r.target = std::bit_cast<float>(get_u32(in, n));
        r.measurement = std::bit_cast<float>(get_u32(in, n));
    }
    return page;
}
} // 命名空间 ec::monitor
