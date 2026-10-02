// M7 监控契约: 静态注册表的记录/分页/代次, 显式小端编码, 以及
// "串口线协议" 与 "SWD 直接读 RAM" 得到同一份逻辑值的离线交叉核对素材.
// 生成物仅供 Python 侧只读核对使用, 不代表任何实机 SWD 连机结果.
#include "ec/control/velocity_loop.hpp"
#include "ec/monitor/registry.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3);
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::monitor::Descriptor;
using ec::monitor::Page;
using ec::monitor::Record;
using ec::monitor::RecordKind;
using ec::monitor::Registry;
using ec::monitor::RejectReason;
using ec::monitor::Validity;
using ec::Tick;

constexpr std::uint16_t device_slots = 8;

float nan_value() { return std::numeric_limits<float>::quiet_NaN(); }

void fill(Registry& registry, Tick now) {
    const auto period = registry.find(RecordKind::period, 0);
    CHECK(period != nullptr);
    period->validity = Validity::valid;
    period->sampled_at = Tick{1000};
    period->published_at = now;
    period->target = 10.0f;
    period->measurement = 2.0f;
    period->detail = 3;
    const auto source = registry.find(RecordKind::source, 0);
    CHECK(source != nullptr);
    source->validity = Validity::stale;
    source->enabled = 0;
    source->reject = RejectReason::link_stale;
    source->sampled_at = Tick{900};
    source->published_at = now;
    source->target = nan_value();
    source->measurement = 100.0f;
    const auto device = registry.find(RecordKind::device, 1);
    CHECK(device != nullptr);
    device->validity = Validity::valid;
    device->mode = 2; // control::ControlMode::velocity
    device->enabled = 1;
    device->sampled_at = Tick{995};
    device->published_at = now;
    device->target = 1.25f;
    device->measurement = 1.2f;
}

// 监控只写记录表, 不参与控制: 开关注册表发布时控制轨迹必须逐位一致.
void monitoring_independence_contracts() {
    const auto run = [](bool publish) {
        ec::control::VelocityLoopConfig config{};
        config.pid = {0.5f, 0.2f, 0.01f, 0.5f, -1.0f, 1.0f};
        config.feedback_ttl = ec::Lifetime{20};
        config.max_target_rad_s = 100.0f;
        config.max_dt_s = 0.01f;
        config.feedback_alpha = 0.3f;
        ec::control::VelocityLoop loop{config};
        Registry registry{};
        for (std::uint16_t i = 0; i < device_slots; ++i) {
            (void)registry.claim(RecordKind::device, i);
        }
        (void)registry.claim(RecordKind::period, 0);
        std::vector<float> outputs;
        unsigned active_steps = 0;
        for (int step = 0; step < 200; ++step) {
            const ec::Tick now{static_cast<std::uint32_t>(step)};
            const auto grant = loop.arm(ec::CommandSource::remote, true, true);
            CHECK(grant.has_value());
            const ec::Command<float> target{0.6f, now, ec::Lifetime{50}, *grant};
            const ec::Sample<float> feedback{0.5f * static_cast<float>(step), now, ec::Validity::valid};
            const auto result = loop.step(now, 0.001f, target, feedback);
            if (result.output) { ++active_steps; }
            outputs.push_back(result.output.value_or(0.0f));
            if (publish) {
                // 真实镜像里发布发生在控制步之后, 只写记录表.
                registry.begin_update();
                registry.find(RecordKind::device, 0)->measurement = feedback.value;
                registry.find(RecordKind::period, 0)->sampled_at = now;
                registry.end_update(now);
            }
        }
        return std::pair{outputs, active_steps};
    };
    const auto [without, without_active] = run(false);
    const auto [with, with_active] = run(true);
    // 必须真的跑过有效周期, 否则"两条轨迹一致"只是两次拒绝相同.
    CHECK(without_active > 100 && without_active == with_active);
    CHECK(without.size() == with.size());
    for (std::size_t i = 0; i < without.size(); ++i) { CHECK(without[i] == with[i]); }
}

void registry_contracts() {
    Registry registry{};
    CHECK(registry.claim(RecordKind::period, 0).has_value());
    CHECK(registry.claim(RecordKind::source, 0).has_value());
    for (std::uint16_t i = 0; i < device_slots; ++i) {
        CHECK(registry.claim(RecordKind::device, i).has_value());
    }
    // 重复认领同一 (kind, id) 与未知 kind 都拒绝, 不做动态扩容.
    CHECK(!registry.claim(RecordKind::period, 0).has_value());
    CHECK(!registry.claim(RecordKind::unassigned, 1).has_value());
    CHECK(registry.claim(RecordKind::environment, 0).has_value());
    CHECK(registry.used() == device_slots + 3);

    fill(registry, Tick{1000});
    const auto before = registry.generation();
    registry.begin_update();
    // 更新窗口内读者仍看到旧代次: 这是识别撕裂读取的依据.
    CHECK(registry.read_page(0).header.generation == before);
    registry.end_update(Tick{1010});
    // 发布协议: 复制前 +1 (奇数 = 发布中), 复制后 +1 (偶数 = 可读).
    CHECK(registry.generation() == before + 2);
    CHECK(registry.generation() % 2 == 0); // 发布完成后一定是偶数

    // 分页: 只包含本页记录, 每条记录保留自己的采样时刻.
    const auto page = registry.read_page(0);
    CHECK(page.count == ec::monitor::page_capacity);
    CHECK(page.total == registry.used());
    CHECK(page.header.generation == registry.generation());
    CHECK(page.header.clock_ms == 1010);
    const auto tail = registry.read_page(ec::monitor::page_capacity);
    CHECK(tail.first_index == ec::monitor::page_capacity);
    CHECK(tail.count == registry.used() - ec::monitor::page_capacity);
    // 越界页返回空页而不是伪造数据.
    const auto empty = registry.read_page(static_cast<std::uint16_t>(registry.used()));
    CHECK(empty.count == 0 && empty.total == registry.used());
    // 未实现的来源必须是 unknown/unavailable, 字段用 NaN 而不是健康零值.
    CHECK(registry.find(RecordKind::environment, 0)->validity == Validity::unknown);
    CHECK(std::isnan(registry.read_page(0).records[1].target));
}

void wire_contracts() {
    Registry registry{};
    (void)registry.claim(RecordKind::period, 0);
    (void)registry.claim(RecordKind::source, 0);
    for (std::uint16_t i = 0; i < device_slots; ++i) {
        (void)registry.claim(RecordKind::device, i);
    }
    (void)registry.claim(RecordKind::environment, 0);
    fill(registry, Tick{1000});
    registry.end_update(Tick{1000});

    Page page = registry.read_page(0);
    std::array<std::uint8_t, 256> bytes{};
    const auto size = ec::monitor::encode_page(page, bytes);
    CHECK(size == ec::monitor::page_wire_size(page.count));
    CHECK(size == 24 + 8 * 24);
    // 显式小端: 头部第 0..3 字节是 layout 版本.
    CHECK(bytes[0] == ec::monitor::layout_version && bytes[1] == 0 && bytes[2] == 0 && bytes[3] == 0);
    const auto decoded = ec::monitor::decode_page({bytes.data(), size});
    CHECK(decoded.has_value());
    CHECK(decoded->count == page.count && decoded->header.generation == page.header.generation);
    CHECK(decoded->header.clock_ms == page.header.clock_ms);
    for (std::size_t i = 0; i < page.count; ++i) {
        const auto& a = page.records[i];
        const auto& b = decoded->records[i];
        CHECK(a.id == b.id && a.kind == b.kind && a.validity == b.validity && a.mode == b.mode &&
            a.enabled == b.enabled && a.reject == b.reject && a.detail == b.detail);
        CHECK(a.sampled_at.ms == b.sampled_at.ms && a.published_at.ms == b.published_at.ms);
        CHECK(std::bit_cast<std::uint32_t>(a.target) == std::bit_cast<std::uint32_t>(b.target));
        CHECK(a.measurement == b.measurement);
    }
    // 长度不符/版本不符必须拒绝, 不能"凑合解码".
    CHECK(!ec::monitor::decode_page({bytes.data(), size - 1}).has_value());
    auto bad = bytes;
    bad[0] = static_cast<std::uint8_t>(ec::monitor::layout_version + 1);
    CHECK(!ec::monitor::decode_page({bad.data(), size}).has_value());
    std::array<std::uint8_t, 16> small{};
    CHECK(ec::monitor::encode_page(page, small) == 0);
}

// R10: 更新窗口内的部分写入对读者不可见; 代次只在发布之后推进; flags 每次重新计算.
void publish_transaction_contracts() {
    Registry registry{};
    (void)registry.claim(RecordKind::period, 0);
    (void)registry.claim(RecordKind::source, 0);
    for (std::uint16_t i = 0; i < device_slots; ++i) {
        (void)registry.claim(RecordKind::device, i);
    }
    fill(registry, Tick{1000});
    registry.end_update(Tick{1000});
    const auto published_before = registry.read_page(0);
    const auto generation_before = registry.generation();

    registry.begin_update();
    CHECK(registry.updating());
    // 窗口内写入两条记录: 读者仍看到上一份发布值.
    registry.find(RecordKind::device, 0)->measurement = 42.5f;
    registry.find(RecordKind::period, 0)->sampled_at = Tick{1010};
    // 槽位顺序: 0 = period, 1 = source, 2.. = device; 本窗口改的是 period(0) 与 device(0)=2.
    const auto during = registry.read_page(0);
    CHECK(during.header.generation == generation_before);
    CHECK(during.records[2].measurement == published_before.records[2].measurement);
    CHECK(during.records[0].sampled_at.ms == published_before.records[0].sampled_at.ms);
    // flags 是“监控已启用”的静态属性, 不是发布状态; 发布状态由 generation 奇偶携带.
    CHECK(during.header.flags == 1);
    Descriptor descriptor{};
    registry.describe(descriptor, nullptr);
    CHECK(descriptor.flags == 1);
    registry.end_update(Tick{1010});
    const auto after = registry.read_page(0);
    CHECK(after.header.generation == generation_before + 2);
    CHECK(after.header.clock_ms == 1010);
    CHECK(after.records[2].measurement == 42.5f);
    CHECK(after.records[0].sampled_at.ms == 1010);
    CHECK(after.header.flags == 1);
    registry.describe(descriptor, nullptr);
    CHECK(descriptor.flags == 1);
    // 窗口内未改动的记录保持上一份发布值.
    CHECK(after.records[3].measurement == published_before.records[3].measurement);
    // 写者视角与读者视角是两个视图.
    CHECK(registry.records().data() != registry.published_records().data());
    CHECK(!registry.updating());
}

// V03: 发布代次必须让“两次读取都落在复制窗口内”的读者也能识别出不一致.
// 这里只能验证写者侧的不变量(偶数代次、每次发布 +2); 复制窗口内部的交错由 SWD 工具
// 对奇数代次的拒绝测试(monitor_swd_selftest)与外部插桩反例覆盖.
void publish_protocol_contracts() {
    Registry registry{};
    (void)registry.claim(RecordKind::period, 0);
    (void)registry.claim(RecordKind::source, 0);
    for (std::uint16_t i = 0; i < device_slots; ++i) {
        (void)registry.claim(RecordKind::device, i); // fill() 需要 device id=1
    }
    fill(registry, Tick{1000});
    registry.end_update(Tick{1000});
    CHECK(registry.generation() % 2 == 0);
    // 连续多次发布都保持“偶数 = 可读”的不变量, 且每次都前进两步.
    std::uint32_t previous = registry.generation();
    for (int round = 0; round < 3; ++round) {
        registry.begin_update();
        registry.end_update(Tick{static_cast<std::uint32_t>(1001 + round)});
        CHECK(registry.generation() == previous + 2);
        CHECK(registry.generation() % 2 == 0);
        previous = registry.generation();
    }
    // 读前/读后相同且为偶数 ⇒ 这份快照可接受.
    const auto first = registry.generation();
    const auto page = registry.read_page(0);
    const auto second = registry.generation();
    CHECK(first == second && second % 2 == 0 && page.header.generation == second);
}

// V04: 任何公开写入入口都必须能发布 —— 不依赖“哪些槽被 find 过”.
void writable_publish_contracts() {
    Registry registry{};
    (void)registry.claim(RecordKind::axis, 1);
    (void)registry.claim(RecordKind::period, 0);
    registry.end_update(Tick{0});
    // (a) 通过可变 span 写入也必须发布.
    registry.begin_update();
    registry.records()[0].measurement = 42.0f;
    registry.end_update(Tick{1});
    CHECK(registry.read_page(0).records[0].measurement == 42.0f);
    // (b) 跨窗口沿用 find() 拿到的裸指针写入也必须发布.
    auto* cached = registry.find(RecordKind::axis, 1);
    CHECK(cached != nullptr);
    registry.begin_update();
    cached->measurement = 10.0f;
    registry.end_update(Tick{2});
    CHECK(registry.read_page(0).records[0].measurement == 10.0f);
    registry.begin_update();
    cached->measurement = 20.0f;
    registry.end_update(Tick{3});
    CHECK(registry.read_page(0).records[0].measurement == 20.0f);
    // (c) 空更新窗口不改变记录值, 只推进代次.
    const auto generation_before = registry.generation();
    registry.begin_update();
    registry.end_update(Tick{4});
    CHECK(registry.generation() == generation_before + 2);
    CHECK(registry.read_page(0).records[0].measurement == 20.0f);
    // (d) 连续多个窗口: 每条记录都跟上最后一次写入.
    for (int round = 0; round < 3; ++round) {
        registry.begin_update();
        registry.find(RecordKind::axis, 1)->measurement = static_cast<float>(100 + round);
        registry.end_update(Tick{static_cast<std::uint32_t>(10 + round)});
        CHECK(registry.read_page(0).records[0].measurement == static_cast<float>(100 + round));
    }
}

void descriptor_contracts() {
    Registry registry{};
    Descriptor descriptor{};
    const char build_id[] = "framework-offline";
    registry.describe(descriptor, build_id);
    CHECK(descriptor.magic == 0x4d4f4e31u);
    CHECK(descriptor.layout == ec::monitor::layout_version);
    CHECK(descriptor.record_size == ec::monitor::record_wire_size);
    CHECK(descriptor.capacity == ec::monitor::record_capacity);
    // 描述符必须指向发布视图: SWD 读者看到的是最近一次已提交的一致快照.
    CHECK(descriptor.records_address == static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(registry.published_records().data())));
    CHECK(descriptor.records_address != static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(registry.records().data())));
    CHECK(descriptor.generation_address != 0 && descriptor.clock_address != 0);
    CHECK(descriptor.used_address != 0);
    CHECK(descriptor.build_id_address ==
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(build_id)));
}

// 生成 Python 侧离线核对素材: ram.bin (假 SWD 内存映像) + page.bin (串口载荷) + image.json.
void write_artifacts(const std::filesystem::path& dir) {
    Registry registry{};
    (void)registry.claim(RecordKind::period, 0);
    (void)registry.claim(RecordKind::source, 0);
    for (std::uint16_t i = 0; i < device_slots; ++i) {
        (void)registry.claim(RecordKind::device, i);
    }
    (void)registry.claim(RecordKind::environment, 0);
    fill(registry, Tick{1000});
    registry.end_update(Tick{1000});
    Page page = registry.read_page(0);
    page.header.clock_ms = 1000;

    // 假内存映像: 描述符在 base, 记录表在 base+0x100, 代次/时钟/构建标识随后.
    constexpr std::uint32_t base = 0x20000000u;
    constexpr std::uint32_t descriptor_at = base;
    constexpr std::uint32_t records_at = base + 0x100;
    constexpr std::uint32_t generation_at = base + 0x400;
    constexpr std::uint32_t clock_at = base + 0x404;
    constexpr std::uint32_t used_at = base + 0x408;
    constexpr std::uint32_t build_id_at = base + 0x500;
    const char build_id[] = "framework-offline";
    std::vector<std::uint8_t> ram(0x540, 0);
    Descriptor descriptor{};
    registry.describe(descriptor, build_id);
    descriptor.records_address = records_at;
    descriptor.generation_address = generation_at;
    descriptor.clock_address = clock_at;
    descriptor.used_address = used_at;
    descriptor.build_id_address = build_id_at;
    std::memcpy(ram.data() + (descriptor_at - base), &descriptor, sizeof(descriptor));
    const auto records = registry.published_records();
    std::memcpy(ram.data() + (records_at - base), records.data(), records.size_bytes());
    const auto generation = registry.generation();
    std::memcpy(ram.data() + (generation_at - base), &generation, sizeof(generation));
    const auto clock = page.header.clock_ms;
    std::memcpy(ram.data() + (clock_at - base), &clock, sizeof(clock));
    const auto used = static_cast<std::uint32_t>(records.size());
    std::memcpy(ram.data() + (used_at - base), &used, sizeof(used));
    std::memcpy(ram.data() + (build_id_at - base), build_id, sizeof(build_id));

    std::array<std::uint8_t, 256> wire{};
    const auto wire_size = ec::monitor::encode_page(page, wire);

    std::filesystem::create_directories(dir);
    {
        std::ofstream out(dir / "ram.bin", std::ios::binary);
        out.write(reinterpret_cast<const char*>(ram.data()), static_cast<std::streamsize>(ram.size()));
    }
    {
        std::ofstream out(dir / "page.bin", std::ios::binary);
        out.write(reinterpret_cast<const char*>(wire.data()), static_cast<std::streamsize>(wire_size));
    }
    {
        std::ofstream out(dir / "image.json");
        out << "{\n"
            << "  \"base\": " << base << ",\n"
            << "  \"descriptor_address\": " << descriptor_at << ",\n"
            << "  \"records_address\": " << records_at << ",\n"
            << "  \"generation_address\": " << generation_at << ",\n"
            << "  \"clock_address\": " << clock_at << ",\n"
            << "  \"build_id_address\": " << build_id_at << ",\n"
            << "  \"build_id\": \"" << build_id << "\",\n"
            << "  \"layout\": " << ec::monitor::layout_version << ",\n"
            << "  \"record_size\": " << ec::monitor::record_wire_size << ",\n"
            << "  \"capacity\": " << ec::monitor::record_capacity << ",\n"
            << "  \"records_used\": " << records.size() << ",\n"
            << "  \"generation\": " << generation << ",\n"
            << "  \"clock_ms\": " << clock << ",\n"
            << "  \"page_wire_size\": " << wire_size << "\n"
            << "}\n";
    }
}
} // 测试辅助

int main(int argc, char** argv) {
    registry_contracts();
    wire_contracts();
    descriptor_contracts();
    publish_transaction_contracts();
    publish_protocol_contracts();
    writable_publish_contracts();
    monitoring_independence_contracts();
    if (argc > 1) { write_artifacts(argv[1]); }
    std::puts("monitor registry contracts passed");
    return 0;
}
