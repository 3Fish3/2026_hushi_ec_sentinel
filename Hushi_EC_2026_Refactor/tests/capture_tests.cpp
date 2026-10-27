// 首轮采集的正式 C++ 接收/查询链测试. 不连接真实电调或替代固件行为.
#include "capture.hpp"
#include "raw_record.hpp"
#include <fstream>
#include <bit>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>
namespace {
namespace cap = ec::sentinel::capture;
void check(bool ok, int line) { if (!ok) { std::fprintf(stderr, "capture check failed at %d\n", line); std::exit(1); } }
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), __LINE__)
struct Tx final : ec::transport::CanTx {
    std::vector<ec::transport::CanFrame> frames;
    ec::transport::SendResult result{ec::transport::SendResult::accepted};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& f) noexcept override {
        CHECK(cap::allowed_read(f)); frames.push_back(f); return result;
    }
};
ec::transport::CanFrame response(std::uint8_t rid, std::uint32_t bits) {
    return {cap::dm_master_id, 8, {1, 0, 0x33, rid, static_cast<std::uint8_t>(bits), static_cast<std::uint8_t>(bits >> 8),
        static_cast<std::uint8_t>(bits >> 16), static_cast<std::uint8_t>(bits >> 24)}};
}
void golden() {
    cap::Collector c;
    c.receive(1, {0x201, 8, {0x1f, 0xff, 0xff, 0x9c, 0x12, 0x34, 42, 0}}, 100, 1000);
    c.receive(1, {0x201, 8, {0, 1, 0, 120, 0xff, 0xfe, 43, 0}}, 101, 169000);
    c.receive(2, {0x207, 8, {0, 123, 0, 5, 0, 0, 255, 0}}, 101, 169000);
    Tx tx; c.poll(tx, 100);
    c.receive(cap::dm_bus, response(0x50, std::bit_cast<std::uint32_t>(1.25f)), 104, 0);
    c.poll(tx, 150);
    c.receive(cap::dm_bus, response(0x51, std::bit_cast<std::uint32_t>(-0.5f)), 154, 0);
    c.receive(2, {0x20a, 8, {0x10, 0, 0, 0, 0, 0, 25, 0}}, 159, 100);
    for (std::uint8_t page : {1, 3, 5, 7, 9}) {
        std::array<std::uint8_t, 256> payload{};
        const auto n = cap::encode_page(c, page, 160, payload);
        std::printf("%u ", page);
        for (std::size_t i = 0; i < n; ++i) { std::printf("%02x", payload[i]); }
        std::puts("");
    }
}
void raw_contracts() {
    namespace raw = ec::sentinel::recording;
    raw::Recorder recorder;
    std::array<std::uint8_t, 12> request{1, 1, 2, 0};
    raw::put32(request, 4, 123); raw::put32(request, 8, 200000);
    std::array<std::uint8_t, 240> reply{};
    CHECK(recorder.request(request, reply, {0xffffff00u}) == 32 && reply[3] == 1);
    CHECK(recorder.fast_dm());
    recorder.can(1, {0x208, 8, {1, 2, 3, 4, 5, 6, 7, 8}}, {0xfffffff0u}, {0xfffffff0u}, 0);
    recorder.can(2, {0x201, 8, {}}, {0xfffffff0u}, {0xfffffff0u}, 0);
    CHECK(recorder.size() == 1);
    recorder.can(1, {0x208, 8, {}}, {100}, {100}, 0); CHECK(recorder.size() == 1);
    CHECK(recorder.request(request, reply, {110}) == 32 && recorder.size() == 1); // 同 run 重传不清窗口
    request[1] = 3;
    CHECK(recorder.request(request, reply, {120}) == 32 && reply[2] == 3); // 活跃记录不可读
    request[1] = 2;
    CHECK(recorder.request(request, reply, {130}) == 32 && reply[3] == 2);
    request[1] = 3; raw::put32(request, 8, 0);
    CHECK(recorder.request(request, reply, {140}) == 96 && reply[9] == 1);
    const auto record = raw::decode(std::span<const std::uint8_t>{reply}.subspan(32, 64));
    CHECK(record && record->run == 123 && record->stream == 4 && record->sampled_us == 0xfffffff0u);
    CHECK(record->data[1] == 0x04030201u && record->data[2] == 0x08070605u);
    auto broken = reply; broken[32] = 9;
    CHECK(!raw::decode(std::span<const std::uint8_t>{broken}.subspan(32, 64)));
    request[1] = 1; request[2] = 1; raw::put32(request, 4, 124); raw::put32(request, 8, 2000000);
    CHECK(recorder.request(request, reply, {1000}) == 32);
    ec::drivers::Bmi088Reading sample;
    sample.times.gyro_valid = sample.times.gyro_updated = true;
    for (unsigned i = 0; i < 600; ++i) {
        sample.times.gyro_estimated = {1000 + 1000*i}; sample.times.gyro = sample.times.gyro_estimated;
        sample.times.gyro_sequence = i;
        recorder.imu(sample, sample.times.gyro);
        recorder.imu(sample, sample.times.gyro); // 重复观察不能增样本
    }
    CHECK(recorder.size() == raw::capacity && recorder.state() == raw::State::full);
    request[1] = 0;
    CHECK(recorder.request(request, reply, {601000}) == 32 && raw::get32(reply, 16) == 512000);
    raw::Recorder pitch;
    request = {1, 1, 2, 0}; raw::put32(request, 4, 125); raw::put32(request, 8, 200000);
    CHECK(pitch.request(request, reply, {0}) == 32);
    pitch.can(1, {0x011, 8, {}}, {1000}, {1000}, 0);
    pitch.can(2, {0x000, 8, {}}, {1000}, {1000}, 0);
    CHECK(pitch.size() == 0);
    pitch.can(2, {0x011, 8, {1, 2, 3, 4, 5, 6, 7, 8}}, {1000}, {1000}, 0);
    CHECK(pitch.size() == 1);
    request[1] = 2; CHECK(pitch.request(request, reply, {2000}) == 32);
    request[1] = 3; raw::put32(request, 8, 0);
    CHECK(pitch.request(request, reply, {2000}) == 96);
    const auto pitch_record = raw::decode(std::span<const std::uint8_t>{reply}.subspan(32, 64));
    CHECK(pitch_record && pitch_record->stream == 9 && pitch_record->data[0] == (2u | (8u << 8) | (0x011u << 16)));
    CHECK(pitch_record->data[1] == 0x04030201u && pitch_record->data[2] == 0x08070605u);
    cap::Collector collector; Tx tx;
    for (unsigned i = 0; i < 100; ++i) { collector.poll(tx, i, true); }
    CHECK(tx.frames.size() == 50);
    for (const auto& frame : tx.frames) { CHECK(cap::allowed_read(frame) && frame.data[2] == 0xcc); }
}
void raw_golden() {
    namespace raw = ec::sentinel::recording;
    raw::Record record{raw::Kind::gyro, 0, 3, 0, 4, 0xfffffff0u, 20, 1, 600, 9};
    record.data[0] = std::bit_cast<std::uint32_t>(1.25f);
    record.data[1] = std::bit_cast<std::uint32_t>(-0.5f);
    std::array<std::uint8_t, 64> out{};
    CHECK(raw::encode(record, out));
    for (auto byte : out) { std::printf("%02x", byte); }
    std::puts("");
}
int raw_replay(const char* path) {
    namespace raw = ec::sentinel::recording;
    std::ifstream file(path, std::ios::binary);
    std::array<char, 8> magic{}; file.read(magic.data(), magic.size());
    CHECK(std::memcmp(magic.data(), "ECRAW1\r\n", 8) == 0);
    std::array<std::uint8_t, 4> length{};
    file.read(reinterpret_cast<char*>(length.data()), length.size());
    const auto size = raw::get32(length, 0); CHECK(file && size <= 1048576);
    file.seekg(size, std::ios::cur);
    std::array<std::uint8_t, 64> wire{};
    unsigned count{};
    while (file.read(reinterpret_cast<char*>(wire.data()), wire.size())) {
        const auto record = raw::decode(wire); CHECK(record);
        ++count;
        std::printf("%u %u %u %.6f\n", record->run, record->sequence, record->sampled_us,
            static_cast<double>(std::bit_cast<float>(record->data[0])));
    }
    CHECK(file.eof() && file.gcount() == 0 && count > 0);
    return 0;
}
} // 测试内部
int main(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1], "--raw-golden") == 0) { raw_golden(); return 0; }
    if (argc == 3 && std::strcmp(argv[1], "--raw-replay") == 0) { return raw_replay(argv[2]); }
    raw_contracts();
    if (argc == 2 && std::strcmp(argv[1], "--golden") == 0) { golden(); return 0; }
    for (unsigned rid = 0; rid < 256; ++rid) {
        const auto f = cap::read_request(static_cast<std::uint8_t>(rid));
        bool expected = false;
        for (const auto s : cap::registers) { expected |= s.rid == rid; }
        CHECK(static_cast<bool>(f) == expected);
        if (f) {
            CHECK(f->id == 0x7ff && f->size == 4 && f->data[0] == 1 && f->data[1] == 0 && f->data[2] == 0x33);
            CHECK(cap::allowed_read(*f));
            for (unsigned byte = 0; byte < 8; ++byte) {
                auto bad = *f; bad.data[byte] ^= 0x80; CHECK(!cap::allowed_read(bad));
            }
            auto bad = *f; bad.id = 1; CHECK(!cap::allowed_read(bad));
            bad = *f; bad.size = 8; CHECK(!cap::allowed_read(bad));
        }
    }
    const auto status = cap::status_request();
    CHECK(status.id == 0x7ff && status.size == 4 && status.data[2] == 0xcc && status.data[3] == 0);
    CHECK(cap::allowed_read(status));
    auto not_read = status; not_read.data[2] = 0xfc; CHECK(!cap::allowed_read(not_read));
    CHECK(cap::installs[4].bus == 1 && cap::installs[4].rx_id == 0x208 && cap::installs[4].node == 4);
    CHECK(cap::installs[5].bus == 2 && cap::installs[5].rx_id == 0x20a && cap::installs[5].node == 6);
    cap::Collector c;
    for (std::size_t i = 0; i < cap::installs.size(); ++i) {
        const auto entry = cap::installs[i];
        ec::transport::CanFrame f{entry.rx_id, 8, {0, 0, 0, 1, 0, 0, 25, 0}};
        if (entry.kind == cap::Kind::dm) { f.data[0] = 1; }
        c.receive(entry.bus, f, 0xfffffff0u, 0xffffff00u);
        c.receive(entry.bus, f, 1, 0x100u);
        CHECK(c.motors()[i].valid && c.motors()[i].received == 2);
        CHECK(c.motors()[i].intervals == 1 && c.motors()[i].min_cycles == 512);
        c.receive(entry.bus, f, 1001, 0);
        CHECK(c.motors()[i].intervals == 1); // 长断流不伪造跨多次 CYCCNT 回绕差值.
        f.size = 2; c.receive(entry.bus, f, 1002, 1);
        CHECK(!c.motors()[i].valid && c.motors()[i].invalid == 1);
    }
    c.receive(2, {0x201, 8, {}}, 2, 0); CHECK(c.diagnostics().unknown == 1);
    c.receive(1, {0x800, 8, {}}, 2, 0); CHECK(c.diagnostics().malformed == 1);
    // 参数响应必须属于当前成功提交的请求, 参数响应永不流入 MIT 状态槽.
    cap::Collector dm; Tx tx;
    CHECK(cap::dm_bus == 2 && cap::dm_can_id == 1 && cap::dm_master_id == 0x011);
    dm.receive(cap::dm_bus, {0, 8, {1, 0, 0, 0, 0, 0, 25, 26}}, 9, 0);
    dm.receive(1, {0x011, 8, {1, 0, 0, 0, 0, 0, 25, 26}}, 9, 0);
    CHECK(!dm.motors()[9].seen && dm.diagnostics().unknown == 2);
    dm.receive(cap::dm_bus, response(0x50, 0), 10, 0); CHECK(dm.diagnostics().unsolicited == 1);
    dm.poll(tx, 0xfffffff0u); CHECK(tx.frames.size() == 1 && tx.frames.back().data[3] == 0x50);
    dm.receive(cap::dm_bus, response(0x51, 0), 1, 0); CHECK(!dm.values()[11].seen);
    dm.receive(cap::dm_bus == 1 ? 2 : 1, response(0x50, 0), 1, 0); CHECK(!dm.values()[10].seen);
    auto bad = response(0x50, 0); bad.data[0] = 2; dm.receive(cap::dm_bus, bad, 1, 0); CHECK(!dm.values()[10].seen);
    dm.receive(cap::dm_bus, response(0x50, std::bit_cast<std::uint32_t>(1.25f)), 1, 0);
    CHECK(dm.values()[10].seen && dm.values()[10].valid && dm.values()[10].received_ms == 1);
    dm.poll(tx, 0x22); CHECK(tx.frames.size() == 2 && tx.frames.back().data[3] == 0x51);
    dm.receive(cap::dm_bus, response(0x51, std::bit_cast<std::uint32_t>(std::numeric_limits<float>::quiet_NaN())), 0x23, 0);
    CHECK(dm.values()[11].seen && !dm.values()[11].valid);
    CHECK(dm.motors()[9].received == 1); // 仅上面故意错误 node 的普通帧进入 raw 槽.
    dm.poll(tx, 0x54); CHECK(tx.frames.back().data[2] == 0xcc);
    dm.receive(cap::dm_bus, {cap::dm_master_id, 8, {1, 0, 0, 0, 0, 0, 25, 26}}, 0x55, 0);
    CHECK(dm.motors()[9].valid && dm.motors()[9].received == 2);
    dm.poll(tx, 0x86); CHECK(tx.frames.back().data[2] == 0x33 && tx.frames.back().data[3] == 7);
    dm.receive(cap::dm_bus, response(7, 0), 0xea, 0); CHECK(!dm.values()[0].seen); // 严格 100 ms 边界.
    dm.poll(tx, 0xea); CHECK(dm.diagnostics().timeouts == 1);
    // busy/error 不伪造发送成功, 调度不会在一个周期补发积压请求.
    cap::Collector busy; Tx fail; fail.result = ec::transport::SendResult::busy;
    busy.poll(fail, 0); busy.poll(fail, 49); CHECK(fail.frames.size() == 1);
    CHECK(busy.diagnostics().queries == 0 && busy.diagnostics().tx_busy == 1);
    fail.result = ec::transport::SendResult::error; busy.poll(fail, 50); CHECK(busy.diagnostics().tx_error == 1);
    busy.receive(cap::dm_bus, response(0x51, 0), 51, 0); CHECK(!busy.values()[11].seen);
    // 单电机台架不等待其它设备; 缺 DM 时的读超时不阻塞 GM6020 原始采集.
    cap::Collector partial; Tx partial_tx;
    for (unsigned t = 0; t < 1000; ++t) {
        partial.receive(1, {0x208, 8, {0x10, 0, 0, 0, 0, 0, 25, 0}}, t, t * 168000);
        partial.poll(partial_tx, t);
    }
    CHECK(partial.motors()[4].valid && partial.motors()[4].received == 1000);
    CHECK(partial.diagnostics().timeouts > 0);
    for (unsigned i = 0; i < partial.motors().size(); ++i) {
        CHECK(partial.motors()[i].seen == (i == 4));
    }
    for (unsigned t = 1000; t < 2000; ++t) { partial.poll(partial_tx, t); }
    CHECK(partial.motors()[4].last_ms == 999); // 缺帧只保留原采样时间, 不伪造心跳.
    // 多轮发送只能落入读白名单, 不因长期无响应而尝试控制模式.
    for (unsigned t = 0; t < 10000; t += 10) { c.poll(tx, t); }
    for (const auto& f : tx.frames) { CHECK(cap::allowed_read(f)); }
    std::array<std::uint8_t, 256> out{};
    CHECK(cap::encode_page(c, 1, 0, out) == 112);
    CHECK(cap::encode_page(c, 7, 0, out) == 88);
    CHECK(cap::encode_page(c, 1, 0, {out.data(), 111}) == 0);
    CHECK(cap::encode_page(c, 0, 0, out) == 0);
    CHECK(cap::encode_page(c, 10, 0, out) == 0);
    std::puts("capture contracts passed");
}
