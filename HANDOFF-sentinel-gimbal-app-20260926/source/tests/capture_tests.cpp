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
    return {0, 8, {1, 0, 0x33, rid, static_cast<std::uint8_t>(bits), static_cast<std::uint8_t>(bits >> 8),
        static_cast<std::uint8_t>(bits >> 16), static_cast<std::uint8_t>(bits >> 24)}};
}
void golden() {
    cap::Collector c;
    c.receive(2, {0x201, 8, {0x1f, 0xff, 0xff, 0x9c, 0x12, 0x34, 42, 0}}, 100, 1000);
    c.receive(2, {0x201, 8, {0, 1, 0, 120, 0xff, 0xfe, 43, 0}}, 101, 169000);
    c.receive(2, {0x207, 8, {0, 123, 0, 5, 0, 0, 255, 0}}, 101, 169000);
    Tx tx; c.poll(tx, 100);
    c.receive(1, response(0x50, std::bit_cast<std::uint32_t>(1.25f)), 104, 0);
    c.poll(tx, 150);
    c.receive(1, response(0x51, std::bit_cast<std::uint32_t>(-0.5f)), 154, 0);
    c.receive(1, {0x208, 8, {0x10, 0, 0, 0, 0, 0, 25, 0}}, 159, 100);
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
    cap::Collector collector; Tx tx;
    for (unsigned i = 0; i < 100; ++i) { collector.poll(tx, i, true); }
    CHECK(tx.frames.size() == 50);
    for (const auto& frame : tx.frames) { CHECK(cap::allowed_read(frame) && frame.data[2] == 0xcc); }
}
void raw_control_domain_contracts() {
    namespace raw = ec::sentinel::recording;
    struct Expected { std::uint8_t domain, stream, bus; std::uint16_t id; };
    // 域编号与 CAN 流编号不是同一顺序; 独立列出真实反馈地址作为断言依据.
    // 权威规范拓扑: CAN1 (bus 1) 包含 0..4 (LF, RF, LB, RB, main_yaw);
    // CAN2 (bus 2) 包含 5..9 (fire_yaw, Pitch DM, frictionL, frictionR, feeder).
    // control 域 6 (Pitch) 映射至 CAN 流 9 (0x011 MST_ID).
    constexpr std::array<Expected, 10> expected{{
        {0, 0, 1, 0x201}, {1, 1, 1, 0x202}, {2, 2, 1, 0x203}, {3, 3, 1, 0x204},
        {4, 4, 1, 0x208}, {5, 5, 2, 0x20a}, {6, 9, 2, 0x011},
        {7, 6, 2, 0x205}, {8, 7, 2, 0x206}, {9, 8, 2, 0x207}
    }};
    for (const auto& selected : expected) {
        raw::Recorder recorder;
        std::array<std::uint8_t, 12> request{1, 1, 3, selected.domain};
        raw::put32(request, 4, 100 + selected.domain); raw::put32(request, 8, 100000);
        std::array<std::uint8_t, 240> reply{};
        CHECK(recorder.request(request, reply, {1000}) == 32 && reply[2] == 0);
        // 同时注入所有设备以及错误总线, 必须只保留所选域的真实反馈.
        for (const auto& source : expected) {
            const ec::transport::CanFrame frame{source.id, 8, {1, 2, 3, 4, 5, 6, 7, 8}};
            recorder.can(source.bus == 1 ? 2 : 1, frame, {2000}, {2000}, 0);
            recorder.can(source.bus, frame, {3000}, {3000}, 0);
        }
        CHECK(recorder.size() == 1);
        request[1] = 2;
        CHECK(recorder.request(request, reply, {4000}) == 32 && reply[3] == 2);
        request[1] = 3; raw::put32(request, 8, 0);
        CHECK(recorder.request(request, reply, {4000}) == 96 && reply[9] == 1);
        const auto record = raw::decode(std::span<const std::uint8_t>{reply}.subspan(32, 64));
        CHECK(record && record->kind == raw::Kind::can && record->stream == selected.stream);
        CHECK((record->data[0] & 255u) == selected.bus);
        CHECK(((record->data[0] >> 8) & 255u) == 8);
        CHECK((record->data[0] >> 16) == selected.id);
        CHECK(record->data[1] == 0x04030201u && record->data[2] == 0x08070605u);
    }
}
void raw_domain6_and_filtering_contracts() {
    namespace raw = ec::sentinel::recording;

    // 1. 验证 CAN2 0x011 映射至 stream 9, 而 CAN1 0x011 或未知 ID 在任意总线均被拒绝未映射
    {
        raw::Recorder recorder;
        std::array<std::uint8_t, 12> request{1, 1, 2, 0}; // rotation profile: 接收 stream 4, 5, 9
        raw::put32(request, 4, 888); raw::put32(request, 8, 100000);
        std::array<std::uint8_t, 240> reply{};
        CHECK(recorder.request(request, reply, {1000}) == 32 && reply[2] == 0);

        // 注入 CAN1 0x011: 错误总线 (Pitch DM 在 CAN2), 必须拒绝
        recorder.can(1, {0x011, 8, {1, 2, 3, 4, 5, 6, 7, 8}}, {1500}, {1500}, 0);
        CHECK(recorder.size() == 0);

        // 注入未知/非法 ID: 绝不能被静默强制转换
        for (std::uint16_t bad : {0x200, 0x209, 0x111, 0x000, 0x012, 0x7ff}) {
            recorder.can(1, {bad, 8, {}}, {1600}, {1600}, 0);
            recorder.can(2, {bad, 8, {}}, {1650}, {1650}, 0);
            CHECK(recorder.size() == 0);
        }

        // 注入 CAN2 0x011: 正确总线与 ID -> 接收为 stream 9
        recorder.can(2, {0x011, 8, {0xaa, 0xbb, 0xcc, 0xdd, 0x11, 0x22, 0x33, 0x44}}, {2000}, {2000}, 0);
        CHECK(recorder.size() == 1);

        request[1] = 2; // 冻结
        CHECK(recorder.request(request, reply, {3000}) == 32);
        request[1] = 3; raw::put32(request, 8, 0); // 读取
        CHECK(recorder.request(request, reply, {3000}) == 96 && reply[9] == 1);
        const auto rec = raw::decode(std::span<const std::uint8_t>{reply}.subspan(32, 64));
        CHECK(rec && rec->kind == raw::Kind::can && rec->stream == 9);
        CHECK((rec->data[0] & 255u) == 2 && (rec->data[0] >> 16) == 0x011);
        CHECK(rec->data[1] == 0xddccbbaau && rec->data[2] == 0x44332211u);
    }

    // 2. 验证 Profile::control 下 domain 6 短窗行为:
    // 仅接收 Pitch CAN (CAN2 0x011 -> stream 9) 与 Pitch 控制轨迹 (stream 6);
    // 过滤其它 CAN (包括相同编号的 stream 6 frictionL) 及其它域控制轨迹;
    // 校验 64 字节 payload 精度完整、非有限数与状态标志
    {
        raw::Recorder recorder;
        std::array<std::uint8_t, 12> request{1, 1, 3, 6}; // profile=3 (control), domain=6 (Pitch)
        raw::put32(request, 4, 606); raw::put32(request, 8, 100000);
        std::array<std::uint8_t, 240> reply{};
        CHECK(recorder.request(request, reply, {1000}) == 32 && reply[2] == 0);
        CHECK(recorder.control_domain() == 6);

        // 其它 CAN 注入 (底盘、主云台、发射云台、摩擦轮、拨弹) -> 必须全被过滤
        recorder.can(1, {0x201, 8, {}}, {1100}, {1100}, 0); // stream 0
        recorder.can(1, {0x208, 8, {}}, {1200}, {1200}, 0); // stream 4
        recorder.can(2, {0x20a, 8, {}}, {1300}, {1300}, 0); // stream 5
        recorder.can(2, {0x205, 8, {}}, {1400}, {1400}, 0); // stream 6 (摩擦轮L, 严禁与 domain 6 混淆!)
        recorder.can(2, {0x207, 8, {}}, {1450}, {1450}, 0); // stream 8
        CHECK(recorder.size() == 0);

        // 其它域控制轨迹注入 -> 必须全被过滤
        for (std::uint8_t other_d : {0, 1, 2, 3, 4, 5, 7, 8, 9}) {
            raw::Record other{raw::Kind::control, other_d, 1, 0, 1, 1500, 1500, 0, 0, 0};
            recorder.push(other, {1500});
            CHECK(recorder.size() == 0);
        }

        // 注入 Pitch CAN: CAN2 0x011 -> 映射至 stream 9
        recorder.can(2, {0x011, 8, {1, 2, 3, 4, 5, 6, 7, 8}}, {2000}, {2000}, 0);
        CHECK(recorder.size() == 1);

        // 注入 Pitch 控制轨迹 (stream 6)
        raw::Record pitch_ctrl{raw::Kind::control, 6, 1, 0, 1, 2100, 2100, 0, 0, 0};
        pitch_ctrl.data[0] = std::bit_cast<std::uint32_t>(0.125f);  // requested rad
        pitch_ctrl.data[1] = std::bit_cast<std::uint32_t>(0.120f);  // adopted rad
        pitch_ctrl.data[2] = std::bit_cast<std::uint32_t>(0.118f);  // measured rad
        pitch_ctrl.data[3] = std::bit_cast<std::uint32_t>(-0.05f);  // measured vel rad/s
        pitch_ctrl.data[4] = std::bit_cast<std::uint32_t>(0.75f);   // feedforward Nm
        pitch_ctrl.data[5] = std::bit_cast<std::uint32_t>(0.001f);  // dt
        pitch_ctrl.data[6] = 2u | (1u << 8) | (3u << 16) | (3u << 20) | (1u << 29) | (1u << 30);
        pitch_ctrl.data[7] = 0; // accepted
        recorder.push(pitch_ctrl, {2100});
        CHECK(recorder.size() == 2);

        // 再次注入非 Pitch CAN -> 过滤
        recorder.can(2, {0x206, 8, {}}, {2200}, {2200}, 0);
        CHECK(recorder.size() == 2);

        // 冻结并读取短窗
        request[1] = 2;
        CHECK(recorder.request(request, reply, {3000}) == 32 && reply[3] == 2);
        request[1] = 3; raw::put32(request, 8, 0);
        CHECK(recorder.request(request, reply, {3000}) == 160 && reply[9] == 2);

        const auto c_can = raw::decode(std::span<const std::uint8_t>{reply}.subspan(32, 64));
        CHECK(c_can && c_can->kind == raw::Kind::can && c_can->stream == 9);
        CHECK((c_can->data[0] & 255u) == 2 && (c_can->data[0] >> 16) == 0x011);

        const auto c_trace = raw::decode(std::span<const std::uint8_t>{reply}.subspan(96, 64));
        CHECK(c_trace && c_trace->kind == raw::Kind::control && c_trace->stream == 6);
        CHECK(std::bit_cast<float>(c_trace->data[0]) == 0.125f);
        CHECK(std::bit_cast<float>(c_trace->data[1]) == 0.120f);
        CHECK(std::bit_cast<float>(c_trace->data[2]) == 0.118f);
        CHECK(std::bit_cast<float>(c_trace->data[3]) == -0.05f);
        CHECK(std::bit_cast<float>(c_trace->data[4]) == 0.75f);
        CHECK(std::bit_cast<float>(c_trace->data[5]) == 0.001f);
        CHECK((c_trace->data[6] & (1u << 29)) != 0); // excitation-active
        CHECK((c_trace->data[6] & (1u << 30)) != 0); // excitation-latch
    }

    // 3. 验证 continuous 流拒绝 Profile::control 请求
    {
        raw::Recorder recorder;
        std::array<std::uint8_t, 12> stream_req{1, 1, 3, 0}; // profile=3 (control)
        raw::put32(stream_req, 4, 111); raw::put32(stream_req, 8, 100000);
        std::array<std::uint8_t, 256> out{};
        CHECK(recorder.stream_request(stream_req, out, {1000}) == 32);
        CHECK(out[2] == 1); // 拒绝: error = 1
    }
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
    raw_control_domain_contracts();
    raw_domain6_and_filtering_contracts();
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
    c.receive(1, {0x201, 8, {}}, 2, 0); CHECK(c.diagnostics().unknown == 1);
    c.receive(1, {0x800, 8, {}}, 2, 0); CHECK(c.diagnostics().malformed == 1);
    // 参数响应必须属于当前成功提交的请求, 参数响应永不流入 MIT 状态槽.
    cap::Collector dm; Tx tx;
    dm.receive(1, response(0x50, 0), 10, 0); CHECK(dm.diagnostics().unsolicited == 1);
    dm.poll(tx, 0xfffffff0u); CHECK(tx.frames.size() == 1 && tx.frames.back().data[3] == 0x50);
    dm.receive(1, response(0x51, 0), 1, 0); CHECK(!dm.values()[11].seen);
    dm.receive(2, response(0x50, 0), 1, 0); CHECK(!dm.values()[10].seen);
    auto bad = response(0x50, 0); bad.data[0] = 2; dm.receive(1, bad, 1, 0); CHECK(!dm.values()[10].seen);
    dm.receive(1, response(0x50, std::bit_cast<std::uint32_t>(1.25f)), 1, 0);
    CHECK(dm.values()[10].seen && dm.values()[10].valid && dm.values()[10].received_ms == 1);
    dm.poll(tx, 0x22); CHECK(tx.frames.size() == 2 && tx.frames.back().data[3] == 0x51);
    dm.receive(1, response(0x51, std::bit_cast<std::uint32_t>(std::numeric_limits<float>::quiet_NaN())), 0x23, 0);
    CHECK(dm.values()[11].seen && !dm.values()[11].valid);
    CHECK(dm.motors()[9].received == 1); // 仅上面故意错误 node 的普通帧进入 raw 槽.
    dm.poll(tx, 0x54); CHECK(tx.frames.back().data[2] == 0xcc);
    dm.receive(1, {0, 8, {1, 0, 0, 0, 0, 0, 25, 26}}, 0x55, 0);
    CHECK(dm.motors()[9].valid && dm.motors()[9].received == 2);
    dm.poll(tx, 0x86); CHECK(tx.frames.back().data[2] == 0x33 && tx.frames.back().data[3] == 7);
    dm.receive(1, response(7, 0), 0xea, 0); CHECK(!dm.values()[0].seen); // 严格 100 ms 边界.
    dm.poll(tx, 0xea); CHECK(dm.diagnostics().timeouts == 1);
    // busy/error 不伪造发送成功, 调度不会在一个周期补发积压请求.
    cap::Collector busy; Tx fail; fail.result = ec::transport::SendResult::busy;
    busy.poll(fail, 0); busy.poll(fail, 49); CHECK(fail.frames.size() == 1);
    CHECK(busy.diagnostics().queries == 0 && busy.diagnostics().tx_busy == 1);
    fail.result = ec::transport::SendResult::error; busy.poll(fail, 50); CHECK(busy.diagnostics().tx_error == 1);
    busy.receive(1, response(0x51, 0), 51, 0); CHECK(!busy.values()[11].seen);
    // 单电机台架不等待其它设备; 缺 DM 时的读超时不阻塞 GM6020 原始采集.
    cap::Collector partial; Tx partial_tx;
    for (unsigned t = 0; t < 1000; ++t) {
        partial.receive(1, {0x208, 8, {0x10, 0, 0, 0, 0, 0, 25, 0}}, t, t * 168000);
        partial.poll(partial_tx, t);
    }
    CHECK(partial.motors()[5].valid && partial.motors()[5].received == 1000);
    CHECK(partial.diagnostics().timeouts > 0);
    for (unsigned i = 0; i < partial.motors().size(); ++i) {
        CHECK(partial.motors()[i].seen == (i == 5));
    }
    for (unsigned t = 1000; t < 2000; ++t) { partial.poll(partial_tx, t); }
    CHECK(partial.motors()[5].last_ms == 999); // 缺帧只保留原采样时间, 不伪造心跳.
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
