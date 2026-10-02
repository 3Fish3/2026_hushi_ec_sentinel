#include "board_io.h"
#include "board_observation.hpp"
#include "ec/channel.hpp"
#include "ec/control/pid.hpp"
#include "ec/drivers/bmi088.hpp"
#include "ec/drivers/bmi088_device.hpp"
#include "ec/drivers/device_watch.hpp"
#include "ec/drivers/dji.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/drivers/dr16.hpp"
#include "ec/platform/host.hpp"
#include "ec/periodic.hpp"
#include "ec/transport/byte_pipe.hpp"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <numbers>
#include <thread>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3); // 非零退出且不触发 CRT 交互弹窗, ctest 要求非交互
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)
bool near(float a, float b) { return std::fabs(a - b) < 0.0001f; }

void time_contracts() {
    CHECK(ec::fresh({100}, {100}, {1}));
    CHECK(!ec::fresh({101}, {100}, {1}));
    CHECK(!ec::fresh({99}, {100}, {10}));
    CHECK(!ec::fresh({0}, {0}, {0}));
    CHECK(!ec::fresh({0}, {0}, {0x80000000u}));
    CHECK(ec::fresh({2}, {0xfffffffeu}, {5}));
    CHECK(!ec::fresh({3}, {0xfffffffeu}, {5}));
    ec::Sample<float> sample{1.0f, {100}, ec::Validity::unknown};
    CHECK(!sample.fresh_at({100}, {1}));
    sample.validity = ec::Validity::valid;
    CHECK(sample.fresh_at({100}, {1}));
    CHECK(!sample.fresh_at({101}, {1}));
}

void periodic_contracts() {
    CHECK(!ec::plan_periodic(0, 0, 0));
    CHECK(!ec::plan_periodic(0, 0, 0x80000000u));
    CHECK(!ec::plan_periodic(100, 99, 10));
    CHECK(ec::plan_periodic(100, 105, 10)->anchor_tick == 100);
    CHECK(ec::plan_periodic(100, 110, 10)->skipped == 0);
    CHECK(ec::plan_periodic(100, 111, 10)->skipped == 1);
    CHECK(ec::plan_periodic(100, 135, 10)->anchor_tick == 130);
    CHECK(ec::plan_periodic(100, 135, 10)->skipped == 3);
    CHECK(ec::plan_periodic(100, 140, 10)->anchor_tick == 130);
    CHECK(ec::plan_periodic(0xfffffffcu, 2, 4)->anchor_tick == 0);
    CHECK(ec::plan_periodic(0xfffffffcu, 2, 4)->skipped == 1);
}

void gate_and_channels() {
    ec::ExecutionGate gate;
    CHECK(!gate.accepts({}));
    CHECK(!gate.arm(ec::CommandSource::host, false, true));
    CHECK(!gate.arm(ec::CommandSource::host, true, false));
    CHECK(!gate.arm(ec::CommandSource::none, true, true));
    CHECK(!gate.arm(static_cast<ec::CommandSource>(99), true, true));
    const auto first = gate.arm(ec::CommandSource::remote, true, true);
    CHECK(first && gate.accepts(*first));
    gate.invalidate();
    CHECK(!gate.accepts(*first));
    const auto second = gate.arm(ec::CommandSource::host, true, true);
    CHECK(second && gate.accepts(*second) && !gate.accepts(*first));
    auto forged = *second;
    forged.source = ec::CommandSource::remote;
    CHECK(!gate.accepts(forged));
    CHECK(!gate.arm(ec::CommandSource::host, false, true));
    CHECK(!gate.accepts(*second));

    ec::host::MutexCriticalSection critical;
    ec::CommandMailbox<int, ec::host::MutexCriticalSection> mailbox(critical);
    CHECK(!mailbox.read_fresh({100}));
    mailbox.publish({1, {100}, {10}, *first});
    mailbox.publish({2, {101}, {10}, *second});
    CHECK(mailbox.read_fresh({102})->value == 2);
    CHECK(mailbox.read_fresh({102})->grant == *second);
    CHECK(!mailbox.read_fresh({100}));
    CHECK(!mailbox.read_fresh({111}));
    mailbox.publish({3, {101}, {0}, *second});
    CHECK(!mailbox.read_fresh({101}));
    mailbox.clear();
    CHECK(!mailbox.read_fresh({102}));

    ec::EventQueue<int, 2, ec::host::MutexCriticalSection> queue(critical);
    CHECK(!queue.try_pop());
    CHECK(queue.try_push(1) == ec::PushResult::accepted);
    CHECK(queue.try_push(2) == ec::PushResult::accepted);
    CHECK(queue.try_push(3) == ec::PushResult::full);
    CHECK(queue.try_pop() == 1);
    CHECK(queue.try_push(3) == ec::PushResult::accepted);
    CHECK(queue.try_pop() == 2);
    CHECK(queue.try_pop() == 3);
    CHECK(!queue.try_pop());
    CHECK(queue.try_push(4) == ec::PushResult::accepted);
    queue.clear();
    CHECK(!queue.try_pop());
    ec::EventQueue<int, 1, ec::host::MutexCriticalSection> single(critical);
    for (int i = 0; i < 100; ++i) {
        CHECK(single.try_push(i) == ec::PushResult::accepted);
        CHECK(single.try_push(0) == ec::PushResult::full);
        CHECK(single.try_pop() == i);
    }
}

void concurrent_channels() {
    struct Pair { unsigned sequence{}; unsigned complement{}; };
    ec::host::MutexCriticalSection lock;
    ec::LatestValue<Pair, ec::host::MutexCriticalSection> cell(lock);
    std::atomic<bool> done{false};
    std::atomic<unsigned> reads{0};
    std::thread writer([&] {
        for (unsigned i = 0; i < 20000; ++i) { cell.publish({i, ~i}); }
        done.store(true);
    });
    const auto reader = [&] {
        do {
            const auto value = cell.read();
            if (value) { CHECK(value->complement == ~value->sequence); ++reads; }
        } while (!done.load());
    };
    std::thread a(reader), b(reader);
    writer.join(); a.join(); b.join();
    CHECK(reads.load() > 0);
    CHECK(cell.read()->sequence == 19999);

    ec::EventQueue<unsigned, 8, ec::host::MutexCriticalSection> queue(lock);
    std::thread producer([&] {
        for (unsigned i = 0; i < 10000; ++i) {
            while (queue.try_push(i) == ec::PushResult::full) { std::this_thread::yield(); }
        }
    });
    std::thread consumer([&] {
        for (unsigned expected = 0; expected < 10000;) {
            const auto value = queue.try_pop();
            if (value) { CHECK(*value == expected); ++expected; }
            else { std::this_thread::yield(); }
        }
    });
    producer.join(); consumer.join();
    CHECK(!queue.try_pop());

    // 收帧预算回归: 消费者每周期只取固定上限(这里 3), 生产者在同一队列上持续入队.
    // 关键性质: 消费后腾出的空间必须能被继续入队, 剩余帧留到下一周期, 队列不会"永久填满".
    ec::EventQueue<unsigned, 8, ec::host::MutexCriticalSection> budgeted(lock);
    for (unsigned i = 0; i < 8; ++i) { CHECK(budgeted.try_push(i) == ec::PushResult::accepted); }
    CHECK(budgeted.try_push(99) == ec::PushResult::full); // 满时明确拒绝, 不静默丢帧
    // FIFO: 先按序取走最初的 8 帧, 再取走中途入队的 2 帧.
    const std::array<unsigned, 10> expected{0, 1, 2, 3, 4, 5, 6, 7, 100, 101};
    std::size_t received = 0;
    for (unsigned cycle = 0; cycle < 4; ++cycle) {
        constexpr unsigned per_cycle = 3; // 与 board 层 can_frames_per_cycle 同一语义
        for (unsigned n = 0; n < per_cycle; ++n) {
            const auto value = budgeted.try_pop();
            if (!value) { break; }
            CHECK(received < expected.size() && *value == expected[received]);
            ++received;
        }
        // 生产者继续入队: 上一周期取走的槽位必须立即可用.
        if (cycle < 2) { CHECK(budgeted.try_push(100 + cycle) == ec::PushResult::accepted); }
    }
    while (const auto value = budgeted.try_pop()) {
        CHECK(received < expected.size() && *value == expected[received]);
        ++received;
    }
    CHECK(received == expected.size());
}

void pid_contracts() {
    using ec::control::Pid;
    Pid invalid({});
    CHECK(!invalid.configured());
    CHECK(!invalid.step(0, 0, 1));
    Pid p({2, 0, 0, 0, -3, 3});
    CHECK(p.configured());
    CHECK(near(*p.step(1, 0, 0.001f), 2));
    CHECK(*p.step(100, 0, 0.001f) == 3);
    CHECK(*p.step(-100, 0, 0.001f) == -3);
    CHECK(!p.step(1, 0, 0));
    CHECK(!p.step(1, 0, -1));
    CHECK(!p.step(std::numeric_limits<float>::quiet_NaN(), 0, 1));
    CHECK(!p.step(0, std::numeric_limits<float>::infinity(), 1));
    CHECK(!p.step(1, 0, std::numeric_limits<float>::infinity()));
    CHECK(!p.step(std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), 1));
    Pid i({0, 1, 0, 0.5f, -10, 10});
    for (int n = 0; n < 100; ++n) { CHECK(*i.step(1, 0, 1) == 0.5f); }
    i.reset(); CHECK(*i.step(0, 0, 1) == 0);
    Pid d({0, 0, 1, 0, -100, 100});
    CHECK(*d.step(1, 2, 0.1f) == 0);
    CHECK(*d.step(10, 2, 0.1f) == 0); // 改变目标不产生微分冲击.
    CHECK(near(*d.step(10, 3, 0.1f), -10));
    CHECK(!d.step(0, 0, 0));
    CHECK(*d.step(0, 20, 1) == 0); // 无效输入后清除历史测量.
    Pid filtered({2,3,.5f,10,-1000,1000,.003f});
    CHECK(filtered.step(1,0,.001f));
    CHECK(filtered.step(1,.4f,.001f));
    CHECK(near(filtered.diagnostics().p,1.2f));
    CHECK(near(filtered.diagnostics().i,.0048f));
    CHECK(near(filtered.diagnostics().d,-50)); // D低通不延迟P/I测量.
    CHECK(filtered.step(1,.4f,.002f));
    CHECK(near(filtered.diagnostics().d,-30)); // 根据实际dt而非固定采样率更新.
    CHECK(filtered.step(10,.4f,.001f));CHECK(near(filtered.diagnostics().d,-22.5f));
    CHECK(!filtered.step(0,0,0));CHECK(filtered.step(0,20,.001f));
    CHECK(filtered.diagnostics().d==0);
    auto cfg=filtered.config();cfg.derivative_tau_s=-1;
    CHECK(!filtered.set_config(cfg));CHECK(!Pid(cfg).configured());
    cfg.derivative_tau_s=std::numeric_limits<float>::quiet_NaN();
    CHECK(!filtered.set_config(cfg));CHECK(!Pid(cfg).configured());
    cfg=filtered.config();cfg.derivative_tau_s=0;CHECK(filtered.set_config(cfg));
    CHECK(filtered.step(0,21,.001f));CHECK(near(filtered.diagnostics().d,-500));
    filtered.reset();CHECK(filtered.step(0,10,.001f));CHECK(filtered.diagnostics().d==0);
}

void dji_contracts() {
    using namespace ec::drivers;
    const auto c610 = encode_dji(DjiMode::c610_current, 1, {10000, -10000, 0, 1});
    CHECK(c610 && c610->id == 0x200 && c610->size == 8);
    CHECK(c610->data == std::array<std::uint8_t, 8>{0x27, 0x10, 0xd8, 0xf0, 0, 0, 0, 1});
    CHECK(!encode_dji(DjiMode::c610_current, 1, {10001, 0, 0, 0}));
    CHECK(!encode_dji(DjiMode::c620_current, 1, {-16385, 0, 0, 0}));
    CHECK(encode_dji(DjiMode::c620_current, 2, {16384, -16384, 0, 0})->id == 0x1ff);
    CHECK(encode_dji(DjiMode::gm6020_current, 1, {})->id == 0x1fe);
    CHECK(encode_dji(DjiMode::gm6020_current, 2, {})->id == 0x2fe);
    CHECK(encode_dji(DjiMode::gm6020_voltage, 1, {25000, -25000, 0, 0})->id == 0x1ff);
    CHECK(encode_dji(DjiMode::gm6020_voltage, 2, {})->id == 0x2ff);
    CHECK(!encode_dji(DjiMode::gm6020_current, 2, {0, 0, 0, 1}));
    CHECK(!encode_dji(DjiMode::gm6020_voltage, 1, {25001, 0, 0, 0}));
    CHECK(!encode_dji(DjiMode::c620_current, 0, {}));
    CHECK(!encode_dji(DjiMode::c620_current, 3, {}));
    CHECK(!encode_dji(static_cast<DjiMode>(99), 1, {}));
    ec::transport::CanFrame frame{0x20a, 8, {0x10, 0, 0xff, 0xc4, 0xff, 0xff, 42, 0}};
    const auto gm = decode_dji(DjiMode::gm6020_current, frame);
    CHECK(gm && gm->device_id == 6);
    CHECK(near(gm->rotor_angle_rad, std::numbers::pi_v<float>));
    CHECK(near(gm->rotor_speed_rad_s, -2 * std::numbers::pi_v<float>));
    CHECK(gm->effort_raw == -1 && gm->temperature_c == 42);
    // 穷举反馈速度字, 确认大端/补码与单位换算不引入半 RPM 偏移; 这不是允许驱动范围.
    for (int rpm = -32768; rpm <= 32767; ++rpm) {
        const auto bits = static_cast<std::uint16_t>(rpm);
        auto sample = frame;
        sample.data[2] = static_cast<std::uint8_t>(bits >> 8);
        sample.data[3] = static_cast<std::uint8_t>(bits & 0xff);
        const auto decoded = decode_dji(DjiMode::gm6020_current, sample);
        CHECK(decoded.has_value());
        const double expected = static_cast<double>(rpm) * 2.0 * std::numbers::pi_v<double> / 60.0;
        CHECK(std::fabs(static_cast<double>(decoded->rotor_speed_rad_s) - expected) < 0.001);
        CHECK(std::lround(static_cast<double>(decoded->rotor_speed_rad_s) * 60.0 /
            (2.0 * std::numbers::pi_v<double>)) == rpm);
    }
    CHECK(!decode_dji(DjiMode::c620_current, frame));
    frame.id = 0x207;
    CHECK(!decode_dji(DjiMode::c610_current, frame)->temperature_c);
    frame.size = 7; CHECK(!decode_dji(DjiMode::c610_current, frame));
    frame.size = 9; CHECK(!decode_dji(DjiMode::c610_current, frame));
    frame.size = 8; frame.data[0] = 0x20;
    CHECK(!decode_dji(DjiMode::c610_current, frame));
    frame.data[0] = 0; frame.id = 0x20c;
    CHECK(!decode_dji(DjiMode::gm6020_current, frame));
    frame.id = 0x800; CHECK(!frame.valid());
}

std::array<std::uint8_t, 18> pack_dr16(std::int16_t c0, std::int16_t c1, std::int16_t c2, std::int16_t c3,
                                       std::uint8_t sw_left, std::uint8_t sw_right) {
    const unsigned u0 = static_cast<unsigned>(c0 + 1024);
    const unsigned u1 = static_cast<unsigned>(c1 + 1024);
    const unsigned u2 = static_cast<unsigned>(c2 + 1024);
    const unsigned u3 = static_cast<unsigned>(c3 + 1024);
    std::array<std::uint8_t, 18> buf{};
    buf[0] = static_cast<std::uint8_t>(u0 & 0xff);
    buf[1] = static_cast<std::uint8_t>(((u0 >> 8) & 0x07) | ((u1 & 0x1f) << 3));
    buf[2] = static_cast<std::uint8_t>(((u1 >> 5) & 0x3f) | ((u2 & 0x03) << 6));
    buf[3] = static_cast<std::uint8_t>((u2 >> 2) & 0xff);
    buf[4] = static_cast<std::uint8_t>(((u2 >> 10) & 0x01) | ((u3 & 0x7f) << 1));
    buf[5] = static_cast<std::uint8_t>(((u3 >> 7) & 0x0f) | ((sw_right & 0x03) << 4) | ((sw_left & 0x03) << 6));
    return buf;
}

void byte_pipe_contracts() {
    using ec::transport::DmaBytePipe;
    using ec::transport::ByteTxQueue;
    using ec::transport::UartError;
    DmaBytePipe<16> rx;
    std::uint8_t scratch[16]{};
    CHECK(rx.available() == 0);
    for (int i = 0; i < 5; ++i) { rx.dma_buffer()[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(i + 1); }
    rx.on_dma_write(5);
    CHECK(rx.available() == 5 && rx.received() == 5);
    CHECK(rx.read({scratch, 3}) == 3);
    CHECK(scratch[0] == 1 && scratch[2] == 3 && rx.available() == 2);
    rx.dma_buffer()[5] = 9; rx.dma_buffer()[6] = 8; rx.on_dma_write(7);
    CHECK(rx.read({scratch, 8}) == 4);
    CHECK(scratch[0] == 4 && scratch[3] == 8 && rx.available() == 0);

    DmaBytePipe<8> wrap;
    for (int i = 0; i < 6; ++i) { wrap.dma_buffer()[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(10 + i); }
    wrap.on_dma_write(6);
    CHECK(wrap.read({scratch, 5}) == 5);
    wrap.dma_buffer()[6] = 1; wrap.dma_buffer()[7] = 2; wrap.dma_buffer()[0] = 3; wrap.dma_buffer()[1] = 4;
    wrap.on_dma_write(2);
    CHECK(wrap.available() == 5);
    CHECK(wrap.read({scratch, 5}) == 5);
    CHECK(scratch[0] == 15 && scratch[1] == 1 && scratch[4] == 4);

    DmaBytePipe<8> overflow;
    for (int i = 0; i < 7; ++i) { overflow.dma_buffer()[static_cast<std::size_t>(i)] = 1; }
    overflow.on_dma_write(7);
    CHECK(overflow.available() == 7);
    overflow.dma_buffer()[7] = 9; overflow.dma_buffer()[0] = 9;
    overflow.on_dma_write(1); // 覆盖未读
    CHECK(overflow.overflows() == 1 && overflow.available() == 0);
    CHECK(overflow.read({scratch, 8}) == 0); // 不交付被覆盖的旧数据
    overflow.dma_buffer()[1] = 42; overflow.on_dma_write(2);
    CHECK(overflow.read({scratch, 1}) == 1 && scratch[0] == 42);
    // DMA 错误后硬件从零重启, 不得把旧尾部误当作新接收数据.
    DmaBytePipe<64> restarted;
    restarted.on_dma_write(18);
    std::uint8_t restart_out[64]{};
    CHECK(restarted.read(restart_out) == 18);
    restarted.on_dma_write(21); // 保留 3 字节未读数据
    restarted.reset();
    CHECK(restarted.available() == 0 && restarted.discarded() == 3);
    CHECK(restarted.received() == 21 && restarted.write_index() == 0);
    restarted.dma_buffer()[0] = 0xa5;
    restarted.dma_buffer()[1] = 0x5a;
    restarted.on_dma_write(2);
    CHECK(restarted.read(restart_out) == 2);
    CHECK(restart_out[0] == 0xa5 && restart_out[1] == 0x5a);
    CHECK(restarted.overflows() == 0 && restarted.received() == 23);

    overflow.note_error(UartError::framing);
    overflow.note_error(UartError::overrun);
    CHECK(overflow.framing() == 1 && overflow.overrun() == 1);

    ByteTxQueue<8> tx;
    const std::uint8_t hello[]{1, 2, 3, 4, 5};
    CHECK(tx.try_push(hello));
    CHECK(!tx.try_push(hello));
    CHECK(tx.peek().size() == 5 && tx.peek()[0] == 1);
    tx.pop(2);
    CHECK(tx.size() == 3 && tx.peek()[0] == 3);
    const std::uint8_t more[]{9, 8, 7};
    CHECK(tx.try_push(more));
    CHECK(tx.peek().size() == 6); // 尚未回绕
    tx.pop(6);
    CHECK(tx.size() == 0);
    CHECK(tx.try_push(hello));
    tx.pop(4);
    const std::uint8_t tail[]{6, 7, 8, 9};
    CHECK(tx.try_push(tail));
    CHECK(tx.peek().size() == 4); // 回绕后只给出连续段
    tx.pop(4);
    CHECK(tx.peek().size() == 1 && tx.peek()[0] == 9);
}

void dr16_contracts() {
    using namespace ec::drivers;
    const auto center = pack_dr16(0, 0, 0, 0, 1, 3);
    const auto sample = decode_dr16(center);
    CHECK(sample && sample->ch0 == 0 && sample->ch3 == 0);
    CHECK(sample->switch_left == Dr16Switch::up && sample->switch_right == Dr16Switch::mid);
    CHECK(near(dr16_axis(660), 1) && near(dr16_axis(-330), -0.5f));
    CHECK(decode_dr16(pack_dr16(660, -660, 1, -1, 2, 1)));
    CHECK(!decode_dr16(pack_dr16(661, 0, 0, 0, 1, 1)));
    CHECK(!decode_dr16(pack_dr16(0, 0, 0, 0, 0, 1)));
    auto bad = pack_dr16(0, 0, 0, 0, 1, 1);
    bad[12] = 2;
    CHECK(!decode_dr16(bad));
    Dr16Stream stream;
    CHECK(!stream.on_idle()); // 首个边界只用于同步
    stream.push({center.data(), 10}); // 模拟 HT 分块, 不能提前发布
    CHECK(stream.accepted == 0);
    stream.push({center.data() + 10, 8});
    const auto assembled = stream.on_idle();
    CHECK(assembled && assembled->switch_left == Dr16Switch::up && stream.accepted == 1);
    const std::uint8_t junk[18]{};
    stream.push(junk);
    CHECK(!stream.on_idle() && stream.rejected == 1);
    for (std::size_t offset = 1; offset < center.size(); ++offset) {
        Dr16Stream shifted;
        shifted.push({center.data() + offset, center.size() - offset});
        CHECK(!shifted.on_idle()); // 任意初始偏移均丢弃至真实帧间隔
        for (int i = 0; i < 100; ++i) {
            shifted.push(center);
            CHECK(shifted.on_idle());
        }
        CHECK(shifted.accepted == 100);
    }
    stream.push({center.data(), 17}); // 丢一个字节
    CHECK(!stream.on_idle());
    stream.push(center);
    stream.push({center.data(), 1}); // 多一个字节
    CHECK(!stream.on_idle());
    stream.push(center);
    stream.push(center); // 缺失 IDLE 不能把两个帧静默切出
    CHECK(!stream.on_idle());
    stream.push(center);
    CHECK(stream.on_idle());
    stream.push({center.data(), 10});
    const auto accepted_before_reset = stream.accepted;
    stream.reset(); // 溢出或 UART 中止使半帧和同步状态一起失效
    stream.push({center.data() + 10, 8});
    CHECK(!stream.on_idle());
    stream.push(center);
    CHECK(stream.on_idle() && stream.accepted == accepted_before_reset + 1);
}

void board_observation_contracts() {
    using namespace ec::drivers;
    CHECK(ec::sentinel::dji_mode(1, 0x208) == DjiMode::gm6020_current);
    CHECK(ec::sentinel::dji_mode(1, 0x20a) == DjiMode::gm6020_current);
    for (std::uint16_t id = 0x201; id <= 0x206; ++id) {
        CHECK(ec::sentinel::dji_mode(2, id) == DjiMode::c620_current);
    }
    CHECK(ec::sentinel::dji_mode(2, 0x207) == DjiMode::c610_current);
    CHECK(!ec::sentinel::dji_mode(1, 0x201));
    CHECK(!ec::sentinel::dji_mode(2, 0x208));
    CHECK(!ec::sentinel::dji_mode(3, 0x207));
    ec::transport::CanFrame frame{0x207, 8, {0, 0, 0x03, 0xe8, 0, 0, 0xaa, 0}};
    auto decoded = decode_dji(*ec::sentinel::dji_mode(2, frame.id), frame);
    CHECK(decoded && !decoded->temperature_c);
    EcCanDiag diag{};
    static_assert(std::is_same_v<decltype(diag.last_speed_rad_s), float>);
    diag.last_speed_rad_s = decoded->rotor_speed_rad_s;
    CHECK(near(diag.last_speed_rad_s, 104.71976f)); // 1000 rpm, 不再窄化
    for (auto hi : {std::uint8_t{0x7f}, std::uint8_t{0x80}}) {
        frame.data[2] = hi;
        frame.data[3] = hi == 0x7f ? 0xff : 0;
        decoded = decode_dji(*ec::sentinel::dji_mode(2, frame.id), frame);
        CHECK(decoded);
        diag.last_speed_rad_s = decoded->rotor_speed_rad_s;
        CHECK(std::isfinite(diag.last_speed_rad_s) && std::fabs(diag.last_speed_rad_s) > 3400);
    }
    frame.id = 0x206;
    decoded = decode_dji(*ec::sentinel::dji_mode(2, frame.id), frame);
    CHECK(decoded && decoded->temperature_c == 0xaa);

    // 模拟 3 秒无 IDLE 的 115200 8N1 输入、同速 TX 和每 10 ms 的有界任务搬运.
    ec::transport::DmaBytePipe<512> rx;
    ec::transport::ByteTxQueue<512> tx;
    std::uint32_t received = 0, echoed = 0;
    const auto poll = [&] {
        std::uint8_t tmp[ec::sentinel::uart1_copy_chunk]{};
        for (std::size_t budget = ec::sentinel::uart1_poll_budget; budget != 0;) {
            const auto room = tx.free();
            const auto limit = room < sizeof(tmp) ? room : sizeof(tmp);
            const auto n = rx.read({tmp, limit < budget ? limit : budget});
            if (n == 0) { break; }
            CHECK(tx.try_push({tmp, n}));
            budget -= n;
        }
    };
    const auto transmit = [&] {
        const auto chunk = tx.peek();
        if (chunk.empty()) { return; }
        CHECK(chunk[0] == static_cast<std::uint8_t>(echoed));
        tx.pop(1);
        ++echoed;
    };
    for (std::uint32_t tick = 1; tick <= 30000; ++tick) { // 每步 100 us
        const auto target = tick * 1152 / 1000;
        while (received < target) {
            transmit();
            rx.dma_buffer()[received % 512] = static_cast<std::uint8_t>(received);
            ++received;
            if (received % 256 == 0) { rx.on_dma_write(received % 512); }
        }
        if (tick % 100 == 0) { poll(); }
    }
    rx.on_dma_write(received % 512); // 最后一个 IDLE 公布尾部
    for (std::uint32_t i = 0; i < 2048; ++i) { poll(); transmit(); }
    CHECK(echoed == received && received == 34560);
    CHECK(rx.overflows() == 0 && rx.discarded() == 0);
}

void bmi088_contracts() {
    using namespace ec::drivers;
    const std::uint8_t acc_raw[6]{0, 0, 0, 0, 0x00, 0x40}; // Z = 16384 LSB
    const auto acc = parse_bmi088_accel_6g(acc_raw);
    CHECK(acc && near((*acc)[0], 0) && near((*acc)[2], 16384 * bmi088_acc_6g_mps2));
    const std::uint8_t gyro_raw[6]{0x00, 0x80, 0, 0, 0, 0}; // X = -32768
    const auto gyro = parse_bmi088_gyro_2000(gyro_raw);
    CHECK(gyro && near((*gyro)[0], -32768 * bmi088_gyro_2000_rad_s));
    CHECK(near(parse_bmi088_temp_c(0, 0), 23));
    CHECK(near(parse_bmi088_temp_c(0x07, 0xe0), 30.875f)); // 0x07e0 >> 5 = 63, *0.125 + 23
    std::uint8_t acc_data[2]{};
    const std::uint8_t acc_rx[4]{0x92, 0xaa, 0x11, 0x22};
    CHECK(parse_bmi088_accel_spi_read(acc_rx, acc_data) && acc_data[0] == 0x11 && acc_data[1] == 0x22);
    std::uint8_t gyro_data[2]{};
    const std::uint8_t gyro_rx[3]{0x82, 0x33, 0x44};
    CHECK(parse_bmi088_gyro_spi_read(gyro_rx, gyro_data) && gyro_data[0] == 0x33);
    CHECK(bmi088_init_steps()[0].reg == bmi088_acc_softreset_reg);
    CHECK(bmi088_init_steps()[5].value == 0x01); // 6 g
}

// 内存 SPI 替身: 只回答芯片 ID 与固定载荷, 用于验证设备层的初始化/时效/回绕行为.
struct FakeBmi088Bus final : ec::drivers::Bmi088Bus {
    bool ok{true};
    bool acc_id_bad{false};
    bool accel_ready{true};
    std::uint8_t gyro_frames{2};
    bool fifo_overrun{};
    std::uint32_t time_us{};
    unsigned temperature_reads{};
    bool temperature_invalid{};
    std::array<std::uint8_t, 6> accel{0, 0, 0, 0, 0x00, 0x40}; // Z = 16384 LSB
    std::array<std::uint8_t, 6> gyro{};
    [[nodiscard]] bool read(bool acc, std::uint8_t reg, std::span<std::uint8_t> out) noexcept override {
        using namespace ec::drivers;
        time_us += 10;
        if (!ok) { return false; }
        if (acc && reg == 0x03 && out.size() == 1) {
            out[0] = accel_ready ? 0x80 : 0;
            return true;
        }
        if (!acc && reg == 0x0e && out.size() == 1) {
            out[0] = static_cast<std::uint8_t>(gyro_frames | (fifo_overrun ? 0x80 : 0));
            return true;
        }
        if (!acc && reg == 0x3f && out.size() == 6 && gyro_frames) {
            --gyro_frames;
            std::memcpy(out.data(), gyro.data(), gyro.size());
            return true;
        }
        if (acc && reg == bmi088_acc_chip_id_reg && out.size() == 1) {
            out[0] = acc_id_bad ? static_cast<std::uint8_t>(bmi088_acc_chip_id + 1u) : bmi088_acc_chip_id;
            return true;
        }
        if (!acc && reg == bmi088_gyro_chip_id_reg && out.size() == 1) {
            out[0] = bmi088_gyro_chip_id;
            return true;
        }
        if (acc && reg == bmi088_acc_data_reg && out.size() == accel.size()) {
            accel_ready = false;
            std::memcpy(out.data(), accel.data(), accel.size());
            return true;
        }
        if (!acc && reg == bmi088_gyro_data_reg && out.size() == gyro.size()) {
            std::memcpy(out.data(), gyro.data(), gyro.size());
            return true;
        }
        if (acc && reg == bmi088_acc_temp_reg && out.size() == 2) {
            ++temperature_reads;
            out[0] = temperature_invalid ? 0x80 : 0x00;
            out[1] = temperature_invalid ? 0x00 : 0x50;
            return true;
        }
        return false;
    }
    [[nodiscard]] bool write(bool accel, std::uint8_t reg, std::uint8_t) noexcept override {
        if (ok && !accel && reg == 0x3e) { gyro_frames = 0; fifo_overrun = false; }
        return ok;
    }
    void delay_ms(std::uint32_t) noexcept override {}
    [[nodiscard]] ec::MicroTick micros() const noexcept override { return {time_us}; }
};

// R06: BMI088 设备层的时间倒退检查同样必须使用无符号差值, 不能直接比较毫秒值.
void bmi088_device_contracts() {
    using namespace ec::drivers;
    FakeBmi088Bus bus{};
    Bmi088Device imu{bus, ec::Lifetime{20}};
    CHECK(imu.read({10}) == Bmi088Result::not_initialized);
    CHECK(imu.init() == Bmi088Result::ok);
    CHECK(imu.initialized() && !imu.ready());
    CHECK(imu.read({100}) == Bmi088Result::ok);
    CHECK(imu.ready() && imu.reads() == 1);
    CHECK(imu.last_reading().dt_ms == 0); // 首帧没有间隔
    CHECK(imu.last_reading().sampled_at.ms == 100);
    CHECK(near(imu.last_reading().sample.acc_mps2[2], 16384 * bmi088_acc_6g_mps2));
    CHECK(imu.reading({119}).has_value() && !imu.reading({120}).has_value()); // 严格 < TTL

    // 传输失败不更新样本, 也不推进时间基准.
    bus.ok = false;
    CHECK(imu.read({130}) == Bmi088Result::transport_error);
    CHECK(imu.transport_errors() == 1);
    CHECK(imu.last_reading().sampled_at.ms == 100);
    bus.ok = true;

    // 正常回绕: 新设备的首次读取就落在回绕附近 (同一会话内不可能从 100 跳到回绕附近),
    // 然后 0xFFFFFFFE -> 1 是前进 3 ms, 必须接受且 dt_ms 正确.
    FakeBmi088Bus wrap_bus{};
    Bmi088Device wrap{wrap_bus, ec::Lifetime{20}};
    CHECK(wrap.init() == Bmi088Result::ok);
    CHECK(wrap.read({0xFFFFFFFEu}) == Bmi088Result::ok);
    CHECK(wrap.read({0x00000001u}) == Bmi088Result::ok);
    CHECK(wrap.last_reading().dt_ms == 3);
    CHECK(wrap.last_reading().previous_at.ms == 0xFFFFFFFEu);
    CHECK(wrap.reading({0x00000001u}).has_value());
    // 真正的倒退仍必须拒绝, 且不改变已接受样本.
    CHECK(wrap.read({0x00000001u - 5u}) == Bmi088Result::time_regression);
    CHECK(wrap.last_reading().sampled_at.ms == 0x00000001u);
    // 回绕之后继续正常接收.
    CHECK(wrap.read({10}) == Bmi088Result::ok);
    CHECK(wrap.last_reading().dt_ms == 9);

    // 芯片 ID 不符: init 失败且不标记已初始化, 后续读取返回 not_initialized.
    FakeBmi088Bus wrong{};
    wrong.acc_id_bad = true;
    Bmi088Device other{wrong, ec::Lifetime{20}};
    CHECK(other.init() == Bmi088Result::chip_id_mismatch);
    CHECK(!other.initialized());
    CHECK(other.read({10}) == Bmi088Result::not_initialized);

    FakeBmi088Bus timed_bus;
    Bmi088Device timed{timed_bus, {20}};
    CHECK(timed.init(true) == Bmi088Result::ok);
    timed_bus.gyro_frames = 2;
    CHECK(timed.read_timed({1}) == Bmi088Result::ok);
    auto timing = timed.last_reading().times;
    CHECK(timing.gyro_sequence == 2 && timing.accel_sequence == 1);
    CHECK(timing.gyro_discarded == 1 && timing.gyro_time_error_us >= 500);
    CHECK(timing.gyro.us < timing.accel.us);
    CHECK(timed.read_timed({2}) == Bmi088Result::no_new_data);
    CHECK(timed.last_reading().times.gyro_sequence == 2);
    CHECK(timed_bus.temperature_reads == 1);
    timed_bus.gyro_frames = 1;
    CHECK(timed.read_timed({3}) == Bmi088Result::ok);
    CHECK(timed.last_reading().times.gyro_updated && !timed.last_reading().times.accel_updated);
    CHECK(timed.last_reading().times.accel.us == timing.accel.us);
    timed_bus.gyro_frames = 99;
    timed_bus.fifo_overrun = true;
    CHECK(timed.read_timed({4}) == Bmi088Result::no_new_data);
    CHECK(timed.last_reading().times.gyro_overruns == 1 && timed_bus.gyro_frames == 0);
    CHECK(timed.last_reading().sampled_at.ms == 3);
    timed_bus.time_us += 1280000;
    timed_bus.temperature_invalid = true;
    CHECK(timed.read_timed({4}) == Bmi088Result::no_new_data);
    CHECK(!timed.last_reading().times.temperature_valid);
    CHECK(timed_bus.temperature_reads == 2);
}

ec::transport::CanFrame dji_feedback(std::uint16_t id, std::uint16_t angle, std::int16_t rpm) {    const auto speed = static_cast<std::uint16_t>(rpm);
    return {id, 8, {static_cast<std::uint8_t>(angle >> 8), static_cast<std::uint8_t>(angle),
        static_cast<std::uint8_t>(speed >> 8), static_cast<std::uint8_t>(speed), 0, 0, 25, 0}};
}

struct RecordingCan final : ec::transport::CanTx {
    ec::transport::SendResult result{ec::transport::SendResult::accepted};
    ec::transport::CanFrame last{};
    unsigned calls{};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        ++calls;
        last = frame;
        return result;
    }
};

void dji_device_contracts() {
    using namespace ec::drivers;
    using ec::Validity;
    constexpr DjiDeviceConfig config{1, DjiMode::c620_current, 1, 20, -1, {15}, 200, 10000};
    DjiDevice device{config};
    CHECK(device.configured());
    CHECK(device.state({0}).velocity_rad_s.validity == Validity::unknown);
    CHECK(!device.reference_position(0, {0}));
    CHECK(device.receive(2, dji_feedback(0x201, 8190, 600), {100}) == DjiReceiveResult::ignored);
    CHECK(device.receive(1, dji_feedback(0x202, 8190, 600), {100}) == DjiReceiveResult::ignored);
    CHECK(device.receive(1, dji_feedback(0x201, 8190, 600), {100}) == DjiReceiveResult::accepted);
    CHECK(near(device.state({100}).velocity_rad_s.value, -std::numbers::pi_v<float>));
    CHECK(device.state({100}).position_rad.validity != Validity::valid);
    CHECK(device.reference_position(1.25f, {100}));
    CHECK(device.receive(1, dji_feedback(0x201, 4, 600), {105}) == DjiReceiveResult::accepted);
    CHECK(near(device.state({105}).position_rad.value, 1.25f - 6 * 2 * std::numbers::pi_v<float> / (8192 * 20)));
    CHECK(device.receive(1, dji_feedback(0x201, 8190, 600), {110}) == DjiReceiveResult::accepted);
    CHECK(near(device.state({110}).position_rad.value, 1.25f));
    CHECK(device.receive(1, dji_feedback(0x201, 8190, 600), {110}) == DjiReceiveResult::accepted);
    CHECK(device.state({124}).position_rad.validity == Validity::valid);
    CHECK(device.state({125}).velocity_rad_s.validity == Validity::invalid);
    CHECK(device.state({125}).position_rad.validity == Validity::invalid);
    CHECK(device.receive(1, dji_feedback(0x201, 8190, 600), {125}) == DjiReceiveResult::accepted);
    CHECK(device.state({125}).position_rad.validity == Validity::invalid); // 重连不自动恢复位置参考.
    CHECK(device.reference_position(2, {125}));
    CHECK(device.receive(1, dji_feedback(0x201, 8190, 600), {124}) == DjiReceiveResult::invalid_time);
    CHECK(device.state({125}).rotor.sampled_at.ms == 125);
    CHECK(device.state({125}).position_rad.validity == Validity::invalid);
    CHECK(device.reference_position(2, {125}));
    auto bad = dji_feedback(0x201, 8190, 600);
    bad.size = 7;
    CHECK(device.receive(1, bad, {126}) == DjiReceiveResult::invalid_frame);
    CHECK(device.state({126}).rotor.sampled_at.ms == 125);
    CHECK(device.state({126}).position_rad.validity == Validity::invalid);
    CHECK(device.receive(1, dji_feedback(0x201, 8192, 0), {126}) == DjiReceiveResult::invalid_frame);
    CHECK(device.receive(1, dji_feedback(0x201, 1, 3000), {126}) == DjiReceiveResult::invalid_frame);
    CHECK(!device.reference_position(std::numeric_limits<float>::quiet_NaN(), {126}));
    CHECK(!device.reference_position(0, {140}));
    CHECK(device.encode_output(1) == -10000);
    CHECK(device.encode_output(-1) == 10000);
    CHECK(device.encode_output(0.25f) == -2500);
    CHECK(!device.encode_output(1.001f));
    CHECK(!device.encode_output(std::numeric_limits<float>::infinity()));
    device.reset();
    CHECK(device.state({0}).rotor.validity == Validity::unknown);
    CHECK(device.receive(1, dji_feedback(0x201, 1, 0), {0xfffffffdu}) == DjiReceiveResult::accepted);
    CHECK(device.reference_position(0, {0xfffffffdu}));
    CHECK(device.receive(1, dji_feedback(0x201, 2, 0), {2}) == DjiReceiveResult::accepted);
    CHECK(device.state({2}).position_rad.validity == Validity::valid);
    CHECK(device.state({17}).position_rad.validity == Validity::invalid);
    device.reset();
    CHECK(device.receive(1, dji_feedback(0x201, 7000, 1500), {0}) == DjiReceiveResult::accepted);
    CHECK(device.reference_position(0, {0}));
    for (unsigned i = 1; i <= 80; ++i) {
        CHECK(device.receive(1, dji_feedback(0x201, static_cast<std::uint16_t>((7000 + i * 1024) % 8192), 1500),
            {i * 5}) == DjiReceiveResult::accepted);
    }
    CHECK(device.state({400}).position_rad.validity == Validity::valid);
    CHECK(near(device.state({400}).position_rad.value, -std::numbers::pi_v<float>)); // 转子十圈 / 20, 反向.

    auto c = config;
    c.feedback_ttl = {100};
    DjiDevice ambiguous{c};
    CHECK(ambiguous.receive(1, dji_feedback(0x201, 0, 0), {0}) == DjiReceiveResult::accepted);
    CHECK(ambiguous.reference_position(0, {0}));
    CHECK(ambiguous.receive(1, dji_feedback(0x201, 0, 0), {20}) == DjiReceiveResult::accepted);
    CHECK(ambiguous.state({20}).position_rad.validity == Validity::invalid); // 可能转过半圈, 即使两帧读数相同.
    CHECK(ambiguous.reference_position(0, {20}));
    CHECK(ambiguous.receive(1, dji_feedback(0x201, 4096, 0), {21}) == DjiReceiveResult::accepted);
    CHECK(ambiguous.state({21}).position_rad.validity == Validity::invalid); // 不可信跳变不能参与累计.
    CHECK(ambiguous.state({21}).velocity_rad_s.validity == Validity::valid);

    c = config; c.bus = 0; CHECK(!DjiDevice{c}.configured());
    c = config; c.direction = 0; CHECK(!DjiDevice{c}.configured());
    c = config; c.device_id = 9; CHECK(!DjiDevice{c}.configured());
    c = config; c.mode = static_cast<DjiMode>(99); CHECK(!DjiDevice{c}.configured());
    c = config; c.rotor_per_axis = 0; CHECK(!DjiDevice{c}.configured());
    c = config; c.rotor_per_axis = std::numeric_limits<float>::infinity(); CHECK(!DjiDevice{c}.configured());
    c = config; c.feedback_ttl = {0}; CHECK(!DjiDevice{c}.configured());
    c = config; c.max_rotor_speed_rad_s = -1; CHECK(!DjiDevice{c}.configured());
    c = config; c.output_limit_raw = 16385; CHECK(!DjiDevice{c}.configured());
    c = config; c.mode = DjiMode::gm6020_current; c.device_id = 8; CHECK(!DjiDevice{c}.configured());
    c = config; c.rotor_per_axis = std::numeric_limits<float>::min();
    DjiDevice overflow{c};
    CHECK(overflow.receive(1, dji_feedback(0x201, 0, 600), {0}) == DjiReceiveResult::invalid_frame);
    c = config; c.mode = DjiMode::c610_current;
    DjiDevice c610{c};
    CHECK(c610.receive(1, dji_feedback(0x201, 0, 0), {0}) == DjiReceiveResult::accepted);
    CHECK(!c610.state({0}).rotor.value.temperature_c);

    std::array layout{config, config};
    CHECK(!valid_dji_layout(layout));
    layout[1].bus = 2; CHECK(valid_dji_layout(layout));
    layout[1].bus = 1; layout[1].device_id = 2; CHECK(valid_dji_layout(layout));
    layout[0].device_id = 5;
    layout[1].mode = DjiMode::gm6020_current; layout[1].device_id = 1;
    CHECK(!valid_dji_layout(layout)); // C620 ID5 与 GM6020 ID1 反馈 ID 冲突.
    layout[1].mode = DjiMode::gm6020_voltage; layout[1].device_id = 2;
    CHECK(!valid_dji_layout(layout)); // 反馈不同, 但命令 0x1ff 跨组号共享.
    CHECK(!valid_dji_layout({}));
}

void dji_group_contracts() {
    using namespace ec::drivers;
    using ec::transport::SendResult;
    RecordingCan bus;
    CHECK(DjiGroupResult{}.status == DjiGroupStatus::not_sent);
    DjiDevice a{{1, DjiMode::c610_current, 1, 1, -1, {15}, 100, 10000}};
    DjiDevice b{{1, DjiMode::c620_current, 2, 1, 1, {15}, 100, 15000}};
    DjiTxGroup group{bus, 1, 1, {5}, {&a, &b, nullptr, nullptr}};
    CHECK(group.configured());
    std::array<std::optional<DjiOutput>, 4> outputs{};
    CHECK(group.send({100}, outputs).status == DjiGroupStatus::missing_output);
    outputs[0] = DjiOutput{1, {100}};
    CHECK(group.send({100}, outputs).status == DjiGroupStatus::missing_output);
    outputs[1] = DjiOutput{0.8f, {95}};
    CHECK(group.send({100}, outputs).status == DjiGroupStatus::stale_output);
    outputs[1] = DjiOutput{0.8f, {101}};
    CHECK(group.send({100}, outputs).status == DjiGroupStatus::stale_output);
    outputs[1] = DjiOutput{std::numeric_limits<float>::quiet_NaN(), {100}};
    CHECK(group.send({100}, outputs).status == DjiGroupStatus::invalid_output);
    outputs[1] = DjiOutput{0.8f, {100}};
    outputs[2] = DjiOutput{0, {100}};
    CHECK(group.send({100}, outputs).status == DjiGroupStatus::invalid_output);
    CHECK(bus.calls == 0);
    outputs[2].reset();
    auto sent = group.send({100}, outputs);
    CHECK(sent.status == DjiGroupStatus::accepted && sent.tx_result == SendResult::accepted);
    CHECK(bus.calls == 1 && bus.last.id == 0x200 && bus.last.size == 8);
    const std::array<std::uint8_t, 8> expected{0xd8, 0xf0, 0x2e, 0xe0, 0, 0, 0, 0};
    CHECK(bus.last.data == expected); // -10000, +12000; C610 限值不能误用于整组 C620 槽位.
    bus.result = SendResult::busy;
    CHECK(group.send({100}, outputs).status == DjiGroupStatus::busy);
    outputs[0].reset();
    CHECK(!group.send({100}, outputs).tx_result && bus.calls == 2); // 不缓存/重发旧帧.
    outputs[0] = DjiOutput{0, {100}};
    bus.result = SendResult::error;
    CHECK(group.send({100}, outputs).tx_result == SendResult::error);
    bus.result = SendResult::invalid;
    CHECK(group.send({100}, outputs).status == DjiGroupStatus::error);
    bus.result = SendResult::accepted;
    outputs[0] = DjiOutput{0, {0xfffffffeu}};
    outputs[1] = DjiOutput{0, {0xfffffffeu}};
    CHECK(group.send({2}, outputs).status == DjiGroupStatus::accepted);
    CHECK(group.send({3}, outputs).status == DjiGroupStatus::stale_output);

    CHECK(!(DjiTxGroup{bus, 2, 1, {5}, {&a, &b, nullptr, nullptr}}.configured()));
    CHECK(!(DjiTxGroup{bus, 1, 2, {5}, {&a, &b, nullptr, nullptr}}.configured()));
    CHECK(!(DjiTxGroup{bus, 1, 1, {0}, {&a, &b, nullptr, nullptr}}.configured()));
    CHECK(!(DjiTxGroup{bus, 1, 1, {5}, {&a, &a, nullptr, nullptr}}.configured()));
    DjiTxGroup empty{bus, 1, 1, {5}, {}};
    CHECK(empty.send({0}, {}).status == DjiGroupStatus::invalid_config);
    DjiDevice gm2{{1, DjiMode::gm6020_current, 2, 1, 1, {15}, 100, 2000}};
    CHECK(!(DjiTxGroup{bus, 1, 1, {5}, {&a, &gm2, nullptr, nullptr}}.configured()));
    DjiDevice gm7{{1, DjiMode::gm6020_current, 7, 1, 1, {15}, 100, 2000}};
    DjiTxGroup gm_group{bus, 1, 2, {5}, {nullptr, nullptr, &gm7, nullptr}};
    CHECK(gm_group.configured());
    outputs = {};
    outputs[2] = DjiOutput{-1, {0}};
    CHECK(gm_group.send({0}, outputs).status == DjiGroupStatus::accepted);
    CHECK(bus.last.id == 0x2fe && bus.last.data[4] == 0xf8 && bus.last.data[5] == 0x30);
    CHECK(bus.last.data[6] == 0 && bus.last.data[7] == 0);
}

void device_watch_contracts() {
    using namespace ec::drivers;
    DeviceWatch<2> watch;
    CHECK(watch.note(1, 0x201, {100}));
    CHECK(watch.note(2, 0x201, {101})); // 不同总线
    CHECK(!watch.note(1, 0x202, {102}));
    CHECK(watch.dropped_ids() == 1);
    CHECK(watch.online(1, 0x201, {120}, {50}));
    CHECK(!watch.online(1, 0x201, {150}, {50}));
    CHECK(watch.get(2, 0x201)->frames == 1);
    CHECK(watch.note(1, 0x201, {130}));
    CHECK(watch.get(1, 0x201)->frames == 2);
    CHECK(!watch.note(0, 0x201, {1}));
    CHECK(!watch.note(1, 0x800, {1}));
}
} // 测试辅助函数

int main() {
    time_contracts();
    periodic_contracts();
    gate_and_channels();
    concurrent_channels();
    pid_contracts();
    dji_contracts();
    dji_device_contracts();
    dji_group_contracts();
    byte_pipe_contracts();
    dr16_contracts();
    board_observation_contracts();
    bmi088_contracts();
    bmi088_device_contracts();
    device_watch_contracts();
    std::puts("core contracts passed");
    return 0;
}
