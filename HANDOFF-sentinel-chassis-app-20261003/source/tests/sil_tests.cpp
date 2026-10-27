#include "sil_models.hpp"
#include "ec/control/velocity_loop.hpp"
#include "ec/drivers/bmi088.hpp"
#include "ec/drivers/dr16.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/transport/byte_pipe.hpp"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <fcntl.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "SIL line %d: %s\n", line, expression);
        std::_Exit(3); // 非零退出且不触发 CRT 交互弹窗, ctest 要求非交互
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)
using ec::control::VelocityStatus;
using ec::transport::SendResult;
using Lock = ec::host::MutexCriticalSection;
constexpr ec::control::VelocityLoopConfig config{{0.03f, 0.25f, 0, 1, -1, 1}, {15}, 80, 0.02f};
constexpr ec::Lifetime command_ttl{40};

struct Telemetry {
    ec::Tick at{};
    ec::control::VelocityResult control{};
    SendResult tx{SendResult::accepted};
    ec::Sample<float> feedback{};
};

// PC 命令源只使用进程内类型接口; 没有臆造或冻结上位机二进制协议/握手字段.
// Grant 由本地应用自检和明确授权后交给夹具, 绝不从模拟线协议反序列化.
struct PcPeer {
    ec::CommandMailbox<float, Lock>& commands;
    ec::LatestValue<Telemetry, Lock>& telemetry;
    ec::Grant grant{};
    void send(float target, ec::Tick now) { commands.publish({target, now, command_ttl, grant}); }
    std::optional<Telemetry> read() const { return telemetry.read(); }
};

// 专用单电机虚拟台架. 故障时发送零电流只是一项明确选择的仿真动作,
// 不代表真实底盘、重力轴或发射机构的安全动作, 也不链接进板级观察固件.
struct Rig {
    ec::sil::Clock clock;
    ec::sil::CanLink command_bus{clock}, feedback_bus{clock};
    ec::sil::Motor motor;
    ec::drivers::DjiDevice device{{1, ec::drivers::DjiMode::c620_current, 1,
        1, 1, config.feedback_ttl, 100, ec::sil::Motor::raw_limit}};
    ec::drivers::DjiTxGroup sender{command_bus, 1, 1, {2}, {&device, nullptr, nullptr, nullptr}};
    Lock lock;
    ec::CommandMailbox<float, Lock> commands{lock};
    ec::Snapshot<float, Lock> feedback{lock};
    ec::LatestValue<Telemetry, Lock> telemetry{lock};
    ec::control::VelocityLoop loop{config};
    PcPeer peer{commands, telemetry};
    ec::control::VelocityResult last{};
    SendResult last_tx{SendResult::accepted};
    std::uint32_t bad_feedback{}, feedback_queue_full{};
    bool corrupt_feedback{false};
    float requested{};
    const char* name;
    std::FILE* csv;

    Rig(const char* scenario, std::FILE* trace, std::uint32_t start_ms = 0)
        : clock(start_ms), name(scenario), csv(trace) {}

    std::optional<ec::Grant> arm(ec::CommandSource source = ec::CommandSource::host,
                               bool authorized = true) {
        const auto state = feedback.read();
        const bool ready = state && state->fresh_at(clock.now(), config.feedback_ttl) &&
            std::isfinite(state->value);
        commands.clear();
        const auto grant = loop.arm(source, authorized, ready);
        if (grant) { peer.grant = *grant; }
        return grant;
    }

    void command(float target) {
        requested = target;
        peer.send(target, clock.now());
    }

    void step(std::uint32_t elapsed_ms = 1) {
        clock.advance(elapsed_ms);
        const auto now = clock.now();
        const float dt_s = static_cast<float>(elapsed_ms) * 0.001f;
        // 每条链路容量为 8, 消费循环有界; 到达时间而不是生产时间作为 CAN 接收戳.
        for (unsigned i = 0; i < 8; ++i) {
            const auto frame = command_bus.receive();
            if (!frame) { break; }
            motor.receive(*frame, now);
        }
        motor.step(now, dt_s);
        if (motor.powered && clock.elapsed_ms() % 5 == 0) {
            auto frame = motor.feedback();
            if (corrupt_feedback) { frame.size = 7; }
            if (feedback_bus.try_send(frame) != SendResult::accepted) { ++feedback_queue_full; }
        }
        for (unsigned i = 0; i < 8; ++i) {
            const auto frame = feedback_bus.receive();
            if (!frame) { break; }
            if (device.receive(1, *frame, now) != ec::drivers::DjiReceiveResult::accepted) {
                ++bad_feedback;
                continue;
            }
            feedback.publish(device.state(now).velocity_rad_s);
        }
        const auto state = feedback.read();
        last = loop.step(now, dt_s, commands.read_fresh(now), state);
        CHECK(!last.output || (std::isfinite(*last.output) && std::fabs(*last.output) <= 1));
        // 零输出是本夹具的显式故障动作, 正式组发送器不负责补零.
        std::array<std::optional<ec::drivers::DjiOutput>, 4> outputs{};
        outputs[0] = ec::drivers::DjiOutput{last.output.value_or(0), now};
        const auto sent = sender.send(now, outputs);
        CHECK(sent.tx_result);
        last_tx = *sent.tx_result;
        if (last_tx != SendResult::accepted) { loop.revoke(); }
        telemetry.publish({now, last, last_tx, state.value_or(ec::Sample<float>{})});
        if (csv) {
            CHECK(std::fprintf(csv, "%s,%llu,%u,%.6f,%.6f,%.6f,%d,%d,%.6f,%d,%d\n", name,
                static_cast<unsigned long long>(clock.elapsed_ms()), now.ms, requested,
                motor.speed_rad_s, state ? state->value : 0,
                state && state->fresh_at(now, config.feedback_ttl) ? 1 : 0,
                last.output.has_value() ? 1 : 0, last.output.value_or(0),
                static_cast<int>(last.status), static_cast<int>(last_tx)) >= 0);
        }
    }

    void run(unsigned ms, bool send_commands = true, float target = 20) {
        for (unsigned i = 0; i < ms; ++i) {
            if (send_commands && i % 5 == 0) { command(target); }
            step();
        }
    }

    void start() {
        CHECK(!arm()); // 没有反馈就不能通过夹具的本地自检
        run(10, false);
        CHECK(arm());
        run(100);
        CHECK(last.output && !motor.foreign_slot_command);
    }
};

void velocity_contracts() {
    ec::control::VelocityLoop loop{config};
    CHECK(loop.configured());
    CHECK(!loop.arm(ec::CommandSource::host, false, true));
    CHECK(!loop.arm(ec::CommandSource::host, true, false));
    CHECK(!loop.arm(ec::CommandSource::none, true, true));
    const ec::Sample<float> good{0, {100}, ec::Validity::valid};
    const auto command = [&] {
        const auto grant = loop.arm(ec::CommandSource::host, true, true);
        CHECK(grant);
        return ec::Command<float>{20, {100}, {40}, *grant};
    };
    auto target = command();
    CHECK(loop.step({100}, 0.001f, target, good).output);
    loop.revoke();
    CHECK(loop.step({100}, 0.001f, target, good).status == VelocityStatus::unauthorized);
    target = command();
    CHECK(loop.step({140}, 0.001f, target, ec::Sample<float>{0, {140}, ec::Validity::valid}).status == VelocityStatus::stale_command);
    target = command();
    CHECK(loop.step({99}, 0.001f, target, good).status == VelocityStatus::stale_command);
    target = command();
    CHECK(loop.step({115}, 0.001f, target, good).status == VelocityStatus::stale_feedback);
    target = command();
    CHECK(loop.step({100}, 0.001f, target, std::nullopt).status == VelocityStatus::no_feedback);
    target = command();
    CHECK(loop.step({100}, 0.001f, target, ec::Sample<float>{0, {100}, ec::Validity::invalid}).status == VelocityStatus::no_feedback);
    for (const float value : {81.0f, -81.0f, std::numeric_limits<float>::quiet_NaN()}) {
        target = command(); target.value = value;
        CHECK(loop.step({100}, 0.001f, target, good).status == VelocityStatus::invalid_target);
    }
    for (const float dt : {0.0f, -0.001f, 0.021f, std::numeric_limits<float>::infinity()}) {
        target = command();
        CHECK(loop.step({100}, dt, target, good).status == VelocityStatus::invalid_dt);
    }
    target = command();
    CHECK(loop.step({100}, 0.001f, target,
        ec::Sample<float>{std::numeric_limits<float>::quiet_NaN(), {100}, ec::Validity::valid}).status == VelocityStatus::controller_error);
    target = command(); target.value = 80;
    CHECK(loop.step({100}, 0.001f, target, good).output == 1); // 饱和但不溢出
    target = command(); target.value = 0;
    CHECK(loop.step({100}, 0.001f, target, good).output == 0); // 重授权清除积分
    auto bad = config; bad.feedback_ttl = {0};
    ec::control::VelocityLoop invalid{bad};
    CHECK(!invalid.configured() && !invalid.arm(ec::CommandSource::host, true, true));
    CHECK(invalid.step({100}, 0.001f, target, good).status == VelocityStatus::invalid_config);
    bad = config; bad.pid.output_max = 2;
    CHECK(!ec::control::VelocityLoop{bad}.configured());
    bad = config; bad.max_dt_s = 0;
    CHECK(!ec::control::VelocityLoop{bad}.configured());
    bad = config; bad.max_target_rad_s = std::numeric_limits<float>::infinity();
    CHECK(!ec::control::VelocityLoop{bad}.configured());
    std::puts("PASS velocity contracts: bounds, expiry, invalid input, authorization, reset");
}

void closed_loop(std::FILE* csv) {
    Rig rig{"tracking", csv};
    rig.start();
    rig.run(3000);
    CHECK(std::fabs(rig.motor.speed_rad_s - 20) < 0.3f);
    const auto report = rig.peer.read();
    CHECK(report && report->control.status == VelocityStatus::active && report->tx == SendResult::accepted);
    CHECK(report->feedback.fresh_at(rig.clock.now(), config.feedback_ttl));
    CHECK(std::fabs(report->feedback.value - rig.motor.speed_rad_s) < 0.2f);
    rig.run(3000, true, -15);
    CHECK(std::fabs(rig.motor.speed_rad_s + 15) < 0.3f);
    CHECK(rig.bad_feedback == 0 && rig.feedback_queue_full == 0 && !rig.motor.foreign_slot_command);
    std::puts("PASS closed loop: positive/negative tracking, real DJI codec, bounded output, PC telemetry");
}

void command_loss(std::FILE* csv) {
    Rig rig{"command_loss", csv}; rig.start();
    const auto old_grant = rig.peer.grant;
    rig.command(20);
    rig.run(39, false); CHECK(rig.last.output);
    rig.step(); CHECK(!rig.last.output); // 到期瞬间禁止输出
    rig.run(100); CHECK(!rig.last.output); // 任意新命令不能恢复旧授权
    CHECK(rig.arm());
    rig.peer.grant = old_grant;
    rig.command(20); rig.step();
    CHECK(rig.last.status == VelocityStatus::unauthorized);
    CHECK(rig.arm()); rig.run(100); CHECK(rig.last.output);
    // 遥控来源断链也只撤销, 不转成上位机授权.
    CHECK(rig.arm(ec::CommandSource::remote)); rig.run(50); CHECK(rig.last.output);
    rig.run(50, false); CHECK(!rig.last.output);
    rig.peer.grant = old_grant; rig.run(20); CHECK(!rig.last.output);
    CHECK(rig.arm(ec::CommandSource::host)); rig.run(20); CHECK(rig.last.output);
    std::puts("PASS command loss: deadline, no automatic takeover, stale grant rejected, explicit re-arm");
}

void local_revoke(std::FILE* csv) {
    Rig rig{"local_revoke", csv}; rig.start();
    rig.loop.revoke();
    rig.run(50); CHECK(!rig.last.output && rig.motor.applied == 0);
    CHECK(!rig.arm(ec::CommandSource::host, false));
    rig.run(20); CHECK(!rig.last.output);
    CHECK(rig.arm()); rig.run(20); CHECK(rig.last.output);
    std::puts("PASS local revoke: incoming commands cannot bypass explicit authorization");
}

void feedback_faults(std::FILE* csv) {
    Rig rig{"feedback_loss", csv}; rig.start();
    rig.feedback_bus.fault = ec::sil::CanFault::drop;
    const auto last_sample = rig.feedback.read(); CHECK(last_sample);
    while (rig.clock.now().ms - last_sample->sampled_at.ms < config.feedback_ttl.ms - 1) {
        rig.command(20); rig.step(); CHECK(rig.last.output);
    }
    rig.command(20); rig.step(); CHECK(rig.last.status == VelocityStatus::stale_feedback);
    rig.feedback_bus.fault = ec::sil::CanFault::none;
    rig.run(30); CHECK(!rig.last.output);
    CHECK(rig.arm()); rig.run(30); CHECK(rig.last.output);
    rig.corrupt_feedback = true;
    rig.run(30); CHECK(!rig.last.output && rig.bad_feedback > 0);
    rig.corrupt_feedback = false;
    rig.run(10); CHECK(rig.arm()); rig.run(20);
    rig.feedback_bus.delay_ms = 30;
    rig.run(20); CHECK(!rig.last.output);
    rig.run(100); CHECK(!rig.last.output); // 延迟帧恢复投递仍不能重新使能
    std::puts("PASS feedback faults: loss, malformed DLC, delayed arrival, recovery stays revoked");
}

void actuator_power_cycle(std::FILE* csv) {
    Rig rig{"actuator_power_cycle", csv}; rig.start();
    const auto old_grant = rig.peer.grant;
    rig.motor.power(false);
    rig.run(50); CHECK(!rig.last.output && !rig.arm());
    rig.motor.power(true); // MCU/时钟/授权代次均未重启
    rig.run(50); CHECK(!rig.last.output && rig.motor.applied == 0);
    CHECK(rig.peer.grant == old_grant);
    CHECK(rig.arm()); rig.run(50); CHECK(rig.last.output && rig.peer.grant != old_grant);
    std::puts("PASS actuator power cycle: feedback timeout, no old target revival, fresh authorization required");
}

void tx_and_clock_faults(std::FILE* csv) {
    for (const auto fault : {ec::sil::CanFault::busy, ec::sil::CanFault::error}) {
        Rig rig{"tx_fault", csv}; rig.start();
        rig.command_bus.fault = fault;
        rig.run(30); CHECK(!rig.last.output && rig.motor.applied == 0);
        CHECK(rig.last_tx == (fault == ec::sil::CanFault::busy ? SendResult::busy : SendResult::error));
        rig.command_bus.fault = ec::sil::CanFault::none;
        rig.run(20); CHECK(!rig.last.output);
        CHECK(rig.arm()); rig.run(20); CHECK(rig.last.output);
    }
    {
        Rig dropped{"tx_drop", csv}; dropped.start();
        dropped.command_bus.fault = ec::sil::CanFault::drop;
        dropped.run(100);
        // 有反馈不等于命令已应用. 当前协议没有执行确认, 不虚构对该故障的检测能力.
        CHECK(dropped.last.output && dropped.last_tx == SendResult::accepted);
        CHECK(dropped.motor.applied == 0 && dropped.command_bus.dropped > 0);
    }
    Rig rig{"tick_wrap", csv, 0xfffffff0u}; rig.start();
    rig.run(3000); CHECK(rig.clock.now().ms < 4000 && rig.last.output);
    CHECK(std::fabs(rig.motor.speed_rad_s - 20) < 0.3f);
    rig.command(20); rig.step(25); // 目标仍新鲜但超过允许控制步长
    CHECK(!rig.last.output && rig.last.status == VelocityStatus::invalid_dt);
    rig.run(10); CHECK(!rig.last.output);
    ec::sil::Clock clock;
    ec::sil::CanLink link{clock}; link.delay_ms = 5;
    const ec::transport::CanFrame frame{0x201, 8, {}};
    for (unsigned i = 0; i < 8; ++i) { CHECK(link.try_send(frame) == SendResult::accepted); }
    CHECK(link.try_send(frame) == SendResult::busy && !link.receive());
    clock.advance(5);
    for (unsigned i = 0; i < 8; ++i) { CHECK(link.receive()); }
    CHECK(!link.receive());
    CHECK(link.try_send({0x800, 8, {}}) == SendResult::invalid);
    link.fault = ec::sil::CanFault::drop;
    CHECK(link.try_send(frame) == SendResult::accepted && !link.receive() && link.dropped == 1);
    std::puts("PASS transport/clock: bounded queues, TX busy/error, accepted-but-dropped, tick wrap, excessive dt");
}

// 将生产管线与分帧器放入受控的 DMA 游标事件序列, 不宣称模拟了 STM32 寄存器或 HAL.
struct UartReplay {
    ec::transport::DmaBytePipe<64> pipe;
    ec::drivers::Dr16Stream parser;
    std::size_t dma{};
    std::optional<ec::drivers::Dr16Sample> last{};
    void event(bool idle) {
        const auto overflows = pipe.overflows();
        pipe.on_dma_write(dma);
        if (pipe.overflows() != overflows) { parser.reset(); last.reset(); }
        std::uint8_t tmp[64]{};
        const auto n = pipe.read(tmp);
        parser.push({tmp, n});
        if (idle) { last = parser.on_idle(); }
    }
    void write(std::span<const std::uint8_t> bytes) {
        for (auto byte : bytes) {
            pipe.dma_buffer()[dma] = byte;
            dma = (dma + 1) % 64;
            if (dma % 32 == 0) { event(false); }
        }
    }
    void error() {
        pipe.note_error(ec::transport::UartError::overrun);
        pipe.reset(); parser.reset(); dma = 0; last.reset();
    }
};

void input_replay() {
    const std::array<std::uint8_t, 18> center{0, 4, 32, 0, 1, 0x78, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
    UartReplay uart;
    uart.write({center.data() + 7, 11}); uart.event(true); CHECK(!uart.last);
    for (unsigned i = 0; i < 100; ++i) {
        uart.write(center);
        // 同一 DMA 游标下 HT/TC 与 IDLE 的先后变化不应重复消费.
        if (i % 2 == 0) { uart.event(false); }
        uart.event(true); CHECK(uart.last && uart.last->ch0 == 0);
        uart.event(false);
    }
    CHECK(uart.parser.accepted == 100);
    uart.write({center.data(), 8}); uart.event(false); uart.error();
    uart.write({center.data() + 8, 10}); uart.event(true); CHECK(!uart.last);
    uart.write(center); uart.event(true); CHECK(uart.last && uart.pipe.overrun() == 1);
    const std::uint8_t extra = 0xff;
    uart.write(center); uart.write({&extra, 1}); uart.event(true); CHECK(!uart.last);
    uart.write(center); uart.event(true); CHECK(uart.last);
    ec::transport::DmaBytePipe<64> overflow;
    overflow.on_dma_write(32); overflow.on_dma_write(0); // 两个半缓冲期间消费者未运行
    CHECK(overflow.overflows() == 1 && overflow.available() == 0);
    overflow.reset(); overflow.dma_buffer()[0] = 42; overflow.on_dma_write(1);
    std::uint8_t byte{}; CHECK(overflow.read({&byte, 1}) == 1 && byte == 42);

    // BMI088 合成 SPI 响应 -> 正式解析 -> 一致性快照 -> 时效判断.
    ec::sil::Clock clock{0xfffffff8u};
    Lock lock;
    ec::Snapshot<ec::drivers::Bmi088Sample, Lock> imu{lock};
    const std::uint8_t acc_rx[8]{0, 0, 0, 0, 0, 0, 0x55, 0x15}; // Z 约 1 g
    const std::uint8_t gyro_rx[7]{0, 1, 0, 0xff, 0xff, 0, 0};
    std::uint8_t acc[6]{}, gyro[6]{};
    CHECK(ec::drivers::parse_bmi088_accel_spi_read(acc_rx, acc));
    CHECK(ec::drivers::parse_bmi088_gyro_spi_read(gyro_rx, gyro));
    const auto a = ec::drivers::parse_bmi088_accel_6g(acc);
    const auto g = ec::drivers::parse_bmi088_gyro_2000(gyro);
    CHECK(a && g && std::fabs((*a)[2] - 9.8f) < 0.02f && (*g)[0] > 0 && (*g)[1] < 0);
    ec::drivers::Bmi088Sample value{{(*a)[0], (*a)[1], (*a)[2]}, {(*g)[0], (*g)[1], (*g)[2]},
        ec::drivers::parse_bmi088_temp_c(0, 0)};
    CHECK(!imu.read());
    imu.publish({value, clock.now(), ec::Validity::valid});
    clock.advance(9); CHECK(imu.read()->fresh_at(clock.now(), {10}));
    clock.advance(1); CHECK(!imu.read()->fresh_at(clock.now(), {10}));
    CHECK(!ec::drivers::parse_bmi088_accel_spi_read({acc_rx, 7}, acc));
    CHECK(!imu.read()->fresh_at(clock.now(), {10})); // 坏响应不续期
    imu.publish({value, clock.now(), ec::Validity::invalid});
    CHECK(!imu.read()->fresh_at(clock.now(), {10}));
    imu.publish({value, clock.now(), ec::Validity::valid});
    CHECK(imu.read()->fresh_at(clock.now(), {10}));
    std::puts("PASS input replay: UART cursor/idle/error/overflow, DR16 resync, IMU framing/staleness/recovery");
}
void grouped_motors() {
    using namespace ec::drivers;
    ec::sil::Clock clock;
    ec::sil::CanLink commands{clock}, feedback{clock};
    std::array<ec::sil::Motor, 4> motors{ec::sil::Motor{1}, ec::sil::Motor{2}, ec::sil::Motor{3}, ec::sil::Motor{4}};
    const std::array<DjiDeviceConfig, 4> configs{{
        {1, DjiMode::c620_current, 1, 1, 1, {15}, 100, 10000},
        {1, DjiMode::c620_current, 2, 2, -1, {15}, 100, 10000},
        {1, DjiMode::c620_current, 3, 4, 1, {15}, 100, 10000},
        {1, DjiMode::c620_current, 4, 8, -1, {15}, 100, 10000}}};
    CHECK(valid_dji_layout(configs));
    std::array<DjiDevice, 4> devices{DjiDevice{configs[0]}, DjiDevice{configs[1]}, DjiDevice{configs[2]}, DjiDevice{configs[3]}};
    DjiTxGroup sender{commands, 1, 1, {2}, {&devices[0], &devices[1], &devices[2], &devices[3]}};
    std::array<ec::control::VelocityLoop, 4> loops{ec::control::VelocityLoop{config},
        ec::control::VelocityLoop{config}, ec::control::VelocityLoop{config}, ec::control::VelocityLoop{config}};
    std::array<ec::Grant, 4> grants{};
    constexpr std::array<float, 4> targets{15, 10, -5, -3};
    for (std::size_t i = 0; i < motors.size(); ++i) {
        CHECK(devices[i].receive(1, motors[i].feedback(), clock.now()) == DjiReceiveResult::accepted);
        CHECK(devices[i].reference_position(0, clock.now())); // 虚拟台架已知的初始位置, 不是实车零位.
        const auto grant = loops[i].arm(ec::CommandSource::host, true,
            devices[i].state(clock.now()).velocity_rad_s.fresh_at(clock.now(), config.feedback_ttl));
        CHECK(grant);
        grants[i] = *grant;
    }
    std::array<std::optional<DjiOutput>, 4> outputs{};
    for (unsigned tick = 0; tick < 6000; ++tick) {
        clock.advance(1);
        const auto now = clock.now();
        if (const auto frame = commands.receive()) {
            for (auto& motor : motors) { motor.receive(*frame, now); }
        }
        for (auto& motor : motors) {
            motor.step(now, 0.001f);
            if (now.ms % 5 == 0) { CHECK(feedback.try_send(motor.feedback()) == SendResult::accepted); }
        }
        for (unsigned n = 0; n < 8; ++n) {
            const auto frame = feedback.receive();
            if (!frame) { break; }
            unsigned recipients = 0;
            for (auto& device : devices) {
                if (device.receive(1, *frame, now) == DjiReceiveResult::accepted) { ++recipients; }
            }
            CHECK(recipients == 1);
        }
        for (std::size_t i = 0; i < loops.size(); ++i) {
            const ec::Command<float> target{targets[i], now, command_ttl, grants[i]};
            const auto result = loops[i].step(now, 0.001f, target, devices[i].state(now).velocity_rad_s);
            CHECK(result.status == VelocityStatus::active && result.output);
            outputs[i] = DjiOutput{*result.output, now};
        }
        CHECK(sender.send(now, outputs).status == DjiGroupStatus::accepted);
    }
    for (std::size_t i = 0; i < devices.size(); ++i) {
        const auto state = devices[i].state(clock.now());
        CHECK(std::fabs(state.velocity_rad_s.value - targets[i]) < 0.2f);
        CHECK(state.position_rad.validity == ec::Validity::valid);
        CHECK(state.position_rad.value * targets[i] > 0);
        CHECK(std::fabs(motors[i].speed_rad_s * configs[i].direction / configs[i].rotor_per_axis - targets[i]) < 0.2f);
    }
    CHECK(commands.receive()); // 取走最后一次已接受但未被模型应用的帧.
    CHECK(!commands.receive());
    loops[1].revoke();
    const auto rejected = loops[1].step(clock.now(), 0.001f,
        ec::Command<float>{targets[1], clock.now(), command_ttl, grants[1]},
        devices[1].state(clock.now()).velocity_rad_s);
    CHECK(!rejected.output && rejected.status == VelocityStatus::unauthorized);
    outputs[1].reset();
    CHECK(sender.send(clock.now(), outputs).status == DjiGroupStatus::missing_output);
    CHECK(!commands.receive());
    CHECK(motors[1].applied != 0); // 拒发不是主动停车; 应用仍须选择经确认的故障动作.
    for (auto& output : outputs) { output = DjiOutput{0, clock.now()}; } // 本虚拟台架显式选择全组零请求.
    CHECK(sender.send(clock.now(), outputs).status == DjiGroupStatus::accepted);
    const auto zero = commands.receive();
    CHECK(zero);
    for (auto& motor : motors) { motor.receive(*zero, clock.now()); CHECK(motor.applied == 0); }
    std::puts("PASS grouped motors: shared production devices, four slots, ratios/directions, explicit fault action");
}

// MinGW 所用 CRT 不一定支持 fopen 的 C11 "x" 模式, 用系统排他创建保证不覆盖.
std::FILE* create_csv(const char* path) {
#ifdef _WIN32
    const int fd = _open(path, _O_WRONLY | _O_CREAT | _O_EXCL | _O_TEXT, _S_IREAD | _S_IWRITE);
    if (fd == -1) { return nullptr; }
    auto* file = _fdopen(fd, "w");
    if (!file) { const int error = errno; _close(fd); errno = error; }
#else
    const int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
    if (fd == -1) { return nullptr; }
    auto* file = fdopen(fd, "w");
    if (!file) { const int error = errno; close(fd); errno = error; }
#endif
    return file;
}
} // 主机 SIL 场景

int main(int argc, char** argv) {
    std::FILE* csv = nullptr;
    if (argc == 3 && std::strcmp(argv[1], "--csv") == 0) {
        // 使用排他创建, 不覆盖既有验收数据.
        csv = create_csv(argv[2]);
        if (!csv) { std::perror("cannot create CSV"); return 2; }
        CHECK(std::fprintf(csv, "scenario,elapsed_ms,tick_ms,target_rad_s,model_speed_rad_s,feedback_rad_s,feedback_fresh,output_valid,normalized_output,status,tx_result\n") >= 0);
    } else if (argc != 1) {
        std::fprintf(stderr, "usage: ec_sil_tests [--csv NEW_FILE]\n");
        return 2;
    }
    velocity_contracts();
    grouped_motors();
    closed_loop(csv);
    command_loss(csv);
    local_revoke(csv);
    feedback_faults(csv);
    actuator_power_cycle(csv);
    tx_and_clock_faults(csv);
    input_replay();
    if (csv && std::fclose(csv) != 0) { std::perror("cannot close CSV"); return 2; }
    std::puts("SIL contracts passed; synthetic model only, no hardware or robot safety acceptance");
    return 0;
}
