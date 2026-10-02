// M2 轴门面与组执行契约: 三模式、使能/停用与参考生命周期、方向与传动、同组发送次数.
// 全部使用假 CAN 与注入反馈, 不连接硬件, 也不证明任何实车参数稳定.
#include "ec/control/current_scale.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/motion/dji_axis.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3); // 非零退出且不触发 CRT 交互弹窗, ctest 要求非交互
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::Tick;
using ec::Validity;
using ec::control::ArcMode;
using ec::control::ControlMode;
using ec::control::CurrentScale;
using ec::drivers::DjiDevice;
using ec::drivers::DjiGroupStatus;
using ec::drivers::DjiMode;
using ec::drivers::DjiOutput;
using ec::drivers::DjiTxGroup;
using ec::motion::AxisStatus;
using ec::motion::DjiAxis;
using ec::motion::DjiAxisConfig;

bool near(float a, float b, float tolerance = 1e-5f) { return std::fabs(a - b) <= tolerance; }

struct FakeCan final : ec::transport::CanTx {
    ec::transport::SendResult result{ec::transport::SendResult::accepted};
    ec::transport::CanFrame last{};
    unsigned calls{};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        ++calls;
        last = frame;
        return result;
    }
};

ec::transport::CanFrame dji_feedback(std::uint16_t id, std::uint16_t encoder, std::int16_t rpm,
                                    std::uint8_t temperature = 25) {
    const auto speed = static_cast<std::uint16_t>(rpm);
    return {id, 8, {static_cast<std::uint8_t>(encoder >> 8), static_cast<std::uint8_t>(encoder),
        static_cast<std::uint8_t>(speed >> 8), static_cast<std::uint8_t>(speed), 0, 0, temperature, 0}};
}

std::int16_t slot_raw(const ec::transport::CanFrame& frame, std::size_t index) {
    return static_cast<std::int16_t>(
        (static_cast<unsigned>(frame.data[2 * index]) << 8) | frame.data[2 * index + 1]);
}

// GM6020 电流模式 ±16384 = ±3 A; 设备 raw 编码上限取协议满量程, 使归一化基准与协议一致.
constexpr float gm6020_full_scale_a = 3.0f;
constexpr CurrentScale gm6020_scale{gm6020_full_scale_a, 16384.0f};
constexpr float yaw_current_limit_a = 3000.0f * gm6020_full_scale_a / 16384.0f; // 0.54931641

ec::drivers::DjiDeviceConfig yaw_device_config() {
    return {1, DjiMode::gm6020_current, 1, 1, 1, {50}, 60, 16384};
}

// 旧主 yaw 参数换算到"电流上限分数"域: 速度 1.3941973/(rad/s), 位置 7.5 rad/s per rad.
DjiAxisConfig yaw_axis_config() {
    const float raw_limit = ec::control::amps_to_raw(gm6020_scale, yaw_current_limit_a);
    const float kp_amp = 73.0f * (180.0f / std::numbers::pi_v<float>) * gm6020_full_scale_a / 16384.0f;
    return {yaw_current_limit_a,
        {ec::control::gain_amp_to_normalized(gm6020_scale, raw_limit, kp_amp), 0.0f, 0.0f, 1.0f, -1.0f, 1.0f},
        {7.5f, 0.0f, 0.0f, 0.0f, -3.4906585f, 3.4906585f},
        ArcMode::continuous, {500}, 3.4906585f, 0.02f, 0.0f};
}

constexpr float yaw_current_ratio = yaw_current_limit_a / gm6020_full_scale_a; // 0.18310547

void axis_config_contracts() {
    DjiDevice yaw{yaw_device_config()};
    CHECK(yaw.configured());
    DjiAxis axis{yaw_axis_config(), yaw};
    CHECK(axis.configured());
    // 默认不使能, 且尚未选定目标语义.
    CHECK(!axis.enabled());
    CHECK(axis.mode() == ControlMode::none);
    CHECK(axis.tick({0}, 0.001f).status == AxisStatus::idle);
    CHECK(!axis.set_velocity(0.0f, {0}));
    CHECK(!axis.set_current(0.0f, {0}));
    CHECK(!axis.set_position(0.0f, {0}));

    auto bad = yaw_axis_config();
    for (const float limit : {0.0f, -0.1f, std::numeric_limits<float>::quiet_NaN()}) {
        bad.current_limit_a = limit;
        CHECK(!DjiAxis{bad, yaw}.configured());
    }
    // 电流上限超过设备编码上限对应电流会让归一化输出越过编码边界.
    bad = yaw_axis_config();
    bad.current_limit_a = 4.0f;
    CHECK(!DjiAxis{bad, yaw}.configured());
    bad.current_limit_a = gm6020_full_scale_a;
    CHECK(DjiAxis{bad, yaw}.configured()); // 恰好等于满量程是允许的上界
    auto bad_ttl = yaw_axis_config();
    bad_ttl.command_ttl = {0};
    CHECK(!DjiAxis{bad_ttl, yaw}.configured());
    auto bad_speed = yaw_axis_config();
    bad_speed.max_velocity_rad_s = std::numeric_limits<float>::quiet_NaN();
    CHECK(!DjiAxis{bad_speed, yaw}.configured());
    // 电压模式没有电流上限语义, 不进入本轴.
    DjiDevice voltage{{1, DjiMode::gm6020_voltage, 1, 1, 1, {50}, 60, 25000}};
    CHECK(voltage.configured());
    CHECK(!DjiAxis{yaw_axis_config(), voltage}.configured());
    // 设备编码上限 3000 raw 只对应 0.549 A, 无法表示 1 A 的轴侧上限.
    DjiDevice narrow{{1, DjiMode::gm6020_current, 1, 1, 1, {50}, 60, 3000}};
    auto wide = yaw_axis_config();
    wide.current_limit_a = 1.0f;
    CHECK(!DjiAxis{wide, narrow}.configured());
    CHECK(near(ec::control::raw_to_amps(gm6020_scale, 3000.0f), yaw_current_limit_a, 1e-6f));

    CHECK(ec::drivers::dji_protocol(DjiMode::gm6020_current)->limit == 16384);
    CHECK(near(*ec::drivers::dji_current_amp_full_scale(DjiMode::c610_current), 10.0f));
    CHECK(near(*ec::drivers::dji_current_amp_full_scale(DjiMode::c620_current), 20.0f));
    CHECK(near(*ec::drivers::dji_current_amp_full_scale(DjiMode::gm6020_current), 3.0f));
    CHECK(!ec::drivers::dji_current_amp_full_scale(DjiMode::gm6020_voltage));
    CHECK(!ec::drivers::dji_current_amp_full_scale(static_cast<DjiMode>(99)));
}

void axis_current_mode_contracts() {
    FakeCan bus;
    DjiDevice yaw{yaw_device_config()};
    DjiAxis axis{yaw_axis_config(), yaw};
    DjiTxGroup group{bus, 1, 1, {20}, {&yaw, nullptr, nullptr, nullptr}};
    CHECK(group.configured());

    // 未使能或模式不匹配时不能建立目标.
    CHECK(!axis.set_current(0.1f, {100}));
    CHECK(axis.enable(ControlMode::none, ec::CommandSource::remote, true, true) == std::nullopt);
    CHECK(axis.enable(ControlMode::current, ec::CommandSource::remote, false, true) == std::nullopt);
    CHECK(axis.enable(ControlMode::current, ec::CommandSource::remote, true, false) == std::nullopt);
    const auto grant = axis.enable(ControlMode::current, ec::CommandSource::remote, true, true);
    CHECK(grant && axis.enabled() && axis.mode() == ControlMode::current);
    CHECK(!axis.set_velocity(0.5f, {100}));
    CHECK(!axis.set_position(0.5f, {100}));
    // 越界目标被拒绝且不改变已有目标.
    CHECK(axis.set_current(0.3f, {100}));
    CHECK(!axis.set_current(yaw_current_limit_a + 0.01f, {100}));
    CHECK(!axis.set_current(std::numeric_limits<float>::quiet_NaN(), {100}));
    const auto active = axis.tick({100}, 0.001f);
    CHECK(active.status == AxisStatus::active && active.target == 0.3f);
    CHECK(near(*active.normalized_output, 0.3f / gm6020_full_scale_a));
    CHECK(axis.output().has_value());
    // 开环电流模式不需要反馈: 一帧 CAN 都没收到也能产生输出.
    CHECK(yaw.state({100}).velocity_rad_s.validity == Validity::unknown);
    // 组发送每次 tick 一帧, 归一化 0.1 编码为 1638 raw.
    CHECK(group.send({100}, {axis.output(), std::nullopt, std::nullopt, std::nullopt}).status ==
        DjiGroupStatus::accepted);
    CHECK(bus.calls == 1 && bus.last.id == 0x1fe);
    CHECK(slot_raw(bus.last, 0) == 1638 && slot_raw(bus.last, 1) == 0);
    CHECK(std::lround(*active.normalized_output * 16384.0f) == 1638);

    // 目标不刷新即失效: 严格小于 TTL, 到期即退出使能并撤销授权.
    CHECK(axis.tick({599}, 0.001f).status == AxisStatus::active);
    const auto expired = axis.tick({600}, 0.001f);
    CHECK(expired.status == AxisStatus::stale_target && !expired.normalized_output);
    CHECK(!axis.enabled() && !axis.output().has_value());
    CHECK(group.send({600}, {axis.output(), std::nullopt, std::nullopt, std::nullopt}).status ==
        DjiGroupStatus::missing_output);
    CHECK(bus.calls == 1); // 缺输出时不补零, 也不重发旧帧
    // 重新使能后必须重新给目标, 旧目标不因链路恢复而有效.
    CHECK(axis.enable(ControlMode::current, ec::CommandSource::remote, true, true));
    CHECK(axis.tick({601}, 0.001f).status == AxisStatus::no_target);
    CHECK(!axis.enabled());
}

void axis_velocity_mode_contracts() {
    DjiDevice yaw{yaw_device_config()};
    auto config = yaw_axis_config();
    // 纯积分便于观察"重复设目标不重置控制器".
    config.velocity_pid = {0.0f, 0.5f, 0.0f, 1.0f, -1.0f, 1.0f};
    DjiAxis axis{config, yaw};

    CHECK(yaw.receive(1, dji_feedback(0x205, 100, 0), {100}) == ec::drivers::DjiReceiveResult::accepted);
    CHECK(axis.enable(ControlMode::velocity, ec::CommandSource::remote, true, true));
    CHECK(!axis.set_current(0.1f, {100}));
    CHECK(!axis.set_velocity(config.max_velocity_rad_s + 0.1f, {100}));
    CHECK(axis.set_velocity(1.0f, {100}));
    float previous = 0.0f;
    for (Tick now{100}; now.ms <= 109; now.ms = now.ms + 1) {
        CHECK(yaw.receive(1, dji_feedback(0x205, 100, 0), now) == ec::drivers::DjiReceiveResult::accepted);
        const auto result = axis.tick(now, 0.001f);
        CHECK(result.status == AxisStatus::active);
        CHECK(axis.set_velocity(1.0f, now)); // 每周期重新下发目标
        CHECK(*result.normalized_output > previous);
        previous = *result.normalized_output;
    }
    // 10 个周期 × 0.5 × 1.0 × 1 ms = 0.005 电流上限分数 → × current_ratio.
    CHECK(near(previous, 0.005f * yaw_current_ratio, 1e-6f));
    CHECK(near(previous, 0.005f * yaw_current_limit_a / gm6020_full_scale_a, 1e-6f));

    // 停用再使能清控制器历史, 位置参考不受影响.
    axis.disable();
    CHECK(!axis.enabled() && !axis.output().has_value());
    CHECK(axis.tick({110}, 0.001f).status == AxisStatus::idle);
    CHECK(axis.enable(ControlMode::velocity, ec::CommandSource::remote, true, true));
    CHECK(axis.set_velocity(1.0f, {110}));
    const auto restarted = axis.tick({110}, 0.001f);
    CHECK(restarted.status == AxisStatus::active);
    CHECK(near(*restarted.normalized_output, previous / 10.0f, 1e-6f));

    // 反馈过期与命令时效分开判定: 设备已把过期样本标为 invalid, 轴报 no_feedback.
    CHECK(yaw.receive(1, dji_feedback(0x205, 100, 0), {111}) == ec::drivers::DjiReceiveResult::accepted);
    CHECK(axis.set_velocity(1.0f, {111}));
    CHECK(axis.tick({161}, 0.001f).status == AxisStatus::no_feedback);
    CHECK(!axis.enabled());
    CHECK(axis.enable(ControlMode::velocity, ec::CommandSource::remote, true, true));
    CHECK(yaw.receive(1, dji_feedback(0x205, 100, 0), {162}) == ec::drivers::DjiReceiveResult::accepted);
    CHECK(axis.tick({162}, 0.001f).status == AxisStatus::no_target);
}

void axis_position_mode_contracts() {
    DjiDevice yaw{yaw_device_config()};
    auto config = yaw_axis_config();
    // 位置环无效时不提供位置模式, 但速度模式仍可用.
    auto no_position = config;
    no_position.position_pid = {};
    DjiAxis limited{no_position, yaw};
    CHECK(limited.configured());
    CHECK(limited.enable(ControlMode::velocity, ec::CommandSource::remote, true, true));
    CHECK(limited.enable(ControlMode::position, ec::CommandSource::remote, true, true) == std::nullopt);
    CHECK(!limited.set_position(1.0f, {0}));

    DjiAxis axis{config, yaw};
    CHECK(yaw.receive(1, dji_feedback(0x205, 0, 0), {100}) == ec::drivers::DjiReceiveResult::accepted);
    CHECK(axis.enable(ControlMode::position, ec::CommandSource::remote, true, true));
    // 没有参考就没有可用的累计位置: 目标已下发但反馈不可用.
    CHECK(!yaw.state({100}).position_rad.fresh_at({100}, {50}));
    CHECK(axis.set_position(1.0f, {100}));
    CHECK(axis.tick({100}, 0.001f).status == AxisStatus::no_feedback);
    // 参考显式建立, 与使能/停用分离.
    CHECK(axis.reference_position(0.0f, {101}));
    CHECK(axis.enable(ControlMode::position, ec::CommandSource::remote, true, true));
    CHECK(axis.set_position(1.0f, {101}));
    const auto saturated = axis.tick({101}, 0.001f);
    CHECK(saturated.status == AxisStatus::active);
    // 位置误差 1 rad → 速度目标 7.5 被限幅到 3.4906585, 内环再限幅到 1.
    CHECK(near(*saturated.normalized_output, yaw_current_ratio));
    CHECK(axis.set_position(0.01f, {102}));
    const auto linear = axis.tick({102}, 0.001f);
    // 误差 0.01 rad → 0.075 rad/s; 内环 1.3941973 × 0.075.
    CHECK(near(*linear.normalized_output, 1.3941973f * 0.075f * yaw_current_ratio, 1e-6f));
    // 位置反馈过期: 参考随之失效, 需要重新显式建立.
    CHECK(axis.tick({152}, 0.001f).status == AxisStatus::no_feedback);
    CHECK(!yaw.state({152}).position_rad.fresh_at({152}, {50}));
    CHECK(!yaw.reference_position(0.0f, {152}));

    // 停用再使能不重置有效参考: 累计位置继续, 而不是每次开始都归零.
    CHECK(yaw.receive(1, dji_feedback(0x205, 0, 0), {153}) == ec::drivers::DjiReceiveResult::accepted);
    CHECK(yaw.reference_position(2.5f, {153}));
    CHECK(yaw.receive(1, dji_feedback(0x205, 100, 0), {154}) == ec::drivers::DjiReceiveResult::accepted);
    const float accumulated = yaw.state({154}).position_rad.value;
    CHECK(near(accumulated, 2.5f + 100.0f * (2.0f * std::numbers::pi_v<float>) / 8192.0f, 1e-5f));
    axis.disable();
    CHECK(axis.enable(ControlMode::position, ec::CommandSource::remote, true, true));
    CHECK(yaw.receive(1, dji_feedback(0x205, 100, 0), {155}) == ec::drivers::DjiReceiveResult::accepted);
    CHECK(yaw.state({155}).position_rad.validity == Validity::valid);
    CHECK(near(yaw.state({155}).position_rad.value, accumulated, 1e-6f));
    CHECK(axis.set_position(accumulated, {155}));
    CHECK(axis.tick({155}, 0.001f).status == AxisStatus::active);
    // 单圈绝对角始终可显示, 不需要参考.
    CHECK(yaw.state({155}).rotor.value.rotor_angle_rad >= 0.0f);
    CHECK(yaw.state({155}).rotor.value.rotor_angle_rad < 2.0f * std::numbers::pi_v<float>);
}

void axis_direction_and_ratio_contracts() {
    // 反向 + 19 减速比: 反馈除以传动比并取反, 输出经同一方向取反.
    DjiDevice geared{{1, DjiMode::c620_current, 1, 19, -1, {50}, 200, 16384}};
    auto config = yaw_axis_config();
    config.current_limit_a = 1.0f;
    config.velocity_pid = {1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f};
    DjiAxis axis{config, geared};
    CHECK(axis.configured());
    CHECK(geared.receive(1, dji_feedback(0x201, 0, 600), {100}) == ec::drivers::DjiReceiveResult::accepted);
    const float rotor = 600.0f * 2.0f * std::numbers::pi_v<float> / 60.0f;
    CHECK(near(geared.state({100}).velocity_rad_s.value, -rotor / 19.0f, 1e-4f));
    CHECK(axis.enable(ControlMode::velocity, ec::CommandSource::remote, true, true));
    CHECK(axis.set_velocity(1.0f, {100}));
    const auto result = axis.tick({100}, 0.001f);
    CHECK(result.status == AxisStatus::active);
    const float error = 1.0f + rotor / 19.0f;
    CHECK(near(*result.normalized_output, std::fmin(error, 1.0f) * config.current_limit_a / 20.0f));
    FakeCan bus;
    DjiTxGroup group{bus, 1, 1, {20}, {&geared, nullptr, nullptr, nullptr}};
    CHECK(group.send({100}, {axis.output(), std::nullopt, std::nullopt, std::nullopt}).status ==
        DjiGroupStatus::accepted);
    CHECK(slot_raw(bus.last, 0) == -static_cast<std::int16_t>(std::lround(*result.normalized_output * 16384.0f)));
    CHECK(slot_raw(bus.last, 0) < 0);
}

void axis_four_axis_group_contracts() {
    FakeCan bus;
    DjiDevice a1{{1, DjiMode::c620_current, 1, 1, 1, {50}, 200, 16384}};
    DjiDevice a2{{1, DjiMode::c620_current, 2, 1, 1, {50}, 200, 16384}};
    DjiDevice a3{{1, DjiMode::c620_current, 3, 1, 1, {50}, 200, 16384}};
    DjiDevice a4{{1, DjiMode::c620_current, 4, 1, 1, {50}, 200, 16384}};
    DjiTxGroup group{bus, 1, 1, {20}, {&a1, &a2, &a3, &a4}};
    CHECK(group.configured());
    auto config = yaw_axis_config();
    config.current_limit_a = 1.0f; // ±1 A 轴侧上限, C620 满量程 ±20 A
    DjiAxis x1{config, a1};
    DjiAxis x2{config, a2};
    DjiAxis x3{config, a3};
    DjiAxis x4{config, a4};
    for (DjiAxis* axis : {&x1, &x2, &x3, &x4}) {
        CHECK(axis->enable(ControlMode::current, ec::CommandSource::remote, true, true));
    }
    struct Target { DjiAxis* axis; float current; };
    const Target targets[4]{{&x1, 0.5f}, {&x2, -1.0f}, {&x3, 1.0f}, {&x4, 0.25f}};
    for (Tick now{100}; now.ms <= 103; now.ms = now.ms + 1) {
        std::array<std::optional<DjiOutput>, 4> outputs{};
        for (std::size_t i = 0; i < 4; ++i) {
            CHECK(targets[i].axis->set_current(targets[i].current, now));
            CHECK(targets[i].axis->tick(now, 0.001f).status == AxisStatus::active);
            outputs[i] = targets[i].axis->output();
        }
        CHECK(group.send(now, outputs).status == DjiGroupStatus::accepted);
    }
    CHECK(bus.calls == 4); // 四轴同组每 tick 只发送一次
    // 每槽按自己的目标独立编码, 没有互相覆盖.
    CHECK(slot_raw(bus.last, 0) == 410);   // 0.5 / 20 × 16384
    CHECK(slot_raw(bus.last, 1) == -819);  // -1.0 / 20 × 16384
    CHECK(slot_raw(bus.last, 2) == 819);
    CHECK(slot_raw(bus.last, 3) == 205);

    // 失能一轴: 缺输出整组拒发, 装配必须显式提交零才能继续.
    x2.disable();
    CHECK(x1.set_current(0.5f, {110}));
    CHECK(x3.set_current(1.0f, {110}));
    CHECK(x4.set_current(0.25f, {110}));
    CHECK(x1.tick({110}, 0.001f).status == AxisStatus::active);
    CHECK(x3.tick({110}, 0.001f).status == AxisStatus::active);
    CHECK(x4.tick({110}, 0.001f).status == AxisStatus::active);
    std::array<std::optional<DjiOutput>, 4> outputs{x1.output(), x2.output(), x3.output(), x4.output()};
    CHECK(!outputs[1].has_value());
    CHECK(group.send({110}, outputs).status == DjiGroupStatus::missing_output);
    CHECK(bus.calls == 4);
    outputs[1] = DjiOutput{0.0f, {110}}; // 显式零是装配动作, 不代表该轴已停止
    CHECK(group.send({110}, outputs).status == DjiGroupStatus::accepted);
    CHECK(bus.calls == 5 && slot_raw(bus.last, 1) == 0);
    // busy 不排队重发旧帧.
    bus.result = ec::transport::SendResult::busy;
    CHECK(x1.set_current(0.5f, {111}));
    CHECK(x1.tick({111}, 0.001f).status == AxisStatus::active);
    outputs[0] = x1.output();
    CHECK(group.send({111}, outputs).status == DjiGroupStatus::busy && bus.calls == 6);

    // GM6020 ID7 在第二组, 第四槽保持零; 没有 ID8.
    bus.result = ec::transport::SendResult::accepted;
    DjiDevice gm7{{1, DjiMode::gm6020_current, 7, 1, 1, {50}, 200, 16384}};
    DjiTxGroup gm_group{bus, 1, 2, {20}, {nullptr, nullptr, &gm7, nullptr}};
    CHECK(gm_group.configured());
    DjiAxis gm_axis{yaw_axis_config(), gm7};
    CHECK(gm_axis.enable(ControlMode::current, ec::CommandSource::remote, true, true));
    CHECK(gm_axis.set_current(0.3f, {120}));
    CHECK(gm_axis.tick({120}, 0.001f).status == AxisStatus::active);
    CHECK(gm_group.send({120}, {std::nullopt, std::nullopt, gm_axis.output(), std::nullopt}).status ==
        DjiGroupStatus::accepted);
    CHECK(bus.last.id == 0x2fe);
    CHECK(slot_raw(bus.last, 2) == 1638 && slot_raw(bus.last, 3) == 0);
    // 未安装槽不能通过填零伪装成已安装轴.
    CHECK(gm_group.send({120}, {std::nullopt, std::nullopt, gm_axis.output(), DjiOutput{0.0f, {120}}}).status ==
        DjiGroupStatus::invalid_output);
    CHECK(bus.calls == 7);
}

void axis_c610_c620_shared_group_contracts() {
    FakeCan bus;
    // C610 与 C620 同组: 每槽按自己的限值编码, 整组用较大编码范围封装.
    DjiDevice c610{{1, DjiMode::c610_current, 2, 1, 1, {50}, 200, 10000}};
    DjiDevice c620{{1, DjiMode::c620_current, 1, 1, 1, {50}, 200, 16384}};
    DjiTxGroup group{bus, 1, 1, {20}, {&c620, &c610, nullptr, nullptr}};
    CHECK(group.configured());
    auto config = yaw_axis_config();
    config.current_limit_a = 1.0f;
    DjiAxis axis_c620{config, c620};
    DjiAxis axis_c610{config, c610};
    CHECK(axis_c620.enable(ControlMode::current, ec::CommandSource::remote, true, true));
    CHECK(axis_c610.enable(ControlMode::current, ec::CommandSource::remote, true, true));
    CHECK(axis_c620.set_current(0.5f, {100}));
    CHECK(axis_c610.set_current(-0.6f, {100}));
    CHECK(!axis_c610.set_current(-1.1f, {100})); // 轴侧上限仍是 1 A
    const auto first = axis_c620.tick({100}, 0.001f);
    const auto second = axis_c610.tick({100}, 0.001f);
    CHECK(near(*first.normalized_output, 0.5f / 20.0f));
    // C610 设备编码上限是 10000 raw, 归一化基准随设备改变.
    CHECK(near(*second.normalized_output, -0.6f / 10.0f));
    CHECK(group.send({100}, {axis_c620.output(), axis_c610.output(), std::nullopt, std::nullopt}).status ==
        DjiGroupStatus::accepted);
    CHECK(bus.calls == 1);
    CHECK(slot_raw(bus.last, 0) == 410 && slot_raw(bus.last, 1) == -600);
}
} // 测试辅助

int main() {
    axis_config_contracts();
    axis_current_mode_contracts();
    axis_velocity_mode_contracts();
    axis_position_mode_contracts();
    axis_direction_and_ratio_contracts();
    axis_four_axis_group_contracts();
    axis_c610_c620_shared_group_contracts();
    std::puts("axis contracts passed");
    return 0;
}
