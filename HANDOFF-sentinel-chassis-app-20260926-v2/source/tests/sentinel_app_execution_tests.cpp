// O2 机构执行与真实 CAN 编组契约:
//  - 底盘 SI twist → 四轮运动学 → 同比例缩放 → 速度闭环, 正常零速仍纠偏, 撤目标撤输出;
//  - 同组多槽不互相覆盖; 已使能过的槽必须给出新鲜零输出而不是整组停发;
//  - 发送失败/队列满不报告为已到达电调;
//  - 关节参考自动恢复/失效、实际位置越界按轴故障、无可达等价分支拒绝;
//  - DM encode/decode、enable/disable 请求与反馈生命周期, 无反馈时不虚构失能完成.
// 所有序列都经过完整 SentinelApp::update + SentinelExecutor::step, 不单独测 codec 后宣称完成.
#include "sentinel_execution.hpp"
#include "../boards/sentinel_f407/include/board_app_config.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <utility>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3);
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::Tick;
using ec::sentinel::app::AcceptResult;
using ec::sentinel::app::AppInputs;
using ec::sentinel::app::ConfigAudience;
using ec::sentinel::app::Domain;
using ec::sentinel::app::DomainSetpoint;
using ec::sentinel::app::Epochs;
using ec::sentinel::app::ExecutorStats;
using ec::sentinel::app::FaultInputs;
using ec::sentinel::app::Reason;
using ec::sentinel::app::SentinelApp;
using ec::sentinel::app::SentinelConfig;
using ec::sentinel::app::SentinelExecutor;
using ec::sentinel::app::Source;
using ec::sentinel::app::TargetSnapshot;
using ec::control::ControlMode;
using ec::drivers::Dr16Switch;
using ec::transport::CanFrame;
using ec::transport::SendResult;

constexpr float tau = 2.0f * std::numbers::pi_v<float>;
constexpr std::uint32_t index(Domain domain) { return ec::sentinel::app::domain_index(domain); }

// 只记录帧并可按需制造发送失败; 不代表任何真实总线时序.
struct FakeBus : ec::transport::CanTx {
    SendResult next{SendResult::accepted};
    std::array<CanFrame, 128> frames{};
    std::size_t count{};
    std::size_t rejected{};
    SendResult try_send(const CanFrame& frame) noexcept override {
        if (next != SendResult::accepted) {
            ++rejected;
            return next;
        }
        if (count < frames.size()) { frames[count++] = frame; }
        return SendResult::accepted;
    }
    void clear() {
        count = 0;
        rejected = 0;
    }
    [[nodiscard]] const CanFrame* last(std::uint16_t id) const {
        for (std::size_t i = count; i > 0; --i) {
            if (frames[i - 1].id == id) { return &frames[i - 1]; }
        }
        return nullptr;
    }
};

[[nodiscard]] std::int16_t slot_raw(const CanFrame& frame, std::size_t slot) {
    const auto high = static_cast<std::uint16_t>(frame.data[2 * slot]) << 8;
    return static_cast<std::int16_t>(high | frame.data[2 * slot + 1]);
}

struct Rig {
    SentinelConfig config;
    SentinelApp app;
    FakeBus bus1{};
    FakeBus bus2{};
    SentinelExecutor exec;
    AppInputs inputs{};
    FaultInputs extra{};
    Tick now{1};
    bool launch_permit{true};

    explicit Rig(SentinelConfig value = ec::sentinel::app::synthetic_config(),
        ec::MicroTick (*clock)() noexcept = nullptr,
        ec::sentinel::app::StartupPolicy startup = ec::sentinel::app::StartupPolicy::remote_required)
        : config(std::move(value)), app(config, ConfigAudience::offline_test, startup),
          exec(config, ConfigAudience::offline_test, bus1, bus2, clock) {
        inputs.link.locally_ready = true;
    }

    void at(std::uint32_t ms) { now = Tick{ms}; }
    void step(float dt_s = 0.001f, std::optional<ec::MicroTick> at_us = std::nullopt) {
        inputs.epochs = Epochs{1, 0};
        // 本文件关注执行与编组, 因此默认给出新鲜发射许可; 许可门禁在感知契约里核对.
        inputs.aux.launch_permit_valid = true;
        inputs.aux.launch_permit = launch_permit;
        inputs.aux.launch_sampled_at = now;
        inputs.aux.launch_ttl = ec::Lifetime{1000};
        inputs.faults = exec.health(now);
        for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
            inputs.faults.feedback_lost[i] |= extra.feedback_lost[i];
            inputs.faults.reference_lost[i] |= extra.reference_lost[i];
            inputs.faults.device_fault[i] |= extra.device_fault[i];
            inputs.faults.device_restarted[i] |= extra.device_restarted[i];
        }
        app.update(now, inputs);
        exec.step(now, dt_s, app, at_us);
    }
    // 建立"遥控本地运行档".
    void go_local(std::uint32_t ms, float dt_s = 0.001f) {
        inputs.link.remote_online = true;
        inputs.link.switch_left = Dr16Switch::up;
        inputs.link.switch_right = Dr16Switch::mid;
        at(ms);
        step(dt_s);
    }
    void feed(Domain domain, float rotor_angle_rad, float rotor_speed_rad_s) {
        const auto slot = app.dji_slot_for(domain);
        CHECK(slot.has_value());
        const auto& cfg = config.dji[*slot];
        const auto protocol = ec::drivers::dji_protocol(cfg.mode);
        CanFrame frame{};
        frame.id = static_cast<std::uint16_t>(protocol->rx_base + cfg.device_id);
        frame.size = 8;
        float angle = std::fmod(rotor_angle_rad, tau);
        if (angle < 0.0f) { angle += tau; }
        const auto encoder = static_cast<unsigned>(angle / tau * 8192.0f);
        const auto speed = static_cast<std::int16_t>(rotor_speed_rad_s * 60.0f / tau);
        frame.data[0] = static_cast<std::uint8_t>(encoder >> 8);
        frame.data[1] = static_cast<std::uint8_t>(encoder & 0xffu);
        frame.data[2] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) >> 8);
        frame.data[3] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) & 0xffu);
        frame.data[6] = 40;
        exec.route(cfg.bus, frame, now, ec::MicroTick{now.ms * 1000u + 123u});
    }
    void feed_dm(std::uint8_t status, float position_rad) {
        const auto& slot = config.dm[0];
        const auto position = ec::drivers::dm_encode_position(*slot.limits, position_rad);
        const auto velocity = ec::drivers::dm_encode_velocity(*slot.limits, 0.0f);
        const auto torque = ec::drivers::dm_encode_torque(*slot.limits, 0.0f);
        CHECK(position && velocity && torque);
        CanFrame frame{};
        frame.id = *slot.master_id;
        frame.size = 8;
        frame.data[0] = static_cast<std::uint8_t>((*slot.can_id & 0x0fu) | (status << 4));
        frame.data[1] = static_cast<std::uint8_t>(*position >> 8);
        frame.data[2] = static_cast<std::uint8_t>(*position & 0xffu);
        frame.data[3] = static_cast<std::uint8_t>(*velocity >> 4);
        frame.data[4] =
            static_cast<std::uint8_t>(((*velocity & 0x0fu) << 4) | (*torque >> 8));
        frame.data[5] = static_cast<std::uint8_t>(*torque & 0xffu);
        frame.data[6] = 30;
        frame.data[7] = 40;
        exec.route(slot.bus, frame, now);
    }
    void feed_wheels(float rotor_speed_rad_s = 0.0f) {
        for (const auto domain : {Domain::chassis_lf, Domain::chassis_rf, Domain::chassis_lb,
                 Domain::chassis_rb}) {
            feed(domain, 0.0f, rotor_speed_rad_s);
        }
    }
    // 台架常态: 所有已装配设备每周期都在上报. 缺少反馈的场景由 extra.feedback_lost 显式构造,
    // 不靠"忘记喂反馈"隐含表达 (从未上报也是必需反馈缺失, A09 依赖集合必须知道).
    void feed_all(float rotor_speed_rad_s = 0.0f) {
        for (std::size_t i = 0; i < ec::sentinel::app::dji_slot_count; ++i) {
            if (!config.dji[i].installed) { continue; }
            feed(config.dji[i].domain, 0.0f, rotor_speed_rad_s);
        }
        if (config.dm[0].installed) { feed_dm(0, 0.0f); }
    }
};

void single_gm6020_execution_contracts() {
    using namespace ec::sentinel::app;
    Rig rig{ec::sentinel::board::single_gm6020_bench_config(
        ec::sentinel::board::deployment_config()), nullptr, StartupPolicy::maintenance_only};
    rig.inputs.link.maintenance_link_alive = true;
    rig.inputs.link.maintenance_request = true;
    rig.inputs.link.maintenance_session_epoch = 1;
    rig.feed(Domain::main_yaw, 1.0f, 0.0f);
    rig.exec.set_joint_velocity_feedback(Domain::main_yaw, {2.0f, rig.now, ec::Validity::valid});
    rig.step();
    CHECK(rig.exec.assembled());
    CHECK(rig.app.authority().source() == Source::maintenance);
    CHECK(rig.exec.joint_feedback(Domain::main_yaw, rig.now).position_rad.validity != ec::Validity::valid);
    CHECK(rig.exec.align_reference(Domain::main_yaw, 0.0f, rig.now));
    rig.step();
    CHECK(rig.exec.joint_feedback(Domain::main_yaw, rig.now).position_rad.validity == ec::Validity::valid);
    CHECK(rig.exec.joint_feedback(Domain::main_yaw, rig.now).velocity_rad_s.value == 2.0f);
    TargetSnapshot packet{};
    packet.domains[index(Domain::main_yaw)] = DomainSetpoint{ControlMode::position, 0.05f};
    CHECK(rig.app.accept_targets(packet, rig.now) == AcceptResult::accepted);
    rig.at(2); rig.feed(Domain::main_yaw, 1.0f, 0.0f); rig.step();
    const auto* output = rig.bus1.last(0x1fe);
    CHECK(output && slot_raw(*output, 3) < 0); // 实际测速2rad/s高于外环目标, 不能偷偷用原生零速.
    CHECK(slot_raw(*output, 0) == 0 && slot_raw(*output, 1) == 0 && slot_raw(*output, 2) == 0);
    CHECK(rig.bus2.count == 0);
    rig.at(3); rig.feed(Domain::main_yaw, 1.0f, 0.0f);
    rig.exec.set_joint_velocity_feedback(Domain::main_yaw, {});
    rig.step();
    CHECK(slot_raw(*rig.bus1.last(0x1fe), 3) == 0); // 选中的反馈失效后不回退原生RPM.
    CHECK(rig.app.accept_targets(TargetSnapshot{}, rig.now) == AcceptResult::accepted);
    rig.at(4); rig.step();
    CHECK(slot_raw(*rig.bus1.last(0x1fe), 3) == 0);
    // 无机械挡的台架允许穿过人工零位及编码器回绕, 不把0/2π伪装成限位.
    for (std::uint32_t t = 5; t < 1005; ++t) {
        rig.at(t); rig.feed(Domain::main_yaw, 1.0f-0.01f*static_cast<float>(t-4), -10.0f);
        rig.exec.set_joint_velocity_feedback(Domain::main_yaw, {-10.0f, rig.now, ec::Validity::valid});
        rig.step();
        CHECK(!rig.exec.health(rig.now).device_fault[index(Domain::main_yaw)]);
    }
    CHECK(rig.exec.joint_feedback(Domain::main_yaw, rig.now).position_rad.value < -tau);
    packet.domains[index(Domain::main_yaw)] = DomainSetpoint{ControlMode::position, -0.1f};
    CHECK(rig.app.accept_targets(packet, rig.now) == AcceptResult::accepted);
    rig.at(1100); rig.step(); // 反馈失联使参考失效, 不能靠重连猜圈数.
    rig.at(1101); rig.feed(Domain::main_yaw, 1.0f, 0.0f); rig.step();
    CHECK(rig.exec.joint_feedback(Domain::main_yaw, rig.now).position_rad.validity != ec::Validity::valid);
    CHECK(rig.exec.align_reference(Domain::main_yaw, 0.0f, rig.now));
    CHECK(std::fabs(rig.exec.joint_feedback(Domain::main_yaw, rig.now).position_rad.value) < 1e-6f);
}

void four_motor_execution_contracts() {
    using namespace ec::sentinel::app;
    Rig rig{ec::sentinel::board::four_motor_bench_config(ec::sentinel::board::deployment_config()),
        nullptr, StartupPolicy::maintenance_only};
    rig.launch_permit = false;
    rig.inputs.link.maintenance_link_alive = true;
    rig.inputs.link.maintenance_request = true;
    rig.inputs.link.maintenance_session_epoch = 1;
    rig.feed_all(); rig.step();
    CHECK(rig.exec.assembled());
    CHECK(rig.exec.align_reference(Domain::main_yaw, 0.0f, rig.now));
    TargetSnapshot packet{};
    for (auto domain : {Domain::main_yaw, Domain::pitch}) {
        packet.domains[index(domain)] = DomainSetpoint{ControlMode::position, 0.05f};
    }
    for (auto domain : {Domain::friction_left, Domain::trigger}) {
        packet.domains[index(domain)] = DomainSetpoint{ControlMode::velocity, 0.5f};
    }
    CHECK(rig.app.accept_targets(packet, rig.now) == AcceptResult::accepted);
    for (unsigned t=2; t<=7; ++t) { rig.at(t); rig.feed_all(); rig.step(); }
    CHECK(!rig.app.snapshot().launch_allowed);
    CHECK(rig.bus2.count == 0);
    CHECK(rig.bus1.last(0x200) && slot_raw(*rig.bus1.last(0x200),1)>0);
    CHECK(rig.bus1.last(0x200) && slot_raw(*rig.bus1.last(0x200),2)>0);
    CHECK(slot_raw(*rig.bus1.last(0x200),0)==0 && slot_raw(*rig.bus1.last(0x200),3)==0);
    CHECK(rig.bus1.last(0x1fe) && slot_raw(*rig.bus1.last(0x1fe),3)>0);
    CHECK(rig.bus1.last(1) != nullptr);
    for (auto domain : {Domain::main_yaw, Domain::pitch, Domain::friction_left, Domain::trigger}) {
        CHECK(rig.app.snapshot().executions[index(domain)].execute);
    }
    rig.extra.device_fault[index(Domain::friction_left)] = true;
    for (unsigned t=8; t<=11; ++t) { rig.at(t); rig.feed_all(); rig.step(); }
    CHECK(!rig.app.snapshot().executions[index(Domain::friction_left)].execute);
    CHECK(rig.app.snapshot().executions[index(Domain::trigger)].execute);
    CHECK(slot_raw(*rig.bus1.last(0x200),1)==0);
    CHECK(slot_raw(*rig.bus1.last(0x200),2)>0);
    rig.extra.device_fault[index(Domain::pitch)] = true;
    for (unsigned t=12; t<=15; ++t) { rig.at(t); rig.feed_all(); rig.step(); }
    CHECK(!rig.app.snapshot().executions[index(Domain::pitch)].execute);
    CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
    CHECK(rig.app.snapshot().executions[index(Domain::trigger)].execute);
    CHECK(rig.app.accept_targets(TargetSnapshot{}, rig.now) == AcceptResult::accepted);
    for (unsigned t=16; t<=19; ++t) { rig.at(t); rig.feed_all(); rig.step(); }
    CHECK(slot_raw(*rig.bus1.last(0x1fe),3)==0 && slot_raw(*rig.bus1.last(0x200),2)==0);
    CHECK(rig.bus1.last(0x1ff)==nullptr); // 包含上电、运行、故障与撤销阶段, 均不发送GM电压帧.
}

TargetSnapshot twist_packet(float vx_mps, float vy_mps = 0.0f, float wz_rad_s = 0.0f) {
    TargetSnapshot out{};
    out.chassis = ec::motion::PlanarTwist{vx_mps, vy_mps, wz_rad_s};
    return out;
}

// 底盘: twist 解算/缩放/闭环/撤输出.
void chassis_contracts() {
    Rig rig;
    rig.feed_wheels();
    rig.go_local(10);
    CHECK(rig.exec.assembled());
    CHECK(rig.exec.dji_device(*rig.app.dji_slot_for(Domain::chassis_lf)) != nullptr);
    rig.bus2.clear();

    // 小 twist: 不缩放, 目标落在线性区, 可直接核对单位换算链.
    CHECK(rig.app.accept_targets(twist_packet(0.02f), Tick{11}) == AcceptResult::accepted);
    rig.at(11);
    rig.step();
    CHECK(rig.app.twist_scale() == 1.0f);
    const auto* wheel_frame = rig.bus2.last(0x200);
    CHECK(wheel_frame != nullptr);
    CHECK(wheel_frame->size == 8);
    const auto lf = slot_raw(*wheel_frame, 0);
    const auto rf = slot_raw(*wheel_frame, 1);
    // 轮 lf 滚动方向 3π/4, rf 为 π/4: vx>0 时两轮轴侧目标符号相反.
    CHECK(lf < 0);
    CHECK(rf > 0);
    // 轴侧目标 = ((v·d)/radius)*gain; 归一化输出 = Kp*误差; raw = 归一化*(I_limit/A_full)*raw_limit.
    // |-0.02*cos(3π/4)/0.06*1.2*(3/20)*16384| ≈ 695.
    CHECK(std::abs(std::abs(lf) - 695) <= 2);

    // 超上限 twist: 四轮同比例缩放, 保留平移/旋转比例.
    // 夹具轮半径 0.06 m, 轴侧上限 50 rad/s → 单轮约 3 m/s, 因此 5 m/s 一定超限.
    CHECK(rig.app.accept_targets(twist_packet(5.0f), Tick{12}) == AcceptResult::accepted);
    rig.at(12);
    rig.step();
    CHECK(rig.app.twist_scale() < 1.0f);
    CHECK(rig.app.requested_twist()->vx_mps == 5.0f);
    CHECK(std::fabs(rig.app.adopted_twist()->vx_mps - 5.0f * rig.app.twist_scale()) < 1e-4f);
    const auto* scaled = rig.bus2.last(0x200);
    CHECK(scaled != nullptr);
    CHECK(std::abs(slot_raw(*scaled, 0)) == std::abs(slot_raw(*scaled, 1)));

    // 正常零目标仍可纠偏: 目标是 0, 反馈不为 0 时必须有反向输出.
    CHECK(rig.app.accept_targets(twist_packet(0.0f), Tick{13}) == AcceptResult::accepted);
    rig.at(13);
    for (const auto domain : {Domain::chassis_lf, Domain::chassis_rf, Domain::chassis_lb,
             Domain::chassis_rb}) {
        rig.feed(domain, 0.0f, 100.0f); // 轴侧 +5.26 rad/s
    }
    rig.bus2.clear();
    rig.step();
    const auto* correcting = rig.bus2.last(0x200);
    CHECK(correcting != nullptr);
    CHECK(slot_raw(*correcting, 0) < 0);

    // 撤目标 → 已使能过的槽按应用动作生成新鲜零输出, 整组仍然发帧.
    TargetSnapshot empty{};
    CHECK(rig.app.accept_targets(empty, Tick{14}) == AcceptResult::accepted);
    rig.at(14);
    rig.step();
    const auto* withdrawn = rig.bus2.last(0x200);
    CHECK(withdrawn != nullptr);
    for (std::size_t s = 0; s < 4; ++s) { CHECK(slot_raw(*withdrawn, s) == 0); }
}

// 同组多槽独立 + 单槽失效 + 发送失败语义.
void group_contracts() {
    Rig rig;
    // 台架常态: 全部设备在上报 (yaw/pitch 的必需反馈缺失会按 A09 禁止正向拨弹).
    rig.feed_all();
    rig.go_local(10);
    rig.bus2.clear();
    TargetSnapshot packet{};
    packet.domains[index(Domain::friction_left)] = DomainSetpoint{ControlMode::velocity, 2.0f};
    packet.domains[index(Domain::friction_right)] = DomainSetpoint{ControlMode::velocity, 2.0f};
    packet.domains[index(Domain::trigger)] = DomainSetpoint{ControlMode::velocity, 3.0f};
    CHECK(rig.app.accept_targets(packet, Tick{11}) == AcceptResult::accepted);
    rig.at(11);
    rig.step();
    const auto* frame = rig.bus2.last(0x1ff);
    CHECK(frame != nullptr);
    // 组 2 槽位: friction_left(id5)=0, friction_right(id6)=1, trigger(id7)=2, 槽 3 未装.
    CHECK(slot_raw(*frame, 0) != 0);
    CHECK(slot_raw(*frame, 1) != 0);
    CHECK(slot_raw(*frame, 2) != 0);

    // 拨弹故障只撤拨弹: 同组其它槽必须是新鲜输出, 不能整组停发.
    rig.at(12);
    rig.extra.device_fault[index(Domain::trigger)] = true;
    rig.bus2.clear();
    rig.step();
    const auto* after = rig.bus2.last(0x1ff);
    CHECK(after != nullptr);
    CHECK(slot_raw(*after, 0) != 0);
    CHECK(slot_raw(*after, 1) != 0);
    CHECK(slot_raw(*after, 2) == 0);
    CHECK(rig.app.snapshot().executions[index(Domain::trigger)].reason ==
        Reason::fault_dependency);
    rig.extra.device_fault[index(Domain::trigger)] = false;

    // 发送失败/队列满: 计入传输失败, 不报告为已到达电调 (该总线上一帧都没出去).
    rig.at(13);
    const auto error_before = rig.exec.stats().group_tx_error;
    const auto frames_before = rig.bus2.count;
    rig.bus2.next = SendResult::busy;
    rig.step();
    CHECK(rig.bus2.count == frames_before);
    CHECK(rig.exec.stats().group_tx_error > error_before);
    rig.bus2.next = SendResult::accepted;
}

// 关节: 参考自动唯一恢复、实际越界按轴故障、无可达等价分支拒绝.
void joint_contracts() {
    Rig rig;
    rig.feed(Domain::main_yaw, tau - 0.5f, 0.0f); // direction=-1 → 轴侧单圈 -0.5+2π → 关节 0.5
    rig.feed(Domain::fire_yaw, 1.0f, 0.0f);
    rig.go_local(10);
    CHECK(rig.exec.reference(Domain::main_yaw).valid);
    CHECK(std::fabs(rig.exec.reference(Domain::main_yaw).joint_rad - 0.5f) < 1e-3f);
    CHECK(rig.exec.reference(Domain::fire_yaw).valid);
    CHECK(std::fabs(rig.exec.reference(Domain::fire_yaw).joint_rad - 1.0f) < 1e-3f);
    // 关节配置齐备且参考已建立 → 没有参考失效故障, 拨弹不被禁止.
    CHECK(!rig.exec.last_health().reference_lost[index(Domain::main_yaw)]);

    // fire yaw 是相对关节角: 越限整包拒绝.
    TargetSnapshot out_of_range{};
    out_of_range.domains[index(Domain::fire_yaw)] = DomainSetpoint{ControlMode::position, 2.0f};
    CHECK(rig.app.accept_targets(out_of_range, Tick{11}) == AcceptResult::limit_violation);

    // 实际位置越界按该轴故障 (A08), 并禁止正向拨弹 (A09).
    rig.at(11);
    rig.feed(Domain::main_yaw, tau - 0.5f - 1.0f, 0.0f); // 继续转 1 rad → 关节 1.5 (仍在范围内)
    rig.step();
    CHECK(std::fabs(rig.exec.reference(Domain::main_yaw).joint_rad - 1.5f) < 1e-2f);
    rig.at(12);
    rig.feed(Domain::fire_yaw, 1.0f + 1.0f, 0.0f); // 关节 2.0 > 上限 1.5
    rig.step();
    CHECK(rig.exec.last_health().device_fault[index(Domain::fire_yaw)]);
    CHECK(rig.app.snapshot().executions[index(Domain::fire_yaw)].reason ==
        Reason::fault_dependency);

    // 单圈范围宽于一圈时不能唯一恢复: 保持参考未知.
    auto wide = ec::sentinel::app::synthetic_config();
    wide.joints[1].min_rad = -4.0f;
    wide.joints[1].max_rad = 4.0f;
    Rig wide_rig{wide};
    wide_rig.feed(Domain::fire_yaw, 0.3f, 0.0f);
    wide_rig.go_local(10);
    CHECK(!wide_rig.exec.reference(Domain::fire_yaw).valid);
    CHECK(wide_rig.exec.last_health().reference_lost[index(Domain::fire_yaw)]);
}

// 主 yaw 等价分支: 可达时选最近分支, 不可达时拒绝且不调其它轴.
void branch_contracts() {
    using ec::sentinel::app::select_equivalent_branch;
    // 单圈编码器经 N:1 传动后轴侧混叠周期是 2π/N, 不能用 2π 当作唯一性依据 (APP16).
    {
        using ec::sentinel::app::recover_single_turn_joint;
        constexpr float pi_f = 3.14159265358979323846f;
        // 直驱 (N=1, 周期 2π): 范围宽 5.2 < 2π → 唯一.
        CHECK(recover_single_turn_joint(-2.6f, 2.6f, 0.0f, 0.5f, 2.0f * pi_f).has_value());
        // N=2 (周期 π): 范围宽 5.2 > π → 同一读数对应多个关节角, 保持参考未知.
        CHECK(!recover_single_turn_joint(-2.6f, 2.6f, 0.0f, 0.5f, pi_f).has_value());
        // N=2 且范围窄于半圈 → 仍可唯一恢复.
        const auto half_turn = recover_single_turn_joint(-1.4f, 1.4f, 0.0f, 0.5f, pi_f);
        CHECK(half_turn.has_value());
        if (half_turn) { CHECK(std::fabs(*half_turn - 0.5f) < 1.0e-3f); }
        // 混叠周期本身非法时不猜.
        CHECK(!recover_single_turn_joint(-1.0f, 1.0f, 0.0f, 0.5f, 0.0f).has_value());
        // 非整数传动比 (N=1.5): 同余必须按真实 alias 取模, 先减 2π 会改变同余类,
        // 把并不等价的读数说成同一个关节角.
        constexpr float alias_1p5 = 2.0f * pi_f / 1.5f;
        CHECK(!recover_single_turn_joint(1.8f, 2.0f, 0.0f, 4.0f, alias_1p5).has_value());
    }
    CHECK(!select_equivalent_branch(-0.5f, 0.5f, 0.0f, 3.0f).has_value());
    // 合法等价方向不能因任意圈数上限被改成不可达: 五圈 (10π) 与 0 等价.
    const auto five_turn = select_equivalent_branch(-2.6f, 2.6f, 0.0f, 5.0f * tau);
    CHECK(five_turn.has_value());
    if (five_turn) { CHECK(std::fabs(*five_turn) < 1.0e-2f); }
    // 大有限值 / 极值 / 非有限值: 有界处理, 不崩溃也不死循环.
    const auto huge = select_equivalent_branch(-2.6f, 2.6f, 0.0f, 1.0e10f);
    CHECK(!huge || (*huge >= -2.6f && *huge <= 2.6f));
    const auto extreme = select_equivalent_branch(-2.6f, 2.6f, 0.0f, 3.0e38f);
    CHECK(!extreme || (*extreme >= -2.6f && *extreme <= 2.6f));
    CHECK(!select_equivalent_branch(-2.6f, 2.6f, 0.0f,
        std::numeric_limits<float>::infinity()).has_value());
    CHECK(!select_equivalent_branch(-2.6f, 2.6f, 0.0f,
        std::numeric_limits<float>::quiet_NaN()).has_value());
    // 范围内的等价圈数仍然可用: 接近软限位与多圈参考都能选到最近分支.
    const auto near_limit = select_equivalent_branch(-2.6f, 2.6f, 2.5f, 2.4f - tau);
    CHECK(near_limit.has_value());
    CHECK(std::fabs(*near_limit - 2.4f) < 1e-3f);
    // 请求方向与另一个 2π 等价分支同值 (浮点加减只差 1 ulp 级, 因此边界留 0.01 rad).
    const auto branch = select_equivalent_branch(-3.01f, 3.01f, 0.0f, 3.0f + tau);
    CHECK(branch.has_value());
    CHECK(std::fabs(*branch - 3.0f) < 1e-3f);
    // 当前位置靠近负向分支时必须选它, 而不是绕远到正向分支.
    const auto nearest = select_equivalent_branch(-4.0f, 4.0f, -3.0f, 3.0f - tau);
    CHECK(nearest.has_value());
    CHECK(std::fabs(*nearest - (3.0f - tau)) < 1e-3f);

    auto narrow = ec::sentinel::app::synthetic_config();
    narrow.joints[0].min_rad = -0.5f;
    narrow.joints[0].max_rad = 0.5f;
    Rig rig{narrow};
    rig.feed(Domain::main_yaw, 0.0f, 0.0f);
    rig.go_local(10);
    CHECK(rig.exec.reference(Domain::main_yaw).valid);
    TargetSnapshot packet{};
    packet.domains[index(Domain::main_yaw)] = DomainSetpoint{ControlMode::position, 3.0f};
    CHECK(rig.app.accept_targets(packet, Tick{11}) == AcceptResult::limit_violation);
    // 机械角越界直接拒绝; 世界方向则由三维运动学判断有无可达分支.
    packet.domains[index(Domain::main_yaw)]->frame = ec::sentinel::app::YawFrame::local_inertial;
    rig.inputs.main_yaw.validity = ec::sentinel::app::DomainValidity::valid;
    rig.inputs.main_yaw.value.orientation = ec::estimation::Quaternion{};
    rig.inputs.main_yaw.value.heading_gain = 1;
    CHECK(rig.app.accept_targets(packet, Tick{11}) == AcceptResult::accepted);
    rig.at(11);
    rig.step();
    CHECK(rig.exec.stats().unreachable_targets == 1);
    // 下一周期由应用层拒绝该目标并撤输出; 不自动调底盘或另一 yaw 补偿.
    rig.at(12);
    rig.step();
    CHECK(!rig.app.target(Domain::main_yaw).has_value());
    CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].reason == Reason::invalid_target);
    rig.at(13);
    rig.step();
    CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].reason == Reason::no_target);
}

// DM pitch: encode/decode、enable/disable 请求与反馈生命周期.
void pitch_contracts() {
    Rig rig;
    rig.bus1.clear();
    rig.go_local(10);
    // 启动后没有任何目标: 撤输出 = disable 请求. 从未本地 enable 也要发 —— 设备与 MCU 可以
    // 独立重启, 接入时设备可能已经 Enabled, 把"我曾使能过"当作前提会漏掉停止请求.
    CHECK(rig.exec.stats().dm_disable_sent == 1);
    CHECK(rig.exec.pitch_disable_requested());
    const CanFrame* disable_frame0 = nullptr;
    for (std::size_t i = 0; i < rig.bus1.count; ++i) {
        if (rig.bus1.frames[i].id == 0x01) { disable_frame0 = &rig.bus1.frames[i]; }
    }
    CHECK(disable_frame0 != nullptr);
    CHECK(disable_frame0->data[7] == 0xfd);

    // 接受目标但从未收到反馈: 必需反馈缺失 (A09) → 保持撤输出, 不重复发 disable.
    rig.bus1.clear();
    TargetSnapshot packet{};
    packet.domains[index(Domain::pitch)] = DomainSetpoint{ControlMode::position, -0.25f};
    CHECK(rig.app.accept_targets(packet, Tick{11}) == AcceptResult::accepted);
    rig.at(11);
    rig.step();
    CHECK(rig.exec.stats().dm_enable_sent == 0);
    CHECK(rig.exec.stats().dm_command_sent == 0);
    CHECK(rig.bus1.last(0x01) == nullptr);
    // 必需反馈缺失会撤输出并作废旧意图: 反馈回来之后必须重新下发目标 (不追旧目标).
    CHECK(!rig.app.target(Domain::pitch).has_value());

    // 设备上报 Disabled: 重新给出目标 → 发使能请求, 但确认 Enabled 之前不发 MIT.
    rig.at(12);
    rig.feed_dm(0x0, -0.25f);
    CHECK(rig.app.accept_targets(packet, Tick{12}) == AcceptResult::accepted);
    rig.bus1.clear();
    rig.step();
    CHECK(rig.exec.stats().dm_enable_sent == 1);
    CHECK(rig.exec.pitch_enable_requested());
    CHECK(!rig.exec.pitch_enable_confirmed());
    CHECK(rig.exec.stats().dm_command_sent == 0); // 未确认使能前不发 MIT
    const CanFrame* enable_frame = nullptr;
    for (std::size_t i = 0; i < rig.bus1.count; ++i) {
        if (rig.bus1.frames[i].id == 0x01) { enable_frame = &rig.bus1.frames[i]; }
    }
    CHECK(enable_frame != nullptr);
    CHECK(enable_frame->data[7] == 0xfc);

    // 反馈确认 Enabled 后才下发 MIT; 位置按零位偏移映射到设备坐标.
    rig.at(13);
    rig.feed_dm(0x1, -0.25f);
    rig.bus1.clear();
    rig.step();
    CHECK(rig.exec.pitch_enable_confirmed());
    const auto* mit = rig.bus1.last(0x01);
    CHECK(mit != nullptr);
    // MIT 命令帧按命令字段核对, 不当作反馈解析.
    const auto position_raw = static_cast<std::uint16_t>(
        (static_cast<unsigned>(mit->data[0]) << 8) | mit->data[1]);
    const auto decoded_position = ec::drivers::dm_decode_position(*rig.config.dm[0].limits,
        position_raw);
    CHECK(decoded_position.has_value());
    CHECK(std::fabs(*decoded_position - (-0.25f)) < 0.01f);
    CHECK(rig.config.dm[0].mit_kp.has_value() && rig.config.dm[0].mit_kd.has_value());
    CHECK(rig.exec.reference(Domain::pitch).valid);

    // 撤目标: 显式 disable 请求, 只发一次; 在反馈确认失能前不声称已失能.
    TargetSnapshot empty{};
    CHECK(rig.app.accept_targets(empty, Tick{14}) == AcceptResult::accepted);
    rig.at(14);
    rig.bus1.clear();
    rig.step();
    CHECK(rig.exec.stats().dm_disable_sent == 2);
    CHECK(rig.exec.pitch_disable_requested());
    CHECK(!rig.exec.pitch_enable_requested());
    const CanFrame* disable_frame = nullptr;
    for (std::size_t i = 0; i < rig.bus1.count; ++i) {
        if (rig.bus1.frames[i].id == 0x01) { disable_frame = &rig.bus1.frames[i]; }
    }
    CHECK(disable_frame != nullptr);
    CHECK(disable_frame->data[7] == 0xfd);
    // 反馈仍报 Enabled: 请求已发但未确认, 不虚构失能完成.
    CHECK(rig.exec.dm_device()->enabled(rig.now));
    CHECK(rig.exec.dm_device()->request_result(rig.now) != ec::drivers::DmRequestResult::accepted);
    rig.at(15);
    rig.bus1.clear();
    rig.step();
    CHECK(rig.bus1.last(0x01) == nullptr); // 不重复发 disable, 也不在停止态发 MIT
    // 反馈确认 Disabled 后请求结果不再报 pending.
    rig.at(16);
    rig.feed_dm(0x0, -0.25f);
    rig.step();
    CHECK(!rig.exec.dm_device()->enabled(rig.now));
    CHECK(rig.exec.dm_device()->request_result(rig.now) ==
        ec::drivers::DmRequestResult::accepted);
}

// F11: pitch 停止请求的生命周期. 决策看当前期望/观测状态, 不被"我曾 enable 过"或
// 历史发送失败次数永久锁存; 发送 accepted 不等于状态确认, 故障态不等同 Disabled.
void pitch_stop_lifecycle_contracts() {
    // (a) 接入时设备已经 Enabled 且本进程从未发 enable: 停止请求必须发出 disable.
    {
        Rig rig;
        rig.feed_dm(0x1, -0.25f);
        rig.go_local(10);
        CHECK(rig.exec.dm_device()->enabled(rig.now));
        CHECK(rig.exec.stats().dm_disable_sent == 1);
        CHECK(rig.exec.pitch_disable_requested());
    }
    // (b) 初始 Disabled: 无需发帧; 设备随后被外部使能 → 仍处于停止请求下必须重新请求.
    {
        Rig rig;
        rig.feed_dm(0x0, 0.0f);
        rig.go_local(10);
        CHECK(rig.exec.stats().dm_disable_sent == 0);
        CHECK(!rig.exec.pitch_disable_requested());
        rig.at(11);
        rig.feed_dm(0x1, 0.0f);
        rig.step();
        CHECK(rig.exec.stats().dm_disable_sent == 1);
        CHECK(rig.exec.pitch_disable_requested());
    }
    // (c) 故障态不是 Disabled: 不把它当作已停止.
    {
        Rig rig;
        rig.feed_dm(0x8, 0.0f); // over_voltage
        rig.go_local(10);
        CHECK(rig.exec.dm_device()->faulted(rig.now));
        CHECK(!rig.exec.dm_device()->enabled(rig.now));
        CHECK(rig.exec.stats().dm_disable_sent == 1);
        CHECK(rig.exec.pitch_disable_requested());
    }
    // (d) 发送连续失败 (busy) 后总线恢复且停止请求保持: 必须有界退避重试,
    // 不能因为旧失败计数使后续所有停止都变成无操作.
    {
        Rig rig;
        rig.feed_dm(0x1, 0.0f);
        rig.go_local(10);
        CHECK(rig.exec.stats().dm_disable_sent == 1);
        rig.bus1.next = SendResult::busy;
        for (int i = 0; i < 5; ++i) {
            rig.at(200u + static_cast<std::uint32_t>(i) * 101u);
            rig.feed_dm(0x1, 0.0f);
            rig.step();
        }
        CHECK(rig.exec.stats().dm_tx_error >= 5);
        CHECK(rig.exec.stats().dm_disable_sent == 1); // 失败不计作已发送
        // 总线恢复: 退避后的下一次尝试必须真的发帧.
        rig.bus1.next = SendResult::accepted;
        rig.at(800);
        rig.feed_dm(0x1, 0.0f);
        rig.step();
        CHECK(rig.exec.stats().dm_disable_sent == 2);
        CHECK(rig.exec.pitch_disable_requested());
    }
}

// F06: 关节几何在线改变后, 旧位置参考必须失效并按新的合法单圈信息重建.
void joint_geometry_reference_contracts() {
    Rig rig;
    rig.feed(Domain::main_yaw, 1.5f, 0.0f);
    rig.go_local(10);
    CHECK(rig.exec.reference(Domain::main_yaw).valid);
    const auto before = rig.exec.reference(Domain::main_yaw).joint_rad;
    // 改零位: 同一单圈读数在新零位下指向另一个关节角, 旧参考不能继续 valid.
    auto next = rig.app.joint_geometry(Domain::main_yaw);
    next.zero_offset_rad = next.zero_offset_rad.value_or(0.0f) + 0.5f;
    CHECK(rig.exec.set_joint_geometry(Domain::main_yaw, next));
    CHECK(!rig.exec.reference(Domain::main_yaw).valid);
    // 下一次健康判定用新的合法单圈信息重建参考 (不是沿用旧零偏下的角度).
    rig.at(11);
    rig.step();
    CHECK(rig.exec.reference(Domain::main_yaw).valid);
    CHECK(std::fabs(rig.exec.reference(Domain::main_yaw).joint_rad - (before - 0.5f)) < 1.0e-2f);
}

// 故障依赖与恢复: 真实反馈过期转为故障输入, 恢复后不追旧目标.
void feedback_loss_contracts() {
    Rig rig;
    rig.feed_wheels();
    rig.feed(Domain::friction_left, 0.0f, 0.0f);
    rig.feed(Domain::friction_right, 0.0f, 0.0f);
    rig.feed(Domain::trigger, 0.0f, 0.0f);
    rig.go_local(10);
    TargetSnapshot packet{};
    packet.domains[index(Domain::friction_left)] = DomainSetpoint{ControlMode::velocity, 2.0f};
    packet.domains[index(Domain::friction_right)] = DomainSetpoint{ControlMode::velocity, 2.0f};
    CHECK(rig.app.accept_targets(packet, Tick{11}) == AcceptResult::accepted);
    rig.at(11);
    rig.step();
    CHECK(rig.app.target(Domain::friction_left).has_value());
    // 反馈静默超过 ttl: 曾经在线 → 必需反馈失效, 该域及拨弹依赖被撤.
    rig.at(80);
    rig.step();
    CHECK(rig.exec.last_health().feedback_lost[index(Domain::friction_left)]);
    CHECK(!rig.app.target(Domain::friction_left).has_value());
    CHECK(rig.app.snapshot().executions[index(Domain::trigger)].reason ==
        Reason::fault_dependency);
    // 反馈恢复: 不追旧目标, 需要新目标.
    rig.at(81);
    rig.feed(Domain::friction_left, 0.0f, 0.0f);
    rig.feed(Domain::friction_right, 0.0f, 0.0f);
    rig.feed(Domain::trigger, 0.0f, 0.0f);
    rig.step();
    CHECK(!rig.exec.last_health().feedback_lost[index(Domain::friction_left)]);
    CHECK(!rig.app.target(Domain::friction_left).has_value());
    CHECK(rig.app.snapshot().executions[index(Domain::friction_left)].reason == Reason::no_target);
    CHECK(!rig.app.snapshot().permission.manual_stop);
}
// 必需反馈与执行结果反馈到目标生命周期 (APP10) 与构造/布局原子性 (APP14).
void readiness_and_layout_contracts() {
    // 从未上报与运行后失联都不满足必需反馈, 但观测上要能区分.
    {
        Rig rig;
        rig.go_local(10);
        // 从未上报: 非关节域按反馈缺失, 关节域同样按反馈缺失 (原因在"没有反馈",
        // 不是"参考不可用") —— 两者都进 A09 依赖集合, 观测上仍是 unknown.
        const auto health = rig.exec.health(Tick{10});
        CHECK(health.feedback_lost[index(Domain::chassis_lf)]);
        CHECK(health.feedback_lost[index(Domain::main_yaw)]);
        CHECK(!health.reference_lost[index(Domain::main_yaw)]);
    }
    {
        Rig rig;
        rig.feed_all();
        rig.go_local(10);
        // 曾经上报而后超时: 关节域按参考失效 (已有反馈的域不再报"从未收到").
        const auto fresh = rig.exec.health(Tick{10});
        CHECK(!fresh.feedback_lost[index(Domain::chassis_lf)]);
        rig.at(80);
        const auto lost = rig.exec.health(Tick{80});
        CHECK(lost.feedback_lost[index(Domain::chassis_lf)]);
        CHECK(lost.reference_lost[index(Domain::main_yaw)]);
        CHECK(!lost.feedback_lost[index(Domain::main_yaw)]);
    }
    // 执行器拒绝上一周期目标后, 旧意图作废, 不自动重试.
    {
        Rig rig;
        rig.feed_all();
        rig.go_local(10);
        TargetSnapshot packet{};
        packet.domains[index(Domain::friction_left)] = DomainSetpoint{ControlMode::velocity, 2.0f};
        CHECK(rig.app.accept_targets(packet, Tick{11}) == AcceptResult::accepted);
        rig.at(11);
        rig.step();
        CHECK(rig.app.target(Domain::friction_left).has_value());
        // 非法 dt: 轴层拒绝 → 下一周期由应用作废旧目标并报 output_rejected.
        rig.at(12);
        rig.step(0.0f);
        rig.at(13);
        rig.step();
        CHECK(!rig.app.target(Domain::friction_left).has_value());
        CHECK(rig.app.snapshot().executions[index(Domain::friction_left)].reason ==
            Reason::output_rejected);
        // 恢复只恢复输入, 不恢复旧意图.
        rig.at(14);
        rig.step();
        CHECK(rig.app.snapshot().executions[index(Domain::friction_left)].reason == Reason::no_target);
    }
    // 合法部分装配必须可启动: 未安装槽不参与布局校验.
    {
        auto partial = ec::sentinel::app::synthetic_config();
        for (std::size_t i = 0; i < ec::sentinel::app::dji_slot_count; ++i) {
            if (partial.dji[i].domain != Domain::friction_left &&
                partial.dji[i].domain != Domain::friction_right) {
                partial.dji[i] = ec::sentinel::app::DjiSlot{};
            }
        }
        partial.dm[0] = ec::sentinel::app::DmSlot{};
        for (auto& joint : partial.joints) { joint = ec::sentinel::app::JointGeometryConfig{}; }
        Rig rig{partial};
        CHECK(rig.exec.assembled());
        rig.feed(Domain::friction_left, 0.0f, 0.0f);
        rig.feed(Domain::friction_right, 0.0f, 0.0f);
        rig.go_local(10);
        TargetSnapshot packet{};
        packet.domains[index(Domain::friction_left)] = DomainSetpoint{ControlMode::velocity, 2.0f};
        CHECK(rig.app.accept_targets(packet, Tick{11}) == AcceptResult::accepted);
        rig.at(11);
        rig.step();
        CHECK(rig.app.snapshot().executions[index(Domain::friction_left)].execute);
        // 未安装域的槽不得被写入任何输出.
        CHECK(!rig.app.dji_slot_for(Domain::main_yaw).has_value());
    }
    // DM 命令 ID 与同总线 DJI 命令组冲突: 构造失败, 且 step 不产生半成品输出.
    {
        auto conflict = ec::sentinel::app::synthetic_config();
        conflict.dm[0].can_id = 0x1ff; // 与同总线 DJI 命令组相同
        conflict.dm[0].bus = 2;
        Rig rig{conflict};
        CHECK(!rig.exec.assembled());
        rig.bus2.clear();
        rig.step();
        CHECK(rig.bus2.count == 0);
        CHECK(rig.exec.stats().group_tx_error == 0);
        CHECK(rig.exec.stats().target_rejections == 0);
    }
}

} // 测试辅助

// N1: 真正执行器的多速率提交、相位与单次 busy 不撤权; 不是仅测试计时器.
void multirate_contracts() {
    auto config = ec::sentinel::app::synthetic_config();
    config.runtime = ec::sentinel::app::RuntimeTiming{};
    Rig rig{config};
    for (unsigned i = 0; i < 10; ++i) {
        rig.at(100 + i);
        rig.feed_all();
        const auto before = rig.bus2.count;
        rig.go_local(100 + i);
        CHECK(rig.bus2.count == before + 1);
        CHECK(rig.bus2.frames[before].id == (i % 2 == 0 ? 0x200 : 0x1ff));
    }
    CHECK(rig.exec.stats().group_sent == 30); // 两 GM 组 20 帧 + CAN2 10 帧
    CHECK(rig.exec.group_stats(0)->computations == 5);
    CHECK(std::fabs(rig.exec.group_stats(0)->last_dt_s - 0.002f) < 1e-7f);
    CHECK(!rig.exec.group_stats(8));
    CHECK(rig.exec.stats().dm_query_sent == 5);
    CHECK(rig.exec.pitch_stats().computations == 5 && rig.exec.pitch_stats().submitted == 5);
    CHECK(rig.exec.stats().dm_enable_sent == 0 && rig.exec.stats().dm_command_sent == 0);
    const auto* query = rig.bus1.last(0x7ff);
    CHECK(query && query->size == 4 && query->data[0] == 1 && query->data[2] == 0xcc);
    CHECK(rig.exec.feedback_time(Domain::main_yaw)->us == 109123);

    rig.at(110);
    rig.feed_all();
    CHECK(rig.app.accept_targets(twist_packet(0.1f), rig.now, Source::remote) == AcceptResult::accepted);
    rig.bus2.next = SendResult::busy;
    rig.step();
    CHECK(rig.exec.axis(Domain::chassis_lf)->enabled());
    CHECK(rig.app.snapshot().permission.authority);
    CHECK(rig.exec.stats().group_tx_error == 1);
    rig.bus2.next = SendResult::accepted;
    rig.at(111);
    rig.feed_all();
    CHECK(rig.app.accept_targets(twist_packet(-0.1f), rig.now, Source::remote) == AcceptResult::accepted);
    rig.step();
    rig.at(112);
    rig.feed_all();
    rig.step();
    CHECK(slot_raw(*rig.bus2.last(0x200), 0) > 0); // 恢复发送的是新目标, 不补旧负电流
    rig.at(113);
    rig.feed_all();
    CHECK(rig.app.accept_targets(TargetSnapshot{}, rig.now, Source::remote) == AcceptResult::accepted);
    const auto before_stop = rig.bus2.count;
    rig.step();
    CHECK(rig.bus2.count == before_stop + 2); // 非到期底盘提前零帧 + 正常 shooter 组
    CHECK(slot_raw(*rig.bus2.last(0x200), 0) == 0);
    CHECK(std::fabs(rig.exec.group_stats(0)->last_dt_s - 0.001f) < 1e-7f);

    Rig jitter{config};
    jitter.go_local(99);
    jitter.bus2.clear();
    for (unsigned i = 0; i < 20; ++i) {
        jitter.at(100 + i);
        jitter.feed_all();
        const auto before = jitter.bus2.count;
        jitter.step(0.001f, ec::MicroTick{(100 + i) * 1000u + (i % 3) * 70u});
        CHECK(jitter.bus2.count == before + 1);
    }
    CHECK(jitter.exec.group_stats(0)->invalid_steps == 0);
    CHECK(jitter.exec.group_stats(0)->computations == 11);
    CHECK(!jitter.exec.stats().timing_observed);
    Rig instrumented{config, []() noexcept {
        static std::uint32_t us{};
        return ec::MicroTick{++us}; // 受控计数时钟, 不作为 F407 性能数据
    }};
    instrumented.go_local(100);
    CHECK(instrumented.exec.stats().timing_observed);
    CHECK(instrumented.exec.stats().control_last_us > 0);
    CHECK(instrumented.exec.stats().tx_last_us > 0);
    const auto timing = *instrumented.exec.group_stats(0);
    CHECK(timing.timing_observed && timing.last_submitted.us > timing.last_computed.us);
    const auto trace = instrumented.exec.trace(Domain::main_yaw, instrumented.now, instrumented.app);
    CHECK(trace && trace->kind == ec::sentinel::recording::Kind::control);
    CHECK(trace->source_sequence == timing.computations);
    CHECK(trace->data[7] == static_cast<std::uint32_t>(ec::drivers::DjiGroupStatus::accepted));
    CHECK(std::bit_cast<float>(trace->data[5]) > 0.0f);
}

// NR02: 正式 App/Executor 与实际 DM 字节链, 真实时间拒绝不能只停一拍再复活旧目标.
void pitch_rejection_contracts() {
    Rig rig;
    rig.at(100); rig.feed_all(); rig.go_local(100);
    TargetSnapshot packet{};
    packet.domains[index(Domain::pitch)] = DomainSetpoint{ControlMode::position, 0.3f, 0};
    CHECK(rig.app.accept_targets(packet, rig.now) == AcceptResult::accepted);
    rig.at(101); rig.feed_dm(0, 0); rig.step();
    CHECK(rig.bus1.last(1) && rig.bus1.last(1)->data[7] == 0xfc);
    rig.at(102); rig.feed_dm(1, 0); rig.step();
    CHECK(rig.exec.stats().dm_command_sent == 1);
    rig.at(103); rig.feed_dm(1, 0); rig.bus1.clear(); rig.step(0.0f);
    CHECK(rig.bus1.last(1) && rig.bus1.last(1)->data[7] == 0xfd);
    CHECK(rig.exec.health(rig.now).execution_rejected[index(Domain::pitch)]);
    const auto enables = rig.exec.stats().dm_enable_sent;
    const auto commands = rig.exec.stats().dm_command_sent;
    for (unsigned i=104; i<109; ++i) {
        rig.at(i); rig.feed_dm(i%2, 0); rig.step();
        CHECK(!rig.app.target(Domain::pitch));
        CHECK(rig.exec.stats().dm_enable_sent == enables);
        CHECK(rig.exec.stats().dm_command_sent == commands);
        CHECK(rig.app.snapshot().permission.authority);
    }
    rig.at(109); rig.feed_dm(0, 0);
    CHECK(rig.app.accept_targets(packet, rig.now) == AcceptResult::accepted);
    rig.step();
    CHECK(rig.exec.stats().dm_enable_sent == enables+1);
    rig.at(110); rig.feed_dm(1, 0); rig.step();
    CHECK(rig.exec.stats().dm_command_sent == commands+1);
}

// NR02/T04: 非到期与一次 busy 不是时间拒绝; 真步长异常和 MIT 编码拒绝必须撤旧意图.
void pitch_multirate_rejection_contracts() {
    auto config = ec::sentinel::app::synthetic_config();
    config.runtime = ec::sentinel::app::RuntimeTiming{};
    Rig rig{config};
    rig.at(100); rig.feed_all(); rig.go_local(100);
    TargetSnapshot packet{};
    packet.domains[index(Domain::pitch)] = DomainSetpoint{ControlMode::position, 0.3f, 0};
    CHECK(rig.app.accept_targets(packet, rig.now) == AcceptResult::accepted);
    rig.at(101); rig.feed_dm(0, 0); rig.step(); // 非到期.
    CHECK(rig.exec.stats().dm_enable_sent == 0 && rig.app.target(Domain::pitch));
    rig.at(102); rig.feed_dm(0, 0); rig.step();
    rig.at(103); rig.feed_dm(1, 0); rig.step();
    rig.at(104); rig.feed_dm(1, 0); rig.bus1.next=SendResult::busy; rig.step();
    CHECK(!rig.exec.health(rig.now).execution_rejected[index(Domain::pitch)]);
    rig.bus1.next=SendResult::accepted;
    rig.at(105); rig.feed_dm(1, 0); rig.step();
    rig.at(106); rig.feed_dm(1, 0); rig.step();
    CHECK(rig.exec.stats().dm_command_sent == 1 && rig.app.target(Domain::pitch));
    const auto generation = rig.app.snapshot().permission.generation;
    rig.at(116); rig.feed_dm(1, 0); rig.step(); // 真实组间隔 10 ms 超出组可靠步长.
    CHECK(rig.exec.health(rig.now).execution_rejected[index(Domain::pitch)]);
    rig.at(117); rig.feed_dm(0, 0); rig.step();
    CHECK(!rig.app.target(Domain::pitch));
    CHECK(rig.app.snapshot().permission.generation == generation);

    // 合法关节请求在当前设备量程里不能编码: 也必须经同一域拒绝通路.
    config = ec::sentinel::app::synthetic_config();
    config.dm[0].limits->position_max_rad=0.1f;
    Rig encoding{config};
    encoding.at(100); encoding.feed_all(); encoding.go_local(100);
    packet.domains[index(Domain::pitch)]->value=0.5f;
    CHECK(encoding.app.accept_targets(packet, encoding.now)==AcceptResult::accepted);
    encoding.at(101); encoding.feed_dm(0,0); encoding.step();
    encoding.at(102); encoding.feed_dm(1,0); encoding.step();
    CHECK(encoding.exec.health(encoding.now).execution_rejected[index(Domain::pitch)]);
    CHECK(encoding.exec.stats().dm_command_sent==0);
    encoding.at(103); encoding.feed_dm(1,0); encoding.step();
    CHECK(!encoding.app.target(Domain::pitch));
    CHECK(encoding.bus1.last(1) && encoding.bus1.last(1)->data[7]==0xfd);
}

struct SubmissionClock {
    static inline std::uint32_t value{}, reads{}, jump{}, jump_at{};
    static ec::MicroTick read() noexcept {
        if (++reads == jump_at) { value += jump; }
        return {value};
    }
    static void reset(std::uint32_t at, std::uint32_t delay = 0, std::uint32_t when = 0) {
        value = at; reads = 0; jump = delay; jump_at = when;
    }
};

// NR03: 保持基础 Tick 不动, 在各组计算后/发送前推进单调时钟, 不把模型零耗时当 TTL 证据.
void submission_deadline_contracts() {
    for (const auto base : {1002000u, 0xffffff00u}) {
        for (const auto age : {1999u, 2000u, 2001u}) {
            auto config = ec::sentinel::app::synthetic_config();
            config.runtime.chassis.output_ttl = {2000};
            SubmissionClock::reset(base-1000);
            Rig rig{config, SubmissionClock::read};
            rig.at(1000); rig.feed_all(); rig.step(0.001f, ec::MicroTick{base-1000});
            rig.inputs.link.remote_online = true;
            rig.inputs.link.switch_left = Dr16Switch::up;
            rig.inputs.link.switch_right = Dr16Switch::mid;
            SubmissionClock::reset(base-500);
            rig.at(1001); rig.feed_all(); rig.step(0.001f, ec::MicroTick{base-500});
            CHECK(rig.app.accept_targets(twist_packet(0.2f), rig.now) == AcceptResult::accepted);
            rig.at(1002); rig.feed_all(); rig.bus1.clear(); rig.bus2.clear();
            // 第一次观察为控制入口, 随后四组计算, 第六次为提交阶段入口.
            SubmissionClock::reset(base, age, 6);
            rig.step(0.001f, ec::MicroTick{base});
            CHECK(rig.exec.group_stats(0)->last_computed.us == base);
            const auto* frame = rig.bus2.last(0x200);
            if (age < 2000) {
                CHECK(frame && slot_raw(*frame, 0) != 0);
            } else {
                CHECK(frame == nullptr);
                CHECK(rig.exec.group_stats(0)->last_result == ec::drivers::DjiGroupStatus::stale_output);
                CHECK(rig.exec.health(rig.now).execution_rejected[index(Domain::chassis_lf)]);
            }
        }
    }
}

// NR03/T06: 一个真实槽撤目标会提前算整组, 同组仍执行的槽不能沿用旧输出或时间.
void early_group_deadline_contracts() {
    for (unsigned scenario=0;scenario<3;++scenario) {
        const auto delay=scenario==1 ? 2000u : 0u;
        const bool busy=scenario==2;
        auto config=ec::sentinel::app::synthetic_config();
        config.runtime.shooter={2000,1000,{2000},6000};
        SubmissionClock::reset(1000000);
        Rig rig{config, SubmissionClock::read};
        rig.at(1000); rig.feed_all(); rig.go_local(1000);
        TargetSnapshot targets{};
        targets.domains[index(Domain::friction_left)]={ControlMode::velocity,3.0f,0};
        targets.domains[index(Domain::friction_right)]={ControlMode::velocity,3.0f,0};
        CHECK(rig.app.accept_targets(targets,rig.now)==AcceptResult::accepted);
        SubmissionClock::reset(1001000);
        rig.at(1001); rig.feed_all(); rig.step();
        CHECK(rig.bus2.last(0x1ff) && slot_raw(*rig.bus2.last(0x1ff),1)!=0);
        targets.domains[index(Domain::friction_left)].reset();
        rig.at(1002); rig.feed_all();
        CHECK(rig.app.accept_targets(targets,rig.now)==AcceptResult::accepted);
        rig.bus2.clear(); rig.bus2.next=busy ? SendResult::busy : SendResult::accepted;
        SubmissionClock::reset(1002000,delay,6); rig.step();
        const auto* frame=rig.bus2.last(0x1ff);
        if (busy) {
            CHECK(!frame);
            CHECK(!rig.exec.health(rig.now).execution_rejected[index(Domain::friction_right)]);
        } else if (!delay) {
            CHECK(frame && slot_raw(*frame,0)==0 && slot_raw(*frame,1)!=0);
        } else {
            CHECK(!frame);
            CHECK(rig.exec.health(rig.now).execution_rejected[index(Domain::friction_right)]);
        }
        CHECK(std::fabs(rig.exec.group_stats(3)->last_dt_s-0.001f)<1e-7f);
        CHECK(rig.exec.group_stats(3)->last_computed.us==1002000);
        if (busy) {
            // 恢复提交时方向已反转, 不重发 busy 前的正向电流, 停止槽仍为零.
            rig.bus2.next=SendResult::accepted;
            rig.at(1004); rig.feed_all(); SubmissionClock::reset(1004000);
            targets.domains[index(Domain::friction_right)]->value=-3.0f;
            CHECK(rig.app.accept_targets(targets,rig.now)==AcceptResult::accepted);
            rig.step();
            const auto* resumed=rig.bus2.last(0x1ff);
            CHECK(resumed && slot_raw(*resumed,0)==0 && slot_raw(*resumed,1)<0);
            CHECK(std::fabs(rig.exec.group_stats(3)->last_dt_s-0.002f)<1e-7f);
        }
    }
}

int main() {
    single_gm6020_execution_contracts();
    four_motor_execution_contracts();
    pitch_rejection_contracts();
    pitch_multirate_rejection_contracts();
    submission_deadline_contracts();
    early_group_deadline_contracts();
    multirate_contracts();
    chassis_contracts();
    group_contracts();
    joint_contracts();
    branch_contracts();
    pitch_contracts();
    pitch_stop_lifecycle_contracts();
    joint_geometry_reference_contracts();
    feedback_loss_contracts();
    readiness_and_layout_contracts();
    std::puts("sentinel app execution contracts passed");
    return 0;
}
