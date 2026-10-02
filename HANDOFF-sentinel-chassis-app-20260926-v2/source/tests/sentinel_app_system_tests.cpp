// O7 全 APP 原生组合与闭环离线验收.
//
// 同一个真实 APP + 执行器 + 感知 + 参数存储 + 协议会话, 运行在合成一阶负载与记录原始
// CAN 字节的软件总线上: 证据链是 "协议请求 → 采用目标 → 真实 C++ 环路 → 原始 CAN 输出
// → 负载运动 → 反馈字节 → 设备解码 → 状态回包". 不是孤立函数测试, 也不是两个 mock 互测.
//
// 边界: 负载是夹具 (不是实车动力学), 因此本文件的结论只覆盖方向/单位/生命周期/编解码
// 往返与软件门禁; 稳定性、动力参数、机械能力仍属实机验收项.
#include "ec/control/control_mode.hpp"
#include "ec/drivers/dji.hpp"
#include "ec/drivers/dm.hpp"
#include "ec/time.hpp"
#include "sentinel_app.hpp"
#include "sentinel_execution.hpp"
#include "sentinel_params.hpp"
#include "sentinel_perception.hpp"
#include "sentinel_protocol.hpp"
#include "sentinel_types.hpp"
#include "sim_plant.hpp"
#include "mechanism_fixture.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <span>
#include <vector>

namespace {
int failures = 0;
void check(bool condition, const char* text, int line) {
    if (condition) { return; }
    std::printf("line %d: %s\n", line, text);
    ++failures;
}
#define CHECK(expr) check((expr), #expr, __LINE__)
[[nodiscard]] bool near(float value, float expected, float tolerance) {
    return std::fabs(value - expected) <= tolerance;
}

using namespace ec;
using namespace ec::sentinel;
using namespace ec::sentinel::app;
using ec::sentinel::host::SimPlant;
using ec::sentinel::host::SoftwareBus;

constexpr float dt_s = 0.001f;
constexpr float pi = 3.14159265358979323846f;

// 闭环试验台: 真实应用对象 + 合成负载 + 软件总线, 所有时序都按 1 ms 显式推进.
struct Rig {
    SentinelConfig config;
    ec::sentinel::host::Mechanism fixture{ec::sentinel::host::Mechanism::all};
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    SentinelApp app;
    SentinelExecutor exec;
    SentinelPerception perception;
    ParamStore params;
    SentinelHostSession session;
    SimPlant plant{};
    AppInputs inputs{};
    Tick now{1000}; // 从非零时刻开始: 0 被用作"没有样本"的哨兵
    std::uint32_t session_id{};
    std::uint32_t sequence{};
    std::array<std::uint8_t, protocol_payload_max> request{};
    std::array<std::uint8_t, protocol_payload_max> response{};
    std::size_t response_length{};
    std::uint8_t response_type{};
    // 目标流: 真实上位机会按 TTL 内的周期刷新目标, 因此 run() 默认也按周期刷新;
    // 停止刷新 (stop_targets) 才能观察目标过期路径.
    struct TargetStream {
        std::array<DomainSetpoint, domain_count> setpoints{};
        std::optional<motion::PlanarTwist> twist{};
        bool active{};
        int refresh_cycles{10};
        int since_refresh{};
    };
    TargetStream stream{};
    // 惯性失效注入: 系统 Rig 每周期从执行器重建故障输入, 因此单独保留注入开关.
    bool inertial_only{};
    ResultCode last_target_result{ResultCode::rejected};

    explicit Rig(SentinelConfig cfg = synthetic_config(), ec::sentinel::host::Mechanism part = ec::sentinel::host::Mechanism::all)
        : config(cfg), fixture(part), app(config, ConfigAudience::offline_test),
            exec(config, ConfigAudience::offline_test, bus1, bus2),
            perception(config, ConfigAudience::offline_test), params(config),
            session(app, params, &exec, &perception) {
        inputs.link.locally_ready = true;
        inputs.link.remote_online = true;
        inputs.link.switch_left = drivers::Dr16Switch::down; // 上位机来源
        inputs.link.switch_right = drivers::Dr16Switch::mid;
        inputs.link.remote_sampled_at = now;
        inputs.aux.launch_permit_valid = true;
        inputs.aux.launch_permit = true;
        inputs.aux.launch_ttl = Lifetime{100};
    }

    // 一个 1 ms 闭环周期: 反馈 → 设备 → 应用 → 执行 → 负载.
    void cycle() {
        plant.emit_feedback(config,
            [this](std::uint8_t bus, const transport::CanFrame& frame, Tick at) {
                if (ec::sentinel::host::selected_feedback(fixture, config, bus, frame.id)) {
                    exec.route(bus, frame, at);
                }
            },
            now);
        inputs.faults = exec.health(now);
        inputs.faults.main_yaw_inertial_only = inertial_only;
        perception.update(now);
        inputs.inertial = perception.inertial();
        for (auto& entry : inputs.feedback) { entry = DomainFeedback{}; }
        for (std::size_t i = 0; i < dji_slot_count; ++i) {
            if (!config.dji[i].installed) { continue; }
            auto* device = exec.dji_device(i);
            if (device == nullptr) { continue; }
            const auto state = device->state(now);
            auto& entry = inputs.feedback[domain_index(config.dji[i].domain)];
            entry.position_rad = state.position_rad;
            entry.velocity_rad_s = state.velocity_rad_s;
        }
        if (config.dm[0].installed) {
            if (auto* device = exec.dm_device()) {
                const auto state = device->state();
                if (state.feedback.validity == Validity::valid) {
                    auto& entry = inputs.feedback[domain_index(config.dm[0].domain)];
                    entry.position_rad = Sample<float>{
                        state.feedback.value.position_rad, state.feedback.sampled_at, Validity::valid};
                    entry.velocity_rad_s = Sample<float>{
                        state.feedback.value.velocity_rad_s, state.feedback.sampled_at, Validity::valid};
                }
            }
        }
        inputs.link.remote_sampled_at = now;
        inputs.aux.launch_sampled_at = now;
        inputs.epochs = Epochs{app.config_epoch(), app.reference_epoch()};
        // 目标刷新在 update 之前: 采用的是本周期刚到达的目标.
        if (stream.active) {
            if (stream.since_refresh == 0) {
                last_target_result = targets(stream.setpoints, stream.twist);
            }
            if (++stream.since_refresh >= stream.refresh_cycles) { stream.since_refresh = 0; }
        }
        app.update(now, inputs);
        bus1.clear();
        bus2.clear();
        exec.step(now, dt_s, app);
        plant.integrate(config, bus1, bus2, dt_s);
        ++now.ms;
    }
    void run(int cycles) {
        for (int i = 0; i < cycles; ++i) { cycle(); }
    }

    // ---- 协议辅助: 请求与响应都走真实载荷编解码 ----
    template<typename Fill>
    void encode_request_at(MessageType type, std::uint32_t seq, Fill&& fill) {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(type));
        w.u32(session_id);
        w.u32(seq);
        fill(w);
        response_length = 0;
        response_type = 0;
        const auto length = session.handle({request.data(), w.size()}, now, response);
        if (length == 0) { return; }
        response_length = length;
        // 响应头统一是 版本/类型/会话/序号 (结果码只出现在 ack 类响应里).
        PayloadReader r{{response.data(), length}};
        static_cast<void>(r.u8());
        response_type = r.u8();
    }
    template<typename Fill>
    void encode_request(MessageType type, Fill&& fill) {
        encode_request_at(type, ++sequence, std::move(fill));
    }
    void hello() {
        // 握手携带客户端 nonce: 服务端回显它, 客户端据此确认这份 hello_ack 属于本次握手.
        // 握手是唯一不带会话/序号的请求 (此时还没有会话身份), 因此不走通用编码辅助.
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::hello));
        w.u32(0x55aa1234u);
        response_length = 0;
        response_type = 0;
        const auto length = session.handle({request.data(), w.size()}, now, response);
        if (length != 0) {
            response_length = length;
            PayloadReader head{{response.data(), length}};
            static_cast<void>(head.u8());
            response_type = head.u8();
        }
        CHECK(response_type == static_cast<std::uint8_t>(MessageType::hello_ack));
        PayloadReader r{{response.data(), response_length}};
        static_cast<void>(r.u16());
        session_id = r.u32();
        static_cast<void>(r.u32()); // config_epoch
        static_cast<void>(r.u32()); // reference_epoch
        static_cast<void>(r.u32()); // now
        static_cast<void>(r.u32()); // boot_id
        static_cast<void>(r.u32()); // client_nonce
        sequence = 0;
    }
    [[nodiscard]] ResultCode result_code() const {
        PayloadReader r{{response.data(), response_length}};
        static_cast<void>(r.u16());
        static_cast<void>(r.u32());
        static_cast<void>(r.u32());
        return static_cast<ResultCode>(r.u8());
    }
    [[nodiscard]] ResultCode authorize(bool grant, Source source) {
        encode_request(MessageType::authorize, [&](PayloadWriter& w) {
            w.u8(grant ? 1 : 0);
            w.u8(static_cast<std::uint8_t>(source));
        });
        return result_code();
    }
    [[nodiscard]] ResultCode targets(std::span<const DomainSetpoint> setpoints,
        std::optional<motion::PlanarTwist> twist, bool use_current_epochs = true) {
        return targets_at(++sequence, setpoints, twist, use_current_epochs);
    }
    // 显式序号: 用于重放/乱序等错误路径 (序号由调用方决定).
    [[nodiscard]] ResultCode targets_at(std::uint32_t seq, std::span<const DomainSetpoint> setpoints,
        std::optional<motion::PlanarTwist> twist, bool use_current_epochs = true) {
        encode_request_at(MessageType::targets, seq, [&](PayloadWriter& w) {
            w.u32(use_current_epochs ? app.config_epoch() : app.config_epoch() + 1);
            w.u32(use_current_epochs ? app.reference_epoch() : app.reference_epoch() + 1);
            // 短于域数的 span (含空 span) 表示"其余域没有目标", 不越界读取.
            const auto mode_at = [&](std::size_t index) {
                return index < setpoints.size() ? setpoints[index].mode
                                                : control::ControlMode::none;
            };
            std::uint8_t count = 0;
            for (std::size_t i = 0; i < domain_count; ++i) {
                if (mode_at(i) != control::ControlMode::none) { ++count; }
            }
            w.u8(count);
            for (std::size_t i = 0; i < domain_count; ++i) {
                if (mode_at(i) == control::ControlMode::none) { continue; }
                w.u8(static_cast<std::uint8_t>(i));
                w.u8(static_cast<std::uint8_t>(setpoints[i].mode));
                w.u8(static_cast<std::uint8_t>(setpoints[i].frame));
                w.f32(setpoints[i].value);
                // 每域自带源年龄: 整包刷新不会把旧域一起续命.
                w.u16(setpoints[i].age_ms);
            }
            w.u8(twist ? 1 : 0);
            if (twist) {
                w.f32(twist->vx_mps);
                w.f32(twist->vy_mps);
                w.f32(twist->wz_rad_s);
                w.u16(0);
            }
        });
        return result_code();
    }
    // 记录目标流并立即下发一次; run() 期间按 refresh_cycles 刷新.
    ResultCode set_targets(std::span<const DomainSetpoint> setpoints,
        std::optional<motion::PlanarTwist> twist) {
        for (std::size_t i = 0; i < domain_count; ++i) {
            stream.setpoints[i] = i < setpoints.size() ? setpoints[i] : DomainSetpoint{};
        }
        stream.twist = twist;
        stream.active = true;
        stream.since_refresh = 1;
        last_target_result = targets(stream.setpoints, stream.twist);
        return last_target_result;
    }
    // 停止刷新: 模拟上位机不再下发目标 (目标随后按 TTL 过期).
    void stop_targets() { stream.active = false; }
    // 只发一个域的简单目标 (其余域保持"没有目标").
    [[nodiscard]] ResultCode target_one(Domain domain, control::ControlMode mode, float value) {
        std::array<DomainSetpoint, domain_count> setpoints{};
        setpoints[domain_index(domain)] = DomainSetpoint{mode, value};
        return set_targets(setpoints, std::nullopt);
    }
    // 读一次状态回包 (只解析需要的字段).
    struct StatusView {
        ResultCode result{ResultCode::rejected};
        Source source{Source::none};
        bool authority{};
        bool manual_stop{};
        std::uint32_t config_epoch{};
        std::uint32_t reference_epoch{};
        struct Entry {
            DomainValidity validity{DomainValidity::unknown};
            control::ControlMode mode{control::ControlMode::none};
            bool execute{};
            Reason reason{Reason::none};
            float target{};
            float position_rad{};
            std::uint32_t position_age_ms{};
            std::uint32_t velocity_age_ms{};
        };
        std::array<Entry, domain_count> domains{};
        [[nodiscard]] const Entry& at(Domain domain) const { return domains[domain_index(domain)]; }
    };
    [[nodiscard]] StatusView status() {
        encode_request(MessageType::status, [](PayloadWriter& w) { w.u8(1); });
        StatusView view{};
        if (response_length == 12) { view.result = static_cast<ResultCode>(response[10]); return view; }
        PayloadReader r{{response.data(), response_length}};
        static_cast<void>(r.u8());
        static_cast<void>(r.u8());
        // 应答回显原请求身份 (会话 + 业务序号).
        static_cast<void>(r.u32());
        static_cast<void>(r.u32());
        CHECK(r.u8() == 1); // 全域执行页
        view.config_epoch = r.u32();
        view.reference_epoch = r.u32();
        static_cast<void>(r.u32()); // now
        view.source = static_cast<Source>(r.u8());
        view.authority = r.u8() != 0;
        view.manual_stop = r.u8() != 0;
        static_cast<void>(r.u8()); // launch
        const auto count = r.u8();
        for (std::uint8_t i = 0; i < count && i < domain_count; ++i) {
            auto& entry = view.domains[i];
            entry.validity = static_cast<DomainValidity>(r.u8());
            entry.mode = static_cast<control::ControlMode>(r.u8());
            entry.execute = r.u8() != 0;
            entry.reason = static_cast<Reason>(r.u8());
            static_cast<void>(r.u8()); // 请求/采用/测量参考系
            entry.target = r.f32();
            entry.position_rad = r.f32();
            entry.position_age_ms = r.u32();
            entry.velocity_age_ms = r.u32();
        }
        view.result = r.ok() ? ResultCode::accepted : ResultCode::bad_length;
        return view;
    }

    // ---- 观测辅助 ----
    [[nodiscard]] std::optional<std::int16_t> command_raw(Domain domain) {
        const auto slot = app.dji_slot_for(domain);
        if (!slot) { return std::nullopt; }
        const auto& entry = config.dji[*slot];
        const auto protocol = drivers::dji_protocol(entry.mode);
        if (!protocol) { return std::nullopt; }
        const auto& bus = entry.bus == 1 ? bus1 : bus2;
        const auto group_id = entry.device_id > 4 ? protocol->second_tx : protocol->first_tx;
        const auto frame = bus.last_with_id(group_id);
        if (!frame) { return std::nullopt; }
        return host::raw_slot(*frame, static_cast<unsigned>((entry.device_id - 1) % 4));
    }
    [[nodiscard]] float axis_velocity(Domain domain) {
        const auto slot = app.dji_slot_for(domain);
        if (!slot) { return 0.0f; }
        return plant.dji[*slot].velocity_rad_s;
    }
    [[nodiscard]] float axis_position(Domain domain) {
        const auto slot = app.dji_slot_for(domain);
        if (!slot) { return 0.0f; }
        return plant.dji[*slot].position_rad;
    }
    // 建立关节参考: 走真实协议的 align 请求 (会按设计上推参考代次, 作废旧目标).
    [[nodiscard]] bool align(Domain domain, float joint_rad) {
        encode_request(MessageType::align, [&](PayloadWriter& w) {
            w.u8(static_cast<std::uint8_t>(domain_index(domain)));
            w.u8(0); // kind = 机械关节参考
            w.f32(joint_rad);
            w.u32(app.reference_epoch());
        });
        return result_code() == ResultCode::accepted;
    }
    // 让负载先动起来, 便于验证"零目标纠偏"和"角保持".
    void spin(Domain domain, float velocity_rad_s) {
        const auto slot = app.dji_slot_for(domain);
        if (!slot) { return; }
        plant.dji[*slot].velocity_rad_s = velocity_rad_s;
    }
};

// 应用/权限/输入链路都齐备时的常用起点: 握手 + 授权 + 拨杆已选上位机.
void ready(Rig& rig, int settle_cycles = 5) {
    rig.hello();
    CHECK(rig.authorize(true, Source::host) == ResultCode::accepted);
    rig.run(settle_cycles);
}
} // 命名空间

// ---------------------------------------------------------------------------
// 场景 1: 九 DJI + 一 DM 的并发闭环
// ---------------------------------------------------------------------------
void test_full_closed_loop() {
    Rig rig;
    ready(rig);
    // 关节参考: 位置域必须先建立参考 (单圈编码器不自行恢复多圈角).
    CHECK(rig.align(Domain::main_yaw, 0.0f));
    CHECK(rig.align(Domain::fire_yaw, 0.0f));
    std::array<DomainSetpoint, domain_count> setpoints{};
    setpoints[domain_index(Domain::main_yaw)] =
        DomainSetpoint{control::ControlMode::position, 0.2f};
    setpoints[domain_index(Domain::fire_yaw)] =
        DomainSetpoint{control::ControlMode::position, -0.15f};
    setpoints[domain_index(Domain::pitch)] =
        DomainSetpoint{control::ControlMode::position, 0.1f};
    setpoints[domain_index(Domain::friction_left)] =
        DomainSetpoint{control::ControlMode::velocity, 3.0f};
    setpoints[domain_index(Domain::friction_right)] =
        DomainSetpoint{control::ControlMode::velocity, 3.0f};
    setpoints[domain_index(Domain::trigger)] =
        DomainSetpoint{control::ControlMode::velocity, 3.0f};
    CHECK(rig.set_targets(setpoints, motion::PlanarTwist{0.1f, 0.0f, 0.0f}) ==
        ResultCode::accepted);
    // 夹具位置环增益偏小 (合成值, 不是实车整定结果): 给足收敛时间.
    rig.run(800);
    // 权限与来源来自协议会话 + 拨杆, 不是"目标到达即授权".
    const auto view = rig.status();
    CHECK(view.source == Source::host);
    CHECK(view.authority);
    CHECK(!view.manual_stop);
    // 每个域都真的在执行, 反馈到位后有效性为 valid.
    for (std::size_t i = 0; i < domain_count; ++i) {
        CHECK(view.domains[i].execute);
        CHECK(view.domains[i].validity == DomainValidity::valid);
    }
    // 底盘 twist 未超限: 不做缩放.
    CHECK(near(rig.app.twist_scale(), 1.0f, 1.0e-3f));
    // 原始 CAN: 原始命令符号与采用目标同号, 负载速度同向 → 方向/单位一致.
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        const auto raw = rig.command_raw(domain);
        CHECK(raw.has_value());
        if (!raw) { continue; }
        CHECK(*raw != 0);
        CHECK(*raw * view.domains[i].target > 0.0f);
        CHECK(rig.axis_velocity(domain) * view.domains[i].target > 0.0f);
        // 稳态速度落在采用目标附近 (夹具速度环无积分, 允许稳态误差).
        CHECK(std::fabs(rig.axis_velocity(domain)) > 0.5f * std::fabs(view.domains[i].target));
    }
    const auto friction_left = rig.command_raw(Domain::friction_left);
    const auto friction_right = rig.command_raw(Domain::friction_right);
    CHECK(friction_left && *friction_left < 0); // 方向 -1: 正目标 → 负原始量
    CHECK(friction_right && *friction_right > 0);
    const auto trigger = rig.command_raw(Domain::trigger);
    CHECK(trigger && *trigger < 0);
    CHECK(rig.axis_velocity(Domain::friction_left) > 0.0f);
    // 达妙: 先发使能, 反馈确认 Enabled 之后才下发 MIT 目标.
    CHECK(rig.exec.stats().dm_enable_sent == 1);
    CHECK(rig.exec.stats().dm_command_sent > 700);
    CHECK(rig.exec.pitch_enable_confirmed());
    CHECK(near(rig.plant.dm.position_rad, 0.1f, 0.01f));
    // 组发送没有被拒绝: 每个已装配槽都有本周期输出.
    CHECK(rig.exec.stats().group_refused == 0);
    CHECK(rig.exec.stats().group_tx_error == 0);
    CHECK(rig.exec.stats().group_sent > 0);
    // 位置域收敛到目标附近 (两 yaw 与 pitch).
    CHECK(near(rig.axis_position(Domain::main_yaw), 0.2f, 0.03f));
    CHECK(near(rig.axis_position(Domain::fire_yaw), -0.15f, 0.03f));
}

// ---------------------------------------------------------------------------
// 场景 2: 部分域为空 — 未下发的域撤输出, 不隐式跟随也不假装零目标
// ---------------------------------------------------------------------------
void test_partial_targets_revoke_others() {
    Rig rig;
    ready(rig);
    CHECK(rig.set_targets({}, motion::PlanarTwist{0.1f, 0.0f, 0.0f}) == ResultCode::accepted);
    rig.run(50);
    const auto view = rig.status();
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        CHECK(view.domains[i].execute);
    }
    // 未下发的域: 无目标 → 显式撤输出 (零电流), 而不是"零速度目标".
    for (std::size_t i = chassis_wheel_count; i < domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        if (domain == Domain::pitch) { continue; } // 达妙走 disable 请求路径
        CHECK(!view.domains[i].execute);
        CHECK(view.domains[i].reason == Reason::no_target);
        const auto raw = rig.command_raw(domain);
        CHECK(raw.has_value());
        if (raw) { CHECK(*raw == 0); }
    }
    // 达妙未下发目标: 不请求使能, 也不发 MIT 帧.
    CHECK(!rig.exec.pitch_enable_requested());
    CHECK(rig.exec.stats().dm_command_sent == 0);
}

// ---------------------------------------------------------------------------
// 场景 3: 设备缺失/故障的依赖影响 (A09) 与同组发送
// ---------------------------------------------------------------------------
void test_device_missing_dependencies() {
    // (a) 底盘单轮失去反馈: A09 撤销四轮输出, 但同总线其它组 (摩擦/拨弹) 不受影响,
    //     且组帧仍然发送 (失效槽位明确为 0), 不是整组静默停发.
    {
        Rig rig;
        ready(rig);
        std::array<DomainSetpoint, domain_count> setpoints{};
        for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
            setpoints[i] = DomainSetpoint{control::ControlMode::velocity, 2.0f};
        }
        setpoints[domain_index(Domain::friction_left)] =
            DomainSetpoint{control::ControlMode::velocity, 2.0f};
        setpoints[domain_index(Domain::friction_right)] =
            DomainSetpoint{control::ControlMode::velocity, 2.0f};
        setpoints[domain_index(Domain::trigger)] =
            DomainSetpoint{control::ControlMode::velocity, 2.0f};
        CHECK(rig.set_targets(setpoints, std::nullopt) == ResultCode::accepted);
        rig.run(50);
        CHECK(rig.status().at(Domain::chassis_lf).execute);
        CHECK(rig.status().at(Domain::friction_left).execute);
        // 左前轮停止上报: 四轮全部撤输出 (底盘整体故障依赖), 摩擦/拨弹继续.
        rig.plant.dji[0].emitting = false;
        rig.run(60);
        const auto view = rig.status();
        for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
            CHECK(!view.domains[i].execute);
            // 协议层只暴露 reason/validity; 故障分类直接读应用快照 (测试侧内部检查).
            const auto fault = rig.app.snapshot().domains[i].fault;
            CHECK(fault == FaultState::dependency_fault || fault == FaultState::feedback_lost);
        }
        CHECK(view.at(Domain::chassis_lf).validity == DomainValidity::stale);
        CHECK(view.at(Domain::friction_left).execute);
        CHECK(view.at(Domain::trigger).execute);
        // CAN2 组 1 (四轮) 仍然发送, 且四个槽位都是 0 (撤输出), 不是"没发帧".
        const auto lf = rig.command_raw(Domain::chassis_lf);
        const auto rf = rig.command_raw(Domain::chassis_rf);
        CHECK(lf.has_value());
        CHECK(rf.has_value());
        if (lf) { CHECK(*lf == 0); }
        if (rf) { CHECK(*rf == 0); }
        CHECK(rig.exec.stats().group_refused == 0);
        // 同总线另一组 (摩擦/拨弹) 的值仍然有效: 故障没有波及无关设备.
        const auto friction = rig.command_raw(Domain::friction_left);
        CHECK(friction.has_value());
        if (friction) { CHECK(*friction != 0); }
    }
    // (b) 主 yaw 失去反馈: 只撤该轴并禁止正向拨弹, fire yaw 与底盘不受影响.
    {
        Rig rig;
        ready(rig);
        CHECK(rig.align(Domain::fire_yaw, 0.0f));
        std::array<DomainSetpoint, domain_count> setpoints{};
        setpoints[domain_index(Domain::main_yaw)] =
            DomainSetpoint{control::ControlMode::position, 0.1f};
        setpoints[domain_index(Domain::fire_yaw)] =
            DomainSetpoint{control::ControlMode::position, 0.1f};
        setpoints[domain_index(Domain::trigger)] =
            DomainSetpoint{control::ControlMode::velocity, 2.0f};
        CHECK(rig.set_targets(setpoints, motion::PlanarTwist{0.05f, 0.0f, 0.0f}) ==
            ResultCode::accepted);
        rig.run(60);
        CHECK(rig.status().at(Domain::main_yaw).execute);
        CHECK(rig.status().at(Domain::trigger).execute);
        rig.plant.dji[4].emitting = false;
        rig.run(60);
        const auto view = rig.status();
        CHECK(!view.at(Domain::main_yaw).execute);
        CHECK(view.at(Domain::fire_yaw).execute);
        CHECK(view.at(Domain::chassis_lf).execute);
        // 正向拨弹被禁止 (原因明确是依赖故障), 不是设备自身故障.
        CHECK(!view.at(Domain::trigger).execute);
        CHECK(view.at(Domain::trigger).reason == Reason::fault_dependency);
        // 人工反转不受该禁止影响: 反向目标仍然执行.
        CHECK(rig.target_one(Domain::trigger, control::ControlMode::velocity, -2.0f) ==
            ResultCode::accepted);
        rig.run(20);
        CHECK(rig.status().at(Domain::trigger).execute);
        const auto trigger = rig.command_raw(Domain::trigger);
        CHECK(trigger.has_value());
        if (trigger) { CHECK(*trigger > 0); }
    }
}

// ---------------------------------------------------------------------------
// 场景 4: 零速度纠偏 — 已经在动的轴必须被反向拉回
// ---------------------------------------------------------------------------
void test_zero_velocity_correction() {
    Rig rig;
    ready(rig);
    rig.spin(Domain::chassis_lf, 2.0f);
    CHECK(rig.target_one(Domain::chassis_lf, control::ControlMode::velocity, 0.0f) ==
        ResultCode::accepted);
    rig.run(5);
    const auto raw = rig.command_raw(Domain::chassis_lf);
    CHECK(raw.has_value());
    if (raw) { CHECK(*raw < 0); } // 正速度 → 负命令
    rig.run(300);
    CHECK(near(rig.axis_velocity(Domain::chassis_lf), 0.0f, 0.05f));
}

// ---------------------------------------------------------------------------
// 场景 5: 角保持 — 位置目标受扰动后回到目标
// ---------------------------------------------------------------------------
void test_angle_hold_under_disturbance() {
    Rig rig;
    ready(rig);
    CHECK(rig.align(Domain::main_yaw, 0.0f));
    CHECK(rig.target_one(Domain::main_yaw, control::ControlMode::position, 0.3f) ==
        ResultCode::accepted);
    // 夹具位置环时间常数约 0.5 s (合成增益): 给足收敛时间.
    rig.run(1500);
    CHECK(near(rig.axis_position(Domain::main_yaw), 0.3f, 0.02f));
    // 外部扰动 (夹具外力, 不是通过命令下发): 位置环必须抵抗它, 并且撤除后回到目标.
    // 直接瞬移负载是不合法的扰动: 设备层会按最大转速判定编码器跳变并丢掉位置.
    rig.plant.external_accel[4] = 2.0f;
    rig.run(300);
    CHECK(rig.status().at(Domain::main_yaw).execute);
    CHECK(std::fabs(rig.axis_position(Domain::main_yaw) - 0.3f) < 0.06f);
    rig.plant.external_accel[4] = 0.0f;
    rig.run(1500);
    CHECK(near(rig.axis_position(Domain::main_yaw), 0.3f, 0.02f));
}

// ---------------------------------------------------------------------------
// 场景 6: 比例缩放 — 超限 twist 同比例缩放且不越协议上限
// ---------------------------------------------------------------------------
void test_twist_scaling() {
    Rig rig;
    ready(rig);
    // 夹具轮半径 0.06 m, 轴侧上限 50 rad/s → 单轮约 3 m/s, 因此 5 m/s 一定超限.
    CHECK(rig.set_targets({}, motion::PlanarTwist{5.0f, 0.0f, 0.0f}) == ResultCode::accepted);
    rig.run(20);
    const auto scale = rig.app.twist_scale();
    CHECK(scale > 0.0f && scale < 1.0f);
    const auto adopted = rig.app.adopted_twist();
    CHECK(adopted.has_value());
    if (adopted) {
        CHECK(near(adopted->vx_mps, 5.0f * scale, 1.0e-3f));
        CHECK(near(adopted->vy_mps, 0.0f, 1.0e-4f));
    }
    // 缩放后每个轮子的原始命令都不越设备编码上限.
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        const auto raw = rig.command_raw(domain);
        const auto slot = rig.app.dji_slot_for(domain);
        CHECK(raw.has_value());
        CHECK(slot.has_value());
        if (raw && slot) {
            CHECK(std::abs(*raw) <= *rig.config.dji[*slot].output_limit_raw);
        }
    }
    // 缩放不是"改方向": 请求值与采用值同号.
    CHECK(adopted && adopted->vx_mps > 0.0f);
}

// ---------------------------------------------------------------------------
// 场景 7: 摩擦轮关闭时拨弹 — 不因为摩擦轮未激活/零目标而阻止拨弹
// ---------------------------------------------------------------------------
void test_friction_off_trigger_allowed() {
    Rig rig;
    ready(rig);
    // 摩擦轮显式零速度目标 (有效零值目标), 拨弹同时给正速度目标.
    std::array<DomainSetpoint, domain_count> setpoints{};
    setpoints[domain_index(Domain::friction_left)] =
        DomainSetpoint{control::ControlMode::velocity, 0.0f};
    setpoints[domain_index(Domain::friction_right)] =
        DomainSetpoint{control::ControlMode::velocity, 0.0f};
    setpoints[domain_index(Domain::trigger)] =
        DomainSetpoint{control::ControlMode::velocity, 3.0f};
    CHECK(rig.set_targets(setpoints, std::nullopt) == ResultCode::accepted);
    rig.run(50);
    const auto view = rig.status();
    CHECK(view.at(Domain::friction_left).execute);
    CHECK(view.at(Domain::friction_right).execute);
    CHECK(view.at(Domain::trigger).execute);
    CHECK(near(view.at(Domain::friction_left).target, 0.0f, 1.0e-6f));
    const auto trigger = rig.command_raw(Domain::trigger);
    CHECK(trigger.has_value());
    if (trigger) { CHECK(*trigger != 0); }
    // 摩擦轮完全没有目标时同样不阻止拨弹 (另一种"摩擦轮关闭").
    Rig second;
    ready(second);
    CHECK(second.target_one(Domain::trigger, control::ControlMode::velocity, 3.0f) ==
        ResultCode::accepted);
    second.run(50);
    CHECK(second.status().at(Domain::trigger).execute);
    const auto friction = second.command_raw(Domain::friction_left);
    CHECK(friction.has_value());
    if (friction) { CHECK(*friction == 0); }
}

// ---------------------------------------------------------------------------
// 场景 8: 主 yaw 惯性降级仍拨弹 (G07) + 惯性恢复后降级解除
// ---------------------------------------------------------------------------
void test_degraded_main_yaw_still_triggers() {
    Rig rig;
    ready(rig);
    CHECK(rig.align(Domain::main_yaw, 0.0f));
    CHECK(rig.target_one(Domain::trigger, control::ControlMode::velocity, 3.0f) ==
        ResultCode::accepted);
    rig.run(30);
    CHECK(rig.status().at(Domain::trigger).execute);
    // 主 yaw 编码器仍有效, 只有惯性反馈失效 → 按 G07 降级保持, 不是设备故障.
    for (int i = 0; i < 5; ++i) {
        rig.inputs.faults.main_yaw_inertial_only = true;
        rig.app.update(rig.now, rig.inputs);
        rig.exec.step(rig.now, dt_s, rig.app);
        ++rig.now.ms;
    }
    CHECK(rig.app.main_yaw_degraded());
    CHECK(rig.app.snapshot().executions[domain_index(Domain::main_yaw)].reason == Reason::degraded);
    // 降级不是设备故障: 拨弹仍然执行, 命令仍然发出.
    CHECK(rig.app.snapshot().executions[domain_index(Domain::trigger)].execute);
    rig.inertial_only = false;
    rig.run(5);
    CHECK(!rig.app.main_yaw_degraded());
    CHECK(rig.status().at(Domain::trigger).execute);
}

// ---------------------------------------------------------------------------
// 场景 9: 人工反转不联动摩擦轮
// ---------------------------------------------------------------------------
void test_manual_reverse_does_not_move_friction() {
    Rig rig;
    ready(rig);
    CHECK(rig.target_one(Domain::trigger, control::ControlMode::velocity, -3.0f) ==
        ResultCode::accepted); // 人工反转
    rig.run(50);
    const auto trigger = rig.command_raw(Domain::trigger);
    CHECK(trigger.has_value());
    if (trigger) { CHECK(*trigger > 0); } // 方向 -1: 负目标 → 正原始量
    // 摩擦轮没有目标: 命令保持 0, 不因为拨弹反转而联动.
    const auto friction = rig.command_raw(Domain::friction_left);
    CHECK(friction.has_value());
    if (friction) { CHECK(*friction == 0); }
    CHECK(rig.axis_velocity(Domain::friction_left) == 0.0f);
}

// ---------------------------------------------------------------------------
// 场景 10: 各维度独立过期 (目标/反馈/权限)
// ---------------------------------------------------------------------------
void test_independent_expiry() {
    // 目标过期: 上位机停止刷新, 目标按 TTL 失效并撤输出.
    {
        Rig rig;
        ready(rig);
        CHECK(rig.target_one(Domain::chassis_lf, control::ControlMode::velocity, 1.0f) ==
            ResultCode::accepted);
        rig.run(5);
        CHECK(rig.status().at(Domain::chassis_lf).execute);
        rig.stop_targets();
        rig.run(60);
        const auto expired = rig.status();
        CHECK(!expired.at(Domain::chassis_lf).execute);
        CHECK(expired.at(Domain::chassis_lf).reason == Reason::stale_target);
        const auto raw = rig.command_raw(Domain::chassis_lf);
        CHECK(raw.has_value());
        if (raw) { CHECK(*raw == 0); }
    }
    // 反馈过期: 设备掉线 → 反馈失效 + 明确故障状态, 而不是"保持上一个值继续跑".
    {
        Rig rig;
        ready(rig);
        CHECK(rig.target_one(Domain::chassis_rf, control::ControlMode::velocity, 1.0f) ==
            ResultCode::accepted);
        rig.run(20);
        rig.plant.dji[1].emitting = false;
        rig.run(60);
        const auto lost = rig.status();
        CHECK(!lost.at(Domain::chassis_rf).execute);
        CHECK(lost.at(Domain::chassis_rf).validity == DomainValidity::stale);
        CHECK(rig.app.snapshot().domains[domain_index(Domain::chassis_rf)].fault ==
            FaultState::feedback_lost);
    }
    // 权限过期: 撤销会话 → 授权消失, 目标不复活.
    {
        Rig rig;
        ready(rig);
        CHECK(rig.target_one(Domain::chassis_lb, control::ControlMode::velocity, 1.0f) ==
            ResultCode::accepted);
        rig.run(20);
        CHECK(rig.status().at(Domain::chassis_lb).execute);
        rig.stop_targets();
        CHECK(rig.authorize(false, Source::host) == ResultCode::accepted);
        rig.run(20);
        const auto revoked = rig.status();
        CHECK(!revoked.authority);
        CHECK(!revoked.at(Domain::chassis_lb).execute);
        CHECK(revoked.at(Domain::chassis_lb).reason == Reason::no_authority);
    }
}

// ---------------------------------------------------------------------------
// 场景 11: 授权来源失效与无关来源掉线
// ---------------------------------------------------------------------------
void test_source_loss_isolation() {
    Rig rig;
    ready(rig);
    CHECK(rig.target_one(Domain::chassis_lf, control::ControlMode::velocity, 1.0f) ==
        ResultCode::accepted);
    rig.run(20);
    CHECK(rig.status().authority);
    // 无关来源掉线 (DR16 失联) 不影响上位机授权, 也不撤输出.
    rig.inputs.link.remote_online = false;
    rig.run(5);
    CHECK(rig.status().authority);
    CHECK(rig.status().at(Domain::chassis_lf).execute);
    CHECK(rig.app.stats().link_losses >= 1);
    // 授权来源自己失效: 会话撤销 → 立刻撤输出.
    rig.stop_targets();
    CHECK(rig.authorize(false, Source::host) == ResultCode::accepted);
    rig.run(5);
    CHECK(!rig.status().authority);
    CHECK(!rig.status().at(Domain::chassis_lf).execute);
}

// ---------------------------------------------------------------------------
// 场景 12: CAN 背压/拒绝不产生"已到达电调"的假象
// ---------------------------------------------------------------------------
void test_can_backpressure() {
    Rig rig;
    ready(rig);
    CHECK(rig.target_one(Domain::chassis_lf, control::ControlMode::velocity, 1.0f) ==
        ResultCode::accepted);
    rig.run(20);
    const auto sent_before = rig.exec.stats().group_sent;
    rig.bus1.refuse = true;
    rig.bus2.refuse = true;
    rig.run(5);
    // 发送被拒绝必须计入 tx_error, 而不是计入已发送.
    CHECK(rig.exec.stats().group_tx_error > 0);
    CHECK(rig.exec.stats().group_sent == sent_before);
    CHECK(rig.bus2.refused > 0);
    // 背压解除后恢复发送, 且不重放积存帧.
    rig.bus1.refuse = false;
    rig.bus2.refuse = false;
    rig.run(5);
    CHECK(rig.exec.stats().group_sent > sent_before);
    // 拒绝期间没有"事后补发": 计数器保留拒绝事实, 不被清零.
    CHECK(rig.exec.stats().group_tx_error > 0);
}

// ---------------------------------------------------------------------------
// 场景 13: 人工停止/恢复, 现场选源, 维护交接
// ---------------------------------------------------------------------------
void test_stop_source_switch_and_maintenance() {
    Rig rig;
    ready(rig);
    CHECK(rig.target_one(Domain::chassis_lf, control::ControlMode::velocity, 1.0f) ==
        ResultCode::accepted);
    rig.run(20);
    CHECK(rig.status().at(Domain::chassis_lf).execute);
    rig.stop_targets(); // 停止刷新, 才能观察"旧目标不复活"
    // 人工整车停止: 只由人工挡触发, 撤掉全部输出.
    rig.inputs.link.switch_right = drivers::Dr16Switch::down;
    rig.run(5);
    const auto stopped = rig.status();
    CHECK(stopped.manual_stop);
    CHECK(!stopped.at(Domain::chassis_lf).execute);
    CHECK(stopped.at(Domain::chassis_lf).reason == Reason::manual_stop);
    CHECK(rig.command_raw(Domain::chassis_lf).value_or(0) == 0);
    // 解除停止: 旧目标不复活 (必须重新下发).
    rig.inputs.link.switch_right = drivers::Dr16Switch::mid;
    rig.run(5);
    CHECK(!rig.status().manual_stop);
    CHECK(!rig.status().at(Domain::chassis_lf).execute);
    CHECK(rig.status().at(Domain::chassis_lf).reason == Reason::no_target);
    CHECK(rig.target_one(Domain::chassis_lf, control::ControlMode::velocity, 1.0f) ==
        ResultCode::accepted);
    rig.run(5);
    CHECK(rig.status().at(Domain::chassis_lf).execute);
    // 现场选源: 左拨杆切到本地 → 来源变遥控, 上位机目标立即失效.
    rig.stop_targets();
    rig.inputs.link.switch_left = drivers::Dr16Switch::up;
    rig.run(5);
    CHECK(rig.status().source == Source::remote);
    CHECK(!rig.status().at(Domain::chassis_lf).execute);
    // 切回上位机来源: 仍然需要新的目标.
    rig.inputs.link.switch_left = drivers::Dr16Switch::down;
    rig.run(5);
    CHECK(rig.status().source == Source::host);
    CHECK(!rig.status().at(Domain::chassis_lf).execute);
    // 维护交接: 左拨杆待机 + 维护会话 → 维护来源取得授权, 可执行维护目标.
    // 维护会话必须经业务协议显式申请: 协议只接受确实持有该来源资格的会话写目标.
    rig.inputs.link.switch_left = drivers::Dr16Switch::mid;
    rig.run(3);
    CHECK(rig.app.snapshot().permission.source == Source::none);
    CHECK(!rig.app.snapshot().permission.authority);
    CHECK(rig.authorize(true, Source::maintenance) == ResultCode::accepted);
    rig.run(5);
    CHECK(rig.app.snapshot().permission.source == Source::maintenance);
    CHECK(rig.app.snapshot().permission.authority);
    CHECK(rig.target_one(Domain::chassis_lf, control::ControlMode::velocity, 1.0f) ==
        ResultCode::accepted);
    rig.run(10);
    CHECK(rig.status().at(Domain::chassis_lf).execute);
    // 维护会话退出: 授权撤销, 目标失效.
    rig.stop_targets();
    CHECK(rig.authorize(false, Source::maintenance) == ResultCode::accepted);
    rig.run(5);
    CHECK(!rig.status().authority);
    CHECK(!rig.status().at(Domain::chassis_lf).execute);
}

// ---------------------------------------------------------------------------
// 场景 14: 参考代次改变, 执行器重启与 MCU 独立重启
// ---------------------------------------------------------------------------
void test_epoch_change_and_restart_models() {
    Rig rig;
    ready(rig);
    CHECK(rig.align(Domain::main_yaw, 0.0f));
    CHECK(rig.target_one(Domain::main_yaw, control::ControlMode::position, 0.2f) ==
        ResultCode::accepted);
    rig.run(30);
    CHECK(rig.status().at(Domain::main_yaw).execute);
    rig.stop_targets();
    // 参考类修改要求受影响机构已退出执行: 先让旧目标按 TTL 过期, 而不是靠改参考抢权.
    rig.now.ms += rig.config.timing.target_ttl.ms + 5;
    rig.run(3);
    CHECK(!rig.status().at(Domain::main_yaw).execute);
    // 矛盾值不能重命名唯一机械参考. 重新确认当前参考仍推进代次并清旧目标;
    // 真正的坐标重定义走零偏参数事务, 不再用任意 align 数值替代它.
    const auto invalidations_before = rig.app.target_invalidations();
    CHECK(!rig.align(Domain::main_yaw, 0.1f));
    CHECK(rig.app.target_invalidations() == invalidations_before);
    const auto actual = rig.exec.joint_feedback(Domain::main_yaw, rig.now).position_rad;
    CHECK(actual.validity == Validity::valid);
    CHECK(rig.align(Domain::main_yaw, actual.value));
    rig.run(5);
    CHECK(!rig.status().at(Domain::main_yaw).execute);
    CHECK(rig.app.target(Domain::main_yaw) == std::nullopt);
    CHECK(rig.app.target_invalidations() > invalidations_before);
    // 旧代次整包拒绝 (显式用错代次下发, 不经过目标流).
    std::array<DomainSetpoint, domain_count> setpoints{};
    setpoints[domain_index(Domain::main_yaw)] =
        DomainSetpoint{control::ControlMode::position, 0.2f};
    CHECK(rig.targets(setpoints, std::nullopt, false) == ResultCode::epoch_mismatch);
    CHECK(rig.targets(setpoints, std::nullopt) == ResultCode::accepted);
    rig.run(5);
    CHECK(rig.status().at(Domain::main_yaw).execute);
    // 执行器重新上电模型: 反馈中断后重新上线, 历史目标不得复活.
    rig.plant.dji[4].emitting = false;
    rig.run(40);
    CHECK(!rig.status().at(Domain::main_yaw).execute);
    rig.plant.dji[4].emitting = true;
    rig.plant.dji[4].position_rad = 0.0f;
    rig.plant.dji[4].velocity_rad_s = 0.0f;
    rig.stop_targets();
    rig.run(40);
    CHECK(!rig.app.snapshot().executions[domain_index(Domain::main_yaw)].execute);
    CHECK(rig.app.target(Domain::main_yaw) == std::nullopt);
    // 重新上线后重新下发即可恢复执行 (参考按单圈反馈重新建立, 不猜圈数).
    CHECK(rig.target_one(Domain::main_yaw, control::ControlMode::position, 0.1f) ==
        ResultCode::accepted);
    rig.run(20);
    CHECK(rig.status().at(Domain::main_yaw).execute);
    // MCU 独立重启模型: 新应用对象不继承旧目标, 必须重新握手与授权.
    Rig restarted;
    restarted.hello();
    restarted.run(5);
    CHECK(!restarted.status().authority);
    CHECK(!restarted.status().at(Domain::chassis_lf).execute);
    CHECK(restarted.app.accepted_packets() == 0);
}

// ---------------------------------------------------------------------------
// 场景 15: 协议错误路径在闭环中不改变状态
// ---------------------------------------------------------------------------
void test_protocol_error_paths_leave_state_intact() {
    Rig rig;
    ready(rig);
    CHECK(rig.target_one(Domain::chassis_lf, control::ControlMode::velocity, 1.0f) ==
        ResultCode::accepted);
    rig.stop_targets(); // 后续用手工序号, 不经过目标流
    rig.run(20);
    CHECK(rig.status().at(Domain::chassis_lf).execute);
    const auto accepted_before = rig.app.accepted_packets();
    // 截断载荷: 必须报 bad_length 且不改变任何目标.
    {
        PayloadWriter w{rig.request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::targets));
        w.u32(rig.session_id);
        w.u32(++rig.sequence);
        const auto length = rig.session.handle({rig.request.data(), w.size()}, rig.now, rig.response);
        CHECK(length != 0);
        PayloadReader r{{rig.response.data(), length}};
        static_cast<void>(r.u16());
        static_cast<void>(r.u32());
        static_cast<void>(r.u32());
        CHECK(static_cast<ResultCode>(r.u8()) == ResultCode::bad_length);
        CHECK(rig.app.accepted_packets() == accepted_before);
    }
    // 重复: 同一序号且内容相同回放原完整结果, 不重复执行也不谎报成功; 换内容按重放拒绝.
    {
        std::array<DomainSetpoint, domain_count> setpoints{};
        setpoints[domain_index(Domain::chassis_lf)] =
            DomainSetpoint{control::ControlMode::velocity, 2.0f};
        const auto sequence = ++rig.sequence;
        CHECK(rig.targets_at(sequence, setpoints, std::nullopt) == ResultCode::accepted);
        const auto after_accept = rig.app.accepted_packets();
        const auto duplicates_before = rig.session.duplicate_requests();
        // 同序号换内容: 不得报成功, 也不改变已采用目标.
        std::array<DomainSetpoint, domain_count> changed{};
        changed[domain_index(Domain::chassis_lf)] =
            DomainSetpoint{control::ControlMode::velocity, 3.0f};
        CHECK(rig.targets_at(sequence, changed, std::nullopt) == ResultCode::replay);
        CHECK(rig.session.duplicate_requests() == duplicates_before);
        CHECK(rig.app.accepted_packets() == after_accept);
        // 同序号同内容重传: 回放上次的完整结果, 不重复执行.
        CHECK(rig.targets_at(sequence, setpoints, std::nullopt) == ResultCode::accepted);
        CHECK(rig.session.duplicate_requests() == duplicates_before + 1);
        CHECK(rig.app.accepted_packets() == after_accept);
        const auto target = rig.app.target(Domain::chassis_lf);
        CHECK(target.has_value());
        if (target) { CHECK(near(target->value, 2.0f, 1.0e-6f)); }
        // 被拒请求的同内容重传回放原结果码; 换内容仍按重放拒绝.
        const auto rejected_seq = ++rig.sequence;
        std::array<DomainSetpoint, domain_count> illegal{};
        illegal[domain_index(Domain::chassis_lf)] =
            DomainSetpoint{control::ControlMode::velocity, 1.0e9f};
        const auto first_illegal = rig.targets_at(rejected_seq, illegal, std::nullopt);
        CHECK(first_illegal != ResultCode::accepted);
        CHECK(rig.targets_at(rejected_seq, illegal, std::nullopt) == first_illegal);
        CHECK(rig.targets_at(rejected_seq, setpoints, std::nullopt) == ResultCode::replay);
        // 真重放: 更小的序号按 replay 拒绝, 且不改变已采用目标.
        const auto replays_before = rig.session.replay_rejects();
        CHECK(rig.targets_at(sequence, setpoints, std::nullopt) == ResultCode::replay);
        CHECK(rig.session.replay_rejects() == replays_before + 1);
    }
    // 未知消息类型: 明确拒绝, 不当作有效请求.
    {
        rig.request[0] = protocol_version;
        rig.request[1] = 0x7f;
        const auto unknown = rig.session.handle({rig.request.data(), 6}, rig.now, rig.response);
        CHECK(unknown != 0);
    }
    // 未授权来源: 协议只接受 host/maintenance, 不接受遥控来源.
    CHECK(rig.authorize(true, Source::remote) == ResultCode::invalid_value);
    // 闭环仍在运行: 状态可读且链路没有被错误请求打乱.
    const auto view = rig.status();
    CHECK(view.authority);
    CHECK(view.at(Domain::chassis_lf).execute);
}

// ---------------------------------------------------------------------------
// 场景 16: 请求洪泛下的工作量有界 (会话级)
// ---------------------------------------------------------------------------
void test_request_flood_is_bounded() {
    Rig rig;
    ready(rig);
    std::size_t responses = 0;
    for (int i = 0; i < 200; ++i) {
        PayloadWriter w{rig.request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::status));
        w.u32(rig.session_id);
        w.u32(++rig.sequence);
        const auto length = rig.session.handle({rig.request.data(), w.size()}, rig.now, rig.response);
        if (length != 0) { ++responses; }
        // 控制周期照常推进: 观测请求不挤占控制路径, 也不产生执行动作.
        if (i % 4 == 0) { rig.cycle(); }
    }
    CHECK(responses == 200);
    CHECK(rig.status().authority);
    // 应用统计只统计应用自己的事件, 不把观测请求算成控制动作.
    CHECK(rig.app.accepted_packets() == 0);
    CHECK(rig.app.stats().cycles > 40);
}

// 惯性降级保持 (APP09): 降级不创造新的执行资格, 保持角固定, 期限沿用原意图.
void test_degraded_hold_contracts() {
    Rig rig;
    ready(rig);
    CHECK(rig.align(Domain::main_yaw, 0.0f));
    // 1) 从未给过 main yaw 目标就发生惯性失效: 降级不产生任何输出.
    rig.inertial_only = true;
    rig.run(5);
    CHECK(rig.app.main_yaw_degraded());
    CHECK(!rig.status().at(Domain::main_yaw).execute);
    rig.inertial_only = false;
    rig.run(2);
    CHECK(!rig.app.main_yaw_degraded());

    // 2) 有有效目标时进入降级: 保持进入瞬间捕获的机械角, 外力位移不跟着移动目标.
    CHECK(rig.target_one(Domain::main_yaw, control::ControlMode::position, 0.2f) ==
        ResultCode::accepted);
    rig.run(30);
    CHECK(rig.status().at(Domain::main_yaw).execute);
    // 旧 v2 请求没有参考系标签; 这里从类型化入口提交惯性意图, 不把机械模式当稳向.
    // 帧标签的协议往返由下一阶段的版本化接口覆盖.
    rig.stop_targets();
    TargetSnapshot inertial_target;
    inertial_target.domains[domain_index(Domain::main_yaw)] =
        DomainSetpoint{control::ControlMode::position, 0.2f, 0, YawFrame::local_inertial};
    CHECK(rig.app.accept_targets(inertial_target, rig.now, Source::host) == AcceptResult::accepted);
    rig.inertial_only = true;
    rig.run(2);
    CHECK(rig.app.main_yaw_degraded());
    const auto held = rig.app.snapshot().executions[domain_index(Domain::main_yaw)].adopted_target;
    CHECK(held.has_value());
    // 外力把机构推走: 测量改变, 但保持目标不变 (不是每周期重新捕获).
    rig.plant.external_accel[4] = 400.0f;
    // 保持沿用原意图期限 (target_ttl), 因此扰动窗口必须短于该期限才能观察到保持本身.
    rig.run(10);
    const auto still_held =
        rig.app.snapshot().executions[domain_index(Domain::main_yaw)].adopted_target;
    CHECK(still_held.has_value() && held.has_value());
    if (held && still_held) { CHECK(std::fabs(*still_held - *held) < 1.0e-6f); }
    CHECK(std::fabs(rig.axis_position(Domain::main_yaw) - *held) > 0.02f);
    // 3) 原意图到期: 降级不续期, 输出被撤.
    rig.stop_targets();
    rig.now.ms += rig.config.timing.target_ttl.ms + 5;
    rig.run(3);
    CHECK(!rig.status().at(Domain::main_yaw).execute);
    // 4) 惯性恢复: 退出降级, 旧惯性目标不复活.
    rig.plant.external_accel[4] = 0.0f;
    rig.inertial_only = false;
    rig.run(5);
    CHECK(!rig.app.main_yaw_degraded());
    CHECK(!rig.status().at(Domain::main_yaw).execute);
    CHECK(!rig.app.target(Domain::main_yaw).has_value());
}

// 未测机构仍声明已装: 不提供其换算/控制/几何和反馈, 必须明确零输出而不能冒充 nullptr.
void test_independent_mechanisms() {
    using host::Mechanism;
    for (const auto part : {Mechanism::chassis, Mechanism::gimbal, Mechanism::shooter}) {
        auto config = host::partial_fixture_config(part);
        Rig rig{config, part};
        CHECK(rig.exec.assembled());
        for (const auto& slot : config.dji) { CHECK(slot.installed); }
        CHECK(config.dm[0].installed);
        rig.run(3); rig.hello(); CHECK(rig.authorize(true, Source::host) == ResultCode::accepted); rig.run(2);
        if (part == Mechanism::chassis) { rig.stream.twist = motion::PlanarTwist{0.05f, 0, 0}; }
        else {
            for (std::size_t i = 0; i < domain_count; ++i) {
                const auto domain = static_cast<Domain>(i);
                if (!host::selected(part, domain)) { continue; }
                rig.stream.setpoints[i] = {is_joint(domain) ? control::ControlMode::position : control::ControlMode::velocity,
                    is_joint(domain) ? 0.12f : 2.0f, 0};
            }
        }
        rig.stream.active = true;
        std::array<bool, domain_count> nonzero{};
        for (int step = 0; step < 1500; ++step) {
            rig.cycle();
            CHECK(rig.last_target_result == ResultCode::accepted);
            for (const auto& slot : config.dji) {
                const auto protocol = *drivers::dji_protocol(slot.mode);
                const auto tx_id = slot.device_id <= 4 ? protocol.first_tx : protocol.second_tx;
                const auto offset = 2u*((slot.device_id-1u)%4u);
                const auto& bus = slot.bus == 1 ? rig.bus1 : rig.bus2;
                for (const auto& frame : bus.frames) {
                    if (frame.id != tx_id) { continue; }
                    const bool on = frame.data[offset] != 0 || frame.data[offset+1] != 0;
                    nonzero[domain_index(slot.domain)] |= on;
                    if (!host::selected(part, slot.domain)) { CHECK(!on); }
                }
            }
            if (part != Mechanism::gimbal) {
                CHECK(rig.exec.stats().dm_enable_sent == 0 && rig.exec.stats().dm_command_sent == 0);
            }
        }
        for (std::size_t i = 0; i < domain_count; ++i) {
            const auto domain = static_cast<Domain>(i);
            if (host::selected(part, domain)) {
                CHECK(rig.app.snapshot().executions[i].execute);
                const auto feedback = rig.exec.joint_feedback(domain, rig.now);
                const auto measured = is_joint(domain) ? feedback.position_rad : feedback.velocity_rad_s;
                CHECK(measured.validity == Validity::valid && std::fabs(measured.value) > 0.02f);
                if (domain != Domain::pitch) { CHECK(nonzero[i]); }
                else { CHECK(rig.exec.stats().dm_command_sent > 0); }
            } else {
                CHECK(!rig.app.snapshot().executions[i].execute);
                CHECK(rig.app.snapshot().domains[i].validity != DomainValidity::valid);
            }
        }
    }
    // 同组保留真实 trigger device 但不配置控制环: 两摩擦轮仍可运行, trigger 槽显式为零.
    auto config = synthetic_config();
    config.dji[8].velocity_pid.reset();
    config.joints[2] = {}; // 原生 pitch 数据不能在未知零偏时伪造关节角.
    Rig rig{config};
    CHECK(rig.exec.assembled());
    CHECK(rig.exec.dji_device(8) != nullptr && rig.exec.axis(Domain::trigger) == nullptr);
    rig.run(3); rig.hello(); CHECK(rig.authorize(true, Source::host) == ResultCode::accepted); rig.run(2);
    rig.stream.setpoints[domain_index(Domain::friction_left)] = {control::ControlMode::velocity, 2.0f, 0};
    rig.stream.setpoints[domain_index(Domain::friction_right)] = {control::ControlMode::velocity, 2.0f, 0};
    rig.stream.active = true;
    rig.run(1200);
    CHECK(rig.app.snapshot().executions[domain_index(Domain::friction_left)].execute);
    CHECK(rig.exec.stats().group_refused == 0);
    CHECK(rig.exec.dji_device(8)->state(rig.now).velocity_rad_s.validity == Validity::valid);
    CHECK(rig.exec.joint_feedback(Domain::pitch, rig.now).position_rad.validity == Validity::unknown);
    CHECK(rig.app.snapshot().domains[domain_index(Domain::trigger)].validity == DomainValidity::incomplete);
    // 最小设备参数缺失只封闭其命令组; 地址未知/冲突仍不能建立拥有者.
    config.dji[8].rotor_per_axis.reset();
    Rig stopped{config};
    CHECK(stopped.exec.assembled());
    stopped.run(4);
    CHECK(stopped.exec.health(stopped.now).execution_rejected[domain_index(Domain::friction_left)]);
    config.dji[8].device_id = 0;
    Rig unknown{config};
    CHECK(!unknown.exec.assembled());
    unknown.run(3);
    CHECK(unknown.bus1.sent == 0 && unknown.bus2.sent == 0);
}

int main() {
    test_independent_mechanisms();
    test_full_closed_loop();
    test_partial_targets_revoke_others();
    test_device_missing_dependencies();
    test_zero_velocity_correction();
    test_angle_hold_under_disturbance();
    test_twist_scaling();
    test_friction_off_trigger_allowed();
    test_degraded_main_yaw_still_triggers();
    test_degraded_hold_contracts();
    test_manual_reverse_does_not_move_friction();
    test_independent_expiry();
    test_source_loss_isolation();
    test_can_backpressure();
    test_stop_source_switch_and_maintenance();
    test_epoch_change_and_restart_models();
    test_protocol_error_paths_leave_state_intact();
    test_request_flood_is_bounded();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("sentinel APP system contracts passed\n");
    return EXIT_SUCCESS;
}
