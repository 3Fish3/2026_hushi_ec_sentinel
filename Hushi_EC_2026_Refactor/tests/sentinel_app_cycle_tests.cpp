// APP06 契约: 板级与主机共用的薄周期编排.
// 覆盖审查要求的三件事:
//  - 原始 18 B DT7 帧经正式编排 (Dr16Stream → Dr16Input → RemoteMapper → 应用 → 执行层)
//    到达真实 CAN 槽位;
//  - IMU 样本经同一入口进入感知重构并产生惯性状态;
//  - 许可有/无/过期只改变正向拨弹资格, 未配置状态只能观察.
#include "sentinel_cycle.hpp"
#include "sentinel_param_apply.hpp"
#include "sentinel_params.hpp"
#include "sim_plant.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
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
using ec::drivers::Dr16Sample;
using ec::drivers::Dr16Stream;
using ec::drivers::Dr16Switch;
using ec::sentinel::app::ConfigAudience;
using ec::sentinel::app::CycleInputs;
using ec::sentinel::app::Domain;
using ec::sentinel::app::SentinelApp;
using ec::sentinel::app::SentinelCycle;
using ec::sentinel::app::SentinelExecutor;
using ec::sentinel::app::SentinelPerception;
using ec::sentinel::app::SourceIntent;
using ec::sentinel::host::SimPlant;
using ec::sentinel::host::SoftwareBus;

constexpr std::uint32_t index(Domain domain) { return ec::sentinel::app::domain_index(domain); }

struct FrameSpec {
    std::array<int, 4> ch{{0, 0, 0, 0}};
    int ch4{};
    Dr16Switch switch_left{Dr16Switch::mid};
    Dr16Switch switch_right{Dr16Switch::mid};
};

// 与 C 板示例一致: ch0–ch3 为 11 bit 位域减 1024, byte5 高位是拨杆, byte16..17 是第 5 通道.
std::array<std::uint8_t, 18> encode_frame(const FrameSpec& spec) {
    std::array<std::uint8_t, 18> buf{};
    std::uint64_t bits = 0;
    for (int i = 0; i < 4; ++i) {
        const auto raw = static_cast<std::uint64_t>(spec.ch[static_cast<std::size_t>(i)] + 1024) &
            0x7ffu;
        bits |= raw << (11 * i);
    }
    for (int i = 0; i < 6; ++i) {
        buf[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>((bits >> (8 * i)) & 0xffu);
    }
    buf[5] = static_cast<std::uint8_t>((buf[5] & 0x0fu) |
        ((static_cast<unsigned>(spec.switch_left) & 0x03u) << 6) |
        ((static_cast<unsigned>(spec.switch_right) & 0x03u) << 4));
    const auto raw4 = static_cast<std::uint16_t>(spec.ch4 + 1024);
    buf[16] = static_cast<std::uint8_t>(raw4 & 0xffu);
    buf[17] = static_cast<std::uint8_t>(raw4 >> 8);
    return buf;
}

int stick(float normalized) { return static_cast<int>(normalized * 660.0f); }

ec::sentinel::app::RemoteMappingConfig mapping() {
    ec::sentinel::app::RemoteMappingConfig config{};
    config.stick_deadzone = 0.05f;
    config.chassis_vx_max_mps = 1.0f;
    config.chassis_vy_max_mps = 1.0f;
    config.chassis_wz_max_rad_s = 2.0f;
    config.yaw_rate_max_rad_s = 1.0f;
    config.pitch_rate_max_rad_s = 1.0f;
    config.friction_rad_s = 25.0f;
    config.trigger_rad_s = 10.0f;
    config.ch4 = ec::sentinel::app::Ch4LayerConfig{0, 300, -300, 50, true};
    return config;
}

// 编排测试台: 与板级相同顺序调用 control/commit, 只有"总线"是软件替身.
struct Rig {
    ec::sentinel::app::SentinelConfig config{ec::sentinel::app::synthetic_config()};
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    SentinelApp app{config, ConfigAudience::offline_test};
    SentinelExecutor executor{config, ConfigAudience::offline_test, bus1, bus2};
    SentinelPerception perception{config, ConfigAudience::offline_test};
    ec::sentinel::app::ParamStore params{config};
    SentinelCycle cycle{app, executor, perception, mapping(), config.timing.remote_ttl};
    SimPlant plant{};
    Dr16Stream stream{};
    Tick now{1000};
    float dt_s{0.001f};
    // 上电后首个 IDLE 只建立帧边界, 不交付样本 (与板级 USART3 一致).
    explicit Rig(ec::sentinel::app::SentinelConfig value = ec::sentinel::app::synthetic_config())
        : config(std::move(value)), app(config, ConfigAudience::offline_test),
          executor(config, ConfigAudience::offline_test, bus1, bus2),
          perception(config, ConfigAudience::offline_test),
          cycle(app, executor, perception, mapping(), config.timing.remote_ttl) {
        const std::uint8_t priming = 0;
        stream.push({&priming, 1});
        static_cast<void>(stream.on_idle());
    }
    bool locally_ready{true};
    std::optional<std::array<float, 3>> gyro{};
    std::optional<std::array<float, 3>> accel{};
    Tick imu_sampled_at{};
    bool fixed_level_imu{}; // 静止参考/映射用例; 动态闭环由正向运动夹具另行验证.
    bool advance_plant{true};

    void feed(const FrameSpec& spec) {
        const auto frame = encode_frame(spec);
        stream.push({frame.data(), frame.size()});
        const auto sample = stream.on_idle();
        if (sample) { pending = *sample; }
    }
    void cycle_once() {
        CycleInputs in{};
        in.now = now;
        in.dt_s = dt_s;
        if (pending) {
            in.remote_sample = *pending;
            in.remote_received_at = now;
            pending.reset();
        }
        in.imu_gyro_rad_s = gyro;
        in.imu_accel_mps2 = accel;
        in.imu_sampled_at = imu_sampled_at;
        if (fixed_level_imu) {
            in.imu_gyro_rad_s = std::array<float, 3>{0, 0, 0};
            in.imu_accel_mps2 = std::array<float, 3>{0, 0, 9.80665f};
            in.imu_sampled_at = now;
        }
        in.locally_ready = locally_ready;
        // 反馈帧先到执行层, 再跑编排 (与板级"先收帧后控制"顺序一致).
        plant.emit_feedback(config,
            [this](std::uint8_t bus, const ec::transport::CanFrame& frame, Tick at) {
                executor.route(bus, frame, at, ec::MicroTick{at.ms*1000u});
            },
            now);
        bus1.clear();
        bus2.clear();
        outcome = cycle.control(in);
        cycle.commit(in);
        if (advance_plant) { plant.integrate(config, bus1, bus2, dt_s); }
        ++now.ms;
    }
    std::optional<Dr16Sample> pending{};
    ec::sentinel::app::CycleOutcome outcome{};
};

// 最近一帧命令的槽位原始值 (大端 int16).
std::int16_t slot_raw(const ec::transport::CanFrame& frame, std::size_t slot) {
    const auto high = static_cast<std::uint16_t>(frame.data[slot * 2]);
    const auto low = static_cast<std::uint16_t>(frame.data[slot * 2 + 1]);
    return static_cast<std::int16_t>((high << 8) | low);
}

// 原始 18 B DT7 帧 → 正式编排 → 真实 CAN 槽位.
void raw_frame_to_can_contracts() {
    Rig rig;
    // 先给一次回中帧: 首次进入角度用途必须先看到回中 (C06).
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    // 左杆前推 + 本地运行档 → 底盘 twist → 四轮速度目标.
    rig.feed(FrameSpec{{0, 0, 0, stick(0.5f)}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    CHECK(rig.outcome.intent == SourceIntent::local);
    CHECK(rig.outcome.accept_attempted);
    CHECK(rig.outcome.accept == ec::sentinel::app::AcceptResult::accepted);
    // 底盘四轮在 CAN2 组 0x200 (合成夹具把 lf/rf/lb/rb 装在 id1..4).
    const auto frame = rig.bus2.last_with_id(0x200);
    CHECK(frame.has_value());
    if (frame) {
        // 轮 lf/rb 滚动方向 3π/4, rf/lb 为 π/4: vx>0 时两组符号相反 (与执行契约一致).
        CHECK(slot_raw(*frame, 0) < 0);
        CHECK(slot_raw(*frame, 1) > 0);
        CHECK(slot_raw(*frame, 2) < 0);
        CHECK(slot_raw(*frame, 3) > 0);
    }
    // 槽位重复读取不重复计数: 板级每周期都会读到同一份样本.
    const auto frames_before = rig.bus2.sent;
    rig.cycle_once();
    CHECK(rig.bus2.sent > frames_before); // 目标仍在有效期内 → 继续按周期发帧
    CHECK(rig.cycle.remote_input().accepted() == 2); // 只有两份真实样本被接受

    // 遥控失联 (样本过期) → 撤销来源与目标, 四轮显式零输出.
    rig.now.ms += 200;
    rig.cycle_once();
    CHECK(!rig.outcome.remote_online);
    CHECK(!rig.app.target(Domain::chassis_lf).has_value());
    const auto zero = rig.bus2.last_with_id(0x200);
    CHECK(zero.has_value());
    if (zero) {
        for (std::size_t slot = 0; slot < 4; ++slot) { CHECK(slot_raw(*zero, slot) == 0); }
    }
}

// 发射机构层: 摩擦轮开关与拨弹经同一编排到达 0x1ff.
void launcher_contracts() {
    Rig rig;
    rig.feed(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    // 低侧 ch4 = 发射机构层; 右 X 回中一次后正向 → 开摩擦轮.
    rig.feed(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    CHECK(rig.outcome.friction_on);
    const auto frame = rig.bus2.last_with_id(0x1ff);
    CHECK(frame.has_value());
    if (frame) {
        // 合成夹具: friction_left=id5 槽0, friction_right=id6 槽1, trigger=id7 槽2.
        // 只核对非零: 符号取决于夹具配置的 direction, 不在编排测试里重复断言.
        CHECK(slot_raw(*frame, 0) != 0);
        CHECK(slot_raw(*frame, 1) != 0);
    }
}

// 遥控样本去重使用回绕安全的时间比较: 相同时间戳是重复, 半个回绕窗口内的倒退才是回退.
void remote_dedup_wrap_contracts() {
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    auto config = ec::sentinel::app::synthetic_config();
    SentinelApp app{config, ConfigAudience::offline_test};
    SentinelExecutor executor{config, ConfigAudience::offline_test, bus1, bus2};
    SentinelPerception perception{config, ConfigAudience::offline_test};
    SentinelCycle cycle{app, executor, perception, mapping(), config.timing.remote_ttl};
    Dr16Sample sample{};
    sample.switch_left = Dr16Switch::up;
    sample.switch_right = Dr16Switch::mid;
    const auto run = [&](Tick at) {
        CycleInputs in{};
        in.now = at;
        in.dt_s = 0.001f;
        in.remote_sample = sample;
        in.remote_received_at = at;
        in.locally_ready = true;
        static_cast<void>(cycle.control(in));
        cycle.commit(in);
    };
    run(Tick{0xfffffff0u});
    CHECK(cycle.remote_input().accepted() == 1);
    // 同一时间戳的重复样本: 不重复计数.
    run(Tick{0xfffffff0u});
    CHECK(cycle.remote_input().accepted() == 1);
    // 时间戳回绕后的前进: 必须接受 (不能用普通 `<` 判成"未更新").
    run(Tick{5u});
    CHECK(cycle.remote_input().accepted() == 2);
    // 半个回绕窗口内的真倒退: 拒绝.
    run(Tick{4u});
    CHECK(cycle.remote_input().accepted() == 2);
}

// DM 反馈必须以关节视图 (减零位偏移) 进入应用, 与健康判定/mapper 同坐标.
void dm_joint_view_contracts() {
    auto config = ec::sentinel::app::synthetic_config();
    config.joints[ec::sentinel::app::joint_index(Domain::pitch)].zero_offset_rad = 0.3f;
    Rig rig{config};
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    const auto* device = rig.executor.dm_device();
    CHECK(device != nullptr);
    if (device != nullptr) {
        const auto native = device->state();
        CHECK(native.feedback.validity == ec::Validity::valid);
        CHECK(std::fabs(native.feedback.value.position_rad) < 1.0e-3f);
    }
    const auto& reported = rig.app.snapshot().domains[index(Domain::pitch)].feedback.position_rad;
    CHECK(reported.validity == ec::Validity::valid);
    CHECK(std::fabs(reported.value - (-0.3f)) < 1.0e-3f);
}

// 参考语义改变后, 该关节已保持的本地角度目标必须丢弃: 不能用旧坐标的保持角继续驱动,
// 本周期能从新参考下的实际姿态重建时才重新接管.
void held_angle_reference_change_contracts() {
    Rig rig;
    rig.fixed_level_imu = true;
    rig.advance_plant = false;
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    rig.feed(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.dt_s = 0.3f; // 只为在测试里拉开"保持角 vs 实际姿态"的差距 (超出 max_dt 不影响本用例)
    rig.cycle_once();
    CHECK(rig.app.target(Domain::main_yaw).has_value());
    const float stale_hold = rig.app.target(Domain::main_yaw)->value;
    const auto joint_before = rig.executor.joint_feedback(Domain::main_yaw, rig.now);
    CHECK(joint_before.position_rad.validity == ec::Validity::valid);
    CHECK(std::fabs(stale_hold - joint_before.position_rad.value) > 0.05f);
    // 零位/机械范围改变 + 回中: 保持角属于旧坐标, 丢弃后从新参考下的实际姿态重建.
    rig.app.invalidate_domain_reference(Domain::main_yaw);
    rig.dt_s = 0.001f;
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    const auto target = rig.app.target(Domain::main_yaw);
    const auto direction = rig.perception.local_direction_rad();
    CHECK(target.has_value() && direction.has_value());
    if (target && direction) {
        CHECK(target->frame == ec::sentinel::app::YawFrame::local_inertial);
        CHECK(std::fabs(target->value - *direction) < 0.02f);
    }
    CHECK(!target.has_value() || std::fabs(target->value - stale_hold) > 0.05f);
}

// 本地每周期全量目标不能把 G07 保持误撤销, 也不能靠不断刷新把原意图续成无限保持.
void local_degraded_lease_contracts() {
    Rig rig;
    rig.fixed_level_imu=true;
    rig.advance_plant=false;
    rig.feed(FrameSpec{{0,0,0,stick(0.3f)},0,Dr16Switch::up,Dr16Switch::mid});
    rig.cycle_once(); rig.cycle_once();
    const auto generation=rig.app.snapshot().permission.generation;
    rig.fixed_level_imu=false;
    for(unsigned i=0;i<7;++i) rig.cycle_once();
    CHECK(rig.app.main_yaw_degraded());
    const auto held=rig.app.snapshot().executions[index(Domain::main_yaw)].adopted_target;
    CHECK(held.has_value());
    for(unsigned i=0;i<5;++i) {
        rig.cycle_once();
        CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].adopted_target==held);
        CHECK(rig.app.snapshot().executions[index(Domain::chassis_lf)].execute);
    }
    for(unsigned i=0;i<20;++i) rig.cycle_once();
    CHECK(!rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
    CHECK(rig.app.snapshot().executions[index(Domain::chassis_lf)].execute);
    CHECK(rig.app.snapshot().permission.generation==generation);
    rig.fixed_level_imu=true;
    rig.feed(FrameSpec{{0,0,0,stick(0.3f)},0,Dr16Switch::up,Dr16Switch::mid});
    rig.cycle_once();
    CHECK(!rig.app.main_yaw_degraded());
    CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].frame==ec::sentinel::app::YawFrame::local_inertial);
    CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
}

// NT06: 完整设备配置 + 未配置映射 (全 0) + 有效遥控回中 + 非零轮速反馈:
// 不得生成"有效零 twist/保持目标", 也不得授予本地执行; 不能靠轮子恰好静止掩盖输出.
void unconfigured_mapping_cycle_contracts() {
    auto config = ec::sentinel::app::synthetic_config();
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    SentinelApp app{config, ConfigAudience::offline_test};
    SentinelExecutor executor{config, ConfigAudience::offline_test, bus1, bus2};
    SentinelPerception perception{config, ConfigAudience::offline_test};
    SentinelCycle cycle{app, executor, perception, ec::sentinel::app::RemoteMappingConfig{},
        config.timing.remote_ttl};
    SimPlant plant{};
    // 非零轮速反馈: 设备健康, 排除"反馈缺失"造成的假阴性.
    for (std::size_t i = 0; i < ec::sentinel::app::dji_slot_count; ++i) {
        plant.dji[i].velocity_rad_s = 5.0f;
    }
    Dr16Stream stream{};
    const std::uint8_t priming = 0;
    stream.push({&priming, 1});
    static_cast<void>(stream.on_idle());
    // 有效遥控回中 + 本地运行档.
    const auto frame = encode_frame(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    stream.push({frame.data(), frame.size()});
    const auto sample = stream.on_idle();
    CHECK(sample.has_value());
    const Tick now{1000};
    plant.emit_feedback(config,
        [&](std::uint8_t bus, const ec::transport::CanFrame& f, Tick at) {
            executor.route(bus, f, at, ec::MicroTick{at.ms*1000u});
        },
        now);
    CycleInputs in{};
    in.now = now;
    in.dt_s = 0.001f;
    in.remote_sample = *sample;
    in.remote_received_at = now;
    in.locally_ready = true;
    const auto outcome = cycle.control(in);
    bus1.clear();
    bus2.clear();
    cycle.commit(in);
    CHECK(outcome.remote_online);
    CHECK(outcome.intent == SourceIntent::local);
    CHECK(!outcome.chassis_active);
    CHECK(!outcome.friction_on);
    for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        CHECK(!app.target(domain).has_value());
        CHECK(!app.snapshot().executions[i].execute);
    }
}

// A03: 真正改零偏 (不是只上推 App 代次) 后, 参考失效期间不得把旧坐标捕获成新目标:
// 关节视图/健康判定/Mapper 必须同属新坐标.
void zero_offset_change_contracts() {
    Rig rig;
    rig.fixed_level_imu = true;
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::mid, Dr16Switch::mid});
    rig.cycle_once();
    rig.cycle_once();
    const auto slot = rig.app.dji_slot_for(Domain::main_yaw);
    CHECK(slot.has_value());
    if (!slot) { return; }
    const auto before = rig.executor.joint_feedback(Domain::main_yaw, rig.now);
    CHECK(before.position_rad.validity == ec::Validity::valid);
    const float before_rad = before.position_rad.value;
    // 参考类参数事务: 真正把零位改到 0.4 rad.
    const auto written = ec::sentinel::app::apply_param(rig.params, rig.app, &rig.executor,
        &rig.perception, ec::sentinel::app::param_id(*slot, 20), 0.4f, true);
    CHECK(written == ec::sentinel::app::ParamWriteResult::accepted);
    CHECK(rig.app.joint_geometry(Domain::main_yaw).zero_offset_rad.value_or(-1.0f) == 0.4f);
    // 切回本地并回中: 旧保持角被丢弃, 新目标只能来自新参考下的实际姿态.
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    rig.cycle_once();
    rig.cycle_once();
    const auto actual = rig.executor.joint_feedback(Domain::main_yaw, rig.now);
    CHECK(actual.position_rad.validity == ec::Validity::valid);
    CHECK(std::fabs(actual.position_rad.value - (-0.4f)) < 0.02f);
    const auto target = rig.app.target(Domain::main_yaw);
    CHECK(target.has_value());
    if (target) {
        // 改机械零位不能把局部惯性方向也平移 0.4 rad.
        CHECK(target->frame == ec::sentinel::app::YawFrame::local_inertial);
        CHECK(std::fabs(target->value - rig.perception.platform().continuous_heading) < 0.02f);
        CHECK(std::fabs(target->value - actual.position_rad.value) > 0.3f);
    }
    CHECK(std::fabs(before_rad) < 0.02f);
}

// 参考不可唯一恢复时不生成位置目标 (维持 unknown, 不用旧坐标补位).
void unresolvable_reference_no_target_contracts() {
    Rig rig;
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::mid, Dr16Switch::mid});
    rig.cycle_once();
    rig.cycle_once();
    const auto slot = rig.app.dji_slot_for(Domain::main_yaw);
    CHECK(slot.has_value());
    if (!slot) { return; }
    // 机械范围改成与当前单圈同余不相容的窄区间: 无法唯一恢复.
    CHECK(ec::sentinel::app::apply_param(rig.params, rig.app, &rig.executor, &rig.perception,
              ec::sentinel::app::param_id(*slot, 21), 2.4f, true) ==
        ec::sentinel::app::ParamWriteResult::accepted);
    CHECK(ec::sentinel::app::apply_param(rig.params, rig.app, &rig.executor, &rig.perception,
              ec::sentinel::app::param_id(*slot, 22), 2.5f, true) ==
        ec::sentinel::app::ParamWriteResult::accepted);
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    rig.cycle_once();
    const auto actual = rig.executor.joint_feedback(Domain::main_yaw, rig.now);
    CHECK(actual.position_rad.validity != ec::Validity::valid);
    CHECK(!rig.app.target(Domain::main_yaw).has_value());
}

// F10 退出标准: dt=0 (重复时刻/回退) 不只影响计数, 而是走既有 invalid_dt 拒绝路径,
// 并把旧目标一并作废, 不静默继续用标称周期驱动.
void zero_dt_rejects_execution_contracts() {
    Rig rig;
    rig.feed(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    rig.feed(FrameSpec{{0, 0, 0, stick(0.5f)}, 0, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    CHECK(rig.app.snapshot().executions[index(Domain::chassis_lf)].execute);
    // 本周期 dt=0: 轴/环按已有 invalid_dt 拒绝, 并回灌为执行拒绝.
    rig.dt_s = 0.0f;
    rig.cycle_once();
    // 拒绝在下一次健康判定里生效: 该域本周期不再执行.
    rig.dt_s = 0.001f;
    rig.cycle_once();
    const auto rejected = rig.app.snapshot().executions[index(Domain::chassis_lf)];
    CHECK(!rejected.execute);
    CHECK(rejected.reason == ec::sentinel::app::Reason::output_rejected);
}

// IMU 样本经同一入口进入感知重构.
void imu_contracts() {
    Rig rig;
    rig.gyro = std::array<float, 3>{0.0f, 0.0f, 0.0f};
    rig.accel = std::array<float, 3>{0.0f, 0.0f, 9.80665f};
    rig.imu_sampled_at = rig.now;
    rig.cycle_once();
    CHECK(rig.outcome.imu_submitted);
    CHECK(rig.perception.inertial().validity == ec::sentinel::app::DomainValidity::valid);
    CHECK(rig.perception.inertial().age_ms == 0);
    CHECK(rig.perception.platform().validity == ec::sentinel::app::DomainValidity::valid);
    const auto epoch = rig.app.reference_epoch();
    // 同一采样时刻的重复样本被拒绝, 但仍保持有效 (不把它当成新数据).
    rig.cycle_once();
    CHECK(rig.perception.stats().rejected_duplicate == 1);
    CHECK(rig.perception.inertial().validity == ec::sentinel::app::DomainValidity::valid);
    // 超时: 惯性状态降级为 stale, 不继续外推.
    rig.imu_sampled_at = rig.now;
    rig.cycle_once();
    rig.now.ms += 100;
    rig.cycle_once();
    CHECK(rig.perception.inertial().validity == ec::sentinel::app::DomainValidity::stale);
    CHECK(!rig.perception.inertial().extrapolated);
    CHECK(rig.app.reference_epoch() == epoch); // 普通质量下降不创造新坐标
    rig.imu_sampled_at = rig.now;
    rig.cycle_once();
    CHECK(rig.app.reference_epoch() == epoch+1);
    CHECK(rig.app.domain_reference_epoch(Domain::main_yaw) == 1);
    CHECK(rig.app.domain_reference_epoch(Domain::fire_yaw) == 0);
    CHECK(rig.app.domain_reference_epoch(Domain::pitch) == 0);
}

// 许可有/无/过期: 只影响正向拨弹资格, 不影响其它域, 也不影响人工反转.
void permit_contracts() {
    Rig rig;
    rig.feed(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    // 许可缺失: 正向拨弹被拒绝 (not_permitted), 摩擦轮不受影响.
    rig.feed(FrameSpec{{stick(0.5f), stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    CHECK(!rig.app.snapshot().executions[index(Domain::trigger)].execute);
    CHECK(rig.app.snapshot().executions[index(Domain::trigger)].reason ==
        ec::sentinel::app::Reason::not_permitted);
    CHECK(rig.outcome.friction_on);
    // 许可到位: 同一份意图在下一个周期取得执行资格.
    rig.app.set_launch_permit(true, rig.now, ec::Lifetime{50});
    rig.cycle_once();
    CHECK(rig.app.snapshot().executions[index(Domain::trigger)].execute);
    // 许可过期: 正向拨弹再次撤输出.
    rig.now.ms += 60;
    rig.cycle_once();
    CHECK(!rig.app.snapshot().executions[index(Domain::trigger)].execute);
    CHECK(rig.app.snapshot().executions[index(Domain::trigger)].reason ==
        ec::sentinel::app::Reason::not_permitted);
    // 负向人工反转不需要发射许可 (独立控制).
    rig.feed(FrameSpec{{0, stick(-0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    rig.cycle_once();
    CHECK(rig.app.target(Domain::trigger).has_value());
    if (const auto target = rig.app.target(Domain::trigger)) { CHECK(target->value < 0.0f); }
}

// 未配置的部署配置: 只能观察, 不产生任何 CAN 输出, 也不伪造健康状态.
void unconfigured_contracts() {
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    const auto config = ec::sentinel::app::real_config_placeholder();
    SentinelApp app{config, ConfigAudience::offline_test};
    SentinelExecutor executor{config, ConfigAudience::offline_test, bus1, bus2};
    SentinelPerception perception{config, ConfigAudience::offline_test};
    SentinelCycle cycle{app, executor, perception, mapping(), config.timing.remote_ttl};
    Dr16Stream stream{};
    const std::uint8_t priming = 0;
    stream.push({&priming, 1});
    static_cast<void>(stream.on_idle());
    const auto frame = encode_frame(FrameSpec{{0, 0, 0, stick(0.5f)}, 0, Dr16Switch::up,
        Dr16Switch::mid});
    stream.push({frame.data(), frame.size()});
    const auto sample = stream.on_idle();
    CHECK(sample.has_value());
    CycleInputs in{};
    in.now = Tick{1000};
    in.dt_s = 0.001f;
    in.remote_sample = *sample;
    in.remote_received_at = Tick{1000};
    in.locally_ready = true;
    static_cast<void>(cycle.control(in));
    cycle.commit(in);
    CHECK(bus1.sent == 0);
    CHECK(bus2.sent == 0);
    // 未装配的域不报健康: 有效性保持 unknown/unavailable, 且不具备执行资格.
    const auto& entry = app.snapshot().executions[index(Domain::chassis_lf)];
    CHECK(!entry.execute);
    const auto validity = app.snapshot().domains[index(Domain::chassis_lf)].validity;
    CHECK(validity == ec::sentinel::app::DomainValidity::unknown ||
        validity == ec::sentinel::app::DomainValidity::unavailable);
}
} // 命名空间

int main() {
    raw_frame_to_can_contracts();
    remote_dedup_wrap_contracts();
    dm_joint_view_contracts();
    held_angle_reference_change_contracts();
    local_degraded_lease_contracts();
    zero_offset_change_contracts();
    unresolvable_reference_no_target_contracts();
    zero_dt_rejects_execution_contracts();
    unconfigured_mapping_cycle_contracts();
    launcher_contracts();
    imu_contracts();
    permit_contracts();
    unconfigured_contracts();
    std::puts("sentinel app cycle contracts passed");
    return 0;
}
