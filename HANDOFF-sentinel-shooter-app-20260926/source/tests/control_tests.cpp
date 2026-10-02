// M1 控制基础契约: 角度误差、电流换算、速度反馈滤波、位置→速度串级与离线对照.
// 这些检查不连接硬件, 也不证明任何实车参数在真实负载下稳定.
#include "ec/control/angle.hpp"
#include "ec/control/current_scale.hpp"
#include "ec/control/pid.hpp"
#include "ec/control/position_loop.hpp"
#include "ec/control/thermal.hpp"
#include "ec/control/velocity_loop.hpp"
#include <algorithm>
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

constexpr float pi = std::numbers::pi_v<float>;

bool near(float a, float b, float tolerance = 0.0001f) {
    return std::fabs(a - b) <= tolerance;
}

void angle_contracts() {
    using ec::control::ArcMode;
    using ec::control::position_error;
    using ec::control::shortest_arc_error;
    using ec::control::wrap_to_pi;
    CHECK(near(wrap_to_pi(0.0f), 0.0f));
    CHECK(near(wrap_to_pi(1.0f), 1.0f));
    CHECK(near(wrap_to_pi(pi), pi));                 // 上边界保留 +pi
    CHECK(near(wrap_to_pi(-pi), pi));                // 下边界映射到 +pi, 区间为 (-pi, pi]
    // 3*pi 在 float 下并不是精确的 3pi, 端点允许 1 ulp 差异, 两种符号都不影响控制语义.
    const auto at_pi_boundary = [](float value) { return std::fabs(std::fabs(value) - pi) < 1e-5f; };
    CHECK(at_pi_boundary(wrap_to_pi(3.0f * pi)));
    CHECK(at_pi_boundary(wrap_to_pi(-3.0f * pi)));
    CHECK(near(wrap_to_pi(2.0f * pi), 0.0f));
    CHECK(near(wrap_to_pi(pi + 0.25f), -pi + 0.25f));
    CHECK(near(wrap_to_pi(-pi - 0.25f), pi - 0.25f));
    // 多圈累计角仍能归一: 1000 rad 在 float 精度下只要求落在区间内.
    for (float turns : {1000.0f, -1000.0f, 12345.678f}) {
        const auto wrapped = wrap_to_pi(turns);
        CHECK(wrapped > -pi - 0.001f && wrapped <= pi + 0.001f);
    }
    CHECK(!std::isfinite(wrap_to_pi(std::numeric_limits<float>::quiet_NaN())));
    CHECK(!std::isfinite(wrap_to_pi(std::numeric_limits<float>::infinity())));
    CHECK(!std::isfinite(wrap_to_pi(-std::numeric_limits<float>::infinity())));

    CHECK(near(shortest_arc_error(3.0f, -3.0f), 6.0f - 2.0f * pi));
    CHECK(near(shortest_arc_error(-3.0f, 3.0f), 2.0f * pi - 6.0f));
    CHECK(near(shortest_arc_error(0.1f, 0.0f), 0.1f));
    CHECK(near(shortest_arc_error(0.1f, 2.0f * pi), 0.1f));
    // continuous 保留多圈差, shortest 只取单圈最短弧; 二者是显式选择而非默认策略.
    CHECK(near(position_error(ArcMode::continuous, 3.0f, -3.0f), 6.0f));
    CHECK(near(position_error(ArcMode::shortest, 3.0f, -3.0f), 6.0f - 2.0f * pi));
    CHECK(near(position_error(ArcMode::continuous, 10.0f, 1.0f), 9.0f));
}

// GM6020 电流模式满量程 ±16384 ↔ ±3 A; C620 ±16384 ↔ ±20 A; C610 ±10000 ↔ ±10 A.
constexpr ec::control::CurrentScale gm6020_scale{3.0f, 16384.0f};

void current_scale_contracts() {
    using namespace ec::control;
    CHECK(current_scale_valid(gm6020_scale));
    CHECK(!current_scale_valid({0.0f, 16384.0f}));
    CHECK(!current_scale_valid({3.0f, 0.0f}));
    CHECK(!current_scale_valid({-3.0f, 16384.0f}));
    CHECK(!current_scale_valid({std::numeric_limits<float>::quiet_NaN(), 16384.0f}));
    CHECK(!current_scale_valid({3.0f, std::numeric_limits<float>::infinity()}));

    // 旧主 yaw 速度环上限 3000 raw 对应的轴侧电流.
    constexpr float old_cap_raw = 3000.0f;
    constexpr float old_cap_a = old_cap_raw * 3.0f / 16384.0f; // 0.54931640625
    CHECK(near(old_cap_a, 0.5493164f, 1e-6f));
    CHECK(near(amps_to_raw(gm6020_scale, old_cap_a), old_cap_raw, 1e-3f));
    CHECK(near(raw_to_amps(gm6020_scale, old_cap_raw), old_cap_a, 1e-6f));
    CHECK(near(amps_to_raw(gm6020_scale, 3.0f), 16384.0f, 1e-3f));
    CHECK(near(raw_to_amps(gm6020_scale, -16384.0f), -3.0f, 1e-6f));

    // 旧 Kp=73 raw/(deg/s) → A 域 → 归一化域 (归一化 ±1 对应 3000 raw).
    const float kp_amp = 73.0f * (180.0f / pi) * 3.0f / 16384.0f;
    CHECK(near(kp_amp, 0.7658554f, 1e-6f));
    const float kp_normalized = gain_amp_to_normalized(gm6020_scale, old_cap_raw, kp_amp);
    CHECK(near(kp_normalized, 73.0f * (180.0f / pi) / old_cap_raw, 1e-5f));
    CHECK(near(kp_normalized, 1.3941973f, 1e-5f));
    // 同一 A 域增益在更大的编码上限下换算出更小的归一化增益 (归一化基准随上限变化).
    const float wider = gain_amp_to_normalized(gm6020_scale, 16384.0f, kp_amp);
    CHECK(near(wider, 0.25528513f, 1e-5f));
    CHECK(wider < kp_normalized);

    constexpr CurrentScale c620{20.0f, 16384.0f};
    CHECK(near(raw_to_amps(c620, 16384.0f), 20.0f, 1e-4f));
    constexpr CurrentScale c610{10.0f, 10000.0f};
    CHECK(near(raw_to_amps(c610, 10000.0f), 10.0f, 1e-4f));
    CHECK(near(amps_to_raw(c610, 1.0f), 1000.0f, 1e-3f));
}

// 旧 PID: 位置式 P + 误差域微分, 输出在 ±MaxOutput 钳位, dt 小于 1 ms 被强制为 1 ms.
// 这里只做本地对照实现, 不进入正式构建.
struct LegacyReference {
    float kp{}, ki{}, kd{}, max_output{}, integral_limit{};
    float integral{}, previous_error{};
    bool has_previous{};
    float step(float target, float measured, float dt_s) {
        const float dt = dt_s < 0.001f ? 0.001f : dt_s;
        const float error = target - measured;
        float output = kp * error + ki * error * dt + (has_previous ? kd * (error - previous_error) / dt : 0.0f);
        integral += ki * error * dt;
        integral = std::clamp(integral, -integral_limit, integral_limit);
        output = std::clamp(output, -max_output, max_output);
        previous_error = error;
        has_previous = true;
        return output;
    }
};

void pid_reference_comparison() {
    using ec::control::Pid;
    // 纯比例: 相同契约部分要求逐点一致 (含饱和与负误差).
    LegacyReference legacy{2.0f, 0.0f, 0.0f, 3.0f, 0.0f};
    Pid modern({2.0f, 0.0f, 0.0f, 0.0f, -3.0f, 3.0f});
    const float errors[]{0.0f, 0.3f, 1.0f, 1.5f, 2.0f, -0.4f, -5.0f};
    for (float error : errors) {
        CHECK(modern.step(error, 0.0f, 0.001f) == legacy.step(error, 0.0f, 0.001f));
    }

    // 微分语义不同: 新框架微分作用于测量值, 目标跳变不进入微分.
    // 目标恒定且测量变化时两者一致; 只有目标跳变会分开.
    LegacyReference legacy_d{0.0f, 0.0f, 1.0f, 100.0f, 0.0f};
    Pid modern_d({0.0f, 0.0f, 1.0f, 0.0f, -100.0f, 100.0f});
    CHECK(modern_d.step(1.0f, 1.0f, 0.001f) == legacy_d.step(1.0f, 1.0f, 0.001f));
    // 测量与目标一起跳: 旧实现把目标跳变算成微分冲击, 新实现保持 0.
    const float legacy_step = legacy_d.step(5.0f, 2.0f, 0.001f);
    CHECK(near(legacy_step, 100.0f, 1e-3f)); // 被 MaxOutput 钳位, 原始值 4000
    CHECK(*modern_d.step(5.0f, 2.0f, 0.001f) == -100.0f); // 测量变化 1 rad / 1 ms, 同样被钳位
    // 目标恒定、测量继续变化时, 新旧都只反映测量变化.
    CHECK(*modern_d.step(5.0f, 3.0f, 0.001f) == -100.0f);
    CHECK(near(legacy_d.step(5.0f, 3.0f, 0.001f), -100.0f, 1e-3f));
    // 目标跳变而不改变测量: 旧实现对目标跳变产生冲击, 新实现为 0.
    Pid fresh({0.0f, 0.0f, 1.0f, 0.0f, -100.0f, 100.0f});
    CHECK(*fresh.step(1.0f, 1.0f, 0.001f) == 0.0f);
    CHECK(*fresh.step(9.0f, 1.0f, 0.001f) == 0.0f);

    // 旧的积分状态是函数内 static, 多实例互相污染; 新实现按对象持有.
    Pid a({0.0f, 1.0f, 0.0f, 10.0f, -10.0f, 10.0f});
    Pid b({0.0f, 1.0f, 0.0f, 10.0f, -10.0f, 10.0f});
    for (int n = 1; n <= 5; ++n) { CHECK(near(*a.step(1.0f, 0.0f, 1.0f), static_cast<float>(n))); }
    CHECK(*b.step(0.0f, 0.0f, 1.0f) == 0.0f); // b 不受 a 的积分影响
    a.reset();
    CHECK(*a.step(0.0f, 0.0f, 1.0f) == 0.0f); // reset 只清自身
    CHECK(near(*b.step(1.0f, 0.0f, 1.0f), 1.0f));
}

ec::control::VelocityLoopConfig velocity_config(float alpha) {
    return {{1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f}, {50}, 10.0f, 0.05f, alpha};
}

void velocity_filter_contracts() {
    using ec::control::VelocityLoop;
    using ec::control::VelocityStatus;
    using ec::Sample;
    using ec::Validity;
    // 旧驱动 alpha=0.9 的平滑等价于 0.1*旧 + 0.9*新; 新配置直接用旧值系数.
    CHECK(!VelocityLoop{velocity_config(1.0f)}.configured());
    CHECK(!VelocityLoop{velocity_config(-0.1f)}.configured());
    CHECK(VelocityLoop{velocity_config(0.1f)}.configured());
    CHECK(VelocityLoop{velocity_config(0.0f)}.configured());

    VelocityLoop loop{velocity_config(0.1f)};
    const auto grant = loop.arm(ec::CommandSource::remote, true, true);
    CHECK(grant);
    const ec::Command<float> target{0.0f, {1000}, {50}, *grant};
    const auto step = [&](float measured, ec::Tick now) {
        return loop.step(now, 0.001f, target, Sample<float>{measured, now, Validity::valid});
    };
    // 首帧直接采用测量值, 不从 0 起步.
    CHECK(near(*step(0.5f, {1000}).output, -0.5f));
    CHECK(near(*step(1.0f, {1001}).output, -0.95f));   // 0.1*0.5 + 0.9*1.0
    CHECK(near(*step(0.5f, {1002}).output, -0.545f));  // 0.1*0.95 + 0.9*0.5
    // 滤波按控制周期应用, 与反馈到达频率无关; 未滤波时逐点直通.
    VelocityLoop plain{velocity_config(0.0f)};
    const auto plain_grant = plain.arm(ec::CommandSource::remote, true, true);
    CHECK(plain_grant);
    const ec::Command<float> plain_target{0.0f, {1000}, {50}, *plain_grant};
    CHECK(near(*plain.step({1000}, 0.001f, plain_target,
        Sample<float>{0.2f, {1000}, Validity::valid}).output, -0.2f));
    CHECK(near(*plain.step({1001}, 0.001f, plain_target,
        Sample<float>{0.6f, {1001}, Validity::valid}).output, -0.6f));

    // 拒绝会撤销授权并清滤波历史; 重新使能后首个样本仍是原始测量值.
    // 命令有效期与反馈时效分开: 这里命令是新的, 只有反馈过期.
    const ec::Command<float> late_command{0.0f, {2000}, {50}, *grant};
    const auto stale = loop.step({2000}, 0.001f, late_command, Sample<float>{0.5f, {1100}, Validity::valid});
    CHECK(stale.status == VelocityStatus::stale_feedback && !stale.output);
    const ec::Command<float> revoked{0.0f, {2001}, {50}, *grant};
    CHECK(loop.step({2001}, 0.001f, revoked, Sample<float>{0.5f, {2001}, Validity::valid}).status ==
        VelocityStatus::unauthorized);
    const auto rearmed = loop.arm(ec::CommandSource::remote, true, true);
    CHECK(rearmed);
    const ec::Command<float> again{0.0f, {2001}, {50}, *rearmed};
    CHECK(near(*loop.step({2001}, 0.001f, again, Sample<float>{0.6f, {2001}, Validity::valid}).output, -0.6f));

    // dt 契约: 非正、非有限或超过 max_dt 一律拒绝; 旧实现把过小 dt 静默抬到 1 ms.
    // 每次拒绝都会撤销授权, 因此每个用例单独重新使能.
    const auto dt_case = [&](float dt_s) {
        const auto fresh_grant = loop.arm(ec::CommandSource::remote, true, true);
        CHECK(fresh_grant);
        const ec::Command<float> command{0.0f, {1003}, {50}, *fresh_grant};
        return loop.step({1003}, dt_s, command, Sample<float>{0.5f, {1003}, Validity::valid});
    };
    CHECK(dt_case(0.0f).status == VelocityStatus::invalid_dt);
    CHECK(dt_case(0.06f).status == VelocityStatus::invalid_dt);
    CHECK(dt_case(std::numeric_limits<float>::quiet_NaN()).status == VelocityStatus::invalid_dt);
    // 过小但为正的 dt 不是速度环的错误契约, 交给 Pid 按真实 dt 计算.
    CHECK(dt_case(0.0001f).status == VelocityStatus::active);
}

constexpr ec::control::PositionLoopConfig yaw_position_config() {
    return {{7.5f, 0.0f, 0.0f, 0.0f, -3.4906585f, 3.4906585f},
            {{1.3941973f, 0.0f, 0.0f, 1.0f, -1.0f, 1.0f}, {50}, 3.4906585f, 0.01f, 0.0f},
            ec::control::ArcMode::continuous};
}

void position_contracts() {
    using ec::control::ArcMode;
    using ec::control::PositionLoop;
    using ec::control::PositionLoopConfig;
    using ec::control::PositionStatus;
    using ec::control::VelocityStatus;
    using ec::Sample;
    using ec::Validity;

    // 外环限幅必须落在内环目标范围内, 否则配置直接无效.
    auto bad = yaw_position_config();
    bad.position_pid.output_max = 4.0f;
    CHECK(!PositionLoop{bad}.configured());
    bad.position_pid.output_max = 3.4906585f;
    bad.position_pid.output_min = -4.0f;
    CHECK(!PositionLoop{bad}.configured());
    CHECK(PositionLoop{yaw_position_config()}.configured());

    PositionLoop loop{yaw_position_config()};
    const auto grant = loop.arm(ec::CommandSource::remote, true, true);
    CHECK(grant);
    const auto step = [&](float target, float position, float velocity, ec::Tick now) {
        const ec::Command<float> command{target, {100}, {50}, *grant};
        return loop.step(now, 0.001f, command,
            Sample<float>{position, now, Validity::valid},
            Sample<float>{velocity, now, Validity::valid});
    };
    // 位置误差 0.5 rad → 速度目标 3.75 rad/s 被限幅到 3.4906585;
    // 内环误差 3.2906585 rad/s → 归一化输出被内环限幅到 1.
    const auto saturated = step(1.0f, 0.5f, 0.2f, {100});
    CHECK(saturated.status == PositionStatus::active);
    CHECK(near(*saturated.velocity_target, 3.4906585f));
    CHECK(near(*saturated.output, 1.0f));
    // 未饱和时逐项可算: 0.01 rad → 0.075 rad/s → 误差 -0.125 rad/s.
    const auto linear = step(0.51f, 0.5f, 0.2f, {101});
    CHECK(near(*linear.velocity_target, 0.075f));
    CHECK(near(*linear.output, 1.3941973f * -0.125f, 1e-5f));

    // 两种误差定义只改变外环误差, 内环不变.
    auto shortest = yaw_position_config();
    shortest.arc = ArcMode::shortest;
    PositionLoop arc_loop{shortest};
    const auto arc_grant = arc_loop.arm(ec::CommandSource::remote, true, true);
    CHECK(arc_grant);
    const ec::Command<float> arc_target{3.0f, {100}, {50}, *arc_grant};
    const auto continuous = loop.step({102}, 0.001f, arc_target,
        Sample<float>{-3.0f, {102}, Validity::valid}, Sample<float>{0.0f, {102}, Validity::valid});
    const auto wrap = arc_loop.step({102}, 0.001f, arc_target,
        Sample<float>{-3.0f, {102}, Validity::valid}, Sample<float>{0.0f, {102}, Validity::valid});
    CHECK(continuous.status == PositionStatus::active && wrap.status == PositionStatus::active);
    CHECK(near(*continuous.velocity_target, 3.4906585f));            // 6.0 rad → 限幅
    CHECK(near(*wrap.velocity_target, 7.5f * (6.0f - 2.0f * pi), 1e-4f)); // -2.1238898 rad/s
    CHECK(*wrap.velocity_target < 0.0f);

    // 内环拒绝向上传递: 原因保留在内环状态里, 且授权已撤销.
    // 命令有效期与反馈时效分开: 这里命令在检查时刻才签发, 只有速度反馈过期.
    const ec::Command<float> late_target{1.0f, {200}, {50}, *grant};
    const auto inner = loop.step({200}, 0.001f, late_target,
        Sample<float>{0.5f, {200}, Validity::valid}, Sample<float>{0.2f, {149}, Validity::valid});
    CHECK(inner.status == PositionStatus::inner_rejected && !inner.output);
    CHECK(inner.velocity_status == VelocityStatus::stale_feedback);
    CHECK(inner.velocity_target.has_value());
    const auto after_revoke = loop.step({200}, 0.001f, late_target,
        Sample<float>{0.5f, {200}, Validity::valid}, Sample<float>{0.2f, {200}, Validity::valid});
    CHECK(after_revoke.status == PositionStatus::inner_rejected);
    CHECK(after_revoke.velocity_status == VelocityStatus::unauthorized);
    // 外环自己的拒绝路径.
    CHECK(loop.step({201}, 0.001f, std::nullopt, Sample<float>{0.5f, {201}, Validity::valid},
        Sample<float>{0.2f, {201}, Validity::valid}).status == PositionStatus::no_command);
    const auto armed = loop.arm(ec::CommandSource::remote, true, true);
    CHECK(armed);
    const ec::Command<float> again{1.0f, {202}, {50}, *armed};
    CHECK(loop.step({202}, 0.001f, again, Sample<float>{0.5f, {500}, Validity::valid},
        Sample<float>{0.2f, {202}, Validity::valid}).status == PositionStatus::stale_feedback);
    CHECK(loop.step({202}, 0.001f, again, Sample<float>{0.5f, {202}, Validity::invalid},
        Sample<float>{0.2f, {202}, Validity::valid}).status == PositionStatus::no_feedback);
    CHECK(loop.step({202}, 0.02f, again, Sample<float>{0.5f, {202}, Validity::valid},
        Sample<float>{0.2f, {202}, Validity::valid}).status == PositionStatus::invalid_dt);
    CHECK(loop.step({202}, 0.001f, again, Sample<float>{std::numeric_limits<float>::quiet_NaN(), {202},
        Validity::valid}, Sample<float>{0.2f, {202}, Validity::valid}).status == PositionStatus::invalid_target);

    // 控制器 reset 是独立操作: 不清授权, 但清掉会经 D 项放大测量跳变的历史.
    auto with_d = yaw_position_config();
    with_d.position_pid.kd = 0.005f;
    with_d.position_pid.integral_limit = 0.0f;
    PositionLoop d_loop{with_d};
    const auto d_grant = d_loop.arm(ec::CommandSource::remote, true, true);
    CHECK(d_grant);
    const ec::Command<float> d_target{0.5f, {100}, {500}, *d_grant};
    const auto d_step = [&](float position, ec::Tick now) {
        return d_loop.step(now, 0.001f, d_target, Sample<float>{position, now, Validity::valid},
            Sample<float>{0.0f, now, Validity::valid});
    };
    // 误差保持在内环限幅以内, 便于直接读出 D 项对速度目标的贡献.
    CHECK(near(*d_step(0.2f, {100}).velocity_target, 2.25f));  // 7.5*0.3, D 首帧为 0
    CHECK(near(*d_step(0.2f, {101}).velocity_target, 2.25f));  // D 仍为 0
    // 测量跳变 0.1 rad / 1 ms → D = -0.005*100 = -0.5 rad/s.
    CHECK(near(*d_step(0.3f, {102}).velocity_target, 7.5f * 0.2f - 0.5f));
    // 重新建立位置参考后必须清历史, 否则该冲击会重复出现.
    d_loop.reset_controller();
    CHECK(near(*d_step(0.3f, {103}).velocity_target, 7.5f * 0.2f));
}

// 一阶速度 plant: v' = (k*u - v) / tau; u 为归一化执行量. 只验证符号与接线.
struct FirstOrderPlant {
    static constexpr float gain = 80.0f;
    static constexpr float tau_s = 0.08f;
    float velocity{};
    float position{};
    void step(float u, float dt_s) noexcept {
        velocity += (gain * u - velocity) * (1.0f - std::exp(-dt_s / tau_s));
        position += velocity * dt_s;
    }
};

void cascade_plant_contracts() {
    using ec::control::PositionLoop;
    using ec::control::VelocityLoop;
    using ec::Sample;
    using ec::Validity;

    // 速度环: 恒定目标下应收敛到目标转速, 且不需要长期满输出.
    VelocityLoop velocity_loop{{{0.02f, 0.05f, 0.0f, 1.0f, -1.0f, 1.0f}, {50}, 10.0f, 0.01f, 0.0f}};
    const auto velocity_grant = velocity_loop.arm(ec::CommandSource::remote, true, true);
    CHECK(velocity_grant);
    const ec::Command<float> velocity_target{1.0f, {0}, {10000}, *velocity_grant};
    FirstOrderPlant velocity_plant;
    float last_output = 0.0f;
    for (ec::Tick now{1}; now.ms <= 3000; now.ms = now.ms + 1) {
        const auto result = velocity_loop.step(now, 0.001f, velocity_target,
            Sample<float>{velocity_plant.velocity, now, Validity::valid});
        CHECK(result.status == ec::control::VelocityStatus::active);
        last_output = *result.output;
        velocity_plant.step(last_output, 0.001f);
    }
    CHECK(near(velocity_plant.velocity, 1.0f, 0.02f));
    CHECK(std::fabs(last_output) < 1.0f);

    // 位置串级: 正目标 + 正输出应使位置正向收敛; 只用来说明符号与接线, 不是实车结论.
    auto config = yaw_position_config();
    config.velocity.pid = {0.05f, 0.5f, 0.0f, 1.0f, -1.0f, 1.0f};
    config.velocity.max_target_rad_s = 5.0f;
    config.velocity.feedback_ttl = {50};
    config.position_pid = {2.0f, 0.0f, 0.0f, 0.0f, -5.0f, 5.0f};
    PositionLoop position_loop{config};
    const auto position_grant = position_loop.arm(ec::CommandSource::remote, true, true);
    CHECK(position_grant);
    const ec::Command<float> position_target{1.0f, {0}, {10000}, *position_grant};
    FirstOrderPlant position_plant;
    float first_output = 0.0f;
    for (ec::Tick now{1}; now.ms <= 6000; now.ms = now.ms + 1) {
        const auto result = position_loop.step(now, 0.001f, position_target,
            Sample<float>{position_plant.position, now, Validity::valid},
            Sample<float>{position_plant.velocity, now, Validity::valid});
        CHECK(result.status == ec::control::PositionStatus::active);
        if (now.ms == 1) { first_output = *result.output; }
        position_plant.step(*result.output, 0.001f);
    }
    CHECK(first_output > 0.0f);
    CHECK(near(position_plant.position, 1.0f, 0.05f));
    CHECK(position_plant.velocity > 0.0f);
}
void thermal_contracts() {
    // 旧配置换算: TIM10 ARR=4999 对应占空比 1.0, 因此 MaxOutput 2000 → 0.4,
    // IntegralLimit 300 → 0.06, Kp 1000/°C → 0.2/°C, Ki 20 → 0.004.
    ec::control::ThermalController heater{{}};
    CHECK(near(heater.config().kp, 0.2f) && near(heater.config().ki, 0.004f));
    CHECK(near(heater.config().duty_limit, 0.4f) && near(heater.config().integral_limit, 0.06f));
    CHECK(heater.config().target_c == 40.0f);
    // 默认关闭: 不给任何输出.
    CHECK(!heater.enabled());
    CHECK(heater.update(20.0f, {0}, {0}, 0.001f) == 0.0f);
    CHECK(heater.status() == ec::control::ThermalController::Status::off);
    // 使能后低于目标: 正输出且不超过占空比上限.
    heater.set_enabled(true);
    float duty = 0.0f;
    for (unsigned t = 0; t < 200; ++t) { duty = heater.update(20.0f, {t}, {t}, 0.001f); }
    CHECK(duty > 0.0f && duty <= heater.config().duty_limit);
    CHECK(heater.status() == ec::control::ThermalController::Status::active);
    // 远高于目标: 输出被限到 0, 不会反向加热.
    duty = heater.update(80.0f, {200}, {200}, 0.001f);
    CHECK(duty == 0.0f);
    // 温度超时: 输出 0, 不沿用上一次输出.
    duty = heater.update(20.0f, {1000}, {2000}, 0.001f);
    CHECK(duty == 0.0f);
    CHECK(heater.status() == ec::control::ThermalController::Status::temperature_stale);
    // 非法温度与非法周期.
    CHECK(heater.update(std::numeric_limits<float>::quiet_NaN(), {2000}, {2000}, 0.001f) == 0.0f);
    CHECK(heater.status() == ec::control::ThermalController::Status::invalid_input);
    CHECK(heater.update(20.0f, {2000}, {2000}, 0.0f) == 0.0f);
    CHECK(heater.status() == ec::control::ThermalController::Status::invalid_dt);
    // 死区: 误差在带内时保持上一次输出(旧 PID 的不重算行为, 可用 deadband_c=0 关闭).
    ec::control::ThermalController steady{{}};
    steady.set_enabled(true);
    const float held = steady.update(30.0f, {0}, {0}, 0.001f);
    CHECK(held > 0.0f);
    CHECK(steady.update(39.95f, {1}, {1}, 0.001f) == held);
    // 关闭后再使能会清空历史: 积分不会在重新使能时一次性释放.
    steady.set_enabled(false);
    CHECK(steady.update(30.0f, {2}, {2}, 0.001f) == 0.0f);
    steady.set_enabled(true);
    const float restarted = steady.update(39.99f, {3}, {3}, 0.001f);
    CHECK(restarted <= 0.2f); // 只应该剩下比例项量级
}
} // 测试辅助

int main() {
    angle_contracts();
    current_scale_contracts();
    pid_reference_comparison();
    velocity_filter_contracts();
    position_contracts();
    cascade_plant_contracts();
    thermal_contracts();
    std::puts("control contracts passed");
    return 0;
}
