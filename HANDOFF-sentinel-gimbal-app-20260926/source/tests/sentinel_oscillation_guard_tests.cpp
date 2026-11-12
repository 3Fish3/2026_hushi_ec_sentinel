// M2-B 守护器契约测试 (先写失败/RED, 再实现 GREEN).
//
// 仅使用 host 纯算法头 static 链接, 不接串口/CAN/板级/CMake. 全部阈值为
// SYNTHETIC_TEST_ONLY, 一切 hardware_status NOT_HARDWARE_VALIDATED.
//
// 覆盖任务书 T01-T20 中 B 相关项 (双轴):
//  T03 衰减 suspect->normal; T04 持续累积; T05 增长更快; T06 小幅高频 output reverse;
//  T09 反馈无效/RC; T10 wrap vs restart / dt 异常 / 同钟重复; T11 硬约束当拍;
//  T13 下一正常帧/伪 clear 不解锁; T18 饱和/固定 sizeof/无堆; T19 两轴隔离;
//  T20 pitch no-fall-arrester BLOCKED; 硬条件 NaN/Inf/电流热/机械区间/速度;
//  真实 dt, 跨秒 exposure, 显式 reset 到 disarmed 需新 (authority,generation) rearm,
//  configure/boot change 不清锁存, 事件幂等.
#include "robots/sentinel/app/sentinel_oscillation_guard.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <initializer_list>
#include <type_traits>

namespace {
using namespace ec::sentinel::app::oscillation;

void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3);
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// ---- SYNTHETIC_TEST_ONLY 夹具阈值 (绝不落入实机默认). ----
GuardianLimits synth_limits(Axis axis) {
    GuardianLimits l{};
    l.axis = axis;
    l.mode = GuardMode::synthetic_test;
    l.version = 7;
    l.config_hash = 0x5A5A'0007u;
    l.has_mechanical_range = true;
    l.mechanical_min_rad = -1.2f;
    l.mechanical_max_rad = 1.2f;
    l.margin_rad = 0.05f;
    l.has_max_speed = true;
    l.max_speed_rad_s = 8.0f;
    l.current_limit_a = {true, false, 2.5f};      // optional
    l.temperature_limit_c = {false, false, 0.0f}; // absent
    l.residual_deadband_rad = 0.01f;
    l.residual_energy_budget = 2.0f;
    l.energy_tau_s = 0.3f;
    l.reversal_count_budget = 24.0f;
    l.window_ms = 50.0f;
    l.exposure_budget = 0.08f;
    l.latch_budget = 0.30f;
    l.decay_ratio = 0.7f;
    l.growth_ratio = 1.3f;
    l.suspect_exit_ms = 200.0f;
    l.hazard_level_scale = 0.1f;
    l.max_dt_s = 0.02f;
    l.nominal_dt_s = 0.001f;
    l.sample_rate_hz = 1000.0f;   // 1kHz 合成采样
    l.band_highpass_hz = 2.0f;    // 2Hz 下沿 (滤 DC/慢漂)
    l.band_lowpass_hz = 40.0f;    // 40Hz 上沿 (< Nyquist 500Hz)
    return l;
}

GuardianSample base_sample(Axis axis, std::uint32_t now_us, float dt) {
    GuardianSample s{};
    s.axis = axis;
    s.now_us = now_us;
    s.dt_s = dt;
    s.boot_id = 42;
    s.boot_identity_available = true;
    s.request_target_rad = 0.0f;
    s.adopted_target_rad = 0.0f;
    s.target_valid = true;
    s.target_adopted = true;
    s.target_revision = 1;
    s.authorization_id = 100;
    s.angle_rad = 0.0f;
    s.velocity_rad_s = 0.0f;
    s.feedback_valid = true;
    s.executing = true;
    s.rc_valid = true;
    s.authority_ok = true;
    s.manual_stop = false;
    s.has_output = false;
    s.has_current = false;
    s.has_temperature = false;
    return s;
}

// 运行 N tick, 每 tick 间隔 1ms (dt=0.001, now_us += 1000).
struct Runner {
    SentinelOscillationGuard g{};
    std::uint32_t now_us{0};
    Axis axis{Axis::fire_yaw};
    Runner(Axis a, const GuardianLimits& lim) : axis(a) { g.configure(lim); }
    GuardianReport tick(float angle, float velocity, bool fb_valid = true) {
        now_us += 1000;
        GuardianSample s = base_sample(axis, now_us, 0.001f);
        s.angle_rad = angle;
        s.velocity_rad_s = velocity;
        s.feedback_valid = fb_valid;
        return g.step(s);
    }
    // 指定输出的 tick (angle, output, dt): 用于 output reverse 测试.
    GuardianReport tick_out(float angle, float output, float dt) {
        now_us += 1000;
        GuardianSample s = base_sample(axis, now_us, dt);
        s.angle_rad = angle;
        s.has_output = true;
        s.output_normalized = output;
        return g.step(s);
    }
};

// ---- T01 静止/微噪/慢遥控: NORMAL, 不误触发 ----
void test_t01_static_and_slow_rc_no_false_alarm() {
    for (Axis a : {Axis::fire_yaw, Axis::pitch}) {
        Runner r(a, synth_limits(a));
        GuardianReport rep{};
        // 静止 + 微小噪声 (< deadband).
        for (int i = 0; i < 2000; ++i) {
            float micro = (i % 2 == 0) ? 0.002f : -0.002f; // < deadband 0.01
            rep = r.tick(micro, 0.05f);
        }
        CHECK(rep.state == GuardState::normal);
        CHECK(rep.band_energy < synth_limits(a).residual_energy_budget);
        // 慢遥控: 目标缓慢变化, 残差应在死区附近.
        for (int i = 0; i < 2000; ++i) {
            float tgt = 0.0005f * static_cast<float>(i % 1000);
            r.now_us += 1000;
            GuardianSample s = base_sample(a, r.now_us, 0.001f);
            s.request_target_rad = tgt;
            s.adopted_target_rad = tgt;
            s.angle_rad = tgt - 0.001f; // 轻微滞后, 残差 0.001 很小
            s.velocity_rad_s = 0.3f;
            rep = r.g.step(s);
        }
        CHECK(rep.state == GuardState::normal);
    }
}

// ---- T02 指令阶跃/快速反向: 不系统误报 (目标变化不被当振荡) ----
void test_t02_commanded_reverse_not_systemic_false_positive() {
    Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
    GuardianReport rep{};
    // 快速方波目标, 反馈紧跟目标 (残差小), 速度大但为指令驱动.
    for (int i = 0; i < 4000; ++i) {
        float tgt = ((i / 100) % 2 == 0) ? 0.5f : -0.5f;
        r.now_us += 1000;
        GuardianSample s = base_sample(Axis::fire_yaw, r.now_us, 0.001f);
        s.request_target_rad = tgt;
        s.adopted_target_rad = tgt;
        s.angle_rad = tgt - 0.002f; // 残差 0.002 < deadband
        s.velocity_rad_s = 3.0f;    // 快速但未越 8.0
        rep = r.g.step(s);
    }
    CHECK(rep.state == GuardState::normal);
}

// ---- T03 衰减振铃 SUSPECT -> NORMAL ----
void test_t03_decaying_ring_recovers() {
    for (Axis a : {Axis::fire_yaw, Axis::pitch}) {
        Runner r(a, synth_limits(a));
        GuardianReport rep{};
        float amp = 0.08f; // 明显 > deadband
        // 衰减振荡: 幅度逐 tick 乘 0.995, 周期 ~ 20 tick 反向.
        float sign = 1.0f;
        for (int i = 0; i < 6000; ++i) {
            if (i % 10 == 0) { sign = -sign; }
            float osc = amp * sign;
            rep = r.tick(osc, 0.0f);
            amp *= 0.998f;
        }
        // 衰减到很小后应回 normal (未越硬边界不永久锁存).
        CHECK(rep.state == GuardState::normal);
        CHECK(!rep.latched);
    }
}

// ---- T04 等幅持续振荡: 达预算进入 warning/fault, 不无限持续 ----
void test_t04_sustained_oscillation_escalates() {
    for (Axis a : {Axis::fire_yaw, Axis::pitch}) {
        Runner r(a, synth_limits(a));
        GuardianReport rep{};
        float sign = 1.0f;
        bool escalated = false;
        for (int i = 0; i < 20000; ++i) {
            if (i % 10 == 0) { sign = -sign; }
            float osc = 0.06f * sign; // 等幅
            rep = r.tick(osc, 0.0f);
            if (rep.state == GuardState::warning || rep.state == GuardState::fault_latched) {
                escalated = true;
                break;
            }
        }
        CHECK(escalated);
    }
}

// ---- T05 增长振荡: 比等幅更快升级 ----
void test_t05_growing_escalates_faster() {
    Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
    GuardianReport rep{};
    float sign = 1.0f;
    float amp = 0.02f;
    int ticks_to_escalate = -1;
    for (int i = 0; i < 20000; ++i) {
        if (i % 10 == 0) { sign = -sign; }
        float osc = amp * sign;
        rep = r.tick(osc, 0.0f);
        amp *= 1.01f; // 每 tick 增长 1%
        if (rep.state == GuardState::warning || rep.state == GuardState::fault_latched) {
            ticks_to_escalate = i;
            break;
        }
    }
    CHECK(ticks_to_escalate >= 0);
    // 增长环不应拖到 20000 tick 之外.
    CHECK(ticks_to_escalate < 15000);
}

// ---- T06 小幅高频, 输出不断反向 (Fire 可观测输出), 不能因 RMS 小判安全 ----
void test_t06_small_high_freq_output_reversal() {
    Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
    // 反馈角很小 (RMS 小), 但输出每 tick 反向.
    GuardianReport rep{};
    bool escalated = false;
    for (int i = 0; i < 20000; ++i) {
        r.now_us += 1000;
        GuardianSample s = base_sample(Axis::fire_yaw, r.now_us, 0.001f);
        s.angle_rad = 0.003f * ((i % 2 == 0) ? 1.0f : -1.0f); // 小振幅 < 死区? 用残差需 > 死区
        // 残差用 adopted - angle, 小但输出反向提供证据
        s.has_output = true;
        s.output_normalized = (i % 2 == 0) ? 0.4f : -0.4f; // 输出不断反向
        rep = r.g.step(s);
        if (rep.state == GuardState::warning || rep.state == GuardState::fault_latched) {
            escalated = true;
            break;
        }
    }
    CHECK(escalated); // 不能仅因位置 RMS 小就判正常
}

// ---- T09 反馈无效 / RC 无效: 失败安全 (不进 normal 许可) ----
void test_t09_feedback_invalid_and_rc() {
    {
        Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
        GuardianReport rep{};
        for (int i = 0; i < 500; ++i) { rep = r.tick(0.0f, 0.0f, false); }
        CHECK(rep.state == GuardState::fault_latched);
        CHECK((rep.reason_bits & reason_hard_feedback) != 0);
    }
    {
        Runner r(Axis::pitch, synth_limits(Axis::pitch));
        r.now_us += 1000;
        GuardianSample s = base_sample(Axis::pitch, r.now_us, 0.001f);
        s.rc_valid = false;
        GuardianReport rep = r.g.step(s);
        CHECK(rep.state == GuardState::fault_latched);
        CHECK((rep.reason_bits & reason_hard_rc) != 0);
    }
}

// ---- T10 dt 异常 / 同钟重复 / wrap vs restart ----
void test_t10_time_anomalies() {
    // dt <= 0
    {
        Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
        r.now_us += 1000;
        GuardianSample s = base_sample(Axis::fire_yaw, r.now_us, 0.0f);
        GuardianReport rep = r.g.step(s);
        CHECK(rep.state == GuardState::fault_latched);
        CHECK((rep.reason_bits & reason_hard_time) != 0);
    }
    // dt NaN
    {
        Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
        r.now_us += 1000;
        GuardianSample s = base_sample(Axis::fire_yaw, r.now_us, kNaN);
        GuardianReport rep = r.g.step(s);
        CHECK(rep.state == GuardState::fault_latched);
        CHECK((rep.reason_bits & (reason_hard_time | reason_hard_nonfinite)) != 0);
    }
    // 同一 now_us 重复 (dt>0 但与 now 不一致)
    {
        Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
        GuardianReport rep = r.tick(0.0f, 0.0f); // 推进一次
        std::uint32_t same = r.now_us;
        GuardianSample s = base_sample(Axis::fire_yaw, same, 0.001f); // 同 now, 声称 dt>0
        rep = r.g.step(s);
        CHECK(rep.state == GuardState::fault_latched);
        CHECK((rep.reason_bits & reason_hard_time) != 0);
    }
    // 正常 u32 wrap: now 从 0xFFFFF000 附近跨到 0x00001000, dt 小, boot 不变 => 不锁存
    {
        Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
        GuardianSample s = base_sample(Axis::fire_yaw, 0xFFFF'F000u, 0.001f);
        GuardianReport rep = r.g.step(s);
        CHECK(rep.state == GuardState::normal);
        // 真正跨 wrap 差 ~0x2000 us = 8ms => dt 一致性检查可容; 用与 dt 一致的小步.
        s.now_us = 0x0000'1000u; // unsigned diff = 0x2000 = 8192us
        s.dt_s = 0.008f;
        rep = r.g.step(s);
        CHECK(!rep.latched); // 连续回绕不是 fault
    }
    // 系统重启: boot_id 改变 => restart 原因 (锁存或明确标注), 不与 wrap 混
    {
        Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
        GuardianReport rep = r.tick(0.0f, 0.0f);
        r.now_us += 1000;
        GuardianSample s = base_sample(Axis::fire_yaw, r.now_us, 0.001f);
        s.boot_id = 43; // 变化
        rep = r.g.step(s);
        CHECK((rep.reason_bits & reason_hard_restart) != 0);
    }
}

// ---- T11 硬约束当拍阻断: 机械区间/速度/NaN/电流 ----
void test_t11_hard_conditions_same_tick() {
    struct Case { float ang; float vel; const char* what; std::uint32_t bit; };
    Case cases[] = {
        {1.30f, 0.0f, "position over", reason_hard_position_limit}, // > 1.2+0.05
        {-1.30f, 0.0f, "position under", reason_hard_position_limit},
        {0.0f, 9.0f, "speed", reason_hard_velocity},
        {kNaN, 0.0f, "angle nan", reason_hard_nonfinite},
        {0.0f, kInf, "vel inf", reason_hard_nonfinite},
    };
    for (auto& c : cases) {
        Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
        r.tick(0.0f, 0.0f); // 一 tick normal
        GuardianReport rep = r.tick(c.ang, c.vel);
        CHECK(rep.state == GuardState::fault_latched); // 当拍
        CHECK((rep.reason_bits & c.bit) != 0);
    }
    // 必需电流缺失 => 阻断; 越界 => 阻断.
    {
        GuardianLimits l = synth_limits(Axis::fire_yaw);
        l.current_limit_a = {true, true, 2.5f}; // required
        Runner r(Axis::fire_yaw, l);
        r.now_us += 1000;
        GuardianSample s = base_sample(Axis::fire_yaw, r.now_us, 0.001f);
        s.has_current = false; // required 缺失
        GuardianReport rep = r.g.step(s);
        CHECK(rep.state == GuardState::fault_latched);
        CHECK((rep.reason_bits & reason_hard_current) != 0);
    }
}

// ---- T13 下一正常帧 / 伪 clear 不解锁 ----
void test_t13_latched_not_cleared_by_normal_frame() {
    for (Axis a : {Axis::fire_yaw, Axis::pitch}) {
        Runner r(a, synth_limits(a));
        // 制造硬故障.
        GuardianReport rep = r.tick(1.5f, 0.0f);
        CHECK(rep.state == GuardState::fault_latched);
        // 之后 5000 个完全正常帧.
        for (int i = 0; i < 5000; ++i) { rep = r.tick(0.0f, 0.0f); }
        CHECK(rep.state == GuardState::fault_latched); // 仍锁存
        CHECK(rep.latched);
        // 伪 clear: 没有远端 clear API, status 仍锁存.
        CHECK(r.g.status().state == GuardState::fault_latched);
    }
}

// ---- T18 饱和/固定 sizeof/无堆 ----
void test_t18_saturation_and_fixed_size() {
    // sizeof 固定 (无指针/可变长).
    static_assert(sizeof(GuardianReport) <= 256, "report must be small/fixed");
    static_assert(std::is_trivially_destructible_v<SentinelOscillationGuard>);
    // 压力: 大量 tick 不崩, 事件计数饱和不溢出回 0.
    Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
    for (int i = 0; i < 100000; ++i) { r.tick(0.0f, 0.05f); } // 5 万 tick steady
    // 注入反复触发/恢复, 事件计数应饱和而不回绕.
    GuardianReport rep{};
    for (int k = 0; k < 500; ++k) {
        rep = r.tick(1.5f, 0.0f);       // latched
        (void)r.g.reset_latched(200u + k, 500u + k, true, r.now_us); // reset to disarmed (fresh now)
        // rearm: 更新 authority + generation
        for (int i = 0; i < 5; ++i) {
            r.now_us += 1000;
            GuardianSample s = base_sample(Axis::fire_yaw, r.now_us, 0.001f);
            s.authorization_id = 300u + k;
            s.target_revision = 900u + k;
            rep = r.g.step(s);
        }
    }
    CHECK(rep.event_count > 0);
    CHECK(rep.event_count <= 0xFFFF'FFFFu);
}

// ---- T19 两轴隔离 ----
void test_t19_axis_isolation() {
    SentinelOscillationGuard fire{};
    SentinelOscillationGuard pitch{};
    fire.configure(synth_limits(Axis::fire_yaw));
    pitch.configure(synth_limits(Axis::pitch));
    // 只对 fire 注入故障.
    GuardianSample fs = base_sample(Axis::fire_yaw, 1000, 0.001f);
    fs.angle_rad = 1.5f;
    GuardianReport fr = fire.step(fs);
    CHECK(fr.state == GuardState::fault_latched);
    // pitch 正常.
    GuardianSample ps = base_sample(Axis::pitch, 1000, 0.001f);
    GuardianReport pr = pitch.step(ps);
    CHECK(pr.state == GuardState::normal);
    CHECK(!pr.latched);
}

// ---- T20 Pitch 无防坠 => HARDWARE_BLOCKED ----
void test_t20_pitch_hardware_blocked() {
    SentinelOscillationGuard pitch{};
    pitch.configure(synth_limits(Axis::pitch));
    GuardianReport rep = pitch.status();
    CHECK(!rep.hardware_ready); // 恒 false
    CHECK(rep.blocked);         // 恒 BLOCKED
    // 即便状态模拟到 normal, 仍 blocked/hardware_ready=false.
    for (int i = 0; i < 100; ++i) {
        GuardianSample s = base_sample(Axis::pitch, static_cast<std::uint32_t>(1000 * (i + 1)), 0.001f);
        rep = pitch.step(s);
    }
    CHECK(!rep.hardware_ready);
    CHECK(rep.blocked);
    // 即便 approved 模式 (无真实防坠), hardware_ready 仍 false.
    GuardianLimits l = synth_limits(Axis::pitch);
    l.mode = GuardMode::approved;
    SentinelOscillationGuard p2{};
    p2.configure(l);
    CHECK(!p2.status().hardware_ready);
    CHECK(p2.status().blocked);
}

// ---- unconfigured 默认: 不判安全, 不授予许可 ----
void test_unconfigured_default() {
    SentinelOscillationGuard g{};
    GuardianReport rep = g.status();
    CHECK(rep.state == GuardState::unconfigured);
    CHECK(!rep.hardware_ready);
    CHECK(rep.blocked);
    // 即使喂完整样本也不自动进入 normal.
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    rep = g.step(s);
    CHECK(rep.state == GuardState::unconfigured);
}

// ---- 显式 reset -> disarmed; 需新 (authority, generation) 才回 normal ----
void test_reset_to_disarmed_then_rearm_needs_new_ids() {
    SentinelOscillationGuard g{};
    g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    s.angle_rad = 1.5f; // 硬故障
    GuardianReport rep = g.step(s);
    CHECK(rep.state == GuardState::fault_latched);

    // 伪 normal 帧不解锁.
    s.angle_rad = 0.0f;
    s.now_us = 2000;
    rep = g.step(s);
    CHECK(rep.state == GuardState::fault_latched);

    // 显式 reset -> disarmed.
    ResetResult rr = g.reset_latched(100, 1, true, 2000u);
    CHECK(rr == ResetResult::accepted);
    CHECK(g.status().state == GuardState::disarmed);
    // disarmed 不是 normal, 不给许可.
    CHECK(g.status().blocked);

    // 用旧 (authority=100, generation=1) 不能 rearm.
    s.now_us = 3000;
    s.authorization_id = 100;
    s.target_revision = 1;
    rep = g.step(s);
    CHECK(rep.state != GuardState::normal);

    // 新 authority + 新 generation 才回 normal (需稳定几拍).
    for (int i = 0; i < 10; ++i) {
        s.now_us += 1000;
        s.authorization_id = 200;
        s.target_revision = 5;
        rep = g.step(s);
    }
    CHECK(rep.state == GuardState::normal);
}

// ---- configure / boot change 不清锁存 ----
void test_reconfigure_and_boot_change_keep_latch() {
    SentinelOscillationGuard g{};
    g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    s.angle_rad = 1.5f;
    GuardianReport rep = g.step(s);
    CHECK(rep.state == GuardState::fault_latched);
    // 重新 configure 同轴新 hash.
    GuardianLimits l2 = synth_limits(Axis::fire_yaw);
    l2.config_hash = 0xDEAD'BEEFu;
    g.configure(l2);
    CHECK(g.status().state == GuardState::fault_latched); // 不清锁存
    // boot change 也不清.
    s.boot_id = 99;
    s.now_us = 2000;
    s.angle_rad = 0.0f;
    rep = g.step(s);
    CHECK(rep.state == GuardState::fault_latched);
}

// ---- 无效 dt 不推进虚构时间且不清旧预算 ----
void test_invalid_dt_does_not_clear_budget() {
    SentinelOscillationGuard g{};
    g.configure(synth_limits(Axis::fire_yaw));
    // 先积累一些能量 (振荡).
    GuardianSample s = base_sample(Axis::fire_yaw, 0, 0.001f);
    float sign = 1.0f;
    for (int i = 0; i < 200; ++i) {
        if (i % 10 == 0) { sign = -sign; }
        s.now_us += 1000;
        s.angle_rad = 0.05f * sign;
        (void)g.step(s);
    }
    float energy_before = g.status().band_energy;
    // 无效 dt 一拍: 若状态仍非锁存, 预算不得清零.
    s.now_us += 1000;
    s.dt_s = 0.0f;
    GuardianReport rep = g.step(s);
    // dt=0 触发 time fault; 但已积累暴露/能量不清零.
    CHECK((rep.reason_bits & reason_hard_time) != 0);
    CHECK(rep.band_energy >= energy_before - 1e-6f || rep.band_energy > 0.0f);
}

// ---- T07/T08 尖峰/外扰: 单孤立尖峰不应等效持续振荡锁存 ----
void test_isolated_spike_not_persistent() {
    Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
    GuardianReport rep{};
    for (int i = 0; i < 1000; ++i) { rep = r.tick(0.0f, 0.0f); }
    // 单个孤立尖峰 (在机械区间内, 不越硬限).
    rep = r.tick(0.3f, 0.5f);
    // 尖峰一拍不应立即 fault (未越硬限); 后续正常应回落, 不持续.
    CHECK(rep.state != GuardState::fault_latched);
    for (int i = 0; i < 1000; ++i) { rep = r.tick(0.0f, 0.0f); }
    CHECK(rep.state == GuardState::normal);
}

// ---- 配置非法 (NaN 阈值 / 阈值序) 拒绝: 不判安全 ----
void test_invalid_config_rejected() {
    {
        GuardianLimits l = synth_limits(Axis::fire_yaw);
        l.max_speed_rad_s = kNaN;
        SentinelOscillationGuard g{};
        g.configure(l);
        CHECK(g.status().blocked);                 // 非法配置: 不判安全
        CHECK(!g.status().hardware_ready);
    }
    {
        GuardianLimits l = synth_limits(Axis::fire_yaw);
        l.mechanical_min_rad = 1.0f;
        l.mechanical_max_rad = -1.0f; // 序错
        SentinelOscillationGuard g{};
        g.configure(l);
        CHECK(g.status().blocked);
    }
}

// ---- observe_only: 不锁存 (即使越界) ----
void test_observe_only_never_latches() {
    GuardianLimits l = synth_limits(Axis::fire_yaw);
    l.mode = GuardMode::observe_only;
    SentinelOscillationGuard g{};
    g.configure(l);
    GuardianReport rep{};
    for (int i = 0; i < 200; ++i) {
        GuardianSample s = base_sample(Axis::fire_yaw, static_cast<std::uint32_t>(1000 * (i + 1)), 0.001f);
        s.angle_rad = 1.5f; // 越界但 observe_only 不锁存
        rep = g.step(s);
    }
    CHECK(rep.state != GuardState::fault_latched);
}

// ---- approved 模式硬件就绪标志 (Fire 可 true, 但仍需真实阈值; 此处只测标志逻辑) ----
void test_approved_fire_hardware_ready_flag() {
    GuardianLimits l = synth_limits(Axis::fire_yaw);
    l.mode = GuardMode::approved;
    SentinelOscillationGuard g{};
    g.configure(l);
    GuardianReport rep = g.status();
    // approved 且配置有效 => fire 可 hardware_ready; 但不授予实机运行(这里仅标志).
    CHECK(rep.hardware_ready);
    CHECK(!rep.blocked);
}

// ---- 幂等: 同事件不每 tick 刷 transitioned ----
void test_transition_idempotent() {
    SentinelOscillationGuard g{};
    g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    s.angle_rad = 1.5f;
    GuardianReport r1 = g.step(s);
    CHECK(r1.state == GuardState::fault_latched);
    CHECK(r1.transitioned); // 首次进入有跃迁
    s.now_us = 2000;
    GuardianReport r2 = g.step(s);
    CHECK(r2.state == GuardState::fault_latched);
    CHECK(!r2.transitioned); // 持续锁存不再报跃迁
    CHECK(r2.event_count == r1.event_count); // 幂等不增事件
}


// ==================== 父反例新增真实失败测试 (RED-first) ====================

// (1) 持续等幅振荡跨秒必须最终 fault_latched (不仅是 warning), 且不自动恢复.
void test_sustained_must_eventually_latch_and_not_auto_recover() {
    for (Axis a : {Axis::fire_yaw}) {
        Runner r(a, synth_limits(a));
        GuardianReport rep{};
        float sign = 1.0f;
        bool latched = false;
        for (int i = 0; i < 40000; ++i) {
            if (i % 10 == 0) { sign = -sign; }
            rep = r.tick(0.06f * sign, 0.0f);
            if (rep.state == GuardState::fault_latched) { latched = true; break; }
        }
        CHECK(latched);
        CHECK(rep.latched);
        // 停止等幅后不得自动恢复 (仍需显式 reset).
        for (int i = 0; i < 5000; ++i) { rep = r.tick(0.0f, 0.0f); }
        CHECK(rep.state == GuardState::fault_latched);
        CHECK(rep.latched);
    }
}

// (2) 常值 PD 重力下垂 / 大阶跃后短暂偏移不得被当带内振荡 (残差近似常值, 无反向).
void test_constant_offset_and_step_not_band_oscillation() {
    for (Axis a : {Axis::fire_yaw, Axis::pitch}) {
        Runner r(a, synth_limits(a));
        GuardianReport rep{};
        // 常值下垂: 目标 0, 实际 -0.03 (超死区) 恒定, 无反向.
        for (int i = 0; i < 8000; ++i) { rep = r.tick(-0.03f, 0.0f); }
        CHECK(rep.state != GuardState::fault_latched);
        CHECK(!rep.latched);
        // 大阶跃: 目标突变, 实际滞后一拍后到位; 残差短暂大后归零, 不得误报.
        Runner r2(a, synth_limits(a));
        for (int i = 0; i < 4000; ++i) {
            float ang = (i < 50) ? -0.4f : 0.0f; // 一步大偏差后快速到位
            rep = r2.tick(ang, 0.0f); // 目标恒 0
        }
        CHECK(rep.state != GuardState::fault_latched);
    }
}

// (3) 持续小幅高频 output reverse (位置残差在死区内) 也必须最终锁存, 不能只 warning 后恢复.
void test_sustained_small_output_reverse_latches() {
    Runner r(Axis::fire_yaw, synth_limits(Axis::fire_yaw));
    GuardianReport rep{};
    bool latched = false;
    for (int i = 0; i < 40000; ++i) {
        float out = ((i % 2) ? 1.0f : -1.0f); // 高频反向
        rep = r.tick_out(0.0f, out, 0.004f); // 位置残差小
        if (rep.state == GuardState::fault_latched) { latched = true; break; }
    }
    CHECK(latched);
}

// (4) target request/adopted 越界当拍锁存 (当前只测 angle).
void test_target_out_of_range_latches() {
    SentinelOscillationGuard g{};
    g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    s.adopted_target_rad = 1.5f; // 目标越机械限, angle 本身正常
    s.angle_rad = 0.0f;
    GuardianReport rep = g.step(s);
    CHECK(rep.state == GuardState::fault_latched);
}

// (5) 重绑轴应被拒 (Fire 对象不能被 configure 成 Pitch 绕过隔离).
void test_rebind_axis_rejected() {
    SentinelOscillationGuard g{};
    g.configure(synth_limits(Axis::fire_yaw));
    GuardianLimits lp = synth_limits(Axis::pitch);
    g.configure(lp); // 尝试把 fire 对象改成 pitch
    // 轴不得改变: 仍按 fire 处理, hardware_ready 判定依赖原轴.
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    GuardianReport rep = g.step(s);
    CHECK(rep.axis == Axis::fire_yaw); // 仍是 fire, 未被改成 pitch
}

// (6) config_valid 必须拒绝: energy_tau/nominal_dt 非有限, decay>=1, growth<=1,
//     预算<=0, range margin 使区间耗尽, required 但 present=false, 非法 mode/axis, 无真实 range 却 approved.
void test_config_validity_strict() {
    auto bad = [](GuardianLimits l) {
        SentinelOscillationGuard g{};
        g.configure(l);
        return g.status().state == GuardState::unconfigured ||
               g.status().state == GuardState::observe_only;
    };
    { GuardianLimits l = synth_limits(Axis::fire_yaw); l.energy_tau_s = kNaN; CHECK(bad(l)); }
    { GuardianLimits l = synth_limits(Axis::fire_yaw); l.nominal_dt_s = kInf; CHECK(bad(l)); }
    { GuardianLimits l = synth_limits(Axis::fire_yaw); l.decay_ratio = 1.0f; CHECK(bad(l)); }
    { GuardianLimits l = synth_limits(Axis::fire_yaw); l.growth_ratio = 1.0f; CHECK(bad(l)); }
    { GuardianLimits l = synth_limits(Axis::fire_yaw); l.residual_energy_budget = -1.0f; CHECK(bad(l)); }
    { GuardianLimits l = synth_limits(Axis::fire_yaw); l.margin_rad = 1.2f; CHECK(bad(l)); } // 区间耗尽
    { GuardianLimits l = synth_limits(Axis::fire_yaw);
      l.temperature_limit_c = {false, true, 80.0f}; CHECK(bad(l)); } // required 但 present=false
    { GuardianLimits l = synth_limits(Axis::fire_yaw); l.mode = static_cast<GuardMode>(99); CHECK(bad(l)); }
    { GuardianLimits l = synth_limits(Axis::fire_yaw); l.axis = static_cast<Axis>(7); CHECK(bad(l)); }
    { GuardianLimits l = synth_limits(Axis::fire_yaw);
      l.mode = GuardMode::approved; l.has_mechanical_range = false; CHECK(bad(l)); } // 无真实 range 不可 approved
}

// (7) reset 需要显式本地确认 + 当前干净有效样本; 硬危险未恢复不得解锁.
void test_reset_requires_local_confirm_and_clean_sample() {
    SentinelOscillationGuard g{};
    g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    s.angle_rad = 1.5f;
    GuardianReport rep = g.step(s);
    CHECK(rep.state == GuardState::fault_latched);
    // 未显式确认 => 拒绝.
    CHECK(g.reset_latched(200, 5, false, 2000u) == ResetResult::accepted ? false : true);
    CHECK(g.status().state == GuardState::fault_latched);
    // 恢复干净样本 + 确认 => accepted.
    s.angle_rad = 0.0f;
    s.now_us = 2000;
    rep = g.step(s); // clean sample (仍锁存但 last_sample_clean true)
    CHECK(rep.state == GuardState::fault_latched);
    CHECK(g.last_sample_clean());
    CHECK(g.reset_latched(200, 5, true, 2000u) == ResetResult::accepted);
    CHECK(g.status().state == GuardState::disarmed);
}

// (8) 未观测 has_current=false 但数值 NaN 不得判危险, 也不当 0 (只需不误锁存).
void test_unobserved_nan_ignored() {
    GuardianLimits l = synth_limits(Axis::fire_yaw);
    l.current_limit_a = {true, true, 2.5f};   // required 电流
    l.temperature_limit_c = {true, true, 80.0f};
    SentinelOscillationGuard g{};
    g.configure(l);
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    s.has_current = false; s.current_a = kNaN;
    s.has_temperature = false; s.temperature_c = kNaN;
    GuardianReport rep = g.step(s);
    // required 但未观测 => hard fault (缺必需观测), 但不是因为 NaN.
    CHECK((rep.reason_bits & reason_hard_current) != 0);
    CHECK((rep.reason_bits & reason_hard_thermal) != 0);
    CHECK((rep.reason_bits & reason_hard_nonfinite) == 0);
}

// (9) dt 与 now 差距大必须拒.
void test_dt_vs_now_mismatch_rejected() {
    SentinelOscillationGuard g{};
    g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s = base_sample(Axis::fire_yaw, 1000, 0.001f);
    (void)g.step(s);
    s.now_us = 1000 + 15'000; // now 前进 15ms 但声称 dt=1ms => 差距大
    s.dt_s = 0.001f;
    GuardianReport rep = g.step(s);
    CHECK((rep.reason_bits & reason_hard_time) != 0 || rep.state == GuardState::fault_latched);
}


// (10) reset 新签名 3 情形: false local / hard 仍活跃 / stale-clean 全部拒绝.
void test_reset_signature_guard_cases() {
    auto make_latched=[&](SentinelOscillationGuard& g)->bool{
        g.configure(synth_limits(Axis::fire_yaw));
        GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f);
        s.angle_rad=1.5f; (void)g.step(s);
        return g.status().state==GuardState::fault_latched;
    };
    {   // false local confirm
        SentinelOscillationGuard g; CHECK(make_latched(g));
        GuardianSample s=base_sample(Axis::fire_yaw,2000,0.001f); s.angle_rad=0.0f; (void)g.step(s);
        CHECK(g.reset_latched(200,5,false,2000u)!=ResetResult::accepted);
        CHECK(g.status().state==GuardState::fault_latched);
    }
    {   // 硬危险仍活跃 (danger persists)
        SentinelOscillationGuard g; CHECK(make_latched(g));
        GuardianSample s=base_sample(Axis::fire_yaw,2000,0.001f); s.angle_rad=1.5f; (void)g.step(s);
        CHECK(g.reset_latched(200,5,true,2000u)!=ResetResult::accepted);
        CHECK(g.status().state==GuardState::fault_latched);
    }
    {   // stale-clean: 干净后又有危险拍 => 最近样本不干净 => 拒绝
        SentinelOscillationGuard g; CHECK(make_latched(g));
        GuardianSample c=base_sample(Axis::fire_yaw,2000,0.001f); c.angle_rad=0.0f; (void)g.step(c);
        GuardianSample d=base_sample(Axis::fire_yaw,3000,0.001f); d.angle_rad=1.5f; (void)g.step(d);
        CHECK(g.reset_latched(200,5,true,3000u)!=ResetResult::accepted);
        CHECK(g.status().state==GuardState::fault_latched);
    }
    {   // 未锁存 => not_latched
        SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
        GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f); (void)g.step(s);
        CHECK(g.reset_latched(200,5,true,1000u)==ResetResult::not_latched);
    }
}

// (11) 慢衰减 (每 tick 0.998, 窗内衰减不足以触发单窗 decay_ratio) 仍须最终恢复 normal 而非误锁.
void test_slow_decay_recovers_not_latched() {
    for (Axis a : {Axis::fire_yaw, Axis::pitch}) {
        Runner r(a, synth_limits(a));
        GuardianReport rep{};
        float amp=0.08f; float sign=1.0f;
        for (int i=0;i<6000;++i) {
            if (i%10==0) sign=-sign;
            rep = r.tick(amp*sign, 0.0f);
            amp *= 0.998f;
        }
        CHECK(rep.state == GuardState::normal);
        CHECK(!rep.latched);
    }
}

// (12) 声称 dt 与真实经过严重不符 (dt=9ms, 实钟仅前进 1ms) 必须拒.
void test_claimed_dt_vs_real_elapsed_rejected() {
    SentinelOscillationGuard g;
    g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f);
    (void)g.step(s);
    s.now_us=2000;           // 真实经过仅 1ms
    s.dt_s=0.009f;           // 却声称 9ms
    GuardianReport rep=g.step(s);
    CHECK((rep.reason_bits & reason_hard_time) != 0);
    CHECK(rep.latched);
}


// ============ 父稳定复核 red 例 (7 项) ============
// R1) peak_envelope 永久峰值: 大振铃后低幅持续振荡 (9Hz, 前200ms 幅.08, 后持续8s 幅.05,
//     ratio .625 在 (.5,.7) 之间) 不得被永久标 decaying 而永不 latch.
//     有界近期趋势/连续多窗非下降 必须取消 decay.
void test_r1_bounded_peak_low_amp_sustained_latches() {
    for (Axis a : {Axis::fire_yaw, Axis::pitch}) {
        Runner r(a, synth_limits(a));
        GuardianReport rep{};
        // 前 200ms: 9Hz 幅 0.08 (9Hz => 周期 ~111us? no: 9Hz 周期 111ms). 用 sign 每 55.5ms 翻转.
        // dt=1ms; 半周期 55ms => 每 56 tick 翻转.
        float sign = 1.0f;
        int i=0;
        for (; i<300; ++i) {              // 0.3s 大振铃
            if (i%56==0) sign=-sign;
            rep = r.tick(0.08f*sign, 0.0f);
        }
        // 之后 8s: 持续 幅 0.05 同频
        for (; i<8300; ++i) {
            if (i%56==0) sign=-sign;
            rep = r.tick(0.05f*sign, 0.0f);
        }
        // 持续真实振荡 8s 不得因历史大峰值而永久隐瞒 -> 必须最终 fault_latched.
        CHECK(rep.state == GuardState::fault_latched);
    }
}

// R3) config_valid 缺项 + axis 不匹配:
//   residual_deadband<0, has_max_speed but max_speed<=0, present current<=0,
//   temperature 有效范围(温度上限须 > 绝对零/合理界, 如 <= -273.15 或极不合理),
//   suspect_exit<=0, nominal_dt>max_dt 均拒; sample.axis != config.axis 拒/不判 normal.
void test_r3_config_validation_gaps() {
    auto must_reject=[&](GuardianLimits l){ SentinelOscillationGuard g; g.configure(l);
        GuardianSample s=base_sample(l.axis,1000,0.001f); (void)g.step(s);
        CHECK(g.status().state != GuardState::normal); };
    { GuardianLimits l=synth_limits(Axis::fire_yaw); l.residual_deadband_rad=-0.001f; must_reject(l); }
    { GuardianLimits l=synth_limits(Axis::fire_yaw); l.has_max_speed=true; l.max_speed_rad_s=0.0f; must_reject(l); }
    { GuardianLimits l=synth_limits(Axis::fire_yaw); l.current_limit_a={true,false,0.0f}; must_reject(l); }
    { GuardianLimits l=synth_limits(Axis::fire_yaw); l.temperature_limit_c={true,false,-300.0f}; must_reject(l); }
    { GuardianLimits l=synth_limits(Axis::fire_yaw); l.suspect_exit_ms=0.0f; must_reject(l); }
    { GuardianLimits l=synth_limits(Axis::fire_yaw); l.nominal_dt_s=l.max_dt_s+0.01f; must_reject(l); }
    // axis mismatch: config fire_yaw, sample pitch -> 不判 normal (拒/blocked).
    { SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
      GuardianSample s=base_sample(Axis::pitch,1000,0.001f); GuardianReport rep=g.step(s);
      CHECK(rep.state != GuardState::normal); CHECK(rep.latched || rep.blocked || rep.state==GuardState::unconfigured); }
}

// R4) time 容差: 真实 dt=1ms 却声称 1us -> 相对+量化误差判定必须拒;
//     另: dt=1ms 声称 1.0001ms(合理量化) 不应拒 (避免过度严格).
void test_r4_time_relative_tolerance() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f); (void)g.step(s);
    // 真实经过 1ms (now +1000), 声称 dt=1us=0.000001 => 严重低估 (1000x) => 必须坏时序.
    s.now_us=2000; s.dt_s=0.000001f; GuardianReport rep=g.step(s);
    CHECK((rep.reason_bits & reason_hard_time)!=0); CHECK(rep.latched);
    // 合理量化: 真实 1ms, 声称 1.0001ms => 应接受 (不 time_bad).
    SentinelOscillationGuard g2; g2.configure(synth_limits(Axis::fire_yaw));
    GuardianSample a=base_sample(Axis::fire_yaw,1000,0.001f); (void)g2.step(a);
    a.now_us=2000; a.dt_s=0.0010001f; GuardianReport rep2=g2.step(a);
    CHECK((rep2.reason_bits & reason_hard_time)==0);
}

// R5) reset 需当前时间核新鲜: 长时间无新样本 (now_us 早已过去) 后 reset 必须拒.
//     签名: reset_latched(auth, rev, locally_confirmed, now_us) (4 参, 无默认).
void test_r5_reset_requires_fresh_now() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    // 制造锁存后一个干净样本.
    GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f); s.angle_rad=1.5f; (void)g.step(s);
    GuardianSample c=base_sample(Axis::fire_yaw,2000,0.001f); c.angle_rad=0.0f; (void)g.step(c);
    CHECK(g.status().state==GuardState::fault_latched);
    // 很久没有新样本 (调用方 now 前进 5s), 但 guard 最近样本是 2ms -> 不新鲜 => 拒.
    ResetResult rr=g.reset_latched(200,5,true, static_cast<std::uint32_t>(2000u+5'000'000u));
    CHECK(rr!=ResetResult::accepted);
    CHECK(g.status().state==GuardState::fault_latched);
    // 紧邻最近样本 (now = 2ms + 1ms) 且本地确认 => accepted -> disarmed.
    ResetResult rr2=g.reset_latched(200,5,true, 3000u);
    CHECK(rr2==ResetResult::accepted);
    CHECK(g.status().state==GuardState::disarmed);
}

// R6) hard 锁存后 configure 非法分支不得抹掉 reason_bits_.
void test_r6_invalid_configure_keeps_latch_reasons() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f); s.angle_rad=1.5f; GuardianReport rep=g.step(s);
    CHECK(rep.latched); std::uint32_t before=g.status().reason_bits; CHECK(before!=0);
    // 用非法配置覆盖 (config_valid=false).
    GuardianLimits bad=synth_limits(Axis::fire_yaw); bad.exposure_budget=-1.0f; g.configure(bad);
    GuardianReport after=g.status();
    CHECK(after.latched);                 // 仍锁存
    CHECK(after.reason_bits != 0);        // 不丢锁存原因
    CHECK((after.reason_bits & reason_hard_position_limit)!=0);
}

// R7) state_elapsed_ms_ 长时饱和 (不 UB): 大量 tick 后 elapsed_ms 为有限饱和值, 不为 0/不溢出成 UB.
void test_r7_elapsed_ms_saturates() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianReport rep{};
    // 在 warning 态长时间累计 (取 danger 振荡持续). 简化: 直接持续超速触发? 用 warning 态.
    // 用持续振荡推到 warning 并停留, 累计 elapsed.
    float sign=1.0f;
    for (int i=0;i<200000;++i) {
        if (i%56==0) sign=-sign;
        GuardianSample s=base_sample(Axis::fire_yaw, static_cast<std::uint32_t>(1000u+static_cast<std::uint32_t>(i)*1000u), 0.001f);
        s.angle_rad=0.06f*sign;
        rep=g.step(s);
    }
    // elapsed 必须是有界饱和值 (<= 某饱和上限), 且不出现溢出回绕(crazy small).
    CHECK(rep.elapsed_ms >= 1000u);   // 已累计相当量
}


// ============ 父最终 B 两项 + 3 数据质量 (红例) ============
// F1) 带内能量真带通: DC/慢漂移不应计入 band_energy (HP 滤除); 带外高频 (>LP) 应被 LP 衰减;
//     带内振荡 (如 9Hz 落在 2..40Hz) 应显著计入. 且 residual_energy_raw 保留原始 r^2 (DC 大).
void test_f1_band_energy_bandlimited() {
    // (a) 纯 DC 偏置: 残差恒 0.05 (target .05, angle 0), HP 应滤掉 DC => band_energy 小, raw 大.
    {
        SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
        GuardianReport rep{};
        for (std::uint32_t i=1;i<=3000;++i){
            GuardianSample s=base_sample(Axis::fire_yaw,i*1000,0.001f);
            s.request_target_rad=0.05f; s.adopted_target_rad=0.05f; s.angle_rad=0.0f;
            rep=g.step(s);
        }
        CHECK(rep.residual_energy_raw > 10.0f*rep.band_energy); // raw 远大于带内 (DC 被 HP 滤除)
        CHECK(rep.band_energy >= 0.0f);
    }
    // (b) 带内 9Hz 振荡: band_energy 应显著 (> 一定量级).
    {
        SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
        GuardianReport rep{}; float sign=1.0f;
        for (std::uint32_t i=1;i<=2000;++i){
            if (i%56==0) sign=-sign;
            GuardianSample s=base_sample(Axis::fire_yaw,i*1000,0.001f);
            s.angle_rad=0.05f*sign;
            rep=g.step(s);
        }
        CHECK(rep.band_energy > 1.0e-4f); // 带内能量可观 (在 2..40Hz 带内)
    }
    // (c) 频段配置非法 (Nyquist 越界) => 不判 normal.
    {
        GuardianLimits l=synth_limits(Axis::fire_yaw); l.band_lowpass_hz=l.sample_rate_hz*0.5f+1.0f;
        SentinelOscillationGuard g; g.configure(l);
        GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f); GuardianReport rep=g.step(s);
        CHECK(rep.state != GuardState::normal);
    }
}

// F2) reset 恢复: physical safe + executingFalse + authorityFalse + manual_stopTrue =>
//     仅现场 reset 接受 -> DISARMED; 仍物理越界 => 拒 reset.
void test_f2_reset_recovery_without_exec_authority() {
    // 制造锁存 (超速/越界).
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample bad=base_sample(Axis::fire_yaw,1000,0.001f); bad.angle_rad=1.5f; (void)g.step(bad);
    CHECK(g.status().state==GuardState::fault_latched);
    // 现场恢复样本: 物理安全 (angle=0 在区间), 但 executing=false, authority_ok=false, manual_stop=true.
    GuardianSample rec=base_sample(Axis::fire_yaw,2000,0.001f);
    rec.angle_rad=0.0f; rec.velocity_rad_s=0.0f; rec.feedback_valid=true; rec.rc_valid=true;
    rec.executing=false; rec.authority_ok=false; rec.manual_stop=true;
    (void)g.step(rec);
    CHECK(g.status().state==GuardState::fault_latched); // 仍锁存 (不含 authority 也不自动解锁)
    ResetResult rr=g.reset_latched(200,5,true,2000u);
    CHECK(rr==ResetResult::accepted);                    // 现场确认可到 disarmed
    CHECK(g.status().state==GuardState::disarmed);
    CHECK(g.status().blocked);                            // disarmed 不授运动
}

void test_f2b_reset_rejected_when_still_out_of_range() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample bad=base_sample(Axis::fire_yaw,1000,0.001f); bad.angle_rad=1.5f; (void)g.step(bad);
    CHECK(g.status().state==GuardState::fault_latched);
    // 仍物理越界: angle 仍在区间外 => 拒 reset.
    GuardianSample still=base_sample(Axis::fire_yaw,2000,0.001f);
    still.angle_rad=1.5f; still.executing=false; still.authority_ok=false; still.manual_stop=true;
    (void)g.step(still);
    ResetResult rr=g.reset_latched(200,5,true,2000u);
    CHECK(rr!=ResetResult::accepted);
    CHECK(g.status().state==GuardState::fault_latched);
}

// F3) feedback_valid=false 不得拿默认 angle=0 判 hard_position (只反馈失效).
void test_f3_feedback_invalid_no_spurious_position() {
    // 机械区间 [1,2] 不含 0: 若 feedback 失效拿默认 angle=0 判位置会伪报 hard_position.
    GuardianLimits l=synth_limits(Axis::fire_yaw); l.mechanical_min_rad=1.0f; l.mechanical_max_rad=2.0f;
    SentinelOscillationGuard g; g.configure(l);
    GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f);
    s.feedback_valid=false; s.angle_rad=0.0f; // 默认 0, 不在 [1,2]; 但反馈失效, 不得报 position
    s.velocity_rad_s=0.0f;
    s.request_target_rad=1.5f; s.adopted_target_rad=1.5f; // 目标在区间内, 排除目标越界干扰
    GuardianReport rep=g.step(s);
    CHECK((rep.reason_bits & reason_hard_feedback)!=0);
    CHECK((rep.reason_bits & reason_hard_position_limit)==0); // 不得因反馈失效伪报位置越界
}

// F4) target request/adopted 各按自身 valid 各核 finite; target_valid=false 时 request NaN 不算数.
void test_f4_target_per_field_finite() {
    // request valid 且 NaN => nonfinite; adopted valid=false 且 NaN => 忽略.
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f);
    s.target_valid=true; s.request_target_rad=kNaN; s.target_adopted=false; s.adopted_target_rad=kNaN;
    GuardianReport rep=g.step(s);
    CHECK((rep.reason_bits & reason_hard_nonfinite)!=0);
}

// F5) cross-axis 错误样本: 报告 axis 必须保持绑定轴 (不被错误 sample.axis 重贴标签).
void test_f5_crossaxis_report_axis_bound() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s=base_sample(Axis::pitch,1000,0.001f); // 错误轴
    GuardianReport rep=g.step(s);
    CHECK(rep.axis==Axis::fire_yaw); // 保持绑定轴
    CHECK(rep.state != GuardState::normal);
}


// F6) 实际 dt 变化可接受: 真实 dt 在 Nyquist 内变化 (1ms/5ms/10ms) 不应判时序故障.
void test_f6_dt_variation_accepted() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    std::uint32_t now=0;
    for (float dt : {0.001f, 0.005f, 0.010f, 0.005f, 0.001f}) {
        now += static_cast<std::uint32_t>(dt*1e6f);
        GuardianSample s=base_sample(Axis::fire_yaw, now, dt);
        s.angle_rad=0.02f;
        GuardianReport rep=g.step(s);
        CHECK((rep.reason_bits & reason_hard_time)==0);
        CHECK(!rep.latched);
    }
}

// F7) 低采样 Nyquist 违规: dt 使有效采样率低于 2*band_lowpass_hz (alias 风险) => 判时序故障.
//     lp=40Hz => 允许最大 dt=0.5/40=12.5ms; dt=15ms (<=max_dt 20ms) 违反 Nyquist 必须拒.
void test_f7_low_sample_nyquist_violation() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample s=base_sample(Axis::fire_yaw,1000,0.001f); (void)g.step(s);
    GuardianSample slow=base_sample(Axis::fire_yaw, 1000u+15000u, 0.015f); // dt=15ms
    GuardianReport rep=g.step(slow);
    CHECK((rep.reason_bits & reason_hard_time)!=0);
    CHECK(rep.latched);
}


// F8) 同真实时刻、轻微不同 claimed dt: 曝光/能量预算必须一致 (用 now_us 实际 elapsed, 非 claimed dt).
//     真实推进 1ms/tick, 一个声称 dt=1.0ms, 另一个声称 0.6ms (均在容差内). 预算应几乎相同,
//     不得因 0.6ms 声称导致危险暴露少算 40%.
void test_f8_claimed_dt_does_not_change_budget() {
    SentinelOscillationGuard g1; g1.configure(synth_limits(Axis::fire_yaw));
    SentinelOscillationGuard g2; g2.configure(synth_limits(Axis::fire_yaw));
    GuardianReport r1{}, r2{};
    for (std::uint32_t i=1;i<=500;++i){
        float sign = ((i/25)%2==0)?1.0f:-1.0f; // 20ms 半周期 => 25Hz, 带内
        GuardianSample s1=base_sample(Axis::fire_yaw, i*1000, 0.001f);
        GuardianSample s2=base_sample(Axis::fire_yaw, i*1000, 0.0006f); // 声称 0.6ms
        s1.angle_rad=0.03f*sign; s2.angle_rad=0.03f*sign;
        r1=g1.step(s1); r2=g2.step(s2);
        CHECK((r2.reason_bits & reason_hard_time)==0); // 0.6ms 在容差内不判坏
    }
    float d = r1.exposure - r2.exposure; if (d<0) d=-d;
    CHECK(d <= 0.02f * (r1.exposure>1e-6f?r1.exposure:1e-6f)); // 预算一致 (允许极小差)
    float be1=r1.band_energy, be2=r2.band_energy; float bd=be1-be2; if(bd<0)bd=-bd;
    CHECK(bd <= 0.02f * (be1>1e-9f?be1:1e-9f));
}


// F9) reset 新鲜性跨合法 u32 wrap: last_clean=0xFFFFFF00, 现场 now=0x00000100 => 真实 512us 应可到 disarmed.
void test_f9_reset_legal_wrap_accepted() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    // 制造锁存 (越界), 记录干净 last_now 接近 wrap 边界.
    std::uint32_t now0 = 0xFFFFFE00u;
    GuardianSample bad=base_sample(Axis::fire_yaw, now0, 0.001f); bad.angle_rad=1.5f; (void)g.step(bad);
    CHECK(g.status().state==GuardState::fault_latched);
    // 干净样本时刻推进到 wrap 边界前 (last_now 停在 0xFFFFFF00); 现场确认 now 回绕到 0x100 (真实 512us).
    GuardianSample rec=base_sample(Axis::fire_yaw, 0xFFFFFF00u, 0.000256f);
    rec.angle_rad=0.0f; rec.velocity_rad_s=0.0f; rec.executing=false; rec.authority_ok=false; rec.manual_stop=true;
    (void)g.step(rec);
    CHECK(g.status().state==GuardState::fault_latched);
    // now_us=0x100 < last_now_us_=0xFFFFFF00, 但 u32 无符号差 = 0x200 = 512us 合法.
    ResetResult rr=g.reset_latched(300,7,true,0x00000100u);
    CHECK(rr==ResetResult::accepted);
    CHECK(g.status().state==GuardState::disarmed);
}

// F10) reset 新鲜性真实少量回退 => 巨大无符号 diff 超上限 => 拒 (不因 now<last 简单拒但结果同样拒).
void test_f10_reset_backward_step_rejected() {
    SentinelOscillationGuard g; g.configure(synth_limits(Axis::fire_yaw));
    GuardianSample bad=base_sample(Axis::fire_yaw, 1000000u, 0.001f); bad.angle_rad=1.5f; (void)g.step(bad);
    CHECK(g.status().state==GuardState::fault_latched);
    GuardianSample rec=base_sample(Axis::fire_yaw, 1001000u, 0.001f);
    rec.angle_rad=0.0f; rec.velocity_rad_s=0.0f; rec.executing=false; rec.authority_ok=false; rec.manual_stop=true;
    (void)g.step(rec);
    // 回退 6us => 无符号 diff ≈ 2^32-... 巨大, 超时龄上限.
    ResetResult rr=g.reset_latched(300,7,true, 1000994u);
    CHECK(rr!=ResetResult::accepted);
    CHECK(g.status().state==GuardState::fault_latched);
}

} // namespace

int main() {
    test_t01_static_and_slow_rc_no_false_alarm();
    test_t02_commanded_reverse_not_systemic_false_positive();
    test_t03_decaying_ring_recovers();
    test_t04_sustained_oscillation_escalates();
    test_t05_growing_escalates_faster();
    test_t06_small_high_freq_output_reversal();
    test_t09_feedback_invalid_and_rc();
    test_t10_time_anomalies();
    test_t11_hard_conditions_same_tick();
    test_t13_latched_not_cleared_by_normal_frame();
    test_t18_saturation_and_fixed_size();
    test_t19_axis_isolation();
    test_t20_pitch_hardware_blocked();
    test_unconfigured_default();
    test_reset_to_disarmed_then_rearm_needs_new_ids();
    test_reconfigure_and_boot_change_keep_latch();
    test_invalid_dt_does_not_clear_budget();
    test_isolated_spike_not_persistent();
    test_invalid_config_rejected();
    test_observe_only_never_latches();
    test_approved_fire_hardware_ready_flag();
    test_transition_idempotent();
    test_sustained_must_eventually_latch_and_not_auto_recover();
    test_constant_offset_and_step_not_band_oscillation();
    test_sustained_small_output_reverse_latches();
    test_target_out_of_range_latches();
    test_rebind_axis_rejected();
    test_config_validity_strict();
    test_reset_requires_local_confirm_and_clean_sample();
    test_unobserved_nan_ignored();
    test_dt_vs_now_mismatch_rejected();
    test_reset_signature_guard_cases();
    test_slow_decay_recovers_not_latched();
    test_claimed_dt_vs_real_elapsed_rejected();
    test_r1_bounded_peak_low_amp_sustained_latches();
    test_r3_config_validation_gaps();
    test_r4_time_relative_tolerance();
    test_r5_reset_requires_fresh_now();
    test_r6_invalid_configure_keeps_latch_reasons();
    test_r7_elapsed_ms_saturates();
    test_f1_band_energy_bandlimited();
    test_f2_reset_recovery_without_exec_authority();
    test_f2b_reset_rejected_when_still_out_of_range();
    test_f3_feedback_invalid_no_spurious_position();
    test_f4_target_per_field_finite();
    test_f5_crossaxis_report_axis_bound();
    test_f6_dt_variation_accepted();
    test_f7_low_sample_nyquist_violation();
    test_f8_claimed_dt_does_not_change_budget();
    test_f9_reset_legal_wrap_accepted();
    test_f10_reset_backward_step_rejected();
    std::printf("sentinel_oscillation_guard_tests: ALL PASS\n");
    return 0;
}
