// O3 感知/反馈重构与时钟契约:
//  - 样本时刻/摄入时刻混淆不得静默通过: 乱序/重叠/回退/过期/间隔过大分别被拒绝;
//  - 链路正常时允许必要外推, 并显式标记外推与样本年龄, 超时则降级为 stale;
//  - 静态噪声、倾斜静态与旋转一致性下姿态/角速度/重力向误差有界;
//  - 主 yaw odom 对齐与参考代次, 惯性失效按 G07 编码器降级保持;
//  - 发射许可缺失/过期时正向拨弹不取得执行资格, 人工反转不受影响;
//  - 轮式反馈里程计与命令预测分离; 异步轮速、回绕、秩和缺口均可观测.
#include "sentinel_execution.hpp"
#include "sentinel_perception.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <algorithm>

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
using ec::sentinel::app::DomainValidity;
using ec::sentinel::app::Epochs;
using ec::sentinel::app::FaultInputs;
using ec::sentinel::app::Reason;
using ec::sentinel::app::SentinelApp;
using ec::sentinel::app::SentinelPerception;
using ec::sentinel::app::Source;
using ec::sentinel::app::TargetSnapshot;
using ec::control::ControlMode;
using ec::drivers::Dr16Switch;

constexpr float gravity = 9.81f;
constexpr std::uint32_t index(Domain domain) { return ec::sentinel::app::domain_index(domain); }
const std::array<float, 3> level_accel{0.0f, 0.0f, gravity};
const std::array<float, 3> zero_gyro{0.0f, 0.0f, 0.0f};

struct Percept {
    SentinelPerception perception{ec::sentinel::app::synthetic_config(),
        ConfigAudience::offline_test};
    // 以 1 ms 周期连续投递同一测量, 便于验证收敛与积分.
    void feed(std::uint32_t start_ms, std::uint32_t count,
        const std::array<float, 3>& gyro, const std::array<float, 3>& accel) {
        for (std::uint32_t i = 0; i < count; ++i) {
            const Tick sampled{start_ms + i};
            perception.submit_imu(gyro, accel, sampled, sampled);
        }
    }
};

void expired_micro_cache_contracts() {
    const auto config = ec::sentinel::app::synthetic_config();
    SentinelPerception p{config, ConfigAudience::offline_test};
    ec::drivers::ImuSamplingTimes times{};
    times.gyro_valid = times.accel_valid = times.gyro_updated = times.accel_updated = true;
    times.gyro_sequence = times.accel_sequence = 1;
    times.gyro = times.gyro_estimated = times.accel = {1000000};
    p.submit_timed_imu(zero_gyro, level_accel, times, {1000}, {1000000});
    const auto joints = [&](Tick at, ec::MicroTick us) {
        ec::sentinel::app::DomainFeedback value{};
        value.position_rad = {0, at, ec::Validity::valid}; value.velocity_rad_s = {0, at, ec::Validity::valid};
        p.submit_joint(Domain::fire_yaw, value, us, 1);
        p.submit_joint(Domain::pitch, value, us, 1);
    };
    joints({1000}, {1000000}); p.update({1000}, ec::MicroTick{1000000});
    CHECK(p.platform().validity == DomainValidity::valid);
    const auto epoch = p.local_epoch();
    for (const std::uint64_t elapsed : {30000ull, 0x80000010ull, 0x100000000ull}) {
        p.update({1000+static_cast<std::uint32_t>(elapsed/1000)},
            ec::MicroTick{1000000+static_cast<std::uint32_t>(elapsed)});
        CHECK(p.inertial().validity != DomainValidity::valid);
        CHECK(p.platform().validity != DomainValidity::valid);
        CHECK(p.local_epoch() == epoch); // 无新数据仅失效, 不循环重建参考.
    }
    const Tick later{1000+4294967u};
    // 微秒数字恰好重复, 新序号仍是新数据; 但旧 accel 不能充当本次重建所需的重力观测.
    times.gyro_sequence = 2; times.accel_updated = false;
    p.submit_timed_imu(zero_gyro, level_accel, times, later, {1000000});
    CHECK(p.local_epoch() == epoch);
    times.accel_sequence = 2; times.accel_updated = true;
    p.submit_timed_imu(zero_gyro, level_accel, times, later, {1000000});
    p.update(later, ec::MicroTick{1000000});
    CHECK(p.inertial().validity == DomainValidity::valid);
    CHECK(p.local_epoch() == epoch+1);
    CHECK(p.platform().validity != DomainValidity::valid); // fire/pitch 旧历史也不能绕一圈复活.
    joints(later, {1000000}); p.update(later, ec::MicroTick{1000000});
    CHECK(p.platform().validity == DomainValidity::valid);
    // 跨半圈的真正新样本允许重新建立时间轴, 而非永久当成乱序拒绝.
    times.gyro_sequence = times.accel_sequence = 3;
    times.gyro = times.gyro_estimated = times.accel = {0x80000010u+1000000u};
    const Tick half_later{later.ms+2147484u};
    p.submit_timed_imu(zero_gyro, level_accel, times, half_later, times.gyro);
    p.update(half_later, times.gyro);
    CHECK(p.inertial().validity == DomainValidity::valid);
    CHECK(p.local_epoch() == epoch+2);
}

void imu_time_contracts() {
    Percept p;
    // 首个样本建立初始姿态; 摄入时刻与样本时刻相同 → 不是外推.
    p.perception.submit_imu(zero_gyro, level_accel, Tick{100}, Tick{100});
    p.perception.update(Tick{100});
    CHECK(p.perception.inertial().validity == DomainValidity::valid);
    CHECK(!p.perception.inertial().extrapolated);
    CHECK(p.perception.inertial().age_ms == 0);
    CHECK(p.perception.inertial().sampled_at.ms == 100);
    CHECK(p.perception.inertial().ingested_at.ms == 100);

    // 同一采样时刻重复帧不算新数据.
    p.perception.submit_imu(zero_gyro, level_accel, Tick{100}, Tick{101});
    CHECK(p.perception.stats().rejected_duplicate == 1);
    // 采样时刻倒退的乱序帧.
    p.perception.submit_imu(zero_gyro, level_accel, Tick{99}, Tick{102});
    CHECK(p.perception.stats().rejected_regression == 1);
    // 摄入时已经超过可用时限的样本.
    p.perception.submit_imu(zero_gyro, level_accel, Tick{103}, Tick{200});
    CHECK(p.perception.stats().rejected_too_old == 1);
    // 非有限值.
    p.perception.submit_imu(std::array<float, 3>{std::nanf(""), 0.0f, 0.0f}, level_accel, Tick{103},
        Tick{103});
    CHECK(p.perception.stats().rejected_invalid == 1);
    // 采样间隔超过 max_dt: 不把它当作积分步长, 而是重建时间轴 —— 否则旧时间基准会一直落后,
    // 后续每个样本都被判为长间隔而永远拒绝 (没有任何恢复入口).
    p.perception.set_yaw_alignment(0.0f, 1);
    p.perception.submit_imu(zero_gyro, level_accel, Tick{103 + 100}, Tick{103 + 100});
    CHECK(p.perception.stats().rejected_long_gap == 1);
    CHECK(p.perception.stats().reprime_events == 1);
    CHECK(p.perception.stats().accepted == 2);
    // 重建后旧对齐偏移不再成立 (它属于旧姿态参考), 但时间轴重新可用.
    p.perception.update(Tick{103 + 101});
    CHECK(!p.perception.inertial().reference_valid);

    // 采样间隔抖动: 正常间隔被接受, 快照年龄按样本时刻计算.
    p.perception.submit_imu(zero_gyro, level_accel, Tick{103 + 101}, Tick{103 + 101});
    p.perception.submit_imu(zero_gyro, level_accel, Tick{103 + 103}, Tick{103 + 103});
    p.perception.update(Tick{103 + 106});
    CHECK(p.perception.inertial().validity == DomainValidity::valid);
    CHECK(p.perception.inertial().extrapolated);
    CHECK(p.perception.inertial().age_ms == 3);
    // 超过 imu_ttl 的快照必须降级, 不再外推.
    p.perception.update(Tick{103 + 103 + 50});
    CHECK(p.perception.inertial().validity == DomainValidity::stale);
    CHECK(p.perception.inertial().reason == Reason::stale_feedback);
    // 从未收到过样本时不得伪造反馈.
    Percept empty;
    empty.perception.update(Tick{10});
    CHECK(empty.perception.inertial().validity == DomainValidity::unknown);
    CHECK(empty.perception.inertial().reason == Reason::no_feedback);
}

void attitude_contracts() {
    // 静态水平: 姿态与重力向收敛到零, 角速度为零.
    Percept level;
    level.feed(100, 400, zero_gyro, level_accel);
    level.perception.update(Tick{499});
    const auto level_rpy = level.perception.inertial().roll_pitch_yaw_rad.value;
    CHECK(std::fabs(level_rpy[0]) < 0.02f);
    CHECK(std::fabs(level_rpy[1]) < 0.02f);
    const auto level_gravity = level.perception.inertial().gravity_direction_body.value;
    CHECK(std::fabs(level_gravity[2] - 1.0f) < 0.02f);

    // 倾斜静态: 加速度计观测到重力在机体系中的方向, roll/pitch 必须跟随.
    const float roll = 0.2f;
    const float pitch = -0.3f;
    // 倾斜测量 = R^T (0,0,g), 即重力在机体系中的方向.
    const auto rot = ec::estimation::rotation_zyx(roll, pitch, 0.0f);
    const auto accel_tilt = ec::estimation::apply_rotation(
        ec::estimation::RotationMatrix3{std::array<float, 9>{rot.m[0], rot.m[3], rot.m[6], rot.m[1],
            rot.m[4], rot.m[7], rot.m[2], rot.m[5], rot.m[8]}},
        level_accel);
    Percept tilted;
    tilted.feed(100, 600, zero_gyro, accel_tilt);
    tilted.perception.update(Tick{699});
    const auto tilt_rpy = tilted.perception.inertial().roll_pitch_yaw_rad.value;
    CHECK(std::fabs(tilt_rpy[0] - roll) < 0.03f);
    CHECK(std::fabs(tilt_rpy[1] - pitch) < 0.03f);

    // 旋转一致性: 恒角速度绕 Z 旋转时 yaw 按积分推进, roll/pitch 不被带偏.
    Percept spinning;
    const std::array<float, 3> wz_gyro{0.0f, 0.0f, 1.0f};
    spinning.feed(100, 200, wz_gyro, level_accel);
    spinning.perception.update(Tick{299});
    const auto spin_rpy = spinning.perception.inertial().roll_pitch_yaw_rad.value;
    CHECK(std::fabs(spin_rpy[2] - 0.2f) < 0.03f);
    CHECK(std::fabs(spin_rpy[0]) < 0.03f);
    CHECK(std::fabs(spin_rpy[1]) < 0.03f);
}

void alignment_contracts() {
    Percept p;
    p.feed(100, 200, zero_gyro, level_accel);
    for (const auto domain : {Domain::fire_yaw, Domain::pitch}) {
        ec::sentinel::app::DomainFeedback feedback;
        feedback.position_rad = {0, {299}, ec::Validity::valid};
        feedback.velocity_rad_s = {0, {299}, ec::Validity::valid};
        p.perception.submit_joint(domain, feedback, {299000}, 0);
    }
    p.perception.update(Tick{299});
    CHECK(!p.perception.alignment_valid());
    CHECK(!p.perception.yaw_direction_rad().has_value());
    p.perception.set_yaw_alignment(1.0f, 1);
    CHECK(p.perception.alignment_valid());
    const auto direction = p.perception.yaw_direction_rad();
    CHECK(direction.has_value());
    CHECK(std::fabs(*direction - 1.0f) < 0.05f);
    CHECK(p.perception.alignment_epoch() == 1);
    // 代次变化 = 对齐被替换, 必须计入可核对的变化次数.
    p.perception.set_yaw_alignment(1.5f, 2);
    CHECK(p.perception.stats().alignment_changes == 2);
    CHECK(p.perception.alignment_epoch() == 2);
    CHECK(std::fabs(*p.perception.yaw_direction_rad() - 1.5f) < 0.05f);
    // 惯性失效后不再提供 odom 方向.
    p.perception.update(Tick{299 + 200});
    CHECK(!p.perception.yaw_direction_rad().has_value());
}

void wheel_odom_contracts() {
    using namespace ec::sentinel::app;
    auto config = synthetic_config();
    SentinelPerception p{config, ConfigAudience::offline_test};
    std::array<DomainFeedback, 4> feedback{};
    std::array<std::optional<ec::MicroTick>, 4> times{};
    const ec::motion::PlanarTwist truth{0.4f, -0.1f, 0.7f};
    const auto feed = [&](std::uint32_t ms, std::uint32_t us) {
        for (std::size_t i=0; i<4; ++i) {
            const auto& g = config.wheels[i];
            // 独立正向几何公式; 不调用被测正/逆解, 不提供任何转子位置参考.
            const float speed = (std::cos(*g.roll_dir_rad)*(truth.vx_mps-*g.y_m*truth.wz_rad_s)
                + std::sin(*g.roll_dir_rad)*(truth.vy_mps+*g.x_m*truth.wz_rad_s))* *g.gain / *g.radius_m;
            feedback[i].velocity_rad_s = {speed, {ms-static_cast<std::uint32_t>(i)}, ec::Validity::valid};
            times[i] = ec::MicroTick{us-static_cast<std::uint32_t>(i)*1000u};
        }
        p.update_wheel_odom(feedback, times, {ms}, {us});
    };
    for (std::uint32_t i=0; i<=1000; ++i) { feed(100+i, 100000+i*1000u); }
    const auto& wheel = p.wheel_odom();
    CHECK(wheel.validity == DomainValidity::valid && wheel.pose.valid && wheel.continuous);
    CHECK(wheel.integrated_us == 1000000 && wheel.skew_us == 3000);
    CHECK(std::fabs(wheel.twist.vx_mps-truth.vx_mps) < 1e-5f);
    CHECK(std::fabs(wheel.twist.vy_mps-truth.vy_mps) < 1e-5f);
    CHECK(std::fabs(wheel.twist.wz_rad_s-truth.wz_rad_s) < 1e-5f);
    CHECK(wheel.residual_rad_s < 1e-5f);
    // 恒定 body twist 的闭式 SE(2) 解.
    const float a = std::sin(truth.wz_rad_s)/truth.wz_rad_s;
    const float b = (1.0f-std::cos(truth.wz_rad_s))/truth.wz_rad_s;
    CHECK(std::fabs(wheel.pose.x_m-(a*truth.vx_mps-b*truth.vy_mps)) < 2e-5f);
    CHECK(std::fabs(wheel.pose.y_m-(b*truth.vx_mps+a*truth.vy_mps)) < 2e-5f);
    CHECK(std::fabs(wheel.pose.yaw_rad-truth.wz_rad_s) < 2e-5f);
    CHECK(!p.commanded_odom().valid); // 没有命令, 仍能由实际轮速累计位姿.
    const auto updates = wheel.updates;
    p.update_wheel_odom(feedback, times, {1101}, {1101000});
    CHECK(wheel.updates == updates && wheel.integrated_us == 1000000);
    const auto before = wheel.pose.x_m;
    feedback[2].velocity_rad_s.validity = ec::Validity::invalid;
    p.update_wheel_odom(feedback, times, {1102}, {1102000});
    p.update_wheel_odom(feedback, times, {1103}, {1103000});
    CHECK(wheel.validity == DomainValidity::invalid && !wheel.pose.valid);
    CHECK(wheel.gaps == 1 && !wheel.continuous && wheel.pose.x_m == before);
    feed(1200, 1200000); // 恢复首样本不跨未知区间积分, 也不重置已有位姿.
    CHECK(wheel.pose.x_m == before && wheel.integrated_us == 1000000);
    feed(1201, 1201000);
    CHECK(wheel.integrated_us == 1001000 && wheel.pose.valid && !wheel.continuous);
    const auto after = wheel.pose.x_m;
    feed(1300, 1300000); // 没有调用过失效分支的长缺口也不可补积分.
    CHECK(wheel.gaps == 2 && wheel.pose.x_m == after);
    feed(1301, 1301000);
    feedback[0].velocity_rad_s.value += 0.5f;
    for (auto& t : times) { t->us += 1000; }
    p.update_wheel_odom(feedback, times, {1302}, {1302000});
    CHECK(wheel.residual_rad_s > 0.01f && wheel.validity == DomainValidity::valid);
    // 正常微秒回绕不得成为断点.
    SentinelPerception wrapped{config, ConfigAudience::offline_test};
    for (std::uint32_t i=0; i<=2; ++i) {
        for (std::size_t j=0; j<4; ++j) {
            times[j] = ec::MicroTick{0xfffffff0u+i*1000u};
            feedback[j].velocity_rad_s.sampled_at = {2000+i};
        }
        wrapped.update_wheel_odom(feedback, times, {2000+i}, {0xfffffff0u+i*1000u});
    }
    CHECK(wrapped.wheel_odom().integrated_us == 2000 && wrapped.wheel_odom().gaps == 0);
    for (auto& g : config.wheels) { g.roll_dir_rad = 0.0f; }
    SentinelPerception singular{config, ConfigAudience::offline_test};
    singular.update_wheel_odom(feedback, times, {2002}, {1984});
    CHECK(!singular.wheel_odom().rank_ok && !singular.wheel_odom().pose.valid);
}

// NR05/T09: 已明确中断的区间不保留判序基线, 包括半圈与完整微秒回绕.
void wheel_long_gap_contracts() {
    using namespace ec::sentinel::app;
    for (const bool observed_gap : {false,true})
    for (const std::uint64_t gap_us : {100000ull, 0x80001000ull, 0x100000000ull}) {
        const auto config=synthetic_config();
        SentinelPerception p{config, ConfigAudience::offline_test};
        std::array<DomainFeedback,4> feedback{};
        std::array<std::optional<ec::MicroTick>,4> times{};
        const auto feed=[&](std::uint64_t us) {
            const Tick ms{static_cast<std::uint32_t>(us/1000)};
            for (std::size_t i=0;i<4;++i) {
                const auto& g=config.wheels[i];
                feedback[i].velocity_rad_s={std::cos(*g.roll_dir_rad)*0.4f* *g.gain / *g.radius_m,
                    ms,ec::Validity::valid};
                times[i]=ec::MicroTick{static_cast<std::uint32_t>(us)};
            }
            p.update_wheel_odom(feedback,times,ms,{static_cast<std::uint32_t>(us)});
        };
        feed(100000); feed(101000);
        const auto before=p.wheel_odom();
        if (observed_gap) {
            feedback[0].velocity_rad_s.validity=ec::Validity::invalid;
            p.update_wheel_odom(feedback,times,{102},{102000});
        }
        const auto resumed=101000+gap_us;
        feed(resumed);
        const auto& current=p.wheel_odom();
        CHECK(current.validity==DomainValidity::valid && current.pose.valid);
        CHECK(current.updates==before.updates+1 && current.gaps==1 && !current.continuous);
        CHECK(current.pose.x_m==before.pose.x_m && current.pose.y_m==before.pose.y_m);
        CHECK(current.pose.yaw_rad==before.pose.yaw_rad && current.integrated_us==before.integrated_us);
        feed(resumed+1000);
        CHECK(current.integrated_us==before.integrated_us+1000);
        CHECK(current.pose.x_m>before.pose.x_m && current.gaps==1 && !current.continuous);
    }
}

void odom_contracts() {
    Percept p;
    const ec::motion::PlanarTwist twist{1.0f, 0.0f, 0.0f};
    p.perception.integrate_commanded_odom(twist, Tick{1000});
    CHECK(p.perception.commanded_odom().valid);
    CHECK(p.perception.commanded_odom().integrated_ms == 0);
    p.perception.integrate_commanded_odom(twist, Tick{1010});
    CHECK(std::fabs(p.perception.commanded_odom().x_m - 0.01f) < 1e-5f);
    CHECK(p.perception.commanded_odom().integrated_ms == 10);
    // 绕 Z 旋转后再直行: 机体 X 已经指向参考系 Y.
    p.perception.integrate_commanded_odom(ec::motion::PlanarTwist{0.0f, 0.0f, 1.0f}, Tick{1020});
    CHECK(std::fabs(p.perception.commanded_odom().yaw_rad - 0.01f) < 1e-5f);
    p.perception.integrate_commanded_odom(twist, Tick{1030});
    // 航向 0.01 rad 时机体 X 直行 10 ms: x 再增约 0.01, y 增约 1e-4.
    CHECK(std::fabs(p.perception.commanded_odom().x_m - 0.02f) < 1e-4f);
    CHECK(p.perception.commanded_odom().y_m > 5e-5f);
    CHECK(p.perception.commanded_odom().y_m < 2e-4f);
    // 时间倒退与间隔过大都不积分.
    const auto x_before = p.perception.commanded_odom().x_m;
    p.perception.integrate_commanded_odom(twist, Tick{1020});
    CHECK(p.perception.commanded_odom().rejected_steps == 1);
    CHECK(p.perception.commanded_odom().x_m == x_before);
    p.perception.integrate_commanded_odom(twist, Tick{1020 + 100});
    CHECK(p.perception.commanded_odom().rejected_steps == 2);
    CHECK(p.perception.commanded_odom().x_m == x_before);
    // 零 twist 不改变位姿.
    p.perception.integrate_commanded_odom(ec::motion::PlanarTwist{}, Tick{1200});
    CHECK(p.perception.commanded_odom().x_m == x_before);
}

// 应用层: 发射许可门禁 + 主 yaw 惯性降级保持.
struct Rig {
    SentinelApp app{ec::sentinel::app::synthetic_config(), ConfigAudience::offline_test};
    AppInputs inputs{};
    Tick now{1};
    Rig() { inputs.link.locally_ready = true; }
    void step(std::uint32_t ms) {
        now = Tick{ms};
        inputs.epochs = Epochs{1, 0};
        app.update(now, inputs);
    }
    void go_local(std::uint32_t ms) {
        inputs.link.remote_online = true;
        inputs.link.switch_left = Dr16Switch::up;
        inputs.link.switch_right = Dr16Switch::mid;
        step(ms);
    }
    [[nodiscard]] const ec::sentinel::app::DomainExecution& exec(Domain domain) const {
        return app.snapshot().executions[index(domain)];
    }
};

TargetSnapshot packet(std::initializer_list<std::pair<Domain, DomainSetpoint>> items) {
    TargetSnapshot out{};
    for (const auto& [domain, setpoint] : items) {
        out.domains[index(domain)] = setpoint;
        // 本文件的主 yaw 目标专用于惯性/G07 契约, 不借用默认的机械维护模式.
        if (domain == Domain::main_yaw) {
            out.domains[index(domain)]->frame = ec::sentinel::app::YawFrame::local_inertial;
        }
    }
    return out;
}

void launch_permit_contracts() {
    Rig rig;
    rig.go_local(10);
    CHECK(rig.app.accept_targets(
              packet({{Domain::trigger, DomainSetpoint{ControlMode::velocity, 3.0f}}}), Tick{10}) ==
        AcceptResult::accepted);
    // 没有许可: 正向拨弹不取得执行资格, 但这不是设备故障.
    rig.step(11);
    CHECK(!rig.app.snapshot().launch_allowed);
    CHECK(rig.exec(Domain::trigger).reason == Reason::not_permitted);
    CHECK(!rig.exec(Domain::trigger).execute);
    CHECK(rig.app.snapshot().domains[index(Domain::trigger)].fault == ec::sentinel::app::FaultState::none);
    // 许可有效且新鲜 → 执行.
    rig.inputs.aux.launch_permit_valid = true;
    rig.inputs.aux.launch_permit = true;
    rig.inputs.aux.launch_sampled_at = Tick{12};
    rig.inputs.aux.launch_ttl = ec::Lifetime{50};
    rig.step(12);
    CHECK(rig.app.snapshot().launch_allowed);
    CHECK(rig.exec(Domain::trigger).execute);
    // 许可过期 → 再次不取得资格 (目标同时刷新, 避免与目标时效混淆).
    CHECK(rig.app.accept_targets(
              packet({{Domain::trigger, DomainSetpoint{ControlMode::velocity, 3.0f}}}), Tick{70}) ==
        AcceptResult::accepted);
    rig.step(70);
    CHECK(!rig.app.snapshot().launch_allowed);
    CHECK(rig.exec(Domain::trigger).reason == Reason::not_permitted);
    // 人工反转不受发射许可限制 (只禁正向).
    CHECK(rig.app.accept_targets(
              packet({{Domain::trigger, DomainSetpoint{ControlMode::velocity, -3.0f}}}), Tick{71}) ==
        AcceptResult::accepted);
    rig.step(71);
    CHECK(!rig.app.snapshot().launch_allowed);
    CHECK(rig.exec(Domain::trigger).execute);
    // 功率/热量只作为可注入观测上报, 不据此新增门禁.
    rig.inputs.aux.power_limited = true;
    rig.inputs.aux.heat_limited = true;
    rig.step(72);
    CHECK(rig.app.snapshot().aux.power_limited);
    CHECK(rig.exec(Domain::trigger).execute);
}

void degraded_hold_contracts() {
    Rig rig;
    rig.go_local(10);
    rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
        ec::Sample<float>{0.3f, rig.now, ec::Validity::valid};
    CHECK(rig.app.accept_targets(
              packet({{Domain::main_yaw, DomainSetpoint{ControlMode::position, 0.5f}}}), Tick{10}) ==
        AcceptResult::accepted);
    rig.step(11);
    CHECK(rig.exec(Domain::main_yaw).execute);
    CHECK(rig.exec(Domain::main_yaw).adopted_target == 0.5f);
    // 仅惯性失效: 撤销惯性方向目标, 捕获当前关节角并保持.
    rig.inputs.faults.main_yaw_inertial_only = true;
    rig.step(12);
    CHECK(rig.app.main_yaw_degraded());
    CHECK(!rig.app.target(Domain::main_yaw).has_value());
    CHECK(rig.exec(Domain::main_yaw).execute);
    CHECK(rig.exec(Domain::main_yaw).mode == ControlMode::position);
    CHECK(rig.exec(Domain::main_yaw).adopted_target == 0.3f);
    CHECK(rig.exec(Domain::main_yaw).reason == Reason::degraded);
    // 降级期间不接受新的惯性方向目标.
    CHECK(rig.app.accept_targets(
              packet({{Domain::main_yaw, DomainSetpoint{ControlMode::position, 0.4f}}}), Tick{13}) ==
        AcceptResult::degraded);
    // 其它域不受影响.
    CHECK(rig.app.accept_targets(
              packet({{Domain::friction_left, DomainSetpoint{ControlMode::velocity, 1.0f}}}), Tick{13}) ==
        AcceptResult::accepted);
    // 人工停止仍覆盖降级保持.
    rig.inputs.link.switch_right = Dr16Switch::down;
    rig.step(14);
    CHECK(rig.app.snapshot().permission.manual_stop);
    CHECK(rig.exec(Domain::main_yaw).reason == Reason::manual_stop);
    CHECK(!rig.exec(Domain::main_yaw).execute);
    rig.inputs.link.switch_right = Dr16Switch::mid;
    // 惯性恢复: 撤销降级保持, 旧惯性方向目标不复活.
    rig.inputs.faults.main_yaw_inertial_only = false;
    rig.step(15);
    CHECK(!rig.app.main_yaw_degraded());
    CHECK(!rig.app.target(Domain::main_yaw).has_value());
    CHECK(rig.exec(Domain::main_yaw).reason == Reason::no_target);
}
// F05: 降级保持属于某一域的意图: 该域被撤销 (缺省域/设备重启) 时必须一起失效,
// 且期限按回绕安全比较, 不用绝对截止毫秒值.
void degraded_hold_invalidation_contracts() {
    // (a) 完整快照省略 main_yaw: 其它域仍在 → 接受, 但保持必须随目标一起撤掉.
    {
        Rig rig;
        rig.go_local(10);
        rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
            ec::Sample<float>{0.3f, Tick{10}, ec::Validity::valid};
        CHECK(rig.app.accept_targets(packet({{Domain::main_yaw,
                  DomainSetpoint{ControlMode::position, 0.5f}}}), Tick{10}) ==
            AcceptResult::accepted);
        rig.step(11);
        rig.inputs.faults.main_yaw_inertial_only = true;
        rig.step(12);
        CHECK(rig.exec(Domain::main_yaw).execute);
        CHECK(rig.app.accept_targets(packet({{Domain::friction_left,
                  DomainSetpoint{ControlMode::velocity, 1.0f}}}), Tick{12}) ==
            AcceptResult::accepted);
        rig.step(13);
        CHECK(rig.app.main_yaw_degraded());
        CHECK(!rig.exec(Domain::main_yaw).execute);
    }
    // (b) 设备重启: 降级保持属于旧意图, 必须一起失效且不复活.
    {
        Rig rig;
        rig.go_local(10);
        rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
            ec::Sample<float>{0.3f, Tick{10}, ec::Validity::valid};
        CHECK(rig.app.accept_targets(packet({{Domain::main_yaw,
                  DomainSetpoint{ControlMode::position, 0.5f}}}), Tick{10}) ==
            AcceptResult::accepted);
        rig.step(11);
        rig.inputs.faults.main_yaw_inertial_only = true;
        rig.step(12);
        CHECK(rig.exec(Domain::main_yaw).execute);
        rig.inputs.faults.device_restarted[index(Domain::main_yaw)] = true;
        rig.step(13);
        CHECK(!rig.exec(Domain::main_yaw).execute);
        rig.inputs.faults.device_restarted[index(Domain::main_yaw)] = false;
        rig.step(14);
        CHECK(!rig.exec(Domain::main_yaw).execute);
    }
    // (c) 回绕安全期限: 原意图在回绕后仍按真实年龄判定过期 (NT09 给定值).
    {
        Rig rig;
        rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
            ec::Sample<float>{0.3f, Tick{0xffffffe8u}, ec::Validity::valid};
        rig.go_local(0xffffffe8u);
        CHECK(rig.app.accept_targets(packet({{Domain::main_yaw,
                  DomainSetpoint{ControlMode::position, 0.5f}}}), Tick{0xffffffe8u}) ==
            AcceptResult::accepted);
        rig.step(0xffffffe9u);
        CHECK(rig.exec(Domain::main_yaw).execute);
        rig.inputs.faults.main_yaw_inertial_only = true;
        rig.step(0xffffffeau);
        CHECK(rig.exec(Domain::main_yaw).execute);
        // 跨回绕的未到期窗口: target_ttl=20, 从 0xffffffe8 到 0xfffffff5 只过了 13ms.
        rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
            ec::Sample<float>{0.3f, Tick{0xfffffff5u}, ec::Validity::valid};
        rig.step(0xfffffff5u);
        CHECK(rig.exec(Domain::main_yaw).execute);
        // 到达到期时刻 (0xfffffffc = accepted+20): 不再执行, 回绕后也不复活.
        rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
            ec::Sample<float>{0.3f, Tick{0xfffffffcU}, ec::Validity::valid};
        rig.step(0xfffffffcU);
        CHECK(!rig.exec(Domain::main_yaw).execute);
        rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
            ec::Sample<float>{0.3f, Tick{0u}, ec::Validity::valid};
        rig.step(0u);
        CHECK(!rig.exec(Domain::main_yaw).execute);
        rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
            ec::Sample<float>{0.3f, Tick{5u}, ec::Validity::valid};
        rig.step(5u);
        CHECK(!rig.exec(Domain::main_yaw).execute);
    }
}
// A04: 源年龄已达时限的 main 字段等价于"没有意图": 必须取消降级保持;
// 未到期的新的惯性方向目标仍按降级策略拒绝.
void source_expired_main_cancels_degraded_contracts() {
    Rig rig;
    rig.go_local(10);
    rig.inputs.feedback[index(Domain::main_yaw)].position_rad =
        ec::Sample<float>{0.3f, Tick{10}, ec::Validity::valid};
    CHECK(rig.app.accept_targets(packet({{Domain::main_yaw,
              DomainSetpoint{ControlMode::position, 0.5f}}}), Tick{10}) ==
        AcceptResult::accepted);
    rig.step(11);
    CHECK(rig.exec(Domain::main_yaw).execute);
    CHECK(rig.exec(Domain::main_yaw).adopted_target == 0.5f);
    // 仅惯性失效 → 降级保持当前机械角.
    rig.inputs.faults.main_yaw_inertial_only = true;
    rig.step(12);
    CHECK(rig.exec(Domain::main_yaw).execute);
    CHECK(rig.exec(Domain::main_yaw).adopted_target == 0.3f);
    // 未到期的 main 新方向: 仍按降级拒绝, 既有保持不变.
    CHECK(rig.app.accept_targets(packet({{Domain::main_yaw,
              DomainSetpoint{ControlMode::position, 0.4f, 0}}}), Tick{12}) ==
        AcceptResult::degraded);
    rig.step(12);
    CHECK(rig.exec(Domain::main_yaw).adopted_target == 0.3f);
    // 源年龄已达 target_ttl 的 main 字段: 没有意图 → 统一域撤销 (取消保持).
    const auto ttl = static_cast<std::uint16_t>(rig.app.target_ttl().ms);
    CHECK(rig.app.accept_targets(packet({{Domain::main_yaw,
              DomainSetpoint{ControlMode::position, 0.5f, ttl}}}), Tick{13}) ==
        AcceptResult::accepted);
    rig.step(13);
    CHECK(!rig.exec(Domain::main_yaw).execute);
    // 无关合法域仍按整包规则照常处理.
    CHECK(rig.app.accept_targets(packet({{Domain::friction_left,
              DomainSetpoint{ControlMode::velocity, 1.0f, 0}}}), Tick{13}) ==
        AcceptResult::accepted);
    rig.step(14);
    CHECK(rig.exec(Domain::friction_left).execute);
}
} // 测试辅助

void independent_imu_step_contract() {
    auto config = ec::sentinel::app::synthetic_config();
    config.timing.max_dt_s = 0.001f;
    config.perception.max_gyro_step_us = 20000;
    SentinelPerception perception{config, ConfigAudience::offline_test};
    perception.submit_imu(zero_gyro, level_accel, {100}, {100});
    perception.submit_imu(zero_gyro, level_accel, {105}, {105});
    CHECK(perception.stats().accepted == 2);
    CHECK(perception.stats().reprime_events == 0);
    perception.update({126}); // 在下一新样本到达前执行更新, 输入延迟不能冒充 sample 长间隔.
    perception.submit_imu(zero_gyro, level_accel, {125}, {126});
    CHECK(perception.stats().reprime_events == 0); // 独立积分上限的精确边界, 加1 ms摄入延迟
    perception.submit_imu(zero_gyro, level_accel, {146}, {146});
    CHECK(perception.stats().reprime_events == 1);
}

int main() {
    expired_micro_cache_contracts();
    independent_imu_step_contract();
    imu_time_contracts();
    attitude_contracts();
    alignment_contracts();
    odom_contracts();
    wheel_odom_contracts();
    wheel_long_gap_contracts();
    launch_permit_contracts();
    degraded_hold_contracts();
    degraded_hold_invalidation_contracts();
    source_expired_main_cancels_degraded_contracts();
    std::puts("sentinel app perception contracts passed");
    return 0;
}
