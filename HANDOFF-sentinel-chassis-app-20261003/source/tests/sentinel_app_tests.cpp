// O0 哨兵 APP 配置边界契约:
//  - 合成测试夹具与真实部署配置是两种显式类别, 不得静默混用;
//  - 配置校验按单位/范围/域身份唯一性拒绝无效项;
//  - 未配置/配置缺项/尚未收到反馈或建立参考, 都要形成有明确原因的快照,
//    不得伪造反馈值、参考有效性或执行授权.
// 这里只验证配置与快照语义, 不代表任何实车装配、参数或动作已完成.
#include "sentinel_app.hpp"
#include "sentinel_config.hpp"
#include "sentinel_runtime.hpp"
#include "../boards/sentinel_f407/include/board_app_config.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3);
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::Tick;
using ec::drivers::DjiMode;
using ec::sentinel::app::ConfigAudience;
using ec::sentinel::app::ConfigIssue;
using ec::sentinel::app::ConfigKind;
using ec::sentinel::app::ConfigState;
using ec::sentinel::app::Domain;
using ec::sentinel::app::DomainValidity;
using ec::sentinel::app::Reason;
using ec::sentinel::app::SentinelApp;
using ec::sentinel::app::SentinelConfig;
using ec::sentinel::app::Source;

// 合成夹具: 结构完整、来源显式为 synthetic, 且四轮几何可正逆解.
void single_gm6020_bench_contracts() {
    using namespace ec::sentinel::app;
    const auto vehicle = ec::sentinel::board::deployment_config();
    CHECK(ec::sentinel::board::summarize(vehicle).installed_dji == 0);
    const auto bench = ec::sentinel::board::single_gm6020_bench_config(vehicle);
    const auto report = validate(bench, ConfigAudience::deployment);
    CHECK(report.startable);
    CHECK(report.domains[domain_index(Domain::main_yaw)] == ConfigState::complete);
    CHECK(ec::sentinel::board::summarize(bench).installed_dji == 1);
    CHECK(bench.dji[4].device_id == 4 && bench.dji[4].bus == 1);
    CHECK(std::fabs(bench.dji[4].velocity_pid->kp - 0.024f) < 1e-6f);
    CHECK(std::fabs(bench.dji[4].velocity_pid->ki - 0.4f) < 1e-6f);
    CHECK(bench.joints[0].manual_continuous && !bench.joints[0].min_rad && !bench.joints[0].max_rad);
    auto missing = bench; missing.joints[0].manual_continuous = false;
    CHECK(validate(missing, ConfigAudience::deployment).domains[domain_index(Domain::main_yaw)] == ConfigState::incomplete);
    auto contradictory = bench; contradictory.joints[0].min_rad = 0.0f;
    CHECK(validate(contradictory, ConfigAudience::deployment).domains[domain_index(Domain::main_yaw)] == ConfigState::incomplete);
    CHECK(!vehicle.joints[0].manual_continuous && !vehicle.joints[1].manual_continuous && !vehicle.joints[2].manual_continuous);
    SentinelAuthority normal;
    SentinelAuthority maintenance{StartupPolicy::maintenance_only};
    LinkInputs input{};
    input.locally_ready = true;
    input.maintenance_link_alive = true;
    input.maintenance_session_epoch = 1;
    CHECK(!maintenance.update(input).token);
    input.maintenance_request = true;
    CHECK(!normal.update(input).token);
    CHECK(maintenance.update(input).source == Source::maintenance);
    input.maintenance_request = false;
    CHECK(!maintenance.update(input).token);
    input.remote_online = true;
    input.switch_left = ec::drivers::Dr16Switch::up;
    input.switch_right = ec::drivers::Dr16Switch::mid;
    CHECK(!maintenance.update(input).token);
    input.switch_left = ec::drivers::Dr16Switch::down;
    input.host_link_alive = true; input.host_request = true;
    CHECK(!maintenance.update(input).token);
}

void synthetic_fixture_contracts() {
    const auto config = ec::sentinel::app::synthetic_config();
    const auto report = ec::sentinel::app::validate(config, ConfigAudience::offline_test);
    CHECK(report.kind == ConfigKind::synthetic_fixture);
    CHECK(report.startable);
    CHECK(report.complete);
    CHECK(report.issue_count == 0);
    for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
        CHECK(report.domains[i] == ConfigState::complete);
    }
    // 关节外的域不需要关节几何; 四轮几何必须真的可解, 不能只是字段非空.
    const auto drive = ec::sentinel::app::omni_drive(config);
    CHECK(drive.has_value());
    CHECK(ec::motion::omni_well_posed(*drive));
    for (const auto& wheel : config.wheels) {
        CHECK(wheel.radius_m.has_value() && *wheel.radius_m > 0.0f);
    }
    // 九 DJI + 一 DM 都能转换成设备配置; 抽查单位与量纲, 不抽取具体数值当实车参数.
    for (std::size_t i = 0; i < ec::sentinel::app::dji_slot_count; ++i) {
        const auto device = ec::sentinel::app::device_config(config.dji[i]);
        CHECK(device.has_value());
        CHECK(device->rotor_per_axis > 0.0f);
        CHECK(device->direction == 1 || device->direction == -1);
        CHECK(device->output_limit_raw > 0);
        CHECK(device->feedback_ttl.valid());
    }
    const auto dm = ec::sentinel::app::device_config(config.dm[0]);
    CHECK(dm.has_value());
    CHECK(dm->can_id == *config.dm[0].can_id);
    CHECK(dm->limits.position_max_rad > 0.0f);

    SentinelApp app{config, ConfigAudience::offline_test};
    CHECK(app.startable());
    CHECK(app.kind() == ConfigKind::synthetic_fixture);
    CHECK(app.dji_slot_for(Domain::main_yaw).has_value());
    CHECK(app.dm_slot_for(Domain::pitch).has_value());
    CHECK(!app.dji_slot_for(Domain::pitch).has_value());

    // 配置齐备但尚无设备/反馈: 状态必须是 unknown + 明确原因, 不是 valid/健康.
    const auto status = app.status(Tick{10});
    CHECK(status.at.ms == 10);
    CHECK(status.permission.source == Source::none);
    CHECK(!status.permission.authority);
    CHECK(!status.permission.manual_stop);
    CHECK(!status.permission.locally_ready);
    CHECK(status.reference_epoch == 0);
    for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        const auto& entry = status.domain(domain);
        CHECK(entry.domain == domain);
        CHECK(entry.configured);
        CHECK(entry.validity == DomainValidity::unknown);
        CHECK(entry.reason == (ec::sentinel::app::is_joint(domain) ? Reason::no_reference
                                                                   : Reason::no_feedback));
        CHECK(!entry.target.has_value());
        CHECK(!entry.output.has_value());
        CHECK(!entry.enabled);
        CHECK(entry.feedback.position_rad.validity == ec::Validity::unknown);
        CHECK(entry.feedback.temperature_c == std::nullopt);
    }
}

// 真实配置入口: 可以启动(用于观察/通信), 但没有域取得执行资格, 且不填猜测值.
void real_placeholder_contracts() {
    const auto config = ec::sentinel::app::real_config_placeholder();
    CHECK(config.kind == ConfigKind::real_deployment);
    const auto report = ec::sentinel::app::validate(config, ConfigAudience::deployment);
    CHECK(report.startable);
    CHECK(!report.complete);
    CHECK(report.has(ConfigIssue::missing_timing));
    for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
        CHECK(report.domains[i] == ConfigState::unavailable);
    }
    // 未被装配的设备不产生"来源未知"错误: 空槽不是填了猜测值.
    CHECK(!report.has(ConfigIssue::unknown_origin_in_deployment));
    CHECK(!report.has(ConfigIssue::synthetic_value_in_deployment));

    SentinelApp app{config, ConfigAudience::deployment};
    CHECK(app.startable());
    const auto status = app.status(Tick{5});
    for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
        const auto& entry = status.domains[i];
        CHECK(!entry.configured);
        CHECK(entry.validity == DomainValidity::unavailable);
        CHECK(entry.reason == Reason::not_configured);
    }
}

// 合成与真实配置不得静默混用.
void mixing_rejected_contracts() {
    // 1. 合成夹具本身: 离线测试接受, 部署用途拒绝.
    auto synthetic = ec::sentinel::app::synthetic_config();
    CHECK(ec::sentinel::app::validate(synthetic, ConfigAudience::offline_test).startable);
    const auto as_deploy = ec::sentinel::app::validate(synthetic, ConfigAudience::deployment);
    CHECK(!as_deploy.startable);
    CHECK(as_deploy.has(ConfigIssue::synthetic_value_in_deployment));
    SentinelApp app{synthetic, ConfigAudience::deployment};
    CHECK(!app.startable());
    // 配置对象本身没有被校验过程修改.
    CHECK(synthetic.kind == ConfigKind::synthetic_fixture);

    // 2. 声明为部署配置但保留合成值: 必须被识别为混用.
    synthetic.kind = ConfigKind::real_deployment;
    const auto mixed = ec::sentinel::app::validate(synthetic, ConfigAudience::deployment);
    CHECK(!mixed.startable);
    CHECK(mixed.has(ConfigIssue::synthetic_value_in_deployment));

    // 3. 声明为部署配置且设备已装配, 但来源未标注: 同样不作为可部署配置.
    auto real = ec::sentinel::app::real_config_placeholder();
    real.dji[0].installed = true;
    real.dji[0].domain = Domain::chassis_lf;
    const auto unknown_origin = ec::sentinel::app::validate(real, ConfigAudience::deployment);
    CHECK(!unknown_origin.startable);
    CHECK(unknown_origin.has(ConfigIssue::unknown_origin_in_deployment));

    // 4. 完全没有配置类别: 两种用途都不能启动.
    const SentinelConfig blank{};
    CHECK(!ec::sentinel::app::validate(blank, ConfigAudience::offline_test).startable);
    CHECK(ec::sentinel::app::validate(blank, ConfigAudience::offline_test)
              .has(ConfigIssue::unconfigured_kind));
}

// 校验按单位/范围/域身份拒绝无效配置.
void validation_contracts() {
    const auto base = ec::sentinel::app::synthetic_config();
    const auto check_issue = [](const SentinelConfig& config, ConfigIssue issue, const char* what) {
        const auto report = ec::sentinel::app::validate(config, ConfigAudience::offline_test);
        if (!report.has(issue)) {
            std::fprintf(stderr, "expected issue %d (%s)\n", static_cast<int>(issue), what);
            std::_Exit(3);
        }
    };

    { // 域身份唯一性
        auto config = base;
        config.dji[1].domain = config.dji[0].domain;
        check_issue(config, ConfigIssue::domain_duplicated, "duplicate domain");
    }
    { // 方向只允许 ±1; 缺失与非法分开
        auto config = base;
        config.dji[0].direction = 0;
        check_issue(config, ConfigIssue::invalid_direction, "direction 0");
        config.dji[0].direction.reset();
        check_issue(config, ConfigIssue::missing_direction, "direction missing");
    }
    { // 输出编码上限不得超过协议满量程
        auto config = base;
        config.dji[0].output_limit_raw = 20000; // C620 满量程 16384
        check_issue(config, ConfigIssue::invalid_output_limit, "output limit too large");
    }
    { // 轴侧电流上限不得超过协议满量程 (C620 = 20 A)
        auto config = base;
        config.dji[0].current_limit_a = 25.0f;
        check_issue(config, ConfigIssue::invalid_current_limit, "current limit too large");
    }
    { // 传动比必须为正有限
        auto config = base;
        config.dji[0].rotor_per_axis = 0.0f;
        check_issue(config, ConfigIssue::invalid_rotor_ratio, "rotor ratio 0");
        config.dji[0].rotor_per_axis = std::nanf("");
        check_issue(config, ConfigIssue::invalid_rotor_ratio, "rotor ratio nan");
        config.dji[0].rotor_per_axis.reset();
        check_issue(config, ConfigIssue::missing_rotor_ratio, "rotor ratio missing");
    }
    { // 电压模式没有电流上限语义
        auto config = base;
        config.dji[0].mode = DjiMode::gm6020_voltage;
        config.dji[0].device_id = 1;
        check_issue(config, ConfigIssue::unsupported_mode, "gm6020_voltage");
    }
    { // 环路增益必须齐备且量纲合法
        auto config = base;
        config.dji[0].velocity_pid.reset();
        check_issue(config, ConfigIssue::missing_gains, "velocity gains missing");
        auto bad = base;
        bad.dji[4].position_pid = ec::control::PidConfig{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f};
        check_issue(bad, ConfigIssue::invalid_gains, "gains output range");
    }
    { // 关节范围必须 min < max, 零位必须提供
        auto config = base;
        config.joints[1].min_rad = 2.0f;
        config.joints[1].max_rad = 1.0f;
        check_issue(config, ConfigIssue::invalid_joint_range, "joint range inverted");
        auto missing = base;
        missing.joints[2].zero_offset_rad.reset();
        check_issue(missing, ConfigIssue::missing_joint_reference, "zero offset missing");
    }
    { // 轮几何: 缺项与非法值分开; 退化布局单独报告
        auto config = base;
        config.wheels[2].radius_m = 0.0f;
        check_issue(config, ConfigIssue::invalid_wheel_geometry, "wheel radius 0");
        auto missing = base;
        missing.wheels[3].y_m.reset();
        check_issue(missing, ConfigIssue::missing_wheel_geometry, "wheel y missing");
        auto degenerate = base;
        for (auto& wheel : degenerate.wheels) { wheel.roll_dir_rad = 0.0f; }
        check_issue(degenerate, ConfigIssue::degenerate_wheel_layout, "degenerate layout");
    }
    { // DM 量程没有通用默认值, MIT 增益有协议范围
        auto config = base;
        config.dm[0].limits.reset();
        check_issue(config, ConfigIssue::missing_dm_limits, "dm limits missing");
        auto invalid = base;
        invalid.dm[0].limits = ec::drivers::DmMitLimits{0.0f, 30.0f, 10.0f};
        check_issue(invalid, ConfigIssue::invalid_dm_limits, "dm pmax 0");
        auto gains = base;
        gains.dm[0].mit_kp = 600.0f; // 协议上限 500
        check_issue(gains, ConfigIssue::invalid_dm_gains, "dm kp out of range");
        auto missing_gain = base;
        missing_gain.dm[0].mit_kd.reset();
        check_issue(missing_gain, ConfigIssue::missing_dm_gains, "dm kd missing");
    }
    { // 时序: 控制周期与各有效期必须显式
        auto config = base;
        config.timing.control_dt_s = 0.0f;
        check_issue(config, ConfigIssue::missing_timing, "control dt 0");
        auto ttl = base;
        ttl.timing.host_link_ttl = ec::Lifetime{0};
        check_issue(ttl, ConfigIssue::missing_timing, "host link ttl 0");
        auto dt = base;
        dt.timing.max_dt_s = 0.0001f; // 小于控制周期
        check_issue(dt, ConfigIssue::invalid_timing, "max dt < control dt");
    }
    { // 未装配设备不因缺少换算参数而报缺项 (未装配 ≠ 参数未知)
        auto config = base;
        config.dji[8].installed = false;
        const auto report = ec::sentinel::app::validate(config, ConfigAudience::offline_test);
        CHECK(!report.has(ConfigIssue::missing_rotor_ratio));
        CHECK(report.domains[ec::sentinel::app::domain_index(Domain::trigger)] ==
            ConfigState::unavailable);
        CHECK(!report.complete);
    }
}

// 每个域恰好由一个设备槽认领, 映射可从应用对象查回.
void domain_identity_contracts() {
    SentinelApp app{ec::sentinel::app::synthetic_config(), ConfigAudience::offline_test};
    for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        const auto dji = app.dji_slot_for(domain);
        const auto dm = app.dm_slot_for(domain);
        CHECK(dji.has_value() != dm.has_value());
        if (dji) {
            CHECK(app.config().dji[*dji].domain == domain);
            CHECK(app.config().dji[*dji].installed);
        }
        if (dm) {
            CHECK(app.config().dm[*dm].domain == domain);
            CHECK(app.config().dm[*dm].installed);
        }
    }
    // pitch 是唯一的 DM 域; 两 yaw 都在 DJI 侧.
    CHECK(app.dji_slot_for(Domain::fire_yaw).has_value());
    CHECK(app.dm_slot_for(Domain::pitch).has_value());
    CHECK(!app.dm_slot_for(Domain::main_yaw).has_value());
}
// 微秒与毫秒独立, 到期/回绕/错相/强制停止均按真实计算间隔解释.
void runtime_contracts() {
    using namespace ec::sentinel::app;
    CHECK(confirmed_friction_rotor_per_axis == 1.0f);
    CHECK(origin_name(Origin::user_confirmed) == std::string_view{"user_confirmed"});
    CHECK(ec::micro_fresh(ec::MicroTick{500}, ec::MicroTick{0xfffffff0u}, ec::MicroLifetime{1000}));
    CHECK(!ec::micro_fresh(ec::MicroTick{1000}, ec::MicroTick{0}, ec::MicroLifetime{1000}));
    CHECK(ec::micro_regressed(ec::MicroTick{99}, ec::MicroTick{100}));
    CHECK(YawFrame::joint != YawFrame::local_inertial && YawFrame::local_inertial != YawFrame::odom);
    const auto timing = RuntimeTiming{};
    CHECK(timing.gm.valid() && timing.chassis.valid() && timing.shooter.valid());
    GroupClock clock;
    CHECK(!clock.poll({10000}, timing.shooter).due);
    auto step = clock.poll({11000}, timing.shooter);
    CHECK(step.due && !step.invalid_time && std::fabs(step.dt_s - 0.002f) < 1e-7f);
    CHECK(!clock.poll({12000}, timing.shooter).due);
    step = clock.poll({12000}, timing.shooter, true);
    CHECK(step.due && std::fabs(step.dt_s - 0.001f) < 1e-7f);
    CHECK(!clock.poll({13000}, timing.shooter).due);
    CHECK(clock.poll({14000}, timing.shooter).due);
    CHECK(clock.poll({13000}, timing.shooter).invalid_time);
    CHECK(clock.poll({25000}, timing.shooter).invalid_time);
    CHECK(!clock.poll({25000}, timing.shooter).due);
    for (const std::uint32_t period : {1000u, 2000u, 5000u}) {
        GroupClock periodic;
        GroupTiming policy{period, 0, {period + 1000}, period * 3};
        unsigned calls = 0;
        for (std::uint32_t ms = 0; ms < 100; ++ms) {
            const auto result = periodic.poll({ms * 1000 + (ms % 3) * 40}, policy,
                false, ec::MicroTick{ms * 1000});
            if (result.due) { ++calls; CHECK(!result.invalid_time); }
        }
        CHECK(calls == 100000u / period);
    }
    GroupClock wrap;
    CHECK(wrap.poll({0xfffffc18u}, timing.chassis).due);
    step = wrap.poll({1000}, timing.chassis);
    CHECK(step.due && !step.invalid_time && std::fabs(step.dt_s - 0.002f) < 1e-7f);
}
} // 测试辅助

void four_motor_bench_contracts() {
    const auto config = ec::sentinel::board::four_motor_bench_config(ec::sentinel::board::deployment_config());
    const auto report = validate(config, ConfigAudience::deployment);
    CHECK(report.startable && config.independent_motor_bench);
    CHECK(ec::sentinel::board::summarize(config).installed_dji == 3);
    CHECK(ec::sentinel::board::summarize(config).installed_dm == 1);
    CHECK(config.dji[4].bus == 1 && config.dji[4].device_id == 4);
    CHECK(config.dji[6].bus == 1 && config.dji[6].device_id == 2);
    CHECK(config.dji[8].bus == 1 && config.dji[8].device_id == 3);
    CHECK(config.dm[0].bus == 2 && config.dm[0].can_id == 1 && config.dm[0].master_id == 0x011);
    auto mode_conflict = config; mode_conflict.dji[8].device_id = 7;
    const auto rejected = validate(mode_conflict, ConfigAudience::deployment);
    CHECK(!rejected.startable && rejected.has(ConfigIssue::dji_command_mode_conflict));
    mode_conflict.dji[8].bus = 2;
    CHECK(validate(mode_conflict, ConfigAudience::deployment).startable);
    for (auto domain : {Domain::main_yaw, Domain::pitch, Domain::friction_left, Domain::trigger}) {
        CHECK(report.domains[domain_index(domain)] == ConfigState::complete);
    }
    auto alias = config; alias.dji[6].bus = 2; alias.dji[6].device_id = 1;
    const auto bad = validate(alias, ConfigAudience::deployment);
    CHECK(!bad.startable && bad.has(ConfigIssue::dm_feedback_id_alias));
    alias.dji[6].bus = 1;
    const auto fixed = validate(alias, ConfigAudience::deployment);
    CHECK(fixed.startable && !fixed.has(ConfigIssue::dm_feedback_id_alias));
}

int main() {
    four_motor_bench_contracts();
    single_gm6020_bench_contracts();
    runtime_contracts();
    synthetic_fixture_contracts();
    real_placeholder_contracts();
    mixing_rejected_contracts();
    validation_contracts();
    domain_identity_contracts();
    std::puts("sentinel app configuration contracts passed");
    return 0;
}
