// 正式 APP 部署配置的离线负向检查 (O6).
// 直接使用板级头文件里的**同一份**部署配置表: 固件与测试因此不会分叉.
// 断言的是"没有实车参数时不给执行资格"和"镜像里不含臆测值", 不是实机行为.
#include "board_app_config.hpp"
#include "board_can_queue.hpp"
#include "board_spi_transfer.hpp"
#include "board_assembly.hpp"
#include "board_role.hpp"
#include "sentinel_app.hpp"
#include "sentinel_types.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace {
int failures = 0;
void check(bool condition, const char* text, int line) {
    if (condition) { return; }
    std::printf("line %d: %s\n", line, text);
    ++failures;
}
#define CHECK(expr) check((expr), #expr, __LINE__)

using namespace ec::sentinel;
using namespace ec::sentinel::app;

void test_deployment_config_is_not_invented() {
    const auto config = board::deployment_config();
    CHECK(config.kind == ConfigKind::real_deployment);
    // 没有实测依据时不得声明任何装配槽 (声明了就等于把未确认的机械事实写进固件).
    for (const auto& slot : config.dji) {
        CHECK(!slot.installed);
        CHECK(slot.origin == Origin::unspecified);
        CHECK(!slot.rotor_per_axis && !slot.direction && !slot.current_limit_a);
        CHECK(!slot.output_limit_raw && !slot.max_rotor_speed_rad_s);
        CHECK(!slot.max_axis_velocity_rad_s && !slot.velocity_pid && !slot.position_pid);
    }
    for (const auto& slot : config.dm) {
        CHECK(!slot.installed);
        CHECK(!slot.can_id && !slot.master_id && !slot.limits && !slot.mit_kp && !slot.mit_kd);
    }
    for (const auto& wheel : config.wheels) {
        CHECK(!wheel.x_m && !wheel.y_m && !wheel.roll_dir_rad && !wheel.radius_m && !wheel.gain);
    }
    for (const auto& joint : config.joints) {
        CHECK(!joint.zero_offset_rad && !joint.min_rad && !joint.max_rad);
    }
    // 软件时序策略必须齐备, 否则应用连"报告缺项"都不成立.
    CHECK(config.timing.control_dt_s == board::app_control_dt_s);
    CHECK(config.timing.target_ttl.valid());
    CHECK(config.timing.remote_ttl.valid());
    CHECK(config.timing.imu_ttl.valid());
    CHECK(config.timing.host_link_ttl.valid());
    // 部署时限必须就是板级常量本身: 漏传会造成"无效时限 = 永不老化".
    CHECK(config.timing.host_link_ttl.ms == board::app_host_link_ttl_ms);
    CHECK(config.timing.max_dt_s.has_value());
    CHECK(*config.timing.max_dt_s > config.timing.control_dt_s);
}

void test_deployment_audience_rejects_and_grants_nothing() {
    const auto config = board::deployment_config();
    const auto report = validate(config, ConfigAudience::deployment);
    CHECK(report.kind == ConfigKind::real_deployment);
    CHECK(!report.complete);
    CHECK(report.has(ConfigIssue::missing_timing) == false);
    for (std::size_t i = 0; i < domain_count; ++i) {
        CHECK(report.domains[i] == ConfigState::unavailable);
    }
    SentinelApp app(config, ConfigAudience::deployment);
    // 配置类别合法 (部署用途可构造), 但没有任何域取得执行资格.
    CHECK(app.startable());
    CHECK(!app.report().complete);
    const auto before = app.status(ec::Tick{1000});
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto& domain = before.domains[i];
        CHECK(!domain.configured);
        CHECK(domain.validity == DomainValidity::unavailable);
        CHECK(domain.reason == Reason::not_configured);
        // 未装配域没有目标, 也不伪造输出.
        CHECK(!domain.target.has_value());
    }
    // 跑一个周期 (无任何输入): 每域执行结论必须是"未配置"而不是任何形式的目标/输出.
    app.update(ec::Tick{1001}, AppInputs{});
    const auto& snapshot = app.snapshot();
    for (std::size_t i = 0; i < domain_count; ++i) {
        CHECK(!snapshot.executions[i].execute);
        CHECK(snapshot.executions[i].reason == Reason::not_configured);
        CHECK(snapshot.executions[i].mode == ec::control::ControlMode::none);
        CHECK(!snapshot.executions[i].adopted_target.has_value());
        CHECK(!snapshot.executions[i].applied_output.has_value());
        // 没有设备可落实时不产生撇输出动作, 也不冒充"已发零电流".
        CHECK(snapshot.executions[i].stop == ec::control::StopAction::unspecified);
    }
    CHECK(!app.snapshot().launch_allowed);
    CHECK(app.snapshot().permission.source == Source::none);
    CHECK(!app.snapshot().permission.authority);
}

void test_synthetic_config_is_rejected_in_deployment() {
    // 合成夹具只能在离线用途使用: 部署用途必须拒绝, 不允许"顺手用夹具顶替实车配置".
    const auto synthetic = synthetic_config();
    const auto report = validate(synthetic, ConfigAudience::deployment);
    CHECK(report.has(ConfigIssue::synthetic_value_in_deployment));
    CHECK(!report.startable);
    SentinelApp app(synthetic, ConfigAudience::deployment);
    CHECK(!app.startable());
}

void test_deployment_summary_matches_config() {
    const auto summary = board::summarize(board::deployment_config());
    CHECK(summary.magic == board::deployment_summary_magic);
    CHECK(summary.layout == board::deployment_summary_layout);
    CHECK(summary.kind == static_cast<std::uint32_t>(ConfigKind::real_deployment));
    CHECK(summary.installed_dji == 0);
    CHECK(summary.installed_dm == 0);
    CHECK(summary.synthetic_values == 0);
    CHECK(summary.unknown_origin_installed == 0);
    // 镜像里声明的容量必须覆盖已验证接线与九 DJI + 一 DM 的应用槽位.
    CHECK(summary.dji_capacity == dji_slot_count);
    CHECK(summary.harness_dji == 9);
    CHECK(summary.harness_dm == 1);
    CHECK(ec_app_deployment_summary.magic == summary.magic);
    CHECK(ec_app_deployment_summary.harness_dji == summary.harness_dji);
}

void test_known_harness_is_consistent_with_app_capacity() {
    // 已验证接线是"待确认清单的机器可读形式", 不是配置: 这里只核对它与应用容量、
    // 与各协议支持的设备 ID 范围一致, 不把设备映射到机构.
    CHECK(board::known_dji_harness.size() == static_cast<std::size_t>(dji_slot_count));
    for (std::size_t i = 0; i < board::known_dji_harness.size(); ++i) {
        const auto& entry = board::known_dji_harness[i];
        CHECK(entry.bus == 1 || entry.bus == 2);
        const auto protocol = ec::drivers::dji_protocol(entry.mode);
        CHECK(protocol.has_value());
        CHECK(entry.device_id >= 1 && entry.device_id <= protocol->count);
        // 同总线同设备 ID 不得重复: 重复会让两个槽位争同一个反馈 ID.
        for (std::size_t j = i + 1; j < board::known_dji_harness.size(); ++j) {
            const auto& other = board::known_dji_harness[j];
            CHECK(!(other.bus == entry.bus && other.device_id == entry.device_id));
        }
    }
    // 双 yaw 在 CAN1 (GM6020), 底盘/摩擦/拨弹在 CAN2: 这是硬件 Checklist 已确认的分布.
    std::size_t bus1 = 0, bus2 = 0;
    for (const auto& entry : board::known_dji_harness) {
        (entry.bus == 1 ? bus1 : bus2) += 1;
    }
    CHECK(bus1 == 2);
    CHECK(bus2 == 7);
}

void test_control_step_boundaries() {
    // 首次周期: 用明确的初始化步长; Tick=0 是合法时刻, 不能当"从未初始化".
    const auto first = board::app_control_step(0u, 0u, false);
    CHECK(first.dt_s == board::app_control_dt_s);
    CHECK(!first.overrun);
    // 真实前进: 1/2/5/20 ms 都交出真实值, 超期只体现在统计标记.
    CHECK(board::app_control_step(1001u, 1000u, true).dt_s == 0.001f);
    CHECK(std::fabs(board::app_control_step(1002u, 1000u, true).dt_s - 0.002f) < 1.0e-6f);
    CHECK(std::fabs(board::app_control_step(1005u, 1000u, true).dt_s - 0.005f) < 1.0e-6f);
    CHECK(!board::app_control_step(1005u, 1000u, true).overrun);
    const auto over = board::app_control_step(1020u, 1000u, true);
    CHECK(std::fabs(over.dt_s - 0.020f) < 1.0e-6f);
    CHECK(over.overrun);
    // 重复时刻/时间回退: dt=0 交给既有拒绝/不推进路径, 不用标称周期冒充正常采样.
    CHECK(board::app_control_step(1000u, 1000u, true).dt_s == 0.0f);
    CHECK(board::app_control_step(999u, 1000u, true).dt_s == 0.0f);
    CHECK(!board::app_control_step(999u, 1000u, true).overrun);
    // 回绕: 半个回绕窗口内的前进是真实步长, 跨越窗口的倒退是回退.
    const auto wrapped_forward = board::app_control_step(5u, 0xfffffff8u, true);
    CHECK(std::fabs(wrapped_forward.dt_s - 0.013f) < 1.0e-6f);
    CHECK(wrapped_forward.overrun);
    CHECK(board::app_control_step(0xfffffff8u, 5u, true).dt_s == 0.0f);
}

void test_micro_time_and_can_fairness() {
    CHECK(std::fabs(board::app_control_step_us(2750, 1000, true).dt_s - 0.00175f) < 1e-7f);
    CHECK(board::app_control_step_us(1000, 1000, true).dt_s == 0.0f);
    CHECK(board::app_control_step_us(900, 1000, true).dt_s == 0.0f);
    CHECK(std::fabs(board::app_control_step_us(1000, 0xfffffc18u, true).dt_s - 0.002f) < 1e-7f);
    struct Lock { int enter() noexcept { return 0; } void exit(int) noexcept {} } lock;
    board::CanInbox<2, Lock> queue{lock};
    ec::drivers::StampedCanFrame frame{};
    frame.bus = 2;
    frame.frame.id = 100;
    CHECK(queue.try_push(frame) == ec::PushResult::accepted);
    frame.frame.id = 101;
    CHECK(queue.try_push(frame) == ec::PushResult::accepted);
    CHECK(queue.try_push(frame) != ec::PushResult::accepted);
    frame.bus = 1;
    frame.frame.id = 1;
    frame.received_us = {12345};
    CHECK(queue.try_push(frame) == ec::PushResult::accepted);
    const auto first = queue.try_pop();
    CHECK(first && first->bus == 1 && first->received_us.us == 12345);
    CHECK(queue.try_pop()->frame.id == 100);
    CHECK(queue.try_pop()->frame.id == 101);
    CHECK(!queue.try_pop());
}

void test_spi_deadline() {
    struct Io {
        std::uint32_t us{};
        bool stalled_tx{}, stalled_rx{}, stalled_busy{};
        std::uint8_t value{};
        unsigned writes{};
        ec::MicroTick now() noexcept { return {us++}; }
        bool tx_ready() const noexcept { return !stalled_tx; }
        bool rx_ready() const noexcept { return !stalled_rx; }
        bool busy() const noexcept { return stalled_busy; }
        bool error() const noexcept { return false; }
        void write(std::uint8_t v) noexcept { value = v; ++writes; }
        std::uint8_t read() const noexcept { return value; }
    };
    std::array<std::uint8_t, 3> tx{1, 2, 3}, rx{};
    Io good;
    CHECK(board::transfer_spi(good, tx, rx, 100) && tx == rx);
    for (int failure = 0; failure < 3; ++failure) {
        Io bad;
        bad.stalled_tx = failure == 0;
        bad.stalled_rx = failure == 1;
        bad.stalled_busy = failure == 2;
        CHECK(!board::transfer_spi(bad, tx, rx, 100));
        CHECK(bad.us <= 102); // 总事务截止, 不是每个字节各等 100 us
    }
    Io wrap;
    wrap.us = 0xfffffffcu;
    CHECK(board::transfer_spi(wrap, tx, rx, 100));
}

void test_role_budgets_are_consistent() {
    // 正式 APP 的收帧预算必须能覆盖九 DJI + 一 DM 的 1 kHz 反馈 (10 帧/ms):
    // 单周期上限 / 周期 ≥ 输入率, 且队列容量覆盖数个周期的服务空窗.
    using namespace ec::sentinel;
    constexpr double input_frames_per_ms = 10.0;
    constexpr double app_capacity_per_ms = static_cast<double>(app_can_frames_per_cycle) /
        static_cast<double>(app_control_period_ms);
    static_assert(app_capacity_per_ms >= input_frames_per_ms,
        "APP 收帧预算必须覆盖 1 kHz × 10 设备的输入率");
    static_assert(app_can_queue_capacity >= static_cast<std::size_t>(input_frames_per_ms * 4),
        "APP 队列必须覆盖数个周期的服务空窗");
    static_assert(can_watch_capacity >= dji_install_max + dm_install_max,
        "在线跟踪表必须覆盖全部装配槽位");
    CHECK(app_control_period_ms == 1);
    CHECK(app_link_period_ms == 10);
    CHECK(dji_install_max == static_cast<std::size_t>(dji_slot_count));
    // 默认角色不因 APP 预算而改变.
    CHECK(can_queue_capacity >= 32);
    CHECK(can_watch_capacity >= 17);
}
} // 命名空间

int main() {
    test_deployment_config_is_not_invented();
    test_deployment_audience_rejects_and_grants_nothing();
    test_synthetic_config_is_rejected_in_deployment();
    test_deployment_summary_matches_config();
    test_known_harness_is_consistent_with_app_capacity();
    test_control_step_boundaries();
    test_micro_time_and_can_fairness();
    test_spi_deadline();
    test_role_budgets_are_consistent();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("board deployment config contracts passed\n");
    return EXIT_SUCCESS;
}
