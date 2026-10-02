// 正式 APP 角色的板级接入实现 (O6).
// 只由 appControl 任务触碰应用对象; 启动阶段 (调度器未运行) 完成全部构造, 运行期
// 不分配内存、不抛异常、不做无界循环.
#include "board_app.hpp"
#include "board_raw.hpp"
#if EC_SENTINEL_APP
#include "board_app_config.hpp"
#if EC_APP_GM6020_BENCH || EC_APP_FOUR_MOTOR_BENCH
#include "gm6020_bench_velocity.hpp"
#endif
#include "board_assembly.hpp"
#include "board_inputs.hpp"
#include "board_io.h"
#include "board_registry.h"
#include "ec/platform/stm32f407.hpp"
#include "ec/transport/uart_frame.hpp"
#include "s1_diagnostics.h"
#include "sentinel_app.hpp"
#include "sentinel_config.hpp"
#include "sentinel_cycle.hpp"
#include "sentinel_execution.hpp"
#include "sentinel_params.hpp"
#include "sentinel_perception.hpp"
#include "sentinel_protocol.hpp"
#include "sentinel_telemetry.hpp"
#include "sentinel_link.hpp"
#include "can.h"
#include <array>
#include <optional>

namespace ec::sentinel {
namespace {
using namespace ec::sentinel::app;

// 静态对象所有权: 全部在 ec_app_init() 里用 emplace 构造在固定存储上, 运行期不再分配.
std::optional<stm32f407::HalCanTx> bus1{};
std::optional<stm32f407::HalCanTx> bus2{};
std::optional<SentinelApp> application{};
std::optional<SentinelExecutor> executor{};
std::optional<SentinelPerception> perception{};
std::optional<ParamStore> params{};
std::optional<SentinelHostSession> session{};
std::optional<SentinelCycle> cycle{};
#if EC_APP_GM6020_BENCH || EC_APP_FOUR_MOTOR_BENCH
BenchEncoderVelocity bench_velocity{};
#endif
// 本周期编排输入: control 与 commit 共享同一份 (板级 assembly_step 分两次调用).
CycleInputs cycle_inputs{};
float cycle_dt_s{};
EcAppObservation observation{};
// USART1 业务协议: 分帧解码器与载荷缓冲 (定长, 不随请求增长).
transport::UartFrameDecoder decoder{};
std::array<std::uint8_t, protocol_payload_max> response_payload{};
ReplyMux replies{};
std::uint32_t handled_this_cycle{};
// 上一次控制提交的 MCU 时刻与超期次数: 真实 dt 与超期统计互相不能替代.
std::uint32_t last_control_us{};
// 是否已经有过一个控制周期. 用独立状态而不是把合法 Tick=0 当哨兵值:
// 上电后首个周期用明确初始化值, 之后重复/回退走既有 dt 拒绝路径.
bool have_last_control{};
std::uint32_t commit_overruns{};
std::uint32_t link_losses{};
std::uint32_t link_dropped{};
std::uint32_t observed_overflows{};
std::uint32_t publish_counter{};
std::size_t config_missing_items{};
bool ready{};

// 本控制周期使用的同一个时刻: 请求处理、会话老化与控制快照必须共用它,
// 否则"先收包(较新时间)再老化(较旧周期时间)"会把刚刷新的会话误判为过期.
std::uint32_t current_cycle_ms{};

// 一个控制周期内最多处理 app_requests_per_cycle 条请求: 超出部分丢弃并计数,
// 不用"读多少处理多少"把周期长度交给对端决定.
void handle_request(const transport::UartFrame& frame) noexcept {
    session->set_frame_rejects(decoder.rejected());
    session->set_tx_diagnostics(replies.diagnostics());
    const auto size = session->handle({frame.payload.data(), frame.size},
        Tick{current_cycle_ms}, response_payload);
    if (size == 0) { return; }
    if (!replies.enqueue(frame, {response_payload.data(), size})) { ++link_dropped; }
}

void publish_observation(Tick now) noexcept {
    const auto& snapshot = application->snapshot();
    const auto& stats = executor->stats();
    observation.fast_size = static_cast<std::uint16_t>(encode_fast_status(*application, &*perception,
        now, session->session_id(), 0, session->boot_id(), observation.fast_payload));
    observation.now_ms = now.ms;
    observation.source = static_cast<std::uint8_t>(snapshot.permission.source);
    observation.authority = snapshot.permission.authority ? 1 : 0;
    observation.manual_stop = snapshot.permission.manual_stop ? 1 : 0;
    observation.locally_ready = snapshot.permission.locally_ready ? 1 : 0;
    observation.host_session = snapshot.input.host_link_alive ? 1 : 0;
    observation.maintenance_session =
        snapshot.permission.source == Source::maintenance ? 1 : 0;
    observation.config_complete = application->report().complete ? 1 : 0;
    observation.config_startable = application->startable() ? 1 : 0;
    observation.config_epoch = application->config_epoch();
    observation.reference_epoch = application->reference_epoch();
    observation.config_missing_items = static_cast<std::uint32_t>(config_missing_items);
    observation.accepted_packets = application->accepted_packets();
    observation.rejected_packets = application->rejected_packets();
    observation.link_losses = link_losses;
    observation.link_dropped = link_dropped;
    observation.frame_rejects = decoder.rejected();
    observation.replay_rejects = session->replay_rejects();
    observation.group_sent = stats.group_sent;
    observation.group_refused = stats.group_refused;
    observation.group_tx_error = stats.group_tx_error;
    observation.dm_tx_error = stats.dm_tx_error;
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto& status = snapshot.domains[i];
        const auto& execution = snapshot.executions[i];
        auto& entry = observation.domains[i];
        entry.validity = static_cast<std::uint8_t>(status.validity);
        entry.mode = static_cast<std::uint8_t>(execution.mode);
        entry.execute = execution.execute ? 1 : 0;
        entry.reason = static_cast<std::uint8_t>(execution.reason);
        entry.fault = static_cast<std::uint8_t>(status.fault);
        entry.sampled_at_ms = status.feedback.velocity_rad_s.sampled_at.ms;
        entry.target = execution.adopted_target.value_or(0.0f);
        entry.measurement = status.feedback.velocity_rad_s.validity == Validity::valid
            ? status.feedback.velocity_rad_s.value : 0.0f;
    }
    // 设备视图: 未装配槽位保持 installed=0, 不填健康零值; 温度只在设备真的给出时置位.
    for (std::size_t i = 0; i < dji_slot_count; ++i) {
        auto& entry = observation.dji[i];
        entry = EcAppDeviceObservation{};
        if (!application->config().dji[i].installed) { continue; }
        auto* device = executor->dji_device(i);
        if (device == nullptr) { continue; }
        const auto state = device->state(now);
        entry.installed = 1;
        entry.bus = device->config().bus;
        entry.sampled_at_ms = state.rotor.sampled_at.ms;
        entry.online = state.rotor.fresh_at(now, device->config().feedback_ttl) ? 1 : 0;
        entry.velocity_rad_s = state.velocity_rad_s.value;
        if (state.rotor.validity == Validity::valid && state.rotor.value.temperature_c) {
            entry.temperature_valid = 1;
            entry.temperature_c = static_cast<std::int8_t>(*state.rotor.value.temperature_c);
        }
    }
    auto& dm_entry = observation.dm[0];
    dm_entry = EcAppDeviceObservation{};
    if (application->config().dm[0].installed) {
        if (auto* device = executor->dm_device()) {
            const auto state = device->state();
            dm_entry.installed = 1;
            dm_entry.bus = device->config().bus;
            dm_entry.sampled_at_ms = state.feedback.sampled_at.ms;
            dm_entry.online = device->online(now) ? 1 : 0;
            dm_entry.enabled = device->enabled(now) ? 1 : 0;
            dm_entry.faulted = device->faulted(now) ? 1 : 0;
            dm_entry.velocity_rad_s = state.feedback.value.velocity_rad_s;
        }
    }
}
} // 内部对象与周期辅助

void ec_app_init() noexcept {
    static_assert(app::domain_count == EC_APP_DOMAIN_COUNT, "观测视图域数与应用域数必须一致");
    static_assert(app::dji_slot_count == EC_APP_DJI_SLOT_COUNT, "观测视图 DJI 槽数与应用槽数必须一致");
    static_assert(app::dm_slot_count == EC_APP_DM_SLOT_COUNT, "观测视图 DM 槽数与应用槽数必须一致");
    const auto config = board::deployment_config();
    // 部署摘要与运行时配置同源: 经 volatile 指针读, 使 Release 下引用不被常量折叠,
    // 符号因此留在镜像里供离线检查核对 (脚本按符号从未压缩镜像读取同一份摘要).
    const volatile auto* summary = &::ec_app_deployment_summary;
    if (summary->magic != board::deployment_summary_magic ||
        summary->kind != static_cast<std::uint32_t>(ConfigKind::real_deployment)) {
        ec_s1_event(EC_S1_APP_CONFIG, 0);
    }
    application.emplace(config, ConfigAudience::deployment, (EC_APP_GM6020_BENCH || EC_APP_FOUR_MOTOR_BENCH)
        ? StartupPolicy::maintenance_only : StartupPolicy::remote_required);
    bus1.emplace(hcan1);
    bus2.emplace(hcan2);
    executor.emplace(config, ConfigAudience::deployment, *bus1, *bus2,
        []() noexcept { return MicroTick{ec_board_micros()}; });
    perception.emplace(config, ConfigAudience::deployment);
    params.emplace(config);
    // 会话链路时限来自部署配置: 漏传会让构造默认的无效时限把老化变成"永不失效".
    session.emplace(*application, *params, &*executor, &*perception, config.timing.host_link_ttl);
    // 操纵映射量程/档位阈值属实测项: 部署配置里为 0 表示"未配置", 不冒充已标定数值.
    cycle.emplace(*application, *executor, *perception, board::deployment_mapping(),
        config.timing.remote_ttl);
    const auto& report = application->report();
    config_missing_items = report.issue_count;
    ready = true;
    // 启动阶段记录: 缺项数与已验证接线条目数 (供 SWD/串口观测, 不是执行许可).
    ec_s1_event(EC_S1_APP_CONFIG,
        static_cast<std::uint32_t>(config_missing_items * 100 + summary->harness_dji));
    // 注册应用状态观测来源; 之后的发布与注册表在同一任务里进行, 不需要额外同步.
    ec_registry_publish_app(&observation);
}

void ec_app_control_tick() noexcept {
    if (!ready) { return; }
    const auto sample_ms = HAL_GetTick();
    // 有界收帧/采样在业务时刻之前, 不把刚采完的样本误判为未来数据.
    ec_board_drain_can(sample_ms, can_frames_per_cycle);
#if EC_APP_GM6020_BENCH || EC_APP_FOUR_MOTOR_BENCH
    executor->set_joint_velocity_feedback(Domain::main_yaw, bench_velocity.value());
#endif
    ec_board_poll_imu(sample_ms);
    const auto now_ms = HAL_GetTick();
    const Tick now{now_ms};
    current_cycle_ms = now_ms;
    // 3. USART1 业务协议: 搬运量有上界, 每周期最多处理 app_requests_per_cycle 条请求.
    //    接收管道溢出说明字节流已不连续: 丢掉半帧状态, 不把跨过丢失区间的字节当一帧.
    handled_this_cycle = 0;
    const auto overflows = ec_board_uart1_overflows();
    if (overflows != observed_overflows) {
        observed_overflows = overflows;
        decoder.reset();
        ++link_losses;
    }
    ec_board_poll_uart1();
    // 4. 应用周期 (assembly_step 调本文件定义的强钩子: 先控制后提交).
    assembly_step(now_ms);
    static_cast<void>(replies.service(now, [](std::span<const std::uint8_t> wire) {
        return ec_board_uart1_try_send(wire.data(), wire.size()) != 0;
    }));
    // 5. 100 Hz 发布监控注册表与应用观测, 不在每个 1 ms 周期复制整表.
    if (++publish_counter % app_publish_period_cycles == 0) {
        publish_observation(now);
        ec_registry_publish(now_ms);
    }
}

// 任务入口 (freertos.c 的 appControl 任务): 固定周期, 不等待外部事件;
// 周期错乱与任何致命故障都走既有 ec_s1_fail 诊断路径, 不静默降速.
extern "C" void ec_app_control_task_entry(void) {
    stm32f407::PeriodicWait wait(pdMS_TO_TICKS(app_control_period_ms));
    for (;;) {
        if (!wait.wait_next()) {
            ec_s1_fail(EC_S1_PERIOD_ERROR, __FILE__, __LINE__);
        }
        ec_app_control_tick();
    }
}

void ec_app_link_bytes(std::span<const std::uint8_t> bytes) noexcept {
    if (!ready) { return; }
    for (const auto byte : bytes) {
        const auto frame = decoder.push(byte);
        if (!frame) { continue; }
        if (handled_this_cycle >= app_requests_per_cycle) {
            ++link_dropped;
            continue;
        }
        ++handled_this_cycle;
        handle_request(*frame);
    }
}

void ec_app_route(std::uint8_t bus, const transport::CanFrame& frame, Tick at, MicroTick at_us) noexcept {
    if (!ready) { return; }
    executor->route(bus, frame, at, at_us);
#if EC_APP_GM6020_BENCH || EC_APP_FOUR_MOTOR_BENCH
    if (bus == 1 && frame.id == 0x208 && frame.size == 8) {
        const auto encoder = static_cast<std::uint16_t>((static_cast<unsigned>(frame.data[0]) << 8) | frame.data[1]);
        if (encoder < 8192) { bench_velocity.receive(encoder, at, at_us); }
    }
#endif
    ec_raw_can(bus, frame, at_us, g_ec_io.can.rx_dropped);
}

const EcAppObservation& ec_app_observation() noexcept { return observation; }
bool ec_app_ready() noexcept { return ready; }
std::size_t ec_app_config_missing_items() noexcept { return config_missing_items; }
std::size_t ec_app_harness_dji_count() noexcept { return board::known_dji_harness.size(); }

// APP 钩子归属: 正式 APP 角色强定义这两个钩子 (默认角色保留弱定义, 不产生任何输出).
// assembly_step 先调 control 后调 commit, 因此两者共享同一份 inputs 与同一任务上下文.
extern "C" void ec_board_app_control(std::uint32_t now_ms) {
    if (!ready) { return; }
    const Tick now{now_ms};
    // 真实控制步长: 用两次控制周期的实际时间差驱动映射积分与 PID, 而不是固定标称周期.
    // 超期仍传真实值 —— 不伪造 dt, 也不错期静默跳过控制; 是否拒绝由轴/环已有的
    // max_dt 判定路径决定 (超限目标不会产生输出, 并回灌为执行拒绝).
    // 首次周期用明确的初始化步长; 之后重复时刻/时间回退不再伪装成标称周期, 而是交给
    // 同一条 dt 拒绝/不推进路径 (dt_s = 0), 并保留真实前进的超期统计.
    const auto now_us = ec_board_micros();
    const auto step = board::app_control_step_us(now_us, last_control_us, have_last_control);
    cycle_dt_s = step.dt_s;
    if (step.overrun) { ++commit_overruns; }
    have_last_control = true;
    last_control_us = now_us;

    cycle_inputs = CycleInputs{};
    cycle_inputs.now = now;
    cycle_inputs.dt_s = cycle_dt_s;
    cycle_inputs.now_us = MicroTick{now_us};
    // 遥控: 只把接收路径解出的**完整**样本交给编排; 槽位未更新时重复读取由编排去重.
    if (const auto remote = input_remote(now)) {
        cycle_inputs.remote_sample = remote->value;
        cycle_inputs.remote_received_at = remote->sampled_at;
    }
    // IMU: 传感器侧量与采样时刻; 是否可用仍由 imu_ttl 判定, 不在这里补零.
    if (const auto imu = input_imu_reading(now)) {
        const auto& sample = imu->value.sample;
        cycle_inputs.imu_gyro_rad_s = std::array<float, 3>{
            sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2]};
        cycle_inputs.imu_accel_mps2 = std::array<float, 3>{
            sample.acc_mps2[0], sample.acc_mps2[1], sample.acc_mps2[2]};
        cycle_inputs.imu_sampled_at = imu->sampled_at;
        cycle_inputs.imu_times = imu->value.times;
    }
    // 会话/链路状态由业务协议显式提供; 未授权会话不产生来源资格.
    const auto link = session->link_state(now);
    const auto role = session->role_source();
    cycle_inputs.host_link_alive = link.alive && role == Source::host;
    cycle_inputs.host_request = link.request && role == Source::host;
    cycle_inputs.maintenance_link_alive = link.alive && role == Source::maintenance;
    cycle_inputs.maintenance_request = link.request && role == Source::maintenance;
    cycle_inputs.host_session_epoch = link.epoch;
    cycle_inputs.maintenance_session_epoch = link.epoch;
    cycle_inputs.locally_ready = ready && g_ec_s1.fault == 0 && g_ec_s1.stage != EC_S1_FATAL;
    // 外部约束: 本版只有业务协议的发射许可 (由应用合并进快照), 功率/热量无来源.
    cycle_inputs.aux = AuxState{};
    static_cast<void>(cycle->control(cycle_inputs));
}

extern "C" void ec_board_app_commit(std::uint32_t now_ms) {
    if (!ready) { return; }
    static_cast<void>(now_ms);
    // 提交半周期: 真实设备编解码与 CAN 组帧, 以及命令预测积分 (不是轮速反馈里程计).
    cycle->commit(cycle_inputs);
    const auto domain = ec_raw_control_domain();
    if (domain >= 0) {
        if (const auto record = executor->trace(static_cast<Domain>(domain), {now_ms}, *application)) { ec_raw_push(*record); }
    }
}

} // 命名空间 ec::sentinel
#endif
