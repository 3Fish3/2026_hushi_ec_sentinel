// 同固件调参/遥控切换: 实际执行目标、MIT 帧及在线 kp/kd 必须沿用同一运行对象.
#include "board_app_config.hpp"
#include "sentinel_cycle.hpp"
#include "sentinel_param_apply.hpp"
#include "sim_plant.hpp"
#include "raw_record.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <tuple>

namespace {
namespace app = ec::sentinel::app;
namespace board = ec::sentinel::board;
namespace host = ec::sentinel::host;
using app::Domain;
using ec::drivers::Dr16Switch;
constexpr auto pitch = app::domain_index(Domain::pitch);
void check(bool ok, int line) {
    if (!ok) { std::fprintf(stderr, "gimbal remote line %d\n", line); std::_Exit(3); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), __LINE__)

struct Rig {
    app::SentinelConfig config{board::deployment_config()};
    host::SoftwareBus bus1{}, bus2{};
    app::SentinelApp application{config, app::ConfigAudience::deployment};
    app::SentinelExecutor executor{config, app::ConfigAudience::deployment, bus1, bus2};
    app::SentinelPerception perception{config, app::ConfigAudience::deployment};
    app::SentinelCycle cycle{application, executor, perception, board::deployment_mapping(),
                            config.timing.remote_ttl};
    app::ParamStore params{config};
    host::SimPlant plant{};
    ec::Tick now{1000};
    Dr16Switch left{Dr16Switch::mid}, right{Dr16Switch::up};
    std::int16_t x{}, y{};
    bool maintenance{true};
    ec::sentinel::recording::Recorder* recorder{};

    Rig() {
        plant.dji[4].position_rad = *config.joints[0].zero_offset_rad;
        plant.dji[5].position_rad = *config.joints[1].zero_offset_rad;
        plant.dm.position_rad = *config.joints[2].zero_offset_rad;
        CHECK(executor.assembled());
    }

    void tick() {
        app::CycleInputs in{};
        in.now = now; in.now_us = ec::MicroTick{now.ms * 1000u}; in.dt_s = 0.001f;
        in.locally_ready = true; in.maintenance_link_alive = maintenance; in.maintenance_request = maintenance;
        in.maintenance_session_epoch = 1;
        ec::drivers::Dr16Sample remote{};
        remote.switch_left = left; remote.switch_right = right;
        remote.ch0 = x; remote.ch1 = y;
        in.remote_sample = remote; in.remote_received_at = now;
        plant.emit_feedback(config, [&](std::uint8_t bus, const ec::transport::CanFrame& frame, ec::Tick at) {
            executor.route(bus, frame, at, *in.now_us);
            if (recorder) { recorder->can(bus, frame, *in.now_us, *in.now_us, 0); }
        }, now);
        bus1.clear(); bus2.clear();
        static_cast<void>(cycle.control(in)); cycle.commit(in); ++now.ms;
        if (recorder) {
            if (const auto record = executor.trace(Domain::pitch, in.now, application)) {
                recorder->push(*record, *in.now_us);
            }
        }
    }

    void warmup() { for (int i = 0; i < 8; ++i) { tick(); } }
};

void commissioned_fire_yaw_defaults_survive_boot() {
    // 调参使用PARAM_WRITE原生值; 冷启动的协议读回与实际内环必须保持相同口径.
    // 曾把0.09/2.00再次除以2.5A, 导致烧录后退化为0.036/0.80.
    Rig boot;
    for (const auto& [id, expected, gain] : {
            std::tuple{0x501, 0.09f, 0}, std::tuple{0x502, 2.0f, 1}}) {
        const auto* entry = boot.params.entry(id);
        CHECK(entry && !entry->overridden);
        CHECK(std::fabs(entry->value - expected) < 1e-6f);
        CHECK(std::fabs(entry->spec.default_value - expected) < 1e-6f);
        CHECK(std::fabs(boot.executor.velocity_gain(5, gain) - expected) < 1e-6f);
    }
}

void gain_and_target_continuity() {
    Rig r; r.warmup();
    CHECK(board::deployment_mapping().layout == app::RemoteLayout::gimbal_validation);
    CHECK(r.application.authority().source() == app::Source::maintenance);
    CHECK(app::apply_param(r.params, r.application, &r.executor, &r.perception, 0x901, 9.0f, true)
          == app::ParamWriteResult::accepted);
    CHECK(app::apply_param(r.params, r.application, &r.executor, &r.perception, 0x902, 0.6f, true)
          == app::ParamWriteResult::accepted);
    app::TargetSnapshot target{};
    target.domains[pitch] = app::DomainSetpoint{ec::control::ControlMode::position, 0.02f};
    CHECK(r.application.accept_targets(target, r.now, app::Source::maintenance) == app::AcceptResult::accepted);
    r.warmup();
    CHECK(r.application.snapshot().executions[pitch].execute);
    const float before = *r.application.snapshot().executions[pitch].adopted_target;
    CHECK(std::fabs(before - 0.02f) < 1e-6f);
    // 同一份启用辨识与遥控的部署配置: 先跑完有采集联锁的方波, 再验证切档.
    ec::sentinel::recording::Recorder recorder;
    r.recorder = &recorder;
    r.executor.bind_pitch_recorder(&recorder);
    std::array<std::uint8_t, 12> request{1, 1, 3, 6};
    std::array<std::uint8_t, 32> response{};
    ec::sentinel::recording::put32(request, 4, 1);
    ec::sentinel::recording::put32(request, 8, 500000);
    CHECK(recorder.stream_request(request, response, {r.now.ms * 1000u}) == 32 && response[2] == 0);
    CHECK(r.executor.start_pitch_identification({1, 0.03f, 20000, 20000, 2, 20000}, r.now, r.application));
    for (int i = 0; i < 125; ++i) {
        CHECK(r.application.accept_targets(target, r.now, app::Source::maintenance) == app::AcceptResult::accepted);
        r.tick();
    }
    CHECK(r.executor.pitch_identification().state() == app::PitchIdentificationState::complete);
    CHECK(!r.executor.pitch_identification().latched());
    // 保留重力保持所需的位置误差; 切到遥控时摇杆即使仍偏转也不得产生新目标跳变.
    r.left = Dr16Switch::up; r.right = Dr16Switch::mid; r.y = 330;
    r.tick();
    CHECK(r.application.authority().source() == app::Source::remote);
    CHECK(r.application.snapshot().executions[pitch].execute);
    CHECK(std::fabs(*r.application.snapshot().executions[pitch].adopted_target - before) < 1e-6f);
    CHECK(r.executor.dm_mit_kp() == 9.0f && r.executor.dm_mit_kd() == 0.6f);
    CHECK(r.params.entry(0x901)->value == 9.0f && r.params.entry(0x902)->value == 0.6f);
    CHECK(!r.application.snapshot().executions[0].execute);
    CHECK(!r.application.snapshot().executions[7].execute);
    // Pitch 发送周期是 2ms; 在下一个正常发送时刻核对编码帧, 不改调度频率迁就测试.
    r.tick();
    bool mit = false;
    for (const auto& frame : r.bus2.frames) {
        if (frame.id == 1 && frame.size == 8 && frame.data[0] != 0xff) {
            const auto command = host::decode_dm_mit_command(frame, *r.config.dm[0].limits);
            CHECK(command);
            CHECK(std::fabs(command->kp - 9.0f) < 0.2f);
            CHECK(std::fabs(command->kd - 0.6f) < 0.02f);
            CHECK(std::fabs(command->position_rad - (*r.config.joints[2].zero_offset_rad + before)) < 0.001f);
            mit = true;
        }
    }
    CHECK(mit);
    r.maintenance = false; r.tick();
    CHECK(r.application.authority().source() == app::Source::remote);
    CHECK(r.application.snapshot().executions[pitch].execute);
    r.y = 0; r.tick();
    r.y = 330; r.tick();
    CHECK(*r.application.snapshot().executions[pitch].adopted_target > before);
    r.y = 0;
    const auto main_before = r.cycle.mapper().held_angle(Domain::main_yaw);
    const auto fire_before = r.cycle.mapper().held_angle(Domain::fire_yaw);
    CHECK(main_before && fire_before);
    r.right = Dr16Switch::up; r.x = 330; r.tick();
    CHECK(r.cycle.mapper().held_angle(Domain::fire_yaw) == fire_before);
    r.x = 0; r.tick(); r.x = 330; r.tick();
    CHECK(r.cycle.mapper().held_angle(Domain::main_yaw) == main_before);
    CHECK(*r.cycle.mapper().held_angle(Domain::fire_yaw) > *fire_before);
    r.x = 0; r.y = 660;
    for (int i = 0; i < 5000; ++i) { r.tick(); }
    CHECK(*r.application.snapshot().executions[pitch].adopted_target <= *r.config.joints[2].max_rad);
    r.right = Dr16Switch::down; r.tick();
    CHECK(!r.application.snapshot().executions[pitch].execute);
    CHECK(r.executor.dm_mit_kp() == 9.0f && r.executor.dm_mit_kd() == 0.6f);
}
}

int main() { commissioned_fire_yaw_defaults_survive_boot(); gain_and_target_continuity(); }
