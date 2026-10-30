// Offline only: bounded square excitation and persistent stop latch contracts.
#include "sentinel_pitch_identification.hpp"
#include "sentinel_execution.hpp"
#include "sim_plant.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string_view>

namespace {
using namespace ec::sentinel::app;
void check(bool value, int line) {
    if (!value) { std::fprintf(stderr, "pitch identification line %d\n", line); std::_Exit(3); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), __LINE__)
PitchIdentificationPolicy policy() { return {-0.4f, 0.4f, 0.1f, 1.0f, 0.5f, 0.05f, 10000, 2000000, 10000, 6000}; }
PitchIdentificationRecipe recipe() { return {1, 0.04f, 10000, 10000, 2, 10000}; }
PitchIdentificationEvidence evidence() {
    PitchIdentificationEvidence e{};
    e.maintenance = e.executing = e.enabled = e.feedback_valid = e.isolated = true;
    e.grant = {1, ec::CommandSource::maintenance};
    e.config_epoch = 1; e.reference_epoch = 1;
    e.target_rad = e.position_rad = e.native_target_rad = e.native_position_rad = 0.1f;
    e.kp = e.encoded_kp = 12.0f; e.kd = e.encoded_kd = 0.4f;
    e.limits = {12.5f, 30.0f, 10.0f};
    return e;
}
void waveform() {
    PitchIdentification test;
    const auto e = evidence();
    CHECK(test.start(policy(), recipe(), e, {1000}));
    CHECK(test.tick(e, {1000}) == 0.0f);
    for (std::uint32_t t = 3000; t < 71000; t += 2000) {
        const auto value = test.tick(e, {t});
        const auto elapsed = t-1000;
        const float expected = elapsed < 10000 || elapsed >= 50000 ? 0.0f :
            ((elapsed-10000)/10000 % 2 == 0 ? 0.04f : -0.04f);
        CHECK(std::fabs(value-expected) < 1e-6f);
    }
    CHECK(test.state() == PitchIdentificationState::complete);
    CHECK(!test.active() && !test.latched());
    CHECK(!test.start(policy(), recipe(), e, {75000})); // same run cannot restart
}
void boundaries_and_wrap() {
    PitchIdentification test;
    auto r = recipe();
    r.cycles = 0; CHECK(!test.start(policy(), r, evidence(), {0}));
    r = recipe(); r.amplitude_nm = 2; CHECK(!test.start(policy(), r, evidence(), {0}));
    r = recipe(); r.half_period_us = 0xffffffffu; CHECK(!test.start(policy(), r, evidence(), {0}));
    r = recipe(); r.amplitude_nm = std::numeric_limits<float>::quiet_NaN();
    CHECK(!test.start(policy(), r, evidence(), {0}));
    constexpr std::uint32_t start = 0xfffff000u;
    CHECK(test.start(policy(), recipe(), evidence(), {start}));
    for (std::uint32_t t=0; t<=60000; t+=2000) { static_cast<void>(test.tick(evidence(), {start+t})); }
    CHECK(test.state() == PitchIdentificationState::complete);
}
void fault_latch() {
    for (unsigned change=0; change<10; ++change) {
        PitchIdentification test;
        auto e = evidence();
        CHECK(test.start(policy(), recipe(), e, {1000}));
        static_cast<void>(test.tick(e, {1000}));
        switch (change) {
        case 0: e.enabled=false; break;
        case 1: e.maintenance=false; break;
        case 2: e.feedback_age_us=11000; break;
        case 3: e.position_rad=0.5f; break;
        case 4: e.velocity_rad_s=0.6f; break;
        case 5: e.kp=13; break;
        case 6: ++e.reference_epoch; break;
        case 7: ++e.grant.generation; break;
        case 8: e.target_rad=0.11f; break;
        case 9: e.isolated=false; break;
        }
        CHECK(test.tick(e, {3000}) == 0.0f);
        CHECK(test.latched());
        CHECK(test.tick(evidence(), {5000}) == 0.0f && test.latched());
        CHECK(!test.start(policy(), recipe(), evidence(), {5000}));
        CHECK(!test.reset(false, true));
        CHECK(!test.reset(true, false));
        CHECK(test.reset(true, true));
        CHECK(!test.start(policy(), recipe(), evidence(), {5000}));
        auto r=recipe(); r.run_id=2;
        CHECK(test.start(policy(), r, evidence(), {5000}));
    }
    PitchIdentification gap;
    CHECK(gap.start(policy(), recipe(), evidence(), {1000}));
    CHECK(gap.tick(evidence(), {8000}) == 0.0f && gap.latched());
    PitchIdentification torque;
    auto e = evidence();
    CHECK(torque.start(policy(), recipe(), e, {1000}));
    e.native_position_rad -= .1f;
    CHECK(torque.tick(e, {3000}) == 0.0f && torque.latched());
}
SentinelConfig fixture(bool available=true) {
    auto config = synthetic_config();
    config.independent_motor_bench = true;
    for (auto& motor : config.dji) { motor.installed=false; }
    config.serial.reset();
    config.timing.target_ttl = {100};
    config.dm[0].bus = 2;
    config.dm[0].can_id = 0x01;
    config.dm[0].master_id = 0x011;
    config.dm[0].mit_kp=12.0f; config.dm[0].mit_kd=.4f;
    if (available) { config.pitch_identification=policy(); }
    return config;
}
namespace raw = ec::sentinel::recording;
// 连续采集流控制 (与主机同一 stream_request 合同): 失败返回 0xff, 其余为协议错误码.
std::uint8_t stream_start(raw::Recorder& recorder, std::uint32_t run, std::uint32_t duration_us, ec::MicroTick at) {
    std::array<std::uint8_t,12> request{1,1,3,6};
    std::array<std::uint8_t,raw::stream_packet_bytes> response{};
    raw::put32(request,4,run); raw::put32(request,8,duration_us);
    if (recorder.stream_request(request,response,at)!=raw::reply_header) { return 0xff; }
    return response[2];
}
std::uint8_t stream_stop(raw::Recorder& recorder, std::uint32_t run, ec::MicroTick at) {
    std::array<std::uint8_t,12> request{1,2,0,0};
    std::array<std::uint8_t,raw::stream_packet_bytes> response{};
    raw::put32(request,4,run);
    if (recorder.stream_request(request,response,at)!=raw::reply_header) { return 0xff; }
    return response[2];
}
std::size_t stream_drain(raw::Recorder& recorder, ec::MicroTick at) {
    std::array<std::uint8_t,raw::stream_packet_bytes> packet{};
    std::size_t count{};
    if (!recorder.stream_packet(packet,at,count) || !count) { return 0; }
    recorder.stream_consume(count);
    return count;
}
PitchIdentificationPolicy capture_policy() {
    auto value = policy();
    value.max_duration_us = 4000000; // 允许 3.1s 长窗用例
    value.require_capture = true;
    return value;
}
PitchIdentificationPolicy deployment_policy() {
    auto value = capture_policy();
    value.deployment_opt_in = true;
    return value;
}
PitchIdentificationRecipe short_recipe() { return {1, 0.04f, 10000, 10000, 2, 10000}; } // 60 ms
// 3.1 s: 基线 10ms + 14 个 110ms 半周期 + 收尾 10ms.
PitchIdentificationRecipe long_recipe() { return {1, 0.04f, 110000, 10000, 14, 10000}; }
SentinelConfig capture_fixture() {
    auto config = fixture();
    config.pitch_identification = capture_policy();
    return config;
}
SentinelConfig deployment_fixture(const PitchIdentificationPolicy& value) {
    auto config = fixture();
    config.kind = ConfigKind::real_deployment;
    config.dm[0].origin = Origin::user_confirmed; // 部署受众拒绝合成来源; 只改来源标注, 不改任何数值
    config.pitch_identification = value;
    return config;
}

struct Rig {
    SentinelConfig config;
    SentinelApp app;
    ec::sentinel::host::SoftwareBus bus1{}, bus2{};
    SentinelExecutor exec;
    ec::sentinel::host::SimPlant plant{};
    AppInputs inputs{};
    // 仅作为记录汇: 采集门槛必须显式 exec.bind_pitch_recorder,
    // 短窗夹具不因“顺手挂了个记录器”而错误地改变激励条件.
    ec::sentinel::recording::Recorder* recorder{};
    ec::Tick now{1};
    explicit Rig(bool available=true) : Rig(fixture(available)) {}
    explicit Rig(SentinelConfig cfg, ConfigAudience audience = ConfigAudience::offline_test)
        : config(cfg), app(config, audience, StartupPolicy::maintenance_only),
          exec(config, audience,bus1,bus2) {
        inputs.link.locally_ready=true; inputs.link.maintenance_link_alive=true;
        inputs.link.maintenance_request=true; inputs.link.maintenance_session_epoch=1;
        plant.dm.position_rad=.1f;
        plant.dm_reports_enabled=false;
        step(1); target(); step(2);
        plant.dm_reports_enabled=true;
        step(3); step(4);
        CHECK(exec.pitch_enable_confirmed());
    }
    void target(float value=.1f) {
        TargetSnapshot packet;
        packet.domains[domain_index(Domain::pitch)]=DomainSetpoint{ec::control::ControlMode::position,value};
        CHECK(app.accept_targets(packet,now,Source::maintenance)==AcceptResult::accepted);
    }
    void step(std::uint32_t ms) {
        now={ms}; bus1.clear(); bus2.clear();
        plant.emit_feedback(config,[&](std::uint8_t bus,const ec::transport::CanFrame& frame,ec::Tick at){
            exec.route(bus,frame,at,ec::MicroTick{at.ms*1000u});
            if (recorder) { recorder->can(bus,frame,{at.ms*1000u},{at.ms*1000u},0); }
        },now);
        inputs.feedback[domain_index(Domain::pitch)]=exec.joint_feedback(Domain::pitch,now);
        inputs.faults=exec.health(now);
        app.update(now,inputs);
        exec.step(now,.001f,app,ec::MicroTick{ms*1000u});
        if (recorder) {
            if (const auto record=exec.trace(Domain::pitch,now,app)) { recorder->push(*record,{ms*1000u}); }
        }
        plant.integrate(config,bus1,bus2,.001f);
    }
};
void execution_chain() {
    Rig disabled(false);
    CHECK(!disabled.exec.pitch_identification_available());
    CHECK(!disabled.exec.start_pitch_identification(recipe(),disabled.now,disabled.app));
    Rig rig;
    CHECK(rig.exec.start_pitch_identification(recipe(),rig.now,rig.app));
    bool positive=false,negative=false;
    for (std::uint32_t ms=5;ms<70;++ms) {
        rig.target(); rig.step(ms);
        const auto frame=rig.bus2.last_with_id(*rig.config.dm[0].can_id);
        const auto decoded=frame ? ec::sentinel::host::decode_dm_mit_command(*frame,*rig.config.dm[0].limits) : std::nullopt;
        if (decoded) { positive |= decoded->torque_nm>.02f; negative |= decoded->torque_nm<-.02f; }
        CHECK(rig.bus1.size()==0);
        const auto tr = rig.exec.trace(Domain::pitch, rig.now, rig.app);
        CHECK(tr && tr->kind == ec::sentinel::recording::Kind::control && tr->stream == domain_index(Domain::pitch));
        CHECK(std::bit_cast<float>(tr->data[0]) == 0.1f);
        CHECK(std::bit_cast<float>(tr->data[5]) > 0.0f);
        if (rig.exec.pitch_identification().active()) { CHECK(tr->data[6] & (1u << 29)); }
        CHECK(tr->data[7] == static_cast<std::uint32_t>(ec::drivers::DjiGroupStatus::accepted));
    }
    CHECK(positive && negative);
    CHECK(rig.exec.pitch_identification().state()==PitchIdentificationState::complete);
    CHECK(rig.exec.stats().dm_disable_sent==0 && rig.exec.pitch_enable_confirmed());
    const auto tr_done = rig.exec.trace(Domain::pitch, rig.now, rig.app);
    CHECK(tr_done && !(tr_done->data[6] & (1u << 29)));
    CHECK(std::fabs(std::bit_cast<float>(tr_done->data[4])) < 0.005f);

    Rig tx_fail;
    CHECK(tx_fail.exec.start_pitch_identification(recipe(), tx_fail.now, tx_fail.app));
    tx_fail.target();
    tx_fail.bus2.refuse = true;
    tx_fail.step(5);
    CHECK(tx_fail.exec.pitch_identification().latched());
    CHECK(tx_fail.exec.pitch_identification().fault() == PitchIdentificationFault::transmit);
    const auto tr_fail = tx_fail.exec.trace(Domain::pitch, tx_fail.now, tx_fail.app);
    CHECK(tr_fail && (tr_fail->data[6] & (1u << 30)));
    CHECK(tr_fail->data[7] == static_cast<std::uint32_t>(ec::drivers::DjiGroupStatus::busy));
    CHECK(tr_fail->arrived_us == tr_fail->sampled_us); // failed MIT must not reuse prior success time
    CHECK(tr_fail->flags & 2u); // no successful MIT submission timestamp

    for (unsigned fault=0;fault<4;++fault) {
        Rig broken;
        CHECK(broken.exec.start_pitch_identification(recipe(),broken.now,broken.app));
        if (fault==0) { CHECK(broken.exec.set_dm_mit_gains(13,.4f)); broken.step(5); }
        if (fault==1) { broken.target(.12f); broken.step(5); }
        if (fault==2) { broken.plant.dm.emitting=false; broken.step(20); }
        if (fault==3) { broken.inputs.link.maintenance_request=false; broken.step(5); }
        CHECK(broken.exec.pitch_identification().latched());
        const auto stopped_trace=broken.exec.trace(Domain::pitch,broken.now,broken.app);
        CHECK(stopped_trace && (stopped_trace->flags & 2u));
        CHECK(std::isnan(std::bit_cast<float>(stopped_trace->data[4])));
        const auto enabled=broken.exec.stats().dm_enable_sent;
        broken.plant.dm.emitting=true;
        for (std::uint32_t ms=21;ms<35;++ms) { broken.step(ms); }
        CHECK(broken.exec.pitch_identification().latched());
        CHECK(broken.exec.stats().dm_enable_sent==enabled);
        CHECK(!broken.exec.reset_pitch_identification(broken.now,broken.app));

        if (fault == 0) {
            TargetSnapshot clear_target{};
            CHECK(broken.app.accept_targets(clear_target, broken.now, Source::maintenance) == AcceptResult::accepted);
            broken.plant.dm_reports_enabled = false;
            broken.step(36);
            CHECK(broken.exec.reset_pitch_identification(broken.now, broken.app));
            CHECK(broken.exec.pitch_identification().state() == PitchIdentificationState::idle);
            CHECK(!broken.exec.pitch_identification().latched());
            CHECK(!broken.exec.start_pitch_identification(recipe(), broken.now, broken.app));
            CHECK(broken.exec.set_dm_mit_gains(12.0f, 0.4f));
            broken.target(0.1f); broken.step(37);
            broken.plant.dm_reports_enabled = true;
            broken.target(0.1f); broken.step(38);
            broken.target(0.1f); broken.step(39);
            CHECK(broken.exec.pitch_enable_confirmed());
            auto r2 = recipe(); r2.run_id = 2;
            CHECK(broken.exec.start_pitch_identification(r2, broken.now, broken.app));
            CHECK(broken.exec.pitch_identification().active());
        }
    }
}
void capture_start_gate() {
    // 1) 策略要求采集但完全未绑定记录器: 拒绝启动.
    {
        Rig rig(capture_fixture());
        CHECK(rig.exec.pitch_identification_available());
        CHECK(!rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
    }
    // 2) 已绑定但从未开启任何采集流 (missing).
    {
        Rig rig(capture_fixture());
        raw::Recorder recorder;
        rig.recorder = &recorder;
        rig.exec.bind_pitch_recorder(&recorder);
        CHECK(!rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
    }
    // 3) 流存在但 run 与配方不同 (wrong run).
    {
        Rig rig(capture_fixture());
        raw::Recorder recorder;
        rig.recorder = &recorder;
        rig.exec.bind_pitch_recorder(&recorder);
        CHECK(stream_start(recorder, 2, 1000000u, {4000}) == 0);
        CHECK(!rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
    }
    // 4) 窗口短于激励本身 (short window).
    {
        Rig rig(capture_fixture());
        raw::Recorder recorder;
        rig.recorder = &recorder;
        rig.exec.bind_pitch_recorder(&recorder);
        CHECK(stream_start(recorder, 1, 50000u, {4000}) == 0);
        CHECK(!rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
    }
    // 5) 窗口恰等于激励时长, 仍缺 2000us 收尾余量 (insufficient).
    {
        Rig rig(capture_fixture());
        raw::Recorder recorder;
        rig.recorder = &recorder;
        rig.exec.bind_pitch_recorder(&recorder);
        CHECK(stream_start(recorder, 1, 60000u, {4000}) == 0);
        CHECK(!rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
    }
    // 6) 剩余窗口随时间缩小时也会失效: 62000us 够用, 但延后 1ms 后不再够.
    {
        Rig rig(capture_fixture());
        raw::Recorder recorder;
        rig.recorder = &recorder;
        rig.exec.bind_pitch_recorder(&recorder);
        CHECK(stream_start(recorder, 1, 62000u, {4000}) == 0);
        CHECK(recorder.pitch_stream_ready(1, 62000u, {4000}));
        CHECK(!recorder.pitch_stream_ready(1, 62000u, {5000}));
        rig.now = {5};
        CHECK(!rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
    }
    // 7) 同 run 就绪且余量足够才放行.
    {
        Rig rig(capture_fixture());
        raw::Recorder recorder;
        rig.recorder = &recorder;
        rig.exec.bind_pitch_recorder(&recorder);
        CHECK(stream_start(recorder, 1, 62000u, {4000}) == 0);
        CHECK(rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
        CHECK(rig.exec.pitch_identification().active());
    }
}
void capture_streaming() {
    // 3.1s 200Hz 连续采集 + 每周期人工刷目标 + 每 10ms 排空队列, 全程不得因采集故障中止.
    Rig rig(capture_fixture());
    raw::Recorder recorder;
    rig.recorder = &recorder;
    rig.exec.bind_pitch_recorder(&recorder);
    const auto recipe = long_recipe();
    const auto stream_duration = static_cast<std::uint32_t>(recipe.duration_us()) + 2000u + 1000000u;
    CHECK(stream_start(recorder, recipe.run_id, stream_duration, {4000}) == 0);
    CHECK(rig.exec.start_pitch_identification(recipe, rig.now, rig.app));
    bool positive = false, negative = false;
    std::size_t drained = 0;
    for (std::uint32_t ms = 5; ms <= 3120; ++ms) {
        rig.target();
        rig.step(ms);
        const auto frame = rig.bus2.last_with_id(*rig.config.dm[0].can_id);
        const auto decoded = frame ? ec::sentinel::host::decode_dm_mit_command(*frame, *rig.config.dm[0].limits) : std::nullopt;
        if (decoded) { positive |= decoded->torque_nm > .02f; negative |= decoded->torque_nm < -.02f; }
        if (ms % 10 == 0) { drained += stream_drain(recorder, {ms*1000u}); }
    }
    CHECK(positive && negative);
    CHECK(rig.exec.pitch_identification().state() == PitchIdentificationState::complete);
    CHECK(!rig.exec.pitch_identification().latched());
    // 两流各 200Hz: 3.1s 内每流约 620 条, 排空总量必须远超 512 容量.
    CHECK(drained > 400);
}
void capture_overflow_and_latch() {
    Rig rig(capture_fixture());
    raw::Recorder recorder;
    rig.recorder = &recorder;
    rig.exec.bind_pitch_recorder(&recorder);
    CHECK(stream_start(recorder, long_recipe().run_id, 5200000u, {4000}) == 0);
    CHECK(rig.exec.start_pitch_identification(long_recipe(), rig.now, rig.app));
    std::uint32_t fault_ms = 0;
    for (std::uint32_t ms = 5; ms <= 2000 && !rig.exec.pitch_identification().latched(); ++ms) {
        rig.target(); rig.step(ms); fault_ms = ms;
    }
    CHECK(rig.exec.pitch_identification().latched());
    CHECK(rig.exec.pitch_identification().fault() == PitchIdentificationFault::capture);
    CHECK(static_cast<std::uint8_t>(PitchIdentificationFault::capture) == 11);
    // 同周期不得提交 MIT: 既无提交时间也无进给量, 仅保留故障锁存位.
    const auto fault_trace = rig.exec.trace(Domain::pitch, rig.now, rig.app);
    CHECK(fault_trace && (fault_trace->flags & 2u));
    CHECK(fault_trace && std::isnan(std::bit_cast<float>(fault_trace->data[4])));
    CHECK(fault_trace && (fault_trace->data[6] & (1u << 30)));
    // 故障同周期已走出真实撕输出路径: 发 disable 且不再声称已使能.
    CHECK(rig.exec.stats().dm_disable_sent >= 1);
    CHECK(!rig.exec.pitch_enable_confirmed());
    const auto commands = rig.exec.stats().dm_command_sent;
    const auto enables = rig.exec.stats().dm_enable_sent;
    // 继续刷目标也无法恢复: 不自动重新使能, 不再提交 MIT.
    for (std::uint32_t ms = fault_ms+1; ms <= fault_ms+40; ++ms) { rig.target(); rig.step(ms); }
    CHECK(rig.exec.pitch_identification().latched());
    CHECK(rig.exec.stats().dm_command_sent == commands);
    CHECK(rig.exec.stats().dm_enable_sent == enables);
    CHECK(!rig.exec.pitch_identification().active());
}
void capture_stop_and_replace() {
    // 采集中断 (stop) 当周期即锁存.
    {
        Rig rig(capture_fixture());
        raw::Recorder recorder;
        rig.recorder = &recorder;
        rig.exec.bind_pitch_recorder(&recorder);
        CHECK(stream_start(recorder, 1, 5200000u, {4000}) == 0);
        CHECK(rig.exec.start_pitch_identification(long_recipe(), rig.now, rig.app));
        for (std::uint32_t ms = 5; ms <= 20; ++ms) { rig.target(); rig.step(ms); stream_drain(recorder, {ms*1000u}); }
        CHECK(rig.exec.pitch_identification().active());
        CHECK(stream_stop(recorder, 1, {21000}) == 0);
        const auto commands = rig.exec.stats().dm_command_sent;
        rig.target(); rig.step(21);
        CHECK(rig.exec.pitch_identification().latched());
        CHECK(rig.exec.pitch_identification().fault() == PitchIdentificationFault::capture);
        CHECK(rig.exec.stats().dm_command_sent == commands); // 当周期无 MIT
        CHECK(rig.exec.stats().dm_disable_sent >= 1);
        CHECK(!rig.exec.pitch_enable_confirmed());
        const auto trace = rig.exec.trace(Domain::pitch, rig.now, rig.app);
        CHECK(trace && (trace->flags & 2u) && std::isnan(std::bit_cast<float>(trace->data[4])));
    }
    // 采集被新 run 替换 (replace): 执行器仍绑定旧 run, 必须判为 capture 而非继续激励.
    {
        Rig rig(capture_fixture());
        raw::Recorder recorder;
        rig.recorder = &recorder;
        rig.exec.bind_pitch_recorder(&recorder);
        CHECK(stream_start(recorder, 1, 5200000u, {4000}) == 0);
        CHECK(rig.exec.start_pitch_identification(long_recipe(), rig.now, rig.app));
        for (std::uint32_t ms = 5; ms <= 20; ++ms) { rig.target(); rig.step(ms); stream_drain(recorder, {ms*1000u}); }
        CHECK(stream_stop(recorder, 1, {21000}) == 0);
        while (stream_drain(recorder, {21000}) != 0) {}
        CHECK(stream_start(recorder, 2, 5200000u, {21000}) == 0);
        CHECK(recorder.pitch_stream_healthy(2, {21000}));
        CHECK(!recorder.pitch_stream_healthy(1, {21000}));
        rig.target(); rig.step(22);
        CHECK(rig.exec.pitch_identification().latched());
        CHECK(rig.exec.pitch_identification().fault() == PitchIdentificationFault::capture);
        const auto commands = rig.exec.stats().dm_command_sent;
        for (std::uint32_t ms = 23; ms <= 40; ++ms) { rig.target(); rig.step(ms); stream_drain(recorder, {ms*1000u}); }
        CHECK(rig.exec.stats().dm_command_sent == commands);
        CHECK(rig.exec.pitch_identification().latched());
    }
}
void capture_reset_and_new_run() {
    Rig rig(capture_fixture());
    raw::Recorder recorder;
    rig.recorder = &recorder;
    rig.exec.bind_pitch_recorder(&recorder);
    CHECK(stream_start(recorder, 1, 5200000u, {4000}) == 0);
    CHECK(rig.exec.start_pitch_identification(long_recipe(), rig.now, rig.app));
    for (std::uint32_t ms = 5; ms <= 20; ++ms) { rig.target(); rig.step(ms); stream_drain(recorder, {ms*1000u}); }
    CHECK(stream_stop(recorder, 1, {21000}) == 0);
    rig.target(); rig.step(21);
    CHECK(rig.exec.pitch_identification().fault() == PitchIdentificationFault::capture);
    // 目标仍在时无复位资格.
    CHECK(!rig.exec.reset_pitch_identification(rig.now, rig.app));
    TargetSnapshot clear{};
    CHECK(rig.app.accept_targets(clear, rig.now, Source::maintenance) == AcceptResult::accepted);
    // 目标已撕, 但负载仍上报使能: 采集中断本身不等于安全复位条件.
    rig.step(22);
    CHECK(!rig.exec.reset_pitch_identification(rig.now, rig.app));
    // 只有显式收到新鲜失能反馈 (由真实帧字段携带, 不是内部标志) 才允许复位.
    rig.plant.dm_reports_enabled = false;
    rig.step(23);
    CHECK(rig.exec.reset_pitch_identification(rig.now, rig.app));
    CHECK(!rig.exec.pitch_identification().latched());
    CHECK(rig.exec.pitch_identification().state() == PitchIdentificationState::idle);
    CHECK(!rig.exec.start_pitch_identification(long_recipe(), rig.now, rig.app)); // 旧 run 不可重放
    // 复位后第一周期仍按上一周期的拒绝撒输出, 之后才重新请求使能并等待反馈确认.
    rig.target(0.1f); rig.step(24);
    rig.plant.dm_reports_enabled = true;
    rig.target(0.1f); rig.step(25);
    rig.target(0.1f); rig.step(26);
    CHECK(rig.exec.pitch_enable_confirmed());
    // 旧 run 的采集流已冻结: 用错误 run 停止必须被拒, 然后用 run 2 开启新采集流.
    CHECK(stream_stop(recorder, 2, {26000}) != 0);
    CHECK(stream_start(recorder, 2, 5200000u, {26000}) == 0);
    auto next = long_recipe();
    next.run_id = 2;
    CHECK(rig.exec.start_pitch_identification(next, rig.now, rig.app));
    CHECK(rig.exec.pitch_identification().active());
    CHECK(rig.exec.pitch_identification().run_id() == 2);
}
void deployment_opt_in_availability() {
    const auto opt_in = deployment_policy();
    {
        struct Check {
            SentinelConfig config;
            ec::sentinel::host::SoftwareBus bus1{}, bus2{};
            SentinelExecutor exec;
            explicit Check(SentinelConfig cfg) : config(cfg),
                exec(config, ConfigAudience::deployment, bus1, bus2) {}
        };
        // 完整 opt-in + 精确拓扑/几何: 可用.
        {
            Check probe(deployment_fixture(opt_in));
            CHECK(probe.exec.assembled());
            CHECK(probe.exec.pitch_identification_available());
        }
        // 旧默认: 策略字段全部为 false, 部署受众一律不可用.
        {
            Check probe(fixture());
            CHECK(!probe.exec.assembled() || !probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(deployment_policy());
            config.pitch_identification->deployment_opt_in = false;
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(deployment_policy());
            config.pitch_identification->require_capture = false;
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(opt_in);
            config.pitch_identification.reset();
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        // 拓扑不符: 总线/控制 ID/主 ID/缺失 MIT 量程均拒绝.
        {
            auto config = deployment_fixture(opt_in);
            config.dm[0].bus = 1;
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(opt_in);
            config.dm[0].can_id = 0x02;
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(opt_in);
            config.dm[0].master_id = 0x012;
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(opt_in);
            config.dm[0].limits.reset();
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        // 几何缺失或非法: 零位/下限缺失或非有限值, 以及策略包络超出关节范围.
        {
            auto config = deployment_fixture(opt_in);
            config.joints[joint_index(Domain::pitch)].zero_offset_rad.reset();
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(opt_in);
            config.joints[joint_index(Domain::pitch)].min_rad.reset();
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(opt_in);
            config.joints[joint_index(Domain::pitch)].zero_offset_rad =
                std::numeric_limits<float>::quiet_NaN();
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
        {
            auto config = deployment_fixture(opt_in);
            config.joints[joint_index(Domain::pitch)] =
                JointGeometryConfig{Domain::pitch, 0.0f, -0.2f, 0.3f};
            Check probe(config);
            CHECK(!probe.exec.pitch_identification_available());
        }
    }
}
void deployment_capture_start() {
    // 真实部署类别下 opt-in 路径可以启动, 但采集门槛仍然生效 (可用不等于可跳采集).
    Rig rig(deployment_fixture(deployment_policy()), ConfigAudience::deployment);
    CHECK(rig.exec.pitch_identification_available());
    CHECK(!rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
    raw::Recorder recorder;
    rig.recorder = &recorder;
    rig.exec.bind_pitch_recorder(&recorder);
    CHECK(stream_start(recorder, 1, 62000u, {4000}) == 0);
    CHECK(rig.exec.start_pitch_identification(short_recipe(), rig.now, rig.app));
    CHECK(rig.exec.pitch_identification().active());
}

void raw_fixture() {
    namespace raw=ec::sentinel::recording;
    Rig rig;
    raw::Recorder recorder;
    rig.recorder=&recorder;
    constexpr std::uint32_t run=101, start_us=4000, duration_us=100000;
    std::array<std::uint8_t,12> request{1,1,3,6};
    std::array<std::uint8_t,224> response{};
    raw::put32(request,4,run); raw::put32(request,8,duration_us);
    CHECK(recorder.request(request,response,{start_us})==raw::reply_header && response[2]==0);
    CHECK(rig.exec.start_pitch_identification(recipe(),rig.now,rig.app));
    for (std::uint32_t ms=5;ms<=105;++ms) { rig.target(); rig.step(ms); }
    CHECK(rig.exec.pitch_identification().state()==PitchIdentificationState::complete);
    CHECK(rig.exec.stats().dm_disable_sent==0);
    std::printf("{\"source\":\"APP_MIT_SIMULATION_ONLY\",\"hardware_ready\":false,\"layout_id\":%u,\"run\":%u,\"start_us\":%u,\"excitation_run_id\":1,\"control_domain\":6,\"window_requested_us\":%u}\n",
        raw::layout_id,run,start_us,duration_us);
    request[1]=3;
    std::uint32_t index=0;
    do {
        raw::put32(request,8,index);
        const auto size=recorder.request(request,response,{105000});
        CHECK(size>=raw::reply_header && response[2]==0 && response[3]!=1);
        const auto count=response[9];
        if (!count) { break; }
        for (std::size_t record=0;record<count;++record) {
            for (std::size_t byte=0;byte<raw::record_bytes;++byte) {
                std::printf("%02x",static_cast<unsigned>(response[raw::reply_header+record*raw::record_bytes+byte]));
            }
            std::puts("");
        }
        index+=count;
    } while (true);
    CHECK(index>100 && index<raw::capacity);
}
}
int main(int argc,char** argv) {
    if (argc==2 && std::string_view{argv[1]}=="--raw-pitch-fixture") { raw_fixture(); return 0; }
    waveform(); boundaries_and_wrap(); fault_latch(); execution_chain();
    capture_start_gate(); capture_streaming(); capture_overflow_and_latch();
    capture_stop_and_replace(); capture_reset_and_new_run();
    deployment_opt_in_availability(); deployment_capture_start();
    std::puts("pitch identification contracts passed");
}
