// 当前接线/实际PID的底盘+云台组合契约. 合成负载只验证数据与闭环组合, 不替代实车稳定性.
#include "board_app_config.hpp"
#include "gm6020_bench_velocity.hpp"
#include "sentinel_cycle.hpp"
#include "serial_plant.hpp"
#include "raw_record.hpp"
#include "sentinel_link.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
using namespace ec;
using namespace ec::sentinel::app;
using namespace ec::sentinel::host;
namespace board=ec::sentinel::board;
using drivers::Dr16Switch;
constexpr auto main_i=domain_index(Domain::main_yaw);
std::FILE* wire_evidence{};
std::FILE* validation_evidence{};
void check(bool ok,const char* expr,int line) {
    if(!ok) { std::fprintf(stderr,"integration line %d: %s\n",line,expr); std::_Exit(3); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)),#__VA_ARGS__,__LINE__)
bool close(float a,float b,float e=1e-5f) { return std::fabs(a-b)<e; }
RemoteMappingConfig mapping() {
    auto m=board::deployment_mapping();
    m.chassis_vx_max_mps=m.chassis_vy_max_mps=2;
    m.chassis_wz_max_rad_s=3;m.spin_wz_rad_s=0.8f;
    m.yaw_rate_max_rad_s=1;m.fire_yaw_rate_rad_s=0.5f;m.pitch_rate_max_rad_s=0.4f;
    return m;
}
void config_contract() {
    const auto c=board::deployment_config();
    const auto report=validate(c,ConfigAudience::deployment);
    for(std::size_t i=0;i<7;++i) { CHECK(report.domains[i]==ConfigState::complete); }
    CHECK(c.serial.has_value()==board::integration_serial_geometry.has_value());
    CHECK(!c.serial || c.serial->valid()); // 现场回填后沿用同一契约, 不把缺参状态锁进测试.
    CHECK(board::deployment_mapping().spin_wz_rad_s==board::integration_spin_rad_s);
    constexpr std::array<std::uint8_t,4> ids{3,1,4,2};
    constexpr std::array<int,4> directions{-1,1,-1,1};
    for(std::size_t i=0;i<6;++i) {
        const auto& s=c.dji[i];
        CHECK(s.max_axis_velocity_rad_s==0 && s.output_limit_raw==16384);
        CHECK(close(*s.current_limit_a,i<4 ? 20.0f : 3.0f));
        CHECK(s.bus==(i<5 ? 1 : 2));
        if(i<4) { CHECK(s.device_id==ids[i] && s.direction==directions[i]); }
    }
    // 2026-11-11 main速度环改为归一化直接给定0.4/0.35/0 = 电流口径1.2/1.05/0.
    CHECK(close(c.dji[4].velocity_pid->kp,0.4f));
    CHECK(close(c.dji[4].velocity_pid->ki,0.35f));
    CHECK(close(c.dji[4].velocity_pid->kp*3,1.2f));
    CHECK(close(c.dji[4].velocity_pid->ki*3,1.05f));
    // 2026-11-11 main位置环配套缩放为6.3/0.45; 位置环尚未单独闭环验证(15s验证旁路外环).
    CHECK(close(c.dji[4].position_pid->kp,6.3f));
    CHECK(close(c.dji[4].position_pid->ki,0.45f));
    CHECK(close(c.dji[5].velocity_pid->kp*3,0.4f));
    CHECK(close(c.dji[5].velocity_pid->ki*3,7.5f));
    CHECK(close(c.dji[5].position_pid->kp,32.0f));
    CHECK(close(c.dji[5].position_pid->kd,0.1f));
    CHECK(c.dm[0].bus==2 && c.dm[0].can_id==1 && c.dm[0].master_id==0x11);
    CHECK(c.dm[0].mit_kp==40 && c.dm[0].mit_kd==1);
    CHECK(c.dm[0].gravity_ff_nm==0.8f);
    CHECK(c.joints[0].commissioning_unbounded && !c.joints[0].min_rad && !c.joints[0].max_rad);
    for(std::size_t i=6;i<9;++i) { CHECK(!c.dji[i].velocity_pid); }
    control::Pid outer{*c.dji[4].position_pid};
    // 位置Kp=6.3时用误差10rad得到63rad/s, 超过旧15rad/s外环限幅值, 证明外环未被限幅.
    CHECK(*outer.step(10,0,0.001f)>56.0f);
    control::VelocityLoop inner{{*c.dji[4].velocity_pid,{50},0,0.005f,0}};
    const auto grant=inner.arm(CommandSource::remote,true,true);CHECK(grant);
    const auto output=inner.step({10},.001f,Command<float>{100,{10},{50},*grant},Sample<float>{0,{10},Validity::valid});
    CHECK(output.status==control::VelocityStatus::active && *output.output==1);
    auto invalid=*c.dji[4].velocity_pid;invalid.limit_output=false;
    CHECK(!inner.set_pid_config(invalid)); // 协议电流输出仍有界.
    const auto model=*synthetic_config().serial;
    const auto q=reachable_heading(model,{},-30.0f,0.1f);
    CHECK(q && std::fabs(*q+30.0f)<3.142f);
    CHECK(!reachable_heading(model,{},0,1,0.0f,0.0f));
}
void mapper_contract() {
    RemoteMapper mapper{mapping(),{100}};
    input::Dr16Snapshot r{};r.switch_left=Dr16Switch::up;r.switch_right=Dr16Switch::up;
    std::array<std::optional<float>,domain_count> actual{};
    actual[main_i]=0.3f;actual[5]=0.1f;actual[6]=0.0f;
    auto out=mapper.step(r,{100},.01f,actual,1.57079633f);
    CHECK(out.targets.chassis && close(out.targets.chassis->wz_rad_s,.8f));
    r.axis[3]=.5f;r.axis[0]=-.5f;
    out=mapper.step(r,{101},.01f,actual,1.57079633f);
    CHECK(close(out.targets.chassis->vx_mps,0) && close(out.targets.chassis->vy_mps,1));
    CHECK(close(out.targets.domains[main_i]->value,.3f));
    CHECK(out.targets.domains[5]->value>.1f && out.targets.domains[5]->frame==YawFrame::joint);
    r.ch4=600; // 拨轮失效/偏置都不影响新布局.
    out=mapper.step(r,{102},.01f,actual,1.57079633f);CHECK(close(out.targets.chassis->wz_rad_s,.8f));
    actual[main_i].reset();out=mapper.step(r,{103},.01f,actual,0.0f);
    CHECK(!out.targets.chassis); // 无惯性参考不继续自旋.
    r.switch_right=Dr16Switch::mid;r.switch_left=Dr16Switch::mid;
    out=mapper.step(r,{104},.01f,actual,0.0f);CHECK(!out.targets.chassis);
    r.switch_right=Dr16Switch::down;
    out=mapper.step(r,{105},.01f,actual,0.0f);CHECK(out.manual_stop && !out.targets.chassis);
    CHECK(remote_source_selection(RemoteLayout::chassis_gimbal_integration,Dr16Switch::mid,Dr16Switch::up)==Dr16Switch::mid);
    CHECK(remote_source_selection(RemoteLayout::chassis_gimbal_integration,Dr16Switch::down,Dr16Switch::up)==Dr16Switch::mid);
}
void capture_contract() {
    namespace cap=ec::sentinel::capture;
    cap::Collector collector{board::current_harness};
    transport::CanFrame fire{0x20a,8,{1,0,0,0,0,0,25,0}};
    collector.receive(2,fire,10,1000);CHECK(collector.motors()[5].valid);
    transport::CanFrame pitch{0x11,8,{0x01,0x80,0,0x80,0,0,25,25}};
    collector.receive(2,pitch,10,1000);CHECK(collector.motors()[9].valid);
    SoftwareBus bus;collector.poll(bus,10,true);
    CHECK(bus.frames.size()==1 && cap::allowed_read(bus.frames[0]));
    namespace raw=ec::sentinel::recording;
    std::array<raw::CanAddress,10> addresses{};
    for(std::size_t i=0;i<addresses.size();++i) { addresses[i]={board::current_harness[i].bus,board::current_harness[i].rx_id}; }
    raw::Recorder recorder{addresses};
    std::array<std::uint8_t,12> request{1,1,2,0};std::array<std::uint8_t,224> reply{};
    raw::put32(request,4,1);raw::put32(request,8,1000000);
    CHECK(recorder.request(request,reply,{10000}));
    recorder.can(2,fire,{11000},{11000},0);recorder.can(2,pitch,{11001},{11001},0);
    CHECK(recorder.size()==2);
}
struct Rig {
    SentinelConfig config;
    SoftwareBus bus1{},bus2{};
    SentinelApp app;SentinelExecutor executor;SentinelPerception perception;SentinelCycle cycle;
    SerialPlant plant{};
    std::array<ec::sentinel::BenchEncoderVelocity,2> velocities{};
    drivers::Dr16Sample remote{.switch_left=Dr16Switch::mid,.switch_right=Dr16Switch::up};
    Tick now{1000};std::uint32_t sequence{};bool imu_on{true};
    CycleOutcome outcome{};
    static SentinelConfig fixture() {
        auto c=board::deployment_config();c.serial=synthetic_config().serial;
        c.serial_origin=Origin::synthetic;return c;
    }
    Rig():config(fixture()),app(config,ConfigAudience::offline_test),
        executor(config,ConfigAudience::offline_test,bus1,bus2),perception(config,ConfigAudience::offline_test),
        cycle(app,executor,perception,mapping(),config.timing.remote_ttl) {
        CHECK(executor.assembled());
        plant.main_zero=*config.joints[0].zero_offset_rad;plant.fire_zero=*config.joints[1].zero_offset_rad;plant.pitch_zero=*config.joints[2].zero_offset_rad;
        plant.load.dji[4].position_rad=static_cast<float>(plant.main_zero);
        plant.load.dji[5].position_rad=static_cast<float>(plant.fire_zero);
        plant.load.dm.position_rad=static_cast<float>(plant.pitch_zero);
        for(unsigned i=0;i<80;++i) { tick(); }
        CHECK(perception.platform().validity==DomainValidity::valid);
    }
    void tick(std::optional<Tick> remote_at={}, bool publish_remote=true) {
        const MicroTick at{now.ms*1000u};
        plant.emit_feedback(config,[&](std::uint8_t bus,const transport::CanFrame& frame,Tick time) {
            executor.route(bus,frame,time,at);
            for(std::size_t i=0;i<2;++i) {
                if(bus==config.dji[i+4].bus && frame.id==0x204u+config.dji[i+4].device_id) {
                    velocities[i].receive(static_cast<std::uint16_t>((frame.data[0]<<8)|frame.data[1]),time,at);
                }
            }
        },now);
        for(std::size_t i=0;i<2;++i) { executor.set_joint_velocity_feedback(i==0 ? Domain::main_yaw : Domain::fire_yaw,velocities[i].value()); }
        CycleInputs in{};in.now=now;in.now_us=at;in.dt_s=.001f;in.locally_ready=true;
        if (publish_remote) { in.remote_sample=remote;in.remote_received_at=remote_at.value_or(now); }
        if(imu_on) {
            const auto sample=plant.sample(*config.serial,at,++sequence);
            in.imu_gyro_rad_s=sample.gyro;in.imu_accel_mps2=sample.accel;in.imu_sampled_at=now;
        }
        bus1.clear();bus2.clear();outcome=cycle.control(in);cycle.commit(in);
        for(std::size_t i=7;i<domain_count;++i) { CHECK(!app.snapshot().executions[i].execute); }
        if(const auto frame=bus2.last_with_id(0x1ff)) { for(auto b:frame->data) { CHECK(b==0); } }
        plant.integrate(config,bus1,bus2,.001f);++now.ms;
    }
};
// USART3中断可在控制周期now之后发布下一毫秒的样本. 不提前取代仍新鲜的旧帧,
// 否则本周期会假失联、撤输出并清保持角, 下一周期又捕获受扰姿态且重新enable pitch.
void remote_arrival_order_contract() {
    Rig rig;
    rig.remote.switch_left=Dr16Switch::up;rig.remote.switch_right=Dr16Switch::up;
    rig.plant.base_rate=.8;
    for(unsigned i=0;i<30;++i) { rig.tick(); }
    CHECK(rig.app.snapshot().executions[main_i].execute);
    CHECK(rig.app.snapshot().executions[domain_index(Domain::pitch)].execute);
    const float held=rig.app.target(Domain::main_yaw)->value;
    const auto disabled=rig.executor.stats().dm_disable_sent;
    const auto enabled=rig.executor.stats().dm_enable_sent;
    const Tick received{rig.now.ms+1};
    rig.tick(received);
    const bool early=rig.outcome.remote_frame_accepted, online=rig.outcome.remote_online;
    rig.tick(received); // 同一帧真正到达本周期时刻后可用, 时间戳不能被提前去重掉.
    for(unsigned i=0;i<10;++i) { rig.tick(); }
    const float change=rig.app.target(Domain::main_yaw)->value-held;
    std::printf("REMOTE_ARRIVAL early=%u online=%u held_change=%.9f disable=%u enable=%u\n",
        early,online,change,rig.executor.stats().dm_disable_sent-disabled,rig.executor.stats().dm_enable_sent-enabled);
    CHECK(!early && online && std::fabs(change)<1e-7f);
    CHECK(rig.executor.stats().dm_disable_sent==disabled && rig.executor.stats().dm_enable_sent==enabled);
    for(unsigned i=0;i<rig.config.timing.remote_ttl.ms+1;++i) { rig.tick({},false); }
    CHECK(!rig.outcome.remote_online && !rig.app.snapshot().executions[main_i].execute);
    CHECK(!rig.app.snapshot().executions[domain_index(Domain::pitch)].execute);
}
void identification_contract() {
    IdentificationPlan plan;
    CHECK(plan.valid());
    // 默认电流激励不再因频率升高被压成毫安级; 只有起停速度受加速度设计约束.
    float previous=0;
    for(unsigned ms=0;ms<=5000;++ms) {
        const auto p=plan.ramp(ms*.001f);
        CHECK(std::fabs(p.rate-previous)<=plan.acceleration*.0011f);previous=p.rate;
        CHECK(close(p.bias_a,.18f*p.rate,1e-6f));
    }
    float offset=0;
    for(unsigned k=0;k<plan.count;++k) {
        const auto peak=plan.scan(offset+1.25f/plan.frequencies[k]);
        CHECK(close(peak.excitation_a,.06f,2e-6f) && peak.measuring && peak.rate==plan.speed);
        CHECK(close(plan.scan(offset).excitation_a,0,1e-6f));offset+=(plan.cycles+2)/plan.frequencies[k];
    }
    CHECK(plan.scan(plan.scan_duration()+plan.ramp_s+plan.zero_s+.001f).done);
    MainYawIdentification quality;
    quality.start(plan,1,1,1,1,3.0f/16384);
    for(unsigned ms=0;ms<=20001;++ms) { static_cast<void>(quality.point({ms*1000},.216f,.001f)); }
    CHECK(quality.state==MainYawIdentification::State::aborted && quality.reason==MainYawIdentification::Reason::working_point);
    CHECK(quality.excitation_start_us==0); // 首轮真实欠速形态不能再被当作完成稳态准备.
    quality.start(plan,1,1,1,1,3.0f/16384);
    for(unsigned ms=0;ms<=7005;++ms) { static_cast<void>(quality.point({ms*1000},.85f,.001f)); }
    CHECK(quality.state==MainYawIdentification::State::running && quality.excitation_start_us>=6999000);
    plan.speed=.1f;plan.amplitude_a=.005f;plan.bias_a=.025f;plan.ramp_s=.3f;plan.zero_s=.1f;plan.settle_timeout_s=5;
    plan.count=1;plan.frequencies[0]=10;plan.cycles=3;CHECK(plan.valid());
    MainYawIdentification wrap;
    wrap.start(plan,1,1,1,1,3.0f/16384);
    CHECK(!wrap.point({0xfffffff0},0,.001f).done);CHECK(!wrap.point({1000},0,.001f).done);
    for(unsigned n=0;n<MainYawIdentification::capacity+9;++n) { wrap.append({n}); }
    CHECK(wrap.dropped==9 && wrap.consumed==9 && wrap.sample(wrap.consumed).at_us==9);

    Rig rig;ParamStore params{rig.config};
    SentinelHostSession session{rig.app,params,&rig.executor,&rig.perception,rig.config.timing.host_link_ttl};
    MainYawIdentification id;
    std::array<std::uint8_t,240> request{},reply{};
    unsigned sequence=0;
    const auto invoke=[&](MessageType type, auto body) {
        PayloadWriter w{request};w.u8(protocol_version);w.u8(static_cast<std::uint8_t>(type));
        if(type==MessageType::hello) { w.u32(123); }
        else { w.u32(session.session_id());w.u32(++sequence);body(w); }
        const auto size=session.handle({request.data(),w.size()},rig.now,reply);
        if(wire_evidence) {
            std::array<std::uint8_t,transport::uart_wire_max> bytes{};
            const auto n=transport::encode_uart_frame(reply[1],0,{reply.data(),size},bytes);
            CHECK(std::fwrite(bytes.data(),1,n,wire_evidence)==n);
        }
        return size;
    };
    CHECK(invoke(MessageType::hello,[](auto&) {})==47);
    CHECK(invoke(MessageType::identification,[](auto& w) { w.u8(2); })==12 && reply[10]==unsigned(ResultCode::unsupported));
    rig.executor.attach_identification(id);
    CHECK(invoke(MessageType::authorize,[](auto& w) { w.u8(1);w.u8(unsigned(Source::maintenance)); })==12 && reply[10]==0);
    for(unsigned i=0;i<3;++i) { rig.tick(); }
    CHECK(rig.app.authority().source()==Source::maintenance);
    TargetSnapshot targets{};
    targets.chassis=motion::PlanarTwist{};
    targets.domains[main_i]=DomainSetpoint{control::ControlMode::position,rig.perception.platform().continuous_heading,0,YawFrame::local_inertial};
    for(const auto domain:{Domain::fire_yaw,Domain::pitch}) {
        targets.domains[domain_index(domain)]=DomainSetpoint{control::ControlMode::position,rig.executor.joint_feedback(domain,rig.now).position_rad.value};
    }
    CHECK(rig.app.accept_targets(targets,rig.now,Source::maintenance)==AcceptResult::accepted);
    rig.tick();CHECK(rig.executor.identification_ready(rig.app));
    const auto start=[&] {
        return invoke(MessageType::identification,[&](auto& w) {
            w.u8(1);w.u8(2);w.f32(plan.speed);w.f32(plan.amplitude_a);w.f32(plan.bias_a);w.f32(plan.ramp_s);
            w.f32(plan.settle_timeout_s);w.f32(plan.zero_s);w.f32(plan.acceleration);
            w.u8(plan.count);w.u8(plan.cycles);for(auto f:plan.frequencies) { w.f32(f); }
        });
    };
    CHECK(start()==45 && id.state==MainYawIdentification::State::armed);
    const auto duplicate=request;const auto run=id.run_id;
    rig.tick();CHECK(id.produced==1);
    CHECK(invoke(MessageType::identification,[](auto& w) { w.u8(2); })==45);
    CHECK(session.handle({duplicate.data(),66},rig.now,reply)==45); // 状态读不覆盖start缓存, 不重置轨迹/积分/序号.
    CHECK(id.run_id==run && id.produced==1);
    transport::UartFrameDecoder decoder;std::uint32_t expected=0,total_bytes=0,uart_ready=0;bool busy=true;
    const auto send=[&](std::span<const std::uint8_t> bytes) {
        if(busy || rig.now.ms*1000<uart_ready) { return false; }
        // 真实921600 8N1线长与下一次1ms任务才能再提交, 不用无限带宽掩盖缓存不足.
        uart_ready=rig.now.ms*1000+static_cast<unsigned>(std::ceil(bytes.size()*10000000.0/921600));
        total_bytes+=bytes.size();
        if(wire_evidence) { CHECK(std::fwrite(bytes.data(),1,bytes.size(),wire_evidence)==bytes.size()); }
        for(const auto byte:bytes) {
            const auto frame=decoder.push(byte);if(!frame) { continue; }
            CHECK(frame->type==unsigned(MessageType::identification_stream));
            PayloadReader r{{frame->payload.data(),frame->size}};
            static_cast<void>(r.u8());static_cast<void>(r.u8());CHECK(r.u32()==session.session_id());CHECK(r.u32()==run);
            CHECK(r.u32()==id.start_us);CHECK(r.u32()==expected);
            const auto count=r.u8();static_cast<void>(r.u8());CHECK(r.u32()==0);CHECK(r.u8()==2);
            for(unsigned i=0;i<count;++i) {
                const auto at=r.u32();static_cast<void>(r.u32());const auto target=r.f32(),feedback=r.f32();
                CHECK(std::isfinite(feedback) && target<=plan.speed+1e-6f);
                static_cast<void>(at);static_cast<void>(r.f32());static_cast<void>(r.f32());
                const float p=r.f32(),integral=r.f32();
                const auto raw=static_cast<std::int16_t>(r.u16());static_cast<void>(r.u16());CHECK(r.u16()<5000);
                CHECK((r.u16()&7u)==7u);const float bias=r.f32(),excitation=r.f32();
                CHECK(std::fabs(raw*id.raw_to_axis_amp-p-integral-bias-excitation)<.0001f);
                ++expected;
            }
            CHECK(r.exact());
        }
        return true;
    };
    for(unsigned ms=0;ms<6500;++ms) {
        CHECK(rig.app.accept_targets(targets,rig.now,Source::maintenance)==AcceptResult::accepted);
        rig.tick();busy=ms<30;
        if(id.state==MainYawIdentification::State::running) {
            const auto frame=rig.bus1.last_with_id(0x1fe);CHECK(frame);
            const auto bits=static_cast<std::uint16_t>((unsigned(frame->data[6])<<8)|frame->data[7]);
            CHECK(static_cast<std::uint16_t>(id.sample(id.produced-1).command_raw)==bits);
        }
        // 普通ACK/快状态优先, 留出比默认10Hz快页更紧的额外UART占用.
        if(ms%20==0 && rig.now.ms*1000>=uart_ready) { uart_ready=rig.now.ms*1000+2700; }
        const auto consumed=id.consumed;
        static_cast<void>(service_identification(session,send));
        if(busy) { CHECK(consumed==id.consumed); }
        if(ms%100==0) { CHECK(invoke(MessageType::identification,[](auto& w) { w.u8(2); })==45); }
    }
    CHECK(id.state==MainYawIdentification::State::complete && id.dropped==0 && id.produced==expected);
    CHECK(expected>3000 && expected<6400 && total_bytes<380000 && decoder.rejected()==0);
    CHECK(rig.executor.axis(Domain::main_yaw)->output()==std::nullopt); // 不恢复旧位置目标.
    CHECK(invoke(MessageType::identification,[](auto& w) { w.u8(3); })==12 && reply[10]==unsigned(ResultCode::reference_in_use));
    targets.domains[main_i].reset();CHECK(rig.app.accept_targets(targets,rig.now,Source::maintenance)==AcceptResult::accepted);rig.tick();
    CHECK(invoke(MessageType::identification,[](auto& w) { w.u8(3); })==45 && !id.engaged());
    targets.domains[main_i]=DomainSetpoint{control::ControlMode::position,rig.perception.platform().continuous_heading,0,YawFrame::local_inertial};
    CHECK(rig.app.accept_targets(targets,rig.now,Source::maintenance)==AcceptResult::accepted);rig.tick();CHECK(start()==45);rig.tick();
    // 原TTL仍终止试验, 反馈/遥控恢复不自动重启; 停止与参考变化同样失去执行条件.
    for(unsigned i=0;i<rig.config.timing.target_ttl.ms+3;++i) { rig.tick(); }
    CHECK(id.state==MainYawIdentification::State::aborted && id.reason==MainYawIdentification::Reason::execution_lost);
    CHECK(rig.app.accept_targets(targets,rig.now,Source::maintenance)==AcceptResult::accepted);rig.tick();
    CHECK(id.state==MainYawIdentification::State::aborted && !rig.executor.axis(Domain::main_yaw)->enabled());
    targets.domains[main_i].reset();CHECK(rig.app.accept_targets(targets,rig.now,Source::maintenance)==AcceptResult::accepted);rig.tick();
    CHECK(invoke(MessageType::identification,[](auto& w) { w.u8(3); })==45);
    targets.domains[main_i]=DomainSetpoint{control::ControlMode::position,rig.perception.platform().continuous_heading,0,YawFrame::local_inertial};
    CHECK(rig.app.accept_targets(targets,rig.now,Source::maintenance)==AcceptResult::accepted);rig.tick();CHECK(start()==45);rig.tick();
    rig.remote.switch_right=Dr16Switch::down;rig.tick();
    CHECK(id.state==MainYawIdentification::State::aborted && !rig.executor.axis(Domain::main_yaw)->enabled());
    CHECK(rig.app.snapshot().permission.manual_stop && !rig.app.snapshot().executions[domain_index(Domain::pitch)].execute);
    std::printf("IDENTIFICATION samples=%u wire_bytes=%u completed, bandwidth/TTL/stop/no restart passed\n",expected,total_bytes);
}
// 完整默认轨迹跨过多圈, 保持旧位置意图但只执行速度内环, 不触发错误可达性/参考切换.
void identification_multiturn_contract() {
    Rig r;MainYawIdentification id;r.executor.attach_identification(id);
    r.app.set_maintenance_session(true,true,7);
    for(unsigned i=0;i<30;++i) { r.tick(); }
    CHECK(r.app.authority().source()==Source::maintenance);
    TargetSnapshot target{};target.chassis=motion::PlanarTwist{};
    for(const auto domain:{Domain::fire_yaw,Domain::pitch}) {
        target.domains[domain_index(domain)]=DomainSetpoint{control::ControlMode::position,r.executor.joint_feedback(domain,r.now).position_rad.value};
    }
    IdentificationPlan plan;
    for(const auto sign:{1,-1}) {
        const float begin=r.executor.joint_feedback(Domain::main_yaw,r.now).position_rad.value;
        target.domains[main_i]=DomainSetpoint{control::ControlMode::position,r.perception.platform().continuous_heading,0,YawFrame::local_inertial};
        CHECK(r.app.accept_targets(target,r.now,Source::maintenance)==AcceptResult::accepted);r.tick();
        plan.speed=static_cast<float>(sign);
        id.start(plan,7,static_cast<unsigned>(sign+2),r.app.config_epoch(),r.app.reference_epoch(),3.0f/16384);
        for(unsigned n=0;n<static_cast<unsigned>(plan.duration()*1000)+3;++n) {
            CHECK(r.app.accept_targets(target,r.now,Source::maintenance)==AcceptResult::accepted);r.tick();
            CHECK(id.state!=MainYawIdentification::State::aborted);
            CHECK(r.app.snapshot().main_yaw.validity==DomainValidity::valid);
            if(n%5==0) { id.consumed=id.produced; }
        }
        CHECK(id.state==MainYawIdentification::State::complete && id.dropped==0 && id.produced>60000);
        const float travel=r.executor.joint_feedback(Domain::main_yaw,r.now).position_rad.value-begin;
        CHECK(sign*travel>20);
        std::printf("IDENTIFICATION_FULL direction=%d samples=%u travel_rad=%.3f no reference loss\n",sign,id.produced,travel);
        id.clear();
    }
    // 完整执行链中模拟负载无法起转: 不进入扫频, 只结束该有限试验, 不重捕获旧目标.
    r.plant.load.load.accel_per_amp=0;
    plan.speed=.1f;plan.ramp_s=.5f;plan.settle_timeout_s=2;plan.amplitude_a=.005f;plan.bias_a=.02f;
    CHECK(plan.valid());
    id.start(plan,7,9,r.app.config_epoch(),r.app.reference_epoch(),3.0f/16384);
    for(unsigned n=0;n<2600;++n) {
        CHECK(r.app.accept_targets(target,r.now,Source::maintenance)==AcceptResult::accepted);r.tick();
        if(n%5==0) { id.consumed=id.produced; }
    }
    CHECK(id.state==MainYawIdentification::State::aborted && id.reason==MainYawIdentification::Reason::working_point);
    CHECK(id.excitation_start_us==0 && !r.executor.axis(Domain::main_yaw)->enabled());
    const auto final=r.bus1.last_with_id(0x1fe);CHECK(final && final->data[6]==0 && final->data[7]==0);
}
void validation_contract() {
    float integral=0,previous=0;
    for(unsigned ms=0;ms<=15000;++ms) {
        const auto p=IdentificationPlan::validation(ms*.001f);
        CHECK(p.bias_a==0 && p.excitation_a==0 && std::fabs(p.rate-previous)<.000787f);
        integral+=p.rate*.001f;previous=p.rate;
    }
    CHECK(std::fabs(integral)<1e-5f && IdentificationPlan::validation(15).done);
    for(unsigned fault=0;fault<9;++fault) {
        Rig r;ParamStore params{r.config};MainYawIdentification id;r.executor.attach_identification(id);
        SentinelHostSession session{r.app,params,&r.executor,&r.perception,r.config.timing.host_link_ttl};
        std::array<std::uint8_t,240> request{},reply{};unsigned seq=0;
        const auto emit=[&](std::span<const std::uint8_t> bytes) {
            if(fault==0 && validation_evidence) { CHECK(std::fwrite(bytes.data(),1,bytes.size(),validation_evidence)==bytes.size()); }
            return true;
        };
        const auto call=[&](MessageType type,auto body) {
            PayloadWriter w{request};w.u8(protocol_version);w.u8(unsigned(type));
            if(type==MessageType::hello) { w.u32(123); }
            else { w.u32(session.session_id());w.u32(++seq);body(w); }
            const auto n=session.handle({request.data(),w.size()},r.now,reply);
            std::array<std::uint8_t,transport::uart_wire_max> bytes{};
            const auto count=transport::encode_uart_frame(reply[1],0,{reply.data(),n},bytes);emit({bytes.data(),count});
            return n;
        };
        CHECK(call(MessageType::hello,[](auto&){})==47);
        // 能力在授权前可读, 未配套增益的start仍拒绝, 不先试动探测版本.
        CHECK(call(MessageType::identification,[](auto&w){w.u8(4);})==115);
        CHECK(call(MessageType::authorize,[](auto&w){w.u8(1);w.u8(unsigned(Source::maintenance));})==12 && reply[10]==0);
        for(unsigned i=0;i<3;++i) { r.tick(); }
        // 2026-11-11 默认配置已等于该固定候选, 故先故意写错一项以保留"增益不匹配则拒绝start"的覆盖.
        CHECK(call(MessageType::param_write,[&](auto&w){w.u16(0x402);w.f32(.2f);})==24 && reply[10]==0);
        CHECK(!r.executor.validation_gains_match());
        CHECK(call(MessageType::identification,[](auto&w){w.u8(5);})==12 && reply[10]==unsigned(ResultCode::invalid_value));
        const std::array<std::uint16_t,6> ids{0x401,0x402,0x403,0x408,0x409,0x40a};
        const std::array<float,6> candidate{.4f,.35f,0,6.3f,.45f,0};
        for(unsigned k=0;k<6;++k) {
            CHECK(call(MessageType::param_write,[&](auto&w){w.u16(ids[k]);w.f32(candidate[k]);})==24 && reply[10]==0);
        }
        CHECK(r.executor.validation_gains_match());
        TargetSnapshot target{};target.chassis=motion::PlanarTwist{};
        target.domains[main_i]=DomainSetpoint{control::ControlMode::position,r.perception.platform().continuous_heading,0,YawFrame::local_inertial};
        for(const auto domain:{Domain::fire_yaw,Domain::pitch}) {
            target.domains[domain_index(domain)]=DomainSetpoint{control::ControlMode::position,r.executor.joint_feedback(domain,r.now).position_rad.value};
        }
        CHECK(r.app.accept_targets(target,r.now,Source::maintenance)==AcceptResult::accepted);r.tick();
        CHECK(call(MessageType::identification,[](auto&w){w.u8(5);})==45);
        const auto duplicate=request;
        CHECK(call(MessageType::identification,[](auto&w){w.u8(4);})==115);
        CHECK(session.handle({duplicate.data(),11},r.now,reply)==45); // 只读元数据不覆盖变更去重.
        CHECK(call(MessageType::param_write,[](auto&w){w.u16(0x402);w.f32(.2f);})==24 && reply[10]==unsigned(ResultCode::reference_in_use));
        float last_i=0;std::uint32_t last_at=0,seen=0;bool saw_zero_with_i=false;
        if(fault==1) { r.plant.load.load.accel_per_amp=0; }
        for(unsigned ms=0;ms<15005;++ms) {
            if(!(fault==4 && ms>10)) { static_cast<void>(r.app.accept_targets(target,r.now,Source::maintenance)); }
            if(ms==100) {
                if(fault==2) { r.plant.load.dji[4].velocity_rad_s=.5f; }
                if(fault==3) { auto p=r.executor.axis(Domain::main_yaw)->active_velocity_pid();p.kp=.3f;CHECK(r.executor.axis(Domain::main_yaw)->set_velocity_gains(p)); }
                if(fault==5) { r.remote.switch_right=Dr16Switch::down; }
                if(fault==6) { r.app.invalidate_domain_reference(Domain::main_yaw); }
                if(fault==7) { r.bus1.refuse=true; }
            }
            if(fault==8 && ms==15000) { r.bus1.refuse=true; } // 最后一帧停机命令被拒不能报完整结束.
            r.tick();
            if(id.produced>seen) {
                const auto&s=id.sample(id.produced-1);
                CHECK(s.bias_a==0 && s.excitation_a==0);
                if(!(s.flags&32)) {
                    if(last_at) { CHECK(std::fabs(s.i-last_i-1.05f*(s.reference-s.feedback)*(s.at_us-last_at)*1e-6f)<1e-6f); }
                    if((s.flags>>8)==4 && std::fabs(s.i)>.001f) { saw_zero_with_i=true; }
                    CHECK(std::fabs(s.command_raw*id.raw_to_axis_amp-s.p-s.i)<.0001f);
                } else { CHECK(s.command_raw==0); }
                last_i=s.i;last_at=s.at_us;seen=id.produced;
            }
            static_cast<void>(service_identification(session,emit));
            if(ms%100==0) { CHECK(call(MessageType::identification,[](auto&w){w.u8(2);})==45); }
            if(fault && !id.active()) { break; }
        }
        static_cast<void>(service_identification(session,emit));
        if(!fault) {
            CHECK(id.state==MainYawIdentification::State::complete && id.produced==15001 && saw_zero_with_i && id.dropped==0);
            CHECK(call(MessageType::identification,[](auto&w){w.u8(4);})==115);
        } else {
            CHECK(id.state==MainYawIdentification::State::aborted);
            const auto expected=fault==1 ? MainYawIdentification::Reason::current_stop : fault==2 ? MainYawIdentification::Reason::speed_stop :
                fault==3 ? MainYawIdentification::Reason::parameters_changed : (fault==7 || fault==8) ? MainYawIdentification::Reason::tx_failure : MainYawIdentification::Reason::execution_lost;
            CHECK(id.reason==expected);
            if(fault<3) { const auto&s=id.sample(id.produced-1);CHECK((s.flags&32) && s.command_raw==0); }
        }
        r.bus1.refuse=false;r.tick();CHECK(!r.executor.axis(Domain::main_yaw)->enabled());
        const auto stop=r.bus1.last_with_id(0x1fe);CHECK(stop && raw_slot(*stop,3)==0);
        CHECK(id.engaged()); // 不自动clear或追赶旧位置, 恢复旧参数在主机撤main意图后做.
    }
    MainYawIdentification wrap;IdentificationPlan p;p.profile=IdentificationPlan::Profile::velocity_validation;
    wrap.start(p,1,1,1,1,3.0f/16384);
    CHECK(!wrap.point({0xfffffff0},0,.001f).done);
    CHECK(wrap.point({15000000u-16},0,.001f).done);
    std::puts("VALIDATION15 waveform/protocol/PID-continuity/current-stop/speed-stop/TTL/RC/reference/TX passed");
}
void closed_loop_contract() {
    Rig rig;
    rig.remote.switch_left=Dr16Switch::up;rig.remote.switch_right=Dr16Switch::mid;
    for(unsigned i=0;i<8;++i) { rig.tick(); }
    TargetSnapshot fast{};fast.chassis=motion::PlanarTwist{30,20,3};
    CHECK(rig.app.accept_targets(fast,rig.now,Source::remote)==AcceptResult::accepted);
    CHECK(close(rig.app.adopted_twist()->vx_mps,30));
    const auto wheels=motion::omni_wheel_speeds(*omni_drive(rig.config),*fast.chassis);
    CHECK(wheels && close(rig.app.target(Domain::chassis_lf)->value,(*wheels)[0]));
    rig.remote.ch3=330;
    bool wheel_frame=false;
    for(unsigned i=0;i<6;++i) {
        rig.tick();
        if(const auto frame=rig.bus1.last_with_id(0x200)) {
            CHECK(*raw_slot(*frame,0)>0 && *raw_slot(*frame,1)>0);
            CHECK(*raw_slot(*frame,2)<0 && *raw_slot(*frame,3)<0);
            wheel_frame=true;
        }
    }
    CHECK(wheel_frame);rig.remote.ch3=0;
    rig.remote.switch_left=Dr16Switch::up;rig.remote.switch_right=Dr16Switch::up;
    for(unsigned i=0;i<100;++i) { rig.tick(); }
    CHECK(rig.app.snapshot().permission.source==Source::remote);
    CHECK(rig.app.snapshot().executions[main_i].execute);
    CHECK(rig.app.target(Domain::main_yaw)->frame==YawFrame::local_inertial);
    rig.plant.base_rate=.8;
    float peak{};unsigned active{};
    const auto initial=rig.plant.heading(*rig.config.serial);
    for(unsigned i=0;i<20000;++i) {
        rig.tick();
        const float e=std::remainder(rig.plant.heading(*rig.config.serial)-initial,6.2831853f);
        if(i>1000) { peak=std::max(peak,std::fabs(e)); }
        if(rig.app.snapshot().executions[main_i].execute) { ++active; }
    }
    std::printf("spin active=%u/20000 heading_peak=%.6f joint=%.6f\n",active,peak,rig.executor.joint_feedback(Domain::main_yaw,rig.now).position_rad.value);
    CHECK(active==20000 && peak<0.35f);
    CHECK(rig.executor.joint_feedback(Domain::main_yaw,rig.now).position_rad.value<-12.56f);
    const float fire_before=rig.app.target(Domain::fire_yaw)->value;
    rig.remote.ch0=-200;
    for(unsigned i=0;i<250;++i) { rig.tick(); }
    CHECK(rig.app.target(Domain::fire_yaw)->value>fire_before+.02f);
    rig.remote.ch0=0;
    for(unsigned i=0;i<500;++i) { rig.tick(); }
    CHECK(rig.app.snapshot().executions[5].execute && rig.executor.main_yaw_stats().frame==YawFrame::local_inertial);
    rig.imu_on=false;
    for(unsigned i=0;i<120;++i) { rig.tick(); }
    CHECK(!rig.app.adopted_twist());
    rig.remote.switch_right=Dr16Switch::down;rig.tick();
    CHECK(rig.app.snapshot().permission.manual_stop);
    for(const auto& e:rig.app.snapshot().executions) { CHECK(!e.execute); }
    rig.remote.switch_right=Dr16Switch::up;rig.remote.switch_left=Dr16Switch::mid;
    for(unsigned i=0;i<10;++i) { rig.tick(); }
    CHECK(rig.app.snapshot().permission.source==Source::none);
}
}
int main(int argc,char** argv) {
    if(argc==3 && std::strcmp(argv[1],"--wire")==0) { wire_evidence=std::fopen(argv[2],"wb");CHECK(wire_evidence); }
    else if(argc==3 && std::strcmp(argv[1],"--validation-wire")==0) { validation_evidence=std::fopen(argv[2],"wb");CHECK(validation_evidence); }
    else { CHECK(argc==1); }
    config_contract();mapper_contract();capture_contract();remote_arrival_order_contract();identification_contract();identification_multiturn_contract();validation_contract();closed_loop_contract();std::puts("integration contracts pass"); }
