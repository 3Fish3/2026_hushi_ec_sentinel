// N4 共用 APP 周期 + 真实电流编码 + 正向传感器的闭环. 不是实车动力学验收.
#include "sentinel_cycle.hpp"
#include "sentinel_param_apply.hpp"
#include "sentinel_link.hpp"
#include "serial_plant.hpp"
#include <cstdio>
#include <cstdlib>

namespace {
using namespace ec;
using namespace ec::sentinel::app;
using namespace ec::sentinel::host;
constexpr auto main_i=domain_index(Domain::main_yaw);
MicroTick simulated_clock_value{};
MicroTick simulated_clock() noexcept { return simulated_clock_value; } // 受控模型时钟, 不测 CPU 耗时.
void check(bool ok,const char* expr,int line) {
    if(!ok) { std::fprintf(stderr,"inertial line %d: %s\n",line,expr); std::_Exit(3); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)),#__VA_ARGS__,__LINE__)
Vector3 unit(Vector3 a) { const float n=std::sqrt(dot(a,a)); for(auto& x:a) x/=n; return a; }
SentinelConfig fixture() {
    auto c=synthetic_config();
    c.runtime=RuntimeTiming{};
    c.serial->main_axis=unit({0.04f,0.08f,1});
    c.serial->fire_axis=unit({0.12f,-0.06f,1});
    c.serial->pitch_axis=unit({0.14f,1,0.05f});
    c.serial->forward=unit({1,0.04f,0.05f});
    c.serial->install=estimation::rotation_zyx(0.2f,-0.3f,0.5f);
    c.serial->gyro_bias_sensor={0.013f,-0.024f,0.007f};
    return c;
}
struct Rig {
    SentinelConfig config;
    SoftwareBus bus1{},bus2{};
    SentinelApp app;
    SentinelExecutor executor;
    SentinelPerception perception;
    ParamStore params;
    std::optional<SentinelHostSession> session{};
    transport::UartFrameDecoder decoder{};
    ReplyMux replies{};
    std::vector<std::uint8_t> wire_rx{};
    std::array<std::uint8_t, protocol_payload_max> last_fast{};
    std::size_t last_fast_size{}, max_rx_bytes{}, max_pending{};
    std::uint32_t wire_sequence{};
    bool uart_busy{}, duplicate_imu{};
    float cycle_dt{0.001f};
    drivers::Dr16Sample remote_input{.switch_left=drivers::Dr16Switch::down,.switch_right=drivers::Dr16Switch::mid};
    drivers::Dr16Stream remote_stream{};
    std::optional<drivers::Dr16Sample> remote_decoded{};
    Tick remote_at{};
    bool raw_remote{},remote_primed{};
    SentinelCycle cycle;
    SerialPlant plant{};
    Tick now{1000};
    std::uint32_t us{0xffffa000u}; // 同一闭环也跨微秒回绕.
    std::uint32_t sequence{};
    std::optional<SerialPlant::ImuSample> pending_imu{};
    bool imu_on{true}, encoder_on{true};
    float scoring_yaw_offset{}; // 重力初始化的 yaw 原点任意; 只用于统一真值评分参考, 不回灌 APP.
    CycleInputs inputs{};
    explicit Rig(SentinelConfig cfg=fixture(), bool timed=false, RemoteMappingConfig mapping={}): config(cfg),app(config,ConfigAudience::offline_test),
        executor(config,ConfigAudience::offline_test,bus1,bus2,timed ? simulated_clock : nullptr),
        perception(config,ConfigAudience::offline_test),params(config),
        cycle(app,executor,perception,mapping,config.timing.remote_ttl) {
        plant.base_roll=0.2; plant.base_pitch=-0.28;
        plant.load.dm_reports_enabled=false;
        for(int i=0;i<12;++i) tick();
        CHECK(app.snapshot().permission.source==Source::host);
        CHECK(perception.platform().validity==DomainValidity::valid);
        scoring_yaw_offset=perception.platform().continuous_heading-plant.heading(*config.serial);
    }
    [[nodiscard]] float true_local_heading() const { return plant.heading(*config.serial)+scoring_yaw_offset; }
    void sense() {
        inputs={}; inputs.now=now; inputs.now_us=MicroTick{us}; inputs.dt_s=cycle_dt;
        inputs.locally_ready=true; inputs.host_link_alive=inputs.host_request=true;
        inputs.host_session_epoch=1;
        if (raw_remote) {
            if(!remote_primed) { const std::uint8_t sync{}; remote_stream.push({&sync,1}); (void)remote_stream.on_idle(); remote_primed=true; }
            if(!remote_decoded || now.ms-remote_at.ms>=14) {
                std::array<std::uint8_t,18> bytes{};
                const std::array channels{remote_input.ch0,remote_input.ch1,remote_input.ch2,remote_input.ch3};
                std::uint64_t bits{};
                for(std::size_t i=0;i<4;++i) bits|=(static_cast<std::uint64_t>(channels[i]+1024)&0x7ffu)<<(11*i);
                for(std::size_t i=0;i<6;++i) bytes[i]=static_cast<std::uint8_t>(bits>>(8*i));
                bytes[5]=static_cast<std::uint8_t>((bytes[5]&15)|(static_cast<unsigned>(remote_input.switch_left)<<6)|(static_cast<unsigned>(remote_input.switch_right)<<4));
                const auto ch4=static_cast<std::uint16_t>(remote_input.ch4+1024);
                bytes[16]=static_cast<std::uint8_t>(ch4); bytes[17]=static_cast<std::uint8_t>(ch4>>8);
                remote_stream.push(bytes); remote_decoded=remote_stream.on_idle(); CHECK(remote_decoded); remote_at=now;
            }
            inputs.remote_sample=remote_decoded; inputs.remote_received_at=remote_at;
        } else { inputs.remote_sample=remote_input; inputs.remote_received_at=now; }
        inputs.aux.launch_permit_valid=inputs.aux.launch_permit=true;
        inputs.aux.launch_sampled_at=now; inputs.aux.launch_ttl={50};
        plant.emit_feedback(config,[&](std::uint8_t bus,const transport::CanFrame& f,Tick at) {
            const auto main_id=drivers::dji_protocol(config.dji[4].mode)->rx_base+config.dji[4].device_id;
            if(encoder_on || bus!=config.dji[4].bus || f.id!=main_id) executor.route(bus,f,at,MicroTick{us});
        },now);
        const auto sample=plant.sample(*config.serial,{us},++sequence);
        if(imu_on && pending_imu) {
            const auto& old=*pending_imu;
            drivers::ImuSamplingTimes times;
            times.gyro_valid=times.gyro_updated=times.accel_valid=times.accel_updated=true;
            times.gyro=times.gyro_estimated=times.accel=old.at;
            times.gyro_sequence=times.accel_sequence=old.sequence-(duplicate_imu ? 1u : 0u);
            inputs.imu_times=times; inputs.imu_gyro_rad_s=old.gyro; inputs.imu_accel_mps2=old.accel;
        }
        pending_imu=sample;
    }
    void tick(const std::optional<TargetSnapshot>& target=std::nullopt, std::uint32_t next_step_us=1000) {
        simulated_clock_value={us};
        sense();
        if (session) {
            max_rx_bytes=std::max(max_rx_bytes,wire_rx.size());
            CHECK(wire_rx.size()<=512);
            unsigned processed{};
            std::array<std::uint8_t,protocol_payload_max> response{};
            for (const auto byte:wire_rx) {
                if (const auto frame=decoder.push(byte)) {
                    CHECK(++processed<=4);
                    session->set_frame_rejects(decoder.rejected());
                    session->set_tx_diagnostics(replies.diagnostics());
                    const auto size=session->handle({frame->payload.data(),frame->size},now,response);
                    CHECK(size); CHECK(replies.enqueue(*frame,{response.data(),size}));
                }
            }
            wire_rx.clear();
            const auto link=session->link_state(now);
            inputs.host_link_alive=link.alive && session->role_source()==Source::host;
            inputs.host_request=link.request && session->role_source()==Source::host;
            inputs.host_session_epoch=link.epoch;
        }
        if(target) CHECK(app.accept_targets(*target,now,Source::host)==AcceptResult::accepted);
        bus1.clear(); bus2.clear();
        static_cast<void>(cycle.control(inputs)); cycle.commit(inputs);
        max_pending=std::max(max_pending,static_cast<std::size_t>(replies.diagnostics()[9]));
        (void)replies.service(now,[&](std::span<const std::uint8_t> data) {
            if (uart_busy) { return false; }
            transport::UartFrameDecoder receive;
            for (const auto byte:data) {
                if (const auto frame=receive.push(byte); frame && frame->type==static_cast<std::uint8_t>(MessageType::status_snapshot) && frame->payload[10]==0) {
                    last_fast_size=frame->size; std::copy_n(frame->payload.begin(),frame->size,last_fast.begin());
                }
            }
            CHECK(receive.rejected()==0); return true;
        });
        cycle_dt=static_cast<float>(next_step_us)*1e-6f;
        plant.integrate(config,bus1,bus2,cycle_dt);
        ++now.ms; us+=next_step_us;
    }
    template<class Fill> void wire_request(MessageType type, Fill&& fill, bool damaged=false) {
        std::array<std::uint8_t,protocol_payload_max> payload{};
        PayloadWriter w{payload}; w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(type));
        if (type!=MessageType::hello) { w.u32(session->session_id()); w.u32(++wire_sequence); }
        fill(w);
        std::array<std::uint8_t,transport::uart_wire_max> encoded{};
        const auto size=transport::encode_uart_frame(static_cast<std::uint8_t>(type),static_cast<std::uint16_t>(wire_sequence),
            {payload.data(),w.size()},encoded);
        CHECK(size);
        if (damaged) { encoded[size/2]^=1; }
        wire_rx.insert(wire_rx.end(),encoded.begin(),encoded.begin()+size);
    }
    void wire_target(const TargetSnapshot& target, bool damaged=false) {
        wire_request(MessageType::targets,[&](PayloadWriter& w) {
            w.u32(app.config_epoch()); w.u32(app.reference_epoch());
            std::uint8_t count{};
            for (const auto& d:target.domains) { if(d) ++count; }
            w.u8(count);
            for (std::size_t i=0;i<domain_count;++i) {
                const auto& d=target.domains[i]; if(!d) continue;
                w.u8(static_cast<std::uint8_t>(i)); w.u8(static_cast<std::uint8_t>(d->mode));
                w.u8(static_cast<std::uint8_t>(d->frame)); w.f32(d->value); w.u16(d->age_ms);
            }
            w.u8(target.chassis ? 1 : 0);
            if(target.chassis) { w.f32(target.chassis->vx_mps); w.f32(target.chassis->vy_mps); w.f32(target.chassis->wz_rad_s); w.u16(target.chassis_age_ms); }
        },damaged);
    }
    TargetSnapshot targets(float heading,float fire=0,float pitch=0,YawFrame frame=YawFrame::local_inertial) const {
        TargetSnapshot t;
        t.domains[main_i]=DomainSetpoint{control::ControlMode::position,heading,0,frame};
        t.domains[domain_index(Domain::fire_yaw)]=DomainSetpoint{control::ControlMode::position,fire};
        t.domains[domain_index(Domain::pitch)]=DomainSetpoint{control::ControlMode::position,pitch};
        return t;
    }
    float current() const {
        const auto& c=config.dji[4]; const auto p=drivers::dji_protocol(c.mode);
        const auto f=bus1.last_with_id(c.device_id>4 ? p->second_tx : p->first_tx);
        CHECK(f);
        return static_cast<float>(*raw_slot(*f,(c.device_id-1)%4))/p->limit*
            *drivers::dji_current_amp_full_scale(c.mode)*static_cast<float>(*c.direction);
    }
};
// NR01: 真实 v3 请求经 Cycle 与串联传感器链, 不能把参考改动当成物理扰动.
void mechanical_alignment_contracts() {
    auto cfg=fixture();
    cfg.dji[5].rotor_per_axis=1.5f;
    cfg.serial->fire_reference=0.2f;
    cfg.serial->pitch_reference=-0.15f;
    cfg.joints[joint_index(Domain::fire_yaw)].min_rad=-1.0f;
    cfg.joints[joint_index(Domain::fire_yaw)].max_rad=1.0f;
    Rig r{cfg};
    r.session.emplace(r.app,r.params,&r.executor,&r.perception,Lifetime{200});
    r.wire_request(MessageType::hello,[](PayloadWriter& w){ w.u32(321); }); r.tick();
    r.wire_request(MessageType::authorize,[](PayloadWriter& w){ w.u8(1); w.u8(static_cast<std::uint8_t>(Source::host)); }); r.tick();
    auto target=r.targets(r.perception.platform().continuous_heading);
    target.domains[domain_index(Domain::fire_yaw)].reset();
    target.chassis=motion::PlanarTwist{0.05f,0,0};
    r.wire_target(target); r.tick();
    CHECK(r.app.target(Domain::main_yaw));
    const auto count=r.session->accepted_commands();
    const auto epoch=r.app.reference_epoch();
    const auto heading=r.perception.platform().continuous_heading;
    const auto align=[&](float angle) {
        r.wire_request(MessageType::align,[&](PayloadWriter& w) {
            w.u8(static_cast<std::uint8_t>(Domain::fire_yaw)); w.u8(0);
            w.f32(angle); w.u32(r.app.reference_epoch());
        }); r.tick();
    };
    align(0.4f); // 与配置唯一恢复的零角矛盾, 不能用任意值重命名已知参考.
    CHECK(r.session->accepted_commands()==count);
    CHECK(r.app.reference_epoch()==epoch);
    CHECK(r.app.target(Domain::main_yaw) && r.app.target(Domain::chassis_lf));
    CHECK(std::fabs(r.perception.platform().continuous_heading-heading)<0.01f);

    // 已知坐标变换走零偏参数事务: 标定姿态平移, 不改变主平台实际方向.
    r.wire_request(MessageType::param_write,[&](PayloadWriter& w) {
        w.u16(param_id(5,20)); w.f32(0.1f);
    }); r.tick();
    CHECK(std::fabs(r.app.config().serial->fire_reference-0.1f)<1e-6f);
    CHECK(r.app.target(Domain::main_yaw) && r.app.target(Domain::chassis_lf));
    for(int i=0;i<4;++i) r.tick();
    CHECK(std::fabs(r.perception.platform().continuous_heading-heading)<0.01f);

    // 扩大范围后单圈不能唯一恢复. 手动建立是参考纠正, 不是有证明的坐标平移.
    auto joint=r.app.joint_geometry(Domain::fire_yaw);
    joint.min_rad=-5.0f; joint.max_rad=5.0f;
    CHECK(r.app.set_joint_geometry(Domain::fire_yaw,joint));
    CHECK(r.executor.set_joint_geometry(Domain::fire_yaw,joint));
    const auto main_epoch=r.app.domain_reference_epoch(Domain::main_yaw);
    r.perception.set_yaw_alignment(0.2f,99);
    align(0.4f);
    CHECK(!r.app.target(Domain::main_yaw));
    CHECK(r.app.target(Domain::chassis_lf));
    CHECK(r.app.domain_reference_epoch(Domain::main_yaw)==main_epoch+1);
    CHECK(!r.perception.alignment_valid());
    CHECK(std::fabs(r.app.config().serial->fire_reference-0.1f)<1e-6f);
    for(int i=0;i<4;++i) r.tick();
    CHECK(!r.app.target(Domain::main_yaw));
    CHECK(r.perception.platform().validity==DomainValidity::valid);
    r.wire_target(r.targets(r.perception.platform().continuous_heading)); r.tick();
    CHECK(r.app.snapshot().executions[main_i].execute);
}

// NR02/T03: v3 授权/目标、真实 Cycle 与 DM 字节闭环; 拒绝后只恢复反馈不能重新使能.
void pitch_cycle_rejection_contracts() {
    Rig r;
    r.session.emplace(r.app,r.params,&r.executor,&r.perception,Lifetime{200});
    r.wire_request(MessageType::hello,[](PayloadWriter& w){ w.u32(322); }); r.tick();
    r.wire_request(MessageType::authorize,[](PayloadWriter& w){ w.u8(1); w.u8(static_cast<std::uint8_t>(Source::host)); }); r.tick();
    const auto target=r.targets(r.perception.platform().continuous_heading,0,0.3f);
    r.wire_target(target); r.tick();
    for (int i=0;i<8;++i) r.tick();
    CHECK(r.executor.stats().dm_command_sent>0);
    r.cycle_dt=0.0f; r.tick();
    const auto enables=r.executor.stats().dm_enable_sent, commands=r.executor.stats().dm_command_sent;
    const auto stopped=r.bus2.last_with_id(*r.config.dm[0].can_id);
    CHECK(stopped && stopped->data[7]==0xfd);
    CHECK(!r.bus1.last_with_id(*r.config.dm[0].can_id)); // pitch 在 CAN2
    for (int i=0;i<8;++i) {
        r.tick();
        CHECK(!r.app.target(Domain::pitch));
        CHECK(r.executor.stats().dm_enable_sent==enables && r.executor.stats().dm_command_sent==commands);
    }
    r.wire_target(target); r.tick();
    for(int i=0;i<6;++i) r.tick();
    CHECK(r.executor.stats().dm_enable_sent==enables+1);
    CHECK(r.executor.stats().dm_command_sent>commands);
}

void downstream_and_closed_loop() {
    Rig r;
    const float start=r.perception.platform().continuous_heading;
    float largest_pose_error{},largest_rate_error{};
    for(unsigned i=0;i<3000;++i) {
        const float t=i*0.001f;
        r.tick(r.targets(start,0.3f*std::sin(1.3f*t),0.15f*std::sin(1.7f*t)));
        CHECK(r.app.snapshot().executions[main_i].execute);
        CHECK(!r.app.main_yaw_degraded());
        largest_pose_error=std::max(largest_pose_error,std::fabs(r.true_local_heading()-start));
        const auto before=r.plant.platform(*r.config.serial,-1e-4),after=r.plant.platform(*r.config.serial,1e-4);
        const auto a=transform64(before,wide(r.config.serial->forward)),b=transform64(after,wide(r.config.serial->forward));
        const double true_rate=std::remainder(std::atan2(b[1],b[0])-std::atan2(a[1],a[0]),6.283185307179586)/2e-4;
        largest_rate_error=std::max(largest_rate_error,std::fabs(r.perception.platform().value.heading_rate-static_cast<float>(true_rate)));
        CHECK(std::fabs(r.current())<=*r.config.dji[4].current_limit_a+0.001f);
    }
    std::printf("downstream max heading=%.6f rate=%.6f\n",largest_pose_error,largest_rate_error);
    CHECK(largest_pose_error<0.035f); CHECK(largest_rate_error<0.15f);
    const auto generation=r.app.snapshot().permission.generation;
    r.imu_on=false; r.tick(r.targets(start)); // 单次缺样仍在有界外推窗口内.
    CHECK(r.app.snapshot().executions[main_i].frame==YawFrame::local_inertial);
    r.imu_on=true; r.bus1.refuse=true; r.tick(r.targets(start));
    r.bus1.refuse=false; r.tick(r.targets(start));
    CHECK(r.app.snapshot().permission.generation==generation);
    CHECK(r.executor.stats().group_tx_error>0);
    const auto initial_steps=r.executor.main_yaw_stats();
    const float desired=start+0.3f;
    r.plant.base_rate=0.22;
    for(unsigned i=0;i<3000;++i) r.tick(r.targets(desired,0.3f,0.15f));
    const float error=std::fabs(r.true_local_heading()-desired);
    std::printf("closed loop heading error=%.6f qmain=%.6f\n",error,r.plant.load.dji[4].position_rad);
    CHECK(error<0.05f);
    CHECK(std::fabs(r.plant.load.dji[5].position_rad-0.3f)<0.025f);
    CHECK(std::fabs(r.plant.load.dm.position_rad-0.15f)<0.006f);
    CHECK(r.plant.load.dji[4].velocity_rad_s<0); // 编码器是相对转速, 与稳住后的惯性速率不同.
    CHECK(std::fabs(r.perception.platform().value.heading_rate)<0.04f);
    const auto final=r.executor.main_yaw_stats();
    CHECK(final.outer_steps-initial_steps.outer_steps==1500);
    CHECK(final.inner_steps-initial_steps.inner_steps==3000);
    CHECK(r.perception.stats().reprime_events==0);
    CHECK(r.app.reference_epoch()==0);
    CHECK(r.executor.stats().target_rejections==0);
}
void direction_and_modes() {
    for(int direction:{-1,1}) for(float main_sign:{-1.0f,1.0f}) {
        auto cfg=fixture(); cfg.dji[4].direction=direction;
        for(auto& v:cfg.serial->main_axis) v*=main_sign;
        Rig r{cfg}; const float heading=r.perception.platform().continuous_heading;
        r.tick(r.targets(heading)); r.plant.base_rate=0.3;
        for(unsigned i=0;i<4;++i) r.tick(r.targets(heading));
        CHECK(r.current()*r.perception.platform().value.heading_gain<0); // 一次符号换算后的真实电流.
        // 机械模式不受惯性质量影响; 控制器切换不继承原惯性环的积分/保持速率.
        const float joint=r.executor.joint_feedback(Domain::main_yaw,r.now).position_rad.value;
        r.plant.base_rate=0;
        r.imu_on=false;
        for(unsigned i=0;i<15;++i) r.tick(r.targets(joint,0,0,YawFrame::joint));
        CHECK(!r.app.main_yaw_degraded());
        CHECK(r.app.snapshot().executions[main_i].frame==YawFrame::joint);
        CHECK(r.app.snapshot().executions[main_i].execute);
        const auto token=r.app.snapshot().permission.generation;
        r.app.invalidate_inertial_reference();
        CHECK(r.app.target(Domain::main_yaw)); // 重建局部惯性原点不作废机械维护意图.
        r.tick(); CHECK(r.app.snapshot().permission.generation==token);
    }
}
void degradation_and_recovery() {
    Rig r;
    const float heading=r.perception.platform().continuous_heading;
    for(unsigned i=0;i<20;++i) r.tick(r.targets(heading+0.15f));
    const auto epoch=r.app.reference_epoch();
    r.imu_on=false;
    for(unsigned i=0;i<6;++i) r.tick();
    CHECK(r.app.main_yaw_degraded());
    const auto held=r.app.snapshot().executions[main_i].adopted_target;
    CHECK(held && r.app.snapshot().executions[main_i].frame==YawFrame::joint);
    r.tick(); CHECK(r.app.snapshot().executions[main_i].adopted_target==held);
    CHECK(r.app.reference_epoch()==epoch);
    CHECK(r.app.accept_targets(r.targets(heading),r.now,Source::host)==AcceptResult::degraded);
    for(unsigned i=0;i<18;++i) r.tick();
    CHECK(!r.app.snapshot().executions[main_i].execute && std::fabs(r.current())<1e-5f);
    r.imu_on=true;
    for(unsigned i=0;i<3;++i) r.tick();
    CHECK(!r.app.main_yaw_degraded());
    CHECK(!r.app.target(Domain::main_yaw)); // 真实重建之后不恢复旧目标.
    r.tick(r.targets(r.perception.platform().continuous_heading));
    CHECK(r.app.snapshot().executions[main_i].execute);
    // DM 失联使串联重构无效, 本地正常停止不会自动冒充人工整车停止.
    r.plant.load.dm.emitting=false;
    for(unsigned i=0;i<7;++i) r.tick();
    CHECK(r.app.main_yaw_degraded());
    CHECK(!r.app.snapshot().permission.manual_stop);
    CHECK(r.app.snapshot().permission.source==Source::host);
    // 编码器也真失效: 原意图仍未到期, 仍无第三种自动兜底.
    auto cfg=fixture(); cfg.timing.target_ttl={100};
    Rig lost{cfg};
    lost.tick(lost.targets(lost.perception.platform().continuous_heading));
    lost.encoder_on=lost.imu_on=false;
    for(unsigned i=0;i<24;++i) lost.tick();
    CHECK(!lost.app.snapshot().executions[main_i].execute);
    CHECK(lost.app.snapshot().domains[main_i].fault==FaultState::reference_lost);
    CHECK(std::fabs(lost.current())<1e-5f);
}
void hot_gain_and_branch() {
    Rig r;
    const float heading=r.perception.platform().continuous_heading;
    r.tick(r.targets(heading+0.2f));
    CHECK(apply_param(r.params,r.app,&r.executor,&r.perception,param_id(4,1),0.6f,false)==ParamWriteResult::accepted);
    CHECK(std::fabs(r.executor.velocity_gain(4,0)-0.6f)<1e-6f);
    r.tick(r.targets(heading+0.2f));
    CHECK(std::fabs(r.executor.velocity_gain(4,0)-0.6f)<1e-6f);
    // 2π 等价目标不绕远; 极限外目标整包拒绝或显式不可达, 不调另一机构补偿.
    r.tick(r.targets(heading+0.2f+6.283185307f));
    CHECK(r.executor.main_yaw_stats().resolved_joint_rad<0.4f);
    for(unsigned i=0;i<30;++i) r.tick(r.targets(heading+2.0f));
    CHECK(std::fabs(r.current())<=3.001f);
    for(unsigned i=0;i<1400;++i) r.tick(r.targets(heading));
    CHECK(std::fabs(r.true_local_heading()-heading)<0.04f); // 饱和解除后不保留旧大输出.
    r.tick(TargetSnapshot{}); // 先显式撤目标退出惯性执行, 再受控对齐.
    const float before_zero=r.perception.platform().continuous_heading;
    CHECK(apply_param(r.params,r.app,&r.executor,&r.perception,param_id(5,20),0.2f,true)==ParamWriteResult::accepted);
    CHECK(std::fabs(r.app.config().serial->fire_reference+0.2f)<1e-6f);
    for(unsigned i=0;i<3;++i) r.tick();
    CHECK(r.perception.platform().validity==DomainValidity::valid);
    CHECK(std::fabs(r.perception.platform().continuous_heading-before_zero)<0.003f);
    r.perception.set_yaw_alignment(0.7f,r.app.reference_epoch()+1);
    r.app.invalidate_inertial_reference();
    r.tick(r.targets(heading+0.7f,0,0,YawFrame::odom));
    CHECK(std::fabs(r.executor.main_yaw_stats().local_target_rad-heading)<1e-5f);
    const auto trace=r.executor.trace(Domain::main_yaw,{r.now.ms-1},r.app);
    CHECK(trace && ((trace->data[6]>>25)&3)==static_cast<unsigned>(YawFrame::odom));
    CHECK(((trace->data[6]>>27)&3)==static_cast<unsigned>(YawFrame::odom));
}
void protocol_composition_metrics() {
    auto config=fixture();
    // 本用例闭环跑到饱和: 夹具的轮轴上界需与100 rad/s转子上界、N=19一致, 不调大设备拒绝界.
    for(std::size_t i=0;i<4;++i) config.dji[i].max_axis_velocity_rad_s=4.0f;
    Rig r{config,true};
    r.session.emplace(r.app,r.params,&r.executor,&r.perception,r.config.timing.host_link_ttl);
    r.wire_request(MessageType::hello,[](PayloadWriter& w){w.u32(1234);}); r.tick();
    r.wire_request(MessageType::authorize,[](PayloadWriter& w){w.u8(1);w.u8(static_cast<std::uint8_t>(Source::host));}); r.tick();
    const auto generation=r.app.snapshot().permission.generation;
    const auto references=r.perception.local_epoch(), changes=r.app.stats().source_changes;
    const auto start_stats=r.executor.stats();
    std::array<std::uint32_t,8> computations{},submitted{};
    for (std::size_t i=0;i<8;++i) if(const auto s=r.executor.group_stats(i)) {computations[i]=s->computations;submitted[i]=s->submitted;}
    const auto pitch_start=r.executor.pitch_stats();
    const float heading=r.perception.platform().continuous_heading+0.15f;
    unsigned tracking{},chassis_tracking{}; std::array<std::uint32_t,3> maximum_ages{};
    std::uint32_t maximum_output_age{}; bool scaled{};
    r.plant.base_rate=0.12;
    for (unsigned i=0;i<2000;++i) {
        if (i%10==0) {
            auto target=r.targets(heading,0.15f*std::sin(i*0.0013f),0.1f*std::cos(i*0.0017f));
            target.chassis=motion::PlanarTwist{i>=800 && i<1200 ? 10.0f : 0.05f,0,0};
            r.wire_target(target,i==250); // 一个损坏目标包, 不续旧租期也不撤全局来源.
        }
        if (i%10==1) r.wire_request(MessageType::status,[](PayloadWriter& w){w.u8(0);});
        if (i%100==2) r.wire_request(MessageType::status,[](PayloadWriter& w){w.u8(1);});
        if (i%200==3) r.wire_request(MessageType::diagnostics,[](PayloadWriter& w){w.u8(2);});
        if (i==600) r.wire_request(MessageType::param_write,[](PayloadWriter& w){w.u16(param_id(4,1));w.f32(0.6f);});
        r.uart_busy=i>=500 && i<700;
        r.bus1.refuse=i==450;
        r.duplicate_imu=i==350;
        r.tick(std::nullopt,i%2 ? 1100 : 900); // 合法调度抖动, 不补旧帧; 完整微秒回绕由感知用例另行覆盖.
        const auto& state=r.app.snapshot();
        tracking += state.executions[main_i].execute && state.executions[main_i].frame==YawFrame::local_inertial;
        scaled |= state.twist_scale<1;
        chassis_tracking+=state.executions[0].execute;
        CHECK(state.permission.generation==generation);
        CHECK(r.perception.local_epoch()==references);
        for (std::size_t j=0;j<3;++j) maximum_ages[j]=std::max(maximum_ages[j],r.perception.platform().ages_us[j]);
        for (std::size_t j=0;j<8;++j) if(const auto s=r.executor.group_stats(j);s && s->last_result==drivers::DjiGroupStatus::accepted) {
            maximum_output_age=std::max(maximum_output_age,s->last_submitted.us-s->last_computed.us);
        }
        if(!r.bus1.refuse) CHECK(std::fabs(r.current())<=*r.config.dji[4].current_limit_a+0.001f);
    }
    r.bus1.refuse=r.uart_busy=r.duplicate_imu=false;
    CHECK(tracking==2000 && chassis_tracking==2000 && scaled);
    CHECK(r.decoder.rejected()==1);
    // CAN1 只剩底盘组与主 yaw 的 GM 首组各一次 busy (发射 yaw 已迁至 CAN2).
    CHECK(r.executor.stats().group_tx_error-start_stats.group_tx_error==2);
    CHECK(r.perception.stats().rejected_duplicate==1);
    CHECK(r.app.stats().source_changes==changes && r.perception.stats().reprime_events==0);
    CHECK(r.max_pending==4 && r.replies.diagnostics()[8]>0 && r.replies.diagnostics()[6]>0);
    CHECK(r.executor.velocity_gain(4,0)==0.6f);
    CHECK(std::fabs(r.true_local_heading()-heading)<0.06f);
    CHECK(r.last_fast_size==214);
    PayloadReader fast{{r.last_fast.data(),r.last_fast_size}};
    (void)fast.u16(); CHECK(fast.u32()==r.session->session_id()); (void)fast.u32(); CHECK(fast.u8()==0);
    CHECK(fast.u32()==r.app.config_epoch()); CHECK(fast.u32()==r.app.reference_epoch());
    std::printf("N6 controlled: tracking=%u/2000 rx_peak=%zuB tx_peak=%zu slots ages_us=%u/%u/%u output_model_age=%u us source_changes=%u reprime=%u crc=%u\n",
        tracking,r.max_rx_bytes,r.max_pending,maximum_ages[0],maximum_ages[1],maximum_ages[2],maximum_output_age,
        r.app.stats().source_changes-changes,r.perception.stats().reprime_events,r.decoder.rejected());
    for (std::size_t i=0;i<8;++i) if(const auto s=r.executor.group_stats(i)) {
        const auto count=s->computations-computations[i];
        std::printf("N6 group bus=%u id=%03x computed=%u submitted=%u max_dt=%.6f\n",s->bus,s->id,count,s->submitted-submitted[i],s->max_dt_s);
        CHECK(count==(s->id==0x1fe || s->id==0x2fe ? 2000u : 1000u));
    }
    CHECK(r.executor.pitch_stats().computations-pitch_start.computations==1000);
    std::printf("N6 pitch computed=%u submitted=%u\n",r.executor.pitch_stats().computations-pitch_start.computations,
        r.executor.pitch_stats().submitted-pitch_start.submitted);
    // 无新目标/长断样后恢复: 撤销旧意图, 实际重建一次; 一个合法新请求即可恢复, 不加确认按钮.
    r.imu_on=false;
    for(unsigned i=0;i<30;++i) r.tick();
    CHECK(!r.app.snapshot().executions[main_i].execute);
    r.imu_on=true;
    for(unsigned i=0;i<3;++i) r.tick();
    CHECK(r.perception.stats().reprime_events==1 && !r.app.target(Domain::main_yaw));
    r.wire_target(r.targets(r.perception.platform().continuous_heading));
    r.tick();
    CHECK(r.app.snapshot().executions[main_i].execute);
    CHECK(r.app.snapshot().permission.generation==generation && !r.app.snapshot().permission.manual_stop);
    std::puts("N6 recovery: new_request_to_execute=1 cycle, reference_rebuilds=1; model times are not F407 WCET");
}

void remote_raw_closed_loop() {
    RemoteMappingConfig mapping{0.05f,1,1,2,1,1,25,10,{0,300,-300,50,true}}; // 仅合成操纵量程.
    Rig r{fixture(),false,mapping};
    r.raw_remote=true; r.remote_input.switch_left=drivers::Dr16Switch::up;
    for(unsigned i=0;i<30;++i) r.tick();
    CHECK(r.app.snapshot().permission.source==Source::remote);
    CHECK(r.app.snapshot().executions[main_i].frame==YawFrame::local_inertial);
    const float initial=r.app.snapshot().executions[main_i].adopted_target.value();
    r.remote_input.ch0=330; r.plant.base_rate=0.1;
    for(unsigned i=0;i<500;++i) r.tick();
    r.remote_input.ch0=0;
    for(unsigned i=0;i<30;++i) r.tick();
    const auto goal=r.app.snapshot().executions[main_i].adopted_target;
    CHECK(goal && *goal>initial+0.2f && *goal<initial+0.3f);
    for(unsigned i=0;i<1500;++i) r.tick();
    CHECK(r.app.snapshot().executions[main_i].adopted_target==goal);
    CHECK(std::fabs(r.true_local_heading()-*goal)<0.05f);
    r.remote_input.switch_right=drivers::Dr16Switch::down;
    for(unsigned i=0;i<15;++i) r.tick();
    CHECK(r.app.snapshot().permission.manual_stop);
    CHECK(!r.app.snapshot().executions[main_i].execute && std::fabs(r.current())<1e-5f);
    CHECK(r.executor.stats().dm_disable_sent>0);
    std::puts("N6 raw DT7(14ms) -> local heading -> current -> serial sensors: hold/manual-stop passed");
}

void mechanical_boundary() {
    auto cfg=synthetic_config(); cfg.joints[0].max_rad=0;
    Rig r{cfg};
    // 瞬时处于上界且基座向负方向转动: 内环本会给出向上界外的正向关节电流.
    // 单独固定本次有效感知输入, 核对应用机械限位后的实际 CAN, 非动力学测量.
    AppInputs input;
    input.link.remote_online=input.link.host_link_alive=input.link.host_request=input.link.locally_ready=true;
    input.link.switch_left=drivers::Dr16Switch::down; input.link.switch_right=drivers::Dr16Switch::mid;
    input.link.host_session_epoch=1;
    input.epochs={r.app.config_epoch(),r.app.reference_epoch()};
    input.faults=r.executor.health(r.now);
    for(unsigned i=0;i<domain_count;++i) {
        input.feedback[i].position_rad=r.app.snapshot().domains[i].feedback.position_rad;
        input.feedback[i].velocity_rad_s=r.app.snapshot().domains[i].feedback.velocity_rad_s;
    }
    input.main_yaw=r.perception.platform();
    input.main_yaw.value.heading_rate=-0.3f*input.main_yaw.value.heading_gain;
    CHECK(r.app.accept_targets(r.targets(input.main_yaw.continuous_heading),r.now,Source::host)==AcceptResult::accepted);
    r.app.update(r.now,input); r.bus1.clear(); r.bus2.clear();
    r.executor.step(r.now,0.001f,r.app,MicroTick{r.us});
    CHECK(r.executor.main_yaw_stats().mechanical_limit);
    CHECK(std::fabs(r.current())<1e-5f);
    ++r.now.ms; r.us+=1000;
    r.tick(r.targets(-0.1f,0,0,YawFrame::joint));
    CHECK(!r.executor.main_yaw_stats().mechanical_limit);
    CHECK(r.current()<0);
}
} // 测试辅助
int main() {
    mechanical_alignment_contracts();
    pitch_cycle_rejection_contracts();
    protocol_composition_metrics(); remote_raw_closed_loop();
    downstream_and_closed_loop(); direction_and_modes(); degradation_and_recovery(); hot_gain_and_branch(); mechanical_boundary();
    std::puts("sentinel inertial contracts passed");
}
