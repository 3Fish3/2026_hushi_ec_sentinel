// 直接使用交付配置, 核对标记零位、无猜测限位和机构独立执行的CAN证据.
#include "board_app_config.hpp"
#include "gm6020_bench_velocity.hpp"
#include "sentinel_cycle.hpp"
#include "sim_plant.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
namespace app=ec::sentinel::app;
namespace board=ec::sentinel::board;
namespace host=ec::sentinel::host;
using app::Domain;
using ec::drivers::Dr16Switch;
namespace {
void check(bool ok,const char* expression,int line) {
    if(!ok) { std::fprintf(stderr,"line %d: %s\n",line,expression); std::_Exit(3); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)),#__VA_ARGS__,__LINE__)
constexpr bool gimbal=EC_APP_GIMBAL_COMMISSIONING!=0;
constexpr float tau=6.2831853071795864769f;
constexpr auto idx(Domain d) { return app::domain_index(d); }
struct Rig {
    app::SentinelConfig config{board::deployment_config()};
    host::SoftwareBus bus1{},bus2{};
    app::SentinelApp application{config,app::ConfigAudience::deployment};
    app::SentinelExecutor executor{config,app::ConfigAudience::deployment,bus1,bus2};
    app::SentinelPerception perception{config,app::ConfigAudience::deployment};
    app::SentinelCycle cycle{application,executor,perception,board::deployment_mapping(),config.timing.remote_ttl};
    host::SimPlant plant{};
    std::array<ec::sentinel::BenchEncoderVelocity,2> velocities{};
    ec::Tick now{1000};
    Dr16Switch left{Dr16Switch::mid},right{Dr16Switch::up};
    bool remote{true},maintenance{true};
    unsigned maintenance_epoch{1};
    std::optional<ec::transport::CanFrame> shooter_frame{};
    Rig() {
        CHECK(executor.assembled());
        if constexpr(gimbal) {
            plant.dji[4].position_rad=1027.0f*tau/8192.0f+0.1f;
            plant.dji[5].position_rad=2017.0f*tau/8192.0f-0.1f;
            plant.dm.position_rad=34282.0f*25.0f/65535.0f-12.5f;
        }
    }
    void tick(bool advance=false) {
        app::CycleInputs in{}; in.now=now; in.now_us=ec::MicroTick{now.ms*1000u}; in.dt_s=0.001f;
        in.locally_ready=true; in.maintenance_link_alive=maintenance; in.maintenance_request=maintenance;
        in.maintenance_session_epoch=maintenance_epoch;
        if(remote) {
            ec::drivers::Dr16Sample sample{}; sample.switch_left=left; sample.switch_right=right;
            // 全部杆值偏转也不得在这两种角色生成底盘、云台或发射遥控目标.
            sample.ch0=330;sample.ch1=330;sample.ch2=330;sample.ch3=330;sample.ch4=660;
            in.remote_sample=sample; in.remote_received_at=now;
        }
        plant.emit_feedback(config,[&](std::uint8_t bus,const ec::transport::CanFrame& f,ec::Tick at) {
            executor.route(bus,f,at,*in.now_us);
            if constexpr(gimbal) {
                if((bus==1 && f.id==0x208) || (bus==2 && f.id==0x20a)) {
                    velocities[f.id==0x208?0:1].receive(static_cast<std::uint16_t>((f.data[0]<<8)|f.data[1]),at,*in.now_us);
                }
            }
        },now);
        if constexpr(gimbal) {
            executor.set_joint_velocity_feedback(Domain::main_yaw,velocities[0].value());
            executor.set_joint_velocity_feedback(Domain::fire_yaw,velocities[1].value());
        }
        bus1.clear();bus2.clear(); static_cast<void>(cycle.control(in));cycle.commit(in);
        if(const auto f=bus2.last_with_id(0x1ff)) { shooter_frame=f; }
        for(std::size_t i=0;i<app::domain_count;++i) {
            const bool active=gimbal ? i>=4 && i<=6 : i>=7;
            if(!active) { CHECK(!application.snapshot().executions[i].execute); }
        }
        for(const auto& f:bus2.frames) {
            if(f.id==0x200 || (gimbal && f.id==0x1ff)) { for(auto b:f.data) { CHECK(b==0); } }
        }
        if constexpr(!gimbal) {
            // CAN1 在发射角色只剩底盘与主 yaw: 未启用机构只允许显式零帧, 不得混入其它帧.
            for(const auto& f:bus1.frames) {
                if(f.id==0x200 || f.id==0x1fe) { for(auto b:f.data) { CHECK(b==0); } }
                else { CHECK(false); }
            }
            // CAN2 承载发射组 0x1ff、未启用的发射 yaw 0x2fe (零帧) 与达妙 pitch 帧.
            for(const auto& f:bus2.frames) {
                if(f.id==0x1ff) { continue; }
                if(f.id==0x2fe) { for(auto b:f.data) { CHECK(b==0); } continue; }
                CHECK((f.id==1 && f.size==8 && f.data[7]==0xfd) || (f.id==0x7ff && f.size==4 && f.data[2]==0xcc));
            }
        }
        if(advance) { plant.integrate(config,bus1,bus2,0.001f); }
        ++now.ms;
    }
    void warmup() { for(int i=0;i<8;++i)tick(); }
    app::AcceptResult targets(Domain d,float value) {
        app::TargetSnapshot t{};t.domains[idx(d)]=app::DomainSetpoint{gimbal?ec::control::ControlMode::position:ec::control::ControlMode::velocity,value};
        return application.accept_targets(t,now,app::Source::maintenance);
    }
};
void configuration() {
    const auto c=board::deployment_config();const auto report=app::validate(c,app::ConfigAudience::deployment);
    CHECK(report.startable && !c.independent_motor_bench);
    for(std::size_t i=0;i<app::domain_count;++i) {
        CHECK((report.domains[i]==app::ConfigState::complete)==(gimbal ? i>=4 && i<=6 : i>=7));
    }
    if constexpr(gimbal) {
        for(const auto& j:c.joints) { CHECK(j.zero_offset_rad && !j.min_rad && !j.max_rad && j.commissioning_unbounded); }
        auto bad=c;bad.dji[4].rotor_per_axis=2;
        CHECK(app::validate(bad,app::ConfigAudience::deployment).domains[idx(Domain::main_yaw)]!=app::ConfigState::complete);
        bad=c;bad.joints[0].commissioning_unbounded=false;
        CHECK(app::validate(bad,app::ConfigAudience::deployment).domains[idx(Domain::main_yaw)]!=app::ConfigState::complete);
    } else {
        CHECK(*c.dji[6].rotor_per_axis==1 && *c.dji[7].rotor_per_axis==1 && *c.dji[8].rotor_per_axis==36);
        CHECK(c.dji[6].device_id==5 && c.dji[7].device_id==6 && c.dji[8].device_id==7);
    }
}
void startup_and_stop() {
    Rig r;r.remote=false;r.tick();CHECK(r.application.authority().source()==app::Source::none);
    r.remote=true;r.warmup();CHECK(r.application.authority().source()==app::Source::maintenance);
    for(const auto& e:r.application.snapshot().executions) { CHECK(!e.execute); }
    CHECK(r.targets(gimbal?Domain::main_yaw:Domain::friction_left,0.5f)==app::AcceptResult::accepted);
    r.tick();r.tick();CHECK(r.application.snapshot().executions[idx(gimbal?Domain::main_yaw:Domain::friction_left)].execute);
    r.right=Dr16Switch::down;r.tick();CHECK(r.application.authority().source()==app::Source::none);
    for(const auto& e:r.application.snapshot().executions) { CHECK(!e.execute); }
    r.left=Dr16Switch::up;r.right=Dr16Switch::mid;r.tick();
    CHECK(r.application.authority().source()==app::Source::none);
}
void gimbal_contracts() {
    Rig r;r.warmup();
    const auto a=r.executor.joint_feedback(Domain::main_yaw,r.now).position_rad;
    const auto b=r.executor.joint_feedback(Domain::fire_yaw,r.now).position_rad;
    CHECK(a.validity==ec::Validity::valid && std::fabs(a.value-0.1f)<0.002f);
    CHECK(b.validity==ec::Validity::valid && std::fabs(b.value+0.1f)<0.002f);
    CHECK(std::fabs(r.executor.joint_feedback(Domain::pitch,r.now).position_rad.value)<0.001f);
    CHECK(!r.executor.align_reference(Domain::main_yaw,0,r.now));
    CHECK(r.executor.align_reference(Domain::main_yaw,a.value,r.now));
    CHECK(r.targets(Domain::main_yaw,0.4f)==app::AcceptResult::accepted);
    r.tick();CHECK(*host::raw_slot(*r.bus1.last_with_id(0x1fe),3)>0);
    CHECK(r.targets(Domain::fire_yaw,-0.4f)==app::AcceptResult::accepted);
    r.tick();CHECK(*host::raw_slot(*r.bus2.last_with_id(0x2fe),1)<0); // 发射 yaw 在 CAN2
    // 未测机械端点不复用旧限位; 此处仅离线接收验证, 不对实物下发大角度.
    CHECK(r.targets(Domain::fire_yaw,4.0f)==app::AcceptResult::accepted);
    CHECK(r.targets(Domain::pitch,12.4f)==app::AcceptResult::limit_violation); // 加零偏后越MIT映射范围
    CHECK(r.targets(Domain::pitch,0.1f)==app::AcceptResult::accepted);
    bool mit=false;
    for(int i=0;i<12;++i) {
        r.tick(true);
        for(const auto& f:r.bus2.frames) if(f.id==1 && f.size==8 && f.data[0]!=0xff) { // pitch 在 CAN2
            const auto decoded=host::decode_dm_mit_command(f,*r.config.dm[0].limits);CHECK(decoded);
            CHECK(std::fabs(decoded->position_rad-(0.1f+*r.config.joints[2].zero_offset_rad))<0.001f);mit=true;
        }
    }
    CHECK(mit);
    CHECK(r.application.accept_targets({},r.now,app::Source::maintenance)==app::AcceptResult::accepted);
    bool disable=false;
    for(int i=0;i<6;++i) {r.tick(true);for(const auto& f:r.bus2.frames)if(f.id==1 && f.size==8 && f.data[7]==0xfd)disable=true;}
    CHECK(disable);
    // 跨单圈边界保持本次连续角, 不在编码0处回零或触发虚构端点.
    for(int i=0;i<1100;++i) { r.plant.dji[4].position_rad+=0.007f;r.tick(); }
    CHECK(r.executor.joint_feedback(Domain::main_yaw,r.now).position_rad.value>6.0f);
    r.plant.dji[4].emitting=false;for(int i=0;i<60;++i)r.tick();
    CHECK(r.executor.joint_feedback(Domain::main_yaw,r.now).position_rad.validity!=ec::Validity::valid);
    r.plant.dji[4].emitting=true;r.warmup();
    CHECK(!r.application.snapshot().executions[idx(Domain::main_yaw)].execute);
}
void shooter_contracts() {
    Rig r;r.warmup();
    CHECK(r.targets(Domain::trigger,1)==app::AcceptResult::accepted);
    r.tick();CHECK(!r.application.snapshot().executions[idx(Domain::trigger)].execute);
    r.application.set_launch_permit(true,r.now,ec::Lifetime{100});
    r.tick();r.tick();CHECK(r.application.snapshot().executions[idx(Domain::trigger)].execute);
    CHECK(r.shooter_frame && *host::raw_slot(*r.shooter_frame,2)>0);
    app::TargetSnapshot targets{};
    targets.domains[idx(Domain::friction_left)]=app::DomainSetpoint{ec::control::ControlMode::velocity,10};
    targets.domains[idx(Domain::friction_right)]=app::DomainSetpoint{ec::control::ControlMode::velocity,-10};
    targets.domains[idx(Domain::trigger)]=app::DomainSetpoint{ec::control::ControlMode::velocity,-1};
    CHECK(r.application.accept_targets(targets,r.now,app::Source::maintenance)==app::AcceptResult::accepted);
    r.application.set_launch_permit(false,r.now,ec::Lifetime{100});
    r.tick();r.tick();const auto f=r.shooter_frame;CHECK(f);
    CHECK(*host::raw_slot(*f,0)>0 && *host::raw_slot(*f,1)<0 && *host::raw_slot(*f,2)<0);
    // 未调pitch曾在线又丢失, 不阻塞空机构拨弹; 摩擦轮自身失联仍联动撤双轮与拨弹.
    r.plant.dm.emitting=false;
    for(int i=0;i<110;++i) {
        r.application.set_launch_permit(true,r.now,ec::Lifetime{100});
        CHECK(r.targets(Domain::trigger,1)==app::AcceptResult::accepted);r.tick();
    }
    CHECK(r.application.snapshot().executions[idx(Domain::trigger)].execute);
    r.plant.dji[6].emitting=false;
    for(int i=0;i<60;++i)r.tick();
    CHECK(!r.application.snapshot().executions[idx(Domain::trigger)].execute);
    CHECK(!r.application.snapshot().executions[idx(Domain::friction_right)].execute);
}
} // 测试夹具
int main() {
    configuration();startup_and_stop();
    if constexpr(gimbal)gimbal_contracts();else shooter_contracts();
    std::puts("mechanism commissioning contracts passed");
}
