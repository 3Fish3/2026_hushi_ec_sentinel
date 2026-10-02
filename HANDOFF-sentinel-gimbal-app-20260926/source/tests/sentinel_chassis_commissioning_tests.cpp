// 专用底盘镜像的原生契约: 同一配置、原始DR16、应用/执行与真实CAN编码.
#include "board_app_config.hpp"
#include "sentinel_cycle.hpp"
#include "sentinel_param_apply.hpp"
#include "sim_plant.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
using namespace ec::sentinel::app;
namespace board = ec::sentinel::board;
using ec::drivers::Dr16Switch;
using ec::sentinel::host::SoftwareBus;
using ec::sentinel::host::raw_slot;
namespace {
void check(bool ok, const char* expr, int line) {
    if (!ok) { std::fprintf(stderr, "line %d: %s\n", line, expr); std::_Exit(3); }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)
struct Remote {
    std::array<int, 4> ch{};
    int wheel{};
    Dr16Switch left{Dr16Switch::mid};
    Dr16Switch right{Dr16Switch::up};
};
std::array<std::uint8_t, 18> frame(const Remote& r) {
    std::array<std::uint8_t, 18> out{};
    std::uint64_t bits = 0;
    for (unsigned i=0; i<4; ++i) { bits |= (static_cast<std::uint64_t>(r.ch[i]+1024)&0x7ffu) << (11*i); }
    for (unsigned i=0; i<6; ++i) { out[i] = static_cast<std::uint8_t>(bits >> (8*i)); }
    out[5] = static_cast<std::uint8_t>((out[5]&0x0fu) | (static_cast<unsigned>(r.left)<<6) | (static_cast<unsigned>(r.right)<<4));
    const auto fifth = static_cast<std::uint16_t>(r.wheel+1024);
    out[16] = static_cast<std::uint8_t>(fifth); out[17] = static_cast<std::uint8_t>(fifth>>8);
    return out;
}
struct Rig {
    SentinelConfig config{board::deployment_config()};
    SoftwareBus bus1{}, bus2{};
    SentinelApp app{config, ConfigAudience::deployment};
    SentinelExecutor executor{config, ConfigAudience::deployment, bus1, bus2};
    SentinelPerception perception{config, ConfigAudience::deployment};
    ParamStore params{config};
    SentinelCycle cycle{app, executor, perception, board::deployment_mapping(), config.timing.remote_ttl};
    ec::drivers::Dr16Stream stream{};
    ec::Tick now{1000};
    bool maintenance{};
    std::uint32_t maintenance_epoch{1};
    Rig() {
        CHECK(executor.assembled());
        const std::uint8_t byte=0; stream.push({&byte,1}); static_cast<void>(stream.on_idle());
    }
    void tick(std::optional<Remote> remote = std::nullopt, int missing_wheel = -1) {
        CycleInputs in{}; in.now=now; in.now_us=ec::MicroTick{now.ms*1000u};
        in.dt_s=0.001f; in.locally_ready=true;
        in.maintenance_link_alive=maintenance; in.maintenance_request=maintenance;
        in.maintenance_session_epoch=maintenance_epoch;
        if (remote) {
            const auto bytes=frame(*remote); stream.push({bytes.data(),bytes.size()});
            in.remote_sample=stream.on_idle(); CHECK(in.remote_sample); in.remote_received_at=now;
        }
        for (unsigned i=0; i<4; ++i) {
            if (static_cast<int>(i)==missing_wheel) { continue; }
            ec::transport::CanFrame feedback{}; feedback.id=0x201u+i; feedback.size=8;
            feedback.data[0]=4; // 原生编码1024, 零RPM/电流, 每周期真实新反馈.
            executor.route(2, feedback, now, *in.now_us);
        }
        bus1.clear(); bus2.clear();
        static_cast<void>(cycle.control(in)); cycle.commit(in);
        // 无论来源或输入如何, 未开放机构均不获得执行资格.
        for (std::size_t i=4; i<domain_count; ++i) { CHECK(!app.snapshot().executions[i].execute); }
        for (const auto& f:bus1.frames) {
            if (f.id==0x1fe || f.id==0x2fe) { for (auto b:f.data) { CHECK(b==0); } }
            else { CHECK((f.id==1 && f.size==8 && f.data[7]==0xfd) ||
                (f.id==0x7ff && f.size==4 && f.data[0]==1 && f.data[2]==0xcc)); }
        }
        if (const auto stop=bus2.last_with_id(0x1ff)) { for (auto b:stop->data) { CHECK(b==0); } }
        ++now.ms;
    }
    std::array<int,4> command(const Remote& remote) {
        for (unsigned retry=0; retry<4; ++retry) {
            tick(remote);
            if (const auto f=bus2.last_with_id(0x200)) {
                std::array<int,4> out{};
                for (unsigned i=0; i<4; ++i) { out[i]=*raw_slot(*f,i); }
                return out;
            }
        }
        CHECK(false); return {};
    }
};
void configuration() {
    const auto c=board::deployment_config(); const auto report=validate(c,ConfigAudience::deployment);
    CHECK(report.startable && !c.independent_motor_bench);
    for (unsigned i=0; i<domain_count; ++i) { CHECK((report.domains[i]==ConfigState::complete)==(i<4)); }
    CHECK(board::deployment_summary.installed_dji==9 && board::deployment_summary.installed_dm==1);
    const auto geometry=omni_drive(c); CHECK(geometry && ec::motion::omni_well_posed(*geometry));
    for (unsigned i=0; i<4; ++i) {
        CHECK(c.dji[i].bus==2 && c.dji[i].device_id==i+1);
        CHECK(*c.dji[i].rotor_per_axis==19.0f && *c.dji[i].max_axis_velocity_rad_s==10.0f);
        CHECK(std::fabs(geometry->wheels[i].radius_m-0.0819702695f)<1e-8f);
    }
    for (const auto twist:std::array<ec::motion::PlanarTwist,3>{{{0.4f,0,0},{0,0.4f,0},{0,0,0.8f}}}) {
        const auto speeds=ec::motion::omni_wheel_speeds(*geometry,twist); CHECK(speeds);
        const auto inverse=ec::motion::omni_twist(*geometry,*speeds); CHECK(inverse);
        CHECK(std::fabs(inverse->vx_mps-twist.vx_mps)<1e-5f);
        CHECK(std::fabs(inverse->vy_mps-twist.vy_mps)<1e-5f);
        CHECK(std::fabs(inverse->wz_rad_s-twist.wz_rad_s)<1e-5f);
    }
    Rig rig; for(unsigned i=0;i<domain_count;++i) { CHECK((rig.executor.axis(static_cast<Domain>(i))!=nullptr)==(i<4)); }
}
void remote_and_stop() {
    Rig rig; Remote drive{{0,0,0,0},0,Dr16Switch::up,Dr16Switch::mid};
    rig.tick(); CHECK(rig.app.snapshot().permission.source==Source::none);
    drive.ch[3]=330; CHECK((rig.command(drive)==std::array<int,4>{})); // 初次偏杆不接管
    drive.ch[3]=0; CHECK((rig.command(drive)==std::array<int,4>{}));
    drive.ch[3]=330; auto forward=rig.command(drive); for(auto value:forward) { CHECK(value>0); }
    drive.wheel=-660; CHECK(rig.command(drive)==forward);
    drive.wheel=660; CHECK(rig.command(drive)==forward); // 拨轮完全不影响驱动
    drive.ch={0,0,-330,0}; auto left=rig.command(drive);
    CHECK(left[0]<0 && left[1]>0 && left[2]>0 && left[3]<0);
    drive.ch={-330,0,0,0}; auto ccw=rig.command(drive);
    CHECK(ccw[0]<0 && ccw[1]>0 && ccw[2]<0 && ccw[3]>0);
    drive.right=Dr16Switch::down; CHECK((rig.command(drive)==std::array<int,4>{}));
    drive.right=Dr16Switch::mid; CHECK((rig.command(drive)==std::array<int,4>{}));
    drive.ch={}; static_cast<void>(rig.command(drive));
    drive.ch[3]=330; for(auto value:rig.command(drive)) { CHECK(value>0); }
    for(unsigned i=0;i<106;++i) { rig.tick(); }
    CHECK(rig.app.snapshot().permission.source==Source::none);
    CHECK((rig.command(Remote{})==std::array<int,4>{}));
}
void maintenance_handoff() {
    Rig rig; rig.maintenance=true; rig.tick();
    CHECK(rig.app.snapshot().permission.source==Source::none); // 保留遥控首启
    Remote standby{}; rig.tick(standby);
    CHECK(rig.app.snapshot().permission.source==Source::maintenance);
    // 逐轮维护走正式目标入口, 未测轮显式0; 非底盘轴仍无执行资格.
    for (unsigned selected=0; selected<4; ++selected) {
        TargetSnapshot packet{};
        for(unsigned i=0;i<4;++i) { packet.domains[i]=DomainSetpoint{ec::control::ControlMode::velocity,i==selected?0.5f:0.0f}; }
        CHECK(rig.app.accept_targets(packet,rig.now,Source::maintenance)==AcceptResult::accepted);
        const auto raw=rig.command(standby);
        for(unsigned i=0;i<4;++i) { CHECK(i==selected ? raw[i]>0 : raw[i]==0); }
    }
    CHECK(rig.app.accept_targets(TargetSnapshot{},rig.now,Source::maintenance)==AcceptResult::accepted);
    CHECK((rig.command(standby)==std::array<int,4>{}));
    CHECK(apply_param(rig.params,rig.app,&rig.executor,&rig.perception,1,0.25f,true)==ParamWriteResult::accepted);
    CHECK(std::fabs(rig.executor.axis(Domain::chassis_lf)->active_velocity_pid().kp-0.25f)<1e-6f);
    Remote drive{{},0,Dr16Switch::up,Dr16Switch::mid}; rig.tick(drive);
    CHECK(rig.app.snapshot().permission.source==Source::remote);
    CHECK(std::fabs(rig.executor.axis(Domain::chassis_lf)->active_velocity_pid().kp-0.25f)<1e-6f);
    rig.tick(standby); CHECK(rig.app.snapshot().permission.source==Source::none);
    ++rig.maintenance_epoch; rig.tick(standby);
    CHECK(rig.app.snapshot().permission.source==Source::maintenance);
    standby.left=Dr16Switch::down; rig.tick(standby);
    CHECK(rig.app.snapshot().permission.source==Source::none); // 其它组合不开放Jetson/维护
    standby.left=Dr16Switch::mid;
    standby.right=Dr16Switch::down; rig.tick(standby);
    CHECK(rig.app.snapshot().permission.source==Source::none);
}
void missing_feedback() {
    Rig rig; Remote drive{{},0,Dr16Switch::up,Dr16Switch::mid};
    static_cast<void>(rig.command(drive)); drive.ch[3]=330;
    for(auto value:rig.command(drive)) { CHECK(value>0); }
    for(unsigned i=0;i<60;++i) { rig.tick(drive,2); }
    for(unsigned i=0;i<4;++i) { CHECK(!rig.app.snapshot().executions[i].execute); }
    const auto output=rig.bus2.last_with_id(0x200); CHECK(output);
    for(unsigned i=0;i<4;++i) { CHECK(*raw_slot(*output,i)==0); }
}
} // 内部测试
int main() { configuration(); remote_and_stop(); maintenance_handoff(); missing_feedback();
    std::puts("chassis commissioning contracts passed"); }
