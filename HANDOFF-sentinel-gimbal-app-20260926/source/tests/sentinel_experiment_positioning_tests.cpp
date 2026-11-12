#include "sentinel_experiment_wire.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace ec::sentinel::app::excitation;
#define CHECK(x) do {if(!(x)){std::cerr<<__LINE__<<": "<<#x<<'\n';std::abort();}}while(false)
// These numbers are OFFLINE TEST FIXTURES, not mechanical approvals.
Config config() {
    Config c{};c.session_id=17;c.nonce=31;c.param_epoch=7;c.guardian_version=1;c.config_hash=101;c.safety_reference_id=201;
    c.center_rad=.138f;c.amplitude_rad=.02f;c.min_rad=0;c.max_rad=.3f;c.max_velocity_rad_s=2;
    c.max_acceleration_rad_s2=200;c.max_travel_rad=.1f;c.max_output_abs=1;c.max_output_integral=.2f;
    c.move_us=40000;c.hold_us=40000;c.duration_us=120000;c.repeats=1;c.heartbeat_timeout_us=200000;c.stop_timeout_us=20000;return c;
}
Guards guards() {
    Guards g{};g.maintenance_authorized=g.operator_approved=g.remote_online=true;
    g.guardian_approved=g.guardian_clear=g.feedback_fresh=g.reference_valid=g.limits_valid=true;
    g.actuator_healthy=g.other_axes_idle=g.stop_strategy_verified=true;
    g.timing_valid=g.link_alive=g.backpressure_ok=g.output_observation_valid=true;
    g.capture_remaining_us=250000;g.capture_sample_us=1000; // explicit synthetic capture fixture
    g.param_epoch=7;g.guardian_version=1;g.config_hash=101;g.safety_reference_id=201;
    g.config_epoch=3;g.reference_epoch=4;g.axis_identity=5;g.actual_position_rad=.128f;g.feedback_sample_us=1000;return g;
}
PositionPolicy policy() {return {123,.001f,.01f,1.f,40000,4000,100000,400000,4000};}
struct Fixture {
    Controller s{true};Guards g=guards();std::uint32_t now=1000;
    Fixture(){CHECK(s.prepare(config(),g,now));CHECK(s.arm(g,now));CHECK(s.request_position(policy(),g,now));}
    void step(){now+=1000;const double x=std::min(1.,double(now-1000)/40000);
        g.actual_position_rad=static_cast<float>(.128+.01*x*x*x*(10+x*(-15+6*x)));
        g.feedback_sample_us=now;CHECK(s.heartbeat(now));CHECK(s.tick(g,now));}
    void ready(){while(now<50000)step();CHECK(s.position_stage()==PositionStage::wait_human);}
    void confirm(){ready();CHECK(s.confirm_position(123,g,now));CHECK(s.position_stage()==PositionStage::confirmed);}
};
void late_capture_cannot_start() {
    Fixture f;f.confirm();f.g.capture_sample_us=f.now;
    f.g.capture_remaining_us=config().duration_us-1;
    CHECK(!f.s.start(f.g,f.now));CHECK(f.s.reason()==Reason::backpressure);
    Fixture delayed;delayed.confirm();delayed.g.capture_sample_us=delayed.now-2;
    delayed.g.capture_remaining_us=config().duration_us+1;
    CHECK(!delayed.s.start(delayed.g,delayed.now));CHECK(delayed.s.reason()==Reason::backpressure);
}
void workflow() {
    late_capture_cannot_start();
    Fixture f;CHECK(!f.s.start(f.g,f.now));CHECK(!f.s.confirm_position(123,f.g,f.now));
    auto r=f.s.tick(f.g,f.now);CHECK(r && std::abs(r->position_rad-.128f)<1e-6);
    f.ready();CHECK(!f.s.confirm_position(124,f.g,f.now));CHECK(f.s.confirm_position(123,f.g,f.now));
    // A real nonzero measured error within explicitly supplied tolerance is admitted.
    f.g.actual_position_rad=config().center_rad+.0005f;CHECK(f.s.start(f.g,f.now));
    CHECK(f.s.position_stage()==PositionStage::running);CHECK(f.s.position_policy().human_token==0);
    CHECK(!f.s.start(f.g,f.now));
    while(f.now<175000)f.step();
    CHECK(f.s.state()==State::armed && f.s.position_stage()==PositionStage::holding_complete);
    CHECK(f.s.tick(f.g,f.now+1000));CHECK(!f.s.start(f.g,f.now+1000));
    f.now+=1000;while(f.now<440000)f.step();
    f.g.feedback_sample_us=450000;CHECK(f.s.heartbeat(450000));CHECK(!f.s.tick(f.g,450000));
    CHECK(f.s.position_stage()==PositionStage::invalidated);
}
void invalidation() {
    for(unsigned i=0;i<14;++i) {
        Fixture f;f.confirm();f.now+=1000;f.g.feedback_sample_us=f.now;
        switch(i){case 0:f.g.manual_stop=true;break;case 1:f.g.maintenance_authorized=false;break;
        case 2:f.g.link_alive=false;break;case 3:f.g.feedback_fresh=false;break;case 4:++f.g.config_epoch;break;
        case 5:++f.g.reference_epoch;break;case 6:++f.g.param_epoch;break;case 7:++f.g.config_hash;break;
        case 8:++f.g.safety_reference_id;break;case 9:f.g.actual_velocity_rad_s=std::numeric_limits<float>::quiet_NaN();break;
        case 10:f.g.actual_position_rad=std::numeric_limits<float>::infinity();break;
        case 11:f.g.feedback_sample_us=f.now-5000;break;case 12:f.g.guardian_clear=false;break;
        case 13:f.g.operator_approved=false;break;}
        CHECK(!f.s.tick(f.g,f.now));CHECK(f.s.state()==State::fault_latched);
        CHECK(f.s.position_stage()==PositionStage::invalidated && f.s.position_policy().human_token==0);
        CHECK(!f.s.start(f.g,f.now));
    }
    Fixture drift;drift.confirm();drift.now+=1000;drift.g.feedback_sample_us=drift.now;
    drift.g.actual_position_rad+=.002f;CHECK(drift.s.tick(drift.g,drift.now));
    CHECK(drift.s.position_stage()==PositionStage::settling);CHECK(!drift.s.start(drift.g,drift.now));
    CHECK(drift.s.position_policy().human_token==0);
    Fixture cancelled;CHECK(!cancelled.s.cancel_position(cancelled.g,cancelled.now));cancelled.ready();
    CHECK(cancelled.s.cancel_position(cancelled.g,cancelled.now));CHECK(cancelled.s.position_stage()==PositionStage::cancelled_hold);
    cancelled.step();CHECK(!cancelled.s.confirm_position(123,cancelled.g,cancelled.now));
    cancelled.s.stop(cancelled.now);CHECK(cancelled.s.state()==State::finishing);
}
void validation() {
    for(unsigned i=0;i<12;++i){Controller s(true);auto g=guards();CHECK(s.prepare(config(),g,1000));CHECK(s.arm(g,1000));auto p=policy();
        switch(i){case 0:p.position_tolerance_rad=0;break;case 1:p.velocity_tolerance_rad_s=0;break;
        case 2:p.speed_limit_rad_s=3;break;case 3:p.move_us=1000;break;case 4:p.stable_dwell_us=0;break;
        case 5:p.positioning_timeout_us=0;break;case 6:p.hold_timeout_us=0;break;case 7:p.feedback_max_age_us=0;break;
        case 8:p.human_token=0;break;case 9:p.position_tolerance_rad=std::numeric_limits<float>::quiet_NaN();break;
        case 10:g.actual_velocity_rad_s=.02f;break;case 11:p.hold_timeout_us=0xffffffff;break;}
        CHECK(!s.request_position(p,g,1000));CHECK(s.position_stage()==PositionStage::none);
    }
    Controller blocked;auto g=guards();CHECK(!blocked.prepare(config(),g,1000));CHECK(!blocked.request_position(policy(),g,1000));
    auto c=config();c.axis=Axis::pitch;Controller pitch(true);CHECK(!pitch.prepare(c,g,1000));CHECK(pitch.reason()==Reason::pitch_blocked);
    Fixture repeated;for(unsigned i=0;i<3;++i){repeated.now+=1000;CHECK(repeated.s.tick(repeated.g,repeated.now));}
    repeated.now+=1000;CHECK(!repeated.s.tick(repeated.g,repeated.now)); // repeated feedback cannot create dwell/freshness
}
void provisioned_gate() {
    // Test-only pinned deployment fixture: never exported as a deployment approval.
    TrustedDeploymentSafetyConfig d{};d.axis_identity=5;d.config_epoch=3;d.reference_epoch=4;
    d.guardian_version=1;d.config_hash=101;d.safety_reference_id=201;d.approval_record_version=1;
    d.firmware_sha256.fill(1);d.min_rad=0;d.max_rad=.3f;d.max_velocity_rad_s=2;d.max_acceleration_rad_s2=200;
    d.max_travel_rad=.1f;d.max_output_abs=1;d.max_output_integral=.2f;d.max_duration_us=180000;
    d.max_heartbeat_timeout_us=200000;d.max_stop_timeout_us=20000;d.stop_velocity_rad_s=.01f;
    d.stop_position_delta_rad=.001f;d.stop_dwell_us=4000;d.stop_policy=StopPolicy::zero_output;
    d.deployment_sha256=deployment_digest(d);
    TrustedDeploymentRoot root{};root.boot_id=99;root.axis_identity=5;root.firmware_sha256=d.firmware_sha256;root.deployment_sha256=d.deployment_sha256;
    HardwareExperimentController s(d,root);CHECK(s.hardware_capable());auto g=guards();g.actual_position_rad=config().center_rad;
    CHECK(s.prepare(config(),g,1000));CHECK(s.arm(g,1000));CHECK(!s.start(g,1000));
    CHECK(s.request_position(policy(),g,1000));CHECK(!s.start(g,1000));
    g.guardian_approved=false;g.feedback_sample_us=2000;CHECK(!s.tick(g,2000));CHECK(s.reason()==Reason::guardian);
    CHECK(s.position_policy().human_token==0);
}
void deadlines() {
    Fixture timeout;for(unsigned i=0;i<100;++i){timeout.now+=1000;timeout.g.feedback_sample_us=timeout.now;
        CHECK(timeout.s.heartbeat(timeout.now));const auto r=timeout.s.tick(timeout.g,timeout.now);
        if(i<99)CHECK(r);else CHECK(!r);
    }
    CHECK(timeout.s.reason()==Reason::stop_timeout); // actual feedback never reached target
    Fixture wait;wait.ready();while(wait.now<444000)wait.step();
    wait.now=445000;wait.g.feedback_sample_us=wait.now;CHECK(wait.s.heartbeat(wait.now));
    CHECK(!wait.s.tick(wait.g,wait.now));CHECK(wait.s.reason()==Reason::stop_timeout);
    Fixture hb;hb.ready();hb.now+=200000;hb.g.feedback_sample_us=hb.now;
    CHECK(!hb.s.start(hb.g,hb.now)); // unconfirmed + lease cannot be stretched by waiting
}
void wire() {
    Wire w(true);auto g=guards();auto& s=w.controller_for_cycle();CHECK(s.prepare(config(),g,1000));CHECK(s.arm(g,1000));
    std::array<std::uint8_t,48>b{3,8,5,1};std::array<std::uint8_t,160>out{};
    auto put=[&](unsigned o,std::uint32_t v){for(unsigned j=0;j<4;++j)b[o+j]=static_cast<std::uint8_t>(v>>(8*j));};
    auto f=[&](unsigned o,float v){put(o,std::bit_cast<std::uint32_t>(v));};
    put(4,17);put(8,31);put(12,123);f(16,.001f);f(20,.01f);f(24,1);put(28,40000);put(32,4000);put(36,100000);put(40,400000);put(44,4000);
    CHECK(w.handle(b,g,1000,std::span{out}.first(159))==0);CHECK(s.position_stage()==PositionStage::none);
    b[8]=32;CHECK(w.handle(b,g,1000,out)==160 && out[2]==4);b[8]=31;
    CHECK(w.handle(b,g,1000,out)==160 && out[2]==0 && out[128]==1 && out[132]==123);
    b[0]=2;b[1]=2;CHECK(w.handle(std::span{b}.first(12),g,1000,out)==128 && out[0]==2);
    b[0]=1;CHECK(w.handle(std::span{b}.first(12),g,1000,out)==32 && out[0]==1);
    b[0]=2;b[1]=8;CHECK(w.handle(b,g,1000,out)==128 && out[2]==2);
}
void wire_capture_phase() {
    Wire w(true);auto g=guards();g.backpressure_ok=false;
    std::array<std::uint8_t,88> b{3,3,5,1};std::array<std::uint8_t,160> out{};
    auto put=[&](unsigned o,std::uint32_t v){for(unsigned j=0;j<4;++j)b[o+j]=static_cast<std::uint8_t>(v>>(8*j));};
    auto f=[&](unsigned o,float v){put(o,std::bit_cast<std::uint32_t>(v));};
    put(4,17);put(8,31);put(12,7);put(16,1);put(20,101);put(24,201);
    f(28,.138f);f(32,.02f);f(36,0);f(40,.3f);f(44,2);f(48,200);f(52,.1f);f(56,1);f(60,.2f);
    put(64,40000);put(68,40000);put(72,120000);put(76,200000);put(80,20000);b[84]=1;
    CHECK(w.handle(b,g,1000,out)==160 && out[2]==0);
    b[1]=4;CHECK(w.handle(std::span{b}.first(12),g,1000,out)==160 && out[2]==0);
    CHECK(!w.controller_for_cycle().tick(g,2000));CHECK(w.controller().state()==State::armed);
    // Same unavailable recorder is STILL false in reply flags, never fabricated.
    CHECK((std::uint32_t{out[38]} & 1u)==0);
}
void capture_phase() {
    // Actual short recorder is CLOSED during positioning and HUMAN hold.
    Controller s(true);auto g=guards();g.backpressure_ok=false;
    CHECK(s.prepare(config(),g,1000,true));CHECK(s.arm(g,1000));
    CHECK(s.request_position(policy(),g,1000));
    std::uint32_t now=1000;
    for(;now<50000;) {now+=1000;g.actual_position_rad=config().center_rad;
        g.feedback_sample_us=now;CHECK(s.heartbeat(now));CHECK(s.tick(g,now));}
    CHECK(s.position_stage()==PositionStage::wait_human);
    CHECK(s.confirm_position(123,g,now));
    // Separate START must fail closed without real capture admission.
    CHECK(!s.start(g,now));CHECK(s.reason()==Reason::backpressure);
    Fixture f;f.confirm();CHECK(f.s.start(f.g,f.now));
    // Running loses capture => latched fault, no fabricated readiness.
    f.g.backpressure_ok=false;f.now+=1000;f.g.feedback_sample_us=f.now;
    CHECK(!f.s.tick(f.g,f.now));CHECK(f.s.reason()==Reason::backpressure);
    Fixture held;held.confirm();CHECK(held.s.start(held.g,held.now));
    while(held.now<175000)held.step();
    held.g.backpressure_ok=false;held.step();
    CHECK(held.s.position_stage()==PositionStage::holding_complete);
    Fixture cancelled;cancelled.ready();cancelled.g.backpressure_ok=false;
    CHECK(cancelled.s.cancel_position(cancelled.g,cancelled.now));cancelled.step();
    CHECK(cancelled.s.position_stage()==PositionStage::cancelled_hold);
}
int main(){wire_capture_phase();capture_phase();workflow();invalidation();validation();provisioned_gate();deadlines();wire();std::cout<<"positioning/hold/HUMAN/separate START offline contracts pass\n";}
