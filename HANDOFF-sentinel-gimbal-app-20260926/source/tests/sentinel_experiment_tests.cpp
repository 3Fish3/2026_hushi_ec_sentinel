#include "sentinel_experiment.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace ec::sentinel::app::excitation;
#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " << #x << '\n'; std::abort(); } } while(false)
Config config() {
    Config c{}; c.session_id=17;c.nonce=31;c.param_epoch=7;c.guardian_version=1;c.config_hash=101;c.safety_reference_id=201;
    c.center_rad=.138f;c.amplitude_rad=.02f;c.min_rad=0;c.max_rad=.3f;
    c.max_velocity_rad_s=2;c.max_acceleration_rad_s2=200;c.max_travel_rad=.1f;
    c.max_output_abs=1;c.max_output_integral=.2f;
    c.move_us=40000;c.hold_us=40000;c.duration_us=120000;c.repeats=1;
    c.heartbeat_timeout_us=200000;c.stop_timeout_us=20000;return c;
}
Guards guards() {
    Guards g{};g.maintenance_authorized=g.operator_approved=g.remote_online=true;
    g.guardian_approved=g.guardian_clear=g.feedback_fresh=g.reference_valid=g.limits_valid=true;
    g.actuator_healthy=g.other_axes_idle=g.pitch_fall_arrest_verified=g.stop_strategy_verified=true;
    g.timing_valid=g.link_alive=g.backpressure_ok=g.output_observation_valid=true;
    g.param_epoch=7;g.guardian_version=1;g.config_hash=101;g.safety_reference_id=201;g.actual_position_rad=.138f;return g;
}
void begin(Controller& s,const Config& c,const Guards& g,std::uint32_t t=1000) {
    CHECK(s.prepare(c,g,t));CHECK(s.arm(g,t));CHECK(s.start(g,t));
}
void generator_tests() {
    auto c=config(); CHECK(valid_config(c));
    for (const auto p : {Pattern::bounded_step,Pattern::trapezoid}) {
        c.pattern=p;
        for (const float sign : {1.0f,-1.0f}) {
            c.amplitude_rad=sign*.02f;
            auto previous=generate(c,0); CHECK(previous);CHECK(previous->position_rad==c.center_rad);
            for(std::uint32_t t=100;t<c.duration_us;t+=100) {
                const auto x=generate(c,t);const auto again=generate(c,t);CHECK(x && again);
                CHECK(x->position_rad==again->position_rad);
                CHECK(x->position_rad>=c.min_rad && x->position_rad<=c.max_rad);
                CHECK(std::abs(x->velocity_rad_s)<=c.max_velocity_rad_s);
                CHECK(std::abs(x->acceleration_rad_s2)<=c.max_acceleration_rad_s2);
                CHECK(std::abs(x->position_rad-previous->position_rad)/.0001f<=c.max_velocity_rad_s+.001f);
                CHECK(std::abs(x->velocity_rad_s-previous->velocity_rad_s)/.0001f<=c.max_acceleration_rad_s2+.01f);
                previous=x;
            }
            CHECK(!generate(c,c.duration_us));
        }
    }
    // Reproducible finite parameter sweep: seed 0x60202026, 200 envelopes.
    std::uint32_t seed=0x60202026;
    for(unsigned i=0;i<200;++i) {
        seed=1664525u*seed+1013904223u;c=config();
        c.amplitude_rad=(.001f+(seed%19000)*1e-6f)*(i%2?-1:1);
        c.pattern=i%2?Pattern::bounded_step:Pattern::trapezoid;CHECK(valid_config(c));
        for(std::uint32_t t=0;t<c.duration_us;t+=1000) { CHECK(generate(c,t)); }
    }
    for(float Config::* member : {&Config::center_rad,&Config::amplitude_rad,&Config::min_rad,&Config::max_rad,
        &Config::max_velocity_rad_s,&Config::max_acceleration_rad_s2,&Config::max_travel_rad,&Config::max_output_abs,&Config::max_output_integral}) {
        for(float invalid : {std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
            c=config();c.*member=invalid;CHECK(!valid_config(c));CHECK(!generate(c,0));
        }
    }
    c=config();c.move_us=0;CHECK(!valid_config(c));c=config();c.hold_us=0xffffffff;CHECK(!valid_config(c));
    c=config();c.duration_us=180001;CHECK(!valid_config(c));c=config();c.repeats=0;CHECK(!valid_config(c));
    c=config();c.max_velocity_rad_s=.1f;CHECK(!valid_config(c));c=config();c.max_acceleration_rad_s2=1;CHECK(!valid_config(c));
    c=config();c.max_travel_rad=.01f;CHECK(!valid_config(c));c=config();c.max_rad=.14f;CHECK(!valid_config(c));
    c=config();c.axis=static_cast<Axis>(4);CHECK(!valid_config(c));c=config();c.pattern=static_cast<Pattern>(3);CHECK(!valid_config(c));
}
void arm_retry_tests() {
    auto c=config();auto g=guards();Controller s(true);
    CHECK(s.prepare(c,g,1000));
    auto stale=g;stale.feedback_fresh=false;
    CHECK(!s.arm(stale,1000));
    CHECK(s.state()==State::idle); // A transient ARM denial latches nothing.
    CHECK(s.reason()!=Reason::none);
    CHECK(s.arm(g,1000)); // Retry with clean guards succeeds.
    CHECK(s.state()==State::armed);CHECK(s.reason()==Reason::none);
    // Armed and never driven: a transient tick denial self-heals.
    CHECK(!s.tick(stale,1000));
    CHECK(s.state()==State::armed);
    CHECK(s.start(g,1000));
    CHECK(s.tick(g,1000));
    // Running: the same denial still latches, as before.
    CHECK(!s.tick(stale,2000));
    CHECK(s.state()==State::fault_latched);
}
void safety_tests() {
    auto c=config();auto g=guards();Controller hardware;
    CHECK(!hardware.prepare(c,g,0));CHECK(!hardware.arm(g,0));CHECK(!hardware.start(g,0));
    hardware.stop(0);CHECK(!hardware.tick(g,0));CHECK(hardware.state()==State::disabled);
    for(bool Guards::* member : {&Guards::maintenance_authorized,&Guards::operator_approved,&Guards::remote_online,
        &Guards::guardian_approved,&Guards::guardian_clear,&Guards::feedback_fresh,&Guards::reference_valid,
        &Guards::limits_valid,&Guards::actuator_healthy,&Guards::other_axes_idle,&Guards::pitch_fall_arrest_verified,
        &Guards::stop_strategy_verified,&Guards::timing_valid,&Guards::link_alive,&Guards::backpressure_ok,&Guards::output_observation_valid}) {
        for(bool during_run : {false,true}) {
            c=config();c.axis=Axis::pitch;g=guards();Controller s(true);begin(s,c,g);
            if(during_run) { CHECK(s.tick(g,1000)); }
            g.*member=false;CHECK(!s.tick(g,during_run?2000:1000));CHECK(s.state()==State::fault_latched);
            g.*member=true;CHECK(!s.tick(g,3000));CHECK(!s.start(g,3000));CHECK(!s.prepare(c,g,3000));
        }
    }
    for(unsigned i=0;i<9;++i) {
        c=config();g=guards();Controller s(true);begin(s,c,g);CHECK(s.tick(g,1000));
        if(i==0) { g.manual_stop=true; }
        if(i==1) { ++g.param_epoch; }
        if(i==2) { ++g.config_hash; }
        if(i==3) { ++g.guardian_version; }
        if(i==4) { ++g.safety_reference_id; }
        if(i==5) { g.actual_position_rad=1; }
        if(i==6) { g.observed_output=2; }
        if(i==7)g.observed_output=std::numeric_limits<float>::quiet_NaN();
        if(i==8)g.actual_position_rad=std::numeric_limits<float>::quiet_NaN();
        CHECK(!s.tick(g,2000));CHECK(s.state()==State::fault_latched);
    }
    g=guards();c=config();g.actual_position_rad+=.001f;Controller offset(true);
    CHECK(offset.prepare(c,g,1000));CHECK(offset.arm(g,1000));CHECK(!offset.start(g,1000));
    g=guards();Controller s(true);begin(s,c,g);CHECK(s.tick(g,1000));s.stop(2000);s.stop(21000);
    CHECK(!s.tick(g,22000));CHECK(s.reason()==Reason::stop_timeout);
    Controller finished(true);begin(finished,c,g);CHECK(finished.tick(g,1000));finished.stop(2000);
    g.stop_confirmed=true;CHECK(!finished.tick(g,3000));CHECK(finished.state()==State::complete);
    CHECK(!finished.prepare(c,g,4000));CHECK(!finished.start(g,4000));
    Controller stale(true);begin(stale,c,g);stale.stop(2000);CHECK(!stale.tick(g,3000));CHECK(stale.state()==State::finishing);
    g=guards();Controller budget(true);c.max_output_integral=.0001f;begin(budget,c,g);
    CHECK(budget.tick(g,1000));g.observed_output=1;CHECK(!budget.tick(g,2000));CHECK(budget.reason()==Reason::output_limit);
    c=config();Controller travel(true);begin(travel,c,g=guards());CHECK(travel.tick(g,1000));
    g.actual_position_rad=.25f;CHECK(!travel.tick(g,2000));CHECK(travel.reason()==Reason::travel_limit);
}
void clock_tests() {
    auto c=config();auto g=guards();
    for(const std::uint32_t bad_delta : {0u,6000u,0xffffffffu}) {
        Controller s(true);begin(s,c,g);CHECK(s.tick(g,1000));CHECK(!s.tick(g,1000+bad_delta));CHECK(s.reason()==Reason::timing);
    }
    Controller expiry(true);c.heartbeat_timeout_us=1500;begin(expiry,c,g);CHECK(expiry.tick(g,1000));
    CHECK(!expiry.heartbeat(2500));CHECK(!expiry.tick(g,2500));CHECK(expiry.reason()==Reason::heartbeat_timeout);
    c=config();Controller wrap(true);constexpr std::uint32_t t=0xfffff000u;begin(wrap,c,g,t);
    for(std::uint32_t elapsed=0;elapsed<c.duration_us;elapsed+=1000) { CHECK(wrap.tick(g,t+elapsed)); }
    CHECK(!wrap.tick(g,t+c.duration_us));CHECK(wrap.state()==State::finishing);
    g.stop_confirmed=true;CHECK(!wrap.tick(g,t+c.duration_us+1000));CHECK(wrap.state()==State::complete);
    Controller bounded(true);g=guards();begin(bounded,c,g);
    for(std::uint32_t elapsed=0;elapsed<c.duration_us;elapsed+=1000) {
        CHECK(bounded.heartbeat(1000+elapsed));CHECK(bounded.tick(g,1000+elapsed));
    }
    CHECK(!bounded.tick(g,121000));CHECK(bounded.state()==State::finishing); // heartbeat cannot extend duration
}
int main(){generator_tests();arm_retry_tests();safety_tests();clock_tests();std::cout<<"experiment generator/fault matrix passes; seed=0x60202026\n";}
