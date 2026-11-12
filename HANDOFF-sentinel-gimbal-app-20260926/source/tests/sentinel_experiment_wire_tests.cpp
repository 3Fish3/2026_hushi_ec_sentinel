#include "sentinel_experiment_wire.hpp"
#include <array>
#include <cstdlib>
#include <bit>
#include <limits>
#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " << #x << "\n"; std::abort(); } } while(false)
#include <iostream>
using namespace ec::sentinel::app::excitation;
void simulated_wire() {
    Wire w(true);Guards g{};
    g.maintenance_authorized=g.operator_approved=g.remote_online=true;
    g.guardian_approved=g.guardian_clear=g.feedback_fresh=g.reference_valid=g.limits_valid=true;
    g.actuator_healthy=g.other_axes_idle=g.pitch_fall_arrest_verified=g.stop_strategy_verified=true;
    g.timing_valid=g.link_alive=g.backpressure_ok=g.output_observation_valid=true;
    g.param_epoch=7;g.guardian_version=1;g.config_hash=101;g.safety_reference_id=201;g.actual_position_rad=.138f;
    std::array<std::uint8_t,88> b{1,3,5,1};std::array<std::uint8_t,32> out{};
    const auto put=[&](std::size_t offset,std::uint32_t n) {
        for(unsigned i=0;i<4;++i) { b[offset+i]=static_cast<std::uint8_t>(n>>(8*i)); }
    };
    const auto f=[&](std::size_t offset,float n) {put(offset,std::bit_cast<std::uint32_t>(n));};
    put(4,17);put(8,31);put(12,7);put(16,1);put(20,101);put(24,201);
    f(28,.138f);f(32,.02f);f(36,0);f(40,.3f);f(44,2);f(48,200);f(52,.1f);f(56,1);f(60,.2f);
    put(64,40000);put(68,40000);put(72,120000);put(76,1500);put(80,20000);b[84]=1;
    CHECK(w.handle(b,g,0,std::span{out}.first(31))==0);CHECK(w.controller().config().session_id==0);
    b[85]=1;CHECK(w.handle(b,g,0,out)==32 && out[2]==2);b[85]=0;
    g.maintenance_authorized=false;CHECK(w.handle(b,g,0,out)==32 && out[2]==3);g.maintenance_authorized=true;
    f(32,std::numeric_limits<float>::quiet_NaN());CHECK(w.handle(b,g,0,out)==32 && out[2]==1);f(32,.02f);
    CHECK(w.handle(b,g,0,out)==32 && out[2]==0);CHECK(out[5]==0 && out[6]==1 && out[7]==1);
    const auto op=[&](std::uint8_t n,std::uint32_t t) {b[1]=n;CHECK(w.handle(std::span{b}.first(12),g,t,out)==32);};
    op(4,0);CHECK(out[2]==0);op(5,0);CHECK(out[2]==0);op(5,0);CHECK(out[2]!=0);
    b[8]^=1;op(6,1);CHECK(out[2]==4);CHECK(w.controller().state()==State::running);b[8]^=1;
    // Read-only/wrong-owner polling cannot keep a running session alive.
    put(4,0);put(8,0);op(2,1000);CHECK(out[2]==0);
    CHECK(!w.controller_for_cycle().tick(g,1500));CHECK(w.controller().reason()==Reason::heartbeat_timeout);
    put(4,17);put(8,31);op(5,1600);CHECK(out[2]!=0); // no reset on fault
    // Heartbeat replies are truthful: a faulted controller rejects (1) and a
    // caller without authority hears unauthorized (3), never a false ACK (0).
    op(2,1700);CHECK(out[2]==1);
    g.maintenance_authorized=false;op(2,1700);CHECK(out[2]==3);g.maintenance_authorized=true;
}
int main() {
    simulated_wire();
    Wire service; // Deployment must never unlock, even with forged network values.
    std::array<std::uint8_t, 160> response{};
    std::array<std::uint8_t, 12> req{1,1,5,1,7,0,0,0,9,0,0,0};
    Guards guards{};
    const auto n=service.handle(req,guards,0,response);
    CHECK(n==32 && response[0]==1 && response[1]==1 && response[2]==0);
    CHECK(response[5]==0); // hardware capable remains false
    req[1]=4; // ARM, still no prepare/permissions
    CHECK(service.handle(req,guards,1,response)==32 && response[2]!=0);
    req[0]=9;
    CHECK(service.handle(req,guards,2,response)==32 && response[2]!=0);
    CHECK(service.handle(std::span<const std::uint8_t>{req}.first(11),guards,3,response)==32);
    CHECK(response[2]!=0);
    req[0]=1; req[1]=1; req[2]=4; // no main yaw domain
    CHECK(service.handle(req,guards,4,response)==32 && response[2]!=0);
    CHECK(service.controller().state()==State::disabled);
    CHECK(!service.controller_for_cycle().tick(guards,5));
    std::cout << "experiment wire fail-closed contracts pass\n";
}
