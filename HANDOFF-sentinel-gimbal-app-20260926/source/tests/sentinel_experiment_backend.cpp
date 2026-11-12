// Native offline generator used by the CLI. Links the SAME MCU pure generator/FSM.
#include "sentinel_experiment.hpp"
#include <iostream>
#include <string_view>
using namespace ec::sentinel::app::excitation;
int main(int argc,char** argv) {
    if (argc!=3) { return 2; }
    const std::string_view axis{argv[1]}, pattern{argv[2]};
    if ((axis!="fire_yaw" && axis!="pitch") || (pattern!="bounded_step" && pattern!="trapezoid")) { return 2; }
    Config c{}; c.session_id=17; c.nonce=31; c.param_epoch=7;
    c.guardian_version=1; c.config_hash=101; c.safety_reference_id=201;
    c.axis=axis=="pitch"?Axis::pitch:Axis::fire_yaw;
    c.pattern=pattern=="trapezoid"?Pattern::trapezoid:Pattern::bounded_step;
    c.center_rad=0.138f; c.amplitude_rad=0.02f; c.min_rad=0.0f; c.max_rad=0.3f;
    c.max_velocity_rad_s=2.0f; c.max_acceleration_rad_s2=200.0f; c.max_travel_rad=0.1f;
    c.max_output_abs=1.0f; c.max_output_integral=0.2f;
    c.move_us=40000; c.hold_us=40000; c.duration_us=120000; c.repeats=1;
    c.heartbeat_timeout_us=200000; c.stop_timeout_us=20000;
    Guards g{}; g.maintenance_authorized=g.operator_approved=g.remote_online=true;
    g.guardian_approved=g.guardian_clear=g.feedback_fresh=g.reference_valid=g.limits_valid=true;
    g.actuator_healthy=g.other_axes_idle=g.pitch_fall_arrest_verified=g.stop_strategy_verified=true;
    g.timing_valid=g.link_alive=g.backpressure_ok=g.output_observation_valid=true;
    g.param_epoch=c.param_epoch; g.guardian_version=c.guardian_version;
    g.config_hash=c.config_hash; g.safety_reference_id=c.safety_reference_id;
    g.actual_position_rad=c.center_rad;
    Controller sim{true};
    if (!sim.prepare(c,g,1000) || !sim.arm(g,1000) || !sim.start(g,1000)) { return 3; }
    std::cout.precision(9);
    std::cout << "sampled_us,generated_reference_rad,generated_velocity_rad_s,generated_acceleration_rad_s2,state\n";
    for (std::uint32_t t=0;t<c.duration_us;t+=1000) {
        const auto r=sim.tick(g,1000+t);
        if (!r) { return 4; }
        std::cout << t << ',' << r->position_rad << ',' << r->velocity_rad_s << ','
            << r->acceleration_rad_s2 << ',' << static_cast<unsigned>(sim.state()) << '\n';
    }
    // Completion is NOT transport acceptance: explicitly simulated actuator confirmation.
    (void)sim.tick(g,1000+c.duration_us); g.stop_confirmed=true;
    (void)sim.tick(g,2000+c.duration_us);
    return sim.state()==State::complete?0:5;
}
