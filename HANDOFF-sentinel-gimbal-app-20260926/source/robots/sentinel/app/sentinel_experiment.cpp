#include "sentinel_experiment.hpp"
#include <cmath>
#include <initializer_list>
#include <bit>
#include <algorithm>

namespace ec::sentinel::app::excitation {
namespace {
constexpr std::uint32_t half_clock=0x80000000u;
bool finite(float x) noexcept { return std::isfinite(x); }
bool forward(std::uint32_t now, std::uint32_t before) noexcept { return now-before < half_clock; }
}
bool valid_config(const Config& c) noexcept {
    if ((c.axis!=Axis::fire_yaw && c.axis!=Axis::pitch) ||
        (c.pattern!=Pattern::bounded_step && c.pattern!=Pattern::trapezoid)) { return false; }
    for (const float v : {c.center_rad,c.amplitude_rad,c.min_rad,c.max_rad,c.max_velocity_rad_s,
         c.max_acceleration_rad_s2,c.max_travel_rad,c.max_output_abs,c.max_output_integral}) {
        if (!finite(v)) { return false; }
    }
    if (!c.session_id || !c.nonce || !c.param_epoch || !c.guardian_version || !c.config_hash ||
        !c.safety_reference_id || c.repeats<1 || c.repeats>4 || c.move_us<2000 ||
        c.duration_us<1000 || c.duration_us>180000 || !c.heartbeat_timeout_us ||
        c.heartbeat_timeout_us>1000000 || !c.stop_timeout_us || c.stop_timeout_us>1000000 ||
        c.min_rad>=c.max_rad || c.amplitude_rad==0 || c.max_velocity_rad_s<=0 ||
        c.max_acceleration_rad_s2<=0 || c.max_travel_rad<=0 || c.max_output_abs<=0 ||
        c.max_output_integral<=0) { return false; }
    const double center=c.center_rad, amplitude=c.amplitude_rad, target=center+amplitude;
    if (center<c.min_rad || center>c.max_rad || target<c.min_rad || target>c.max_rad) { return false; }
    const auto schedule=(2ull*c.move_us+c.hold_us)*c.repeats;
    if (schedule>c.duration_us || 2.0*std::abs(amplitude)*c.repeats>c.max_travel_rad) { return false; }
    const double seconds=c.move_us*1e-6, a=std::abs(amplitude);
    const double peak_v=(c.pattern==Pattern::bounded_step ? 1.875 : 4.0/3.0)*a/seconds;
    const double peak_a=(c.pattern==Pattern::bounded_step ? 5.773502691896258 : 16.0/3.0)*a/(seconds*seconds);
    return peak_v<=c.max_velocity_rad_s && peak_a<=c.max_acceleration_rad_s2;
}
std::optional<Reference> generate(const Config& c, std::uint32_t elapsed) noexcept {
    if (!valid_config(c) || elapsed>=c.duration_us) { return std::nullopt; }
    const std::uint64_t period=2ull*c.move_us+c.hold_us;
    if (elapsed>=period*c.repeats) { return Reference{c.center_rad,0,0}; }
    const auto phase=static_cast<std::uint32_t>(elapsed%period);
    bool returning=false; std::uint32_t t=phase;
    if (phase>=c.move_us && phase<c.move_us+c.hold_us) {
        return Reference{static_cast<float>(double(c.center_rad)+c.amplitude_rad),0,0};
    }
    if (phase>=c.move_us+c.hold_us) { returning=true; t=phase-c.move_us-c.hold_us; }
    const double x=double(t)/c.move_us, seconds=c.move_us*1e-6;
    double p{},v{},a{};
    if (c.pattern==Pattern::bounded_step) {
        p=x*x*x*(10.0+x*(-15.0+6.0*x));
        v=30.0*x*x*(1.0-x)*(1.0-x);
        a=60.0*x*(1.0-x)*(1.0-2.0*x);
    } else {
        constexpr double acceleration=16.0/3.0, velocity=4.0/3.0;
        if (x<.25) { p=.5*acceleration*x*x; v=acceleration*x; a=acceleration; }
        else if (x<.75) { p=1.0/6.0+velocity*(x-.25); v=velocity; a=0; }
        else { const double remain=1.0-x; p=1.0-.5*acceleration*remain*remain; v=acceleration*remain; a=-acceleration; }
    }
    const double sign=returning?-1.0:1.0, amplitude=c.amplitude_rad;
    const Reference r{static_cast<float>(double(c.center_rad)+amplitude*(returning?1.0-p:p)),
        static_cast<float>(sign*amplitude*v/seconds),static_cast<float>(sign*amplitude*a/(seconds*seconds))};
    if (!finite(r.position_rad) || !finite(r.velocity_rad_s) || !finite(r.acceleration_rad_s2)) { return std::nullopt; }
    return r;
}

std::array<std::uint8_t,32> deployment_digest(const TrustedDeploymentSafetyConfig& d) noexcept {
    // Canonical v1 little-endian record, no structure padding / native endian.
    std::array<std::uint8_t,256> bytes{}; std::size_t n=0;
    const auto put=[&](std::uint32_t v){for(unsigned i=0;i<4;++i) bytes[n++]=static_cast<std::uint8_t>(v>>(8*i));};
    put(1); for(auto v:d.firmware_sha256) bytes[n++]=v;
    for(auto v:{d.axis_identity,d.config_epoch,d.reference_epoch,d.guardian_version,d.config_hash,d.safety_reference_id,d.approval_record_version}) put(v);
    for(float v:{d.min_rad,d.max_rad,d.max_velocity_rad_s,d.max_acceleration_rad_s2,d.max_travel_rad,d.max_output_abs,d.max_output_integral,d.stop_velocity_rad_s,d.stop_position_delta_rad}) put(std::bit_cast<std::uint32_t>(v));
    for(auto v:{d.max_duration_us,d.max_heartbeat_timeout_us,d.max_stop_timeout_us,d.stop_dwell_us,static_cast<std::uint32_t>(d.stop_policy)}) put(v);
    const auto bits=static_cast<std::uint64_t>(n)*8; bytes[n++]=0x80;
    while(n%64!=56) bytes[n++]=0;
    for(int i=7;i>=0;--i) bytes[n++]=static_cast<std::uint8_t>(bits>>(8*i));
    constexpr std::array<std::uint32_t,64> k{0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
    std::array<std::uint32_t,8> h{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    for(std::size_t block=0;block<n;block+=64) {
        std::array<std::uint32_t,64> w{};
        for(unsigned i=0;i<16;++i) for(unsigned j=0;j<4;++j) w[i]=(w[i]<<8)|bytes[block+4*i+j];
        for(unsigned i=16;i<64;++i) {const auto a=w[i-15],b=w[i-2];w[i]=w[i-16]+(std::rotr(a,7)^std::rotr(a,18)^(a>>3))+w[i-7]+(std::rotr(b,17)^std::rotr(b,19)^(b>>10));}
        auto a=h[0],b=h[1],c=h[2],e=h[4],f=h[5],g=h[6],hh=h[7],dd=h[3];
        for(unsigned i=0;i<64;++i) {const auto t1=hh+(std::rotr(e,6)^std::rotr(e,11)^std::rotr(e,25))+((e&f)^(~e&g))+k[i]+w[i];const auto t2=(std::rotr(a,2)^std::rotr(a,13)^std::rotr(a,22))+((a&b)^(a&c)^(b&c));hh=g;g=f;f=e;e=dd+t1;dd=c;c=b;b=a;a=t1+t2;}
        h[0]+=a;h[1]+=b;h[2]+=c;h[3]+=dd;h[4]+=e;h[5]+=f;h[6]+=g;h[7]+=hh;
    }
    std::array<std::uint8_t,32> out{};for(unsigned i=0;i<8;++i)for(unsigned j=0;j<4;++j)out[4*i+j]=static_cast<std::uint8_t>(h[i]>>(24-8*j));return out;
}
Controller::Controller(const TrustedDeploymentSafetyConfig& d,const TrustedDeploymentRoot& r) noexcept
    : simulation_(false),deployment_(d),root_(r),state_(State::disabled),reason_(Reason::hardware_blocked) {
    const auto nonzero=[](const auto& a){return std::any_of(a.begin(),a.end(),[](auto x){return x!=0;});};
    if(!d.approval_record_version && !nonzero(d.deployment_sha256)) return;
    provisioning_state_=2;
    if(!r.boot_id || !r.axis_identity || r.axis_identity!=d.axis_identity || !d.config_epoch || !d.reference_epoch || !d.guardian_version || !d.config_hash || !d.safety_reference_id || !d.approval_record_version || !nonzero(r.firmware_sha256) || !nonzero(r.deployment_sha256) || r.firmware_sha256!=d.firmware_sha256 || r.deployment_sha256!=d.deployment_sha256 || deployment_digest(d)!=d.deployment_sha256) return;
    for(float v:{d.min_rad,d.max_rad,d.max_velocity_rad_s,d.max_acceleration_rad_s2,d.max_travel_rad,d.max_output_abs,d.max_output_integral,d.stop_velocity_rad_s,d.stop_position_delta_rad}) if(!finite(v)) return;
    if(d.min_rad>=d.max_rad || d.max_velocity_rad_s<=0 || d.max_acceleration_rad_s2<=0 || d.max_travel_rad<=0 || d.max_output_abs<=0 || d.max_output_integral<=0 || d.stop_velocity_rad_s<=0 || d.stop_position_delta_rad<=0 || !d.stop_dwell_us || !d.max_stop_timeout_us || d.stop_dwell_us>=d.max_stop_timeout_us || d.max_stop_timeout_us>1000000 || !d.max_duration_us || d.max_duration_us>180000 || !d.max_heartbeat_timeout_us || d.max_heartbeat_timeout_us>1000000 || d.stop_policy!=StopPolicy::zero_output) return;
    hardware_=true;provisioning_state_=1;state_=State::idle;reason_=Reason::none;
}
bool Controller::within_deployment(const Config& c) const noexcept {
    const auto& d=deployment_;
    return c.axis==Axis::fire_yaw && c.guardian_version==d.guardian_version && c.config_hash==d.config_hash && c.safety_reference_id==d.safety_reference_id && c.min_rad>=d.min_rad && c.max_rad<=d.max_rad && c.max_velocity_rad_s<=d.max_velocity_rad_s && c.max_acceleration_rad_s2<=d.max_acceleration_rad_s2 && c.max_travel_rad<=d.max_travel_rad && c.max_output_abs<=d.max_output_abs && c.max_output_integral<=d.max_output_integral && c.duration_us<=d.max_duration_us && c.heartbeat_timeout_us<=d.max_heartbeat_timeout_us && c.stop_timeout_us<=d.max_stop_timeout_us && c.stop_timeout_us>d.stop_dwell_us;
}
void Controller::reference_withdrawn(std::uint32_t now) noexcept {
    if(!hardware_ || (state_!=State::finishing && state_!=State::fault_latched) || stop_stage_==StopStage::stop_unknown || stop_stage_==StopStage::complete) return;
    if(stop_stage_==StopStage::none) {finish_us_=now;stop_stage_=StopStage::stop_requested;}
    if(stop_stage_==StopStage::stop_requested) {stop_stage_=StopStage::reference_withdrawn;withdrawn_us_=now;dwell_started_=false;stop_sample_us_=now;}
}
void Controller::observe_stop(const Guards& g,std::uint32_t now) noexcept {
    if(!hardware_ || stop_stage_==StopStage::none || stop_stage_==StopStage::complete || stop_stage_==StopStage::stop_unknown) return;
    if(!forward(now,finish_us_) || now-finish_us_>=config_.stop_timeout_us) {stop_stage_=StopStage::stop_unknown;if(state_!=State::fault_latched) fault(Reason::stop_timeout);return;}
    if(stop_stage_==StopStage::stop_requested) return;
    // Accepted command evidence must be after this withdrawal and never from the future.
    // Reusing an original timestamp across non-due cycles does not renew its TTL.
    if(!g.stop_policy_executed || !forward(now,g.stop_policy_sample_us) ||
       !forward(g.stop_policy_sample_us,withdrawn_us_) || g.stop_policy_sample_us==withdrawn_us_) {
        dwell_started_=false;stop_stage_=StopStage::reference_withdrawn;return;
    }
    if(stop_stage_!=StopStage::policy_executing) stop_policy_start_us_=g.stop_policy_sample_us;
    stop_stage_=StopStage::policy_executing;
    if(!g.feedback_fresh || !finite(g.actual_position_rad) || !finite(g.actual_velocity_rad_s) || !forward(now,g.feedback_sample_us) || !forward(g.feedback_sample_us,stop_policy_start_us_) || g.feedback_sample_us==stop_policy_start_us_ || !forward(g.feedback_sample_us,stop_sample_us_) || g.feedback_sample_us==stop_sample_us_ || std::abs(g.actual_velocity_rad_s)>deployment_.stop_velocity_rad_s) {dwell_started_=false;return;}
    stop_sample_us_=g.feedback_sample_us;
    if(!dwell_started_ || std::abs(g.actual_position_rad-stop_position_)>deployment_.stop_position_delta_rad) {dwell_started_=true;dwell_start_us_=g.feedback_sample_us;stop_position_=g.actual_position_rad;return;}
    if(g.feedback_sample_us-dwell_start_us_>=deployment_.stop_dwell_us) {stop_stage_=StopStage::complete;if(state_!=State::fault_latched)state_=State::complete;}
}

Reason Controller::check(const Guards& g, bool require_capture) const noexcept {
    if (hardware_ && (g.config_epoch!=deployment_.config_epoch || g.reference_epoch!=deployment_.reference_epoch || g.axis_identity!=deployment_.axis_identity)) return Reason::identity;
    if (g.manual_stop) { return Reason::manual_stop; }
    if (!g.maintenance_authorized || !g.operator_approved || !g.remote_online) { return Reason::authorization; }
    if (!g.guardian_approved || !g.guardian_clear) { return Reason::guardian; }
    if (!g.feedback_fresh) { return Reason::feedback; }
    if (!g.reference_valid || !finite(g.actual_position_rad)) { return Reason::reference; }
    if (!g.limits_valid) { return Reason::limits; }
    if (!g.actuator_healthy || !g.stop_strategy_verified) { return Reason::actuator; }
    if (!g.other_axes_idle) { return Reason::other_axes; }
    if (config_.axis==Axis::pitch && !g.pitch_fall_arrest_verified) { return Reason::pitch_blocked; }
    if (!g.timing_valid) { return Reason::timing; }
    if (!g.link_alive) { return Reason::link; }
    // Short-window capture is required for START/running, not human positioning/hold.
    if (require_capture && !g.backpressure_ok) { return Reason::backpressure; }
    if (!g.output_observation_valid || !finite(g.observed_output)) { return Reason::observation; }
    if (g.param_epoch!=config_.param_epoch || g.guardian_version!=config_.guardian_version ||
        g.config_hash!=config_.config_hash || g.safety_reference_id!=config_.safety_reference_id) { return Reason::identity; }
    if (g.actual_position_rad<config_.min_rad || g.actual_position_rad>config_.max_rad) { return Reason::position_limit; }
    if (hardware_ && (!finite(g.actual_velocity_rad_s) || std::abs(g.actual_velocity_rad_s)>config_.max_velocity_rad_s)) { return Reason::limits; }
    if (std::abs(g.observed_output)>config_.max_output_abs) { return Reason::output_limit; }
    return Reason::none;
}
bool Controller::prepare(const Config& c, const Guards& g, std::uint32_t now, bool positioned_session) noexcept {
    if (!enabled()) { return false; }
    // One-shot sessions: no reset/re-arm, even after completion. Recreate only an offline instance.
    if (state_!=State::idle || prepared_) { return false; }
    if (!valid_config(c) || (hardware_ && !within_deployment(c))) { reason_=Reason::invalid_config; return false; }
    config_=c;
    const auto r=check(g,!positioned_session);
    if (r!=Reason::none) { reason_=r; return false; }
    positioned_session_=positioned_session;
    prepared_=true; reason_=Reason::none; heartbeat_us_=now; return true;
}
bool Controller::arm(const Guards& g, std::uint32_t now) noexcept {
    if (!enabled() || state_!=State::idle || !prepared_) { return false; }
    const auto r=check(g,!positioned_session_);
    // Like prepare(), a failed ARM latches nothing: no reference has been
    // issued yet, so a transient denial (e.g. one stale feedback frame) must
    // not burn the one-shot session. The host may retry ARM; START and tick
    // (once running or positioning) still latch on any denial.
    if (r!=Reason::none) { reason_=r; return false; }
    if (!forward(now,heartbeat_us_) || now-heartbeat_us_>=config_.heartbeat_timeout_us) { reason_=Reason::heartbeat_timeout; return false; }
    state_=State::armed; last_us_=now; reason_=Reason::none; return true;
}
bool Controller::position_feedback_valid(const Guards& g, std::uint32_t now) const noexcept {
    return g.feedback_fresh && finite(g.actual_position_rad) && finite(g.actual_velocity_rad_s) &&
        forward(now,g.feedback_sample_us) && now-g.feedback_sample_us<position_policy_.feedback_max_age_us &&
        forward(g.feedback_sample_us,position_sample_us_) &&
        g.config_epoch==position_config_epoch_ && g.reference_epoch==position_reference_epoch_ &&
        g.axis_identity==position_axis_identity_;
}
bool Controller::position_at_target(const Guards& g) const noexcept {
    return std::abs(double(g.actual_position_rad)-config_.center_rad)<=position_policy_.position_tolerance_rad &&
        std::abs(g.actual_velocity_rad_s)<=position_policy_.velocity_tolerance_rad_s;
}
bool Controller::request_position(const PositionPolicy& p,const Guards& g,std::uint32_t now) noexcept {
    if(!enabled() || state_!=State::armed || position_stage_!=PositionStage::none) return false;
    const auto r=check(g,false); if(r!=Reason::none) { fault(r);return false; }
    if(!p.human_token || !finite(p.position_tolerance_rad) || !finite(p.velocity_tolerance_rad_s) ||
       !finite(p.speed_limit_rad_s) || p.position_tolerance_rad<=0 || p.velocity_tolerance_rad_s<=0 ||
       p.speed_limit_rad_s<=0 || p.speed_limit_rad_s>config_.max_velocity_rad_s ||
       p.velocity_tolerance_rad_s>p.speed_limit_rad_s ||
       config_.center_rad-p.position_tolerance_rad<config_.min_rad ||
       config_.center_rad+p.position_tolerance_rad>config_.max_rad ||
       !p.move_us || !p.stable_dwell_us || p.move_us>=p.positioning_timeout_us ||
       p.stable_dwell_us>=p.positioning_timeout_us-p.move_us ||
       p.positioning_timeout_us>=half_clock || !p.hold_timeout_us || p.hold_timeout_us>=half_clock ||
       !p.feedback_max_age_us || p.feedback_max_age_us>config_.heartbeat_timeout_us ||
       !finite(g.actual_velocity_rad_s) || std::abs(g.actual_velocity_rad_s)>p.velocity_tolerance_rad_s ||
       !forward(now,g.feedback_sample_us) || now-g.feedback_sample_us>=p.feedback_max_age_us ||
       !forward(now,heartbeat_us_) || now-heartbeat_us_>=config_.heartbeat_timeout_us) return false;
    const double distance=std::abs(double(config_.center_rad)-g.actual_position_rad), seconds=p.move_us*1e-6;
    if(1.875*distance/seconds>p.speed_limit_rad_s ||
       5.773502691896258*distance/(seconds*seconds)>config_.max_acceleration_rad_s2 ||
       distance+2.0*std::abs(config_.amplitude_rad)*config_.repeats>config_.max_travel_rad ||
       (hardware_ && p.positioning_timeout_us>deployment_.max_duration_us)) return false;
    position_policy_=p;position_stage_=PositionStage::moving;position_start_us_=last_us_=now;
    position_origin_rad_=last_position_=g.actual_position_rad;position_sample_us_=g.feedback_sample_us;
    position_config_epoch_=g.config_epoch;position_reference_epoch_=g.reference_epoch;position_axis_identity_=g.axis_identity;
    travel_=output_integral_=0; first_tick_=true; stable_started_=hold_started_=false;
    return true;
}
bool Controller::confirm_position(std::uint32_t token,const Guards& g,std::uint32_t now) noexcept {
    if(!enabled() || state_!=State::armed || position_stage_!=PositionStage::wait_human ||
       !token || token!=position_policy_.human_token) return false;
    const auto r=check(g,false);
    if(r!=Reason::none) {fault(r);return false;}
    if(!position_feedback_valid(g,now) || !position_at_target(g) ||
       !forward(now,hold_start_us_) || now-hold_start_us_>=position_policy_.hold_timeout_us ||
       !forward(now,heartbeat_us_) || now-heartbeat_us_>=config_.heartbeat_timeout_us) {
        fault(Reason::feedback);return false;
    }
    // This is an explicit host assertion of HUMAN action, not MCU proof of a human.
    position_stage_=PositionStage::confirmed;return true;
}
bool Controller::cancel_position(const Guards& g,std::uint32_t now) noexcept {
    if(!enabled() || state_!=State::armed || position_stage_==PositionStage::none ||
       position_stage_==PositionStage::moving || position_stage_==PositionStage::invalidated) return false;
    const auto r=check(g,false);if(r!=Reason::none) {fault(r);return false;}
    if(!position_feedback_valid(g,now)) {fault(Reason::feedback);return false;}
    position_policy_.human_token=0;position_stage_=PositionStage::cancelled_hold;return true;
}
std::optional<Reference> Controller::position_tick(const Guards& g,std::uint32_t now) noexcept {
    if(!position_feedback_valid(g,now)) {fault(Reason::feedback);return std::nullopt;}
    if(hold_started_ && (!forward(now,hold_start_us_) || now-hold_start_us_>=position_policy_.hold_timeout_us)) {
        fault(Reason::stop_timeout);return std::nullopt;
    }
    if(state_==State::armed && std::abs(g.actual_velocity_rad_s)>position_policy_.speed_limit_rad_s) {fault(Reason::limits);return std::nullopt;}
    if(state_==State::running) {position_sample_us_=g.feedback_sample_us;return generate(config_,now-start_us_);}
    const auto elapsed=now-position_start_us_;
    if(!hold_started_ && elapsed>=position_policy_.positioning_timeout_us) {fault(Reason::stop_timeout);return std::nullopt;}
    if(position_stage_==PositionStage::moving && elapsed<position_policy_.move_us) {
        position_sample_us_=g.feedback_sample_us;
        const double x=double(elapsed)/position_policy_.move_us, s=position_policy_.move_us*1e-6;
        const double d=double(config_.center_rad)-position_origin_rad_;
        return Reference{static_cast<float>(position_origin_rad_+d*x*x*x*(10+x*(-15+6*x))),
            static_cast<float>(d*30*x*x*(1-x)*(1-x)/s),static_cast<float>(d*60*x*(1-x)*(1-2*x)/(s*s))};
    }
    if(position_stage_==PositionStage::moving) position_stage_=PositionStage::settling;
    if(position_stage_!=PositionStage::holding_complete && position_stage_!=PositionStage::cancelled_hold) {
        if(!position_at_target(g)) {
            // A lost confirmation is never automatically restored by re-settling.
            if(position_stage_==PositionStage::confirmed) position_policy_.human_token=0;
            position_stage_=PositionStage::settling;stable_started_=false;
        } else if(g.feedback_sample_us!=position_sample_us_) {
            if(!stable_started_) {stable_started_=true;stable_start_us_=g.feedback_sample_us;}
            if(position_stage_==PositionStage::settling && g.feedback_sample_us-stable_start_us_>=position_policy_.stable_dwell_us) {
                position_stage_=position_policy_.human_token?PositionStage::wait_human:PositionStage::cancelled_hold;
                if(!hold_started_) {hold_started_=true;hold_start_us_=now;}
            }
        }
    }
    position_sample_us_=g.feedback_sample_us;
    return Reference{config_.center_rad,0,0};
}
bool Controller::start(const Guards& g, std::uint32_t now) noexcept {
    if (!enabled() || state_!=State::armed) { return false; }
    const auto r=check(g);
    if (r!=Reason::none) { fault(r); return false; }
    if (!forward(now,last_us_) || !forward(now,heartbeat_us_) || now-heartbeat_us_>=config_.heartbeat_timeout_us) { fault(Reason::heartbeat_timeout); return false; }
    // Hardware START cannot bypass measured positioning + separate HUMAN confirmation.
    if((hardware_ || positioned_session_) && position_stage_==PositionStage::none) return false;
    if(position_stage_!=PositionStage::none) {
        if(position_stage_!=PositionStage::confirmed) return false;
        // Check the actual recorder deadline at MCU command execution, not host ACK time.
        if(g.capture_remaining_us < config_.duration_us ||
           now-g.capture_sample_us > g.capture_remaining_us-config_.duration_us) {
            fault(Reason::backpressure);return false;
        }
        if(!position_feedback_valid(g,now) || !position_at_target(g) ||
           !forward(now,hold_start_us_) || now-hold_start_us_>=position_policy_.hold_timeout_us) {
            fault(Reason::feedback);return false;
        }
        position_stage_=PositionStage::running;position_policy_.human_token=0;
    } else if (std::abs(double(g.actual_position_rad)-config_.center_rad)>1e-6) {
        // Legacy offline v1/v2 simulation only; NOT a physical positioning policy.
        fault(Reason::reference); return false;
    }
    state_=State::running; start_us_=last_us_=now; first_tick_=true;
    previous_stop_confirmed_=g.stop_confirmed; last_position_=g.actual_position_rad;
    if(position_stage_==PositionStage::none) { travel_=output_integral_=0; }
    return true;
}
bool Controller::heartbeat(std::uint32_t now) noexcept {
    if (!enabled() || (state_!=State::armed && state_!=State::running) ||
        !forward(now,heartbeat_us_) || now-heartbeat_us_>=config_.heartbeat_timeout_us) { return false; }
    heartbeat_us_=now; return true;
}
void Controller::stop(std::uint32_t now) noexcept {
    if (!enabled() || state_==State::disabled || state_==State::complete || state_==State::fault_latched || state_==State::finishing) { return; }
    if (prepared_) {
        if (position_stage_ != PositionStage::none) {
            position_stage_ = PositionStage::invalidated; position_policy_.human_token=0;
            finish(now);
        } else if (state_ == State::idle || state_ == State::armed) {
            // Never driven: tick() issues no reference before START and no
            // positioning motion was requested, so there is no motion to
            // confirm stopped. Entering the stop-confirmation chain here
            // could only manufacture a stop_timeout fault on a still axis.
            stop_stage_ = StopStage::complete; state_ = State::complete;
        } else { finish(now); }
    }
}
std::optional<Reference> Controller::tick(const Guards& g, std::uint32_t now) noexcept {
    if (!enabled() || state_==State::disabled || state_==State::idle || state_==State::complete || (!hardware_ && state_==State::fault_latched)) { return std::nullopt; }
    if (hardware_ && (state_==State::finishing || state_==State::fault_latched)) { observe_stop(g,now);return std::nullopt; }
    if (state_==State::finishing) {
        if (!forward(now,finish_us_) || now-finish_us_>=config_.stop_timeout_us) { fault(Reason::stop_timeout); }
        else if (g.stop_confirmed && !previous_stop_confirmed_) { state_=State::complete; }
        previous_stop_confirmed_=g.stop_confirmed; return std::nullopt;
    }
    const bool require_capture=state_==State::running || (!positioned_session_ && position_stage_==PositionStage::none);
    const auto r=check(g,require_capture);
    if (r!=Reason::none) {
        // Armed but never driven (no reference before START, no positioning
        // motion): a transient guard denial must not burn the one-shot
        // session; it self-heals on the next clean tick. Persistent denial
        // is still enforced at the START gate, and any denial latches once
        // running or positioning. Timing/heartbeat faults below still latch.
        if (state_==State::armed && position_stage_==PositionStage::none) { return std::nullopt; }
        fault(r); return std::nullopt;
    }
    if (!forward(now,last_us_) || (!first_tick_ && now==last_us_) || now-last_us_>5000) { fault(Reason::timing); return std::nullopt; }
    if (!forward(now,heartbeat_us_) || now-heartbeat_us_>=config_.heartbeat_timeout_us) { fault(Reason::heartbeat_timeout); return std::nullopt; }
    if (state_==State::armed && position_stage_==PositionStage::none) { last_us_=now; first_tick_=false; return std::nullopt; }
    const auto dt=(now-last_us_)*1e-6;
    travel_+=std::abs(double(g.actual_position_rad)-last_position_);
    output_integral_+=std::abs(double(g.observed_output))*dt;
    if (travel_>config_.max_travel_rad) { fault(Reason::travel_limit); return std::nullopt; }
    if (output_integral_>config_.max_output_integral) { fault(Reason::output_limit); return std::nullopt; }
    last_us_=now; first_tick_=false; last_position_=g.actual_position_rad;
    previous_stop_confirmed_=g.stop_confirmed;
    if(position_stage_!=PositionStage::none) {
        if(state_==State::running && now-start_us_>=config_.duration_us) {
            // Nominal completion is still an actively monitored hold, not torque-off.
            state_=State::armed;position_stage_=PositionStage::holding_complete;
        }
        return position_tick(g,now);
    }
    if (now-start_us_>=config_.duration_us) { finish(now); return std::nullopt; }
    return generate(config_,now-start_us_);
}
} // namespace ec::sentinel::app::excitation
