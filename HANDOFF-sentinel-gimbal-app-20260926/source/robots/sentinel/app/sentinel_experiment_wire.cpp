#include "sentinel_experiment_wire.hpp"
#include <bit>
#include <algorithm>
namespace ec::sentinel::app::excitation {
namespace {
std::uint32_t u32(std::span<const std::uint8_t> b, std::size_t p) noexcept {
    return std::uint32_t{b[p]} | (std::uint32_t{b[p+1]}<<8) |
        (std::uint32_t{b[p+2]}<<16) | (std::uint32_t{b[p+3]}<<24);
}
float f32(std::span<const std::uint8_t> b, std::size_t p) noexcept {
    return std::bit_cast<float>(u32(b,p));
}
void put(std::span<std::uint8_t> b, std::size_t p, std::uint32_t v) noexcept {
    for (unsigned i=0;i<4;++i) { b[p+i]=static_cast<std::uint8_t>(v>>(8*i)); }
}
}
std::size_t Wire::handle(std::span<const std::uint8_t> b,const Guards& g,
    std::uint32_t now,std::span<std::uint8_t> out) noexcept {
    auto& controller_=controller_for_cycle();
    const bool v3=!b.empty() && b[0]==3;
    const bool v2=!b.empty() && (b[0]==2 || v3);
    const std::size_t reply_size=v3?160:(v2?128:32);
    if (out.size()<reply_size) { return 0; } // NO side effects if ACK cannot be encoded.
    std::fill_n(out.begin(),reply_size,0);
    out[0]=v3?3:(v2?2:1); out[1]=b.size()>1?b[1]:0; out[2]=1;
    const auto reply=[&](std::uint8_t result) {
        out[2]=result; out[3]=static_cast<std::uint8_t>(controller_.reason());
        out[4]=static_cast<std::uint8_t>(controller_.state());
        out[5]=controller_.hardware_capable()?1:0; // provisioning is not runtime authorization
        out[6]=controller_.simulation()?1:0; out[7]=1; // Pitch hardware blocked
        put(out,8,controller_.config().session_id); put(out,12,controller_.config().nonce);
        put(out,16,controller_.config().param_epoch); put(out,20,now);
        put(out,24,controller_.config().config_hash); put(out,28,controller_.config().guardian_version);
        if(v2) {
            const auto& d=controller_.deployment(); const auto& r=controller_.root();
            put(out,32,r.boot_id);
            const std::uint32_t flags=(controller_.hardware_capable()?1u:0u)|(g.guardian_approved?2u:0u)|(g.guardian_clear?4u:0u)|(g.manual_stop?8u:0u)|(g.remote_online?16u:0u)|(g.maintenance_authorized?32u:0u)|(g.operator_approved?64u:0u)|(g.feedback_fresh?128u:0u)|(g.stop_strategy_verified?256u:0u)|(g.lease_active?512u:0u)|(g.stop_policy_executed?1024u:0u)|(g.reference_valid?2048u:0u)|(g.limits_valid?4096u:0u)|(g.actuator_healthy?8192u:0u)|(g.other_axes_idle?16384u:0u)|(g.link_alive?32768u:0u)|(g.backpressure_ok?65536u:0u)|(g.output_observation_valid?131072u:0u);
            put(out,36,flags);put(out,40,g.param_epoch);put(out,44,g.config_epoch);put(out,48,g.reference_epoch);put(out,52,d.approval_record_version);
            out[56]=controller_.config().session_id?static_cast<std::uint8_t>(controller_.config().axis):0;
            out[57]=static_cast<std::uint8_t>(controller_.stop_stage());out[58]=g.guardian_state;out[59]=controller_.provisioning_state();put(out,60,r.axis_identity);
            std::copy(r.firmware_sha256.begin(),r.firmware_sha256.end(),out.begin()+64);
            std::copy(r.deployment_sha256.begin(),r.deployment_sha256.end(),out.begin()+96);
        }
        if(v3) {
            out[128]=static_cast<std::uint8_t>(controller_.position_stage());
            // The human_token echo is a consistency check, not a credential
            // channel: the host compares it with its own policy token to prove
            // the device adopted the same positioning policy. The token is a
            // 32-bit human-assertion nonce, never a secret; the authenticated
            // session is the trust boundary.
            put(out,132,controller_.position_policy().human_token);
            put(out,136,controller_.position_config_epoch());put(out,140,controller_.position_reference_epoch());
            put(out,144,std::bit_cast<std::uint32_t>(controller_.config().center_rad));
            put(out,148,std::bit_cast<std::uint32_t>(g.actual_position_rad));
            put(out,152,std::bit_cast<std::uint32_t>(g.actual_velocity_rad_s));put(out,156,g.feedback_sample_us);
        }
        return reply_size;
    };
    if (b.size()<12 || (b[0]!=1 && b[0]!=2 && !v3) || (b[2]!=5 && b[2]!=6) ||
        (b[3]!=1 && b[3]!=2) || b[1]<1 || b[1]>(v3?10:7)) { return reply(2); }
    const auto op=b[1];
    if (op==3) {
        if (b.size()!=88 || b[85]!=0 || b[86]!=0 || b[87]!=0) { return reply(2); }
        if (!g.maintenance_authorized || !g.operator_approved) { return reply(3); }
        Config c{}; c.axis=static_cast<Axis>(b[2]); c.pattern=static_cast<Pattern>(b[3]);
        c.session_id=u32(b,4); c.nonce=u32(b,8); c.param_epoch=u32(b,12);
        c.guardian_version=u32(b,16); c.config_hash=u32(b,20); c.safety_reference_id=u32(b,24);
        c.center_rad=f32(b,28); c.amplitude_rad=f32(b,32); c.min_rad=f32(b,36); c.max_rad=f32(b,40);
        c.max_velocity_rad_s=f32(b,44); c.max_acceleration_rad_s2=f32(b,48); c.max_travel_rad=f32(b,52);
        c.max_output_abs=f32(b,56); c.max_output_integral=f32(b,60);
        c.move_us=u32(b,64); c.hold_us=u32(b,68); c.duration_us=u32(b,72);
        c.heartbeat_timeout_us=u32(b,76); c.stop_timeout_us=u32(b,80); c.repeats=b[84];
        return reply(controller_.prepare(c,g,now,v3)?0:1);
    }
    const std::size_t expected=(v3 && op==8)?48:((v3 && op==9)?16:12);
    if (b.size()!=expected) { return reply(2); }
    if (op==1 || ((op==2 || op==7) && u32(b,4)==0 && u32(b,8)==0)) { return reply(0); }
    const auto& c=controller_.config();
    if (u32(b,4)!=c.session_id || u32(b,8)!=c.nonce || b[2]!=static_cast<std::uint8_t>(c.axis) ||
        b[3]!=static_cast<std::uint8_t>(c.pattern) || c.session_id==0) { return reply(4); }
    if (op==2 || op==7) {
        // Report the heartbeat outcome truthfully. ACKing a heartbeat that was
        // never applied (no authority, or the controller rejected it) would
        // leave the host believing a dead session is still alive.
        if (!g.maintenance_authorized || !g.operator_approved) { return reply(3); }
        return reply(controller_.heartbeat(now) ? 0 : 1);
    }
    if(op==6) { controller_.stop(now);return reply(0); } // matching identity STOP remains available after authority loss
    if (!g.maintenance_authorized || !g.operator_approved) { return reply(3); }
    switch (op) {
    case 8: {
        PositionPolicy p{};p.human_token=u32(b,12);p.position_tolerance_rad=f32(b,16);
        p.velocity_tolerance_rad_s=f32(b,20);p.speed_limit_rad_s=f32(b,24);p.move_us=u32(b,28);
        p.stable_dwell_us=u32(b,32);p.positioning_timeout_us=u32(b,36);p.hold_timeout_us=u32(b,40);
        p.feedback_max_age_us=u32(b,44);return reply(controller_.request_position(p,g,now)?0:1);
    }
    case 9: return reply(controller_.confirm_position(u32(b,12),g,now)?0:1);
    case 10: return reply(controller_.cancel_position(g,now)?0:1);
    case 4: return reply(controller_.arm(g,now)?0:1);
    case 5: return reply(controller_.start(g,now)?0:1);
    case 6: controller_.stop(now); return reply(0); // REQUEST accepted != stop confirmed
    default: return reply(2);
    }
}
} // namespace ec::sentinel::app::excitation
