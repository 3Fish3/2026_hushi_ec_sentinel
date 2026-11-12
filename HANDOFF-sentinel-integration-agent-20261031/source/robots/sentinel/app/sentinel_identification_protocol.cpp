#include "sentinel_protocol.hpp"
#include "ec/transport/uart_frame.hpp"

namespace ec::sentinel::app {
std::size_t SentinelHostSession::handle_identification(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};static_cast<void>(r.u8());static_cast<void>(r.u8());
    const auto session=r.u32(), sequence=r.u32();const auto action=r.u8();
    IdentificationPlan plan{};
    std::uint8_t schema=IdentificationPlan::schema;
    if(action==1) {
        schema=r.u8();
        plan.speed=r.f32();plan.amplitude_a=r.f32();plan.bias_a=r.f32();plan.ramp_s=r.f32();
        plan.settle_timeout_s=r.f32();plan.zero_s=r.f32();plan.acceleration=r.f32();
        plan.count=r.u8();plan.cycles=r.u8();
        for(auto& f:plan.frequencies) { f=r.f32(); }
    }
    const auto error=[&](ResultCode code) { return ack(MessageType::ack,session,sequence,code,0,response); };
    if(!r.exact()) { return error(ResultCode::bad_length); }
    const auto checked=check_session(session,sequence,true,now);
    if(!checked.proceed) { return error(checked.code); }
    auto* id=executor_ ? executor_->identification() : nullptr;
    if(!id || schema!=IdentificationPlan::schema) { return error(ResultCode::unsupported); }
    if(action>5 || (action==1 && !plan.valid())) { return error(ResultCode::invalid_value); }
    if(action!=2 && action!=4 && (!requester_holds() || role_source_!=Source::maintenance)) { return error(ResultCode::not_permitted); }
    auto* main=executor_->axis(Domain::main_yaw);
    if(action==4 && !main) { return error(ResultCode::unsupported); }
    if(action==5) {
        plan.profile=IdentificationPlan::Profile::velocity_validation;
        if(!executor_->validation_gains_match()) { return error(ResultCode::invalid_value); }
    }
    if(action==1 || action==5) {
        if(id->engaged() || !executor_->identification_ready(*app_)) { return error(ResultCode::not_permitted); }
        const auto slot=app_->dji_slot_for(Domain::main_yaw);
        const auto* device=slot ? executor_->dji_device(*slot) : nullptr;
        if(!device || device->config().mode!=drivers::DjiMode::gm6020_current) { return error(ResultCode::unsupported); }
        const float scale=device->config().direction*3.0f/16384.0f;
        id->start(plan,session,sequence,app_->config_epoch(),app_->reference_epoch(),scale);
        ++accepted_commands_;
    } else if(action==0) { id->abort(MainYawIdentification::Reason::cancelled); }
    else if(action==3) {
        if(id->active() || !domain_idle(Domain::main_yaw)) { return error(ResultCode::reference_in_use); }
        id->clear();
    }
    accept_activity(now);
    PayloadWriter w{response};w.u8(protocol_version);w.u8(static_cast<std::uint8_t>(MessageType::identification_status));
    w.u32(session);w.u32(sequence);w.u8(static_cast<std::uint8_t>(id->state));w.u8(static_cast<std::uint8_t>(id->reason));
    w.u32(id->run_id);w.u32(id->start_us);w.u32(id->produced);w.u32(id->consumed);w.u32(id->dropped);
    w.f32(id->plan.duration());w.f32(id->raw_to_axis_amp);w.u8(IdentificationPlan::schema);w.u32(id->excitation_start_us);
    if(action==4) {
        // 只读能力/实参页: 不改变V2 start/status和48字节流布局, 旧脚本仍可复核旧记录.
        w.u8(1);w.u8(static_cast<std::uint8_t>(id->plan.profile));
        w.f32(IdentificationPlan::validation_duration);w.f32(IdentificationPlan::stop_current_a);w.f32(IdentificationPlan::stop_speed);
        const auto v=main->active_velocity_pid(),p=main->position_pid();
        for(const float value:{v.kp,v.ki,v.kd,p.kp,p.ki,p.kd,v.integral_limit,p.integral_limit,
            main->current_limit(),main->config().max_dt_s,main->config().feedback_alpha}) { w.f32(value); }
        const auto& timing=executor_->runtime_timing();
        w.u32(timing.gm.period_us);w.u32(timing.direction_period_us);w.u32(timing.gm.max_step_us);
    }
    return w.overflow ? 0 : w.size();
}

std::size_t SentinelHostSession::identification_stream(std::span<std::uint8_t> payload) noexcept {
    auto* id=executor_ ? executor_->identification() : nullptr;
    if(!id || !id->engaged() || id->session!=session_id_ || !has_session_ || link_expired_) { return 0; }
    const auto count=std::min(id->pending(),MainYawIdentification::page_size);
    if(!count || (count<MainYawIdentification::page_size && id->active())) { return 0; }
    PayloadWriter w{payload};w.u8(protocol_version);w.u8(static_cast<std::uint8_t>(MessageType::identification_stream));
    w.u32(id->session);w.u32(id->run_id);w.u32(id->start_us);w.u32(id->consumed);
    w.u8(static_cast<std::uint8_t>(count));w.u8(static_cast<std::uint8_t>(id->state));w.u32(id->dropped);w.u8(IdentificationPlan::schema);
    for(unsigned i=0;i<count;++i) {
        const auto& s=id->sample(id->consumed+i);
        w.u32(s.at_us);w.u32(s.submit_us);w.f32(s.reference);w.f32(s.feedback);w.f32(s.joint);w.f32(s.heading_gain);
        w.f32(s.p);w.f32(s.i);w.u16(static_cast<std::uint16_t>(s.command_raw));w.u16(static_cast<std::uint16_t>(s.effort_raw));
        w.u16(s.source_age_us);w.u16(s.flags);w.f32(s.bias_a);w.f32(s.excitation_a);
    }
    return w.overflow ? 0 : w.size();
}
} // 命名空间 ec::sentinel::app
