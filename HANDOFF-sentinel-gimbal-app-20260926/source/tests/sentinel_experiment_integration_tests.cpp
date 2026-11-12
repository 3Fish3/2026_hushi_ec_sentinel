// Real v3 framing + existing app/executor fixture, no device transport.
#define main sentinel_old_protocol_fixture_main
#include "sentinel_app_protocol_tests.cpp"
#undef main
#include "sentinel_cycle.hpp"
namespace {
std::array<std::uint8_t,22> experiment_request(Rig& r,std::uint8_t op,std::uint32_t seq) {
    std::array<std::uint8_t,22> b{}; PayloadWriter w{b};
    w.u8(protocol_version);w.u8(static_cast<std::uint8_t>(MessageType::experiment));
    w.u32(r.session_id());w.u32(seq);w.u8(1);w.u8(op);w.u8(5);w.u8(1);
    w.u32(17);w.u32(31);return b;
}
void protocol_isolation() {
    Rig r; (void)r.hello();
    // Handler-only exchange; round_trip also commits ordinary safe CAN frames.
    const auto exchange=[&](std::span<const std::uint8_t> bytes) {
        return r.deliver(r.request_frame(bytes,static_cast<std::uint8_t>(MessageType::experiment)),r.now);
    };
    auto b=experiment_request(r,1,r.next_seq());
    const auto initial_sent=r.bus1.sent+r.bus2.sent;
    const auto reply=exchange(b);
    CHECK(reply.size==42);CHECK(reply.payload[1]==0xca);CHECK(reply.payload[12]==0);
    CHECK(reply.payload[15]==0); // deployment disabled
    const auto again=exchange(b);
    CHECK(again.size==reply.size);
    CHECK(std::memcmp(again.payload.data(),reply.payload.data(),reply.size)==0);
    CHECK(r.session.duplicate_requests()==1);
    b=experiment_request(r,4,r.next_seq());
    const auto arm=exchange(b);
    CHECK(arm.payload[12]!=0);
    CHECK(r.session.experiment_controller().state()==excitation::State::disabled);
    CHECK(r.bus1.sent+r.bus2.sent==initial_sent); // no new CAN from query/ARM
    b=experiment_request(r,5,r.next_seq());b[2]^=0x7f;
    const auto wrong=exchange(b);
    CHECK(wrong.payload[1]==static_cast<std::uint8_t>(MessageType::ack));
    CHECK(r.session.experiment_controller().state()==excitation::State::disabled);
}
void cycle_no_extra_output() {
    Rig plain,bound;
    SentinelCycle a(plain.app,plain.exec,plain.perception,RemoteMappingConfig{},Lifetime{100});
    SentinelCycle b(bound.app,bound.exec,bound.perception,RemoteMappingConfig{},Lifetime{100});
    b.bind_experiment(bound.session.experiment_controller());
    for (std::uint32_t t=0;t<10;++t) {
        CycleInputs in{};in.now=Tick{1000+t};in.now_us=MicroTick{(1000+t)*1000};in.dt_s=.001f;
        (void)a.control(in);(void)b.control(in);a.commit(in);b.commit(in);
        CHECK(plain.bus1.sent==bound.bus1.sent);CHECK(plain.bus2.sent==bound.bus2.sent);
        for (std::size_t j=0;j<domain_count;++j) {
            CHECK(plain.app.snapshot().executions[j].execute==bound.app.snapshot().executions[j].execute);
        }
    }
    CHECK(bound.session.experiment_controller().state()==excitation::State::disabled);
}
}
void stop_priority() {
    ReplyMux mux{50}; transport::UartFrame request{};
    request.type=static_cast<std::uint8_t>(MessageType::experiment);request.size=22;
    std::array<std::uint8_t,42> response{};response[0]=3;response[1]=0xca;
    request.payload[11]=6;request.sequence=81;CHECK(mux.enqueue(request,response));
    request.payload[11]=2;request.sequence=82;CHECK(mux.enqueue(request,response));
    request.type=static_cast<std::uint8_t>(MessageType::authorize);request.sequence=83;
    CHECK(mux.enqueue(request,response));
    // Busy DMA retains STOP, and later ordinary replies cannot replace it.
    CHECK(!mux.service(Tick{1},[](auto){return false;}));
    std::uint16_t seen{};
    CHECK(mux.service(Tick{2},[&](auto bytes){
        transport::UartFrameDecoder decoder;
        for(auto byte:bytes) { if(auto frame=decoder.push(byte)) { seen=frame->sequence; } }
        return true;
    }));CHECK(seen==81);CHECK(mux.diagnostics()[9]==2);
}
void active_experiment_retains_gains() {
    Rig r; (void)r.hello();
    constexpr auto id=param_id(5,1);
    const auto old=r.params.entry(id)->value;
    r.exec.set_experiment_active(true);
    CHECK(r.session.write_param(id,old+0.01f,true)==ParamWriteResult::reference_in_use);
    CHECK(r.params.entry(id)->value==old);
    CHECK(result_of(r.param_write(r.next_seq(),id,old+0.01f))==ResultCode::reference_in_use);
    CHECK(r.params.entry(id)->value==old);
    r.exec.set_experiment_active(false);
    CHECK(r.session.write_param(id,old,true)==ParamWriteResult::accepted);
}
int main() {protocol_isolation();cycle_no_extra_output();stop_priority();active_experiment_retains_gains();
    if(failures){return 1;}std::cout<<"experiment UART/app/cycle fail-closed integration passes\n";return 0;}
