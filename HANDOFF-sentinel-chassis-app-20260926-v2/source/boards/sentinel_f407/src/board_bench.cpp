#include "board_bench.hpp"
#if EC_S5_GM6020_BENCH
#include "gm6020_bench_trace.hpp"
#include "board_io.h"
#include "ec/platform/stm32f407.hpp"
#include "can.h"
extern "C" { const volatile std::uint32_t g_ec_s5_signature=ec::sentinel::bench_magic; }
namespace {
static_assert(EC_S5_GM6020_ID>=1 && EC_S5_GM6020_ID<=7);
ec::sentinel::Gm6020Bench bench{EC_S5_GM6020_ID};
ec::MicroTick micros() { return {ec_board_micros()}; }
// 本台架独占 CAN1, 设备和命令组/槽来自同一映射, 不改电机保存 ID.
ec::stm32f407::HalCanTx tx{hcan1};
ec::sentinel::BenchObservedTx observed{tx,micros};
ec::drivers::DjiTxGroup group{observed,1,bench.group_number(),{20},bench.group_devices()};
ec::sentinel::BenchTrace trace;
std::uint32_t tx_errors{};
ec::drivers::DjiGroupStatus tx_status{};
}
void ec_bench_receive(std::uint8_t bus,const ec::transport::CanFrame& frame,ec::Tick at,ec::MicroTick at_us) {
    bench.receive(bus,frame,at,at_us);
}
void ec_bench_transport_lost() { if(bench.active()){bench.stop(ec::sentinel::BenchReason::command_timeout);} }
bool ec_bench_request(const ec::transport::UartFrame& frame) {
    if(frame.type<0x20 || (frame.type>0x25 && frame.type!=0x27)){return false;}
    const auto now=ec::stm32f407::now();
    if(frame.type==0x20 && frame.size==0){return true;}
    if(frame.type==0x23 && frame.size==0){bench.stop();return true;}
    if(frame.type==0x24 && frame.size==0){if(!bench.set_reference(now)){bench.invalid_request();}return true;}
    const std::span<const std::uint8_t> bytes{frame.payload.data(),frame.size};
    if(frame.type==0x27){
        (void)ec::sentinel::apply_bench_excitation(bench,bytes,now,micros());return true;
    }
    if(frame.type==0x22 && frame.size==2){
        (void)bench.heartbeat(ec::sentinel::bench_wire::get16(bytes,0),now);return true;
    }
    if(frame.type==0x21 && frame.size==ec::sentinel::bench_settings_size){
        const auto cfg=ec::sentinel::decode_bench_settings(bytes);
        if(cfg){(void)bench.start(*cfg,frame.sequence,now,micros());}else{bench.invalid_request();}
        return true;
    }
    if(frame.type==0x25 && frame.size==10){
        using namespace ec::sentinel::bench_wire;
        (void)bench.update_target(get16(bytes,0),get_float(bytes,2),get32(bytes,6),now,micros());return true;
    }
    bench.invalid_request();return true;
}
extern "C" void ec_bench_tick() {
    const auto now=ec::stm32f407::now();
    const auto amperes=bench.step(now,micros());
    auto sample=ec::sentinel::bench_trace_sample(bench,now);
    observed.reset();
    // 保留既有台架发送策略, 不借波形功能更改停机/恢复契约.
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=3){
        (void)HAL_CAN_AbortTxRequest(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
        if(bench.active()){bench.stop(ec::sentinel::BenchReason::tx_failed);}
    }
    const float output=bench.active()?amperes:0;
    std::array<std::optional<ec::drivers::DjiOutput>,4> outputs{};
    outputs[bench.slot()]=ec::drivers::DjiOutput{output/ec::sentinel::Gm6020Bench::amp_full_scale,now};
    const auto result=group.send(now,outputs);
    tx_status=result.status;
    if(result.status!=ec::drivers::DjiGroupStatus::accepted){++tx_errors;bench.stop(ec::sentinel::BenchReason::tx_failed);}
    observed.annotate(sample,bench,tx_status);
    trace.push(sample,bench,now,tx_errors);
}
std::size_t ec_bench_snapshot(std::span<std::uint8_t> out) {
    return ec::sentinel::encode_bench_status(bench,ec::stm32f407::now(),
        static_cast<std::uint8_t>(tx_status),tx_errors,out,g_ec_s5_signature);
}
std::size_t ec_bench_trace_request(std::span<const std::uint8_t> in,std::span<std::uint8_t> out) {
    return trace.request(in,out,bench,ec::stm32f407::now(),static_cast<std::uint8_t>(tx_status),tx_errors,EC_FIRMWARE_ID);
}
#else
void ec_bench_receive(std::uint8_t,const ec::transport::CanFrame&,ec::Tick,ec::MicroTick) {}
void ec_bench_transport_lost() {}
bool ec_bench_request(const ec::transport::UartFrame&) {return false;}
std::size_t ec_bench_snapshot(std::span<std::uint8_t>) {return 0;}
std::size_t ec_bench_trace_request(std::span<const std::uint8_t>,std::span<std::uint8_t>) {return 0;}
extern "C" void ec_bench_tick() {}
#endif
