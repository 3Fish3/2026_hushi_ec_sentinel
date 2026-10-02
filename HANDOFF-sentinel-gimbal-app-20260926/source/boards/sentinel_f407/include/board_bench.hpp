#pragma once
#include "ec/transport/uart_frame.hpp"
#include "ec/transport/can.hpp"
#include "ec/time.hpp"
// 仅 S5 显式启用时装配; 不在中断中调用这些入口.
void ec_bench_receive(std::uint8_t bus,const ec::transport::CanFrame& frame,ec::Tick at,ec::MicroTick at_us);
bool ec_bench_request(const ec::transport::UartFrame& frame);
std::size_t ec_bench_snapshot(std::span<std::uint8_t> output);
std::size_t ec_bench_trace_request(std::span<const std::uint8_t> input,std::span<std::uint8_t> output);
void ec_bench_transport_lost();
extern "C" void ec_bench_tick();
