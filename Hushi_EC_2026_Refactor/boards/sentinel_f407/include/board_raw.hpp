#pragma once
#include "ec/drivers/bmi088_device.hpp"
#include "ec/transport/can.hpp"
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
#include "raw_record.hpp"
void ec_raw_imu(const ec::drivers::Bmi088Reading& reading) noexcept;
void ec_raw_can(std::uint8_t bus, const ec::transport::CanFrame& frame, ec::MicroTick at, std::uint32_t lost) noexcept;
void ec_raw_push(const ec::sentinel::recording::Record& record) noexcept;
bool ec_raw_fast_dm() noexcept;
int ec_raw_control_domain() noexcept;
std::size_t ec_raw_request(std::span<const std::uint8_t> in, std::span<std::uint8_t> out) noexcept;
#endif
