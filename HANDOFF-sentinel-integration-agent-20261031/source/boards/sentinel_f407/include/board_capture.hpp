#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#ifndef EC_SENTINEL_CAPTURE
#define EC_SENTINEL_CAPTURE 0
#endif
#if EC_SENTINEL_CAPTURE
#include "ec/transport/can.hpp"
// 只读采集角色独立入口; 默认 S1/S5 不链接本实现.
void ec_capture_init() noexcept;
void ec_capture_start_task() noexcept;
void ec_capture_receive(std::uint8_t bus, const ec::transport::CanFrame& frame) noexcept;
std::size_t ec_capture_snapshot(std::uint8_t page, std::span<std::uint8_t> out) noexcept;
#endif
