#pragma once
#include "ec/time.hpp"
#include <cstdint>
#include <span>

namespace ec::sentinel::board {
// SPI1 的有界 8 bit 全双工算法. IO 只封装寄存器读写与时钟, 便于原生模拟超时;
// 一个截止覆盖整个事务, 不为每个字节重新开始超时预算.
template<class Io>
[[nodiscard]] bool transfer_spi(Io& io, std::span<const std::uint8_t> tx,
    std::span<std::uint8_t> rx, std::uint32_t timeout_us) noexcept {
    if (tx.size() != rx.size()) { return false; }
    const auto start = io.now();
    const auto wait = [&](auto ready) {
        while (!ready()) {
            if (io.now().us - start.us >= timeout_us) { return false; }
        }
        return io.now().us - start.us < timeout_us;
    };
    for (std::size_t i = 0; i < tx.size(); ++i) {
        if (!wait([&] { return io.tx_ready(); })) { return false; }
        io.write(tx[i]);
        if (!wait([&] { return io.rx_ready(); })) { return false; }
        rx[i] = io.read();
    }
    return wait([&] { return !io.busy(); }) && !io.error();
}
} // 命名空间 ec::sentinel::board
