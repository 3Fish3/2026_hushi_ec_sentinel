#include "board_raw.hpp"
#include "board_io.h"
#include "ec/channel.hpp"
#include <algorithm>
#include "ec/platform/stm32f407.hpp"
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
namespace {
ec::sentinel::recording::Recorder recorder;
ec::stm32f407::IrqCriticalSection lock;
}
void ec_raw_imu(const ec::drivers::Bmi088Reading& reading) noexcept {
    // 唯一快任务写入; 慢任务 request 使用临界区, 不可在 ISR 调用本接口.
    recorder.imu(reading, {ec_board_micros()});
}
void ec_raw_can(std::uint8_t bus, const ec::transport::CanFrame& frame, ec::MicroTick at, std::uint32_t lost) noexcept {
    recorder.can(bus, frame, at, {ec_board_micros()}, lost);
}
void ec_raw_push(const ec::sentinel::recording::Record& record) noexcept { recorder.push(record, {ec_board_micros()}); }
bool ec_raw_fast_dm() noexcept {
    recorder.advance({ec_board_micros()});
    return recorder.fast_dm();
}
int ec_raw_control_domain() noexcept {
    recorder.advance({ec_board_micros()});
    return recorder.control_domain();
}
std::size_t ec_raw_request(std::span<const std::uint8_t> in, std::span<std::uint8_t> out) noexcept {
    ec::ScopedCritical guard(lock);
#if EC_SENTINEL_CAPTURE
    if (in.size() == 12 && in[1] == 1 && in[2] == 3) {
        // 只读角色没有执行轨迹; 明确拒绝而不回一个只有 IMU 的伪控制窗口.
        std::array<std::uint8_t, 12> invalid{};
        std::copy(in.begin(), in.end(), invalid.begin());
        invalid[2] = 0;
        return recorder.request(invalid, out, {ec_board_micros()});
    }
#endif
    return recorder.request(in, out, {ec_board_micros()});
}
#endif
