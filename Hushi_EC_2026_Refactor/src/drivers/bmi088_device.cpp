#include "ec/drivers/bmi088_device.hpp"
#include <array>

namespace ec::drivers {
Bmi088Result Bmi088Device::init(bool fifo_samples) noexcept {
    initialized_ = false;
    fifo_samples_ = false;
    has_sample_ = false;
    have_temperature_poll_ = false;
    last_ = {};
    for (const auto& step : bmi088_init_steps()) {
        if (!bus_.write(step.accel, step.reg, step.value)) {
            ++transport_errors_;
            last_result_ = Bmi088Result::transport_error;
            return last_result_;
        }
        if (step.delay_ms != 0) { bus_.delay_ms(step.delay_ms); }
    }
    std::array<std::uint8_t, 1> accel_id{};
    std::array<std::uint8_t, 1> gyro_id{};
    if (!bus_.read(true, bmi088_acc_chip_id_reg, accel_id) ||
        !bus_.read(false, bmi088_gyro_chip_id_reg, gyro_id)) {
        ++transport_errors_;
        initialized_ = false;
        last_result_ = Bmi088Result::transport_error;
        return last_result_;
    }
    accel_id_ = accel_id[0];
    gyro_id_ = gyro_id[0];
    initialized_ = accel_id_ == bmi088_acc_chip_id && gyro_id_ == bmi088_gyro_chip_id;
    last_result_ = initialized_ ? Bmi088Result::ok : Bmi088Result::chip_id_mismatch;
    if (initialized_ && fifo_samples) {
        if (!bus_.write(false, 0x3e, 0x80)) {
            initialized_ = false;
            ++transport_errors_;
            return last_result_ = Bmi088Result::transport_error;
        }
        fifo_samples_ = true;
    }
    return last_result_;
}

Bmi088Result Bmi088Device::read(Tick now) noexcept {
    if (!initialized_) {
        last_result_ = Bmi088Result::not_initialized;
        return last_result_;
    }
    if (has_sample_ && tick_regressed(now, last_.sampled_at)) {
        last_result_ = Bmi088Result::time_regression;
        return last_result_;
    }
    std::array<std::uint8_t, 6> acc{};
    std::array<std::uint8_t, 6> gyro{};
    std::array<std::uint8_t, 2> temp{};
    if (!bus_.read(true, bmi088_acc_data_reg, acc) || !bus_.read(false, bmi088_gyro_data_reg, gyro) ||
        !bus_.read(true, bmi088_acc_temp_reg, temp)) {
        ++transport_errors_;
        last_result_ = Bmi088Result::transport_error;
        return last_result_;
    }
    const auto acc_si = parse_bmi088_accel_6g(acc);
    const auto gyro_si = parse_bmi088_gyro_2000(gyro);
    if (!acc_si || !gyro_si) {
        ++transport_errors_;
        last_result_ = Bmi088Result::transport_error;
        return last_result_;
    }
    Bmi088Reading next{};
    for (std::size_t i = 0; i < 3; ++i) {
        next.sample.acc_mps2[i] = (*acc_si)[i];
        next.sample.gyro_rad_s[i] = (*gyro_si)[i];
    }
    next.sample.temp_c = parse_bmi088_temp_c(temp[0], temp[1]);
    next.sampled_at = now;
    next.previous_at = has_sample_ ? last_.sampled_at : now;
    next.dt_ms = has_sample_ ? now.ms - last_.sampled_at.ms : 0;
    last_ = next;
    has_sample_ = true;
    ++reads_;
    last_result_ = Bmi088Result::ok;
    return last_result_;
}
Bmi088Result Bmi088Device::read_timed(Tick now) noexcept {
    auto& t = last_.times;
    t.gyro_updated = t.accel_updated = t.temperature_updated = false;
    if (!initialized_ || !fifo_samples_) { return last_result_ = Bmi088Result::not_initialized; }
    if (has_sample_ && tick_regressed(now, last_.sampled_at)) {
        return last_result_ = Bmi088Result::time_regression;
    }
    bool failed = false;
    const auto vector = [&](bool accel) {
        std::array<std::uint8_t, 1> status{};
        const auto before_status = bus_.micros();
        // gyro DRDY 会自动清除, 用 FIFO_STATUS 判新; acc 状态读出数据后才清除.
        if (!bus_.read(accel, accel ? 0x03 : 0x0e, status)) { failed = true; return; }
        const auto observed = bus_.micros();
        unsigned count = accel ? ((status[0] & 0x80) ? 1u : 0u) : (status[0] & 0x7fu);
        if (!accel && ((status[0] & 0x80) || count > 4)) {
            // 丢弃超预算的积压而非无限追赶; 重新等待新帧, 不补造姿态样本.
            t.gyro_discarded += count;
            if (status[0] & 0x80) { ++t.gyro_overruns; }
            if (!bus_.write(false, 0x3e, 0x80)) { failed = true; }
            return;
        }
        if (count == 0) { return; }
        std::array<std::uint8_t, 6> raw{};
        // 最多 4 帧且每次完整读取 6 B, 末帧用于 1 kHz 控制, 前帧显式计数.
        for (unsigned i = 0; i < count; ++i) {
            if (!bus_.read(accel, accel ? bmi088_acc_data_reg : 0x3f, raw)) {
                failed = true;
                if (!accel) { t.gyro_discarded += i + 1; }
                return;
            }
            if (!accel) { ++t.gyro_sequence; }
        }
        if (!accel) {
            t.gyro_discarded += count - 1;
            t.gyro_estimated = observed;
            // 当前 ODR=2 kHz (±0.3%): 一周期相位未知 + 帧间漂移 + 状态读取时长.
            t.gyro_time_error_us = 502u + 2u * count + observed.us - before_status.us;
        }
        const auto at = bus_.micros();
        const auto si = accel ? parse_bmi088_accel_6g(raw) : parse_bmi088_gyro_2000(raw);
        if (!si) { failed = true; return; }
        for (std::size_t i = 0; i < 3; ++i) {
            if (accel) { last_.sample.acc_mps2[i] = (*si)[i]; }
            else { last_.sample.gyro_rad_s[i] = (*si)[i]; }
        }
        if (accel) {
            t.accel = at; ++t.accel_sequence; t.accel_valid = t.accel_updated = true;
        } else {
            t.gyro = at; t.gyro_valid = t.gyro_updated = true;
            last_.previous_at = has_sample_ ? last_.sampled_at : now;
            last_.dt_ms = has_sample_ ? now.ms - last_.sampled_at.ms : 0;
            last_.sampled_at = now;
            ++reads_;
        }
    };
    vector(false);
    vector(true);
    const auto at = bus_.micros();
    if (!have_temperature_poll_ || at.us - last_temperature_poll_.us >= 1280000u) {
        last_temperature_poll_ = at;
        have_temperature_poll_ = true;
        std::array<std::uint8_t, 2> raw{};
        if (bus_.read(true, bmi088_acc_temp_reg, raw)) {
            if (raw[0] == 0x80 && raw[1] == 0) {
                t.temperature_valid = false; // 手册定义的未就绪码, 不报告为 -105 °C
            } else {
                last_.sample.temp_c = parse_bmi088_temp_c(raw[0], raw[1]);
                t.temperature = bus_.micros(); ++t.temperature_sequence;
                t.temperature_valid = t.temperature_updated = true;
            }
        } else { failed = true; }
    }
    has_sample_ = t.gyro_valid && t.accel_valid;
    if (failed) { ++transport_errors_; return last_result_ = Bmi088Result::transport_error; }
    return last_result_ = t.gyro_updated || t.accel_updated || t.temperature_updated
        ? Bmi088Result::ok : Bmi088Result::no_new_data;
}
} // 命名空间 ec::drivers
