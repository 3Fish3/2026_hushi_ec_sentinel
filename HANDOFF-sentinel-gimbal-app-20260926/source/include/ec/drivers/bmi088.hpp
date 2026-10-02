#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace ec::drivers {
constexpr std::uint8_t bmi088_acc_chip_id = 0x1e;
constexpr std::uint8_t bmi088_gyro_chip_id = 0x0f;
constexpr std::uint8_t bmi088_acc_chip_id_reg = 0x00;
constexpr std::uint8_t bmi088_acc_data_reg = 0x12;
constexpr std::uint8_t bmi088_acc_temp_reg = 0x22;
constexpr std::uint8_t bmi088_acc_conf_reg = 0x40;
constexpr std::uint8_t bmi088_acc_range_reg = 0x41;
constexpr std::uint8_t bmi088_acc_pwr_conf_reg = 0x7c;
constexpr std::uint8_t bmi088_acc_pwr_ctrl_reg = 0x7d;
constexpr std::uint8_t bmi088_acc_softreset_reg = 0x7e;
constexpr std::uint8_t bmi088_gyro_chip_id_reg = 0x00;
constexpr std::uint8_t bmi088_gyro_data_reg = 0x02;
constexpr std::uint8_t bmi088_gyro_range_reg = 0x0f;
constexpr std::uint8_t bmi088_gyro_bandwidth_reg = 0x10;
constexpr std::uint8_t bmi088_gyro_lpm1_reg = 0x11;
constexpr std::uint8_t bmi088_gyro_softreset_reg = 0x14;
constexpr std::uint8_t bmi088_read_bit = 0x80;

// 与现有运行配置一致: 加速度 ±6 g, 陀螺 ±2000 °/s. 换算为 SI, 不含零偏标定.
constexpr float bmi088_acc_6g_mps2 = 0.00179443359375f; // 6*9.80665/32768
constexpr float bmi088_gyro_2000_rad_s = 0.0010652644360316953f; // 2000*pi/(180*32768)

struct Bmi088Sample {
    float acc_mps2[3]{};
    float gyro_rad_s[3]{};
    float temp_c{};
};

[[nodiscard]] inline std::int16_t le_i16(std::uint8_t lo, std::uint8_t hi) noexcept {
    const auto raw = static_cast<std::uint16_t>(lo | (static_cast<unsigned>(hi) << 8));
    return static_cast<std::int16_t>(raw);
}

[[nodiscard]] inline std::optional<std::array<float, 3>> parse_bmi088_accel_6g(
    std::span<const std::uint8_t, 6> raw) noexcept {
    std::array<float, 3> out{};
    for (int i = 0; i < 3; ++i) {
        out[static_cast<std::size_t>(i)] =
            static_cast<float>(le_i16(raw[static_cast<std::size_t>(2 * i)],
                                      raw[static_cast<std::size_t>(2 * i + 1)])) *
            bmi088_acc_6g_mps2;
    }
    return out;
}

[[nodiscard]] inline std::optional<std::array<float, 3>> parse_bmi088_gyro_2000(
    std::span<const std::uint8_t, 6> raw) noexcept {
    std::array<float, 3> out{};
    for (int i = 0; i < 3; ++i) {
        out[static_cast<std::size_t>(i)] =
            static_cast<float>(le_i16(raw[static_cast<std::size_t>(2 * i)],
                                      raw[static_cast<std::size_t>(2 * i + 1)])) *
            bmi088_gyro_2000_rad_s;
    }
    return out;
}

// 温度寄存器高字节在前. 11 位有符号, T(°C) = LSB * 0.125 + 23.
[[nodiscard]] inline float parse_bmi088_temp_c(std::uint8_t high, std::uint8_t low) noexcept {
    auto raw = static_cast<std::int16_t>(
        static_cast<std::uint16_t>((static_cast<unsigned>(high) << 8) | low));
    raw = static_cast<std::int16_t>(raw >> 5);
    return static_cast<float>(raw) * 0.125f + 23.0f;
}

// 加速度 SPI 读: 首字节为寄存器|0x80, 随后 1 字节虚读, 再跟有效数据.
[[nodiscard]] inline bool parse_bmi088_accel_spi_read(
    std::span<const std::uint8_t> rx, std::span<std::uint8_t> data) noexcept {
    if (rx.size() != data.size() + 2) { return false; }
    for (std::size_t i = 0; i < data.size(); ++i) { data[i] = rx[i + 2]; }
    return true;
}

[[nodiscard]] inline bool parse_bmi088_gyro_spi_read(
    std::span<const std::uint8_t> rx, std::span<std::uint8_t> data) noexcept {
    if (rx.size() != data.size() + 1) { return false; }
    for (std::size_t i = 0; i < data.size(); ++i) { data[i] = rx[i + 1]; }
    return true;
}

struct Bmi088InitStep {
    bool accel{};
    std::uint8_t reg{};
    std::uint8_t value{};
    std::uint16_t delay_ms{};
};

// 软复位与进入正常模式的有序步骤; 延时由调用方执行, 不在核心里忙等.
[[nodiscard]] constexpr std::array<Bmi088InitStep, 10> bmi088_init_steps() noexcept {
    return {{
        {true, bmi088_acc_softreset_reg, 0xb6, 50},
        {false, bmi088_gyro_softreset_reg, 0xb6, 50},
        {true, bmi088_acc_pwr_conf_reg, 0x00, 1},
        {true, bmi088_acc_pwr_ctrl_reg, 0x04, 50},
        {true, bmi088_acc_conf_reg, 0x80 | (0x2 << 4) | 0x0b, 1}, // normal, 800 Hz
        {true, bmi088_acc_range_reg, 0x01, 1}, // ±6 g
        {false, bmi088_gyro_range_reg, 0x00, 1}, // ±2000 °/s
        {false, bmi088_gyro_bandwidth_reg, 0x80 | 0x01, 1}, // 2000 Hz ODR / 230 Hz BW
        {false, bmi088_gyro_lpm1_reg, 0x00, 1},
        {true, bmi088_acc_pwr_ctrl_reg, 0x04, 0},
    }};
}
} // 命名空间 ec::drivers
