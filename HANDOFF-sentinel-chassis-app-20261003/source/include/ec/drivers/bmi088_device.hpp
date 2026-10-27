#pragma once
#include "ec/drivers/bmi088.hpp"
#include "ec/time.hpp"
#include <cstdint>
#include <optional>
#include <span>

namespace ec::drivers {
// SPI 适配: 片选、时钟与虚读字节由适配实现负责(见 bmi088.hpp 的 parse_*_spi_read),
// 这里只表达寄存器语义. 同一个 SPI 外设只能由一个任务使用.
class Bmi088Bus {
public:
    [[nodiscard]] virtual bool read(bool accel, std::uint8_t reg, std::span<std::uint8_t> out) noexcept = 0;
    [[nodiscard]] virtual bool write(bool accel, std::uint8_t reg, std::uint8_t value) noexcept = 0;
    virtual void delay_ms(std::uint32_t ms) noexcept = 0;
    [[nodiscard]] virtual MicroTick micros() const noexcept = 0;
protected:
    ~Bmi088Bus() = default;
};

// 量程与更新率来自已验证的寄存器配置: 加速度 ±6 g, 800 Hz ODR; 陀螺 ±2000 °/s,
// 2000 Hz ODR / 230 Hz 带宽. 采集任务按自己的周期读取, 逐周期不等待新数据.
struct ImuSamplingTimes {
    MicroTick gyro{}, accel{}, temperature{}; // 各路采集完成时间
    MicroTick gyro_estimated{}; // FIFO 帧时间估计, 不是源端硬件时间戳
    std::uint32_t gyro_time_error_us{};
    std::uint32_t gyro_discarded{}, gyro_overruns{};
    std::uint32_t gyro_sequence{}, accel_sequence{}, temperature_sequence{};
    bool gyro_valid{}, accel_valid{}, temperature_valid{};
    bool gyro_updated{}, accel_updated{}, temperature_updated{};
};

struct Bmi088Reading {
    Bmi088Sample sample{};
    Tick sampled_at{};    // 本次 SPI 读取完成(采集)时刻
    Tick previous_at{};   // 上一次成功读取时刻
    std::uint32_t dt_ms{}; // 与上一次成功读取的间隔; 首次读取为 0
    ImuSamplingTimes times{}; // read_timed 的独立采集完成时刻, 不是硬件源端时刻
};

enum class Bmi088Result : std::uint8_t {
    ok, transport_error, chip_id_mismatch, not_initialized, time_regression, no_new_data,
};

class Bmi088Device {
public:
    Bmi088Device(Bmi088Bus& bus, Lifetime sample_ttl) noexcept : bus_(bus), ttl_(sample_ttl) {}
    Bmi088Device(const Bmi088Device&) = delete;
    Bmi088Device& operator=(const Bmi088Device&) = delete;
    // 按 bmi088_init_steps() 的次序配置并核对芯片 ID; 延时由总线实现的 delay_ms 执行.
    [[nodiscard]] Bmi088Result init(bool fifo_samples = false) noexcept;
    // 读一次加速度/角速度/温度; 任一传输失败都返回错误且不更新样本.
    [[nodiscard]] Bmi088Result read(Tick now) noexcept;
    // gyro 每次最多 4 个 FIFO 帧, accel 按 data-ready; 温度 1.28 s.
    // 单路失败不伪造其它路的新样本. gyro 输出最后一帧, 中间帧计入显式降采样计数.
    [[nodiscard]] Bmi088Result read_timed(Tick now) noexcept;
    [[nodiscard]] bool initialized() const noexcept { return initialized_; }
    [[nodiscard]] bool ready() const noexcept { return initialized_ && has_sample_; }
    [[nodiscard]] std::uint8_t accel_chip_id() const noexcept { return accel_id_; }
    [[nodiscard]] std::uint8_t gyro_chip_id() const noexcept { return gyro_id_; }
    [[nodiscard]] Bmi088Result last_result() const noexcept { return last_result_; }
    // 最近一次成功读取的原始结果, 不做时效判定; 调用方可用 reading(now) 取带时效的样本.
    [[nodiscard]] const Bmi088Reading& last_reading() const noexcept { return last_; }
    [[nodiscard]] std::uint32_t transport_errors() const noexcept { return transport_errors_; }
    [[nodiscard]] std::uint32_t reads() const noexcept { return reads_; }
    // 样本时效由调用方给的时间戳判定; 未初始化或读取失败时为无效.
    [[nodiscard]] std::optional<Bmi088Reading> reading(Tick now) const noexcept {
        if (!has_sample_ || !fresh(now, last_.sampled_at, ttl_)) { return std::nullopt; }
        return last_;
    }
private:
    Bmi088Bus& bus_;
    Lifetime ttl_{};
    Bmi088Reading last_{};
    Bmi088Result last_result_{Bmi088Result::not_initialized};
    std::uint8_t accel_id_{};
    std::uint8_t gyro_id_{};
    bool initialized_{};
    bool fifo_samples_{};
    bool has_sample_{};
    std::uint32_t transport_errors_{};
    std::uint32_t reads_{};
    MicroTick last_temperature_poll_{};
    bool have_temperature_poll_{};
};
} // 命名空间 ec::drivers
