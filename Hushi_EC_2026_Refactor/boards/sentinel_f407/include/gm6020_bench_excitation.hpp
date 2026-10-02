#pragma once
#include <cmath>
#include <cstdint>

namespace ec::sentinel {
// 单次有限辨识激励, 用 MCU 微秒时基求值, 不依赖串口发包间隔.
// 幅值及偏置使用当前模式单位; 结束后保持偏置, 不增加停机/恢复策略.
struct BenchExcitation {
    enum class Kind : std::uint8_t { sine=1, square=2 };
    Kind kind{Kind::sine};
    float bias{}, amplitude{}, frequency_hz{};
    std::uint32_t duration_us{};
    [[nodiscard]] bool valid() const noexcept {
        return (kind==Kind::sine || kind==Kind::square) &&
            std::isfinite(bias) && std::isfinite(amplitude) && amplitude>0 &&
            std::isfinite(frequency_hz) && frequency_hz>0 && frequency_hz<500 &&
            duration_us>0 && duration_us<0x80000000u;
    }
    [[nodiscard]] float value(std::uint32_t elapsed_us) const noexcept {
        if (elapsed_us>=duration_us) { return bias; }
        // 双精度相位避免长窗口末端的 float 时间分辨率损失.
        const double phase=(static_cast<double>(elapsed_us)*frequency_hz)/1000000.0;
        const double cycle=phase-std::floor(phase);
        const float wave=kind==Kind::sine ? static_cast<float>(std::sin(6.2831853071795864769*cycle))
                                        : (cycle<0.5 ? 1.0f : -1.0f);
        return bias+amplitude*wave;
    }
};
} // 命名空间 ec::sentinel
