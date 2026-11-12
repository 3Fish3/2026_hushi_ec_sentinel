#pragma once
#include "ec/estimation/attitude.hpp"

namespace ec::estimation {
struct HeadingTiltConfig {
    // 研究候选的固定起点, 非实车调优值; 限制的是估计倾斜校正, 不是电机运动输出.
    float time_constant_s{0.5f};
    float max_tilt_correction_rad_s{0.5f};
    float norm_tolerance_mps2{0.8f};
    float min_projection_squared{0.01f};
    float max_predict_dt_s{0.02f};
    float max_accel_dt_s{0.02f};
};
enum class TiltResult : std::uint8_t {
    applied, no_change, invalid_input, invalid_dt, rejected_norm,
    rejected_projection, rejected_antipodal
};
struct HeadingTiltDiagnostics {
    float innovation_rad{}, tilt_step_rad{}, heading_injection_rad{};
    std::uint32_t corrections{}, skips{};
};

// Hamilton S->W姿态: 陀螺右乘传播, S²倾斜校正后绕世界Z规约到更新前的实际heading.
// 不维护协方差, 不从加速度残差学习gyro零偏; 并不保证任意加速度下的全轨迹航向精度.
// 时间、关节参考、历史和失效由调用方管理. 非法输入不改变姿态.
class HeadingTilt {
public:
    explicit HeadingTilt(HeadingTiltConfig config = {}, RotationMatrix3 install = {}) noexcept;
    [[nodiscard]] bool reset(Quaternion q, std::array<float,3> calibration_sensor = {}) noexcept;
    // 仅显式确认零角速的观测可设置三轴残余; 不推断静止, 不改参考, 下一份gyro起生效.
    [[nodiscard]] bool set_residual_bias_sensor(std::array<float,3> residual) noexcept;
    [[nodiscard]] TiltResult predict(std::array<float,3> gyro_sensor, float dt_s) noexcept;
    // accel已安装旋转并对齐到gyro状态时刻; anchor=K(t_gyro)^T forward, 必须来自实际关节历史.
    [[nodiscard]] TiltResult correct_accel_aligned(std::array<float,3> accel,
        std::array<float,3> anchor, float accel_dt_s) noexcept;
    [[nodiscard]] const Quaternion& orientation() const noexcept { return q_; }
    [[nodiscard]] const std::array<float,3>& gyro_rad_s() const noexcept { return rate_; }
    [[nodiscard]] const std::array<float,3>& calibration_sensor_rad_s() const noexcept { return calibration_; }
    [[nodiscard]] const std::array<float,3>& residual_bias_sensor_rad_s() const noexcept { return residual_; }
    [[nodiscard]] const RotationMatrix3& install_rotation() const noexcept { return install_; }
    [[nodiscard]] const HeadingTiltDiagnostics& diagnostics() const noexcept { return diagnostics_; }
private:
    HeadingTiltConfig config_{};
    RotationMatrix3 install_{};
    Quaternion q_{};
    std::array<float,3> calibration_{}, residual_{}, rate_{};
    HeadingTiltDiagnostics diagnostics_{};
    bool ready_{}, config_valid_{};
};
} // 命名空间 ec::estimation
