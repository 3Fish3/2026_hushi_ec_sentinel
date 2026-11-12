#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ec::estimation {
// 坐标与四元数约定 (全链路固定, 不随调用方变化):
// - 参考系 n 与机体系 b 都是右手系, 参考系 +Z 朝天.
// - 四元数按 Hamilton 约定, 顺序为 w,x,y,z, 表示 b -> n 旋转: 用 R(q) 左乘机体系
//   向量即得参考系向量. 单位四元数与 -q 表示同一姿态.
// - 加速度计测量比力: 静止且机体 +Z 朝天时为 (0,0,+g). 因此观测量是
//   "参考系 +Z 在机体系中的方向" = R(q)^T (0,0,1).
// - 欧拉角按 ZYX (yaw-pitch-roll) 从 R(q) 反解, 单位 rad, 范围 yaw/pitch/roll
//   分别为 (-pi,pi] / [-pi/2,pi/2] / (-pi,pi]. 旧仓库 QuaternionEKF 的 Pitch/Roll
//   字段实际对应本文件的 roll/pitch, 名称互换, 不复用旧命名.
// - yaw 是相对复位时参考方向的角度, 加速度计只观测 roll/pitch, 因此 yaw 会随未观测
//   的 Z 轴零偏漂移, 不能当绝对航向使用.

struct Quaternion {
    float w{1.0f};
    float x{};
    float y{};
    float z{};
};

[[nodiscard]] Quaternion normalized(const Quaternion& q) noexcept;
// Hamilton 积: 先施加 b, 再施加 a.
[[nodiscard]] Quaternion multiply(const Quaternion& a, const Quaternion& b) noexcept;
[[nodiscard]] std::array<float, 3> rotate_body_to_reference(
    const Quaternion& q, const std::array<float, 3>& body) noexcept;
[[nodiscard]] std::array<float, 3> rotate_reference_to_body(
    const Quaternion& q, const std::array<float, 3>& reference) noexcept;
[[nodiscard]] std::array<float, 3> roll_pitch_yaw_rad(const Quaternion& q) noexcept;

// 行主序 3x3 旋转矩阵, 用于显式表达 IMU 安装方向.
struct RotationMatrix3 {
    std::array<float, 9> m{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
};
[[nodiscard]] constexpr RotationMatrix3 identity_rotation() noexcept { return RotationMatrix3{}; }
// ZYX 欧拉角构成的旋转矩阵 R = Rz(yaw) Ry(pitch) Rx(roll).
[[nodiscard]] RotationMatrix3 rotation_zyx(float roll, float pitch, float yaw) noexcept;
[[nodiscard]] std::array<float, 3> apply_rotation(
    const RotationMatrix3& r, const std::array<float, 3>& v) noexcept;

// 由静止比力取对齐参考+Z的最短旋转. 一般姿态下不等于Euler yaw=0.
// 加速度计无法确定绕重力轴的转角; 此规约不是机械零点或绝对航向来源.
[[nodiscard]] Quaternion quaternion_from_gravity(const std::array<float, 3>& accel_mps2) noexcept;

// 静止窗口零偏估计: 是否静止、窗口何时开始与结束都由调用方决定, 本组件不做静止判定.
class GyroBiasEstimator {
public:
    explicit GyroBiasEstimator(std::size_t min_samples = 200) noexcept : min_samples_(min_samples) {}
    void add(const std::array<float, 3>& gyro_rad_s) noexcept;
    [[nodiscard]] bool settled() const noexcept { return samples_ >= min_samples_; }
    [[nodiscard]] std::size_t samples() const noexcept { return samples_; }
    [[nodiscard]] std::optional<std::array<float, 3>> estimate() const noexcept;
    void reset() noexcept;
private:
    std::size_t min_samples_{};
    std::size_t samples_{};
    std::array<double, 3> sum_{};
};

struct AttitudeConfig {
    // 默认取旧仓库 ins_task 的有效配置: Q1=10, Q2=0.001, R=1e6, 渐消因子 1,
    // 加速度低通时间常数 0 (旧代码另有 0.0085 的常量, 但未接入该滤波器).
    float process_noise_quaternion{10.0f};
    float process_noise_bias{0.001f};
    float measure_noise{1000000.0f};
    float fading{1.0f};
    float accel_lpf_s{0.0f};
    float chi_square_threshold{1e-8f};
};

struct AttitudeState {
    Quaternion q{};
    std::array<float, 3> gyro_bias_rad_s{};
    std::array<float, 3> gyro_rad_s{};  // 去偏后的机体系角速度
    std::array<float, 3> accel_mps2{};  // 低通后的机体系比力
    std::array<float, 3> roll_pitch_yaw_rad{};
    float chi_square{};
    bool converged{};
    bool stable{};
    std::uint64_t updates{};
    std::uint64_t predictions_only{};
    std::uint64_t measurement_skips{};
};

enum class AttitudeResult : std::uint8_t { ok, invalid_input, invalid_dt };

// 旧六状态加性EKF, 保留示例/离线对照, 不用于APP稳向. 状态为[q0,q1,q2,q3,bx,by].
// 固定机体Z残余为0并不表达任意姿态下的重力不可观测方向; 清q.z也不能隔离航向校正.
// APP使用HeadingTilt及实际关节锚点, 旧模型的偏航/协方差局限见姿态估计文档.
// 零偏分两处, 不得混为一谈:
//  - 静态标定零偏 (set_gyro_calibration_rad_s): 传感器系三轴, 在安装旋转之前只扣一次;
//  - 在线残余零偏: EKF 状态, 在机体系 X/Y 上估计, 与标定值叠加生效.
// 扣偏顺序: 原始测量 - 标定(传感器系) → 安装旋转 → - 在线残余(机体系).
// 单任务拥有, 无动态分配、无全局状态; 与旧实现共用同一数学模型与参数, 但未继承旧容器.
class AttitudeEstimator {
public:
    explicit AttitudeEstimator(const AttitudeConfig& config = {},
        const RotationMatrix3& install = identity_rotation()) noexcept;
    // 复位: 设置初始姿态与静态标定零偏(传感器系三轴), 在线残余零偏归零, 协方差回到初值.
    void reset(const Quaternion& q, const std::array<float, 3>& gyro_calibration_sensor_rad_s = {}) noexcept;
    // 只更新静态标定零偏, 不改变姿态与在线估计状态.
    void set_gyro_calibration_rad_s(const std::array<float, 3>& calibration_sensor_rad_s) noexcept {
        calibration_rad_s_ = calibration_sensor_rad_s;
    }
    [[nodiscard]] const std::array<float, 3>& gyro_calibration_rad_s() const noexcept {
        return calibration_rad_s_;
    }
    void set_install_rotation(const RotationMatrix3& install) noexcept { install_ = install; }
    [[nodiscard]] const RotationMatrix3& install_rotation() const noexcept { return install_; }
    [[nodiscard]] const AttitudeConfig& config() const noexcept { return config_; }
    // 输入为传感器系原始测量; 安装旋转在这里应用一次.
    [[nodiscard]] AttitudeResult update(const std::array<float, 3>& gyro_sensor_rad_s,
        const std::array<float, 3>& accel_sensor_mps2, float dt_s) noexcept;
    // 分离采样入口: gyro 只传播, accel 仅在有新观测时校正. dt 各属自己的采样时基.
    [[nodiscard]] AttitudeResult predict_gyro(const std::array<float, 3>& gyro_sensor_rad_s, float dt_s) noexcept;
    [[nodiscard]] AttitudeResult correct_accel(const std::array<float, 3>& accel_sensor_mps2,
        float dt_s, float max_gravity_error_mps2 = 1e6f) noexcept;
    [[nodiscard]] const AttitudeState& state() const noexcept { return state_; }
    // 协方差对角与对称性, 供数值对照与自检使用; 只读.
    [[nodiscard]] const std::array<float, 36>& covariance() const noexcept { return p_; }
private:
    void predict(const std::array<float, 3>& gyro, float dt_s) noexcept;
    void correct(const std::array<float, 3>& accel, float dt_s) noexcept;
    // 只消除浮点累加舍入造成的不对称; 两个更新在解析上已经对称.
    void symmetrize_covariance() noexcept;
    [[nodiscard]] std::array<float, 3> expected_gravity_direction() const noexcept;

    AttitudeConfig config_{};
    RotationMatrix3 install_{};
    AttitudeState state_{};
    std::array<float, 3> calibration_rad_s_{}; // 传感器系静态标定零偏
    std::array<float, 36> p_{};
    std::array<float, 3> accel_lpf_{};
    bool has_accel_{};
    std::uint32_t error_count_{};
};
} // 命名空间 ec::estimation
