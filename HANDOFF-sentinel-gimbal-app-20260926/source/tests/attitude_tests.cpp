// M4 IMU 链契约: 四元数/旋转约定、模型数值对照、零角速、已知匀速旋转、静态倾斜、
// 安装轴旋转、零偏估计与采样间隔异常. 合成信号只证明公式与数值行为, 不代表实机标定.
#include "ec/estimation/attitude.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3); // 非零退出且不触发 CRT 交互弹窗, ctest 要求非交互
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::estimation::AttitudeConfig;
using ec::estimation::AttitudeEstimator;
using ec::estimation::AttitudeResult;
using ec::estimation::GyroBiasEstimator;
using ec::estimation::Quaternion;
using ec::estimation::rotation_zyx;

constexpr float g0 = 9.80665f;
constexpr float half_pi = static_cast<float>(std::numbers::pi / 2.0);
bool near(float a, float b, float tolerance) { return std::fabs(a - b) <= tolerance; }

// 由真值姿态产生静止时的比力: 参考系 +Z 在机体系中的方向 * g.
std::array<float, 3> gravity_measurement(const Quaternion& truth) {
    const auto direction = ec::estimation::rotate_reference_to_body(truth, {0.0f, 0.0f, 1.0f});
    return {direction[0] * g0, direction[1] * g0, direction[2] * g0};
}

Quaternion from_zyx(float roll, float pitch, float yaw) {
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f);
    const float sy = std::sin(yaw * 0.5f);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy};
}

// 测试自带的 Hamilton 乘法与"机体系角速度右乘"传播, 不复用被测实现, 避免自证.
Quaternion test_multiply(const Quaternion& a, const Quaternion& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

// q' = normalize(q + 0.5*dt*(q (x) omega)): 与声明约定 (q 为 body->reference,
// 陀螺仪给机体系角速度) 一致的一阶传播.
Quaternion body_rate_step(const Quaternion& q, const std::array<float, 3>& omega_body, float dt_s) {
    const Quaternion w{0.0f, omega_body[0], omega_body[1], omega_body[2]};
    const auto dq = test_multiply(q, w);
    const float half = 0.5f * dt_s;
    const Quaternion raw{q.w + half * dq.w, q.x + half * dq.x, q.y + half * dq.y, q.z + half * dq.z};
    const float norm = std::sqrt(raw.w * raw.w + raw.x * raw.x + raw.y * raw.y + raw.z * raw.z);
    return {raw.w / norm, raw.x / norm, raw.y / norm, raw.z / norm};
}

// R02: 机体系角速度必须以 q (x) omega 右乘传播, 且与 F 的雅可比保持一致.
// 初始 yaw 非零时, 左乘 omega (x) q 会给出反号的机体系分量, 单轴零姿态测试无法发现.
void body_rate_propagation_contracts() {
    const float dt = 0.01f;
    const std::array<float, 3> body_rate{1.0f, 0.0f, 0.0f};
    const auto q0 = from_zyx(0.0f, 0.0f, half_pi);
    AttitudeConfig config{};
    AttitudeEstimator est{config, ec::estimation::RotationMatrix3{}};
    est.reset(q0, {0.0f, 0.0f, 0.0f});
    // 零比力: 量测更新跳过, 只观察传播本身.
    CHECK(est.update(body_rate, {0.0f, 0.0f, 0.0f}, dt) == AttitudeResult::ok);
    const auto expected = body_rate_step(q0, body_rate, dt);
    const auto got = est.state().q;
    CHECK(near(got.w, expected.w, 1e-5f));
    CHECK(near(got.x, expected.x, 1e-5f));
    CHECK(near(got.y, expected.y, 1e-5f));
    CHECK(near(got.z, expected.z, 1e-5f));

    // 非交换组合: 先绕 body X 再绕 body Y 与相反顺序必须给出不同结果, 且都与解析一致.
    const std::array<float, 3> rate_x{0.5f, 0.0f, 0.0f};
    const std::array<float, 3> rate_y{0.0f, 0.5f, 0.0f};
    const auto start = from_zyx(0.2f, -0.1f, 1.0f);
    AttitudeEstimator xy{config, ec::estimation::RotationMatrix3{}};
    xy.reset(start, {0.0f, 0.0f, 0.0f});
    (void)xy.update(rate_x, {0.0f, 0.0f, 0.0f}, dt);
    (void)xy.update(rate_y, {0.0f, 0.0f, 0.0f}, dt);
    AttitudeEstimator yx{config, ec::estimation::RotationMatrix3{}};
    yx.reset(start, {0.0f, 0.0f, 0.0f});
    (void)yx.update(rate_y, {0.0f, 0.0f, 0.0f}, dt);
    (void)yx.update(rate_x, {0.0f, 0.0f, 0.0f}, dt);
    const auto expected_xy = body_rate_step(body_rate_step(start, rate_x, dt), rate_y, dt);
    const auto expected_yx = body_rate_step(body_rate_step(start, rate_y, dt), rate_x, dt);
    CHECK(near(xy.state().q.x, expected_xy.x, 2e-5f));
    CHECK(near(xy.state().q.y, expected_xy.y, 2e-5f));
    CHECK(near(xy.state().q.z, expected_xy.z, 2e-5f));
    CHECK(near(yx.state().q.x, expected_yx.x, 2e-5f));
    CHECK(near(yx.state().q.y, expected_yx.y, 2e-5f));
    CHECK(near(yx.state().q.z, expected_yx.z, 2e-5f));
    CHECK(std::fabs(xy.state().q.y - yx.state().q.y) > 1e-6f);
}

void quaternion_contracts() {
    // body -> reference: 绕 +Z 转 +90° 把机体 +X 送到参考 +Y.
    const auto yaw90 = from_zyx(0.0f, 0.0f, static_cast<float>(std::numbers::pi / 2.0));
    const auto moved = ec::estimation::rotate_body_to_reference(yaw90, {1.0f, 0.0f, 0.0f});
    CHECK(near(moved[0], 0.0f, 1e-6f) && near(moved[1], 1.0f, 1e-6f) && near(moved[2], 0.0f, 1e-6f));
    // 反向旋转把参考向量送回机体系.
    const auto back = ec::estimation::rotate_reference_to_body(yaw90, moved);
    CHECK(near(back[0], 1.0f, 1e-6f) && near(back[1], 0.0f, 1e-6f));
    // 单位四元数归一化与 -q 等价性.
    const auto scaled = ec::estimation::normalized(Quaternion{2.0f, 0.0f, 0.0f, 0.0f});
    CHECK(near(scaled.w, 1.0f, 1e-6f));
    const Quaternion negated{-yaw90.w, -yaw90.x, -yaw90.y, -yaw90.z};
    const auto moved_negated = ec::estimation::rotate_body_to_reference(negated, {1.0f, 0.0f, 0.0f});
    CHECK(near(moved_negated[0], moved[0], 1e-6f) && near(moved_negated[1], moved[1], 1e-6f));
    // 旋转矩阵与四元数一致 (Rz(90°) 的实现对照).
    const auto matrix = rotation_zyx(0.0f, 0.0f, static_cast<float>(std::numbers::pi / 2.0));
    const auto by_matrix = ec::estimation::apply_rotation(matrix, {1.0f, 0.0f, 0.0f});
    CHECK(near(by_matrix[0], moved[0], 1e-5f) && near(by_matrix[1], moved[1], 1e-5f));
    // ZYX 欧拉角反解: 旧实现的 Pitch/Roll 命名与本实现互换, 数值对应关系必须一致.
    const auto rpy = ec::estimation::roll_pitch_yaw_rad(from_zyx(0.3f, -0.4f, 1.1f));
    CHECK(near(rpy[0], 0.3f, 1e-5f) && near(rpy[1], -0.4f, 1e-5f) && near(rpy[2], 1.1f, 1e-5f));
    // 由静止比力求初始姿态: 绕重力轴的 yaw 无初值来源, 固定为 0.
    const auto initial = ec::estimation::quaternion_from_gravity({0.0f, 0.0f, g0});
    CHECK(near(initial.w, 1.0f, 1e-6f) && near(initial.z, 0.0f, 1e-6f));
    const float tilt = 0.5f;
    const auto tilted = ec::estimation::quaternion_from_gravity({std::sin(tilt) * g0, 0.0f,
        std::cos(tilt) * g0});
    const auto tilted_rpy = ec::estimation::roll_pitch_yaw_rad(tilted);
    CHECK(near(tilted_rpy[0], 0.0f, 1e-5f));
    // 参考 +Z 在机体系中偏向 +X: 对应 ZYX 下的负 pitch.
    CHECK(near(tilted_rpy[1], -tilt, 1e-5f));
    CHECK(near(tilted_rpy[2], 0.0f, 1e-5f));
    // 反向重力与零长度输入不产生 NaN.
    const auto flipped = ec::estimation::quaternion_from_gravity({0.0f, 0.0f, -g0});
    CHECK(std::isfinite(flipped.w) && near(flipped.w, 0.0f, 1e-6f));
    const auto empty = ec::estimation::quaternion_from_gravity({0.0f, 0.0f, 0.0f});
    CHECK(near(empty.w, 1.0f, 1e-6f));
}

void bias_estimator_contracts() {
    GyroBiasEstimator estimator{10};
    const std::array<float, 3> bias{0.01f, -0.02f, 0.03f};
    for (int i = 0; i < 9; ++i) { estimator.add(bias); }
    CHECK(!estimator.settled());
    CHECK(!estimator.estimate());
    estimator.add(bias);
    CHECK(estimator.settled() && estimator.samples() == 10);
    const auto mean = estimator.estimate();
    CHECK(mean && near((*mean)[0], 0.01f, 1e-6f) && near((*mean)[1], -0.02f, 1e-6f));
    // 非有限样本不计入窗口.
    estimator.add({std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f});
    CHECK(estimator.samples() == 10);
    estimator.reset();
    CHECK(!estimator.settled() && !estimator.estimate());
}

void zero_rate_contracts() {
    AttitudeEstimator estimator{};
    CHECK(near(estimator.covariance()[0], 100000.0f, 1e-3f));
    CHECK(near(estimator.covariance()[4 * 6 + 4], 100.0f, 1e-3f));
    CHECK(near(estimator.covariance()[1], 0.1f, 1e-6f));
    for (int i = 0; i < 2000; ++i) {
        CHECK(estimator.update({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, g0}, 0.001f) == AttitudeResult::ok);
    }
    const auto& state = estimator.state();
    const auto rpy = state.roll_pitch_yaw_rad;
    CHECK(near(rpy[0], 0.0f, 1e-4f) && near(rpy[1], 0.0f, 1e-4f) && near(rpy[2], 0.0f, 1e-4f));
    CHECK(state.converged && state.stable);
    // 归一化误差必须保持很小, 而不是"只输出有限数".
    const auto q = state.q;
    const float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    CHECK(near(norm, 1.0f, 1e-5f));
    // 零角速时零偏估计不应被推动: 零偏修正有单步限幅, 2000 步后仍应接近 0.
    CHECK(std::fabs(state.gyro_bias_rad_s[0]) < 1e-3f && std::fabs(state.gyro_bias_rad_s[1]) < 1e-3f);
    CHECK(state.gyro_bias_rad_s[2] == 0.0f); // Z 轴零偏不可观测, 恒为 0
    CHECK(state.updates == 2000);
    // 协方差随量测更新收缩, 且保持对称.
    CHECK(estimator.covariance()[0] < 100000.0f);
    for (std::size_t i = 0; i < 6; ++i) {
        for (std::size_t j = 0; j < 6; ++j) {
            CHECK(near(estimator.covariance()[i * 6 + j], estimator.covariance()[j * 6 + i], 1e-3f));
        }
    }
}

void static_tilt_contracts() {
    const float pitch = -0.35f;
    const float roll = 0.2f;
    const auto truth = from_zyx(roll, pitch, 0.0f);
    const auto accel = gravity_measurement(truth);
    // 初始姿态由重力方向给出, 随后静止保持.
    AttitudeEstimator estimator{};
    estimator.reset(ec::estimation::quaternion_from_gravity(accel));
    for (int i = 0; i < 3000; ++i) {
        CHECK(estimator.update({0.0f, 0.0f, 0.0f}, accel, 0.001f) == AttitudeResult::ok);
    }
    const auto rpy = estimator.state().roll_pitch_yaw_rad;
    CHECK(near(rpy[0], roll, 2e-3f));
    CHECK(near(rpy[1], pitch, 2e-3f));
    // yaw 只受复位时参考约束: 加速度计不观测 yaw, 量测修正还有小幅偏航渗漏,
    // 因此只要求它稳定在一个小范围, 而不是等于 0 或持续漂移.
    const float yaw_after_tilt = rpy[2];
    CHECK(std::fabs(yaw_after_tilt) < 0.05f);
    for (int i = 0; i < 10000; ++i) {
        CHECK(estimator.update({0.0f, 0.0f, 0.0f}, accel, 0.001f) == AttitudeResult::ok);
    }
    CHECK(near(estimator.state().roll_pitch_yaw_rad[2], yaw_after_tilt, 1e-4f));
}

void rotation_contracts() {
    // 绕机体 +Z 匀速旋转: 加速度不变, yaw 应随时间线性增长.
    const float rate = 0.5f;
    AttitudeEstimator estimator{};
    const float dt = 0.001f;
    const int steps = 2000;
    for (int i = 0; i < steps; ++i) {
        CHECK(estimator.update({0.0f, 0.0f, rate}, {0.0f, 0.0f, g0}, dt) == AttitudeResult::ok);
    }
    CHECK(near(estimator.state().roll_pitch_yaw_rad[2], rate * dt * static_cast<float>(steps), 0.02f));
    // roll/pitch 仍被加速度约束住.
    CHECK(near(estimator.state().roll_pitch_yaw_rad[0], 0.0f, 1e-3f));
    CHECK(near(estimator.state().roll_pitch_yaw_rad[1], 0.0f, 1e-3f));

    // 绕机体 +Y 匀速旋转: 用真值姿态产生的加速度作为量测, 估计应跟踪真值.
    AttitudeEstimator tracker{};
    Quaternion truth{};
    const float pitch_rate = 0.4f;
    for (int i = 0; i < 3000; ++i) {
        const auto accel = gravity_measurement(truth);
        CHECK(tracker.update({0.0f, pitch_rate, 0.0f}, accel, dt) == AttitudeResult::ok);
        truth = ec::estimation::normalized(ec::estimation::multiply(
            {std::cos(pitch_rate * dt * 0.5f), 0.0f, std::sin(pitch_rate * dt * 0.5f), 0.0f}, truth));
    }
    const auto tracked = tracker.state().roll_pitch_yaw_rad;
    const auto truth_rpy = ec::estimation::roll_pitch_yaw_rad(truth);
    CHECK(near(tracked[1], truth_rpy[1], 5e-3f));
    CHECK(near(tracked[0], truth_rpy[0], 5e-3f));
}

// 把机体量投到传感器系: sensor = R_install^T * body.
std::array<float, 3> to_sensor(const ec::estimation::RotationMatrix3& install,
    const std::array<float, 3>& body) {
    return {install.m[0] * body[0] + install.m[3] * body[1] + install.m[6] * body[2],
        install.m[1] * body[0] + install.m[4] * body[1] + install.m[7] * body[2],
        install.m[2] * body[0] + install.m[5] * body[1] + install.m[8] * body[2]};
}

void install_rotation_contracts() {
    // 传感器相对机体绕 Z 装了 90°: 机体绕 -X 旋转时, 传感器看到的是 +Y 角速度.
    const auto install = rotation_zyx(0.0f, 0.0f, half_pi);
    const std::array<float, 3> body_rate{-0.5f, 0.0f, 0.0f};
    const float dt = 0.001f;
    AttitudeEstimator estimator{AttitudeConfig{}, install};
    AttitudeEstimator unrotated{AttitudeConfig{}, ec::estimation::identity_rotation()};
    Quaternion truth{};
    for (int i = 0; i < 2000; ++i) {
        const auto body_accel = ec::estimation::rotate_reference_to_body(truth, {0.0f, 0.0f, g0});
        const auto sensor_gyro = to_sensor(install, body_rate);
        const auto sensor_accel = to_sensor(install, body_accel);
        CHECK(estimator.update(sensor_gyro, sensor_accel, dt) == AttitudeResult::ok);
        CHECK(unrotated.update(sensor_gyro, sensor_accel, dt) == AttitudeResult::ok);
        truth = ec::estimation::normalized(ec::estimation::multiply(
            {std::cos(-0.25f * dt), std::sin(-0.25f * dt), 0.0f, 0.0f}, truth));
    }
    const auto truth_rpy = ec::estimation::roll_pitch_yaw_rad(truth);
    const auto rpy = estimator.state().roll_pitch_yaw_rad;
    CHECK(truth_rpy[0] < -0.9f); // 2 s * 0.5 rad/s
    CHECK(near(rpy[0], truth_rpy[0], 5e-3f));
    CHECK(near(rpy[1], truth_rpy[1], 5e-3f));
    // 安装旋转确实生效: 不提供时同一组读数会被当成机体 +Y 角速度, 追不上真值.
    CHECK(std::fabs(unrotated.state().roll_pitch_yaw_rad[0] - truth_rpy[0]) > 0.3f);
    // 安装旋转对重力方向同样生效: 传感器 X 朝上的比力在机体上是 -Z 朝上.
    AttitudeEstimator tilted{AttitudeConfig{},
        rotation_zyx(-static_cast<float>(std::numbers::pi) / 2.0f, 0.0f, 0.0f)};
    for (int i = 0; i < 3000; ++i) {
        CHECK(tilted.update({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, g0}, 0.001f) == AttitudeResult::ok);
    }
    CHECK(near(tilted.state().roll_pitch_yaw_rad[0], half_pi, 3e-3f));
    CHECK(near(tilted.state().roll_pitch_yaw_rad[1], 0.0f, 3e-3f));
}

void bias_observation_contracts() {
    // 固定零偏 + 静止: 滤波器应把 x/y 零偏估到真值附近, z 零偏始终为 0.
    const std::array<float, 3> bias{0.02f, -0.015f, 0.05f};
    AttitudeEstimator estimator{};
    estimator.reset(Quaternion{});
    for (int i = 0; i < 20000; ++i) {
        CHECK(estimator.update(bias, {0.0f, 0.0f, g0}, 0.001f) == AttitudeResult::ok);
    }
    const auto estimated = estimator.state().gyro_bias_rad_s;
    CHECK(std::fabs(estimated[0] - bias[0]) < 5e-3f);
    CHECK(std::fabs(estimated[1] - bias[1]) < 5e-3f);
    CHECK(estimated[2] == 0.0f);
    // yaw 不可观测边界: 加速度计看不到绕重力轴的转动, 所以 0.05 rad/s 的 Z 零偏
    // 只能被积分成 yaw 漂移, 估计器不会把它识别为零偏.
    CHECK(std::fabs(estimator.state().roll_pitch_yaw_rad[2]) > 0.5f);
    // 显式零偏输入: 复位时给入由静止窗口估计的三轴零偏, 作为传感器系静态标定扣除.
    const auto window = [&] {
        GyroBiasEstimator estimator_window{3};
        for (int i = 0; i < 3; ++i) { estimator_window.add(bias); }
        return *estimator_window.estimate();
    }();
    AttitudeEstimator seeded{};
    seeded.reset(Quaternion{}, window);
    CHECK(seeded.update(bias, {0.0f, 0.0f, g0}, 0.001f) == AttitudeResult::ok);
    CHECK(near(seeded.state().gyro_rad_s[0], 0.0f, 1e-6f));
    CHECK(near(seeded.state().gyro_rad_s[1], 0.0f, 1e-6f));
    CHECK(near(seeded.state().gyro_rad_s[2], 0.0f, 1e-6f)); // 标定的 Z 也生效
    CHECK(seeded.state().gyro_bias_rad_s[2] == 0.0f); // 但 Z 残余仍不估计
    // 未去偏时角速度就是测量值.
    AttitudeEstimator plain{};
    CHECK(plain.update(bias, {0.0f, 0.0f, g0}, 0.001f) == AttitudeResult::ok);
    CHECK(near(plain.state().gyro_rad_s[0], bias[0], 1e-6f));
}

// R13: 静态标定零偏(传感器系三轴)与 EKF 在线残余零偏分开; 标定在安装旋转前只扣一次.
void gyro_calibration_contracts() {
    const std::array<float, 3> static_bias{0.01f, -0.02f, 0.05f};
    const auto static_accel = std::array<float, 3>{0.0f, 0.0f, g0};

    // 未标定: Z 零偏只能被积分成 yaw 漂移.
    AttitudeEstimator plain{};
    plain.reset(Quaternion{});
    for (int i = 0; i < 2000; ++i) { (void)plain.update(static_bias, static_accel, 0.001f); }
    CHECK(std::fabs(plain.state().roll_pitch_yaw_rad[2]) > 0.05f);
    CHECK(plain.state().gyro_bias_rad_s[2] == 0.0f); // Z 残余不估计

    // 注入静态标定后去偏角速度接近零, yaw 不再漂移.
    AttitudeEstimator calibrated{};
    calibrated.reset(Quaternion{}, static_bias);
    CHECK(calibrated.gyro_calibration_rad_s() == static_bias);
    for (int i = 0; i < 2000; ++i) { (void)calibrated.update(static_bias, static_accel, 0.001f); }
    CHECK(near(calibrated.state().gyro_rad_s[0], 0.0f, 1e-6f));
    CHECK(near(calibrated.state().gyro_rad_s[1], 0.0f, 1e-6f));
    CHECK(near(calibrated.state().gyro_rad_s[2], 0.0f, 1e-6f));
    CHECK(std::fabs(calibrated.state().roll_pitch_yaw_rad[2]) < 1e-3f);

    // 标定在传感器系: 安装旋转为绕 Z 90° 时, 传感器 X 上的零偏仍由传感器 X 标定扣除.
    const auto install = ec::estimation::rotation_zyx(0.0f, 0.0f, half_pi);
    const std::array<float, 3> sensor_x_bias{0.05f, 0.0f, 0.0f};
    AttitudeEstimator rotated{AttitudeConfig{}, install};
    rotated.reset(Quaternion{}, sensor_x_bias);
    CHECK(rotated.update(sensor_x_bias, static_accel, 0.001f) == AttitudeResult::ok);
    CHECK(near(rotated.state().gyro_rad_s[0], 0.0f, 1e-6f));
    CHECK(near(rotated.state().gyro_rad_s[1], 0.0f, 1e-6f));
    // 错误轴系的标定(扣机体系 X)无法消除该零偏.
    AttitudeEstimator wrong_axis{AttitudeConfig{}, install};
    wrong_axis.reset(Quaternion{}, std::array<float, 3>{0.0f, 0.05f, 0.0f});
    CHECK(wrong_axis.update(sensor_x_bias, static_accel, 0.001f) == AttitudeResult::ok);
    CHECK(std::fabs(wrong_axis.state().gyro_rad_s[1]) > 0.04f);

    // 标定是配置, 在线残余是状态: set_gyro_calibration 不改变姿态与残余估计.
    AttitudeEstimator mixed{};
    mixed.reset(Quaternion{});
    for (int i = 0; i < 200; ++i) { (void)mixed.update(static_bias, static_accel, 0.001f); }
    const auto residual_before = mixed.state().gyro_bias_rad_s;
    const auto yaw_before = mixed.state().roll_pitch_yaw_rad[2];
    mixed.set_gyro_calibration_rad_s(static_bias);
    CHECK(mixed.state().gyro_bias_rad_s == residual_before);
    CHECK(mixed.state().roll_pitch_yaw_rad[2] == yaw_before);
    // 下一次 update 起标定立即生效(Z 残余仍为 0, 但标定的 Z 已扣除).
    CHECK(mixed.update(static_bias, static_accel, 0.001f) == AttitudeResult::ok);
    CHECK(near(mixed.state().gyro_rad_s[2], 0.0f, 1e-6f));
}

// 4.3: 协方差自检 —— 对称、有限、对角非负; 纯预测下不确定度增长而非发散.
// 边界: "对称 + 对角非负"**不是**半正定证明, 也不证明一般姿态下的修正方向正确;
// 这里只排除 NaN/负对角/明显不对称这类实现错误. 滤波器发散与否需要独立反例或实测,
// 不能由本用例通过来宣布.
void covariance_sanity_contracts() {
    AttitudeEstimator estimator{};
    estimator.reset(Quaternion{});
    const auto check_covariance = [](const std::array<float, 36>& p) {
        for (std::size_t i = 0; i < 6; ++i) {
            CHECK(std::isfinite(p[i * 6 + i]) && p[i * 6 + i] >= 0.0f);
            for (std::size_t j = 0; j < 6; ++j) {
                CHECK(std::isfinite(p[i * 6 + j]));
                // 两个更新在解析上对称, 这里要求到舍入量级; 代码在更新后会做一次对称化.
                const float scale = std::fmax(1.0f, std::fmax(std::fabs(p[i * 6 + j]),
                    std::fabs(p[j * 6 + i])));
                CHECK(std::fabs(p[i * 6 + j] - p[j * 6 + i]) <= 1e-5f * scale);
            }
        }
    };
    check_covariance(estimator.covariance());
    const float q_variance_before = estimator.covariance()[0];
    for (int i = 0; i < 50; ++i) {
        // 零比力: 量测更新跳过, 只做预测.
        CHECK(estimator.update({0.2f, -0.1f, 0.05f}, {0.0f, 0.0f, 0.0f}, 0.01f) == AttitudeResult::ok);
        check_covariance(estimator.covariance());
    }
    CHECK(estimator.covariance()[0] >= q_variance_before);
    CHECK(estimator.covariance()[0] < 1e9f);
    // 加入量测后仍然自洽.
    for (int i = 0; i < 200; ++i) {
        (void)estimator.update({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, g0}, 0.01f);
        check_covariance(estimator.covariance());
    }
}

// 4.3: 加速度量测不修正 yaw —— 水平静止且 yaw 非零时姿态保持, 不向零收敛.
void yaw_not_corrected_contracts() {
    const float yaw = 1.1f;
    const auto truth = from_zyx(0.0f, 0.0f, yaw);
    AttitudeEstimator estimator{};
    estimator.reset(truth);
    for (int i = 0; i < 3000; ++i) {
        (void)estimator.update({0.0f, 0.0f, 0.0f}, gravity_measurement(truth), 0.001f);
    }
    const auto rpy = estimator.state().roll_pitch_yaw_rad;
    CHECK(near(rpy[0], 0.0f, 0.02f));
    CHECK(near(rpy[1], 0.0f, 0.02f));
    CHECK(std::fabs(rpy[2] - yaw) < 0.05f);
}

void input_guard_contracts() {
    AttitudeEstimator estimator{};
    const std::array<float, 3> ok_gyro{0.0f, 0.0f, 0.0f};
    const std::array<float, 3> ok_accel{0.0f, 0.0f, g0};
    CHECK(estimator.update(ok_gyro, ok_accel, 0.0f) == AttitudeResult::invalid_dt);
    CHECK(estimator.update(ok_gyro, ok_accel, -0.001f) == AttitudeResult::invalid_dt);
    CHECK(estimator.update(ok_gyro, ok_accel, std::numeric_limits<float>::quiet_NaN()) ==
        AttitudeResult::invalid_dt);
    CHECK(estimator.update({std::numeric_limits<float>::infinity(), 0.0f, 0.0f}, ok_accel, 0.001f) ==
        AttitudeResult::invalid_input);
    CHECK(estimator.update(ok_gyro, {0.0f, std::numeric_limits<float>::quiet_NaN(), g0}, 0.001f) ==
        AttitudeResult::invalid_input);
    // 被拒绝的输入不推进状态.
    CHECK(estimator.state().updates == 0);
    // 采样间隔异常但有限: 仍然接受, 输出保持有限且单位化; 这是精度下降而不是故障.
    CHECK(estimator.update(ok_gyro, ok_accel, 0.2f) == AttitudeResult::ok);
    CHECK(std::isfinite(estimator.state().q.w));
    const auto q = estimator.state().q;
    CHECK(near(std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z), 1.0f, 1e-5f));
    // 自由落体(比力接近 0)只做预测, 不用不可信方向修正姿态.
    AttitudeEstimator falling{};
    const auto before = falling.state().q;
    CHECK(falling.update({0.1f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.001f) == AttitudeResult::ok);
    CHECK(falling.state().measurement_skips == 1);
    CHECK(falling.state().updates == 0);
    CHECK(!near(falling.state().q.x, before.x, 1e-9f)); // 只有陀螺积分生效
}
} // 测试辅助

int main() {
    quaternion_contracts();
    body_rate_propagation_contracts();
    bias_estimator_contracts();
    zero_rate_contracts();
    static_tilt_contracts();
    rotation_contracts();
    install_rotation_contracts();
    bias_observation_contracts();
    gyro_calibration_contracts();
    covariance_sanity_contracts();
    yaw_not_corrected_contracts();
    input_guard_contracts();
    std::puts("attitude contracts passed");
    return 0;
}
