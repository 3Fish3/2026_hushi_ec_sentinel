#include "ec/estimation/attitude.hpp"
#include <cmath>

namespace ec::estimation {
namespace {
constexpr float pi = 3.14159265358979323846f;
constexpr std::size_t n = 6; // 状态维数: 四元数 4 + 零偏 2

float dot3(const std::array<float, 3>& a, const std::array<float, 3>& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
float norm3(const std::array<float, 3>& v) noexcept { return std::sqrt(dot3(v, v)); }
bool finite3(const std::array<float, 3>& v) noexcept {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// 3x3 逆: 伴随矩阵除以行列式; 行列式过小视为退化.
bool invert3(const std::array<float, 9>& a, std::array<float, 9>& out) noexcept {
    const float c0 = a[4] * a[8] - a[5] * a[7];
    const float c1 = a[5] * a[6] - a[3] * a[8];
    const float c2 = a[3] * a[7] - a[4] * a[6];
    const float det = a[0] * c0 + a[1] * c1 + a[2] * c2;
    if (!std::isfinite(det) || std::fabs(det) < 1e-30f) { return false; }
    const float inv = 1.0f / det;
    out[0] = c0 * inv;
    out[1] = (a[2] * a[7] - a[1] * a[8]) * inv;
    out[2] = (a[1] * a[5] - a[2] * a[4]) * inv;
    out[3] = c1 * inv;
    out[4] = (a[0] * a[8] - a[2] * a[6]) * inv;
    out[5] = (a[2] * a[3] - a[0] * a[5]) * inv;
    out[6] = c2 * inv;
    out[7] = (a[1] * a[6] - a[0] * a[7]) * inv;
    out[8] = (a[0] * a[4] - a[1] * a[3]) * inv;
    return true;
}
} // 内部辅助函数

Quaternion normalized(const Quaternion& q) noexcept {
    const float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!std::isfinite(norm) || norm <= 0.0f) { return Quaternion{}; }
    const float inv = 1.0f / norm;
    return {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

Quaternion multiply(const Quaternion& a, const Quaternion& b) noexcept {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

std::array<float, 3> rotate_body_to_reference(const Quaternion& q, const std::array<float, 3>& v) noexcept {
    const Quaternion p{0.0f, v[0], v[1], v[2]};
    const Quaternion conj{q.w, -q.x, -q.y, -q.z};
    const auto r = multiply(multiply(q, p), conj);
    return {r.x, r.y, r.z};
}

std::array<float, 3> rotate_reference_to_body(const Quaternion& q, const std::array<float, 3>& v) noexcept {
    const Quaternion p{0.0f, v[0], v[1], v[2]};
    const Quaternion conj{q.w, -q.x, -q.y, -q.z};
    const auto r = multiply(multiply(conj, p), q);
    return {r.x, r.y, r.z};
}

std::array<float, 3> roll_pitch_yaw_rad(const Quaternion& q) noexcept {
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;
    const float roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    const float sin_pitch = 2.0f * (w * y - z * x);
    const float pitch = std::asin(std::fmax(-1.0f, std::fmin(1.0f, sin_pitch)));
    const float yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    return {roll, pitch, yaw};
}

RotationMatrix3 rotation_zyx(float roll, float pitch, float yaw) noexcept {
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);
    return RotationMatrix3{{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp,
        sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr}}};
}

std::array<float, 3> apply_rotation(const RotationMatrix3& r, const std::array<float, 3>& v) noexcept {
    return {r.m[0] * v[0] + r.m[1] * v[1] + r.m[2] * v[2],
        r.m[3] * v[0] + r.m[4] * v[1] + r.m[5] * v[2],
        r.m[6] * v[0] + r.m[7] * v[1] + r.m[8] * v[2]};
}

Quaternion quaternion_from_gravity(const std::array<float, 3>& accel_mps2) noexcept {
    if (!finite3(accel_mps2)) { return Quaternion{}; }
    const float len = norm3(accel_mps2);
    if (len <= 0.0f) { return Quaternion{}; }
    // 测量方向对齐参考 +Z 的最小旋转: 轴 = 测量 × 参考, 角 = 夹角.
    const std::array<float, 3> measured{accel_mps2[0] / len, accel_mps2[1] / len, accel_mps2[2] / len};
    const std::array<float, 3> reference{0.0f, 0.0f, 1.0f};
    const float cosine = std::fmax(-1.0f, std::fmin(1.0f, dot3(measured, reference)));
    const float angle = std::acos(cosine);
    const std::array<float, 3> axis{measured[1] * reference[2] - measured[2] * reference[1],
        measured[2] * reference[0] - measured[0] * reference[2],
        measured[0] * reference[1] - measured[1] * reference[0]};
    const float axis_norm = norm3(axis);
    if (axis_norm <= 1e-6f) {
        // 已对齐或完全反向: 反向时绕机体 X 轴转 pi, yaw 仍取 0.
        return cosine >= 0.0f ? Quaternion{} : Quaternion{0.0f, 1.0f, 0.0f, 0.0f};
    }
    const float inv = std::sin(angle * 0.5f) / axis_norm;
    return normalized(Quaternion{std::cos(angle * 0.5f), axis[0] * inv, axis[1] * inv, axis[2] * inv});
}

void GyroBiasEstimator::add(const std::array<float, 3>& gyro_rad_s) noexcept {
    if (!finite3(gyro_rad_s)) { return; }
    for (std::size_t i = 0; i < 3; ++i) { sum_[i] += static_cast<double>(gyro_rad_s[i]); }
    ++samples_;
}

std::optional<std::array<float, 3>> GyroBiasEstimator::estimate() const noexcept {
    if (!settled()) { return std::nullopt; }
    const double inv = 1.0 / static_cast<double>(samples_);
    return std::array<float, 3>{static_cast<float>(sum_[0] * inv), static_cast<float>(sum_[1] * inv),
        static_cast<float>(sum_[2] * inv)};
}

void GyroBiasEstimator::reset() noexcept {
    samples_ = 0;
    sum_ = {};
}

AttitudeEstimator::AttitudeEstimator(const AttitudeConfig& config, const RotationMatrix3& install) noexcept
    : config_(config), install_(install) {
    reset(Quaternion{});
}

void AttitudeEstimator::reset(const Quaternion& q, const std::array<float, 3>& gyro_calibration_sensor_rad_s) noexcept {
    state_ = {};
    state_.q = normalized(q);
    // 静态标定是配置, 在线残余零偏从零开始; Z 轴残余不估计, 但标定的 Z 仍会扣除.
    calibration_rad_s_ = gyro_calibration_sensor_rad_s;
    state_.gyro_bias_rad_s = {};
    state_.roll_pitch_yaw_rad = roll_pitch_yaw_rad(state_.q);
    has_accel_ = false;
    error_count_ = 0;
    // 与旧实现一致的初始协方差: 四元数 1e5, 零偏 100, 交叉项 0.1.
    p_.fill(0.1f);
    for (std::size_t i = 0; i < 4; ++i) { p_[i * n + i] = 100000.0f; }
    p_[4 * n + 4] = 100.0f;
    p_[5 * n + 5] = 100.0f;
}

void AttitudeEstimator::symmetrize_covariance() noexcept {
    // Joseph 与 F P F^T 在解析上都是对称的; 这里只消除 float 累加舍入带来的不对称
    // (P 量级可达 1e5, 相对舍入约 1e-7/项, 累积后可达 1e-4 相对量级), 使下游使用同一矩阵.
    for (std::size_t i = 0; i < 6; ++i) {
        for (std::size_t j = i + 1; j < 6; ++j) {
            const float mean = 0.5f * (p_[i * 6 + j] + p_[j * 6 + i]);
            p_[i * 6 + j] = mean;
            p_[j * 6 + i] = mean;
        }
    }
}

std::array<float, 3> AttitudeEstimator::expected_gravity_direction() const noexcept {
    // 参考系 +Z 在机体系中的方向, 即量测函数 h(x).
    const float w = state_.q.w;
    const float x = state_.q.x;
    const float y = state_.q.y;
    const float z = state_.q.z;
    return {2.0f * (x * z - w * y), 2.0f * (w * x + y * z), w * w - x * x - y * y + z * z};
}

void AttitudeEstimator::predict(const std::array<float, 3>& gyro, float dt_s) noexcept {
    auto& q = state_.q;
    // 状态转移 F = I + dt * [0.5*Omega(w)  dq/dbias; 0 0]
    // 其中四元数块表达 q ⊗ (0,w) 的右乘导数, dq/dbias 对应 b = (bx, by, 0).
    std::array<float, 36> f{};
    for (std::size_t i = 0; i < n; ++i) { f[i * n + i] = 1.0f; }
    const float hx = 0.5f * gyro[0] * dt_s;
    const float hy = 0.5f * gyro[1] * dt_s;
    const float hz = 0.5f * gyro[2] * dt_s;
    f[1] = -hx; f[2] = -hy; f[3] = -hz;
    f[6] = hx;  f[8] = hz;  f[9] = -hy;
    f[12] = hy; f[13] = -hz; f[15] = hx;
    f[18] = hz; f[19] = hy; f[20] = -hx;
    const float half = 0.5f * dt_s;
    f[4] = half * q.x;  f[5] = half * q.y;
    f[10] = -half * q.w; f[11] = half * q.z;
    f[16] = -half * q.z; f[17] = -half * q.w;
    f[22] = half * q.y;  f[23] = -half * q.x;

    // 渐消因子只作用于零偏方差, 与旧实现一样在传播前施加并限幅.
    if (config_.fading > 0.0f && config_.fading < 1.0f) {
        p_[4 * n + 4] = std::fmin(p_[4 * n + 4] / config_.fading, 10000.0f);
        p_[5 * n + 5] = std::fmin(p_[5 * n + 5] / config_.fading, 10000.0f);
    }
    // P' = F P F^T + Q
    std::array<float, 36> fp{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            float sum = 0.0f;
            for (std::size_t k = 0; k < n; ++k) { sum += f[i * n + k] * p_[k * n + j]; }
            fp[i * n + j] = sum;
        }
    }
    std::array<float, 36> next{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            float sum = 0.0f;
            for (std::size_t k = 0; k < n; ++k) { sum += fp[i * n + k] * f[j * n + k]; }
            next[i * n + j] = sum;
        }
    }
    const float q_noise = config_.process_noise_quaternion * dt_s;
    const float bias_noise = config_.process_noise_bias * dt_s;
    for (std::size_t i = 0; i < 4; ++i) { next[i * n + i] += q_noise; }
    next[4 * n + 4] += bias_noise;
    next[5 * n + 5] += bias_noise;
    p_ = next;
    symmetrize_covariance();

    // 四元数传播: 机体系角速度对应 q' = q + 0.5 * (q (x) w) * dt, 再归一化.
    // 乘法顺序必须与 F 的四元数块一致(右乘): 左乘会在非零初始姿态下给出反号的机体系分量.
    const Quaternion w{0.0f, gyro[0], gyro[1], gyro[2]};
    const auto dq = multiply(q, w);
    const float half_dt = 0.5f * dt_s;
    q = normalized(Quaternion{q.w + half_dt * dq.w, q.x + half_dt * dq.x, q.y + half_dt * dq.y,
        q.z + half_dt * dq.z});
}

void AttitudeEstimator::correct(const std::array<float, 3>& accel, float dt_s) noexcept {
    const float len = norm3(accel);
    if (len <= 1e-3f) {
        // 自由落体或无效量测: 没有可信的重力方向, 只保留预测.
        ++state_.measurement_skips;
        return;
    }
    const std::array<float, 3> z{accel[0] / len, accel[1] / len, accel[2] / len};
    const auto h = expected_gravity_direction();

    // 观测量与各轴方向余弦: 方向余弦用于缩放零偏增益.
    std::array<float, 3> cosine{};
    for (std::size_t i = 0; i < 3; ++i) {
        cosine[i] = std::acos(std::fmin(1.0f, std::fabs(h[i])));
    }
    const std::array<float, 3> residual{z[0] - h[0], z[1] - h[1], z[2] - h[2]};

    const float w = state_.q.w;
    const float x = state_.q.x;
    const float y = state_.q.y;
    const float zz = state_.q.z;
    std::array<float, 18> hmat{}; // 3x6, 后两列为零
    hmat[0] = -2.0f * y;  hmat[1] = 2.0f * zz; hmat[2] = -2.0f * w; hmat[3] = 2.0f * x;
    hmat[6] = 2.0f * x;   hmat[7] = 2.0f * w;  hmat[8] = 2.0f * zz; hmat[9] = 2.0f * y;
    hmat[12] = 2.0f * w;  hmat[13] = -2.0f * x; hmat[14] = -2.0f * y; hmat[15] = 2.0f * zz;

    // S = H P H^T + R
    std::array<float, 18> ph{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            float sum = 0.0f;
            for (std::size_t k = 0; k < n; ++k) { sum += p_[i * n + k] * hmat[j * n + k]; }
            ph[i * 3 + j] = sum;
        }
    }
    std::array<float, 9> s{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            float sum = 0.0f;
            for (std::size_t k = 0; k < n; ++k) { sum += hmat[i * n + k] * ph[k * 3 + j]; }
            s[i * 3 + j] = sum;
        }
    }
    s[0] += config_.measure_noise;
    s[4] += config_.measure_noise;
    s[8] += config_.measure_noise;
    std::array<float, 9> s_inv{};
    if (!invert3(s, s_inv)) {
        ++state_.measurement_skips;
        return;
    }

    // 卡方检验: r^T S^-1 r
    std::array<float, 3> sr{};
    for (std::size_t i = 0; i < 3; ++i) {
        sr[i] = s_inv[i * 3] * residual[0] + s_inv[i * 3 + 1] * residual[1] + s_inv[i * 3 + 2] * residual[2];
    }
    state_.chi_square = residual[0] * sr[0] + residual[1] * sr[1] + residual[2] * sr[2];
    const float threshold = config_.chi_square_threshold;
    if (state_.chi_square < 0.5f * threshold) { state_.converged = true; }

    float gain_scale = 1.0f;
    if (state_.chi_square > threshold && state_.converged) {
        if (state_.stable) { ++error_count_; } else { error_count_ = 0; }
        if (error_count_ <= 50) {
            ++state_.predictions_only;
            return; // 残差未通过检验: 只保留预测
        }
        // 连续被拒并超过上限: 认为滤波器需要重新收敛, 正常做一次量测更新.
        state_.converged = false;
    } else {
        if (state_.chi_square > 0.1f * threshold && state_.converged) {
            gain_scale = (threshold - state_.chi_square) / (0.9f * threshold);
        }
        error_count_ = 0;
    }

    // K = P H^T S^-1, 再乘自适应增益
    std::array<float, 18> k{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            float sum = 0.0f;
            for (std::size_t m = 0; m < 3; ++m) { sum += ph[i * 3 + m] * s_inv[m * 3 + j]; }
            k[i * 3 + j] = sum * gain_scale;
        }
    }
    // 零偏增益按方向余弦缩放: 该轴方向信噪比低时减少零偏修正.
    for (std::size_t i = 4; i < 6; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            k[i * 3 + j] *= cosine[i - 4] / (pi * 0.5f);
        }
    }

    std::array<float, 6> dx{};
    for (std::size_t i = 0; i < n; ++i) {
        dx[i] = k[i * 3] * residual[0] + k[i * 3 + 1] * residual[1] + k[i * 3 + 2] * residual[2];
    }
    // 零偏修正限幅: 收敛后单步最多 0.01*dt, 避免量测突变直接冲进零偏.
    if (state_.converged) {
        const float limit = 0.01f * dt_s;
        for (std::size_t i = 4; i < 6; ++i) {
            dx[i] = std::fmax(-limit, std::fmin(limit, dx[i]));
        }
    }
    dx[3] = 0.0f; // 不修正 yaw 分量

    state_.q = normalized(Quaternion{state_.q.w + dx[0], state_.q.x + dx[1], state_.q.y + dx[2],
        state_.q.z + dx[3]});
    // 零偏状态同样由量测更新推动; Z 轴零偏不估计, 恒为 0.
    state_.gyro_bias_rad_s[0] += dx[4];
    state_.gyro_bias_rad_s[1] += dx[5];
    state_.gyro_bias_rad_s[2] = 0.0f;

    // P = (I - K H) P (I - K H)^T + K R K^T (Joseph 形式).
    // 本滤波器的 K 经自适应缩放与零偏行缩放改写, 不再是“最优增益”, 因此 (I-KH)P 既不保证
    // 对称也不保证半正定; Joseph 形式对任意 K 都保持对称与非负定(R 为各向同性).
    std::array<float, 36> kh{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            float sum = 0.0f;
            for (std::size_t m = 0; m < 3; ++m) { sum += k[i * 3 + m] * hmat[m * n + j]; }
            kh[i * n + j] = sum;
        }
    }
    std::array<float, 36> i_kh{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            i_kh[i * n + j] = (i == j ? 1.0f : 0.0f) - kh[i * n + j];
        }
    }
    std::array<float, 36> a{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            float sum = 0.0f;
            for (std::size_t m = 0; m < n; ++m) { sum += i_kh[i * n + m] * p_[m * n + j]; }
            a[i * n + j] = sum;
        }
    }
    std::array<float, 36> next{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            float sum = 0.0f;
            for (std::size_t m = 0; m < n; ++m) { sum += a[i * n + m] * i_kh[j * n + m]; }
            float krk = 0.0f;
            for (std::size_t r = 0; r < 3; ++r) { krk += k[i * 3 + r] * k[j * 3 + r]; }
            next[i * n + j] = sum + config_.measure_noise * krk;
        }
    }
    p_ = next;
    symmetrize_covariance();
    ++state_.updates;
}

AttitudeResult AttitudeEstimator::update(const std::array<float, 3>& gyro_sensor_rad_s,
    const std::array<float, 3>& accel_sensor_mps2, float dt_s) noexcept {
    if (!finite3(gyro_sensor_rad_s) || !finite3(accel_sensor_mps2)) { return AttitudeResult::invalid_input; }
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) { return AttitudeResult::invalid_dt; }
    const auto result = predict_gyro(gyro_sensor_rad_s, dt_s);
    return result == AttitudeResult::ok ? correct_accel(accel_sensor_mps2, dt_s) : result;
}

AttitudeResult AttitudeEstimator::predict_gyro(const std::array<float, 3>& gyro_sensor_rad_s, float dt_s) noexcept {
    if (!finite3(gyro_sensor_rad_s)) { return AttitudeResult::invalid_input; }
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) { return AttitudeResult::invalid_dt; }
    // 先扣传感器系静态标定(只扣一次), 再安装旋转到机体系, 最后扣 EKF 在线残余零偏.
    const std::array<float, 3> gyro_calibrated{gyro_sensor_rad_s[0] - calibration_rad_s_[0],
        gyro_sensor_rad_s[1] - calibration_rad_s_[1], gyro_sensor_rad_s[2] - calibration_rad_s_[2]};
    const auto gyro_body = apply_rotation(install_, gyro_calibrated);
    for (std::size_t i = 0; i < 3; ++i) { state_.gyro_rad_s[i] = gyro_body[i] - state_.gyro_bias_rad_s[i]; }
    predict(state_.gyro_rad_s, dt_s);
    state_.roll_pitch_yaw_rad = roll_pitch_yaw_rad(state_.q);
    return AttitudeResult::ok;
}

AttitudeResult AttitudeEstimator::correct_accel(const std::array<float, 3>& accel_sensor_mps2,
    float dt_s, float max_gravity_error_mps2) noexcept {
    if (!finite3(accel_sensor_mps2)) { return AttitudeResult::invalid_input; }
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) { return AttitudeResult::invalid_dt; }
    const auto accel_body = apply_rotation(install_, accel_sensor_mps2);
    if (std::fabs(norm3(accel_body) - 9.81f) > max_gravity_error_mps2) {
        ++state_.measurement_skips;
        return AttitudeResult::ok;
    }

    // 加速度低通: 一阶, 时间常数为 0 时直接取当前值.
    if (!has_accel_ || config_.accel_lpf_s <= 0.0f) {
        accel_lpf_ = accel_body;
        has_accel_ = true;
    } else {
        const float a = config_.accel_lpf_s;
        const float weight = dt_s / (a + dt_s);
        for (std::size_t i = 0; i < 3; ++i) {
            accel_lpf_[i] = accel_lpf_[i] * (1.0f - weight) + accel_body[i] * weight;
        }
    }
    state_.accel_mps2 = accel_lpf_;
    // 静止判据: 角速度小且比力接近 1 g, 用于卡方检验的连续拒绝判定.
    state_.stable = norm3(state_.gyro_rad_s) < 0.3f && std::fabs(norm3(accel_lpf_) - 9.8f) < 0.5f;
    correct(accel_lpf_, dt_s);
    state_.roll_pitch_yaw_rad = roll_pitch_yaw_rad(state_.q);
    return AttitudeResult::ok;
}
} // 命名空间 ec::estimation
