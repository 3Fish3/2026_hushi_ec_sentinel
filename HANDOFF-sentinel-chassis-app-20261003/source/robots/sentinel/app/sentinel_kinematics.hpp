#pragma once
#include "ec/estimation/attitude.hpp"
#include "ec/time.hpp"
#include <array>
#include <optional>

namespace ec::sentinel::app {
using Vector3 = std::array<float, 3>;
using Quaternion = estimation::Quaternion;
[[nodiscard]] float dot(const Vector3& a, const Vector3& b) noexcept;
[[nodiscard]] Vector3 cross(const Vector3& a, const Vector3& b) noexcept;
[[nodiscard]] Quaternion axis_rotation(const Vector3& axis, float radians) noexcept;
[[nodiscard]] Quaternion conjugate(const Quaternion& q) noexcept;
[[nodiscard]] Quaternion propagate(const Quaternion& q, const Vector3& rate, float seconds) noexcept;

// V=S0 是坐标定义. install 将原始芯片系转到日志/计算 S 系, bias 仍属于原始芯片系.
struct SerialGeometry {
    Vector3 fire_axis{}, pitch_axis{}, main_axis{}, forward{};
    Vector3 gyro_bias_sensor{};
    estimation::RotationMatrix3 install{};
    float fire_reference{}, pitch_reference{};
    // 正 lag: gyro(t) 对应 encoder(t-lag). 位置与速度可以有不同滤波延迟.
    std::array<std::int32_t, 2> position_lag_us{}, velocity_lag_us{}; // fire/pitch
    float min_projection_squared{}, min_heading_gain{}, accel_norm_error_mps2{};
    [[nodiscard]] bool valid() const noexcept;
};
struct PlatformKinematics {
    Quaternion orientation{}; // V -> W
    Vector3 rate_V{};
    float heading{}, heading_rate{}, heading_gain{}, projection_squared{};
    bool valid{};
};
[[nodiscard]] PlatformKinematics reconstruct(const SerialGeometry& model, const Quaternion& sensor_orientation,
    const Vector3& sensor_rate, float fire_angle, float fire_rate, float pitch_angle, float pitch_rate) noexcept;
// 父级在本次解算内固定, 由真实三维主轴旋转模型求机械可达分支, 不借另一轴补偿.
[[nodiscard]] std::optional<float> reachable_heading(const SerialGeometry& model, const Quaternion& orientation,
    float current_joint, float target_heading, float min_joint, float max_joint) noexcept;

struct JointTimed {
    MicroTick at{};
    float position{}, velocity{};
};
// 16 个真实样本的短历史. 插值或有界匀速外推, 不修改原记录; 参考失效必须 clear.
class JointHistory {
public:
    bool push(JointTimed sample) noexcept;
    void clear() noexcept { count_ = 0; }
    [[nodiscard]] std::optional<JointTimed> at(MicroTick time, MicroLifetime extrapolation) const noexcept;
    [[nodiscard]] std::optional<MicroTick> latest_time() const noexcept {
        return count_ ? std::optional{samples_[count_-1].at} : std::nullopt;
    }
private:
    std::array<JointTimed, 16> samples_{};
    std::size_t count_{};
};
} // 命名空间 ec::sentinel::app
