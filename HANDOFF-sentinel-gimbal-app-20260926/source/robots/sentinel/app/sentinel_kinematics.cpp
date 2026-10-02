#include "sentinel_kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace ec::sentinel::app {
namespace {
constexpr float tau = 2.0f * std::numbers::pi_v<float>;
bool finite(const Vector3& v) noexcept { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }
Vector3 scale(const Vector3& v, float s) noexcept { return {v[0]*s, v[1]*s, v[2]*s}; }
Vector3 add(const Vector3& a, const Vector3& b) noexcept { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
float heading_derivative(const Vector3& f, const Vector3& rate, float horizontal_squared) noexcept {
    const auto derivative = cross(rate, f);
    return (f[0]*derivative[1] - f[1]*derivative[0]) / horizontal_squared;
}
} // 内部辅助
float dot(const Vector3& a, const Vector3& b) noexcept { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
Vector3 cross(const Vector3& a, const Vector3& b) noexcept {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
Quaternion axis_rotation(const Vector3& axis, float radians) noexcept {
    const auto s = std::sin(0.5f*radians);
    return {std::cos(0.5f*radians), axis[0]*s, axis[1]*s, axis[2]*s};
}
Quaternion conjugate(const Quaternion& q) noexcept { return {q.w, -q.x, -q.y, -q.z}; }
Quaternion propagate(const Quaternion& q, const Vector3& rate, float seconds) noexcept {
    const auto norm = std::sqrt(dot(rate, rate));
    if (norm < 1e-12f) { return q; }
    return estimation::normalized(estimation::multiply(q, axis_rotation(scale(rate, 1.0f/norm), norm*seconds)));
}
bool SerialGeometry::valid() const noexcept {
    for (const auto& v : {fire_axis, pitch_axis, main_axis, forward}) {
        if (!finite(v) || std::fabs(dot(v, v)-1.0f) > 0.001f) { return false; }
    }
    if (!finite(gyro_bias_sensor) || !std::isfinite(fire_reference) || !std::isfinite(pitch_reference) ||
        !std::isfinite(min_projection_squared) || min_projection_squared <= 0 || min_projection_squared >= 1 ||
        !std::isfinite(min_heading_gain) || min_heading_gain <= 0 ||
        !std::isfinite(accel_norm_error_mps2) || accel_norm_error_mps2 <= 0) { return false; }
    for (auto lag : position_lag_us) { if (lag < -10000 || lag > 10000) { return false; } }
    for (auto lag : velocity_lag_us) { if (lag < -10000 || lag > 10000) { return false; } }
    const Vector3 x{install.m[0], install.m[3], install.m[6]}, y{install.m[1], install.m[4], install.m[7]}, z{install.m[2], install.m[5], install.m[8]};
    return finite(x) && finite(y) && finite(z) && std::fabs(dot(x,x)-1)<0.001f && std::fabs(dot(y,y)-1)<0.001f &&
        std::fabs(dot(z,z)-1)<0.001f && std::fabs(dot(x,y))<0.001f && dot(cross(x,y),z)>0.999f;
}
PlatformKinematics reconstruct(const SerialGeometry& model, const Quaternion& sensor_orientation,
    const Vector3& sensor_rate, float fire_angle, float fire_rate, float pitch_angle, float pitch_rate) noexcept {
    PlatformKinematics out;
    if (!finite(sensor_rate) || !std::isfinite(fire_angle) || !std::isfinite(pitch_angle) ||
        !std::isfinite(fire_rate) || !std::isfinite(pitch_rate)) { return out; }
    const auto ef = axis_rotation(model.fire_axis, fire_angle-model.fire_reference);
    const auto ep = axis_rotation(model.pitch_axis, pitch_angle-model.pitch_reference);
    const auto k = estimation::multiply(ef, ep);
    out.orientation = estimation::normalized(estimation::multiply(sensor_orientation, conjugate(k)));
    const auto pitch_axis = estimation::rotate_body_to_reference(ef, model.pitch_axis);
    out.rate_V = estimation::rotate_body_to_reference(k, sensor_rate);
    for (std::size_t i = 0; i < 3; ++i) { out.rate_V[i] -= model.fire_axis[i]*fire_rate + pitch_axis[i]*pitch_rate; }
    const auto forward = estimation::rotate_body_to_reference(out.orientation, model.forward);
    out.projection_squared = forward[0]*forward[0]+forward[1]*forward[1];
    if (out.projection_squared < model.min_projection_squared) { return out; }
    const auto rate_world = estimation::rotate_body_to_reference(out.orientation, out.rate_V);
    const auto main_world = estimation::rotate_body_to_reference(out.orientation, model.main_axis);
    out.heading = std::atan2(forward[1], forward[0]);
    out.heading_rate = heading_derivative(forward, rate_world, out.projection_squared);
    out.heading_gain = heading_derivative(forward, main_world, out.projection_squared);
    out.valid = std::isfinite(out.heading_rate) && std::fabs(out.heading_gain) >= model.min_heading_gain;
    return out;
}
std::optional<float> reachable_heading(const SerialGeometry& model, const Quaternion& orientation,
    float current_joint, float target_heading, float min_joint, float max_joint) noexcept {
    if (!std::isfinite(current_joint) || !std::isfinite(target_heading) || !std::isfinite(min_joint) ||
        !std::isfinite(max_joint) || min_joint > max_joint) { return std::nullopt; }
    const auto parallel = scale(model.main_axis, dot(model.main_axis, model.forward));
    const auto u = estimation::rotate_body_to_reference(orientation, add(model.forward, scale(parallel, -1)));
    const auto v = estimation::rotate_body_to_reference(orientation, cross(model.main_axis, model.forward));
    const auto w = estimation::rotate_body_to_reference(orientation, parallel);
    const Vector3 target{std::cos(target_heading), std::sin(target_heading), 0};
    const Vector3 normal{-target[1], target[0], 0};
    const auto a = dot(normal,u), b = dot(normal,v), c = dot(normal,w);
    const auto radius = std::hypot(a,b);
    if (radius < 1e-7f || std::fabs(c) > radius) { return std::nullopt; }
    const auto centre = std::atan2(b,a), offset = std::acos(std::clamp(-c/radius, -1.0f, 1.0f));
    std::optional<float> best;
    for (const auto root : {centre+offset, centre-offset}) {
        const auto lo = std::ceil((min_joint-current_joint-root)/tau), hi = std::floor((max_joint-current_joint-root)/tau);
        if (lo > hi) { continue; }
        const auto branch = std::clamp(std::round(-root/tau), lo, hi);
        const auto delta = root+tau*branch;
        const auto direction = add(add(scale(u, std::cos(delta)), scale(v, std::sin(delta))), w);
        if (dot(target, direction) <= 0 || direction[0]*direction[0]+direction[1]*direction[1] < model.min_projection_squared) { continue; }
        const auto joint = current_joint+delta;
        if (!best || std::fabs(delta) < std::fabs(*best-current_joint)) { best = joint; }
    }
    return best;
}
bool JointHistory::push(JointTimed sample) noexcept {
    if (!std::isfinite(sample.position) || !std::isfinite(sample.velocity)) { return false; }
    if (count_ && (sample.at.us == samples_[count_-1].at.us || micro_regressed(sample.at, samples_[count_-1].at))) { return false; }
    if (count_ == samples_.size()) {
        for (std::size_t i = 1; i < count_; ++i) { samples_[i-1] = samples_[i]; }
        --count_;
    }
    samples_[count_++] = sample;
    return true;
}
std::optional<JointTimed> JointHistory::at(MicroTick time, MicroLifetime extrapolation) const noexcept {
    if (!count_) { return std::nullopt; }
    const auto& last = samples_[count_-1];
    if (!micro_regressed(time, last.at)) {
        const auto gap = time.us-last.at.us;
        if (gap && (!extrapolation.valid() || gap >= extrapolation.us)) { return std::nullopt; }
        return JointTimed{time, last.position+last.velocity*static_cast<float>(gap)*1e-6f, last.velocity};
    }
    for (std::size_t i = 1; i < count_; ++i) {
        const auto& left = samples_[i-1]; const auto& right = samples_[i];
        if (micro_regressed(time, left.at) || micro_regressed(right.at, time)) { continue; }
        if (right.at.us-left.at.us > extrapolation.us) { return std::nullopt; }
        const auto weight = static_cast<float>(time.us-left.at.us)/static_cast<float>(right.at.us-left.at.us);
        return JointTimed{time, left.position+weight*(right.position-left.position), left.velocity+weight*(right.velocity-left.velocity)};
    }
    return std::nullopt;
}
} // 命名空间 ec::sentinel::app
