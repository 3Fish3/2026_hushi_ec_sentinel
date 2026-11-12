#include "ec/estimation/heading_tilt.hpp"
#include <algorithm>
#include <cmath>

namespace ec::estimation {
namespace {
using Vec = std::array<float,3>;
bool finite(Vec v) noexcept { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }
float dot(Vec a, Vec b) noexcept { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
Vec scale(Vec v, float s) noexcept { return {v[0]*s,v[1]*s,v[2]*s}; }
bool valid_quaternion(Quaternion q) noexcept {
    const float n=q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
    return std::isfinite(n) && n>1e-12f;
}
bool valid_install(RotationMatrix3 r) noexcept {
    const Vec x{r.m[0],r.m[3],r.m[6]}, y{r.m[1],r.m[4],r.m[7]}, z{r.m[2],r.m[5],r.m[8]};
    const Vec c{x[1]*y[2]-x[2]*y[1],x[2]*y[0]-x[0]*y[2],x[0]*y[1]-x[1]*y[0]};
    return finite(x) && finite(y) && finite(z) && std::fabs(dot(x,x)-1)<0.001f &&
        std::fabs(dot(y,y)-1)<0.001f && std::fabs(dot(z,z)-1)<0.001f &&
        std::fabs(dot(x,y))<0.001f && std::fabs(dot(y,z))<0.001f &&
        std::fabs(dot(z,x))<0.001f && dot(c,z)>0.999f;
}
float horizontal_squared(Vec v) noexcept { return v[0]*v[0]+v[1]*v[1]; }
Quaternion yaw(float angle) noexcept { return {std::cos(angle*0.5f),0,0,std::sin(angle*0.5f)}; }
// 将候选水平投影转回之前的方向, 避免直接相减跨越atan2分支.
float heading_delta(Vec before, Vec candidate) noexcept {
    return std::atan2(candidate[0]*before[1]-candidate[1]*before[0],
        candidate[0]*before[0]+candidate[1]*before[1]);
}
} // 内部辅助

HeadingTilt::HeadingTilt(HeadingTiltConfig config, RotationMatrix3 install) noexcept
    : config_(config), install_(install) {
    config_valid_=valid_install(install) &&
        std::isfinite(config.time_constant_s) && config.time_constant_s>0 &&
        std::isfinite(config.max_tilt_correction_rad_s) && config.max_tilt_correction_rad_s>0 &&
        std::isfinite(config.norm_tolerance_mps2) && config.norm_tolerance_mps2>0 &&
        std::isfinite(config.min_projection_squared) && config.min_projection_squared>0 && config.min_projection_squared<1 &&
        std::isfinite(config.max_predict_dt_s) && config.max_predict_dt_s>0 &&
        std::isfinite(config.max_accel_dt_s) && config.max_accel_dt_s>0;
}
bool HeadingTilt::reset(Quaternion q, Vec calibration_sensor) noexcept {
    if (!config_valid_ || !valid_quaternion(q) || !finite(calibration_sensor)) { return false; }
    q_=normalized(q); calibration_=calibration_sensor; residual_={}; rate_={};
    diagnostics_={}; ready_=true;
    return true;
}
bool HeadingTilt::set_residual_bias_sensor(Vec residual) noexcept {
    if (!ready_ || !finite(residual)) { return false; }
    residual_=residual;
    return true;
}
TiltResult HeadingTilt::predict(Vec gyro_sensor, float dt_s) noexcept {
    if (!ready_ || !finite(gyro_sensor)) { return TiltResult::invalid_input; }
    if (!std::isfinite(dt_s) || dt_s<=0 || dt_s>config_.max_predict_dt_s) { return TiltResult::invalid_dt; }
    Vec calibrated{};
    for (unsigned i=0;i<3;++i) { calibrated[i]=gyro_sensor[i]-calibration_[i]-residual_[i]; }
    const auto w=apply_rotation(install_,calibrated);
    const float angle2=dot(w,w)*dt_s*dt_s;
    if (!finite(w) || !std::isfinite(angle2)) { return TiltResult::invalid_input; }
    // 指数映射的小角级数, 常见1kHz步长避免调用sin/cos, 不依赖q符号.
    const float angle=std::sqrt(angle2);
    const float c=angle2<1e-6f ? 1-angle2/8+angle2*angle2/384 : std::cos(angle*0.5f);
    const float s=angle2<1e-6f ? 0.5f-angle2/48+angle2*angle2/3840 : std::sin(angle*0.5f)/angle;
    const auto candidate=multiply(q_,{c,w[0]*dt_s*s,w[1]*dt_s*s,w[2]*dt_s*s});
    if (!valid_quaternion(candidate)) { return TiltResult::invalid_input; }
    q_=normalized(candidate); rate_=w;
    return TiltResult::applied;
}
TiltResult HeadingTilt::correct_accel_aligned(Vec accel, Vec anchor, float dt_s) noexcept {
    diagnostics_.innovation_rad=diagnostics_.tilt_step_rad=diagnostics_.heading_injection_rad=0;
    const auto skip=[&](TiltResult result) { ++diagnostics_.skips; return result; };
    if (!ready_ || !finite(accel) || !finite(anchor)) { return skip(TiltResult::invalid_input); }
    if (!std::isfinite(dt_s) || dt_s<=0 || dt_s>config_.max_accel_dt_s) { return skip(TiltResult::invalid_dt); }
    const float a2=dot(accel,accel), u2=dot(anchor,anchor);
    if (!std::isfinite(a2) || !std::isfinite(u2) || a2<1e-6f || u2<1e-6f) { return skip(TiltResult::invalid_input); }
    const float len=std::sqrt(a2);
    if (std::fabs(len-9.81f)>config_.norm_tolerance_mps2) { return skip(TiltResult::rejected_norm); }
    const auto u=scale(anchor,1/std::sqrt(u2));
    const auto before=rotate_body_to_reference(q_,u);
    if (horizontal_squared(before)<config_.min_projection_squared) { return skip(TiltResult::rejected_projection); }
    const auto z=rotate_body_to_reference(q_,scale(accel,1/len));
    const float sn=std::hypot(z[0],z[1]), theta=std::atan2(sn,z[2]);
    diagnostics_.innovation_rad=theta;
    // 反向附近最短轴不稳定, 1e-4rad锥覆盖任意q旋转/归一化舍入, 不任取一根轴.
    if (z[2]<0 && sn<1e-4f) { return skip(TiltResult::rejected_antipodal); }
    if (sn<1e-7f) { return TiltResult::no_change; }
    const float gain=dt_s/(config_.time_constant_s+dt_s);
    const float step=std::min(gain*theta,config_.max_tilt_correction_rad_s*dt_s);
    const float factor=std::sin(0.5f*step)/sn;
    const Quaternion tilt{std::cos(0.5f*step),z[1]*factor,-z[0]*factor,0}; // z×世界Z
    auto candidate=normalized(multiply(tilt,q_));
    auto forward=rotate_body_to_reference(candidate,u);
    if (horizontal_squared(forward)<config_.min_projection_squared) { return skip(TiltResult::rejected_projection); }
    candidate=normalized(multiply(yaw(heading_delta(before,forward)),candidate));
    forward=rotate_body_to_reference(candidate,u);
    // 一次有界的浮点残差校正, 不改变候选的重力方向.
    candidate=normalized(multiply(yaw(heading_delta(before,forward)),candidate));
    const auto after=rotate_body_to_reference(candidate,u);
    if (!valid_quaternion(candidate) || !finite(after) || horizontal_squared(after)<config_.min_projection_squared) {
        return skip(TiltResult::rejected_projection);
    }
    diagnostics_.heading_injection_rad=-heading_delta(before,after);
    diagnostics_.tilt_step_rad=step;
    q_=candidate; ++diagnostics_.corrections;
    return TiltResult::applied;
}
} // 命名空间 ec::estimation
