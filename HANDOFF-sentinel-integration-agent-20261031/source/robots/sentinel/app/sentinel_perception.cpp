#include "sentinel_perception.hpp"
#include <cmath>
#include <algorithm>
#include <numbers>

namespace ec::sentinel::app {
namespace {
bool finite3(const Vector3& v) noexcept { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }
bool coarse_expired(Tick now, Tick sample, std::uint64_t horizon_us) noexcept {
    const auto ms=now.ms-sample.ms;
    return ms<0x80000000u && ms>horizon_us/1000u+1u; // 不把毫秒取整差/倒退当作一个巨大正间隔.
}
Tick millisecond_time(MicroTick sample, Tick now, MicroTick now_us) noexcept {
    return {now.ms - (now_us.us-sample.us)/1000u};
}
estimation::HeadingTiltConfig tilt_config(const SentinelConfig& config) noexcept {
    estimation::HeadingTiltConfig out;
    out.max_predict_dt_s=out.max_accel_dt_s=static_cast<float>(config.perception.max_gyro_step_us)*1e-6f;
    if (config.serial) {
        out.norm_tolerance_mps2=config.serial->accel_norm_error_mps2;
        out.min_projection_squared=config.serial->min_projection_squared;
    }
    return out;
}
} // 内部辅助
SentinelPerception::SentinelPerception(const SentinelConfig& config, ConfigAudience audience) noexcept
    : config_(config), report_(validate(config, audience)),
      attitude_(tilt_config(config), config.serial ? config.serial->install : estimation::identity_rotation()),
      wheel_drive_(omni_drive(config)) {
    wheel_.rank_ok = wheel_drive_ && motion::omni_well_posed(*wheel_drive_);
    for (const auto& slot : config.dji) {
        if (slot.installed && is_chassis(slot.domain)) { wheel_ttl_[domain_index(slot.domain)] = slot.feedback_ttl; }
    }
}
bool SentinelPerception::reset_attitude(const Vector3& accel_body) noexcept {
    const auto bias=config_.serial ? config_.serial->gyro_bias_sensor : Vector3{};
    if (!attitude_.reset(estimation::quaternion_from_gravity(accel_body),bias)) { return false; }
    primed_ = true; gyro_gap_ = false; heading_started_ = false; alignment_valid_ = false;
    ++local_epoch_;
    return true;
}
void SentinelPerception::expire_joint_samples(Tick now, MicroTick now_us) noexcept {
    for (std::size_t i=0;i<2;++i) {
        if (const auto last=joint_history_[i].latest_time(); last &&
            (!micro_fresh(now_us,*last,config_.perception.joint_age) ||
             coarse_expired(now,joint_sample_ms_[i],config_.perception.joint_age.us))) { joint_history_[i].clear(); }
    }
}
std::optional<Vector3> SentinelPerception::heading_anchor(MicroTick now_us) noexcept {
    if (!config_.serial || !report_.startable) { return std::nullopt; }
    const auto& model=*config_.serial;
    std::array<float,2> angle{};
    for (std::size_t i=0;i<2;++i) {
        const auto last=joint_history_[i].latest_time();
        if (!last || !micro_fresh(now_us,*last,config_.perception.joint_age)) { return std::nullopt; }
        tilt_.source_age_us[i]=now_us.us-last->us;
        const MicroTick query{gyro_us_.us-static_cast<std::uint32_t>(model.position_lag_us[i])};
        tilt_.queried_at[i]=query;
        const auto sample=joint_history_[i].at(query,config_.perception.extrapolation);
        if (!sample) { return std::nullopt; }
        if (query.us!=last->us && !micro_regressed(query,*last)) {
            tilt_.extrapolated_mask|=static_cast<std::uint8_t>(1u<<i);
        }
        angle[i]=sample->position;
    }
    const auto k=estimation::multiply(axis_rotation(model.fire_axis,angle[0]-model.fire_reference),
        axis_rotation(model.pitch_axis,angle[1]-model.pitch_reference));
    return estimation::rotate_reference_to_body(k,model.forward);
}
void SentinelPerception::expire_samples(Tick now, MicroTick now_us) noexcept {
    // 毫秒时钟只跨接微秒半圈以外的失效判断, 正常采样/积分仍用原微秒时间.
    const auto expired = [&](MicroTick at, Tick at_ms, MicroLifetime ttl) {
        return !micro_fresh(now_us, at, ttl) || coarse_expired(now,at_ms,ttl.us);
    };
    const auto gap = now_us.us-gyro_us_.us;
    // 留出允许的摄入延迟: sample 间隔正好可积分时, 晚到一点不能被服务时刻误判为重建.
    const std::uint64_t horizon=static_cast<std::uint64_t>(config_.perception.max_gyro_step_us)+config_.perception.gyro_age.us;
    if (primed_ && ((gap < 0x80000000u && gap > horizon) ||
        coarse_expired(now,last_sample_at_,horizon))) { gyro_gap_ = true; }
    if (accel_seen_ && expired(accel_us_, last_accel_at_, config_.perception.gyro_age)) { accel_fresh_ = false; }
}
void SentinelPerception::submit_imu(const Vector3& gyro, const Vector3& accel, Tick sampled, Tick ingested) noexcept {
    drivers::ImuSamplingTimes times;
    times.gyro = times.gyro_estimated = times.accel = {sampled.ms*1000u};
    times.gyro_valid = times.accel_valid = times.gyro_updated = times.accel_updated = true;
    times.gyro_sequence = times.accel_sequence = sampled.ms;
    times.gyro_time_error_us = 1000; // 毫秒适配入口不伪装成微秒精度
    submit_timed_imu(gyro, accel, times, ingested, {ingested.ms*1000u});
}
void SentinelPerception::submit_timed_imu(const Vector3& gyro, const Vector3& accel,
    const drivers::ImuSamplingTimes& times, Tick ingested, MicroTick ingested_us) noexcept {
    const auto at = times.gyro_estimated;
    if (!config_.perception.max_gyro_step_us) { ++stats_.rejected_invalid; return; }
    expire_samples(ingested, ingested_us);
    expire_joint_samples(ingested, ingested_us); // 校正前淘汰, 不让微秒绕完整圈的旧历史成为锚点.
    bool new_gyro = times.gyro_valid && times.gyro_updated;
    bool new_accel = times.accel_valid && times.accel_updated;
    if (new_gyro && !finite3(gyro)) { ++stats_.rejected_invalid; new_gyro = false; }
    if (new_accel && !finite3(accel)) { ++stats_.accel_rejected; new_accel = false; }
    if (new_gyro && primed_) {
        if (times.gyro_sequence == gyro_sequence_ || (!gyro_gap_ && at.us == gyro_us_.us)) { ++stats_.rejected_duplicate; new_gyro = false; }
        else if (!gyro_gap_ && micro_regressed(at, gyro_us_)) { ++stats_.rejected_regression; new_gyro = false; }
    }
    if (new_gyro && !micro_fresh(ingested_us, at, config_.perception.gyro_age)) {
        ++stats_.rejected_too_old; new_gyro = false;
    }
    if (new_accel && accel_seen_ && (times.accel_sequence == accel_sequence_ ||
        (accel_fresh_ && !micro_regressed(accel_us_, times.accel)))) { new_accel = false; }
    if (new_accel && !micro_fresh(ingested_us, times.accel, config_.perception.gyro_age)) { new_accel = false; ++stats_.accel_rejected; }
    const float accel_dt = accel_fresh_ ? static_cast<float>(times.accel.us-accel_us_.us)*1e-6f : config_.timing.control_dt_s;
    if (new_accel) {
        // 原始比力新鲜度与是否适合重力校正分开; 独立到达的 accel 可供随后 gyro 重建使用.
        last_accel_mps2_ = estimation::apply_rotation(attitude_.install_rotation(), accel);
        accel_us_ = times.accel; accel_sequence_ = times.accel_sequence; accel_seen_ = accel_fresh_ = true;
        last_accel_at_ = millisecond_time(times.accel, ingested, ingested_us);
    }
    if (new_gyro) {
        const bool reset = !primed_ || gyro_gap_ || at.us-gyro_us_.us > config_.perception.max_gyro_step_us;
        if (reset) {
            const auto difference=static_cast<std::int32_t>(accel_us_.us-at.us);
            if (!accel_fresh_ || std::abs(static_cast<std::int64_t>(difference))>config_.perception.extrapolation.us ||
                std::fabs(std::sqrt(dot(last_accel_mps2_,last_accel_mps2_))-9.81f) >
                (config_.serial ? config_.serial->accel_norm_error_mps2 : 0.8f)) {
                gyro_gap_=true;
                ++stats_.rejected_invalid; return;
            }
            Vector3 calibrated{};
            const auto bias=config_.serial ? config_.serial->gyro_bias_sensor : Vector3{};
            for (std::size_t i=0;i<3;++i) { calibrated[i]=gyro[i]-bias[i]; }
            const auto rate=estimation::apply_rotation(attitude_.install_rotation(),calibrated);
            const auto aligned=estimation::rotate_body_to_reference(
                propagate({},rate,static_cast<float>(difference)*1e-6f),last_accel_mps2_);
            const bool was_primed=primed_;
            if (!reset_attitude(aligned)) { ++stats_.rejected_invalid; return; }
            if (was_primed) { ++stats_.rejected_long_gap; ++stats_.reprime_events; }
            reference_at_ = ingested;
            last_gyro_rad_s_ = rate;
        } else {
            const auto result = attitude_.predict(gyro, static_cast<float>(at.us-gyro_us_.us)*1e-6f);
            if (result != estimation::TiltResult::applied) { ++stats_.rejected_invalid; return; }
            last_gyro_rad_s_ = attitude_.gyro_rad_s();
        }
        gyro_us_ = at; gyro_sequence_ = times.gyro_sequence; gyro_error_us_ = times.gyro_time_error_us;
        last_sample_at_ = millisecond_time(at, ingested, ingested_us); last_ingested_at_ = ingested;
        ++stats_.accepted;
    }
    if (new_accel && primed_ && !gyro_gap_) {
        tilt_.local_epoch=local_epoch_;
        tilt_.accel_at=times.accel; tilt_.gyro_at=gyro_us_; tilt_.evaluated_at=ingested_us;
        tilt_.queried_at={}; tilt_.source_age_us={0xffffffffu,0xffffffffu}; tilt_.extrapolated_mask=0;
        tilt_.innovation_rad=tilt_.tilt_step_rad=tilt_.heading_injection_rad=0;
        const auto rejected=[&](std::uint8_t reason) { tilt_.result=reason; ++tilt_.counts[reason]; ++stats_.accel_rejected; };
        // 已到达的accel与gyro在既有窗口内对齐, 不重复旧样本, 不延后重放缺锚点的量测.
        const auto difference = static_cast<std::int32_t>(times.accel.us-gyro_us_.us);
        if (!micro_fresh(ingested_us,gyro_us_,config_.perception.gyro_age) ||
            std::abs(static_cast<std::int64_t>(difference)) > config_.perception.extrapolation.us) {
            rejected(TiltFusionDiagnostics::unaligned); return;
        }
        const auto anchor=heading_anchor(ingested_us);
        if (!anchor) { rejected(TiltFusionDiagnostics::missing_anchor); return; }
        const auto correction = propagate({}, last_gyro_rad_s_, static_cast<float>(difference)*1e-6f);
        const auto aligned_body = estimation::rotate_body_to_reference(correction, last_accel_mps2_);
        const auto result=attitude_.correct_accel_aligned(aligned_body,*anchor,accel_dt);
        const auto& diagnostic=attitude_.diagnostics();
        tilt_.innovation_rad=diagnostic.innovation_rad;
        tilt_.tilt_step_rad=diagnostic.tilt_step_rad;
        tilt_.heading_injection_rad=diagnostic.heading_injection_rad;
        if (result!=estimation::TiltResult::applied && result!=estimation::TiltResult::no_change) {
            rejected(static_cast<std::uint8_t>(result)); return;
        }
        tilt_.result=static_cast<std::uint8_t>(result); ++tilt_.counts[tilt_.result];
        if (result==estimation::TiltResult::applied) { ++stats_.accel_updates; }
        tilt_.max_abs_heading_injection_rad=std::max(tilt_.max_abs_heading_injection_rad,
            std::fabs(tilt_.heading_injection_rad));
    }
}
void SentinelPerception::submit_joint(Domain domain, const DomainFeedback& feedback, MicroTick sampled,
    std::uint32_t reference_epoch) noexcept {
    if (domain != Domain::fire_yaw && domain != Domain::pitch) { return; }
    const auto i = domain == Domain::fire_yaw ? 0u : 1u;
    if (reference_epoch != joint_epochs_[i]) {
        joint_epochs_[i] = reference_epoch;
        joint_history_[i].clear();
        // 关节坐标基准改变使过去历史不可拼接; 不重新设 IMU 原点.
    }
    if (feedback.position_rad.validity != Validity::valid || feedback.velocity_rad_s.validity != Validity::valid) {
        joint_history_[i].clear(); return;
    }
    // 新的毫秒样本明确跨过历史覆盖时, 不能再拿微秒半圈比较把它永久误认成倒退.
    if (joint_history_[i].latest_time() && coarse_expired(feedback.position_rad.sampled_at,
        joint_sample_ms_[i],config_.perception.joint_age.us)) { joint_history_[i].clear(); }
    if (joint_history_[i].push({sampled, feedback.position_rad.value, feedback.velocity_rad_s.value})) {
        joint_sample_ms_[i] = feedback.position_rad.sampled_at;
    }
}
void SentinelPerception::reconstruct_platform(MicroTick now, const Quaternion& sensor_pose) noexcept {
    platform_ = PlatformFeedback{};
    platform_.local_epoch = local_epoch_; platform_.gyro_at = gyro_us_; platform_.evaluated_at = now;
    platform_.odom_aligned = alignment_valid_; platform_.odom_offset_rad = alignment_offset_rad_;
    platform_.alignment_epoch = alignment_epoch_;
    platform_.ages_us[0] = now.us-gyro_us_.us;
    platform_.time_error_us[0] = gyro_error_us_;
    if (!config_.serial || !report_.startable) { return; }
    if (inertial_.validity != DomainValidity::valid) {
        platform_.validity = inertial_.validity; platform_.reason = inertial_.reason; return;
    }
    const auto& model = *config_.serial;
    std::array<float, 2> position{}, velocity{};
    for (std::size_t i = 0; i < 2; ++i) {
        const auto last = joint_history_[i].latest_time();
        platform_.ages_us[i+1] = last ? now.us-last->us : 0xffffffffu;
        const auto p = joint_history_[i].at({now.us-static_cast<std::uint32_t>(model.position_lag_us[i])}, config_.perception.extrapolation);
        const auto v = joint_history_[i].at({now.us-static_cast<std::uint32_t>(model.velocity_lag_us[i])}, config_.perception.extrapolation);
        if (!last || !micro_fresh(now, *last, config_.perception.joint_age) || !p || !v) {
            platform_.validity = last ? DomainValidity::stale : DomainValidity::invalid;
            platform_.reason = last ? Reason::stale_feedback : Reason::no_reference;
            ++stats_.reconstruction_missing; return;
        }
        position[i] = p->position; velocity[i] = v->velocity;
    }
    platform_.value = reconstruct(model, sensor_pose, last_gyro_rad_s_, position[0], velocity[0], position[1], velocity[1]);
    if (!platform_.value.valid) { platform_.validity = DomainValidity::invalid; platform_.reason = Reason::invalid_feedback; ++stats_.projection_rejected; return; }
    const auto heading = platform_.value.heading;
    if (!heading_started_) { continuous_heading_ = heading; heading_started_ = true; }
    else { continuous_heading_ += std::remainder(heading-last_heading_, 2.0f*std::numbers::pi_v<float>); }
    last_heading_ = heading;
    platform_.continuous_heading = continuous_heading_;
    platform_.local_reference = true; platform_.validity = DomainValidity::valid; platform_.reason = Reason::none;
}
void SentinelPerception::update(Tick now, std::optional<MicroTick> now_us) noexcept {
    const auto time = now_us.value_or(MicroTick{now.ms*1000u});
    expire_samples(now, time);
    expire_joint_samples(now,time);
    inertial_ = InertialFeedback{};
    const auto finish = [&](const Quaternion& pose) { reconstruct_platform(time, pose); };
    if (!config_.perception.max_gyro_step_us) { inertial_.reason = Reason::incomplete_config; finish({}); return; }
    if (!primed_) { inertial_.validity = DomainValidity::unknown; inertial_.reason = Reason::no_feedback; finish({}); return; }
    const auto age_us = time.us-gyro_us_.us;
    inertial_.sampled_at = last_sample_at_; inertial_.ingested_at = last_ingested_at_; inertial_.age_ms = age_us/1000u;
    // 设备在线 TTL 与可可靠传播窗口不同. 超出外推界就不可用, 不借大 TTL 无限预测.
    if (gyro_gap_ || !micro_fresh(time, gyro_us_, config_.perception.gyro_age) || (age_us && age_us > config_.perception.extrapolation.us)) {
        inertial_.validity = DomainValidity::stale; inertial_.reason = Reason::stale_feedback;
        ++stats_.invalidations; finish({}); return;
    }
    auto q = attitude_.orientation();
    if (age_us) { q = propagate(q, last_gyro_rad_s_, static_cast<float>(age_us)*1e-6f); ++stats_.extrapolations; }
    inertial_.validity = DomainValidity::valid; inertial_.reason = Reason::none;
    inertial_.gyro_rad_s = {last_gyro_rad_s_, last_sample_at_, Validity::valid};
    inertial_.accel_mps2 = {last_accel_mps2_, last_accel_at_,
        accel_fresh_ && micro_fresh(time, accel_us_, config_.perception.gyro_age) ? Validity::valid : Validity::invalid};
    inertial_.roll_pitch_yaw_rad = {estimation::roll_pitch_yaw_rad(q), now, Validity::valid};
    inertial_.gravity_direction_body = {estimation::rotate_reference_to_body(q, {0,0,1}), now, Validity::valid};
    inertial_.extrapolated = age_us != 0;
    inertial_.reference_valid = alignment_valid_; inertial_.reference_established_at = reference_at_;
    finish(q);
}
void SentinelPerception::shift_joint_zero(Domain domain, float delta_joint_rad) noexcept {
    if (!config_.serial || (domain != Domain::fire_yaw && domain != Domain::pitch)) { return; }
    const auto i = domain == Domain::fire_yaw ? 0u : 1u;
    auto& reference = i == 0 ? config_.serial->fire_reference : config_.serial->pitch_reference;
    reference += delta_joint_rad;
    joint_history_[i].clear();
}
void SentinelPerception::invalidate_joint_reference(Domain domain) noexcept {
    if (!config_.serial || (domain != Domain::fire_yaw && domain != Domain::pitch)) { return; }
    joint_history_[domain == Domain::fire_yaw ? 0u : 1u].clear();
    platform_ = {};
    platform_.validity = DomainValidity::invalid;
    platform_.reason = Reason::no_reference;
    alignment_valid_ = false;
    inertial_.reference_valid = false;
    heading_started_ = false;
}
void SentinelPerception::set_yaw_alignment(float offset_rad, std::uint32_t epoch) noexcept {
    if (!std::isfinite(offset_rad)) { return; }
    if (!alignment_valid_ || epoch != alignment_epoch_ || offset_rad != alignment_offset_rad_) { ++stats_.alignment_changes; }
    alignment_valid_ = true; alignment_offset_rad_ = offset_rad; alignment_epoch_ = epoch;
}
std::optional<float> SentinelPerception::local_direction_rad() const noexcept {
    return platform_.validity == DomainValidity::valid ? std::optional{platform_.continuous_heading} : std::nullopt;
}
std::optional<float> SentinelPerception::yaw_direction_rad() const noexcept {
    const auto direction = local_direction_rad();
    return alignment_valid_ && direction ? std::optional{*direction+alignment_offset_rad_} : std::nullopt;
}
void SentinelPerception::integrate_commanded_odom(const motion::PlanarTwist& adopted_twist, Tick at) noexcept {
    if (!std::isfinite(adopted_twist.vx_mps) || !std::isfinite(adopted_twist.vy_mps) || !std::isfinite(adopted_twist.wz_rad_s)) {
        ++odom_.rejected_steps; return;
    }
    if (!odom_started_) { odom_started_ = true; odom_at_ = at; odom_.at = at; odom_.valid = true; return; }
    const float dt_s = static_cast<float>(at.ms-odom_at_.ms)*0.001f;
    const float max_dt_s = config_.timing.max_dt_s.value_or(0.0f);
    if (!(dt_s > 0.0f) || (max_dt_s > 0.0f && dt_s > max_dt_s)) { ++odom_.rejected_steps; odom_at_ = at; return; }
    const float cos_yaw = std::cos(odom_.yaw_rad), sin_yaw = std::sin(odom_.yaw_rad);
    odom_.x_m += (adopted_twist.vx_mps*cos_yaw - adopted_twist.vy_mps*sin_yaw)*dt_s;
    odom_.y_m += (adopted_twist.vx_mps*sin_yaw + adopted_twist.vy_mps*cos_yaw)*dt_s;
    odom_.yaw_rad += adopted_twist.wz_rad_s*dt_s; odom_.at = at;
    odom_.integrated_ms += at.ms-odom_at_.ms; odom_at_ = at; odom_.valid = true;
}
void SentinelPerception::update_wheel_odom(const std::array<DomainFeedback, 4>& wheels,
    const std::array<std::optional<MicroTick>, 4>& times, Tick now, MicroTick now_us) noexcept {
    const auto gap = [&]() {
        if (wheel_.updates != 0 && !wheel_gap_) { ++wheel_.gaps; wheel_.continuous = false; }
        wheel_gap_ = true; wheel_interval_started_ = false; wheel_.pose.valid = false;
    };
    if (!wheel_.rank_ok) { wheel_.validity = DomainValidity::unavailable; return; }
    std::array<float, 4> speed{};
    std::uint32_t oldest = 0, youngest = 0xffffffffu, min_ttl = 0xffffffffu;
    auto quality = DomainValidity::valid;
    for (std::size_t i=0; i<4; ++i) {
        const auto& sample = wheels[i].velocity_rad_s;
        wheel_.ages_us[i] = times[i] ? now_us.us-times[i]->us : 0xffffffffu;
        if (times[i]) { wheel_.samples[i] = *times[i]; }
        speed[i] = sample.value;
        if (!sample.fresh_at(now, wheel_ttl_[i]) || wheel_.ages_us[i]>=0x80000000u ||
            wheel_.ages_us[i]/1000u>=wheel_ttl_[i].ms ||
            !std::isfinite(sample.value)) {
            if (sample.validity == Validity::invalid || !std::isfinite(sample.value)) {
                quality = DomainValidity::invalid;
            } else if (quality != DomainValidity::invalid) {
                quality = wheel_.updates ? DomainValidity::stale : DomainValidity::unavailable;
            }
        }
        oldest = std::max(oldest, wheel_.ages_us[i]);
        youngest = std::min(youngest, wheel_.ages_us[i]);
        min_ttl = std::min(min_ttl, wheel_ttl_[i].ms);
    }
    wheel_.validity = quality;
    if (quality != DomainValidity::valid) { gap(); return; }
    wheel_.skew_us = oldest-youngest;
    const MicroTick watermark{now_us.us-oldest};
    const Tick watermark_ms=millisecond_time(watermark, now, now_us);
    // 任务若也停过, 毫秒时基仍能证明半圈/整圈以外的未知间隔, 先撤销旧判序基线.
    if (wheel_interval_started_ && coarse_expired(watermark_ms, wheel_.pose.at,
            static_cast<std::uint64_t>(min_ttl)*1000u)) { gap(); }
    if (wheel_interval_started_ && watermark.us == wheel_.at.us) { return; } // 控制周期重复读同组反馈不重复积分.
    const auto measured = motion::omni_twist(*wheel_drive_, speed);
    if (!measured) { wheel_.validity=DomainValidity::invalid; gap(); return; }
    const auto predicted = motion::omni_wheel_speeds(*wheel_drive_, *measured);
    if (!predicted) { wheel_.validity=DomainValidity::invalid; gap(); return; }
    float residual_squared{};
    for (std::size_t i=0; i<4; ++i) {
        const float error = speed[i]-(*predicted)[i];
        residual_squared += error*error;
    }
    wheel_.residual_rad_s = std::sqrt(residual_squared/4.0f);
    const auto dt_us = watermark.us-wheel_.at.us;
    // 水位线中断/倒退后只重建积分区间, 不跨未知时间外推, 也不重置已有轮式位置.
    if (wheel_interval_started_ && dt_us>=0x80000000u) { wheel_.validity=DomainValidity::invalid; gap(); return; }
    if (wheel_interval_started_ && dt_us/1000u>=min_ttl) { gap(); }
    if (wheel_interval_started_) {
        const float dt = static_cast<float>(dt_us)*1e-6f;
        const float vx = (previous_wheel_twist_.vx_mps+measured->vx_mps)*0.5f;
        const float vy = (previous_wheel_twist_.vy_mps+measured->vy_mps)*0.5f;
        const float turn = (previous_wheel_twist_.wz_rad_s+measured->wz_rad_s)*0.5f*dt;
        // 常 twist 的 SE(2) 增量; 小角级数避免 1-cos(theta) 的消减误差.
        const float a = std::fabs(turn)<0.001f ? 1-turn*turn/6 : std::sin(turn)/turn;
        const float b = std::fabs(turn)<0.001f ? turn*0.5f-turn*turn*turn/24 : (1-std::cos(turn))/turn;
        const float x = dt*(a*vx-b*vy), y = dt*(b*vx+a*vy);
        const float c = std::cos(wheel_.pose.yaw_rad), s = std::sin(wheel_.pose.yaw_rad);
        wheel_.pose.x_m += c*x-s*y; wheel_.pose.y_m += s*x+c*y;
        wheel_.pose.yaw_rad += turn;
        wheel_.integrated_us += dt_us; wheel_.pose.integrated_ms = wheel_.integrated_us/1000u;
    }
    wheel_.pose.at = watermark_ms;
    wheel_.pose.valid = true; wheel_.at = watermark; wheel_.twist = *measured;
    previous_wheel_twist_ = *measured;
    wheel_interval_started_ = true; wheel_gap_ = false; ++wheel_.updates;
}
} // 命名空间 ec::sentinel::app
