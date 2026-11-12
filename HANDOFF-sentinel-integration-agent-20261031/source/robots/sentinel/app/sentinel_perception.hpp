#pragma once
// 感知单写者: gyro/acc 独立新样本、微秒历史对齐与末端到主平台的旋转重构.
// 原始惯性快照与主平台反馈分开, 末端 IMU 在线不等于主平台反馈有效.
#include "ec/drivers/bmi088_device.hpp"
#include "ec/estimation/heading_tilt.hpp"
#include "ec/motion/omni_kinematics.hpp"
#include "sentinel_app.hpp"
#include "sentinel_config.hpp"

namespace ec::sentinel::app {
struct ImuStats {
    std::uint32_t accepted{}, rejected_invalid{}, rejected_duplicate{}, rejected_regression{}, rejected_too_old{};
    std::uint32_t rejected_long_gap{}, reprime_events{}, extrapolations{}, invalidations{}, alignment_changes{};
    std::uint32_t accel_updates{}, accel_rejected{}, reconstruction_missing{}, projection_rejected{};
};
// 0..6沿用TiltResult; 7/8表示调用侧缺关节锚点/时刻不对齐. 累计计数与峰值不因低频读取丢失.
struct TiltFusionDiagnostics {
    static constexpr std::uint8_t missing_anchor=7, unaligned=8;
    std::array<std::uint32_t,9> counts{};
    std::uint32_t local_epoch{}; // 最后一次校正请求的代次, 不是读取时的新代次.
    MicroTick accel_at{}, gyro_at{}, evaluated_at{};
    std::array<MicroTick,2> queried_at{};
    std::array<std::uint32_t,2> source_age_us{0xffffffffu,0xffffffffu};
    float innovation_rad{}, tilt_step_rad{}, heading_injection_rad{}, max_abs_heading_injection_rad{};
    std::uint8_t result{255}, extrapolated_mask{}; // 255表示尚无校正请求.
};
// 命令预测不是轮速反馈里程计, 不作为主 yaw 的惯性输入.
struct OdomPose {
    float x_m{}, y_m{}, yaw_rad{};
    Tick at{};
    std::uint32_t integrated_ms{}, rejected_steps{};
    bool valid{};
};
// 轮速逆解, 不包含命令或 IMU 替代值. at 是四轮最新测量的最旧水位线;
// 各轮原始时刻/年龄和 skew 保留, 异步零阶保持不伪称严格同步或无滑移真值.
struct WheelOdometry {
    motion::PlanarTwist twist{};
    OdomPose pose{};
    DomainValidity validity{DomainValidity::unavailable};
    MicroTick at{};
    std::array<MicroTick, 4> samples{};
    std::array<std::uint32_t, 4> ages_us{0xffffffffu,0xffffffffu,0xffffffffu,0xffffffffu};
    std::uint32_t skew_us{}, integrated_us{}, gaps{}, updates{};
    float residual_rad_s{};
    bool rank_ok{}, continuous{true}; // 连续性一旦出现未知间隙就如实保留, 不重新捕获零点.
};
class SentinelPerception {
public:
    SentinelPerception(const SentinelConfig& config, ConfigAudience audience) noexcept;
    SentinelPerception(const SentinelPerception&) = delete;
    SentinelPerception& operator=(const SentinelPerception&) = delete;
    [[nodiscard]] const ConfigReport& report() const noexcept { return report_; }
    [[nodiscard]] const ImuStats& stats() const noexcept { return stats_; }
    // 毫秒样本的显式适配入口 (旧离线夹具); 新板级传独立采集序号与微秒时间.
    void submit_imu(const Vector3& gyro, const Vector3& accel, Tick sampled, Tick ingested) noexcept;
    void submit_timed_imu(const Vector3& gyro, const Vector3& accel, const drivers::ImuSamplingTimes& times,
        Tick ingested, MicroTick ingested_us) noexcept;
    void submit_joint(Domain domain, const DomainFeedback& feedback, MicroTick sampled,
        std::uint32_t reference_epoch) noexcept;
    void update(Tick now, std::optional<MicroTick> now_us = std::nullopt) noexcept;
    [[nodiscard]] const InertialFeedback& inertial() const noexcept { return inertial_; }
    [[nodiscard]] const PlatformFeedback& platform() const noexcept { return platform_; }
    [[nodiscard]] const TiltFusionDiagnostics& tilt_diagnostics() const noexcept { return tilt_; }
    void set_yaw_alignment(float offset_rad, std::uint32_t epoch) noexcept;
    // 机械零位坐标变更时同步移动标定姿态数值, V 的物理定义不变; 旧样本历史不可拼接.
    void shift_joint_zero(Domain domain, float delta_joint_rad) noexcept;
    // 手动纠正下游参考没有可证明的坐标平移: 清相关历史与派生对齐, 保留 IMU 本身.
    void invalidate_joint_reference(Domain domain) noexcept;
    [[nodiscard]] std::uint32_t alignment_epoch() const noexcept { return alignment_epoch_; }
    [[nodiscard]] bool alignment_valid() const noexcept { return alignment_valid_; }
    [[nodiscard]] float yaw_alignment_rad() const noexcept { return alignment_offset_rad_; }
    [[nodiscard]] std::uint32_t local_epoch() const noexcept { return local_epoch_; }
    [[nodiscard]] std::optional<float> local_direction_rad() const noexcept;
    [[nodiscard]] std::optional<float> yaw_direction_rad() const noexcept;
    void integrate_commanded_odom(const motion::PlanarTwist& adopted_twist, Tick at) noexcept;
    [[nodiscard]] const OdomPose& commanded_odom() const noexcept { return odom_; }
    void suspend_commanded_odom() noexcept { odom_.valid = false; odom_started_ = false; }
    void update_wheel_odom(const std::array<DomainFeedback, 4>& wheels,
        const std::array<std::optional<MicroTick>, 4>& times, Tick now, MicroTick now_us) noexcept;
    [[nodiscard]] const WheelOdometry& wheel_odom() const noexcept { return wheel_; }
private:
    [[nodiscard]] bool reset_attitude(const Vector3& accel_body) noexcept;
    void expire_samples(Tick now, MicroTick now_us) noexcept;
    void expire_joint_samples(Tick now, MicroTick now_us) noexcept;
    [[nodiscard]] std::optional<Vector3> heading_anchor(MicroTick now_us) noexcept;
    void reconstruct_platform(MicroTick now, const Quaternion& sensor_pose) noexcept;
    SentinelConfig config_{};
    ConfigReport report_{};
    estimation::HeadingTilt attitude_{};
    TiltFusionDiagnostics tilt_{};
    InertialFeedback inertial_{};
    PlatformFeedback platform_{};
    ImuStats stats_{};
    OdomPose odom_{};
    WheelOdometry wheel_{};
    std::optional<motion::OmniDrive> wheel_drive_{};
    std::array<Lifetime, 4> wheel_ttl_{};
    motion::PlanarTwist previous_wheel_twist_{};
    bool wheel_interval_started_{}, wheel_gap_{};
    std::array<JointHistory, 2> joint_history_{};
    std::array<std::uint32_t, 2> joint_epochs_{};
    std::array<Tick, 2> joint_sample_ms_{};
    Tick last_sample_at_{}, last_accel_at_{}, last_ingested_at_{}, reference_at_{}, odom_at_{};
    MicroTick gyro_us_{}, accel_us_{};
    std::uint32_t gyro_sequence_{}, accel_sequence_{}, local_epoch_{}, gyro_error_us_{0xffffffffu};
    Vector3 last_gyro_rad_s_{}, last_accel_mps2_{};
    bool primed_{}, accel_seen_{}, odom_started_{}, heading_started_{}, alignment_valid_{};
    bool gyro_gap_{}, accel_fresh_{}; // 已失效缓存不可因 MicroTick 绕完整一圈复活.
    float last_heading_{}, continuous_heading_{}, alignment_offset_rad_{};
    std::uint32_t alignment_epoch_{};
};
} // 命名空间 ec::sentinel::app
