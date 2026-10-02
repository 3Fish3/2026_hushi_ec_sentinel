// 主机离线回放入口. 文本由 sentinel_replay.py 从 .sraw/标定候选生成; 不链接硬件后端.
#include "sentinel_perception.hpp"
#include <cstdint>
#include <iostream>
#include <string>

using namespace ec::sentinel::app;
int main() {
    std::string tag;
    if (!(std::cin >> tag)) { return 2; }
    const bool rate_only = tag == "SERIAL_RATE1";
    if (!rate_only && tag != "SERIAL1") { return 2; }
    auto config = synthetic_config();
    auto& model = *config.serial;
    for (auto* v : {&model.fire_axis, &model.pitch_axis, &model.main_axis, &model.forward, &model.gyro_bias_sensor}) {
        for (auto& x : *v) { std::cin >> x; }
    }
    std::cin >> model.fire_reference >> model.pitch_reference;
    for (auto& lag : model.position_lag_us) { std::cin >> lag; }
    for (auto& lag : model.velocity_lag_us) { std::cin >> lag; }
    std::cin >> model.min_projection_squared >> model.min_heading_gain >> model.accel_norm_error_mps2;
    if (rate_only) {
        // 降频记录的离线插值预算必须显式提供, 不改生产默认值或普通SERIAL1行为.
        std::uint32_t window{};
        std::cin >> window;
        if (!window || window > config.perception.joint_age.us) { return 3; }
        config.perception.extrapolation = ec::MicroLifetime{window};
    }
    if (!std::cin || !model.valid()) { return 3; }
    // 其它合成设备值不参与回放, 这里只构造原生感知模块, 不构造执行器.
    config.serial_origin = Origin::reference_converted;
    SentinelPerception perception{config, ConfigAudience::offline_test};
    ec::drivers::ImuSamplingTimes times;
    Vector3 gyro{}, accel{};
    bool started = false;
    ec::MicroTick previous{};
    std::uint64_t extended{};
    const auto flush = [&]() {
        const ec::Tick ms{static_cast<std::uint32_t>(extended/1000)};
        if (times.gyro_updated || times.accel_updated) {
            perception.submit_timed_imu(gyro, accel, times, ms, previous);
        }
        perception.update(ms, previous);
        times.gyro_updated = times.accel_updated = false;
        const auto& out = perception.platform();
        std::cout << previous.us << ',' << static_cast<unsigned>(out.validity) << ',';
        // 缺少实物朝前参考时只检查生产链重构角速度, 不发布任意测试轴的heading.
        if (rate_only) {
            std::cout << out.value.rate_V[0] << ',' << out.value.rate_V[1] << ',' << out.value.rate_V[2];
        } else {
            std::cout << out.continuous_heading << ',' << out.value.heading_rate << ',' << out.value.heading_gain;
        }
        std::cout << ',' << out.ages_us[0] << ',' << out.ages_us[1] << ','
            << out.ages_us[2] << ',' << out.local_epoch << '\n';
    };
    std::cout << (rate_only
        ? "evaluated_us,validity,rate_x_V_rad_s,rate_y_V_rad_s,rate_z_V_rad_s,gyro_age_us,fire_age_us,pitch_age_us,local_epoch\n"
        : "evaluated_us,validity,heading_rad,heading_rate_rad_s,heading_gain,gyro_age_us,fire_age_us,pitch_age_us,local_epoch\n");
    while (std::cin >> tag && tag != "end") {
        std::uint32_t sample{}, arrival{}, sequence{};
        if (!(std::cin >> sample >> arrival >> sequence)) { return 4; }
        if (started && arrival != previous.us) {
            if (ec::micro_regressed({arrival}, previous)) { return 5; }
            flush();
            extended += arrival-previous.us;
        } else if (!started) { extended = arrival; }
        previous = {arrival}; started = true;
        if (tag == "gyro" || tag == "accel") {
            auto& v = tag == "gyro" ? gyro : accel;
            for (auto& x : v) { std::cin >> x; }
            if (tag == "gyro") {
                times.gyro_estimated = {sample}; times.gyro = {arrival}; times.gyro_sequence = sequence;
                times.gyro_valid = times.gyro_updated = true;
            } else {
                times.accel = {sample}; times.accel_sequence = sequence; times.accel_valid = times.accel_updated = true;
            }
        } else if (tag == "fire" || tag == "pitch") {
            DomainFeedback feedback;
            std::cin >> feedback.position_rad.value >> feedback.velocity_rad_s.value;
            feedback.position_rad.validity = feedback.velocity_rad_s.validity = ec::Validity::valid;
            const ec::Tick at{static_cast<std::uint32_t>((extended-(arrival-sample))/1000)};
            feedback.position_rad.sampled_at = feedback.velocity_rad_s.sampled_at = at;
            perception.submit_joint(tag == "fire" ? Domain::fire_yaw : Domain::pitch, feedback, {sample}, 0);
        } else { return 6; }
        if (!std::cin) { return 7; }
    }
    if (started) { flush(); }
    return 0;
}
