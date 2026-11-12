#pragma once
// 后续运行接口: 参考系、分组调度与采样质量分开表达, 不引入第二套权限状态机.
#include "ec/time.hpp"
#include <cstdint>
#include <optional>

namespace ec::sentinel::app {
enum class YawFrame : std::uint8_t { joint = 0, local_inertial, odom };

struct GroupTiming {
    std::uint32_t period_us{};
    std::uint32_t phase_us{};
    MicroLifetime output_ttl{};
    std::uint32_t max_step_us{};
    [[nodiscard]] constexpr bool valid() const noexcept {
        return period_us > 0 && period_us < 0x80000000u && phase_us < period_us &&
            output_ttl.valid() && max_step_us >= period_us && max_step_us < 0x80000000u;
    }
};

// 策略候选不表示电调已保存相同反馈率; 构造时可覆盖, 不能运行中静默变频.
struct RuntimeTiming {
    GroupTiming gm{1000, 0, {2000}, 5000};
    GroupTiming chassis{2000, 0, {3000}, 6000};
    GroupTiming shooter{2000, 1000, {3000}, 6000};
    GroupTiming pitch{2000, 0, {3000}, 6000};
    std::uint32_t direction_period_us{2000};
    std::uint32_t direction_max_step_us{10000};
};

struct PerceptionTiming {
    // 必须按采样误差预算提供, 不继承控制器 max_dt 或设备在线 TTL.
    MicroLifetime gyro_age{};
    MicroLifetime joint_age{};
    MicroLifetime extrapolation{};
    std::uint32_t max_gyro_step_us{};
};

struct ScheduledStep {
    bool due{};
    bool invalid_time{};
    float dt_s{};
};

// 一个发送组一个时钟. force 仅用于当前撤输出/来源变化, 不补发旧周期.
class GroupClock {
public:
    [[nodiscard]] ScheduledStep poll(MicroTick now, GroupTiming config, bool force = false,
        std::optional<MicroTick> cadence = std::nullopt) noexcept {
        if (!config.valid()) { return {false, true, 0.0f}; }
        // 板级 cadence 来自基础任务 Tick, dt 来自真实微秒; SPI 抖动不能让 2 ms 组变成 3 ms.
        const auto schedule = cadence.value_or(now);
        if (!started_) {
            started_ = true;
            anchor_ = schedule;
            next_ = {schedule.us + config.phase_us};
        }
        if (micro_regressed(schedule, anchor_) || (computed_ && micro_regressed(now, last_))) {
            return {false, true, 0.0f};
        }
        anchor_ = schedule;
        const bool due = schedule.us - next_.us < 0x80000000u;
        if (!force && !due) { return {}; }
        const auto elapsed = computed_ ? now.us - last_.us : config.period_us;
        if (elapsed == 0) { return {}; }
        if (due) {
            next_.us += ((schedule.us - next_.us) / config.period_us + 1u) * config.period_us;
        } else {
            next_.us = schedule.us + config.period_us;
        }
        computed_ = true;
        last_ = now;
        return {true, elapsed > config.max_step_us, static_cast<float>(elapsed) * 1e-6f};
    }
private:
    MicroTick anchor_{};
    MicroTick next_{};
    MicroTick last_{};
    bool started_{};
    bool computed_{};
};
} // 命名空间 ec::sentinel::app
