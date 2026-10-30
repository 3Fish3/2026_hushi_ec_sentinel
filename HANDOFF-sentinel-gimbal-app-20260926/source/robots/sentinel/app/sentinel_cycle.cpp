#include "sentinel_cycle.hpp"
#include <algorithm>
#include <cmath>

namespace ec::sentinel::app {
SentinelCycle::SentinelCycle(SentinelApp& app, SentinelExecutor& executor,
    SentinelPerception& perception, RemoteMappingConfig mapping, Lifetime remote_ttl) noexcept
    : app_(app), executor_(executor), perception_(perception), remote_(remote_ttl),
      mapper_(mapping, remote_ttl) {
    reference_epochs_ = {app_.domain_reference_epoch(Domain::main_yaw),
        app_.domain_reference_epoch(Domain::fire_yaw),
        app_.domain_reference_epoch(Domain::pitch)};
}

bool SentinelCycle::accept_remote(const CycleInputs& inputs) noexcept {
    if (!inputs.remote_sample) { return false; }
    // 槽位语义是"最近一份样本", 控制任务可能连续多个周期读到同一份:
    // 按到达时刻去重, 否则会重复计数并反复重建连续基准 (边沿判定因此失效).
    // 毫秒时间戳按 2^32 回绕: 相同时间戳是重复, 半个回绕窗口内的倒退才是真回退,
    // 不能用普通 `<` 把回绕后的前进误判成"不更新".
    if (has_remote_) {
        const std::uint32_t gap = inputs.remote_received_at.ms - last_remote_at_.ms;
        if (gap == 0u || gap >= 0x80000000u) { return false; }
    }
    if (!remote_.accept(*inputs.remote_sample, inputs.remote_received_at)) { return false; }
    has_remote_ = true;
    last_remote_at_ = inputs.remote_received_at;
    return true;
}

void SentinelCycle::refresh_feedback(Tick now) noexcept {
    for (auto& entry : inputs_.feedback) { entry = DomainFeedback{}; }
    for (std::size_t i = 0; i < dji_slot_count; ++i) {
        if (!app_.config().dji[i].installed) { continue; }
        auto* device = executor_.dji_device(i);
        if (device == nullptr) { continue; }
        const auto domain = app_.config().dji[i].domain;
        // 与控制器共用有效关节参考及选定测速, 不回报另一份原生RPM或失效旧坐标.
        inputs_.feedback[domain_index(domain)] = executor_.joint_feedback(domain, now);
    }
    if (app_.config().dm[0].installed) {
        // DM 域: 与健康判定/Mapper 共用同一个关节视图 (已减零位偏移),
        // 不能向应用回传设备原生角 —— 非零零位时两者不是同一个坐标.
        const auto domain = app_.config().dm[0].domain;
        inputs_.feedback[domain_index(domain)] = executor_.joint_feedback(domain, now);
    }
}

CycleOutcome SentinelCycle::control(const CycleInputs& inputs) noexcept {
    outcome_ = CycleOutcome{};
    outcome_.remote_frame_accepted = accept_remote(inputs);

    // 感知先消费真实关节视图; health 建立唯一可恢复的机械参考, 不提供假零位.
    inputs_.faults = executor_.health(inputs.now);
    for (const auto domain : {Domain::fire_yaw, Domain::pitch}) {
        const auto time = executor_.feedback_time(domain);
        if (time) {
            perception_.submit_joint(domain, executor_.joint_feedback(domain, inputs.now), *time,
                app_.domain_reference_epoch(domain));
        }
    }
    if (inputs.imu_gyro_rad_s && inputs.imu_accel_mps2) {
        if (inputs.imu_times) {
            perception_.submit_timed_imu(*inputs.imu_gyro_rad_s, *inputs.imu_accel_mps2, *inputs.imu_times,
                inputs.now, inputs.now_us.value_or(MicroTick{inputs.now.ms*1000u}));
        } else {
            perception_.submit_imu(*inputs.imu_gyro_rad_s, *inputs.imu_accel_mps2, inputs.imu_sampled_at, inputs.now);
        }
        outcome_.imu_submitted = true;
    }
    perception_.update(inputs.now, inputs.now_us);
    if (local_epoch_ != perception_.local_epoch()) {
        if (local_epoch_ != 0) { app_.invalidate_inertial_reference(); }
        local_epoch_ = perception_.local_epoch();
    }

    // 1. 操纵映射: 缺省样本 (失联/从未收到) 会让映射器清掉旧摩擦与拨弹意图.
    //    实际关节角来自执行层的关节视图 (减零位后的同一坐标), 不用设备原生位置.
    std::array<std::optional<float>, domain_count> joint_rad{};
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        if (!is_joint(domain)) { continue; }
        const auto feedback = executor_.joint_feedback(domain, inputs.now);
        if (feedback.position_rad.validity == Validity::valid) {
            joint_rad[i] = feedback.position_rad.value;
        }
    }
    const auto main_index = domain_index(Domain::main_yaw);
    const auto main_joint = joint_rad[main_index];
    // 主 yaw 本地操纵捕获局部惯性方向, 其它两个域仍是机械关节角.
    joint_rad[main_index] = mapper_.config().layout == RemoteLayout::gimbal_validation
        ? main_joint : (main_joint ? perception_.local_direction_rad() : std::nullopt);
    if (!joint_rad[main_index]) { mapper_.drop_held_angle(Domain::main_yaw); }
    const auto snapshot = remote_.snapshot(inputs.now);
    outcome_.remote_online = snapshot.has_value();
    // 参考语义改变 (零位/范围/对齐) 后, 该关节已保持的本地目标角属于旧坐标: 丢弃它,
    // 下次接管时按新的实际姿态重新建立, 不用旧数值继续驱动.
    const auto drop_stale_hold = [&](Domain domain) {
        const auto slot = joint_index(domain);
        const auto epoch = app_.domain_reference_epoch(domain);
        if (reference_epochs_[slot] == epoch) { return; }
        reference_epochs_[slot] = epoch;
        mapper_.drop_held_angle(domain);
    };
    drop_stale_hold(Domain::main_yaw);
    drop_stale_hold(Domain::fire_yaw);
    drop_stale_hold(Domain::pitch);
    // 调参切遥控时继承仍在执行的机械角目标, 不把重力保持误差清成零.
    // 仅此角色使用机械参考; 未执行、反馈无效或惯性参考目标仍按实际姿态初始化.
    if (mapper_.config().layout == RemoteLayout::gimbal_validation && snapshot &&
        snapshot->switch_left == drivers::Dr16Switch::up &&
        snapshot->switch_right != drivers::Dr16Switch::down &&
        app_.snapshot().permission.source == Source::maintenance) {
        for (const auto domain : {Domain::main_yaw, Domain::fire_yaw, Domain::pitch}) {
            const auto i = domain_index(domain);
            const auto& execution = app_.snapshot().executions[i];
            const auto old_target = app_.target(domain);
            if (joint_rad[i] && execution.execute && execution.frame == YawFrame::joint &&
                old_target && old_target->fresh_at(inputs.now) &&
                execution.adopted_target && std::isfinite(*execution.adopted_target)) {
                joint_rad[i] = execution.adopted_target;
            }
        }
    }
    const auto previous_heading = mapper_.held_angle(Domain::main_yaw);
    auto command = mapper_.step(snapshot, inputs.now, inputs.dt_s, joint_rad);
    for (const auto domain : {Domain::fire_yaw, Domain::pitch}) {
        auto& target = command.targets.domains[domain_index(domain)];
        const auto& range = app_.joint_geometry(domain);
        if (target && range.min_rad && range.max_rad) {
            // 目标优先夹到操作限幅(若配置); 机械止点仍只用于反馈越界保护, 不放宽目标.
            const float lo = range.command_min_rad.value_or(*range.min_rad);
            const float hi = range.command_max_rad.value_or(*range.max_rad);
            target->value = std::clamp(target->value, lo, hi);
            mapper_.limit_held_angle(domain, target->value);
        }
    }
    auto& main_target = command.targets.domains[main_index];
    const auto& platform = perception_.platform();
    if (main_target && main_joint && app_.config().serial) {
        const auto& range = app_.joint_geometry(Domain::main_yaw);
        const auto reachable = [&](float heading) {
            return range.min_rad && range.max_rad && reachable_heading(*app_.config().serial,
                platform.value.orientation, *main_joint, heading, *range.min_rad, *range.max_rad).has_value();
        };
        if (!reachable(main_target->value)) {
            main_target->value = previous_heading && reachable(*previous_heading) ? *previous_heading : platform.continuous_heading;
            mapper_.limit_held_angle(Domain::main_yaw, main_target->value);
        }
    }
    outcome_.intent = command.intent;
    outcome_.manual_stop = command.manual_stop;
    outcome_.layer = command.layer;
    outcome_.chassis_active = command.chassis_active;
    outcome_.friction_on = command.friction_on;

    // 3. 输入视图: 反馈/惯性/链路/外部约束都来自真实来源; 缺失保持 unknown/unavailable.
    inputs_.inertial = perception_.inertial();
    inputs_.main_yaw = perception_.platform();
    inputs_.faults.main_yaw_inertial_only = inputs_.main_yaw.validity != DomainValidity::valid;
    inputs_.aux = inputs.aux;
    inputs_.epochs = Epochs{app_.config_epoch(), app_.reference_epoch()};
    refresh_feedback(inputs.now);
    std::array<DomainFeedback, 4> wheels{};
    std::array<std::optional<MicroTick>, 4> wheel_times{};
    for (std::size_t i=0; i<4; ++i) {
        wheels[i] = inputs_.feedback[i];
        wheel_times[i] = executor_.feedback_time(static_cast<Domain>(i));
    }
    perception_.update_wheel_odom(wheels, wheel_times, inputs.now,
        inputs.now_us.value_or(MicroTick{inputs.now.ms*1000u}));
    inputs_.link = LinkInputs{};
    inputs_.link.remote_layout = mapper_.config().layout;
    inputs_.link.remote_online = outcome_.remote_online;
    if (snapshot) {
        inputs_.link.switch_left = snapshot->switch_left;
        inputs_.link.switch_right = snapshot->switch_right;
        inputs_.link.remote_sampled_at = snapshot->sampled_at;
    }
    inputs_.link.host_link_alive = inputs.host_link_alive;
    inputs_.link.host_request = inputs.host_request;
    inputs_.link.host_session_epoch = inputs.host_session_epoch;
    inputs_.link.maintenance_link_alive = inputs.maintenance_link_alive;
    inputs_.link.maintenance_request = inputs.maintenance_request;
    inputs_.link.maintenance_session_epoch = inputs.maintenance_session_epoch;
    inputs_.link.locally_ready = inputs.locally_ready;

    // 4. 本地来源的目标经同一入口交给应用. 必须在 update 之前: 执行层读的是 update
    //    产出的执行判定, 目标晚一个周期才生效会让"已接受"与"本周期实际下发"错位.
    //    停止态/非本地来源不下发本地目标 (来源资格仍由应用仲裁判定).
    if (command.intent == SourceIntent::local && !command.manual_stop) {
        if (mapper_.config().layout == RemoteLayout::gimbal_validation &&
            app_.authority().source() != Source::remote) {
            // 先使新遥控授权生效, 再以新 token 接受本周期目标. 两次评估之间不提交 CAN,
            // 因此切档不会先发送一次 disable; 旧维护目标仍按原规则作废.
            app_.update(inputs.now, inputs_);
        }
        outcome_.accept_attempted = true;
        outcome_.accept = app_.accept_remote_targets(command.targets, inputs.now,
            inputs_.main_yaw.validity == DomainValidity::valid ||
                mapper_.config().layout == RemoteLayout::gimbal_validation);
    }

    // 5. 应用仲裁: 来源选择/人工停止/故障影响/目标生命周期都在应用层完成.
    app_.update(inputs.now, inputs_);
    return outcome_;
}

void SentinelCycle::commit(const CycleInputs& inputs) noexcept {
    // 执行提交: 真实设备编解码与 CAN 组帧都在执行层, 编排只负责调用顺序.
    executor_.step(inputs.now, inputs.dt_s, app_, inputs.now_us);
    // 命令预测 (不是轮式反馈里程计): 用应用实际采用的 twist 积分.
    const auto& execution = app_.snapshot().executions;
    const bool executing_chassis = std::all_of(execution.begin(), execution.begin()+4,
        [](const auto& e) { return e.execute && e.mode == control::ControlMode::velocity; });
    if (const auto twist = app_.adopted_twist(); twist && executing_chassis) {
        perception_.integrate_commanded_odom(*twist, inputs.now);
    } else { perception_.suspend_commanded_odom(); }
}
} // 命名空间 ec::sentinel::app
