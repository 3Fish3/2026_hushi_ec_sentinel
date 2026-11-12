#include "sentinel_app.hpp"
#include <cmath>

namespace ec::sentinel::app {
SentinelApp::SentinelApp(const SentinelConfig& config, ConfigAudience audience, StartupPolicy startup) noexcept
    : config_(config), report_(validate(config, audience)), authority_(startup) {
    for (std::size_t i = 0; i < dji_slot_count; ++i) {
        const auto& slot = config_.dji[i];
        if (slot.installed) { dji_index_[domain_index(slot.domain)] = i; }
    }
    for (std::size_t i = 0; i < dm_slot_count; ++i) {
        const auto& slot = config_.dm[i];
        if (slot.installed) { dm_index_[domain_index(slot.domain)] = i; }
    }
    config_epoch_ = config_.kind == ConfigKind::unconfigured ? 0u : 1u;
    reference_epoch_ = 0;
}

control::StopAction SentinelApp::stop_action_for(Domain domain) noexcept {
    // A08/A09 已选动作: 九颗 DJI 显式零电流, DM pitch 请求 disable.
    return domain == Domain::pitch ? control::StopAction::disable_output
                                   : control::StopAction::zero_output;
}

void SentinelApp::invalidate_domain(std::size_t i) noexcept {
    bool any = false;
    if (targets_[i]) {
        targets_[i].reset();
        any = true;
    }
    if (static_cast<Domain>(i) == Domain::main_yaw && degraded_hold_) {
        degraded_hold_.reset();
        any = true;
    }
    if (any) { ++target_invalidations_; }
}

void SentinelApp::invalidate_domain_reference(Domain domain) noexcept {
    const auto index = domain_index(domain);
    ++domain_reference_epoch_[index];
    ++reference_epoch_;
    invalidate_domain(index);
}

void SentinelApp::invalidate_inertial_reference() noexcept {
    const auto index = domain_index(Domain::main_yaw);
    ++domain_reference_epoch_[index];
    ++reference_epoch_;
    if (degraded_hold_ || (targets_[index] && targets_[index]->frame != YawFrame::joint)) {
        invalidate_domain(index);
    }
}

AcceptResult SentinelApp::accept_remote_targets(const TargetSnapshot& snapshot, Tick now,
    bool inertial_available) noexcept {
    if (last_source_ != Source::remote) { return accept_targets(snapshot, now, Source::remote); }
    if (inertial_available) {
        // 已有本周期的新测量与新操纵意图, 不因上一快照的质量位多拒绝一个周期.
        if (main_yaw_degraded_) { main_yaw_degraded_ = false; degraded_hold_.reset(); }
        return accept_targets(snapshot, now, Source::remote);
    }
    auto local = snapshot;
    const auto index = domain_index(Domain::main_yaw);
    const auto held = degraded_hold_;
    const auto previous = held ? held : targets_[index];
    const bool retain = previous && fresh(now, previous->accepted_at, previous->ttl);
    if (retain) {
        local.domains[index] = DomainSetpoint{previous->mode, previous->value, 0, previous->frame};
    } else { local.domains[index].reset(); }
    const auto result = accept_targets(local, now, Source::remote);
    if (result == AcceptResult::accepted && retain) {
        // 整包值校验后保留原意图对象的时刻/期限/身份, 不用短线协议 age 表达本地租约.
        if (held) {
            targets_[index].reset();
            degraded_hold_ = held; // G07 唯一持有, 不变成刷新生成的正常机械目标.
        } else { targets_[index] = previous; }
    }
    return result;
}

void SentinelApp::clear_targets() noexcept {
    bool any = false;
    for (auto& target : targets_) {
        if (target) {
            target.reset();
            any = true;
        }
    }
    // 降级保持属于旧目标/旧参考: 与目标一起失效, 不留下无期限的保持资格.
    degraded_hold_.reset();
    // 底盘 twist 也是目标: 来源/停止/代次变化后不得用旧 twist 重新解算.
    if (requested_twist_) {
        requested_twist_.reset();
        adopted_twist_.reset();
        twist_scale_ = 1.0f;
        any = true;
    }
    if (any) { ++target_invalidations_; }
}

StatusSnapshot SentinelApp::status(Tick now) const noexcept {
    StatusSnapshot out{};
    out.at = now;
    // 权限/自检/输入链路在没有 update 时没有来源: 保持"无来源、无授权、未就绪、未人工停止",
    // 不因为配置齐备就暗示已经取得执行资格.
    out.permission = PermissionState{};
    out.input = InputState{};
    out.config_epoch = config_epoch_;
    out.reference_epoch = reference_epoch_;
    for (std::size_t i = 0; i < domain_count; ++i) {
        auto& entry = out.domains[i];
        entry.domain = static_cast<Domain>(i);
        switch (report_.domains[i]) {
        case ConfigState::unavailable:
            entry.configured = false;
            entry.validity = DomainValidity::unavailable;
            entry.reason = Reason::not_configured;
            break;
        case ConfigState::incomplete:
            entry.configured = true;
            entry.validity = DomainValidity::incomplete;
            entry.reason = Reason::incomplete_config;
            break;
        case ConfigState::complete:
            // 配置齐备不等于现在可执行: 尚无设备反馈, 关节也尚未建立参考.
            entry.configured = true;
            entry.validity = DomainValidity::unknown;
            entry.reason = is_joint(entry.domain) ? Reason::no_reference : Reason::no_feedback;
            break;
        }
    }
    return out;
}

bool SentinelApp::set_joint_geometry(Domain domain, const JointGeometryConfig& joint) noexcept {
    if (!is_joint(domain)) { return false; }
    const bool finite = (!joint.zero_offset_rad || std::isfinite(*joint.zero_offset_rad)) &&
        (!joint.min_rad || std::isfinite(*joint.min_rad)) &&
        (!joint.max_rad || std::isfinite(*joint.max_rad));
    if (!finite || (joint.manual_continuous &&
        (domain == Domain::pitch || joint.min_rad || joint.max_rad))) { return false; }
    // 成对范围: 只改一个端点会让区间失效, 因此两边都给出时必须仍然 min < max.
    if (joint.min_rad && joint.max_rad && !(*joint.min_rad < *joint.max_rad)) { return false; }
    const auto old_zero = config_.joints[joint_index(domain)].zero_offset_rad;
    if (config_.serial && old_zero && joint.zero_offset_rad) {
        const auto delta = *old_zero-*joint.zero_offset_rad;
        if (domain == Domain::fire_yaw) { config_.serial->fire_reference += delta; }
        if (domain == Domain::pitch) { config_.serial->pitch_reference += delta; }
    }
    config_.joints[joint_index(domain)] = joint;
    // 参考语义改变: 同一数值目标指向不同物理角, 只作废该域目标与保持并上推该域代次,
    // 不顺手停掉其它域.
    invalidate_domain_reference(domain);
    return true;
}

bool SentinelApp::set_target_ttl(Lifetime ttl) noexcept {
    if (!ttl.valid()) { return false; }
    config_.timing.target_ttl = ttl;
    // 已存目标的到期时刻是按旧时限算出来的, 改时限后不再有意义: 一并作废.
    bump_reference_epoch();
    return true;
}

void SentinelApp::set_host_session(bool alive, bool request, std::uint32_t epoch) noexcept {
    host_session_alive_ = alive;
    host_session_request_ = request;
    if (epoch != 0) { host_session_epoch_ = epoch; }
}

void SentinelApp::set_maintenance_session(bool alive, bool request, std::uint32_t epoch) noexcept {
    maintenance_session_alive_ = alive;
    maintenance_session_request_ = request;
    if (epoch != 0) { maintenance_session_epoch_ = epoch; }
}

void SentinelApp::revoke_sessions() noexcept {
    host_session_request_ = false;
    host_session_alive_ = false;
    maintenance_session_request_ = false;
    maintenance_session_alive_ = false;
}

const char* SentinelApp::build_identity() const noexcept { return "sentinel_app"; }

AppStats SentinelApp::stats() const noexcept { return stats_; }

void SentinelApp::update(Tick now, const AppInputs& inputs) noexcept {
    snapshot_ = status(now);

    // 1. 代次: 参考重置/配置改动都作废已有目标. 参考变化不是真实转动, 配置改动也不是新意图.
    if (inputs.epochs.reference != reference_epoch_) {
        reference_epoch_ = inputs.epochs.reference;
        clear_targets();
    }
    if (inputs.epochs.config != config_epoch_) {
        config_epoch_ = inputs.epochs.config;
        clear_targets();
    }
    snapshot_.config_epoch = config_epoch_;
    snapshot_.reference_epoch = reference_epoch_;

    // 2. 来源仲裁. 来源/授权代次变化或人工停止都作废旧目标 (积存目标不得复活).
    // 业务协议设置的会话状态与调用方直接给出的链路观察合并; 协议只表达"上位机/维护
    // 会话是否存在并请求", 来源选择仍由拨杆位置决定.
    auto link = inputs.link;
    link.host_link_alive = link.host_link_alive || host_session_alive_;
    link.host_request = link.host_request || host_session_request_;
    link.host_session_epoch = host_session_epoch_ != 0 ? host_session_epoch_ : link.host_session_epoch;
    link.maintenance_link_alive = link.maintenance_link_alive || maintenance_session_alive_;
    link.maintenance_request = link.maintenance_request || maintenance_session_request_;
    link.maintenance_session_epoch = maintenance_session_epoch_ != 0 ? maintenance_session_epoch_
                                                                     : link.maintenance_session_epoch;
    const auto decision = authority_.update(link);
    const bool authority_changed =
        decision.source != last_source_ || !(decision.token == last_token_);
    if (authority_changed || decision.manual_stop) { clear_targets(); }
    last_source_ = decision.source;
    last_token_ = decision.token;
    snapshot_.permission.source = decision.source;
    snapshot_.permission.authority = authority_.holds();
    snapshot_.permission.manual_stop = decision.manual_stop;
    snapshot_.permission.locally_ready =
        inputs.link.locally_ready && authority_.running_state_reached();
    snapshot_.permission.generation = decision.token ? decision.token->generation : 0;
    snapshot_.input.remote_online = inputs.link.remote_online;
    snapshot_.input.host_link_alive = inputs.link.host_link_alive;
    snapshot_.input.remote_sampled_at = inputs.link.remote_sampled_at;
    snapshot_.input.host_sampled_at = inputs.link.host_sampled_at;
    snapshot_.inertial = inputs.inertial;
    snapshot_.main_yaw = inputs.main_yaw;
    for (std::size_t i = 0; i < domain_count; ++i) {
        auto& entry = snapshot_.domains[i];
        entry.feedback.position_rad = inputs.feedback[i].position_rad;
        entry.feedback.velocity_rad_s = inputs.feedback[i].velocity_rad_s;
        entry.feedback.current_a = inputs.feedback[i].current_a;
        // 反馈到位后该域才算"可用": 关节域还需要已建立参考. 故障/过期在第 3 步
        // 按 A09 覆盖成 invalid/stale; 这里只区分"从未收到反馈"与"已可用".
        const auto domain = static_cast<Domain>(i);
        const bool sample_ready = is_joint(domain)
            ? inputs.feedback[i].position_rad.validity == Validity::valid
            : inputs.feedback[i].velocity_rad_s.validity == Validity::valid;
        if (report_.domains[i] == ConfigState::complete && sample_ready &&
            !inputs.faults.feedback_lost[i] && !inputs.faults.reference_lost[i]) {
            entry.validity = DomainValidity::valid;
        }
    }
    snapshot_.aux = inputs.aux;
    // 发射许可: 协议/本地来源显式设置的许可与调用方注入的观察合并; 两者都是“缺失即不可用”.
    if (launch_permit_valid_) {
        snapshot_.aux.launch_permit_valid = true;
        snapshot_.aux.launch_permit = launch_permit_;
        snapshot_.aux.launch_sampled_at = launch_sampled_at_;
        snapshot_.aux.launch_ttl = launch_ttl_;
    }
    snapshot_.launch_allowed = snapshot_.aux.launch_allowed(now);
    ++stats_.cycles;
    if (remote_was_online_ && !inputs.link.remote_online) { ++stats_.link_losses; }
    remote_was_online_ = inputs.link.remote_online;

    // 3. 故障影响集合 (A09). 自动故障只撤相关输出, 绝不自动切入人工整车停止挡.
    auto faults = evaluate_faults(inputs.faults);
    if (config_.independent_motor_bench) {
        // 裸电机彼此无机械依赖; 不伪造健康状态, 只撤故障电机自身输出.
        faults.output_revoked = faults.faulted;
        faults.trigger_inhibited = false;
    }
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        auto& observation = snapshot_.domains[i];
        if (inputs.faults.device_fault[i]) {
            observation.fault = FaultState::device_fault;
        } else if (inputs.faults.reference_lost[i]) {
            observation.fault = FaultState::reference_lost;
        } else if (inputs.faults.feedback_lost[i]) {
            observation.fault = FaultState::feedback_lost;
        } else if (faults.output_revoked[i]) {
            observation.fault = FaultState::dependency_fault;
        } else if (faults.main_yaw_degraded && domain == Domain::main_yaw &&
            (!targets_[i] || targets_[i]->frame != YawFrame::joint)) {
            observation.fault = FaultState::degraded;
        }
        if (inputs.faults.device_fault[i] || inputs.faults.reference_lost[i]) {
            observation.validity = DomainValidity::invalid;
        } else if (inputs.faults.feedback_lost[i]) {
            // 曾经可用而当前超出时效: 与"反馈被判定无效"区分开, 便于客户端判断是
            // "链路/设备掉了"还是"数据本身不可信". 从未收到过样本的域保持 unknown:
            // 两者都不满足必需反馈, 但"从未见过"与"掉了"不能混为一谈.
            const bool had_sample = inputs.feedback[i].position_rad.validity != Validity::unknown ||
                inputs.feedback[i].velocity_rad_s.validity != Validity::unknown;
            if (had_sample) { observation.validity = DomainValidity::stale; }
        }
        // 撤输出、执行器重新上电或执行器拒绝上一周期目标后, 旧目标立即作废;
        // 恢复必须来自新的目标接受 (V01/R05).
        if (faults.output_revoked[i] || inputs.faults.device_restarted[i] ||
            inputs.faults.execution_rejected[i]) {
            invalidate_domain(i);
        }
    }

    // 4. 每域执行结论. 无目标/目标过期/授权丢失分别是不同的原因, 并各自给出撤输出动作.
    const Grant token = decision.token.value_or(Grant{});
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        auto& execution = snapshot_.executions[i];
        execution.at = now;
        execution.token = token;
        const auto stop = stop_action_for(domain);
        if (report_.domains[i] != ConfigState::complete) {
            execution.reason = report_.domains[i] == ConfigState::unavailable
                ? Reason::not_configured
                : Reason::incomplete_config;
            continue; // 没有设备可落实, 不产生撤输出动作
        }
        if (inputs.faults.device_restarted[i]) {
            execution.reason = Reason::restarted;
            execution.stop = stop;
            continue;
        }
        if (faults.output_revoked[i]) {
            execution.reason = Reason::fault_dependency;
            execution.stop = stop;
            continue;
        }
        if (inputs.faults.execution_rejected[i]) {
            // 执行器拒绝了上一周期目标: 旧意图已作废, 必须等新目标, 不自动重试.
            execution.reason = Reason::output_rejected;
            execution.stop = stop;
            continue;
        }
        // 目标在机械范围内无可达等价分支: 拒绝该目标 (不是设备故障), 旧目标随之作废.
        if (inputs.faults.target_unreachable[i] && targets_[i]) {
            invalidate_domain(i);
            execution.reason = Reason::invalid_target;
            execution.stop = stop;
            continue;
        }
        if (decision.manual_stop) {
            execution.reason = Reason::manual_stop;
            execution.stop = stop;
            continue;
        }
        if (!authority_.holds()) {
            execution.reason = decision.reason;
            execution.stop = stop;
            continue;
        }
        // 主 yaw 惯性降级 (G07): 仅惯性失效而编码器/参考仍有效时, 撤销惯性方向目标并保持
        // 进入降级瞬间捕获的机械角. 保持不是新的执行资格: 只有原本存在且仍有效的域意图
        // 才能进入保持, 且保持沿用原意图的到期时刻, 不随时间刷新.
        if (domain == Domain::main_yaw) {
            if (inputs.faults.main_yaw_inertial_only &&
                (!targets_[i] || targets_[i]->frame != YawFrame::joint)) {
                if (!main_yaw_degraded_) {
                    main_yaw_degraded_ = true;
                    degraded_hold_.reset();
                    const auto previous = targets_[i];
                    if (previous && previous->fresh_at(now) &&
                        inputs.feedback[i].position_rad.validity == Validity::valid) {
                        // 原意图的期限与进入瞬间的机械角一起固定下来; 保持沿用同一
                        // accepted_at/ttl, 因此到期判定与目标走同一套回绕安全比较.
                        degraded_hold_ = DomainTarget{control::ControlMode::position,
                            inputs.feedback[i].position_rad.value, previous->accepted_at,
                            previous->ttl, previous->grant};
                    }
                    if (targets_[i]) {
                        targets_[i].reset();
                        ++target_invalidations_;
                    }
                }
                execution.reason = Reason::degraded;
                if (degraded_hold_ && degraded_hold_->fresh_at(now)) {
                    execution.execute = true;
                    execution.mode = control::ControlMode::position;
                    execution.adopted_target = degraded_hold_->value;
                    execution.frame = YawFrame::joint;
                } else {
                    // 原意图不存在或已到期: 降级不创造输出, 按 A08 撤输出.
                    // reason 仍报 degraded (域处于降级保持态), 撤输出由 stop 动作表达.
                    execution.stop = stop;
                }
                continue;
            }
            if (main_yaw_degraded_) {
                main_yaw_degraded_ = false;
                degraded_hold_.reset();
                if (targets_[i] && targets_[i]->frame != YawFrame::joint) {
                    targets_[i].reset();
                    ++target_invalidations_;
                }
            }
        }
        const auto target = targets_[i];
        if (!target) {
            execution.reason = Reason::no_target;
            execution.stop = stop;
            continue;
        }
        if (!target->fresh_at(now)) {
            execution.reason = Reason::stale_target;
            execution.stop = stop;
            continue;
        }
        // 禁止正向拨弹不影响人工反转与零目标; 摩擦轮按独立请求继续.
        if (domain == Domain::trigger && target->value > 0.0f && !config_.independent_motor_bench) {
            if (faults.trigger_inhibited) {
                execution.reason = Reason::fault_dependency;
                execution.stop = stop;
                continue;
            }
            // 发射许可缺失/过期/未授权: 正向拨弹不取得执行资格 (A11).
            if (!snapshot_.launch_allowed) {
                execution.reason = Reason::not_permitted;
                execution.stop = stop;
                continue;
            }
        }
        execution.execute = true;
        execution.mode = target->mode;
        execution.adopted_target = target->value;
        execution.frame = target->frame;
    }
    // 本周期故障/时效判定完成后再发布原请求; 与采用值、参考代次同一快照.
    for (std::size_t i=0; i<domain_count; ++i) { snapshot_.domains[i].target = targets_[i]; }
    snapshot_.requested_twist = requested_twist_;
    snapshot_.adopted_twist = adopted_twist_;
    snapshot_.twist_scale = twist_scale_;
}

AcceptResult SentinelApp::accept_targets(const TargetSnapshot& snapshot, Tick now,
    Source requester) noexcept {
    if (authority_.manual_stop()) {
        ++rejected_packets_;
        return AcceptResult::manual_stop;
    }
    if (!authority_.holds() || !authority_.token()) {
        ++rejected_packets_;
        return AcceptResult::no_authority;
    }
    // “当前有人持权”不等于“本请求方有权写目标”: 换会话/换来源后旧会话不得借用当前 token.
    if (requester != Source::none && authority_.source() != requester) {
        ++rejected_packets_;
        return AcceptResult::no_authority;
    }
    const std::uint32_t ttl_ms = config_.timing.target_ttl.ms;
    if (ttl_ms == 0) {
        // 未配置目标时限: 无法给任何目标定出可验证的期限, 不执行.
        ++rejected_packets_;
        return AcceptResult::not_configured;
    }
    // 源年龄 → 接受时刻: 期限 = 接受时刻 + target_ttl, 因此剩余有效期随源年龄缩短.
    // 源年龄已达/超过时限的域视为“没有意图”, 不续期也不当作新意图.
    const auto accepted_at_for = [&](std::uint16_t age_ms) noexcept -> std::optional<Tick> {
        if (age_ms >= ttl_ms) { return std::nullopt; }
        return Tick{now.ms - age_ms};
    };
    // 底盘以整车 twist 下发: 不得同时给出单个底盘轮位姿, 否则意图冲突.
    bool chassis_setpoints = false;
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
        chassis_setpoints |= snapshot.domains[i].has_value();
    }
    if (snapshot.chassis && chassis_setpoints) {
        ++rejected_packets_;
        return AcceptResult::invalid_target;
    }
    std::optional<std::array<float, chassis_wheel_count>> wheels{};
    // 底盘采用值先算到局部量: 其它域校验失败时不能留下半提交的 twist 与缩放系数.
    std::optional<motion::PlanarTwist> chassis_requested{};
    std::optional<motion::PlanarTwist> chassis_adopted{};
    float chassis_scale = 1.0f;
    std::optional<Tick> chassis_accepted_at{};
    if (snapshot.chassis) {
        const auto twist = *snapshot.chassis;
        if (!std::isfinite(twist.vx_mps) || !std::isfinite(twist.vy_mps) ||
            !std::isfinite(twist.wz_rad_s)) {
            ++rejected_packets_;
            return AcceptResult::invalid_target;
        }
        chassis_accepted_at = accepted_at_for(snapshot.chassis_age_ms);
        const auto drive = omni_drive(config_);
        if (!drive) {
            ++rejected_packets_;
            return AcceptResult::not_configured;
        }
        const auto speeds = motion::omni_wheel_speeds(*drive, twist);
        if (!speeds) {
            ++rejected_packets_;
            return AcceptResult::not_configured;
        }
        // 四轮同比例缩放: 用四轮中最小的轴侧速度上限做统一系数, 保留平移/旋转比例,
        // 不独立夹单轮目标 (否则整机方向会静默改变).
        float limit = 0.0f;
        for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
            const auto slot = dji_index_[i];
            if (!slot) {
                ++rejected_packets_;
                return AcceptResult::not_configured;
            }
            const auto value = config_.dji[*slot].max_axis_velocity_rad_s.value_or(0.0f);
            limit = i == 0 ? value : (value < limit ? value : limit);
        }
        float peak = 0.0f;
        for (const auto speed : *speeds) {
            const float magnitude = std::fabs(speed);
            peak = magnitude > peak ? magnitude : peak;
        }
        const float scale = peak > limit ? limit / peak : 1.0f;
        if (!std::isfinite(scale) || scale <= 0.0f) {
            ++rejected_packets_;
            return AcceptResult::not_configured;
        }
        std::array<float, chassis_wheel_count> scaled{};
        for (std::size_t i = 0; i < chassis_wheel_count; ++i) { scaled[i] = (*speeds)[i] * scale; }
        wheels = scaled;
        chassis_scale = scale;
        chassis_requested = twist;
        // 采用 twist = 按同一系数缩放后的 twist, 用于回报实际采用的整车量.
        chassis_adopted = motion::PlanarTwist{twist.vx_mps * scale, twist.vy_mps * scale,
            twist.wz_rad_s * scale};
    }
    // 整包预检: 任一域不合法就整体拒绝, 不部分执行, 也不改变任何已有目标 (拒绝不续期).
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto& setpoint = snapshot.domains[i];
        if (!setpoint) { continue; }
        const auto domain = static_cast<Domain>(i);
        if (report_.domains[i] != ConfigState::complete) {
            ++rejected_packets_;
            return AcceptResult::not_configured;
        }
        // 模式—域白名单: 未实现的组合 (关节速度、速度域位置、current) 明确拒绝.
        if (!mode_allowed_for(domain, setpoint->mode) || !std::isfinite(setpoint->value) ||
            static_cast<unsigned>(setpoint->frame) > static_cast<unsigned>(YawFrame::odom) ||
            (domain != Domain::main_yaw && setpoint->frame != YawFrame::joint)) {
            ++rejected_packets_;
            return AcceptResult::invalid_target;
        }
        if (domain == Domain::main_yaw && setpoint->frame != YawFrame::joint && main_yaw_degraded_) {
            // 源年龄已达时限的 main 字段按"没有意图"处理: 它与省略该域等价, 进入统一域撤销
            // (降级保持一并失效), 而不是作为新的惯性方向目标被降级策略拒绝后保留旧保持.
            if (!accepted_at_for(setpoint->age_ms)) { continue; }
            // 惯性降级保持期间不接受新的惯性方向目标.
            ++rejected_packets_;
            return AcceptResult::degraded;
        }
        // 速度域的显式目标必须落在该轴自己的目标量程内: 越限是无效目标, 在应用边界
        // 拒绝而不是交给轴层稍后再拒 (后者会先接受再失败).
        if (setpoint->mode == control::ControlMode::velocity) {
            const auto slot = dji_index_[i];
            if (!slot) {
                ++rejected_packets_;
                return AcceptResult::not_configured;
            }
            const float limit = config_.dji[*slot].max_axis_velocity_rad_s.value_or(0.0f);
            if (!(std::fabs(setpoint->value) <= limit)) {
                ++rejected_packets_;
                return AcceptResult::limit_violation;
            }
        }
        // 惯性方向通过三维模型选择机械分支; 显式机械角 (含 main 维护模式) 越限整包拒绝.
        if (is_joint(domain) && setpoint->frame == YawFrame::joint &&
            setpoint->mode == control::ControlMode::position) {
            const auto& joint = config_.joints[joint_index(domain)];
            if (joint.min_rad && joint.max_rad &&
                (setpoint->value < *joint.min_rad || setpoint->value > *joint.max_rad)) {
                ++rejected_packets_;
                return AcceptResult::limit_violation;
            }
        }
    }
    // 全部校验通过: 统一提交, 提交阶段不再有拒绝分支.
    const auto token = *authority_.token();
    if (wheels && chassis_accepted_at) {
        for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
            targets_[i] = DomainTarget{control::ControlMode::velocity, (*wheels)[i],
                *chassis_accepted_at, config_.timing.target_ttl, token};
        }
        twist_scale_ = chassis_scale;
        requested_twist_ = chassis_requested;
        adopted_twist_ = chassis_adopted;
    } else if (!snapshot.chassis) {
        // 没有 twist 时, 底盘只接受逐轮速度目标 (单一来源, 不混合).
        for (std::size_t i = 0; i < chassis_wheel_count; ++i) {
            const auto& setpoint = snapshot.domains[i];
            const auto accepted_at = setpoint ? accepted_at_for(setpoint->age_ms) : std::nullopt;
            if (setpoint && accepted_at) {
                targets_[i] = DomainTarget{setpoint->mode, setpoint->value, *accepted_at,
                    config_.timing.target_ttl, token};
            } else {
                invalidate_domain(i);
            }
        }
        requested_twist_.reset();
        adopted_twist_.reset();
        twist_scale_ = 1.0f;
    } else {
        // 带 twist 但源年龄已达时限: 视为没有底盘意图, 撤掉四轮并清掉命令派生状态.
        for (std::size_t i = 0; i < chassis_wheel_count; ++i) { invalidate_domain(i); }
        requested_twist_.reset();
        adopted_twist_.reset();
        twist_scale_ = 1.0f;
    }
    for (std::size_t i = chassis_wheel_count; i < domain_count; ++i) {
        const auto& setpoint = snapshot.domains[i];
        const auto accepted_at = setpoint ? accepted_at_for(setpoint->age_ms) : std::nullopt;
        if (!setpoint || !accepted_at) {
            invalidate_domain(i);
            continue;
        }
        targets_[i] = DomainTarget{setpoint->mode, setpoint->value, *accepted_at,
            config_.timing.target_ttl, token, setpoint->frame};
        if (static_cast<Domain>(i) == Domain::main_yaw && setpoint->frame == YawFrame::joint) {
            degraded_hold_.reset();
        }
    }
    ++accepted_packets_;
    return AcceptResult::accepted;
}
} // 命名空间 ec::sentinel::app
