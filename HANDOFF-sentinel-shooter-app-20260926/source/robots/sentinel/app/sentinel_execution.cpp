#include "sentinel_execution.hpp"
#include "ec/control/angle.hpp"
#include "ec/drivers/bus_layout.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <limits>

namespace ec::sentinel::app {
namespace {
constexpr float tau_rad = 2.0f * std::numbers::pi_v<float>;
constexpr double tau_rad_d = 2.0 * std::numbers::pi_v<double>;
constexpr std::size_t group_capacity = 8;
// 撤输出期间 disable 重试的最小间隔 (ms): 有界退避, 不用"历史失败次数"永久阻断停止.
constexpr std::uint32_t pitch_disable_retry_ms = 100;

[[nodiscard]] CommandSource command_source_of(Source source) noexcept {
    switch (source) {
    case Source::remote: return CommandSource::remote;
    case Source::host: return CommandSource::host;
    case Source::maintenance: return CommandSource::maintenance;
    case Source::none: break;
    }
    return CommandSource::none;
}

// A08: 缺省撤输出动作由 SentinelApp::stop_action_for 决定, 这里只落实.
} // 内部辅助

std::optional<float> recover_single_turn_joint(float min_rad, float max_rad,
    float zero_offset_rad, float axis_single_turn_rad, float alias_period_rad) noexcept {
    if (!std::isfinite(min_rad) || !std::isfinite(max_rad) || !(min_rad < max_rad)) {
        return std::nullopt;
    }
    if (!std::isfinite(zero_offset_rad) || !std::isfinite(axis_single_turn_rad)) {
        return std::nullopt;
    }
    if (!std::isfinite(alias_period_rad) || alias_period_rad <= 0.0f) { return std::nullopt; }
    // 范围宽度必须小于混叠周期, 否则同一单圈读数会对应多个关节角, 不能唯一恢复.
    if (max_rad - min_rad >= alias_period_rad) { return std::nullopt; }
    // 同余关系必须在**原始差值 + 真实 alias 周期**上直接取模: 先减 2π 只有在 2π 恰为
    // alias 的整数倍时才保持同余 (即整数传动比), 任意正传动比下会制造出假的唯一解.
    const double period = static_cast<double>(alias_period_rad);
    double residue = std::fmod(static_cast<double>(axis_single_turn_rad) -
            static_cast<double>(zero_offset_rad),
        period);
    if (residue < 0.0) { residue += period; }
    // 范围宽度 < period 时格点至多一个: 直接解析, 不循环大数.
    const double first = std::ceil((static_cast<double>(min_rad) - residue) / period);
    const double last = std::floor((static_cast<double>(max_rad) - residue) / period);
    if (first > last) { return std::nullopt; }
    const float candidate = static_cast<float>(residue + first * period);
    if (!std::isfinite(candidate) || candidate < min_rad || candidate > max_rad) {
        return std::nullopt;
    }
    return candidate;
}

std::optional<float> select_equivalent_branch(
    float min_rad, float max_rad, float current_rad, float requested_rad) noexcept {
    if (!std::isfinite(min_rad) || !std::isfinite(max_rad) || !std::isfinite(current_rad) ||
        !std::isfinite(requested_rad) || !(min_rad < max_rad)) {
        return std::nullopt;
    }
    // 解析候选区间 + 最接近当前位置的那一个 (及其左右邻居, 共至多三个候选).
    // 不逐一加 1 遍历: 大数下 k + 1 == k 会让循环永不前进. 不设任意圈数上限:
    // 合法的 2π 等价方向不能因为一个与契约无关的常量被改成不可达.
    const double lo = std::ceil((static_cast<double>(min_rad) - requested_rad) / tau_rad_d);
    const double hi = std::floor((static_cast<double>(max_rad) - requested_rad) / tau_rad_d);
    if (lo > hi) { return std::nullopt; }
    const double nearest = std::round((static_cast<double>(current_rad) - requested_rad) / tau_rad_d);
    const double clamped = std::min(std::max(nearest, lo), hi);
    std::optional<float> best{};
    for (int delta = -1; delta <= 1; ++delta) {
        const double k = clamped + static_cast<double>(delta);
        if (k < lo || k > hi) { continue; }
        const float candidate = static_cast<float>(requested_rad + k * tau_rad_d);
        if (!std::isfinite(candidate) || candidate < min_rad || candidate > max_rad) { continue; }
        if (!best || std::fabs(candidate - current_rad) < std::fabs(*best - current_rad)) {
            best = candidate;
        }
    }
    return best;
}

SentinelExecutor::SentinelExecutor(const SentinelConfig& config, ConfigAudience audience,
    transport::CanTx& bus1, transport::CanTx& bus2, MicroTick (*clock)() noexcept) noexcept
    : config_(config), report_(validate(config, audience)), bus1_(&bus1), bus2_(&bus2), clock_(clock) {
    if (!report_.startable) { return; }
    // 只把真正安装的槽放进布局检查: 合法部分装配不能被"未安装"占位项判为非法.
    std::array<drivers::DjiDeviceConfig, dji_slot_count> layout{};
    std::size_t layout_size = 0;
    std::array<drivers::DmDeviceConfig, dm_slot_count> dm_layout{};
    std::size_t dm_layout_size = 0;
    for (std::size_t i = 0; i < dji_slot_count; ++i) {
        const auto& slot = config_.dji[i];
        if (!slot.installed) { continue; }
        const auto protocol = drivers::dji_protocol(slot.mode);
        if (!protocol || slot.bus == 0 || slot.bus > 2 || slot.device_id == 0 ||
            slot.device_id > protocol->count || runtime_[domain_index(slot.domain)]) { return; }
        // 地址先于运动参数核对; 默认零字段只用于地址检查, 绝不拿来创建虚构 device.
        drivers::DjiDeviceConfig address{};
        address.bus = slot.bus; address.mode = slot.mode; address.device_id = slot.device_id;
        layout[layout_size++] = address;
        if (const auto device_config = app::device_config(slot)) {
            dji_devices_[i].emplace(*device_config);
            if (!dji_devices_[i]->configured()) { return; }
        }
        const unsigned group = (slot.device_id - 1) / 4 + 1;
        const std::uint16_t tx_id = group == 1 ? protocol->first_tx : protocol->second_tx;
        std::size_t group_slot = group_capacity;
        for (std::size_t g = 0; g < group_count_; ++g) {
            if (groups_[g].bus == slot.bus && groups_[g].tx_id == tx_id) { group_slot = g; }
        }
        if (group_slot == group_capacity) {
            if (group_count_ >= group_capacity) { return; }
            group_slot = group_count_++;
            groups_[group_slot].bus = slot.bus;
            groups_[group_slot].tx_id = tx_id;
            groups_[group_slot].group = group;
            groups_[group_slot].stats.bus = slot.bus;
            groups_[group_slot].stats.id = tx_id;
            groups_[group_slot].timing = is_joint(slot.domain) ? config_.runtime.gm
                : is_chassis(slot.domain) ? config_.runtime.chassis : config_.runtime.shooter;
            if (!groups_[group_slot].timing.valid()) { return; }
        }
        const std::size_t in_group = (slot.device_id - 1) % 4;
        if (groups_[group_slot].indices[in_group]) { return; } // 同槽重复: 配置校验应已拒绝
        groups_[group_slot].indices[in_group] = i;

        auto& runtime = runtime_[domain_index(slot.domain)];
        runtime = DomainRuntime{i, group_slot, in_group, dji_devices_[i] ? &*dji_devices_[i] : nullptr};
        if (!dji_devices_[i]) {
            groups_[group_slot].stats.stop_only = true;
            groups_[group_slot].zero_frame = drivers::encode_dji(slot.mode, group, {});
            if (!groups_[group_slot].zero_frame) { return; }
            continue;
        }
        // 原始设备与控制器分开: 已知设备仍占真实槽位, 未配完本轴控制/几何时明确发零.
        if (report_.domains[domain_index(slot.domain)] != ConfigState::complete) { continue; }
        if (is_joint(slot.domain)) {
            const auto& joint = config_.joints[joint_index(slot.domain)];
            if (joint.zero_offset_rad && (joint.manual_continuous || joint.commissioning_unbounded || (joint.min_rad && joint.max_rad))) {
                axes_[domain_index(slot.domain)].emplace(motion::DjiAxisConfig{
                    slot.current_limit_a.value_or(0.0f),
                    slot.velocity_pid.value_or(control::PidConfig{}),
                    slot.position_pid.value_or(control::PidConfig{}),
                    control::ArcMode::continuous,
                    config_.timing.target_ttl,
                    slot.max_axis_velocity_rad_s.value_or(0.0f),
                    static_cast<float>(groups_[group_slot].timing.max_step_us) * 1e-6f,
                    0.0f}, *dji_devices_[i]);
                if (!axes_[domain_index(slot.domain)]->configured()) { return; }
            }
        } else {
            axes_[domain_index(slot.domain)].emplace(motion::DjiAxisConfig{
                slot.current_limit_a.value_or(0.0f),
                slot.velocity_pid.value_or(control::PidConfig{}),
                control::PidConfig{},
                control::ArcMode::continuous,
                config_.timing.target_ttl,
                slot.max_axis_velocity_rad_s.value_or(0.0f),
                static_cast<float>(groups_[group_slot].timing.max_step_us) * 1e-6f,
                0.0f}, *dji_devices_[i]);
            if (!axes_[domain_index(slot.domain)]->configured()) { return; }
        }
    }
    if (layout_size != 0 && !drivers::valid_dji_addresses({layout.data(), layout_size})) { return; }

    for (std::size_t g = 0; g < group_count_; ++g) {
        auto& group = groups_[g];
        if (group.stats.stop_only) { continue; } // 不以 nullptr 冒充未装, 缺最小 device 数据时整组只发零.
        std::array<const drivers::DjiDevice*, 4> devices{};
        bool any = false;
        for (std::size_t s = 0; s < 4; ++s) {
            if (!group.indices[s]) { continue; }
            devices[s] = &*dji_devices_[*group.indices[s]];
            any = true;
        }
        if (!any) { continue; }
        group.sender.emplace(bus_for(group.bus), group.bus, group.group,
            Lifetime{(group.timing.output_ttl.us + 999u) / 1000u}, devices);
        if (!group.sender->configured()) { return; }
    }

    for (std::size_t i = 0; i < dm_slot_count; ++i) {
        const auto& slot = config_.dm[i];
        if (!slot.installed) { continue; }
        if (slot.bus == 0 || slot.bus > 2 || !slot.can_id || !slot.master_id ||
            runtime_[domain_index(slot.domain)]) { return; }
        drivers::DmDeviceConfig address{};
        address.bus = slot.bus; address.can_id = *slot.can_id; address.master_id = *slot.master_id;
        dm_layout[dm_layout_size++] = address;
        if (const auto device_config = app::device_config(slot)) {
            dm_devices_[i].emplace(*device_config);
            if (!dm_devices_[i]->configured()) { return; }
        }
        pitch_stats_.stop_only = !dm_devices_[i];
    }
    // 复用框架已有的布局校验 (DM 内部唯一性 + DJI/DM 同总线 ID 含义冲突),
    // 不在这里维护第二套规则. 地址不明/冲突仍整体拒绝, 运动数据缺项只撤相关输出.
    if (dm_layout_size != 0 && !drivers::valid_dm_addresses({dm_layout.data(), dm_layout_size})) {
        return;
    }
    if ((dm_layout_size || layout_size) && !drivers::valid_mixed_bus({dm_layout.data(), dm_layout_size},
            {layout.data(), layout_size})) {
        return;
    }
    assembled_ = true;
}

transport::CanTx& SentinelExecutor::bus_for(std::uint8_t bus_id) noexcept {
    return bus_id == 1 ? *bus1_ : *bus2_;
}

drivers::DjiDevice* SentinelExecutor::dji_device(std::size_t slot) noexcept {
    return slot < dji_slot_count && dji_devices_[slot] ? &*dji_devices_[slot] : nullptr;
}

motion::DjiAxis* SentinelExecutor::axis(Domain domain) noexcept {
    const auto index = domain_index(domain);
    return axes_[index] ? &*axes_[index] : nullptr;
}

void SentinelExecutor::route(std::uint8_t bus, const transport::CanFrame& frame, Tick at,
    std::optional<MicroTick> at_us) noexcept {
    for (std::size_t i = 0; i < dji_devices_.size(); ++i) {
        auto& device = dji_devices_[i];
        if (device && device->receive(bus, frame, at) == drivers::DjiReceiveResult::accepted) {
            feedback_times_[domain_index(config_.dji[i].domain)] = at_us;
        }
    }
    for (std::size_t i = 0; i < dm_devices_.size(); ++i) {
        auto& device = dm_devices_[i];
        if (device && device->receive(bus, frame, at) == drivers::DmReceiveResult::accepted) {
            feedback_times_[domain_index(config_.dm[i].domain)] = at_us;
        }
    }
}

bool SentinelExecutor::align_reference(Domain domain, float joint_rad, Tick now) noexcept {
    const auto index = domain_index(domain);
    if (!is_joint(domain) || !runtime_[index] || !runtime_[index]->device || !std::isfinite(joint_rad)) { return false; }
    const auto& joint = config_.joints[joint_index(domain)];
    if (!joint.zero_offset_rad || (!joint.manual_continuous && !joint.commissioning_unbounded &&
        (!joint.min_rad || !joint.max_rad || joint_rad < *joint.min_rad || joint_rad > *joint.max_rad))) { return false; }
    const auto& device = *runtime_[index]->device;
    const auto state = device.state(now);
    if (!state.rotor.fresh_at(now, device.config().feedback_ttl)) { return false; }
    const float period = tau_rad / device.config().rotor_per_axis;
    const float tolerance = period / 8192.0f; // 一格编码量化容差, 不给矛盾参考任意容忍带.
    if (joint.commissioning_unbounded) {
        // 已有标记零位不是任意align零点; 只能确认本次连续坐标, 不允许重置成当前位置0.
        return references_[index].valid && state.position_rad.validity == Validity::valid &&
            std::fabs(joint_rad - state.position_rad.value) <= tolerance;
    }
    const auto unique = joint.manual_continuous ? std::optional<float>{} :
        recover_single_turn_joint(*joint.min_rad, *joint.max_rad,
            *joint.zero_offset_rad, state.rotor.value.rotor_angle_rad * device.config().direction /
                device.config().rotor_per_axis, period);
    if (unique) {
        // align 恢复当前配置坐标, 不是改零位; 坐标重定义必须走协同标定的参数事务.
        if (std::fabs(joint_rad-*unique) > tolerance) { return false; }
        joint_rad = *unique;
    }
    if (state.position_rad.validity == Validity::valid &&
        std::fabs(state.position_rad.value-joint_rad) <= tolerance) {
        references_[index] = {true, state.position_rad.value};
        return true; // 一致性确认不移动已知坐标, 避免量化误差成为虚假姿态扰动.
    }
    const bool aligned = axes_[index] ? axes_[index]->reference_position(joint_rad, now) :
        runtime_[index]->device->reference_position(joint_rad, now);
    if (!aligned) { return false; }
    references_[index] = JointReference{true, joint_rad};
    return true;
}

bool SentinelExecutor::feedback_ready(const DomainRuntime& runtime, Tick now) const noexcept {
    const auto ttl = runtime.device->config().feedback_ttl;
    return runtime.device->state(now).velocity_rad_s.fresh_at(now, ttl);
}

DomainFeedback SentinelExecutor::joint_feedback(Domain domain, Tick now) const noexcept {
    DomainFeedback out{};
    const auto index = domain_index(domain);
    // DM 域: 设备原生绝对位置 → 关节角 (减零位偏移), 与健康判定同一坐标.
    if (dm_devices_[0] && config_.dm[0].installed && config_.dm[0].domain == domain) {
        const auto state = dm_devices_[0]->state();
        if (state.feedback.validity == Validity::valid) {
            if (const auto offset = config_.joints[joint_index(domain)].zero_offset_rad) {
                out.position_rad = Sample<float>{state.feedback.value.position_rad - *offset,
                    state.feedback.sampled_at, Validity::valid};
            }
            out.velocity_rad_s = Sample<float>{state.feedback.value.velocity_rad_s,
                state.feedback.sampled_at, Validity::valid};
        }
        return out;
    }
    if (!runtime_[index] || !runtime_[index]->device) { return out; }
    // 关节位置只在该域位置参考有效时才有意义: 参考失效 (零位/范围改变、设备重启、
    // 反馈丢失) 后设备侧样本可能仍是旧参考下的有效值, 不能把它当成本周期的关节视图 ——
    // 否则 Mapper 会先把旧坐标捕获成"新目标", 健康判定随后才按新参考重建.
    if (is_joint(domain) && !references_[index].valid) { return out; }
    const auto state = runtime_[index]->device->state(now);
    out.position_rad = state.position_rad;
    out.velocity_rad_s = joint_velocity_feedback_[index].value_or(state.velocity_rad_s);
    return out;
}

std::optional<float> SentinelExecutor::branch_for(
    Domain domain, float requested_rad, Tick now) noexcept {
    const auto index = domain_index(domain);
    const auto& joint = config_.joints[joint_index(domain)];
    if (!joint.min_rad || !joint.max_rad) { return requested_rad; }
    if (domain != Domain::main_yaw) {
        // fire yaw 是相对关节角, 已在接受整包时限幅, 不做等价分支选择.
        return requested_rad;
    }
    const auto state = axes_[index]->state(now);
    const float current = state.position_rad.validity == Validity::valid ? state.position_rad.value
                                                                        : references_[index].joint_rad;
    return select_equivalent_branch(*joint.min_rad, *joint.max_rad, current, requested_rad);
}

FaultInputs SentinelExecutor::health(Tick now) noexcept {
    health_ = FaultInputs{};
    // 上一周期发现的不可达目标与执行器拒绝要回灌给应用层: 由应用层作废/拒绝该目标.
    health_.target_unreachable = unreachable_;
    health_.execution_rejected = rejected_;
    if (!assembled_) {
        for (const auto& slot : config_.dji) { if (slot.installed) { health_.execution_rejected[domain_index(slot.domain)] = true; } }
        for (const auto& slot : config_.dm) { if (slot.installed) { health_.execution_rejected[domain_index(slot.domain)] = true; } }
        return health_;
    }
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto& runtime = runtime_[i];
        if (!runtime) { continue; } // 未装配或 DM 域 (单独处理)
        if (groups_[runtime->group].stats.stop_only) { health_.execution_rejected[i] = true; }
        if (!runtime->device) { continue; }
        const auto domain = static_cast<Domain>(i);
        const auto* joint_config = is_joint(domain) ? &config_.joints[joint_index(domain)] : nullptr;
        const bool joint_ready = joint_config && joint_config->zero_offset_rad &&
            (joint_config->manual_continuous || joint_config->commissioning_unbounded || (joint_config->min_rad && joint_config->max_rad));
        const bool expected = axes_[i].has_value() || joint_ready;
        const auto ttl = runtime->device->config().feedback_ttl;
        auto state = runtime->device->state(now);
        // 设备层会把过期样本标成 invalid, 因此 unknown 才表示"从未收到过反馈".
        const bool ever_online = state.rotor.validity != Validity::unknown;
        const bool online = state.rotor.fresh_at(now, ttl);
        if (!online) {
            // 必需反馈不新鲜: 从未收到与运行后失联都不满足执行条件 (依赖集合必须知道),
            // 但两者在观测上要能区分 —— 关节域按参考失效, 从未收到时同时标反馈缺失.
            if (expected || ever_online) {
                if (is_joint(domain)) {
                    health_.reference_lost[i] = ever_online;
                    health_.feedback_lost[i] = !ever_online;
                } else { health_.feedback_lost[i] = true; }
            }
            references_[i].valid = false;
            continue;
        }
        if (!joint_ready) { references_[i].valid = false; continue; }
        const auto& joint = *joint_config;
        auto& reference = references_[i];
        // 设备因跨圈歧义/超时丢弃了位置: 参考不再可信 (只有一个权威来源).
        if (reference.valid && state.position_rad.validity != Validity::valid) {
            reference.valid = false;
        }
        if (!reference.valid && joint.commissioning_unbounded && joint.zero_offset_rad) {
            const float marked = std::remainder(state.rotor.value.rotor_angle_rad *
                static_cast<float>(runtime->device->config().direction) - *joint.zero_offset_rad, tau_rad);
            const bool aligned = axes_[i] ? axes_[i]->reference_position(marked, now) :
                runtime->device->reference_position(marked, now);
            if (aligned) { reference = JointReference{true, marked}; state = runtime->device->state(now); }
        }
        if (!reference.valid && !joint.manual_continuous && joint.min_rad && joint.max_rad && joint.zero_offset_rad) {
            // 单圈反馈 + 已配置范围唯一确定关节角时才自动建立参考, 不猜圈数.
            // 轴侧混叠周期是 2π/N: 转子单圈编码器经传动后看不到整圈.
            const float axis_single = state.rotor.value.rotor_angle_rad *
                static_cast<float>(runtime->device->config().direction) /
                runtime->device->config().rotor_per_axis;
            const auto recovered = recover_single_turn_joint(*joint.min_rad, *joint.max_rad,
                *joint.zero_offset_rad, axis_single,
                tau_rad / runtime->device->config().rotor_per_axis);
            const bool aligned = recovered && (axes_[i] ? axes_[i]->reference_position(*recovered, now) :
                runtime->device->reference_position(*recovered, now));
            if (aligned) {
                reference = JointReference{true, *recovered};
                // 参考刚建立: 设备位置样本此刻才有效, 重新读取而不是拿旧快照判定.
                state = runtime->device->state(now);
            }
        }
        if (reference.valid) {
            const auto position = state.position_rad;
            reference.joint_rad = position.value;
            // 真实关节反馈超出已配置机械范围: 按该轴故障处理.
            if (joint.min_rad && joint.max_rad &&
                (position.value < *joint.min_rad || position.value > *joint.max_rad)) {
                health_.device_fault[i] = true;
            }
        } else {
            // 在线但无法唯一确定参考: 保持参考未知, 不自动寻零.
            health_.reference_lost[i] = true;
        }
    }
    // DM 域 (pitch): 原生绝对位置反馈, 不做单圈恢复.
    if (dm_devices_[0] && config_.dm[0].installed) {
        auto& device = *dm_devices_[0];
        const auto index = domain_index(config_.dm[0].domain);
        const auto& joint = config_.joints[joint_index(config_.dm[0].domain)];
        const bool expected = joint.zero_offset_rad && ((joint.min_rad && joint.max_rad) || joint.commissioning_unbounded);
        const bool ever_online = device.state().feedback.validity != Validity::unknown;
        const bool online = device.online(now);
        if (device.faulted(now)) { health_.device_fault[index] = true; }
        if (!online) {
            // 同上: 从未收到与运行后失联都不满足执行条件, 但区分二者.
            if (expected || ever_online) {
                health_.reference_lost[index] = ever_online;
                health_.feedback_lost[index] = !ever_online;
            }
            references_[index].valid = false;
        } else if (joint.zero_offset_rad) {
            const float value = device.state().feedback.value.position_rad - *joint.zero_offset_rad;
            references_[index] = JointReference{true, value};
            if (joint.min_rad && joint.max_rad &&
                (value < *joint.min_rad || value > *joint.max_rad)) {
                health_.device_fault[index] = true;
            }
        }
    }
    return health_;
}

float SentinelExecutor::velocity_gain(std::size_t dji_slot, int which) const noexcept {
    if (dji_slot >= dji_slot_count || !config_.dji[dji_slot].installed) { return 0.0f; }
    const auto index = domain_index(config_.dji[dji_slot].domain);
    if (!axes_[index]) { return 0.0f; }
    // 读当前模式真正在用的环路: 位置模式读位置串级内环, 速度模式读独立速度环.
    const auto pid = axes_[index]->active_velocity_pid();
    if (which == 1) { return pid.ki; }
    if (which == 2) { return pid.kd; }
    return pid.kp;
}

bool SentinelExecutor::set_joint_geometry(
    Domain domain, const JointGeometryConfig& joint) noexcept {
    if (!is_joint(domain) || (joint.manual_continuous &&
        (domain == Domain::pitch || joint.min_rad || joint.max_rad)) ||
        (joint.commissioning_unbounded && (joint.manual_continuous || joint.min_rad || joint.max_rad))) { return false; }
    config_.joints[joint_index(domain)] = joint;
    // 零位/机械范围改变后, 已有位置参考属于旧坐标: 失效旧参考, 由 health 按新的合法
    // 单圈信息重建; 不能用旧零偏下依然 "valid" 的位置跟着新范围判定.
    references_[domain_index(domain)].valid = false;
    return true;
}

bool SentinelExecutor::set_dm_mit_gains(float kp, float kd) noexcept {
    if (!config_.dm[0].installed || !config_.dm[0].mit_kp || !config_.dm[0].mit_kd) {
        return false;
    }
    if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0f || kd < 0.0f) { return false; }
    config_.dm[0].mit_kp = kp;
    config_.dm[0].mit_kd = kd;
    return true;
}

std::optional<recording::Record> SentinelExecutor::trace(Domain domain, Tick now, const SentinelApp& app) const noexcept {
    const auto i = domain_index(domain);
    if (!runtime_[i] || !axes_[i]) { return std::nullopt; }
    const auto& group = groups_[runtime_[i]->group];
    if (!group.step.due) { return std::nullopt; }
    const auto& execution = app.snapshot().executions[i];
    const auto target = app.target(domain);
    const auto feedback = joint_feedback(domain, now);
    const auto output = outputs_[runtime_[i]->dji_slot];
    const float missing = std::numeric_limits<float>::quiet_NaN();
    recording::Record r{recording::Kind::control, static_cast<std::uint8_t>(i), 1, 0,
        group.stats.computations, group.stats.last_computed.us, group.stats.last_computed.us, group.stats.tx_failed, 0, 0};
    const std::array values{target ? target->value : missing, execution.adopted_target.value_or(missing),
        feedback.position_rad.value, feedback.velocity_rad_s.value, output ? output->normalized : missing, group.step.dt_s};
    for (std::size_t n = 0; n < values.size(); ++n) { r.data[n] = std::bit_cast<std::uint32_t>(values[n]); }
    // 状态/有效性显式打包. bit24 是轴侧电流限幅触及, 不是额外控制门禁.
    const auto& device = dji_devices_[runtime_[i]->dji_slot]->config();
    const auto full = drivers::dji_current_amp_full_scale(device.mode);
    const auto protocol = drivers::dji_protocol(device.mode);
    const bool limited = output && full && protocol && std::fabs(output->normalized * *full *
        static_cast<float>(device.output_limit_raw) / static_cast<float>(protocol->limit)) >= axes_[i]->current_limit();
    r.data[6] = static_cast<std::uint32_t>(execution.mode) | (static_cast<std::uint32_t>(execution.reason) << 8)
        | (static_cast<std::uint32_t>(feedback.position_rad.validity) << 16)
        | (static_cast<std::uint32_t>(feedback.velocity_rad_s.validity) << 20) | (limited ? 1u << 24 : 0);
    r.data[6] |= static_cast<std::uint32_t>(target ? target->frame : execution.frame) << 25;
    r.data[6] |= static_cast<std::uint32_t>(execution.frame) << 27;
    r.data[7] = static_cast<std::uint32_t>(group.stats.last_result);
    return r;
}

void SentinelExecutor::send_groups(Tick now) noexcept {
    for (std::size_t g = 0; g < group_count_; ++g) {
        auto& group = groups_[g];
        if (!group.step.due) { continue; }
        if (group.stats.stop_only) {
            // 构造时已经核对所有真实槽与地址, 本组所有槽只能执行 A08 零输出, 不合并半成品请求.
            const auto result = bus_for(group.bus).try_send(*group.zero_frame);
            if (result == transport::SendResult::accepted) {
                ++stats_.group_sent; ++group.stats.submitted;
                group.stats.last_result = drivers::DjiGroupStatus::accepted;
                if (clock_) { group.stats.last_submitted = clock_(); }
            } else {
                ++stats_.group_tx_error; ++group.stats.tx_failed;
                group.stats.last_result = result == transport::SendResult::busy ? drivers::DjiGroupStatus::busy : drivers::DjiGroupStatus::error;
            }
            continue;
        }
        std::array<std::optional<drivers::DjiOutput>, 4> slots{};
        for (std::size_t s = 0; s < 4; ++s) {
            if (group.indices[s]) { slots[s] = outputs_[*group.indices[s]]; }
        }
        // 用实际单调提交时刻检查本组最早计算的输出, 不给结果重贴 now 或补旧槽.
        // HAL accepted 仅是本地提交成功, 此处不承诺 CAN 线上送达时刻.
        if (clock_ && !micro_fresh(clock_(), group.stats.last_computed, group.timing.output_ttl)) {
            group.stats.last_result = drivers::DjiGroupStatus::stale_output;
            ++stats_.group_refused;
            for (const auto slot : group.indices) {
                if (slot) { rejected_[domain_index(config_.dji[*slot].domain)] = true; }
            }
            continue;
        }
        const auto result = group.sender->send(now, slots);
        group.stats.last_result = result.status;
        switch (result.status) {
        case drivers::DjiGroupStatus::accepted:
            ++stats_.group_sent; ++group.stats.submitted;
            if (clock_) { group.stats.last_submitted = clock_(); }
            break;
        case drivers::DjiGroupStatus::busy:
        case drivers::DjiGroupStatus::error:
            ++stats_.group_tx_error; ++group.stats.tx_failed; break;
        case drivers::DjiGroupStatus::invalid_config:
        case drivers::DjiGroupStatus::missing_output:
        case drivers::DjiGroupStatus::stale_output:
        case drivers::DjiGroupStatus::invalid_output:
        case drivers::DjiGroupStatus::not_sent: ++stats_.group_refused; break;
        }
    }
}

bool SentinelExecutor::send_pitch_frame(const transport::CanFrame& frame) noexcept {
    const auto result = bus_for(config_.dm[0].bus).try_send(frame);
    const bool accepted = result == transport::SendResult::accepted;
    pitch_stats_.last_result = accepted ? drivers::DjiGroupStatus::accepted :
        result == transport::SendResult::busy ? drivers::DjiGroupStatus::busy : drivers::DjiGroupStatus::error;
    if (accepted) {
        ++pitch_stats_.submitted;
        if (clock_) { pitch_stats_.last_submitted = clock_(); }
    } else { ++pitch_stats_.tx_failed; }
    return accepted;
}

void SentinelExecutor::query_pitch() noexcept {
    // 已核实的状态问询, 不通过使能/MIT 获取失能设备的反馈.
    const auto id = config_.dm[0].can_id.value_or(0);
    transport::CanFrame frame{};
    frame.id = 0x7ff;
    frame.size = 4;
    frame.data = {static_cast<std::uint8_t>(id), static_cast<std::uint8_t>(id >> 8), 0xcc, 0};
    if (send_pitch_frame(frame)) {
        ++stats_.dm_query_sent;
    } else { ++stats_.dm_tx_error; }
}

bool SentinelExecutor::pitch_output_fresh() noexcept {
    if (!clock_ || micro_fresh(clock_(), pitch_stats_.last_computed, config_.runtime.pitch.output_ttl)) { return true; }
    rejected_[domain_index(config_.dm[0].domain)] = true;
    ++stats_.target_rejections;
    pitch_stats_.last_result = drivers::DjiGroupStatus::stale_output;
    return false;
}

void SentinelExecutor::step_pitch(Tick now, const SentinelApp& app, bool force_stop) noexcept {
    if (!config_.dm[0].installed) { return; }
    if (!dm_devices_[0]) {
        // 地址明确但 MIT 量程未测: 只按既有间隔请求 disable, 不构造假量程或使能.
        if (!pitch_disable_attempted_ || !fresh(now, pitch_last_disable_at_, Lifetime{pitch_disable_retry_ms})) {
            pitch_disable_attempted_ = true; pitch_last_disable_at_ = now;
            const auto frame = drivers::encode_dm_action(*config_.dm[0].can_id, drivers::DmAction::disable);
            if (frame && send_pitch_frame(*frame)) { ++stats_.dm_disable_sent; pitch_disable_requested_ = true; }
            else { ++stats_.dm_tx_error; }
        }
        return;
    }
    auto& device = *dm_devices_[0];
    const auto& execution = app.snapshot().executions[domain_index(config_.dm[0].domain)];
    if (!execution.execute || force_stop) {
        // 撤输出 = 显式 disable 请求. 是否发送取决于**当前观测状态**, 不取决于本进程是否
        // 发过 enable, 也不被历史 latch 永久阻断: MCU/电调可以独立重启, 接入时设备可能已
        // 经 Enabled; 发送被 accepted 不当作状态确认, 只有新鲜反馈确认 Disabled (且非故障
        // 态) 才认为停止已落实并重建尝试预算.
        if (device.online(now) && !device.faulted(now) && !device.enabled(now)) {
            pitch_disable_requested_ = false;
            pitch_disable_attempted_ = false;
            pitch_enable_requested_ = false;
            pitch_enable_confirmed_ = false;
            query_pitch();
            return;
        }
        pitch_enable_requested_ = false;
        pitch_enable_confirmed_ = false;
        // 有界重试: 每个重试间隔至多一次, 总线恢复且停止请求仍保持时能重新尝试.
        if (pitch_disable_attempted_ &&
            fresh(now, pitch_last_disable_at_, Lifetime{pitch_disable_retry_ms})) {
            query_pitch();
            return;
        }
        pitch_disable_attempted_ = true;
        pitch_last_disable_at_ = now;
        const auto frame = device.encode_disable();
        if (!frame) {
            ++stats_.target_rejections;
            return;
        }
        if (send_pitch_frame(*frame)) {
            ++stats_.dm_disable_sent;
            pitch_disable_requested_ = true; // 已发"请求", 未确认
        } else {
            ++stats_.dm_tx_error;
        }
        return;
    }
    // 期望状态重新变成使能: 重建 disable 尝试预算.
    pitch_disable_attempted_ = false;
    if (!pitch_enable_requested_) {
        pitch_disable_requested_ = false;
        const auto frame = device.encode_enable();
        if (!frame) {
            ++stats_.target_rejections;
            rejected_[domain_index(config_.dm[0].domain)] = true;
            return;
        }
        if (!pitch_output_fresh()) { return; }
        if (send_pitch_frame(*frame)) {
            ++stats_.dm_enable_sent;
            pitch_enable_requested_ = true;
        } else {
            ++stats_.dm_tx_error;
        }
        return; // 使能请求与 MIT 目标不在同一周期发出
    }
    // 只有新鲜反馈确认 Enabled 才下发 MIT, 不用"已发使能帧"冒充已使能.
    pitch_enable_confirmed_ = device.enabled(now);
    if (!pitch_enable_confirmed_) { query_pitch(); return; }
    const auto target = app.target(config_.dm[0].domain);
    if (!target) { return; }
    const auto& joint = config_.joints[joint_index(config_.dm[0].domain)];
    const auto command = drivers::DmMitCommand{target->value + joint.zero_offset_rad.value_or(0.0f),
        0.0f, config_.dm[0].mit_kp.value_or(0.0f), config_.dm[0].mit_kd.value_or(0.0f), 0.0f};
    const auto frame = device.encode_mit(command);
    // 真编码拒绝回灌既有域失效入口; 反馈恢复不再自动重试同一份旧意图.
    if (!frame) {
        ++stats_.target_rejections;
        rejected_[domain_index(config_.dm[0].domain)] = true;
        return;
    }
    if (!pitch_output_fresh()) { return; }
    if (send_pitch_frame(*frame)) {
        ++stats_.dm_command_sent;
    } else {
        ++stats_.dm_tx_error;
    }
}

void SentinelExecutor::step(Tick now, float dt_s, const SentinelApp& app,
    std::optional<MicroTick> now_us) noexcept {
    ++stats_.cycles;
    unreachable_.fill(false);
    rejected_.fill(false);
    outputs_.fill(std::nullopt);
    // 构造未完成 (缺项/布局冲突) 时不发布任何半成品输出, 也不产生部分组帧.
    if (!assembled_) { return; }
    const auto control_start = clock_ ? clock_() : MicroTick{};
    const auto at = now_us.value_or(MicroTick{now.ms * 1000u});
    const bool bad_dt = !std::isfinite(dt_s) || dt_s <= 0.0f;
    std::array<bool, group_capacity> force{};
    for (std::size_t i = 0; i < domain_count; ++i) {
        if (runtime_[i] && was_executing_[i] && !app.snapshot().executions[i].execute) {
            force[runtime_[i]->group] = true;
        }
    }
    for (std::size_t g = 0; g < group_count_; ++g) {
        auto& group = groups_[g];
        group.compute_started = false;
        group.stats.timing_observed = clock_ != nullptr;
        group.step = group.clock.poll(at, group.timing, force[g], MicroTick{now.ms * 1000u});
        if (bad_dt) { group.step = {true, true, dt_s}; }
        if (group.step.invalid_time) { group.step.due = true; }
        if (!group.step.due) { ++stats_.skipped_groups; }
        else {
            ++group.stats.computations;
            group.stats.last_computed = at;
            group.stats.last_dt_s = group.step.dt_s;
            if (std::isfinite(group.step.dt_s)) {
                group.stats.max_dt_s = std::max(group.stats.max_dt_s, group.step.dt_s);
            }
            if (group.step.invalid_time) { ++group.stats.invalid_steps; }
        }
    }
    const auto source = command_source_of(app.snapshot().permission.source);
    const bool authorized = app.snapshot().permission.authority;
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto domain = static_cast<Domain>(i);
        auto& runtime = runtime_[i];
        if (!runtime) { continue; } // 未装配或 DM 域 (单独处理)
        was_executing_[i] = app.snapshot().executions[i].execute;
        const auto& scheduled = groups_[runtime->group].step;
        if (!scheduled.due) { continue; }
        auto& group = groups_[runtime->group];
        if (!group.compute_started) {
            group.compute_started = true;
            if (clock_) { group.stats.last_computed = clock_(); }
        }
        const auto& execution = app.snapshot().executions[i];
        const auto slot = runtime->dji_slot;
        if (group.stats.stop_only || !axes_[i]) {
            outputs_[slot] = drivers::DjiOutput{0.0f, now};
            if (axes_[i] && axes_[i]->enabled()) { axes_[i]->disable(); }
            continue;
        }
        auto& axis = *axes_[i];
        if (domain == Domain::main_yaw) {
            main_yaw_stats_.frame = execution.frame;
            main_yaw_stats_.mechanical_limit = false;
        }
        const auto stop_output = [&]() {
            // 已装配且配置齐备的槽必须有本周期输出: 按应用选定的撤输出动作明确生成,
            // 而不是留空导致同组其它电机一起停发.
            outputs_[slot] = drivers::DjiOutput{0.0f, now};
        };
        if (!execution.execute) {
            if (axis.enabled()) { axis.disable(); }
            stop_output();
            continue;
        }
        if (!execution.adopted_target) {
            axis.disable();
            stop_output();
            continue;
        }
        const bool inertial_main = domain == Domain::main_yaw && execution.frame != YawFrame::joint;
        const auto& platform = app.snapshot().main_yaw;
        Sample<float> direction{}, direction_rate{};
        if (domain == Domain::main_yaw) {
            if (main_inertial_active_ != inertial_main) {
                axis.disable(); // 坐标变换清积分/测量历史, 不把两套输出叠加
                direction_clock_ = GroupClock{};
                ++main_yaw_stats_.feedback_switches;
            }
            main_inertial_active_ = inertial_main;
        }
        std::optional<float> resolved = execution.adopted_target;
        if (inertial_main) {
            const auto joint = joint_feedback(domain, now);
            const auto& range = config_.joints[joint_index(domain)];
            const auto local = *execution.adopted_target - (execution.frame == YawFrame::odom ? platform.odom_offset_rad : 0.0f);
            if (!config_.serial || platform.validity != DomainValidity::valid ||
                (execution.frame == YawFrame::odom && !platform.odom_aligned) ||
                joint.position_rad.validity != Validity::valid || !range.min_rad || !range.max_rad) {
                resolved.reset();
            } else {
                const auto mechanical = reachable_heading(*config_.serial, platform.value.orientation,
                    joint.position_rad.value, local, *range.min_rad, *range.max_rad);
                if (!mechanical) { resolved.reset(); }
                else {
                    const auto gain = platform.value.heading_gain;
                    // 精确三维反解选择机械分支, 把其有符号误差通过当前作用增益拉回方向坐标.
                    // PID 的测量及 D 项仍来自真实连续方向; 内环再按同一增益映射惯性速率.
                    resolved = platform.continuous_heading + gain*(*mechanical-joint.position_rad.value);
                    const auto age = *std::max_element(platform.ages_us.begin(), platform.ages_us.end());
                    const Tick source_at{now.ms-(age+999u)/1000u};
                    direction = {platform.continuous_heading, source_at, Validity::valid};
                    direction_rate = {platform.value.heading_rate, source_at, Validity::valid};
                    main_yaw_stats_.local_target_rad = local;
                    main_yaw_stats_.resolved_joint_rad = *mechanical;
                    main_yaw_stats_.equivalent_rate_rad_s = platform.value.heading_rate/gain;
                }
            }
        } else if (execution.mode == control::ControlMode::position && domain != Domain::main_yaw) {
            resolved = branch_for(domain, *execution.adopted_target, now);
        }
        if (!resolved) {
            // 机械范围内无可达等价方向: 拒绝该目标, 不调其它轴补偿.
            unreachable_[i] = true;
            ++stats_.unreachable_targets;
            axis.disable();
            stop_output();
            continue;
        }
        if (!axis.enabled()) {
            if (inertial_main) { direction_clock_ = GroupClock{}; }
            if (!axis.enable(execution.mode, source, authorized, feedback_ready(*runtime, now))) {
                ++stats_.enable_failures;
                stop_output();
                continue;
            }
        }
        const bool target_ok = execution.mode == control::ControlMode::velocity
            ? axis.set_velocity(*resolved, now)
            : axis.set_position(*resolved, now);
        if (!target_ok) {
            ++stats_.target_rejections;
            rejected_[i] = true;
            axis.disable();
            stop_output();
            continue;
        }
        motion::DjiAxisResult result;
        if (inertial_main) {
            const GroupTiming timing{config_.runtime.direction_period_us, 0,
                {config_.runtime.direction_max_step_us}, config_.runtime.direction_max_step_us};
            const auto outer = direction_clock_.poll(at, timing, false, MicroTick{now.ms*1000u});
            const auto outer_dt = outer.invalid_time ? std::optional{0.0f}
                : outer.due ? std::optional{outer.dt_s} : std::nullopt;
            if (outer.due) { ++main_yaw_stats_.outer_steps; main_yaw_stats_.outer_dt_s = outer.dt_s; }
            ++main_yaw_stats_.inner_steps;
            result = axis.tick_external_position(now, scheduled.invalid_time ? 0.0f : scheduled.dt_s,
                direction, direction_rate, outer_dt, platform.value.heading_gain);
        } else if (joint_velocity_feedback_[i]) {
            result = axis.tick_joint_velocity(now, scheduled.invalid_time ? 0.0f : scheduled.dt_s,
                *joint_velocity_feedback_[i]);
        } else {
            result = axis.tick(now, scheduled.invalid_time ? 0.0f : scheduled.dt_s);
        }
        if (result.normalized_output) {
            float output = *result.normalized_output;
            if (inertial_main) {
                const auto position = joint_feedback(domain, now).position_rad.value;
                const auto& range = config_.joints[joint_index(domain)];
                main_yaw_stats_.mechanical_limit = (position >= *range.max_rad && output > 0) ||
                    (position <= *range.min_rad && output < 0);
                if (main_yaw_stats_.mechanical_limit) { output = 0; }
            }
            outputs_[slot] = drivers::DjiOutput{output, now};
        } else {
            // 轴层拒绝 (无反馈/过期/控制器错误): 本周期没有有效输出, 明确撤输出,
            // 并把拒绝结果回灌给应用层作废旧意图 (不自动重试同一份旧目标).
            ++stats_.target_rejections;
            rejected_[i] = true;
            stop_output();
        }
    }
    const auto tx_start = clock_ ? clock_() : MicroTick{};
    if (clock_) {
        stats_.timing_observed = true;
        stats_.control_last_us = tx_start.us - control_start.us;
        stats_.control_max_us = std::max(stats_.control_max_us, stats_.control_last_us);
    }
    send_groups(now);
    const auto pitch = domain_index(Domain::pitch);
    const bool stop_pitch = was_executing_[pitch] && !app.snapshot().executions[pitch].execute;
    was_executing_[pitch] = app.snapshot().executions[pitch].execute;
    const auto scheduled = pitch_clock_.poll(at, config_.runtime.pitch, stop_pitch, MicroTick{now.ms * 1000u});
    if (config_.dm[0].installed && (scheduled.due || scheduled.invalid_time || bad_dt)) {
        pitch_stats_.bus = config_.dm[0].bus;
        pitch_stats_.id = config_.dm[0].can_id.value_or(0); // 设备 ID, 查询帧自身为 0x7ff
        ++pitch_stats_.computations;
        pitch_stats_.timing_observed = clock_ != nullptr;
        pitch_stats_.last_computed = clock_ ? clock_() : at;
        pitch_stats_.last_dt_s = scheduled.dt_s;
        pitch_stats_.max_dt_s = std::max(pitch_stats_.max_dt_s, scheduled.dt_s);
        if (scheduled.invalid_time || bad_dt) {
            ++pitch_stats_.invalid_steps;
            rejected_[pitch] = true;
        }
        step_pitch(now, app, scheduled.invalid_time || bad_dt);
    }
    if (clock_) {
        stats_.tx_last_us = clock_().us - tx_start.us;
        stats_.tx_max_us = std::max(stats_.tx_max_us, stats_.tx_last_us);
    }
}
} // 命名空间 ec::sentinel::app
