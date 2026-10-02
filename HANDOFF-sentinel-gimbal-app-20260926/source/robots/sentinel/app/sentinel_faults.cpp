#include "sentinel_faults.hpp"

namespace ec::sentinel::app {
FaultEffects evaluate_faults(const FaultInputs& inputs) noexcept {
    FaultEffects out{};
    for (std::size_t i = 0; i < domain_count; ++i) {
        out.faulted[i] = inputs.feedback_lost[i] || inputs.reference_lost[i] ||
            inputs.device_fault[i];
    }

    // 底盘: 任一底盘轮故障 → 四轮撤输出 (不独立夹其它轮).
    bool chassis_fault = false;
    for (std::size_t i = 0; i < chassis_wheel_count; ++i) { chassis_fault |= out.faulted[i]; }
    if (chassis_fault) {
        for (std::size_t i = 0; i < chassis_wheel_count; ++i) { out.output_revoked[i] = true; }
    }

    // 两 yaw: 该轴必需编码器/参考失效 → 该轴撤输出并禁止正向拨弹.
    for (const auto domain : {Domain::main_yaw, Domain::fire_yaw}) {
        const auto index = domain_index(domain);
        if (out.faulted[index]) {
            out.output_revoked[index] = true;
            out.trigger_inhibited = true;
        }
    }

    // pitch: 必需反馈/参考失效 → 撤输出 (应用按 disable 策略落实) 并禁止正向拨弹.
    if (out.faulted[domain_index(Domain::pitch)]) {
        out.output_revoked[domain_index(Domain::pitch)] = true;
        out.trigger_inhibited = true;
    }

    // 摩擦轮: 任一故障 → 两摩擦轮与拨弹撤输出.
    if (out.faulted[domain_index(Domain::friction_left)] ||
        out.faulted[domain_index(Domain::friction_right)]) {
        out.output_revoked[domain_index(Domain::friction_left)] = true;
        out.output_revoked[domain_index(Domain::friction_right)] = true;
        out.output_revoked[domain_index(Domain::trigger)] = true;
    }

    // 拨弹自身故障 → 仅拨弹撤输出, 摩擦轮不受牵连.
    if (out.faulted[domain_index(Domain::trigger)]) {
        out.output_revoked[domain_index(Domain::trigger)] = true;
    }

    // 仅惯性失效且编码器/参考仍有效: 降级保持, 不是设备故障, 不影响拨弹资格.
    if (inputs.main_yaw_inertial_only && !out.faulted[domain_index(Domain::main_yaw)]) {
        out.main_yaw_degraded = true;
    }
    return out;
}
} // 命名空间 ec::sentinel::app
