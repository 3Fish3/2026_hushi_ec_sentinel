#pragma once
// 故障影响集合 (O1): 把 Checklist A09 已选的依赖表映射为可执行结论.
// 只做映射, 不新增同义保护层, 不判断阈值 (阈值与真实可检测事件仍待配置/实测),
// 也不触发人工整车停止挡 —— 自动故障只撤相关输出.
#include "sentinel_types.hpp"
#include <array>

namespace ec::sentinel::app {
// 各域的原始健康观察. O1 由上层/测试显式注入; O2/O3 接入真实反馈后走同一接口.
struct FaultInputs {
    // 速度域的必需反馈/编码器失效.
    std::array<bool, domain_count> feedback_lost{};
    // 位置/机械参考缺失或失效 (位置域).
    std::array<bool, domain_count> reference_lost{};
    // 已由反馈判定出的设备故障.
    std::array<bool, domain_count> device_fault{};
    // 检测到该域执行器重新上电 (A12): 不是故障, 但必须清理旧目标并重新建立.
    std::array<bool, domain_count> device_restarted{};
    // 上一周期该域目标在机械范围内无可达等价分支 (G07): 拒绝该目标, 不作为设备故障,
    // 也不自动调其它轴补偿.
    std::array<bool, domain_count> target_unreachable{};
    // 上一周期该域的执行器拒绝了目标/输出 (控制器错误、非法 dt、反馈在使能后失效等):
    // 这不是设备故障, 但旧意图必须作废 —— 否则条件恢复后会把同一份旧目标重新盖上新的时刻执行.
    std::array<bool, domain_count> execution_rejected{};
    // 主 yaw 仅惯性反馈失效而编码器仍有效: 按 G07 降级保持, 不按设备故障撤输出.
    bool main_yaw_inertial_only{};
};

struct FaultEffects {
    std::array<bool, domain_count> faulted{};
    // 必须撤掉主动输出的域 (DJI 零电流 / pitch disable 由应用策略落实).
    std::array<bool, domain_count> output_revoked{};
    bool trigger_inhibited{}; // 禁止正向拨弹; 人工反转不受此限
    bool main_yaw_degraded{}; // 惯性降级保持 (不含编码器/参考失效)
};

[[nodiscard]] FaultEffects evaluate_faults(const FaultInputs& inputs) noexcept;
} // 命名空间 ec::sentinel::app
