#pragma once
// 10310221底盘+云台联调. 仅保留协议输出边界, 不沿用调试运动限额.
#include "board_app_chassis_config.hpp"
#include "board_app_mechanism_config.hpp"
#include "board_app_integration_site.hpp"
#include "board_current_harness.hpp"

namespace ec::sentinel::board {
[[nodiscard]] constexpr app::SentinelConfig integration_config(app::SentinelConfig config) noexcept {
    config = mechanism_commissioning_config(chassis_commissioning_config(config), true);
    constexpr float old_wheel_current = 6000.0f*20.0f/16384.0f;
    for (std::size_t i=0; i<6; ++i) {
        auto& slot=config.dji[i];
        const bool wheel=i<4;
        slot.bus=current_harness[i].bus;
        slot.device_id=current_harness[i].node;
        slot.origin=app::Origin::user_confirmed;
        slot.current_limit_a=wheel ? 20.0f : 3.0f;
        slot.output_limit_raw=16384;
        slot.max_axis_velocity_rad_s=0.0f; // 明确取消软件速度目标上限, 不是巨大假限额.
        if (wheel) {
            slot.velocity_pid=control::PidConfig{0.362873f*old_wheel_current/20.0f,0,0,1,-1,1};
        } else {
            // 2026-11-03 云台参数表终版: 2.5A下的原生值换算到联调3A, 保持物理增益。
            // 2026-11-11 main_yaw 改用归一化(3A域)直接给定, 不再经2.5A域换算:
            //   速度0.4/0.35/0 = 电流口径1.2/1.05/0 A/(rad/s).
            //   依据 velocity15 四次同规格15s无偏置试验: Ki=0.4末秒角跨度6LSB不合格;
            //   Ki=0.3正反末秒均速误差-0.038/+0.051超0.025被淘汰; Ki=0.35两次A_numeric
            //   (末秒角跨度1LSB, 正反误差+0.011/+0.010), 第三次6LSB属零速段约0.10A残余
            //   越过约0.107A静摩擦线的边界抖动, 非参数可消除.
            //   fire_yaw 保持现场确认值: 0.16/3.0(2.5A域) -> 归一化0.1333/2.5.
            const float kp=i==4 ? 0.4f : 0.16f*2.5f/3.0f;
            const float ki=i==4 ? 0.35f : 3.0f*2.5f/3.0f;
            slot.velocity_pid=control::PidConfig{kp,ki,0,1,-1,1};
            // 沿用联调角色的位置外环不限速策略, 仅更新经用户确认的PID增益。
            // 2026-11-04 main位置环Kp由28提高到42(Ki 2->3), 依实测大幅扫描: 跨378.5度,
            // 跟踪RMS 4.09度, 误差均值0.12度, 换向超调约1.9度, 稳定不发散。
            // 2026-11-07 现场对照: 位置Kp60在自旋加减速后回中振铃更明显, 降到42后振铃幅度与
            // 恢复时间均改善; 速度内环Kp同批由0.0416667提到0.060, 恢复进一步改善但0.060与0.050
            // 收益接近。
            // 2026-11-11 main位置环改为6.3/0.45: 与旧0.060/42按同一总环增益比例(6.67x)配套缩放,
            // 使外环×内环的乘积维持原比例纠偏电流系数. 该配套关系来自增益比例推导, 15s验证
            // 旁路外环, 位置环尚未单独闭环验证.
            const float pos_kp = i==4 ? 6.3f : 32.0f;
            const float pos_ki = i==4 ? 0.45f : 0.0f;
            slot.position_pid=control::PidConfig{pos_kp,pos_ki,
                i==4 ? 0.0f : 0.1f,15,0,0,0,false};
        }
    }
    // 由回报旧零偏+朝前/水平关节读数建立新软件0, 不写电调硬件零点.
    constexpr float tau=6.2831853071795864769f;
    config.joints[0]={app::Domain::main_yaw,1660.0f*tau/8192.0f,{},{},false,true};
    config.joints[1]={app::Domain::fire_yaw,2183.0f*tau/8192.0f,-1.18116523f,0.28838839f};
    // 2026-11-03 用户现场反馈下限太紧导致动不起来, 下限从-0.39253831放宽0.1rad到-0.49253831; 上限不变.
    config.joints[2]={app::Domain::pitch,0.61131473f,-0.49253831f,0.37193870f};
    config.dm[0].bus=current_harness.back().bus;
    config.dm[0].master_id=current_harness.back().rx_id;
    config.dm[0].mit_kp=40.0f;config.dm[0].mit_kd=1.0f;
    config.dm[0].gravity_ff_nm=0.80f;
    // 发射三个槽仅保留原地址/停止责任, 不配控制参数, 不开放发射.
    // 沿用N阶段快环的软件采样预算, 与运动输出限幅不同; 现场核对真实采样时序.
    config.perception=app::PerceptionTiming{{50000},{20000},{5000},20000};
    config.serial=integration_serial_geometry;
    config.serial_origin=app::Origin::reference_converted; // 实录拟合与现场对齐参考换算的候选。
    return config;
}
} // 命名空间 ec::sentinel::board
