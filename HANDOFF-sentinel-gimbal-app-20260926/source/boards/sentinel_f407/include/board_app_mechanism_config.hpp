#pragma once
// 云台/发射独立调试: 地址与撤输出责任保留整车, 只配齐本次测试机构.
#include "sentinel_config.hpp"

namespace ec::sentinel::board {
[[nodiscard]] constexpr app::SentinelConfig mechanism_commissioning_config(
    app::SentinelConfig config, bool gimbal) noexcept {
    constexpr std::array<std::uint8_t, 9> buses{{1,1,1,1,1,2,2,2,2}};
    constexpr std::array<std::uint8_t, 9> ids{{1,2,3,4,4,6,5,6,7}};
    for (std::size_t i=0; i<config.dji.size(); ++i) {
        auto& slot=config.dji[i];
        slot.domain=static_cast<app::Domain>(i<6 ? i : i+1);
        slot.installed=true; slot.origin=app::Origin::user_confirmed;
        slot.bus=buses[i]; slot.device_id=ids[i];
        slot.mode=(i==4 || i==5) ? drivers::DjiMode::gm6020_current :
            i==8 ? drivers::DjiMode::c610_current : drivers::DjiMode::c620_current;
    }
    auto& pitch=config.dm[0];
    pitch.domain=app::Domain::pitch; pitch.installed=true; pitch.origin=app::Origin::user_confirmed;
    pitch.bus=2; pitch.can_id=1; pitch.master_id=0x011;
    pitch.limits=drivers::DmMitLimits{12.5f,30.0f,10.0f}; pitch.feedback_ttl=Lifetime{100};
    if (gimbal) {
        for (std::size_t i=4; i<=5; ++i) {
            auto& slot=config.dji[i];
            slot.rotor_per_axis=1.0f; slot.direction=1;
            slot.current_limit_a=2.5f; slot.output_limit_raw=16384;
            slot.max_rotor_speed_rad_s=60.0f; slot.max_axis_velocity_rad_s=15.0f;
            slot.feedback_ttl=Lifetime{50};
            // 2026-10-30 参数表终版(原生归一化, 不除2.5A): main yaw 速度 0.040/0.002/0, 位置 P=25.
            slot.velocity_pid=control::PidConfig{0.040f,0.002f,0,1,-1,1};
            slot.position_pid=control::PidConfig{25.0f,0,0,15,-15,15};
        }
        // 2026-11-03 main_yaw 定版: vKp=0.05, 位置 Kp=28 + Ki=2.0 (抗陀螺反作用力矩的稳态偏移).
        config.dji[4].velocity_pid=control::PidConfig{0.050f,0.002f,0,1,-1,1};
        config.dji[4].position_pid=control::PidConfig{28.0f,2.0f,0,15,-15,15};
        // 2026-10-29 fire_yaw 自瞄(小信号跟踪)带宽调参终版. 目标是响应快且准(小范围跟瞄).
        // 判据从"阶跃超调"改为"频率响应带宽": 在中心-0.2rad做0.5/1/2Hz正弦, 测增益/相位滞后.
        // 结果(内置posKp=16, velKp原地0.09, velKi=2.0): 1Hz增益约0.95/滞后约28deg(原7.5/0.055约0.75/52),
        // 2Hz增益约0.85/滞后约49deg, 0.5Hz增益约1.0; 静止保持峰峰0.0008rad, +-0.1rad阶跃超调约1%/0%.
        // 扫参脚本用param_write直接写原生值0.09/2.00; 这里必须原样持久化, 不能再除2.5A.
        // 对应界面电流口径为Kp=0.225、Ki=5.0; posKp=20超调升到6.4%, 保留16.
        // 只覆盖fire_yaw; main_yaw保持上方初值. 实测: build/identification/pitch/fy-bandwidth-*, fy-stability-*.
        // 2026-11-03 参数表终版: fire_yaw 速度 0.16/3.0/0, 位置 P=32 Ki=0 Kd=0.1(压超调, posKp>=33自持振荡).
        // 前一条 2026-10-30 记录的 0.17/位置P28 已被 11-03 定版取代; 注释与下方代码、测试三者同口径.
        config.dji[5].velocity_pid=control::PidConfig{0.16f,3.0f,0,1,-1,1};
        config.dji[5].position_pid=control::PidConfig{32.0f,0,0.1f,15,-15,15};
        // 2026-11-03 fire_yaw MCU 电流方波辨识: 保守包络/幅值, 仅 maintenance + 新鲜目标 + 隔离时可启动.
        // require_capture=false: 采集由上位机状态页轮询完成; 越限/超时/电流超界仍在固件内自锁并撤该轴输出.
        config.dji_identification[5] = app::DjiIdentificationPolicy{
            .min_rad = -0.60f, .max_rad = 0.40f,
            .max_amplitude_a = 0.20f, .max_expected_current_a = 2.5f,
            .max_velocity_rad_s = 2.0f, .max_tracking_error_rad = 0.05f,
            .min_half_period_us = 20000u, .max_duration_us = 6000000u,
            .max_feedback_age_us = 20000u, .max_step_us = 3000u,
            .require_capture = false, .deployment_opt_in = true,
        };
        // 2026-11-03 main_yaw MCU 电流方波辨识: 连续轴无机械 min/max, 用宽包络约束, 其余由 check() 每周期执行.
        config.dji_identification[4] = app::DjiIdentificationPolicy{
            .min_rad = -3.2f, .max_rad = 3.2f,
            .max_amplitude_a = 0.20f, .max_expected_current_a = 2.5f,
            .max_velocity_rad_s = 2.0f, .max_tracking_error_rad = 0.05f,
            .min_half_period_us = 20000u, .max_duration_us = 6000000u,
            .max_feedback_age_us = 20000u, .max_step_us = 3000u,
            .require_capture = false, .deployment_opt_in = true,
        };
        constexpr float tau=6.2831853071795864769f;
        config.joints[0]={app::Domain::main_yaw,1027.0f*tau/8192.0f,{},{},false,true};
        // fire_yaw 有机械止点: 2026-10-29 遥控实测两端推杆到底机构卡死, 约 -1.2395..+0.8598 rad
        // ( -71.02..+49.26 deg, 总行程约120deg ). 改为明确机械限位, 不再是无限连续轴.
        // 限位在 zero_offset 下的关节角坐标; 留少量向内余量避免常态顶死报 fault.
        config.joints[1].domain=app::Domain::fire_yaw;
        config.joints[1].zero_offset_rad=2017.0f*tau/8192.0f;
        config.joints[1].min_rad=-1.235f;   // 实测卡死-1.2395, 向内留0.0045rad
        config.joints[1].max_rad=0.855f;    // 实测卡死+0.8598, 向内留0.0048rad
        config.joints[1].manual_continuous=false;
        config.joints[1].commissioning_unbounded=false;
        // 2026-10-30 参数表终版: fire_yaw 命令带 [-0.93, +0.479] rad (仅夹下发目标, 非机械止点).
        config.joints[1].command_min_rad=-0.93f;
        config.joints[1].command_max_rad=0.479f;
        // 2026-10-27 实车: 水平姿态在旧零位下读到 joint=-0.10452408rad,
        // 故新零位 = 旧零位 + 该读数 = 0.4732206rad, 水平姿态此后显示 0.
        // 限位由旧零位实测(下 -0.3772791 / 上 0.35172069)整体平移 +0.10452408 换算:
        // 下限采用实测下止点无余量(-0.272755rad), 避免云台受重力停在下止点报 fault;
        // 上限向内保留 0.02rad(约1.15°)缓冲余量(+0.436245rad), 防止高速抬头冲顶.
        constexpr float pitch_zero = 34282.0f * 25.0f / 65535.0f - 12.5f - 0.10452408f;
        constexpr float pitch_min = -0.3772791f + 0.10452408f;
        constexpr float pitch_max = 0.35172069f + 0.10452408f - 0.02f;
        // 2026-10-29 实车实测操作带(显式赋值, 不依赖位置初始化顺序):
        //   历史操作带[-0.132,+0.235]; 2026-11-02下界扩到-0.268以覆盖下垂启动姿态.
        //   保留机械下界-0.272755; 起始保持通过后限速抬升到经确认的采集中心.
        constexpr float pitch_command_min = -0.268f;
        constexpr float pitch_command_max = 0.235f;
        config.joints[2].domain = app::Domain::pitch;
        config.joints[2].zero_offset_rad = pitch_zero;
        config.joints[2].min_rad = pitch_min;    // 机械止点: 仅越界故障保护
        config.joints[2].max_rad = pitch_max;
        config.joints[2].manual_continuous = false;
        config.joints[2].commissioning_unbounded = false;
        config.joints[2].command_min_rad = pitch_command_min;  // 目标只允许在此带内
        config.joints[2].command_max_rad = pitch_command_max;
        // 2026-10-29 实车实测: 台架初值(35/4095*500=4.2735, kd=0)在重力负载下无法抬升.
        // 方波辨识前的抬升验证: 稳态跟随误差 = 重力力矩/Kp, 旧值约0.042rad, 超过0.03rad限值.
        // Kp=50, Kd=1.5 实测可从 -0.259rad 抬到 -0.018rad(约1.05deg重力下垂), 7.1s稳定.
        // 仍需在方波辨识与遥控验证中复核; RAM内可被上位机热改, 断电恢复为这里的值.
        // 2026-10-29 自瞄调参: 140/3.0用户反馈明显抖动, 90/2.2快速全行程时持续嗡鸣.
        // 改用已在线测试的75/2.0, 同0.6rad/s往返误差RMS约0.0167rad(90/2.2约0.0159).
        // 用户授权将75/2.0固化; 可听嗡鸣成因仍需结合现场验证, 不由100Hz角度记录推断.
        // 记录: build/identification/pitch/two-axis-diagnose-20261029-171108.
        // 2026-10-30 参数表终版: Pitch MIT Kp=70.0, Kd=2.0 (原75/2.0).
        // 2026-11-03 用户定版: Pitch MIT Kp=40, Kd=1 (RAM 验证后固化).
        pitch.mit_kp=40.0f; pitch.mit_kd=1.0f;
        // 2026-10-29 实车实测(Kp=50/Kd=1.5): -0.10/0/+0.10/+0.20rad 四点稳态保持力矩
        // 0.803/0.852/0.842/0.833N·m, 下垂恒为约0.0175rad(1°)且不随角度放大.
        // 取均值0.875N·m做常驻重力前馈, 使 MIT 在目标处不再需要靠位置误差出力顶重力,
        // 到位误差从约1°降到接近0. 该常数只在实测±0.2rad区间成立, 不是cos重力模型;
        // 力矩编码±10N·m, 远未触限. 前馈过大会把机构顶过目标, 调整须重新按此方法实测.
        // 2026-11-02 用户选择恢复重力前馈0.80N·m; 已完成限速抬升与力矩方波采集.
        pitch.gravity_ff_nm=0.80f;
    } else {
        config.shooter_commissioning=true;
        for (std::size_t i=6; i<9; ++i) {
            auto& slot=config.dji[i];
            slot.direction=1; slot.feedback_ttl=Lifetime{50};
            slot.rotor_per_axis=i==8 ? 36.0f : 1.0f;
            slot.current_limit_a=i==8 ? 8.0f : 6000.0f*20.0f/16384.0f;
            slot.output_limit_raw=i==8 ? 10000 : 16384;
            slot.max_rotor_speed_rad_s=1000.0f;
            slot.max_axis_velocity_rad_s=1000.0f/ *slot.rotor_per_axis;
            // M3508用本次已通过台架的19:1轴侧增益换算到直驱转子侧, 不抄旧摩擦轮D项.
            const float kp=i==8 ? 0.618794f : 2.657763f/19.0f;
            slot.velocity_pid=control::PidConfig{kp/ *slot.current_limit_a,0,0,1,-1,1};
        }
    }
    return config;
}
} // 命名空间 ec::sentinel::board
