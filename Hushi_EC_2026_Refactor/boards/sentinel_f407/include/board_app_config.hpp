#pragma once
// 正式 APP 的部署配置表 (O6).
// 本文件只使用 ec::sentinel::app 的配置类型, 不含 HAL/板级类型, 因此同一份表既能编进
// F407 固件, 也能被主机测试与离线检查脚本直接核对, 避免"测试用的表"和"烧录的表"分叉.
//
// 当前状态: **没有可用的实车配置**. 已确认的只有接线 (哪条总线/哪种协议/哪个设备 ID),
// 而机构分配、传动比、方向、电流上限、速度上界、关节零位与机械范围、DM 量程与保存设置
// 都还没有实测依据. 因此 deployment_config() 只声明配置类别与软件时序策略, 不装配任何
// 设备槽: 校验结论是"配置不齐备", 每个域都是未配置, 不授予任何执行资格.
// 合成夹具 (app::synthetic_config()) 只用于主机离线测试, 绝不作为本镜像的默认配置.
#include "sentinel_config.hpp"
#include "sentinel_remote.hpp"
#include <array>
#include <cstdint>

namespace ec::sentinel::board {
// 已确认接线 (2026-10-27 实车只读采集与用户接线确认, 与 board_observation.hpp 的 dji_mode
// 一致): CAN1 四轮 C620 (反馈 0x201..0x204) 与主 yaw GM6020 (反馈 0x208); CAN2 发射 yaw
// GM6020 (0x20a)、两摩擦轮 C620 (0x205/0x206) 与 ID7 的 C610 (0x207).
// 这里只记录已验证的总线/协议/设备 ID。
// **不含设备到机构的分配**: 哪颗电机接哪个轮/哪个 yaw/哪个摩擦轮属于尚未确认的机械事实,
// 不在此处臆测, 因此这张表不是配置, 只是待确认清单的机器可读形式。
struct HarnessEntry {
    std::uint8_t bus{};
    drivers::DjiMode mode{drivers::DjiMode::c620_current};
    std::uint8_t device_id{};
};
inline constexpr std::array<HarnessEntry, 9> known_dji_harness{{
    {1, drivers::DjiMode::c620_current, 1},    // 反馈 0x201
    {1, drivers::DjiMode::c620_current, 2},    // 反馈 0x202
    {1, drivers::DjiMode::c620_current, 3},    // 反馈 0x203
    {1, drivers::DjiMode::c620_current, 4},    // 反馈 0x204
    {1, drivers::DjiMode::gm6020_current, 4},  // 反馈 0x208
    {2, drivers::DjiMode::gm6020_current, 6},  // 反馈 0x20a
    {2, drivers::DjiMode::c620_current, 5},    // 反馈 0x205
    {2, drivers::DjiMode::c620_current, 6},    // 反馈 0x206
    {2, drivers::DjiMode::c610_current, 7},    // 反馈 0x207
}};
// 达妙 pitch 在 CAN2, 控制 ID1 / 反馈 ID 0x011 已由只读参数回复读回.
inline constexpr bool known_dm_harness = true;

// 软件时序策略 (不是实测机械/电气参数): 控制周期由板级 1 kHz 任务决定, 遥控/IMU 时效
// 沿用板级既有口径 (board_inputs.hpp), 目标与上位机链路时效是首版策略默认值, 待与
// 上位机回传率核对后确认.
inline constexpr float app_control_dt_s = 0.001f;
// 单次控制步长的上限 (ms): 超过它按超期计数; 真实 dt 仍照传给轴/环, 由它们既有的
// max_dt 拒绝路径处理, 不用标称周期冒充正常采样.
inline constexpr std::uint32_t app_max_dt_ms = 5;

// 单个控制周期的真实步长决策 (纯函数, 便于用受控时钟覆盖边界).
//  - 首次周期: 用明确的初始化步长 (不把合法 Tick=0 当哨兵);
//  - 重复时刻 (elapsed==0) 或时间回退 (半个回绕窗口外的倒退): dt_s = 0,
//    交给轴/环已有的 dt 拒绝/不推进路径, 不伪装成标称周期;
//  - 否则用真实毫秒差, 并单独给出超期标记供统计.
struct AppControlStep {
    float dt_s{app_control_dt_s};
    bool overrun{};
};
[[nodiscard]] constexpr AppControlStep app_control_step(
    std::uint32_t now_ms, std::uint32_t last_ms, bool have_last) noexcept {
    AppControlStep out{};
    if (!have_last) { return out; }
    const std::uint32_t elapsed_ms = now_ms - last_ms;
    if (elapsed_ms == 0u || elapsed_ms >= 0x80000000u) {
        out.dt_s = 0.0f;
        return out;
    }
    out.overrun = elapsed_ms > app_max_dt_ms;
    out.dt_s = static_cast<float>(elapsed_ms) * 0.001f;
    return out;
}

// 正式 APP 控制使用微秒差; 毫秒版本保留给已有观测/离线入口.
[[nodiscard]] constexpr AppControlStep app_control_step_us(
    std::uint32_t now_us, std::uint32_t last_us, bool have_last) noexcept {
    if (!have_last) { return {}; }
    const auto elapsed = now_us - last_us;
    if (elapsed == 0 || elapsed >= 0x80000000u) { return {0.0f, false}; }
    return {static_cast<float>(elapsed) * 1e-6f, elapsed > app_max_dt_ms * 1000u};
}

// 本地操纵映射量程/档位阈值 (C06). 这些是**实测项** (拨杆量程与 ch4 档位实测值), 因此
// 部署配置里全部为 0: 0 不是"保守值"而是"未配置" —— 本地操纵因此不产生运动 (目标量程 0),
// 直到实测回填. 不在这里拄一个看起来合理的数字冒充已标定.
[[nodiscard]] constexpr app::RemoteMappingConfig deployment_mapping() noexcept {
    return app::RemoteMappingConfig{};
}
inline constexpr std::uint32_t app_target_ttl_ms = 100;
inline constexpr std::uint32_t app_host_link_ttl_ms = 200;
inline constexpr float app_max_dt_s = 0.005f;

// 部署配置: 只有类别与上述软件策略, 没有任何装配槽.
[[nodiscard]] constexpr app::SentinelConfig deployment_config() noexcept {
    app::SentinelConfig config{};
    config.kind = app::ConfigKind::real_deployment;
    config.timing.control_dt_s = app_control_dt_s;
    config.timing.target_ttl = Lifetime{app_target_ttl_ms};
    config.timing.remote_ttl = Lifetime{100};
    config.timing.imu_ttl = Lifetime{50};
    config.timing.host_link_ttl = Lifetime{app_host_link_ttl_ms};
    config.timing.max_dt_s = app_max_dt_s;
    return config;
}

// 部署配置的静态摘要: 固定布局、无指针, 供离线检查脚本从未压缩镜像按符号读取,
// 核对镜像里不是合成配置、也不含"装配了但来源未知"的槽位.
inline constexpr std::uint32_t deployment_summary_dji_slots = 9;
struct DeploymentSummary {
    std::uint32_t magic;
    std::uint32_t layout;
    std::uint32_t kind;              // app::ConfigKind 的数值
    std::uint32_t dji_capacity;      // 配置可声明的 DJI 槽数
    std::uint32_t dm_capacity;
    std::uint32_t installed_dji;     // 已声明装配的 DJI 槽数
    std::uint32_t installed_dm;
    std::uint32_t synthetic_values;  // 部署配置里的合成来源值 (必须为 0)
    std::uint32_t unknown_origin_installed; // 装配但来源未知 (部署用途拒绝)
    std::uint32_t harness_dji;       // 已验证接线条目数
    std::uint32_t harness_dm;
    // 每槽实际总线/设备 ID: bus | device_id<<8, 0 表示该槽未装配.
    // 只报装配数量时, "底盘挂错总线"这类镜像照样能通过审计; 地址进摘要才能离线核对.
    std::uint32_t dji_address[deployment_summary_dji_slots];
    std::uint32_t dm_address;        // bus | can_id<<8 | master_id<<16
};
inline constexpr std::uint32_t deployment_summary_magic = 0x45434150u; // "ECAP"
inline constexpr std::uint32_t deployment_summary_layout = 2; // 2: 增补每槽地址表

[[nodiscard]] constexpr DeploymentSummary summarize(const app::SentinelConfig& config) noexcept {
    DeploymentSummary summary{deployment_summary_magic, deployment_summary_layout,
        static_cast<std::uint32_t>(config.kind),
        static_cast<std::uint32_t>(config.dji.size()),
        static_cast<std::uint32_t>(config.dm.size()), 0, 0, 0, 0,
        static_cast<std::uint32_t>(known_dji_harness.size()), known_dm_harness ? 1u : 0u,
        {}, 0}; // 地址表与 DM 地址先清零, 下面按实际装配填写
    std::uint32_t slot_index = 0;
    for (const auto& slot : config.dji) {
        const std::uint32_t index = slot_index++;
        if (!slot.installed) { continue; }
        ++summary.installed_dji;
        if (slot.origin == app::Origin::synthetic) { ++summary.synthetic_values; }
        if (slot.origin == app::Origin::unspecified) { ++summary.unknown_origin_installed; }
        if (index < deployment_summary_dji_slots) {
            summary.dji_address[index] = static_cast<std::uint32_t>(slot.bus) |
                (static_cast<std::uint32_t>(slot.device_id) << 8);
        }
    }
    for (const auto& slot : config.dm) {
        if (!slot.installed) { continue; }
        ++summary.installed_dm;
        if (slot.origin == app::Origin::synthetic) { ++summary.synthetic_values; }
        if (slot.origin == app::Origin::unspecified) { ++summary.unknown_origin_installed; }
    }
    if (!config.dm.empty() && config.dm[0].installed) {
        summary.dm_address = static_cast<std::uint32_t>(config.dm[0].bus) |
            (static_cast<std::uint32_t>(config.dm[0].can_id.value_or(0)) << 8) |
            (static_cast<std::uint32_t>(config.dm[0].master_id.value_or(0)) << 16);
    }
    return summary;
}

inline constexpr DeploymentSummary deployment_summary = summarize(deployment_config());
static_assert(deployment_summary.magic == deployment_summary_magic);
static_assert(deployment_summary.kind == static_cast<std::uint32_t>(app::ConfigKind::real_deployment),
    "部署配置不得声明为合成夹具或未配置");
static_assert(deployment_summary.synthetic_values == 0, "部署配置不得含合成来源的值");
static_assert(deployment_summary.installed_dji == 0 && deployment_summary.installed_dm == 0,
    "尚无实车参数: 部署配置不得声明任何装配槽");
static_assert(deployment_summary.dji_capacity >= known_dji_harness.size(),
    "已验证接线条目数不得超过配置可声明的 DJI 槽数");
} // 命名空间 ec::sentinel::board

extern "C" {
// 部署摘要符号: 单一定义在包含本头文件的板级 TU (固件: board_app.cpp; 主机: 部署配置测试).
// 外部链接 + used 保证它被发出; 读取方必须经 volatile 指针读, 否则 Release 下引用会被
// 常量折叠, 链接器就会把符号 GC 掉 (镜像里就没有可读的部署摘要了).
// 因此本头文件不得被同一二进制里的多个 TU 同时包含.
extern const ec::sentinel::board::DeploymentSummary ec_app_deployment_summary;
[[gnu::used]] const ec::sentinel::board::DeploymentSummary ec_app_deployment_summary =
    ec::sentinel::board::deployment_summary;
}
