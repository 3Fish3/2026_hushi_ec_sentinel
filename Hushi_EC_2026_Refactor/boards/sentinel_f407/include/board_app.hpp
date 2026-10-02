#pragma once
// 正式 APP 角色 (EC_SENTINEL_APP=1) 的板级接入 (O6).
// 职责: 在调度器启动前构造配置/应用/执行/感知/参数/会话对象; 在 1 kHz 控制任务里完成
// "有界收帧 → 有界 USART1 搬运 → 组装输入 → 应用仲裁 → 执行与提交"; 每 10 个控制周期
// 发布一次监控注册表. 本层不实现控制算法、不做权限判定, 也不解析业务载荷.
//
// 单写者原则: 应用对象/设备/协议会话/注册表发布全部只由控制任务触碰, 因此不需要
// 跨任务锁; 10 ms 任务只做 USART3 遥控接收、USART6 监控服务与外设统计.
//
// 静态预算 (编译期常量, 不做动态扩缩; 实测 WCET/栈高水位待上板记录):
//  - 控制周期 1 ms: CAN 收帧上限 can_frames_per_cycle, 一次 SPI IMU 读,
//    USART1 搬运上限 uart1_poll_budget 字节, 每周期最多 app_requests_per_cycle 条业务请求;
//  - 慢客户端只造成 usart1_tx 有界拒绝并计数, 不阻塞控制任务、不无限增长队列.
#include "board_app_observation.h"
#include "board_role.hpp"
#include "ec/time.hpp"
#include "ec/transport/can.hpp"
#include <cstddef>
#include <cstdint>
#include <span>

#if EC_SENTINEL_APP
namespace ec::sentinel {
// 单个控制周期内最多处理的业务请求数: 1 kHz × 4 远超任何真实主机请求率,
// 同时给"一个周期内的协议工作量"一个硬上界 (超出部分丢弃并计数).
inline constexpr std::size_t app_requests_per_cycle = 4;
// 监控注册表/观测发布周期 (控制周期数): 1 kHz / 10 = 100 Hz, 不在每个 1 ms 周期复制整表.
inline constexpr std::uint32_t app_publish_period_cycles = 10;
// 明确的 APP 串口配置, 尚待线束/适配器实测; 降速需同步修改主机 --baud, 不自动切换.
inline constexpr std::uint32_t app_uart_baud = 921600;
inline constexpr std::uint8_t app_fast_request = 0x36, app_fast_response = 0xb6;

// 调度器启动前调用一次: 校验部署配置, 构造应用/执行/感知/参数/会话与观测区.
// 配置不齐备时仍照常构造并如实报告缺项 (不授予任何执行资格), 不静默使用合成配置.
void ec_app_init() noexcept;
// 1 kHz 控制任务体 (由 freertos.c 的 appControl 任务调用).
void ec_app_control_tick() noexcept;
// 板级 USART1 接收路径调用 (控制任务上下文): 投递收到的字节.
void ec_app_link_bytes(std::span<const std::uint8_t> bytes) noexcept;
// CAN 收帧路由目标: 正式 APP 的接收设备属于应用执行层, 不属于板级静态装配层.
void ec_app_route(std::uint8_t bus, const transport::CanFrame& frame, Tick at,
    MicroTick at_us) noexcept;
// 最近一份应用状态观测 (控制任务写, 注册表/观测读).
[[nodiscard]] const EcAppObservation& ec_app_observation() noexcept;
// 初始化结论与缺项数: 供启动阶段事件记录与离线核对.
[[nodiscard]] bool ec_app_ready() noexcept;
[[nodiscard]] std::size_t ec_app_config_missing_items() noexcept;
[[nodiscard]] std::size_t ec_app_harness_dji_count() noexcept;
} // 命名空间 ec::sentinel
#endif
