#pragma once
// 板级角色开关与由此推出的静态预算 (O6).
// 每个可烧录角色是**独立镜像**, 不同角色必须使用独立构建目录; 这里在编译期再挡一次
// 宏层面的混用, 使"角色组合冲突"不依赖构建脚本是否记得检查:
//  - s1:      默认非运动镜像 (S1-S4), 无执行器发送路径, 默认安装表为空;
//  - s5:      GM6020 单电机台架, 唯一允许 S5 标记的角色;
//  - capture: 只读采集 (DJI 只收, DM 只读固定参数/状态), 不驱动任何执行器;
//  - app:     正式 APP (九 DJI + 一 DM, 业务协议, 1 kHz 控制周期).
// 角色宏由构建目标定义; 未定义时在此统一为 0, 避免不同 TU 对 #if 求出不同结果.
#include <cstddef>
#include <cstdint>

// 角色宏可能被构建设置成 1/0 或 ON/OFF: 两者在 #if 里都能判定, 但在 C++ 常量表达式里
// ON/OFF 不是合法标识符. 这里统一归一化成 0/1, 使角色判定既能用于预处理也能用于 constexpr.
#ifndef EC_SENTINEL_APP
#define EC_SENTINEL_APP 0
#endif
#ifndef EC_SENTINEL_CAPTURE
#define EC_SENTINEL_CAPTURE 0
#endif
#ifndef EC_S5_GM6020_BENCH
#define EC_S5_GM6020_BENCH 0
#endif
#ifndef EC_MONITOR_REGISTRY
#define EC_MONITOR_REGISTRY 0
#endif
#ifndef EC_S1_UART6_MONITOR
#define EC_S1_UART6_MONITOR 0
#endif

#if EC_SENTINEL_APP
#define EC_ROLE_APP 1
#else
#define EC_ROLE_APP 0
#endif
#if EC_SENTINEL_CAPTURE
#define EC_ROLE_CAPTURE 1
#else
#define EC_ROLE_CAPTURE 0
#endif
#if EC_S5_GM6020_BENCH
#define EC_ROLE_S5 1
#else
#define EC_ROLE_S5 0
#endif

namespace ec::sentinel {
enum class Role : std::uint8_t { s1 = 0, s5_bench, capture, app };

// 0 表示默认的 s1 角色; 打开多于一个专用角色宏时直接编译失败.
inline constexpr int active_role_count = EC_ROLE_APP + EC_ROLE_CAPTURE + EC_ROLE_S5;
static_assert(active_role_count <= 1, "一个镜像只能有一个角色: 角色宏不得同时打开");

inline constexpr Role active_role = EC_ROLE_APP       ? Role::app
    : EC_ROLE_CAPTURE                                 ? Role::capture
    : EC_ROLE_S5                                      ? Role::s5_bench
                                                      : Role::s1;

[[nodiscard]] constexpr const char* role_name(Role role) noexcept {
    switch (role) {
    case Role::s1: return "s1";
    case Role::s5_bench: return "s5";
    case Role::capture: return "capture";
    case Role::app: return "app";
    }
    return "s1";
}

// CAN 收帧预算 (编译期常量, 不做动态扩缩).
//
// 输入率 = 已装配设备数 × 反馈率. DJI 电调按 1 kHz 反馈, 达妙 MIT 反馈同为 1 kHz;
// 正式 APP 装配九 DJI + 一 DM = 10 帧/ms = 10 000 帧/s.
//   * 队列容量 = 输入率 × 最坏服务空窗. 控制任务每 1 ms 服务一次, 队列需覆盖任务被
//     高优先级中断/临界区拖延数毫秒的情况: 4 ms × 10 帧 = 40, 取 64 (6 ms 空窗).
//   * 单周期解析上限 = 16 帧/ms = 16 000 帧/s, 是输入率的 1.6 倍, 且给每个周期的工作量
//     一个硬上界 (不做无界 drain: 持续入队时无界循环会拖长控制周期).
// 正式 APP 的设计预算 (与当前编译的角色无关): 供离线核对、文档与主机测试引用,
// 保证"APP 的预算数字"只有一个维护位置.
inline constexpr std::size_t app_control_period_ms = 1;
inline constexpr std::size_t app_link_period_ms = 10;
inline constexpr std::size_t app_can_queue_capacity = 64;
inline constexpr std::size_t app_can_frames_per_cycle = 16;
// 设备在线跟踪表: 9 DJI + 8 DM 槽位共 17, 取 24 留出未安装 ID 的余量.
// 所有非采集角色共用该容量 (板级静态装配表的声明上限与角色无关).
inline constexpr std::size_t can_watch_capacity = 24;
#if EC_ROLE_APP
inline constexpr std::size_t can_queue_capacity = app_can_queue_capacity;
inline constexpr std::size_t can_frames_per_cycle = app_can_frames_per_cycle;
#else
inline constexpr std::size_t can_queue_capacity = 32;
inline constexpr std::size_t can_frames_per_cycle = 32;
#endif
// 设备在线跟踪表的容量必须覆盖"已装配设备数 + 总线上出现的未安装 ID": 满员后拒绝
// 新 ID 并计入 dropped_ids, 不覆盖已有设备 (容量与装配上限的关系在 board_assembly.hpp 断言).
} // 命名空间 ec::sentinel
