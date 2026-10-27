#pragma once
#include "board_role.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/drivers/dm_device.hpp"
#include "ec/time.hpp"
#include "ec/transport/can.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

// 弱符号钩子在 GCC/Clang 工具链 (固件) 上生效, APP 可直接强定义覆盖;
// MSVC 没有等价机制, 主机测试里钩子为普通强定义 (测试不覆盖它们).
#if defined(__GNUC__) || defined(__clang__)
#define EC_BOARD_WEAK __attribute__((weak))
#else
#define EC_BOARD_WEAK
#endif

namespace ec::sentinel {
// 板级静态装配: 安装表 → 设备实例 → 接收路由 → 控制/提交钩子.
// 职责边界:
//  - 中断只做有界入队与时间打点 (见 board_io.cpp 的 CAN/UART 回调);
//  - 本文件在任务上下文里逐帧路由并更新设备状态, 不解析协议细节, 每周期解析量有硬上界;
//  - 控制与执行器发送由 APP 提供的钩子完成, 默认钩子不下发任何输出.
// 安装表在下方 board_assembly.cpp 中填写; 默认镜像为空表, 因此默认镜像没有任何
// 电机发送路径 (check_firmware.py 会核对这一点).
// 九 DJI 槽位是正式 APP 的机构容量 (四轮 + 主/从 yaw + 两摩擦 + 拨弹); 监控注册表的
// 设备记录 ID 由本常量推导 (DJI 为 0..dji_install_max-1, DM 从 dji_install_max 开始),
// 因此改这里即同步监控 ID 与 DM 偏移, 不需要另一张表.
inline constexpr std::size_t dji_install_max = 9;
inline constexpr std::size_t dm_install_max = 8;
// 在线跟踪表容量必须覆盖全部装配槽位 (含未安装但出现在总线上的 ID 需另留余量).
static_assert(can_watch_capacity >= dji_install_max + dm_install_max,
    "设备在线跟踪表容量必须覆盖 DJI 与 DM 装配上限之和");
// 收帧任务单周期最多解析的 CAN 帧数 (= 接收队列容量). 超出的帧留到下一周期,
// 队列自身溢出仍计入 rx_dropped: 单周期工作量因此有硬上界, 不随总线负载增长.
// 角色相关的取值与推导见 board_role.hpp.

enum class AssemblyStatus : std::uint8_t {
    empty,       // 安装表为空: 未装配任何电机
    ready,       // 表有效, 设备与路由已建立
    too_many,    // 条目数超过上限
    invalid_dji, // DJI 布局无效 (重复槽位/同总线反馈 ID 冲突等)
    invalid_dm,  // DM 布局无效 (同总线同 Master ID 下节点字段冲突等)
    id_conflict, // DJI 与 DM 在同一总线上的接收 ID 归属冲突
};

struct AssemblyState {
    AssemblyStatus status{AssemblyStatus::empty};
    std::size_t dji_count{};
    std::size_t dm_count{};
    std::size_t routes{};
    std::uint32_t routed{};   // 已交给设备的反馈帧
    std::uint32_t unrouted{}; // 未命中安装表的反馈帧 (诊断用, 不代表错误)
};
[[nodiscard]] const AssemblyState& assembly_state() noexcept;
// 只在启动上下文调用一次; 布局无效时状态里给出明确原因, 不安装任何设备.
// 无参版本使用本板的安装表; 带参版本供 APP/测试直接传入表, 行为完全一致.
void assembly_init() noexcept;
void assembly_init(std::span<const ec::drivers::DjiDeviceConfig> dji,
    std::span<const ec::drivers::DmDeviceConfig> dm) noexcept;
// 已安装设备的访问 (表无效或为空时返回空指针).
// 返回可写指针: APP 需要用非 const 设备构造轴/发送器并更新位置参考与使能请求;
// 但控制策略、仲裁与发送调度仍由 APP 负责, 本层只保证设备实例与接收路由的唯一性.
[[nodiscard]] ec::drivers::DjiDevice* dji_device(std::size_t index) noexcept;
[[nodiscard]] ec::drivers::DmDevice* dm_device(std::size_t index) noexcept;
// 由收帧任务逐帧调用 (bus: 1 = CAN1, 2 = CAN2).
void assembly_route(std::uint8_t bus, const ec::transport::CanFrame& frame, ec::Tick at) noexcept;
// 每个控制周期调用一次: 先控制, 后提交.
void assembly_step(std::uint32_t now_ms) noexcept;

// APP 覆写点 (弱符号). 默认实现什么都不做: 默认镜像没有执行器输出路径.
// 控制钩子内可读设备状态并计算输出; 提交钩子负责把输出交给组发送器.
extern "C" void ec_board_app_control(std::uint32_t now_ms);
extern "C" void ec_board_app_commit(std::uint32_t now_ms);
} // 命名空间 ec::sentinel
