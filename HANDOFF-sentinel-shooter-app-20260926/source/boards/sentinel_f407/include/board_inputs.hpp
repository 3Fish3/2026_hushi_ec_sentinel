#pragma once
#include "ec/drivers/bmi088_device.hpp"
#include "ec/drivers/dr16.hpp"
#include "ec/time.hpp"
#include <optional>

// 板级输入交接: 把接收路径解出的**完整**样本 (含鼠标/按键/拨杆/通道) 和 IMU 样本
// 交给控制任务, 样本自带采样时刻与有效性. APP 因此不必重新解析遥控协议, 也不必
// 直接拼读跨上下文的诊断浮点字段.
//
// 职责边界:
//  - 单写者(接收/采集路径)与单读者(控制任务)各一个槽位, 用序数校验避免读到半更新;
//  - 本层不做仲裁、不做失联保护动作, 只回答"最近一次样本是什么、是否仍在 TTL 内";
//  - 不新增全局 HAL 回调或第二套 IO 注册: board_io.cpp 的既有路径在解出样本后调用
//    input_publish_* 即可.
namespace ec::sentinel {
// 与板级诊断同一套时效口径: 遥控 100 ms, IMU 50 ms.
inline constexpr ec::Lifetime kRemoteTtl{100};
inline constexpr ec::Lifetime kImuTtl{50};

// 接收路径调用 (USART3 的 IDLE/错误处理里, 中断上下文): 发布一份完整遥控样本.
void input_publish_remote(const ec::drivers::Dr16Sample& sample, ec::Tick at) noexcept;
// 接收路径调用: 分帧器溢出或复位后, 之前那份样本不再代表当前输入.
void input_remote_lost() noexcept;
// 采集路径调用 (任务上下文): 发布一份 IMU 样本.
void input_publish_imu(const ec::drivers::Bmi088Sample& sample, ec::Tick at) noexcept;
void input_publish_imu_reading(const ec::drivers::Bmi088Reading& sample) noexcept;
[[nodiscard]] std::optional<ec::Sample<ec::drivers::Bmi088Reading>> input_imu_reading(ec::Tick now) noexcept;

// 控制任务调用: 最近一份样本. 从未收到或已明确失联时返回 nullopt;
// 收到过但超出 TTL 时返回 validity = invalid 的样本 (时间戳保留, 由调用方决定动作).
[[nodiscard]] std::optional<ec::Sample<ec::drivers::Dr16Sample>> input_remote(ec::Tick now) noexcept;
[[nodiscard]] std::optional<ec::Sample<ec::drivers::Bmi088Sample>> input_imu(ec::Tick now) noexcept;

// 只用于启动/测试: 清空两个槽位.
void input_reset() noexcept;
} // 命名空间 ec::sentinel
