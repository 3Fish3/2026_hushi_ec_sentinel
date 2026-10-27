---
date: 2026-09-16
updated: 2026-09-16
tags: [robomaster, platform, host]
---

# 主机平台适配 (Host Platform)

源码: [host.hpp](../../include/ec/platform/host.hpp)，位于 `ec::host` 命名空间。环境搭建与测试运行请参阅 [环境部署](../getting-started/环境部署.md) 与 [第一次构建与运行](../getting-started/第一次构建与运行.md)。

## 1. 主机适配层提供什么？

在个人电脑（Windows/Linux）上开发和测试控制算法时，我们无法调用 STM32 的硬件关中断指令（`__disable_irq()`），也无法直接使用 FreeRTOS 系统的 API。
为了让通用的核心数据结构（如通道、快照、邮箱）能在 PC 操作系统上顺利运行，主机平台层提供了基于标准 C++ 库的适配实现：
- **`ec::host::MutexCriticalSection`**：基于 C++ 标准库 `std::mutex` 实现的互斥锁策略，完整实现了核心层 `CriticalSection` 概念约束（`enter()` / `exit()`）。它允许我们在主机多线程测试中验证数据竞争与一致性。

> **能力边界认知**：
> 主机适配层**不模拟底层芯片的硬件外设**（如定时器中断优先级、DMA 控制器、引脚寄存器等）。主机契约测试验证已覆盖场景中的算法与逻辑状态机行为，不证明不存在其他缺陷，也不能直接替代真实板级微秒级的中断时序验证。

---

## 2. 主机可运行示例

下面的示例演示了在主机环境下，如何组装互斥锁策略与状态快照通道：

```cpp
#include "ec/channel.hpp"
#include "ec/platform/host.hpp"
#include <iostream>

// 主机可运行示例: 演示主机平台互斥锁与状态快照通道的组装
int main() {
    // 1. 构造主机平台的互斥锁对象 (底层封装 std::mutex)
    ec::host::MutexCriticalSection lock;

    // 2. 构造快照通道，传入互斥锁的引用
    //    该快照负责在临界区保护下同步电机的角速度样本 (float)
    ec::Snapshot<float, ec::host::MutexCriticalSection> speed_snapshot{lock};

    // 3. 发布者线程写入数据:
    //    发布一个 1.0 rad/s 的转速样本，时间戳为 100 ms，状态有效
    speed_snapshot.publish({1.0f, ec::Tick{100}, ec::Validity::valid});

    // 4. 消费者线程读取数据:
    //    从快照中原子复制出最新的样本数据
    const auto sample = speed_snapshot.read();

    // 5. 校验结果:
    //    在 101 ms 时刻核验，流逝时间 1 ms < TTL 10 ms，数据应当新鲜！
    if (sample && sample->fresh_at(ec::Tick{101}, ec::Lifetime{10}) && sample->value == 1.0f) {
        return 0; // 契约验证通过！
    }

    return 1;
}
```

---

## 3. 并发安全纪律与避坑

1. **锁与通道的生命周期顺序**：
   - 互斥锁 `lock` 必须先于通道构造，且必须在通道完全销毁之后才能析构。
   - 如果测试用例启动了多线程（`std::thread`），必须在主线程退出或销毁对象前调用 `join()` 等待所有子线程彻底结束。
2. **非递归互斥锁特性**：
   - `std::mutex` 是**非递归锁**！严禁在已经持有该锁的代码块中再次调用同一个通道对象，否则会导致线程永久自死锁。
3. **不能直接搬到 MCU 中断中使用**：
   - `std::mutex::lock()` 在遇到竞争时会引发操作系统线程上下文切换与休眠，这在单片机中断服务函数（ISR）中是绝对被禁止的！STM32 必须使用硬件关中断策略（见下节）。
