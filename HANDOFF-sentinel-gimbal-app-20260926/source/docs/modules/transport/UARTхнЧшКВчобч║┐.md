---
date: 2026-09-16
updated: 2026-09-16
tags: [robomaster, module, uart, dma]
---

# UART 环形字节管线 (Byte Pipe)

源码: [byte_pipe.hpp](../../../include/ec/transport/byte_pipe.hpp)，位于 `ec::transport` 命名空间。对应契约测试: `tests/core_tests.cpp` 中的 `byte_pipe_contracts`。

## 1. 为什么需要软件游标管线来管理环形 DMA？

在 STM32 的高性能串口开发中，最经典的接收架构是 **“DMA 循环模式 (Circular Mode) + 串口空闲中断 (IDLE)”**：
- 硬件 DMA 控制器在后台默默地把串口收到的字节一个个填入内存中的环形缓冲区（Ring Buffer），全程不占用 CPU；
- 当一帧数据发完并产生空闲中断时，CPU 醒来读取数据。

**初学者最容易踩的坑：DMA 正在读写，软件该怎么安全地把数据拷出来？**
如果直接通过两个简单的全局变量指针去索引缓冲区，极易发生以下险情：
1. **未读数据被 DMA 悄无声息地覆盖**（数据溢出回绕）；
2. **读写指针竞争**：中断更新了写位置，但主循环读到一半被打断，造成指针错乱；
3. **缺少丢包诊断**：根本不知道线上到底丢失了多少字节。

`DmaBytePipe<N>` 专门为解决此问题而设计：它维护一块固定容量为 $N$ 的内联连续内存，并提供确定性的软件读写游标。

---

## 2. 核心工作机制与溢出防线

- **写游标推进 (`on_dma_write`)**：
  在硬件中断中，通过查询 DMA 计数器寄存器（如 STM32 的 `NDTR` 递减寄存器）换算出当前的绝对写入偏移量并调用该函数。
- **批量安全复制 (`read`)**：
  消费者传入目标缓冲区 `std::span<uint8_t>`，管线计算当前已公布但尚未读取的字节量并完成拷贝，随后自动推进读游标。
- **溢出防御**：
  当已公布的游标增量表明新数据将覆盖未读区时，管线会**丢弃受损未读区，并递增 `overflows()` 计数器**。这不是对任意调度条件下所有覆盖事件的检测保证。

> **接收成立的前提**：所有游标读写必须互斥，可由同优先级 ISR 独占消费，或由任务在短临界区中复制。必须及时处理 HT/TC，不能漏掉整圈：相同写游标无法区分“没有新数据”和“已绕了一圈”。屏蔽 CPU 中断不会停止 DMA，复制必须在数据被覆盖前完成；缓冲容量与最坏消费延迟仍需验证。管线不保存协议帧边界，DR16 还需保留真实 IDLE 信息。

---

## 3. 主机可运行示例

下面的示例演示了模拟 DMA 写入、游标更新、安全读取以及复位操作：

```cpp
#include "ec/transport/byte_pipe.hpp"
#include <iostream>

// 主机可运行示例: 演示 DmaBytePipe 环形缓冲区的 DMA 写入同步与安全提取
int main() {
    // 1. 创建固定容量为 64 字节的环形 DMA 字节管线
    ec::transport::DmaBytePipe<64> pipe;

    // 2. 模拟硬件 DMA 动作:
    //    将数据直接搬运到底层缓冲区 pipe.dma_buffer() 的第 0 个位置
    //    (在实际板级开发中，pipe.dma_buffer() 会直接作为 HAL_UART_Receive_DMA 的目标地址)
    pipe.dma_buffer()[0] = 42;

    // 3. 模拟中断触发: 通知管线 DMA 写指针已经推进到了索引位置 1
    pipe.on_dma_write(1);

    // 此时管线中可读取的有效字节数应当恰好为 1
    if (pipe.available() != 1) {
        return 1;
    }

    // 4. 消费者读取数据:
    //    提供一个本地目标数组，调用 read(dst) 提取数据
    std::uint8_t output_buffer[8]{};
    const std::size_t bytes_read = pipe.read(output_buffer);

    // 校验读取结果: 读取字节数必须为 1，且提取出的数值必须是 42
    if (bytes_read != 1 || output_buffer[0] != 42) {
        return 2;
    }

    // 5. 模拟复位管线 (在硬件发生错误重启 DMA 时调用)
    //    注意: 在真实板级中，调用 reset() 前必须先确认硬件 DMA 已经完全停止！
    pipe.reset();

    // 复位后可读字节数应清零
    if (pipe.available() == 0) {
        return 0; // 契约验证通过！
    }

    return 3;
}
```

---

## 4. 硬件排错与关键诊断指示

`DmaBytePipe` 提供了丰富的诊断计数接口，帮助你在调试中快速抓出硬件问题：
- `pipe.received()`: 软件游标累计确认接收到的字节总量。
- `pipe.overflows()`: 环形缓冲区被追尾覆盖的溢出事件总数（若该数字大于 0，说明消费任务周期太慢或 DMA 缓冲区开得太小）。
- `pipe.discarded()`: 溢出或重置时丢弃的已知未读字节量，不是线上全部丢字节的精确计数。
- `pipe.note_error(error)`: 逐项传入 `UartError::framing/noise/overrun`，不是传入 HAL 错误位掩码；这些计数由调用方记录，不自动读取硬件寄存器。

重启前须确认 RX DMA 已停止且将从起点启动，并同步清理上层半帧。TX 错误不一定意味着 RX 已停止，不能无条件重置仍在运行的 RX 管线。

同文件中的 `ByteTxQueue` 同样没有内部锁：`try_push()` 空间不足时拒绝整段，`peek()` 只暴露当前连续块，DMA 完成后才可 `pop()`；不能提前复用仍被 DMA 引用的内存。

## 5. 在字节管线之上的封装

- [`UartFrameDecoder`](../../../include/ec/transport/uart_frame.hpp): 单消费者逐字节调用 `push()`; 仅完整且版本、长度、CRC 合法时返回 `UartFrame`. `reset()` 清半帧而保留拒绝计数; `encode_uart_frame()` 写入调用者缓冲, 空间不足返回 0. 不提供可靠交付、身份认证或命令授权.
- [`UartDmaPort<RxCapacity, TxCapacity>`](../../../platform/stm32f407/include/ec/platform/uart_dma.hpp): STM32 单任务端口. `read()` 返回字节数和重启/溢出代次; `try_send()` 使用单帧自有缓冲, 忙时不排队; `stats()` 取一致计数. `start()` 只在初始化或 DMA 停止后使用, 错误由 IRQ 标记后在任务的 `poll_recovery()` 中终止并重启. 装配者仍须接上 UART/HT/TC/TX/错误回调并保证中断优先级与复制时限.

完整板级接入、只读协议和 Python 使用方式见 [串口观测与应用接入](../../guides/串口观测与应用接入.md). 不能把这些传输能力视作机器人应用层已经完成.
