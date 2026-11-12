---
date: 2026-09-16
updated: 2026-09-16
tags: [robomaster, module, can]
---

# CAN 帧与发送接口 (CAN Transport)

源码: [can.hpp](../../../include/ec/transport/can.hpp)，位于 `ec::transport` 命名空间。STM32 板级实现见 `HalCanTx`，测试与 SIL 仿真实现见 `tests/sil_models.hpp` 中的 `CanLink`。

## 1. 为什么必须将 CAN 发送抽象为通用接口？

在传统的 STM32 电控代码中，电机驱动常常直接写死调用底层库：
```c
HAL_CAN_AddTxMessage(&hcan1, &pHeader, aData, &pTxMailbox);
```
**这种写法是造成“只能上真机调试、无法在 PC 上写测试”的罪魁祸首！**
- 它让电机算法与 STM32 HAL 库发生强耦合，导致控制算法代码根本无法在 Windows/Linux 电脑上编译运行；
- 无法在离线环境下注入故障（例如模拟 CAN 发送邮箱打满、总线断开、丢包等）。

**`CanTx` 纯虚接口的解耦价值**：
核心层定义了抽象的纯虚接口 `CanTx`：
- 在 **STM32 单片机** 上：通过 `HalCanTx` 适配器将接口对接到真实的 `hcan1` / `hcan2` 硬件句柄；
- 在 **主机开发与 SIL 仿真** 中：测试夹具注入一个轻量级的内存模拟链路 `CanLink`，在纯软件中就能完整测试电机的控制收发逻辑！

---

## 2. 经典 CAN 帧结构与返回状态语义

`CanFrame` 严格对应工业标准的经典 CAN 2.0 数据帧：
- `id`: 标准标识符（$\le 0x7FF$）；
- `size`: 载荷有效长度（$0 \le \text{size} \le 8$ 字节）；
- `data`: 固定 8 字节的内联数组，零堆开销。

### `SendResult` 发送结果枚举

| 发送状态枚举值 | 状态含义与系统认知 |
|---|---|
| **`accepted`** | **报文已成功交接给发送端**（已写入硬件邮箱或软件发送队列）。<br>⚠️ **认知红线**：这只代表“单片机已经交出报文”，**绝不代表报文已在物理双绞线上完成应答，更不代表电机已经开始转动！** |
| **`busy`** | 硬件 3 个发送邮箱全满，或软件队列已满。接口立即返回，绝不在中断或实时任务中死等阻塞。 |
| **`invalid`** | 传入的报文格式非法（如数据长度超过 8 字节、或使用了非法拓展 ID）。 |
| **`error`** | 适配器拒绝发送或底层返回失败，不等于完整的总线故障分类。当前 `HalCanTx` 不保证在每次 Bus-Off 时都返回此状态，仍需专门的错误监测与恢复设计。 |

---

## 3. 接口调用示例与安全规约

下面的代码展示了如何通过 `CanTx` 抽象接口安全下发心跳报文，并强制检查返回值：

```cpp
#include "ec/transport/can.hpp"

// 接口调用片段: 用 [[nodiscard]] 提醒检查结果，调用方仍需实际处理失败
// 参数 bus 接收任意实现了 CanTx 接口的引用 (如板级 HalCanTx 或测试用例中的 CanLink)
[[nodiscard]] ec::transport::SendResult send_heartbeat_frame(ec::transport::CanTx& bus) {
    // 1. 构造一个 2 字节的标准心跳数据帧 (ID = 0x100, 长度 = 2)
    const ec::transport::CanFrame frame{
        0x100,              // 标准 CAN 标识符
        2,                  // 有效数据长度为 2 字节
        {0xAA, 0x55}        // 负载内容
    };

    // 2. 调用非阻塞的 try_send 发送接口
    //    [[nodiscard]] 仅用于编译诊断，不自动保证调用方已正确处理结果
    const ec::transport::SendResult result = bus.try_send(frame);

    // 3. 返回发送状态供应用层状态机决策 (如 busy 时做有界降级或记录总线拥塞)
    return result;
}
```

---

## 4. 硬件所有权与并发纪律

- **单发送者原则（Single-Writer）**：
  在真实板级上，STM32 的硬件 CAN 外设句柄（如 `hcan1`）**只能由唯一的发送任务拥有并调用**！严禁在多个不同的 FreeRTOS 任务或中断中并发调用同一个 CAN 外设的发送函数。
- **槽位合并责任**：
  电机控制报文必须由总线拥有者在发送前完成同组合并（详见 [DJI 协议编解码](../drivers/DJI电机.md)），禁止各电机散乱发送造成相互覆盖。
