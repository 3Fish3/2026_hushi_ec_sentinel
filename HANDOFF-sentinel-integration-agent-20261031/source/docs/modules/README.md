---
date: 2026-09-16
updated: 2026-09-16
tags: [robomaster, module, navigation]
---

# 模块索引

先按问题选组件, 再看对应契约. 文件名不是 C++ 命名空间: 例如 `channel.hpp` 中的类型位于 `ec`, 并没有 `ec::channel` 命名空间.

| 问题 | 组件说明 |
|---|---|
| 数据是否过期? | [时间与时效](core/时间与时效.md) |
| 周期超时后如何跳过旧周期? | [周期规划](core/周期规划.md) |
| 如何跨任务传目标、反馈和事件? | [通道与数据交换](core/通道与数据交换.md) |
| 如何撤销本轮执行许可, 拒绝旧目标? | [授权门](core/授权门.md) |
| 如何计算 PID 输出? | [PID](control/PID.md) |
| 如何把时效、授权和 PID 接成速度控制链? | [速度控制链](control/速度控制链.md) |
| 位置控制怎么串到速度环上? | [位置串级](control/位置串级.md) |
| IMU 加热占空比怎么算? | [温度控制](control/温度控制.md) |
| 一个轴怎么把设备、闭环和模式合成一个对象? | [DJI 轴门面](motion/DJI轴门面.md) |
| `vx/vy/wz` 怎么换算到各轮角速度? | [全向轮运动学](motion/全向轮运动学.md) |
| 怎么把加速度和角速度融合成姿态? | [姿态估计](estimation/姿态估计.md) |
| 会话、控制权和执行许可分别管什么? | [会话、控制权与执行上下文](core/会话与执行上下文.md) |
| 整机状态怎么同时给串口和 SWD 看? | [监控注册表](monitor/监控注册表.md) |
| 如何交接 UART DMA 字节或缓存发送数据? | [UART 字节管线](transport/UART字节管线.md) |
| 怎样与具体 CAN 硬件解耦? | [CAN 接口](transport/CAN接口.md) |
| 怎样编解码 DJI 电机帧? | [DJI 电机](drivers/DJI电机.md) |
| 怎样转换轴侧反馈、管理位置参考与合并控制槽位? | [DJI 设备与分组执行](drivers/DJI设备与分组执行.md) |
| 怎样按 IDLE 边界解析遥控数据? | [DR16](drivers/DR16遥控.md) |
| 怎样解析 IMU 响应并换算单位? | [BMI088](drivers/BMI088惯导.md) |
| 如何记录总线设备最近出现时间? | [设备在线监视](drivers/设备在线监视.md) |

完整的主机组装可运行示例见 [host_assembly.cpp](../../examples/host_assembly.cpp) (单轴链)、[host_single_axis.cpp](../../examples/host_single_axis.cpp) (最小单轴入门) 与 [host_app_example.cpp](../../examples/host_app_example.cpp) (四轮 + DM 关节 + 遥控 + IMU + 控制权 + 监控); 接入步骤见 [应用装配与接入指南](../guides/应用装配与接入.md). 电机闭环和故障夹具见 [sil_tests.cpp](../../tests/sil_tests.cpp); 其中模拟参数和零电流故障动作不能直接部署到实车.
