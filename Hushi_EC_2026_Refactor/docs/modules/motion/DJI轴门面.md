---
date: 2026-09-17
updated: 2026-09-17
tags: [robomaster, module, axis, control]
---

# DJI 轴门面 (DjiAxis)

源码: [dji_axis.hpp](../../../include/ec/motion/dji_axis.hpp)、[dji_axis.cpp](../../../src/motion/dji_axis.cpp), 类型位于 `ec::motion`.
契约测试: `tests/axis_tests.cpp`; 端到端主机示例: [host_single_axis.cpp](../../../examples/host_single_axis.cpp) (最小) 与 [host_app_example.cpp](../../../examples/host_app_example.cpp) (多轴).

## 1. 解决什么问题

一个轴 = 一台电机 + 一条闭环 + 一种目标语义. 如果每个机器人自己拼 `DjiDevice` + `VelocityLoop` + 电流换算, 同一段换算会被抄很多遍, 而且很容易在某一处漏掉时效或授权检查.
`DjiAxis` 把这三件事合成一个对象: **模式选择一次, 每周期只设目标**, 输出是归一化执行量, 交给 [DJI 设备与分组执行](../drivers/DJI设备与分组执行.md) 编码.

它**不负责**: 控制权仲裁、机构安全动作、总线发送、位置零位标定. 设备由组装层持有, 生命周期必须长于本对象.

## 2. 模式

`control::ControlMode` 只有三种有电流上限语义的模式:

| 模式 | 目标单位 | 反馈来源 |
|---|---|---|
| `current` | A (轴侧电流) | 不闭环, 只检查限幅 |
| `velocity` | rad/s (轴侧角速度) | `DjiDevice` 换算后的轴侧速度 |
| `position` | rad (轴侧角, 相对本次参考) | 累计角, 必须已建立位置参考 |

**模式切换 = `disable()` 后重新 `enable()`**, 不提供隐式热切换; `enable()` 也是唯一重置控制器与授权的时机. GM6020 电压模式没有可比的电流上限, 需要另建电压域配置, 不在这里复用.

## 3. 配置与单位

| 字段 | 含义 |
|---|---|
| `current_limit_a` | 轴侧电流上限, 必须 > 0 且不超过设备编码上限对应电流; 归一化输出以它为基准 |
| `velocity_pid` / `position_pid` | 直接传给 [PID](../control/PID.md); 单位是 A/(rad/s) 与 (rad/s)/rad |
| `arc` | 位置模式误差语义: `continuous` (最短弧, 允许跨圈) 或 `bounded` (限制在 ±π) |
| `command_ttl` | 目标有效期, 到期即退出使能, 旧目标不复用 |
| `max_velocity_rad_s` | 位置环内部速度上限, 也是速度目标量程 |
| `max_dt_s` | 单周期最大步长, 超限拒绝而不是"按最大步长算" |
| `feedback_alpha` | 透传给速度环的反馈一阶低通系数, 0 表示不滤波 |

`direction` 与 `rotor_per_axis` 只在设备配置里维护一份, 轴不重复持有, 避免两处传动参数同时生效.

## 4. 每周期怎么用

```text
enable(mode, source, authorized, locally_ready)   // 显式状态转换, 一次
set_velocity(...) / set_position(...) / set_current(...)  // 每周期更新目标
tick(now, dt_s) -> { status, target, normalized_output }  // 计算, 不改设备状态
state(now)      -> 目标/反馈/输出的一致快照
disable()       // 立即撤销
```

任一被拒绝的周期 (`stale_target`、`stale_feedback`、`unauthorized`、`invalid_dt` …) 都会退出使能, 旧目标连同旧授权一起失效. 恢复必须重新 `enable()` 并重新下发目标; 位置模式还要先重新建立参考.

**`normalized_output` 为 `std::nullopt` 只表示"本周期禁止执行"**, 不代表机构已经停下; 减速、保持、零输出或失能由组装层显式选择并提交.

`tick_joint_velocity(now, dt_s, velocity)` 允许组装层显式提供关节速率测量, 同时保留设备的机械位置参考. 仅适用于速度/位置模式, 沿用原目标/反馈时效与授权规则, 不在外部测量失效时回退到设备RPM. 当前用于S5可选编码器测速对照, 不改变普通 `tick()` 或正式APP默认行为. `state()` 的反馈仍是设备原始解码状态; 查看当前真正用于控制的内环测量应读 `velocity_diagnostics()`.

## 5. 常见错误

- 把 `normalized_output` 当 A 或 N·m 用: 它是以 `current_limit_a` 为基准的分数, 必须经设备换算.
- 期望 `enable()` 后旧目标仍然有效: 不会, 必须重新下发.
- 在位置模式下跳过 `reference_position()`: 累计角无效, 轴会拒绝执行而不是用单圈值猜测.
- 多个轴共用一台 `DjiDevice`: 不允许, 一个设备只属于一个轴.
