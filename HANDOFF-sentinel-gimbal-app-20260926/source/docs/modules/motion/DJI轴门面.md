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

`tick_joint_velocity(now, dt_s, velocity)` 允许组装层显式提供关节速率测量, 同时保留设备的机械位置参考. 仅适用于速度/位置模式, 沿用原目标/反馈时效与授权规则, 不在外部测量失效时回退到设备RPM. 当前用于S5可选编码器测速对照, 以及显式单GM/四电机APP台架中GM6020的固定反馈选择; 不改变普通 `tick()` 或整车默认反馈源. `state()` 的反馈仍是设备原始解码状态; 查看当前真正用于控制的内环测量应读 `velocity_diagnostics()`.

### 4.1 台架的五点编码器测速

实际实现为`boards/sentinel_f407/include/gm6020_bench_velocity.hpp`的`BenchEncoderVelocity`, 是最近五个已接收样本的**线性最小二乘斜率**, 不是五点四阶差分, 不使用未来样本, 不外推到当前时刻.

1. 按8192计数/转对相邻角度差解圈, 假设相邻真实运动小于半圈; 只保留局部计数, 不累加长时间浮点多圈角.
2. 每个点使用自身CAN接收微秒时间戳, 不是按1kHz控制循环次数推时间. 记解圈计数为`q_i`, 相对最新样本的时间为`t_i`, 计算`sum((t_i-mean_t)*(q_i-mean_q))/sum((t_i-mean_t)^2) * 2π/8192`, 得到转子rad/s.
3. 等间隔T、从最旧到最新依次`q0..q4`时, 等价于`(-2*q0-q1+q3+2*q4)/(10*T) * 2π/8192`. 最新样本到达即可计算, 但代表窗口中间附近的速率: 1kHz反馈时约滞后2ms, 500Hz时约4ms. 控制任务1kHz不证明CAN反馈也是1kHz.
4. 样本时间戳标在窗口平均时间, 毫秒年龄向上取整. 不足五个点时没有新有效估计; 相同微秒时间戳不重复求导, 距上一帧达50ms则清窗重建. 失效时不暗中切回RPM.

当前入口: S5控制台有`编码器5点因果测速`复选框; **正式APP维护台没有测速源参数/切换项**, 单GM与四电机台架在`board_app.cpp`中固定把CAN1 0x208馈入该估计器, 仅用于main_yaw. M3508/M2006仍用电调原生RPM并按各自传动比换算, J4310仍用原生速度与内部MIT控制. 这不是所有电机都已启用该算法.

数学拟合可以复用, 但当前类是直驱GM6020台架实现, 不等于通用电机测速组件或已证明更优:

- M3508/M2006若接入, 必须按实际N和direction将转子速率换为轴侧速率, 各轴独立窗口; 相邻样本仍需满足半圈唯一性. 高速或丢帧后即使未到50ms也可能有多圈歧义, 不能把50ms重置门槛当作解圈保证. 转子编码器也不能直接观测齿隙/弹性引起的输出轴差异.
- J4310当前反馈是PMAX映射的输出轴位置, 不是8192计数单圈值; 不可套用此取模/比例, 更不能把Gr再除一遍. MCU算出的另一份速度也不会自动替换电调内部MIT的反馈.
- 其它电机需对比原生速度的静止噪声、低速量化、动态延迟与闭环表现后再选择; 更多点能平滑部分噪声, 同时引入延迟. 如后续开放切换, 应按轴显式选择并读回实际反馈源, 不以全局默认替换或静默回退推广. 当前GM台架已有实测与人工试用证据, 其它电机没有该反馈源的闭环验收.

## 5. 常见错误

- 把 `normalized_output` 当 A 或 N·m 用: 它是以 `current_limit_a` 为基准的分数, 必须经设备换算.
- 期望 `enable()` 后旧目标仍然有效: 不会, 必须重新下发.
- 在位置模式下跳过 `reference_position()`: 累计角无效, 轴会拒绝执行而不是用单圈值猜测.
- 多个轴共用一台 `DjiDevice`: 不允许, 一个设备只属于一个轴.
