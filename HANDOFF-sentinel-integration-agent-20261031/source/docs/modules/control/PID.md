---
date: 2026-09-16
updated: 2026-09-16
tags: [robomaster, module, pid]
---

# PID 控制器 (PID Controller)

源码: [pid.hpp](../../../include/ec/control/pid.hpp)、[pid.cpp](../../../src/control/pid.cpp)，位于 `ec::control` 命名空间。对应契约测试: `tests/core_tests.cpp` 中的 `pid_contracts`。

## 1. 算法特性与设计边界

`ec::control::Pid` 是一个高效、确定性的经典闭环控制算法组件。它接收**目标值、实际测量值与时间步长 `dt_s`**，计算并输出受限的控制量。

### 核心算法改进与工程考量
- **测量值微分 (Derivative on Measurement)，消除微分冲击 (Derivative Kick)**：
  在传统 PID 公式中，微分项是对误差求导：$D = K_d \cdot \frac{d(e)}{dt} = K_d \cdot \frac{d(\text{target} - \text{measured})}{dt}$。
  当操作手突然推到底摇杆时，`target` 会发生瞬时阶跃。若对误差直接求导，微分项会在这一瞬间产生一个巨大的冲击脉冲（微分冲击），导致电机瞬间剧烈震颤甚至打齿！
  本项目将微分项改为**对测量值求导**：$D = -K_d \cdot \frac{d(\text{measured})}{dt}$。这样能避免设定值阶跃直接进入微分项，但比例项仍可跳变，测量噪声或编码器角度回绕也可能造成微分尖峰；不保证整体输出平滑或机构稳定。
- **积分有界限幅 (Integral Clamping)**：
  积分项输出被限制在 `[-integral_limit, +integral_limit]` 范围内，约束积分累积。这是积分钳位，不是根据执行器真实饱和状态进行回算的抗饱和机制，也不证明堵转安全。
- **职责纯粹性**：
  PID 类只负责数学运算！它不负责向 CAN 发送电机报文，不判断数据是否过期，不越权决定机构故障后的急停动作。

---

## 2. 配置参数与单位契约

| 参数字段 | 物理意义与合法性约束 |
|---|---|
| **`kp`** | 比例增益，必须为有限浮点数（非 NaN、非 Inf） |
| **`ki`** | 积分增益，必须为有限浮点数 |
| **`kd`** | 微分增益，必须为有限浮点数 |
| **`integral_limit`** | 积分输出绝对值上限（已乘 `ki` 的最终积分输出上限），必须有限且 $\ge 0$ |
| **`output_min`, `output_max`** | 控制器总输出边界, 必须有限; `limit_output=true`时须满足`output_min < output_max` |
| **`limit_output`** | 默认`true`, 保持既有输出限幅; 位置外环可显式设为`false`取消软件速度输出限幅. 不改变积分钳位; 归一化电流内环不允许关闭 |

> **关键单位纪律**：
> - **输入量纲必须绝对一致**：若目标值使用标准角速度 `rad/s`，则测量值也必须是经过转换的同轴 `rad/s`，绝不能直接传入电机的原始 RPM！
> - **时间步长 `dt_s` 单位为秒**：例如 1 kHz 控制周期对应的 `dt_s` 为 `0.001f`。
> - 增益符号（正负）由控制回路的机械方向与接线极性决定，禁止靠盲目乱填负增益来掩盖底层电机反装或坐标系定义错误。

---

## 3. 主机可运行示例

下面的代码演示了 PID 的配置、步进计算与输出范围检验：

```cpp
#include "ec/control/pid.hpp"
#include <iostream>

// 主机可运行示例: 演示通用 PID 的参数配置、单步计算与限幅约束
int main() {
    // 1. 初始化 PID 参数结构体
    //    参数顺序: {kp, ki, kd, integral_limit, output_min, output_max}
    //    注意: 此处为教学演示参数，输入量纲为 rad/s，输出为归一化控制量 [-1.0, 1.0]
    const ec::control::PidConfig config{
        0.05f,  // kp: 比例增益
        0.2f,   // ki: 积分增益
        0.001f, // kd: 微分增益 (作用于测量值)
        0.3f,   // integral_limit: 积分最大贡献绝对值不得超过 0.3
        -1.0f,  // output_min: 输出绝对下限
        1.0f    // output_max: 输出绝对上限
    };

    // 2. 构造 PID 控制器对象
    ec::control::Pid pid{config};

    // 检查配置是否合法 (若 output_min >= output_max 等非法情况，configured() 为 false)
    if (!pid.configured()) {
        return 1;
    }

    // 3. 执行单步计算 step(目标值, 测量值, 实际步长 dt_s)
    //    设当前目标转速为 2.0 rad/s，实测转速为 0.0 rad/s，周期流逝 5 ms (0.005 s)
    const auto out = pid.step(2.0f, 0.0f, 0.005f);

    // 4. 校验返回值 (如果入参出现 NaN/Inf 或非正 dt，step 会重置内部状态并返回 std::nullopt)
    if (!out) {
        return 2; // 计算异常
    }

    // 5. 校验输出是否严格落在限幅范围 [-1.0, 1.0] 之内
    if (*out >= -1.0f && *out <= 1.0f) {
        return 0; // 计算成功！
    }

    return 3;
}
```

---

## 4. 运行期调用、失效保护与复位规则

- **传入真实的流逝时间 `dt_s`**：
  不要未经依据就假定 `dt_s` 为常数；只有固定步长确实成立（如确定性 SIL）时才可使用常量。如果因调度抖动导致实际两次计算间隔了 2ms，却依然向 PID 传入 1ms，微分项的斜率估计就会产生成倍的误差。
- **单任务拥有权（非线程安全）**：
  PID 内部持有历史积分值 `integral_` 与上一周期的测量值 `previous_measured_`，它属于有状态对象，**必须由单一控制任务独占访问，严禁跨任务并发调用**。
- **`reset()` 复位时机**：
  当电机重新上电使能、或者控制回路从断链急停中恢复时，必须显式调用 `pid.reset()`。这会清空历史累积积分与微分记忆，但不等于实现无扰切换；重新切入时的目标、测量和执行器状态仍需由应用协调。
- **返回空值（`std::nullopt`）的处理**：
  当 PID 返回空值时，绝不能当作“0 输出”盲目发给电机，上层控制链必须及时切断执行许可并触发报警。

若需要将时效性核验、代次授权门与 PID 闭环完整结合，请阅读 [速度控制链](速度控制链.md)！(￣▽￣)／
