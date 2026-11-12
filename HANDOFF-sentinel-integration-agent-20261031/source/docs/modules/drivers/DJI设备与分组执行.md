---
date: 2026-09-16
updated: 2026-09-16
tags: [robomaster, module, dji, feedback, execution]
---

# DJI 设备反馈与分组执行

源码: [dji_device.hpp](../../../include/ec/drivers/dji_device.hpp)、[dji_device.cpp](../../../src/drivers/dji_device.cpp), 类型位于 `ec::drivers`. 协议字节定义仍由 [DJI 编解码](DJI电机.md) 维护.

## 1. 为什么编码器和 PID 之间还需要这一层?

假设轮子转一圈需要电机转二十圈, 而电机安装方向与轮子正方向相反. 协议解码出的转子 rad/s 不能直接当轮子 rad/s 使用. 同样, 四个电机共用一帧控制报文, 也不能各自发送一帧把其他槽位清零.

本模块补齐这两段胶合逻辑:
- `DjiDevice`: 按总线和 ID 接收反馈, 换算轴侧速度, 管理显式位置参考与累计角, 将归一化输出映射为协议原始量.
- `DjiTxGroup`: 合并一组四个槽位, 核对输出范围与时效, 一次交给 `CanTx`.
- `valid_dji_layout()`: 在初始化时检查提供的 DJI 配置集合, 拒绝重复反馈 ID 和不支持的命令 ID 共享.

**它们不包含 PID、控制权仲裁、自动归零或机构停止策略.** 应用仍须使用 [速度控制链](../control/速度控制链.md) 等正式组件验证目标、反馈与本地许可. 当前板级观察固件没有接入此发送链.

## 2. 配置与单位

所有配置必须显式给出, 默认零初始化不是可用配置.

| 字段 | 含义 |
|---|---|
| `bus`、`mode`、`device_id` | 非零总线号、已确认的协议模式、协议允许的设备 ID; 不自动识别电调类型 |
| `rotor_per_axis` | 转子角 / 轴侧角的有限正比例, 不从电机名称猜减速比 |
| `direction` | 仅允许 `+1` 或 `-1`, 反馈与输出采用同一正方向 |
| `feedback_ttl` | 本地接收反馈的有效期, 到期瞬间失效 |
| `max_rotor_speed_rad_s` | 整个采样间隔内成立的转子速度绝对值上界, 用于判断累计角是否仍可唯一追踪 |
| `output_limit_raw` | 归一化输出 `±1` 对应的协议数值幅度, 必须为正且不超过协议满量程; 不是自动推导的安全电流 |

速度换算是 `direction × rotor_speed / rotor_per_axis`. 控制输出换算是 `round(normalized × direction × output_limit_raw)`, `normalized` 必须有限且位于 `[-1,1]`; 不静默裁剪超界输入. 它不是 A 或 N·m, 也不自动执行功率控制. 协议范围由 `dji_protocol()` 与底层编解码共用.

## 3. 位置为什么需要显式参考?

单圈编码器不能告诉程序机器人上电前转了几圈. 所以:
1. 首个有效帧可以提供速度, 但 `position_rad` 仍无有效参考.
2. 只有调用方知道**最近有效反馈样本时刻**的真实轴角时, 才调用 `reference_position(axis_rad, now)`.
3. 后续用相邻转子单圈角的最短差累加轴侧位移. 这只在可证明采样间没有跨过半圈歧义时成立.

实现使用配置的速度上界、接收时间间隔, 加上 1 ms 时间量化余量和两格编码器误差, 保守检查可能角差是否小于 π. 超时、不可解释跳变、半圈歧义或收到本设备的非法帧后, 位置参考失效; 后续收到正常帧也不会自动恢复累计角, 必须重新建立参考.

`DjiDeviceState` 的 `rotor`、`velocity_rad_s`、`position_rad` 分别带有采样时间和有效性. 位置失效不一定意味着速度也失效. 错误帧不刷新最后有效速度的时间戳, 旧速度按原 TTL 到期; 是否提前因错误触发保护由应用决定. `reset()` 清空全部反馈和位置参考, 不改变配置.

**限制**: 正确的速度上界与时间打点是前提. 本地接收戳不等于源端测量戳; 无序号/启动标识时, 不能可靠识别所有延迟、重复帧或短暂电调重启. 该累计角不是绝对多圈编码器或持久化位置. 比较时间仍遵守 [半回绕观察范围](../core/时间与时效.md).

## 4. 主机最小示例

下面只记录待发帧, 不连接 CAN 硬件. 参数及零位是教学数据, 不是实车配置.

```cpp
#include "ec/drivers/dji_device.hpp"

struct MemoryCan final : ec::transport::CanTx {
    ec::transport::CanFrame last{};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        last = frame;
        return ec::transport::SendResult::accepted;
    }
};

int main() {
    using namespace ec::drivers;
    MemoryCan bus;
    DjiDevice motor{{1, DjiMode::c620_current, 1, 20, -1, {15}, 100, 10000}};
    if (!motor.configured()) { return 1; }
    const ec::transport::CanFrame feedback{0x201, 8, {}};
    if (motor.receive(1, feedback, {100}) != DjiReceiveResult::accepted) { return 2; }
    if (motor.state({100}).position_rad.validity == ec::Validity::valid) { return 3; }
    // 仅在这个已知初始位置的主机示例中, 将反馈样本对应轴角设置为 0.
    if (!motor.reference_position(0, {100})) { return 4; }

    // 本台架明确只装 ID1, 另外三槽未装设备. bus 必须实际对应总线号 1.
    DjiTxGroup group{bus, 1, 1, {2}, {&motor, nullptr, nullptr, nullptr}};
    std::array<std::optional<DjiOutput>, 4> outputs{};
    // 正式应用必须先经过授权/控制检查; 此处只演示纯协议映射.
    outputs[0] = DjiOutput{0.5f, {101}};
    const auto result = group.send({101}, outputs);
    if (result.status != DjiGroupStatus::accepted) { return 5; }
    // 方向为 -1, 因而 ID1 的协议原始值为 -5000 (0xec78).
    return bus.last.id == 0x200 && bus.last.data[0] == 0xec && bus.last.data[1] == 0x78 ? 0 : 6;
}
```

控制、反馈与发送的完整组装见 [host_assembly.cpp](../../../examples/host_assembly.cpp); 该示例选择的故障零请求只用于虚拟台架.

## 5. 分组发送的关键约束

- 组 1 四槽对应 ID1–4, 组 2 对应 ID5–8. GM6020 没有 ID8, 末槽必须未配置.
- 同一命令 ID 只能有一个发送拥有者, 同一硬件句柄只能由一个发送任务使用. `CanTx` 不暴露物理总线号, 正确绑定句柄由组装层负责.
- `nullptr` 表示未装设备, **不是“已装设备但本轮不想控制”**. 未装槽编码为零; 已装槽必须逐个提供输出. 有效的显式零请求是合法输出, 缺失值则拒发整组.
- 每槽先按自身模式与限值转换, 因而 C610/C620 可同组合并, 不会把 C610 的范围套到其他 C620 槽位. 不支持跨协议族/组号共用同一个命令 ID.
- 构造时只检查本组. 应在组装时对所有 DJI 节点调用 `valid_dji_layout()`; 它不检查未提供的节点、非 DJI 协议或重复创建的发送拥有者.
- `send()` 不缓存、不自动重试. `missing_output`、`stale_output`、`invalid_output`、`invalid_config` 都不会调用传输; `tx_result` 此时为空. 默认结果为 `not_sent`.
- 真正调用传输后, 返回 `accepted`、`busy` 或 `error`, 并保留原始 `tx_result`; `accepted` 仍不代表电调已收到或应用.
- **拒发不是停车**: 电调可能仍保持上一条命令. 整组内任一槽拒发也会影响其他设备的刷新, 应用必须定义这一故障传播与处理方式; 不能直接复制虚拟台架的全组零请求作为机构安全策略.

配置固定, 组件无内部锁、不分配动态内存. `DjiDevice`、`DjiTxGroup` 和绑定的 `CanTx` 由单一任务操作; 通信 ISR 可先投递帧给该任务. 总线与设备对象的生命周期必须覆盖组发送器. 状态向其他任务发布时使用 [快照](../core/通道与数据交换.md), 不直接并发读取成员.

## 6. 验证入口与当前边界

- [core_tests.cpp](../../../tests/core_tests.cpp): `dji_device_contracts` / `dji_group_contracts`, 覆盖配置冲突、方向与比例、位置参考/回绕/重连、非法输入、混合 C610/C620、GM6020 分组及发送失败.
- [sil_tests.cpp](../../../tests/sil_tests.cpp): 原单电机故障场景复用本模块; 四个虚构 C620 同组闭环验证不同方向/比例、槽位隔离和显式故障动作. 多电机结果输出断言, 不额外生成 CSV 曲线.
- F407 编译会实例化 `DjiDeviceState` 的快照契约, 但实际复制耗时、控制预算和 CAN 行为仍需上板测量.

DM MIT、新上位机协议、全车仲裁、机构安全动作及参数标定不属于本组件已完成的内容. 实机仍按 [实施与验收](../../robots/sentinel/实施与验收.md) 从 S1 开始.
