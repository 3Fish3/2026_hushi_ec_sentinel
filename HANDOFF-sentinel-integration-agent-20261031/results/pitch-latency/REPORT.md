# DM pitch 反馈年龄与平台重建：只读链路核查

## 核心结论

已实机确认一个具体故障机制：**失能状态的低优先级0x7FF pitch查询会在CAN2仲裁中失败；硬件自动重传关闭；软件把“装入邮箱”计为发送成功，未统计这类发送完成失败。** 因此高查询提交频率与dm_tx_error=0并不意味着查询可靠送达。

这导致待机时反馈更新出现长空档，再依次触发5ms历史查询界限与20ms历史过期清空，平台状态在valid/stale/no_reference间切换。

同时，既有7段实际PID试验中pitch已经使能并由MIT控制帧保持，反馈年龄最大不到2.8ms、试验期重建缺失计数增量全部为0。因此**不能把待机查询问题直接当成所有试验异常或整车摇晃的唯一根因**。

本次未授权电机、未发目标、未写参数、未改时序门限、未烧录；SWD寄存器检查全程CPU保持running，没有halt/reset/resume或寄存器写入。

## 1. 30秒落盘观测

目录：`readonly-20261111T004019/`，summary与每个状态响应均先落盘。

- 30.0069秒，1086份状态；source全为none、authority全为false。
- pitch前后均installed/online=true、enabled=false、faulted=false，CAN2，device_id=1、feedback_id=0x11。
- dm_query_sent增量15033，约501次/秒；dm_command_sent与dm_enable_sent增量均为0。
- dm_tx_error、group_tx_error、group_refused增量均为0。
- pitch调度约2ms，未见invalid_steps增加；不是代码只每几十毫秒查询一次。
- 平台：valid=396（36.5%）、stale=262（24.1%）、invalid/no_reference=428（39.4%）。

这里测到的是“距上次已接受反馈的年龄”，不是单个查询的请求-响应延迟。没有为每个请求与响应做逐帧配对，不能把年龄分布直接命名为电机处理时延。

### 同一份响应中的年龄与有效性

| pitch位置年龄/ms | 样本数 | 平台状态 |
|---|---:|---|
| [0,5) | 346 | 全部valid |
| [5,10) | 115 | 50 valid / 65 stale |
| [10,20) | 171 | 全部stale |
| [20,50) | 324 | 26 stale / 298 invalid |
| [50,100) | 125 | 全部invalid |
| [100,1000) | 5 | 全部invalid |

状态反馈使用控制快照，而年龄按应答时刻计算；本次clock_ms-control_ms约1ms，因此整数毫秒的阈值附近会有约1ms偏移。

## 2. 被动CAN原始接收窗口

目录：`rawpassive-20261111T005214/`，使用已有APP的观测口原始短窗，各200ms，不控制任何电机。原缓存如非空会先导出，不丢弃已有数据。

- burst-1：257条原始记录，其中pitch CAN反馈21条。
- burst-2：260条原始记录，其中pitch CAN反馈27条。
- pitch帧均为CAN2 / 0x11，载荷节点低4位均为1、状态高4位均为0（Disabled），地址与路由匹配。
- 录制器未满，lost=0；对应原始CAN记录里的接收丢失计数也为0。
- 第二窗口出现48.181ms与48.133ms的pitch接收间隔；这两处source_sequence增量均为1，即中间没有另一条匹配的pitch帧仅仅被记录器采样省略。

记录器对CAN有2ms采样间隔，故不能直接由记录条数估计完整总线帧率。source_sequence用于区分是否存在未保存的中间帧。CAN记录的arrived_us在现有实现中被填成sampled_us，不能把二者相等解释成实测软件排队延迟为零。

## 3. 实机CAN寄存器证据

目录：`canregs-20261111T005746/`、`mailbox-20261111T005906/`。

两次日志均确认CPU_STATE_BEFORE/AFTER=running。只读MCR/TSR/ESR及发送邮箱内容。

### 3.1 自动重传确实关闭

CAN2_MCR=`0x00010054`，NART(bit4)=1，表示No Automatic Retransmission。源配置也为：

`source/boards/sentinel_f407/cubemx/sentinel_s1/Core/Src/can.c:82`：AutoRetransmission=DISABLE。

### 3.2 仲裁失败对应的就是pitch查询帧

代表性双读一致快照：

```text
CAN2_TSR_before = 0x1C050303
CAN2_TIR2       = 0xFFE00000
CAN2_TDTR2      = 0x00000004
CAN2_TDLR2      = 0x00CC0001
CAN2_TSR_after  = 0x1C050303
```

解释：

- 标准ID=(TIR2>>21)&0x7FF=0x7FF，TXRQ=0。
- DLC=4，数据为`01 00 CC 00`，与query_pitch()构造的节点1状态问询完全一致。
- TSR的RQCP2(bit16)=1：该发送请求已结束。
- ALST2(bit18)=1：仲裁失败。
- TXOK2(bit17)=0：该次发送没有成功。
- NART=1：此类失败不会自动重传。

24组快照中，有6组同时满足前后TSR一致、邮箱未在发送、ID/数据匹配查询、RQCP2+ALST2置位、TXOK2未置位。**这是查询发送失败的实测证据，但不能把6/24当成丢帧率**：状态会锁存，多组可能重复读取同一结果，且并非同步事务追踪。

第一组通用寄存器观测中，CAN2 ESR均为0，没有在这些快照里观察到bus-off等错误；这不是整个运行期无电气错误的证明。

## 4. 为什么软件计数仍称成功

- `source/robots/sentinel/app/sentinel_execution.cpp:577-586`：状态问询用0x7FF、4字节`01 00 CC 00`；send_pitch_frame返回accepted即增加dm_query_sent。
- `source/platform/stm32f407/src/adapter.cpp:46-65`：accepted来自HAL_CAN_AddTxMessage()==HAL_OK，仅表示请求已进入发送邮箱，不检查后续TXOK/ALST。
- `source/boards/sentinel_f407/src/board_io.cpp:259-264`：正式APP启用RX FIFO通知，没有启用/处理发送邮箱完成通知来维护这些统计。

所以这次观察到的“约501次/秒查询提交、软件发送错误0”与“硬件查询仲裁失败”并不矛盾。0x7FF在标准CAN里优先级很低，低优先级问询与其它周期帧竞争、且没有重传，是已确认的可靠性问题，不应简单说“达妙天然就是这么慢”。

## 5. 历史重建为何会失效

配置：joint_age=20ms、extrapolation=5ms；fire/pitch几何时移均为0。

1. `JointHistory::at()`在查询时刻晚于最新样本5ms或更多时拒绝外推；插值跨距超过5ms也会拒绝。
2. `expire_joint_samples()`在20ms过期后清空历史；因此后面报no_reference，而不只是stale。
3. DM设备的feedback_ttl=100ms与平台可用性预算不是同一个概念。位置valid标志表示有合法样本，不单独证明它满足平台5/20ms时序要求。

代码位置：

- `source/robots/sentinel/app/sentinel_kinematics.cpp:103-128`。
- `source/robots/sentinel/app/sentinel_perception.cpp:42-46,177-217`。
- `source/robots/sentinel/app/sentinel_execution.cpp:283-297`。

因此，之前按pitch年龄CDF计算“joint_age改50ms即可94%平台可用”不成立：5ms外推约束和其它源条件仍在。即使外推改15ms，也不能以50ms年龄的覆盖比例当成真实可用率。

## 6. 待机与实际PID试验必须分开

已有7段velocity15记录的runtime_before/after及高频CSV：

| 记录 | 时长/s | dm_query_sent增量 | dm_command_sent增量 | reconstruction_missing增量 | 最大源年龄/ms |
|---|---:|---:|---:|---:|---:|
| base44-r2-20261111T001526-t1 | 2.356 | 0 | 1355 | 0 | 2.559 |
| ki015-20261110T195506 | 3.601 | 0 | 1961 | 0 | 2.746 |
| ki03-20261110T203120-t1 | 15.001 | 0 | 7734 | 0 | 2.765 |
| ki035-20261110T203634-t1 | 15.001 | 0 | 7723 | 0 | 2.724 |
| ki035-repeat-20261110T203945-t1 | 15.001 | 0 | 7721 | 0 | 2.672 |
| run-20261110T192801 | 11.393 | 0 | 5906 | 0 | 2.611 |
| startcheck-20261110T200106-t2 | 15.000 | 0 | 7718 | 0 | 2.745 |

使能保持时发送正常MIT命令（设备ID=1，优先级不同于0x7FF查询），反馈也随之更新。各统计跨度包括读元数据等时间，故不直接用command计数/CSV长度声称精确500Hz。

结论：待机查询可靠性问题主要能解释前置有效性检查经常拒绝；现有资料不支持它直接解释上述试验中的全部速度波动。

## 7. 对此前“gain=0就让PID除零”的纠正

平台重建失败时PlatformFeedback会被清空并标为invalid/stale，默认gain=0是“不可用”的占位，并不等于该值已被送入PID。

- 辨识路径在`identification_ready()`检查平台有效性；step()发现丢失执行条件会abort，随后inactive分支撤输出，位于rate/gain计算之前。
- 正式惯性位置路径也先检查platform.validity，失败时不构造方向反馈。

代码位置：`sentinel_execution.cpp:698-705,727-734,791-800,867-886`（当前源码行号）。所以不能凭未授权待机状态页的零gain，断言发生了除零驱动或把它当全部PID异常根因。

同理，用20ms编码器差分和瞬时陀螺比较，或对8192线量化角度做1ms逐点差分，不能唯一证明IMU错误；这部分仍需时间对齐后的独立验证，不在此次查询链结论中强行归因。

## 8. 建议修复方向（本次未实施）

优先处理状态查询的可靠发送与正确统计，而不是先放宽感知门限：

1. 区分“入邮箱”“发送成功”“仲裁失败/发送错误”，给问询帧保留有限的事务状态。
2. 针对无运动语义的状态问询，设计带截止时间和次数上限的补发/错峰策略，避免与高优先级周期帧反复同相碰撞。
3. 不直接对整条CAN2盲开自动重传：控制帧还有输出时效/旧命令不能无限滞留的问题，需要保留原控制安全语义。
4. 修复后先重复同样的无授权观测，比较真实RX间隔、年龄、平台有效率和查询成功/失败统计，再决定是否需要调整历史重建预算。

硬件仲裁失败机制已经实测确认，但各次长空档究竟丢了多少个具体查询，仍需逐发送完成与RX配对才能计量。本次没有将快照比值当概率，也未定位所有整车摇晃原因。

## 产物

- `diagnosis.json`：年龄条件分布、计数差分、原始接收间隔、寄存器逐项解码、历史试验对照、输入SHA256。
- 四个时间戳目录：原始状态JSONL、短窗.raw及CAN JSON、SWD日志。
- `scripts/`：可复现的无授权采样、被动记录及只读寄存器脚本。
