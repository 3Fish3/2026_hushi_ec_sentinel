# 项目交接记录：Pitch 方波辨识、连续采集与部署集成

- **生成时间**：2026-10-28 23:33 -04:00（本地时间；最终验收快照见下述 state.json）。
- **任务标识**：`pitch-continuous-ds-integration`。
- **Git 根目录**：`<PROJECT_ROOT>`。
- **分支 / HEAD**：`main` / `37b5660fcad5608a1f7c870fdae5c281052eb9c5`。
- **本次交接前核验**：29 个已跟踪文件有改动、28 个未跟踪条目、暂存 0。计数包含既有工作，未跟踪目录是条目而非逐文件数。无本轮 Git 暂存或提交。
- **实际工作源码树**：`HANDOFF-sentinel-gimbal-app-20260926/source/`；不是 `Hushi_EC_2026_Refactor/`，也不是 chassis/shooter 包。下文 `S` 均指这个云台源码树。
- **总状态**：**电脑上的原生虚拟电机完整流程已有成功数据；源码和部署准备部分完成，最终验收未通过，不可直接当作实车方波调试工具交付。**
- **最新全量结果**：原生构建成功，CTest **52/54**，退出码 **8**。失败项是本轮尚未同步的 Python 协议断言，以及历史文档链接检查。
- **硬件边界**：这轮连续采集开发没有打开实机串口、没有 SWD、没有烧录、没有使能真实电机。目标板当前内容未读回核验。
- **恢复入口优先级**：本文件覆盖前一阶段记录中的“持续采集尚未实现”等过时结论；旧记录 `docs/handoffs/2026-10-28_pitch-identification-luna-offline.md` 仅用于查短窗阶段历史。

---

## 一、关键修改与已闭环事实（Completed & Verified）

### 1. 用户究竟要什么

用户不想继续手工猜 PID，也不想扫频，希望：对 Pitch 做受限方波激励，电脑收集响应数据，解析、分析，最后给出并验证候选参数。已选择的方案是**固定位置保持 + 小幅正负 MIT 前馈力矩方波**，不是重力负载下的开环电流方波。

实现分两端：

- C++ 固件：MCU 定时产生扰动、执行 MIT 控制、记录时间与反馈、维护安全门槛。
- Python 主机：单串口会话、持续目标续期、试验编排、异步写盘、CSV/质量报告。

**真实数据辨识、候选 kp/kd 输出和实车参数部署尚未实现。**先前 `sentinel_pitch_identify.py` 是第一阶段数值模拟器，不是当前原生采集文件的通用拟合器。

### 2. 最新真实运行产物：可以检查的证据

仓库根目录：

`build/identification/pitch/continuous-native-proof-04/`

| 文件 | 内容 |
|---|---|
| `pitch.sraw` | ECRAW1 原始文件，包含元数据与完整 64B 记录 |
| `pitch.csv` | 经正式 `sentinel_raw` 解码导出 |
| `manifest.json` | 试验配方、工况、源类型、文件与质量摘要 |
| `summary.json` | 成功/失败、清理结果、最终记录计数和最终标记 |

独立重新读取并断言通过：

- `run=44`，源类型 **`APP_MIT_SIMULATION_ONLY`**。
- 方波持续 **2.4 s**：baseline 200000 us + 2 × half_period 100000 us × 10 cycles + final_hold 200000 us。
- 方波结束后额外保持约 0.5 s，再显式停止。
- 共 **1198 条记录**：599 条 CAN、599 条 control；数据时间跨度约 **2.99 s**。
- 全局序号缺口 **0**，最大记录 lost **0**。
- 最终 MCU 流状态 `state=3 / generated=1198 / dropped=0 / buffered=0`。
- 主机 `finished=true`，收集条数与最终 generated 一致。
- `error=null`、`cleanup_errors=[]`、`revoked=true`，质量 `complete=true`。
- 这里的 complete 仅指这次**软件采集链路**完整，不代表物理模型已辨识或实车可安全运行。

哈希：

```text
pitch.sraw   ac27218799551a7a38cb9a8bd336ebad737632b1b1c7313617e0c620a63d88d4
pitch.csv    0637c45f5951a5f4efc97f48f6d41afc415b5dad5e0217524083e9df1eb579d0
manifest.json 89a16c365e9569e7bc923f0f2c9eddd6afb67b12c7d4d4ec47ae1fffdc6291cc
summary.json  c59840a161aa1103bfe40bd6c2dcf08dd990f630eea6fffa9c6b3d31fa25a440
```

这些文件位于生成目录，不能假定随 Git 源码提交保留。失败试验 `continuous-native-proof-01/02/`、此前成功但未增加最终排空确认的 `03/` 也保留着；优先检查 `04/`。

### 3. C++ 执行与安全联锁

| `S/` 下路径 | 当前职责 |
|---|---|
| `robots/sentinel/app/sentinel_pitch_identification.hpp` | 可信策略、配方、baseline/square/final_hold/complete/fault 状态机；无主机授权、无目标续期能力 |
| `robots/sentinel/app/sentinel_config.hpp` | 可选 `pitch_identification` 配置，默认无策略 |
| `robots/sentinel/app/sentinel_execution.hpp/.cpp` | MIT 前馈注入、量化后预期力矩检查、实际 CAN 提交、故障锁存、采集联锁、Pitch trace |
| `tests/pitch_identification_tests.cpp` | 执行链、安全/失效、采集绑定与软件负载测试；保留短窗夹具入口 |

关键新增行为：

- `PitchIdentificationPolicy` 末尾增加 `require_capture`、`deployment_opt_in`，默认 false，不从主机消息写入。
- `SentinelExecutor::bind_pitch_recorder(Recorder*)` 绑定同一记录器。
- 需要采集时，启动前必须 `pitch_stream_ready(recipe.run_id, recipe.duration_us()+2000, now)`。
- 每个活动控制周期及发送前复查 `pitch_stream_healthy`。流终止、被替换、溢出等触发采集故障锁存，而不是只在收到查询时检查。
- 正常方波结束仅撤扰动；既有有效位置保持继续。
- 异常进入失能/锁存。恢复反馈、续发目标或重新连线不能自动清锁；复位须目标已撤销且有新鲜失能反馈，再使用新的递增 run_id。
- 已保存独立的 MIT 尝试/成功提交时间，不能让查询/失能帧的提交时间冒充 MIT 时间。没有尝试 MIT 时 trace 前馈为 NaN。
- 故障码保留原 0–9，**10 预留，11=`capture`**。这一扩展引起下面记录的待修测试断言。

### 4. 连续原始记录与封包

| `S/` 下路径 | 当前职责 |
|---|---|
| `robots/sentinel/recording/raw_record.hpp/.cpp` | 固定 512 条队列、连续 control6、过滤/降采样、记录就绪/健康查询 |
| `robots/sentinel/host/sentinel_pitch_stream.py` | 纯字节解码器；无串口或运动入口 |
| `robots/sentinel/app/sentinel_pitch_stream_link.hpp` | APP 板与原生后端共用的原始流请求/发送封装 |
| `tests/capture_stream_tests.cpp` | 固件流式记录及真实 C++ golden 样例 |
| `tests/pitch_identification/test_stream.py` | Python 解码和 C++ 互操作测试 |
| `tests/pitch_stream_link_tests.cpp` | 请求门槛、失败不消费、最终空帧等测试 |

协议事实：

- 原始流请求外层 `0x32`，ACK `0xB2`，非请求数据 `0xB3`，payload v1，头为 `<4B7I`、32B。
- Header：version/op/error/state/run/generated/dropped/buffered/start_us/end_us/layout_id。
- 连续 Pitch 仅 `Profile::control=3`、控制域 **6**；反馈 CAN 原始流 **9**，两者不能混用。
- CAN 记录传输仍 **44B**，control 记录 **64B**，gyro 旧路径 **48B**。按**记录种类**决定长度，不能按 profile 一刀切。
- 每包最多 3 条，payload 上限 224B；CAN 补齐到 64B 后进入文件。
- Pitch CAN 和 control 分别最多 **200Hz**（5000 us）。连续 Pitch 过滤掉 IMU 和其他电机，不沿用全车高频记录量。
- 队列固定容量，背压溢出丢新记录并计数/显露序号缺口；不是循环覆盖。
- 最终合法空数据帧为 `op3/state3/buffered0`；Python 必须接受它作为排空确认。STOP ACK 先到不代表数据已经收全。
- 之前代理首次交付存在 CAN 被编码为64B、Python却按44B读取，以及拒绝最终空帧两处问题，均经父会话指出并修正。不要只信手工组包 mock。

control6 六个 float：请求位置 rad、采用位置 rad、测量位置 rad、测量速度 rad/s、MIT 编码前馈 N·m、dt s。状态字位29/30为激励活动/锁存；提交结果只是本地 CAN 提交状态，不是电调确认或独立电流测量。

### 5. 主机单串口编排与后续父级修正

| `S/` 下路径 | 当前职责 |
|---|---|
| `robots/sentinel/host/sentinel_async.py` | 唯一串口读写线程；v3业务与原始流交错分发，目标续期与采集sink |
| `robots/sentinel/host/sentinel_client.py` | 辨识编解码、响应表、故障码映射 |
| `robots/sentinel/host/sentinel_pitch_experiment.py` | 有界试验编排、固定位置保持复用、后台 SrawWriter、排空、CSV与报告 |
| `tests/pitch_identification/test_async_stream.py` | 原始流/目标ACK交错、提前数据、丢失、过期、换会话、最终标记 |
| `tests/pitch_identification/test_experiment.py` | 编排、目录保护、元数据、磁盘异常、缺失最终标记等 |

父会话已实际修正并验证的点：

1. 元数据原 1024B 不够容纳真实参数快照，首次整流程在授权前失败；扩大到固定 8192B。
2. 无终端软件后端把 stdin EOF 当急停，第二次失败；仅 `--backend` 路径用 `nullcontext(None)`，物理串口保留 Keys/EOF 停止。
3. ACK 先到时不应因尚无2个数据包立即拒绝，改为在有界等待内继续监督，直到满足预采集包数。
4. 创建已有目录失败后不得还调用 finalise 覆盖其中 summary/manifest；新增 `directory_owned` 判断及保持用户字节不变的回归。
5. 清理先显式停止 guard，再等待可能阻塞的 cancel/stop ACK，避免控制循环停止监督但仍处于“保持中”的模糊状态。此步骤会撤扭矩，必须机械防坠。
6. STOP ACK 后保留 run/sink 接收剩余队列；最终标记与 generated 总数一致才确认完成。缺标记、序号缺口、生成计数回退、dropped>0 都显式失败。
7. 已确认采集后超过 **50ms** 没有数据包即不可逆采集故障，并清除目标来源；不会悄悄续期再恢复。
8. 实车 CLI 要显式给出边界及 `--arm --support-ready --accept-release`，包括 stream-duration、hold-timeout、post-hold；不是未经确认就填入虚拟默认值。
9. 采集时长必须覆盖方波 + post-hold + 100ms 同步余量，并小于 uint31 回绕半窗。
10. 产物明确标记 simulation/real-unvalidated；`hardware_ready=false`、`deployable_gains=false`。

**没有新建第二个 RawClient/StreamClient 来抢同一UART。没有写 PID、零点或限位。**

### 6. 板级、原生后端和默认关闭的部署入口

| `S/` 下路径 | 当前职责 |
|---|---|
| `boards/sentinel_f407/include/board_raw.hpp`、`src/board_raw.cpp` | APP获取同一Recorder；CAPTURE角色拒绝 continuous control，避免假装有执行trace |
| `boards/sentinel_f407/src/board_app.cpp` | 绑定Recorder、处理0x32、业务应答优先后尝试一次非阻塞流发送 |
| `robots/sentinel/app/sentinel_link.hpp` | 原始流ACK使用B2，而非把v1误解为v3；提供pending判断 |
| `examples/sentinel_app_host_link.cpp` | 显式 `--pitch-identification` 虚拟配置，独立1ms控制时钟、POSIX poll、非阻塞输出固定帧队列 |
| `boards/sentinel_f407/CMakeLists.txt` | 新 `EC_APP_PITCH_IDENTIFICATION` 默认OFF；ON需要gimbal APP和实际策略头路径 |
| `boards/sentinel_f407/include/board_app_config.hpp` | 注入可信编译策略、编译期边界检查、单独21字段Pitch部署描述符 |
| `boards/sentinel_f407/check_sentinel_app.py` | 审计该描述符、显式启用选择、拓扑、有限限位、TTL、64B记录 |
| `tests/pitch_identification/test_deployment_audit.py` | 默认关闭/显式开启/非法值/截断等纯描述符单测 |

部署选项：

- `EC_APP_PITCH_IDENTIFICATION=OFF` 默认禁用；没有远程消息可以提供或提升可信策略。
- ON时还必须提供 `EC_APP_PITCH_POLICY_HEADER`，头中定义 `ec::sentinel::board::trusted_pitch_identification_policy`。
- 策略要求 `deployment_opt_in=true`、`require_capture=true`、有限行程/速度/力矩/周期/时长/新鲜度边界；必须位于已有几何限位内。
- 执行器还检查真实配置种类、部署受众、Pitch CAN2/node1/master0x011、有限几何及MIT量程声明。
- **没有创建真正的实车策略头，没有选择实车幅度/频率/行程。**缺头时配置构建失败已验证。
- 新描述符 `ec_pitch_identification_summary`：magic `0x50494431`、layout1、21×uint32；与既有 DeploymentSummary layout2 分离，未悄悄改变旧布局。
- 原生后端的虚拟策略不能复制到实车：Kp12/Kd0.4、位置范围±0.2等只为软件夹具。
- 原生后端调度积压超过20ms显式退出，不做无界补时。输出单帧缓存+非阻塞write；这不是对Linux实时性的保证，更不等于板端DMA/CAN时序实测。

### 7. 验证结论和原始证据位置

| 检查 | 最近结果 | 证据/限制 |
|---|---|---|
| 本次交接前全量原生构建 | 成功，退出0 | `.tmp/pitch-continuous-handoff/build.log` |
| 本次交接前全量CTest | **52/54，退出8** | `.tmp/pitch-continuous-handoff/ctest-full.log`，见第三节 |
| 异步采集Python单测 | 18/18通过 | 增加50ms过期、缺口、最终计数等回归 |
| 编排Python单测 | 15/15通过 | 包括大型参数快照、已有目录保护、最终标记缺失 |
| 部署描述符Python单测 | 5/5通过 | 纯软件，不是物理参数确认 |
| 独立Pyright（当时相关主机/测试文件） | 0 errors/0 warnings | 后续改动未再统一全扫，不扩写为“当前所有文件全绿” |
| 活动LSP（最近4个主机/测试路径） | 无当前诊断，3clean/1inconclusive | 不将inconclusive当作通过 |
| 默认关闭ARM云台构建 | 成功，离线镜像审计通过 | 新独立构建目录，不覆盖旧备份 |
| ON但无策略头 | 按预期拒绝配置 | `.tmp/pitch-identification-review/policy-negative.log` |
| 完整原生程序实跑04 | 成功1198条，最终标记、零丢失 | 上述生成目录；无实机 |

默认关闭的ARM产物：

```text
S/build/app-gimbal-continuous-arm-Release/boards/sentinel_f407/sentinel_app.bin
size = 217720 bytes
sha256 = 1bcc59ec0dee913b4d0e86860a88aa15d418a22e408eff5f80353ac27dc11453
```

该镜像**辨识禁用**，即便烧录也不能启动新试验；不提供这里的刷写命令。默认OFF构建成功也不代表ON策略分支已经完成交叉编译和系统验收。

---

## 二、核心架构与工程决策（Key Decisions & Reasoning）

### 1. 硬件事实与不可破坏的原配置

- RoboMaster **C板 STM32F407**，不是Type-A STM32F427。
- APP主机USART1：PA9 TX、PB7 RX；APP波特率921600。
- DAPLink历史上可用；FIREPHX在921600多次无应答。现在设备号/健康均未检查，不固定写死 `/dev/ttyACM0`。
- 旧capture固件常用115200，且是请求应答；不要从“被动读0字节”推断串口坏了。
- Pitch：CAN2，控制ID1，反馈/master0x011，MIT；量程位置12.5rad、速度30rad/s、力矩10N·m。实机历史读回CTRL_MODE=1，旧2的猜测不再采用。
- CAN1：底盘0x201–204、main_yaw0x208（GM6020 ID4）。CAN2：fire_yaw0x20A（GM6020 ID6）、摩擦0x205/206、拨弹0x207、Pitch0x011。
- 现有Scheme B：Pitch zero约0.4732204rad，min约−0.2727550rad，max约+0.4362448rad。
- 现有gimbal Pitch增益仍为 `35/4095*500≈4.2735`、kd0，重力中段保持能力**没有随本轮得到实车验证**。模拟的12/0.4不能冒充已验证实车参数。
- 用户此前发生过掉落压机械止点导致振动；保持正常跨段不撤力矩，但**任何急停、断电、通信失败都会失去力矩**，必须独立防坠支撑。
- 遥控器maintenance为左中/右上，右下急停；gimbal commissioning遥控摇杆不生成运动目标。
- 目标TTL100ms、链路TTL200ms、主机保守50ms监督预算不放宽。

### 2. 为什么不做开环电流方波

DM MIT输入是力矩N·m，不是电流A。没有确认Kt就不能称为电流；重力轴若开环负向/零力矩，可能直接掉到止点。当前采用固定位置保持上叠小力矩扰动，MCU定时，不用电脑串口抖动充当方波时基。

正常方波结束与停止程序不是一回事：前者撤扰动继续保持，后者在明确的有界post-hold后撤保持并释放权限。用户若要求无限保持，应另行设计，不准通过关闭超时来伪造。

### 3. 代理与协作实际状态

- 第一阶段三个Luna代理完成短窗执行/协议/记录；父会话独立复核并修正过实质问题。
- 第二阶段曾再派Luna，用户要求停止；确认停止时未报告新源码修改，仅备份。
- 三个DS首轮请求 `unknown_error`，0工具调用，无代码副作用；用户确认恢复后沿用同一精确模型 `CPA/deepseek-v4.1-flash`。
- DS实际完成本轮持续记录、采集联锁、单UART编排代码，父会话继续做板级/后端集成与实际运行修复。
- 后期旧代理句柄查询返回not found，不能假定仍有后台代理接着工作。恢复后根据当前文件开展；不要为了等旧代理而空转。
- 对代理“已完成”报告不能直接当作验收；本文件的52/54与04样例是父会话重新执行的结果。

### 4. 版本控制和回退材料

工作区很脏，包含早于本轮的PID、几何限位、GUI和文档更改。**不得 `git restore` 整树、reset、从HEAD覆盖文件，不能把全部dirty归入本轮。**

旧稳定云台镜像：

`build/rollback/pitch-identification/pre-feature-ada9340b/sentinel_app.bin`

SHA-256再次核验为：

`ada9340b22b5e18ba8a9ad2c38c7cf59bf5c9052d3542979b1e9c0dcd5591ccf`

旧bin为207352B；这里的匹配表示本地备份匹配历史已烧录镜像，不是本次读取了MCU。还原烧录必须明确授权，且检查镜像、目标与全片擦除影响。

源码检查点：

| 目录 | 覆盖与限制 |
|---|---|
| `.tmp/pitch-identification-rollback/final-reviewed/` | 前一短窗阶段16文件before/after、逆向补丁，曾在副本验证；已不等于当前源码，不能直接应用 |
| `.tmp/pitch-continuous-rollback/parent/` | 本轮父级开始集成前8文件精确备份，有manifest哈希 |
| `.tmp/pitch-continuous-rollback/recording/` | 记录器h/cpp、capture_stream_tests.cpp等改前检查点 |
| `.tmp/pitch-continuous-rollback/safety/` | app下执行/策略文件及tests；路径是`app/`，不要假设含`robots/sentinel/` |
| `.tmp/pitch-continuous-rollback/host/` | 之前主机async/client快照；文件直接放目录下 |
| `.tmp/pitch-continuous-handoff/state.json` | **当前关键文件和04产物的SHA-256/状态清单，不是备份或回退补丁** |

没有生成覆盖整个持续采集阶段、经副本验证的最终回退补丁。各目录来源/覆盖不同；新增文件也需单独核对。所有临时/生成产物都可能不受Git跟踪；需要长期保留时由用户决定归档，不擅自提交或迁移。

---

## 三、遗留风险与未解决阻碍（Residual Risks & Blockers）

### P0：现在为什么仍不能直接上车

1. **最新全量测试有新增失败，最终Advisor审查尚未完成。**不能沿用“只有历史文档失败”结论。
2. **没有实车策略头**。默认镜像disabled；ON分支完整交叉编译、描述符审计、部署集成与实机保护测试仍待完成。
3. **没有确认实车激励安全包络**：姿态、行程、幅度、频率、次数、速度、力矩、温度、最大保持时间和独立防坠条件需要核定，不能照抄模拟值。
4. **现有低刚度默认增益的中段独立保持能力未实测通过**。如果本身不能稳定保持，不能叠加辨识激励。应先解决支撑、保持能力与重力下垂问题，不能无授权强行写增益。
5. **真实MCU/CAN/UART/调度带宽与失效退出未测**。200Hz双流仅软件目标，不能保证实车零丢包；板端单线程、DMA/中断与权限切换仍需审查。
6. **真实数据拟合/可部署参数输出尚无实现**。目前CSV只是数据与质量验证；不能说“采完自动给可用参数”已交付。

### P1：最新全量CTest的具体失败

**A. `sentinel_pitch_identification_contracts`：本轮遗留，必须修。**

- Python discovery共运行76项，其中1项失败：
  `tests/pitch_identification/test_protocol.py::TestPitchIdentificationProtocol::test_constants_and_response_mapping`。
- 当前客户端 `PITCH_IDENT_FAULTS` 是原10项 + `reserved_10` + `capture`，而测试仍精确要求旧10项。
- C++为capture显式数值11，10未用；不能简单把capture放到10。
- 推荐恢复第一步：保留旧0–9稳定性断言，增加10预留/11capture断言及响应解码用例；依据明确协议扩展同步期望，不删除测试、不禁用断言。
- 重跑单项、整个Python目录、原生协议测试和全量CTest；需检查是否还有未显露问题。

**B. `doc_link_contracts`：历史失败，未改动旧README。**

- 检查51个Markdown、241条本地链接，broken224，主要在包内 `source/docs/README.md`。
- 不要删除测试、添加skip，或为修这一条顺手重排全包文档。若不在当前范围处理，交付写明受阻即可。

### P2：已观察到但尚未闭环的工程问题

- 新 `sentinel_pitch_stream_link_contracts` 已注册运行，但 `S/CMakeLists.txt` 的 `_ec_expected_tests` 清单尚未加入它。当前注册契约只检查期望项存在，不会因此自动失败；需补齐防止以后遗漏。
- 主机CLI完整实跑目前是人工执行，尚未注册稳定的**原生程序+Python runner整体端到端**回归。已有mock单测和C++golden互操作不能替代它。
- 04的manifest中`files.manifest`和`files.summary`为空字符串，summary实际存在。这是写manifest时回填文件路径次序的问题，数据未丢失，但应精准修正并补测。
- Writer写入短写、异步错误/最终flush/close边界、finalise的异常结果、故障后目标续期抑制等仍需最终审查，不能仅靠成功一例宣称所有保护完成。
- 已复核主要Python路径独立Pyright，LSP最近部分inconclusive；有过新模块创建后导入缓存未更新，刷新会话专属Pyright进程后实际错误清零。**不要反复浪费时间或通过转接导入/ignore隐藏问题。**真实诊断要修；不要扩大到未改的老GUI/整仓格式清理。
- `sentinel_async.py`最近Ruff仍报告既有`I001`和`SIM115`；未宣称整文件Ruff全通过。需要区分本轮新增和历史项。
- 旧 `Pitch方波辨识离线验证.md`主体仍写着“continuous control拒绝、多秒未实现、52/53”，已过时。此次仅在顶部加新交接指引，未伪称完整改写指南。
- `S/robots/sentinel/capture/capture.hpp/.cpp`本身的历史配置并不因APP Recorder修复就全部同步。不要误烧CAPTURE角色来验证本方案。
- 主Yaw/FireYaw尚未接入本方案；用户曾问能否测，回答“可采用类似方法”不是已授权扩展实施。GUI曲线真实交互验收继续暂停，单独待办保留。

### P3：用户沟通与授权边界

用户多次追问“好了没/到底行不行”，原因是前面长时间只有分派与诊断汇报。不要再把“代理已启动”“模块已写”当作可用结果。

当前允许继续写代码和离线验证，用户后续明确选择过“先完成方波支持再烧录”。**本文件不构成烧录或实机运动授权。**下一会话先交付可重复的验证结果，再明确准备烧录的是普通APP还是有策略的辨识APP；不能绕过门槛满足“烧录一下”。

---

## 四、下一会话恢复行动指南（Next Immediate Actions）

### 1. 第一步：只读核验，然后处理新增失败

先读本文件及 `.tmp/pitch-continuous-handoff/state.json`，确认工作区没有新的用户修改。下面Git命令都是只读：`rev-parse --show-toplevel`定位根，`branch --show-current`读取分支，`status --short`展示暂存/工作区差异，`diff --name-only`只列差异文件；不改变索引/引用。

```bash
cd <PROJECT_ROOT>
git rev-parse --show-toplevel
git branch --show-current
git status --short
git diff --name-only
```

接着检查 `S/tests/pitch_identification/test_protocol.py:43` 的旧故障表断言，按第三节同步协议扩展并增加11capture解码验证。先不要开串口、烧录或重启现有上位机。

### 2. 原生构建与回归命令

以下在仓库根执行；变量只为减少路径长度。确保使用项目 `.venv`，避免SciPy等分析依赖缺失导致假故障。保留CTest原始退出码，不用grep筛掉abort/失败。

```bash
S=HANDOFF-sentinel-gimbal-app-20260926/source
cmake -S "$S" -B "$S/build/host-tests" \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build "$S/build/host-tests" -j4

ctest --test-dir "$S/build/host-tests" \
  -R '^(sentinel_pitch_excitation_contracts|sentinel_pitch_identification_protocol_contracts|sentinel_pitch_stream_link_contracts|sentinel_capture_stream_contracts|sentinel_pitch_identification_contracts|sentinel_pitch_recording_pipeline_contracts|sentinel_pitch_hold_contracts|sentinel_host_client_contracts)$' \
  --output-on-failure

PATH="$PWD/.venv/bin:$PATH" \
  ctest --test-dir "$S/build/host-tests" --output-on-failure
```

当前参考日志：

- `.tmp/pitch-continuous-handoff/configure.log`
- `.tmp/pitch-continuous-handoff/build.log`
- `.tmp/pitch-continuous-handoff/ctest-full.log`
- `.tmp/pitch-identification-review/experiment-native-04.log`
- `.tmp/pitch-identification-review/continuous-arm-config.log`
- `.tmp/pitch-identification-review/continuous-arm-build.log`
- `.tmp/pitch-identification-review/policy-negative.log`

### 3. 重跑完整程序：只准使用下面的软件后端

**命令中的参数都是虚拟模型参数，不得改成串口路径后照用。**输出目录必须不存在，保留旧证据，不覆盖04。正常退出后检查生成的summary，不能只看shell退出0。

```bash
S=HANDOFF-sentinel-gimbal-app-20260926/source
.venv/bin/python "$S/robots/sentinel/host/sentinel_pitch_experiment.py" \
  --backend "$S/build/host-tests/ec_sentinel_app_host" \
  --output build/identification/pitch/continuous-native-resume-01 \
  --arm --run 144 \
  --physical-low -0.4 --physical-high 0.4 --excursion 0.02 \
  --max-temperature 60 --max-torque 3 --max-speed 1.5 --max-error 0.03 \
  --amplitude-nm 0.03 --half-period-us 100000 \
  --baseline-us 200000 --cycles 10 --final-hold-us 200000 \
  --stream-duration-us 5000000
```

这条命令启动的是本地可执行程序，ProcessTransport自动附加：

`--plant --fixture gimbal --pitch-identification`

验收要点：`source=APP_MIT_SIMULATION_ONLY`，波形complete，final marker true，collected==generated，dropped/gap/lost都为0，cleanup为空；不同运行调度可能导致记录数略变，不把1198固定成唯一正确数。

### 4. 下一开发顺序

1. 修正当前协议测试和期望测试清单；补入上述完整软件runner的回归。
2. 审查并补测写盘短写、最终排空/计数、故障后旧目标不复活、权限切换与所有异常退出。保持数据不完整时显式失败。
3. 修正manifest自引用路径回填与指南过时段落；别借此重构老GUI。
4. 执行独立类型检查/相关静态分析、原生构建、CTest、默认OFF交叉编译和负面配置测试；最终调用Advisor对关键执行/采集/主机/板级路径审查。上次Advisor只做阶段指导，还没给最终验收结论。
5. **实车之前**：确认安全中段、独立防坠、原增益可保持、实际电机协议与量程、明确物理策略。生成单独可信策略头并在隔离build目录验证ON分支；不填入仿真值。
6. 将镜像角色、CAN拓扑、Pitch策略描述符、源码/镜像哈希、备份与回退范围交给用户确认后，才安排烧录；随后先只读连通/参数核验，再保持，再最小激励。
7. 拿到可靠真实数据后，再实现质量筛选/闭环辨识/候选参数及小幅验证。不要让“已采到CSV”冒充“参数算好可部署”。

**下一位最重要的一句话：现在已经有真实执行的新软件端到端证据，不要推倒重写；但最新全量有一条本轮未同步测试，实机启用策略和最终安全验收尚未完成。先把证据与缺口对齐，再继续。**
