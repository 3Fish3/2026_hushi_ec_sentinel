# 云台辅助调参、自动抬升与同固件遥控验证交接

- 完成时间：2026-10-29 10:43 -04:00，America/New_York。
- 工作区：`<HOME>/.codex/worktrees/b824/sentinel`。
- Git：detached HEAD `37b5660fcad5608a1f7c870fdae5c281052eb9c5`；未暂存、未提交。
- 活动源码：`HANDOFF-sentinel-gimbal-app-20260926/source/`，下文记作 `S/`。
- 本工作区起初是干净检出；此前方波和保持功能尚未提交，缺失于新检出。经说明后，从 `<PROJECT_ROOT>` 复制了云台包范围内 59 个既有改动/新增文件作为基线，旧目录未修改。不可将所有 dirty 都归为本阶段。
- 用户要求本阶段不再调用顾问，已遵守。没有连接实车、烧录、使能真实电机或改写真实参数。

## 一、已实现

### 上位机

- `S/robots/sentinel/host/sentinel_console.py`：保留人工目标、参数草稿与读回，新增云台专用“辅助调参”页。提供自动到水平选项、水平目标、限速与稳定阈值、方波配方、调整后保持、采集/停止、原始记录读取、角度/速度/前馈曲线与误差统计。
- 水平检测复用标定后的关节角与角速度；默认目标 0 rad 是之前标定值，不是本轮重新测出的 IMU 水平。修改 kp/kd 不改变零偏；读回修改前后的 `0x914` 已有原生回归核对。
- `sentinel_pitch_tuning.py`：无 GUI 依赖的数据层和有界后台任务，使用现有 `MaintenanceSession.peer`。不会再开第二个串口，也不会重新 HELLO。自动过程期间锁定人工目标/增益/零位写入；结束后可继续人工调参。
- `sentinel_pitch_experiment.py`：抬升以当前角启用，再用既有目标限速器走向水平。实际角度、角速度和新反馈序号/接收时刻连续满足阈值后，收窄到采集小包络，再启动同 run 的固件叠加信号和连续采集。抬不动、超时、停止、旧反馈重复、离开小包络均拒绝继续。
- GUI 的“调整到水平并保持”与“开始采集”可分开点击，期间不撤掉保持目标源。等待有明确上限。
- 方波正常完成后，先排空采集；勾选遥控交接时继续保持原目标，等待新鲜遥控执行状态。确认后撤掉 PC 自己的目标源、结束维护请求，避免把遥控接管误当作维护故障。超时/停止仍走撤权。
- `sentinel_async.py`：只允许在上一轮最终标记、排空、无丢条、无故障且无待处理请求后，以更大 run 编号开始下一轮。故障锁存不自动清除。辅助页使用固件上一次 run + 1。
- `scripts/sentinel_ui_adapters.py`：统一工具中心复用新增辅助页；返回页面时请求任务退出并等待线程结束，串口门禁仍生效；后台构建路径优先使用本工作区 `build/host-tests`。
- 修复了原界面的 Dear PyGui 2.3.1 散点 `size` 参数不兼容问题，真实控件创建及刷新已测试。

当前“自动”是抬升、稳定、采集与控制交接自动化；真实记录的系统模型拟合、自动计算并写入候选 kp/kd 尚未实现。页面不伪称已提供一键 PID 优化。

### 固件

- `S/boards/sentinel_f407/CMakeLists.txt` 新增显式 `EC_APP_GIMBAL_REMOTE_VERIFY`，默认 OFF，只有云台 APP 可用。
- `board_app_config.hpp`、`sentinel_authority.hpp`、`sentinel_remote.cpp`、`sentinel_cycle.cpp`：新增独立云台验证映射，不改原云台维护角色的默认行为，不依赖拨轮、不驱动底盘或发射机构。
- 档位：左中+右上=上位机调参；左上+右中=主 Yaw/Pitch 遥控验证；左上+右上=发射 Yaw/Pitch 验证；右下=人工停止。右杆上下操纵 Pitch，左右操纵选定 Yaw。
- 切控制来源不加载另一组 PID。维护目标仍有效且正在执行时，继承 MCU 实际采用的机械角目标，保留重力保持误差；回中前只保持、不接受旧摇杆偏转。没有可继承目标时使用有效实际姿态。主 Yaw 验证使用机械参考。
- 切档周期先使遥控授权生效，再用新 token 接受新目标，然后评估和提交输出；两次评估之间不发送 CAN，不先提交一次 disable。其余角色沿用原调度。
- `kp/kd` 为同一执行器的在线 RAM 参数，人工写入/方波/遥控共用；断电保存未实现。
- 遥控目标速率 0.5 rad/s（Yaw）、0.1 rad/s（Pitch）及 `pitch_experiment_policy.hpp` 的力矩/时长等数值都是待现场核对的初始试验策略，不能描述成实测额定值。几何零点和行程来自既有记录。
- 方波仍在活动时切档属于中止，沿用锁存规则。应等待上位机提示方波已结束、等待遥控后再切档；正常完成后的交接已验证。

## 二、验证

- 原生构建成功，当前完整 CTest 为 **54/55**，退出码 8；唯一失败仍是历史 `doc_link_contracts`（241 条本地链接中 224 条断链），未跳过或放宽断言。
- Pitch 辨识 Python discovery **106 项通过**；保持控制 **81 项通过**；原人工维护数据层 **13 项通过**。
- 新增 `tests/pitch_identification/test_tuning.py`：真实原生后端同会话抬升→等待点击→方波采集→恢复人工 kp/kd→同连接再采一轮；确认无第二次 HELLO、自动流程无参数写入、PID 修改不改变水平零偏。
- Dear PyGui 真实控件实例化：人工参数页与辅助页同时存在、自动运行时人工写按钮锁定、停止后恢复。未创建桌面 viewport，不能把此检查描述成实车曲线或桌面视觉验收。
- 统一 `AppPage` 软件后端连接与 READY 状态下返回清理实测：辅助线程、UART 和子进程均释放。
- 新增 `tests/sentinel_gimbal_remote_tests.cpp`，注册 `sentinel_gimbal_remote_validation_contracts`：编译与一体固件相同的启用策略和验证映射；在线修改 kp=9/kd=0.6，跑完有 Recorder 联锁的方波，再切遥控并解码实际 MIT 帧。增益和目标保持一致，旧维护请求结束不停止遥控，切 Yaw 层先回中，Pitch 目标不越软上限，人工停止撤销执行。
- 默认关闭普通角色原测试仍通过；本轮未改波特率、线协议布局、参数编号或 TTL。

## 三、一体镜像

文件：`S/build/app-gimbal-assisted-arm-Release/boards/sentinel_f407/sentinel_app.bin`

- 219440 B。
- SHA-256：`1fdb1bca24e0002bd3d9db20d0bfa55afba03db226127b7faeb7752509008dcc`。
- 标识：`app-gimbal-assisted-remote-validation`。
- `EC_APP_GIMBAL_COMMISSIONING=ON`、`EC_APP_GIMBAL_REMOTE_VERIFY=ON`、`EC_APP_PITCH_IDENTIFICATION=ON`。
- 策略头：`S/boards/sentinel_f407/include/pitch_experiment_policy.hpp`。
- 交叉编译与 POST_BUILD 角色/拓扑/Pitch 描述符审计通过。该镜像包含执行能力，但本轮未烧录、未实车验证。

本工作区没有独立 `.venv`，验证时使用已有 `<PROJECT_ROOT>/.venv/bin/python`；SDK 为旧工程已有的只读 `Hushi_EC_2026_Refactor/build/deps/STM32Cube_FW_F4_V1.28.0`，未安装依赖或复制 SDK。主机编译显式用 `/usr/bin/gcc`、`/usr/bin/g++`，避免初次 ccache 环境导致 Threads 探测失败。

## 四、恢复入口与后续

从本工作区打开工具中心：

```bash
<PROJECT_ROOT>/.venv/bin/python scripts/sentinel_ui.py \
  --source HANDOFF-sentinel-gimbal-app-20260926/source
```

选择“云台控制 → 上位机调试 → 辅助调参”。软件演练选择“Pitch辨识仿真”，不会打开实机串口。

恢复时先核对工作区、文件和 build 目录：旧目录仍保留前序版本；本工作区未提交文件包含继承基线，不应整包盲目提交。继续运行：

```bash
SOURCE=HANDOFF-sentinel-gimbal-app-20260926/source
cmake --build "$SOURCE/build/host-tests" -j4
ctest --test-dir "$SOURCE/build/host-tests" --output-on-failure
```

实车后续需核对水平零点、遥控方向、保持能力和试验阈值，再在明确烧录/运动授权范围内验证。同一镜像完成调参和遥控验证后无需再次换固件；这不等于已经验证实车电流绝对无突变。真实数据拟合与参数推荐仍为后续工作。

## 五、2026-10-29 11:08:53 -04:00 后续烧录检查点（America/New_York）

用户随后明确要求烧录本版本，并沿用其“不备份、直接擦写”的要求。ST-Link 目标检查确认 STM32F407（ID 0x413）、1024 KiB；只擦除扇区 0–5，写入本记录列明的 219440 B 一体镜像，未改保护位。

OpenOCD 返回 `FLASH_PROGRAM_VERIFIED`、`FLASH_PROGRAM_SUCCESS`。独立再次连接并读回 219440 B，与镜像逐字节一致，SHA-256 同为 `1fdb1bca24e0002bd3d9db20d0bfa55afba03db226127b7faeb7752509008dcc`；随后 `reset run`。工作目录：`build/flash-runs/gimbal-assisted-20261029-final/`，含 `image.bin`、`readback.bin` 与配置脚本。

此前“本轮未烧录”的表述是开发完成时的历史状态；截至本检查点，一体固件已烧录并读回验证。没有在本次烧录操作中连接业务串口、申请维护权或发送电机目标；实际抬升、采集、遥控及电流平稳性仍需实车验收。

## 六、2026-10-29 11:45 -04:00 实车接口与慢回包调度修复

真实串口已确认：APP `config_kind=2`，17组只读接口正常，三个云台电机在线，辨识可用且 idle。ST-Link 不停核读取 `g_ec_io.remote` 两次：左拨杆3（中）、右拨杆1（上）、四个杆值0、在线1，帧计数增加。未改写该诊断区。

用户多次启动辅助流程均在 `holding=false` 时因状态页1超时失败并已撤权，尚未进入抬升/激励。根因是固件 `ReplyMux` 的50ms共享慢页槽：两份诊断夹住状态页1，实际状态回包间隔约150ms，超过120ms新鲜度门槛。

修复 `sentinel_pitch_hold_model.py`：`SupervisorConfig.shared_slow_lane` 在真实配置启用，状态页1与单份诊断交替、诊断0/1按最久未查询顺序轮换；管道后端沿用原策略。`sentinel_pitch_experiment.py` 与 `sentinel_pitch_hold.py` 按配置种类启用。120ms新鲜度及100/200ms TTL未放宽。

新增 `tests/pitch_hold/test_slow_read_schedule.py` 用每50ms只回一份的共享通道复现：改前超时、改后保持成功。保持、辨识、人工控制三组CTest均通过。真实MCU仅交替做只读查询20轮，状态回包最大延迟53.36ms、最大间隔103.09ms，没有授权或目标命令。本检查点没有重新进行真实抬升/方波，不把只读查询成功当作运动测试通过。旧窗口需重新打开加载主机修复。

## 七、2026-10-29 11:57 -04:00 有界实车抬升验证

用户要求继续验证后，串口确认空闲，使用与界面相同的 `MaintenanceSession`、`start_assisted` 运行一次到水平测试。沿用 MCU 读回的 kp=4.273504257、kd=0，未写任何增益；目标0rad、目标限速0.05rad/s、抬升最长12s、最大跟踪误差0.03rad；不请求方波采集。

- 保持握手通过，进入 LIFTING，状态页最大回包间隔100.87ms，本次没有再出现120ms状态超时。
- 实际角度从 -0.25902188rad 仅变化到约 -0.25864041rad；MCU采用目标从 -0.25902188rad 前进到日志中最后的 -0.23316966rad，随后监督器以 `Pitch tracking error exceeds limit` 中止。
- Pitch反馈 enabled=true、faulted=false，反馈力矩从约0增加到0.105N·m。使能与目标通路已工作，但此次力度未能带动负载；不把原因直接等同于某个PID数值，也不自动放宽误差阈值。
- report.holding=true、lift.settled=false、revoked=true。未进入 READY、未启动叠加信号或采集。整体抬升验收仍未通过。
- 停止后另一次只读确认：来源none、Pitch execute=false、enabled=false、faulted=false、反馈新鲜（1–2ms）。
- 证据：`build/identification/pitch/hardware-lift-verify-20261029-115723/hardware-verification.json`；原始串口日志 `build/sentinel-console-logs/hardware-lift-20261029-115723.wire.jsonl`。

后续先检查负载/机械阻力及保持力矩与增益，再验证能实际抬升和稳定后才能继续方波。

## 八、2026-10-29 12:44 -04:00 增益抬升 + 重力前馈补偿（已实车验证）

用户确认：改 Kp/Kd 后担心的“指令角与实际上下位对不上”属重力静态下垂。实测定位后分两步处理。

**一、MIT 增益由台架初值改为可抬升值。** `board_app_mechanism_config.hpp` 原 `kp=35/4095*500≈4.2735, kd=0`，实测抬升时关节几乎不动，等效力矩仅 0.10~0.16N·m，约 1s 即触发 `Pitch tracking error exceeds limit`。改为 `kp=50.0f, kd=1.5f` 后，实测从 −0.259rad 抬到 −0.018rad、7.1s 稳定。`sentinel_console.py` 辅助调参“到位允许误差”默认由 0.005 改 0.02rad（重力下垂恒约 0.0175rad，0.005 物理上到不了）。

**二、零点/限位不受增益影响。** 读回 0x914/0x915/0x916 恒为 0.4732204 / −0.2727550 / +0.4362448，`overridden=false`；改 Kp/Kd 走 `set_dm_mit_gains`，改限位走 `set_joint_geometry`，两条路径独立。

**三、加入重力前馈补偿。** 实测 Kp=50 下四点（−0.10/0/+0.10/+0.20rad）保持力矩 0.803/0.852/0.842/0.833N·m，下垂恒约 0.0175rad（1°）且不随角度放大。
- 新增 `DmSlot::gravity_ff_nm`（`sentinel_config.hpp`），可选常驻前馈，缺省 nullopt；
- `sentinel_config.cpp` 校验其有限且 ≤ `torque_max_nm`，新增 `ConfigIssue::invalid_dm_gravity_ff`；
- `sentinel_execution.cpp` 将 `gravity_ff + 辨识激励` 作为 MIT `ff_nm`；
- 实车配置 `pitch.gravity_ff_nm=0.875f`（力矩均值，编码 ±10N·m 远未触限）。

**实车验证（同一进程四点复测，Kp=50/Kd=1.5，前馈 0.875N·m）：** 下垂由约 1° 降到 −0.04°~+0.07°（残差 ±0.0013rad，编码器/噪声量级），力矩仍 0.8~0.9N·m。证据 `build/identification/pitch/droop-map-20261029-124444/droop-map.json`（对照前 `...-123344`）。

**烧录：** 镜像 220448B，sha256 `7261a3f58b67f9c87262a2d2f54b74c4d753cb193ea5fd458f660ffe90c1db75`，ST-Link（V2J46S7，`stlink.cfg`+`hla_swd`）擦扇区 0–5 后写入，`FLASH_PROGRAM_VERIFIED/SUCCESS`，独立读回前 220448B 逐字节一致。工作目录 `build/flash-runs/gimbal-gravity-ff-20261029/`。

**测试：** 55 项 CTest 仅原有 `doc_link_contracts` 失败（224 处本地文档链接失效）；`sentinel_pitch_hold_contracts` 首次新编译并发运行时偶发超时，隔离与后续连跑通过。`gravity_ff_nm` 为常数前馈，只在实测 ±0.2rad 区间成立，非 cos 重力模型；改增益或换负载须重测。方波激励幅度上限仍为 0.05N·m（重力/摩擦 ~0.9N·m），激励几乎推不动机构，辨识价值有限，待后续单独调策。

## 九、2026-10-29 13:20 -04:00 操作限幅（command band）实车验证

用户测得 Pitch 摩擦轮可用带（关节角）：上位 +0.235、下位 −0.132（水平≈0.05，最下面 −0.259，断电靠重力会到 −0.259）。要求：**下发的 pitch 目标只能落在 [−0.132, +0.235] 内**，且不改机械标定参数（0x914/0x915/0x916）。

实现（不改标定，另加一层目标限幅）：
- `JointGeometryConfig` 末尾新增 `command_min_rad`/`command_max_rad`（可选，放在末尾保持既有位置化初始化语义）。
- `sentinel_app.cpp::accept_targets`：pitch 机械角目标判定从"仅 min/max"改为"优先 command 带，缺省回退机械止点"，越界返回 `limit_violation` 整包拒绝（不静默夹）。
- `sentinel_cycle.cpp`：遥控 mapper 产生的目标同样优先夹到 command 带。
- `sentinel_config.cpp`：command 带必须成对、有限、严格落在 [min_rad,max_rad] 内且 min<max，否则该关节配置不完整。
- 实车配置 `board_app_mechanism_config.hpp`：`command_min=-0.132f, command_max=0.235f`（显式字段赋值）。`pitch_experiment_policy.hpp` 辨识行程同步收进 −0.132..0.235。
- 机械止点 −0.2728/+0.4362 **保持原样**，仍只做反馈越界故障保护；断电掉到 −0.259 不误报。

测试：`sentinel_app_protocol_tests` 新增 `command_band_clamps_targets`（越界拒绝/带内放行/机械止点不被改写）与 `command_band_validation`（非法带拒配）；`sentinel_mechanism_commissioning_tests` 更新为操作带判定。全套 CTest 仅原有 `doc_link_contracts` 失败。

烧录：镜像 220592B，sha256 `0de293dda7266b41ebab60cfb316b5db32e92a692108b59b675c4c137bc6ab76`，ST-Link 擦扇区0–5写入，`FLASH_PROGRAM_VERIFIED/SUCCESS`，独立读回前220592B逐字节一致。工作目录 `build/flash-runs/gimbal-command-band-20261029/`。

实车验证（同一时刻边界测试，`build/identification/pitch/band-verify-20261029-132053`）：
- 下发 +0.300 / +0.2355（均机械内但越操作带上界）→ 拒绝，关节停在 −0.2548 不动；
- 下发 −0.1325（越下界）→ 拒绝，停在 −0.2579；
- 下发 +0.234 / −0.131（带内）→ 接受，分别到 +0.2346 / −0.1301。
- 带内运动跟踪误差 −0.7~−3.1 mrad（重力前馈生效）。

**残余风险：** command 带是常数边界，换机械/摩擦轮位置须重测；边界值只做了抽样（0.2355/−0.1325 越界、0.234/−0.131 通过），未做全量扫描。

## 十、2026-10-29 13:26 -04:00 Pitch 遥控验证通过 + 参数记录

用户确认 **pitch 遥控验证通过**，要求记录参数。

**遥控状态（上位机实连 MCU 显示，13:26）：**
- 来源 = **remote**（遥控接管），执行资格=True，人工停止=False，启动就绪=True。
- pitch 关节角 ≈ −0.1286 rad，速度 ≈ −0.0073 rad/s。
- MCU 采用目标 −0.13200 rad，反馈 −0.12856 rad（稳定跟随，正处于操作带下界附近）。

**当前实车生效参数（本版固件内置，RAM/默认值）：**

| 项 | 参数 ID | 值 | 说明 |
|---|---|---|---|
| Pitch MIT Kp | 0x901 | **50.0** | 抬升/保持刚度（原 4.2735 抬不动） |
| Pitch MIT Kd | 0x902 | **1.5** | 阻尼 |
| 重力前馈 | (结构体 gravity_ff_nm) | **0.875 N·m** | 抵消自重下垂，实测四点残差 ±0.07°~±0.0013rad |
| 关节零偏 | 0x914 | **0.4732204 rad** | 标定基准，未改 |
| 关节下界（机械） | 0x915 | **−0.2727550 rad** | 只做反馈越界故障保护，未改 |
| 关节上界（机械） | 0x916 | **+0.4362448 rad** | 同上，未改 |
| 操作限幅下界 | (结构体 command_min_rad) | **−0.132 rad** | 目标只允许在此之上 |
| 操作限幅上界 | (结构体 command_max_rad) | **+0.235 rad** | 目标只允许在此之下 |

**操作带（关节角）：** 上位摩擦轮可用 +0.235、下位 −0.132、水平≈0、最下面（断电重力）−0.259。目标越出 [−0.132,+0.235] 整包拒绝（0.2355/−0.1325 越界测试已拒，0.234/−0.131 通过）。

**固件：** 镜像 220592B，sha256 `0de293dda7266b41ebab60cfb316b5db32e92a692108b59b675c4c137bc6ab76`（`EC_APP_GIMBAL_REMOTE_VERIFY=ON`）。

**遥控操作（gimbal_validation 布局）：** 右拨杆回中 + 左拨杆打上 → 遥控接管；右摇杆 X=Yaw、Y=Pitch；右拨杆打下=人工停止。

**未完成/风险：** 记录时 DAPLink 整块从 USB 掉线（SWD 和 CDC 串口均消失），实时复读未做；本条参数取自 13:26 实连界面读数与固件内置值。探针插回后应再读一次 0x901/0x902/0x914-0x916 核对。

## 十一、2026-10-29 14:05 -04:00 fire_yaw 机械限位实测 + 固件生效

用户指出 fire_yaw 是有机械限制的（此前代码按 `commissioning_unbounded` 无限连续轴处理）。实测确定止点并改为明确限位。

**实测方法：** 遥控接管、右拨杆上(fire_yaw层)、人为缓慢左/右推杆到底至机构卡死(用户确认"推到底卡住了")，记录关节角轨迹 `build/identification/pitch/fire-yaw-range-20261029-134730/range.json`。两端均出现约4s角度平台(推杆继续但角度不动、速度≈0)为撞止点特征。
- 下止点 ≈ **−1.2395 rad (−71.02°)**，上止点 ≈ **+0.8598 rad (+49.26°)**，总行程约 120°。
- zero_offset=2017τ/8192=1.5470rad，换算电机原生行程 0.31~2.41rad，单圈内合法。

**固件改动** `board_app_mechanism_config.hpp`：
`config.joints[1]` 由 `{fire_yaw, zero, {}, {}, false, true}`(unbounded) 改为显式：
min_rad=**−1.235**、max_rad=**+0.855**（各向内留约0.005rad余量防常态顶死报fault）、manual_continuous=false、commissioning_unbounded=false。

**测试：** 更新 `sentinel_mechanism_commissioning_tests`：main_yaw 仍无限轴、fire_yaw 改为有界断言；fire_yaw 4.0/−1.3/0.9 越界拒绝、0.5/−1.0 带内接受。55 项 CTest 仅原有 `doc_link_contracts` 失败。

**烧录：** 镜像 220624B，sha256 `e13c9172d9060089818559148c818923b0f3f3db81db5b1439457e22ed8d97c2`，ST-Link 擦扇区0–5，`FLASH_PROGRAM_VERIFIED/SUCCESS`，读回前220624B逐字节一致。`build/flash-runs/gimbal-fireyaw-limits-20261029/`。

**实车验证(主机下发, source=none)：** 读回 0x515=−1.235、0x516=+0.855(此前无此二参数)；目标 +1.0 越上界被拒(停在+0.17不会冲)、−1.5 越下界被拒(停在−1.10)、±0.855/−1.235 边界接受、±0.3 带内精确跟随。冲撞机械止点的风险已由软件限位挡在 −1.235/+0.855 之内。

**待办：** main_yaw 仍是无限连续轴(未测止点)；fire_yaw 速度/位置环增益仍是台架初值，尚未闭环调参。探针(该 DAPLink)当天 USB 反复掉线多次，需换线/换口。

## 十二、2026-10-29 14:25 -04:00 fire_yaw 闭环调参 + 持久化烧录

用户要求调 fire_yaw 增益。串级结构: 位置环PID → 速度环PID → 电流(2.5A上限)。可调: 速度环0x501/502/503, 位置环0x508/509/50a。

**调参方法:** 主机接管(source=none)、0.2rad 阶跃、多次重复取中位数、比较超调/一致性/稳态误差。5组对照(每组3次)+2组5次头对头确认。

**坑(已纠正):** `write_physical_parameter` 会把物理值再除以电流上限2.5, 我误用时把增益压低到0.4倍; 已用原生 `write_parameter` 恢复并用原生值调参。参数表读回是原生值, 物理值=原生×2.5。

**结果(5次重复):**
- 原 velKp=0.024/Ki=0.4/posKp=7.5: 超调7.0% [−3,7,25,14,−3] 有25%离群, sserr 0.0058
- 新 velKp=0.040/Ki=0.3/posKp=7.5: 超调**2.8%** [3,−1,9,−1,3] 无离群, sserr 0.0048
- 其它尝试: Ki=0.2更差(7%,有30%离群); posKp=10超调22%(淘汰); velKp=0.04/Ki=0.4超调4.7%。

**固件持久化** `board_app_mechanism_config.hpp`: 单独 `config.dji[5]`(fire_yaw)覆盖为 物理 velKp=0.10/Ki=0.75 → `PidConfig{0.10/2.5,0.75/2.5,0,1,-1,1}`, posKp=7.5 不变。main_yaw 保持初值(未测)。

**烧录:** 镜像 220648B, sha256 `adac4d219b537537004050ddfac24e780284a8a03360512fd0e9e4a0a09bc14f`, 擦扇区0–5, `FLASH_PROGRAM_VERIFIED/SUCCESS`, 读回前220648B逐字节一致。`build/flash-runs/gimbal-fireyaw-gains-20261029/`。

**烧录后复测(3次):** 超调中位3.5% [4,2,12], sserr 0.0063 — 与调参时一致, 证明持久化增益生效。读回确认 fire_yaw 0x501=0.04/0x502=0.30/0x508=7.5/0x515=−1.235/0x516=0.855; pitch 50/1.5/限位不变; main_yaw 仍初值0.024/0.4/7.5。

**局限:** 仅测0.2rad小阶跃、单方向、空载; 大角度、带发射机构负载、反向、抗扰动未验。速度环Kp偏软(0.10物理), 响应偏慢。属一阶可用参数, 非终版。

## 十三、2026-10-29 15:25 -04:00 遥控"走一段停一段"卡顿诊断（只读分析）

用户报告：持续推杆时云台"走一段、停一下、再走一段"。写只读采集脚本 `/tmp/sentinel-record-remote-motion.py` 采 60s（5646 fast / 1064 domain 样本，全程 source=remote，应用层 execute=True/none 未掉）。

**记录位置** `build/sentinel-console-logs/remote-motion-20261029-152333/`（samples.jsonl / fire_yaw.csv / pitch.csv / wire.jsonl / summary.json / fire_yaw_analysis.png）。

**分域结论：**
- **pitch 完全正常**：中位跟踪误差 1.7mrad，最大 5.7mrad，无卡顿。（DM 电机 + 重力前馈 0.875N·m + Kp50/Kd1.5）
- **fire_yaw 是问题所在**：目标指令以稳定 ~0.50rad/s 斜坡变化时，实测中位速度仅 0.18rad/s，但解卡瞬间能冲到 0.87/1.39/1.65rad/s（p90/p99/max）——典型"粘着-滑脱"(slip-stick)。活跃运动段误差中位 0.12rad、p90 0.275、p95 0.395、max **0.465rad(26.6°)**。

**卡顿归类**（测量冻结 >0.2s 的 29 段）：25 段是"目标也冻结"（摇杆回中/打到限位，正常操作），4 段是"目标仍在动、测量却冻住"= 真卡涩。真卡涩集中在两个机械止点附近：上止点 idx92（目标反向走了 −0.212rad 关节纹丝不动）、下止点 idx191（目标 +0.072 不动）、下止点 idx393（+0.091 不动）。

**机理判断（16:08复核更正）：** 该记录支持fire_yaw在连续目标下存在跟随滞后、速度起伏，静摩擦是待验证假设。尚未同步记录电流指令、积分状态及原始遥控通道，不能把根因确定为机械摩擦，也不能由来源一直为remote排除所有遥控输入间歇。后续旧扫参脚本的“冻结占比”把主机重复读取的同一反馈计入，旧保持峰峰值还混合了不同位置，均不作为选参证据。采用去重后的同轨迹实测重新比较，见[复测交接](2026-10-29_fire-yaw-tuning-revalidation.md)。

**后续进展：** 用户已授权在线调参，Ki=2.00的同轨迹复测改善跟随，位置Kp升高则显著增加小阶跃超调，最终保留7.5。机械处理与摩擦前馈仍未实施。另：PID积分项有幅度钳位，但这不等于具备条件积分/回算抗饱和，不能以此宣称任意提高Ki均安全。
