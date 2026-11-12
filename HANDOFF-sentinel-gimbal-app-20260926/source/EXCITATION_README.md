# Gimbal MCU 位置激励与记录交付

## 本包定位

已接入 FireYaw GM6020 级联 PID / Pitch DM MIT 的位置激励状态机、独立协议、Profile4 双记录、主机解析和离线分析。**这是可构建固件及离线验证包，不是已解锁的实机自动整定版本。** 默认 MCU 实验控制器为 disabled / hardware_blocked，simulation=false。离线 CLI 的 simulation=true 是独立进程的能力，不表示板端启用了仿真或运动。

**HARDWARE_BLOCKED 只封锁新增实验通道，不代表整个固件没有驱动能力。** 本包保留 gimbal remote commissioning 正常遥控执行路径，属于 ACTUATING 固件，不能当作无输出采集固件。未烧录、未连接设备、未验证热加载过程；交付的是供现有加载工具使用的 bin/elf/hex，不宣称支持运行中无缝热替换。

## 新增定位保持 / 人工确认流程

Pitch 另有专用 MCU 位置波形：沿 `PitchIdentification` / `0x49` 通道扩展，独立于下述通用位置实验的硬件准入。`op=4 start_position` 使用 51 字节请求，保留原力矩字段为零，末尾新增位置振幅(rad)与目标斜率(rad/s)；`op=5 query_position` 为只读能力查询。旧 `op=0..3` 和力矩方波报文不变。位置策略默认零值关闭，必须显式配置 `max_position_amplitude_rad` / `max_position_speed_rad_s`。详细流程见 [Pitch连续保持与采集](docs/guides/Pitch连续保持与采集.md)。

本轮已接入实际上位机的独立定位保持、人工确认、START 与有限记录窗口；协议、操作顺序和当前验证结果见 [定位流程交接](EXCITATION_POSITIONING.md)。默认硬件封锁不变。下文离线演示与原 [测试报告](EXCITATION_TEST_REPORT.md) 保留为前一轮交付记录，不是当前定位流程的全量回归结果。

## FireYaw 实机准入门（本轮新增，默认 blocked）

本轮在软件层接入 FireYaw 实机准入的证据链，但**默认构建仍不解锁任何实机实验**：

- 执行器层实时振荡守护：`SentinelExecutor::configure_guardian(Domain::fire_yaw, limits)` 只接受版本化、含来源/批准/单位备注与 on_site_approved 的 approved 模式阈值（或离线 synthetic_test/observe_only），deployment 受众直接拒绝 synthetic_test；配置版本反回退。执行器每周期用真实反馈/目标/输出构造 GuardianSample 喂 `SentinelOscillationGuard`，速度/行程等硬条件越限即 fault_latched 并当周期把该槽输出置零 + disable，锁存保持到复位。
- 板级证据源：board_app.cpp 的 `local_runtime` 命名空间提供固定源 id 0x45435254（"ECRT"）+ 5 ms TTL 的 RuntimeBinding，每次查询从应用/执行器真实状态读取 param/config/reference epoch、Guardian 报告、boot 身份（受控 deployment root），绝不回显 query。lease_active 恒 false——本地运行时无租约来源，不授予运动资格。
- 默认固件：`deployment::experiment_guardian_limits()` 返回零值 GuardianLimits（unconfigured），configure_guardian 拒绝，守护恒不进入 normal，experiment_guards 的 guardian 批准位恒 false ⇒ 实验控制器保持 disabled/hardware_blocked。解锁需要同时满足：受控头注入版本化+现场批准的守护阈值、真实测量的 deployment 限值与批准记录、以及独立人工批准（Gate D），缺一不可。
- 离线验收：`tests/sentinel_experiment_admission_tests.cpp` 五组（默认 unconfigured、observe_only 永不 hardware_ready/绝不 fault_latched、runtime 绑定身份/新鲜度/缺失判定、速度越限锁存+恢复后保持、manual_stop 不锁存）；`tests/sentinel_experiment_runtime_tests.cpp` 覆盖绑定/身份/TTL 语义。

## 已实现

- 确定性 bounded_step（五次平滑位置往返）和 trapezoid（梯形速度积分的位置轨迹），不是电流方波；有限数、速度、加速度、行程、输出积分、心跳和时间检查，最大实验时长 180000 us。
- cycle 在本周期权限/遥控/故障仲裁之后、executor 提交之前 tick 实验；仅目标域5/6。显式 simulation Controller 不进入真实 target/执行器路径。
- 独立 experiment 0x60 / snapshot 0xca，维护会话与序号校验；STOP 应答有独立优先槽，不受低频状态页节流。应答成功不等于机械停止。
- Profile4、Kind6/7，每个计算 tick 原子接纳 2×64 B；保留旧版 raw 格式、Profile1–3、Pitch 原单位和状态字段。序号/run 由 recorder 管理，参数 epoch 从实际 Fire 位置/速度环六增益和 Pitch MIT 两增益观察产生。
- 记录输出只陈述已观察事实：Pitch 无归一化输出/实际力矩时保持 NaN；CAN 本地 accepted 不等于电调执行或线上到达。
- 活动实验拒绝协议参数写；离线候选不可自动应用，缺少指标标记 UNAVAILABLE。

## 必须保留的硬件边界

- 空 trusted deployment 保持阻止；Wire{} **不是**仿真。没有伪造 guardian/feedback/limit/stop/批准证据。
- gimbal 本轮已接入 FireYaw 实时振荡守护（SentinelOscillationGuard，版本化+现场批准阈值才可配置，锁存即停发并保持）与本地运行时证据源（board_app.cpp `local_runtime`，独立源 id、逐次真实读取、5 ms TTL、无租约来源恒 false）。**默认固件两者均为 unconfigured，experiment_guards 的 guardian 批准位恒 false，实验控制器保持 disabled/hardware_blocked。** 只提供 trusted provider 头文件也不能绕过缺失的物理实测事实，不能宣称改一个宏即可实机运行。
- Pitch 硬件实验仍 HARDWARE_BLOCKED，防坠/停止策略未验证。
- 本地 Fire stop 证据还要求本周期零值帧被本地 CAN transport 接纳；这仍不是物理停车证明。
- 实验采集就绪只接受同轴、非零 run、无丢条、未截止且至少容纳下一对记录的**短窗**，不接受连续串流作为已证明带宽方案。
- 128 B/tick 在 1 kHz 为 128000 B/s。512条为32 KiB，只容纳256对；建议100–180 ms。即使串口1 Mbaud/8N1约100000 B/s也不足（115200 baud约11520 B/s），不能用PC接收率假设无限实时流。
- 停记录、STOP实验、独立物理急停三者不同。CLI stop 没有实机 transport，明确返回 STOP_NOT_CONFIRMED / exit2。
- WCET_NOT_MEASURED；未做现场反馈时效、CAN交付、停止时延、机械范围或安全验收。Gate D 未开展。

## 离线使用（从本 source 目录运行）

Pitch MIT 方波有独立的 `PitchIdentification` 路径：更新后的 `build/app-gimbal-pitch-on-Release` 显式启用 `EC_APP_PITCH_IDENTIFICATION` 与既有策略头，不依赖本文件所述的位置激励准入。其连续采集、力矩/温度反馈与离线报告见 [Pitch 连续保持与采集](docs/guides/Pitch连续保持与采集.md)。开启构建开关不代表完成现场验收；本文件中默认 blocked 的位置激励控制器仍保持原准入条件。

以下命令不打开串口或调试探针。已有构建目录可直接使用；换机器后先按项目原构建说明配置工具链和构建目录。

~~~sh
cmake --build build/host-tests -j 6
ctest --test-dir build/host-tests --output-on-failure
python3 -S robots/sentinel/host/sentinel_experiment.py status
python3 -S robots/sentinel/host/sentinel_experiment.py run --axis fire_yaw --pattern bounded_step --backend build/host-tests/ec_sentinel_experiment_backend --output build/new-offline-fire-run
python3 -S robots/sentinel/host/sentinel_experiment.py run --axis pitch --pattern trapezoid --backend build/host-tests/ec_sentinel_experiment_backend --output build/new-offline-pitch-run
python3 -S robots/sentinel/host/sentinel_experiment.py analyze --input build/new-offline-fire-run --output build/new-offline-analysis --read-only
cmake --build build/gimbal-remote-verify-arm -j 6
~~~

输出目录要求不存在，避免覆盖历史数据。CLI 不提供 --port / enable / live；两轴两种波形的示例均为 SYNTHETIC_NOT_HARDWARE，native backend 使用同一 MCU 生成器/FSM，但反馈来自简化合成模型，不能用于实机 PID 优劣结论。

## 固件与参数

本次 ARM Release profile 为 gimbal commissioning + remote verify；保留已确认参数表值：main yaw 速度Kp/Ki=0.050/0.002、位置Kp/Ki=28/2.0；Fire速度Kp/Ki=0.16/3.0、位置Kp/Kd=32/0.1，命令带[-0.93,+0.479]；Pitch MIT Kp/Kd=70/2、重力前馈0，命令带[-0.132,+0.235]。这不是自动整定产生的新参数，且没有写入连接中的硬件。

构建/测试结果见[验收报告](<EXCITATION_TEST_REPORT.md>)；协议与接入点见[协议说明](<EXCITATION_PROTOCOL.md>)。交付压缩包含固件、源码快照、当前Linux主机native离线后端、四组标明合成的示例、日志和逐文件SHA256。
