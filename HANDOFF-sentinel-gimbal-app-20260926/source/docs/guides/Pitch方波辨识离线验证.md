# Pitch 方波辨识：离线方法验证

> **2026-10-28 连续采集阶段更新**：本文第一阶段数值模拟说明仍适用，但下文第二阶段的“仅短窗、所有部署角色硬拒绝、多秒未实现、52/53”等是历史检查点。当前持续记录、单串口编排及默认关闭部署准备已落地，软件实跑1198条；最新全量为53/54（仅剩历史 `doc_link_contracts`），实车策略与最终验收未完成。请先读[详细交接与当前验证结论](../handoffs/2026-10-28_pitch-continuous-ds-integration.md)，不要据旧段落判定可上车。

## 当前边界

当前工具**只做仿真，不支持串口，不控制真实电机**。它验证“固定保持环叠加固定周期方波力矩 → 保存原始数据 → 拟合局部模型 → 独立轨迹验证”的方法链路。所有默认机械参数、激励幅度和保护阈值都是虚拟对象设置，不是实车标定或推荐值。

实现：`robots/sentinel/host/sentinel_pitch_identify.py`。

- 不扫频，不改变保持环 kp/kd，不生成实车增益建议。
- 无烧录、使能、参数写入或 CAN 发送入口；不会占用正在运行的维护台串口。
- 方波只叠加到模拟的前馈力矩，总力矩还包括固定 PD 保持项，因此不是严格的纯电流方波。
- 仿真从静止开始，激励前后保留无扰动保持段。真实系统断电、失联或急停仍可能失去力矩，必须有独立防坠保护。

## 运行

在 Sentinel 仓库根目录执行，使用已有的项目 `.venv`。需要 NumPy、SciPy 和 Matplotlib，不额外安装新依赖。

```bash
.venv/bin/python HANDOFF-sentinel-gimbal-app-20260926/source/robots/sentinel/host/sentinel_pitch_identify.py \
  --output build/identification/pitch/simulation-example --seed 27
```

输出目录必须不存在，重复使用同一路径会报错，不覆盖已有数据。默认种子可复现实验噪声。控制台和图均标注 `SIMULATION_ONLY`；`hardware_ready` 始终为 `false`。

### 输出

| 文件 | 内容 |
|---|---|
| `manifest.json` | 格式版本、虚拟对象真值、训练/验证设置、假设及随机种子 |
| `train.csv` | 训练实验完整样本，不在分析时改写 |
| `validation.csv` | 同一虚拟对象上的独立轨迹：改变扰动幅度、反向初始极性、使用另一噪声种子 |
| `report.json` | 模型参数、延迟、验证分数及逐点预测 |
| `response.png` | 验证方波、模拟编码器与预测对比、预测残差 |
| `report.md` | 离线结果和适用边界 |

CSV 包含序号、仿真时间（微秒）、方波扰动、保持目标、kp/kd、力矩偏置、带噪角度/速度、模拟总力矩以及使能/故障/饱和/丢样标志。`simulated_applied_torque_nm` 是仿真量，不是实测电流或独立力矩传感器读数。

## 计算方法与判据

局部机械模型：

```text
J*q_ddot + B*q_dot + G0 + Kg*(q-target) = motor_torque
motor_torque = kp*(target-q) - kd*q_dot + bias + square
```

仿真用 RK4 积分。分析使用已知固定 kp/kd 的闭环输出误差拟合，采用精确零阶保持离散化；不对带噪角度做两次差分。遍历 0～5 个样本的输入延迟候选只属于离线拟合，不是实机扫频。延迟仅施加到方波通道，不包含完整电调内环或总链路延迟模型。

拟合不读取 `simulated_truth` 作为答案。模型参数及延迟只用训练集选择，验证集不参与拟合。输出惯量、黏性阻尼和重力局部斜率的推算依赖**力矩标度已知且固定 PD 确切**；不具备这些条件时不能直接解释成真实机械参数。

数据检查拒绝：

- 缺失基线、未完成全部周期或缺失最终保持段；
- 序号丢失/重排、重复时间戳、时基偏移或不规则采样；
- NaN/无穷值、增益或保持参考在实验期间改变；
- 未使能、故障、饱和、丢样及角度/速度越界；
- 与实验计划不符的输入或激励不足。

当前验证门槛：验证段归一化 RMS 误差小于响应跨度的 10%，且 R² 大于 0.9；惯量为正、黏性阻尼非负。它们是本离线工具的回归门槛，不是实车安全验收指标。独立仿真轨迹通过，也不能证明模型结构符合实车。当前不提供参数置信区间或全行程重力模型。

## 测试

配置已有主机测试目录时显式选择项目 Python；不更换工具链：

```bash
cmake -S HANDOFF-sentinel-gimbal-app-20260926/source \
  -B HANDOFF-sentinel-gimbal-app-20260926/source/build/host-tests \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build HANDOFF-sentinel-gimbal-app-20260926/source/build/host-tests \
  --target ec_pitch_identification_tests ec_pitch_identification_protocol_tests -j4
ctest --test-dir HANDOFF-sentinel-gimbal-app-20260926/source/build/host-tests \
  -R '^sentinel_pitch_(identification|identification_protocol|excitation|recording_pipeline)_contracts$' --output-on-failure
```

直接运行测试：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=HANDOFF-sentinel-gimbal-app-20260926/source/robots/sentinel/host \
.venv/bin/python -m unittest discover \
  -s HANDOFF-sentinel-gimbal-app-20260926/source/tests/pitch_identification -p 'test_*.py' -v
```

## 原生 APP → MIT → 原始记录：第二阶段离线证明

本阶段代码只在 `ConfigAudience::offline_test` 且 `ConfigKind::synthetic_fixture`、配置显式包含合法 `pitch_identification` 策略时允许启动。部署角色**默认全部关闭**：只有 `-DEC_APP_PITCH_IDENTIFICATION=ON` 且提供 `EC_APP_PITCH_POLICY_HEADER` 中 `deployment_opt_in=true`、`require_capture=true` 的可信编译期策略时才参与可用性判定；主机消息始终不能设置或提升策略。普通 APP 仍拒绝 current 目标，普通 MIT 前馈仍为零。没有新增自动使能或自动续期；试验开始前必须已有新鲜反馈确认的独立 Pitch 位置保持。

### 接入与安全语义

- `sentinel_pitch_identification.hpp`：有限时长的基线、固定周期正负方波、最终无扰动保持；正常结束仅撤前馈。
- `sentinel_execution.cpp`：在既有 MIT 编码和提交路径叠加扰动。按量化后的位置/增益/速度/前馈估计总力矩；这不是独立电流测量或硬件限流承诺。
- 授权、目标、参考/配置代次、增益改变，反馈过期、越界、调度失效或发送失败时故障锁存并走 disable 路径。反馈恢复或继续刷新旧目标不能自动恢复；复位必须先撤目标并收到新鲜失能反馈，再使用递增 run_id。
- 协议 v3 增加请求 `0x49`、响应 `0xC9`。query/start/cancel/reset 的操作码为 0/1/2/3；请求分别为 19/43/19/19 字节，状态响应 27 字节，拒绝也可用现有 12 字节短 ACK。start/reset 要求维护持权与代次匹配；重复 start 重放原结果，不重启方波；换会话、撤权和会话超时中止活动试验。
- 短窗夹具仍走 control domain 6；多秒持续流改用独立请求 `0x32`/应答 `0xB2`/数据 `0xB3`（profile 3、domain 6），CAN 流 9 对应 CAN2 `0x011`，不能把它与控制域 6 混淆。持续采集细节见 [Pitch连续保持与采集](Pitch连续保持与采集.md)。control 的第 5 个浮点量是编码后的 MIT 前馈 N·m，不是归一化电流。位 29/30 表示激励活动/锁存；提交状态只证明本地 CAN 提交结果。
- Pitch trace 的 `arrived_us` 使用独立保存的 MIT 成功提交时间，不共用会被查询/失能帧更新的统计时间。未成功提交时回填尝试时刻；未尝试则回填计算时刻，并设置记录 `flags` 位 1（估计）及未知时间误差。没有尝试 MIT 时前馈字段为 NaN。以上都不是 ESC 实际执行时间。

原生测试夹具让同一个 APP、执行器、MIT 帧、虚拟电机和 Recorder 运行，然后经正式 Python 解码器保存 `.sraw` 和 CSV：

```bash
.venv/bin/python HANDOFF-sentinel-gimbal-app-20260926/source/tests/pitch_recording_pipeline_tests.py \
  --save-offline-sample \
  HANDOFF-sentinel-gimbal-app-20260926/source/build/host-tests/ec_pitch_identification_tests \
  build/identification/pitch/native-example
```

输出目录必须不存在；包含 `samples.sraw`、`samples.csv`、`manifest.json`、`summary.json`。样本明确标注 `APP_MIT_SIMULATION_ONLY`，不是第一阶段数值模拟器的 `SIMULATION_ONLY` 数据，不能直接送入其拟合入口。夹具现有 100 ms 请求窗口产生 99 条反馈和 99 条控制 trace，覆盖正负方波及最终保持，无序号缺口；它没有模拟板上 IMU 流量，不能据此承诺实车覆盖时长或串口吞吐。

### 回退边界

- 已校验的旧云台镜像保存在仓库根 `build/rollback/pitch-identification/pre-feature-ada9340b/`，二进制 SHA-256 为 `ada9340b22b5e18ba8a9ad2c38c7cf59bf5c9052d3542979b1e9c0dcd5591ccf`；未在本次读取目标 Flash。恢复烧录需另外授权。
- 三组原始快照在仓库根 `.tmp/pitch-identification-rollback/luna-{execution,protocol,recording}/`。它们是**子代理开始修改前**的不同检查点，部分包含父会话先前的草稿，不能误称完整的功能开发前仓库快照。
- `luna-recording-typing/` 单独保留类型修复前版本；`parent-integration/` 保留新增协议/管线测试注册前的 CMake。父级集成后的逆向差异应重新核对；回退前必须确认文件没有用户后续改动。不自动使用 Git 整树恢复，不覆盖已有 PID、零点、限位和 UI 修改。
- 父级集成后的恢复材料在 `.tmp/pitch-identification-rollback/final-reviewed/`：包含 before/after 副本、正反补丁及哈希清单。已在临时副本执行反向补丁并逐一校验 16 个文件的恢复哈希；未对工作区执行恢复。该验证只证明能回到所存检查点，不弥补清单声明的历史基线和覆盖缺口。
- 上述产物位于生成/临时目录，不随 Git 源码提交自动保存；清理这些目录前须保留所需恢复材料。

### 当前验证范围

连续采集没有记录时，程序记录 `empty capture` 错误并返回失败，即使已经收到最终标记。报告写入失败会标记数据不完整；若 manifest 写失败且 summary 纠正回写仍失败，会移除本轮生成的旧 summary，保留 `.sraw` 与已生成的 CSV。报告缺失的目录应视为未完成试验，不要只凭目录或采样文件存在判断成功。

原生构建通过，全套 CTest 为 53/54；唯一失败为既有 `doc_link_contracts` 文档链接测试，未删除或跳过。MIT trace 的失败发送时间与非 MIT 帧不冒充成功提交已纳入回归。独立 Pyright 对本轮相关 Python 文件检查为 0 错误；主动 LSP 部分路径为 `inconclusive`，不将其当作确认全绿。`sentinel_raw.py` 整文件 Ruff 仍有原有导入排序/路径引导风格告警，本次未添加忽略规则来掩盖。多秒连续采集与单串口编排已在软件后端实跑通过；实车验收与可部署参数建议仍未完成。

## 实车下一阶段：尚未实现

1. 根据已确认的实车边界设计单独部署方案；辨识默认编译关闭，必须显式开启并提供可信策略头，不能仅靠串口命令绕开。
2. ~~在保持正常目标续期的同时实现多秒持续采集和落盘~~ **已实现**：软件后端实跑 `run=144` 记录 1198 条（599 CAN + 599 control）、序号缺口 0、最大丢样 0、最终标记 `finished=true`。实车 200Hz 双流带宽与零丢包仍未实测。
3. 预算并实测 IMU、CAN 与控制 trace 合计记录率、真实串口吞吐及丢样。921600 8N1 理论仅约 92 KB/s，尚未计帧和转义开销。本包 capture 角色的 `capture.hpp/cpp` 仍保留旧配置，不属于本阶段 APP Recorder 路径的修复范围，不应拿它生成实车 capture 镜像。
4. 先在安全中段验证保持能力和转矩方向。独立防坠装置正常运动时不得接触负载，否则会污染模型。
5. 增加真实记录质量筛选、闭环辨识和参数建议适配；当前未生成任何可部署实车 kp/kd，也不支持主 Yaw/Fire Yaw 方波辨识。
6. 明确实车激励幅度、半周期、次数、允许行程、速度与力矩阈值后，取得针对烧录和实机实验的授权，再采集。不得把本工具默认的虚拟参数直接用于实车。

UI 曲线修复独立于本工具，尚待真实交互验收，不能因本离线测试通过而宣称 UI 已修好。
