# B02fix 首报：遥控帧修正与 main 位置 Kp 对照（未判定）

## 五行结论

```text
问题: 遥控未来帧导致的假失联/重捕获是否修正; main位置Kp 42->28 是否有改善
实际运行镜像: b02fix-kp42 = 28886aa2…87eb79 (214656B); b02fix-kp28 = 348112a7…23642 (214664B)
做了什么/结果: 修正补丁3文件应用并核对; 主机49/49通过; ARM integration构建通过; 两次烧录均回读逐字节一致; 两份采集完整
证据: results/B02fix/{hold-230307,kp28-231137}; 均complete、0超时、0丢包、0触限
卡点: 两次采集动作次数不同(基线2次 vs 本轮各5次), 且固件修正与Kp变更同时生效, 本轮不下优劣结论
```

## 1. 源码与构建

- 增量包 `HANDOFF-sentinel-B02-fix-20261108-v2`，修正提交 `fb463a137d174fec905f1f5a8a18b9c06124260c`，基于 B02 的 `8c31cdb`。
- 补丁 3 文件（`sentinel_cycle.cpp`、`sentinel_app_cycle_tests.cpp`、`sentinel_integration_tests.cpp`），归一化行尾后与 `SOURCE_CHANGES.json` 的 after_sha256 **3/3 匹配**。
- 核心改动为 `accept_remote()` 入口一行守卫：
  `if (tick_regressed(inputs.now, inputs.remote_received_at)) { return false; }`
  不推进去重水位、不放宽失联条件、不改接收时间戳。
- 原包补丁为 LF、现场文件为 CRLF，`git apply` 与 `patch --dry-run` 均因行尾失败。处理方式为备份后临时转 LF 应用、再恢复原有 CRLF 并清理本轮 `.rej/.orig`。该变通属本地工具链适配，不作为补丁本身"检查通过"的证据。
- 主机 Release **49/49 通过**。ARM Release integration 构建通过。
- 本轮**额外修改了 main 位置 Kp 42→28**（`board_app_integration_config.hpp`），这一变更不在 `HANDOFF-sentinel-B02-fix-20261108-v2` 包内，是应现场要求追加的。原包明确要求"先修正代码、原参数不变验证目标保持"，本批**未遵守该顺序**。

### 实际固件

| 项 | Kp42 版 | Kp28 版 |
|---|---|---|
| 标识 | `site-candidate-20261108-b02fix` | `site-candidate-20261108-b02fix-kp28` |
| BIN SHA256 | `28886aa2b1d5047fb92c332b5ac80ca0e5da6553dfc506a4767c6b16f487eb79` | `348112a7b0b3a156370762842e74ee26e0873b4c379346ce67caa5afee123642` |
| 大小 | 214656 B | 214664 B |
| 静态 BSS | 71808 B | 71808 B |
| 回读 | 逐字节一致 | 逐字节一致 |
| 运行 | `CFSR=0` | `CFSR=0` |

Kp28 版本 BIN 内 `28.0f` 出现 2 次、`42.0f` 为 0；与 Kp42 版本相差 650 字节。复位后参数读回 main 位置 `28/3/0`、`overridden=false`（取自 ROM）；速度环 `0.060/0.001667/0` 未变。

site 与标定未改动；fire/pitch 参数、3A 量程、遥控比例未改动。

## 2. 采集完整性

| 目录 | 固件 / Kp | 时长 | 快页 | 状态 |
|---|---|---:|---:|---|
| `hold-230307/` | b02fix / 42 | 20.38 s | 1003/1003 valid | complete，0 超时，0 丢包 |
| `kp28-231137/` | b02fix-kp28 / 28 | 30.40 s | 1494/1494 valid | complete，0 超时，0 丢包 |

两份均含 `wire.jsonl / samples.jsonl / summary.json / main_yaw-control.csv / fire_yaw-control.csv`，采集前后 63 项参数一致，`limited_samples` 与 `limited_frames` 均为 0。

## 3. 观测值（不作优劣判定）

| 指标 | B02 基线 `A-221039`（旧固件·Kp42） | `hold-230307`（修正·Kp42） | `kp28-231137`（修正·Kp28） |
|---|---:|---:|---:|
| main 峰值 \|command\| | 0.609 | 0.336 | 0.482 |
| main 峰值 effort_raw | 9916 | 5675 | 7867 |
| main 跟踪 P95 | 0.57° | 3.86° | 5.70° |
| main 跟踪峰值 | 5.14° | 7.38° | 11.54° |
| 单段最大航向跨度 | 21.2° | 13.8° | 25.6° |
| 触限样本 | 0 | 0 | 0 |

跟踪误差来自约 6 Hz 域页采样，不是 1 kHz 峰值；航向跨度按 5 秒网格切分，未与操作员动作逐段对齐。

## 4. 为什么本轮不下结论

以轮速里程计 |v| ≥ 0.05 m/s 识别运动段：

| 记录 | 运动段数 |
|---|---:|
| `A-221039` 基线 | **2** |
| `hold-230307` | **5** |
| `kp28-231137` | **5** |

基线与本轮两次记录的**动作次数不同**，因此表中"21.2° vs 25.6°"不能作为 Kp 效果对照。

此外：

- 两份新记录虽同为 5 段，但**操作幅度、方向、间隔未标定**，仍不是受控对照。
- 固件修正（`tick_regressed` 守卫）与 Kp 变更**同时生效**，无法分离归因。
- 现场操作员主观感受为 Kp28 "好一点"，与采样指标方向不一致；两种信息并列保留，不取其一。
- 修正包本身的原生复现证据（提前接纳未来1ms样本 → 本周期仍在线、保持目标变化 0.024 rad → 0、pitch disable/enable 1/1 → 0/0）来自该包作者环境，本机未独立复跑该复现。

**本轮结论：未判定 Kp28 是否更优，也未从实车确认目标非预期跳变已消失。**

## 5. 缺项与建议

- 未按任务书要求先以原参数验证目标保持，直接叠加了 Kp 变更。
- 无 1 kHz 原始 gyro/accel 与逐周期电流波形，不能判断高频行为。
- 任务 WCET、运行期栈高水位未实测。
- 建议下一步：固定次数、固定幅度的侧移起停，同一操作脚本分别跑 Kp42 与 Kp28，每组 3 次重复，比较首峰、后续回摆次数和恢复时间；若无法保证动作一致，则只报告"未观察到明确差异"。
- 建议同时补做一次"仅修正固件、Kp 保持 42"的目标保持验证，把两个变量拆开。

## 6. 回传

以最初 20261031 完整交接 ZIP 为基线：

```text
python source/boards/sentinel_f407/tools/integration_handoff.py return --baseline <HOME>/Downloads/HANDOFF-sentinel-integration-agent-20261031.zip --source source --results results/B02fix --output RETURN-sentinel-controls-B02fix.zip
```

本目录含两份原始采集、两版 BIN/ELF、构建与测试日志；文件校验见 `SHA256SUMS.txt`。
