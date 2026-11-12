# main 速度内环辨识：首报

## 五行结论

```text
本轮问题: main 电流命令→速度反馈动态与当前速度闭环带宽
实际镜像: app-integration-main-ident-20261109, BIN 27046d9e…fd8918, 230532 B, 已烧录并回读一致
做了什么/结果: 24文件补丁接入、main位置Kp回42、50/50主机测试、ARM角色审计通过、正反各62.5s辨识完成
证据: results/main-ident/run-20261109T194010 与 analysis/; 正反各约6.2万样本、dropped=0
结论/卡点: 离线分析10个频点全部usable=false, 未生成PID候选; 根因是main轴严重欠驱动而非采样率
```

## 1. 源码接入与参数

- 交接包 `HANDOFF-sentinel-main-ident-20261109.zip`，SHA256 `ae5d316bab218e38c59c89c23b25dae047d6fbb06821c493c9fc98e08eb3ed93`；包内 45 项 SHA256 校验 PASS。
- 24 文件接入现用 B02fix 源树：17 个与现场基线匹配、7 个为新增。写入后归一化哈希 **24/24 匹配**。
- **补丁工具在本包状态下不可直接使用**：`git apply` 在未跟踪目录会"全部 Skipped 但返回 0"（假通过）；`patch` 因 CRLF/LF 差异全部 hunk 失败。故按 `SOURCE_CHANGES.json` 的 `files/` 参考版本逐文件写入，并保留原有行尾风格。
- 现场源码备份：`<HOME>/.claude/jobs/b64d51fa/tmp/main-ident-pre-062028/`（17 文件）。
- main 位置 Kp 由现场 **35** 恢复为 **42**；`restore-main42.patch` 期望输入是 28，与现场不符，未直接应用，改为单点手工修改。Ki=3、Kd=0 未变。
- 速度环 `0.060/0.001667/0` 未变；pitch 重力前馈 0.8 未变；`board_app_integration_site.hpp` 哈希 `872faf31…41a5f` 未变。
- 本包 `SOURCE_CHANGES.json` 不涉及任何 config/site 文件。

## 2. 构建与烧录

| 项 | 结果 |
|---|---|
| 主机 Release CTest | **50/50 通过**（19.85 s） |
| ARM Release integration 构建 | 通过 |
| 角色审计 | pass；installed_dji=9、installed_dm=1、synthetic=0、unknown=0 |
| 辨识符号 | `ec_main_identification_signature`、`DjiAxis::tick_identification`、`handle_identification`、`identification_stream` 均在 |
| 固件标识 | `app-integration-main-ident-20261109` |
| BIN | 230532 B，SHA256 `27046d9e3b302064c8c03961a6b7573d5d2f466b1ace5f0f0682918f45fd8918` |
| text/data/bss | 186832 / 43688 / 71856（data+bss=115544 B，与包内声明一致） |
| 烧录 | UID `0x260050/0x30345109/0x30343537`，擦 sector 0–5，verify 通过，独立回读逐字节一致 |
| 运行 | `reset run` 后 `CFSR=0` |

## 3. 辨识执行

一次启动完成正反两向，未中途调整参数、未重试。

| 方向 | 样本 | 时长 | produced/consumed | dropped |
|---|---:|---:|---|---:|
| positive (+1 rad/s) | 61805 | 62.500 s | 61805/61805 | **0** |
| negative (−1 rad/s) | 61756 | 62.500 s | 61756/61756 | **0** |

- 两份 `summary.json` `complete=true`；前后 63 项参数完全一致；PID 全程 42/3/0 + 0.060/0.001667。
- `group_refused`、`group_tx_error`、`target_rejections`、`imu_invalid`、`imu_long_gap`、`imu_reprime` 增量均为 0。
- 总统计：sent 7867 / replies 7866，**stream_timeouts=1**（1 次请求层超时；但 produced==consumed 且 dropped=0，数据本身无丢样）。
- 实测采样率 **约 999 Hz**（中位间隔 1001 µs，p95 1188 µs，最大 2023 µs）。

## 4. 离线分析结论：无可信模型

`analysis.json` 结果：

```json
status = "frequency_response_only"
candidate = null
deployment_ready = false
reason = "至少三个可信频点且频率跨度4倍, 否则只回传频响"
```

10 个频点（正反各 5 个）**全部 usable=false**。分析器判据为 `repeat<=0.25 且 median_snr>=2 且 moving`，三项均不达标。

| 方向 | 频点 Hz | 每周期增益 \|Y/U\| | SNR 中位 | moving |
|---|---:|---|---:|---|
| 正向 | 0.25 | 10.14 / 22.84 / 10.06 / 19.20 | 2.03–3.63 | ❌ false |
| 正向 | 0.5 | 2.69 / 6.97 / 2.14 / 2.27 | 0.77–2.52 | ❌ |
| 正向 | 1.0 | 21.38 / 1.73 / 3.12 / 3.92 | 0.68–4.51 | ❌ |
| 正向 | 2.0 | 3.51 / 2.55 / 0.75 / 3.36 | 0.15–1.32 | ❌ |
| 正向 | 4.0 | 1.94 / 1.54 / 2.69 / 3.40 | 0.11–0.47 | ❌ |
| 反向 | 0.25 | 0.95 / 2.72 / 13.51 / 9.94 | 0.27–2.70 | ❌ false |
| 反向 | 0.5 | 0.46 / 2.50 / 1.68 / 2.99 | 0.27–1.55 | ❌ |
| 反向 | 1.0 | 3.40 / 7.76 / 2.11 / 2.35 | 0.56–2.88 | ❌ |
| 反向 | 2.0 | 2.49 / 3.34 / 1.21 / 2.30 | 0.25–1.12 | ❌ |
| 反向 | 4.0 | 2.33 / 2.22 / 1.05 / 1.99 | 0.19–0.87 | ❌ |

### 关键实测：main 轴严重欠驱动

指令恒定 **1.0 rad/s** 的保持阶段（stage 1，3 秒）：

| 指标 | 实测 |
|---|---:|
| 实际速度均值 | **0.216 rad/s** |
| 稳态误差 | **0.784 rad/s（缺 78.4%）** |
| 命令电流均值 | 0.158（归一化） |
| 命令电流区间峰值 | **0.266（归一化）≈ 0.8 A @3A** |
| effort 原始均值/峰值 | 864 / 1126 |

**电流已给到约 0.8 A，实测转速只有目标的 21.6%。** 这不是跟随误差，是转不动。

### 0.25 Hz 首周期异常

```text
指令峰峰 0.400 rad/s
反馈峰峰 0.642 rad/s   （比指令大 60%）
反馈均值 0.170 rad/s
```

反馈峰峰值大于指令，但均值严重偏低 —— 说明有效信号被大幅衰减，同时叠加了更大扰动。

## 5. 判读与边界

1. **不是采样率问题**。本次实际 1 kHz，对最高 4 Hz 激励足够；此前"记录 10 Hz"的说法不成立（那是 10 Hz 状态页的误读，samples.csv 是 1 kHz 控制样本）。
2. **不是辨识工具保守**。`repeat` 最高 4.4、SNR 最低 0.11、`moving=false`，三项判据均未过；工具拒绝建模型是正确的。
3. **同频点周期间增益差 2.3 倍**（10.14 vs 22.84），表现为强非线性，线性传递函数模型不适用。
4. **未生成 PID 候选，未写入任何参数，未烧录候选。**
5. 本次只证明"当前状态下 main 无法跟踪 1 rad/s"，不能据此判定是机械阻力、测速口径还是电流不足；这三者需分别验证。
6. 分析器给出的频响数值本身也受上述质量问题污染，不作为模型使用。

## 6. 建议下一步（未执行）

| 优先级 | 动作 | 理由 |
|---|---|---|
| 1 | 断动力手转 main，感受阻力/卡滞 | 排除机械原因 |
| 2 | 用已知匀速核对 main 编码器测速准确性 | 若测速偏低，控制器会持续加流但"看不到"速度 |
| 3 | 确认 3 A 限流是否足够驱动该负载 | 0.8 A 带不动可能是电流确实不足 |
| 4 | 上述澄清后再考虑降频点（≤0.5 Hz）或用更低幅值重测 | 现数据 SNR 不足以支撑建模 |

## 7. 文件

- `run-20261109T194010/summary.json`、`wire.jsonl`（18 MB）
- `run-20261109T194010/positive/{samples.csv,summary.json}`
- `run-20261109T194010/negative/{samples.csv,summary.json}`
- `analysis/analysis.json`、`analysis/analysis.png`
- `deploy/`（BIN/ELF、主机 CTest 日志、ARM 构建日志、SOURCE_CHANGES）
- `DEPLOY.md`（部署过程与两处补丁工具问题的处置）
- 校验见 `SHA256SUMS.txt`（含原始 samples.csv 的 SHA256，供回传方复核）
