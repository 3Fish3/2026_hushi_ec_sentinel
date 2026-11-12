# V2 原配置重复采集报告

- 原始目录：`repeat-20261109T221358`；正反向完成状态：`True`。
- 使用现有 V2 固件和采集脚本，未修改 PID、偏置、激励或冻结积分；未应用候选。
- 输入：目标 ±1 rad/s、偏置 ±0.18 A、独立正弦幅值 0.06 A、0.25/0.5/1/2/4 Hz。
- 链路统计：`{"sent": 7387, "replies": 7387, "unmatched": 0, "invalid_replies": 0, "retries": 0, "superseded": 0, "stream_timeouts": 0, "dropped_epochs": 0}`。

| 方向 | 样本 | 工作点建立/s | 扫频均速 rad/s | 上轮同口径均速 | 实际净位移/圈 | 全程命令峰值/A | 丢样 |
|---|---:|---:|---:|---:|---:|---:|---:|
| positive | 62283 | 8.002 | 0.9606 | 0.9642 | 8.166 | 0.3572 | 0 |

| negative | 61275 | 7.001 | -0.9553 | -0.9308 | -7.985 | 0.2950 | 0 |


## 离线分析：frequency_response_only

- 判据说明：至少三个可信频点且频率跨度4倍, 否则只回传频响。
- 自动部署：`False`；候选是否生成：`False`。

| 方向 | Hz | repeat | SNR | usable | 拒绝原因 |
|---|---:|---:|---:|---|---|
| positive | 0.25 | 0.295 | 3.53 | False | cycle_variation |
| positive | 0.5 | 0.181 | 2.97 | True |  |
| positive | 1 | 0.427 | 1.44 | False | cycle_variation, signal_noise |
| positive | 2 | 0.549 | 1.28 | False | cycle_variation, signal_noise |
| positive | 4 | 0.533 | 0.76 | False | cycle_variation, signal_noise |
| negative | 0.25 | 0.387 | 2.89 | False | cycle_variation |
| negative | 0.5 | 0.234 | 2.28 | True |  |
| negative | 1 | 0.422 | 1.98 | False | cycle_variation, signal_noise |
| negative | 2 | 0.763 | 1.05 | False | cycle_variation, signal_noise |
| negative | 4 | 0.477 | 0.94 | False | cycle_variation, signal_noise |

## 判断边界

扫频均速仅统计 stage 2–6，不含平滑退出及零速段；command_a 已是 A，不再乘 3。
闭环反馈参与试验是本辨识设计的一部分。周期重复性差本身不能定位为“PID 与激励打架”，本轮不据此修改控制器或冻结积分。
数据完整性、运动状态和模型可用性分别以 summary.json、metrics.json 和 analysis.json 为依据。
