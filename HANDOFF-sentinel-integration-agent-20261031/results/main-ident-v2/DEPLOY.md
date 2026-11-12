# main 速度辨识 V2：部署记录（未烧录、未启动）

## 状态

**部署完成，未烧录、未启动辨识。** 等鱼鱼下达启动指令。

## 一、本次与 V1 的关系

V2 直接针对 V1 复盘暴露的问题：

| V1 问题 | V2 处理 |
|---|---|
| 激励是"速度参考正弦"，实际只产生 3–6 mA 有效电流 | 改为**闭环输出端独立电流正弦**（`excitation_a`），`command_a` 作为总命令单独记录 |
| 报告把 `command_a` 又乘了 3，写成 "≈0.8 A" | 明确 `command_a` 已是 A，**全程峰值约 0.27 A**，不再乘 3 |
| 3 s 保持段补不上负载偏置，工作点没建立 | 5 s 平滑带入 ±0.18 A 偏置，最多 15 s 等**实测工作点**（连续两个 1 s 窗口） |
| 分析器把闭环 `Y/U` 当对象模型 | 以 `excitation_a` 为工具变量，`closed_loop` 改为 `1 - U_control/D` 互补灵敏度 |
| 频点被拒只说没用 | 新增 `rejected_by` 逐项理由 |

## 二、源码接入

- 包 `HANDOFF-sentinel-main-ident-v2-20261109.zip`，SHA256 `811bd6a1da1cb5d959ed03647add83724d95447350c97e2b6592bbdddb774b0e`
- 包内 36 项 SHA256 **36/36 PASS**
- 用包自带 `应用增量.py` / `integration_handoff.py apply`，**15/15 文件写入**，归一化哈希 15/15 匹配
- 二次预检 `changed: []`，幂等确认
- 现场源码备份：`<HOME>/.claude/jobs/b64d51fa/tmp/v2-pre-084425/`（15 文件）

> 注：`SOURCE_CHANGES.json` 中 `sentinel_execution.cpp`、`sentinel_integration_tests.cpp` 的 `after_sha256` 与 `local_after_sha256` 不同，是包方对 CRLF/LF 的两种记录口径；工具按 `after_sha256`（归一化）判定，实际写入与 `files/` 参考版本一致。

## 三、构建

| 项 | 结果 |
|---|---|
| 主机 Release CTest | **50/50 通过**（21.48 s） |
| ARM Release integration | 通过 |
| 角色审计 | `installed_dji=9`、`installed_dm=1`、`synthetic=0`、`unknown_origin=0` |
| 辨识签名 | `ec_main_identification_signature` = `.rodata` 字节 `32 44 49 4d` → `0x4D494432` = **MID2** |
| 辨识符号 | `tick_identification(...,float)` 带第 5 个前馈参数、`handle_identification`、`identification_stream`、`MainYawIdentification::point` 均在 |

> 主机测试需 scipy/numpy/matplotlib/pyserial；本机系统 Python 无这些包，已在隔离 venv `<HOME>/.claude/jobs/b64d51fa/tmp/v2-venv/` 安装，并用 `-DPython3_EXECUTABLE` 指向它。未污染系统 Python。

### 固件

| 项 | 值 |
|---|---|
| 标识 | `app-integration-20261109-v2ident` |
| BIN | `deploy/sentinel_app.bin`，**230736 B** |
| BIN SHA256 | `266c22ef3c040ee12b3bf1e62c33a543767979bacb54f22662851aadba5b923d` |
| ELF SHA256 | `202ea67c0aefa065778b094c6165645ef592c7149e75a0e198702f88f8cba9f1` |
| text/data/bss | 187648 / 43076 / 71864 |

## 四、已部署到 Jetson（192.168.1.108）

| 文件 | 位置 | 校验 |
|---|---|---|
| `sentinel_client.py` | `~/sentinel-host/` | ✅ `c1c59266…` |
| `sentinel_async.py` | `~/sentinel-host/` | ✅ `ff17385c…` |
| `sentinel_identify.py` | `~/sentinel-host/` | ✅ `85e3d334…` |
| `sentinel_identify_analyze.py` | `~/sentinel-host/` | ✅ `922e1b18…` |
| `sentinel_app.bin`（V2） | `~/main-ident-v2_app.bin` | ✅ `266c22ef…` |
| V2 交接 ZIP | `~/` | ✅ |

Jetson 侧 numpy 1.21.5 / scipy 1.8.0 可用。串口：`ttyACM0`=8308（SWD）、`ttyACM1`=353、`ttyACM2`=384（业务口）。

## 五、预览已跑通（未动电机）

```text
speed 1.0, amplitude_a 0.06, bias_a 0.18, acceleration 0.8
ramp 5s, settle_timeout 15s, zero 3s
frequencies [0.25,0.5,1,2,4], cycles 4
maximum_duration_per_direction_s = 74.5, directions = both
```

仅打印计划，**未打开串口、未下发目标**。

## 六、启动前必须满足

1. **烧录 V2 固件**（当前板上仍是 V1 辨识版，`schema=1`）——需单独授权。
2. **机械/线束允许每方向连续约 9.3–11.4 圈**。V2 比 V1 多，因为偏置把速度顶到接近 1 rad/s。V1 实测只走了 4.1 圈，**不能直接沿用 V1 的转动空间结论**。
3. 底盘静止、上装旋转空间清空。
4. fire/pitch 固定在可稳定保持姿态；结束或异常退出后 **pitch 会失能**，需保持承托。
5. 遥控人工停止有效，硬断电可用。
6. 关闭占用业务口的维护台/Jetson 连接。

## 七、启动命令

```bash
ssh jetson-108
cd ~/sentinel-host
python3 sentinel_identify.py --serial /dev/ttyACM2 --output ~/captures/main-ident-v2/<标签>
# 确认无报错后加 --execute
```

分析：

```bash
python3 sentinel_identify_analyze.py \
  --train ~/captures/main-ident-v2/<标签>/positive \
  --validate ~/captures/main-ident-v2/<标签>/negative \
  --native ./ec_sentinel_ident_sim \
  --output ~/captures/main-ident-v2/<标签>/analysis.json
```

## 八、仍未解决的风险（V2 不消除）

1. **V1 数据里同电流对应速度散布极大**：0.14–0.16 A 区间速度 p10→p90 跨 0.89 rad/s。这说明被控对象强非线性/非平稳，不只是摩擦死区。V2 用固定偏置能改善工作点，但不改变这个非线性本质，线性模型可能仍不可用。
2. **V1 中速度环积分在扫频期间持续漂移**：+2.18 mA/s，全段累计约 +119 mA，是 V2 激励幅值 60 mA 的 **2 倍**。V2 的固定偏置是开环叠加，**不会停止积分器**，慢漂移仍可能与激励混叠。这点需要在分析时用去趋势或分周期差分处理。
3. `stale_feedback` 隐患未在 V2 修复：`sentinel_execution.cpp:792` 仍把 `platform.value.heading_rate` 直接标 `Validity::valid`，未检查 `main_platform.validity`。V1 曾观测到 26% stale。
4. 电调真实模式未做硬件确认（固件要求 `gm6020_current`，理论上是 0x1FE 电流帧）。
