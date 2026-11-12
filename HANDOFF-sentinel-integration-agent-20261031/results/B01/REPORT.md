# B01 首报：加速度校正直接注入隔离与真实转动响应

## 五行结论（Agent-B01 §5）

```text
B01问题: 加速度校正对 main heading 的直接注入隔离，以及真实转动响应。
实际运行镜像/有效参数: app-integration-site-candidate-20261108-b01；BIN SHA256 013ef8a3f5f1b8bfd8e1ff15323e30b19b652cbb3d72290b88e6923056aab806，211768 B；本次采集前后63项有效参数完全一致，site哈希保持不变。
做了什么/结果: 完成补丁接入、主机49/49测试、联合APP ARM构建、烧录及独立回读比对；用户完成约30秒采集。诊断显示直接注入维持浮点小量，运动段heading有同向非零响应；不作为最终航向精度验收。
证据: run-20261108T201513/；首末诊断间applied增加23894、rejected_norm增加365、missing_anchor增加0；max|ΔH|由2.2118303e-7增至2.2495745e-7 rad；page0有效1502/1502；原始帧解码拒绝0、记录丢失0。
卡点/建议下一步: 动作时段未人工打标，无独立航向真值；另两次静止记录曾出现持续stale_feedback/no_reference，原始数据尚未收进本目录。按本批范围先回传，不追加PID或算法调试。
```

## 1. 版本与应用范围

- 增量包：`HANDOFF-sentinel-heading-B01-20261108`，修正提交 `8111013fe0f48ff80718ce7e7a96b01d4743e829`。
- 补丁涉及19个文件，应用后的内容按 CRLF→LF 归一化后与 `SOURCE_CHANGES.json` 的 after_sha256 匹配。部分文件原有CRLF，故原始字节哈希与增量包的LF文件不同。
- 初次 `git apply` 在父仓库上下文中跳过了补丁，不构成应用成功证据；后续采用 `patch` 及包内完整文件完成接入。CMake及文档曾整体采用包内版本，因此不把最终哈希匹配解释为所有现场既有修改均已保留。
- site配置应用前后SHA256：`872faf31f93c2b58c608e8e89b7bca3d7d207248d8314a1cc9a855ad58441a5f`。
- 本批未主动调PID、遥控比例、安装几何、机械零位、行程及输出范围。采集前后参数一致性见第3节；没有单独保存刷写前RAM全量参数，不能据此证明刷写前后RAM覆盖值一致。

### 实际使用的固件

| 项 | 值 |
|---|---|
| 镜像标识 | `app-integration-site-candidate-20261108-b01` |
| BIN | `firmware/sentinel_app.bin`，211768 B |
| BIN SHA256 | `013ef8a3f5f1b8bfd8e1ff15323e30b19b652cbb3d72290b88e6923056aab806` |
| ELF | `firmware/sentinel_app.elf`，328556 B |
| ELF SHA256 | `5aa467fd8dc86f17dcb16cbb69302087cab489b82c26cf4b45290b456787c8ac` |
| 角色 | `EC_APP_CHASSIS_GIMBAL_INTEGRATION=ON` |
| 生产算法符号 | ELF存在HeadingTilt，未发现AttitudeEstimator符号 |

前一轮实际使用的 `20261108-clean` BIN/ELF 补充归档在 `firmware/previous-clean/`；该BIN SHA256为 `1676ec844c784c1a4125bf2f44db33ac9b3ef28e5b3c7509b1f8abfa7b4cea73`。

## 2. 构建和部署证据

- 本机主机Release测试49/49通过，15.55秒，包含 `heading_tilt_contracts`。单独算法测试输出 `CHECKS=500877 FAILURES=0 RESULT=PASS`。
- 本机联合APP ARM构建及角色审计通过，日志见 `evidence/`。本机使用GCC16.2 Linux，包作者使用GCC14.2 Windows，且现场配置不同；没有要求两者BIN相同，也未逐项归因二进制体积差异。
- 静态栈报告：`correct_accel_aligned` 200 B、`predict` 112 B、`reset` 40 B。仅为各函数自身栈帧，不是调用链总量或运行高水位；`arm-stack-usage.log` 是栈审计构建日志，不是完整的栈帧清单。
- 烧录时核对ID `0x101f6413`、Flash1024 KiB、UID `0x260050/0x30345109/0x30343537`，擦sector0–5，写入 `0x08000000`；工具输出verify通过，独立回读与源BIN逐字节一致。
- `reset run` 后读取 `CFSR=0`、`HFSR=0`，两路串口可应答。原始烧录完整日志和回读文件尚未归档到本目录，以上部署结论依据本会话工具输出。
- 观察口build_id被截断为32字节，不足以单独区分clean/B01；镜像身份依据实际烧录及回读哈希。

## 3. 本次原始采集

目录：`run-20261108T201513/`，由用户运行原包提供的 `sentinel_heading_capture.py`；仅握手、读参数和查询，不发目标或申请控制权。动作由现场遥控完成。

| 检查 | 本次结果 |
|---|---|
| summary.complete | true |
| summary.duration_s | 30.699254 |
| 快页0 | 1502份，1502/1502 validity=valid；首末快页覆盖约30.625秒 |
| 域页1／请求页2 | 各200份采样记录 |
| tilt诊断记录 | 201份，含首末诊断 |
| 发送／应答 | 2116／2116 |
| unmatched／invalid_replies／retries／stream_timeouts | 均0 |
| frame_rejected／log_dropped | 均0 |
| 采集前后参数 | 各63项，JSON内容完全一致 |

`summary.json` 同时保存前后诊断2。不同页并非同一时刻快照，分析必须按各自时间戳关联。

`evidence/capture-run.log` 属于较早的 `capture-20261108T200356`，不是本次用户执行命令的终端输出；本次完整性以本目录的summary和原始日志为准。

## 4. 判据与结果

### 4.1 直接注入及校正是否实际生效

首末诊断均为schema=1、available=true、local_epoch=2。约30.084秒的首末诊断间：

| 计数 | 首次（开机累计） | 末次（开机累计） | 本次增量 |
|---|---:|---:|---:|
| applied | 505871 | 529765 | **23894** |
| rejected_norm | 24137 | 24502 | **365** |
| missing_anchor | 666373 | 666373 | **0** |
| unaligned | 1 | 1 | **0** |
| no_change／invalid_input／invalid_dt／rejected_projection／rejected_antipodal | 均0 | 均0 | 均0 |

- 开机累计 `max_abs_heading_injection_rad` 从 `2.211830292e-7` 增加到 **`2.249574464e-7 rad`**。峰值在本次区间内增加，因此本次出现了该量级的校正；峰值本身由MCU逐更新累计，而非约6Hz主机采样重建。
- 末次 `heading_injection_rad=-4.646613050e-8 rad`。
- 实际applied明显增加，排除“整个过程关闭校正或全跳过”的情况。观测到的直接注入维持浮点小量，与任务书本机测试量级接近，没有触及任务书举例的 `1e-5 rad` 异常观察线；该观察线不作为已批准的实车精度阈值。
- **此前依据累计missing_anchor断言“本次大量缺锚点”不成立，本次增量为0。**

### 4.2 运动响应和静止恢复

仅用同页 `main_platform.validity=valid` 的heading，解卷绕后计算末值减首值。本报告统一以 `summary.sampling_start_monotonic_s` 为零点，并仅取采样起止区间内的数据；该次区间内1472/1472快页有效。此前口头分析以首个有效快页为零点，包含正式采样前的查询，分段结果不同，以下表为准。各段未与操作员逐段打标。

| 相对时段 | heading变化 | main机械关节角变化 |
|---|---:|---:|
| 0–5s | +0.178° | −0.044° |
| 5–10s | −57.077° | −58.491° |
| 10–20s | −59.292° | −59.194° |
| 20s至采样结束 | −0.318° | −4.614° |

总heading跨度120.619°，机械关节角跨度约124.10°。运动段heading与机械角均呈同向非零变化，支持真实转动响应仍在。机械角是相对底盘的关节角，不是独立惯性航向真值；二者差值小不单独证明航向准确，也不据此把10–20秒认定为纯平移、或把末段差异归因为机械回弹。

20秒至采样结束的heading净变化−0.318°只是恢复段观测值，该段是否完全静止未单独验证，不记作严格静止漂移验收。

### 4.3 此前两份静止对照

此前用户表示未动的两份采集仍在Jetson上，尚未收进本报告目录：

| 远端目录（`/home/jetson/captures/B01/`下） | 有效／总快页 | 有效样本heading跨度 | 有效样本净变化 |
|---|---:|---:|---:|
| `capture-20261108T200356` | 490/1505 | 1.289° | +1.246° |
| `static-20261108T201030` | 496/1502 | 1.287° | +1.266° |

这些结果来自当时会话分析，尚未在本地重新核验。两份记录有约2/3的stale_feedback/no_reference，不应只引用有效样本指标而忽略失效。早先28°/45°的跨度是误把invalid/stale的0占位值纳入统计，已撤销。

## 5. 本批结束状态与缺项

- 本次记录支持B01关心的直接注入维持小量、校正确实发生以及运动时heading有响应；不扩大为最终航向精度或全部小摆已解决的结论。
- 没有完整全速原始gyro/accel、电流或独立光学真值，不能做逐样本旧新A/B或分解全部误差。
- 先前静止记录持续失效的原因尚未定位；本次没有出现同类快页失效，不追加现场反复试车来凑材料。
- 刷写前全量RAM参数、原始烧录日志/回读、任务WCET和栈高水位仍缺；缺项如实回报。
- 按B01任务书范围结束本批采集，不继续调PID或倾斜增益。

## 6. 回传

当前材料为本报告、两次完整采集 [`run-20261108T201513/`](run-20261108T201513/) 和 [`run-20261108T202244/`](run-20261108T202244/)、实际新旧BIN/ELF及 `evidence/` 构建日志。两次各包含 `wire.jsonl`、`samples.jsonl`、`summary.json`，第4节和第7节为各自分析，第8节为统一口径对照。文件校验见 `SHA256SUMS.txt`。

正式回传仍以最初20261031完整交接ZIP为基线，不能用B01增量包替代：

```text
python source/boards/sentinel_f407/tools/integration_handoff.py return --baseline <HOME>/Downloads/HANDOFF-sentinel-integration-agent-20261031.zip --source source --results results/B01 --output RETURN-sentinel-integration-B01.zip
```

## 7. 补充记录：用户说明“向左移动一次”

目录：`run-20261108T202244/`，原始wire、samples和summary均已取回保留。用户说明本次向左平移一次；前两次Ctrl+C中断记录保留在Jetson原目录，不删除或重采。

- 实际采样区间30.039秒，summary总时长30.707秒。summary.complete=true，但通信有1次stream_timeout（2115发、2114收）；frame_rejected=0、log_dropped=0，不记作全程零超时。
- 整个文件快页0共1500份，其中采样起止区间内1471份，均valid、reason=none、local_epoch=2。采集前后63项参数一致。
- 首末倾斜诊断间applied增加23896、rejected_norm增加368；missing_anchor和unaligned增量均0。累计最大单次直接注入从2.2495745e-7增加到2.7407191e-7 rad。
- 时间分段以summary的sampling_start_monotonic_s为零点，不以首个有效样本重新对时。

| 相对时段 | 观测 |
|---|---|
| 0–5秒 | heading跨度0.118°，净变化+0.011°，轮速里程计近静止 |
| 5–10秒 | main目标净变化约+96.903°，机械角+97.998°，heading+97.165°；不把这一段误标为恒定目标的纯平移 |
| 10–15秒 | main目标不变，heading跨度0.210° |
| 15–20秒 | main目标不变，heading跨度22.833°、净变化+0.352°；约6Hz域页采样到的main目标/测量误差峰值13.455°，不是高频真实峰值 |
| 20–25秒 | main目标不变，heading跨度0.156° |
| 25秒至采样结束 | main目标不变，heading跨度0.085° |

轮速里程计平移速度模长≥0.05 m/s的样本出现在17.652–18.451秒，约0.8秒，峰值约0.851 m/s。该数据用于确认存在平移工况，不等同于独立测得的地面位移或方向真值。

本次再次观测到：直接校正注入维持浮点小量，平移期间仍有显著的估计航向/跟踪残余扰动。两者分别报告，不能把小注入解释为全部航向误差消失，也不能单凭残余扰动判定新算法直接注入隔离失败。按B01范围保留该段，不追加PID调试。

## 8. 两次完整采集与分析对照

| 项目 | 第一份：`run-20261108T201513` | 第二份：`run-20261108T202244` |
|---|---:|---:|
| 原始记录 | [wire](run-20261108T201513/wire.jsonl)、[samples](run-20261108T201513/samples.jsonl)、[summary](run-20261108T201513/summary.json) | [wire](run-20261108T202244/wire.jsonl)、[samples](run-20261108T202244/samples.jsonl)、[summary](run-20261108T202244/summary.json) |
| 分析位置 | 第3、4节：运动响应与恢复 | 第7节：用户说明向左平移一次 |
| 正式采样时长 | 30.034秒 | 30.039秒 |
| 采样区间快页有效／总数 | 1472/1472 | 1471/1471 |
| 采集前后63项参数 | 一致 | 一致 |
| 实际校正次数增量 | 23894 | 23896 |
| 模长拒绝增量 | 365 | 368 |
| 缺锚点增量 | 0 | 0 |
| 末次累计最大单次注入 | 2.2495745e-7 rad | 2.7407191e-7 rad |
| 通信发送／应答 | 2116/2116 | 2115/2114 |
| stream_timeouts | 0 | 1 |
| frame_rejected／log_dropped | 均0 | 均0 |

两份数据独立保留，不拼接为一段，也不把不同动作当成严格旧新算法A/B。快页分段统一以各自summary的正式采样开始时刻为零点；诊断计数使用首末诊断之差，峰值仍明确为MCU开机累计字段。两次均支持“加速度校正的直接航向注入维持浮点小量”，同时保留运动残余扰动和一次通信超时的事实。
