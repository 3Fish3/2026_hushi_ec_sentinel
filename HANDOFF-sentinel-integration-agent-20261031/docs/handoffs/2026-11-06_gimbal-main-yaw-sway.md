# 项目交接记录 (Project Session Handoff)

- **生成时间**：2026-11-06
- **任务标识**：gimbal-main-yaw-sway（底盘＋云台联调；自旋时 main 航向摆动定位与保守量程烧录）
- **Git 仓库根与分支**：`<PROJECT_ROOT>`，分支 `feature/fire-yaw-sweep`。**本交接包整体未被父仓库跟踪**（`git ls-files README.md` 失败），改动均在包内，不影响父仓库已有工作。
- **工作根**：`<PROJECT_ROOT>/HANDOFF-sentinel-integration-agent-20261031`，源码在 `source/`。

---

### 一、 关键修改与已闭环事实 (Completed & Verified)

**1. C 板已更换，几何全部重拟合（2026-11-03）**

- 新板 UID `0x260050/0x30345109/0x30343537`（旧板 `0x1a0039/0x30345106/0x30343537`）。
- 新旧拟合 main/fire 轴 **X 分量符号相反**，证明 IMU 安装方向不同，**旧几何绝不可复用**。
- 采集：对齐×2、静止×2、三轴各×2，全部换板后重录；对齐/静止均 0 缺口 0 CRC。
- 拟合结果（`results/calibration/site-candidate-board2-20261103.json`）：

| 轴 | 留出 RMS rad/s | R² | 训练/留出轴向差 |
|---|---:|---:|---:|
| main | 0.02233 | 0.99453 | 0.1029° |
| fire | 0.02391 | 0.99172 | 0.0221° |
| pitch | 0.03460 | 0.96265 | 0.0441° |

- 已写入 `source/boards/sentinel_f407/include/board_app_integration_site.hpp` 并烧录验证。

**2. 输入量程经历多轮下调，当前源码值（未全部烧录）**

| 参数 | 源码当前 | 说明 |
|---|---:|---|
| translation | **1.0 m/s** | 满杆实测达成率约 75%，实际约 0.75 m/s |
| turn | **3.0 rad/s** | 172 °/s |
| spin | **3.0 rad/s** | 172 °/s、29 rpm |
| main_rate | 0.8 rad/s | 未改 |
| fire_rate | 0.7 rad/s | 未改 |
| pitch_rate | 0.5 rad/s | 未改 |

历史量程：2.828/9.256/9.256 → 2.828/9.256/6.0 → **1.0/3.0/3.0（当前）**。

**3. 保守版已烧录并运行**

- 镜像 `site-candidate-20261104-t3s3`，BIN SHA256 `49df79b6b5778466508215b9f309167e7edf658eac6fd20164bea3adfb4ec125`，215928 B。
- 烧录：核 UID → 擦 sector 0–5 → 写 → verify → 独立回读，**逐字节一致**。
- 已 `reset run` 启动；SWD 读得 PC=`0x080218fa`、CFSR=`0x00000000`，**无 HardFault**。

**4. main 航向摆动：已定位到姿态估计算法（离线复现，未经 PID）**

复现程序 `source/.tmp/yaw-review/minimal_accel.cpp`，只链接真实 `attitude.cpp` + `sentinel_kinematics.cpp`，**不含 PID、执行器、遥控映射或动力学模型**：

- 真实航向固定在 0.7 rad，陀螺仪扣偏后为 0（平台真的没转）。
- 加入水平加速度 `[2sin(6t), 2cos(6t), 0] m/s²`，模长偏离重力仅 0.2018 m/s²（低于 0.8 门限）。
- 结果：**估计航向偏差峰值 10.37°**；相同输入关闭加速度校正是 0°；重复运行结果完全一致。

机理定位：
- `src/estimation/attitude.cpp:413` 以"加速度模长与 g 的差"筛选，水平运动加速度可混进重力观测。
- `src/estimation/attitude.cpp:338` 的 `dx[3] = 0.0f` 只冻结四元数 z 分量，**不等价于保持 Euler yaw**；删除该行也不能修复，反而使同工况假航向由 10.37° 恶化到 18.18°（2026-11-07 复测）。
- ⚠️ **更正（2026-11-07）**：本文原称"生产代码未改"不成立。当时 `attitude.cpp` 已删除该行，偏离了 `SOURCE_BASELINE`（`d4602f1f…331ec`）。已回滚至基线并新增 `tests/attitude_tests.cpp` 回归用例；缺陷根因仍未修复。详见 `results/REPORT.md` §13。

**5. 遥测链已打通并录制基线**

- 新增只读全量记录脚本 `source/robots/sentinel/host/sentinel_record_all.py`（单客户端持有序号，逐页存原始响应，不申请控制、不发目标）。
- 关键端口事实：**APP 业务串口 = `/dev/ttyACM1`（DAP `332107148384`）**，921600 8N1，握手 `identity=sentinel_app` 成功。
- 已录 60 秒静止基线 `/home/jetson/captures/app-all-20261104.jsonl`：600 样本、page0/1/2 各 600、0 错误、`main_platform.validity` 100% valid。
- 基线分析：惯性航向跨度 1.40°、机械关节角 1.05°、相关系数 0.185、航向首末漂移 +1.05° 而机械角 −0.35°。**幅度仅 1° 级，且全程静止，不足以判定摆动根因。**

**6. 在线调参能力已确认**

- 26 个 PID 参数可热改（`kind=hot`），改完立即生效、断电不保留。
- 输入量程、几何、零偏、关节限位**不可热改**，必须重新编译烧录。
- 已确认 `param_write` 需要 `requester_may_configure()`：**无人持权时允许配置；有遥控持权时只有持权来源本身可配置**（`sentinel_protocol.cpp:146-150`）。因此纯 host 客户端在遥控持权时写入会被拒 `NOT_PERMITTED`。

---

### 二、 核心架构/工程决策记录 (Key Decisions & Reasoning)

- **决定 1：保留"按 main 指向平移"的小陀螺坐标语义，不改为车体系平移。**
  依据：`sentinel_cycle.cpp:88` 的 `main_joint` 是**机械关节角**（line 90 只覆盖数组内的 main 条目），此前"坐标变换构成正反馈"的推断已证伪。改语义会改变操纵习惯，且不解决假航向。

- **决定 2：先修姿态估计算法，再考虑 PID。**
  依据：离线已证明 PID 之前的测量即可产生 10° 级假航向；先调 PID 只会压住表象并牺牲跟随能力。

- **决定 3：main_yaw 是唯一依赖陀螺仪的轴。**
  四轮用 ESC RPM、fire 用 GM6020 编码器、pitch 用 DM 位置反馈，均不经过姿态估计。因此"调硬 main"等于放大估计误差的执行力度。

- **决定 4：Jetson 侧自行编译 openocd 0.12.0。**
  原因：Ubuntu 22.04 apt 仅 0.11.0，对这批 Horco CMSIS-DAP 报 `SWD not supported` / `CMD_INFO failed`。已源码编译安装到 `/usr/local/bin/openocd`，依赖补装 `libhidapi-dev`、`libjaylink-dev`。语法支持 `adapter serial`。

- **决定 5：烧录只传 BIN 到 Jetson（用户明确要求）。**
  不再传 ELF/HEX，也不走本机 SWD。

---

### 三、 遗留风险与未解决阻碍 (Residual Risks & Blockers)

- ⚠️ **摆动根因未在实车闭环确认。** 仅有离线复现；实车 60 秒数据是静止基线，未抓到自旋+平移工况。**必须补录一次真实自旋+平移，对比 `heading_rad` 与 `joints.main_yaw.angle_rad` 是否同步。**

- ⚠️ **探针 UART 桥不稳定。** 实测：8308（ttyACM0）可写、384（ttyACM1）可写且是 APP 业务口、353（ttyACM2）**打开即卡死不可用**。重插后 tty 映射会变，每次须重新确认。历史记录中 `CMD_INFO failed` 只需物理拔插即可恢复，USB 软复位无效。

- ⚠️ **main_yaw 调参已被阻断。** 用户要求"调硬 main_yaw"，但写入被 `NOT_PERMITTED` 拒绝：遥控持权时只有遥控来源可配置。**需要先把遥控拨到撤运动档/待机，使无人持权，再用 host 客户端写入。**

- ⚠️ **保守量程仅短时验证。** 1.0 m/s / 3.0 rad/s 未做长时间稳定性、打滑、过热验证；75% 达成率是按单次实测反推，非标定结论。

- ⚠️ **`gravity_ff = +0.8 N·m` 只验证了"放手不下垂"**，未做不同 pitch 角下的前馈标定。

- ✅ **回传证据已补（2026-11-08）**：安装照片与动作视频在 `results/media/app-20261108/`；五步联调原始 wire 日志在 `results/logs/app-20261108T121953/`，动作区间按 `wheel_odom.twist` 自动划分并写入 `results/REPORT.md` §12.5.1。划分依据是运动量而非操作员逐段标记。§6.4/§6.5 的采集留档要求现已满足。

- ⚠️ **`results/REPORT.md` 目前只覆盖到 11-03 换板联调（§11），未包含 11-04 的量程下调、姿态估计审查结论与本次基线数据。**

---

### 四、 下一会话恢复行动指南 (Next Immediate Actions)

1. **先确认环境**：`ssh jetson-111 'ls -l /dev/serial/by-id/ | grep -i horco'`，确认哪支是 ttyACM 几；用 `python3 probe_port.py /dev/ttyACMn` 逐个试到 `APP_OK identity=sentinel_app` 为止（当前应为 ttyACM1，但重插会变）。

2. **补录自旋+平移工况（关键）**：
   ```
   ssh jetson-111 'cd ~/sentinel-host && python3 sentinel_record_all.py \
     --port /dev/ttyACM1 --seconds 60 --output ~/captures/spin-translate.jsonl'
   ```
   动作建议：小陀螺档静止 15 s → 推左杆前后 15 s → 推左右 15 s → 回中 15 s。录完用 `compare.py` 对比惯性航向与机械角。

3. **若证实是假航向**：修改 `src/estimation/attitude.cpp` 的加速度观测准入与重力校正路径（水平加速度不应污染航向），在 `.tmp/yaw-review/minimal_accel` 上先做到 `FIXED_TRUE_HEADING_UNDER_ACCELERATION=PASS`，再回到实车同工况对照。

4. **若要继续 main_yaw 调参**：先把遥控拨到撤运动档（无人持权），再执行 `param_write`。建议幅度：位置 Kp 28→35~42、Ki 2→3，一次只改一个，改完立即做同工况对比。**在假航向未修好前，调硬的收益与风险都需实测确认。**

5. **补文档**：把 11-04 之后的量程变更、姿态估计审查结论、基线数据补入 `results/REPORT.md` 新增小节。

**关键路径速查**

| 内容 | 路径 |
|---|---|
| 当前量程与几何 | `source/boards/sentinel_f407/include/board_app_integration_site.hpp` |
| 联调配置（PID/限位/前馈） | `source/boards/sentinel_f407/include/board_app_integration_config.hpp` |
| 姿态估计 | `source/src/estimation/attitude.cpp` |
| main 惯性与分支反解 | `source/robots/sentinel/app/sentinel_execution.cpp:748-840` |
| 遥控映射（三档） | `source/robots/sentinel/app/sentinel_remote.cpp:331-391` |
| 全量记录脚本 | `source/robots/sentinel/host/sentinel_record_all.py` |
| 离线复现与结论 | `source/.tmp/yaw-review/README.md` |
| 保守固件 | `source/build/site-app-20261104-t3s3/boards/sentinel_f407/sentinel_app.bin` |
| 实车基线数据 | Jetson `/home/jetson/captures/app-all-20261104.jsonl` |
