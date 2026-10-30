# 云台开机增益修复与 pitch 嗡鸣对照

- 完成时间：2026-10-29 17:18 -04:00，America/New_York。
- 仓库：`<HOME>/.codex/worktrees/b824/sentinel`，detached HEAD `37b5660fcad5608a1f7c870fdae5c281052eb9c5`。
- 工作区：原有多文件改动保留，未暂存/提交/切换分支。本阶段编辑两个板级配置头、云台遥控测试及现有交接中的错误结论。
- 用户要求：排查fire_yaw卡顿、pitch快速从下往上时持续嗡鸣；随后明确授权烧录当前在线版本。

## 当前烧录默认

| 项目 | 数值 |
| --- | --- |
| fire_yaw 原生速度 Kp/Ki/Kd | 0.09 / 2.00 / 0 |
| fire_yaw 位置 Kp | 16 |
| 对应界面电流口径速度 Kp/Ki | 0.225 / 5.00（电流上限2.5A） |
| pitch MIT Kp/Kd | 75 / 2.0 |
| pitch 遥控满杆目标速度 | 0.6rad/s |
| fire_yaw 遥控满杆目标速度 | 0.5rad/s |

零位、机械限位、pitch操作带−0.132…+0.235rad及重力前馈0.875Nm保持不变。重启后串口读回上述六个增益均匹配，`overridden=false`，即固件默认而非临时覆盖。最终状态source=none、manual_stop=false、两轴standby，pitch已下垂至约−0.255rad。未在验证阶段自动运动。

## fire_yaw 确定缺陷

此前扫参脚本用`param_write`写入原生Kp0.09/Ki2.00，持久化时误按物理口径再次除以2.5，烧录后变成0.036/0.80。17:06只读核对确认这一偏差。

`board_app_mechanism_config.hpp`现直接存原生0.09/2.00，注释明确界面口径与原生值的区别。`tests/sentinel_gimbal_remote_tests.cpp`新增`commissioned_fire_yaw_defaults_survive_boot`，同时核对新建ParamStore默认读回与执行器实际内环增益，防止重启后再次缩放。

同0.5rad/s、−0.65↔+0.35rad往返对照（100Hz控制快照去重，40ms角度差估速）：

| 指标 | 错误开机值0.036/0.80 | 修正0.09/2.00 |
| --- | --- | --- |
| 角度跟踪RMS | 0.04031rad | 0.03270rad |
| 最大跟踪误差 | 0.08060rad | 0.05502rad |
| 40ms角度估算速度误差RMS | 0.31054rad/s | 0.13351rad/s |
| 运动窗口反向次数 | 7 | 0 |

窗口统计不等同于独立事件次数或声音频率；此次参数修复改善跟随，不据此声称已排除所有机械因素。

## pitch 对照及声音边界

用户明确嗡鸣发生于整个快速转动过程。0.6rad/s、−0.08↔+0.17rad往返，对比结果：

| 增益与轨迹 | 跟踪RMS | 最大误差 | 40ms速度误差RMS |
| --- | --- | --- | --- |
| 90/2.2、线性目标 | 0.01592rad | 0.02952rad | 0.14896rad/s |
| 50/1.5、线性目标 | 0.01915rad | 0.03431rad | 0.16601rad/s |
| 75/2.0、线性目标 | 0.01672rad | 0.03086rad | 0.14840rad/s |
| 90/2.2、主机加速度限幅目标 | 0.01363rad | 0.02949rad | 0.09300rad/s |

四组角度估算均未出现运动反向。75/2.0保留接近90/2.2的跟随精度，在线应用供用户试用后，用户授权烧录该版。尚未收到“嗡鸣消失”的明确反馈；100Hz角度不能覆盖可听高频振动，声音根因仍未定。之前将嗡鸣直接归因于MIT速度目标为0的说法证据不足，本轮未修改MIT速度字段、未实现新前馈/固件加速度限幅。

采集目录：

- `build/identification/pitch/two-axis-diagnose-20261029-170844/`：服务器中断，保存了前两组partial.json和wire.jsonl，未保存完整samples。
- `build/identification/pitch/two-axis-diagnose-20261029-171108/`：从检查点续跑，保存75/2、平滑目标、fire_yaw对照及最终参数读回；samples.json/slow.json/summary.json完整。最终遥控接管被正确识别，进程已退出。
- 测试温度约pitch30°C、fire_yaw26°C；慢页采到pitch力矩峰值约0.98Nm，这不是高频/全时段峰值。

## 构建、烧录及核验

- 更新后的ARM `sentinel_app`构建成功。
- 两项云台测试重新构建并通过：`sentinel_gimbal_remote_validation_contracts`、`sentinel_gimbal_commissioning_contracts`，2/2。
- 对本阶段修改文件运行`git diff --check`通过。
- 镜像220680B，SHA256 `ef4a727ee80c25c9cecd6aea3d0069fe44f8c44b38966e77d07db8ba68c12517`。
- ST-Link V2，STM32F407，目标电压约3.24V；请求200kHz，探针实际125kHz。擦扇区0–5，`FLASH_PROGRAM_VERIFIED`、`FLASH_PROGRAM_SUCCESS`，复位运行。
- 读回全部220680B与image.bin逐字节一致；随后通过串口只读核对开机默认0x501=0.09、0x502=2、0x503=0、0x508=16、0x901=75、0x902=2，六项全部无RAM覆盖。
- 证据：`build/flash-runs/gimbal-two-axis-fix-20261029-171606/`内`image.bin`、`readback.bin`、`program.cfg`、`manifest.json`、`verification.json`。

## 下一步

用户左上/右上、摇杆回中后接管验证卡顿和pitch声音。若仍持续嗡鸣，先获取更高采样率的命令/力矩/角速度及声音或用户现场对照，再区分机械共振、增益、测量噪声与目标轨迹；不可仅放宽速度/新鲜度/稳定判据来获得通过。重现卡顿时优先核对实际开机原生参数，避免再次把临时调参值与固件默认混淆。
