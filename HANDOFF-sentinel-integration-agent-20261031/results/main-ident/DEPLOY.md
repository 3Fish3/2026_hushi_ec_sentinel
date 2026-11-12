# main 速度辨识：部署记录（未启动辨识）

## 状态

**部署完成，未烧录、未启动辨识。** 按用户要求停在"等启动指令"。

## 一、源码接入

- 交接包 `HANDOFF-sentinel-main-ident-20261109.zip`，SHA256 `ae5d316bab218e38c59c89c23b25dae047d6fbb06821c493c9fc98e08eb3ed93`。
- 包内 45 项 SHA256 校验 **PASS**。
- 24 文件接入现用 B02fix 源树：17 个与现场基线匹配、7 个为新增。
- **补丁工具不可直接使用**：`git apply` 在本包未跟踪目录下会"全跳过但返回0"，`patch` 因 CRLF/LF 差异全部 hunk 失败。因此按 `SOURCE_CHANGES.json` 的 `files/` 参考版本逐文件写入，写入后归一化哈希 **24/24 匹配**，并保留原有 CRLF/LF 风格。
- 现场源码备份：`<HOME>/.claude/jobs/b64d51fa/tmp/main-ident-pre-062028/`（17 个原文件）。

## 二、参数处理

- 包要求 main 位置回到 **42/3/0**；现场此前为 **35/3/0**。
- `restore-main42.patch` 期望输入是 28，与 35 不符，故**未直接应用**，而是手工将 `pos_kp` 由 35 改回 42。Ki=3、Kd=0 未变。
- 速度环 `0.060/0.001667/0` 未变；pitch 重力前馈 0.8 未变。
- `board_app_integration_site.hpp` 哈希仍为 `872faf31…41a5f`，**site 未改**。
- 本包 `SOURCE_CHANGES.json` 不涉及任何 config/site 文件，确认未被动过。

## 三、构建结果

| 项 | 结果 |
|---|---|
| 主机 Release CTest | **50/50 通过**（19.85 s） |
| ARM Release integration 构建 | 通过 |
| 角色审计 | `result=pass`，installed_dji=9、installed_dm=1、synthetic=0、unknown=0 |
| 辨识符号 | `ec_main_identification_signature`、`DjiAxis::tick_identification`、`handle_identification`、`identification_stream` 均存在 |
| 固件标识 | `app-integration-main-ident-20261109` |

**固件**

| 项 | 值 |
|---|---|
| BIN | `deploy/sentinel_app.bin`，230532 B |
| BIN SHA256 | `27046d9e3b302064c8c03961a6b7573d5d2f466b1ace5f0f0682918f45fd8918` |
| ELF | `deploy/sentinel_app.elf` |
| text/data/bss | 186832 / 43688 / 71856 |

> 包内自报现场 RAM 为 115544 B；本次构建 bss 71856 + data 43688 = 115544 B，一致。

## 四、已部署到 Jetson（192.168.1.108）

| 文件 | 位置 | 校验 |
|---|---|---|
| `sentinel_client.py` | `~/sentinel-host/` | ✅ 哈希一致 |
| `sentinel_async.py` | `~/sentinel-host/` | ✅ 哈希一致 |
| `sentinel_identify.py` | `~/sentinel-host/` | ✅ 哈希一致 |
| `sentinel_identify_analyze.py` | `~/sentinel-host/` | ✅ 哈希一致 |
| `sentinel_app.bin`（辨识固件） | `~/main-ident_app.bin` | ✅ 哈希一致 |
| 交接 ZIP | `~/HANDOFF-sentinel-main-ident-20261109.zip` | ✅ 哈希一致 |

Jetson 侧 numpy 1.21.5 / scipy 1.8.0 / matplotlib / pyserial 均可用。

## 五、预览已跑通（未动电机）

```text
speed 1.0, amplitude 0.2, acceleration 0.8
ramp 5s, dwell 3s, frequencies [0.25,0.5,1,2,4], cycles 4
duration_per_direction_s = 62.5, directions = both
```

**仅打印计划，未打开串口、未下发目标。**

## 六、启动前必须满足

1. **烧录辨识固件**（当前板上仍是 Kp35 的 B02fix 版，不是辨识版）——需单独授权。
2. **机械/线束允许 main 每方向连续约 8.7 圈**（正反各一段连续旋转，不是小幅摆动）。
3. 底盘静止、上装旋转空间清空。
4. fire/pitch 固定在当前可稳定保持姿态；结束或异常退出后 **pitch 会失能**，需保持承托。
5. 遥控人工停止有效，硬断电可用。
6. 关闭占用业务口的维护台/Jetson 连接。

## 七、启动命令（用户自行执行或授权后执行）

```bash
cd ~/sentinel-host
python3 sentinel_identify.py --serial /dev/ttyACM2 \
  --output ~/captures/main-ident/run-$(date +%Y%m%dT%H%M%S) --execute
```

离线分析：

```bash
python3 sentinel_identify_analyze.py \
  --train <输出目录>/positive --validate <输出目录>/negative \
  --native <主机侧 ec_sentinel_ident_sim> --output <输出目录>/analysis.json
```

## 八、本次未做

- 未烧录、未复位、未打开业务串口发目标、未运行任何机构。
- 未应用候选参数、未写 PID、未改 site/标定。
- 未重跑 B02 A/B/C 矩阵。
