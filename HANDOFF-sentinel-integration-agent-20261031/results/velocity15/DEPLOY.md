# velocity15 部署记录（已烧录 MID3）

## 状态

**已烧录并回读校验通过，能力页确认到位。未写 PID、未启动任何动作。**

### 烧录

| 项 | 结果 |
|---|---|
| SWD 目标 | `TARGET_OK id=0x101f6413 flash_kib=0x400` |
| UID | `0x260050 / 0x30345109 / 0x30343537`（与预期一致） |
| 擦除 | sector 0–5 |
| 写入与校验 | `VERIFY_OK` |
| 独立回读 | `readback_velocity15.bin` SHA256 与 BIN **完全一致** |
| 复位 | `reset run` |

### 运行后能力页（只读，未授权）

- `hello`：identity=`sentinel_app`、config_kind=2、startable=True
- `main_platform.validity=valid`；main_yaw / fire_yaw / pitch 位置均有效
- `source=none`、`authority=False`、`manual_stop=False`（未持权）
- `validation.version=1`、`duration_s=15.0`、`stop_current_a=0.35`、`stop_speed=0.4`、`current_limit_a=3.0`、`feedback_alpha=0.0`
- 当前实际增益仍为现场旧值：速度 `0.060 / 0.001667 / 0`，位置 `42 / 3 / 0`
- 内外环周期 1000 / 2000 µs，最大步长 5000 µs

## 一、源码接入

- 包 `HANDOFF-sentinel-velocity15-20261110.zip`，SHA256 `b0f29a5a3bd80ea641f3481bb209d47a3fbf5b1a98612558e89d958c1a26184f`
- 包内 31 项 SHA256 校验 **31/31 PASS**
- 14 文件增量写入现场源码，归一化哈希 **14/14 匹配**，二次预检 `changed: []`
- 基线为已应用的 main-ident V2 源码；14/14 文件预检前均与包方预期一致，无冲突
- 现场源码备份：`<HOME>/.claude/jobs/b64d51fa/tmp/velocity15-pre-053951/`（12 个被覆盖文件）

## 二、构建

| 项 | 结果 |
|---|---|
| 主机 Release CTest | **50/50 通过**（24.40 s） |
| ARM Release integration | 通过 |
| 角色审计 | installed_dji=9、installed_dm=1、synthetic=0、unknown_origin=0 |
| 辨识签名 | `.rodata` 实读 `33 44 49 4d` = `0x4D494433` = **MID3** |
| 校验符号 | `SentinelExecutor::validation_gains_match()` 存在 |
| 固件标识 | `app-integration-20261110-velocity15` |

### 固件

| 项 | 值 |
|---|---|
| BIN | `deploy/sentinel_app.bin`，**232716 B** |
| BIN SHA256 | `1bffd1bc4a3e3add3af970fe6d76559eba37dfefa621a6f818ee60a223ab27b3` |
| ELF SHA256 | `bed82ac5ef3181a9acc11a08f4cfccffe2336e1b5c4cdc9a471986eea5478e9a` |
| text/data/bss | 189624 / 43080 / 71864 |

## 三、已部署到 Jetson（**192.168.1.105**）

| 文件 | 位置 | 校验 |
|---|---|---|
| `sentinel_client.py` | `~/sentinel-host/` | ✅ `a634f2c6…` |
| `sentinel_async.py` | `~/sentinel-host/` | ✅ `2fd61746…` |
| `sentinel_velocity_validate.py` | `~/sentinel-host/` | ✅ `ef6e96c7…` |
| `sentinel_velocity_validate_analyze.py` | `~/sentinel-host/` | ✅ `b94162d8…` |
| `sentinel_app.bin` | `~/velocity15/` | ✅ `1bffd1bc…` |
| `sentinel_app.elf` | `~/velocity15/` | ✅ `bed82ac5…` |

远端 6 项 SHA256 与本地逐一核对一致。

## 四、地址变更处理

- Jetson 从 `.108` 改为 **`.105`**。ARP 显示两地址 MAC 均为 `48:8f:4b:61:44:ce`，确认同一台机器；`.108` 现被其它设备占用。
- `known_hosts` 中 `.105` 的旧记录（指向另一台机器）已先行备份到 `~/.ssh/known_hosts.bak-*` 后删除（`ssh-keygen -R`）。
- `~/.ssh/config` 追加 `Host jetson-105`（`IdentityFile ~/.ssh/id_ed25519_pi`，`IdentitiesOnly yes`），原配置已备份为 `~/.ssh/config.bak-*`。未修改任何既有 Host 段。

## 五、预览已跑通（未开串口）

```text
profile: main-yaw-zero-bias-15s/1
duration_s: 15
bias_a: 0, excitation_a: 0, speed_rad_s: 0.25
stop_current_a: 0.35, stop_speed_rad_s: 0.4
velocity_physical: [1.2, 1.2, 0]
position: [6.3, 0.45, 0]
```

## 六、启动前必须满足

1. 底盘 0 速，fire/pitch 保持当前稳定姿态。
3. 主 yaw 有足够空间完成每方向约 **0.875 rad（50°）** 的运动并留余量。
4. **pitch 需承托**：结束或异常退出撤权后 pitch 失能。
5. 关闭占用业务串口（尾号 `8384`）的控制台或其它连接。
6. 遥控人工停止有效，可硬断电。

## 七、启动命令（部署后）

```bash
ssh jetson-105
cd ~/sentinel-host
python3 sentinel_velocity_validate.py \
  --serial /dev/serial/by-id/usb-Horco_Horco_CMSIS-DAP_332107148384-if02 \
  --output ~/captures/velocity15/run-$(date +%Y%m%dT%H%M%S) \
  --firmware-bin ~/velocity15/sentinel_app.bin \
  --firmware-elf ~/velocity15/sentinel_app.elf \
  --execute
```

## 八、边界

- 本次验证**仍绕过位置外环**。即使 `A_numeric`，也不等于横移急停回摆已解决。
- 候选为固定组，不接受现场扫参；结束自动恢复旧增益并撤权。
- `reconstruction_missing` 此前测得约 36.4%，本次部署未处理，仍可能影响记录完整性。
