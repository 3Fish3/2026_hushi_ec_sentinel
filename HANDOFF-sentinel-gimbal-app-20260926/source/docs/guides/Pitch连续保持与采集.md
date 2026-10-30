# Pitch 连续保持与采集

## 范围与验收状态

此工具复用 `AsyncSentinelClient` 的单串口线程，在采集、分段和另一终端分析数据时持续续发同一个 Pitch 位置目标源。只发送 Pitch 位置目标，不写 PID、零偏、参数、发射许可或其他电机目标，不修改固件通信超时。它不是自动 PID 搜索工具。

- 主程序：`source/robots/sentinel/host/sentinel_pitch_hold.py`。
- 监督状态机：同目录 `sentinel_pitch_hold_model.py`。
- 只读分析：同目录 `sentinel_pitch_hold_report.py`。
- Python 3.11 及以上，使用仓库 `.venv`；实机入口仅面向 Linux。
- 软件测试使用原生 `ec_sentinel_app_host --plant --fixture gimbal` 和 Linux 伪终端，不连接实物。
- 实车独立承重、真实温升/力矩、失能反馈和串口拥塞仍需现场验收。模拟器固定回报 `enabled=True`，所以模拟测试只确认 APP 停止执行，不伪造真实失能证明。

**停止、断联、超时和掉电都会撤销保持力矩。必须有能承受云台重量的机械支撑/制动措施，不能以这个软件替代防坠措施。**

## 云台维护界面的辅助调参页

工具中心进入“云台控制 → 上位机调试 → 辅助调参”。原来的目标下发、状态曲线、参数草稿/读回和人工 `kp/kd` 修改仍保留。

1. 连接云台 APP。界面从 MCU 参数读取已配置的关节上下界，水平目标初始为 `0 rad`。这是此前标定的关节水平零点，不是重新测出的 IMU 水平；应核对其与实际姿态一致。修改 `kp/kd` 不改变该零偏。
2. 检查水平目标、抬升速度/最长时长、到位误差、稳定速度和连续稳定时长。默认数值只是待确认草稿，不是实车额定值。
3. “调整到水平并保持”先在当前位置启用，再由已有目标限速器向水平移动。抬升期间使用覆盖整条路径的包络，到位后收窄为采集用的小包络。只有实际角度、角速度与新鲜反馈持续满足阈值才进入等待采集状态。
4. 点击“开始采集并叠加方波”，仍使用同一 UART 会话。MCU 在固定位置保持上叠加正负力矩方波，主机异步收取 CAN/Pitch control 原始记录。不勾选“采集前自动调整到水平”时，以当前姿态开始试验。
5. 采集完成后，页面自动读回 `.sraw`，显示目标/角度、角速度、编码前馈力矩曲线，以及记录数、缺口、丢样和跟踪误差统计。`.sraw`、CSV、manifest 和 summary 保存于仓库 `build/identification/pitch/assisted-*/`。也可输入已有 `.sraw` 路径并点击“读取采集结果”。这一步不自动计算或写入可部署 `kp/kd`。
6. 辅助流程运行时，人工目标/增益/零位写入暂时锁定；完成或停止后恢复。勾选“采集完成后保持目标，等待拨杆切入遥控验证”时，方波完成后先排空采集，再继续用原目标与当前 `kp/kd` 保持。页面提示等待切遥控后再拨档；确认遥控已执行 Pitch 后，上位机撤掉自己的目标源并结束维护请求。未勾选时沿用有界后置保持再撤权；两种等待均有时限，超时会释放保持力矩。

“Pitch辨识仿真”连接选项使用原生软件后端的辨识夹具，可练习整个按钮流程。真实固件必须启用辨识并提供策略，才能开始方波；普通 APP 可以先调整到水平并保持，但采集请求会被明确拒绝。抬不动、超时、反馈过期或停止时不会继续自动进入方波。

独立命令行 `sentinel_pitch_experiment.py` 也可通过 `--lift-to-rad 0` 启用抬升，并必须同时填写 `--lift-speed`、`--lift-timeout`、`--settle-seconds`、`--settle-error`、`--settle-speed`。未提供抬升参数时沿用原来的启动姿态试验方式。

### 同一固件切遥控验证

构建时同时启用 `EC_APP_GIMBAL_COMMISSIONING=ON`、`EC_APP_GIMBAL_REMOTE_VERIFY=ON`。若需要方波，还需 `EC_APP_PITCH_IDENTIFICATION=ON` 和可信策略头。默认普通云台镜像仍不启用摇杆验证。

| 拨杆组合 | 功能 |
|---|---|
| 左中 + 右上 | 上位机人工调参 / 辅助试验 |
| 左上 + 右中 | 遥控验证，右摇杆左右操纵主 Yaw，上下操纵 Pitch |
| 左上 + 右上 | 遥控验证，右摇杆左右操纵发射 Yaw，上下操纵 Pitch |
| 右下 | 人工停止 |

上述验证档不依赖拨轮，不驱动底盘或发射机构。操纵通道切换后先回中再接受新偏转，保持目标从本次仍有效的 MCU 采用值接续；没有可继承的执行目标时才从有效实际姿态建立目标。主 Yaw 验证使用机械关节参考。

人工写入、辅助试验、遥控验证共用执行器中的同一组在线增益，切档不重建控制器或加载默认 `kp/kd`；关节零偏也不改变。运行时参数仍在 RAM 中，断电/重新烧录后的持久保存未实现。此处初始遥控目标速率 0.5 rad/s（Yaw）和 0.1 rad/s（Pitch）是待现场核对的调试策略，不是实测额定值。

只有方波正常完成后才可走无额外叠加信号的遥控交接。方波仍在执行时切档属于中止，会沿用固件故障锁存规则；不会把被打断的波形标成完成。

## 使用前条件

实车固件将状态页1和诊断共用的慢回包槽限制为每50ms一份。保持监督器在 `config_kind=2` 时交替查询状态页1与单个诊断页，两种诊断轮换，不同时排队堵塞执行状态；仍保持原120ms状态新鲜度、目标TTL与链路TTL。软件后端不采用该物理串口限速。修复查询调度后应重新打开上位机加载代码，不需要为此重烧固件。

1. 确认现场机械支撑、防碰撞空间及急停措施有效；机体和人员远离运动范围。
2. 确认 DM 实机为 MIT 模式，Pitch 为 CAN2、ESC ID `0x01`、反馈 ID `0x011`。程序会核对 APP 身份和这组地址，接收反馈本身不是模式证明。
3. 使用已验证的 DAPLink 高波特率连接。APP 为 USART1、921600 baud、PA9 TX / PB7 RX、共地；以稳定的 `/dev/serial/by-id/...` 路径选择串口。
4. 关闭原维护上位机。程序先检查占用，再使用 Linux `TIOCEXCL` 排他打开；无法检查占用时拒绝继续。
5. DR16 左开关中、右开关上，摇杆回中。右开关下为硬件链路上的人工停止；键盘停止不是硬件急停。
6. 当前姿态必须处于自由行程。整个小幅包络距离两端机械止挡至少各 `0.02 rad`，否则不发送目标。不能在下止挡附近做保持测试。
7. 人工确定温度、力矩、速度、跟踪误差和最大运行时长的保守阈值。串口模式必须显式填写，模拟器默认值不能当作实车额定值照抄。

当前 Scheme B 坐标：水平为约 `0 rad`，物理下止挡约 `-0.27275502 rad`，物理上止挡约 `+0.45624477 rad`；固件上软限位为 `+0.43624477 rad`。这些是已记录的几何值，不是本工具再次测量的结果。

## 离线运行

以下命令从 Sentinel 仓库根目录执行，只启动软件模拟器：

```bash
SOURCE=HANDOFF-sentinel-gimbal-app-20260926/source
HOST="$SOURCE/robots/sentinel/host"
cmake -S "$SOURCE" -B "$SOURCE/build/host-tests" \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build "$SOURCE/build/host-tests" --target ec_sentinel_app_host -j2
mkdir -p build/pitch-hold
.venv/bin/python "$HOST/sentinel_pitch_hold.py" \
  --backend "$SOURCE/build/host-tests/ec_sentinel_app_host" \
  --arm --demo --duration 2 --output build/pitch-hold/demo.jsonl
.venv/bin/python "$HOST/sentinel_pitch_hold_report.py" build/pitch-hold/demo.jsonl
ctest --test-dir "$SOURCE/build/host-tests" \
  -R '^(sentinel_pitch_hold_contracts|sentinel_host_client_contracts|sentinel_console_contracts|sentinel_yaw_test_contracts)$' \
  --output-on-failure
```

输出文件必须尚不存在；不会覆盖历史原始数据。再次运行时换一个文件名。

## 实机命令模板

这是需要现场授权与检查的模板，不是自动运行指令。先设置 `PORT` 和经评估的 `TEMP_C`、`TORQUE_NM`、`SPEED_RAD_S`、`ERROR_RAD`、`DURATION_S`；命令中的 shell 检查防止空值。现场首次使用还须验收真正的失能反馈。

```bash
.venv/bin/python "$HOST/sentinel_pitch_hold.py" \
  --serial "${PORT:?填写当前 DAPLink by-id 路径}" \
  --arm --support-ready --accept-release \
  --physical-low -0.27275502 --physical-high 0.45624477 \
  --max-temperature "${TEMP_C:?填写温度上限摄氏度}" \
  --max-torque "${TORQUE_NM:?填写力矩上限 Nm}" \
  --max-speed "${SPEED_RAD_S:?填写速度上限 rad/s}" \
  --max-error "${ERROR_RAD:?填写跟踪误差上限 rad}" \
  --duration "${DURATION_S:?填写最大运行秒数}" \
  --excursion 0.003 --output build/pitch-hold/vehicle-first.jsonl
```

没有 `--arm` 时不打开连接。实机禁止 `--demo`，也不自动重连、重使能或调增益。启动前读取当前会话的诊断页 0，核验固件 `target_ttl_ms` 和 `link_ttl_ms`；缺失、非法或续发周期留不出余量时拒绝授权。只读保存参数后再次读取并比对 TTL，确认未变化才申请维护授权。

初始目标取**经过预检的当前角度**，不是上次运行目标。`--excursion` 是围绕本次初始角度的最大偏移，最多 `0.01 rad`，默认 `0.005 rad`。固定的命令斜率上限为 `0.05 rad/s`，同时每次增量不超过 `0.005 rad`。

`--duration` 是独立硬运行保护的上限；正常采集提前预留 `min(0.05s, duration/4)` 请求停止，不延长或忽略看门狗。实际 `normal_run_s` 写入首条日志。调度停顿导致硬保护触发仍按故障报告。

### 按键

| 按键 | 效果 |
|---|---|
| `+` / `-` | 目标增减 `0.001 rad`；越出本次小包络时拒绝该步 |
| `n` | 新建采集段；保持目标源，不撤销力矩 |
| 空格 / Esc / `q` / Ctrl-C | 请求停止并撤销授权；保持力矩将消失 |
| stdin EOF / SIGTERM | 请求停止，不进入无人值守保持 |

切段、采样与另开终端读取日志不会撤销目标。到达运行时长上限仍会停止；不能靠关闭安全超时获得无限保持。

另一终端可在原进程继续保持时只读分析：

```bash
.venv/bin/python "$HOST/sentinel_pitch_hold_report.py" \
  build/pitch-hold/vehicle-first.jsonl --live
```

分析器按状态页会话/代次/序号去重，分段输出误差均值、RMS、标准差、最大绝对值和速度峰值。实时分析只忽略末尾尚未写完的一行；完整行损坏会报错。报告保留结束错误和停止证据，不把统计结果自动转换为 PID 参数或验收结论。

## 保护与测试对应

`source/tests/pitch_hold/` 中的测试由 `sentinel_pitch_hold_contracts` 注册运行，保留失败断言，无跳过实机风险的假通过：

| 保护 | 验证方式 |
|---|---|
| 预检完成前不装目标源；首目标取实测角度；按时间限斜率 | 状态机单测与连续分段原生后端联测 |
| 授权丢失、人工停止、错误控制源、其他域意外执行、设备故障/离线 | 状态机失败注入；不自动恢复 |
| 会话/配置代次/参考代次变化、序号回退、时间戳缺失/非有限/过期 | 状态机单测 |
| 目标 ACK 过期、拒绝、先拒绝后成功或乱序覆盖 | 实际异步接收路径测试；原生后端回复层丢 ACK/拒绝注入 |
| 旧 ACK 不能完成使能握手；诊断必须来自本轮请求且仍新鲜 | 握手及延迟 future 单测 |
| 显式重新 HELLO 后清除旧 ACK 栅栏，累计拒绝计数保留 | 原生客户端二次握手、重新授权并再次保持的联测 |
| 身份/拓扑预检失败不发送控制变更；授权 ACK 丢失后仍尝试撤权 | 主机客户端失败注入，核对调用与停止报告 |
| 角度包络、机械边界、速度、温度、力矩、跟踪误差、运行预算 | 状态机阈值失败注入 |
| 监督循环或串口目标续发停顿超过 TTL 预算、目标 ACK 过期、TTL 变化 | 可控时钟单测及真实异步线程的软件后端阻塞发送注入；确认故障后不生成新位置目标，已在途旧帧单独计数 |
| 故障锁存；停止不能清掉故障；撤销失败可见 | 单测与断开原生后端的停止报告测试 |
| `+`/`-` 小步、`n` 连续切段、拒绝越界按键；键盘停止、EOF、Ctrl-C | 子进程真实 stdin 按键/信号测试，核对日志中的目标包络与段内持续保持 |
| 停止 ACK 丢失时不冒称已失能 | 原生后端丢回复测试 |
| 串口占用拒绝、Linux 排他打开 | 模拟占用测试与真实 Linux 软件伪终端 `TIOCEXCL` 测试 |
| 日志队列溢出、磁盘错误、禁止覆盖、采集段去重 | 日志/分析器单测 |

实机目标续发为 50 Hz、快页轮询为 50 Hz；纯软件后端为适配其合成配置的 20/500 ms TTL，以 200 Hz 续发。两种模式均取 `min(target_ttl_ms, link_ttl_ms) / 2` 作为保护预算，且要求至少容纳两个目标发送周期；部署配置 100/200 ms 时预算为 50 ms，软件后端 20/500 ms 时为 10 ms。日志记录读取值、发送周期和实际预算。域状态页最多 25 Hz、两个诊断页各最多 10 Hz。目标源调用间隔、监督心跳和 ACK 使用该预算与各自原限值中的较小值；TTL 运行中变化、超限或回复缺失时锁存故障，不自动恢复。Python 调度和 USB 缓冲并非硬实时，已编码或正在发送的帧不能撤回，真实串口阻塞和电机失能仍需现场测量。

本次已尝试申请授权时，故障或停止先撤销目标，再请求撤销维护授权，分别记录 `revoke`、`control_inactive`、`disabled_feedback`。申请授权的回复丢失也必须尝试撤权，因为设备可能已执行。尚未尝试申请授权的身份/拓扑预检失败只关闭连接，`revoke=not_requested`，不向错误设备发送控制变更。通信失败时状态可能为未知/未确认；程序返回失败，不声称电机已停止。确认失能也不是防坠措施。

## 静态检查与已知仓库问题

包根 `pyrightconfig.json` 只增加 `source/robots/sentinel/host` 导入路径，不关闭任何检查。独立 Pyright 应使用仓库 `.venv/bin/python` 及这个配置检查主机模块和测试。仓库根本地已有 Pyright 配置时，本轮保留其 `.venv`/`scripts` 设置，仅为 `source/tests/pitch_hold` 添加执行环境导入映射，其他目录仍采用原环境。编辑器未重新加载配置时应重载工作区，并用独立 Pyright 核验；不通过动态导入、忽略注释或关闭类型检查规避错误。

完整快照 CTest 中原有 `doc_link_contracts` 报告 224 条本地文档断链，应与本工具新增测试的结果分别记录，不删除该测试或放宽链接断言。全量重编译还发现机构契约旧二进制曾掩盖 Pitch 无限位旧断言；本轮将夹具改为当前水平零偏，并严格核验 Scheme B 数值、边界内外目标和有限位与无界标记互斥，两 yaw 的原无猜测限位断言保留。工作区已有 Pitch 几何配置及此前文档改动不属于本次主机工具变更；本轮不修改控制算法、PID 或实机固件。
