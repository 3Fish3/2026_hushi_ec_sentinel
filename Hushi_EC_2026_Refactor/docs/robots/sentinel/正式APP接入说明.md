# 哨兵正式 APP 业务协议接入说明

当前业务版本为 **v3**. 不兼容 v2 的目标、状态与诊断布局; 帧层仍为 v1.
实现: `robots/sentinel/app/sentinel_protocol.*`, `sentinel_telemetry.*`, `sentinel_link.hpp`.
主机: `robots/sentinel/host/sentinel_client.py` (编解码/顺序调用), `sentinel_async.py`
(异步收发拥有者), `sentinel_test.py` (最小分机构测试).

不提供任意 CAN/内存写、反馈注入、电调保存设置或 MCU Flash 参数保存.
`examples/sentinel_app_host_link.cpp` 的注入与合成负载只用于离线测试, 不属于协议.

## 1. 分帧与物理端口

复用 `include/ec/transport/uart_frame.hpp`:

```
7e | version=01 | type u8 | sequence le16 | size le16 | payload | crc16 le16 | 7e
```

- `7e` 转义为 `7d 5e`, `7d` 转义为 `7d 5d`; CRC-16/CCITT-FALSE 覆盖帧版本至载荷.
- 通用载荷上限 256 B, APP 业务上限 240 B. 不传输 C++ 内存布局.
- 外层类型为载荷声明的响应类型, 外层序号回显对应请求的帧序号.
- CRC/分帧/帧版本错误丢弃重同步, 不回业务 ACK. 业务版本/长度错误有明确错误响应.
- 正式 APP **USART1 = C 板丝印 UART2, 921600 8N1**. 波特率是显式软件配置,
  不自动降速; 线束、适配器与真实吞吐待 H 阶段验证.
- **USART6 = 丝印 UART1, 115200 8N1**, 保留原只读观测服务.
  新增空载荷请求 `0x36` / 响应 `0xb6`, 外层序号回显, 载荷与业务快状态页完全相同
  (内层类型为 `0xc6`, 会话/业务序号为 0). 它复制已提交的一致性快照,
  不读取半周期对象、不握手、不续业务会话、不改变控制权.
- 正式 APP 的 USART6 快页不是 `sentinel_capture` 固件的采集页; 不混用旧采集 GUI 的协议.

## 2. 消息、生命周期与时效

载荷前两字节为 `version=3, type`. 其余整数/浮点为小端, 浮点为 IEEE754 f32.

| 请求 | 成功响应 | 含义 |
|---|---|---|
| 40 hello | c0 hello_ack | 新会话, nonce 回显, 版本/启动身份可用性/配置来源 |
| 41 authorize | c1 ack | 请求或释放 host / maintenance 来源; 不授权 remote |
| 42 targets | c2 targets_ack | 整车原子目标, 各域独立源年龄, 主 yaw 显式参考系 |
| 43 align | c1 ack | 机械关节参考或主 yaw 惯性对齐 |
| 44 param_read | c4 param_value | 单项或分页读参数 |
| 45 param_write | c1 ack | 单参数 RAM 生效事务与实际值读回 |
| 46 status | c6 status_snapshot | 快状态 / 域执行 / 请求与预测分页 |
| 47 diagnostics | c7 diagnostics_page | 设备、运行时、发送组与协议诊断 |
| 48 permit | c1 ack | 有时限的正向发射许可, 不联动摩擦轮和人工反转 |

### 会话、唯一请求身份与变更重试

- HELLO 总是替换旧会话并撤销旧会话请求. **同 nonce 不代表幂等 HELLO**;
  异步客户端不自动重发它, 延迟旧 ACK 不能被当作新握手成功.
- HELLO 后显式 AUTHORIZE, 并持续提供新目标. 首启有效遥控运行态、人工停止和现场选源
  仍由 MCU 仲裁. ACK 接受不等于已经持权; 以状态页为准.
- 维护资格与身份隔离沿用既有契约: 人工显式选源抢占维护, 静态拨杆不反复抢占;
  新会话/新申请不继承旧目标. 同周期人工选源仍优先. 遥控失联不自动变成自主来源.
- 除 HELLO 外使用 `(session u32, sequence u32)`. 一个 UART 拥有者分配递增业务序号;
  序号到头需新会话, 不在原会话回绕到 0. 帧序号是另一个 u16, 不能替代业务身份.
- MCU 缓存最近普通结果, **另保留最近一次 AUTHORIZE / ALIGN / PARAM_WRITE 结果**.
  因此连续 TARGETS / PERMIT / STATUS 不覆盖最近变更结果.
- 在当前仍有效的会话内, 原身份且原载荷逐字节相同的变更可返回原结果, 不重复生效.
  同号异内容、其它倒退序号及已被后来变更替换的结果回 `REPLAY`.
- 主机同时最多一个待确认变更. 丢 ACK 只原样重发原线帧; 不为重试分配新业务序号.
  `REPLAY`、`NO_SESSION` 或重试超时明确为 **结果未知/已失效**, 不是“未执行”或“成功”.
  先读回当前参数、参考或权限, 再决定新操作; 不盲重做对齐.
- 接收路径用当前周期时刻先老化再查缓存, 不依赖外部先调用老化. 到期请求不能复活会话,
  也不能拿旧缓存 ACCEPTED. 重复缓存命中不续链路、不续目标、不再授权.
- 只有业务被接受的新请求才刷新会话时效 (包括合法 STATUS/PARAM_READ/DIAGNOSTICS).
  业务越限、错误参数/页、权限/代次拒绝同样不续会话; 身份和新序号合法并不例外.
  已进入业务处理的拒绝请求仍消耗序号并可缓存原拒绝结果, 不允许换内容复用该序号.
- 会话失效须新 HELLO/AUTHORIZE. 缓存不跨会话/MCU 重启, 不声称掉电下 exactly-once.
  当前 `boot_identity_available=false` 时, 不得把 `boot_id=0` 当成可靠上电身份.

### 全量目标、参考与年龄

- 每包是整车原子快照: 未声明域撤销旧意图, 不沿用历史值; 任一标签/模式/限位/代次非法则
  整包拒绝, 旧有效包不续期. 底盘 twist 在 `base_link`, SI: m/s, rad/s.
- 参考标签: `joint=0`, `local_inertial=1`, `odom=2`. 除主 yaw 外只允许 `joint`.
  主 yaw 本地稳向不等待 odom 对齐; `odom` 需要已建立对齐. 外来不可达目标仍拒绝, 不裁剪.
- TARGETS 携带当前 config/reference 代次. 参考变更只作废受影响域与保持, 不顺带作废
  其它域现有的合法目标; 全局 reference 代次同时上推, 后续新包须使用新代次.
- DJI 机械 ALIGN 是恢复/确认当前配置坐标, 不是任意重命名角度. 与编码器、传动、零偏及
  范围共同确定的唯一参考矛盾时回 `ALIGN_FAILED`; 一致性确认不移动当前坐标.
  fire 的未知参考合法建立/纠正会清相关关节历史、派生 odom 对齐和旧主 yaw 惯性意图,
  新鲜下游反馈重新建立重构后只接受新意图, 不撤无关底盘目标. 坐标重定义使用零偏参数事务,
  同步平移串联标定姿态; 不以平移标定掩盖未知或矛盾的 ALIGN.
- 主机固定位置生成器锁定下发时的参考代次. 参考变化后撤销, 不把旧位置数字重新贴代次.
  PID 热调不应停止其它合法目标流; 主机只更新已确认/观测到的配置代次, 不回退到旧缓存代次.
- 每域 `age_ms u16`, twist 也有自己的年龄. 接受时刻按 `now-age` 计算; 到原期限后视为无意图.
  新包、发射许可、普通查询或会话保活都不把旧域变新. 主机邮箱滞留时间计入源年龄.
- 控制、会话和源年龄使用 MCU 单调毫秒; 采样、外推和发送统计另有明确命名的 u32 微秒字段.
  两者分别按 2^32 回绕; 不是 UTC, 也不是未经同步的 Jetson 时钟.
- 反馈年龄取采样时刻; 不可用为 `0xffffffff`. `evaluated_us`/`control_ms` 标识统一计算时刻,
  从属 `ages_us` 相对此时刻. 不把回包时刻冒充硬件采样时刻.

### 接受、采用、发送、测量不是同一件事

TARGETS_ACK 仅表示接收判定; 域页给出资格/采用目标/原因, 发送组页给出提交或 busy/error,
实际响应看有效测量. `HAL/CAN accepted` 也不代表电调已收到或已到位.
位置模式报告匹配 `measured_frame` 的角度 rad (主 yaw 可以为本地/odom), 其它模式报告
轴侧速度 rad/s. 位置/速度年龄分开, 轮系位置无参考不表示速度反馈丢失.
无目标不等于反馈失效, 数值 0 也不等于有效. 必须看有效位/域有效性/时间和原因.

发送前按实际单调微秒时刻检查组最早计算结果的 `output_ttl`; 过期输出不入本地邮箱,
不重贴计算时间、不补历史槽值. 板级已接 TIM5 时钟; 不带时钟的离线夹具不能作为耗时证明.
停止专用组仍只提交应用明确生成的零帧. pitch 真时间异常或 MIT 编码拒绝经既有域拒绝入口
作废旧意图, 仅恢复反馈/时间不会重新 enable/MIT; 普通非到期或单次 busy 不撤目标或全局来源.

## 3. 结果码

按序号 0..14:
`ACCEPTED, REJECTED, BAD_VERSION, BAD_LENGTH, NO_SESSION, REPLAY, EPOCH_MISMATCH,
NOT_PERMITTED, UNKNOWN_REQUEST, UNKNOWN_PARAM, INVALID_VALUE, REFERENCE_IN_USE,
NO_TARGET, ALIGN_FAILED, UNSUPPORTED`.

业务 TARGETS 拒绝仍可回 c2; 版本/会话/格式/重放等协议级错误回 c1.
参数读取失败回短 c1, 不能按参数表解码. ACK 精确长度只允许 12/20/24 B.

## 4. 参数与分机构配置边界

ID=`(slot<<8)|local`, DJI 槽 0..8, DM 槽 9, 公共槽 10; 以参数页返回的域为准, 不从 CAN ID 猜槽.

- `hot`: 已实现的速度/位置 PID、DM MIT 增益. 修改后从运行对象读回实际值,
  不重置积分/滤波历史, 但不声称无扰切换.
- `reference`: 零位/范围、电流 cap 等. 受影响域退出执行才可改;
  主 yaw 对齐偏移虽保留公共槽 ID, 作用域仅主 yaw. 目标 TTL 仍为整车级参数.
- 返回 `writable` 表示有生效实现, 不免除权限、状态和必要运行对象检查.
  输出编码限幅/速度界/反馈时限等无在线路径的项目为只读, 不能谎报 effective.
- 不为缺失 PID 等数据注册“值为零”的假参数. 默认值与 RAM 覆盖明确区分.
- 保存文件是**候选参数**而非验收记录; 带业务版本、来源类别、实际域与可写性.
  合成文件不得直接导入真实部署. 逐参数加载不是批量原子事务; 出错停止, 已成功项不会自动回滚.
- 测试工具只选择目标, **不修改安装声明、不屏蔽 A09 故障**. 已连接且共享 CAN 帧的设备必须保留
  真实槽位. 已知地址但最小 device 换算尚缺时, 该命令组只允许显式零命令; 控制/关节几何缺项
  但 device 已齐时, 仅本轴发零, 同组其它齐备轴仍走真实 DjiTxGroup.
- 未配完、不期望运行且从未收到反馈的机构不是人为制造的设备失联; 真实已见后失联/明确设备故障
  仍执行原 A09. 未知地址、冲突和禁止的部署来源不能构造正常输出拥有者, 需核实或明确实物隔离.
  DM 地址已知但量程未知只请求 disable, 不伪造角度/量程或发送 MIT/enable.
- 参数与真实配置回填见 [应用配置与换算基线](应用配置与换算基线.md).

## 5. 最小测试入口

以下动作示例只对**合成后端**成立, 不是推荐的实车数值:

```powershell
cmake -S . -B build/sentinel-app-host
cmake --build build/sentinel-app-host --config Release

# 三类缺项场景仍保留所有安装槽, 不打开串口或 CAN
.venv/Scripts/python.exe robots/sentinel/host/sentinel_test.py --host-backend build/sentinel-app-host/Release/ec_sentinel_app_host.exe --plant --fixture chassis --mechanism chassis --twist 0.05 0 0 --duration 2 --log build/chassis-wire.jsonl
.venv/Scripts/python.exe robots/sentinel/host/sentinel_test.py --host-backend build/sentinel-app-host/Release/ec_sentinel_app_host.exe --plant --fixture gimbal --mechanism gimbal --target main_yaw position 0.12 --yaw-frame joint --duration 2
.venv/Scripts/python.exe robots/sentinel/host/sentinel_test.py --host-backend build/sentinel-app-host/Release/ec_sentinel_app_host.exe --plant --fixture shooter --mechanism shooter --permit-forward --target trigger velocity 2 --duration 2

# 最小交互: 无默认运动目标; 输入与打印等待不停止后台 UART 拥有者
.venv/Scripts/python.exe robots/sentinel/host/sentinel_test.py --host-backend build/sentinel-app-host/Release/ec_sentinel_app_host.exe --plant --mechanism gimbal --interactive
```

交互命令: `status`, `params`, `set ID VALUE`, `target DOMAIN MODE VALUE [FRAME]`,
`twist VX VY WZ`, `permit 0/1`, `stop`, `release`, `save PATH`, `load PATH`, `quit`.
`stop` 撤销意图但保持维护连接用于读写参数, 不等于 DT7 人工停止档.
位置目标/增益由操作者提供, 不增加向导、二次确认或假定实测的默认动作量.
加载参考参数前先退出相应域执行; 运行中仅热调. 不自动试动、对齐、寻零或反转解堵.

真实业务接入用 `--serial COMx --mechanism ...` (默认仅观察), 动作还需显式目标;
并行只读用 `--observe COMy --duration ...`. 指定业务串口会 HELLO 替换业务会话,
因此不能拿它旁听 Jetson; USART6 观测才是不干扰来源的路径. 实物操作另行授权.

`--save-profile`/`--load-profile` 是本机构候选文件. 无人持权可配置, 否则由 MCU 检查来源资格.
`--set ID VALUE` 显示实际生效值; 结果未知时仅追加读回, 不自动换身份重写.
`--log` 记录 tx_attempt 和全部 rx 原始字节 (含坏帧), 独立有界写盘队列记 gap/end 丢失数;
日志不是 IMU/CAN 精确采样记录, 后者使用 [原始采集与标定指南](../../guides/哨兵只读采集与人工标定.md).

离线后端每请求推进 1 ms, 不是墙钟实时 MCU, 不模拟真实 UART/CAN 仲裁时延.
`--plant` 是纯电机合成负载; 串联惯性闭环另由原生 `sentinel_inertial_contracts` 验证.
`--inject` 文件支持 `imu ms ax ay az gx gy gz`, `dji ms domain rotor_angle rotor_speed`,
`dm ms position velocity torque`, 仅测试入口, 不形成任何实车协议注入接口.

## 6. 调度与带宽

- `appControl` 独占协议、设备与控制对象, 仍为 1 kHz. CAN/IMU/协议输入→仲裁→执行/提交共用
  `SentinelCycle`, 不能让 CLI/板级各有一份控制语义.
- 各 CAN 组独立时钟, 具体策略与尚待实测项见
  [资源与并发预算](../../boards/sentinel_f407/资源与并发预算.md).
- 每周期 UART 搬运上限 512 B、处理请求最多 4 条. 接收溢出丢弃跨缺口半帧并计数.
- 固定四个输出槽: 变更结果、目标/许可 ACK、快状态、低频页; 按此优先级, 每次最多提交一帧.
  只覆盖尚未进入 DMA 的同类旧响应, 不积压状态; 已在 DMA 中的帧不能抢占.
  底层 busy 留本槽待下次机会, 不在控制任务等待 ACK/串口完成. 非幂等结果另有业务缓存.
- 默认目标/快状态 100 Hz, 全域状态建议 10 Hz, 慢出口合计至多 20 Hz. 请求驱动, 无隐式推送.
  客户端只保留最新目标意图、最多一个变更、一个在途普通读取和有界读队列/应答跟踪.
- 最坏全转义: 目标 ACK 12 B×100 Hz、快页 214 B×100 Hz、最坏慢页 240 B×20 Hz、变更 ACK
  24 B×20 Hz, 加帧头/CRC/分隔后至多 **60080 B/s = 600.8 kbit/s** 单向 8N1,
  低于 921600. 实际编码双向预算有测试; 115200 不够这组计划频率.
  这是静态上界, 不是已验证的线束时延、任务 WCET 或总线利用率.

## 7. 观测与待测边界

注册表仍有 source / axis / device 记录, USART6 快页则提供完整已提交载荷.
`config_complete` 不能代替单域执行资格, `startable` 不能代替反馈或真实布局核对.
默认部署表不填未测参数; 选机构不等于物理隔离. 原始只读采集角色的白名单和零动作审计不放宽.

软件验证可证明编解码、请求生命周期、原生控制/CAN/传感器回传与有界队列; 不能证明真实电调频率、
安装几何/零位、动力稳定性、惯性漂移/温漂、重启身份、真实 FIFO/SPI 时间精度、栈高水位与 WCET.
身份字符串目前固定, 无构建哈希; 启动身份不可用时明确标记. MCU Flash、实车运动与发弹仍须另行授权.
证据统一在 [实施与验收](实施与验收.md), 不以合成曲线推导实车 PID.

## 8. 枚举

从 0 开始:

- domain: chassis_lf, chassis_rf, chassis_lb, chassis_rb, main_yaw, fire_yaw, pitch, friction_left, friction_right, trigger.
- mode: none, current, velocity, position.
- source: none, remote, host, maintenance.
- frame: joint, local_inertial, odom.
- align.kind: joint_reference, inertial_alignment; param.kind: hot, reference.
- validity: unknown, unavailable, incomplete, valid, stale, invalid.
- reason: none, not_configured, incomplete_config, no_reference, no_feedback, stale_feedback, invalid_feedback,
  no_target, stale_target, invalid_target, no_authority, source_mismatch, locally_not_ready, manual_stop,
  fault_dependency, limit_violation, controller_error, output_rejected, unsupported, awaiting_remote,
  standby, restarted, degraded, not_permitted.
- config_kind: unconfigured, synthetic_fixture, real_deployment.

## 9. ABI (可独立构造 golden)

偏移从载荷 `version` 字节算起. 每条消息前 2 B 为 `3,type`. u8/u16/u32/f32 分别 1/2/4/4 B.

### 请求

| 类型 | 总长 B | 字段及偏移 |
|---|---:|---|
| 40 | 6 | nonce u32@2 |
| 41 | 12 | session u32@2, seq u32@6, grant u8@10, source u8@11 |
| 42 | 20+9N+(0或14) | session@2, seq@6, config@10, reference@14 (均u32), count u8@18; 从19起N条 `domain u8,mode u8,frame u8,value f32,age_ms u16`; 之后 twist_present u8, 为1则 `vx,vy,wz f32,age_ms u16` |
| 43 | 20 | session@2,seq@6,domain u8@10,kind u8@11,value f32@12,reference u32@16 |
| 44 | 14 | session@2,seq@6,offset u16@10,id u16@12 (ffff=全表) |
| 45 | 16 | session@2,seq@6,id u16@10,value f32@12 |
| 46/47 | 11 | session@2,seq@6,page u8@10 |
| 48 | 13 | session@2,seq@6,allow u8@10,ttl_ms u16@11 |

### 通用响应

| 类型 | 总长 B | 字段及偏移 |
|---|---:|---|
| c0 | 47 | session@2,config@6,reference@10,mcu_ms@14,boot_id@18,nonce@22 (均u32),role u8@26,complete@27,startable@28,boot_available@29,identity[16]@30,config_kind u8@46 |
| c1/c2 | 12 | session u32@2,seq u32@6,result u8@10,reason u8@11 |
| c1 align成功 | 20 | 前12B同上,reference u32@12,config u32@16 |
| c1 param_write | 24 | 前12B同上,effective f32@12,reference u32@16,config u32@20; 格式/冲突错误可为短12B |
| c4 | 16+16N | session@2,seq@6,count u16@10,first u16@12,total u16@14; 从16起N条 `id u16,kind u8,meta u8,value f32,min f32,max f32` |

参数 meta: bit0=RAM覆盖, bits1..4=domain, bit5=整车作用域, bit6=有在线生效路径; bit7保留.
每页最多14条. 缺配置项目不填伪造零值.

### 状态响应 c6

共同头 23 B: `session u32@2,seq u32@6,page u8@10,config u32@11,reference u32@15,mcu_ms u32@19`.

**page=0 快页, 固定214 B:**

| 偏移 | 内容 |
|---:|---|
| 23,27,31 | control_ms, evaluated_us, boot_id (u32) |
| 35..37 | source u8,flags u8,config_kind u8 |
| 38..40 | main_validity, main_reason, reference_flags (u8) |
| 41,45 | local_epoch, alignment_epoch (u32) |
| 49 | main/fire/pitch 各17 B: `joint_rad f32,rate_rad_s f32,position_age_ms u32,velocity_age_ms u32,valid_bits u8` |
| 100 | q_WV 四个f32: w,x,y,z (虚拟主平台 V → MCU 局部惯性 W) |
| 116 | 主平台角速度向量 rate_V 三个f32 (rad/s) |
| 128,132,136 | continuous_heading, odom_offset, heading_gain (f32) |
| 140 | gyro_sample_us u32 |
| 144 | gyro/fire/pitch 三个 age_us u32 |
| 156 | 三个时间不确定度上界 time_error_us u32; 未知为ffffffff |
| 168,169 | wheel_validity u8, wheel_flags u8 |
| 170 | measured wheel twist (vx,vy,wz f32) |
| 182 | measured wheel pose (x,y,yaw f32) |
| 194,198,202,206,210 | pose_age_ms u32,skew_us u32,residual_rad_s f32,gaps u32,twist_scale f32 |

快页 flags bits0..5: authority,manual_stop,launch_allowed,locally_ready,boot_available,main_yaw_degraded.
reference_flags bit0=local_reference,bit1=odom_aligned. joint valid_bits bit0=位置,bit1=速度.
wheel_flags bit0=几何满秩,bit1=当前累计pose有效,bit2=累计历史未经历未知缺口.
缺口不补积分; 恢复后可从新样本区间继续累计, 但不会把已有缺口抹成连续轨迹.
wheel twist/odom 来自测量, 不是命令. V=S0 是标定零姿态的虚拟主平台计算系, `rate_V` 是该系表达的主平台角速度.
`q_WV` 的 W 是 MCU 自身惯性参考, 不暗示北向或 Jetson odom 已对齐.

**page=1 域页, 固定238 B:** 共同头之后 `source,authority,manual_stop,launch_allowed,count=10` (u8),
从28起每域21 B: `validity,mode,execute,reason,frames` (u8), `adopted f32,measurement f32,
position_age_ms u32,velocity_age_ms u32`. frames bits0..1=requested_frame,bits2..3=adopted_frame,
bits4..5=measured_frame,bit6=有请求,bit7=测量有效. 无执行时的0不是已采用的零目标.

**page=2 请求页, 固定227 B:** 共同头之后每域15 B, 共10域:
`present u8,mode u8,frame u8,value f32,age_ms u32,ttl_ms u32`.
偏移173起为命令预测 `valid u8,x/y/yaw f32,at_ms u32,integrated_ms u32,rejected_steps u32`.
偏移198为twist flags(bit0请求,bit1采用), 199起请求twist三个f32,211起采用twist三个f32,223为缩放f32.
命令预测有单独标签, 不可作反馈里程计; 无执行/未知间隙不继续预测积分.

### 诊断响应 c7

头12 B: `session u32@2,seq u32@6,page u8@10,count u8@11`.

- page0 (124 B): count=28, 后续全部u32. 顺序:
  `cycles,rejected_targets,target_invalidations,link_losses,source_changes,replay_rejects,accepted_commands,
  session_id,config_epoch,reference_epoch,config_issue_count,config_complete,frame_rejects,duplicate_requests,
  expired_sessions,boot_id,target_ttl_ms,link_ttl_ms`, 再四类 `sent[4],replaced[4],busy,pending`.
- page1 (232 B): count=10, 每设备22 B:
  `domain u8,flags u8,bus u8,protocol u8,device_id u16,feedback_id u16,age_ms u32,velocity f32,
  effort f32,temp_mos u8,temp_rotor u8`. flags bits0/1/2/3/4/5/6/7=
  installed/online/enabled/faulted/temp_mos有效/temp_rotor有效/effort有效/device已构造.
  protocol=DJI mode 或128(DM); DJI effort为raw, DM为Nm, 不统一标电流A.
- page2 (188 B): count=44, 前36个u32为
  `cycles,enable_failures,target_rejections,unreachable_targets,group_sent,group_refused,group_tx_error,
  dm_enable_sent,dm_disable_sent,dm_command_sent,dm_tx_error,dm_query_sent,skipped_groups,timing_observed,
  control_last_us,control_max_us,tx_last_us,tx_max_us,imu_accepted,imu_invalid,imu_duplicate,
  imu_regression,imu_too_old,imu_long_gap,imu_reprime,imu_extrapolations,imu_invalidations,
  alignment_changes,accel_updates,accel_rejected,reconstruction_missing,projection_rejected,
  wheel_updates,wheel_gaps,wheel_integrated_us,wheel_continuous`, 最后四轮sample_us[4]与age_us[4].
- pages10..17为实际DJI发送组下标0..7, page20为pitch. count=0表示不存在(12B),count=1时49B:
  `bus u8,id u16,result u8,flags u8,computations u32,submitted u32,tx_failed u32,invalid_steps u32,
  last_dt_s f32,max_dt_s f32,last_computed_us u32,last_submitted_us u32`.
  flags bit0=真实微秒计时可用,bit1=配置缺项只允许撤输出. 没有计时的主机夹具不得宣称零时延.

诊断页读取时刻可不同; 不把跨页数据当同一采样事件. 快页/域页本身来自一致控制快照.
测试入口与验收证据见第5节和 [实施与验收](实施与验收.md).
