# 定位保持 → 人工检查确认 → 独立开始激励

本文件描述本轮新增需求及交付边界。既有2026-11-01交付包不包含本轮功能，不能使用旧bin冒充更新固件。

## 人工操作契约

1. 操作者选择维护模式并显式取得维护控制权。切入模式本身不能自动定位或启动。
2. 上位机下发本次实验的准备位置与明确的移动边界。准备位置不是程序推测的“安全点”，须由操作者结合现场决定；禁止默认把机械范围中点当作安全位置。
3. MCU受限速移动，持续观察真实位置/速度与有效反馈，满足配置的位置误差、速度阈值、稳定时间后保持在准备位置。程序不能把目标指令值当成实际到位。
4. 上位机显示保持状态并等待操作者检查。到位后不自动ARM/START，等待期间不因界面刷新而生成确认。
5. 操作者单独点击“确认准备完成”。该确认只属于本次会话、轴、准备位置、配方和参数/参考版本；不是通用永久批准。
6. 操作者再单独点击“开始激励”。启动时MCU重新检查确认和反馈，才执行轨迹并记录。一次按钮操作不能隐含完成定位、确认与开始三个动作。

## 必须失效的情况

- 未取得/失去维护控制权，人工停止、反馈失效、链路/心跳中断、时间不连续或超时。
- 确认后位置或速度不再处于配置的稳定范围；更改轴、准备位置、轨迹、session/nonce、参数/配置/参考版本。
- 旧确认、旧响应、不同会话的数据不能恢复“已确认”状态；重连必须重新定位/检查。
- 配方或反馈NaN/Inf、整个激励行程越界，拒绝操作，不填入猜测阈值。

失效不能自动重新确认或自动重新启动。反馈恢复后仍需要新的人工检查。界面“指令已发/ACK已收”与MCU“到位稳定/确认有效/正在激励”分别显示。

## Pitch与停止边界

普通等待人工检查不是停止指令，必须保持原有明确的位置保持目标，不能把等待实现成撤掉力矩。正常退出/取消、紧急人工停止、授权/反馈失效应分别表达，不能声称失联后还能保证支撑Pitch。现有停止策略、防坠证据和物理支撑要求继续有效；本轮人工确认只证明操作者完成这次检查，不替代guardian/机械限位/停止/防坠批准。

## 开发与验证边界

只修改软件并进行离线验证；不自动连接串口、不使用调试探针、不烧录、不对实机发目标或写参数。新协议中的定位也是运动操作，不能为了允许定位而绕过原硬件证据门槛。仿真不能走真实CAN目标通道。

## 本轮已接线的软件流程

实际上位机的“实验定位 / 人工确认”页提供独立的 PREPARE、ARM、POSITION / HOLD、HUMAN CONFIRM、START 按钮。加载 JSON 只校验草稿，不发指令；每次动作都必须单独点击。状态刷新不能代替人工确认。JSON 分为 recipe、policy、capture 三部分，界面列出完整字段；不附未经测量的实车默认阈值。

- PREPARE 绑定完整轨迹、实验 session/nonce 与版本；ARM 本身不启动轨迹。POSITION 以五次平滑参考移动到配方 center_rad；位置误差、速度阈值、反馈时间戳和连续稳定时间共同决定到位。
- 等待人工检查期间保持同一位置参考，不自动打开记录。HUMAN CONFIRM 只表示操作者本次显式确认，不能替代现场安全证据；START 必须另点。
- 完成激励后仍保留位置参考：MCU State::armed / PositionStage::holding_complete。CANCEL 撤销确认、保持位置，不承诺撤掉力矩；移动中拒绝 CANCEL。STOP 为单独实验停止，不是物理急停。
- 显式保持期限从首次进入等待人工状态开始，确认/开始/完成/取消都不会无限延长期限。反馈失效、授权/模式变化、超时或漂移会使确认失效；不保证故障后 Pitch 仍有支撑。
- 实验取得目标拥有权后，Async 拒绝普通 TARGETS 竞争；重新 HELLO 才释放该主机侧独占状态。所有协议、原始记录请求使用同一个 UART owner，不创建第二串口读线程。

## v3 协议补充

外层业务协议仍为 v3，experiment=0x60、snapshot=0xca；内层新增版本3，保留旧版1/2编码。内层12字节头为版本、op、axis、pattern、experiment_session(u32)、nonce(u32)，均使用既有小端编码。

| op | 动作 | 内层长度 / 附加字段 |
|---|---|---|
| 3 | PREPARE | 88 B，既有完整配方 |
| 4 | ARM | 12 B |
| 8 | POSITION | 48 B：human_token@12；位置容差@16、速度容差@20、移动速度限值@24 为 f32；move_us@28、stable_dwell_us@32、position_timeout_us@36、hold_timeout_us@40、feedback_max_age_us@44 为 u32 |
| 9 | HUMAN CONFIRM | 16 B，human_token@12 |
| 5 | START | 12 B，必须已有本次有效人工确认 |
| 10 | CANCEL | 12 B，撤销确认且保留授权保持 |
| 6 | STOP | 12 B，独立停止流程 |

v3 回包160 B：前128 B延续v2；PositionStage@128，129..131保留为零；token@132、config_epoch@136、reference_epoch@140、target@144、actual@148、velocity@152、feedback_sample_us@156。位置/速度为f32，其余u32。阶段0..8依次为 none、moving、settling、wait_human、confirmed、running、holding_complete、cancelled_hold、invalidated。未知实测值不得填成到位值。

## 有限记录与 START 的时序

定位、保持和人工确认不要求短窗口已开启；仅分离记录准入条件，所有真实授权/guardian/反馈/限位/执行器证据仍逐周期检查。START 和正在激励必须满足真实记录条件。

capture 必须显式给出 run、scheduling_margin_us、全新 output_path。点击 START 后先查询记录器，拒绝覆盖已有活动/冻结非空记录，再开启同轴 Profile4 短窗口。窗口=实验时长+显式调度余量，必须小于256000 us且满足512条容量。收到原始记录ACK不代表激励已开始。

上位机从开启请求排队前计时，队列到 UART 实际发送均检查 START 截止时间；过期不重发。MCU收到 START 时还检查剩余窗口能覆盖完整激励时长，防止链路延迟消耗预算。记录状态与目标/确认状态分别呈现，不伪造 backpressure 或停止证据。

结束后单独点击“读取冻结短窗口 / 导出 .sraw”。导出验证 run、轴、时间窗、丢条、连续序号和 Kind6/7 原子配对，以排他创建方式保存，不停止电机。失败不自动清空原记录。读取/导出未在实机验证。

## 验证结果与交付限制

- 针对此流程的 C++/Python 相关 CTest：11/11通过。四组 Python 用例合计54项，包含真实 Async owner 的模拟 transport 循环、会话切换、迟到ACK、人工确认失效和 START 截止时间；均无真实设备I/O。
- 最终全量 CTest：66/67通过，36.43 s；唯一失败 doc_link_contracts：53个 Markdown、243个本地链接中225个失效。此项没有隔离基线证明，不能声称全绿或断言全部早已存在。
- ARM Release 编译通过；bin 249792 B，SHA256 1bf629f63df503392d9cc735dcdd09638ad9a8a8af3851de36ab465e94e2fdf2。静态RAM 107512/131072 B（82.03%），FLASH 249792/1048576 B（23.82%）。仅观察到 newlib _close/_read/_write/_lseek 未实现链接警告。未测动态栈、WCET或现场停止时延。
- 默认 MCU 仍 disabled / hardware_blocked，simulation=false；上位机取消 Hardware blocked 草稿开关也不能解除 MCU 封锁。缺少 guardian 实时批准/版本/lease 的证据源；Pitch 防坠/停止未验证。因此不是已解锁实机定位/激励版本，也未执行热加载验证。
- 本轮产物为更新后的 positioning 软件包；旧 gimbal-excitation-20261101-blocked 包只对应前一轮交付。当前固件仍保留普通遥控执行能力，不能作为无输出固件使用。

构建与回归命令从 source 目录执行：

~~~sh
cmake --build build/host-tests -j4
ctest --test-dir build/host-tests --output-on-failure
cmake --build build/gimbal-remote-verify-arm -j4
~~~

此文档与测试结果均不是现场安全批准，不授权连接、烧录或实机运动。
