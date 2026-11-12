# main yaw带载速度闭环辨识

V2是显式诊断扩展, 不改变比赛控制模式或默认PID. 底盘静止、fire/pitch固定, 使用实际负载与正式`heading_rate/heading_gain`速度反馈. 绕过位置外环但复用`DjiAxis.position_`原有速度PID; 普通targets不新增速度/current语义.

## 1. V2变更与试验依据

首轮1rad/s速度参考辨识已完成高频采集, 但工作点未建立: 实际物理速度PI为0.18/0.005/0, 积分时间36s, 3s保持段不足以补负载偏置. 4Hz速度参考幅值仅0.025rad/s, 有效电流扰动约3–6mA. `command_a`已经是A, 全程峰值约0.27–0.29A, 不能再乘3解释为0.8A.

V2在闭环输出端注入独立电流, 控制律为:

```text
i_total = clamp(i_existing_PID + bias_a + excitation_a, -current_limit_a, current_limit_a)
```

- 保留原PID、参考/权限/TTL/停止链, 总电流沿用轴/电调可编码范围. 不扩大3A协议范围、不自动写候选增益.
- 固定偏置幅值0.18A来自首轮带载命令均值约0.17–0.20A的量级, 不是已标定的摩擦补偿. 随速度目标用5s五次曲线带入、退出, 正反取相应符号.
- 独立正弦幅值0.06A. 相比首轮4Hz的3–6mA有效输入提高约一个数量级, 但最终电流仍受闭环反馈影响, 不是保证实际总电流的同频幅值恰为0.06A.
- 保持0.25/0.5/1/2/4Hz, 不升最高频率. 每频点1周期平滑入场、4周期测量、1周期退出; 不用方波或硬反转.

上述数值是本轮**待验证试验输入**, 不冒充实测可用控制参数或实际加速度上限. `acceleration=0.8rad/s²`仅检查起停速度参考曲线, 不限制独立电流正弦引起的实物加速度.

## 2. 构建与执行角色

独立Release integration构建目录增加:

```text
-DEC_BUILD_SENTINEL_APP=ON
-DEC_APP_CHASSIS_GIMBAL_INTEGRATION=ON
-DEC_APP_MAIN_YAW_IDENTIFICATION=ON
```

保留现场site/零位/参数/pitch前馈. 常规刷写工具明确关闭辨识选项; 角色审计需同时指定`--chassis-gimbal-integration --main-yaw-identification`. 标识`ec_main_identification_signature=0x4d494432`及协议schema=2区分V2. 普通APP不分配记录环且返回UNSUPPORTED.

V2脚本在取得维护权/发目标**之前**检查schema. 不允许把旧V1固件配新脚本开始动作. 新解码器仍能离线读取V1记录, 分析方法按实际数据类型选择.

主机原生模型模拟器为`ec_sentinel_ident_sim`, 复用实际C++串级速度内环. 它只验证局部模型上的候选响应, 不代表实车验收.

## 3. 有限流程与停止语义

1. 沿用维护授权, main参考有效、无物理行程限位, fire/pitch已有保持意图; 底盘意图为0速、发射不执行.
2. 重置原内环历史, 同时平滑带入±1rad/s目标和±0.18A偏置.
3. 最多等待15s建立工作点. 连续两个1s窗口满足: 同向均速≥目标幅值的50%, 标准差≤目标幅值的20%, 两窗口均值差≤目标幅值的10%. 这是辨识质量条件, 不要求事先手调到零稳态误差, 也不是比赛运行限制.
4. 满足后才开始正弦, 记录真实扫频起点. 偏置和速度目标固定, 不在扫频时调偏置/增益. 15s仍未达到条件则以`working_point`原因结束, 不继续输出无效频点或自动提高驱动.
5. 正常完成后5s平滑退出速度目标和偏置、零目标3s, 随后撤main输出并锁存. cancel/失联/参考失效沿用原撤输出行为, 不追赶旧位置目标. 撤main意图后显式clear才可启动另一方向.
6. 正反段之间保持fire/pitch和底盘0速; 全轮结束/异常退出撤目标与维护权, **pitch随后失能, 仍需现场承托**.

单方向最短约61.5s、最长74.5s, 两段净动作约123–149s. 准备失败时单向在约20s内结束. 目标位移约53.5–66.5rad/方向(约8.5–10.6圈), 不是只有360°小摆动; 当前机械/线束需允许连续多圈, 否则返回阻塞, 不强转. 实际位移由反馈记录, 不将目标积分当作已转角.

普通快状态/域页仍是位置意图与执行许可, 不能把旧位置目标当成辨识的速度目标; 实际试验状态以0xc9、速度/电流以0xca记录为准.

## 4. 上位机操作

业务口为USART1/丝印UART2, 921600 8N1, 不与维护台/Jetson并行占口. 使用既有遥控首启/维护与人工停止方式. 不加第二串口或新GUI.

```text
# 只预览计划, 不打开串口
python robots/sentinel/host/sentinel_identify.py --serial COM3 --output results/main-ident-v2
# 一次执行正向辨识和反向留出, 不改PID
python robots/sentinel/host/sentinel_identify.py --serial COM3 --output results/main-ident-v2 --execute
# 离线分析, 当前目录必须是新输出目录
python robots/sentinel/host/sentinel_identify_analyze.py --train results/main-ident-v2/positive --validate results/main-ident-v2/negative --native build/host/ec_sentinel_ident_sim.exe --output results/main-ident-v2/analysis.json
```

`--current-amplitude`/`--bias-current`单位为A, 不再用旧`--amplitude`速度单位. 本轮默认计划不在现场扫描这些参数. 异常结束即返回已有文件, 不自动重连、加幅或重试.

整轮`wire.jsonl`是原始字节; 每方向`summary.json/samples.csv`带参数、真实起点、状态与序号. `command_a`已经换算为A. 记录完整性和撤权ACK是否确认分别报告; 撤权ACK丢失不丢弃已完整数据, 也不谎报确认停止. 工作点失败会先排空已产生轨迹再报告, 不为了齐套执行第二段.

## 5. 高频协议与资源

请求0x49原包头`version,type,session u32,seq u32,action u8`; action=cancel/start/status/clear分别0/1/2/3. V2 start再接:

```text
schema u8(2), speed/amplitude_a/bias_a/ramp_s/settle_timeout_s/zero_s/acceleration f32×7,
count u8, cycles u8, frequencies f32×6
```

start/cancel/clear沿用变更去重, status不覆盖start缓存. V2 0xc9响应在原40B状态后附`schema u8(2),excitation_start_us u32`, 共45B. duration_s是最大时长, start_us是起步时刻, excitation_start_us是实际进入扫频时刻. reason=6表示工作点未建立.

0xca主动流头在原24B后附schema u8(2), 每帧最多4条48B记录:

```text
at_us/submit_us u32×2,
reference/feedback/joint/heading_gain/P_A/I_A f32×6,
command_raw/effort_raw i16×2, source_age_us/flags u16×2,
bias_a/excitation_a f32×2
```

flags低位: 控制成功/本地CAN接受/反馈有效/内环或总电流触限/测量周期分别bits0..4; 高8位stage为0起步、1准备、2..7频点、8降速、9零目标. 时基/序号均u32回绕. source_age是采用反馈的最大来源年龄, 不是传感器同步采样时刻. command_raw与真正提交CAN帧的字值原生核对, 乘有符号raw_to_axis_amp得到轴向命令A; effort保持raw, 不乘3A量程当作反馈电流.

记录环200×48=9600B, 约200ms抖动容量; UART本地接受后才消费. 覆盖计数/序号缺口明确暴露. 约1kHz、250帧/s, 无转义时约56.75KB/s, 加普通应答/10Hz快状态约60KB/s, 低于921600 8N1的92.16KB/s. 普通应答优先, 流每次最多一帧; 实际转义、吞吐与调度仍通过序号核对, 不以带宽计算替代现场结果.

## 6. 分析与交付结果

- 在实际扫频起点后的完整周期上拟合同频正余弦、常值和趋势. V2独立工具变量为注入电流D, 对象频响`P=ΣY·D*/ΣU·D*`, U是最终总电流, 不只是扰动分量.
- U使用本地CAN提交时刻, Y使用真正参与PID的反馈时刻. D使用生成时刻; 同一D相位在比值中消去. 不人为去除传感器/反馈延迟.
- 另用控制计算时刻的U估计`S=U/D`, `T=1-S`, 给出当前闭环互补灵敏度. 这是根据输出端扰动推断的参考闭环, 不是额外执行速度阶跃. V1仍用Y/reference, 图和JSON标明方法.
- 各频点单列`rejected_by`: 周期离散>25%、中位同频幅值/残差噪声比<2、非连续转动. 不把“一个条件失败”写成三个都失败; 输出分阶段实际速度、电流A统计, 避免单位误读.
- 至少3可信频点且跨度4倍才拟合`a exp(-Ls)/(s+b)`. 训练最大复数相对误差≤20%, 反向留出≤25%; 不一致则只返回频响/原因. 不宣称已分离Kt/J或任意姿态动力学.
- 候选交越不超可信最高频率一半, 相位裕度目标至少60°, 经实际C++控制器小信号模拟. 无通过项就明确返回, 不自动写PID. 有候选仍`deployment_ready=false`, 内环单独验证后才重新匹配位置外环.

V1无可用模型不代表必须更换控制算法. V2本轮停点是有效频响/验证模型或明确受阻原因, 不以重复试动代替结论. 方法背景: [闭环PID频响辨识](https://www.mathworks.com/help/slcontrol/ug/closedlooppidautotuner.html)、[输入与扰动相关性](https://www.mathworks.com/matlabcentral/answers/11670-closed-loop-system-identification-toolbox).

## 7. 固定候选的15s无偏置速度验证

研究后的有限验证是独立profile `main-yaw-zero-bias-15s/1`, 不是再扫频. 沿用辨识构建选项, 标识更新为MID3 (`0x4d494433`), V2采集协议和48B流保持可用. 普通APP仍不分配记录环, 不支持本能力.

候选速度PID为电流口径1.2/1.05/0, 在3A量程下MCU值0.4/0.35/0; 配套位置PID6.3/0.45/0. 位置环在本次执行中绕过. 这组组合按旧0.060/42的总环增益比例配套缩放, 不能把新内环与旧42/3位置环混用. 候选仅RAM应用, 不自动改变正式构建默认参数, 不持久化0.18A诊断偏置.

四档单变量对照(同一15s轨迹): Ki=0.4末秒角跨度6LSB不合格; Ki=0.3正反末秒均速误差-0.038/+0.051超0.025被淘汰; Ki=0.15因电流超限在3.6s提前结束; Ki=0.35两次`A_numeric`(末秒角跨度1LSB, 正反误差+0.011/+0.010), 第三次6LSB. 已核查零速段: 积分项在进入零速时即带-0.10~-0.125A反向残余(源自反向平台3s累积-0.14~-0.23A), 残余越过约0.107A静摩擦线时轴在相邻几格抖动(6LSB≈0.264°), 属该工况物理边界, 不是PID参数可消除项. 因此不继续对零速段加补偿, 也不放宽3LSB判据.

2026-11-11 起, 普通联调固件的main_yaw默认值按该结论更新为速度0.4/0.35/0、位置6.3/0.45/0. 正式默认参数与诊断候选此前分离, 现在两者一致; 位置环仍未单独闭环验证, 整车横移急停的方向保持效果需实车验收.

固定波形: 0–2s零速; 2–2.5s半余弦升至+0.25rad/s; 2.5–5.5s保持; 5.5–6s回0; 6–8s零速; 8–8.5s至−0.25rad/s; 8.5–11.5s保持; 11.5–12s回0; 12–15s零速. bias和独立电流激励始终0. 单次start中途不clear、不重置积分, 每向目标积分约0.875rad/50.1°. 回零是速度回0, 不是回到起始位置.

MCU在送CAN前检查本轮退出线: |未饱和电流需求|>0.35A或|采用速度|>0.4rad/s时, 当前周期明确发送零输出并锁存结束, 不夹住输入继续采集. 此退出线只属于本诊断profile, 不修改比赛3A协议范围或既有运动契约. 原TTL/遥控停止/参考/反馈/CAN失败链保持. 本profile要求六项增益成套匹配且量程3A, 活跃期间拒绝主轴参数热写, 若运行配置被其它内部路径改变则结束. 完成或退出后不自动追旧位置目标.

上位机先读能力, 再维护持权、撤main意图并等到不执行, 保存实际六项增益, 成套写候选并读回, 发一次15s动作. finally撤main意图, 在确认main空闲后恢复旧值并读回, 最后撤目标/维护权; **不自动clear结束锁存**. 未确认恢复/撤权会单独报告, 不自动重试动作. fire/pitch在动作和正常恢复阶段保持, 最后撤权后pitch失能, 沿用现场承托. 不用本机占位配置当回退值.

```text
# 预览, 不开串口
python robots/sentinel/host/sentinel_velocity_validate.py --serial COM3 --output results/velocity15
# 一次执行, 构建文件用于自动记录身份; 实刷对应关系仍由现场烧录日志核对
python robots/sentinel/host/sentinel_velocity_validate.py --serial COM3 --output results/velocity15 --firmware-bin build/ident/sentinel_app.bin --firmware-elf build/ident/sentinel_app.elf --execute
# 离线重算时域指标/绘图
python robots/sentinel/host/sentinel_velocity_validate_analyze.py results/velocity15
```

输出为wire.jsonl、samples.csv、trial.json、summary.json和analysis.json/png. 原始流保留失败/触发样本: terminal bit5=32, 正常结束时P/I=0且command_raw=0; 退出线触发时P/I保留需求分项而实际command_raw=0, 所以终止样本不能用“command=P+I”核验. 普通样本仍核验到编码量化精度. CAN未接受最终零帧不报动作完整结束. reason新增7电流退出、8速度退出、9固定增益被改变.

只读action=4返回能力及实际参数, 不覆盖start去重缓存; action=5启动上述固定profile, 无附加波形参数. 两者请求仍为0x49. action4的0xc9响应在原45B后附`extension u8=1, profile u8(0扫频/1验证), f32×14, u32×3`, 共115B. 14个浮点依次为验证时长/电流退出线/速度退出线、内PID三项、外PID三项、内积分限幅(归一化)/外积分限幅(rad/s)、电流量程、max_dt_s、feedback_alpha; 三个整数为内周期/外周期/内环最大步长(us). 新脚本在授权、写参数和发目标之前检查能力, 不拿旧V2固件直接试动探测.

分析按真实MCU时基计算平台最后1s均速误差(≤0.025rad/s), 最后1s角跨度(≤3 encoder LSB≈0.002301rad)和速度RMS(≤0.02rad/s), 同时核对平台末秒编码器与采用速度均值差≤0.025rad/s, 回传每个平台误差RMS/跨度/I斜率及整个末尾3s零目标段的位移. 输出`A_numeric`仅表示这些数值筛查与恢复/撤权完整, 连同一次动作的平顺/振动观察决定后续; 不由均速合格推断没有持续振荡. 数据缺失为incomplete, 提前结束或不满足判据为B. 不修改比赛精度要求, 不自动应用下一候选或扩展位置/小陀螺试验.
