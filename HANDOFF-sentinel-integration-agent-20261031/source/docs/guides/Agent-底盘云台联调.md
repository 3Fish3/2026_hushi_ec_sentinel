# Agent跨机器交接: 底盘＋云台联调

## 1. 任务、输入与包结构

目标: 按`source/references/records/10310221`两份CSV的接线、物理方向及已调PID, 完成底盘运动、main惯性稳向、fire/pitch可操控的小陀螺联调. 发射机构、ADRC和高级建模不在本轮任务中, 不再追查现场版本差异或重做整套单轴PID调试.

本包包含**当前工作树源码快照**, 包括尚未提交的修改和新增文件, 不是只导出Git HEAD. `PACKAGE.json`记录交接身份和两份固件, `SOURCE_BASELINE.json`记录源码逐文件哈希. 基础Git提交号只供追溯, 回传差异应以本包源码为基线.

```text
交接包/
  README.md                     入口与校验/回传命令
  PACKAGE.json                  本包身份、固件身份和路径
  SOURCE_BASELINE.json           精确源码基线
  SHA256SUMS.txt                 全包文件校验
  source/                       可构建源码、原始上位机、两份回报CSV
    build/app-integration-arm-Release/boards/sentinel_f407/sentinel_app.*
    build/capture-current-arm-Release/boards/sentinel_f407/sentinel_capture.*
    docs/guides/Agent-底盘云台联调.md   本文
  evidence/                     本地验证日志与镜像元数据
```

不含Python环境/安装包、SDK、工具链、Git数据库、本机Agent规则、历史原始采集或本机CMake缓存. `requirements-dev.txt`仅保留原工程版本清单, 使用接收机器已有环境. 保留收到的原ZIP, 回传工具会用它精确计算新增/修改/删除, 不依赖接收机器Git状态.

后文命令若未特别注明, 在`source/`目录执行. 原始固件来自已通过本地验证的源码版本, 本包整理阶段没有烧录或试动.

## 2. 当前停点与现场工作

需完成两类现场配置, 唯一回填入口为[board_app_integration_site.hpp](../../boards/sentinel_f407/include/board_app_integration_site.hpp):

1. 遥控满杆输入比例、固定自旋指令. 它们是输入单位换算, 不是给执行器增加运动限额.
2. **当前重新安装C板**的轴向、偏置、前向几何. 不使用09251300/09252124旧安装轴向.

原始交付镜像的输入比例为0、`integration_serial_geometry`为空, 能进行机械关节/底盘维护, 但**尚不能直接自旋稳向**. 2026-11-02现场候选已根据当前两批实录与对齐参考回填, 采用保守输入比例, 仅编译而未烧录; 原始交付镜像仍保留. 候选来源为`reference_converted`, 实际分析及构建记录在包根`results/REPORT.md`. 此角色不含发射、Jetson自主策略或自动pitch回零.

当前候选输入比例: 平移0.8m/s、普通转向1.2rad/s、固定自旋6.0rad/s、main/fire/pitch操纵速率0.8/0.7/0.5rad/s. 用户要求小陀螺按原1.2rad/s的5倍调整为6.0rad/s (约57.3转/分). 当前新参数候选身份为`app-integration-site-candidate-20261103-retune`, 烧录与实车状态见包根`results/REPORT.md`. 这些是用户指定的命令量程, 实际转速与稳向效果待现场验证. 此前比例保留在`...20261103-spin5x`、`...20261103-scale`和`...20261102`候选及历史记录中. 本次仅按下表更新云台控制增益与pitch前馈; 关节软件零偏、接线与协议输出范围保持当前联调配置.

任务内源码和工具修改放在`source/`, 数据、报告和新固件放在包根`results/`. 发现的修复、优化改动或建议均纳入回传, 区分已实现验证的改动和仅提出的建议.

## 3. 固件、串口和重建

| 用途 | 相对包根的路径/身份 |
|---|---|
| 联合APP | `source/build/app-integration-arm-Release/boards/sentinel_f407/sentinel_app.{elf,bin,hex,map}` |
| APP标识 | `app-integration-20261031-032014`, BIN206016B, RAM103120B |
| APP BIN SHA256 | `ed2a878a243c8a41ba0c5045dc7482851f5babaa68511fda34be18761acb54db` |
| 当前接线只读采集 | `source/build/capture-current-arm-Release/boards/sentinel_f407/sentinel_capture.{elf,bin,hex,map}` |
| 采集标识 | `capture-current-20261031-032021`, BIN78480B |
| 采集 BIN SHA256 | `a750840115064988ec676027fdc92cde4c6c0ac464d4e258ad07ac44535de809` |

- APP: **丝印UART2 / USART1 / 921600 8N1**. 维护台命令: `python robots/sentinel/host/sentinel_console.py --serial COM3 --mechanism integration --log-dir ../results/logs/app`. 端口改成本机实际端口; 支持底盘twist＋三关节同包, 不混入逐轮速度与twist. GUI连接只查询, 下发前通过界面取得MCU维护控制权.
- 只读采集: **丝印UART1 / USART6 / 1000000**. 启动`python robots/sentinel/host/sentinel_capture_station.py`, 输出保存到`results/`. APP使用同一观察口时是115200, 与采集角色不同.
- Windows刷写工具为`python boards/sentinel_f407/tools/firmware_tool.py`, profile选`integration`或`capture_current`, 路径按接收机环境填写. 两角色现成ELF放在工具默认位置. 其它系统也可使用本机已有SWD工具.
- MCU目标为F405/407、1MiB Flash; 当前APP/采集分别覆盖扇区0–5/0–4. 按镜像范围擦写并校验, 不做额外Flash备份、整片擦除或Option Bytes修改.
- USB串口只接TX/RX/GND, 不接VCC; PC与Jetson TX不并联. pitch失能会失去主动承重, 现场准备好物理承托和电源硬切.

包内无旧CMake缓存, 用现有编译环境配置新目录. 以下占位值先替换为本机路径/短标识, GNU工具链已在PATH时可省略`ARM_GNU_ROOT`:

```text
cmake -S . -B build/site-app -G Ninja -DCMAKE_TOOLCHAIN_FILE=<source绝对路径>/cmake/arm-none-eabi.cmake -DARM_GNU_ROOT=<工具链根目录> -DEC_STM32CUBE_F4_ROOT=<CubeF4根目录> -DCMAKE_BUILD_TYPE=Release -DEC_BUILD_TESTS=OFF -DEC_BUILD_EXAMPLES=OFF -DEC_BUILD_SENTINEL_APP=ON -DEC_APP_CHASSIS_GIMBAL_INTEGRATION=ON -DEC_FIRMWARE_ID=<本次标识>
cmake --build build/site-app --target sentinel_app
cmake -S . -B build/site-capture -G Ninja -DCMAKE_TOOLCHAIN_FILE=<source绝对路径>/cmake/arm-none-eabi.cmake -DARM_GNU_ROOT=<工具链根目录> -DEC_STM32CUBE_F4_ROOT=<CubeF4根目录> -DCMAKE_BUILD_TYPE=Release -DEC_BUILD_TESTS=OFF -DEC_BUILD_EXAMPLES=OFF -DEC_BUILD_SENTINEL_CAPTURE=ON -DEC_CAPTURE_CURRENT_HARNESS=ON -DEC_FIRMWARE_ID=<本次标识>
cmake --build build/site-capture --target sentinel_capture
```

SDK采用当前工程固定的CubeF4 1.28.3. 重建会自动执行角色审计, 手动审计分别使用`check_sentinel_app.py --chassis-gimbal-integration`和`check_capture.py --current-harness`, 另传`--elf`与`--nm`. 修改后的构建目录与包内原始产物分开. 源码路径和本次标识不同可能改变二进制哈希, 回传实际使用镜像的身份和SHA256即可.

## 4. 已固定配置和量纲

配置为[board_app_integration_config.hpp](../../boards/sentinel_f407/include/board_app_integration_config.hpp), 接线表为[board_current_harness.hpp](../../boards/sentinel_f407/include/board_current_harness.hpp).

| 域 | 总线/ID | 方向、传动 |
|---|---|---|
| LF/RF/LB/RB | CAN1 C620 ID3/1/4/2 | -1/+1/-1/+1, 19:1; CAD轮几何不变 |
| main | CAN1 GM6020 ID4, RX0x208/TX0x1FE第4槽 | +1, 1:1, 连续轴 |
| fire | CAN2 GM6020 ID6, RX0x20A/TX0x2FE第2槽 | +1, 1:1 |
| pitch | CAN2 控制0x001/反馈0x011 | 原生输出轴角, 正向抬头; 不再除Gr |

两GM使用5点因果编码器测速, 四轮使用ESC RPM. 发射CAN2 ID5/6/7只保留停止责任, 未开放控制.

### 输出和PID

调试电流限额、四轮速度缩放及GM位置外环15rad/s限幅已撤除. 输出采用C620协议±20A、GM电流协议±3A、raw±16384; pitch保留MIT跨度12.5rad/30rad/s/10N·m. 首启、停止、TTL、反馈故障和关节区间处理保持有效. 这些协议跨度不代表机械行程或连续额定能力.

2026-11-03用户提供`<PROJECT_ROOT>/参数/云台参数表.csv`作为下一轮增益来源. 表中的DJI速度PID是**2.5A下MCU归一化值**, 联调继续保留3A协议输出范围, 按`K_new=K_report*2.5/3`换算以保持物理增益:

| 域 | 新固件归一化速度Kp/Ki/Kd | GUI物理电流Kp/Ki/Kd | 位置PID |
|---|---|---|---|
| 四轮 | 约0.132888/0/0 | 约2.657761/0/0 | — |
| main | 0.0416667/0.00166667/0 | 0.125/0.005/0 | 28/2/0 |
| fire | 0.1333333/2.5/0 | 0.400/7.5/0 | 32/0/0.1 |

pitch采用MIT Kp=40、Kd=1, 常数重力前馈`gravity_ff_nm=+0.80 N·m`; 不做DJI电流换算. 此可选字段缺省为0, 配置必须有限且绝对值不超过TMAX, 仅在已有执行授权、有效目标及反馈确认Enabled后写入MIT扭矩字段; 停止和失联仍沿用disable逻辑. 本轮前馈为编译期配置, 维护台未新增在线修改项. 联调保留3A饱和输出及位置外环不限速策略, 因此与独立云台候选在大误差下的行为仍有差异.

新表的零位/命令带沿用旧云台坐标, 本轮只迁移增益和前馈, 不把这些角度直接覆盖现场对齐后的软件参考和已确认区间. 本包尚未引入独立`command_min_rad/command_max_rad`字段; 遥控目标仍按本包当前关节区间裁剪. 此区别在候选ROM快照和报告中记录, 不宣称已与独立云台镜像完全一致.

`max_axis_velocity_rad_s=0`明确表示不设软件速度上限; 空值仍为缺项. 位置PID以`limit_output=false`取消外环速度限幅, 电流内环保持协议归一化±1. `max_rotor_speed_rad_s`用于单圈解包歧义判断, 不夹运动目标.

### 软件参考

- main软件0对应编码1660, 零偏约1.273204rad; 启动取相对标记最近单圈后连续跟踪, 不恢复掉电圈数. 不捕获开机姿态为0, 不用巨大假限位实现连续惯性闭环.
- fire软件0对应编码2183, 零偏约1.674340rad; 报告可用区间在新坐标为`[-1.18116523,+0.28838839]rad`.
- pitch软件零偏`0.61131473rad`; 新坐标报告可用区间`[-0.49253831,+0.37193870]rad` (2026-11-03现场反馈下限太紧, 从-0.39253831放宽0.1rad).

fire/pitch区间是回报的可用空间, 不是硬止点实测. 实际越过配置区间仍按既有轴故障撤输出. 核对对齐时反馈; fire朝前应相对main平台定义, 若记录时main没朝前, 关联当时main角修正软件零偏. 不修改电调硬件零点.

## 5. 现场任务顺序

### 5.1 当前安装几何与输入比例

- 使用当前接线采集镜像记录静止基线、main/fire/pitch主要运动及独立留出段. 已有当前安装的合格段可复用. 参照[分阶段标定规范](哨兵只读采集与人工标定.md)§5.6: 自然串动按实测反馈处理, 不要求人工精确锁住多轴、反复回同一R0或完成无依据的指定角. pitch承托仅防坠落, 只读capture不能锁轴.
- `.sraw`与`.summary.json`一同保存. 连续流为gyro/CAN至多250Hz、accel100Hz, 用于安装标定而非快环采样验收. 检查CRC、序号缺口、源侧丢失和时间单调性.
- 标定清单显式填写物理地址`main={bus:1,can_id:520}`, `fire={bus:2,can_id:522}`, `pitch={bus:2,can_id:17}`. DM量程用当前只读寄存器回报核对. 使用`sentinel_dataset_calibrate.py`和原生C++ replay形成候选与独立验证.
- 将当前`main_axis/fire_axis/pitch_axis`、`install`、`forward`、gyro bias及参考姿态组成有效[SerialGeometry](../../robots/sentinel/app/sentinel_kinematics.hpp), 回填site头. `forward`代表主平台前向, 向量在一致参考系表达; `fire_reference/pitch_reference`按**本版新软件零偏**换算. 保留原始候选、拟合窗口和留出验证结果.
- 填translation(m/s)、普通转向/固定自旋(rad/s)、main/fire/pitch操纵角速度(rad/s). 正自旋为逆时针. 重建并记录新镜像身份/哈希.
- 软件采样预算沿用gyro50ms/关节20ms/外推5ms/gyro单步20ms. 同步检查真实频率、时间戳和CAN负载; 如需调整, 说明观测依据和验证结果.

少量传输缺口的离线分析入口为`sentinel_segmented_calibrate.py`, 用法见[分段拟合规范](哨兵只读采集与人工标定.md). 必须显式接受原始wire缺口并保留摘要和哈希, 不能把CRC或缺包标记改成完整. 本次候选前向结合用户确认的朝前/水平姿态、静止加速度与正pitch抬头符号建立, 采用枪轴垂直pitch铰轴的粗机械假设. 位置/速度时移使用明确的零时移候选基线, 未把优化时移当作实测硬件延迟.

### 5.2 遥控联调

不使用ch4拨轮, 两拨杆位置如下:

| 右拨杆 | 左拨杆 | 行为 |
|---|---|---|
| 下 | 任意 | 整车撤运动/发射请求, DJI显式0、pitch disable, 保留观察 |
| 上 | 中 | 待机/PC维护 |
| 中 | 上 | 驾驶: 左杆X/Y横移/前后, 右杆X转向、Y pitch; main保持惯性方向、fire保持相对角 |
| 中 | 中 | 云台: 左杆X改main方向, 右杆X/Y控fire/pitch; 底盘无运动请求 |
| 上 | 上 | 小陀螺: 固定自旋, 左杆按main平台指向平移; 右杆X/Y控fire/pitch, main自动稳向 |
| 其它 | 其它 | 待机, 不开放Jetson自主或发射 |

MCU沿用真实DR16运行首启. 用途变化的通道回中一次, 云台页与自旋页保留同一fire/pitch目标. 离开自旋档撤自旋, 左杆居中只停平移. IMU/关节参考不可用时撤自旋意图; 遥控失联清目标、不自动换源. pitch有命令前不自动回0.

联调顺序: 坐标/方向核对 → 普通底盘运动＋云台保持 → 原地自旋保持 → 自旋中fire/pitch操纵 → 加平移. 记录目标、实际轮速、main惯性角/速率、fire/pitch角度/速率、execute/reason和代次.

重点评价枪口相对外界是否随底盘摆动, 同时fire/pitch输入能否平顺瞄准. main负责稳主平台, fire仍是相对关节控制, 不是新增fire世界坐标控制器. 区分main残余扰动与fire跟踪误差; 自旋目标和真实底盘转速分别记录.

维护台自动保存`app-*.wire.jsonl`, 约10Hz状态用于慢变化. 需要DJI瞬态时, APP另一路UART1/115200可使用`sentinel_raw.py capture --profile control --domain 4`(main)/`5`(fire), 补齐端口、时长、输出路径. 短窗不能替代长时间多轴高频记录, pitch暂不支持control轨迹域.

## 6. 问题、优化与回传

在包根建立`results/REPORT.md`, 这是回传结论的主要入口, 写清:

1. **完成情况**: 各阶段实际做了什么、通过/未通过/未做, 未闭合项及原因; 不用离线测试代替实物结论.
2. **最终有效配置**: site头对应安装批次和全部输入比例、接线/方向/传动/零偏/区间、完整MCU参数导出. 分清ROM初值、RAM有效值和GUI物理单位, 说明与交付基线的变化.
3. **实际固件**: 使用/重建的APP及采集身份、BIN SHA256、构建方式和关键工具版本. 将最终`.elf/.bin/.hex`、构建/审计日志放`results/firmware/`; 未改的采集镜像可引用本包哈希.
4. **安装标定**: 当前照片/安装说明、原始`.sraw`及summary、mapping读回、训练/留出窗口、候选及原生回放指标. 明确各向量/参考的坐标、方向和单位.
5. **联调证据**: 原始wire日志/控制短窗、相关CSV、能说明动作的照片或视频. 实验条目对应文件和相对时间段, 记录自旋/平移/瞄准输入、异常发生条件和恢复情况. 能算的误差/速率/饱和/时效指标附计算方法, 缺信号处直接标明.
6. **发现的问题和改动**: 每项给现象、定位依据、根因或假设、修改点及对应验证. 保留任务相关的全部源码、工具、测试、配置改动, 不只回传site头或Git提交号.
7. **优化与建议**: 已实现优化列出收益和验证; 未实现建议说明理由、所需数据/工作量和推荐优先级. 尚未闭合的问题给复现办法及下一步建议.

源码工具为`boards/sentinel_f407/tools/integration_handoff.py`, 仅使用Python标准库. 从**包根目录**运行:

```text
python source/boards/sentinel_f407/tools/integration_handoff.py verify <收到的原ZIP>
python source/boards/sentinel_f407/tools/integration_handoff.py return --baseline <收到的原ZIP> --source source --results results --output RETURN-sentinel-integration-<日期>.zip
```

回传工具以原ZIP为基线, 自动生成`RETURN.json`、`changes/source.patch`(UTF-8文本)、`changes/files/`(全部修改/新增文件, 含二进制)、`changes/DELETED.txt`和校验清单, 并收入整个`results/`. 二进制和文本完整文件均保留, 仅有文本patch并不足以覆盖二进制改动. 无需Git或将本机HEAD当基线. 记录和报告文件写完后再打包, 避免封包期间文件持续变化.

交回该RETURN ZIP及其SHA256. 原ZIP用于定位精确交付基线, 无需再回传未修改的全套源码或环境. 若现场另有独立源码分支中的实现, 将相关源码/补丁和适用基线放入`results/`并在报告关联.

## 7. 已有离线证据

`evidence/`中保存48/48主机CTest、后续3/3针对性与1/1 site兼容检查、GUI smoke和ARM Release日志. 原生组合覆盖当前总线/符号、物理增益保持、无限速命令、跨多圈自旋、fire操控、IMU失效撤自旋及发射零输出. 20s合成基座旋转中main执行20000/20000, 机械累计约-15.95rad; 基座角速度由夹具施加, 不用于推断实车稳定性.

APP、当前采集及默认APP的Release审计已通过, 包内只交付前两份固件. 主机全量构建含测试中`optional<float>`整数字面量的MSVC警告; 本地PowerShell收尾曾超时, 主命令退出0及完整CTest日志已核对. 本包不携带原机编译缓存, 后续变更使用接收机现有环境验证并回传结果.
