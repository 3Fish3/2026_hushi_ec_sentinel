# 电控通用核心框架

面向 STM32F407 / HAL / FreeRTOS, 以现有哨兵为首个落地机器人. 手写代码使用 C++20, 实时控制路径不分配动态内存、不使用异常. 初始化阶段与平台依赖的资源约束见 [架构设计](docs/架构设计.md). 原工程的Agent规则文件未随本交接包提供.

## 从这里开始

- [文档中心与学习路线](docs/README.md)
- [环境部署](docs/getting-started/环境部署.md) → [第一次构建与运行](docs/getting-started/第一次构建与运行.md)
- [项目分层与代码导航](docs/guides/项目分层与代码导航.md)
- [应用装配与接入指南](docs/guides/应用装配与接入.md)
- [模块使用手册](docs/modules/README.md)
- [板级生成与构建](docs/boards/sentinel_f407/生成与构建.md)
- [哨兵实施与验收](docs/robots/sentinel/实施与验收.md) / [现有能力与待确认 Checklist](docs/robots/sentinel/现有能力与待确认Checklist.md)

## 当前能力与边界

已提供通用核心组件 (设备与轴、控制、输入、姿态估计、运动学、会话与授权、监控注册表)、主机装配示例及 S1–S4 非运动板级观察固件, **不是完整整车控制固件**, 也尚未开始哨兵 APP 应用层.

| 路径 | 内容 |
|---|---|
| `include/ec/`, `src/` | 时间、通道、授权门、PID/速度控制链/位置串级、轴门面、运动学、姿态估计、温度控制、监控注册表、传输接口及 DJI/DM 协议编解码 |
| `include/ec/platform/host.hpp` | 主机锁策略 |
| `platform/stm32f407/` | HAL/FreeRTOS 平台适配与 DWT 短窗口计时 |
| `boards/sentinel_f407/` | CubeMX 工程、观察固件、静态装配入口、RAM 诊断及离线产物检查 |
| `examples/host_single_axis.cpp` | 最小单轴入门示例 |
| `examples/host_app_example.cpp` | 合成多设备装配示例 (四轮 + DM 关节 + 遥控 + IMU + 控制权 + 可选监控) |
| `examples/host_assembly.cpp`、`examples/host_link_example.cpp` | 单轴链与示例协议链路 |
| `tests/` | 核心契约测试与虚拟电调、UART/IMU 回放等 SIL 场景 |

板级观察固件包含启动/链接、RTOS 周期诊断、USART1 回显、USART3 DR16、CAN 接收/DJI 解码和 BMI088 采样, 以及静态装配入口 (默认安装表为空, 两个 APP 钩子为弱符号空实现) 与只读监控注册表. 默认镜像不启动 USB/PWM、不发送电机控制帧, IMU 加热保持关闭; `check_firmware.py` 会核对这一点. 尚未实现新上位机线协议、DM 参数查询、整车任务与机构安全动作配置; 双 yaw 稳向反馈仍有待确认事项.

主机测试和交叉编译不能代替实机验收. 硬件到位后从 S1 开始按依赖测试; 烧录、动力测试和发弹均须单独明确授权.

## 仓库与参考资料

`references/motors/` 的厂商手册与 `references/motors/sources.json`、以及 `docs/boards/sentinel_f407/references/` 下的官方原件均为**本地资料, 不入库**. `references/Sentinel`、`hardware`、`official` 和接口资料可能是指向外部目录的本地链接. 克隆不会自动取得这些资料或旧工程完整快照; 文档只保留文件名、版本与 SHA256 记录.

主机测试不依赖旧工程. 板级固件使用显式配置的外部 STM32CubeF4 SDK, 见环境部署与板级构建说明. 文档维护约定见 [文档规范](docs/文档规范.md).
