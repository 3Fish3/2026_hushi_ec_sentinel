# 三轴方波系统辨识扩展（fire_yaw / pitch / main_yaw）

- 完成时间：2026-10-29 21:00（本机本地时区，America/New_York）。
- 仓库路径：`<HOME>/.codex/worktrees/b824/sentinel`。
- Git：detached HEAD，上一提交 `ec20127 feat(gimbal): pitch square-wave identification, assisted tuning and yaw retune`；本阶段在其上继续，未暂存/未提交/未改分支。
- 活动源码：`HANDOFF-sentinel-gimbal-app-20260926/source/`。
- 本阶段只做离线实现与主机/ARM 构建验证。**未连接实车、未烧录、未激励真实电机、未改写任何部署默认增益。**

## 一、目标与设计

用户要求：提交已有改动后，把"方波系统辨识"方法扩展到 fire_yaw、pitch、main_yaw 三轴。

- pitch 原有辨识不变：DM 电机 MIT 力矩叠加，`PitchIdentification`（N·m 域）。
- fire_yaw / main_yaw 是 DJI（GM6020 电流模式），没有力矩叠加通路，需要**电流域**激励：在位置/速度环输出之上叠加一个已知方波电流分量，再按轴 `current_limit` 硬限幅。
- 采集连续流原本硬编码只支持 control 域 6（pitch）；扩展为 4/5/6（main_yaw/fire_yaw/pitch），底层 `mapping` 本就通用。

## 二、改动清单

固件（`robots/sentinel/app`）：
- 新增 `sentinel_dji_identification.hpp`：`DjiIdentification` 类，结构与 `PitchIdentification` 对齐（policy/recipe/evidence/state/fault），作用在电流域。安全语义：
  - 仅维护授权 + 目标新鲜 + 执行中 + 隔离（其它轴无 execute）+ 反馈新鲜才可推进；
  - 幅度上限、位置/速度/跟随误差 envelope、step 时间间隔、run 单调；
  - 叠加电流期望上限为 `max_expected_current_a`（部署取轴 current_limit），超限锁存 `current` 故障；
  - 任一不满足即锁存 fault 并撤输出，需 `reset(no_target, disabled_confirmed)` 才能复位；run 号不回退。
- `sentinel_config.hpp`：`SentinelConfig` 增 `std::array<std::optional<DjiIdentificationPolicy>, dji_slot_count> dji_identification{}`（默认全空）。
- `sentinel_execution.hpp/.cpp`：
  - 构造期按槽计算可用性（离线夹具或显式 deployment opt-in）并 emplace 激励对象；
  - `dji_slot_for(Domain)`、`dji_identification_evidence(...)`、`dji_capture_healthy(...)`、`start/cancel/reset_dji_identification(...)`、`dji_identification(slot)` 访问器；
  - `step()` 中先采集门槛后推进每槽波形，锁存即撤该轴输出；
  - 轴输出处把归一化输出换算成安培 `loop_a = output*full`，叠加 `requested_a()`，在 `±current_limit` 限幅后回写归一化；记录 `dji_last_excitation_a_`。
- `raw_record.cpp/.hpp`：连续采集健康检查从固定域 6 放宽到域 4/5/6；新增 `stream_domain()` 只读访问器。

协议（`sentinel_protocol.cpp`）：
- `handle_pitch_identification`：在既有 19（非 start）/43（start）字节后**可选 1 字节 domain**（缺省 6=pitch，历史报文长度仍合法）。按 domain 分派到 pitch（DM）或 DJI 槽（main_yaw=4/fire_yaw=5）；未知域 query/cancel/start 分别返回 invalid_value/rejected；响应沿用 27 字节状态（DJI 用 requested 字段表达安培）。

上位机（`robots/sentinel/host`）：
- `sentinel_client.py`：`encode_pitch_identification(..., domain=None)` 与 `pitch_identification(..., domain=None)` 追加可选域字节；缺省保持历史长度。
- `sentinel_async.py`：`pitch_identification(..., domain=None)` 透传。

测试：
- 新增 `tests/dji_identification_tests.cpp`（波形、幅度/电流 envelope、field 故障锁存、step 时序、reset 契约）。
- `tests/pitch_identification_protocol_tests.cpp`：新增 `test_dji_domain_dispatch`（main_yaw/fire_yaw query、未知域拒绝、无 policy start 拒绝、非法长度）；旧"20/44 字节非法"断言改为 21/45（20/44 现在是合法域报文）。
- `CMakeLists.txt`：注册 `sentinel_dji_excitation_contracts`；`sentinel_pitch_identification_contracts` 超时 60→180（并行构建时该 Python 测试曾因调度超时误判，单独运行稳定 <11s）。

## 三、验证

- 主机 `ctest`：**55/55 通过**（排除历史 `doc_link_contracts`，其 243 条本地链接中 225 条断链，属既有问题，与本阶段无关；断链清单不含本次新增文件）。
- Python `unittest`（pitch_identification）：**106/106 通过**。
- ARM 交叉构建 `build/app-gimbal-assisted-arm-Release`：`sentinel_app` **构建成功**（EC_APP_GIMBAL_COMMISSIONING / REMOTE_VERIFY / PITCH_IDENTIFICATION=ON）。
- 新增 `ec_dji_identification_tests`、`sentinel_dji_excitation_contracts` 均通过。

## 四、未验证 / 待办

- **未上实车**：DJI 电流激励的真实响应、限幅行为、锁存边界只在主机契约测试与仿真层验证。
- DJI 轴激励尚未接入上位机辅助调参界面（目前通过协议 domain 字节 + client 直接调用可用）；界面按轴选择与三轴配方、真实数据二阶拟合、自动 kp/kd 建议仍未实现（与 pitch 相同的边界）。
- 采集连续流的域 4/5 流映射已在主机契约层通过，但**实车 CAN 流编号与 200 Hz 带宽**需现场核对。
- 部署默认仍为无激励（各轴 `dji_identification` 为空、pitch policy 仅实验注入），`main_yaw` 无持久化改动。
- 建议下一步：在实验车上先对 fire_yaw（有机械限位、负载小）做一次低幅度电流方波，核对采集流与限幅，再仿照 pitch 的带宽/阶跃方法选增益。

## 五、相关文件

- 新类：`robots/sentinel/app/sentinel_dji_identification.hpp`
- 执行器：`robots/sentinel/app/sentinel_execution.{hpp,cpp}`
- 配置：`robots/sentinel/app/sentinel_config.hpp`
- 协议：`robots/sentinel/app/sentinel_protocol.cpp`
- 采集：`robots/sentinel/recording/raw_record.{hpp,cpp}`
- 上位机：`robots/sentinel/host/sentinel_client.py`、`sentinel_async.py`
- 测试：`tests/dji_identification_tests.cpp`、`tests/pitch_identification_protocol_tests.cpp`
