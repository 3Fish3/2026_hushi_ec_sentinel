#pragma once
// 薄周期编排 (APP06): 主机后端与 F407 正式 APP 共用**同一段**周期逻辑:
//   原始完整样本 → 输入视图 → 来源选择 → 本地/host 目标接受 → 感知 → 执行 → 观测.
// 只把 HAL/总线换成调用方提供的输入与已注入的发送器; 本模块不做设备协议解析,
// 也不新增第二套权限/故障状态机 (仍走 SentinelApp/SentinelExecutor/SentinelPerception).
//
// 分两次调用是为了匹配板级 assembly_step 的 control/commit 顺序:
//   control(): 遥控样本 → 映射 → 感知 → 输入视图 → 应用仲裁 → 目标接受
//   commit() : 执行器提交 (真实 CAN 组帧) → 命令预测积分
// 主机测试按同一顺序调用这两个函数, 因此"板级接入"与"主机证据"是同一条代码路径.
#include "ec/input/dr16_input.hpp"
#include "ec/drivers/bmi088_device.hpp"
#include "sentinel_app.hpp"
#include "sentinel_execution.hpp"
#include "sentinel_perception.hpp"
#include "sentinel_remote.hpp"
#include "sentinel_experiment.hpp"
#include <array>
#include <optional>

namespace ec::sentinel::app {
// 本周期从链路/采集层拿到的输入. 缺省字段表示"本周期没有新鲜样本", 由各自时效判定,
// 不在这里补零或伪造在线.
struct CycleInputs {
    // 遥控: 一份**完整**样本及其到达时刻 (板级在 USART3 分帧后发布, 主机测试直接喂帧).
    // 同一份样本被重复读取时不重复计数 (按到达时刻去重).
    std::optional<drivers::Dr16Sample> remote_sample{};
    Tick remote_received_at{};
    // IMU: 传感器侧量与采样时刻; 摄入时刻由编排填 now (两者必须分离, 见 O3).
    std::optional<std::array<float, 3>> imu_gyro_rad_s{};
    std::optional<std::array<float, 3>> imu_accel_mps2{};
    Tick imu_sampled_at{};
    std::optional<drivers::ImuSamplingTimes> imu_times{};
    // 调用方注入的外部约束 (功率/热量/许可观察); 缺省保持"不可用", 不虚构许可.
    AuxState aux{};
    // 会话/链路状态由业务协议层显式提供: 目标到达本身不改变权限.
    bool host_link_alive{};
    bool host_request{};
    std::uint32_t host_session_epoch{};
    bool maintenance_link_alive{};
    bool maintenance_request{};
    // 维护会话身份代次 (与 host 同样语义): 会话被替换时旧维护授权立即失效.
    std::uint32_t maintenance_session_epoch{};
    bool locally_ready{};
    Tick now{};
    float dt_s{};
    std::optional<MicroTick> now_us{};
};

struct CycleOutcome {
    bool remote_online{};
    bool remote_frame_accepted{}; // 本周期确实收到并接受了一份新样本
    SourceIntent intent{SourceIntent::none};
    bool manual_stop{};
    Ch4Layer layer{Ch4Layer::unknown};
    bool chassis_active{};
    bool friction_on{};
    // 本地目标是否被应用采用 (未采用时原因在 app.snapshot().executions[].reason).
    AcceptResult accept{AcceptResult::no_authority};
    bool accept_attempted{};
    bool imu_submitted{};
};

class SentinelCycle {
public:
    SentinelCycle(SentinelApp& app, SentinelExecutor& executor, SentinelPerception& perception,
        RemoteMappingConfig mapping, Lifetime remote_ttl) noexcept;
    SentinelCycle(const SentinelCycle&) = delete;
    SentinelCycle& operator=(const SentinelCycle&) = delete;

    // 前半周期. 只在单一控制任务上下文调用.
    CycleOutcome control(const CycleInputs& inputs) noexcept;
    // 后半周期: 提交执行并积分命令预测.
    void commit(const CycleInputs& inputs) noexcept;

    // 绑定 MCU 主动激励控制器 (方案B: 默认仿真控制器驱动状态机与记录, 硬件门由 deployment 兜底).
    void bind_experiment(excitation::Controller& controller) noexcept { experiment_ = &controller; }

    [[nodiscard]] const RemoteMapper& mapper() const noexcept { return mapper_; }
    [[nodiscard]] const input::Dr16Input& remote_input() const noexcept { return remote_; }
    [[nodiscard]] const CycleOutcome& outcome() const noexcept { return outcome_; }
    [[nodiscard]] const AppInputs& app_inputs() const noexcept { return inputs_; }

private:
    // 各域反馈只报告真实设备样本及其采样时刻, 缺失保持 unknown, 不用零值顶替.
    void refresh_feedback(Tick now) noexcept;
    // 遥控样本按到达时刻去重: 槽位被重复读取时不应重复计数或重复建立基准.
    bool accept_remote(const CycleInputs& inputs) noexcept;
    // 激励实验每周期推进: 生成位置参考→经应用层目标接受→记录; 状态机推进与撤参考.
    void experiment_step(const CycleInputs& inputs) noexcept;

    SentinelApp& app_;
    SentinelExecutor& executor_;
    SentinelPerception& perception_;
    input::Dr16Input remote_;
    RemoteMapper mapper_;
    AppInputs inputs_{};
    CycleOutcome outcome_{};
    Tick last_remote_at_{};
    bool has_remote_{};
    std::uint32_t local_epoch_{};
    // 各关节域的参考代次快照: 参改变后丢弃该域已保持的本地角度目标.
    std::array<std::uint32_t, joint_slot_count> reference_epochs_{};
    excitation::Controller* experiment_{};
};
} // 命名空间 ec::sentinel::app
