#pragma once
// 哨兵正式 APP 的应用对象.
// O0: 持有配置与校验结论, 能形成"明确说明为什么不可用"的状态快照.
// O1: 接入来源/权限仲裁 (首启许可、人工整车停止、互斥维护)、整车原子目标快照、
//     参考/配置代次作废、A09 故障影响集合与程序化恢复的判定, 输出每域执行结论.
// 本对象仍不持有设备、不发任何 CAN 帧; 设备接收与执行落实在 O2/O6 接入.
#include "sentinel_authority.hpp"
#include "sentinel_config.hpp"
#include "sentinel_faults.hpp"
#include "sentinel_types.hpp"
#include "ec/motion/omni_kinematics.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ec::sentinel::app {
// 启动会话身份代次. 0 被核心仲裁用作“不提供身份”, 因此启动代次从 1 开始.
// 业务协议的第一个会话也使用这个代次: 首次握手是“建立身份”而不是“替换身份”,
// 后者会让核心仲裁把新会话的第一个请求边沿吞掉 (表现为一次多余的 not_permitted).
inline constexpr std::uint32_t boot_session_epoch = 1;

// 参考/配置代次. 任一变化都作废已有目标: 参考重置不是真实转动, 配置改动也不是新意图.
struct Epochs {
    std::uint32_t config{};
    std::uint32_t reference{};
};

// 单域反馈输入: 与状态快照中的反馈结构一致, 避免两套表示.
struct DomainFeedback {
    Sample<float> position_rad{};
    Sample<float> velocity_rad_s{};
    Sample<float> current_a{};
};

// 应用级统计: 只统计应用自己观察到的事件, 不冒充设备或总线质量.
struct AppStats {
    std::uint32_t cycles{};
    std::uint32_t link_losses{};
    std::uint32_t source_changes{};
};

struct AppInputs {
    LinkInputs link{};
    Epochs epochs{};
    FaultInputs faults{};
    // 统一反馈抽象: 惯性快照与各位置域当前关节角由执行层/感知层每周期提供,
    // 应用层不自行读设备, 也不在这里伪造缺失反馈.
    InertialFeedback inertial{};
    PlatformFeedback main_yaw{};
    AuxState aux{};
    // 每域反馈由执行层/感知层每周期提供: 只报告真实样本及其采样时刻,
    // 缺失反馈保持 unknown, 不用零值或收包时刻顶替.
    std::array<DomainFeedback, domain_count> feedback{};
};

// 单域设定值. present 语义由 TargetSnapshot 的 optional 表达; mode 不得是 none.
// age_ms 是该意图在发送侧的源年龄 (0 = 刚刚产生): 接收端据此设定期限, 既避免整包刷新
// 把旧域一起续命, 也避免发送侧积压很久的意图被当成新意图. 派生目标 (底盘 twist → 轮速、
// 位置内环) 继承同一域的期限.
struct DomainSetpoint {
    control::ControlMode mode{control::ControlMode::none};
    float value{};
    std::uint16_t age_ms{};
    YawFrame frame{YawFrame::joint}; // 只有 main 可选 local_inertial/odom, 其它域必须为 joint
};

// 一份整车原子目标快照: 未置有效位的域表示"撤销旧目标", 与"有效零值目标"不同.
// 底盘作为整车 twist 下发 (SI: m/s, rad/s), 由应用解算为四轮轴侧角速度并同比例缩放;
// 若同时给出单个底盘轮位姿则视为意图冲突, 整包拒绝.
struct TargetSnapshot {
    std::optional<motion::PlanarTwist> chassis{};
    std::uint16_t chassis_age_ms{}; // 与各域同样的源年龄语义
    std::array<std::optional<DomainSetpoint>, domain_count> domains{};
};

enum class AcceptResult : std::uint8_t {
    accepted = 0,
    no_authority,
    manual_stop,
    not_configured,
    invalid_target,
    limit_violation,
    degraded, // 主 yaw 惯性降级保持期间不接受新的惯性方向目标
};

class SentinelApp {
public:
    // 构造时按用途校验配置; 校验失败不抛异常, 但 startable() 为假.
    SentinelApp(const SentinelConfig& config, ConfigAudience audience) noexcept;
    SentinelApp(const SentinelApp&) = delete;
    SentinelApp& operator=(const SentinelApp&) = delete;

    [[nodiscard]] const SentinelConfig& config() const noexcept { return config_; }
    [[nodiscard]] const ConfigReport& report() const noexcept { return report_; }
    // 参考类配置的原位生效入口 (只由参数事务调用): 关节零位/机械范围改变后同一数值目标
    // 指向不同物理角, 因此必须同步作废受影响域的目标并上推参考代次.
    // 这不是第二份配置来源: 事务只从这一个入口写入, 读回也从这里读.
    [[nodiscard]] bool set_joint_geometry(Domain domain, const JointGeometryConfig& joint) noexcept;
    [[nodiscard]] const JointGeometryConfig& joint_geometry(Domain domain) const noexcept {
        return config_.joints[joint_index(domain)];
    }
    // 目标时限是整车参考类配置: 修改后旧目标的到期时刻不再有意义, 因此一并作废.
    [[nodiscard]] bool set_target_ttl(Lifetime ttl) noexcept;
    [[nodiscard]] Lifetime target_ttl() const noexcept { return config_.timing.target_ttl; }
    // 该配置是否允许按构造时的用途启动. 部署用途下合成夹具/未知来源配置为假.
    [[nodiscard]] bool startable() const noexcept { return report_.startable; }
    [[nodiscard]] ConfigKind kind() const noexcept { return config_.kind; }
    [[nodiscard]] std::uint32_t config_epoch() const noexcept { return config_epoch_; }
    [[nodiscard]] std::uint32_t reference_epoch() const noexcept { return reference_epoch_; }
    // 参考语义变化 (对齐/参考类参数生效) 时上推, 用于作废旧惯性方向目标.
    // 参考语义改变 (对齐/重建参考): 代次上推并作废已有目标 —— 同一数值目标在新参考下
    // 指向不同的物理角, 旧目标不得继续执行. 这是应用自己的参考事件, 与调用方上报的
    // inputs.epochs 变化互为补充, 两者都会作废旧目标. 降级保持一并清除: 保持角属于旧参考.
    void bump_reference_epoch() noexcept {
        ++reference_epoch_;
        clear_targets();
    }
    // 单域参考语义改变 (关节零位/机械范围/对齐): 只作废该域的目标与降级保持, 并上推该域
    // 参考代次与全局代次, 不顺手把其它域 (尤其底盘) 一起停掉.
    void invalidate_domain_reference(Domain domain) noexcept;
    // 惯性坐标重建不改变机械维护目标的含义.
    void invalidate_inertial_reference() noexcept;
    // 本地目标装配: 失去方向测量时把仍有效的 G07 原意图显式放入整包, 不续期.
    // 外部业务快照仍严格使用 accept_targets 的缺域撤销语义.
    AcceptResult accept_remote_targets(const TargetSnapshot& snapshot, Tick now,
        bool inertial_available) noexcept;
    [[nodiscard]] std::uint32_t domain_reference_epoch(Domain domain) const noexcept {
        return domain_reference_epoch_[domain_index(domain)];
    }
    [[nodiscard]] const char* build_identity() const noexcept;
    [[nodiscard]] AppStats stats() const noexcept;

    [[nodiscard]] std::optional<std::size_t> dji_slot_for(Domain domain) const noexcept {
        return dji_index_[domain_index(domain)];
    }
    [[nodiscard]] std::optional<std::size_t> dm_slot_for(Domain domain) const noexcept {
        return dm_index_[domain_index(domain)];
    }

    // 观测视图: 未配置/配置缺项/尚未收到反馈或建立参考都给出明确原因;
    // 不伪造反馈值、参考有效性或授权, 也不做来源仲裁.
    [[nodiscard]] StatusSnapshot status(Tick now) const noexcept;

    // 一个控制周期: 来源仲裁 → 代次作废 → 故障影响 → 每域执行结论.
    // 只更新本对象状态与快照, 不产生任何执行器动作.
    void update(Tick now, const AppInputs& inputs) noexcept;

    // 接受一份整车原子目标: 先整体校验 (未配置/非有限/模式—域组合/关节越限) 再整体提交,
    // 任一域不合法则整包拒绝且不改变任何已有目标 (包括底盘 twist 与缩放系数). 拒绝不续期.
    // requester 声明请求方来源: 与当前持权来源不符时按 no_authority 拒绝;
    // Source::none 表示进程内调用方不声明来源 (仍要求当前有人持权).
    [[nodiscard]] AcceptResult accept_targets(const TargetSnapshot& snapshot, Tick now,
        Source requester = Source::none) noexcept;

    // 最近一次 update 形成的一致快照.
    [[nodiscard]] const StatusSnapshot& snapshot() const noexcept { return snapshot_; }
    [[nodiscard]] const SentinelAuthority& authority() const noexcept { return authority_; }
    [[nodiscard]] std::optional<DomainTarget> target(Domain domain) const noexcept {
        return targets_[domain_index(domain)];
    }
    [[nodiscard]] std::uint32_t accepted_packets() const noexcept { return accepted_packets_; }
    [[nodiscard]] std::uint32_t rejected_packets() const noexcept { return rejected_packets_; }
    [[nodiscard]] std::uint32_t target_invalidations() const noexcept {
        return target_invalidations_;
    }
    // 底盘 twist: 请求值、采用值与同比例缩放系数. 缩放 <1 表示已按四轮最小轴侧上限缩放.
    [[nodiscard]] std::optional<motion::PlanarTwist> requested_twist() const noexcept {
        return requested_twist_;
    }
    [[nodiscard]] std::optional<motion::PlanarTwist> adopted_twist() const noexcept {
        return adopted_twist_;
    }
    [[nodiscard]] float twist_scale() const noexcept { return twist_scale_; }
    // 主 yaw 是否处于惯性降级保持 (G07); 降级期间不接受新的惯性方向目标.
    [[nodiscard]] bool main_yaw_degraded() const noexcept { return main_yaw_degraded_; }    // 上位机/维护会话状态由业务协议显式设置: 目标到达本身不会改变它, 因此
    // "显式禁止的权限不因普通目标到达而开放".
    void set_host_session(bool alive, bool request, std::uint32_t epoch) noexcept;
    // 维护会话同样登记身份代次: 会话被替换时旧维护授权必须失效, 不能只靠 bool 电平.
    void set_maintenance_session(bool alive, bool request, std::uint32_t epoch = 0) noexcept;
    void revoke_sessions() noexcept;
    [[nodiscard]] std::uint32_t maintenance_session_epoch() const noexcept {
        return maintenance_session_epoch_;
    }
    // 会话失效/替换: 按该会话当前授予的来源撤销其会话状态, 不触碰其它来源.
    // 业务协议在超时或重新握手时用它清掉对应内部状态, 避免协议层已判定失效而
    // 内部 bool 仍与调用方输入 OR 合并后保活.
    void clear_role_session(Source role) noexcept {
        if (role == Source::host) { set_host_session(false, false, host_session_epoch_); }
        else if (role == Source::maintenance) { set_maintenance_session(false, false); }
    }
    // 发射许可由明确来源显式设置 (协议 permit 消息或本地来源); 到期/缺失一律不当作许可.
    // 未设置时保持“不可用”, 不用常量 true 冒充许可.
    void set_launch_permit(bool permit, Tick sampled_at, Lifetime ttl) noexcept {
        launch_permit_valid_ = true;
        launch_permit_ = permit;
        launch_sampled_at_ = sampled_at;
        launch_ttl_ = ttl;
    }
    void clear_launch_permit() noexcept { launch_permit_valid_ = false; launch_permit_ = false; }

    // 供上层构造 ExecutionContext/轴时使用: 维护持有期间是维护门, 其余是正常来源门.
    [[nodiscard]] const ExecutionGate& active_gate() const noexcept {
        return authority_.active_gate();
    }

private:
    // 撤输出动作是机构决策, 由应用显式选择: DJI 域零电流, pitch 请求 disable.
    [[nodiscard]] static control::StopAction stop_action_for(Domain domain) noexcept;
    void clear_targets() noexcept;
    // 单域作废: 目标与降级保持属于同一域的意图, 必须一起失效 (不能只 reset 普通目标).
    void invalidate_domain(std::size_t i) noexcept;

    SentinelConfig config_{};
    ConfigReport report_{};
    std::array<std::optional<std::size_t>, domain_count> dji_index_{};
    std::array<std::optional<std::size_t>, domain_count> dm_index_{};
    SentinelAuthority authority_{};
    std::array<std::optional<DomainTarget>, domain_count> targets_{};
    StatusSnapshot snapshot_{};
    std::optional<Grant> last_token_{};
    std::optional<motion::PlanarTwist> requested_twist_{};
    std::optional<motion::PlanarTwist> adopted_twist_{};
    float twist_scale_{1.0f};
    Source last_source_{Source::none};
    std::uint32_t config_epoch_{};
    std::uint32_t reference_epoch_{};
    std::array<std::uint32_t, domain_count> domain_reference_epoch_{};
    std::uint32_t accepted_packets_{};
    std::uint32_t rejected_packets_{};
    std::uint32_t target_invalidations_{};
    bool main_yaw_degraded_{};
    // 降级保持: 进入降级时捕获一次的原意图 (机械保持角 + 原到期时刻). 保持不是新的执行
    // 资格, 也不随时间刷新: 原意图不存在/已到期时降级不产生任何输出. 用 DomainTarget
    // 表达使到期判定走同一套回绕安全比较, 不用绝对截止毫秒值.
    std::optional<DomainTarget> degraded_hold_{};
    bool host_session_alive_{};
    bool host_session_request_{};
    std::uint32_t host_session_epoch_{boot_session_epoch};
    bool maintenance_session_alive_{};
    bool maintenance_session_request_{};
    std::uint32_t maintenance_session_epoch_{};
    bool launch_permit_valid_{};
    bool launch_permit_{};
    Tick launch_sampled_at_{};
    Lifetime launch_ttl_{};
    AppStats stats_{};
    bool remote_was_online_{};
};
} // 命名空间 ec::sentinel::app
