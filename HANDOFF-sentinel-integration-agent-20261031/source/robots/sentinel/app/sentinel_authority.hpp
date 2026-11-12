#pragma once
// 应用级控制来源仲裁 (O1). 在核心 ControlAuthority (遥控 vs 上位机) 之上补齐三件事:
//  1. 首启许可: 每次启动必须先见到有效遥控运行状态, Jetson/维护会话不能独立启动驱动;
//  2. 人工整车停止: 右拨杆下位锁存, 只由新鲜遥控帧回到运行挡才解除, 失联不解除;
//  3. 互斥维护来源: 仅在无正常来源且左拨杆待机位可用, 人工停止态可建会话但不得驱动.
// 只决定"谁可以下发目标", 不解析协议、不执行机构动作、不替代轴层授权门.
#include "ec/control/authority.hpp"
#include "ec/drivers/dr16.hpp"
#include "sentinel_types.hpp"
#include <cstdint>
#include <optional>

namespace ec::sentinel::app {
// 只在专用底盘调试镜像启用新组合; 未支持的组合待机, 不隐式启用Jetson或其它机构.
enum class RemoteLayout : std::uint8_t { legacy_ch4 = 0, chassis_commissioning, mechanism_commissioning, chassis_gimbal_integration };
[[nodiscard]] constexpr drivers::Dr16Switch remote_source_selection(RemoteLayout layout,
    drivers::Dr16Switch left, drivers::Dr16Switch right) noexcept {
    if (layout == RemoteLayout::legacy_ch4) { return left; }
    if (left == drivers::Dr16Switch::unknown || right == drivers::Dr16Switch::unknown) {
        return drivers::Dr16Switch::unknown;
    }
    if (layout == RemoteLayout::chassis_gimbal_integration) {
        const bool local = (right == drivers::Dr16Switch::mid &&
            (left == drivers::Dr16Switch::up || left == drivers::Dr16Switch::mid)) ||
            (right == drivers::Dr16Switch::up && left == drivers::Dr16Switch::up);
        return local ? drivers::Dr16Switch::up : drivers::Dr16Switch::mid;
    }
    return layout == RemoteLayout::chassis_commissioning &&
        left == drivers::Dr16Switch::up && right == drivers::Dr16Switch::mid
        ? drivers::Dr16Switch::up : drivers::Dr16Switch::mid;
}
// 输入链路与来源选择的原始观察. 拨杆已由解码层保证是 up/mid/down; 会话/请求电平由上层填入.
// 注意: 拨杆位置只在 remote_online (新鲜有效遥控帧) 时才有意义; 调用方在掉线时可以填 unknown,
// 本组件保留最后一次有效选源 —— 遥控目标数值/新鲜度与来源选择是两件事.
struct LinkInputs {
    bool remote_online{};
    drivers::Dr16Switch switch_left{drivers::Dr16Switch::unknown};
    drivers::Dr16Switch switch_right{drivers::Dr16Switch::unknown};
    Tick remote_sampled_at{};
    bool host_link_alive{};
    bool host_request{};
    Tick host_sampled_at{};
    std::uint32_t host_session_epoch{};
    bool maintenance_link_alive{};
    bool maintenance_request{};
    // 维护会话身份代次: 与获得授权时不同即视为会话被替换, 旧维护授权立即失效.
    // 同一个控制批内 hello→authorize 不能靠"先清 bool 再置位"保住旧 token.
    std::uint32_t maintenance_session_epoch{};
    bool locally_ready{};
    RemoteLayout remote_layout{RemoteLayout::legacy_ch4};
};

struct AuthorityDecision {
    Source source{Source::none};
    std::optional<Grant> token{};
    bool manual_stop{};
    Reason reason{Reason::none};
};

// 台架仅由显式维护申请启动; 整车默认仍要求真实遥控首启.
enum class StartupPolicy : std::uint8_t { remote_required, maintenance_only };

class SentinelAuthority {
public:
    explicit SentinelAuthority(StartupPolicy policy = StartupPolicy::remote_required) noexcept
        : startup_policy_(policy), running_state_reached_(policy == StartupPolicy::maintenance_only) {}
    SentinelAuthority(const SentinelAuthority&) = delete;
    SentinelAuthority& operator=(const SentinelAuthority&) = delete;

    // 每周期调用一次; 只读输入, 不产生任何设备动作.
    [[nodiscard]] AuthorityDecision update(const LinkInputs& inputs) noexcept;
    // 整车级撤销 (人工停止、参考/配置代次变化、会话失效等): 立即失效两类授权.
    void revoke() noexcept;

    [[nodiscard]] Source source() const noexcept { return source_; }
    [[nodiscard]] bool holds() const noexcept { return holds_; }
    [[nodiscard]] std::optional<Grant> token() const noexcept { return token_; }    [[nodiscard]] bool manual_stop() const noexcept { return manual_stop_; }
    [[nodiscard]] bool running_state_reached() const noexcept { return running_state_reached_; }
    [[nodiscard]] Reason reason() const noexcept { return reason_; }
    // 当前有效的执行门: 维护来源持有期间是维护门, 其余情况是正常来源门.
    [[nodiscard]] const ExecutionGate& active_gate() const noexcept {
        return maintenance_holds_ ? maintenance_gate_ : normal_.gate();
    }
    [[nodiscard]] std::uint32_t switches() const noexcept {
        return normal_.switches() + maintenance_switches_;
    }
    [[nodiscard]] std::uint32_t revocations() const noexcept {
        return normal_.revocations() + maintenance_revocations_;
    }
    // 人工停止锁存/解除的边沿计数, 供观测与测试核对"只由人工触发".
    [[nodiscard]] std::uint32_t stop_latches() const noexcept { return stop_latches_; }
    [[nodiscard]] std::uint32_t stop_clears() const noexcept { return stop_clears_; }
    // 最后一次有效选源 (左拨杆) 与显式选源动作计数: 遥控掉线不改来源选择, 只停目标值.
    [[nodiscard]] drivers::Dr16Switch selection() const noexcept { return selection_; }
    [[nodiscard]] std::uint32_t selection_edges() const noexcept { return selection_edges_; }
    [[nodiscard]] bool maintenance_holds() const noexcept { return maintenance_holds_; }

private:
    StartupPolicy startup_policy_{StartupPolicy::remote_required};
    control::ControlAuthority normal_{};
    ExecutionGate maintenance_gate_{};
    std::optional<Grant> token_{};
    std::optional<Grant> maintenance_token_{};
    // 最后一次有效选源: 只在新鲜遥控帧上更新, 与"遥控目标数值/新鲜度"分离.
    drivers::Dr16Switch selection_{drivers::Dr16Switch::unknown};
    std::uint32_t selection_edges_{};
    bool maintenance_holds_{};
    bool commissioning_maintenance_selected_{};
    bool manual_stop_{};
    bool running_state_reached_{};
    bool holds_{};
    Source source_{Source::none};
    Reason reason_{Reason::awaiting_remote};
    std::uint32_t maintenance_switches_{};
    std::uint32_t maintenance_revocations_{};
    std::uint32_t maintenance_session_epoch_{};
    // 明确人工选源 (本地/Jetson) 后, 当前维护请求的资格被撤销, 直到出现新的显式维护申请
    // (请求/链路先退出再提出). 没有它, 旧请求电平会在同一周期把维护重新签回来.
    bool maintenance_barred_{};
    std::uint32_t stop_latches_{};
    std::uint32_t stop_clears_{};
};
} // 命名空间 ec::sentinel::app
