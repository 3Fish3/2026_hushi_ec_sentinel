#pragma once
#include "ec/execution_gate.hpp"
#include <cstdint>
#include <optional>

namespace ec::control {
// 停止动作必须由上层为每个机构显式选择. 本层不提供默认动作, 也不自行把某个机构归零:
// 例如"pitch 停止就把目标写 0"是机构决策, 不属于通用框架.
enum class StopAction : std::uint8_t { unspecified = 0, hold_position, zero_output, disable_output };

struct ExecutionPolicy {
    StopAction on_target_stale{StopAction::unspecified};
    StopAction on_authority_lost{StopAction::unspecified};
};

enum class DenyReason : std::uint8_t {
    none, not_ready, no_grant, target_stale, link_stale, policy_missing,
};

// 执行上下文: 每个控制周期唯一一处做许可与时效检查, 给出明确结论.
// 协议层只报告输入, 轴/环只执行结论, 发送层不做许可判断; 三处不再各写一套故障状态机.
struct ExecutionRequest {
    Grant token{};          // 由组级控制权组件产生
    bool locally_ready{};   // 本地自检/就绪
    bool target_fresh{};    // 目标在有效期内
    bool link_alive{};      // 会话/遥控链路存活
};

struct ExecutionDecision {
    bool execute{};
    DenyReason reason{DenyReason::none};
    // 不执行时上层应执行的动作; unspecified 表示上层还没配置策略, 必须报告而不是静默归零.
    StopAction stop{StopAction::unspecified};
    [[nodiscard]] bool actionable() const noexcept {
        return execute || stop != StopAction::unspecified;
    }
};

class ExecutionContext {
public:
    ExecutionContext(const ExecutionGate& gate, ExecutionPolicy policy) noexcept
        : gate_(gate), policy_(policy) {}
    [[nodiscard]] ExecutionDecision evaluate(const ExecutionRequest& request) const noexcept {
        if (!request.locally_ready) {
            return {false, DenyReason::not_ready, policy_.on_authority_lost};
        }
        if (!gate_.accepts(request.token)) {
            return {false, DenyReason::no_grant, policy_.on_authority_lost};
        }
        if (!request.link_alive) {
            return {false, DenyReason::link_stale, policy_.on_authority_lost};
        }
        if (!request.target_fresh) {
            return {false, DenyReason::target_stale, policy_.on_target_stale};
        }
        return {true, DenyReason::none, StopAction::unspecified};
    }
    [[nodiscard]] const ExecutionPolicy& policy() const noexcept { return policy_; }
private:
    const ExecutionGate& gate_;
    ExecutionPolicy policy_{};
};
} // 命名空间 ec::control
