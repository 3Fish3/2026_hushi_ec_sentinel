#pragma once
#include "ec/execution_gate.hpp"
#include <cstdint>
#include <optional>

namespace ec::control {
// 组级控制来源仲裁: 只决定"谁可以下发目标", 不解析协议、不替代安全状态机.
// 规则来自整车控制权契约:
//  - 遥控在线本身不等于接管, 必须有明确的接管动作;
//  - 遥控接管请求优先于上位机, 即使上位机同时请求;
//  - 遥控掉线只撤销授权, 绝不自动把控制权交给上位机;
//  - 撤销后上位机必须重新提出请求(请求边沿), 持续保持的旧请求不能重新获得授权;
//  - 上位机还需要有效会话与本地显式授权, 链路存活本身不等于授权.
// 层次说明: 本组件的 token 是"来源仲裁"结果; 轴/环内部的 ExecutionGate 仍是执行路径上的
// 可撤销 token. 两者都建立在同一个 ExecutionGate 之上, 不引入第二套故障状态机.
// 单任务拥有, 每周期调用一次 update.
class ControlAuthority {
public:
    struct Inputs {
        bool remote_online{};
        bool remote_takeover{};    // 操作者的接管动作(拨杆/按键到位)
        bool host_session_alive{}; // 上位机会话链路存活, 与目标有效期无关
        bool host_requests{};      // 上位机请求控制权(电平)
        bool host_authorized{};    // 本地对上位机的显式授权
        bool local_ready{};        // 本地自检与执行上下文允许动作
        // 上位机会话身份代次 (SessionTracker::epoch): 与获得授权时不同即视为会话被替换,
        // 已持有的 host 授权立即失效, 必须重新请求. 默认 0 表示“不提供身份”, 此时不额外失效.
        std::uint32_t host_session_epoch{};
    };
    struct Decision {
        CommandSource source{CommandSource::none};
        Grant token{}; // 只有 source != none 时有效
    };

    ControlAuthority() = default;
    ControlAuthority(const ControlAuthority&) = delete;
    ControlAuthority& operator=(const ControlAuthority&) = delete;

    // 返回本周期允许的来源; 返回空表示调用方必须撤销所有轴.
    [[nodiscard]] std::optional<Decision> update(const Inputs& inputs) noexcept;
    // 强制撤销(故障动作、模式切换、人工停止).
    void revoke() noexcept;
    [[nodiscard]] bool holds() const noexcept { return held_; }
    [[nodiscard]] CommandSource source() const noexcept { return source_; }
    // 当前持有的 token; 未持有时为空. 执行上下文与通道都用它核对授权.
    [[nodiscard]] std::optional<Grant> token() const noexcept {
        if (!held_) { return std::nullopt; }
        return token_;
    }
    [[nodiscard]] const ExecutionGate& gate() const noexcept { return gate_; }
    [[nodiscard]] bool accepts(const Grant& token) const noexcept { return gate_.accepts(token); }
    [[nodiscard]] std::uint32_t switches() const noexcept { return switches_; }
    [[nodiscard]] std::uint32_t revocations() const noexcept { return revocations_; }
    // 上位机当前是否持续保持控制权请求(仅诊断用; 请求边沿只在本周期有效).
    [[nodiscard]] bool host_request_held() const noexcept { return previous_host_request_; }

private:
    [[nodiscard]] bool drop() noexcept;

    ExecutionGate gate_{};
    Grant token_{};
    CommandSource source_{CommandSource::none};
    bool held_{};
    bool previous_host_request_{};
    std::uint32_t host_session_epoch_{};
    bool host_session_epoch_known_{};
    std::uint32_t switches_{};
    std::uint32_t revocations_{};
};
} // 命名空间 ec::control
