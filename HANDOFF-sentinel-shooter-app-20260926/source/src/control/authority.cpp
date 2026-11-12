#include "ec/control/authority.hpp"

namespace ec::control {
bool ControlAuthority::drop() noexcept {
    // 未持有时不做任何事: idle 路径上的重复 drop 不应制造无意义的代次变化.
    // 已持有时必须立即失效(撤销/切换/故障动作都依赖这一点).
    if (!held_) { return false; }
    gate_.invalidate();
    held_ = false;
    source_ = CommandSource::none;
    ++revocations_;
    return false;
}

void ControlAuthority::revoke() noexcept {
    static_cast<void>(drop());
}

std::optional<ControlAuthority::Decision> ControlAuthority::update(const Inputs& inputs) noexcept {
    // 会话身份代次: 与上次不同即视为会话被替换. 本周期上位机按“未请求”处理, 旧授权与旧目标
    // 全部失效; 要重新取得控制权必须重新产生请求边沿(与遥控接管后重新请求的规则一致).
    const bool host_identity_changed = host_session_epoch_known_ &&
        inputs.host_session_epoch != host_session_epoch_;
    host_session_epoch_ = inputs.host_session_epoch;
    host_session_epoch_known_ = true;
    const bool host_requests = inputs.host_requests && !host_identity_changed;

    // 请求边沿只在本周期有效: 如果本周期不是上位机获得授权, 该请求不会被保留.
    // 因此遥控失联后必须重新提出请求, 不能靠保持旧电平自动接权.
    const bool host_request_edge = host_requests && !previous_host_request_;
    // 会话替换后, 旧会话保持的请求电平不算新会话的请求: 下一个周期重新产生边沿.
    previous_host_request_ = host_identity_changed ? false : inputs.host_requests;

    CommandSource wanted = CommandSource::none;
    if (inputs.remote_online && inputs.remote_takeover) {
        wanted = CommandSource::remote; // 接管请求优先, 上位机同周期的请求被丢弃
    } else if (host_request_edge && inputs.host_session_alive && inputs.host_authorized) {
        wanted = CommandSource::host; // 靠边沿取得控制权
    } else if (held_ && source_ == CommandSource::host && host_requests &&
        inputs.host_session_alive && inputs.host_authorized) {
        // 已持有: 请求电平与条件都仍成立时保持, 不刷新 token. 撤销后本分支不生效.
        wanted = CommandSource::host;
    }
    if (wanted == CommandSource::none) {
        static_cast<void>(drop());
        return std::nullopt;
    }
    if (held_ && wanted == source_) {
        // 同一来源: 只在本地条件仍成立时保持, 不刷新 token.
        const bool still_valid = inputs.local_ready &&
            (wanted != CommandSource::host || inputs.host_session_alive);
        if (!still_valid) {
            static_cast<void>(drop());
            return std::nullopt;
        }
        return Decision{source_, token_};
    }
    // 来源切换或首次授权: ExecutionGate::arm 会先撤销旧授权, 因此旧 token 立即失效.
    const auto token = gate_.arm(wanted, true, inputs.local_ready);
    if (!token) {
        static_cast<void>(drop());
        return std::nullopt;
    }
    if (held_) { ++switches_; }
    source_ = wanted;
    token_ = *token;
    held_ = true;
    return Decision{source_, token_};
}
} // 命名空间 ec::control
