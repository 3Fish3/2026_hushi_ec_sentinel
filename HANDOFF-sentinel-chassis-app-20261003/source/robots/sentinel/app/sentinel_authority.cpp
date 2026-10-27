#include "sentinel_authority.hpp"

namespace ec::sentinel::app {
namespace {
[[nodiscard]] Source to_app_source(CommandSource source) noexcept {
    switch (source) {
    case CommandSource::remote: return Source::remote;
    case CommandSource::host: return Source::host;
    case CommandSource::maintenance: return Source::maintenance;
    case CommandSource::none: break;
    }
    return Source::none;
}
} // 内部辅助

AuthorityDecision SentinelAuthority::update(const LinkInputs& inputs) noexcept {
    // 1. 首启许可: 第一份有效遥控样本即视为已见到运行状态 (非法帧/非法拨杆不会走到这里).
    if (inputs.remote_online) { running_state_reached_ = true; }

    // 2. 人工整车停止: 只在新鲜遥控帧上更新; 遥控失联时保持既有锁存, 不自动解除.
    if (inputs.remote_online) {
        const bool stop = inputs.switch_right == drivers::Dr16Switch::down;
        if (stop && !manual_stop_) { ++stop_latches_; }
        if (!stop && manual_stop_) { ++stop_clears_; }
        manual_stop_ = stop;
    }
    const bool stop = manual_stop_;
    // 3. 来源选择 (左拨杆) 是**锁存状态**: 只在新鲜遥控帧上更新. 遥控掉线不改来源选择,
    //    因此 host 持权期间遥控掉线不会把 host 拖下来; 遥控目标数值/新鲜度仍由各自时效判定.
    const auto selected = remote_source_selection(
        inputs.remote_layout, inputs.switch_left, inputs.switch_right);
    const bool fresh_selection = inputs.remote_online && selected != drivers::Dr16Switch::unknown;
    bool selection_edge = false;
    if (fresh_selection && selected != selection_) {
        selection_edge = true;
        ++selection_edges_;
        selection_ = selected;
    }
    if (inputs.remote_online) {
        commissioning_maintenance_selected_ = inputs.switch_left == drivers::Dr16Switch::mid &&
            inputs.switch_right == drivers::Dr16Switch::up;
    }
    const auto left = selection_;
    const bool local_selected = left == drivers::Dr16Switch::up;
    const bool host_selected = left == drivers::Dr16Switch::down;

    // 4. 正常来源 (遥控 vs 上位机): 复用核心仲裁.
    //    维护持有期间不重复抢权: 只有**新鲜明确的选源动作**能抢回控制 (I05). 稳定拨杆位置
    //    与普通 Jetson 请求都不能抢正在执行的维护来源; 维护结束后 Jetson 仍需重新请求.
    //    新鲜显式选到 Jetson 位与新鲜选到本地位同等: 先撤销维护及其目标.
    const bool fresh_host_takeover = host_selected && selection_edge && !stop;
    if (fresh_host_takeover && maintenance_holds_) {
        maintenance_gate_.invalidate();
        maintenance_holds_ = false;
        maintenance_token_.reset();
        ++maintenance_revocations_;
    }
    const bool maintenance_active = maintenance_holds_;
    control::ControlAuthority::Inputs normal{};
    normal.remote_online = inputs.remote_online && !stop;
    normal.remote_takeover = local_selected && !stop && (!maintenance_active || selection_edge);
    normal.host_session_alive = inputs.host_link_alive && !stop;
    normal.host_requests =
        host_selected && inputs.host_link_alive && inputs.host_request && !stop;
    // 维护持有时本地对上位机的显式授权暂时不成立 (互斥), 但不伪造请求电平:
    // 核心仲裁仍能看到真实请求, 因此维护结束后一个持续保持的旧电平不会产生假边沿.
    normal.host_authorized = host_selected && !stop && !maintenance_active;
    normal.local_ready = inputs.locally_ready && running_state_reached_ && !stop &&
        startup_policy_ == StartupPolicy::remote_required;
    normal.host_session_epoch = inputs.host_session_epoch;
    const auto normal_decision = normal_.update(normal);

    // 维护会话被替换 (会话身份代次变化): 旧维护授权与 token 立即失效, 且旧申请留下的封禁
    // 不适用于新身份 —— 新的 hello + authorize 就是一次新的显式维护申请, 不能因为旧申请
    // 被人工撤销过就把它一起拦住. 即使新会话在同一批里重新授权维护, 也只能得到新
    // generation 的 token, 旧目标不会静默沿用原期限.
    if (inputs.maintenance_session_epoch != 0 &&
        inputs.maintenance_session_epoch != maintenance_session_epoch_) {
        maintenance_session_epoch_ = inputs.maintenance_session_epoch;
        maintenance_barred_ = false;
        if (maintenance_holds_) {
            maintenance_gate_.invalidate();
            maintenance_holds_ = false;
            maintenance_token_.reset();
            ++maintenance_revocations_;
        }
    }
    // 明确人工选源 (本地/Jetson) 撤销当前维护请求的本次资格: 新来源尚未就绪时保持无来源等待,
    // 不能在同一周期因旧维护请求电平仍为真而回落维护. 只有维护请求/链路退出后再次提出
    // (新的显式申请) 或新会话身份接入才解除这条限制. 顺序上放在身份处理之后: 同一周期
    // 同时出现人工选源与新身份申请时, 人工决定仍优先.
    if (!inputs.maintenance_link_alive || !inputs.maintenance_request) {
        maintenance_barred_ = false;
    } else if (selection_edge && (local_selected || host_selected)) {
        maintenance_barred_ = true;
    }
    // 5. 互斥维护来源: 仅在无正常来源 (原来源已释放)、非人工停止、会话/请求具备时可用.
    //    不要求左拨杆停在待机位: Jetson 可以撤请求释放来源以交接维护 (I05).
    //    人工停止态允许建立会话, 但不签发执行授权.
    const bool maintenance_allowed = !stop && running_state_reached_ && inputs.locally_ready &&
        !maintenance_barred_ && !normal_decision.has_value() && inputs.maintenance_link_alive &&
        inputs.maintenance_request && (inputs.remote_layout == RemoteLayout::legacy_ch4 ||
            commissioning_maintenance_selected_);
    if (maintenance_allowed) {
        if (!maintenance_holds_) {
            const auto token =
                maintenance_gate_.arm(CommandSource::maintenance, true, inputs.locally_ready);
            if (token) {
                maintenance_token_ = *token;
                maintenance_holds_ = true;
                ++maintenance_switches_;
            }
        } else if (!maintenance_token_ || !maintenance_gate_.accepts(*maintenance_token_)) {
            maintenance_holds_ = false;
            maintenance_token_.reset();
        }
    } else if (maintenance_holds_) {
        maintenance_gate_.invalidate();
        maintenance_holds_ = false;
        maintenance_token_.reset();
        ++maintenance_revocations_;
    }

    AuthorityDecision out{};
    out.manual_stop = stop;
    if (normal_decision) {
        out.source = to_app_source(normal_decision->source);
        out.token = normal_decision->token;
        out.reason = Reason::none;
    } else if (maintenance_holds_) {
        out.source = Source::maintenance;
        out.token = maintenance_token_;
        out.reason = Reason::none;
    } else if (stop) {
        out.reason = Reason::manual_stop;
    } else if (!running_state_reached_) {
        out.reason = Reason::awaiting_remote;
    } else if (local_selected || host_selected) {
        // 已显式选了来源但未取得授权: 原因由正常仲裁与维护状态共同决定.
        out.reason = Reason::no_authority;
    } else {
        out.reason = Reason::standby;
    }
    holds_ = out.token.has_value();
    token_ = out.token;
    source_ = out.source;
    reason_ = out.reason;
    return out;
}

void SentinelAuthority::revoke() noexcept {
    normal_.revoke();
    if (maintenance_holds_) {
        maintenance_gate_.invalidate();
        maintenance_holds_ = false;
        maintenance_token_.reset();
        ++maintenance_revocations_;
    }
    holds_ = false;
    token_.reset();
    source_ = Source::none;
}
} // 命名空间 ec::sentinel::app
