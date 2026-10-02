// 示例链路: 一个"示例协议"(demo_*) 通过 UartFrame 传 typed 消息, 解码后进 Mailbox/事件队列,
// 再由组级控制权与执行上下文决定能否下发给执行器.
// 明确声明: 这是通用示例协议, 不是整车/上位机正式协议, 不定义比赛语义也不冻结字节布局.
// 全部离线运行, 用内存收发代替真实串口; 执行器是替身, 不代表任何机构动作.
#include "ec/channel.hpp"
#include "ec/control/authority.hpp"
#include "ec/control/execution_context.hpp"
#include "ec/platform/host.hpp"
#include "ec/session.hpp"
#include "ec/transport/uart_frame.hpp"
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <vector>

namespace {
// 离散动作: 保存动作 ID 与发出时的授权身份/期限, 消费时重新核对.
// 放在命名空间作用域: 嵌套类在外层尚未完整时, 其隐式默认构造在 GCC 下不满足
// ChannelValue 的 nothrow 默认构造约束 (MSVC 不报错), 会让示例只在部分工具链上可编译.
struct PendingAction {
    std::uint32_t action{};
    ec::Grant grant{};
    ec::Tick received_at{};
    ec::Lifetime ttl{};
};

constexpr std::uint8_t demo_session = 1;   // 建立/重建会话
constexpr std::uint8_t demo_heartbeat = 2; // 链路存活
constexpr std::uint8_t demo_intent = 3;    // 连续目标: 最新值语义
constexpr std::uint8_t demo_action = 4;    // 离散动作: 有界事件队列
constexpr std::size_t demo_id_size = 12;   // source + boot + session
constexpr std::size_t demo_intent_size = 24;
constexpr std::size_t demo_action_size = 20;

void put_u32(std::span<std::uint8_t> out, std::size_t offset, std::uint32_t value) {
    out[offset] = static_cast<std::uint8_t>(value);
    out[offset + 1] = static_cast<std::uint8_t>(value >> 8);
    out[offset + 2] = static_cast<std::uint8_t>(value >> 16);
    out[offset + 3] = static_cast<std::uint8_t>(value >> 24);
}
std::uint32_t get_u32(std::span<const std::uint8_t> in, std::size_t offset) {
    return static_cast<std::uint32_t>(in[offset]) | (static_cast<std::uint32_t>(in[offset + 1]) << 8) |
        (static_cast<std::uint32_t>(in[offset + 2]) << 16) |
        (static_cast<std::uint32_t>(in[offset + 3]) << 24);
}
void put_float(std::span<std::uint8_t> out, std::size_t offset, float value) {
    std::uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    put_u32(out, offset, raw);
}
float get_float(std::span<const std::uint8_t> in, std::size_t offset) {
    const auto raw = get_u32(in, offset);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}
std::span<const std::uint8_t> payload_of(const ec::transport::UartFrame& frame) {
    return {frame.payload.data(), frame.size};
}

// 连续目标载荷: 时间戳/有效期/token 由通用 Command 包装携带, 载荷只放业务值.
struct DemoIntent {
    float target{};
};

// 执行器替身: 只记录最后被允许下发的目标与停止动作.
struct FakeActuator {
    std::optional<float> applied{};
    ec::control::StopAction last_stop{ec::control::StopAction::unspecified};
    unsigned applied_count{};
    unsigned stopped_count{};
    void apply(float value) {
        applied = value;
        ++applied_count;
    }
    void stop(ec::control::StopAction action) {
        last_stop = action;
        applied.reset();
        ++stopped_count;
    }
};

enum class Outcome : std::uint8_t {
    short_frame, ignored_type, session, request, queued, duplicate, stale, expired, no_session,
    queue_full, applied, denied,
};

// 接收侧: 帧 → 路由 → 通道. 不做许可判断; 许可只在 control() 里检查一次.
class DemoLink {
public:
    DemoLink()
        : mailbox_(lock_), actions_(lock_),
          tracker_(ec::session::SessionTracker::Config{ec::Lifetime{500}, 64}) {}

    [[nodiscard]] Outcome receive(const ec::transport::UartFrame& frame, ec::Tick now) {
        const auto payload = payload_of(frame);
        switch (frame.type) {
        case demo_session: {
            if (payload.size() != demo_id_size) { return Outcome::short_frame; }
            const auto id = read_id(payload);
            const auto before = tracker_.epoch();
            // 身份变化只有在本地明确允许重建时才接受: allow_rebuild_ 由应用在本地流程里置位.
            if (!tracker_.begin(id, now, allow_rebuild_)) { return Outcome::no_session; }
            allow_rebuild_ = false; // 一次性: 不留在后续帧里
            if (tracker_.epoch() != before) {
                // 会话被替换或重建: 旧目标与旧事件不得续用.
                mailbox_.clear();
                actions_.clear();
            }
            return Outcome::session;
        }
        case demo_heartbeat: {
            if (payload.size() != demo_id_size) { return Outcome::short_frame; }
            if (!tracker_.heartbeat(read_id(payload), now)) { return Outcome::no_session; }
            host_request_ = true; // 会话存活即视为持续请求(电平; 只有重新连接才产生新边沿)
            return Outcome::session;
        }
        case demo_intent: {
            if (payload.size() != demo_intent_size) { return Outcome::short_frame; }
            // 先核对来源与控制权: 未授权时直接拒绝, 不消耗序号状态,
            // 否则上位机重新接管后无法重发同一目标.
            const auto token = authority_.token();
            if (!token || authority_.source() != ec::CommandSource::host) {
                mailbox_.clear();
                return Outcome::denied;
            }
            const auto ttl = ec::Lifetime{get_u32(payload, 16)};
            switch (tracker_.accept_intent(read_id(payload), get_u32(payload, 12), now, ttl)) {
            case ec::session::IntentResult::accepted: break;
            case ec::session::IntentResult::duplicate: return Outcome::duplicate;
            case ec::session::IntentResult::stale: return Outcome::stale;
            case ec::session::IntentResult::expired: return Outcome::expired;
            case ec::session::IntentResult::no_session: return Outcome::no_session;
            case ec::session::IntentResult::not_ready: return Outcome::expired;
            }
            // token 只能来自本地控制权组件: 线上帧不携带 generation.
            mailbox_.publish({{get_float(payload, 20)}, now, ttl, *token});
            return Outcome::queued;
        }
        case demo_action: {
            if (payload.size() != demo_action_size) { return Outcome::short_frame; }
            // 先核对来源与控制权: 未授权时直接拒绝, 不消耗序号状态.
            const auto token = authority_.token();
            if (!token || authority_.source() != ec::CommandSource::host) {
                actions_.clear();
                return Outcome::denied;
            }
            const auto id = read_id(payload);
            const auto sequence = get_u32(payload, 12);
            const auto ttl = ec::Lifetime{200};
            // 事务顺序: 先判定 → 入队 → 才提交已接受序号.
            // 队列满不得消耗序号, 否则上位机重试同一序号会被误判为重复/乱序.
            const auto verdict = tracker_.check_intent(id, sequence, now, ttl);
            if (verdict != ec::session::IntentResult::accepted) { return Outcome::stale; }
            const PendingAction pending{get_u32(payload, 16), *token, now, ttl};
            if (actions_.try_push(pending) == ec::PushResult::full) { return Outcome::queue_full; }
            if (!tracker_.commit_intent(id, sequence, now, ttl)) { return Outcome::stale; }
            return Outcome::queued;
        }
        default: return Outcome::ignored_type;
        }
    }

    // 控制周期: 唯一一处许可与时效检查, 再决定下发或执行上层配置的停止动作.
    [[nodiscard]] Outcome control(ec::Tick now) {
        const auto decision = authority_.update(inputs(now));
        if (!decision) {
            mailbox_.clear();
            actions_.clear();
            // 授权丢失: 按上层为本机构配置的停止动作执行, 而不是默认归零.
            actuator_.stop(context_.evaluate({ec::Grant{}, true, false, false}).stop);
            return Outcome::denied;
        }
        const auto command = mailbox_.read_fresh(now);
        // 目标自带发出时的 token: 来源切换后旧目标必须被拒绝, 而不是继续执行.
        // 没有目标时用当前授权 token, 使拒绝原因落在"目标过期"而不是"未授权".
        const auto token = command ? command->grant : decision->token;
        const auto verdict = context_.evaluate({token, true, command.has_value(),
            tracker_.link_alive(now)});
        if (!verdict.execute) {
            actuator_.stop(verdict.stop);
            return Outcome::denied;
        }
        actuator_.apply(command->value.target);
        return Outcome::applied;
    }

    void revoke() {
        authority_.revoke();
        mailbox_.clear();
        actions_.clear();
    }
    // 应用层明确进入重建流程时置位: 只对下一次会话帧生效, 避免任意远端 begin 自动产生新身份.
    void allow_session_rebuild() noexcept { allow_rebuild_ = true; }
    void set_remote(bool online, bool takeover) {
        remote_online_ = online;
        remote_takeover_ = takeover;
    }
    [[nodiscard]] const FakeActuator& actuator() const noexcept { return actuator_; }
    [[nodiscard]] const ec::control::ControlAuthority& authority() const noexcept { return authority_; }
    [[nodiscard]] const ec::session::SessionTracker& tracker() const noexcept { return tracker_; }
    [[nodiscard]] std::size_t drained_actions() noexcept {
        std::size_t count = 0;
        while (actions_.try_pop().has_value()) { ++count; }
        return count;
    }
    // 消费一个离散动作: 必须仍然属于当前授权, 且在自身期限内; 否则丢弃不执行.
    [[nodiscard]] std::optional<std::uint32_t> next_action(ec::Tick now) noexcept {
        while (const auto pending = actions_.try_pop()) {
            if (!authority_.accepts(pending->grant)) { continue; }
            if (!ec::fresh(now, pending->received_at, pending->ttl)) { continue; }
            return pending->action;
        }
        return std::nullopt;
    }

private:
    static ec::session::SessionId read_id(std::span<const std::uint8_t> payload) {
        return {get_u32(payload, 0), get_u32(payload, 4), get_u32(payload, 8)};
    }
    [[nodiscard]] ec::control::ControlAuthority::Inputs inputs(ec::Tick now) {
        ec::control::ControlAuthority::Inputs in{};
        in.remote_online = remote_online_;
        in.remote_takeover = remote_takeover_;
        in.host_session_alive = tracker_.link_alive(now);
        in.host_requests = host_request_ && tracker_.link_alive(now);
        in.host_authorized = true;
        in.local_ready = true;
        in.host_session_epoch = tracker_.epoch(); // 会话被替换时撤销旧授权
        return in;
    }

    ec::host::MutexCriticalSection lock_{};
    ec::CommandMailbox<DemoIntent, ec::host::MutexCriticalSection> mailbox_;
    ec::EventQueue<PendingAction, 4, ec::host::MutexCriticalSection> actions_;
    ec::session::SessionTracker tracker_;
    ec::control::ControlAuthority authority_{};
    ec::control::ExecutionPolicy policy_{ec::control::StopAction::hold_position,
        ec::control::StopAction::disable_output};
    ec::control::ExecutionContext context_{authority_.gate(), policy_};
    FakeActuator actuator_{};
    bool remote_online_{true};
    bool remote_takeover_{false};
    bool host_request_{};
    bool allow_rebuild_{};
};

// 主机侧编码: 只用示例协议, 不代表正式上位机协议.
struct HostFrames {
    static ec::transport::UartFrame identity_frame(std::uint8_t type, const ec::session::SessionId& id) {
        ec::transport::UartFrame frame{};
        frame.type = type;
        frame.size = static_cast<std::uint16_t>(demo_id_size);
        put_u32(frame.payload, 0, static_cast<std::uint32_t>(id.source));
        put_u32(frame.payload, 4, id.boot);
        put_u32(frame.payload, 8, id.session);
        return frame;
    }
    static ec::transport::UartFrame intent(const ec::session::SessionId& id, std::uint32_t sequence,
        float value, std::uint32_t ttl_ms) {
        auto frame = identity_frame(demo_intent, id);
        frame.size = static_cast<std::uint16_t>(demo_intent_size);
        put_u32(frame.payload, 12, sequence);
        put_u32(frame.payload, 16, ttl_ms);
        put_float(frame.payload, 20, value);
        return frame;
    }
    static ec::transport::UartFrame action(const ec::session::SessionId& id, std::uint32_t sequence,
        std::uint32_t action) {
        auto frame = identity_frame(demo_action, id);
        frame.size = static_cast<std::uint16_t>(demo_action_size);
        put_u32(frame.payload, 12, sequence);
        put_u32(frame.payload, 16, action);
        return frame;
    }
    // 整帧编码/解码往返, 证明示例协议走的是通用 UartFrame 而不是另起一套分帧.
    static bool round_trip(const ec::transport::UartFrame& frame) {
        std::array<std::uint8_t, ec::transport::uart_wire_max> wire{};
        const auto size = ec::transport::encode_uart_frame(frame.type, frame.sequence,
            payload_of(frame), wire);
        if (size == 0) { return false; }
        ec::transport::UartFrameDecoder decoder{};
        std::optional<ec::transport::UartFrame> decoded{};
        for (std::size_t i = 0; i < size; ++i) {
            const auto result = decoder.push(wire[i]);
            if (result) { decoded = result; }
        }
        return decoded && decoded->type == frame.type && decoded->size == frame.size &&
            std::memcmp(decoded->payload.data(), frame.payload.data(), frame.size) == 0;
    }
};
} // 示例协议

int main() {
    using ec::Tick;
    const ec::session::SessionId host{0x11, 0xa001, 1};
    DemoLink link{};
    const auto log = [](const char* what, Outcome outcome) {
        std::printf("%-30s -> %d\n", what, static_cast<int>(outcome));
    };

    // 1. 帧往返: 示例协议复用通用 UartFrame, 不另起一套分帧.
    if (!HostFrames::round_trip(HostFrames::identity_frame(demo_session, host))) { return 1; }

    // 控制周期按 10 ms 推进: 边沿与时序判据都必须在循环里被观测到.
    std::uint32_t now = 0;
    Outcome last = Outcome::denied;
    const auto step_to = [&](std::uint32_t target) {
        while (now < target) {
            now = (target - now > 10) ? now + 10 : target; // 精确落在目标时刻
            last = link.control(Tick{now});
        }
        return last;
    };
    const auto deliver = [&](const char* what, const ec::transport::UartFrame& frame) {
        const auto outcome = link.receive(frame, Tick{now});
        log(what, outcome);
        return outcome;
    };

    // 2. 会话建立 + 心跳(请求控制权) + 遥控在线但无接管动作.
    step_to(100);
    if (deliver("session begin", HostFrames::identity_frame(demo_session, host)) != Outcome::session) { return 2; }
    if (deliver("heartbeat", HostFrames::identity_frame(demo_heartbeat, host)) != Outcome::session) { return 3; }
    // 会话身份刚变化: 本周期按“未请求”处理, 旧授权不得续用.
    if (step_to(110) != Outcome::denied || link.authority().holds()) { return 4; }
    if (step_to(120) != Outcome::denied) { return 4; } // 新会话重新产生边沿并取得授权, 但还没有目标
    if (!link.authority().holds() || link.authority().source() != ec::CommandSource::host) { return 4; }

    // 3. 最新值目标与自身时效.
    if (deliver("intent #1 = 1.25", HostFrames::intent(host, 1, 1.25f, 200)) != Outcome::queued) { return 5; }
    if (step_to(130) != Outcome::applied) { return 5; }
    if (!link.actuator().applied || *link.actuator().applied != 1.25f) { return 5; }
    // 重传同一序号只算重复, 不延长意图期限.
    if (deliver("intent #1 retransmit", HostFrames::intent(host, 1, 1.25f, 200)) != Outcome::duplicate) { return 6; }
    if (step_to(319) != Outcome::applied) { return 6; }
    // 到达期限(意图在 120 收到, TTL 200; 未被重传延长): 执行上层配置的停止动作.
    if (step_to(320) != Outcome::denied) { return 7; }
    if (link.actuator().last_stop != ec::control::StopAction::hold_position) { return 7; }
    // 乱序回退与不合法期限.
    step_to(350);
    if (deliver("intent #0 out of order", HostFrames::intent(host, 0, 1.0f, 200)) != Outcome::stale) { return 8; }
    if (deliver("intent #2 ttl=0", HostFrames::intent(host, 2, 1.0f, 0)) != Outcome::expired) { return 8; }

    // 4. 遥控接管: 上位机旧目标立刻失效, 停止动作来自上层策略.
    if (deliver("intent #3 = 2.0", HostFrames::intent(host, 3, 2.0f, 2000)) != Outcome::queued) { return 9; }
    if (step_to(360) != Outcome::applied) { return 9; }
    link.set_remote(true, true);
    if (step_to(370) != Outcome::denied) { return 10; }
    if (link.authority().source() != ec::CommandSource::remote) { return 10; }
    if (link.actuator().last_stop != ec::control::StopAction::disable_output) { return 10; }

    // 5. 遥控失联: 只撤销, 不转交上位机; 持续保持的请求电平不算新请求.
    link.set_remote(false, false);
    if (step_to(420) != Outcome::denied || link.authority().holds()) { return 11; }
    if (deliver("intent #4 while no owner", HostFrames::intent(host, 4, 3.0f, 500)) != Outcome::denied) { return 11; }
    // 上位机链路断开超过 TTL: 控制周期能观测到请求电平落下.
    if (step_to(1220) != Outcome::denied || link.authority().holds()) { return 12; }
    if (deliver("heartbeat reconnect", HostFrames::identity_frame(demo_heartbeat, host)) != Outcome::session) { return 12; }
    if (step_to(1230) != Outcome::denied) { return 12; } // 重新握手后还没有新目标
    if (!link.authority().holds() || link.authority().source() != ec::CommandSource::host) { return 12; }
    // 未授权期间拒绝过的序号没有被消耗, 重新接管后可以重发同一序号.
    if (deliver("intent #4 = 3.5", HostFrames::intent(host, 4, 3.5f, 500)) != Outcome::queued) { return 13; }
    if (step_to(1240) != Outcome::applied) { return 13; }
    if (!link.actuator().applied || *link.actuator().applied != 3.5f) { return 13; }

    // 6. 离散动作: 有界队列, 满时给出确定结果; 队列满不得消耗序号.
    step_to(1300);
    for (std::uint32_t i = 0; i < 4; ++i) {
        if (deliver("action", HostFrames::action(host, 5 + i, 100 + i)) != Outcome::queued) { return 14; }
    }
    if (deliver("action #9 (queue full)", HostFrames::action(host, 9, 200)) != Outcome::queue_full) { return 14; }
    // 队列中的动作必须属于当前授权且未过期: 逐个消费.
    const auto first_action = link.next_action(Tick{1300});
    if (!first_action || *first_action != 100) { return 14; }
    // 释放一个槽后用同一序号重试必须成功, 且只执行一次.
    if (deliver("action #9 retry", HostFrames::action(host, 9, 200)) != Outcome::queued) { return 14; }
    std::vector<std::uint32_t> drained{};
    while (const auto action = link.next_action(Tick{1300})) { drained.push_back(*action); }    if (drained.size() != 4 || drained[0] != 101 || drained[3] != 200) { return 14; }
    // 已接受并执行的序号重发不得再执行.
    if (deliver("action #9 duplicate", HostFrames::action(host, 9, 200)) != Outcome::stale) { return 14; }
    if (link.next_action(Tick{1300}).has_value()) { return 14; }
    // 排队后在自身期限内未消费: 过期不得执行.
    if (deliver("action #10", HostFrames::action(host, 10, 300)) != Outcome::queued) { return 14; }
    if (link.next_action(Tick{1300}).has_value() == false) { return 14; } // 期限内可见
    if (link.next_action(Tick{1501}).has_value()) { return 14; }

    // 7. 来源切换即撤销: 遥控接管后上位机旧 token 与邮箱里的旧目标都不能复用.
    const auto old_token = link.authority().token();
    if (!old_token || old_token->source != ec::CommandSource::host) { return 15; }
    link.set_remote(true, true);
    if (step_to(1310) != Outcome::denied) { return 15; }
    if (link.authority().source() != ec::CommandSource::remote) { return 15; }
    if (link.authority().accepts(*old_token)) { return 15; }
    if (deliver("intent #10 after takeover", HostFrames::intent(host, 10, 4.0f, 500)) != Outcome::denied) { return 15; }

    // 8. 本地强制撤销(故障动作/人工停止)同样使旧 token 失效.
    link.revoke();
    if (link.authority().holds() || link.authority().accepts(*old_token)) { return 16; }
    if (step_to(1320) != Outcome::denied) { return 16; }

    // 9. 会话替换 (R03): 对端重启建立新会话后, 旧会话未过期的目标与本地 host 授权都必须失效.
    //    目标必须自带发出时的会话/授权身份, 不能在执行时换发当前 token.
    link.set_remote(false, false);
    const ec::session::SessionId host_b{0x11, 0xa002, 1}; // 同来源, 新 boot
    // 先让链路自然过期, 请求电平落下; 持续保持的电平不算新请求, 必须重新握手.
    if (step_to(1800) != Outcome::denied || link.authority().holds()) { return 17; }
    if (deliver("A: heartbeat", HostFrames::identity_frame(demo_heartbeat, host)) != Outcome::session) { return 17; }
    if (step_to(1810) != Outcome::denied) { return 17; }
    if (deliver("A: intent #20 = 5.0", HostFrames::intent(host, 20, 5.0f, 2000)) != Outcome::queued) { return 17; }
    if (step_to(1820) != Outcome::applied) { return 17; }
    if (!link.actuator().applied || *link.actuator().applied != 5.0f) { return 17; }
    // 新会话建立: 旧目标仍在本地 TTL 内, 但不得再执行.
    if (deliver("B: session begin (not allowed)", HostFrames::identity_frame(demo_session, host_b)) != Outcome::no_session) { return 18; }
    // 本地明确进入重建流程后才接受新身份.
    link.allow_session_rebuild();
    if (deliver("B: session begin", HostFrames::identity_frame(demo_session, host_b)) != Outcome::session) { return 18; }
    if (step_to(1830) != Outcome::denied) { return 18; }
    // 旧会话的后续命令不能夺回控制权, 也不能被当成新目标(先被授权检查拦下或会话不符).
    const auto stale_intent = deliver("A: intent #21 after B", HostFrames::intent(host, 21, 6.0f, 500));
    if (stale_intent != Outcome::no_session && stale_intent != Outcome::denied) { return 18; }
    if (link.actuator().applied && *link.actuator().applied == 6.0f) { return 18; }
    // 新会话自己重新握手与下发目标后可以正常执行.
    if (deliver("B: heartbeat", HostFrames::identity_frame(demo_heartbeat, host_b)) != Outcome::session) { return 18; }
    if (step_to(1840) != Outcome::denied) { return 18; }
    if (deliver("B: intent #1 = 7.0", HostFrames::intent(host_b, 1, 7.0f, 500)) != Outcome::queued) { return 18; }
    if (step_to(1850) != Outcome::applied) { return 18; }
    if (!link.actuator().applied || *link.actuator().applied != 7.0f) { return 18; }
    std::puts("phase 9: session replacement drops the old target and the old grant");

    // 10. 排队后的动作在撤销时不执行: 队列元素自带授权身份与期限.
    if (deliver("B: action #40", HostFrames::action(host_b, 40, 400)) != Outcome::queued) { return 19; }
    if (!link.next_action(Tick{1850})) { return 19; } // 授权有效时可见
    if (deliver("B: action #41", HostFrames::action(host_b, 41, 401)) != Outcome::queued) { return 19; }
    link.revoke();
    if (link.next_action(Tick{1850}).has_value()) { return 19; }
    std::puts("phase 10: revoke drops queued actions and their grant identity");

    std::puts("demo link: session, authority, mailbox and event path executed (example protocol only)");
    return 0;
}
