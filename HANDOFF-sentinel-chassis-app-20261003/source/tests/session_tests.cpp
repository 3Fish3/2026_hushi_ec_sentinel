// M5 契约: 控制权来源仲裁、最小会话生命周期、执行上下文与接收结果.
// 全部离线: 用确定性会话 ID 与本地 Tick 模拟双端重启, 不声称已完成真实上位机握手.
#include "ec/channel.hpp"
#include "ec/control/authority.hpp"
#include "ec/control/execution_context.hpp"
#include "ec/platform/host.hpp"
#include "ec/session.hpp"
#include <cstdio>
#include <cstdlib>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3); // 非零退出且不触发 CRT 交互弹窗, ctest 要求非交互
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::CommandSource;
using ec::Grant;
using ec::Tick;
using ec::control::ControlAuthority;
using ec::session::BootNonceState;
using ec::session::IntentResult;
using ec::session::SessionId;
using ec::session::SessionTracker;

ControlAuthority::Inputs host_requesting() {
    ControlAuthority::Inputs inputs{};
    inputs.remote_online = true;
    inputs.remote_takeover = false;
    inputs.host_session_alive = true;
    inputs.host_requests = true;
    inputs.host_authorized = true;
    inputs.local_ready = true;
    return inputs;
}

// R05: ExecutionGate 只承诺“启动内”撤销: 本地 token 从不跨重启保存, 线协议也不携带本地
// generation, 因此不用随机初值伪装跨重启隔离; 跨重启身份由会话层 (SessionId.boot) 与
// 本地显式重建流程负责.
void gate_scope_contracts() {
    ec::ExecutionGate gate{};
    const auto first = gate.arm(CommandSource::host, true, true);
    const auto second = gate.arm(CommandSource::host, true, true);
    CHECK(first.has_value() && second.has_value());
    // 同一启动内重新 arm 使旧 token 失效 (已确认行为, 不能被跨重启修法破坏).
    CHECK(first->generation != second->generation);
    CHECK(!gate.accepts(*first));
    CHECK(gate.accepts(*second));
    // 未持有时反复 drop/invalidate 不制造代次变化 (idle 路径不推进状态).
    ControlAuthority idle{};
    CHECK(!idle.update(ec::control::ControlAuthority::Inputs{}).has_value());
    CHECK(!idle.update(ec::control::ControlAuthority::Inputs{}).has_value());
    CHECK(idle.revocations() == 0);
    // 撤销后同一启动内不再接受任何旧 token.
    gate.invalidate();
    CHECK(!gate.accepts(*second));
    CHECK(!gate.arm(CommandSource::none, true, true).has_value());
    CHECK(!gate.arm(CommandSource::host, false, true).has_value());
    CHECK(!gate.arm(CommandSource::host, true, false).has_value());
}

// R03: 会话身份变化必须切断旧目标的执行身份.
void session_identity_change_contracts() {
    const SessionId a{0x11, 0xa001, 1};
    const SessionId a_rebuilt{0x11, 0xa001, 2};
    const SessionId b{0x11, 0xa002, 1}; // 同来源, 新启动实例
    SessionTracker tracker{{500, 64}};
    CHECK(tracker.begin(a, {0}));
    const auto epoch_a = tracker.epoch();
    CHECK(tracker.begin(a, {10})); // 同会话重复 begin: 保活
    CHECK(tracker.epoch() == epoch_a);
    // 同来源同启动实例但会话号更大: 对端主动重建, 身份未变.
    CHECK(tracker.begin(a_rebuilt, {20}));
    CHECK(tracker.epoch() != epoch_a);
    const auto epoch_rebuilt = tracker.epoch();
    // 身份不同且本地未明确允许: 拒绝, 旧会话不被顶掉.
    CHECK(!tracker.begin(b, {30}));
    CHECK(tracker.epoch() == epoch_rebuilt);
    CHECK(tracker.current() == a_rebuilt);
    CHECK(tracker.heartbeat(a_rebuilt, {40})); // 旧会话仍有效
    // 本地明确进入重建流程后才接受新身份; 之后旧身份不能夺回.
    CHECK(tracker.begin(b, {50}, true));
    CHECK(tracker.epoch() != epoch_rebuilt);
    CHECK(tracker.current() == b);
    CHECK(!tracker.begin(a, {60}));
    CHECK(!tracker.begin(a_rebuilt, {60}));
    CHECK(tracker.current() == b);
    // 本地复位也计为一次身份变化.
    const auto epoch_b = tracker.epoch();
    tracker.reset();
    CHECK(tracker.epoch() != epoch_b);
    CHECK(!tracker.has_session());
    // 期限非法的意图判为 expired (不是"还没有意图"); 计数发生在结束点, 预检本身不计数.
    CHECK(tracker.begin(a, {70}) || tracker.begin(a, {70}, true));
    const auto expired_before = tracker.expired();
    CHECK(tracker.check_intent(a, 1, {70}, ec::Lifetime{0}) == IntentResult::expired);
    CHECK(tracker.expired() == expired_before); // check_intent 是纯预检
    CHECK(tracker.accept_intent(a, 1, {70}, ec::Lifetime{0}) == IntentResult::expired);
    CHECK(tracker.expired() == expired_before + 1);
    CHECK(tracker.check_intent(a, 1, {70}, ec::Lifetime{50}) == IntentResult::accepted);
    CHECK(tracker.expired() == expired_before + 1);
}

// R03: 授权层据会话身份代次撤销旧授权; 替换后的新会话重新产生边沿才能重新取得控制权.
void session_replacement_authority_contracts() {
    ControlAuthority authority{};
    auto inputs = host_requesting();
    inputs.host_session_epoch = 1;
    const auto first = authority.update(inputs);
    CHECK(first.has_value() && first->source == CommandSource::host);
    CHECK(authority.holds());
    const auto old_token = *authority.token();
    CHECK(authority.accepts(old_token));
    // 会话被替换: 本周期不授予任何来源, 旧 token 立即失效.
    inputs.host_session_epoch = 2;
    CHECK(!authority.update(inputs).has_value());
    CHECK(!authority.holds());
    CHECK(!authority.accepts(old_token));
    // 新会话重新握手(电平仍保持)后重新取得控制权, 且 token 与旧的不同.
    const auto second = authority.update(inputs);
    CHECK(second.has_value() && second->source == CommandSource::host);
    CHECK(second->token.generation != old_token.generation);
    // 会话替换的同一周期里, 遥控接管仍然可以生效.
    ControlAuthority other{};
    auto remote_inputs = host_requesting();
    remote_inputs.host_session_epoch = 5;
    CHECK(other.update(remote_inputs).has_value());
    remote_inputs.host_session_epoch = 6;
    remote_inputs.remote_takeover = true;
    const auto takeover = other.update(remote_inputs);
    CHECK(takeover.has_value() && takeover->source == CommandSource::remote);
}

// R04: 先判定 → 入队 → 才提交序号; 队列满不得消耗序号, 重试同一序号必须成功.
void intent_transaction_contracts() {
    const SessionId host{0x11, 0xa001, 1};
    SessionTracker tracker{{500, 64}};
    CHECK(tracker.begin(host, {0}));
    const ec::Lifetime ttl{200};
    // 只判定不提交: 重复判定同一序号仍为 accepted, 说明未消耗序号.
    CHECK(tracker.check_intent(host, 5, {10}, ttl) == IntentResult::accepted);
    CHECK(tracker.check_intent(host, 5, {10}, ttl) == IntentResult::accepted);
    CHECK(!tracker.has_intent());
    // 提交后同一序号算重复, 回退算乱序.
    CHECK(tracker.commit_intent(host, 5, {10}, ttl));
    CHECK(tracker.has_intent() && tracker.last_sequence() == 5);
    CHECK(tracker.check_intent(host, 5, {11}, ttl) == IntentResult::duplicate);
    CHECK(tracker.check_intent(host, 4, {11}, ttl) == IntentResult::stale);
    CHECK(tracker.check_intent(host, 6, {11}, ttl) == IntentResult::accepted);
    // 队列满的场景: 判定通过但未提交, 释放一个槽后用同一序号重试仍然 accepted.
    CHECK(tracker.check_intent(host, 7, {12}, ttl) == IntentResult::accepted);
    CHECK(tracker.check_intent(host, 7, {13}, ttl) == IntentResult::accepted);
    CHECK(tracker.commit_intent(host, 7, {13}, ttl));
    CHECK(tracker.check_intent(host, 7, {14}, ttl) == IntentResult::duplicate);
    // 会话/链路/期限条件在判定阶段就生效.
    CHECK(tracker.check_intent(host, 8, {14}, ec::Lifetime{0}) == IntentResult::expired);
    CHECK(tracker.check_intent(SessionId{0x99, 0xa001, 1}, 8, {14}, ttl) == IntentResult::no_session);
    CHECK(tracker.check_intent(host, 8, {600}, ttl) == IntentResult::not_ready);
    CHECK(!tracker.commit_intent(SessionId{0x99, 0xa001, 1}, 8, {14}, ttl));
    // accept_intent 仍是判定+提交的合成入口.
    CHECK(tracker.accept_intent(host, 9, {14}, ttl) == IntentResult::accepted);
    CHECK(tracker.accept_intent(host, 9, {14}, ttl) == IntentResult::duplicate);
    CHECK(tracker.last_sequence() == 9);
}

void authority_host_contracts() {
    ControlAuthority authority{};
    auto inputs = host_requesting();
    // 遥控在线但没有接管动作: 上位机凭显式请求获得控制权.
    const auto first = authority.update(inputs);
    CHECK(first && first->source == CommandSource::host && authority.holds());
    CHECK(authority.accepts(first->token));
    // 请求电平保持时不再产生新边沿: token 不变, 不反复重新授权.
    const auto second = authority.update(inputs);
    CHECK(second && second->token == first->token);
    CHECK(authority.switches() == 0 && authority.revocations() == 0);
    // 会话丢失即撤销, 即使请求仍在.
    inputs.host_session_alive = false;
    CHECK(!authority.update(inputs));
    CHECK(!authority.holds() && !authority.accepts(first->token));
    CHECK(authority.revocations() == 1);
    // 会话恢复但请求电平没有重新抬起: 不算新请求, 不给授权.
    inputs.host_session_alive = true;
    CHECK(!authority.update(inputs));
    // 明确重新请求(边沿)后才恢复, 且是新 token.
    inputs.host_requests = false;
    CHECK(!authority.update(inputs));
    inputs.host_requests = true;
    const auto again = authority.update(inputs);
    CHECK(again && again->source == CommandSource::host);
    CHECK(again->token != first->token);
    // 本地未就绪时不授权.
    inputs.local_ready = false;
    CHECK(!authority.update(inputs));
    CHECK(authority.revocations() == 2);
    // 本地没给上位机授权时不授权(链路存活不等于授权).
    inputs.local_ready = true;
    inputs.host_requests = false;
    CHECK(!authority.update(inputs));
    inputs.host_requests = true;
    inputs.host_authorized = false;
    CHECK(!authority.update(inputs));
}

void authority_takeover_contracts() {
    ControlAuthority authority{};
    auto inputs = host_requesting();
    const auto host_token = authority.update(inputs);
    CHECK(host_token && host_token->source == CommandSource::host);

    // 遥控接管: 同周期内接管请求优先于上位机请求.
    inputs.remote_takeover = true;
    inputs.host_requests = false;
    const auto taken = authority.update(inputs);
    CHECK(taken && taken->source == CommandSource::remote);
    CHECK(!authority.accepts(host_token->token)); // 切换来源即撤销旧 token
    CHECK(authority.accepts(taken->token));
    CHECK(authority.switches() == 1);
    const auto stable = authority.update(inputs);
    CHECK(stable && stable->token == taken->token); // 保持时不换 token

    // 遥控掉线: 只撤销, 不转交上位机, 即使上位机一直在请求.
    inputs.host_requests = true;
    CHECK(authority.update(inputs));
    inputs.remote_online = false;
    inputs.remote_takeover = false;
    CHECK(!authority.update(inputs));
    CHECK(!authority.holds() && !authority.accepts(stable->token));
    // 保持旧请求电平多周期仍然不授权.
    for (int i = 0; i < 5; ++i) { CHECK(!authority.update(inputs)); }
    CHECK(!authority.holds());
    // 重新请求才恢复.
    inputs.host_requests = false;
    CHECK(!authority.update(inputs));
    inputs.host_requests = true;
    const auto recovered = authority.update(inputs);
    CHECK(recovered && recovered->source == CommandSource::host);
    // 强制撤销(故障动作)后旧 token 立即失效.
    authority.revoke();
    CHECK(!authority.holds() && !authority.accepts(recovered->token));
}

void authority_token_contracts() {
    ControlAuthority authority{};
    const auto decision = authority.update(host_requesting());
    CHECK(decision);
    // 线协议里的 generation 不能当成本地授权: 只有本组件的 token 有效.
    const Grant forged{decision->token.generation + 1, CommandSource::host};
    CHECK(!authority.accepts(forged));
    const Grant wrong_source{decision->token.generation, CommandSource::remote};
    CHECK(!authority.accepts(wrong_source));
    // 空来源永远无效.
    CHECK(!authority.accepts(Grant{}));
}

// V08: 公开计数必须在事件结束点更新一次 —— check_intent 是纯预检, 不计数.
void intent_counter_contracts() {
    ec::session::SessionTracker tracker{{100, 8}};
    const ec::session::SessionId host{3, 0x1234, 1};
    const ec::Lifetime ttl{50};
    CHECK(tracker.begin(host, {10}));
    // accepted / duplicate / stale 各一次: 返回值与计数必须一一对应.
    CHECK(tracker.accept_intent(host, 2, {20}, ttl) == ec::session::IntentResult::accepted);
    CHECK(tracker.accept_intent(host, 2, {21}, ttl) == ec::session::IntentResult::duplicate);
    CHECK(tracker.accept_intent(host, 1, {22}, ttl) == ec::session::IntentResult::stale);
    CHECK(tracker.accepted() == 1 && tracker.duplicates() == 1 && tracker.stale() == 1);
    // 预检不计数: 同一个事件预检多次也不改变计数.
    const auto accepted_before = tracker.accepted();
    CHECK(tracker.check_intent(host, 5, {23}, ttl) == ec::session::IntentResult::accepted);
    CHECK(tracker.check_intent(host, 5, {23}, ttl) == ec::session::IntentResult::accepted);
    CHECK(tracker.accepted() == accepted_before);
    // 非法期限: 判定为 expired 并计数一次.
    const auto expired_before = tracker.expired();
    CHECK(tracker.accept_intent(host, 6, {24}, ec::Lifetime{0}) == ec::session::IntentResult::expired);
    CHECK(tracker.expired() == expired_before + 1);
    // 无会话/链路不新鲜不是意图判定, 不计入任何计数器.
    const auto totals = std::array<std::uint32_t, 4>{tracker.accepted(), tracker.duplicates(),
        tracker.stale(), tracker.expired()};
    CHECK(tracker.accept_intent(ec::session::SessionId{9, 1, 1}, 7, {25}, ttl) ==
        ec::session::IntentResult::no_session);
    CHECK(tracker.accept_intent(host, 7, {1000}, ttl) == ec::session::IntentResult::not_ready);
    CHECK(tracker.accepted() == totals[0] && tracker.duplicates() == totals[1] &&
        tracker.stale() == totals[2] && tracker.expired() == totals[3]);
    // 分阶段接收: 队列满不得消耗序号, 也不得计入 accepted; 提交点才计数.
    ec::session::SessionTracker staged{{100, 8}};
    CHECK(staged.begin(host, {10}));
    const auto verdict = staged.check_intent(host, 3, {20}, ttl);
    CHECK(verdict == ec::session::IntentResult::accepted);
    CHECK(staged.accepted() == 0); // 还没提交: 队列满等失败路径不应留下计数
    CHECK(staged.commit_intent(host, 3, {20}, ttl));
    CHECK(staged.accepted() == 1);
    CHECK(staged.commit_intent(host, 3, {21}, ttl) == false);
    CHECK(staged.duplicates() == 1);
    CHECK(staged.commit_intent(host, 2, {22}, ttl) == false);
    CHECK(staged.stale() == 1);
    CHECK(staged.commit_intent(host, 4, {23}, ec::Lifetime{0}) == false);
    CHECK(staged.expired() == 1);
}

void execution_context_contracts() {
    ec::ExecutionGate gate{};
    const auto token = gate.arm(CommandSource::remote, true, true);
    CHECK(token);
    // 未配置停止策略时不能在拒绝时静默选择动作.
    const ec::control::ExecutionContext unset{gate, {}};
    const auto denied = unset.evaluate({*token, true, false, true});
    CHECK(!denied.execute && denied.reason == ec::control::DenyReason::target_stale);
    CHECK(denied.stop == ec::control::StopAction::unspecified && !denied.actionable());
    // 显式配置后给出明确动作.
    ec::control::ExecutionPolicy policy{};
    policy.on_target_stale = ec::control::StopAction::hold_position;
    policy.on_authority_lost = ec::control::StopAction::disable_output;
    const ec::control::ExecutionContext context{gate, policy};
    const auto stale = context.evaluate({*token, true, false, true});
    CHECK(stale.stop == ec::control::StopAction::hold_position && stale.actionable());
    const auto lost_link = context.evaluate({*token, true, true, false});
    CHECK(!lost_link.execute && lost_link.reason == ec::control::DenyReason::link_stale);
    CHECK(lost_link.stop == ec::control::StopAction::disable_output);
    // 未持有时按机构策略处理, 而不是复用上一次输出.
    gate.invalidate();
    const auto no_grant = context.evaluate({*token, true, true, true});
    CHECK(!no_grant.execute && no_grant.reason == ec::control::DenyReason::no_grant);
    CHECK(context.evaluate({*token, false, true, true}).reason == ec::control::DenyReason::not_ready);
    // 全部条件满足才执行.
    const auto fresh_token = gate.arm(CommandSource::remote, true, true);
    CHECK(fresh_token);
    const auto ok = context.evaluate({*fresh_token, true, true, true});
    CHECK(ok.execute && ok.reason == ec::control::DenyReason::none);
}

void session_contracts() {
    SessionTracker tracker{{500, 64}};
    const SessionId host{7, 0x1234, 1};
    CHECK(!tracker.has_session());
    CHECK(!tracker.link_alive({100}));
    // 没有会话时意图被拒绝.
    CHECK(tracker.accept_intent(host, 1, {100}, {100}) == IntentResult::no_session);
    CHECK(tracker.begin(host, {100}));
    CHECK(tracker.has_session() && tracker.link_alive({100}) && !tracker.link_alive({600}));
    // 第一次意图: 记录首次接收时刻与期限.
    CHECK(tracker.accept_intent(host, 1, {100}, {100}) == IntentResult::accepted);
    CHECK(tracker.intent_alive({150}) && !tracker.intent_alive({200}));
    CHECK(tracker.intent_deadline() && tracker.intent_deadline()->ms == 200);
    // 同一序号重传: 算重复, 不延长意图期限.
    CHECK(tracker.accept_intent(host, 1, {150}, {100}) == IntentResult::duplicate);
    CHECK(tracker.intent_deadline()->ms == 200);
    CHECK(!tracker.intent_alive({200}));
    // 心跳只刷新链路, 不影响意图期限.
    CHECK(tracker.heartbeat(host, {150}));
    CHECK(tracker.link_alive({600}));
    CHECK(tracker.intent_deadline()->ms == 200);
    // 乱序回退序号被拒绝.
    CHECK(tracker.accept_intent(host, 0, {160}, {100}) == IntentResult::stale);
    // 向前跳跃超过窗口: 要求显式重建会话.
    CHECK(tracker.accept_intent(host, 1 + 64 + 1, {160}, {100}) == IntentResult::stale);
    // 合法的下一序号被接受.
    CHECK(tracker.accept_intent(host, 2, {160}, {100}) == IntentResult::accepted);
    CHECK(tracker.intent_alive({250}) && !tracker.intent_alive({260}));
    // 空 TTL 不能构成有效意图.
    CHECK(tracker.accept_intent(host, 3, {170}, {0}) == IntentResult::expired);
    // 身份不匹配的心跳/意图无效.
    const SessionId other{8, 0x1234, 1};
    CHECK(!tracker.heartbeat(other, {170}));
    CHECK(tracker.accept_intent(other, 3, {170}, {100}) == IntentResult::no_session);
    // 链路失效后意图不被接受(需要先恢复链路).
    CHECK(tracker.accept_intent(host, 3, {1000}, {100}) == IntentResult::not_ready);
    CHECK(!tracker.intent_alive({1000}));
    CHECK(tracker.heartbeat(host, {1000}));
    CHECK(tracker.accept_intent(host, 3, {1000}, {100}) == IntentResult::accepted);

    // 显式重建: 同一身份但会话号更大 → 重置序号窗口.
    const SessionId rebuilt{7, 0x1234, 2};
    CHECK(tracker.begin(rebuilt, {1100}));
    CHECK(tracker.last_sequence() == 0 && !tracker.has_intent());
    CHECK(tracker.accept_intent(rebuilt, 1, {1100}, {100}) == IntentResult::accepted);
    // 同一身份且会话号不变: 只当心跳, 不重置窗口.
    const auto accepted_before = tracker.accepted();
    CHECK(tracker.begin(rebuilt, {1110}));
    CHECK(tracker.accept_intent(rebuilt, 1, {1110}, {100}) == IntentResult::duplicate);
    CHECK(tracker.accepted() == accepted_before);
    // 旧会话号重放不能夺回会话, 也不能重置序号窗口.
    const SessionId old_session{7, 0x1234, 1};
    CHECK(!tracker.begin(old_session, {1120}));
    CHECK(tracker.current() == rebuilt);
    CHECK(tracker.accept_intent(old_session, 5, {1120}, {100}) == IntentResult::no_session);
    // 对端重启(新 boot): 身份变化必须由本地明确允许重建后才接受, 否则旧会话不被顶掉.
    const SessionId rebooted{7, 0x9999, 1};
    CHECK(!tracker.begin(rebooted, {1200}));
    CHECK(tracker.current() == rebuilt);
    CHECK(!tracker.has_intent() || tracker.current() == rebuilt);
    CHECK(tracker.begin(rebooted, {1210}, true));
    CHECK(tracker.current() == rebooted && tracker.last_sequence() == 0);
    CHECK(tracker.accept_intent(rebooted, 1, {1210}, {100}) == IntentResult::accepted);
    // 本地复位后所有会话与意图失效.
    tracker.reset();
    CHECK(!tracker.has_session() && !tracker.link_alive({1200}));
    CHECK(tracker.accept_intent(rebooted, 2, {1200}, {100}) == IntentResult::no_session);
}

void boot_nonce_contracts() {
    CHECK(ec::session::boot_nonce_state(std::nullopt, 0) == BootNonceState::unavailable);
    CHECK(ec::session::boot_nonce_state(0, 0) == BootNonceState::not_unique);
    CHECK(ec::session::boot_nonce_state(0x1234, 0x1234) == BootNonceState::not_unique);
    CHECK(ec::session::boot_nonce_state(0x1234, 0x4321) == BootNonceState::ok);
    // 平台适配的最小实现: 没有可靠来源时必须返回空, 而不是常量.
    struct NoSource final : ec::session::BootNonceProvider {
        std::optional<std::uint32_t> value{};
        [[nodiscard]] std::optional<std::uint32_t> acquire() noexcept override { return value; }
    } provider{};
    CHECK(ec::session::boot_nonce_state(provider.acquire(), 0x4321) == BootNonceState::unavailable);
    provider.value = 0x7777;
    CHECK(ec::session::boot_nonce_state(provider.acquire(), 0x4321) == BootNonceState::ok);
}

void restart_and_power_contracts() {
    // 双端重启模拟: 本机重启后旧会话与旧 token 都不能复用.
    // 注意: 跨重启身份由会话层 (SessionId.boot) 与本地显式重建负责, 授权门只提供启动内撤销;
    // 旧 token 失效不靠“代次不重叠”, 而靠本地 token 不跨重启保存、通道与门一起复位.
    ec::ExecutionGate gate{};
    SessionTracker tracker{{500, 64}};
    const SessionId host{3, 0xabc, 1};
    CHECK(tracker.begin(host, {100}));
    CHECK(tracker.accept_intent(host, 1, {100}, {1000}) == IntentResult::accepted);
    const auto old_token = gate.arm(CommandSource::host, true, true);
    CHECK(old_token && gate.accepts(*old_token) && tracker.intent_alive({500}));
    // 本机重启: 授权门与通道一起复位, 旧 token 不能被新门接受(新门尚未 arm).
    ec::ExecutionGate restarted{};
    tracker.reset();
    // 新门尚未 arm: 不接受任何 token.
    CHECK(!restarted.accepts(*old_token));
    CHECK(tracker.accept_intent(host, 2, {500}, {1000}) == IntentResult::no_session);
    // 重启后同样来源获得授权: 旧 token 之所以不能用, 不是靠“代次不重叠”(不同启动实例的
    // 代次可以相同), 而是靠本地 token 从不跨重启保存、线协议不携带本地 generation.
    const auto restarted_token = restarted.arm(CommandSource::host, true, true);
    CHECK(restarted_token && restarted.accepts(*restarted_token));
    // 旧 token 在来源/代次不匹配时一律拒绝; 同一启动内撤销后同样失效.
    restarted.invalidate();
    CHECK(!restarted.accepts(*restarted_token));
    const auto rearmed = restarted.arm(CommandSource::host, true, true);
    CHECK(rearmed && restarted.accepts(*rearmed));
    // 显式重新握手后才恢复: 新会话 + 新 token, 旧目标必须重新下发.
    CHECK(tracker.begin(host, {500}));
    CHECK(tracker.accept_intent(host, 1, {500}, {1000}) == IntentResult::accepted);

    // 执行器供电恢复: 只能靠明确的供电事件或已观测到的重连; 这里用显式事件模拟.
    CHECK(restarted.accepts(*rearmed));
    restarted.invalidate(); // 供电事件: 与执行器一起撤销授权
    CHECK(!restarted.accepts(*restarted_token));
    const auto after_power = restarted.arm(CommandSource::host, true, true);
    CHECK(after_power && *after_power != *restarted_token);
    // 旧的邮箱目标带的是旧 token, 消费者用自己的门就能拒绝.
    ec::host::MutexCriticalSection lock{};
    ec::CommandMailbox<float, ec::host::MutexCriticalSection> mailbox{lock};
    mailbox.publish({1.5f, {600}, {100}, *restarted_token});
    const auto command = mailbox.read_fresh({600});
    CHECK(command.has_value());
    CHECK(!restarted.accepts(command->grant)); // 旧授权在供电恢复后无效
    CHECK(!mailbox.read_fresh({700}).has_value()); // 目标自身也过期
}

void channel_result_contracts() {
    ec::host::MutexCriticalSection lock{};
    // 队列满返回确定结果, 不静默丢也没覆盖.
    ec::EventQueue<int, 2, ec::host::MutexCriticalSection> events{lock};
    CHECK(events.try_push(1) == ec::PushResult::accepted);
    CHECK(events.try_push(2) == ec::PushResult::accepted);
    CHECK(events.try_push(3) == ec::PushResult::full);
    CHECK(events.try_pop() == 1 && events.try_pop() == 2 && !events.try_pop());
    // 连续目标: 只保留最新值, 并带自己的 TTL.
    ec::CommandMailbox<float, ec::host::MutexCriticalSection> mailbox{lock};
    const Grant token{42, CommandSource::host};
    mailbox.publish({0.5f, {10}, {20}, token});
    mailbox.publish({0.6f, {11}, {20}, token});
    const auto latest = mailbox.read_fresh({12});
    CHECK(latest && latest->value == 0.6f);
    CHECK(!mailbox.read_fresh({31}).has_value());
}
} // 测试辅助

int main() {
    authority_host_contracts();
    gate_scope_contracts();
    session_identity_change_contracts();
    session_replacement_authority_contracts();
    intent_transaction_contracts();
    authority_takeover_contracts();
    authority_token_contracts();
    execution_context_contracts();
    intent_counter_contracts();
    session_contracts();
    boot_nonce_contracts();
    restart_and_power_contracts();
    channel_result_contracts();
    std::puts("session and authority contracts passed");
    return 0;
}
