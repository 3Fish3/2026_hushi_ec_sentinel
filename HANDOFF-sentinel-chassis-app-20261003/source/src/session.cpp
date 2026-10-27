#include "ec/session.hpp"

namespace ec::session {
void SessionTracker::reset() noexcept {
    current_ = {};
    has_session_ = false;
    link_at_ = {};
    last_sequence_ = 0;
    has_intent_ = false;
    intent_seen_at_ = {};
    intent_ttl_ = {};
    ++epoch_; // 本地复位也是一次身份变化: 旧目标/旧授权不得续用.
}

bool SessionTracker::begin(const SessionId& id, Tick now, bool allow_identity_change) noexcept {
    if (!config_.link_ttl.valid()) { return false; }
    if (!has_session_) {
        current_ = id;
        has_session_ = true;
        link_at_ = now;
        // 新会话从零开始: 旧序号不能沿用, 否则序号回退会被误判为乱序.
        last_sequence_ = 0;
        has_intent_ = false;
        intent_ttl_ = {};
        ++epoch_;
        return true;
    }
    if (id == current_) {
        // 同一会话重复 begin: 只当作一次心跳, 不重置序号窗口与意图期限.
        return heartbeat(id, now);
    }
    const bool same_peer = id.source == current_.source && id.boot == current_.boot;
    if (!(same_peer && id.session > current_.session) && !allow_identity_change) {
        // 身份不同且本地未明确允许重建: 拒绝, 旧会话的序号窗口与意图保持不变.
        return false;
    }
    current_ = id;
    link_at_ = now;
    last_sequence_ = 0;
    has_intent_ = false;
    intent_ttl_ = {};
    ++epoch_;
    return true;
}

bool SessionTracker::heartbeat(const SessionId& id, Tick now) noexcept {
    if (!has_session_ || !(id == current_)) { return false; }
    link_at_ = now;
    return true;
}

// 计数口径: 每次"实际接收事件"只在它的结束点计一次 —— accept_intent() 或 commit_intent().
// check_intent() 是纯预检, 不计数; 同一个事件被预检多次不会多计.
IntentResult SessionTracker::accept_intent(const SessionId& id, std::uint32_t sequence,
    Tick now, Lifetime intent_ttl) noexcept {
    const auto verdict = check_intent(id, sequence, now, intent_ttl);
    if (verdict != IntentResult::accepted) {
        note(verdict);
        return verdict;
    }
    return commit_intent(id, sequence, now, intent_ttl) ? IntentResult::accepted : IntentResult::no_session;
}

void SessionTracker::note(IntentResult verdict) noexcept {
    switch (verdict) {
    case IntentResult::duplicate: ++duplicates_; return;
    case IntentResult::stale: ++stale_; return;
    case IntentResult::expired: ++expired_; return;
    case IntentResult::accepted: ++accepted_; return;
    default: return; // no_session / not_ready 不是意图判定, 不计入任何计数器
    }
}

IntentResult SessionTracker::check_intent(const SessionId& id, std::uint32_t sequence,
    Tick now, Lifetime intent_ttl) const noexcept {
    if (!has_session_ || !(id == current_)) { return IntentResult::no_session; }
    if (!link_alive(now)) { return IntentResult::not_ready; }
    if (!intent_ttl.valid()) {
        // 期限非法(0 或负)不是"还没有意图": 明确判为 expired, 由结束点计数.
        return IntentResult::expired;
    }
    if (has_intent_ && sequence == last_sequence_) {
        // 同一序号重传: 只算重复, 不延长原意图期限.
        return IntentResult::duplicate;
    }
    if (has_intent_ && sequence < last_sequence_) { return IntentResult::stale; }
    if (sequence - last_sequence_ > config_.sequence_window) {
        // 向前跳跃过大: 视为旧会话残留或链路丢失过多, 要求显式重建会话.
        return IntentResult::stale;
    }
    return IntentResult::accepted;
}

bool SessionTracker::commit_intent(const SessionId& id, std::uint32_t sequence,
    Tick now, Lifetime intent_ttl) noexcept {
    if (!has_session_ || !(id == current_)) { return false; }
    if (!intent_ttl.valid()) { note(IntentResult::expired); return false; }
    // 重复/乱序在提交点再判一次: 分阶段接收路径用这里作为事件的结束点.
    if (has_intent_ && sequence == last_sequence_) { note(IntentResult::duplicate); return false; }
    if (has_intent_ && sequence < last_sequence_) { note(IntentResult::stale); return false; }
    last_sequence_ = sequence;
    has_intent_ = true;
    intent_seen_at_ = now;
    intent_ttl_ = intent_ttl;
    note(IntentResult::accepted);
    return true;
}

std::optional<Tick> SessionTracker::intent_deadline() const noexcept {
    if (!has_intent_ || !intent_ttl_.valid()) { return std::nullopt; }
    return Tick{intent_seen_at_.ms + intent_ttl_.ms};
}

bool SessionTracker::intent_alive(Tick now) const noexcept {
    return has_intent_ && fresh(now, intent_seen_at_, intent_ttl_);
}
} // 命名空间 ec::session
