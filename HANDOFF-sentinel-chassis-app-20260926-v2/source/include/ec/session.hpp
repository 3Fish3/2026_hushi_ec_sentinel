#pragma once
#include "ec/time.hpp"
#include <cstdint>
#include <optional>

namespace ec::session {
// 会话身份由对端声明, 本地只做核对. boot 是每次启动唯一的 nonce,
// 否则旧会话的序号与动作幂等记录会跨重启误判.
struct SessionId {
    std::uint64_t source{};  // 来源身份(例如上位机节点 id)
    std::uint32_t boot{};    // 对端启动实例
    std::uint32_t session{}; // 显式(重)建会话时递增的会话号
    [[nodiscard]] bool operator==(const SessionId&) const = default;
};

enum class IntentResult : std::uint8_t {
    accepted,    // 新意图, 已记录首次接收时刻
    duplicate,   // 同一序号重传; 不刷新已有意图的有效期
    stale,       // 乱序或回退序号, 不采用
    expired,     // 意图自身已过期
    no_session,  // 没有当前会话或身份不匹配
    not_ready,   // 会话存在但链路已失效(需要先重建/心跳)
};

// 接收侧会话跟踪: 维护当前会话、链路存活与意图序号窗口.
// 关键区分:
//  - 链路存活由心跳/会话帧维持(link_ttl), 与目标有效期无关;
//  - 意图有效期在首次接收时按本地时钟与外层 TTL 固定下来, 重传与心跳都不会延长它.
// 不做跨机时钟同步, 也不信任对端时间戳; 所有判据使用本地 Tick.
// 单任务拥有(或由通道保护), 不分配内存.
class SessionTracker {
public:
    struct Config {
        Lifetime link_ttl{500};
        std::uint32_t sequence_window{64}; // 允许的向前跳跃; 超过视为旧会话或丢失严重
    };

    SessionTracker() noexcept : SessionTracker(Config{}) {}
    explicit SessionTracker(const Config& config) noexcept : config_(config) {}

    // 显式建立/重建会话. 只有以下情况接受:
    //  - 还没有会话;
    //  - 身份完全相同: 当作一次心跳, 不重置序号窗口与意图期限;
    //  - 同来源同启动实例但会话号更大: 对端主动重建, 身份未变;
    //  - 身份不同(换对端/对端重启)且调用方明确允许重建(allow_identity_change):
    //    本地接入层在明确进入重建流程时才会置位, 不允许任意远端 begin 自动产生可执行新身份.
    [[nodiscard]] bool begin(const SessionId& id, Tick now, bool allow_identity_change = false) noexcept;
    // 心跳: 身份必须是当前会话; 只刷新链路存活, 不影响任何意图期限.
    [[nodiscard]] bool heartbeat(const SessionId& id, Tick now) noexcept;
    // 接收意图: link_ttl 之外还必须满足会话与序号约束.
    [[nodiscard]] IntentResult accept_intent(const SessionId& id, std::uint32_t sequence,
        Tick now, Lifetime intent_ttl) noexcept;
    // 只判定不提交: 供“先写有界队列, 再提交已接受序号”的事务顺序使用.
    // 队列满不得消耗序号, 否则重试会被误判为重复/乱序.
    //
    // 关于 sequence_window: 前向跳跃超过窗口一律拒绝并要求显式重建会话, 连续最新值与离散
    // 动作共用这一条规则. 依据是"跳到很远的序号"通常意味着丢帧很多或旧会话残留, 与其猜测
    // 中间语义不如让上层重建; 它不是为了限制正常更新率 —— 正常周期下每帧只前进 1.
    [[nodiscard]] IntentResult check_intent(const SessionId& id, std::uint32_t sequence,
        Tick now, Lifetime intent_ttl) const noexcept;
    // 提交一个已通过 check_intent 的意图; 身份已变化或判定不成立时返回 false.
    [[nodiscard]] bool commit_intent(const SessionId& id, std::uint32_t sequence,
        Tick now, Lifetime intent_ttl) noexcept;

    [[nodiscard]] bool has_session() const noexcept { return has_session_; }
    [[nodiscard]] const SessionId& current() const noexcept { return current_; }
    [[nodiscard]] bool link_alive(Tick now) const noexcept {
        return has_session_ && fresh(now, link_at_, config_.link_ttl);
    }
    // 最近一次被接受的意图序号; 没有意图时为 0 且 has_intent() 为假.
    [[nodiscard]] std::uint32_t last_sequence() const noexcept { return last_sequence_; }
    [[nodiscard]] bool has_intent() const noexcept { return has_intent_; }
    // 意图截止时刻在首次接收时确定; 重传不改变它.
    [[nodiscard]] std::optional<Tick> intent_deadline() const noexcept;
    [[nodiscard]] bool intent_alive(Tick now) const noexcept;
    [[nodiscard]] std::uint32_t accepted() const noexcept { return accepted_; }
    [[nodiscard]] std::uint32_t duplicates() const noexcept { return duplicates_; }
    [[nodiscard]] std::uint32_t stale() const noexcept { return stale_; }
    [[nodiscard]] std::uint32_t expired() const noexcept { return expired_; }
    // 会话身份变化代次: 接受新的会话身份(换来源/换启动实例/更高会话号)时递增.
    // 接入层与授权层据此识别“会话被替换”, 从而撤销旧授权并清空旧目标.
    [[nodiscard]] std::uint32_t epoch() const noexcept { return epoch_; }
    // 本地复位(例如本机重启): 丢弃会话与意图状态.
    void reset() noexcept;

private:
    Config config_{};
    SessionId current_{};
    bool has_session_{};
    Tick link_at_{};
    std::uint32_t last_sequence_{};
    bool has_intent_{};
    Tick intent_seen_at_{};
    Lifetime intent_ttl_{};
    // 计数只在事件结束点 (accept_intent/commit_intent) 更新一次; check_intent 是纯预检.
    void note(IntentResult verdict) noexcept;
    std::uint32_t accepted_{}, duplicates_{}, stale_{}, expired_{};
    std::uint32_t epoch_{};
    // 该计数在 const 判定入口 (check_intent) 里累加: 判定本身必须能反映"被期限拒绝"的次数.
};

// 启动实例: 每次上电必须唯一. 由平台提供(例如硬件 RNG), 失败必须显式报告,
// 不允许用常量、芯片 UID 或开机为 0 的 tick 冒充"每次启动唯一".
class BootNonceProvider {
public:
    virtual ~BootNonceProvider() = default;
    // 返回空表示本机没有可靠来源; 调用方必须据此报告缺口, 而不是继续宣称跨重启保护成立.
    [[nodiscard]] virtual std::optional<std::uint32_t> acquire() noexcept = 0;
};

enum class BootNonceState : std::uint8_t { ok, unavailable, not_unique };
// nonce 为 0 或与上一次相同都视为不可用(重启循环、RNG 失效或读取错误).
[[nodiscard]] constexpr BootNonceState boot_nonce_state(std::optional<std::uint32_t> nonce,
    std::uint32_t previous) noexcept {
    if (!nonce) { return BootNonceState::unavailable; }
    if (*nonce == 0 || *nonce == previous) { return BootNonceState::not_unique; }
    return BootNonceState::ok;
}
} // 命名空间 ec::session
