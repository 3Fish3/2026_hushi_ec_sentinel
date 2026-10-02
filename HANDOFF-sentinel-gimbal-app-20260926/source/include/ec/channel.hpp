#pragma once
#include "ec/execution_gate.hpp"
#include "ec/time.hpp"
#include <array>
#include <concepts>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>

namespace ec {
// enter/exit 必须同步内存且不抛异常. 中断屏蔽策略应恢复原掩码, 不能直接开中断.
// 互斥锁策略仅适用于任务/线程上下文.
template<class L> concept CriticalSection = requires(L& lock) {
    { lock.enter() } noexcept;
    { lock.exit(lock.enter()) } noexcept -> std::same_as<void>;
};
template<CriticalSection L> class ScopedCritical {
public:
    explicit ScopedCritical(L& lock) noexcept : lock_(lock), token_(lock.enter()) {}
    ~ScopedCritical() { lock_.exit(token_); }
    ScopedCritical(const ScopedCritical&) = delete;
    ScopedCritical& operator=(const ScopedCritical&) = delete;
private:
    L& lock_;
    decltype(std::declval<L&>().enter()) token_;
};

template<class T> concept ChannelValue = std::is_trivially_copyable_v<T> &&
    std::is_nothrow_default_constructible_v<T> &&
    std::is_nothrow_copy_constructible_v<T> && std::is_nothrow_copy_assignable_v<T> &&
    (sizeof(T) <= 256);

// 每个临界区只做有界复制; 存储与锁的生命周期必须覆盖所有使用者.
// 临界区内不回调、不分配、不等待数据, 也不递归调用通道.
template<ChannelValue T, CriticalSection L> class LatestValue {
public:
    explicit LatestValue(L& lock) noexcept : lock_(lock) {}
    LatestValue(const LatestValue&) = delete;
    LatestValue& operator=(const LatestValue&) = delete;
    void publish(const T& value) noexcept {
        ScopedCritical guard(lock_);
        value_ = value;
    }
    [[nodiscard]] std::optional<T> read() const noexcept {
        ScopedCritical guard(lock_);
        return value_;
    }
    void clear() noexcept {
        ScopedCritical guard(lock_);
        value_.reset();
    }
private:
    L& lock_;
    std::optional<T> value_{};
};

template<class T> struct Command {
    T value{};
    Tick received_at{};
    Lifetime ttl{};
    Grant grant{};
};
template<ChannelValue T, CriticalSection L> class CommandMailbox {
public:
    explicit CommandMailbox(L& lock) noexcept : storage_(lock) {}
    void publish(const Command<T>& command) noexcept { storage_.publish(command); }
    void clear() noexcept { storage_.clear(); }
    [[nodiscard]] std::optional<Command<T>> read_fresh(Tick now) const noexcept {
        const auto command = storage_.read();
        if (!command || !fresh(now, command->received_at, command->ttl)) {
            return std::nullopt;
        }
        return command; // 消费者还必须用自己的授权门检查 grant.
    }
private:
    LatestValue<Command<T>, L> storage_;
};

template<ChannelValue T, CriticalSection L>
using Snapshot = LatestValue<Sample<T>, L>;

enum class PushResult { accepted, full };
// 队列满时拒绝新事件, 不静默覆盖一次性动作.
// 会话/序号去重由命令接收层负责.
template<ChannelValue T, std::size_t Capacity, CriticalSection L> class EventQueue {
    static_assert(Capacity > 0);
public:
    explicit EventQueue(L& lock) noexcept : lock_(lock) {}
    EventQueue(const EventQueue&) = delete;
    EventQueue& operator=(const EventQueue&) = delete;
    [[nodiscard]] PushResult try_push(const T& value) noexcept {
        ScopedCritical guard(lock_);
        if (count_ == Capacity) { return PushResult::full; }
        data_[write_] = value;
        write_ = (write_ + 1) % Capacity;
        ++count_;
        return PushResult::accepted;
    }
    [[nodiscard]] std::optional<T> try_pop() noexcept {
        ScopedCritical guard(lock_);
        if (count_ == 0) { return std::nullopt; }
        const T value = data_[read_];
        read_ = (read_ + 1) % Capacity;
        --count_;
        return value;
    }
    void clear() noexcept {
        ScopedCritical guard(lock_);
        read_ = write_ = count_ = 0;
    }
private:
    L& lock_;
    std::array<T, Capacity> data_{};
    std::size_t read_{0}, write_{0}, count_{0};
};
} // 命名空间 ec
