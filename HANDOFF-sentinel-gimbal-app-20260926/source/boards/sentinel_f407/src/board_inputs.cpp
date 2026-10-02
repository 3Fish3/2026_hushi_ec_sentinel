#include "board_inputs.hpp"
#include <atomic>
#include <cstring>

// 单写者/单读者槽位 + 序数校验 (seqlock):
//   写者: 序数 +1(变奇) → 写数据 → 序数 +1(变偶);
//   读者: 读序数 → 读数据 → 再读序数, 只有"两次相同且为偶"才接受.
// 目标上写者是中断上下文, 读者是控制任务; 主机测试里两者都是同一线程.
// 这里不需要真正的多核原子性, 只需要不被编译器重排, 因此用 relaxed 存取 + 屏障.
namespace ec::sentinel {
namespace {
template<class T> struct Slot {
    std::atomic<std::uint32_t> sequence{0};
    T value{};
    ec::Tick sampled_at{};
    bool present{false};
};

Slot<ec::drivers::Dr16Sample> remote_slot{};
Slot<ec::drivers::Bmi088Reading> imu_slot{};

template<class T> void publish(Slot<T>& slot, const T& value, ec::Tick at) noexcept {
    const auto current = slot.sequence.load(std::memory_order_relaxed);
    slot.sequence.store(current + 1, std::memory_order_relaxed); // 奇数: 写入中
    std::atomic_signal_fence(std::memory_order_release);
    slot.value = value;
    slot.sampled_at = at;
    slot.present = true;
    std::atomic_signal_fence(std::memory_order_release);
    slot.sequence.store(current + 2, std::memory_order_relaxed); // 偶数: 可读
}

template<class T> std::optional<ec::Sample<T>> read(const Slot<T>& slot, ec::Tick now,
    ec::Lifetime ttl) noexcept {
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto before = slot.sequence.load(std::memory_order_relaxed);
        if (before % 2 != 0) { continue; } // 正在写入
        ec::Sample<T> sample{};
        sample.value = slot.value;
        sample.sampled_at = slot.sampled_at;
        const bool present = slot.present;
        std::atomic_signal_fence(std::memory_order_acquire);
        const auto after = slot.sequence.load(std::memory_order_relaxed);
        if (before != after) { continue; } // 期间被写入, 重试
        if (!present) { return std::nullopt; }
        sample.validity = ec::fresh(now, sample.sampled_at, ttl) ? ec::Validity::valid
                                                                : ec::Validity::invalid;
        return sample;
    }
    return std::nullopt; // 连续三次都撞上写入: 本周期不给样本, 由调用方下周期再问
}
} // 命名空间

void input_publish_remote(const ec::drivers::Dr16Sample& sample, ec::Tick at) noexcept {
    publish(remote_slot, sample, at);
}

void input_remote_lost() noexcept {
    const auto current = remote_slot.sequence.load(std::memory_order_relaxed);
    remote_slot.sequence.store(current + 1, std::memory_order_relaxed);
    std::atomic_signal_fence(std::memory_order_release);
    remote_slot.present = false;
    std::atomic_signal_fence(std::memory_order_release);
    remote_slot.sequence.store(current + 2, std::memory_order_relaxed);
}

void input_publish_imu(const ec::drivers::Bmi088Sample& sample, ec::Tick at) noexcept {
    ec::drivers::Bmi088Reading reading{};
    reading.sample = sample;
    reading.sampled_at = at;
    publish(imu_slot, reading, at);
}

std::optional<ec::Sample<ec::drivers::Dr16Sample>> input_remote(ec::Tick now) noexcept {
    return read(remote_slot, now, kRemoteTtl);
}

void input_publish_imu_reading(const ec::drivers::Bmi088Reading& sample) noexcept {
    publish(imu_slot, sample, sample.sampled_at);
}

std::optional<ec::Sample<ec::drivers::Bmi088Reading>> input_imu_reading(ec::Tick now) noexcept {
    return read(imu_slot, now, kImuTtl);
}

std::optional<ec::Sample<ec::drivers::Bmi088Sample>> input_imu(ec::Tick now) noexcept {
    const auto reading = input_imu_reading(now);
    if (!reading) { return std::nullopt; }
    return ec::Sample<ec::drivers::Bmi088Sample>{reading->value.sample, reading->sampled_at, reading->validity};
}

void input_reset() noexcept {
    remote_slot.sequence.store(0, std::memory_order_relaxed);
    remote_slot.present = false;
    remote_slot.value = {};
    remote_slot.sampled_at = {};
    imu_slot.sequence.store(0, std::memory_order_relaxed);
    imu_slot.present = false;
    imu_slot.value = {};
    imu_slot.sampled_at = {};
}
} // 命名空间 ec::sentinel
