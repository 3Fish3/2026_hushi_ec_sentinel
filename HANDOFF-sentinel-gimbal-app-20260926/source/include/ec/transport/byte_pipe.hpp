#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace ec::transport {
enum class UartError : std::uint8_t { framing, noise, overrun };

// 环形 DMA 接收管线: 硬件写入 buf_, ISR 公布写指针, 消费者复制后前进读指针.
// write==read 表示空; 覆盖未读数据时丢弃全部未读, 本轮不交付, 避免新旧字节拼接.
// 调用约定: 所有索引操作互斥; 可由同优先级 ISR 独占消费, 或由任务屏蔽 ISR 后消费.
// 屏蔽 CPU 中断不会停止 DMA, 必须保证复制及时完成且 HT/TC 不丢失整圈.
template<std::size_t Capacity>
class DmaBytePipe {
    static_assert(Capacity >= 8 && Capacity <= 4096);
public:
    [[nodiscard]] std::uint8_t* dma_buffer() noexcept { return buf_.data(); }
    [[nodiscard]] const std::uint8_t* dma_buffer() const noexcept { return buf_.data(); }
    [[nodiscard]] static constexpr std::size_t capacity() noexcept { return Capacity; }

    void on_dma_write(std::size_t dma_write) noexcept {
        if (dma_write >= Capacity) { return; }
        const std::size_t prev = write_;
        if (dma_write == prev) { return; }
        const std::size_t produced = delta(prev, dma_write);
        const std::size_t unread = delta(read_, prev);
        if (unread + produced >= Capacity) {
            discarded_ += unread;
            ++overflows_;
            read_ = write_ = dma_write;
            return;
        }
        write_ = dma_write;
        received_ += produced;
    }

    // DMA 已停止且即将从缓冲区起点重启时调用; 不清零累计计数.
    void reset() noexcept {
        discarded_ += available();
        read_ = write_ = 0;
    }

    [[nodiscard]] std::size_t available() const noexcept { return delta(read_, write_); }

    std::size_t read(std::span<std::uint8_t> dst) noexcept {
        const std::size_t n = dst.size() < available() ? dst.size() : available();
        for (std::size_t i = 0; i < n; ++i) {
            dst[i] = buf_[read_];
            read_ = (read_ + 1) % Capacity;
        }
        return n;
    }

    void note_error(UartError error) noexcept {
        switch (error) {
        case UartError::framing: ++framing_; break;
        case UartError::noise: ++noise_; break;
        case UartError::overrun: ++overrun_; break;
        }
    }

    [[nodiscard]] std::uint32_t overflows() const noexcept { return overflows_; }
    [[nodiscard]] std::uint32_t received() const noexcept { return received_; }
    [[nodiscard]] std::uint32_t discarded() const noexcept { return discarded_; }
    [[nodiscard]] std::uint32_t framing() const noexcept { return framing_; }
    [[nodiscard]] std::uint32_t noise() const noexcept { return noise_; }
    [[nodiscard]] std::uint32_t overrun() const noexcept { return overrun_; }
    [[nodiscard]] std::size_t write_index() const noexcept { return write_; }
    [[nodiscard]] std::size_t read_index() const noexcept { return read_; }

private:
    [[nodiscard]] static constexpr std::size_t delta(std::size_t from, std::size_t to) noexcept {
        return (to + Capacity - from) % Capacity;
    }

    std::array<std::uint8_t, Capacity> buf_{};
    std::size_t read_{0};
    std::size_t write_{0};
    std::uint32_t received_{0};
    std::uint32_t discarded_{0};
    std::uint32_t overflows_{0};
    std::uint32_t framing_{0};
    std::uint32_t noise_{0};
    std::uint32_t overrun_{0};
};

// 有界发送缓冲: 空间不足时拒绝整段, 不拆开一次写入.
// peek 给出从读指针起的连续块, DMA 完成后再 pop; 环形回绕时一次只提供一段.
template<std::size_t Capacity>
class ByteTxQueue {
    static_assert(Capacity >= 8 && Capacity <= 4096);
public:
    [[nodiscard]] bool try_push(std::span<const std::uint8_t> bytes) noexcept {
        if (bytes.empty() || bytes.size() > Capacity - count_) { return false; }
        for (auto byte : bytes) {
            buf_[write_] = byte;
            write_ = (write_ + 1) % Capacity;
        }
        count_ += bytes.size();
        return true;
    }

    [[nodiscard]] std::span<const std::uint8_t> peek() const noexcept {
        if (count_ == 0) { return {}; }
        const std::size_t contiguous = Capacity - read_ < count_ ? Capacity - read_ : count_;
        return {buf_.data() + read_, contiguous};
    }

    void pop(std::size_t n) noexcept {
        if (n > count_) { n = count_; }
        read_ = (read_ + n) % Capacity;
        count_ -= n;
    }

    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    [[nodiscard]] std::size_t free() const noexcept { return Capacity - count_; }

private:
    std::array<std::uint8_t, Capacity> buf_{};
    std::size_t read_{0};
    std::size_t write_{0};
    std::size_t count_{0};
};
} // 命名空间 ec::transport
