#pragma once
#include "ec/platform/stm32f407.hpp"
#include "ec/transport/byte_pipe.hpp"
#include <array>
#include <span>

namespace ec::stm32f407 {
struct UartDmaStats {
    std::uint32_t rx_bytes{}, tx_bytes{}, overflows{}, discarded{}, framing{}, noise{}, overrun{};
    std::uint32_t rx_restarts{}, tx_errors{}, busy_rejects{}, last_hal_error{};
};
struct UartRead { std::size_t size{}; std::uint32_t generation{}; };
// 单个任务负责 read/try_send/stats; UART 与 DMA 回调须同一抢占优先级.
// DMA RX 不会因 CPU 屏蔽中断停止, 复制预算与 HT/TC 完整性仍由装配者保证.
// TX 为单个有界自有缓冲, busy 拒绝整段, 不保存旧请求等待重试.
template<std::size_t RxCapacity, std::size_t TxCapacity> class UartDmaPort {
public:
    explicit UartDmaPort(UART_HandleTypeDef& handle) noexcept : handle_(handle) {}
    // 仅用于初始化或 DMA 已停止后的重启, 不允许外部在活动接收中直接调用.
    bool start() noexcept {
        rx_.reset();
        generation_ = generation_ + 1;
        const auto ok = HAL_UART_Receive_DMA(&handle_, rx_.dma_buffer(), RxCapacity) == HAL_OK;
        if (ok) { handle_.Instance->CR1 = handle_.Instance->CR1 | USART_CR1_IDLEIE; }
        return ok;
    }
    UartRead read(std::span<std::uint8_t> output) noexcept {
        const auto saved = lock_.enter();
        const UartRead result{recovering_ ? 0 : rx_.read(output), generation_};
        lock_.exit(saved);
        return result;
    }
    transport::SendResult try_send(std::span<const std::uint8_t> bytes) noexcept {
        if (bytes.empty() || bytes.size() > tx_.size()) { return transport::SendResult::invalid; }
        const auto saved = lock_.enter();
        // DMA 故障结束后只在硬件不再引用缓冲时释放; 不在 ISR 中轮询等待.
        if (busy_ && handle_.gState == HAL_UART_STATE_READY &&
            handle_.hdmatx->State != HAL_DMA_STATE_BUSY && !(handle_.hdmatx->Instance->CR & DMA_SxCR_EN)) {
            busy_ = false; tx_errors_ = tx_errors_ + 1;
        }
        if (busy_ || recovering_) { busy_rejects_ = busy_rejects_ + 1; lock_.exit(saved); return transport::SendResult::busy; }
        for (std::size_t i = 0; i < bytes.size(); ++i) { tx_[i] = bytes[i]; }
        inflight_ = bytes.size(); busy_ = true;
        lock_.exit(saved);
        // HAL 调用在临界区外; 仅任务可启动 TX, 回调仅释放本次缓冲.
        const auto status = HAL_UART_Transmit_DMA(&handle_, tx_.data(), static_cast<std::uint16_t>(bytes.size()));
        if (status == HAL_OK) { return transport::SendResult::accepted; }
        const auto restore = lock_.enter();
        busy_ = false; inflight_ = 0; tx_errors_ = tx_errors_ + 1;
        lock_.exit(restore);
        return status == HAL_BUSY ? transport::SendResult::busy : transport::SendResult::error;
    }
    void on_rx_event() noexcept {
        if (recovering_) { return; }
        const auto before = rx_.overflows();
        rx_.on_dma_write((RxCapacity - __HAL_DMA_GET_COUNTER(handle_.hdmarx)) % RxCapacity);
        if (rx_.overflows() != before) { generation_ = generation_ + 1; }
    }
    void on_tx_complete() noexcept {
        tx_bytes_ = tx_bytes_ + static_cast<std::uint32_t>(inflight_);
        inflight_ = 0; busy_ = false;
    }
    void on_error(std::uint32_t error) noexcept {
        if (error & HAL_UART_ERROR_FE) { rx_.note_error(transport::UartError::framing); }
        if (error & HAL_UART_ERROR_NE) { rx_.note_error(transport::UartError::noise); }
        if (error & HAL_UART_ERROR_ORE) { rx_.note_error(transport::UartError::overrun); }
        // HAL 的 DMA 错误路径可能只结束 UART 状态, 另一 DMA stream 仍活动.
        // 先使当前字节流失效, 由任务统一终止硬件再重启, 不在 ISR 中等待.
        last_error_ = error;
        handle_.Instance->CR1 = handle_.Instance->CR1 & ~USART_CR1_IDLEIE;
        recovering_ = true;
        generation_ = generation_ + 1;
    }
    bool poll_recovery() noexcept {
        if (!recovering_) { return true; }
        if (HAL_UART_Abort(&handle_) != HAL_OK) { return false; }
        const auto saved = lock_.enter();
        if (busy_) { tx_errors_ = tx_errors_ + 1; }
        busy_ = false; inflight_ = 0;
        recovering_ = false;
        rx_restarts_ = rx_restarts_ + 1;
        lock_.exit(saved);
        return start();
    }
    UartDmaStats stats() noexcept {
        const auto saved = lock_.enter();
        const UartDmaStats result{rx_.received(), tx_bytes_, rx_.overflows(), rx_.discarded(),
            rx_.framing(), rx_.noise(), rx_.overrun(), rx_restarts_, tx_errors_, busy_rejects_, last_error_};
        lock_.exit(saved);
        return result;
    }
private:
    static_assert(TxCapacity > 0 && TxCapacity <= 4096);
    UART_HandleTypeDef& handle_;
    IrqCriticalSection lock_{};
    transport::DmaBytePipe<RxCapacity> rx_{};
    std::array<std::uint8_t, TxCapacity> tx_{};
    // 所有任务访问在临界区内, IRQ 写者彼此不抢占.
    volatile std::uint32_t generation_{}, tx_bytes_{}, rx_restarts_{}, tx_errors_{}, busy_rejects_{};
    volatile std::size_t inflight_{};
    volatile bool busy_{}, recovering_{};
    volatile std::uint32_t last_error_{};
};
} // 命名空间 ec::stm32f407
