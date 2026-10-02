#include "board_monitor.h"
#include "board_capture.hpp"
#include "board_raw.hpp"
#include "board_app.hpp"
#if EC_SENTINEL_CAPTURE
#include "capture.hpp"
#endif
#include "board_bench.hpp"
#include "board_monitor_protocol.hpp"
#include "board_registry.h"
#include "ec/platform/uart_dma.hpp"
#include "ec/transport/uart_frame.hpp"
#include "usart.h"
#include <algorithm>

#if EC_S1_UART6_MONITOR
namespace {
ec::stm32f407::UartDmaPort<512, ec::transport::uart_wire_max> port{huart6};
ec::transport::UartFrameDecoder decoder;
std::array<std::uint8_t, ec::transport::uart_wire_max> wire{};
std::array<std::uint8_t, ec::sentinel::monitor_payload_size> payload{};
std::uint32_t generation{}, last_reply{}, unknown{}, dropped{};
#if EC_SENTINEL_CAPTURE
std::array<std::uint8_t,32> stream_ack{};
std::uint16_t stream_ack_sequence{}, stream_sequence{};
bool stream_ack_pending{};
void service_stream() {
    std::size_t count{};
    const auto size=stream_ack_pending ? stream_ack.size() : ec_stream_packet(payload,count);
    if (!size) { return; }
    const auto data=stream_ack_pending ? std::span<const std::uint8_t>(stream_ack) : std::span<const std::uint8_t>(payload.data(),size);
    const auto length=ec::transport::encode_uart_frame(stream_ack_pending ? 0xb2 : 0xb3,
        stream_ack_pending ? stream_ack_sequence : stream_sequence,data,wire);
    if (length && port.try_send({wire.data(),length})==ec::transport::SendResult::accepted) {
        if (stream_ack_pending) { stream_ack_pending=false; }
        else { ec_stream_consume(count);++stream_sequence; }
    }
}
#endif
}
extern "C" void ec_monitor_init(void) {
    if (!port.start()) { Error_Handler(); }
}
extern "C" void ec_monitor_rx_event(void) { port.on_rx_event(); }
extern "C" void ec_monitor_tx_complete(void) { port.on_tx_complete(); }
extern "C" void ec_monitor_uart_error(std::uint32_t error) {
    port.on_error(error);
}
extern "C" void ec_monitor_poll(void) {
    if (!port.poll_recovery()) { Error_Handler(); }
    bool requested = false;
    bool bench_reply = false;
    bool registry_reply = false;
#if EC_S5_GM6020_BENCH
    bool bench_trace_reply = false;
    std::array<std::uint8_t,12> bench_trace_in{};
    std::size_t bench_trace_size{};
#endif
#if EC_SENTINEL_APP
    bool app_fast_reply = false;
#endif
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
    bool raw_reply = false;
    std::array<std::uint8_t, 12> raw_request{};
#endif
#if EC_SENTINEL_CAPTURE
    std::uint8_t capture_page = 255;
#endif
    std::uint16_t registry_first = 0;
    std::uint16_t sequence = 0;
    std::array<std::uint8_t, 64> chunk{};
    // 分块短临界区复制, 解帧/CRC 在临界区之外, 每周期处理量有上界.
    for (std::size_t budget = 512; budget; ) {
        const auto result = port.read({chunk.data(), chunk.size() < budget ? chunk.size() : budget});
        if (result.generation != generation) {
            decoder.reset(); requested = false; generation = result.generation;
            ec_bench_transport_lost();
        }
        if (!result.size) { break; }
        budget -= result.size;
        for (std::size_t i = 0; i < result.size; ++i) {
            if (const auto frame = decoder.push(chunk[i])) {
#if EC_SENTINEL_CAPTURE
                if (frame->type==0x32) {
                    if (frame->size!=12) { ++unknown;continue; }
                    if (ec_stream_request({frame->payload.data(),frame->size},stream_ack)) {
                        stream_ack_sequence=frame->sequence;stream_ack_pending=true;
                    }
                    continue;
                }
#endif
#if EC_S5_GM6020_BENCH
                if (frame->type==0x26) {
                    if (requested) { ++dropped; }
                    requested=true;bench_trace_reply=true;bench_reply=false;registry_reply=false;sequence=frame->sequence;
                    bench_trace_size=frame->size==bench_trace_in.size() ? frame->size : 0;
                    std::copy_n(frame->payload.begin(),bench_trace_size,bench_trace_in.begin());
                    continue;
                }
#endif
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
                if (frame->type == ec::sentinel::recording::request_type) {
                    if (frame->size != raw_request.size()) { ++unknown; continue; }
                    if (requested) { ++dropped; }
                    requested = true; raw_reply = true; sequence = frame->sequence;
#if EC_SENTINEL_APP
                    app_fast_reply = false;
#endif
                    std::copy_n(frame->payload.begin(), raw_request.size(), raw_request.begin());
                    continue;
                }
#endif
#if EC_SENTINEL_APP
                if (frame->type == ec::sentinel::app_fast_request && frame->size == 0) {
                    if (requested) { ++dropped; }
                    requested = true; app_fast_reply = true; raw_reply = false;
                    sequence = frame->sequence;
                    continue;
                }
#endif
                if (frame->type == ec::sentinel::monitor_request && frame->size == 0) {
                    // 同周期只保留最后一个只读请求, 不排队积压响应.
                    if (requested) { ++dropped; }
                    requested = true; bench_reply = false; registry_reply = false; sequence = frame->sequence;
#if EC_SENTINEL_CAPTURE
                    capture_page = 255;
#endif
                } else if (frame->type == ec::sentinel::registry_page_request) {
                    // 注册表分页请求: 只读快照, 载荷是起始记录序号.
                    if (frame->size != 2) { ++unknown; continue; }
                    if (requested) { ++dropped; }
                    requested = true; bench_reply = false; registry_reply = true;
                    registry_first = static_cast<std::uint16_t>(frame->payload[0] | (frame->payload[1] << 8));
                    sequence = frame->sequence;
#if EC_SENTINEL_CAPTURE
                    capture_page = 255;
                } else if (frame->type == ec::sentinel::capture::request_type) {
                    if (frame->size != 1 || frame->payload[0] >= ec::sentinel::capture::page_count) { ++unknown; continue; }
                    if (requested) { ++dropped; }
                    requested = true; bench_reply = false; registry_reply = false;
                    sequence = frame->sequence; capture_page = frame->payload[0];
#endif
                } else if (ec_bench_request(*frame)) {
                    requested = true; bench_reply = true; sequence = frame->sequence;
                } else { ++unknown; continue; }
                // 只有已识别的新请求才覆盖此前的响应类别; 无关坏帧不改变已排队的只读请求.
#if EC_S5_GM6020_BENCH
                bench_trace_reply=false;
#endif
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
                raw_reply = false;
#endif
#if EC_SENTINEL_APP
                app_fast_reply = false;
#endif
            }
        }
    }
    if (!requested) {
#if EC_SENTINEL_CAPTURE
        service_stream();
#endif
        return;
    }
    const auto now = HAL_GetTick();
    // S5 唯一客户端串行请求, 波形下载仍受 DMA 非阻塞出口约束; 不让只读观测的50ms限频吞掉控制ACK.
#if EC_S5_GM6020_BENCH
    if (!bench_reply && !bench_trace_reply)
#endif
    if (now - last_reply < 50) { ++dropped; return; }
    last_reply = now;
#if EC_S5_GM6020_BENCH
    if (bench_trace_reply) {
        const auto size=ec_bench_trace_request({bench_trace_in.data(),bench_trace_size},payload);
        const auto length=size ? ec::transport::encode_uart_frame(0xa1,sequence,{payload.data(),size},wire) : 0;
        if (!length || port.try_send({wire.data(),length})!=ec::transport::SendResult::accepted) { ++dropped; }
        return;
    }
#endif
#if EC_SENTINEL_APP
    if (app_fast_reply) {
        std::size_t size{};
        {
            ec::stm32f407::IrqCriticalSection lock;
            const auto saved = lock.enter();
            const auto& observation = ec::sentinel::ec_app_observation();
            static_assert(sizeof(observation.fast_payload) <= payload.size());
            size = observation.fast_size;
            std::copy_n(observation.fast_payload, size, payload.begin());
            lock.exit(saved);
        }
        const auto length = size ? ec::transport::encode_uart_frame(ec::sentinel::app_fast_response,
            sequence, {payload.data(), size}, wire) : 0;
        if (!length || port.try_send({wire.data(), length}) != ec::transport::SendResult::accepted) { ++dropped; }
        return;
    }
#endif
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
    if (raw_reply) {
        const auto size = ec_raw_request(raw_request, payload);
        const auto length = size ? ec::transport::encode_uart_frame(ec::sentinel::recording::response_type,
            sequence, {payload.data(), size}, wire) : 0;
        if (!length || port.try_send({wire.data(), length}) != ec::transport::SendResult::accepted) { ++dropped; }
        return;
    }
#endif
#if EC_SENTINEL_CAPTURE
    if (capture_page != 255) {
        auto size = ec_capture_snapshot(capture_page, payload);
        if (capture_page == 0 && size) {
            const auto s = port.stats();
            ec::sentinel::capture::Writer tail({payload.data() + size, payload.size() - size});
            for (const auto value : {s.rx_bytes, s.tx_bytes, s.overflows, s.discarded,
                    s.framing, s.noise, s.overrun, s.rx_restarts, s.tx_errors, s.busy_rejects,
                    s.last_hal_error, decoder.rejected(), unknown, dropped}) { tail.u32(value); }
            size = tail.size() ? size + tail.size() : 0;
        }
        const auto length = size ? ec::transport::encode_uart_frame(ec::sentinel::capture::response_type,
            sequence, {payload.data(), size}, wire) : 0;
        if (!length || port.try_send({wire.data(), length}) != ec::transport::SendResult::accepted) { ++dropped; }
        return;
    }
#endif
    if (bench_reply) {
        const auto size=ec_bench_snapshot(payload);
        const auto length=ec::transport::encode_uart_frame(0xa0,sequence,{payload.data(),size},wire);
        if (!length || port.try_send({wire.data(),length})!=ec::transport::SendResult::accepted) { ++dropped; }
        return;
    }
    if (registry_reply) {
        const auto size = ec_registry_encode_page(registry_first, payload.data(), payload.size());
        if (size != 0) {
            const auto length = ec::transport::encode_uart_frame(ec::sentinel::registry_page_response,
                sequence, {payload.data(), size}, wire);
            if (!length || port.try_send({wire.data(), length}) != ec::transport::SendResult::accepted) {
                ++dropped;
            }
        } else {
            // 注册表未启用或序号越界: 不回笼资料, 只计数, 不阻塞控制周期.
            ++dropped;
        }
        return;
    }
    const auto s = port.stats();
    const std::array<std::uint32_t, 14> link{s.rx_bytes, s.tx_bytes, s.overflows, s.discarded,
        s.framing, s.noise, s.overrun, s.rx_restarts, s.tx_errors, s.busy_rejects, s.last_hal_error,
        decoder.rejected(), unknown, dropped};
    ec::stm32f407::IrqCriticalSection lock;
    const auto saved = lock.enter();
    // 仅固定长度字段复制; 采样时刻在禁止 ISR 更新后读取, 不使用滞后的任务入口时间.
    const auto size = ec::sentinel::encode_monitor_snapshot(g_ec_s1, g_ec_io, uwTick, link, payload);
    lock.exit(saved);
    const auto length = ec::transport::encode_uart_frame(ec::sentinel::monitor_response,
        sequence, {payload.data(), size}, wire);
    if (!length || port.try_send({wire.data(), length}) != ec::transport::SendResult::accepted) { ++dropped; }
}
#else
extern "C" void ec_monitor_init(void) {}
extern "C" void ec_monitor_poll(void) {}
extern "C" void ec_monitor_rx_event(void) {}
extern "C" void ec_monitor_tx_complete(void) {}
extern "C" void ec_monitor_uart_error(std::uint32_t) {}
#endif
