#pragma once
#include "ec/channel.hpp"
#include "ec/drivers/device_watch.hpp"

namespace ec::sentinel::board {
// 两总线各自保留容量, 轮流消费; 一侧空闲时另一侧使用剩余周期预算.
// 每路保持 FIFO, 不在设备编码器展开之前合并/覆盖位置帧.
template<std::size_t CapacityPerBus, class Critical>
class CanInbox {
public:
    explicit CanInbox(Critical& critical) noexcept : first_(critical), second_(critical) {}
    [[nodiscard]] PushResult try_push(const drivers::StampedCanFrame& frame) noexcept {
        return frame.bus == 1 ? first_.try_push(frame) : second_.try_push(frame);
    }
    [[nodiscard]] std::optional<drivers::StampedCanFrame> try_pop() noexcept {
        first_next_ = !first_next_;
        auto result = first_next_ ? first_.try_pop() : second_.try_pop();
        if (!result) { result = first_next_ ? second_.try_pop() : first_.try_pop(); }
        return result;
    }
private:
    EventQueue<drivers::StampedCanFrame, CapacityPerBus, Critical> first_;
    EventQueue<drivers::StampedCanFrame, CapacityPerBus, Critical> second_;
    bool first_next_{};
};
} // 命名空间 ec::sentinel::board
