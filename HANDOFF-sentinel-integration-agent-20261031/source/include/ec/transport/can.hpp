#pragma once
#include <array>
#include <cstdint>

namespace ec::transport {
// 首个适配仅支持经典 CAN 标准数据帧; 不隐式截断扩展 ID 或远程帧.
struct CanFrame {
    std::uint16_t id{};
    std::uint8_t size{};
    std::array<std::uint8_t, 8> data{};
    [[nodiscard]] constexpr bool valid() const noexcept { return id <= 0x7ff && size <= 8; }
};
enum class SendResult { accepted, busy, invalid, error };
// accepted 表示发送端已复制帧, 不代表总线发送完成或设备接受.
// 实现不等待硬件空闲; 每个外设由一个任务拥有, 接收由 BSP 向通道显式投递.
class CanTx {
public:
    [[nodiscard]] virtual SendResult try_send(const CanFrame& frame) noexcept = 0;
protected:
    // 接口只借用显式组装的对象, 禁止经接口指针销毁对象或表达所有权.
    ~CanTx() = default;
};
} // 命名空间 ec::transport
