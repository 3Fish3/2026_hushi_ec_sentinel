#pragma once
#include "ec/drivers/dm.hpp"
#include "ec/time.hpp"
#include <span>

namespace ec::drivers {
struct DmDeviceConfig {
    std::uint8_t bus{};
    // 发送帧 ID, 也就是 MIT 命令帧 ID. 反馈载荷只用它的低 4 位做节点识别,
    // 因此共用 Master ID 的设备必须低 4 位不同, 否则无法区分 (见 valid_dm_layout).
    std::uint16_t can_id{};
    std::uint16_t master_id{}; // 反馈帧 ID, 与发送 ID 分开
    DmMitLimits limits{};      // PMAX/VMAX/TMAX 必须显式给出, 不设通用默认值
    Lifetime feedback_ttl{};
};

// 同一总线上命令 ID 必须唯一, 且 (master_id, 节点低 4 位) 必须唯一, 否则无法按载荷节点字段区分设备.
// 地址核对不读取 PMAX/VMAX/TMAX; 不能据此使未知量程设备产生有效物理反馈或 MIT 控制.
[[nodiscard]] bool valid_dm_addresses(std::span<const DmDeviceConfig> devices) noexcept;
[[nodiscard]] bool valid_dm_layout(std::span<const DmDeviceConfig> devices) noexcept;

enum class DmReceiveResult { accepted, ignored, invalid_config, invalid_frame, invalid_node, invalid_time };
enum class DmRequest : std::uint8_t { none, enable, disable };
// 只由反馈状态确认: 编码/发送成功不代表设备已使能, 也不代表故障已清除.
// 见 DmDevice::request_result: 表达"期望状态当前是否满足", 不是命令 ACK.
enum class DmRequestResult : std::uint8_t { unknown, pending, accepted, faulted };

struct DmDeviceState {
    Sample<DmFeedback> feedback{};
    DmRequest pending_request{DmRequest::none};
    bool enable_confirmed{}; // 曾在新鲜反馈中观察到 Enabled
};

// 单任务拥有, 不持有 CanTx: 因此结构上不可能自行发帧, 不存在自动 enable/清错/写零位.
// 只提供 MIT 目标与显式使能/失能编码; 原生位置速度模式、参数读写、自动零位与故障清除
// 属于后续能力, 本版不实现. 设备原生输出轴就是本接口的轴, 不做外部传动换算.
class DmDevice {
public:
    explicit DmDevice(DmDeviceConfig config) noexcept;
    DmDevice(const DmDevice&) = delete;
    DmDevice& operator=(const DmDevice&) = delete;
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] const DmDeviceConfig& config() const noexcept { return config_; }
    [[nodiscard]] const DmMitLimits& limits() const noexcept { return config_.limits; }
    // 反馈 ID 相同的其他节点不会被本设备接受, 也不会刷新本设备的时效.
    [[nodiscard]] DmReceiveResult receive(std::uint8_t bus,
        const transport::CanFrame& frame, Tick received_at) noexcept;
    [[nodiscard]] DmDeviceState state() const noexcept { return state_; }
    [[nodiscard]] bool online(Tick now) const noexcept {
        return state_.feedback.fresh_at(now, config_.feedback_ttl);
    }
    [[nodiscard]] bool enabled(Tick now) const noexcept {
        return online(now) && dm_status_enabled(state_.feedback.value.status);
    }
    [[nodiscard]] bool faulted(Tick now) const noexcept {
        return online(now) && dm_status_fault(state_.feedback.value.status);
    }
    // 请求结果: "期望状态**当前**是否满足", 不是本次命令的 ACK, 也不代表机械动作完成.
    // 它由"最近一次 encode_enable/disable 的方向"与"新鲜反馈里的状态位"决定, 因此:
    //  - 设备本来就处于目标状态时, 即使本次帧还没发出去也可能立刻报 accepted;
    //  - 没有新鲜反馈时一律 unknown; 故障状态报 faulted, 不自动清错或重试.
    // 需要"命令已送达/动作已完成"语义时, 由上层协议自己配对请求与反馈.
    [[nodiscard]] DmRequestResult request_result(Tick now) const noexcept;
    // 编码不改变设备状态; 调用方负责发送时机与授权.
    [[nodiscard]] std::optional<transport::CanFrame> encode_mit(const DmMitCommand& command) const noexcept;
    [[nodiscard]] std::optional<transport::CanFrame> encode_enable() noexcept;
    [[nodiscard]] std::optional<transport::CanFrame> encode_disable() noexcept;
    void reset() noexcept;
private:
    DmDeviceConfig config_;
    bool configured_{};
    DmDeviceState state_{};
    Tick last_received_{};
    bool has_received_{};
};
} // 命名空间 ec::drivers
