#pragma once
#include "ec/drivers/dji.hpp"
#include "ec/time.hpp"
#include <array>
#include <span>

namespace ec::drivers {
struct DjiDeviceConfig {
    std::uint8_t bus{};
    DjiMode mode{};
    std::uint8_t device_id{};
    float rotor_per_axis{}; // 转子角 / 轴侧角的正传动比, 不从型号推定.
    int direction{}; // 仅允许 +1 或 -1, 同时作用于反馈与输出.
    Lifetime feedback_ttl{};
    float max_rotor_speed_rad_s{}; // 整个采样间隔内的已知速度上界, 不是单帧实测值.
    std::int16_t output_limit_raw{}; // 归一化输出 ±1 对应的协议量, 不得超过协议范围.
};

// 初始化时检查全总线布局; 仅覆盖传入的 DJI 节点, 其他协议的 ID 仍需组装层核对.
// 仅验证总线地址/命令组, 不凭缺省换算值宣称设备可执行; 供已知地址但未配完运动参数的装配使用.
[[nodiscard]] bool valid_dji_addresses(std::span<const DjiDeviceConfig> devices) noexcept;
[[nodiscard]] bool valid_dji_layout(std::span<const DjiDeviceConfig> devices) noexcept;

enum class DjiReceiveResult { accepted, ignored, invalid_config, invalid_frame, invalid_time };
struct DjiDeviceState {
    Sample<DjiFeedback> rotor{};
    Sample<float> velocity_rad_s{};
    Sample<float> position_rad{};
};

// 单任务拥有, 不含 PID、仲裁或硬件输出. 首帧不推定关节零位/圈数.
// 配置和地址在对象生命周期内固定, 引用者必须先于本对象销毁.
class DjiDevice {
public:
    explicit DjiDevice(DjiDeviceConfig config) noexcept;
    DjiDevice(const DjiDevice&) = delete;
    DjiDevice& operator=(const DjiDevice&) = delete;
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] const DjiDeviceConfig& config() const noexcept { return config_; }
    [[nodiscard]] DjiReceiveResult receive(std::uint8_t bus,
        const transport::CanFrame& frame, Tick received_at) noexcept;
    [[nodiscard]] DjiDeviceState state(Tick now) const noexcept;
    // 调用方已知“最近一次反馈样本时刻”的真实轴角时才可建立参考, 不是自动归零.
    [[nodiscard]] bool reference_position(float axis_rad, Tick now) noexcept;
    [[nodiscard]] std::optional<std::int16_t> encode_output(float normalized) const noexcept;
    void reset() noexcept;
private:
    const DjiDeviceConfig config_;
    bool configured_{};
    DjiDeviceState state_{};
    std::uint16_t previous_encoder_{};
    double position_rad_{};
    void lose_position() noexcept;
};

// 输出由同一任务的正式控制链计算; 此结构不是授权令牌或上位机消息.
struct DjiOutput { float normalized{}; Tick computed_at{}; };
enum class DjiGroupStatus { not_sent, accepted, invalid_config, missing_output, stale_output, invalid_output, busy, error };
struct DjiGroupResult {
    DjiGroupStatus status{DjiGroupStatus::not_sent};
    std::optional<transport::SendResult> tx_result{}; // 未调用传输时为空.
};

// 每个 (总线, 命令 CAN ID) 仅一个拥有者. 四槽对应组内 ID 顺序.
// nullptr 明确表示该槽未装设备; 不能用来跳过已装但失能的电机.
// 所有已装槽须提供新鲜、有限、已获许可的输出; 缺失时不发帧, 不自动补零.
// 不缓存/重试旧帧, 不替机构决定停止动作; 拒发不等于电机已停止.
class DjiTxGroup {
public:
    DjiTxGroup(transport::CanTx& bus, std::uint8_t bus_id, unsigned group,
        Lifetime output_ttl, std::array<const DjiDevice*, 4> devices) noexcept;
    DjiTxGroup(const DjiTxGroup&) = delete;
    DjiTxGroup& operator=(const DjiTxGroup&) = delete;
    [[nodiscard]] bool configured() const noexcept { return configured_; }
    [[nodiscard]] DjiGroupResult send(Tick now,
        const std::array<std::optional<DjiOutput>, 4>& outputs) noexcept;
private:
    transport::CanTx& bus_;
    unsigned group_;
    Lifetime output_ttl_;
    std::array<const DjiDevice*, 4> devices_;
    DjiMode encoding_mode_{};
    bool configured_{};
};
} // 命名空间 ec::drivers
