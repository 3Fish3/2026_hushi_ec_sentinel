// 最小单轴示例: 只做 "配置 → 使能 → 设目标 → tick → 读状态", 不引入其它层.
// 用内存 CAN 代替真实总线; accepted 只表示帧被交给发送层, 不代表电调已应用.
// 数值是演示参数, 不对应任何实车机构或安全动作.
#include "ec/drivers/dji_device.hpp"
#include "ec/motion/dji_axis.hpp"
#include <cstdint>
#include <cstdio>

namespace {
struct MemoryCan final : ec::transport::CanTx {
    ec::transport::CanFrame last{};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        last = frame;
        return ec::transport::SendResult::accepted;
    }
};

// 伪造一帧 C620 反馈: 编码器与转速都在原始域.
ec::transport::CanFrame feedback(std::uint16_t encoder, std::int16_t rpm) {
    const auto speed = static_cast<std::uint16_t>(rpm);
    return {0x201, 8, {static_cast<std::uint8_t>(encoder >> 8), static_cast<std::uint8_t>(encoder),
        static_cast<std::uint8_t>(speed >> 8), static_cast<std::uint8_t>(speed), 0, 0, 25, 0}};
}
} // 演示辅助

int main() {
    MemoryCan bus;
    ec::drivers::DjiDevice device{{1, ec::drivers::DjiMode::c620_current, 1, 1, 1, {10}, 100, 16384}};
    ec::motion::DjiAxis axis{{1.0f,
            {1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f}, // 演示 Kp=1: 误差 1 rad/s → 1 A
            {}, ec::control::ArcMode::continuous, {10}, 10.0f, 0.02f, 0.0f},
        device};
    ec::drivers::DjiTxGroup sender{bus, 1, 1, {2}, {&device, nullptr, nullptr, nullptr}};

    if (!axis.configured() || axis.enabled()) { return 1; }
    if (axis.set_velocity(1.0f, {10})) { return 1; } // 未使能时目标不被接受
    if (device.receive(1, feedback(0, 0), {10}) != ec::drivers::DjiReceiveResult::accepted) { return 1; }
    const auto ready = device.state({10}).velocity_rad_s.fresh_at({10}, {10});
    if (!axis.enable(ec::control::ControlMode::velocity, ec::CommandSource::remote, true, ready)) {
        return 1;
    }

    // 每周期: 下发目标 → 计算 → 组发送.
    if (!axis.set_velocity(1.0f, {10})) { return 2; }
    const auto result = axis.tick({10}, 0.001f);
    if (result.status != ec::motion::AxisStatus::active || !result.normalized_output) { return 2; }
    const auto sent = sender.send({10}, {ec::drivers::DjiOutput{*result.normalized_output, {10}},
        std::nullopt, std::nullopt, std::nullopt});
    if (sent.status != ec::drivers::DjiGroupStatus::accepted) { return 2; }

    // 状态回传: 目标与反馈在同一次快照里, APP 不需要访问协议槽位.
    const auto state = axis.state({10});
    if (!state.enabled || state.mode != ec::control::ControlMode::velocity || !state.output) { return 3; }
    std::printf("single axis: mode=velocity target=%.3f rad/s output=%.4f frame_id=0x%03x\n",
        static_cast<double>(state.target.value), static_cast<double>(state.output->normalized),
        static_cast<unsigned>(bus.last.id));
    return 0;
}
