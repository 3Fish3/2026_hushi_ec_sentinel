// 主机装配示例: 正式轴链 (DjiAxis + DjiTxGroup) + 内存 CAN, 不连接硬件.
// 这里演示"配置设备 → 使能 → 每周期设目标 → tick → 组发送 → 读状态"的完整通用路径.
#include "ec/drivers/dji_device.hpp"
#include "ec/motion/dji_axis.hpp"
#include <cstdint>
#include <cstdio>
#include <optional>

namespace {
// 仅记录待发帧, 不连接硬件; accepted 不代表真实电调应用成功.
struct MemoryCan final : ec::transport::CanTx {
    ec::transport::CanFrame last{};
    unsigned calls{};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        last = frame;
        ++calls;
        return ec::transport::SendResult::accepted;
    }
};

ec::transport::CanFrame feedback(std::uint16_t id, std::uint16_t encoder, std::int16_t rpm) {
    const auto speed = static_cast<std::uint16_t>(rpm);
    return {id, 8, {static_cast<std::uint8_t>(encoder >> 8), static_cast<std::uint8_t>(encoder),
        static_cast<std::uint8_t>(speed >> 8), static_cast<std::uint8_t>(speed), 0, 0, 25, 0}};
}

// 纯主机演示参数, 不对应任何实车电机或机构安全动作.
// 轴侧电流上限 1 A, C620 满量程 ±20 A, 因此归一化输出 ±1 对应 1 A.
struct Assembly {
    MemoryCan bus;
    ec::drivers::DjiDevice device{{1, ec::drivers::DjiMode::c620_current, 1, 1, 1, {10}, 100, 16384}};
    ec::drivers::DjiTxGroup sender{bus, 1, 1, {2}, {&device, nullptr, nullptr, nullptr}};
    ec::motion::DjiAxis axis{{1.0f,
            {1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f},
            {}, ec::control::ArcMode::continuous, {10}, 10.0f, 0.02f, 0.0f},
        device};
    ec::drivers::DjiGroupResult last_send{};

    ec::motion::DjiAxisResult tick(ec::Tick now, float dt_s) {
        const auto result = axis.tick(now, dt_s);
        // 仅此虚拟台架选择无输出时发零; 这是装配动作, 组发送器不会替实车作这个决定.
        const float normalized = result.normalized_output.value_or(0.0f);
        last_send = sender.send(now, {ec::drivers::DjiOutput{normalized, now}, std::nullopt, std::nullopt,
            std::nullopt});
        return result;
    }
};

std::int16_t slot_raw(const ec::transport::CanFrame& frame) {
    return static_cast<std::int16_t>(
        (static_cast<unsigned>(frame.data[0]) << 8) | frame.data[1]);
}
} // 演示组装

int main() {
    Assembly robot;
    if (!robot.axis.configured() || robot.axis.enabled()) { return 1; }
    // 未使能时不能设目标.
    if (robot.axis.set_velocity(0.5f, {100})) { return 1; }
    if (robot.device.receive(1, feedback(0x201, 0, 0), {100}) !=
        ec::drivers::DjiReceiveResult::accepted) { return 1; }
    const auto ready = robot.device.state({100}).velocity_rad_s.fresh_at({100}, {10});
    if (!robot.axis.enable(ec::control::ControlMode::velocity, ec::CommandSource::remote, true, ready)) {
        return 1;
    }
    if (!robot.axis.set_velocity(0.5f, {100})) { return 1; }
    // 误差 0.5 rad/s × Kp 1.0 = 0.5 电流上限分数 → × 1 A/20 A → 409.6 raw.
    const auto active = robot.tick({100}, 0.001f);
    if (active.status != ec::motion::AxisStatus::active || !active.normalized_output) { return 2; }
    if (robot.bus.calls != 1 || robot.bus.last.id != 0x200 || slot_raw(robot.bus.last) != 410) { return 2; }
    if (robot.last_send.status != ec::drivers::DjiGroupStatus::accepted) { return 2; }
    // 状态读取: 目标与反馈都在同一次快照里, APP 不需要访问编码槽位.
    const auto state = robot.axis.state({100});
    if (!state.enabled || state.mode != ec::control::ControlMode::velocity || !state.output) { return 2; }
    if (state.target.value != 0.5f || !state.velocity_rad_s.fresh_at({100}, {10})) { return 2; }

    // 目标有效期极短且不刷新: 到期即退出使能, 旧目标不复用.
    const auto expired = robot.tick({110}, 0.001f);
    if (expired.status != ec::motion::AxisStatus::stale_target || robot.axis.enabled()) { return 3; }
    if (expired.normalized_output || robot.bus.calls != 2) { return 3; } // 装配提交了显式零
    if (robot.axis.set_velocity(0.5f, {111})) { return 3; } // 未重新使能, 目标不被接受
    // 重新使能后必须重新下发目标, 旧目标不因链路恢复而有效.
    if (robot.device.receive(1, feedback(0x201, 0, 0), {111}) !=
        ec::drivers::DjiReceiveResult::accepted) { return 3; }
    const auto ready_again = robot.device.state({111}).velocity_rad_s.fresh_at({111}, {10});
    if (!robot.axis.enable(ec::control::ControlMode::velocity, ec::CommandSource::remote, true, ready_again)) {
        return 3;
    }
    if (robot.axis.tick({111}, 0.001f).status != ec::motion::AxisStatus::no_target) { return 4; }
    if (!robot.axis.enable(ec::control::ControlMode::velocity, ec::CommandSource::remote, true, ready_again)) {
        return 4;
    }
    if (!robot.axis.set_velocity(0.5f, {111})) { return 4; }
    if (robot.tick({111}, 0.001f).status != ec::motion::AxisStatus::active) { return 4; }
    if (robot.bus.calls != 3) { return 4; }
    std::puts("assembly: axis chain executed; expiry and re-enable require a new target");
    return 0;
}
