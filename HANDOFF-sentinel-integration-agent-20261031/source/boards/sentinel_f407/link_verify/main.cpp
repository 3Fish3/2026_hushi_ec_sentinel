// 非部署链接验证产物: 只负责在真实链接脚本与 --gc-sections 下引用通用能力的目标代码,
// 让链接器必须保留这些符号; 符号清单由 check_link_verify.py 核对.
// 这里不初始化外设、不发 CAN、不实现 APP 钩子, 也不进入调度器: 不是可烧录固件.
#include "board_assembly.hpp"
#include "ec/control/authority.hpp"
#include "ec/control/execution_context.hpp"
#include "ec/control/position_loop.hpp"
#include "ec/control/thermal.hpp"
#include "ec/control/velocity_loop.hpp"
#include "ec/drivers/bmi088_device.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/drivers/dm_device.hpp"
#include "ec/drivers/dr16.hpp"
#include "ec/estimation/attitude.hpp"
#include "ec/input/dr16_input.hpp"
#include "ec/monitor/registry.hpp"
#include "ec/motion/dji_axis.hpp"
#include "ec/motion/omni_kinematics.hpp"
#include "ec/session.hpp"
#include "ec/transport/uart_frame.hpp"
#include <array>
#include <cstdint>

namespace {
// 静态池: 链接验证不允许动态分配, 所有对象都在静态存储期.
ec::drivers::DjiDeviceConfig dji_config{};
ec::drivers::DmDeviceConfig dm_config{};
ec::monitor::Registry registry{};
ec::monitor::Descriptor descriptor{};
ec::control::ControlAuthority authority{};
ec::session::SessionTracker tracker{{500, 64}};
ec::estimation::AttitudeEstimator attitude{};
ec::control::VelocityLoop velocity{{}};
ec::control::PositionLoop position{{}};
ec::control::ThermalController thermal{{}};
ec::motion::OmniDrive omni_drive{};
ec::input::Dr16Input dr16{{200}};
ec::transport::UartFrameDecoder uart_decoder{};

// 假 sink: 把结果汇总成一个整数, 防止编译器把上面的对象优化掉.
std::uint32_t sink{};

void touch_monitor() noexcept {
    const auto slot = registry.claim(ec::monitor::RecordKind::period, 0);
    registry.begin_update();
    if (slot) {
        if (auto* record = registry.find(ec::monitor::RecordKind::period, 0)) {
            record->validity = ec::monitor::Validity::valid;
            record->sampled_at = ec::Tick{1};
            record->measurement = 1.0f;
        }
    }
    registry.end_update(ec::Tick{2});
    registry.describe(descriptor, "link-verify");
    const auto page = registry.read_page(0);
    sink += static_cast<std::uint32_t>(page.count) + registry.generation() + descriptor.capacity;
}

void touch_control() noexcept {
    const auto velocity_grant = velocity.arm(ec::CommandSource::host, true, true);
    if (velocity_grant) {
        const auto result = velocity.step(ec::Tick{10}, 0.001f,
            ec::Command<float>{1.0f, ec::Tick{10}, ec::Lifetime{20}, *velocity_grant},
            ec::Sample<float>{0.5f, ec::Tick{10}, ec::Validity::valid});
        sink += result.output ? 1u : 0u;
    }
    const auto position_grant = position.arm(ec::CommandSource::host, true, true);
    if (position_grant) {
        const auto result = position.step(ec::Tick{11}, 0.001f,
            ec::Command<float>{0.25f, ec::Tick{11}, ec::Lifetime{20}, *position_grant},
            ec::Sample<float>{0.2f, ec::Tick{11}, ec::Validity::valid},
            ec::Sample<float>{0.0f, ec::Tick{11}, ec::Validity::valid});
        sink += result.output ? 1u : 0u;
    }
    const auto decision = authority.update(ec::control::ControlAuthority::Inputs{
        true, false, true, true, true, true, 0});
    sink += decision.has_value() ? 1u : 0u;
    // 执行上下文: 每个控制周期唯一的许可与时效判定入口.
    const ec::control::ExecutionContext context{authority.gate(),
        {ec::control::StopAction::hold_position, ec::control::StopAction::disable_output}};
    const auto verdict = context.evaluate({authority.token().value_or(ec::Grant{}), true, true, true});
    sink += verdict.execute ? 1u : 0u;
    // 热保护只给出占空比与状态, 不在这里驱动任何加热或输出.
    sink += static_cast<std::uint32_t>(thermal.update(80.0f, ec::Tick{20}, ec::Tick{20}, 0.1f) * 100.0f);
    sink += static_cast<std::uint32_t>(thermal.status());
}

void touch_estimation_and_input() noexcept {
    const std::array<float, 3> gyro{0.0f, 0.0f, 0.0f};
    const std::array<float, 3> accel{0.0f, 0.0f, 9.80665f};
    const auto attitude_result = attitude.update(gyro, accel, 0.001f);
    sink += attitude_result == ec::estimation::AttitudeResult::ok ? 1u : 0u;
    sink += static_cast<std::uint32_t>(attitude.state().roll_pitch_yaw_rad[0] == 0.0f);

    ec::drivers::Dr16Sample remote{};
    sink += dr16.accept(remote, ec::Tick{1}) ? 1u : 0u;
    sink += static_cast<std::uint32_t>(dr16.online(ec::Tick{2}));
    if (const auto snapshot = dr16.snapshot(ec::Tick{2})) {
        sink += static_cast<std::uint32_t>(snapshot->switch_left == ec::drivers::Dr16Switch::up);
    }

    const std::array<std::uint8_t, 6> accel_raw{};
    const std::array<std::uint8_t, 6> gyro_raw{};
    sink += ec::drivers::parse_bmi088_accel_6g(accel_raw) ? 1u : 0u;
    sink += ec::drivers::parse_bmi088_gyro_2000(gyro_raw) ? 1u : 0u;
    sink += static_cast<std::uint32_t>(ec::drivers::parse_bmi088_temp_c(gyro_raw[0], gyro_raw[1]));

    const auto frame = uart_decoder.push(0x7e);
    sink += frame ? 1u : 0u;

    // 会话跟踪: 建立 → 判定 → 提交.
    const ec::session::SessionId host{7, 0x1234, 1};
    sink += tracker.begin(host, ec::Tick{1}) ? 1u : 0u;
    sink += tracker.heartbeat(host, ec::Tick{2}) ? 1u : 0u;
    if (tracker.check_intent(host, 1, ec::Tick{2}, ec::Lifetime{100}) ==
        ec::session::IntentResult::accepted) {
        sink += tracker.commit_intent(host, 1, ec::Tick{2}, ec::Lifetime{100}) ? 1u : 0u;
    }
    sink += tracker.link_alive(ec::Tick{3}) ? 1u : 0u;
}

void touch_devices_and_kinematics() noexcept {
    // 空配置的设备与驱动器都必须明确报无效, 而不是"看起来可用".
    ec::drivers::DjiDevice dji{dji_config};
    sink += dji.configured() ? 1u : 0u;
    sink += static_cast<std::uint32_t>(dji.receive(1, {}, ec::Tick{1}));
    sink += dji.encode_output(0.0f) ? 1u : 0u;
    ec::drivers::DmDevice dm{dm_config};
    sink += dm.configured() ? 1u : 0u;
    sink += dm.online(ec::Tick{1}) ? 1u : 0u;
    sink += dm.encode_enable() ? 1u : 0u;

    sink += static_cast<std::uint32_t>(ec::motion::omni_valid(omni_drive));
    const ec::motion::PlanarTwist twist{0.1f, 0.0f, 0.0f};
    if (const auto wheels = ec::motion::omni_wheel_speeds(omni_drive, twist)) {
        sink += static_cast<std::uint32_t>(ec::motion::omni_peak_wheel_speed(omni_drive, twist));
        sink += static_cast<std::uint32_t>((*wheels)[0] == 0.0f);
    }

    ec::motion::DjiAxis axis{{}, dji};
    const auto axis_result = axis.tick(ec::Tick{1}, 0.001f);
    sink += static_cast<std::uint32_t>(axis_result.status == ec::motion::AxisStatus::invalid_config);

    ec::sentinel::assembly_init();
    sink += static_cast<std::uint32_t>(ec::sentinel::assembly_state().status);
    if (const auto* device = ec::sentinel::dji_device(0)) {
        sink += static_cast<std::uint32_t>(device->config().device_id);
    }
    ec::sentinel::assembly_route(1, {}, ec::Tick{1});
    ec::sentinel::assembly_step(3);
}
} // 命名空间

// 弱钩子覆盖点: 链接验证不实现任何执行器输出, 因此保持空实现 (默认镜像同样如此).
extern "C" void ec_board_app_control(std::uint32_t now_ms) { (void)now_ms; }
extern "C" void ec_board_app_commit(std::uint32_t now_ms) { (void)now_ms; }

extern "C" int main() {
    touch_monitor();
    touch_control();
    touch_estimation_and_input();
    touch_devices_and_kinematics();
    return static_cast<int>(sink == 0xffffffffu); // 只为让链接器保留全部引用
}
