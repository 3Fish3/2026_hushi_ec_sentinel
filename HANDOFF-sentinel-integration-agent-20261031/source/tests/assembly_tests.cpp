// M6 装配层契约: 安装表 → 设备 → 接收路由, 以及"默认镜像没有发送路径"的静态事实.
// 这里只验证路由与布局判定, 不代表任何实车电机已装或已动作.
#include "board_assembly.hpp"
#include "board_inputs.hpp"
#include "ec/drivers/dji.hpp"
#include "ec/motion/dji_axis.hpp"
#include "ec/transport/can.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3);
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::sentinel::AssemblyStatus;
using ec::sentinel::assembly_init;
using ec::sentinel::assembly_state;

ec::drivers::DjiDeviceConfig c620(std::uint8_t bus, std::uint8_t device_id, std::uint8_t slot) {
    // slot 只用于构造合法布局: 同一总线上每条命令 ID 最多 4 个槽位.
    (void)slot;
    return {bus, ec::drivers::DjiMode::c620_current, device_id, 1.0f, 1, {20}, 100, 16384};
}

ec::transport::CanFrame feedback(std::uint16_t id, std::uint16_t encoder, std::int16_t rpm) {
    const auto speed = static_cast<std::uint16_t>(rpm);
    return {id, 8, {static_cast<std::uint8_t>(encoder >> 8), static_cast<std::uint8_t>(encoder),
        static_cast<std::uint8_t>(speed >> 8), static_cast<std::uint8_t>(speed), 0, 0, 25, 0}};
}

// 外设替身: 只替换总线, 不替换编码/轴/组发送.
struct MemoryCan final : ec::transport::CanTx {
    std::vector<ec::transport::CanFrame> log{};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        log.push_back(frame);
        return ec::transport::SendResult::accepted;
    }
    [[nodiscard]] const ec::transport::CanFrame* last(std::uint16_t id) const noexcept {
        for (auto it = log.rbegin(); it != log.rend(); ++it) {
            if (it->id == id) { return &*it; }
        }
        return nullptr;
    }
};

void empty_install_contracts() {
    assembly_init({}, {});
    CHECK(assembly_state().status == AssemblyStatus::empty);
    CHECK(assembly_state().dji_count == 0 && assembly_state().dm_count == 0);
    CHECK(assembly_state().routes == 0);
    CHECK(ec::sentinel::dji_device(0) == nullptr && ec::sentinel::dm_device(0) == nullptr);
    // 空表时任何反馈都只计入未路由, 不创建隐式设备.
    ec::sentinel::assembly_route(1, feedback(0x201, 0, 0), {100});
    CHECK(assembly_state().routed == 0 && assembly_state().unrouted == 1);
}

// R08: 装配层交出的设备必须能直接用于轴/发送路径 —— 只读指针无法构造 DjiAxis 或编码使能.
void device_ownership_contracts() {
    const std::array<ec::drivers::DjiDeviceConfig, 1> motors{c620(1, 1, 0)};
    assembly_init(motors, {});
    CHECK(assembly_state().status == AssemblyStatus::ready);
    ec::drivers::DjiDevice* device = ec::sentinel::dji_device(0);
    CHECK(device != nullptr);
    // 位置参考需要可写设备: 只读指针下无法编译. 参考必须先有新鲜反馈.
    CHECK(!device->reference_position(0.0f, {10}));
    ec::sentinel::assembly_route(1, feedback(0x201, 100, 500), {100});
    CHECK(device->reference_position(0.0f, {100}));
    CHECK(device->state({100}).position_rad.validity == ec::Validity::valid);
    // 用装配层交出的设备直接构造轴 (DjiAxis 需要 DjiDevice&): 这一行在只读指针下无法编译.
    // 默认配置为空, 轴必须明确拒绝而不是"凑合可用".
    ec::motion::DjiAxis axis{ec::motion::DjiAxisConfig{}, *device};
    CHECK(!axis.configured());
    const auto verdict = axis.tick({100}, 0.001f);
    CHECK(verdict.status == ec::motion::AxisStatus::invalid_config);
    // 装配表为空时仍是空指针, 不返回悬垂引用.
    assembly_init({}, {});
    CHECK(ec::sentinel::dji_device(0) == nullptr && ec::sentinel::dm_device(0) == nullptr);
}

// V06: 装配层不只交设备指针, 还要能跑通"接收→路由→反馈→轴→组发送"的完整路径.
// 这是 R08 剩余部分: 空配置只能证明 const 障碍消失, 不能证明有效装配链可用.
void valid_assembly_chain_contracts() {
    const std::array<ec::drivers::DjiDeviceConfig, 2> motors{c620(1, 1, 0), c620(1, 2, 1)};
    assembly_init(motors, {});
    CHECK(assembly_state().status == AssemblyStatus::ready);
    ec::drivers::DjiDevice* first = ec::sentinel::dji_device(0);
    ec::drivers::DjiDevice* second = ec::sentinel::dji_device(1);
    CHECK(first != nullptr && second != nullptr);
    // 反馈经路由进入设备 (不是直接调 device->update).
    ec::sentinel::assembly_route(1, feedback(0x201, 0, 500), {100});
    ec::sentinel::assembly_route(1, feedback(0x202, 0, 500), {100});
    CHECK(assembly_state().routed == 2);
    CHECK(first->state({100}).rotor.fresh_at({100}, {20}));
    CHECK(first->reference_position(0.0f, {100}));
    // 有效轴配置: 与示例同一套换算口径 (归一化输出, 轴侧电流上限).
    const ec::motion::DjiAxisConfig axis_config{3.0f, {1.2f, 0.05f, 0.0f, 0.0f, -1.0f, 1.0f}, {},
        ec::control::ArcMode::continuous, {20}, 8.0f, 0.02f, 0.0f};
    ec::motion::DjiAxis axis_a{axis_config, *first};
    ec::motion::DjiAxis axis_b{axis_config, *second};
    CHECK(axis_a.configured() && axis_b.configured());
    CHECK(axis_a.enable(ec::control::ControlMode::velocity, ec::CommandSource::remote, true, true));
    CHECK(axis_b.enable(ec::control::ControlMode::velocity, ec::CommandSource::remote, true, true));
    CHECK(axis_a.set_velocity(2.0f, {100}));
    CHECK(axis_b.set_velocity(2.0f, {100}));
    CHECK(axis_a.tick({100}, 0.001f).status == ec::motion::AxisStatus::active);
    CHECK(axis_b.tick({100}, 0.001f).status == ec::motion::AxisStatus::active);
    // 组发送: 真实编码帧必须出现在总线上, 而不是"配置通过"就算数.
    MemoryCan bus{};
    ec::drivers::DjiTxGroup group{bus, 1, 1, {2}, {first, second, nullptr, nullptr}};
    CHECK(group.configured());
    const auto result = group.send({100}, {axis_a.output(), axis_b.output(), std::nullopt, std::nullopt});
    CHECK(result.status == ec::drivers::DjiGroupStatus::accepted);
    const auto* frame = bus.last(0x200);
    CHECK(frame != nullptr && frame->size == 8);
    // 非零输出必须真正写进帧: 两个轴都按 2 rad/s 误差计算, 前两槽非零, 未安装槽保持 0.
    CHECK(!(frame->data[0] == 0 && frame->data[1] == 0));
    CHECK(!(frame->data[2] == 0 && frame->data[3] == 0));
    CHECK(frame->data[4] == 0 && frame->data[5] == 0 && frame->data[6] == 0 && frame->data[7] == 0);
    // 缺新鲜输出的轴: 整组拒发, 不猜补值.
    const auto refused = group.send({101}, {axis_a.output(), std::nullopt, std::nullopt, std::nullopt});
    CHECK(refused.status == ec::drivers::DjiGroupStatus::missing_output);
    assembly_init({}, {});
}

// V06: 完整 typed 输入交接 (含键鼠/按键) 与时效口径.
void typed_input_handoff_contracts() {
    ec::sentinel::input_reset();
    // 从未收到样本: 不是"零值有效样本".
    CHECK(!ec::sentinel::input_remote({100}).has_value());
    CHECK(!ec::sentinel::input_imu({100}).has_value());
    ec::drivers::Dr16Sample remote{};
    remote.ch0 = 660;
    remote.ch1 = -660;
    remote.ch2 = 123;
    remote.ch3 = -123;
    remote.switch_left = ec::drivers::Dr16Switch::up;
    remote.switch_right = ec::drivers::Dr16Switch::down;
    remote.mouse_x = -12;
    remote.mouse_y = 34;
    remote.mouse_z = 5;
    remote.mouse_left = true;
    remote.mouse_right = false;
    remote.keys = 0x0005;
    ec::sentinel::input_publish_remote(remote, {1000});
    const auto published = ec::sentinel::input_remote({1050});
    CHECK(published.has_value());
    CHECK(published->validity == ec::Validity::valid);
    CHECK(published->sampled_at.ms == 1000);
    // 完整样本: 通道/拨杆/鼠标/按键都必须保留, 不能只留四通道子集.
    CHECK(published->value.ch0 == 660 && published->value.ch3 == -123);
    CHECK(published->value.switch_left == ec::drivers::Dr16Switch::up);
    CHECK(published->value.switch_right == ec::drivers::Dr16Switch::down);
    CHECK(published->value.mouse_x == -12 && published->value.mouse_y == 34 &&
        published->value.mouse_z == 5);
    CHECK(published->value.mouse_left && !published->value.mouse_right);
    CHECK(published->value.keys == 0x0005);
    // 超出 TTL: 样本还在, 但有效性明确为 invalid (动作由调用方决定).
    const auto stale = ec::sentinel::input_remote({1200});
    CHECK(stale.has_value() && stale->validity == ec::Validity::invalid);
    CHECK(stale->sampled_at.ms == 1000);
    // 失联边沿: 明确作废后不再返回旧样本.
    ec::sentinel::input_remote_lost();
    CHECK(!ec::sentinel::input_remote({1001}).has_value());
    // IMU: 样本时刻与调用方传入的采集时刻一致, 而不是"读取那一刻".
    ec::drivers::Bmi088Sample imu{};
    imu.acc_mps2[2] = 9.81f;
    imu.gyro_rad_s[0] = 0.5f;
    imu.temp_c = 31.5f;
    ec::sentinel::input_publish_imu(imu, {2000});
    const auto imu_sample = ec::sentinel::input_imu({2010});
    CHECK(imu_sample.has_value() && imu_sample->validity == ec::Validity::valid);
    CHECK(imu_sample->sampled_at.ms == 2000);
    CHECK(imu_sample->value.acc_mps2[2] == 9.81f && imu_sample->value.temp_c == 31.5f);
    CHECK(!ec::sentinel::input_imu({2100}).has_value() ||
        ec::sentinel::input_imu({2100})->validity == ec::Validity::invalid);
    ec::sentinel::input_reset();
}

void routing_contracts() {
    // 同总线两个 C620: 设备 ID 1 → 反馈 0x201, ID 2 → 0x202.
    const std::array<ec::drivers::DjiDeviceConfig, 2> motors{c620(1, 1, 0), c620(1, 2, 1)};
    assembly_init(motors, {});
    CHECK(assembly_state().status == AssemblyStatus::ready);
    CHECK(assembly_state().dji_count == 2 && assembly_state().routes == 2);
    CHECK(ec::sentinel::dji_device(0) != nullptr && ec::sentinel::dji_device(1) != nullptr);

    ec::sentinel::assembly_route(1, feedback(0x202, 100, 500), {100});
    CHECK(assembly_state().routed == 1);
    // 只有匹配的设备被更新, 另一个保持无反馈.
    const auto second = ec::sentinel::dji_device(1)->state({100});
    const auto first = ec::sentinel::dji_device(0)->state({100});
    CHECK(second.rotor.fresh_at({100}, {20}) && second.rotor.value.rotor_speed_rad_s != 0.0f);
    CHECK(!first.rotor.fresh_at({100}, {20}));
    // 未安装的 ID 与错误总线都不路由.
    ec::sentinel::assembly_route(1, feedback(0x203, 0, 0), {100});
    ec::sentinel::assembly_route(2, feedback(0x202, 0, 0), {100});
    CHECK(assembly_state().routed == 1 && assembly_state().unrouted == 2);
}

// 供布局冲突用例使用: 一个占用 0x201 反馈的 DJI 设备.
const std::array<ec::drivers::DjiDeviceConfig, 1>& motors_with_id() {
    static const std::array<ec::drivers::DjiDeviceConfig, 1> motors{c620(1, 1, 0)};
    return motors;
}

void layout_rejection_contracts() {
    // 同总线重复槽位: DJI 布局校验必须拒绝, 且不安装任何设备.
    const std::array<ec::drivers::DjiDeviceConfig, 2> duplicate{c620(1, 1, 0), c620(1, 1, 0)};
    assembly_init(duplicate, {});
    CHECK(assembly_state().status == AssemblyStatus::invalid_dji);
    CHECK(ec::sentinel::dji_device(0) == nullptr);
    CHECK(assembly_state().routes == 0);
    // 总线号 0 非法.
    const std::array<ec::drivers::DjiDeviceConfig, 1> no_bus{c620(0, 1, 0)};
    assembly_init(no_bus, {});
    CHECK(assembly_state().status == AssemblyStatus::invalid_dji);
    // DM 布局: 同总线同 Master ID 下节点字段必须不同.
    const ec::drivers::DmMitLimits limits{12.5f, 30.0f, 10.0f};
    const std::array<ec::drivers::DmDeviceConfig, 2> dm_clash{{{1, 0x21, 0x10, limits, {20}},
        {1, 0x31, 0x10, limits, {20}}}};
    assembly_init({}, dm_clash);
    CHECK(assembly_state().status == AssemblyStatus::invalid_dm);
    // DJI 与 DM 在同一总线上的接收 ID 归属冲突.
    const std::array<ec::drivers::DmDeviceConfig, 1> dm_conflict{{{1, 0x01, 0x201, limits, {20}}}};
    assembly_init(motors_with_id(), dm_conflict);
    CHECK(assembly_state().status == AssemblyStatus::id_conflict);
}

void dm_node_routing_contracts() {
    // 两个 DM 共用 Master ID, 按反馈载荷的节点字段区分.
    const ec::drivers::DmMitLimits limits{12.5f, 30.0f, 10.0f};
    const std::array<ec::drivers::DmDeviceConfig, 2> dm{{{1, 0x01, 0x10, limits, {20}},
        {1, 0x02, 0x10, limits, {20}}}};
    assembly_init({}, dm);
    CHECK(assembly_state().status == AssemblyStatus::ready && assembly_state().routes == 2);
    // 一帧只能刷新它自己的节点; 另一个必须仍然没有反馈.
    // 节点字段 1 (状态 nibble = 1 已使能): 只刷新节点 1 的设备.
    ec::sentinel::assembly_route(1, {0x10, 8, {0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 40, 50}}, {100});
    CHECK(assembly_state().routed == 1);
    CHECK(ec::sentinel::dm_device(0)->online({100}));
    CHECK(!ec::sentinel::dm_device(1)->online({100}));
}
} // 测试辅助

int main() {
    empty_install_contracts();
    routing_contracts();
    device_ownership_contracts();
    valid_assembly_chain_contracts();
    typed_input_handoff_contracts();
    layout_rejection_contracts();
    dm_node_routing_contracts();
    std::puts("assembly routing contracts passed");
    return 0;
}
