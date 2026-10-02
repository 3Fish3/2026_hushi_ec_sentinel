// M3 达妙 MIT 契约: 独立 golden 帧、映射/范围/非有限值、共享 Master ID 的节点路由、
// 请求结果语义与混合总线核对. 不使用 DM 实物, 也不代表任何实机通过.
#include "ec/drivers/bus_layout.hpp"
#include "ec/drivers/dm.hpp"
#include "ec/drivers/dm_device.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3); // 非零退出且不触发 CRT 交互弹窗, ctest 要求非交互
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::Tick;
using ec::drivers::DmAction;
using ec::drivers::DmDevice;
using ec::drivers::DmDeviceConfig;
using ec::drivers::DmMitCommand;
using ec::drivers::DmMitLimits;
using ec::drivers::DmReceiveResult;
using ec::drivers::DmRequest;
using ec::drivers::DmRequestResult;
using ec::drivers::DmStatus;
using ec::transport::CanFrame;

bool near(float a, float b, float tolerance = 1e-6f) { return std::fabs(a - b) <= tolerance; }

constexpr DmMitLimits j4310_limits{12.5f, 30.0f, 10.0f};
constexpr std::uint16_t master = 0x10;
constexpr std::uint16_t node_a = 0x01;
constexpr std::uint16_t node_b = 0x02;

DmDeviceConfig device_config(std::uint16_t can_id, std::uint16_t master_id = master) {
    return {1, can_id, master_id, j4310_limits, {20}};
}

CanFrame feedback_frame(std::uint16_t id, std::uint8_t node, std::uint8_t status,
                        std::uint16_t position, std::uint16_t velocity, std::uint16_t torque,
                        std::uint8_t mos = 30, std::uint8_t rotor = 40) {
    CanFrame frame{id, 8, {}};
    frame.data[0] = static_cast<std::uint8_t>((node & 0x0fu) | (status << 4));
    frame.data[1] = static_cast<std::uint8_t>(position >> 8);
    frame.data[2] = static_cast<std::uint8_t>(position & 0xff);
    frame.data[3] = static_cast<std::uint8_t>(velocity >> 4);
    frame.data[4] = static_cast<std::uint8_t>(((velocity & 0x0fu) << 4) | (torque >> 8));
    frame.data[5] = static_cast<std::uint8_t>(torque & 0xff);
    frame.data[6] = mos;
    frame.data[7] = rotor;
    return frame;
}

void mapping_contracts() {
    // 无效映射范围必须显式失败, 不能拿任意默认值顶上.
    const DmMitLimits broken{0.0f, 30.0f, 10.0f};
    CHECK(!ec::drivers::dm_encode_position(broken, 0.0f));
    CHECK(!ec::drivers::dm_decode_position(broken, 0));
    CHECK(!ec::drivers::dm_encode_velocity({12.5f, -1.0f, 10.0f}, 0.0f));
    CHECK(!ec::drivers::dm_encode_torque({12.5f, 30.0f, std::numeric_limits<float>::infinity()}, 0.0f));
    CHECK(!ec::drivers::dm_decode_velocity({12.5f, std::numeric_limits<float>::quiet_NaN(), 10.0f}, 0));

    // 位置 16 bit: 端点与中点.
    CHECK(*ec::drivers::dm_encode_position(j4310_limits, -12.5f) == 0);
    CHECK(*ec::drivers::dm_encode_position(j4310_limits, 12.5f) == 65535);
    CHECK(*ec::drivers::dm_encode_position(j4310_limits, 0.0f) == 32767);
    CHECK(!ec::drivers::dm_encode_position(j4310_limits, 12.51f));
    CHECK(!ec::drivers::dm_encode_position(j4310_limits, -12.51f));
    CHECK(!ec::drivers::dm_encode_position(j4310_limits, std::numeric_limits<float>::quiet_NaN()));
    CHECK(near(*ec::drivers::dm_decode_position(j4310_limits, 0), -12.5f));
    CHECK(near(*ec::drivers::dm_decode_position(j4310_limits, 65535), 12.5f));
    // 中点不是精确 0: 65535 为奇数, 32767 对应略小于 0 的角度.
    CHECK(near(*ec::drivers::dm_decode_position(j4310_limits, 32767), -12.5f / 65535.0f, 1e-6f));

    // 速度/力矩 12 bit: 端点与单步.
    CHECK(*ec::drivers::dm_encode_velocity(j4310_limits, -30.0f) == 0);
    CHECK(*ec::drivers::dm_encode_velocity(j4310_limits, 30.0f) == 4095);
    CHECK(*ec::drivers::dm_encode_velocity(j4310_limits, 0.0f) == 2047);
    CHECK(!ec::drivers::dm_encode_velocity(j4310_limits, 30.01f));
    CHECK(near(*ec::drivers::dm_decode_velocity(j4310_limits, 0), -30.0f));
    CHECK(near(*ec::drivers::dm_decode_velocity(j4310_limits, 4095), 30.0f));
    // 单量化步: 一个 LSB = 60/4095; 截断取整使 0 附近上下不对称.
    constexpr float velocity_step = 60.0f / 4095.0f;
    CHECK(*ec::drivers::dm_encode_velocity(j4310_limits, velocity_step) == 2048);
    CHECK(*ec::drivers::dm_encode_velocity(j4310_limits, -velocity_step) == 2046);
    CHECK(*ec::drivers::dm_encode_torque(j4310_limits, -10.0f) == 0);
    CHECK(*ec::drivers::dm_encode_torque(j4310_limits, 10.0f) == 4095);
    CHECK(*ec::drivers::dm_encode_torque(j4310_limits, 0.0f) == 2047);
    CHECK(!ec::drivers::dm_encode_torque(j4310_limits, -10.01f));
    CHECK(near(*ec::drivers::dm_decode_torque(j4310_limits, 4095), 10.0f));

    // Kp/Kd 的协议范围与 PMAX/VMAX/TMAX 无关, 是手册固定值.
    CHECK(*ec::drivers::dm_encode_kp(0.0f) == 0);
    CHECK(*ec::drivers::dm_encode_kp(500.0f) == 4095);
    CHECK(!ec::drivers::dm_encode_kp(500.1f));
    CHECK(!ec::drivers::dm_encode_kp(-0.001f));
    CHECK(!ec::drivers::dm_encode_kp(std::numeric_limits<float>::infinity()));
    CHECK(*ec::drivers::dm_encode_kd(0.0f) == 0);
    CHECK(*ec::drivers::dm_encode_kd(5.0f) == 4095);
    CHECK(!ec::drivers::dm_encode_kd(5.1f));
    CHECK(near(*ec::drivers::dm_decode_kp(0), 0.0f));
    CHECK(near(*ec::drivers::dm_decode_kp(4095), 500.0f));
    CHECK(near(*ec::drivers::dm_decode_kd(4095), 5.0f));
    // 旧代码用 raw 35 作位置增益: 物理值是 35*500/4095 ≈ 4.2735, 两者必须能互相换回.
    const float legacy_kp = 35.0f * 500.0f / 4095.0f;
    CHECK(near(*ec::drivers::dm_decode_kp(35), legacy_kp, 1e-6f));
    CHECK(near(legacy_kp, 4.2735043f, 1e-6f));
    // 编码不四舍五入而是截断: float 化后的 4.2735043 回编落在 34, 不是 35.
    CHECK(*ec::drivers::dm_encode_kp(legacy_kp) == 34);
    // 物理值与 raw 是两件事: 物理 35 对应 raw 286, 不是 raw 35.
    CHECK(*ec::drivers::dm_encode_kp(35.0f) == 286); // 35*4095/500 = 286.65
}

void golden_frame_contracts() {
    // 零点: 位置 32767, 速度/力矩 2047, 增益 0.
    const DmMitCommand zero{};
    const auto midpoint = ec::drivers::encode_dm_mit(node_a, j4310_limits, zero);
    CHECK(midpoint && midpoint->id == node_a && midpoint->size == 8);
    const std::array<std::uint8_t, 8> expected_midpoint{0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x07, 0xff};
    CHECK(midpoint->data == expected_midpoint);
    // 正向端点全 0xFF, 负向端点全 0x00.
    const auto upper = ec::drivers::encode_dm_mit(node_a, j4310_limits,
        DmMitCommand{12.5f, 30.0f, 500.0f, 5.0f, 10.0f});
    CHECK(upper && upper->data == std::array<std::uint8_t, 8>{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff});
    const auto lower = ec::drivers::encode_dm_mit(node_a, j4310_limits,
        DmMitCommand{-12.5f, -30.0f, 0.0f, 0.0f, -10.0f});
    CHECK(lower && lower->data == std::array<std::uint8_t, 8>{});
    // 越界与非有限值不编码也不钳位.
    CHECK(!ec::drivers::encode_dm_mit(node_a, j4310_limits, DmMitCommand{12.51f, 0.0f, 0.0f, 0.0f, 0.0f}));
    CHECK(!ec::drivers::encode_dm_mit(node_a, j4310_limits,
        DmMitCommand{0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}));
    CHECK(!ec::drivers::encode_dm_mit(node_a, j4310_limits,
        DmMitCommand{0.0f, 0.0f, 0.0f, 0.0f, std::numeric_limits<float>::infinity()}));
    CHECK(!ec::drivers::encode_dm_mit(0x800, j4310_limits, zero));

    // 独立 golden 帧: 非自编自解, 直接给出字节并核对解码结果.
    // 位置 raw 0x8000, 速度 raw 0x123, 力矩 raw 0x456; 状态 Enabled, 节点 0x0A.
    const auto frame = feedback_frame(master, 0x0a, 0x1, 0x8000, 0x123, 0x456, 45, 62);
    const auto decoded = ec::drivers::decode_dm_feedback(frame, j4310_limits);
    CHECK(decoded);
    CHECK(decoded->node_id == 0x0a);
    CHECK(decoded->status == DmStatus::enabled);
    CHECK(near(decoded->position_rad, 0.00019073486f, 1e-6f));
    CHECK(near(decoded->velocity_rad_s, -25.7362637f, 1e-4f));
    CHECK(near(decoded->torque_nm, -4.57875458f, 1e-4f));
    CHECK(decoded->mos_temperature_c == 45 && decoded->rotor_temperature_c == 62);
    // DLC 必须正好 8; 少于/多于都是无效帧.
    auto short_frame = frame;
    short_frame.size = 7;
    CHECK(!ec::drivers::decode_dm_feedback(short_frame, j4310_limits));
    short_frame.size = 9;
    CHECK(!short_frame.valid());
    CHECK(!ec::drivers::decode_dm_feedback(short_frame, j4310_limits));

    // 状态半字节: 0 掉电默认 Disabled, 1 Enabled, 8..E 故障, 其余未定义.
    for (std::uint8_t nibble = 0; nibble <= 0xf; ++nibble) {
        const auto status = ec::drivers::dm_status(nibble);
        if (nibble == 0) { CHECK(status == DmStatus::disabled); }
        else if (nibble == 1) { CHECK(status == DmStatus::enabled); }
        else if (nibble >= 8 && nibble <= 0xe) {
            CHECK(status == static_cast<DmStatus>(nibble));
            CHECK(ec::drivers::dm_status_fault(status));
            CHECK(!ec::drivers::dm_status_enabled(status));
        } else {
            CHECK(status == DmStatus::unknown);
            CHECK(!ec::drivers::dm_status_fault(status));
        }
    }
    CHECK(ec::drivers::dm_status_valid(0x0) && ec::drivers::dm_status_valid(0x1));
    CHECK(ec::drivers::dm_status_valid(0xe) && !ec::drivers::dm_status_valid(0xf));

    // 使能/失能是固定载荷: 7 字节 0xFF 加最后一个命令字节; 与 MIT 满量程帧不混淆.
    const auto enable = ec::drivers::encode_dm_action(node_a, DmAction::enable);
    CHECK(enable && enable->id == node_a);
    CHECK(enable->data == std::array<std::uint8_t, 8>{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc});
    const auto disable = ec::drivers::encode_dm_action(node_b, DmAction::disable);
    CHECK(disable->data == std::array<std::uint8_t, 8>{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd});
    CHECK(static_cast<std::uint8_t>(DmAction::enable) == 0xfc);
    CHECK(static_cast<std::uint8_t>(DmAction::disable) == 0xfd);
}

// R06: Tick 是 uint32 毫秒且会回绕; "时间倒退"必须用无符号差值判定.
void time_wrap_contracts() {
    // 首次接收直接落在回绕附近: 同一会话内不可能从 0 跳到回绕附近, 那是半回绕歧义区.
    DmDevice device{device_config(node_a)};
    CHECK(device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {0xFFFFFFFEu}) ==
        DmReceiveResult::accepted);
    CHECK(device.online({0xFFFFFFFEu}));
    // 正常前进 3 ms 的回绕不得被判为倒退, 且必须刷新时效.
    CHECK(device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {0x00000001u}) ==
        DmReceiveResult::accepted);
    CHECK(device.state().feedback.sampled_at.ms == 0x00000001u);
    CHECK(device.online({0x00000001u}));
    // 真正的倒退仍必须拒绝 (同一半回绕区间内).
    CHECK(device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {0x00000001u - 5u}) ==
        DmReceiveResult::invalid_time);
    CHECK(device.state().feedback.sampled_at.ms == 0x00000001u);
    // 回绕之后继续正常接收.
    CHECK(device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {10}) ==
        DmReceiveResult::accepted);
    CHECK(device.online({10}));
}

void device_contracts() {
    DmDevice device{device_config(node_a)};
    CHECK(device.configured());
    CHECK(!device.online({0}) && !device.enabled({0}) && !device.faulted({0}));
    CHECK(device.request_result({0}) == DmRequestResult::unknown);
    // 反馈 ID 与配置的 Master ID 不一致时忽略, 也不刷新时效.
    CHECK(device.receive(1, feedback_frame(0x11, 0x01, 0x01, 0, 0, 0), {100}) == DmReceiveResult::ignored);
    CHECK(device.receive(2, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {100}) == DmReceiveResult::ignored);
    CHECK(!device.online({100}));
    // DLC 错误与时间倒退.
    auto bad = feedback_frame(master, 0x01, 0x01, 0, 0, 0);
    bad.size = 7;
    CHECK(device.receive(1, bad, {100}) == DmReceiveResult::invalid_frame);
    CHECK(device.receive(1, feedback_frame(master, 0x01, 0x00, 0, 0, 0), {100}) == DmReceiveResult::accepted);
    CHECK(device.online({100}) && !device.enabled({100}));
    CHECK(device.state().feedback.value.status == DmStatus::disabled);
    CHECK(device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {99}) == DmReceiveResult::invalid_time);
    CHECK(device.state().feedback.sampled_at.ms == 100);
    // 时效: 严格小于 TTL.
    CHECK(device.online({119}));
    CHECK(!device.online({120}));
    CHECK(device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {120}) ==
        DmReceiveResult::accepted);
    CHECK(device.online({120}));
    CHECK(device.enabled({120}));

    // 请求结果由反馈状态确认, 不由“编码成功”推断.
    DmDevice request_device{device_config(node_a)};
    const auto enable = request_device.encode_enable();
    CHECK(enable && request_device.state().pending_request == DmRequest::enable);
    CHECK(request_device.request_result({0}) == DmRequestResult::unknown); // 还没有反馈
    CHECK(request_device.receive(1, feedback_frame(master, 0x01, 0x00, 0, 0, 0), {100}) ==
        DmReceiveResult::accepted);
    CHECK(request_device.request_result({100}) == DmRequestResult::pending);
    CHECK(!request_device.enabled({100}));
    CHECK(request_device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {110}) ==
        DmReceiveResult::accepted);
    CHECK(request_device.request_result({110}) == DmRequestResult::accepted);
    CHECK(request_device.enabled({110}));
    CHECK(request_device.state().enable_confirmed);
    // 故障状态优先于请求方向; 不自动清错, 也不改变 pending 记录.
    CHECK(request_device.receive(1, feedback_frame(master, 0x01, 0x0b, 0, 0, 0), {120}) ==
        DmReceiveResult::accepted);
    CHECK(request_device.request_result({120}) == DmRequestResult::faulted);
    CHECK(request_device.faulted({120}) && !request_device.enabled({120}));
    CHECK(request_device.request_result({200}) == DmRequestResult::unknown); // 反馈过期后不给结论
    const auto disable = request_device.encode_disable();
    CHECK(disable && request_device.state().pending_request == DmRequest::disable);
    CHECK(request_device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {210}) ==
        DmReceiveResult::accepted);
    CHECK(request_device.request_result({210}) == DmRequestResult::pending);
    CHECK(request_device.receive(1, feedback_frame(master, 0x01, 0x00, 0, 0, 0), {220}) ==
        DmReceiveResult::accepted);
    CHECK(request_device.request_result({220}) == DmRequestResult::accepted);
    // 曾经使能过的事实保留, 便于上层识别“已使能过”的上下文.
    CHECK(request_device.state().enable_confirmed);
    request_device.reset();
    CHECK(!request_device.state().enable_confirmed);
    CHECK(request_device.request_result({220}) == DmRequestResult::unknown);
    CHECK(request_device.state().pending_request == DmRequest::none);

    // DM 设备不持有 CanTx: 结构上不存在自动使能/清错/写零位帧; 编码本身也不改变反馈状态.
    DmDevice config_device{device_config(node_a)};
    CHECK(config_device.encode_mit(DmMitCommand{0.0f, 0.0f, 10.0f, 0.5f, 0.0f}));
    CHECK(config_device.encode_enable());
    CHECK(config_device.state().feedback.validity == ec::Validity::unknown);
    CHECK(config_device.state().pending_request == DmRequest::enable);
    // 无效配置下不产生任何帧.
    DmDevice broken{{1, 0x800, master, j4310_limits, {20}}};
    CHECK(!broken.configured());
    CHECK(!broken.encode_enable() && !broken.encode_disable() && !broken.encode_mit({}));
    CHECK(broken.receive(1, feedback_frame(master, 0, 1, 0, 0, 0), {0}) == DmReceiveResult::invalid_config);
}

void shared_master_routing_contracts() {
    // 两个 DM 共用反馈 Master ID, 按载荷节点字段路由.
    DmDevice first{device_config(node_a)};
    DmDevice second{device_config(node_b)};
    CHECK(first.configured() && second.configured());
    CHECK(first.receive(1, feedback_frame(master, 0x02, 0x01, 0x1000, 0x100, 0x100), {100}) ==
        DmReceiveResult::invalid_node);
    CHECK(!first.online({100})); // 另一节点的帧不刷新本设备时效
    CHECK(second.receive(1, feedback_frame(master, 0x02, 0x01, 0x1000, 0x100, 0x100), {100}) ==
        DmReceiveResult::accepted);
    CHECK(second.online({100}) && second.enabled({100}));
    CHECK(!first.online({100}));
    CHECK(first.receive(1, feedback_frame(master, 0x01, 0x01, 0x2000, 0x100, 0x100), {100}) ==
        DmReceiveResult::accepted);
    CHECK(first.online({100}));
    // 两个设备的解码结果各自独立.
    CHECK(near(first.state().feedback.value.position_rad,
               *ec::drivers::dm_decode_position(j4310_limits, 0x2000), 1e-6f));
    CHECK(near(second.state().feedback.value.position_rad,
               *ec::drivers::dm_decode_position(j4310_limits, 0x1000), 1e-6f));

    // 布局: 命令 ID 必须唯一; 共用 Master ID 时节点低 4 位必须唯一.
    const std::array<DmDeviceConfig, 2> shared_master{{device_config(0x01), device_config(0x02)}};
    CHECK(ec::drivers::valid_dm_layout(shared_master));
    // 0x21 与 0x31 的低 4 位相同, 共用同一 Master ID 时无法区分.
    const std::array<DmDeviceConfig, 2> clashing_master{{device_config(0x21), device_config(0x31)}};
    CHECK(!ec::drivers::valid_dm_layout(clashing_master));
    // 不同 Master ID 时低 4 位可以相同.
    const std::array<DmDeviceConfig, 2> split_master{
        {device_config(0x21, 0x10), device_config(0x31, 0x11)}};
    CHECK(ec::drivers::valid_dm_layout(split_master));
    // 相同命令 ID 直接冲突.
    const std::array<DmDeviceConfig, 2> duplicate_command{
        {device_config(0x03, 0x10), device_config(0x03, 0x11)}};
    CHECK(!ec::drivers::valid_dm_layout(duplicate_command));
    const std::array<DmDeviceConfig, 2> different_bus{
        {device_config(0x03, 0x10), {2, 0x03, 0x10, j4310_limits, {20}}}};
    CHECK(ec::drivers::valid_dm_layout(different_bus));

    // 无效配置: 空布局、bus 0、ID 越界、映射范围非法、TTL 缺失.
    CHECK(!ec::drivers::valid_dm_layout(std::span<const DmDeviceConfig>{}));
    const std::array<DmDeviceConfig, 1> no_bus{{{0, 0x01, master, j4310_limits, {20}}}};
    CHECK(!ec::drivers::valid_dm_layout(no_bus));
    const std::array<DmDeviceConfig, 1> wide_id{{{1, 0x801, master, j4310_limits, {20}}}};
    CHECK(!ec::drivers::valid_dm_layout(wide_id));
    const std::array<DmDeviceConfig, 1> wide_master{{{1, 0x01, 0x801, j4310_limits, {20}}}};
    CHECK(!ec::drivers::valid_dm_layout(wide_master));
    const std::array<DmDeviceConfig, 1> no_limits{{{1, 0x01, master, {}, {20}}}};
    CHECK(!ec::drivers::valid_dm_layout(no_limits));
    const std::array<DmDeviceConfig, 1> no_ttl{{{1, 0x01, master, j4310_limits, {0}}}};
    CHECK(!ec::drivers::valid_dm_layout(no_ttl));
}

// R07: 同一总线上一个 11 位 ID 只能有一种含义. 跨设备的命令/反馈交叉冲突必须被拒绝,
// 而“多台电机共享一个反馈 ID, 按载荷节点区分”仍必须允许.
void dm_layout_collision_contracts() {
    using ec::drivers::valid_dm_layout;
    // A 的反馈 ID 恰好是 B 的命令 ID (反之亦然): 通过校验会把一方的反馈当成另一方的命令.
    const std::array<DmDeviceConfig, 2> cross{{device_config(0x01, 0x02), device_config(0x02, 0x03)}};
    CHECK(!valid_dm_layout(cross));
    const std::array<DmDeviceConfig, 2> cross_swapped{{device_config(0x02, 0x03), device_config(0x01, 0x02)}};
    CHECK(!valid_dm_layout(cross_swapped));
    // 自己的反馈 ID 等于自己的命令 ID 同样不允许.
    const std::array<DmDeviceConfig, 1> self_collision{{device_config(0x05, 0x05)}};
    CHECK(!valid_dm_layout(self_collision));
    // 共享 Master ID 且载荷节点不同: 受支持.
    const std::array<DmDeviceConfig, 2> shared{{device_config(0x01, master), device_config(0x02, master)}};
    CHECK(valid_dm_layout(shared));
    // 不同总线上的相同 ID 互不影响.
    const std::array<DmDeviceConfig, 2> other_bus{{device_config(0x01, 0x02), {2, 0x02, 0x03, j4310_limits, {20}}}};
    CHECK(valid_dm_layout(other_bus));
    // 互不重叠的常规布局仍然通过.
    const std::array<DmDeviceConfig, 2> disjoint{{device_config(0x01, 0x21), device_config(0x02, 0x22)}};
    CHECK(valid_dm_layout(disjoint));
}

void mixed_bus_contracts() {
    using ec::drivers::DjiDeviceConfig;
    using ec::drivers::DjiMode;
    const std::array<DmDeviceConfig, 1> dm{{{1, 0x01, master, j4310_limits, {20}}}};
    // C620 反馈 0x201-0x208, 命令 0x200/0x1FF.
    const std::array<DjiDeviceConfig, 1> c620{{{1, DjiMode::c620_current, 1, 1, 1, {20}, 100, 16384}}};
    CHECK(ec::drivers::valid_mixed_bus(dm, c620));
    // GM6020 ID1 反馈 0x205; DM 反馈 ID 与之重叠必须拒绝.
    const std::array<DjiDeviceConfig, 1> gm6020{{{1, DjiMode::gm6020_current, 1, 1, 1, {20}, 100, 16384}}};
    const std::array<DmDeviceConfig, 1> clashing_master{{{1, 0x01, 0x205, j4310_limits, {20}}}};
    CHECK(!ec::drivers::valid_mixed_bus(clashing_master, gm6020));
    CHECK(!ec::drivers::valid_mixed_bus(clashing_master, c620)); // C620 组 1 反馈覆盖 0x201-0x208
    // 两种协议反馈区间之外才可用.
    const std::array<DmDeviceConfig, 1> free_master{{{1, 0x01, 0x210, j4310_limits, {20}}}};
    CHECK(ec::drivers::valid_mixed_bus(free_master, c620));
    CHECK(ec::drivers::valid_mixed_bus(free_master, gm6020)); // GM6020 反馈覆盖 0x205-0x20B
    // DM 反馈 ID 等于 DJI 命令 ID 也不允许.
    const std::array<DmDeviceConfig, 1> command_id_master{{{1, 0x01, 0x200, j4310_limits, {20}}}};
    CHECK(!ec::drivers::valid_mixed_bus(command_id_master, c620));
    // DM 命令 ID 落在 DJI 反馈区间或等于 DJI 命令 ID 同样冲突.
    const std::array<DmDeviceConfig, 1> feedback_range_command{{{1, 0x205, master, j4310_limits, {20}}}};
    CHECK(!ec::drivers::valid_mixed_bus(feedback_range_command, gm6020));
    const std::array<DmDeviceConfig, 1> command_clash{{{1, 0x1ff, master, j4310_limits, {20}}}};
    CHECK(!ec::drivers::valid_mixed_bus(command_clash, c620));
    // 不同总线互不影响; 两侧都为空返回失败.
    const std::array<DmDeviceConfig, 1> other_bus{{{2, 0x01, 0x205, j4310_limits, {20}}}};
    CHECK(ec::drivers::valid_mixed_bus(other_bus, gm6020));
    CHECK(!ec::drivers::valid_mixed_bus(std::span<const DmDeviceConfig>{},
        std::span<const DjiDeviceConfig>{}));
    // 本函数只核对混合 ID 归属; DM 自身合法性由 valid_dm_layout 负责.
    const std::array<DmDeviceConfig, 1> broken_limits{{{1, 0x01, master, {0.0f, 1.0f, 1.0f}, {20}}}};
    CHECK(ec::drivers::valid_mixed_bus(broken_limits, c620));
    CHECK(!ec::drivers::valid_dm_layout(broken_limits));
}
void send_path_contracts() {
    // DM 没有组发送器: 装配把编码结果直接交给 CanTx, 发送结果由调用方处理.
    struct FakeCan final : ec::transport::CanTx {
        ec::transport::SendResult result{ec::transport::SendResult::accepted};
        CanFrame last{};
        unsigned calls{};
        ec::transport::SendResult try_send(const CanFrame& frame) noexcept override {
            last = frame;
            ++calls;
            return result;
        }
    } bus;
    DmDevice device{device_config(node_a)};
    const auto enable = device.encode_enable();
    CHECK(enable && bus.try_send(*enable) == ec::transport::SendResult::accepted);
    CHECK(bus.calls == 1 && bus.last.id == node_a && bus.last.data[7] == 0xfc);
    // 编码成功不代表发送成功: busy/error 必须能区分并回传.
    bus.result = ec::transport::SendResult::busy;
    const auto mit = device.encode_mit(DmMitCommand{0.0f, 0.0f, 10.0f, 0.5f, 0.0f});
    CHECK(mit && bus.try_send(*mit) == ec::transport::SendResult::busy);
    bus.result = ec::transport::SendResult::error;
    CHECK(bus.try_send(*mit) == ec::transport::SendResult::error);
    CHECK(bus.calls == 3);
    // 失联: 没有新鲜反馈就不能当作在线.
    CHECK(!device.online({1000}));
    CHECK(device.receive(1, feedback_frame(master, 0x01, 0x01, 0, 0, 0), {1000}) ==
        DmReceiveResult::accepted);
    CHECK(device.online({1019}) && !device.online({1020}));
}
} // 测试辅助

int main() {
    mapping_contracts();
    golden_frame_contracts();
    device_contracts();
    time_wrap_contracts();
    send_path_contracts();
    shared_master_routing_contracts();
    dm_layout_collision_contracts();
    mixed_bus_contracts();
    std::puts("dm contracts passed");
    return 0;
}
