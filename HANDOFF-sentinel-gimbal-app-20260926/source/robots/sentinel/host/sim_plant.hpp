#pragma once
// 主机侧合成负载与软件总线 (O7): 只用于离线闭环与操作端联调.
//
// 边界说明 (不要误用):
//  - 这不是实车动力学/摩擦/传动模型, 也不是电调或达妙固件的行为模型; 它只提供
//    "命令帧 → 运动 → 反馈帧" 的最小闭环, 用来验证方向/单位/生命周期与编解码往返;
//  - 因此不得用它推导实车稳定性、动力参数或机械能力, 这些仍属实机验收项;
//  - 只在主机上编译 (使用标准容器), 固件不包含本文件.
#include "ec/drivers/dji.hpp"
#include "ec/drivers/dm.hpp"
#include "ec/transport/can.hpp"
#include "sentinel_config.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace ec::sentinel::host {
using app::DjiSlot;
using app::DmSlot;
using app::SentinelConfig;
using app::dji_slot_count;
// 记录原始帧的软件总线. 不模拟总线时序/仲裁/错误帧, 但可以拒绝发送以验证背压路径:
// 拒绝时返回 busy, 调用方必须把它当作"没发出去"而不是"已到达电调".
struct SoftwareBus : transport::CanTx {
    std::vector<transport::CanFrame> frames{};
    std::size_t sent{};
    std::size_t refused{};
    std::size_t record_capacity{64}; // 记录有界, 避免长时间运行无界增长
    bool refuse{};

    transport::SendResult try_send(const transport::CanFrame& frame) noexcept override {
        if (refuse) {
            ++refused;
            return transport::SendResult::busy;
        }
        ++sent;
        if (frames.size() < record_capacity) { frames.push_back(frame); }
        return transport::SendResult::accepted;
    }
    void clear() noexcept { frames.clear(); }
    [[nodiscard]] std::size_t size() const noexcept { return frames.size(); }
    [[nodiscard]] std::optional<transport::CanFrame> at(std::size_t index) const noexcept {
        if (index >= frames.size()) { return std::nullopt; }
        return frames[index];
    }
    [[nodiscard]] std::optional<transport::CanFrame> last() const noexcept {
        if (frames.empty()) { return std::nullopt; }
        return frames.back();
    }
    // 最近一帧指定命令 ID 的帧 (设备命令按 ID 分组, 帧内槽位顺序固定).
    [[nodiscard]] std::optional<transport::CanFrame> last_with_id(std::uint16_t id) const noexcept {
        for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
            if (it->id == id) { return *it; }
        }
        return std::nullopt;
    }
};

// 原始命令帧 → 组内槽位原始量. 这是 encode_dji 的逆运算, 独立写在主机侧:
// 闭环证据必须包含"电调实际会收到的字节", 而不是只核对控制层内部数值.
[[nodiscard]] inline std::optional<std::int16_t> raw_slot(
    const transport::CanFrame& frame, unsigned slot) noexcept {
    if (slot >= 4 || frame.size != 8) { return std::nullopt; }
    const auto bits = static_cast<std::uint16_t>(
        (static_cast<unsigned>(frame.data[2 * slot]) << 8) | frame.data[2 * slot + 1]);
    const auto value = bits < 0x8000 ? static_cast<int>(bits) : static_cast<int>(bits) - 65536;
    return static_cast<std::int16_t>(value);
}

// 达妙 MIT 命令帧解码 (encode_dm_mit 的逆): 位置 12 bit, 速度/Kp/Kd/力矩各 12 bit.
// 只用于主机侧闭环与核对, 固件不需要解码自己发出的命令.
struct DmMitDecoded {
    float position_rad{};
    float velocity_rad_s{};
    float kp{};
    float kd{};
    float torque_nm{};
};
[[nodiscard]] inline std::optional<DmMitDecoded> decode_dm_mit_command(
    const transport::CanFrame& frame, const drivers::DmMitLimits& limits) noexcept {
    if (frame.size != 8) { return std::nullopt; }
    const auto position = static_cast<std::uint16_t>(
        (static_cast<unsigned>(frame.data[0]) << 8) | frame.data[1]);
    const auto velocity = static_cast<std::uint16_t>(
        (static_cast<unsigned>(frame.data[2]) << 4) | (frame.data[3] >> 4));
    const auto kp = static_cast<std::uint16_t>(
        ((static_cast<unsigned>(frame.data[3]) & 0x0fu) << 8) | frame.data[4]);
    const auto kd = static_cast<std::uint16_t>(
        (static_cast<unsigned>(frame.data[5]) << 4) | (frame.data[6] >> 4));
    const auto torque = static_cast<std::uint16_t>(
        ((static_cast<unsigned>(frame.data[6]) & 0x0fu) << 8) | frame.data[7]);
    const auto decoded_position = drivers::dm_decode_position(limits, position);
    const auto decoded_velocity = drivers::dm_decode_velocity(limits, velocity);
    const auto decoded_torque = drivers::dm_decode_torque(limits, torque);
    if (!decoded_position || !decoded_velocity || !decoded_torque) { return std::nullopt; }
    DmMitDecoded out{};
    out.position_rad = *decoded_position;
    out.velocity_rad_s = *decoded_velocity;
    out.kp = drivers::dm_decode_kp(kp).value_or(0.0f);
    out.kd = drivers::dm_decode_kd(kd).value_or(0.0f);
    out.torque_nm = *decoded_torque;
    return out;
}

// 合成负载: 每个已装配 DJI 槽位是一阶速度系统 (轴侧电流 → 加速度, 粘性阻尼);
// 达妙按命令自带的 MIT 律 (kp/kd) 做二阶位置响应, 负载惯性/阻尼来自夹具.
// 参数是夹具值, 不对应任何真实机构.
struct SimPlant {
    struct Axis {
        float position_rad{};
        float velocity_rad_s{};
        bool emitting{true}; // false = 该设备停止上报 (模拟掉线/未上电)
    };
    struct Load {
        float accel_per_amp{20.0f}; // rad/s² per A
        float damping_per_s{8.0f};  // 1/s
        float dm_inertia{0.05f};    // kg·m²
        float dm_damping{0.05f};    // N·m·s/rad
        float dm_feedback_torque_nm{};
    };

    std::array<Axis, dji_slot_count> dji{};
    Axis dm{};
    Load load{};
    // 外部扰动入口: 夹具外力 (rad/s² 等效加速度) 与达妙外力矩, 用于验证位置环抵抗
    // 扰动而不是"只跟随自己发出的命令".
    std::array<float, dji_slot_count> external_accel{};
    float dm_external_torque_nm{};
    // 达妙反馈状态位控制: 可让负载按"未使能/故障"上报, 用于验证执行层不把
    // "已发使能帧"当成已使能, 也不把故障当成正常.
    bool dm_reports_enabled{true};
    bool dm_reports_fault{false};

    // 消费一个周期内发出的原始帧并推进运动 (只按设备自己的换算折回轴侧电流).
    void integrate(const SentinelConfig& config, const SoftwareBus& bus1, const SoftwareBus& bus2,
        float dt_s) noexcept {
        for (std::size_t slot = 0; slot < dji_slot_count; ++slot) {
            const auto& entry = config.dji[slot];
            if (!entry.installed) { continue; }
            const auto& bus = entry.bus == 1 ? bus1 : bus2;
            const auto protocol = drivers::dji_protocol(entry.mode);
            float amps = 0.0f;
            if (protocol && entry.output_limit_raw && entry.current_limit_a &&
                *entry.output_limit_raw != 0) {
                const auto group_id = entry.device_id > 4 ? protocol->second_tx : protocol->first_tx;
                const auto command = bus.last_with_id(group_id);
                if (command) {
                    const auto raw = raw_slot(*command, static_cast<unsigned>((entry.device_id - 1) % 4));
                    if (raw) {
                        // 设备层把方向同时作用在反馈与输出上 (encode_output 乘 direction),
                        // 因此原始量折回轴侧电流时必须再乘一次方向.
                        // ESC 按协议满量程解释字节, 不是按软件轴电流 cap 再缩放一次.
                        const auto full = drivers::dji_current_amp_full_scale(entry.mode);
                        if (full) {
                            amps = static_cast<float>(*raw) / static_cast<float>(protocol->limit) *
                                *full * static_cast<float>(*entry.direction);
                        }
                    }
                }
            }
            auto& axis = dji[slot];
            const float accel = load.accel_per_amp * amps - load.damping_per_s * axis.velocity_rad_s +
                external_accel[slot];
            axis.velocity_rad_s += accel * dt_s;
            axis.position_rad += axis.velocity_rad_s * dt_s;
        }
        const auto& slot = config.dm[0];
        if (slot.installed && slot.limits && slot.can_id && slot.master_id) {
            const auto& bus = slot.bus == 1 ? bus1 : bus2;
            const auto command = last_dm_command(slot, bus);
            float accel = 0.0f;
            const auto decoded = command ? decode_dm_mit_command(*command, *slot.limits) : std::nullopt;
            if (decoded) {
                const float error = decoded->position_rad - dm.position_rad;
                const float damping = decoded->velocity_rad_s - dm.velocity_rad_s;
                accel = (decoded->kp * error + decoded->kd * damping + decoded->torque_nm -
                            load.dm_damping * dm.velocity_rad_s + dm_external_torque_nm) /
                    load.dm_inertia;
            } else {
                // 没有命令时不产生保持力矩: 只有阻尼 (不假装电机仍在保持).
                accel = -load.dm_damping * dm.velocity_rad_s / load.dm_inertia;
            }
            dm.velocity_rad_s += accel * dt_s;
            dm.position_rad += dm.velocity_rad_s * dt_s;
        }
    }

    // 生成一个周期的反馈帧并投递: 编码为真实协议字节 (DJI 单圈编码器 + 达妙 MIT 反馈).
    template<typename Route>
    void emit_feedback(const SentinelConfig& config, Route&& route, Tick now) const {
        constexpr float tau = 6.28318530717958647692f;
        for (std::size_t slot = 0; slot < dji_slot_count; ++slot) {
            const auto& entry = config.dji[slot];
            if (!entry.installed || !dji[slot].emitting) { continue; }
            const auto protocol = drivers::dji_protocol(entry.mode);
            if (!protocol || !entry.rotor_per_axis || !entry.direction) { continue; }
            // 轴侧 → 转子侧: 方向与传动比与设备配置一致, 不在这里另设换算.
            const auto& axis = dji[slot];
            const float rotor_angle = axis.position_rad * *entry.rotor_per_axis /
                static_cast<float>(*entry.direction);
            const float rotor_speed = axis.velocity_rad_s * *entry.rotor_per_axis /
                static_cast<float>(*entry.direction);
            float wrapped = std::fmod(rotor_angle, tau);
            if (wrapped < 0.0f) { wrapped += tau; }
            const auto encoder = static_cast<unsigned>(wrapped / tau * 8192.0f) & 0x1fffu;
            const auto speed = static_cast<std::int16_t>(rotor_speed * 60.0f / tau);
            transport::CanFrame frame{};
            frame.id = static_cast<std::uint16_t>(protocol->rx_base + entry.device_id);
            frame.size = 8;
            frame.data[0] = static_cast<std::uint8_t>(encoder >> 8);
            frame.data[1] = static_cast<std::uint8_t>(encoder & 0xffu);
            frame.data[2] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) >> 8);
            frame.data[3] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) & 0xffu);
            frame.data[6] = 40; // 温度字段: 夹具值
            route(entry.bus, frame, now);
        }
        const auto& slot = config.dm[0];
        if (slot.installed && slot.limits && slot.can_id && slot.master_id && dm.emitting) {
            const auto position = drivers::dm_encode_position(*slot.limits, dm.position_rad);
            const auto velocity = drivers::dm_encode_velocity(*slot.limits, dm.velocity_rad_s);
            const auto torque = drivers::dm_encode_torque(*slot.limits, load.dm_feedback_torque_nm);
            if (!position || !velocity || !torque) { return; }
            const auto status = dm_reports_fault ? std::uint8_t{0x8}
                : dm_reports_enabled              ? std::uint8_t{0x1}
                                                  : std::uint8_t{0x0};
            transport::CanFrame frame{};
            frame.id = *slot.master_id;
            frame.size = 8;
            frame.data[0] = static_cast<std::uint8_t>((status << 4) | (*slot.can_id & 0x0fu));
            frame.data[1] = static_cast<std::uint8_t>(*position >> 8);
            frame.data[2] = static_cast<std::uint8_t>(*position & 0xffu);
            frame.data[3] = static_cast<std::uint8_t>(*velocity >> 4);
            frame.data[4] = static_cast<std::uint8_t>(
                ((*velocity & 0x0fu) << 4) | (*torque >> 8));
            frame.data[5] = static_cast<std::uint8_t>(*torque & 0xffu);
            frame.data[6] = 40;
            frame.data[7] = 41;
            route(slot.bus, frame, now);
        }
    }

    void reset() noexcept {
        for (auto& axis : dji) { axis = Axis{}; }
        dm = Axis{};
        external_accel.fill(0.0f);
        dm_external_torque_nm = 0.0f;
    }

private:
    // 达妙使能/失能是 void 语义命令: 同一 CAN ID, 载荷前 7 字节全 0xFF.
    // 它不是 MIT 目标, 负载必须忽略, 否则会把使能帧当成"极限位置 + 最大增益"命令.
    [[nodiscard]] static bool is_dm_void_command(const transport::CanFrame& frame) noexcept {
        for (std::size_t i = 0; i < 7; ++i) {
            if (frame.data[i] != 0xffu) { return false; }
        }
        return true;
    }
    [[nodiscard]] static std::optional<transport::CanFrame> last_dm_command(
        const DmSlot& slot, const SoftwareBus& bus) noexcept {
        for (auto it = bus.frames.rbegin(); it != bus.frames.rend(); ++it) {
            if (it->id != *slot.can_id || it->size != 8) { continue; }
            if (is_dm_void_command(*it)) { continue; }
            return *it;
        }
        return std::nullopt;
    }
};
} // 命名空间 ec::sentinel::host
