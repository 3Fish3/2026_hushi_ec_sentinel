// 合成 APP 装配示例 (通用, 无哨兵机械常数, 不含比赛语义):
//   DR16 输入 → 来源仲裁 → 单位目标 → 四轴 DJI 速度闭环 + 一个 DM MIT 关节 → 分组发送 → 状态回传,
//   附带 BMI088 采样 + 姿态估计, 以及可选的监控注册表发布.
// 虚拟时间与内存 CAN/内存 SPI 只替换外设; 控制链、授权、时效、编码全部走正式组件.
// 演示参数不对应任何实车机构、接线或安全动作; 停机动作由本文件作为"应用层"显式选择.
#include "ec/control/authority.hpp"
#include "ec/control/execution_context.hpp"
#include "ec/drivers/bmi088.hpp"
#include "ec/drivers/bmi088_device.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/drivers/dm_device.hpp"
#include "ec/drivers/dr16.hpp"
#include "ec/estimation/attitude.hpp"
#include "ec/input/dr16_input.hpp"
#include "ec/monitor/registry.hpp"
#include "ec/motion/dji_axis.hpp"
#include "ec/session.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <vector>

namespace {
using ec::Tick;
using ec::drivers::DjiDevice;
using ec::drivers::DjiMode;
using ec::drivers::DjiOutput;
using ec::drivers::DjiTxGroup;
using ec::motion::DjiAxis;
using ec::motion::DjiAxisConfig;

constexpr std::size_t wheel_count = 4;
constexpr std::uint8_t wheel_bus = 1;
constexpr std::uint8_t joint_bus = 2;
constexpr std::uint16_t joint_can_id = 0x01;
constexpr std::uint16_t joint_master_id = 0x11;

// 演示参数: 轴侧 3 A 上限 (C620 满量程 ±20 A); 摇杆满量程对应 8 rad/s.
constexpr float wheel_current_limit_a = 3.0f;
constexpr float wheel_max_rad_s = 8.0f;
constexpr ec::Lifetime wheel_command_ttl{20};
// 旧仓库实测配置换算后的量纲正确版本 (见 docs/modules/control/速度控制链.md), 仅作对照.
constexpr ec::control::PidConfig example_gains{1.2f, 0.05f, 0.0f, 0.0f, -1.0f, 1.0f};

// ---------------------------------------------------------------------------
// 外设替身 (只替换外设, 不替换控制/授权/编码)
// ---------------------------------------------------------------------------
struct MemoryCan final : ec::transport::CanTx {
    std::vector<ec::transport::CanFrame> log{};
    ec::transport::SendResult result{ec::transport::SendResult::accepted};
    unsigned attempts{}; // 真实调用次数: 区分"没走到发送"与"发送返回 busy"
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        ++attempts;
        if (result != ec::transport::SendResult::accepted) { return result; }
        log.push_back(frame);
        return ec::transport::SendResult::accepted;
    }
    [[nodiscard]] const ec::transport::CanFrame* last(std::uint16_t id) const noexcept {
        for (auto it = log.rbegin(); it != log.rend(); ++it) {
            if (it->id == id) { return &*it; }
        }
        return nullptr;
    }
    void clear() noexcept { log.clear(); }
};

// 内存 SPI: 只回答芯片 ID 与固定的加速度/角速度/温度载荷.
struct MemorySpi final : ec::drivers::Bmi088Bus {
    // 演示 SPI 的合成耗时, 不是板级 WCET.
    ec::MicroTick clock{};
    [[nodiscard]] ec::MicroTick micros() const noexcept override { return clock; }
    std::array<std::uint8_t, 6> accel{};
    std::array<std::uint8_t, 6> gyro{};
    bool ok{true};
    [[nodiscard]] bool read(bool acc, std::uint8_t reg, std::span<std::uint8_t> out) noexcept override {
        clock.us += 10;
        if (!ok) { return false; }
        if (acc && reg == ec::drivers::bmi088_acc_chip_id_reg && out.size() == 1) {
            out[0] = ec::drivers::bmi088_acc_chip_id;
            return true;
        }
        if (!acc && reg == ec::drivers::bmi088_gyro_chip_id_reg && out.size() == 1) {
            out[0] = ec::drivers::bmi088_gyro_chip_id;
            return true;
        }
        if (acc && reg == ec::drivers::bmi088_acc_data_reg && out.size() == accel.size()) {
            std::memcpy(out.data(), accel.data(), accel.size());
            return true;
        }
        if (!acc && reg == ec::drivers::bmi088_gyro_data_reg && out.size() == gyro.size()) {
            std::memcpy(out.data(), gyro.data(), gyro.size());
            return true;
        }
        if (acc && reg == ec::drivers::bmi088_acc_temp_reg && out.size() == 2) {
            out[0] = 0x00;
            out[1] = 0x50; // 约 26 °C 的演示值
            return true;
        }
        return false;
    }
    [[nodiscard]] bool write(bool, std::uint8_t, std::uint8_t) noexcept override { return ok; }
    void delay_ms(std::uint32_t) noexcept override {}
};

void put_i16(std::span<std::uint8_t, 6> out, std::size_t index, std::int16_t value) {
    const auto raw = static_cast<std::uint16_t>(value);
    out[2 * index] = static_cast<std::uint8_t>(raw);
    out[2 * index + 1] = static_cast<std::uint8_t>(raw >> 8);
}

// 把物理量写回原始字节, 用于构造主机回放输入.
void put_accel_mps2(std::array<std::uint8_t, 6>& out, float x, float y, float z) {
    const std::array<float, 3> values{x, y, z};
    for (std::size_t i = 0; i < 3; ++i) {
        const auto raw = static_cast<std::int16_t>(std::lround(values[i] / ec::drivers::bmi088_acc_6g_mps2));
        put_i16(out, i, raw);
    }
}

// DR16 原始帧构造: 字段布局与驱动解码一致, 只用于主机回放.
std::array<std::uint8_t, 18> dr16_frame(std::int16_t ch0, std::int16_t ch1, std::int16_t ch2,
    std::int16_t ch3, ec::drivers::Dr16Switch left, ec::drivers::Dr16Switch right,
    std::uint16_t keys = 0) {
    const auto raw = [](std::int16_t value) {
        return static_cast<unsigned>(value + 1024) & 0x07ffu;
    };
    const auto switches = (static_cast<unsigned>(left) << 2) | static_cast<unsigned>(right);
    std::array<std::uint8_t, 18> frame{};
    frame[0] = static_cast<std::uint8_t>(raw(ch0));
    frame[1] = static_cast<std::uint8_t>((raw(ch0) >> 8) | (raw(ch1) << 3));
    frame[2] = static_cast<std::uint8_t>((raw(ch1) >> 5) | (raw(ch2) << 6));
    frame[3] = static_cast<std::uint8_t>(raw(ch2) >> 2);
    frame[4] = static_cast<std::uint8_t>((raw(ch2) >> 10) | (raw(ch3) << 1));
    frame[5] = static_cast<std::uint8_t>((raw(ch3) >> 7) | (switches << 4));
    frame[14] = static_cast<std::uint8_t>(keys);
    frame[15] = static_cast<std::uint8_t>(keys >> 8);
    return frame;
}

// ---------------------------------------------------------------------------
// 装配: 设备与轴的生命周期都在这里, 轴不拥有设备.
// ---------------------------------------------------------------------------
struct App {
    MemoryCan wheel_tx{};
    MemoryCan joint_tx{};
    std::array<DjiDevice, wheel_count> devices{{
        DjiDevice{{wheel_bus, DjiMode::c620_current, 1, 1, 1, {10}, 100, 16384}},
        DjiDevice{{wheel_bus, DjiMode::c620_current, 2, 1, 1, {10}, 100, 16384}},
        DjiDevice{{wheel_bus, DjiMode::c620_current, 3, 1, 1, {10}, 100, 16384}},
        DjiDevice{{wheel_bus, DjiMode::c620_current, 4, 1, 1, {10}, 100, 16384}},
    }};
    std::array<std::optional<DjiAxis>, wheel_count> axes{};
    DjiTxGroup group{wheel_tx, wheel_bus, 1, {2},
        {&devices[0], &devices[1], &devices[2], &devices[3]}};

    ec::drivers::DmDevice joint{{joint_bus, joint_can_id, joint_master_id, {12.5f, 30.0f, 10.0f}, {20}}};

    ec::input::Dr16Input remote{{100}};
    MemorySpi spi{};
    ec::drivers::Bmi088Device imu{spi, {20}};
    ec::estimation::AttitudeEstimator attitude{};
    Tick last_imu_at{};
    unsigned imu_reads{};

    ec::control::ControlAuthority authority{};
    ec::session::SessionTracker session{{200, 64}};
    ec::monitor::Registry registry{};
    bool monitoring{true};

    // 停机动作是应用层决策: 轮子给显式零输出, 关节保持最后位置目标.
    ec::control::ExecutionPolicy wheel_policy{ec::control::StopAction::zero_output,
        ec::control::StopAction::zero_output};
    ec::control::ExecutionPolicy joint_policy{ec::control::StopAction::hold_position,
        ec::control::StopAction::hold_position};

    std::array<float, wheel_count> target_rad_s{};
    std::array<Tick, wheel_count> target_at{};
    std::array<bool, wheel_count> target_valid{};
    // 轮目标自带“接受时”的授权身份: 执行时不再换发当前 token. 绑定发生在目标有效性
    // false→true 的那一周期(一次新的接受), 之后不再刷新, 因此换来源/换会话后旧目标失效.
    std::array<std::optional<ec::Grant>, wheel_count> wheel_grant{};
    std::array<bool, wheel_count> target_seen{}; // 上一周期是否已接受, 用于识别新的接受
    unsigned axis_rejects{}; // 轴层拒绝(反馈过期/非法 dt/使能失败)次数, 仅供示例断言
    // 本周期显式选择的停机输出 (只在本周期有效, 不跨周期保留).
    std::array<std::optional<DjiOutput>, wheel_count> stop_outputs{};
    // 只有“曾经真正使能过”的轴才需要执行停机动作; 启动前无授权时不主动给电调发零.
    std::array<bool, wheel_count> ever_enabled{};
    bool joint_target_valid{};
    float joint_target_rad{};
    // 关节目标自带接受时的授权身份: 执行时不再换发当前 token, 换会话/换来源后旧目标失效.
    std::optional<ec::Grant> joint_grant{};
    bool joint_enable_sent{};
    bool joint_ready{}; // 最近一次周期是否观察到关节已使能
    ec::control::DenyReason last_deny{ec::control::DenyReason::none};
    std::optional<ec::CommandSource> last_source{};
    unsigned stop_commits{};

    App() {
        for (std::size_t i = 0; i < wheel_count; ++i) {
            axes[i].emplace(DjiAxisConfig{wheel_current_limit_a, example_gains, {},
                                ec::control::ArcMode::continuous, wheel_command_ttl, wheel_max_rad_s,
                                0.02f, 0.0f},
                devices[i]);
        }
        (void)registry.claim(ec::monitor::RecordKind::period, 0);
        (void)registry.claim(ec::monitor::RecordKind::source, 0);
        (void)registry.claim(ec::monitor::RecordKind::environment, 0);
        for (std::size_t i = 0; i < wheel_count; ++i) {
            (void)registry.claim(ec::monitor::RecordKind::axis, static_cast<std::uint16_t>(i));
        }
        (void)registry.claim(ec::monitor::RecordKind::device, 0);
    }

    // --- 输入侧 -------------------------------------------------------------
    // 返回 true 表示这一帧被接受; 掉线由时效判定, 不由"最后一帧"判定.
    bool feed_remote(Tick now, std::int16_t ch0, std::int16_t ch1, std::int16_t ch2, std::int16_t ch3,
        ec::drivers::Dr16Switch left, ec::drivers::Dr16Switch right) {
        const auto frame = dr16_frame(ch0, ch1, ch2, ch3, left, right);
        const auto sample = ec::drivers::decode_dr16(frame);
        return sample.has_value() && remote.accept(*sample, now);
    }

    void feed_feedback(Tick now, std::int16_t rpm) {
        for (std::size_t i = 0; i < wheel_count; ++i) {
            feed_feedback(now, rpm, i);
        }
    }

    // 只刷新指定轴的反馈, 用于构造"同组中单轴反馈失效"的故障.
    void feed_feedback(Tick now, std::int16_t rpm, std::size_t index) {
        const auto speed = static_cast<std::uint16_t>(rpm);
        const auto encoder = static_cast<std::uint16_t>(index * 100);
        const ec::transport::CanFrame frame{static_cast<std::uint16_t>(0x201 + index), 8,
            {static_cast<std::uint8_t>(encoder >> 8), static_cast<std::uint8_t>(encoder),
                static_cast<std::uint8_t>(speed >> 8), static_cast<std::uint8_t>(speed), 0, 0, 25, 0}};
        (void)devices[index].receive(wheel_bus, frame, now);
    }

    // DM 反馈注入: 反馈 ID 是配置的 Master ID, 载荷低 4 位是命令 ID 的节点字段.
    bool feed_joint(Tick now, std::uint8_t status) {
        ec::transport::CanFrame frame{joint_master_id, 8, {}};
        frame.data[0] = static_cast<std::uint8_t>((joint_can_id & 0x0fu) | (status << 4));
        frame.data[1] = 0x08;
        frame.data[2] = 0x00;
        frame.data[3] = 0x01;
        frame.data[4] = 0x00;
        frame.data[5] = 0x01;
        frame.data[6] = 30;
        frame.data[7] = 40;
        return joint.receive(joint_bus, frame, now) == ec::drivers::DmReceiveResult::accepted;
    }

    // --- 控制周期 -----------------------------------------------------------
    void tick(Tick now, float dt) {
        const auto snapshot = remote.snapshot(now);
        const bool remote_online = snapshot.has_value();
        // 演示的接管动作: 右拨杆下位. 具体拨杆含义属于应用映射, 不属于框架.
        const bool takeover = snapshot.has_value() &&
            snapshot->switch_right == ec::drivers::Dr16Switch::down;
        const ec::control::ControlAuthority::Inputs inputs{remote_online, takeover,
            session.link_alive(now), host_requests, host_authorized, true, session.epoch()};
        const auto decision = authority.update(inputs);
        last_source = decision ? std::optional{decision->source} : std::nullopt;

        for (std::size_t i = 0; i < wheel_count; ++i) {
            auto& axis = *axes[i];
            // 一次新的目标接受: 绑定当时的授权身份. 执行检查用它, 而不是当前 token ——
            // “当前有人持权”不能证明这份旧目标属于当前持权者.
            if (target_valid[i] && !target_seen[i]) { wheel_grant[i] = authority.token(); }
            target_seen[i] = target_valid[i];
            const bool target_fresh = target_valid[i] && wheel_grant[i].has_value() &&
                (now.ms - target_at[i].ms) < wheel_command_ttl.ms;
            const ec::control::ExecutionRequest request{wheel_grant[i].value_or(ec::Grant{}), true,
                target_fresh, remote_online || session.link_alive(now)};
            const ec::control::ExecutionContext context{authority.gate(), wheel_policy};
            const auto verdict = context.evaluate(request);
            if (!verdict.execute) {
                last_deny = verdict.reason;
                // 停机动作由本层显式选择: 撤销执行许可, 只对“曾经使能过”且配置为零输出的轴
                // 提交本周期零输出. 不得回填历史控制量.
                axis.disable();
                if (ever_enabled[i] && verdict.stop == ec::control::StopAction::zero_output) {
                    stop_outputs[i] = DjiOutput{0.0f, now};
                    ++stop_commits;
                }
                clear_target(i);
                continue;
            }
            if (!axis.enabled()) {
                const bool ready = devices[i].state(now).velocity_rad_s.fresh_at(now, {10});
                if (!axis.enable(ec::control::ControlMode::velocity, decision->source, true, ready)) {
                    // 使能失败也消耗本次接受: 只恢复反馈不得重新执行旧目标.
                    clear_target(i);
                    ++axis_rejects;
                    continue;
                }
                ever_enabled[i] = true;
            }
            bool executed = false;
            if (axis.set_velocity(target_rad_s[i], now)) {
                executed = axis.tick(now, dt).status == ec::motion::AxisStatus::active;
            }
            if (!executed) {
                // 轴层拒绝(反馈过期/非法 dt/控制器错误)时本层目标一并失效:
                // 恢复必须来自一次新的目标接受, 不能靠“轴未使能”自动重建旧命令.
                clear_target(i);
                ++axis_rejects;
            }
        }

        // 组发送: 每槽位只能是本周期真实计算的轴输出, 或本层显式选择的停机输出.
        // 轴拒绝(反馈过期/非法 dt/控制器错误)时该槽位留空, 由组发送器拒整组.
        std::array<std::optional<DjiOutput>, wheel_count> outputs{};
        for (std::size_t i = 0; i < wheel_count; ++i) {
            if (const auto& out = axes[i]->output()) { outputs[i] = *out; }
            else if (stop_outputs[i]) { outputs[i] = stop_outputs[i]; }
            stop_outputs[i].reset();
        }
        const auto group_result = group.send(now, outputs);
        group_status = group_result.status;

        // DM 关节: 只有上位机来源驱动; 停机动作是保持最后位置目标.
        drive_joint(now);
        if (monitoring) { publish(now); }
    }

    // 失效本层保存的轮目标与它的授权身份: 之后必须重新接受才能执行.
    void clear_target(std::size_t index) {
        target_valid[index] = false;
        target_seen[index] = false;
        wheel_grant[index].reset();
    }

    void drive_joint(Tick now) {
        const bool host_owns = last_source == ec::CommandSource::host;
        // 执行身份取自目标被接受时的授权; 当前来源不是上位机时不得用当前 token 顶替.
        const ec::Grant token = (host_owns && joint_grant) ? *joint_grant : ec::Grant{};
        const bool target_fresh = host_owns && joint_target_valid && session.intent_alive(now);
        const ec::control::ExecutionContext context{authority.gate(), joint_policy};
        const auto verdict = context.evaluate({token, true, target_fresh, session.link_alive(now)});
        if (!verdict.execute) {
            if (!host_owns) {
                joint_enable_sent = false;
                joint_grant.reset();
                joint_target_valid = false;
            }
            return;
        }
        if (!joint_enable_sent) {
            if (const auto enable = joint.encode_enable()) {
                // 编码成功不等于已发出: busy/error 不得记为“已发送使能”.
                if (joint_tx.try_send(*enable) == ec::transport::SendResult::accepted) {
                    joint_enable_sent = true;
                }
            }
            return; // 使能命令与目标不在同一周期下发
        }
        // 必须先观察到设备的新鲜使能反馈, 再下发 MIT 目标; 否则只能等下一周期.
        joint_ready = joint.enabled(now);
        if (!joint_ready) { return; }
        const ec::drivers::DmMitCommand command{joint_target_rad, 0.0f, 20.0f, 1.0f, 0.0f};
        if (const auto frame = joint.encode_mit(command)) { (void)joint_tx.try_send(*frame); }
    }

    // --- 监控 ---------------------------------------------------------------
    void publish(Tick now) {
        registry.begin_update();
        if (auto* period = registry.find(ec::monitor::RecordKind::period, 0)) {
            period->validity = ec::monitor::Validity::valid;
            period->sampled_at = now;
            period->published_at = now;
            period->target = 10.0f;
            period->measurement = 10.0f;
        }
        if (auto* source = registry.find(ec::monitor::RecordKind::source, 0)) {
            const auto snapshot = remote.snapshot(now);
            source->validity = snapshot ? ec::monitor::Validity::valid : ec::monitor::Validity::stale;
            source->enabled = last_source.has_value() ? 1 : 0;
            source->mode = last_source.has_value() ? static_cast<std::uint8_t>(*last_source) : 0;
            source->reject = snapshot ? ec::monitor::RejectReason::none
                                      : ec::monitor::RejectReason::link_stale;
            source->sampled_at = snapshot ? snapshot->sampled_at : Tick{};
            source->published_at = now;
        }
        if (auto* environment = registry.find(ec::monitor::RecordKind::environment, 0)) {
            const auto reading = imu.reading(now);
            environment->validity = reading ? ec::monitor::Validity::valid
                                            : ec::monitor::Validity::unavailable;
            environment->sampled_at = last_imu_at;
            environment->published_at = now;
            environment->measurement = reading ? reading->sample.temp_c : 0.0f;
        }
        for (std::size_t i = 0; i < wheel_count; ++i) {
            if (auto* axis = registry.find(ec::monitor::RecordKind::axis, static_cast<std::uint16_t>(i))) {
                const auto state = axes[i]->state(now);
                axis->validity = state.enabled ? ec::monitor::Validity::valid
                                               : ec::monitor::Validity::stale;
                axis->enabled = state.enabled ? 1 : 0;
                axis->mode = static_cast<std::uint8_t>(state.mode);
                axis->reject = static_cast<ec::monitor::RejectReason>(0);
                axis->sampled_at = state.velocity_rad_s.sampled_at;
                axis->published_at = now;
                axis->target = state.target.value;
                axis->measurement = state.velocity_rad_s.value;
            }
        }
        if (auto* device = registry.find(ec::monitor::RecordKind::device, 0)) {
            device->validity = joint.online(now) ? ec::monitor::Validity::valid
                                                 : ec::monitor::Validity::stale;
            device->enabled = joint.enabled(now) ? 1 : 0;
            device->detail = static_cast<std::uint8_t>(joint.state().feedback.value.status);
            device->sampled_at = joint.state().feedback.sampled_at;
            device->published_at = now;
            device->measurement = joint.state().feedback.value.position_rad;
        }
        registry.end_update(now);
    }

    void read_imu(Tick now) {
        if (imu.read(now) == ec::drivers::Bmi088Result::ok) {
            last_imu_at = now;
            ++imu_reads;
            const auto dt = imu.last_reading().dt_ms;
            const auto step = dt == 0 ? 0.01f : static_cast<float>(dt) / 1000.0f;
            std::array<float, 3> gyro{};
            std::array<float, 3> accel{};
            for (std::size_t i = 0; i < 3; ++i) {
                gyro[i] = imu.last_reading().sample.gyro_rad_s[i];
                accel[i] = imu.last_reading().sample.acc_mps2[i];
            }
            (void)attitude.update(gyro, accel, step);
        }
    }

    // 应用层接受关节目标: 记录当前授权身份, 供执行时核对.
    void set_joint_target(float rad) {
        joint_target_rad = rad;
        joint_target_valid = true;
        joint_grant = authority.token();
    }

    // 上位机请求与本地授权 (本示例里由 main 的相位显式设置).
    bool host_requests{};
    bool host_authorized{true};
    ec::drivers::DjiGroupStatus group_status{ec::drivers::DjiGroupStatus::not_sent};
};

std::int16_t slot_raw(const ec::transport::CanFrame& frame, std::size_t slot) {
    return static_cast<std::int16_t>(
        (static_cast<unsigned>(frame.data[2 * slot]) << 8) | frame.data[2 * slot + 1]);
}
} // 演示命名空间

int main() {
    App app;
    if (app.imu.init() != ec::drivers::Bmi088Result::ok) { return 1; }
    put_accel_mps2(app.spi.accel, 0.0f, 0.0f, 9.80665f); // 水平静止
    app.read_imu({1});

    // 相位 1: 没有遥控输入, 没有授权 → 不发任何控制帧.
    app.tick({10}, 0.001f);
    if (app.last_source || app.group_status != ec::drivers::DjiGroupStatus::missing_output) { return 2; }
    if (!app.wheel_tx.log.empty() || !app.joint_tx.log.empty()) { return 2; }
    std::puts("phase 1: no input, no authority -> no control frame");

    // 相位 2: 遥控接管 → 单位目标 → 四轴闭环 → 同组一帧 → 状态回传.
    app.feed_feedback({10}, 0);
    for (int i = 0; i < 4; ++i) {
        if (!app.feed_remote({10}, 660, -660, 660, -660, ec::drivers::Dr16Switch::up,
                ec::drivers::Dr16Switch::down)) {
            return 3;
        }
    }
    // 输入归一化 → 单位目标 (应用层换算, 框架只认 rad/s).
    for (std::size_t i = 0; i < wheel_count; ++i) {
        const auto snapshot = app.remote.snapshot({10});
        if (!snapshot) { return 3; }
        const auto& axes = snapshot->axis;
        const std::array<float, wheel_count> values{axes[0], axes[1], axes[2], axes[3]};
        app.target_rad_s[i] = values[i] * wheel_max_rad_s;
        app.target_at[i] = {10};
        app.target_valid[i] = true;
    }
    app.tick({10}, 0.001f);
    if (!app.last_source || *app.last_source != ec::CommandSource::remote) { return 3; }
    if (app.group_status != ec::drivers::DjiGroupStatus::accepted) { return 3; }
    const auto* frame = app.wheel_tx.last(0x200);
    if (frame == nullptr) { return 3; }
    // 满量程摇杆 → 目标 8 rad/s; 误差 8 rad/s × Kp 1.2 = 9.6 → PID 输出限幅到 1.0,
    // 再按轴侧电流上限换算: 1.0 × 3 A / 20 A = 0.15 归一化 → C620 原始域 2458.
    const auto expected = app.devices[0].encode_output(1.0f * wheel_current_limit_a / 20.0f);
    if (!expected || slot_raw(*frame, 0) != *expected) { return 3; }
    const auto state = app.axes[0]->state({10});
    if (!state.enabled || state.mode != ec::control::ControlMode::velocity) { return 3; }
    if (std::fabs(state.target.value - wheel_max_rad_s) > 1e-4f) { return 3; }
    std::printf("phase 2: remote takeover, targets=%.2f..%.2f rad/s, one group frame with 4 slots\n",
        static_cast<double>(app.target_rad_s[0]), static_cast<double>(app.target_rad_s[3]));

    // 相位 3: IMU 采样 + 姿态估计 (水平静止应给出接近 0 的 roll/pitch).
    for (int i = 1; i <= 50; ++i) { app.read_imu({static_cast<std::uint32_t>(10 + i * 10)}); }
    const auto attitude = app.attitude.state();
    if (!std::isfinite(attitude.roll_pitch_yaw_rad[0]) || !std::isfinite(attitude.roll_pitch_yaw_rad[1])) {
        return 4;
    }
    if (std::fabs(attitude.roll_pitch_yaw_rad[0]) > 0.05f ||
        std::fabs(attitude.roll_pitch_yaw_rad[1]) > 0.05f) {
        return 4;
    }
    std::printf("phase 3: imu reads=%u roll=%.4f pitch=%.4f rad (yaw not observable here)\n",
        app.imu_reads, static_cast<double>(attitude.roll_pitch_yaw_rad[0]),
        static_cast<double>(attitude.roll_pitch_yaw_rad[1]));

    // 相位 4: 遥控失联 → 撤销授权 → 应用层执行停机动作 (显式零输出), 不是"不发帧".
    app.wheel_tx.clear();
    app.feed_feedback({300}, 0);
    app.tick({300}, 0.001f);
    if (app.last_source || app.authority.holds()) { return 5; }
    if (app.last_deny != ec::control::DenyReason::no_grant) { return 5; }
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || slot_raw(*frame, 0) != 0 || slot_raw(*frame, 3) != 0) { return 5; }
    std::puts("phase 4: link lost -> authority revoked, explicit zero commit on the same group frame");

    // 相位 5: 恢复后必须重新使能并重新下发目标, 旧目标不复用.
    app.wheel_tx.clear();
    app.feed_feedback({320}, 0);
    app.target_valid = {};
    if (app.axes[0]->enabled()) { return 6; }
    if (app.axes[0]->set_velocity(1.0f, {320})) { return 6; } // 未使能 → 目标不被接受
    for (int i = 0; i < 4; ++i) {
        if (!app.feed_remote({320}, 0, 0, 0, 0, ec::drivers::Dr16Switch::up,
                ec::drivers::Dr16Switch::down)) {
            return 6;
        }
    }
    app.tick({320}, 0.001f);
    if (!app.last_source || *app.last_source != ec::CommandSource::remote) { return 6; }
    for (std::size_t i = 0; i < wheel_count; ++i) {
        app.target_rad_s[i] = 2.0f;
        app.target_at[i] = {320};
        app.target_valid[i] = true;
    }
    app.tick({320}, 0.001f);
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || slot_raw(*frame, 0) == 0) { return 6; }
    const auto recovered = app.axes[0]->state({320});
    if (!recovered.enabled || std::fabs(recovered.target.value - 2.0f) > 1e-4f) { return 6; }
    std::puts("phase 5: recovery requires re-enable and a fresh target; old target was not reused");

    // 相位 5b (R01): 来源与目标仍有效, 但同组单轴反馈过期时不得把历史输出重新盖上当前时间.
    // 轴层拒绝后整组必须缺输出而拒发; 应用层只能用显式停机动作, 不能回填缓存值.
    app.wheel_tx.clear();
    const auto hold_wheels = [&app](Tick now) {
        for (int i = 0; i < 4; ++i) {
            if (!app.feed_remote(now, 0, 0, 0, 0, ec::drivers::Dr16Switch::up,
                    ec::drivers::Dr16Switch::down)) {
                return false;
            }
        }
        for (std::size_t i = 0; i < wheel_count; ++i) {
            app.target_rad_s[i] = 2.0f;
            app.target_at[i] = now;
            app.target_valid[i] = true;
        }
        return true;
    };
    if (!hold_wheels({340})) { return 16; }
    app.feed_feedback({340}, 0);
    app.tick({340}, 0.001f);
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || slot_raw(*frame, 2) == 0) { return 16; }
    const auto running_before = slot_raw(*frame, 2);
    // 只停掉 2 号轴反馈; 遥控输入与四轴目标保持新鲜, 避免先走显式零输出路径掩盖本问题.
    app.wheel_tx.clear();
    if (!hold_wheels({360})) { return 16; }
    app.feed_feedback({360}, 0, 0);
    app.feed_feedback({360}, 0, 1);
    app.feed_feedback({360}, 0, 3); // 2 号轴反馈停留在 340, TTL 10 ms
    app.tick({360}, 0.001f);
    if (app.axes[2]->output().has_value()) { return 16; }
    frame = app.wheel_tx.last(0x200);
    if (frame != nullptr) { return 16; } // 缺新鲜输出必须整组拒发, 不猜补值
    if (app.group_status != ec::drivers::DjiGroupStatus::missing_output) { return 16; }
    // 反馈恢复后必须重新使能并重新计算, 而不是继续用故障前的输出.
    if (!hold_wheels({380})) { return 16; }
    app.feed_feedback({380}, 0);
    app.tick({380}, 0.001f);
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || !app.axes[2]->enabled()) { return 16; }
    const auto fresh = app.axes[2]->output();
    if (!fresh || fresh->computed_at.ms != 380) { return 16; }
    std::printf("phase 5b: one stale wheel slot -> whole group refused (no re-stamped output); %d -> %d\n",
        static_cast<int>(running_before), static_cast<int>(slot_raw(*frame, 2)));

    // 相位 5c (V01): 只有反馈恢复、没有新目标时, 不得自动重新使能并复用故障前的目标.
    // 5b 在恢复前重新接受了目标, 因此它证明不了这一点; 这里保持目标不动.
    app.wheel_tx.clear();
    if (!hold_wheels({385})) { return 18; }
    app.feed_feedback({385}, 0);
    app.tick({385}, 0.001f);
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || slot_raw(*frame, 2) == 0) { return 18; }
    // 2 号轴反馈停在 385: 下一周期过期, 轴层拒绝.
    app.wheel_tx.clear();
    app.feed_feedback({395}, 0, 0);
    app.feed_feedback({395}, 0, 1);
    app.feed_feedback({395}, 0, 3);
    app.tick({395}, 0.001f);
    if (app.axes[2]->enabled() || app.target_valid[2] || app.wheel_grant[2]) { return 18; }
    // 只恢复反馈: 目标仍有效但已被拒绝消耗, 不得重新使能或输出旧值, 只允许显式停机.
    app.wheel_tx.clear();
    app.feed_feedback({396}, 0, 2);
    app.tick({396}, 0.001f);
    if (app.axes[2]->enabled() || app.axes[2]->output().has_value()) { return 18; }
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || slot_raw(*frame, 2) != 0) { return 18; }
    // 真正的新目标到达后才允许恢复.
    app.wheel_tx.clear();
    app.target_rad_s[2] = 2.0f;
    app.target_at[2] = {397};
    app.target_valid[2] = true;
    app.feed_feedback({397}, 0);
    app.tick({397}, 0.001f);
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || !app.axes[2]->enabled() || slot_raw(*frame, 2) == 0) { return 18; }
    std::puts("phase 5c: feedback-only recovery does not resume the pre-fault target; a new target does");

    // 相位 6: 上位机接管关节 (会话 + 意图), 轮子按策略停机, 关节收到 MIT 目标.
    const ec::session::SessionId host{7, 0x1234, 1};
    if (!app.session.begin(host, {400})) { return 7; }
    if (!app.session.heartbeat(host, {400})) { return 7; }
    if (app.session.accept_intent(host, 1, {400}, ec::Lifetime{200}) != ec::session::IntentResult::accepted) {
        return 7;
    }
    app.host_requests = true;
    // 遥控拨杆回中位: 不再请求接管, 但链路仍在线.
    for (int i = 0; i < 4; ++i) {
        if (!app.feed_remote({400}, 0, 0, 0, 0, ec::drivers::Dr16Switch::up,
                ec::drivers::Dr16Switch::mid)) {
            return 7;
        }
    }
    app.joint_tx.clear();
    app.wheel_tx.clear();
    // 会话身份刚变化: 本周期不授予任何来源, 旧目标不得续用.
    app.tick({400}, 0.001f);
    if (app.last_source.has_value() || app.joint_tx.log.size() != 0) { return 7; }
    // 6a: 使能帧发送 busy 时不得记为已发送, 也不得下发 MIT 目标.
    app.joint_tx.result = ec::transport::SendResult::busy;
    app.tick({405}, 0.001f); // host 取得授权; 本周期使能尝试被 busy 拒绝
    if (app.joint_enable_sent || !app.joint_tx.log.empty()) { return 7; }
    if (!app.last_source || *app.last_source != ec::CommandSource::host) { return 7; }
    // 目标在授权建立后接受, 因此自带当时的授权身份.
    app.set_joint_target(1.5f);
    if (!app.joint_grant) { return 7; }
    // 有有效目标后仍处于 busy: 必须真的走到发送并被拒, 而不是"根本没走到发送".
    // 只检查"日志为空"无法区分这两者, 因此核对真实调用次数.
    const auto attempts_before = app.joint_tx.attempts;
    app.tick({406}, 0.001f);
    if (app.joint_tx.attempts != attempts_before + 1) { return 7; }
    if (app.joint_enable_sent || !app.joint_tx.log.empty()) { return 7; }
    // 6b: 发送成功但还没有观察到设备使能反馈: 只发使能帧, 不发目标.
    // 使能帧在 6a 被 busy 拒绝, 因此这里仍要重新编码发送 (而不是记成已发送).
    app.joint_tx.result = ec::transport::SendResult::accepted;
    app.tick({410}, 0.001f);
    if (!app.joint_enable_sent || app.joint_tx.log.size() != 1) { return 7; }
    app.tick({415}, 0.001f);
    if (app.joint_tx.log.size() != 1) { return 7; }
    // 6c: 只有观察到设备新鲜使能反馈后, 才下发 MIT 目标.
    if (!app.feed_joint({420}, 0x01)) { return 7; }
    app.tick({425}, 0.001f);
    if (!app.joint_ready || app.joint_tx.log.size() != 2) { return 7; }
    if (!app.last_source || *app.last_source != ec::CommandSource::host) { return 7; }
    const auto* mit = app.joint_tx.last(joint_can_id);
    if (mit == nullptr || mit->size != 8) { return 7; }
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || slot_raw(*frame, 0) != 0) { return 7; } // 轮子按策略停机
    std::puts("phase 6: host session owns the joint; enable result and observed status gate the target");

    // 相位 6b (R03): 会话被替换后旧关节目标不得继续执行, 也不得被换发当前 token.
    const ec::session::SessionId host_b{7, 0x9999, 1}; // 同来源, 新启动实例
    if (app.session.begin(host_b, {430}, true)) { /* 本地明确允许重建 */ } else { return 7; }
    app.tick({430}, 0.001f); // 身份刚变化: 本周期不授予来源
    if (app.last_source.has_value() || app.authority.holds()) { return 7; }
    const auto joint_frames_before = app.joint_tx.log.size();
    app.tick({435}, 0.001f);
    if (app.joint_tx.log.size() != joint_frames_before) { return 7; } // 旧目标不再执行
    if (app.joint_enable_sent || app.joint_grant || app.joint_target_valid) { return 7; }
    std::puts("phase 6b: session replacement drops the joint target and its grant identity");

    // 相位 6c (V02): 轮目标自带接受时的授权身份. 会话早已存在、来源直接切换时,
    // 旧来源接受的轮目标不得用当前 token 重新执行.
    if (!app.session.heartbeat(host_b, {440})) { return 19; }
    app.wheel_tx.clear();
    app.host_requests = false;
    for (int i = 0; i < 4; ++i) {
        if (!app.feed_remote({440}, 0, 0, 0, 0, ec::drivers::Dr16Switch::up,
                ec::drivers::Dr16Switch::down)) { return 19; }
    }
    app.feed_feedback({440}, 0);
    app.tick({440}, 0.001f);
    if (!app.last_source || *app.last_source != ec::CommandSource::remote) { return 19; }
    // 遥控持有期间接受四个轮目标 (授权身份 = remote).
    app.wheel_tx.clear();
    for (std::size_t i = 0; i < wheel_count; ++i) {
        app.target_rad_s[i] = 3.0f;
        app.target_at[i] = {440};
        app.target_valid[i] = true;
    }
    app.feed_feedback({440}, 0);
    app.tick({440}, 0.001f);
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || slot_raw(*frame, 0) == 0) { return 19; }
    // 直接切权到上位机: 会话未变、反馈与旧目标都新鲜, 且不更新轮目标.
    app.wheel_tx.clear();
    for (int i = 0; i < 4; ++i) {
        if (!app.feed_remote({441}, 0, 0, 0, 0, ec::drivers::Dr16Switch::up,
                ec::drivers::Dr16Switch::mid)) { return 19; }
    }
    app.host_requests = true;
    app.feed_feedback({441}, 0);
    app.tick({441}, 0.001f);
    if (!app.last_source || *app.last_source != ec::CommandSource::host) { return 19; }
    if (app.target_valid[0] || app.wheel_grant[0]) { return 19; } // 旧轮目标及其授权必须失效
    frame = app.wheel_tx.last(0x200);
    if (frame == nullptr || slot_raw(*frame, 0) != 0) { return 19; } // 只允许显式停机输出
    std::puts("phase 6c: a source switch does not re-sign the previous owner's wheel target");

    // 相位 7: 监控开关不影响控制轨迹与发送结果 (逐帧逐字节比较).
    const auto capture = [&app](Tick now) {
        const auto wheel_before = app.wheel_tx.log.size();
        const auto joint_before = app.joint_tx.log.size();
        app.tick(now, 0.001f);
        std::vector<ec::transport::CanFrame> frames{};
        for (std::size_t i = wheel_before; i < app.wheel_tx.log.size(); ++i) {
            frames.push_back(app.wheel_tx.log[i]);
        }
        for (std::size_t i = joint_before; i < app.joint_tx.log.size(); ++i) {
            frames.push_back(app.joint_tx.log[i]);
        }
        return frames;
    };
    app.monitoring = true;
    const auto with_monitor = capture({445});
    const auto generation_before = app.registry.generation();
    app.monitoring = false;
    const auto without_monitor = capture({445});
    if (with_monitor.empty() || with_monitor.size() != without_monitor.size()) { return 8; }
    for (std::size_t i = 0; i < with_monitor.size(); ++i) {
        const auto& a = with_monitor[i];
        const auto& b = without_monitor[i];
        if (a.id != b.id || a.size != b.size || a.data != b.data) { return 8; }
    }
    if (app.registry.generation() != generation_before) { return 8; } // 关闭后不再推进代次
    if (app.registry.used() == 0) { return 8; }
    std::printf("phase 7: monitoring records=%zu generation=%u; %zu frames identical with monitoring on/off\n",
        app.registry.used(), app.registry.generation(), with_monitor.size());

    // 相位 8: 旧参数主机回放对照. 只回答"旧参数换算到新域之后大概处在什么量级",
    // 不模拟真实电机与负载, 因此不能用来宣称 S5 无负载速度发散问题已修复.
    // 量纲: VelocityLoop 的输入是 rad/s, 输出是 [-1, 1] 的满量程分数, 不是 A.
    // 旧仓库主 yaw 有效配置换算: Kp = 73 × (180/π) × (3/16384) ≈ 0.765855 A/(rad/s), Ki = 0.
    constexpr float amp_full_scale_a = 20.0f; // C620 满量程 ±20 A
    constexpr float old_kp_a_per_rad_s = 0.765855f;
    constexpr float old_output_cap_a = 3000.0f / 16384.0f * 3.0f; // 旧 3000 raw 限幅 ≈ 0.549316 A
    // 回放: 正式 VelocityLoop, 误差可变, 输出限幅与该回放的归一化基准一致.
    // 旧 Kp 是 A/(rad/s), 而 VelocityLoop 的增益/限幅都以"满量程分数"为单位:
    // 必须把 A 域数值除以满量程后再填入, 限幅同理 (旧 3000 raw ≈ 0.549316 A).
    const ec::control::PidConfig old_gains_normalized{old_kp_a_per_rad_s / amp_full_scale_a,
        0.0f, 0.0f, 0.0f, -old_output_cap_a / amp_full_scale_a, old_output_cap_a / amp_full_scale_a};
    const auto replay = [](ec::control::PidConfig gains_normalized, float current_limit_a,
                            float error_rad_s) {
        ec::control::VelocityLoop loop{{gains_normalized, {20}, 100.0f, 0.02f, 0.0f}};
        const auto grant = loop.arm(ec::CommandSource::remote, true, true);
        std::vector<float> outputs{};
        for (int step = 0; step < 5; ++step) {
            const Tick now{static_cast<std::uint32_t>(step)};
            const ec::Command<float> target{error_rad_s, now, {20}, *grant};
            const ec::Sample<float> feedback{0.0f, now, ec::Validity::valid}; // 恒定误差, 不是电机模型
            const auto result = loop.step(now, 0.001f, target, feedback);
            // 归一化输出 → A: 乘回满量程; 这里用同一满量程常量, 不另写一套换算.
            outputs.push_back(result.output.value_or(0.0f) * current_limit_a);
        }
        return outputs;
    };
    // 不饱和与饱和两种误差都走正式速度环, 再转回 A 与独立期望值比较.
    const auto old_unsaturated = replay(old_gains_normalized, amp_full_scale_a, 0.1f);
    const auto old_saturated = replay(old_gains_normalized, amp_full_scale_a, 1.0f);
    const auto new_replay = replay(example_gains, amp_full_scale_a, 1.0f);
    if (std::fabs(old_unsaturated.front() - 0.0765855f) > 1e-6f) { return 9; }
    if (std::fabs(old_saturated.front() - old_output_cap_a) > 1e-5f) { return 9; }
    std::printf("phase 8: old replay via VelocityLoop -> %.4f A at 0.1 rad/s (unsaturated), "
                "%.4f A at 1 rad/s (old cap=%.4f A, Kp=%.6f A/(rad/s)); new gains -> %.4f A\n",
        static_cast<double>(old_unsaturated.front()), static_cast<double>(old_saturated.front()),
        static_cast<double>(old_output_cap_a), static_cast<double>(old_kp_a_per_rad_s),
        static_cast<double>(new_replay.front()));
    std::puts("phase 8: host replay only; old filter/D term and real load were not reproduced,"
              " so the S5 no-load speed divergence stays an open measurement issue");
    return 0;
}
