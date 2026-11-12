// O4 DT7/ch4 全链路契约: 从原始 18 B 帧经 Dr16Stream/Dr16Input/RemoteMapper 到应用采用目标.
// 覆盖串流分片/边界、ch4 中位与两侧、切层抖动、右旋转档覆盖、重复帧、掉线后首帧、
// 时间回绕, 以及改用途通道的回中接管、拨弹不锁存、摩擦跨层保持与换源/停止/失联清除.
// ch4 中位/方向/量程是显式夹具值, 不代表实机取值.
#include "sentinel_execution.hpp"
#include "sentinel_remote.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

namespace {
void check(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "line %d: %s\n", line, expression);
        std::_Exit(3);
    }
}
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__, __LINE__)

using ec::Tick;
using ec::drivers::Dr16Stream;
using ec::drivers::Dr16Switch;
using ec::input::Dr16Input;
using ec::input::Dr16Snapshot;
using ec::sentinel::app::AcceptResult;
using ec::sentinel::app::AxisPurpose;
using ec::sentinel::app::Ch4Layer;
using ec::sentinel::app::Ch4LayerConfig;
using ec::sentinel::app::ConfigAudience;
using ec::sentinel::app::Domain;
using ec::sentinel::app::DomainSetpoint;
using ec::sentinel::app::RemoteCommand;
using ec::sentinel::app::RemoteMapper;
using ec::sentinel::app::RemoteMappingConfig;
using ec::sentinel::app::SentinelApp;
using ec::sentinel::app::SourceIntent;
using ec::control::ControlMode;
using ec::transport::CanFrame;

constexpr std::uint32_t index(Domain domain) { return ec::sentinel::app::domain_index(domain); }

struct FrameSpec {
    std::array<int, 4> ch{{0, 0, 0, 0}};
    int ch4{};
    Dr16Switch switch_left{Dr16Switch::mid};
    Dr16Switch switch_right{Dr16Switch::mid};
};

// 按官方 C 板示例的位域构造 18 B 帧: ch0–ch3 为 11 bit 位域减 1024, byte5 高位是拨杆,
// byte16..17 是第 5 通道 (小端, 减 1024).
std::array<std::uint8_t, 18> encode_frame(const FrameSpec& spec) {
    std::array<std::uint8_t, 18> buf{};
    std::uint64_t bits = 0;
    for (int i = 0; i < 4; ++i) {
        const auto raw = static_cast<std::uint64_t>(spec.ch[static_cast<std::size_t>(i)] + 1024) &
            0x7ffu;
        bits |= raw << (11 * i);
    }
    for (int i = 0; i < 6; ++i) {
        buf[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>((bits >> (8 * i)) & 0xffu);
    }
    buf[5] = static_cast<std::uint8_t>((buf[5] & 0x0fu) |
        ((static_cast<unsigned>(spec.switch_left) & 0x03u) << 6) |
        ((static_cast<unsigned>(spec.switch_right) & 0x03u) << 4));
    const auto raw4 = static_cast<std::uint16_t>(spec.ch4 + 1024);
    buf[16] = static_cast<std::uint8_t>(raw4 & 0xffu);
    buf[17] = static_cast<std::uint8_t>(raw4 >> 8);
    return buf;
}

RemoteMappingConfig mapping() {
    RemoteMappingConfig config{};
    config.stick_deadzone = 0.05f;
    config.chassis_vx_max_mps = 1.0f;
    config.chassis_vy_max_mps = 1.0f;
    config.chassis_wz_max_rad_s = 2.0f;
    config.yaw_rate_max_rad_s = 1.0f;
    config.pitch_rate_max_rad_s = 1.0f;
    config.friction_rad_s = 25.0f;
    config.trigger_rad_s = 10.0f;
    // 夹具: 中位 0, 高侧 >300, 低侧 <-300, 滞回 50, 低侧为发射机构层.
    config.ch4 = Ch4LayerConfig{0, 300, -300, 50, true};
    return config;
}

// 摇杆归一化: dr16_axis 把 ±660 映到 ±1, 这里给百分比值.
int stick(float normalized) { return static_cast<int>(normalized * 660.0f); }

struct Rig {
    Dr16Stream stream{};
    Dr16Input input{ec::Lifetime{100}};
    RemoteMapper mapper{mapping(), ec::Lifetime{100}};
    std::array<std::optional<float>, ec::sentinel::app::domain_count> actual{};
    Tick now{1000};
    std::uint32_t sequence{};

    Rig() {
        actual[index(Domain::main_yaw)] = 0.0f;
        actual[index(Domain::fire_yaw)] = 0.0f;
        actual[index(Domain::pitch)] = 0.0f;
        // 上电后首个 IDLE 只建立帧边界, 不交付样本.
        const std::uint8_t priming = 0;
        stream.push({&priming, 1});
        static_cast<void>(stream.on_idle());
    }
    // 分片投递: 一帧被拆成两段 DMA 块, 仅真实 IDLE 结束一帧.
    bool push(const FrameSpec& spec, Tick at, bool fragmented = true) {
        const auto frame = encode_frame(spec);
        if (fragmented) {
            stream.push({frame.data(), 9});
            stream.push({frame.data() + 9, frame.size() - 9});
        } else {
            stream.push(frame);
        }
        const auto sample = stream.on_idle();
        return sample ? input.accept(*sample, at) : false;
    }
    RemoteCommand step(std::uint32_t ms, float dt_s = 0.001f) {
        now = Tick{ms};
        const auto snapshot = input.snapshot(now);
        ++sequence;
        return mapper.step(snapshot, now, dt_s, actual);
    }
};

void raw_frame_contracts() {
    Dr16Stream stream;
    const auto frame = encode_frame(FrameSpec{{100, -200, 300, -400}, 500, Dr16Switch::up,
        Dr16Switch::down});
    // 启动后的首个 IDLE 只建立边界, 不交付样本.
    stream.push({frame.data(), 3});
    CHECK(!stream.on_idle().has_value());
    // 一帧被拆成两段 DMA 块: 分段之间没有 IDLE, 只有帧尾 IDLE 才交付.
    stream.push({frame.data(), 9});
    stream.push({frame.data() + 9, frame.size() - 9});
    const auto sample = stream.on_idle();
    CHECK(sample.has_value());
    CHECK(sample->ch0 == 100);
    CHECK(sample->ch1 == -200);
    CHECK(sample->ch2 == 300);
    CHECK(sample->ch3 == -400);
    CHECK(sample->ch4 == 500);
    CHECK(sample->switch_left == Dr16Switch::up);
    CHECK(sample->switch_right == Dr16Switch::down);

    // 半帧 + IDLE 被丢弃, 不当作有效样本.
    const auto rejected_before = stream.rejected;
    stream.push({frame.data(), 5});
    CHECK(!stream.on_idle().has_value());
    CHECK(stream.rejected == rejected_before + 1);

    // ch4 小端字节序: 0x0123 原始值 → 0x0123 - 1024.
    auto little = frame;
    little[16] = 0x23;
    little[17] = 0x01;
    stream.push(little);
    const auto endian_sample = stream.on_idle();
    CHECK(endian_sample.has_value());
    CHECK(endian_sample->ch4 == 0x0123 - 1024);

    // 通道超范围仍被拒绝 (ch0–ch3 的 ±660 判定不变), 但 ch4 不受该判定限制.
    auto wide = frame;
    wide[16] = 0xff;
    wide[17] = 0xff;
    stream.push(wide);
    const auto wide_sample = stream.on_idle();
    CHECK(wide_sample.has_value());
    // 与官方示例的 int16 算术一致: 原始 0xFFFF 减去 1024 后按 16 位有符号解释.
    CHECK(wide_sample->ch4 == -1025);

    // 掉线后首帧只建立基准, 不产生边沿; 时间回绕不误判为倒退.
    Dr16Input input{ec::Lifetime{100}};
    CHECK(input.accept(*sample, Tick{0xFFFFFFF0u}));
    CHECK(input.online(Tick{0xFFFFFFF5u}));
    const auto wrapped = input.snapshot(Tick{0x00000010u}); // 回绕后仍在有效期内
    CHECK(wrapped.has_value());
    // 回绕后的新帧被接受 (相对上一帧前进), 且不因为时间轴而拒绝.
    CHECK(input.accept(*sample, Tick{0x00000011u}));
    CHECK(input.snapshot(Tick{0x00000011u}).has_value());
}

void layer_contracts() {
    Rig rig;
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 0}, Tick{1000}));
    CHECK(rig.step(1000).layer == Ch4Layer::main_gimbal);
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 400}, Tick{1001}));
    CHECK(rig.step(1001).layer == Ch4Layer::fire_yaw);
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, -400}, Tick{1002}));
    CHECK(rig.step(1002).layer == Ch4Layer::launcher);
    // 中位附近抖动: 进入高侧需要越过滞回带, 回到中位立即生效.
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 310}, Tick{1003}));
    CHECK(rig.step(1003).layer == Ch4Layer::launcher); // 仍在滞回带内
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 360}, Tick{1004}));
    CHECK(rig.step(1004).layer == Ch4Layer::fire_yaw);
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 320}, Tick{1005}));
    CHECK(rig.step(1005).layer == Ch4Layer::fire_yaw); // 滞回带内保持
    CHECK(rig.push(FrameSpec{{0, 0, 0, 0}, 290}, Tick{1006}));
    CHECK(rig.step(1006).layer == Ch4Layer::main_gimbal);
    CHECK(rig.mapper.diagnostics().layer_changes >= 5);
}

void purpose_contracts() {
    Rig rig;
    // 本地来源, 普通运行档, 主云台层.
    // 首次进入角度用途也要先回中: 上电时摇杆仍在偏转不得直接接管 (C06).
    // 回中周期本身只把目标钉在当前姿态 (不回中周期不给目标).
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{998});
    auto command = rig.step(998, 0.1f);
    CHECK(command.intent == SourceIntent::local);
    CHECK(!command.targets.domains[index(Domain::main_yaw)].has_value());
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{999});
    command = rig.step(999, 0.1f);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(std::fabs(command.targets.domains[index(Domain::main_yaw)]->value) < 1e-6f);
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    command = rig.step(1000, 0.1f);
    CHECK(command.intent == SourceIntent::local);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    // 右 X 在主云台层给主 yaw 方向目标 (从有效实际姿态 0 起按 1 rad/s × 0.1 s 积分).
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(std::fabs(command.targets.domains[index(Domain::main_yaw)]->value - 0.05f) < 1e-4f);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());

    // 原 yaw 偏转直接切到发射机构层: 不得立即开摩擦轮.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1001});
    command = rig.step(1001, 0.1f);
    CHECK(command.layer == Ch4Layer::launcher);
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    CHECK(rig.mapper.diagnostics().recentre_required >= 1);
    // 回中一次后接管: 该周期仍不开轮.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1002});
    command = rig.step(1002, 0.1f);
    CHECK(!command.friction_on);
    // 再正向偏转才开轮.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1003});
    command = rig.step(1003, 0.1f);
    CHECK(command.friction_on);
    CHECK(command.targets.domains[index(Domain::friction_left)].has_value());
    CHECK(command.targets.domains[index(Domain::friction_right)].has_value());
    // 中位保持 (不锁存停止): 摩擦意图保留.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1004});
    command = rig.step(1004, 0.1f);
    CHECK(command.friction_on);
    // 负向关轮.
    rig.push(FrameSpec{{stick(-0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1005});
    command = rig.step(1005, 0.1f);
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());

    // 原 pitch 偏转切成拨弹: 切层那一刻右 Y 仍在偏转, 不得立即送弹.
    // 先回到主云台层, 回中一次接管 pitch 用途, 再偏转建立 pitch 角度目标.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1006});
    command = rig.step(1006, 0.1f);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    rig.push(FrameSpec{{0, stick(0.8f), 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1007});
    command = rig.step(1007, 0.1f);
    CHECK(command.targets.domains[index(Domain::pitch)].has_value());
    CHECK(command.targets.domains[index(Domain::pitch)]->value > 0.0f);
    // 直接切到发射机构层且右 Y 不松: 不得送弹.
    rig.push(FrameSpec{{0, stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1008});
    command = rig.step(1008, 0.1f);
    CHECK(command.layer == Ch4Layer::launcher);
    CHECK(!command.targets.domains[index(Domain::trigger)].has_value());
    // 切层只改变"当前操纵哪个通道": 已保持的 pitch 角度目标必须仍在快照里,
    // 否则完整快照会把缺省域当作撤销, "没操纵的轴保持"会变成"被撤输出".
    CHECK(command.targets.domains[index(Domain::pitch)].has_value());
    CHECK(command.targets.domains[index(Domain::pitch)]->value > 0.0f);
    // 回中一次后才接管: 该周期仍不送弹.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1009});
    command = rig.step(1009, 0.1f);
    CHECK(!command.targets.domains[index(Domain::trigger)].has_value());
    // 再正向偏转才送弹.
    rig.push(FrameSpec{{0, stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1010});
    command = rig.step(1010, 0.1f);
    CHECK(command.targets.domains[index(Domain::trigger)].has_value());
    CHECK(command.targets.domains[index(Domain::trigger)]->value > 0.0f);
    // 回中立即取消拨弹 (不锁存).
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1011});
    command = rig.step(1011, 0.1f);
    CHECK(!command.targets.domains[index(Domain::trigger)].has_value());
    // 负向人工反转.
    rig.push(FrameSpec{{0, stick(-0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1012});
    command = rig.step(1012, 0.1f);
    CHECK(command.targets.domains[index(Domain::trigger)].has_value());
    CHECK(command.targets.domains[index(Domain::trigger)]->value < 0.0f);
}

void gear_and_hold_contracts() {
    Rig rig;
    // 主云台层: 先回中一次接管角度用途 (首次进入同样要求回中), 再偏转建立主 yaw 目标.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{999});
    rig.step(999, 0.1f);
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    auto command = rig.step(1000, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    const float held = command.targets.domains[index(Domain::main_yaw)]->value;
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1001});
    command = rig.step(1001, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == held);
    // 右拨杆上 = 本地底盘旋转操作: 右 X 改给 wz, 不再给 yaw 方向目标.
    // 该通道换用途同样先回中: 回中周期只解除等待, wz 仍为 0.
    rig.push(FrameSpec{{stick(0.5f), 0, stick(0.25f), stick(-0.5f)}, 0, Dr16Switch::up,
                  Dr16Switch::up},
        Tick{1002});
    command = rig.step(1002, 0.1f);
    CHECK(command.rotation_gear);
    CHECK(command.chassis_active);
    CHECK(std::fabs(command.targets.chassis->wz_rad_s) < 1e-6f);
    rig.push(FrameSpec{{0, 0, stick(0.25f), stick(-0.5f)}, 0, Dr16Switch::up, Dr16Switch::up},
        Tick{1003});
    command = rig.step(1003, 0.1f);
    rig.push(FrameSpec{{stick(0.5f), 0, stick(0.25f), stick(-0.5f)}, 0, Dr16Switch::up,
                  Dr16Switch::up},
        Tick{1004});
    command = rig.step(1004, 0.1f);
    CHECK(command.rotation_gear);
    CHECK(command.chassis_active);
    CHECK(std::fabs(command.targets.chassis->wz_rad_s - 1.0f) < 1e-4f);
    CHECK(std::fabs(command.targets.chassis->vx_mps - (-0.5f)) < 1e-4f);
    CHECK(std::fabs(command.targets.chassis->vy_mps - 0.25f) < 1e-4f);
    // 已保持的主 yaw 角度目标仍在快照里 (旋转档占用右 X 不等于撤销该轴).
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(command.targets.domains[index(Domain::main_yaw)]->value == held);
    // 旋转档下切到发射机构层也不开摩擦 (右 X 已被旋转档占用).
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::up}, Tick{1005});
    command = rig.step(1005, 0.1f);
    CHECK(command.rotation_gear);
    CHECK(!command.friction_on);
}

void source_and_loss_contracts() {
    Rig rig;
    // 建立摩擦轮开启意图.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    rig.step(1000, 0.1f);
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1001});
    auto command = rig.step(1001, 0.1f);
    CHECK(command.friction_on);
    // 跨层保留: 切到主云台层仍然是开轮.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1002});
    command = rig.step(1002, 0.1f);
    CHECK(command.layer == Ch4Layer::main_gimbal);
    CHECK(command.friction_on);
    CHECK(command.targets.domains[index(Domain::friction_left)].has_value());
    // 换源 (左拨杆中) 清掉人工摩擦意图, 且不给本地目标.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::mid, Dr16Switch::mid}, Tick{1003});
    command = rig.step(1003, 0.1f);
    CHECK(command.intent == SourceIntent::standby);
    CHECK(!command.friction_on);
    CHECK(!command.chassis_active);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    // 回到本地来源: 摩擦不自动恢复; 改用途后的摩擦开关通道同样需要先回中一次.
    rig.push(FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1004});
    command = rig.step(1004, 0.1f);
    CHECK(command.intent == SourceIntent::local);
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    // 失联: 清除摩擦/拨弹意图, 目标时效由输入层判定.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid}, Tick{1005});
    command = rig.step(1005, 0.1f);
    CHECK(command.friction_on);
    command = rig.step(1200, 0.1f); // 超过 remote_ttl
    CHECK(rig.mapper.diagnostics().link_lost);
    CHECK(!command.friction_on);
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    // 人工停止: 右拨杆下锁存, 不给任何本地目标; 拨回运行档后恢复资格.
    rig.push(FrameSpec{{stick(0.5f), stick(0.5f), stick(0.5f), stick(0.5f)}, 0, Dr16Switch::up,
                  Dr16Switch::down},
        Tick{1300});
    command = rig.step(1300, 0.1f);
    CHECK(command.manual_stop);
    CHECK(!command.chassis_active);
    CHECK(!command.targets.domains[index(Domain::main_yaw)].has_value());
    CHECK(!command.targets.domains[index(Domain::friction_left)].has_value());
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1301});
    command = rig.step(1301, 0.1f);
    CHECK(!command.manual_stop);
    CHECK(command.chassis_active);
}

// 全链路: 原始 18 B 帧 → 映射 → 应用采用目标.
struct Chain {
    Rig remote{};
    SentinelApp app{ec::sentinel::app::synthetic_config(), ConfigAudience::offline_test};
    ec::sentinel::app::AppInputs inputs{};
    Chain() { inputs.link.locally_ready = true; }
    void step(std::uint32_t ms, const FrameSpec& spec, float dt_s = 0.01f) {
        remote.push(spec, Tick{ms});
        const auto command = remote.step(ms, dt_s);
        inputs.epochs = ec::sentinel::app::Epochs{1, 0};
        inputs.link.remote_online = true;
        inputs.link.remote_sampled_at = remote.now;
        inputs.link.switch_left = spec.switch_left;
        inputs.link.switch_right = spec.switch_right;
        inputs.aux.launch_permit_valid = true;
        inputs.aux.launch_permit = true;
        inputs.aux.launch_sampled_at = Tick{ms};
        inputs.aux.launch_ttl = ec::Lifetime{1000};
        for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
            inputs.feedback[i].position_rad = remote.actual[i]
                ? ec::Sample<float>{*remote.actual[i], Tick{ms}, ec::Validity::valid}
                : ec::Sample<float>{};
        }
        app.update(Tick{ms}, inputs);
        if (command.intent == SourceIntent::local && !command.manual_stop) {
            CHECK(app.accept_targets(command.targets, Tick{ms}) == AcceptResult::accepted);
        }
        app.update(Tick{ms}, inputs);
    }
};

void chain_contracts() {
    Chain chain;
    // 主云台层: 首次进入角度用途先回中 (上电时摇杆偏转不得直接接管), 之后右 X 才给主 yaw 目标.
    chain.step(999, FrameSpec{{0, 0, stick(0.5f), stick(0.5f)}, 0, Dr16Switch::up,
                   Dr16Switch::mid});
    chain.step(1000, FrameSpec{{stick(0.5f), 0, stick(0.5f), stick(0.5f)}, 0, Dr16Switch::up,
                   Dr16Switch::mid});
    const auto twist = chain.app.requested_twist();
    CHECK(twist.has_value());
    CHECK(std::fabs(twist->vx_mps - 0.5f) < 1e-4f);
    CHECK(twist->wz_rad_s == 0.0f);
    CHECK(chain.app.target(Domain::main_yaw).has_value());
    CHECK(chain.app.snapshot().permission.source == ec::sentinel::app::Source::remote);
    CHECK(!chain.app.snapshot().permission.manual_stop);
    // 发射机构层: 摩擦开关与拨弹经应用采用, 目标值是配置转速.
    chain.step(1001, FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    chain.step(1002, FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    chain.step(1003, FrameSpec{{stick(0.5f), 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    CHECK(chain.app.target(Domain::friction_left).has_value());
    CHECK(chain.app.target(Domain::friction_right).has_value());
    chain.step(1004, FrameSpec{{0, stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    chain.step(1005, FrameSpec{{0, 0, 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    chain.step(1006, FrameSpec{{0, stick(0.8f), 0, 0}, -400, Dr16Switch::up, Dr16Switch::mid});
    CHECK(chain.app.target(Domain::trigger).has_value());
    CHECK(chain.app.target(Domain::trigger)->value > 0.0f);
    // 右拨杆下: 人工停止经应用生效, 不再有本地目标.
    chain.step(1007, FrameSpec{{0, stick(0.8f), stick(0.8f), stick(0.8f)}, -400, Dr16Switch::up,
                   Dr16Switch::down});
    CHECK(chain.app.snapshot().permission.manual_stop);
    CHECK(!chain.app.snapshot().permission.authority);
}
} // 测试辅助

// 未标定的映射量程 (全 0) 不得产生"有效零目标": 零值不等于已标定的零运动意图.
void unconfigured_mapping_contracts() {
    RemoteMappingConfig empty{};
    empty.stick_deadzone = 0.05f;
    RemoteMapper mapper{empty, ec::Lifetime{100}};
    std::array<std::optional<float>, ec::sentinel::app::domain_count> actual{};
    actual[index(Domain::main_yaw)] = 0.0f;
    actual[index(Domain::fire_yaw)] = 0.0f;
    actual[index(Domain::pitch)] = 0.0f;
    Dr16Snapshot neutral{};
    neutral.switch_left = Dr16Switch::up;
    neutral.switch_right = Dr16Switch::mid;
    const auto idle = mapper.step(neutral, Tick{1000}, 0.1f, actual);
    CHECK(idle.intent == SourceIntent::local);
    CHECK(!idle.chassis_active);
    CHECK(!idle.friction_on);
    for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
        CHECK(!idle.targets.domains[i].has_value());
    }
    // 即使摇杆偏转也不产生目标 (未标定量程).
    Dr16Snapshot moved_sample{};
    moved_sample.switch_left = Dr16Switch::up;
    moved_sample.switch_right = Dr16Switch::mid;
    moved_sample.axis = {0.5f, 0.5f, 0.5f, 0.5f};
    const auto moved = mapper.step(moved_sample, Tick{1001}, 0.1f, actual);
    CHECK(!moved.chassis_active);
    CHECK(!moved.targets.chassis.has_value());
    CHECK(!moved.friction_on);
    for (std::size_t i = 0; i < ec::sentinel::app::domain_count; ++i) {
        CHECK(!moved.targets.domains[i].has_value());
    }
}

// 切回一个已保持角度的用途时: "先回中"只拦新操纵增量, 不取消已有保持角.
void held_angle_on_purpose_return_contracts() {
    Rig rig;
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{998});
    static_cast<void>(rig.step(998, 0.1f));
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{999});
    auto command = rig.step(999, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    const float held = command.targets.domains[index(Domain::main_yaw)]->value;
    // 切到 fire_yaw 层: 主云台保持角仍在快照里.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 400, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    command = rig.step(1000, 0.1f);
    CHECK(command.layer == Ch4Layer::fire_yaw);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    // x 未回中时切回主云台层: 新增量被回中门拦住, 已有保持角不得消失.
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1001});
    command = rig.step(1001, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    if (command.targets.domains[index(Domain::main_yaw)]) {
        CHECK(command.targets.domains[index(Domain::main_yaw)]->value == held);
    }
    // 回中周期只解除回中门, 保持不变.
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1002});
    command = rig.step(1002, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    if (command.targets.domains[index(Domain::main_yaw)]) {
        CHECK(command.targets.domains[index(Domain::main_yaw)]->value == held);
    }
    // 回中后再次偏转才接受新增量 (从保持角继续积分).
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1003});
    command = rig.step(1003, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    if (command.targets.domains[index(Domain::main_yaw)]) {
        CHECK(command.targets.domains[index(Domain::main_yaw)]->value > held);
    }
}

// 参考语义改变 → 丢弃该域的保持角; 无有效实际姿态时本周期不重建.
void drop_held_angle_contracts() {
    Rig rig;
    rig.push(FrameSpec{{0, 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{998});
    static_cast<void>(rig.step(998, 0.1f));
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{999});
    auto command = rig.step(999, 0.1f);
    CHECK(command.targets.domains[index(Domain::main_yaw)].has_value());
    rig.mapper.drop_held_angle(Domain::main_yaw);
    rig.actual[index(Domain::main_yaw)] = std::nullopt; // 新参考下实际姿态不可用 → 不能重建
    rig.push(FrameSpec{{stick(0.5f), 0, 0, 0}, 0, Dr16Switch::up, Dr16Switch::mid}, Tick{1000});
    command = rig.step(1000, 0.1f);
    CHECK(!command.targets.domains[index(Domain::main_yaw)].has_value());
}

int main() {
    raw_frame_contracts();
    layer_contracts();
    purpose_contracts();
    unconfigured_mapping_contracts();
    held_angle_on_purpose_return_contracts();
    drop_held_angle_contracts();
    gear_and_hold_contracts();
    source_and_loss_contracts();
    chain_contracts();
    std::puts("sentinel app remote contracts passed");
    return 0;
}
