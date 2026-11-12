// 哨兵正式 APP 的主机软件后端 (O5 离线模式).
//  - 不打开任何串口/CAN: 设备反馈由 --inject 文件按 MCU 时间注入 (测试入口, 不属于业务协议);
//  - 业务请求走真实 0x7e 分帧: stdin 收字节 → UartFrameDecoder → SentinelHostSession → stdout;
//  - 每个请求推进一个 1 ms 控制周期, 使时序完全确定;
//  - --golden 打印 C++ 侧编码向量供 Python 交叉核对.
// 业务协议本身不提供任意注入/内存/CAN 写入能力.
#include "ec/transport/uart_frame.hpp"
#include "sentinel_app.hpp"
#include "sentinel_config.hpp"
#include "sentinel_cycle.hpp"
#include "sentinel_execution.hpp"
#include "sentinel_params.hpp"
#include "sentinel_perception.hpp"
#include "sentinel_protocol.hpp"
#include "sentinel_link.hpp"
#include "sentinel_pitch_stream_link.hpp"
#include "sim_plant.hpp"
#include "mechanism_fixture.hpp"
#include <array>
#include <charconv>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#endif

namespace {
using namespace ec;
using namespace ec::sentinel::app;
using ec::transport::CanFrame;

// 软件总线与合成负载共用主机侧夹具 (robots/sentinel/host/sim_plant.hpp):
// 只记录帧, 不模拟任何真实总线时序或电调行为.
using ec::sentinel::host::SimPlant;
using ec::sentinel::host::SoftwareBus;

struct Injection {
    std::uint32_t at_ms{};
    int kind{}; // 0 = imu, 1 = dji, 2 = dm
    Domain domain{Domain::pitch};
    float a[6]{};
};

// Explicit virtual-only settings. None of these values are deployment defaults.
SentinelConfig host_config(ec::sentinel::host::Mechanism fixture, bool identification) {
    auto config = ec::sentinel::host::partial_fixture_config(fixture);
    if (!identification) { return config; }
    config.independent_motor_bench = true;
    config.serial.reset();
    for (auto& slot : config.dji) { slot.installed = false; }
    config.dm[0].bus = 2;
    config.dm[0].can_id = 1;
    config.dm[0].master_id = 0x011;
    config.dm[0].mit_kp = 12.0f;
    config.dm[0].mit_kd = 0.4f;
    config.joints[2] = JointGeometryConfig{Domain::pitch, 0.0f, -0.4f, 0.4f};
    config.timing.target_ttl = Lifetime{100};
    config.timing.host_link_ttl = Lifetime{200};
    config.timing.max_dt_s = 0.005f;
    PitchIdentificationPolicy policy{};
    policy.min_rad = -0.2f;
    policy.max_rad = 0.2f;
    policy.max_amplitude_nm = 0.1f;
    policy.max_expected_torque_nm = 3.0f;
    policy.max_velocity_rad_s = 1.5f;
    policy.max_tracking_error_rad = 0.06f;
    policy.min_half_period_us = 50000;
    policy.max_duration_us = 10000000;
    policy.max_feedback_age_us = 10000;
    policy.max_step_us = 6000;
    policy.require_capture = true;
    policy.max_position_amplitude_rad = 0.01f;
    policy.max_position_speed_rad_s = 0.1f;
    config.pitch_identification = policy;
    return config;
}

struct Host {
    ec::sentinel::host::Mechanism fixture;
    SentinelConfig config;
    bool identification_enabled{};
    ec::sentinel::recording::Recorder recorder{};
    PitchStreamLink stream{recorder};
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    SentinelApp app;
    SentinelExecutor exec;
    SentinelPerception perception;
    ParamStore params;
    SentinelHostSession session;
    // 与板级共用同一段周期逻辑 (SentinelCycle): 主机只替换"总线"与样本来源,
    // 不再维护与固件平行的第二条周期管线.
    SentinelCycle cycle;
    Tick now{1};
    std::vector<Injection> injections{};
    std::size_t next_injection{};
    // --plant: 用合成负载闭合回路 (命令帧 → 运动 → 反馈帧), 而不是注入固定反馈.
    // 这是主机侧夹具, 不是实车动力学模型, 结论不外推到稳定性或动力参数.
    bool plant_enabled{};
    SimPlant plant{};
    // 离线后端没有真实 DT7: 用一份恒定的运行档样本维持首启许可与来源选择.
    // 来源仍由拨杆位置决定; 这是夹具输入, 不代表任何真实遥控链路.
    drivers::Dr16Sample remote_sample{};

    explicit Host(ec::sentinel::host::Mechanism part = ec::sentinel::host::Mechanism::all,
        bool identification = false)
        : fixture(part), config(host_config(part, identification)), identification_enabled(identification), app(config, ConfigAudience::offline_test, config.independent_motor_bench
              ? StartupPolicy::maintenance_only : StartupPolicy::remote_required),
          exec(config, ConfigAudience::offline_test, bus1, bus2),
          perception(config, ConfigAudience::offline_test), params(config),
          session(app, params, &exec, &perception, config.timing.host_link_ttl),
          cycle(app, exec, perception, RemoteMappingConfig{}, config.timing.remote_ttl) {
        cycle.bind_experiment(session.experiment_controller());
        exec.bind_pitch_recorder(&recorder);
        if (identification_enabled) {
            plant.dm.position_rad = 0.1f;
        }
        // 映射量程保持未标定 (全 0): 离线后端不生成任何本地操纵目标,
        // 与部署配置"缺项不产生有效零目标"的契约一致.
        remote_sample.switch_left = drivers::Dr16Switch::down; // 上位机来源
        remote_sample.switch_right = drivers::Dr16Switch::mid;
    }

    void cycle_once() {
        // 先收反馈帧 (合成负载/注入), 再跑与板级相同的 control/commit 顺序.
        if (plant_enabled) {
            plant.emit_feedback(config,
                [this](std::uint8_t bus, const CanFrame& frame, Tick at) {
                    if (ec::sentinel::host::selected_feedback(fixture, config, bus, frame.id)) {
                        exec.route(bus, frame, at, ec::MicroTick{at.ms*1000u});
                        if (identification_enabled) {
                            recorder.can(bus, frame, {at.ms*1000u}, {at.ms*1000u}, 0);
                        }
                    }
                },
                now);
        }
        CycleInputs in{};
        in.now = now;
        in.now_us = MicroTick{now.ms * 1000u}; // Explicit synthetic microsecond clock.
        in.dt_s = 0.001f;
        in.remote_sample = remote_sample;
        in.remote_received_at = now;
        in.locally_ready = true;
        // 会话/链路状态由业务协议显式提供; 未授权会话不产生来源资格.
        const auto link = session.link_state(now);
        const auto role = session.role_source();
        in.host_link_alive = link.alive && role == Source::host;
        in.host_request = link.request && role == Source::host;
        in.host_session_epoch = link.epoch;
        in.maintenance_link_alive = link.alive && role == Source::maintenance;
        in.maintenance_request = link.request && role == Source::maintenance;
        in.maintenance_session_epoch = link.epoch;
        // 外部约束: 发射许可由业务协议 permit 显式设置到应用内部, 不在这里注入观察.
        in.aux = AuxState{};
        static_cast<void>(cycle.control(in));
        bus1.clear();
        bus2.clear();
        cycle.commit(in);
        recorder.advance(*in.now_us);
        if (const auto axis = recorder.autotune_axis(); axis >= 0) {
            if (const auto pair = exec.autotune_trace(static_cast<Domain>(axis), now, app)) {
                recorder.push_bundle(pair->state, pair->aux, *in.now_us);
            }
        }
        if (identification_enabled) {
            recorder.advance({now.ms*1000u});
            if (const auto trace = exec.trace(Domain::pitch, now, app)) {
                recorder.push(*trace, {now.ms*1000u});
            }
        }
        if (plant_enabled) { plant.integrate(config, bus1, bus2, 0.001f); }
    }

    void apply_injections() {
        while (next_injection < injections.size() && injections[next_injection].at_ms <= now.ms) {
            const auto& item = injections[next_injection++];
            if (item.kind == 0) {
                // 注入 = 传感器样本: 采样时刻与摄入时刻都取当前 MCU 时间.
                perception.submit_imu({item.a[3], item.a[4], item.a[5]},
                    {item.a[0], item.a[1], item.a[2]}, now, now);
            } else if (item.kind == 1) {
                const auto slot = app.dji_slot_for(item.domain);
                if (!slot) { continue; }
                const auto& cfg = config.dji[*slot];
                const auto protocol = drivers::dji_protocol(cfg.mode);
                if (!protocol) { continue; }
                CanFrame frame{};
                frame.id = static_cast<std::uint16_t>(protocol->rx_base + cfg.device_id);
                frame.size = 8;
                float angle = std::fmod(item.a[0], 2.0f * 3.14159265358979f);
                if (angle < 0.0f) { angle += 2.0f * 3.14159265358979f; }
                const auto encoder = static_cast<unsigned>(angle / (2.0f * 3.14159265358979f) * 8192.0f);
                const auto speed = static_cast<std::int16_t>(item.a[1] * 60.0f / (2.0f * 3.14159265358979f));
                frame.data[0] = static_cast<std::uint8_t>(encoder >> 8);
                frame.data[1] = static_cast<std::uint8_t>(encoder & 0xffu);
                frame.data[2] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) >> 8);
                frame.data[3] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) & 0xffu);
                frame.data[6] = 40;
                exec.route(cfg.bus, frame, now, ec::MicroTick{now.ms*1000u});
            } else {
                const auto& slot = config.dm[0];
                if (!slot.installed || !slot.limits || !slot.can_id || !slot.master_id) { continue; }
                const auto position = drivers::dm_encode_position(*slot.limits, item.a[0]);
                const auto velocity = drivers::dm_encode_velocity(*slot.limits, item.a[1]);
                const auto torque = drivers::dm_encode_torque(*slot.limits, item.a[2]);
                if (!position || !velocity || !torque) { continue; }
                CanFrame frame{};
                frame.id = *slot.master_id;
                frame.size = 8;
                frame.data[0] = static_cast<std::uint8_t>((*slot.can_id & 0x0fu) | (1u << 4));
                frame.data[1] = static_cast<std::uint8_t>(*position >> 8);
                frame.data[2] = static_cast<std::uint8_t>(*position & 0xffu);
                frame.data[3] = static_cast<std::uint8_t>(*velocity >> 8);
                frame.data[4] = static_cast<std::uint8_t>(*velocity & 0xffu);
                frame.data[5] = static_cast<std::uint8_t>(*torque >> 8);
                frame.data[6] = static_cast<std::uint8_t>(*torque & 0xffu);
                exec.route(slot.bus, frame, now, ec::MicroTick{now.ms*1000u});
            }
        }
    }

    // 处理一条请求, 返回响应长度 (0 表示本帧不产生响应).
    // 与板级同一受控先后关系: 本周期请求先按当前时刻处理, 再推进控制/提交; 两者共用同一个 now.
    std::size_t handle_payload(std::span<const std::uint8_t> request,
        std::span<std::uint8_t> out) {
        apply_injections();
        const auto size = session.handle(request, now, out);
        if (!identification_enabled) { cycle_once(); }
        return size;
    }

    std::size_t handle(const transport::UartFrame& frame, std::span<std::uint8_t> out) {
        if (identification_enabled && frame.type == PitchStreamLink::request_type) {
            return stream.request(frame, out, {now.ms*1000u},
                app.authority().holds() && app.authority().source() == Source::maintenance);
        }
        return handle_payload({frame.payload.data(), frame.size}, out);
    }
};

[[nodiscard]] bool parse_domain(const std::string& text, Domain& out) {
    for (std::size_t i = 0; i < domain_count; ++i) {
        const auto name = domain_name(static_cast<Domain>(i));
        if (text == name) { out = static_cast<Domain>(i); return true; }
    }
    return false;
}

void load_injections(const std::string& path, Host& host) {
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') { continue; }
        std::istringstream stream(line);
        std::string kind;
        Injection item{};
        stream >> kind >> item.at_ms;
        if (kind == "imu") {
            item.kind = 0;
            for (auto& value : item.a) { stream >> value; }
        } else if (kind == "dji") {
            item.kind = 1;
            std::string domain;
            stream >> domain;
            if (!parse_domain(domain, item.domain)) { continue; }
            stream >> item.a[0] >> item.a[1];
        } else if (kind == "dm") {
            item.kind = 2;
            stream >> item.a[0] >> item.a[1] >> item.a[2];
        } else {
            continue;
        }
        host.injections.push_back(item);
    }
}

void emit_golden(Host& host) {
    std::array<std::uint8_t, protocol_payload_max> request{};
    std::array<std::uint8_t, protocol_payload_max> response{};
    // 每条请求都经过真实周期编排 (注入 → 周期 → 会话): 与主循环同一条路径,
    // 因此黄金向量里包含真实来源仲裁与设备反馈的影响, 不是"直接调会话"的孤立结果.
    host.plant_enabled = true;
    for (int warmup = 0; warmup < 5; ++warmup) {
        ++host.now.ms;
        host.cycle_once();
    }
    const auto exchange = [&](std::span<const std::uint8_t> req) {
        ++host.now.ms;
        return host.handle_payload(req, response);
    };
    const auto print = [&](const char* label, std::span<const std::uint8_t> bytes) {
        std::cout << label << ' ';
        for (auto byte : bytes) {
            char text[4]{};
            std::snprintf(text, sizeof(text), "%02x", byte);
            std::cout << text;
        }
        std::cout << '\n';
    };
    {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::hello));
        w.u32(0x11223344u);
        print("REQ hello", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES hello", {response.data(), size});
    }
    const auto session = host.session.session_id();
    {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::authorize));
        w.u32(session);
        w.u32(1);
        w.u8(1);
        w.u8(static_cast<std::uint8_t>(Source::host));
        print("REQ authorize", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES authorize", {response.data(), size});
    }
    {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::targets));
        w.u32(session);
        w.u32(2);
        w.u32(host.app.config_epoch());
        w.u32(host.app.reference_epoch());
        w.u8(1);
        w.u8(static_cast<std::uint8_t>(Domain::pitch));
        w.u8(static_cast<std::uint8_t>(control::ControlMode::position));
        w.u8(static_cast<std::uint8_t>(YawFrame::joint));
        w.f32(0.1f);
        w.u16(0); // 源年龄: 0 = 刚产生
        w.u8(1);
        w.f32(0.2f);
        w.f32(0.0f);
        w.f32(0.0f);
        w.u16(0);
        print("REQ targets", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES targets", {response.data(), size});
    }
    {
        // 对齐带 kind: 0 = 机械关节参考, 1 = 惯性/odom 对齐偏移.
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::align));
        w.u32(session);
        w.u32(3);
        w.u8(static_cast<std::uint8_t>(Domain::pitch));
        w.u8(0);
        w.f32(0.0f);
        w.u32(host.app.reference_epoch());
        print("REQ align", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES align", {response.data(), size});
    }
    {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::param_write));
        w.u32(session);
        w.u32(4);
        w.u16(param_id(0, 1)); // 第一个 DJI 槽的速度环 Kp
        w.f32(1.5f);
        print("REQ param_write", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES param_write", {response.data(), size});
    }
    {
        // 尚未实现生效路径的参数: 必须明确报 unsupported, 不用新数值冒充已生效.
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::param_write));
        w.u32(session);
        w.u32(5);
        w.u16(param_id(0, 5)); // output_limit_raw: 设备构造期换算, 本版无在线生效路径
        w.f32(10000.0f);
        print("REQ param_write_unsupported", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES param_write_unsupported", {response.data(), size});
    }
    {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::param_read));
        w.u32(session);
        w.u32(6);
        w.u16(0);
        w.u16(param_id(0, 1));
        print("REQ param_read", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES param_read", {response.data(), size});
    }
    {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::permit));
        w.u32(session);
        w.u32(7);
        w.u8(1);
        w.u16(500);
        print("REQ permit", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES permit", {response.data(), size});
    }
    {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::status));
        w.u32(session);
        w.u32(8); w.u8(1);
        print("REQ status", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES status", {response.data(), size});
    }
    {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::diagnostics));
        w.u32(session);
        w.u32(9); w.u8(0);
        print("REQ diagnostics", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print("RES diagnostics", {response.data(), size});
    }
    for (const auto page : {StatusPage::fast, StatusPage::requests}) {
        PayloadWriter w{request};
        w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(MessageType::status));
        w.u32(session); w.u32(page == StatusPage::fast ? 10u : 11u);
        w.u8(static_cast<std::uint8_t>(page));
        print(page == StatusPage::fast ? "REQ fast" : "REQ requests", {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print(page == StatusPage::fast ? "RES fast" : "RES requests", {response.data(), size});
    }
    std::uint32_t seq = 11;
    for (const auto page : {1, 2, 10, 11, 12, 13, 14, 15, 16, 17, 20}) {
        PayloadWriter w{request};
        w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(MessageType::diagnostics));
        w.u32(session); w.u32(++seq); w.u8(static_cast<std::uint8_t>(page));
        const auto request_label = std::string{"REQ diag"}+std::to_string(page);
        const auto response_label = std::string{"RES diag"}+std::to_string(page);
        print(request_label.c_str(), {request.data(), w.size()});
        const auto size = exchange({request.data(), w.size()});
        print(response_label.c_str(), {response.data(), size});
    }
}
} // 命名空间

int main(int argc, char** argv) {
    // stdin/stdout 必须无缓冲且为二进制: 交互式主机客户端按请求/响应成对通信,
    // 不能等缓冲区满; 文本模式的 LF→CRLF 转换会破坏帧字节流.
    std::setvbuf(stdin, nullptr, _IONBF, 0);
    std::setvbuf(stdout, nullptr, _IONBF, 0);
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    bool golden = false, plant = false, identification = false;
    std::string injection_path;
    auto fixture = ec::sentinel::host::Mechanism::all;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--golden") { golden = true; }
        else if (arg == "--inject" && i + 1 < argc) { injection_path = argv[++i]; }
        else if (arg == "--plant") { plant = true; }
        else if (arg == "--pitch-identification") { identification = true; }
        else if (arg == "--fixture" && i+1 < argc) {
            const auto part = ec::sentinel::host::mechanism(argv[++i]);
            if (!part) { std::cerr << "unknown fixture\n"; return 2; }
            fixture = *part;
        }
        else if (arg == "--help") {
            std::cout << "usage: sentinel_app_host [--golden] [--inject FILE] [--plant] [--fixture all|chassis|gimbal|shooter|bench] [--pitch-identification]\n";
            return 0;
        } else { std::cerr << "invalid option: " << arg << '\n'; return 2; }
    }
    if (identification && (!plant || fixture != ec::sentinel::host::Mechanism::gimbal ||
        golden || !injection_path.empty())) {
        std::cerr << "pitch identification requires --plant --fixture gimbal, no golden/injection\n";
        return 2;
    }
    Host host{fixture, identification};
    host.plant_enabled = plant;
    if (!injection_path.empty()) { load_injections(injection_path, host); }
    if (golden) {
        emit_golden(host);
        return 0;
    }
    transport::UartFrameDecoder decoder{};
    std::array<std::uint8_t, protocol_payload_max> response{};
    // 同一个四槽出口; 管道没有配置物理 UART 带宽, 明确不启用串口低频间隔.
    ReplyMux replies{0};
    if (identification) {
#ifdef _WIN32
        std::cerr << "realtime offline streaming backend requires POSIX poll\n";
        return 2;
#else
        // Independent 1 ms virtual control clock: silence/EOF does not freeze
        // firmware watchdogs. This is a software fixture, not a realtime claim.
        const int flags = ::fcntl(STDOUT_FILENO, F_GETFL);
        if (flags < 0 || ::fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) { return 3; }
        std::array<std::uint8_t, transport::uart_wire_max> pending{};
        std::size_t pending_size{}, pending_offset{};
        bool output_error{};
        const auto flush_output = [&]() {
            if (!pending_size) { return; }
            const auto wrote = ::write(STDOUT_FILENO, pending.data() + pending_offset,
                pending_size - pending_offset);
            if (wrote < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) { output_error = true; }
                return;
            }
            if (!wrote) { output_error = true; return; }
            pending_offset += static_cast<std::size_t>(wrote);
            if (pending_offset == pending_size) { pending_size = pending_offset = 0; }
        };
        const auto start = std::chrono::steady_clock::now();
        const auto origin_ms = host.now.ms;
        for (;;) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();
            const auto desired = origin_ms + static_cast<std::uint32_t>(elapsed);
            if (elapsed < 0 || elapsed > 1800000 || desired - host.now.ms > 20) {
                std::cerr << "offline streaming backend missed its bounded clock budget\n";
                return 4; // Fail visibly; never run unbounded catch-up or fake continuous sampling.
            }
            while (host.now.ms < desired) { ++host.now.ms; host.cycle_once(); }
            pollfd input{STDIN_FILENO, POLLIN, 0};
            const auto polled = ::poll(&input, 1, 1);
            if (polled < 0) { if (errno == EINTR) { continue; } return 3; }
            if (polled && (input.revents & POLLIN)) {
                std::array<std::uint8_t, 256> bytes{};
                const auto count = ::read(STDIN_FILENO, bytes.data(), bytes.size());
                if (count <= 0) { return count == 0 ? 0 : 3; }
                for (std::size_t i = 0; i < static_cast<std::size_t>(count); ++i) {
                    const auto frame = decoder.push(bytes[i]);
                    if (!frame) { continue; }
                    host.session.set_frame_rejects(decoder.rejected());
                    host.session.set_tx_diagnostics(replies.diagnostics());
                    const auto size = host.handle(*frame, response);
                    if (size) { static_cast<void>(replies.enqueue(*frame, {response.data(), size})); }
                }
            } else if (polled && (input.revents & (POLLHUP | POLLERR | POLLNVAL))) { return 0; }
            flush_output();
            if (output_error) { return 3; }
            const auto send = [&](std::span<const std::uint8_t> wire) {
                if (pending_size || wire.size() > pending.size()) { return false; }
                std::copy(wire.begin(), wire.end(), pending.begin());
                pending_size = wire.size();
                pending_offset = 0;
                flush_output();
                return !output_error; // One fixed queued frame, never a blocking fwrite.
            };
            const auto replied = replies.service(host.now, send);
            if (!replied && !replies.pending()) {
                static_cast<void>(host.stream.service({host.now.ms * 1000u}, send));
            }
        }
#endif
    }
    int byte = 0;
    while ((byte = std::getchar()) != EOF) {
        const auto frame = decoder.push(static_cast<std::uint8_t>(byte));
        if (!frame) { continue; }
        ++host.now.ms;
        host.session.set_frame_rejects(decoder.rejected());
        host.session.set_tx_diagnostics(replies.diagnostics());
        const auto size = host.handle(*frame, response);
        if (size == 0) { continue; }
        static_cast<void>(replies.enqueue(*frame, {response.data(), size}));
        static_cast<void>(replies.service(host.now, [](std::span<const std::uint8_t> wire) {
            return std::fwrite(wire.data(), 1, wire.size(), stdout) == wire.size();
        }));
    }
    std::fflush(stdout);
    return 0;
}
