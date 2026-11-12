// O5 业务协议与会话契约: 分片/粘包/CRC/长度/版本错、截断、重放/乱序、超时、
// 参考/配置过期、重复非幂等请求、会话失效、接受与实际执行状态分开、参数读写.
// 帧层用真实 UartFrame 编码器与解码器, 不直接调用处理函数冒充"链路已通".
#include "ec/transport/uart_frame.hpp"
#include "sentinel_app.hpp"
#include "sentinel_config.hpp"
#include "sentinel_execution.hpp"
#include "sentinel_params.hpp"
#include "sentinel_perception.hpp"
#include "sentinel_protocol.hpp"
#include "sentinel_link.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <numbers>
#include <string_view>
#include <vector>

using namespace ec;
using namespace ec::sentinel::app;
using ec::transport::CanFrame;

namespace {
int failures = 0;
void check(bool ok, int line) {
    if (!ok) {
        std::cout << "line " << line << ": contract failed\n";
        ++failures;
    }
}
#define CHECK(x) check((x), __LINE__)

constexpr std::uint32_t index(Domain domain) { return domain_index(domain); }

// 测试侧设定值: 域身份、设定值、源年龄与 v3 显式参考系.
struct Point {
    Domain domain{Domain::pitch};
    control::ControlMode mode{control::ControlMode::position};
    float value{};
    std::uint16_t age_ms{};
    YawFrame frame{YawFrame::joint};
};

struct SoftwareBus : transport::CanTx {
    std::size_t sent{};
    std::vector<CanFrame> frames{};
    transport::SendResult try_send(const CanFrame& frame) noexcept override {
        ++sent;
        if (frames.size() < 512) { frames.push_back(frame); }
        return transport::SendResult::accepted;
    }
    [[nodiscard]] const CanFrame* last(std::uint16_t id) const {
        for (std::size_t i = frames.size(); i > 0; --i) {
            if (frames[i - 1].id == id) { return &frames[i - 1]; }
        }
        return nullptr;
    }
};

struct Rig {
    SentinelConfig config;
    SoftwareBus bus1{};
    SoftwareBus bus2{};
    SentinelApp app;
    SentinelExecutor exec;
    SentinelPerception perception;
    ParamStore params;
    SentinelHostSession session;
    AppInputs inputs{};
    Tick now{1000};
    std::uint16_t frame_seq{};
    std::uint32_t command_seq{};

    explicit Rig(Lifetime link_ttl = {}, SentinelConfig value = synthetic_config())
        : config(value), app(config, ConfigAudience::offline_test),
          exec(config, ConfigAudience::offline_test, bus1, bus2),
          perception(config, ConfigAudience::offline_test), params(config),
          session(app, params, &exec, &perception, link_ttl) {
        inputs.link.locally_ready = true;
        inputs.link.remote_online = true;
        inputs.link.switch_left = drivers::Dr16Switch::down;
        inputs.link.switch_right = drivers::Dr16Switch::mid;
        inputs.link.remote_sampled_at = now;
    }

    void cycle() {
        // 板级每周期都会推进会话链路时效: 主机测试用同一条路径, 不靠请求才老化.
        static_cast<void>(session.link_state(now));
        perception.update(now);
        inputs.inertial = perception.inertial();
        inputs.epochs = Epochs{app.config_epoch(), app.reference_epoch()};
        inputs.aux.launch_permit_valid = true;
        inputs.aux.launch_permit = true;
        inputs.aux.launch_sampled_at = now;
        inputs.aux.launch_ttl = Lifetime{100};
        inputs.link.remote_sampled_at = now;
        inputs.link.host_link_alive = true;
        app.update(now, inputs);
        exec.step(now, 0.001f, app);
    }

    // 请求封装 → 0x7e 分帧 → 逐字节解码: 测试用与生产相同的分帧路径.
    [[nodiscard]] transport::UartFrame request_frame(std::span<const std::uint8_t> payload,
        std::uint8_t type) {
        std::array<std::uint8_t, transport::uart_wire_max> wire{};
        const auto sequence = static_cast<std::uint16_t>(++frame_seq);
        const auto size = transport::encode_uart_frame(type, sequence, payload, wire);
        CHECK(size != 0);
        transport::UartFrameDecoder decoder{};
        for (std::size_t i = 0; i < size; ++i) {
            if (auto frame = decoder.push(wire[i])) { return *frame; }
        }
        return {};
    }

    // 在指定时刻处理一条已解码请求, 并按生产规则封装响应: 外层类型取自响应载荷自带的业务
    // 类型, 帧序号回显请求帧序号; 两者都直接断言, 否则测试替身会掩盖封装错误.
    [[nodiscard]] transport::UartFrame deliver(const transport::UartFrame& request, Tick at) {
        std::array<std::uint8_t, protocol_payload_max> response{};
        const auto response_size = session.handle({request.payload.data(), request.size}, at,
            response);
        if (response_size == 0) { return {}; }
        std::array<std::uint8_t, transport::uart_wire_max> back{};
        const auto response_type = declared_response_type({response.data(), response_size});
        const auto back_size = transport::encode_uart_frame(
            response_type, request.sequence, {response.data(), response_size}, back);
        CHECK(back_size != 0);
        transport::UartFrameDecoder back_decoder{};
        std::optional<transport::UartFrame> reply{};
        for (std::size_t i = 0; i < back_size; ++i) {
            if (auto frame = back_decoder.push(back[i])) { reply = frame; }
        }
        CHECK(reply.has_value());
        CHECK(reply->type == response_type);
        CHECK(reply->sequence == request.sequence);
        return reply.value_or(transport::UartFrame{});
    }

    // 完整链路: 请求载荷 → 0x7e 分帧 → 逐字节喂入解码器 → 处理 → 分帧回包 → 解码.
    [[nodiscard]] transport::UartFrame round_trip(std::span<const std::uint8_t> payload,
        std::uint8_t type, bool corrupt_crc = false, bool truncate = false) {
        std::array<std::uint8_t, transport::uart_wire_max> wire{};
        const auto request_sequence = static_cast<std::uint16_t>(++frame_seq);
        auto size = transport::encode_uart_frame(type, request_sequence, payload, wire);
        CHECK(size != 0);
        // size>=2 must be a real branch condition the optimizer can see:
        // CHECK(size != 0) alone cannot rule out size==1, where size-2
        // underflows (the -Werror=stringop-overflow build break).
        CHECK(!corrupt_crc || size >= 2);
        if (corrupt_crc && size >= 2) { wire[size - 2] ^= 0x5a; }
        if (truncate) { size -= 3; }
        transport::UartFrameDecoder decoder{};
        std::optional<transport::UartFrame> request{};
        for (std::size_t i = 0; i < size; ++i) {
            if (auto frame = decoder.push(wire[i])) { request = frame; }
        }
        if (!request) { return {}; }
        // 生产相位: 本周期先处理请求, 再推进周期 (板级 poll UART → control/commit,
        // 主机后端同样是请求先于 control/commit).
        ++now.ms;
        const auto reply = deliver(*request, now);
        cycle();
        return reply;
    }

    [[nodiscard]] std::uint32_t next_seq() noexcept { return ++command_seq; }

    [[nodiscard]] transport::UartFrame hello() {
        std::array<std::uint8_t, 16> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::hello));
        w.u32(0x1234);
        return round_trip({payload.data(), w.size()}, static_cast<std::uint8_t>(MessageType::hello));
    }
    [[nodiscard]] std::uint32_t session_id() const noexcept { return session.session_id(); }

    [[nodiscard]] transport::UartFrame authorize(bool grant, Source source, std::uint32_t seq) {
        std::array<std::uint8_t, 16> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::authorize));
        w.u32(session_id());
        w.u32(seq);
        w.u8(grant ? 1 : 0);
        w.u8(static_cast<std::uint8_t>(source));
        return round_trip({payload.data(), w.size()}, static_cast<std::uint8_t>(MessageType::authorize));
    }

    [[nodiscard]] transport::UartFrame targets(std::uint32_t seq, std::uint32_t config_epoch,
        std::uint32_t reference_epoch, std::span<const Point> setpoints,
        bool with_twist = false) {
        std::array<std::uint8_t, protocol_payload_max> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::targets));
        w.u32(session_id());
        w.u32(seq);
        w.u32(config_epoch);
        w.u32(reference_epoch);
        w.u8(static_cast<std::uint8_t>(setpoints.size()));
        for (const auto& point : setpoints) {
            w.u8(static_cast<std::uint8_t>(point.domain));
            w.u8(static_cast<std::uint8_t>(point.mode));
            w.u8(static_cast<std::uint8_t>(point.frame));
            w.f32(point.value);
            w.u16(point.age_ms);
        }
        w.u8(with_twist ? 1 : 0);
        if (with_twist) {
            w.f32(0.2f);
            w.f32(0.0f);
            w.f32(0.0f);
            w.u16(0);
        }
        return round_trip({payload.data(), w.size()}, static_cast<std::uint8_t>(MessageType::targets));
    }

    [[nodiscard]] transport::UartFrame param_write(std::uint32_t seq, std::uint16_t id, float value) {
        std::array<std::uint8_t, 16> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::param_write));
        w.u32(session_id());
        w.u32(seq);
        w.u16(id);
        w.f32(value);
        return round_trip({payload.data(), w.size()},
            static_cast<std::uint8_t>(MessageType::param_write));
    }

    [[nodiscard]] transport::UartFrame status(std::uint32_t seq, StatusPage page = StatusPage::domains) {
        std::array<std::uint8_t, 16> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::status));
        w.u32(session_id());
        w.u32(seq);
        w.u8(static_cast<std::uint8_t>(page));
        return round_trip({payload.data(), w.size()}, static_cast<std::uint8_t>(MessageType::status));
    }

    void feed(Domain domain, float rotor_angle_rad, float rotor_speed_rad_s) {
        const auto slot = app.dji_slot_for(domain);
        CHECK(slot.has_value());
        const auto& cfg = config.dji[*slot];
        const auto protocol = drivers::dji_protocol(cfg.mode);
        CHECK(protocol.has_value());
        CanFrame frame{};
        frame.id = static_cast<std::uint16_t>(protocol->rx_base + cfg.device_id);
        frame.size = 8;
        constexpr float tau = 2.0f * std::numbers::pi_v<float>;
        float angle = std::fmod(rotor_angle_rad, tau);
        if (angle < 0.0f) { angle += tau; }
        const auto encoder = static_cast<unsigned>(angle / tau * 8192.0f);
        const auto speed = static_cast<std::int16_t>(rotor_speed_rad_s * 60.0f / tau);
        frame.data[0] = static_cast<std::uint8_t>(encoder >> 8);
        frame.data[1] = static_cast<std::uint8_t>(encoder & 0xffu);
        frame.data[2] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) >> 8);
        frame.data[3] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(speed) & 0xffu);
        frame.data[6] = 40;
        exec.route(cfg.bus, frame, now);
    }
};

// 主 yaw 在其 DJI 组帧中的原始输出: 0 表示应用选定的撤输出动作, 不是"已经动过".
[[nodiscard]] std::int16_t main_yaw_raw(const Rig& rig) {
    const auto slot = rig.app.dji_slot_for(Domain::main_yaw);
    if (!slot) { return 0; }
    const auto& cfg = rig.config.dji[*slot];
    const auto protocol = drivers::dji_protocol(cfg.mode);
    if (!protocol) { return 0; }
    const unsigned group = (cfg.device_id - 1) / 4 + 1;
    const auto tx_id = group == 1 ? protocol->first_tx : protocol->second_tx;
    const SoftwareBus& bus = cfg.bus == 1 ? rig.bus1 : rig.bus2;
    const auto* frame = bus.last(tx_id);
    if (frame == nullptr) { return 0; }
    const std::size_t group_slot = (cfg.device_id - 1) % 4;
    const auto high = static_cast<std::uint16_t>(frame->data[2 * group_slot]) << 8;
    return static_cast<std::int16_t>(high | frame->data[2 * group_slot + 1]);
}

[[nodiscard]] ResultCode result_of(const transport::UartFrame& frame) {
    PayloadReader r{{frame.payload.data(), frame.size}};
    static_cast<void>(r.u8());
    static_cast<void>(r.u8());
    static_cast<void>(r.u32());
    static_cast<void>(r.u32());
    return static_cast<ResultCode>(r.u8());
}

[[nodiscard]] std::uint32_t field_u32(const transport::UartFrame& frame, std::size_t offset) {
    PayloadReader r{{frame.payload.data() + offset, frame.size - offset}};
    return r.u32();
}

void session_and_replay_contracts() {
    Rig rig;
    // 未握手: 任何请求都必须按无会话拒绝, 且不能因此获得权限.
    const auto early = rig.authorize(true, Source::host, rig.next_seq());
    CHECK(result_of(early) == ResultCode::no_session);
    CHECK(!rig.app.snapshot().permission.authority);

    const auto ack = rig.hello();
    CHECK(static_cast<MessageType>(ack.payload[1]) == MessageType::hello_ack);
    const auto session = rig.session_id();
    CHECK(session != 0);
    CHECK(field_u32(ack, 2) == session);
    CHECK(rig.app.config_epoch() == field_u32(ack, 6));
    CHECK(rig.app.reference_epoch() == field_u32(ack, 10));

    // 重新握手: 旧会话立即失效.
    const auto second = rig.hello();
    CHECK(rig.session_id() != session);
    static_cast<void>(second);
    std::array<std::uint8_t, 16> payload{};
    PayloadWriter w{payload};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::authorize));
    w.u32(session);
    w.u32(99);
    w.u8(1);
    w.u8(static_cast<std::uint8_t>(Source::host));
    const auto stale = rig.round_trip({payload.data(), w.size()},
        static_cast<std::uint8_t>(MessageType::authorize));
    CHECK(result_of(stale) == ResultCode::no_session);

    // 授权: 显式授权后来源才是 host; 撤销后立即失去权限.
    const auto granted = rig.authorize(true, Source::host, rig.next_seq());
    CHECK(result_of(granted) == ResultCode::accepted);
    // 会话身份代次变化的那一周期按"未请求"处理, 因此需要下一个周期产生请求边沿.
    rig.cycle();
    rig.cycle();
    CHECK(rig.app.snapshot().permission.source == Source::host);
    CHECK(rig.app.snapshot().permission.authority);
    const auto revoked = rig.authorize(false, Source::host, rig.next_seq());
    CHECK(result_of(revoked) == ResultCode::accepted);
    rig.cycle();
    rig.cycle();
    CHECK(!rig.app.snapshot().permission.authority);
    // 权限被撤销后, 普通目标到达不得重新打开权限.
    const Point point{Domain::pitch, control::ControlMode::position, 0.05f};
    const auto rejected = rig.targets(rig.next_seq(), rig.app.config_epoch(),
        rig.app.reference_epoch(), {&point, 1});
    CHECK(result_of(rejected) == ResultCode::not_permitted);
    CHECK(!rig.app.snapshot().permission.authority);

    // 遥控来源不接受协议授权.
    static_cast<void>(rig.authorize(true, Source::remote, rig.next_seq()));
    const auto bad_source = rig.authorize(true, Source::remote, rig.next_seq());
    CHECK(result_of(bad_source) == ResultCode::invalid_value);

    // 重放与重复: 同一序号且**内容相同**的重传回放原完整响应, 不重复执行也不谎报成功;
    // 同一序号换内容按重放拒绝, 不拿旧结果冒充本次操作; 更小的序号同样是重放.
    const auto id = param_id(0, 1);
    const auto seq = rig.next_seq();
    const auto first = rig.param_write(seq, id, 1.25f);
    CHECK(result_of(first) == ResultCode::accepted);
    const auto* entry = rig.params.entry(id);
    CHECK(entry != nullptr);
    const float after_first = entry->value;
    const auto duplicate = rig.param_write(seq, id, 1.25f);
    CHECK(result_of(duplicate) == ResultCode::accepted);
    CHECK(rig.params.entry(id)->value == after_first);
    CHECK(rig.session.duplicate_requests() == 1);
    CHECK(rig.session.replay_rejects() == 0);
    // 同序号换内容: 不得报成功, 也不改变状态.
    const auto changed = rig.param_write(seq, id, 9.0f);
    CHECK(result_of(changed) == ResultCode::replay);
    CHECK(rig.params.entry(id)->value == after_first);
    CHECK(rig.session.replay_rejects() == 1);
    // 乱序 (更小的序号) 同样按重放拒绝.
    const auto older = rig.param_write(seq - 1, id, 3.0f);
    CHECK(result_of(older) == ResultCode::replay);
    CHECK(rig.params.entry(id)->value == after_first);
    CHECK(rig.session.replay_rejects() == 2);
    // 被拒的非幂等请求重传时不得变成成功: 内容相同回放原结果码, 换内容按重放拒绝.
    const auto bad_seq = rig.next_seq();
    const auto bad_first = rig.param_write(bad_seq, id, 1.0e6f);
    CHECK(result_of(bad_first) == ResultCode::invalid_value);
    const auto bad_again = rig.param_write(bad_seq, id, 1.0e6f);
    CHECK(result_of(bad_again) == ResultCode::invalid_value);
    const auto bad_changed = rig.param_write(bad_seq, id, 1.5f);
    CHECK(result_of(bad_changed) == ResultCode::replay);
    CHECK(rig.params.entry(id)->value == after_first);

    // 版本错与截断: 不静默丢弃, 也不当成已接受.
    std::array<std::uint8_t, 8> wrong{};
    wrong[0] = 99;
    wrong[1] = static_cast<std::uint8_t>(MessageType::status);
    const auto version = rig.round_trip({wrong.data(), wrong.size()},
        static_cast<std::uint8_t>(MessageType::status));
    CHECK(result_of(version) == ResultCode::bad_version);
    std::array<std::uint8_t, 16> short_payload{};
    PayloadWriter sw{short_payload};
    sw.u8(protocol_version);
    sw.u8(static_cast<std::uint8_t>(MessageType::param_write));
    sw.u32(rig.session_id());
    sw.u32(rig.next_seq());
    // 故意不写 param_id/value: 载荷截断.
    const auto truncated = rig.round_trip({short_payload.data(), sw.size()},
        static_cast<std::uint8_t>(MessageType::param_write));
    CHECK(result_of(truncated) == ResultCode::bad_length);

    // 未知请求类型.
    std::array<std::uint8_t, 4> unknown{};
    unknown[0] = protocol_version;
    unknown[1] = 0x7f;
    const auto unknown_type = rig.round_trip({unknown.data(), unknown.size()}, 0x7f);
    CHECK(result_of(unknown_type) == ResultCode::unknown_request);

    // CRC 损坏与截断帧: 帧层必须拒绝, 不产生任何响应.
    std::array<std::uint8_t, 16> body{};
    PayloadWriter bw{body};
    bw.u8(protocol_version);
    bw.u8(static_cast<std::uint8_t>(MessageType::status));
    bw.u32(rig.session_id());
    bw.u32(rig.next_seq());
    const auto broken = rig.round_trip({body.data(), bw.size()},
        static_cast<std::uint8_t>(MessageType::status), true);
    CHECK(broken.size == 0);
    const auto cut = rig.round_trip({body.data(), bw.size()},
        static_cast<std::uint8_t>(MessageType::status), false, true);
    CHECK(cut.size == 0);
}

// NR06/T10: 正确身份/递增序号/CRC 不等于业务被接受, 拒绝不得续链路或目标.
void rejected_activity_contracts() {
    Rig rig{Lifetime{50}};
    static_cast<void>(rig.hello());
    CHECK(result_of(rig.authorize(true,Source::host,rig.next_seq()))==ResultCode::accepted);
    const std::array valid{Point{Domain::fire_yaw,control::ControlMode::position,0.2f}};
    CHECK(result_of(rig.targets(rig.next_seq(),rig.app.config_epoch(),rig.app.reference_epoch(),valid))==ResultCode::accepted);
    const auto accepted_at=rig.now.ms;
    const auto deadline=rig.app.target(Domain::fire_yaw)->accepted_at.ms;
    const std::array bad{Point{Domain::fire_yaw,control::ControlMode::position,100.0f}};
    for (const auto elapsed : {20u,40u,60u,80u}) {
        rig.now.ms=accepted_at+elapsed-1;
        const auto reply=rig.targets(rig.next_seq(),rig.app.config_epoch(),rig.app.reference_epoch(),bad);
        CHECK(result_of(reply)==(elapsed<50 ? ResultCode::invalid_value : ResultCode::no_session));
        if (elapsed<50) {
            CHECK(rig.session.link_state(rig.now).alive);
            const auto target=rig.app.target(Domain::fire_yaw);
            CHECK(!target || target->accepted_at.ms==deadline);
        } else {
            CHECK(!rig.session.link_state(rig.now).alive && !rig.app.target(Domain::fire_yaw));
        }
    }
}

// NR06/T11: 帧级/会话级/业务级拒绝和缓存 ACK 都不续期, 正常只读请求可以续期.
void activity_boundary_contracts() {
    for (unsigned fault=0;fault<7;++fault) {
        Rig rig{Lifetime{50}};
        static_cast<void>(rig.hello());
        const auto seq=rig.next_seq(), id=static_cast<std::uint32_t>(param_id(0,1));
        const auto original=rig.param_write(seq,static_cast<std::uint16_t>(id),0.2f);
        CHECK(result_of(original)==ResultCode::accepted);
        const auto accepted_at=rig.now.ms;
        const auto type=fault==6 ? MessageType::param_write : MessageType::status;
        std::array<std::uint8_t,32> payload{}; PayloadWriter w{payload};
        w.u8(fault==2 ? protocol_version+1 : protocol_version);
        w.u8(static_cast<std::uint8_t>(type));
        w.u32(fault==3 ? rig.session_id()^0x80000000u : rig.session_id());
        w.u32(fault==4 ? seq : rig.next_seq());
        if (fault==6) { w.u16(0xffff); w.f32(1.0f); }
        else { w.u8(fault==5 ? 99 : 0); }
        rig.now.ms=accepted_at+39;
        const auto response=rig.round_trip({payload.data(),w.size()-(fault==1 ? 1u : 0u)},
            static_cast<std::uint8_t>(type),fault==0);
        const std::array expected{ResultCode::accepted,ResultCode::bad_length,ResultCode::bad_version,
            ResultCode::no_session,ResultCode::replay,ResultCode::invalid_value,ResultCode::unknown_param};
        if (fault==0) { CHECK(response.size==0); }
        else { CHECK(result_of(response)==expected[fault]); }
        if (fault!=6) { // 新变更本身会占用变更结果槽; 其它坏请求不得污染原变更缓存.
            const auto cached=rig.param_write(seq,static_cast<std::uint16_t>(id),0.2f);
            CHECK(cached.size==original.size);
            CHECK(std::equal(original.payload.begin(),original.payload.begin()+original.size,cached.payload.begin()));
            CHECK(rig.session.accepted_commands()==1);
        }
        rig.now.ms=accepted_at+49;
        CHECK(rig.session.link_state(rig.now).alive);
        CHECK(result_of(rig.status(rig.next_seq()))==ResultCode::no_session);
    }
    for (const auto type : {MessageType::status,MessageType::param_read,MessageType::diagnostics}) {
        Rig rig{Lifetime{50}}; static_cast<void>(rig.hello());
        for (int i=0;i<3;++i) {
            std::array<std::uint8_t,16> payload{}; PayloadWriter w{payload};
            w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(type));
            w.u32(rig.session_id()); w.u32(rig.next_seq());
            if (type==MessageType::param_read) { w.u16(0); w.u16(0xffff); }
            else { w.u8(0); }
            rig.now.ms+=39;
            const auto response=rig.round_trip({payload.data(),w.size()},static_cast<std::uint8_t>(type));
            CHECK(response.size>12 && response.payload[1]!=static_cast<std::uint8_t>(MessageType::ack));
            CHECK(rig.session.link_state(rig.now).alive);
            CHECK(!rig.app.snapshot().permission.authority);
        }
    }
}

void session_lifetime_contracts() {
    // F01: 会话链路按真实收包时刻老化; 超时或重握手必须按该会话的来源撤销内部状态.
    Rig rig{Lifetime{50}};
    static_cast<void>(rig.hello());
    const auto first_session = rig.session_id();
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    rig.cycle();
    CHECK(rig.app.snapshot().permission.source == Source::host);
    // 持续有效通信: 每 10ms 一个 status, 超过 hello 年龄也不得失效.
    for (int i = 0; i < 30; ++i) {
        rig.now.ms += 10;
        static_cast<void>(rig.status(rig.next_seq()));
    }
    CHECK(rig.app.snapshot().permission.authority);
    CHECK(rig.app.snapshot().permission.source == Source::host);
    // 静默超过 host_link_ttl: 撤销该来源授权, 后续请求必须先重新握手.
    rig.now.ms += 60;
    rig.cycle();
    CHECK(!rig.app.snapshot().permission.authority);
    const auto after_silence = rig.status(rig.next_seq());
    CHECK(result_of(after_silence) == ResultCode::no_session);
    // 重新握手: 身份推进, 但仍需重新授权才重新取得权限.
    static_cast<void>(rig.hello());
    CHECK(rig.session_id() != first_session);
    rig.cycle();
    CHECK(!rig.app.snapshot().permission.authority);
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    rig.cycle();
    rig.cycle();
    CHECK(rig.app.snapshot().permission.source == Source::host);

    // 维护会话超时: 必须按 role 清掉维护会话状态, 不能只清 host 而把维护 bool 留下.
    Rig maintenance{Lifetime{50}};
    static_cast<void>(maintenance.hello());
    static_cast<void>(maintenance.authorize(true, Source::maintenance, maintenance.next_seq()));
    maintenance.cycle();
    maintenance.cycle();
    CHECK(maintenance.app.snapshot().permission.source == Source::maintenance);
    CHECK(maintenance.app.authority().maintenance_holds());
    maintenance.now.ms += 60;
    maintenance.cycle();
    CHECK(!maintenance.app.authority().maintenance_holds());
    CHECK(!maintenance.app.snapshot().permission.authority);
    maintenance.cycle();
    CHECK(maintenance.app.snapshot().permission.source == Source::none);
    // 过期后的请求按无会话拒绝, 不能靠旧会话号继续下发.
    const auto stale = maintenance.status(maintenance.next_seq());
    CHECK(result_of(stale) == ResultCode::no_session);
}

void manual_continuous_align_contracts() {
    auto config = synthetic_config();
    config.joints[0] = JointGeometryConfig{Domain::main_yaw, 0.0f, {}, {}, true};
    Rig rig{Lifetime{}, config};
    static_cast<void>(rig.hello());
    CHECK(result_of(rig.authorize(true, Source::maintenance, rig.next_seq())) == ResultCode::accepted);
    rig.feed(Domain::main_yaw, 1.0f, 0.0f);
    static_cast<void>(rig.exec.health(rig.now));
    CHECK(rig.exec.joint_feedback(Domain::main_yaw, rig.now).position_rad.validity != Validity::valid);
    const auto before = rig.app.reference_epoch();
    std::array<std::uint8_t, 24> payload{};
    PayloadWriter w{payload};
    w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(MessageType::align));
    w.u32(rig.session_id()); w.u32(rig.next_seq());
    w.u8(static_cast<std::uint8_t>(Domain::main_yaw)); w.u8(0); w.f32(0.0f); w.u32(before);
    const auto reply = rig.round_trip({payload.data(), w.size()}, static_cast<std::uint8_t>(MessageType::align));
    CHECK(result_of(reply) == ResultCode::accepted);
    CHECK(rig.app.reference_epoch() != before);
    const auto feedback = rig.exec.joint_feedback(Domain::main_yaw, rig.now);
    CHECK(feedback.position_rad.validity == Validity::valid && std::fabs(feedback.position_rad.value) < 1e-6f);
    const Point point{Domain::main_yaw, control::ControlMode::position, -0.1f};
    CHECK(result_of(rig.targets(rig.next_seq(), rig.app.config_epoch(), rig.app.reference_epoch(), {&point, 1})) == ResultCode::accepted);
    CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
}

void epoch_and_timeout_contracts() {
    Rig rig;
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    rig.cycle();
    const Point point{Domain::pitch, control::ControlMode::position, 0.05f};
    const auto accepted = rig.targets(rig.next_seq(), rig.app.config_epoch(),
        rig.app.reference_epoch(), {&point, 1});
    CHECK(result_of(accepted) == ResultCode::accepted);
    // 配置/参考代次不匹配: 整包拒绝.
    const auto bad_config = rig.targets(rig.next_seq(), rig.app.config_epoch() + 1,
        rig.app.reference_epoch(), {&point, 1});
    CHECK(result_of(bad_config) == ResultCode::epoch_mismatch);
    const auto bad_reference = rig.targets(rig.next_seq(), rig.app.config_epoch(),
        rig.app.reference_epoch() + 1, {&point, 1});
    CHECK(result_of(bad_reference) == ResultCode::epoch_mismatch);
    // 对齐使参考代次上推: 旧代次目标随后失效.
    // 建立位置参考需要新鲜反馈, 因此先给主 yaw 反馈.
    for (int i = 0; i < 20; ++i) {
        rig.feed(Domain::main_yaw, 0.0f, 0.0f);
        rig.now.ms += 1;
        rig.cycle();
    }
    const auto before = rig.app.reference_epoch();
    std::array<std::uint8_t, 24> payload{};
    PayloadWriter w{payload};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::align));
    w.u32(rig.session_id());
    w.u32(rig.next_seq());
    w.u8(static_cast<std::uint8_t>(Domain::main_yaw));
    w.u8(0); // kind = 机械关节参考 (与惯性/odom 对齐是两个不同的量)
    w.f32(0.0f); // 与当前单圈反馈/配置一致; 重定义零位不能借 align 绕过参数事务.
    w.u32(before);
    const auto aligned = rig.round_trip({payload.data(), w.size()},
        static_cast<std::uint8_t>(MessageType::align));
    CHECK(result_of(aligned) == ResultCode::accepted);
    CHECK(rig.app.reference_epoch() != before);
    // 对齐应答直接带新代次: 客户端不必猜, 也不需要额外查询.
    // 成功布局固定 20 B: 结果/原因 @10, 参考代次 @12, 配置代次 @16.
    CHECK(aligned.size == 20);
    CHECK(field_u32(aligned, 12) == rig.app.reference_epoch());
    CHECK(field_u32(aligned, 16) == rig.app.config_epoch());
    const auto stale_targets = rig.targets(rig.next_seq(), rig.app.config_epoch(), before,
        {&point, 1});
    CHECK(result_of(stale_targets) == ResultCode::epoch_mismatch);

    // 超时: 目标有效期过后执行必须停止, 且状态给出可解释原因.
    static_cast<void>(rig.targets(rig.next_seq(), rig.app.config_epoch(),
        rig.app.reference_epoch(), {&point, 1}));
    rig.cycle();
    CHECK(rig.app.snapshot().executions[index(Domain::pitch)].execute ||
        rig.app.snapshot().domains[index(Domain::pitch)].validity == DomainValidity::unknown);
    rig.now.ms += rig.config.timing.target_ttl.ms + 5;
    rig.cycle();
    CHECK(!rig.app.snapshot().executions[index(Domain::pitch)].execute);
    CHECK(rig.app.snapshot().domains[index(Domain::pitch)].validity != DomainValidity::valid);
}

void accept_versus_execution_contracts() {
    Rig rig;
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    rig.cycle();
    const Point point{Domain::main_yaw, control::ControlMode::position, 0.05f};
    const auto accepted = rig.targets(rig.next_seq(), rig.app.config_epoch(),
        rig.app.reference_epoch(), {&point, 1});
    // 接受 ≠ 已执行: 没有反馈/参考时目标被接受但执行被拒绝, 状态里必须能看出来.
    CHECK(result_of(accepted) == ResultCode::accepted);
    rig.cycle();
    const auto snapshot = rig.status(rig.next_seq());
    PayloadReader r{{snapshot.payload.data(), snapshot.size}};
    static_cast<void>(r.u8());
    static_cast<void>(r.u8());
    // 应答回显原请求身份: 快照与本次请求的配对不依赖 CRC 正确.
    CHECK(r.u32() == rig.session_id());
    static_cast<void>(r.u32());
    CHECK(r.u8() == 1);
    const auto config_epoch = r.u32();
    const auto reference_epoch = r.u32();
    const auto clock = r.u32();
    const auto source = r.u8();
    const auto authority = r.u8();
    const auto manual_stop = r.u8();
    const auto launch = r.u8();
    const auto count = r.u8();
    CHECK(config_epoch == rig.app.config_epoch());
    CHECK(reference_epoch == rig.app.reference_epoch());
    CHECK(clock == rig.now.ms);
    CHECK(source == static_cast<std::uint8_t>(Source::host));
    CHECK(authority == 1);
    CHECK(manual_stop == 0);
    CHECK(launch == 1);
    CHECK(count == domain_count);
    bool saw_pitch = false;
    for (std::uint8_t i = 0; i < count; ++i) {
        const auto validity = r.u8();
        const auto mode = r.u8();
        const auto execute = r.u8();
        const auto reason = r.u8();
        static_cast<void>(r.u8()); // 参考系与测量有效性
        const auto target = r.f32();
        const auto measurement = r.f32();
        const auto position_age = r.u32();
        const auto velocity_age = r.u32();
        static_cast<void>(target);
        static_cast<void>(measurement);
        // 反馈不可用时年龄用 0xffffffff 表示, 不用 0 冒充"刚刚采样".
        CHECK(position_age == 0xffffffffu || position_age < 1000u);
        CHECK(velocity_age == 0xffffffffu || velocity_age < 1000u);
        if (static_cast<Domain>(i) != Domain::main_yaw) { continue; }
        saw_pitch = true;
        CHECK(mode == static_cast<std::uint8_t>(control::ControlMode::position));
        // 无反馈: 域有效性必须是 unknown, 不能因为目标被接受就变成 valid.
        CHECK(validity == static_cast<std::uint8_t>(DomainValidity::unknown));
        // 应用层"已具备执行资格"不等于电机已经动: 没有反馈时轴层不给输出, 总线上没有帧.
        CHECK(execute == 1);
        CHECK(reason == static_cast<std::uint8_t>(Reason::none));
        // 组帧仍然发送, 但该槽是本应用选定的撤输出动作: 目标被接受不等于电机被驱动.
        CHECK(main_yaw_raw(rig) == 0);
    }
    CHECK(saw_pitch);
    CHECK(r.ok());
    // 无反馈域不能把 0 冒充成有效测量: 年龄字段用 0xffffffff 表示不可用.
    // 反馈到达后同一目标才开始执行, 说明"接受"与"执行"确实是两件事.
    for (int i = 0; i < 20; ++i) {
        rig.feed(Domain::main_yaw, 0.0f, 0.0f);
        rig.now.ms += 1;
        rig.cycle();
    }
    // 位置模式还需要显式位置参考: 单圈编码器不能自行恢复掉电前的多圈角.
    CHECK(rig.exec.align_reference(Domain::main_yaw, 0.0f, rig.now));
    static_cast<void>(rig.targets(rig.next_seq(), rig.app.config_epoch(),
        rig.app.reference_epoch(), {&point, 1}));
    rig.cycle();
    const auto& execution = rig.app.snapshot().executions[index(Domain::main_yaw)];
    CHECK(execution.mode == control::ControlMode::position);
    // 反馈到位后轴层才产生输出: 同一槽位出现非零执行量.
    CHECK(main_yaw_raw(rig) != 0);
}

// F06: 单域参考参数变更只作废受影响的域, 不把其它正在执行的合法目标 (尤其底盘) 一起停掉.
void scoped_reference_invalidation_contracts() {
    Rig rig;
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    rig.cycle();
    rig.cycle();
    // 底盘 twist 正在执行; 目标域 (fire_yaw) 保持空闲.
    const std::span<const Point> none{};
    const auto ack = rig.targets(
        rig.next_seq(), rig.app.config_epoch(), rig.app.reference_epoch(), none, true);
    CHECK(result_of(ack) == ResultCode::accepted);
    rig.cycle();
    CHECK(rig.app.target(Domain::chassis_lf).has_value());
    const auto fire_epoch_before = rig.app.domain_reference_epoch(Domain::fire_yaw);
    const auto global_before = rig.app.reference_epoch();
    const auto write = rig.param_write(rig.next_seq(),
        param_id(*rig.app.dji_slot_for(Domain::fire_yaw), 20), 0.1f);
    CHECK(result_of(write) == ResultCode::accepted);
    // 受影响域代次上推; 底盘目标仍在, 不被通用"清全部目标"顺带停掉.
    CHECK(rig.app.domain_reference_epoch(Domain::fire_yaw) == fire_epoch_before + 1);
    CHECK(rig.app.reference_epoch() > global_before);
    CHECK(rig.app.target(Domain::chassis_lf).has_value());
    // 未受影响的域代次不变.
    CHECK(rig.app.domain_reference_epoch(Domain::main_yaw) == 0);
}

// A01: 到期判定必须由接收路径自己按当前时刻完成, 不能依赖调用方先跑老化;
// 也不因为先处理了较新的请求就把过期会话刷成新鲜.
void expired_request_order_contracts() {
    // (a) 请求先到 (未跑 link_state): 仍须按当前时刻判定过期.
    {
        Rig rig{Lifetime{50}};
        static_cast<void>(rig.hello());
        const auto sid = rig.session.session_id();
        rig.now.ms += 50; // 恰好到达时限: 严格小于才算新鲜
        std::array<std::uint8_t, 16> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::status));
        w.u32(sid);
        w.u32(1); w.u8(1);
        std::array<std::uint8_t, protocol_payload_max> response{};
        const auto size = rig.session.handle({payload.data(), w.size()}, rig.now, response);
        CHECK(size == 12);
        CHECK(static_cast<MessageType>(response[1]) == MessageType::ack);
        CHECK(static_cast<ResultCode>(response[10]) == ResultCode::no_session);
    }
    // (b) 同一到期请求, 老化先跑: 结果必须一致.
    {
        Rig rig{Lifetime{50}};
        static_cast<void>(rig.hello());
        const auto sid = rig.session.session_id();
        rig.now.ms += 50;
        const auto link = rig.session.link_state(rig.now);
        CHECK(!link.alive);
        CHECK(link.expired);
        std::array<std::uint8_t, 16> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::status));
        w.u32(sid);
        w.u32(1); w.u8(1);
        std::array<std::uint8_t, protocol_payload_max> response{};
        const auto size = rig.session.handle({payload.data(), w.size()}, rig.now, response);
        CHECK(size == 12);
        CHECK(static_cast<ResultCode>(response[10]) == ResultCode::no_session);
    }
    // (c) 边界内 (时限前 1ms) 的有效请求照常放行.
    {
        Rig rig{Lifetime{50}};
        static_cast<void>(rig.hello());
        rig.now.ms += 49;
        std::array<std::uint8_t, 16> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::status));
        w.u32(rig.session.session_id());
        w.u32(1); w.u8(1);
        std::array<std::uint8_t, protocol_payload_max> response{};
        const auto size = rig.session.handle({payload.data(), w.size()}, rig.now, response);
        CHECK(size > 12);
        CHECK(static_cast<MessageType>(response[1]) == MessageType::status_snapshot);
    }
    // (d) 时限内的正常通信持续续会话 (1000 → 1040 → 1080).
    {
        Rig rig{Lifetime{50}};
        static_cast<void>(rig.hello());
        auto seq = rig.next_seq();
        rig.now.ms += 39;
        const auto first = rig.status(seq);
        CHECK(declared_response_type({first.payload.data(), first.size}) ==
            static_cast<std::uint8_t>(MessageType::status_snapshot));
        seq = rig.next_seq();
        rig.now.ms += 39;
        const auto second = rig.status(seq);
        CHECK(declared_response_type({second.payload.data(), second.size}) ==
            static_cast<std::uint8_t>(MessageType::status_snapshot));
        CHECK(rig.session.link_state(rig.now).alive);
    }
    // (e) 过期后必须重新握手 (且只有新会话重新取得授权) 才能继续.
    {
        Rig rig{Lifetime{50}};
        static_cast<void>(rig.hello());
        rig.now.ms += 50;
        CHECK(result_of(rig.status(rig.next_seq())) == ResultCode::no_session);
        static_cast<void>(rig.hello());
        CHECK(rig.session.link_state(rig.now).alive);
        CHECK(result_of(rig.authorize(true, Source::host, rig.next_seq())) == ResultCode::accepted);
        rig.cycle();
        rig.cycle();
        CHECK(rig.app.snapshot().permission.source == Source::host);
        CHECK(rig.app.snapshot().permission.authority);
    }
}

// R1: 同序号缓存重传同样受会话时效约束 —— 到期后必须回 NO_SESSION, 不能拿到缓存里的
// ACCEPTED 或 REPLAY. 覆盖 TTL-1/TTL/TTL+1 三个时点, 并按两种调度顺序各跑一次.
void expired_retransmission_contracts() {
    struct Probe {
        std::size_t size{};
        ResultCode code{ResultCode::accepted};
        std::uint8_t outer_type{};
        std::uint16_t echo{};
        float value{};
        std::uint32_t accepted{};
    };
    const auto id = param_id(0, 1);
    const auto probe = [&](std::uint32_t age_ms, bool content_conflict, bool poll_first) {
        Rig rig{Lifetime{50}};
        static_cast<void>(rig.hello());
        const auto seq = rig.next_seq();
        CHECK(result_of(rig.param_write(seq, id, 1.0f)) == ResultCode::accepted);
        const auto last = rig.now;
        std::array<std::uint8_t, 16> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::param_write));
        w.u32(rig.session_id());
        w.u32(seq);
        w.u16(id);
        w.f32(content_conflict ? 2.0f : 1.0f);
        const auto request = rig.request_frame({payload.data(), w.size()},
            static_cast<std::uint8_t>(MessageType::param_write));
        CHECK(request.size != 0);
        const Tick at{last.ms + age_ms};
        // 两种调度顺序: 先请求 (只有接收路径自己的到期判定) 与先老化轮询.
        if (poll_first) { static_cast<void>(rig.session.link_state(at)); }
        const auto reply = rig.deliver(request, at);
        const auto* entry = rig.params.entry(id);
        return Probe{reply.size, result_of(reply), reply.type, reply.sequence,
            entry ? entry->value : 0.0f, rig.session.accepted_commands()};
    };
    // TTL-1: 未到期 → 同内容回放原完整响应, 不同内容按重放拒绝; 两种顺序一致.
    for (const bool poll_first : {false, true}) {
        const auto replay = probe(49, false, poll_first);
        CHECK(replay.size == 24);
        CHECK(replay.code == ResultCode::accepted);
        CHECK(replay.outer_type == static_cast<std::uint8_t>(MessageType::ack));
        CHECK(replay.value == 1.0f);            // 原写入值仍在, 未被重传改写
        CHECK(replay.accepted == 1);            // 只执行过那一次参数写入
        const auto conflict = probe(49, true, poll_first);
        CHECK(conflict.size == 12);
        CHECK(conflict.code == ResultCode::replay);
        CHECK(conflict.outer_type == static_cast<std::uint8_t>(MessageType::ack));
        CHECK(conflict.value == 1.0f);
        CHECK(conflict.accepted == 1);
    }
    // TTL / TTL+1: 到期后一律 NO_SESSION, 不再是缓存的 ACCEPTED/REPLAY, 也不改参数值.
    for (const std::uint32_t age : {50u, 51u}) {
        for (const bool poll_first : {false, true}) {
            for (const bool conflict : {false, true}) {
                const auto result = probe(age, conflict, poll_first);
                CHECK(result.size == 12);
                CHECK(result.code == ResultCode::no_session);
                CHECK(result.outer_type == static_cast<std::uint8_t>(MessageType::ack));
                CHECK(result.echo != 0);
                CHECK(result.value == 1.0f);
                CHECK(result.accepted == 1);
            }
        }
    }
}

// 失败响应的内/外类型与长度必须一致: targets 失败回 0xc2, 重放冲突回 12 字节 C1.
void failure_envelope_contracts() {
    Rig rig;
    static_cast<void>(rig.hello());
    CHECK(result_of(rig.authorize(true, Source::host, rig.next_seq())) == ResultCode::accepted);
    rig.cycle();
    rig.cycle();
    // 代次不匹配的 targets: 外层还是 targets_ack, 不借用 ack 类型.
    const Point point{Domain::pitch, control::ControlMode::position, 0.05f};
    const auto mismatched = rig.targets(rig.next_seq(), rig.app.config_epoch() + 1,
        rig.app.reference_epoch(), {&point, 1});
    CHECK(result_of(mismatched) == ResultCode::epoch_mismatch);
    CHECK(declared_response_type({mismatched.payload.data(), mismatched.size}) ==
        static_cast<std::uint8_t>(MessageType::targets_ack));
    CHECK(mismatched.size == 12);
    // 非幂等请求: 同序号换内容按重放拒绝, 回 12 字节 ack.
    const auto id = param_id(0, 1);
    const auto seq = rig.next_seq();
    CHECK(result_of(rig.param_write(seq, id, 1.0f)) == ResultCode::accepted);
    const auto conflict = rig.param_write(seq, id, 2.0f);
    CHECK(result_of(conflict) == ResultCode::replay);
    CHECK(conflict.size == 12);
    CHECK(declared_response_type({conflict.payload.data(), conflict.size}) ==
        static_cast<std::uint8_t>(MessageType::ack));
}

// 只用幂等 status 续会话时, 会话保持但旧域目标不被续期.
void session_traffic_does_not_extend_target_contracts() {
    Rig rig;
    static_cast<void>(rig.hello());
    CHECK(result_of(rig.authorize(true, Source::host, rig.next_seq())) == ResultCode::accepted);
    rig.cycle();
    rig.cycle();
    const Point point{Domain::pitch, control::ControlMode::position, 0.05f};
    CHECK(result_of(rig.targets(rig.next_seq(), rig.app.config_epoch(),
              rig.app.reference_epoch(), {&point, 1})) == ResultCode::accepted);
    rig.cycle();
    CHECK(rig.app.target(Domain::pitch).has_value());
    CHECK(rig.app.snapshot().executions[index(Domain::pitch)].execute);
    for (int i = 0; i < 4; ++i) {
        rig.now.ms += 10;
        const auto reply = rig.status(rig.next_seq());
        CHECK(declared_response_type({reply.payload.data(), reply.size}) ==
            static_cast<std::uint8_t>(MessageType::status_snapshot));
    }
    CHECK(rig.session.link_state(rig.now).alive);
    CHECK(!rig.app.snapshot().executions[index(Domain::pitch)].execute);
    CHECK(rig.app.snapshot().executions[index(Domain::pitch)].reason == Reason::stale_target);
}

// A05: 错误 session (但序号碰巧等于当前 last_seq) 的请求不得覆盖合法请求的结果缓存.
void wrong_session_cache_isolation_contracts() {
    Rig rig;
    static_cast<void>(rig.hello());
    const auto sid = rig.session.session_id();
    const auto id = param_id(0, 1);
    const auto seq = rig.next_seq();
    const auto first = rig.param_write(seq, id, 1.25f);
    CHECK(result_of(first) == ResultCode::accepted);
    std::array<std::uint8_t, protocol_payload_max> original{};
    std::memcpy(original.data(), first.payload.data(), first.size);
    const auto original_size = first.size;
    // 错误 session + 相同 seq: 必须只回 NO_SESSION.
    std::array<std::uint8_t, 16> payload{};
    PayloadWriter w{payload};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::param_write));
    w.u32(sid ^ 0x80000000u);
    w.u32(seq);
    w.u16(id);
    w.f32(9.0f);
    std::array<std::uint8_t, protocol_payload_max> response{};
    const auto size = rig.session.handle({payload.data(), w.size()}, rig.now, response);
    CHECK(size == 24);
    CHECK(static_cast<ResultCode>(response[10]) == ResultCode::no_session);
    // 合法请求的同序号同内容重传仍回放原完整响应 (缓存未被污染).
    const auto again = rig.param_write(seq, id, 1.25f);
    CHECK(again.size == original_size);
    CHECK(std::memcmp(again.payload.data(), original.data(), original_size) == 0);
    CHECK(rig.params.entry(id)->value == 1.25f);
}

// A01 残余: 同一控制批内维护会话被替换 (hello) 并重新授权维护时, 旧维护授权与旧目标
// 必须立即失效 (重新签发产生新 generation), 不能因 bool 先清后置就继承旧 token.
void maintenance_session_replacement_contracts() {
    Rig rig;
    std::array<std::uint8_t, protocol_payload_max> request{};
    std::array<std::uint8_t, protocol_payload_max> response{};
    const auto direct = [&](std::size_t size) {
        return rig.session.handle({request.data(), size}, rig.now, response);
    };
    const auto send_hello = [&]() {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::hello));
        w.u32(0x1234u);
        CHECK(direct(w.size()) > 0);
    };
    const auto send_authorize = [&](std::uint32_t sid, std::uint32_t seq, Source source) {
        PayloadWriter w{request};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::authorize));
        w.u32(sid);
        w.u32(seq);
        w.u8(1);
        w.u8(static_cast<std::uint8_t>(source));
        CHECK(direct(w.size()) > 0);
        CHECK(static_cast<ResultCode>(response[10]) == ResultCode::accepted);
    };
    send_hello();
    const auto sid1 = rig.session.session_id();
    send_authorize(sid1, rig.next_seq(), Source::maintenance);
    // 维护来源需要已见运行状态且无正常来源: 用待机位 + 在线遥控满足首启许可.
    rig.inputs.link.switch_left = drivers::Dr16Switch::mid;
    rig.inputs.link.remote_online = true;
    rig.cycle();
    rig.cycle();
    CHECK(rig.app.snapshot().permission.source == Source::maintenance);
    const Point point{Domain::pitch, control::ControlMode::position, 0.05f};
    CHECK(result_of(rig.targets(rig.next_seq(), rig.app.config_epoch(),
              rig.app.reference_epoch(), {&point, 1})) == ResultCode::accepted);
    rig.cycle();
    CHECK(rig.app.target(Domain::pitch).has_value());
    const auto generation_before = rig.app.snapshot().permission.generation;
    // R2 组合前提: 人工显式选到 Jetson 位 (host 未就绪) → 撤销维护并封禁本次申请.
    rig.inputs.link.switch_left = drivers::Dr16Switch::down;
    rig.cycle();
    CHECK(rig.app.snapshot().permission.source != Source::maintenance);
    rig.cycle();
    CHECK(rig.app.snapshot().permission.source != Source::maintenance);
    // 同一批: 新 hello 替换旧维护会话, 紧接着重新授权维护 (中间不跑控制周期).
    // 新的显式申请重建资格, 不继承刚被人工撤销那一次的封禁.
    send_hello();
    const auto sid2 = rig.session.session_id();
    CHECK(sid2 != sid1);
    send_authorize(sid2, rig.next_seq(), Source::maintenance);
    rig.cycle();
    rig.cycle();
    CHECK(rig.app.snapshot().permission.source == Source::maintenance);
    CHECK(rig.app.snapshot().permission.generation != generation_before);
    CHECK(!rig.app.target(Domain::pitch).has_value());
}

void parameter_contracts() {
    Rig rig;
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    rig.cycle();
    // 读全部: 计数与存储一致, 且读回值等于实际生效值.
    std::array<std::uint8_t, 16> payload{};
    PayloadWriter w{payload};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::param_read));
    w.u32(rig.session_id());
    w.u32(rig.next_seq());
    w.u16(0);
    w.u16(0xffff);
    const auto all = rig.round_trip({payload.data(), w.size()},
        static_cast<std::uint8_t>(MessageType::param_read));
    PayloadReader r{{all.payload.data(), all.size}};
    static_cast<void>(r.u8());
    static_cast<void>(r.u8());
    static_cast<void>(r.u32());
    static_cast<void>(r.u32());
    const auto count = r.u16();
    const auto offset = r.u16();
    const auto total = r.u16();
    CHECK(total == rig.params.size());
    CHECK(rig.params.size() > 0);
    CHECK(offset == 0);
    // 一页放不下时按页返回, 不静默截断也不超长.
    CHECK(count == (rig.params.size() < param_page_max ? rig.params.size() : param_page_max));
    CHECK(all.size == 16 + count * 16);
    // 未知参数 id.
    const auto missing = rig.param_write(rig.next_seq(), 0xfffe, 1.0f);
    CHECK(result_of(missing) == ResultCode::unknown_param);
    // 参数读回错误也用 ACK 布局: 内外类型一致, 接收方能按结果码处理而不是按条目表解码.
    std::array<std::uint8_t, 16> read_payload{};
    PayloadWriter rw{read_payload};
    rw.u8(protocol_version);
    rw.u8(static_cast<std::uint8_t>(MessageType::param_read));
    rw.u32(rig.session_id());
    rw.u32(rig.next_seq());
    rw.u16(0);
    rw.u16(0xfffe);
    const auto unknown_read = rig.round_trip({read_payload.data(), rw.size()},
        static_cast<std::uint8_t>(MessageType::param_read));
    CHECK(result_of(unknown_read) == ResultCode::unknown_param);
    CHECK(declared_response_type({unknown_read.payload.data(), unknown_read.size}) ==
        static_cast<std::uint8_t>(MessageType::ack));
    // 越界值拒绝.
    const auto out_of_range = rig.param_write(rig.next_seq(), param_id(0, 1), 1.0e6f);
    CHECK(result_of(out_of_range) == ResultCode::invalid_value);
    // 参考类参数: 空闲机构可以改, 仍在执行的机构必须拒绝.
    const auto friction_slot = rig.app.dji_slot_for(Domain::friction_left);
    CHECK(friction_slot.has_value());
    const auto idle_id = param_id(*friction_slot, 4); // current_limit_a
    const auto* spec = rig.params.find(idle_id);
    CHECK(spec != nullptr && spec->kind == ParamKind::reference);
    const auto idle_write = rig.param_write(rig.next_seq(), idle_id, 1.0f);
    CHECK(result_of(idle_write) == ResultCode::accepted);
    // 让主 yaw 真正进入执行: 反馈 + 位置目标.
    const auto yaw_slot = rig.app.dji_slot_for(Domain::main_yaw);
    CHECK(yaw_slot.has_value());
    for (int i = 0; i < 20; ++i) {
        rig.feed(Domain::main_yaw, 0.0f, 0.0f);
        rig.now.ms += 1;
        rig.cycle();
    }
    const Point yaw_point{Domain::main_yaw, control::ControlMode::position, 0.1f};
    static_cast<void>(rig.targets(rig.next_seq(), rig.app.config_epoch(),
        rig.app.reference_epoch(), {&yaw_point, 1}));
    rig.cycle();
    CHECK(rig.app.snapshot().executions[index(Domain::main_yaw)].execute);
    const auto busy_id = param_id(*yaw_slot, 4);
    const auto busy = rig.param_write(rig.next_seq(), busy_id, 1.0f);
    CHECK(result_of(busy) == ResultCode::reference_in_use);
    // 热调参数: 写入后读回等于执行层的实际控制配置.
    const auto hot_id = param_id(*friction_slot, 1);
    const auto slot = friction_slot;
    CHECK(slot.has_value());
    const auto written = rig.param_write(rig.next_seq(), hot_id, 1.75f);
    CHECK(result_of(written) == ResultCode::accepted);
    // 成功布局固定 24 B: 生效值 @12, 参考代次 @16, 配置代次 @20.
    CHECK(written.size == 24);
    CHECK(field_u32(written, 16) == rig.app.reference_epoch());
    CHECK(field_u32(written, 20) == rig.app.config_epoch());
    PayloadReader effective{{written.payload.data(), written.size}};
    static_cast<void>(effective.u8());
    static_cast<void>(effective.u8());
    static_cast<void>(effective.u32());
    static_cast<void>(effective.u32());
    static_cast<void>(effective.u8());
    static_cast<void>(effective.u8());
    CHECK(std::fabs(effective.f32() - 1.75f) < 1e-5f);
    CHECK(rig.exec.velocity_gain(*slot, 0) == 1.75f);
    CHECK(rig.params.entry(hot_id)->value == rig.exec.velocity_gain(*slot, 0));
    // 固件默认值仍可查: 覆盖标记与实际值分开.
    CHECK(rig.params.entry(hot_id)->overridden);
    CHECK(rig.params.entry(hot_id)->spec.default_value != 1.75f);
}

void mutation_stream_and_frames_contracts() {
    Rig rig{Lifetime{50}};
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    const auto seq = rig.next_seq();
    const auto id = param_id(4, 20);
    const auto original = rig.param_write(seq, id, 0.2f);
    CHECK(result_of(original) == ResultCode::accepted);
    const auto epoch = rig.app.reference_epoch();
    for (int i=0; i<3; ++i) {
        ++rig.now.ms;
        CHECK(result_of(rig.targets(rig.next_seq(), rig.app.config_epoch(), epoch, {}, true)) == ResultCode::accepted);
        static_cast<void>(rig.status(rig.next_seq()));
    }
    const auto last_fresh = rig.now;
    const auto accepted = rig.session.accepted_commands();
    const auto target_at = rig.app.target(Domain::chassis_lf)->accepted_at;
    rig.now.ms += 10;
    const auto replay = rig.param_write(seq, id, 0.2f);
    CHECK(replay.size == original.size);
    CHECK(std::equal(original.payload.begin(), original.payload.begin()+original.size, replay.payload.begin()));
    CHECK(rig.session.accepted_commands() == accepted && rig.app.reference_epoch() == epoch);
    CHECK(rig.app.target(Domain::chassis_lf)->accepted_at.ms == target_at.ms);
    CHECK(result_of(rig.param_write(seq, id, 0.3f)) == ResultCode::replay);
    rig.now.ms = last_fresh.ms+50;
    CHECK(result_of(rig.param_write(seq, id, 0.2f)) == ResultCode::no_session);
    // 新变更覆盖唯一变更槽; 非到达请求也不能靠旧序号执行.
    Rig other;
    static_cast<void>(other.hello());
    const auto old = other.next_seq();
    CHECK(result_of(other.param_write(old, param_id(0, 1), 1.2f)) == ResultCode::accepted);
    CHECK(result_of(other.param_write(other.next_seq(), param_id(0, 1), 1.3f)) == ResultCode::accepted);
    CHECK(result_of(other.param_write(old, param_id(0, 1), 1.2f)) == ResultCode::replay);
    const auto missed = other.next_seq();
    static_cast<void>(other.status(other.next_seq()));
    CHECK(result_of(other.param_write(missed, param_id(0, 1), 1.4f)) == ResultCode::replay);

    Rig frames;
    static_cast<void>(frames.hello());
    static_cast<void>(frames.authorize(true, Source::host, frames.next_seq()));
    auto& platform = frames.inputs.main_yaw;
    platform.validity = DomainValidity::valid; platform.local_reference = true;
    platform.odom_aligned = true; platform.odom_offset_rad = 0.3f;
    platform.continuous_heading = 1.2f; platform.value.orientation = axis_rotation({0,0,1}, 1.2f);
    platform.value.valid = true; platform.value.heading_gain = 1.0f;
    auto& feedback = frames.inputs.feedback[index(Domain::main_yaw)];
    feedback.position_rad = {0.4f, frames.now, Validity::valid};
    feedback.velocity_rad_s = {0.0f, frames.now, Validity::valid};
    frames.cycle();
    for (const auto frame : {YawFrame::joint, YawFrame::local_inertial, YawFrame::odom}) {
        const Point target{Domain::main_yaw, control::ControlMode::position,
            frame == YawFrame::joint ? 0.5f : (frame == YawFrame::odom ? 1.6f : 1.3f), 0, frame};
        CHECK(result_of(frames.targets(frames.next_seq(), frames.app.config_epoch(),
            frames.app.reference_epoch(), {&target, 1})) == ResultCode::accepted);
        frames.cycle();
        const auto response = frames.status(frames.next_seq());
        CHECK(response.size == 238);
        PayloadReader r{{response.payload.data()+28+index(Domain::main_yaw)*21, 21}};
        for (int i=0; i<4; ++i) { static_cast<void>(r.u8()); }
        const auto flags = r.u8();
        CHECK((flags & 3u) == static_cast<unsigned>(frame));
        CHECK(((flags >> 4) & 3u) == static_cast<unsigned>(frame));
        CHECK(flags & 128u);
        static_cast<void>(r.f32());
        const auto expected = frame == YawFrame::joint ? 0.4f : (frame == YawFrame::odom ? 1.5f : 1.2f);
        CHECK(std::fabs(r.f32()-expected) < 1e-5f);
    }
    CHECK(frames.status(frames.next_seq(), StatusPage::fast).size == 214);
    CHECK(frames.status(frames.next_seq(), StatusPage::requests).size == 227);
    std::array<std::uint8_t, 2> v2{2, static_cast<std::uint8_t>(MessageType::status)};
    const auto old_version = frames.round_trip(v2, v2[1]);
    CHECK(result_of(old_version) == ResultCode::bad_version && old_version.payload[0] == 3);
}

void reply_mux_contracts() {
    ReplyMux mux{10};
    const auto put = [&](MessageType kind, std::uint16_t sequence, std::uint8_t page=0) {
        transport::UartFrame request{};
        request.type = static_cast<std::uint8_t>(kind); request.sequence = sequence;
        request.size = 11; request.payload[10] = page;
        std::array<std::uint8_t, 12> payload{};
        PayloadWriter w{payload};
        w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(MessageType::ack));
        w.u32(1); w.u32(sequence); w.u8(0); w.u8(0);
        CHECK(mux.enqueue(request, payload));
    };
    std::vector<std::uint16_t> delivered;
    const auto send = [&](std::span<const std::uint8_t> wire) {
        transport::UartFrameDecoder decoder;
        for (auto byte : wire) {
            if (const auto frame = decoder.push(byte)) {
                CHECK(frame->type == static_cast<std::uint8_t>(MessageType::ack));
                PayloadReader r{{frame->payload.data()+6, 4}};
                CHECK(r.u32() == frame->sequence);
                delivered.push_back(frame->sequence);
            }
        }
        return true;
    };
    put(MessageType::status, 1, 1);
    CHECK(!mux.service({0}, [](auto) { return false; }));
    put(MessageType::param_write, 2);
    put(MessageType::targets, 3);
    put(MessageType::status, 4, 0);
    put(MessageType::diagnostics, 5);
    CHECK(mux.diagnostics()[9] == 4 && mux.diagnostics()[7] == 1);
    for (int i=0; i<4; ++i) { CHECK(mux.service({1}, send)); }
    CHECK((delivered == std::vector<std::uint16_t>{2,3,4,5}));
    put(MessageType::param_read, 6);
    CHECK(!mux.service({2}, send));
    put(MessageType::targets, 7);
    CHECK(mux.service({2}, send) && delivered.back() == 7);
    CHECK(!mux.service({10}, send));
    CHECK(mux.service({11}, send) && delivered.back() == 6);
    put(MessageType::status, 8, 0); put(MessageType::status, 9, 0);
    CHECK(mux.service({12}, send) && delivered.back() == 9);
    CHECK(mux.diagnostics()[6] == 1);
    Rig rig;
    static_cast<void>(rig.hello());
    const auto cycles = rig.app.stats().cycles;
    put(MessageType::param_write, 10);
    for (int i=0; i<100; ++i) {
        ++rig.now.ms; rig.cycle();
        CHECK(!mux.service(rig.now, [](auto) { return false; }));
    }
    CHECK(rig.app.stats().cycles == cycles+100 && mux.diagnostics()[9] == 1);
}

void diagnostics_contracts() {
    Rig rig;
    static_cast<void>(rig.hello());
    static_cast<void>(rig.authorize(true, Source::host, rig.next_seq()));
    for (int i = 0; i < 5; ++i) { rig.cycle(); }
    std::array<std::uint8_t, 16> payload{};
    PayloadWriter w{payload};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::diagnostics));
    w.u32(rig.session_id());
    w.u32(rig.next_seq()); w.u8(0);
    const auto page = rig.round_trip({payload.data(), w.size()},
        static_cast<std::uint8_t>(MessageType::diagnostics));
    PayloadReader r{{page.payload.data(), page.size}};
    static_cast<void>(r.u8());
    static_cast<void>(r.u8());
    static_cast<void>(r.u32());
    static_cast<void>(r.u32());
    CHECK(r.u8() == 0); CHECK(r.u8() == 28);
    const auto cycles = r.u32();
    static_cast<void>(r.u32());
    static_cast<void>(r.u32());
    static_cast<void>(r.u32());
    static_cast<void>(r.u32());
    static_cast<void>(r.u32());
    const auto commands = r.u32();
    const auto session = r.u32();
    CHECK(cycles >= 6); // 5 次显式周期 + 每次请求一个周期
    CHECK(commands == 1);
    CHECK(session == rig.session_id());
    CHECK(r.ok());}
} // 命名空间


// 操作限幅: 主机下发的pitch目标只允许落在command带内, 越界整包拒绝, 机械止点保持不变.
void command_band_clamps_targets() {
    auto config = synthetic_config();
    auto& pitch = config.joints[2];
    pitch.command_min_rad = -0.132f;
    pitch.command_max_rad = 0.235f;
    const float mech_lo = *pitch.min_rad;
    const float mech_hi = *pitch.max_rad;
    Rig rig(Lifetime{}, config);
    static_cast<void>(rig.hello());
    CHECK(result_of(rig.authorize(true, Source::host, rig.next_seq())) == ResultCode::accepted);
    rig.cycle();

    // 目标越过操作带上界(但在机械止点内): 整包拒绝, 不是静默夹到机械上界.
    const Point high{Domain::pitch, control::ControlMode::position, 0.40f};
    CHECK(result_of(rig.targets(rig.next_seq(), rig.app.config_epoch(),
              rig.app.reference_epoch(), {&high, 1})) == ResultCode::invalid_value);
    rig.cycle();
    const auto hi_target = rig.app.target(Domain::pitch);
    CHECK(!hi_target || std::fabs(hi_target->value - 0.40f) > 1e-4f);

    // 目标越过操作带下界: 同样拒绝.
    const Point low{Domain::pitch, control::ControlMode::position, -0.30f};
    CHECK(result_of(rig.targets(rig.next_seq(), rig.app.config_epoch(),
              rig.app.reference_epoch(), {&low, 1})) == ResultCode::invalid_value);

    // 带内目标: 原样接受.
    const Point inside{Domain::pitch, control::ControlMode::position, 0.10f};
    CHECK(result_of(rig.targets(rig.next_seq(), rig.app.config_epoch(),
              rig.app.reference_epoch(), {&inside, 1})) == ResultCode::accepted);
    rig.cycle();
    const auto in_target = rig.app.target(Domain::pitch);
    CHECK(in_target.has_value());
    if (in_target) { CHECK(std::fabs(in_target->value - 0.10f) < 1e-4f); }

    // 机械止点参数不被操作带改写.
    CHECK(std::fabs(*rig.app.joint_geometry(Domain::pitch).min_rad - mech_lo) < 1e-6f);
    CHECK(std::fabs(*rig.app.joint_geometry(Domain::pitch).max_rad - mech_hi) < 1e-6f);
}

// 操作带必须落在机械止点内且min<max, 否则配置判为不完整.
void command_band_validation() {
    auto bad = synthetic_config();
    bad.joints[2].command_min_rad = 0.40f;  // 高于机械上界0.5? -> 实际高于机械上界
    bad.joints[2].command_max_rad = 0.60f;  // 0.60 超出机械上界0.5, 整带非法
    const auto bad_report = validate(bad, ConfigAudience::offline_test);
    CHECK(bad_report.domains[domain_index(Domain::pitch)] != ConfigState::complete);
    CHECK(!bad_report.complete);

    auto ok = synthetic_config();
    ok.joints[2].command_min_rad = -0.10f;
    ok.joints[2].command_max_rad = 0.20f;
    const auto ok_report = validate(ok, ConfigAudience::offline_test);
    CHECK(ok_report.domains[domain_index(Domain::pitch)] == ConfigState::complete);
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--golden") {
        // 由主机后端打印, 这里只保证协议契约自身可运行.
        return 0;
    }
    session_and_replay_contracts();
    session_lifetime_contracts();
    rejected_activity_contracts();
    activity_boundary_contracts();
    expired_request_order_contracts();
    expired_retransmission_contracts();
    failure_envelope_contracts();
    session_traffic_does_not_extend_target_contracts();
    wrong_session_cache_isolation_contracts();
    maintenance_session_replacement_contracts();
    manual_continuous_align_contracts();
    epoch_and_timeout_contracts();
    scoped_reference_invalidation_contracts();
    accept_versus_execution_contracts();
    parameter_contracts();
    mutation_stream_and_frames_contracts();
    reply_mux_contracts();
    diagnostics_contracts();
    command_band_clamps_targets();
    command_band_validation();
    if (failures != 0) {
        std::cout << failures << " contract(s) failed\n";
        return 1;
    }
    std::cout << "sentinel business protocol contracts passed\n";
    return 0;
}
