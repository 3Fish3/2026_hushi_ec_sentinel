#include "sentinel_protocol.hpp"
#include "sentinel_param_apply.hpp"
#include "sentinel_telemetry.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <cstring>

namespace ec::sentinel::app {
namespace {
[[nodiscard]] std::size_t index(Domain domain) noexcept { return domain_index(domain); }

void write_identity(PayloadWriter& w, const char* text) noexcept {
    std::array<std::uint8_t, identity_length> buffer{};
    if (text != nullptr) {
        for (std::size_t i = 0; i < buffer.size() && text[i] != '\0'; ++i) {
            buffer[i] = static_cast<std::uint8_t>(text[i]);
        }
    }
    w.bytes(buffer);
}

// 模式—域白名单定义在 sentinel_types.hpp: 协议层与应用层共用同一份, 不维护第二份近似规则.
} // 内部辅助

void PayloadWriter::u8(std::uint8_t value) noexcept {
    if (used + 1 > out.size()) { overflow = true; return; }
    out[used++] = value;
}
void PayloadWriter::u16(std::uint16_t value) noexcept {
    u8(static_cast<std::uint8_t>(value));
    u8(static_cast<std::uint8_t>(value >> 8));
}
void PayloadWriter::u32(std::uint32_t value) noexcept {
    u16(static_cast<std::uint16_t>(value));
    u16(static_cast<std::uint16_t>(value >> 16));
}
void PayloadWriter::f32(float value) noexcept { u32(std::bit_cast<std::uint32_t>(value)); }
void PayloadWriter::bytes(std::span<const std::uint8_t> value) noexcept {
    for (auto byte : value) { u8(byte); }
}

std::uint8_t PayloadReader::u8() noexcept {
    if (used + 1 > in.size()) { underflow = true; return 0; }
    return in[used++];
}
std::uint16_t PayloadReader::u16() noexcept {
    const auto low = u8();
    const auto high = u8();
    return static_cast<std::uint16_t>(low | (static_cast<unsigned>(high) << 8));
}
std::uint32_t PayloadReader::u32() noexcept {
    const auto low = u16();
    const auto high = u16();
    return static_cast<std::uint32_t>(low | (static_cast<std::uint32_t>(high) << 16));
}
float PayloadReader::f32() noexcept { return std::bit_cast<float>(u32()); }

SentinelHostSession::LinkState SentinelHostSession::link_state(Tick now) noexcept {
    LinkState out{};
    if (!has_session_) { return out; }
    if (expire_if_stale(now)) { out.expired = true; return out; }
    if (link_expired_) { return out; }
    out.alive = true;
    out.request = request_;
    out.epoch = epoch_;
    return out;
}

SentinelHostSession::SessionCheck SentinelHostSession::check_session(std::uint32_t session,
    std::uint32_t seq, bool non_idempotent, Tick now) noexcept {
    if (!has_session_ || session != session_id_) { return {ResultCode::no_session, false}; }
    // 会话链路已超时: 必须先按**当前时刻**判定到期才能刷新收包时间.
    // 否则一个到期后的新序号请求会先把 last_request_at_ 刷到现在, 使后续老化误判为新鲜.
    static_cast<void>(expire_if_stale(now));
    if (link_expired_) { return {ResultCode::no_session, false}; }
    // 同一序号重复: 非幂等请求已在 handle() 用缓存响应处理, 这里只可能是幂等请求
    // (或缓存缺失) 的重发 —— 统一按重放拒绝, 不重复执行也不谎报成功.
    if (has_seq_ && (seq == last_seq_ || seq < last_seq_)) {
        static_cast<void>(non_idempotent);
        ++replay_rejects_;
        return {ResultCode::replay, false};
    }
    last_seq_ = seq;
    has_seq_ = true;
    // 消耗新序号并允许缓存拒绝结果, 但业务接受前不刷新链路时效.
    request_validated_ = true;
    return {ResultCode::accepted, true};
}

bool SentinelHostSession::expire_if_stale(Tick now) noexcept {
    if (!has_session_ || link_expired_) { return false; }
    // 会话按真实收包时刻老化; 未配置链路时限时不做老化 (由调用方保证配置齐备).
    if (!link_ttl_.valid() || fresh(now, last_request_at_, link_ttl_)) { return false; }
    // 链路超时: 撤销本会话授予来源的会话状态, 并丢掉本会话身份.
    // 必须按 role 清理: 只清 host 会留下维护会话的内部 bool, 与调用方输入 OR 后保活.
    link_expired_ = true;
    ++expired_sessions_;
    app_->clear_role_session(role_source_);
    role_source_ = Source::none;
    request_ = false;
    last_result_ = {}; mutation_result_ = {};
    return true;
}

bool SentinelHostSession::non_idempotent_request(MessageType type) noexcept {
    switch (type) {
    case MessageType::authorize:
    case MessageType::targets:
    case MessageType::align:
    case MessageType::param_write:
    case MessageType::permit: return true;
    default: return false;
    }
}

bool SentinelHostSession::ResultCache::matches(std::span<const std::uint8_t> payload) const noexcept {
    return request_size != 0 && payload.size() == request_size &&
        std::memcmp(request.data(), payload.data(), request_size) == 0;
}

void SentinelHostSession::ResultCache::store(std::uint32_t sequence,
    std::span<const std::uint8_t> payload, std::span<const std::uint8_t> result) noexcept {
    std::memcpy(request.data(), payload.data(), payload.size());
    std::memcpy(response.data(), result.data(), result.size());
    seq = sequence; request_size = payload.size(); response_size = result.size();
}

bool SentinelHostSession::domain_idle(Domain domain) const noexcept {
    return !app_->snapshot().executions[index(domain)].execute;
}

bool SentinelHostSession::all_domains_idle() const noexcept {
    const auto& snapshot = app_->snapshot();
    for (std::size_t i = 0; i < domain_count; ++i) {
        if (snapshot.executions[i].execute) { return false; }
    }
    return true;
}

bool SentinelHostSession::requester_holds() const noexcept {
    const auto& authority = app_->authority();
    return authority.holds() && authority.source() == role_source_ && role_source_ != Source::none;
}

bool SentinelHostSession::requester_may_configure() const noexcept {
    const auto& authority = app_->authority();
    if (!authority.holds()) { return true; } // 无人持权: 允许观察与配置
    return authority.source() == role_source_ && role_source_ != Source::none;
}

std::size_t SentinelHostSession::ack(MessageType type, std::uint32_t session, std::uint32_t seq,
    ResultCode result, std::uint8_t reason, std::span<std::uint8_t> response) noexcept {
    PayloadWriter w{response};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(type));
    w.u32(session);
    w.u32(seq);
    w.u8(static_cast<std::uint8_t>(result));
    w.u8(reason);
    return w.overflow ? 0 : w.size();
}

std::size_t SentinelHostSession::handle_hello(std::span<std::uint8_t> response, Tick now,
    std::span<const std::uint8_t> payload) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); // 版本
    static_cast<void>(r.u8()); // 类型
    const auto nonce = r.u32();
    if (!r.exact()) { return ack(MessageType::ack, 0, 0, ResultCode::bad_length, 0, response); }
    // 新握手: 替换旧会话. 旧会话的执行授权与请求立即失效 (它替换的是同一个来源身份),
    // 但不触碰其它来源 (遥控目标不受影响). 目标清理由应用在来源变化时统一完成.
    app_->revoke_sessions();
    request_ = false;
    role_source_ = Source::none;
    link_expired_ = false;
    last_result_ = {}; mutation_result_ = {};
    // 首次握手建立身份 (沿用构造时登记的启动身份); 之后的握手才是替换, 推进序号.
    // 若首次握手也推进, 新会话的第一个请求边沿会被核心仲裁吞掉.
    if (has_session_) {
        ++session_counter_;
        epoch_ = compute_epoch(session_counter_);
    }
    session_id_ = epoch_;
    app_->set_host_session(false, false, epoch_); // 登记新会话身份
    has_session_ = true;
    has_seq_ = false;
    last_seq_ = 0;
    client_nonce_ = nonce;
    accept_activity(now);
    PayloadWriter w{response};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::hello_ack));
    w.u32(session_id_);
    w.u32(app_->config_epoch());
    w.u32(app_->reference_epoch());
    w.u32(now.ms);
    w.u32(boot_id_);
    w.u32(client_nonce_);
    w.u8(0); // role: 0 = 正式 APP (其它角色有各自独立协议)
    w.u8(app_->report().complete ? 1 : 0);
    w.u8(app_->startable() ? 1 : 0);
    w.u8(boot_identity_available() ? 1 : 0);
    write_identity(w, app_->build_identity());
    w.u8(static_cast<std::uint8_t>(app_->config().kind));
    return w.overflow ? 0 : w.size();
}

std::size_t SentinelHostSession::handle_authorize(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); // 版本
    static_cast<void>(r.u8()); // 类型
    const auto session = r.u32();
    const auto seq = r.u32();
    const auto grant = r.u8();
    const auto source = r.u8();
    if (!r.exact()) { return ack(MessageType::ack, session, seq, ResultCode::bad_length, 0, response); }
    const auto check = check_session(session, seq, true, now);
    if (!check.proceed) {
        return ack(MessageType::ack, session, seq, check.code, 0, response);
    }
    // 显式授权/撤销: 普通目标到达不会打开权限, 授权只能由本消息改变.
    if (grant != 0) {
        // 只接受上位机与维护会话; 遥控来源由拨杆选择, 不接受协议授权.
        const auto requested = static_cast<Source>(source);
        if (requested == Source::host || requested == Source::maintenance) {
            // 换角色时先撤销旧角色的会话状态, 否则两个来源的内部 bool 会同时为真.
            if (role_source_ != Source::none && role_source_ != requested) {
                app_->clear_role_session(role_source_);
            }
            if (requested == Source::host) {
                app_->set_host_session(true, true, epoch_);
            } else {
                app_->set_maintenance_session(true, true, epoch_);
            }
        } else {
            return ack(MessageType::ack, session, seq, ResultCode::invalid_value, 0, response);
        }
        role_source_ = requested;
        request_ = true;
    } else {
        // 撤销只清请求, 不改变拨杆选择的来源.
        app_->revoke_sessions();
        role_source_ = Source::none;
        request_ = false;
    }
    ++accepted_commands_;
    accept_activity(now);
    return ack(MessageType::ack, session, seq, ResultCode::accepted, 0, response);
}

std::size_t SentinelHostSession::handle_targets(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); // 版本
    static_cast<void>(r.u8()); // 类型
    const auto session = r.u32();
    const auto seq = r.u32();
    const auto config_epoch = r.u32();
    const auto reference_epoch = r.u32();
    const auto count = r.u8();
    TargetSnapshot snapshot{};
    std::array<bool, domain_count> seen{};
    bool bad = false;
    for (std::uint8_t i = 0; i < count; ++i) {
        const auto domain = r.u8();
        const auto mode = r.u8();
        const auto frame = static_cast<YawFrame>(r.u8());
        const auto value = r.f32();
        const auto age_ms = r.u16();
        if (domain >= domain_count || !mode_allowed_for(static_cast<Domain>(domain),
                static_cast<control::ControlMode>(mode))) {
            bad = true;
            continue;
        }
        if (seen[index(static_cast<Domain>(domain))]) {
            // 同一域重复出现: 意图不明确, 整包拒绝.
            bad = true;
            continue;
        }
        seen[index(static_cast<Domain>(domain))] = true;
        snapshot.domains[index(static_cast<Domain>(domain))] =
            DomainSetpoint{static_cast<control::ControlMode>(mode), value, age_ms, frame};
    }
    const auto twist_present = r.u8();
    if (twist_present > 1) { bad = true; }
    if (twist_present != 0) {
        const auto vx = r.f32();
        const auto vy = r.f32();
        const auto wz = r.f32();
        const auto age_ms = r.u16();
        snapshot.chassis = motion::PlanarTwist{vx, vy, wz};
        snapshot.chassis_age_ms = age_ms;
    }
    if (!r.exact()) { return ack(MessageType::ack, session, seq, ResultCode::bad_length, 0, response); }
    const auto check = check_session(session, seq, true, now);
    if (!check.proceed) {
        return ack(MessageType::targets_ack, session, seq, check.code, 0, response);
    }
    // 只有当前持权的来源会话可以写目标: "有人持权"不等于"本会话有权".
    if (!requester_holds()) {
        return ack(MessageType::targets_ack, session, seq, ResultCode::not_permitted, 0, response);
    }
    if (bad) {
        return ack(MessageType::targets_ack, session, seq, ResultCode::invalid_value, 0, response);
    }
    // 代次不匹配: 目标按旧参考/配置解释没有意义, 拒绝整包.
    if (config_epoch != app_->config_epoch() || reference_epoch != app_->reference_epoch()) {
        return ack(MessageType::targets_ack, session, seq, ResultCode::epoch_mismatch, 0, response);
    }
    const auto result = app_->accept_targets(snapshot, now, role_source_);
    ResultCode code = ResultCode::accepted;
    switch (result) {
    case AcceptResult::accepted: code = ResultCode::accepted; break;
    case AcceptResult::no_authority: code = ResultCode::not_permitted; break;
    case AcceptResult::manual_stop: code = ResultCode::not_permitted; break;
    case AcceptResult::not_configured: code = ResultCode::rejected; break;
    case AcceptResult::invalid_target: code = ResultCode::invalid_value; break;
    case AcceptResult::limit_violation: code = ResultCode::invalid_value; break;
    case AcceptResult::degraded: code = ResultCode::not_permitted; break;
    }
    if (code == ResultCode::accepted) { ++accepted_commands_; accept_activity(now); }
    return ack(MessageType::targets_ack, session, seq, code, 0, response);
}

std::size_t SentinelHostSession::handle_align(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); // 版本
    static_cast<void>(r.u8()); // 类型
    const auto session = r.u32();
    const auto seq = r.u32();
    const auto domain = r.u8();
    const auto kind = r.u8();
    const auto value = r.f32();
    const auto reference_epoch = r.u32();
    if (!r.exact()) { return ack(MessageType::ack, session, seq, ResultCode::bad_length, 0, response); }
    const auto check = check_session(session, seq, true, now);
    if (!check.proceed) {
        return ack(MessageType::ack, session, seq, check.code, 0, response);
    }
    // 对齐是参考/配置类操作: 必须来自当前持权来源 (或当前无人持权), 且相关机构已退出执行.
    if (!requester_may_configure()) {
        return ack(MessageType::ack, session, seq, ResultCode::not_permitted, 0, response);
    }
    if (domain >= domain_count ||
        kind > static_cast<std::uint8_t>(AlignKind::inertial_alignment)) {
        return ack(MessageType::ack, session, seq, ResultCode::invalid_value, 0, response);
    }
    const auto id = static_cast<Domain>(domain);
    const auto align_kind = static_cast<AlignKind>(kind);
    if (align_kind == AlignKind::inertial_alignment && id != Domain::main_yaw) {
        // 惯性/odom 对齐只对主 yaw 有意义; 机械关节参考走另一种 kind.
        return ack(MessageType::ack, session, seq, ResultCode::invalid_value, 0, response);
    }
    if (!domain_idle(id)) {
        return ack(MessageType::ack, session, seq, ResultCode::reference_in_use, 0, response);
    }
    if (executor_ == nullptr || perception_ == nullptr) {
        return ack(MessageType::ack, session, seq, ResultCode::rejected, 0, response);
    }
    if (align_kind == AlignKind::joint_reference) {
        const auto before = executor_->joint_feedback(id, now).position_rad;
        if (!executor_->align_reference(id, value, now)) {
            return ack(MessageType::ack, session, seq, ResultCode::align_failed, 0, response);
        }
        const auto after = executor_->joint_feedback(id, now).position_rad;
        if (app_->config().serial && (id == Domain::fire_yaw || id == Domain::pitch) &&
            (before.validity != Validity::valid || before.value != after.value)) {
            // 这是原参考的建立/纠正, 不能靠平移标定数值掩盖不可信的旧参考.
            perception_->invalidate_joint_reference(id);
            app_->invalidate_inertial_reference();
        }
    } else {
        // 惯性对齐: 只改 MCU 惯性参考与上位机 odom 的偏移, 不动机械零位/位置参考.
        perception_->set_yaw_alignment(value, reference_epoch);
    }
    // 参考语义改变: 该域代次与全局代次上推, 并只作废该域已有目标/保持
    // (对齐数值变化后同一数值指向不同物理角); 其它域的合法目标不受牵连.
    app_->invalidate_domain_reference(id);
    ++accepted_commands_;
    accept_activity(now);
    // 回包带上新代次: 客户端不必猜, 也不需要额外的状态查询.
    PayloadWriter w{response};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::ack));
    w.u32(session);
    w.u32(seq);
    w.u8(static_cast<std::uint8_t>(ResultCode::accepted));
    w.u8(0);
    w.u32(app_->reference_epoch());
    w.u32(app_->config_epoch());
    return w.overflow ? 0 : w.size();
}

std::size_t SentinelHostSession::handle_param_read(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); // 版本
    static_cast<void>(r.u8()); // 类型
    const auto session = r.u32();
    const auto seq = r.u32();
    const auto offset = r.u16();
    const auto requested = r.u16();
    if (!r.exact()) {
        // 错误包统一回 ACK 布局, 不借用 param_value 的成功类型: 否则接收方会按条目表解码.
        return ack(MessageType::ack, session, seq, ResultCode::bad_length, 0, response);
    }
    const auto check = check_session(session, seq, false, now);
    if (!check.proceed) {
        return ack(MessageType::ack, session, seq, check.code, 0, response);
    }
    const std::size_t total = params_->size();
    std::size_t count = 0;
    std::size_t first = 0;
    if (requested == 0xffffu) {
        // 全表按页读: 一页放不下的部分由客户端继续请求, 不静默截断.
        first = offset < total ? offset : total;
        const auto remaining = total - first;
        count = remaining < param_page_max ? remaining : param_page_max;
    } else if (params_->has(requested)) {
        first = 0;
        count = 1;
    } else {
        return ack(MessageType::ack, session, seq, ResultCode::unknown_param, 0, response);
    }
    PayloadWriter w{response};
    w.u8(protocol_version);
    w.u8(static_cast<std::uint8_t>(MessageType::param_value));
    accept_activity(now);
    w.u32(session);
    w.u32(seq);
    w.u16(static_cast<std::uint16_t>(count));
    w.u16(static_cast<std::uint16_t>(first));
    w.u16(static_cast<std::uint16_t>(total));
    for (std::size_t i = 0; i < count; ++i) {
        const ParamEntry* entry = requested == 0xffffu ? &params_->at(first + i)
                                                       : params_->entry(requested);
        if (entry == nullptr) { continue; }
        w.u16(entry->spec.id);
        w.u8(static_cast<std::uint8_t>(entry->spec.kind));
        w.u8(static_cast<std::uint8_t>((entry->overridden ? 1 : 0) |
            (static_cast<std::uint8_t>(entry->spec.domain) << 1) | (entry->spec.whole_vehicle ? 32 : 0) |
            (param_has_apply_path(*app_, entry->spec.id) ? 64 : 0)));
        w.f32(entry->value);
        w.f32(entry->spec.min_value);
        w.f32(entry->spec.max_value);
    }
    return w.overflow ? 0 : w.size();
}

std::size_t SentinelHostSession::handle_param_write(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); // 版本
    static_cast<void>(r.u8()); // 类型
    const auto session = r.u32();
    const auto seq = r.u32();
    const auto id = r.u16();
    const auto value = r.f32();
    // 写参数的回包始终带实际生效值与当前代次: 参考类参数成功后会推进参考代次,
    // 客户端必须能立即拿到新版本, 否则下一次目标仍用旧代次下发.
    const auto reply = [&](ResultCode code) {
        const auto* entry = params_->entry(id);
        PayloadWriter w{response};
        w.u8(protocol_version);
        w.u8(static_cast<std::uint8_t>(MessageType::ack));
        w.u32(session);
        w.u32(seq);
        w.u8(static_cast<std::uint8_t>(code));
        w.u8(0);
        w.f32(entry != nullptr ? entry->value : 0.0f);
        w.u32(app_->reference_epoch());
        w.u32(app_->config_epoch());
        return w.overflow ? 0 : w.size();
    };
    if (!r.exact()) {
        return ack(MessageType::ack, session, seq, ResultCode::bad_length, 0, response);
    }
    const auto check = check_session(session, seq, true, now);
    if (!check.proceed) { return reply(check.code); }
    // 参数写入是配置类操作: 必须来自当前持权来源 (或当前无人持权).
    if (!requester_may_configure()) {
        return reply(ResultCode::not_permitted);
    }
    const auto* spec = params_->find(id);
    if (spec == nullptr) {
        return reply(ResultCode::unknown_param);
    }
    // 参考类参数要求受影响机构已退出执行; 全车参数必须所有域都空闲, 不能因为
    // 其中一个域空闲就放行整车语义的修改.
    const bool idle = spec->whole_vehicle ? all_domains_idle() : domain_idle(spec->domain);
    const auto result = write_param(id, value, idle);
    ResultCode code = ResultCode::accepted;
    switch (result) {
    case ParamWriteResult::accepted: code = ResultCode::accepted; break;
    case ParamWriteResult::unknown_param: code = ResultCode::unknown_param; break;
    case ParamWriteResult::invalid_value: code = ResultCode::invalid_value; break;
    case ParamWriteResult::reference_in_use: code = ResultCode::reference_in_use; break;
    case ParamWriteResult::not_configured: code = ResultCode::rejected; break;
    case ParamWriteResult::unsupported: code = ResultCode::unsupported; break;
    }
    if (code == ResultCode::accepted) {
        // 实际生效与读回回填已由参数事务完成; 只接受成功写入为链路活动.
        ++accepted_commands_;
        accept_activity(now);
    }
    return reply(code);
}

std::size_t SentinelHostSession::handle_permit(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); // 版本
    static_cast<void>(r.u8()); // 类型
    const auto session = r.u32();
    const auto seq = r.u32();
    const auto permit = r.u8();
    const auto ttl_ms = r.u16();
    if (!r.exact()) { return ack(MessageType::ack, session, seq, ResultCode::bad_length, 0, response); }
    const auto check = check_session(session, seq, true, now);
    if (!check.proceed) {
        return ack(MessageType::ack, session, seq, check.code, 0, response);
    }
    // 发射许可属于运行授权: 必须来自当前持权来源 (或当前无人持权).
    if (!requester_may_configure()) {
        return ack(MessageType::ack, session, seq, ResultCode::not_permitted, 0, response);
    }
    if (permit > 1 || ttl_ms == 0) {
        return ack(MessageType::ack, session, seq, ResultCode::invalid_value, 0, response);
    }
    app_->set_launch_permit(permit != 0, now, Lifetime{ttl_ms});
    ++accepted_commands_;
    accept_activity(now);
    return ack(MessageType::ack, session, seq, ResultCode::accepted, 0, response);
}

std::size_t SentinelHostSession::handle_status(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); static_cast<void>(r.u8());
    const auto session = r.u32(); const auto seq = r.u32();
    const auto page = static_cast<StatusPage>(r.u8());
    if (!r.exact()) { return ack(MessageType::ack, session, seq, ResultCode::bad_length, 0, response); }
    const auto check = check_session(session, seq, false, now);
    if (!check.proceed) { return ack(MessageType::ack, session, seq, check.code, 0, response); }
    if (page > StatusPage::requests) { return ack(MessageType::ack, session, seq, ResultCode::invalid_value, 0, response); }
    accept_activity(now);
    if (page == StatusPage::fast) {
        return encode_fast_status(*app_, perception_, now, session, seq, boot_id_, response);
    }
    const auto& snapshot = app_->snapshot();
    PayloadWriter w{response};
    w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(MessageType::status_snapshot));
    w.u32(session); w.u32(seq); w.u8(static_cast<std::uint8_t>(page));
    w.u32(snapshot.config_epoch); w.u32(snapshot.reference_epoch); w.u32(now.ms);
    if (page == StatusPage::domains) {
        w.u8(static_cast<std::uint8_t>(snapshot.permission.source));
        w.u8(snapshot.permission.authority ? 1 : 0); w.u8(snapshot.permission.manual_stop ? 1 : 0);
        w.u8(snapshot.launch_allowed ? 1 : 0); w.u8(static_cast<std::uint8_t>(domain_count));
        for (std::size_t i = 0; i < domain_count; ++i) {
            const auto& execution = snapshot.executions[i];
            const auto measured = domain_measurement(*app_, static_cast<Domain>(i), now);
            w.u8(static_cast<std::uint8_t>(snapshot.domains[i].validity));
            w.u8(static_cast<std::uint8_t>(execution.mode)); w.u8(execution.execute ? 1 : 0);
            w.u8(static_cast<std::uint8_t>(execution.reason)); w.u8(measured.frames);
            w.f32(execution.adopted_target.value_or(0.0f)); w.f32(measured.value);
            w.u32(measured.position_age_ms); w.u32(measured.velocity_age_ms);
        }
    } else {
        // 原请求与采用值分开. 原请求仍可能已过期, 不凭存在就声明正在执行.
        for (const auto& status : snapshot.domains) {
            const auto& target = status.target;
            w.u8(target ? 1 : 0);
            w.u8(target ? static_cast<std::uint8_t>(target->mode) : 0);
            w.u8(target ? static_cast<std::uint8_t>(target->frame) : 0);
            w.f32(target ? target->value : 0.0f);
            w.u32(target ? now.ms-target->accepted_at.ms : 0xffffffffu);
            w.u32(target ? target->ttl.ms : 0);
        }
        const OdomPose empty{};
        const auto& prediction = perception_ ? perception_->commanded_odom() : empty;
        w.u8(prediction.valid ? 1 : 0);
        w.f32(prediction.x_m); w.f32(prediction.y_m); w.f32(prediction.yaw_rad);
        w.u32(prediction.at.ms); w.u32(prediction.integrated_ms); w.u32(prediction.rejected_steps);
        const auto requested = snapshot.requested_twist, adopted = snapshot.adopted_twist;
        w.u8(static_cast<std::uint8_t>((requested ? 1u : 0u) | (adopted ? 2u : 0u)));
        for (const auto& value : {requested, adopted}) {
            const auto t = value.value_or(motion::PlanarTwist{});
            w.f32(t.vx_mps); w.f32(t.vy_mps); w.f32(t.wz_rad_s);
        }
        w.f32(snapshot.twist_scale);
    }
    return w.overflow ? 0 : w.size();
}

std::size_t SentinelHostSession::handle_diagnostics(std::span<std::uint8_t> response,
    std::span<const std::uint8_t> payload, Tick now) noexcept {
    PayloadReader r{payload};
    static_cast<void>(r.u8()); static_cast<void>(r.u8());
    const auto session = r.u32(), seq = r.u32(); const auto page = r.u8();
    if (!r.exact()) { return ack(MessageType::ack, session, seq, ResultCode::bad_length, 0, response); }
    const auto check = check_session(session, seq, false, now);
    if (!check.proceed) { return ack(MessageType::ack, session, seq, check.code, 0, response); }
    if (page > 2 && (page < 10 || page > 17) && page != 20) {
        return ack(MessageType::ack, session, seq, ResultCode::invalid_value, 0, response);
    }
    PayloadWriter w{response};
    w.u8(protocol_version); w.u8(static_cast<std::uint8_t>(MessageType::diagnostics_page));
    accept_activity(now);
    w.u32(session); w.u32(seq); w.u8(page);
    if (page == 0) {
        const auto& s = app_->stats();
        const std::array<std::uint32_t, 18> values{s.cycles, app_->rejected_packets(), app_->target_invalidations(),
            s.link_losses, s.source_changes, replay_rejects_, accepted_commands_, session_id_,
            app_->config_epoch(), app_->reference_epoch(), static_cast<std::uint32_t>(app_->report().issue_count),
            app_->report().complete ? 1u : 0u, frame_rejects_, duplicate_requests_, expired_sessions_, boot_id_,
            app_->config().timing.target_ttl.ms, link_ttl_.ms};
        w.u8(static_cast<std::uint8_t>(values.size()+tx_diagnostics_.size()));
        for (const auto value : values) { w.u32(value); }
        for (const auto value : tx_diagnostics_) { w.u32(value); }
    } else if (page == 1) {
        w.u8(static_cast<std::uint8_t>(dji_slot_count+dm_slot_count));
        for (std::size_t i=0; i<dji_slot_count+dm_slot_count; ++i) {
            std::uint8_t domain{}, flags{}, bus{}, protocol{}, temp1{}, temp2{};
            std::uint16_t device_id{}, feedback_id{};
            std::uint32_t age_ms{0xffffffffu}; float velocity{}, effort{};
            if (i < dji_slot_count) {
                const auto& c = app_->config().dji[i];
                domain = static_cast<std::uint8_t>(c.domain); flags = c.installed ? 1u : 0u;
                bus = c.bus; protocol = static_cast<std::uint8_t>(c.mode); device_id = c.device_id;
                if (const auto p = drivers::dji_protocol(c.mode)) {
                    feedback_id = static_cast<std::uint16_t>(p->rx_base+c.device_id);
                }
                auto* device = executor_ ? executor_->dji_device(i) : nullptr;
                if (device) {
                    flags |= 128u;
                    const auto state = device->state(now);
                    if (state.rotor.validity == Validity::valid) {
                        age_ms = now.ms-state.rotor.sampled_at.ms;
                        if (state.rotor.fresh_at(now, c.feedback_ttl)) { flags |= 2u; }
                        // DJI 原始 effort 不能套用命令电流满量程当成反馈 A.
                        effort = state.rotor.value.effort_raw; flags |= 64u;
                        if (state.rotor.value.temperature_c) { temp1 = *state.rotor.value.temperature_c; flags |= 16u; }
                    }
                    velocity = state.velocity_rad_s.value;
                }
            } else {
                const auto& c = app_->config().dm[0];
                domain = static_cast<std::uint8_t>(c.domain); flags = c.installed ? 1u : 0u;
                bus = c.bus; protocol = 128; device_id = c.can_id.value_or(0); feedback_id = c.master_id.value_or(0);
                auto* device = executor_ ? executor_->dm_device() : nullptr;
                if (device) {
                    flags |= 128u;
                    const auto state = device->state();
                    if (state.feedback.validity == Validity::valid) {
                        age_ms = now.ms-state.feedback.sampled_at.ms;
                        flags |= 16u|32u|64u;
                        temp1 = state.feedback.value.mos_temperature_c; temp2 = state.feedback.value.rotor_temperature_c;
                        effort = state.feedback.value.torque_nm; velocity = state.feedback.value.velocity_rad_s;
                    }
                    if (device->online(now)) { flags |= 2u; }
                    if (device->enabled(now)) { flags |= 4u; }
                    if (device->faulted(now)) { flags |= 8u; }
                }
            }
            w.u8(domain); w.u8(flags); w.u8(bus); w.u8(protocol); w.u16(device_id); w.u16(feedback_id);
            w.u32(age_ms); w.f32(velocity); w.f32(effort); w.u8(temp1); w.u8(temp2);
        }
    } else if (page == 2) {
        const auto s = executor_ ? executor_->stats() : ExecutorStats{};
        const auto p = perception_ ? perception_->stats() : ImuStats{};
        const WheelOdometry empty{};
        const auto& wheel = perception_ ? perception_->wheel_odom() : empty;
        const std::array<std::uint32_t, 36> values{s.cycles, s.enable_failures, s.target_rejections, s.unreachable_targets,
            s.group_sent, s.group_refused, s.group_tx_error, s.dm_enable_sent, s.dm_disable_sent,
            s.dm_command_sent, s.dm_tx_error, s.dm_query_sent, s.skipped_groups, s.timing_observed ? 1u : 0u,
            s.control_last_us, s.control_max_us, s.tx_last_us, s.tx_max_us, p.accepted, p.rejected_invalid,
            p.rejected_duplicate, p.rejected_regression, p.rejected_too_old, p.rejected_long_gap,
            p.reprime_events, p.extrapolations, p.invalidations, p.alignment_changes, p.accel_updates,
            p.accel_rejected, p.reconstruction_missing, p.projection_rejected, wheel.updates, wheel.gaps,
            wheel.integrated_us, wheel.continuous ? 1u : 0u};
        w.u8(static_cast<std::uint8_t>(values.size()+8));
        for (const auto value : values) { w.u32(value); }
        for (const auto sample : wheel.samples) { w.u32(sample.us); }
        for (const auto age : wheel.ages_us) { w.u32(age); }
    } else {
        const auto group = !executor_ ? std::optional<ExecutorGroupStats>{} :
            page == 20 ? (app_->config().dm[0].installed ? std::optional{executor_->pitch_stats()} : std::nullopt) :
            executor_->group_stats(page-10);
        w.u8(group ? 1 : 0);
        if (group) {
            w.u8(group->bus); w.u16(group->id); w.u8(static_cast<std::uint8_t>(group->last_result));
            w.u8(static_cast<std::uint8_t>((group->timing_observed ? 1 : 0) | (group->stop_only ? 2 : 0)));
            w.u32(group->computations); w.u32(group->submitted); w.u32(group->tx_failed); w.u32(group->invalid_steps);
            w.f32(group->last_dt_s); w.f32(group->max_dt_s);
            w.u32(group->last_computed.us); w.u32(group->last_submitted.us);
        }
    }
    return w.overflow ? 0 : w.size();
}

// 参数写入只经过一个入口: 生效事务负责落到运行对象并从运行对象读回实际值.
ParamWriteResult SentinelHostSession::write_param(std::uint16_t id, float value,
    bool idle) noexcept {
    if (params_ == nullptr) { return ParamWriteResult::not_configured; }
    return apply_param(*params_, *app_, executor_, perception_, id, value, idle);
}

std::size_t SentinelHostSession::handle(std::span<const std::uint8_t> request, Tick now,
    std::span<std::uint8_t> response) noexcept {
    // 业务载荷至少要有版本与类型两个字节: 外层分帧正确不代表业务头完整.
    if (request.size() < 2 || request.size() > protocol_payload_max) {
        return ack(MessageType::ack, 0, 0, ResultCode::bad_length, 0, response);
    }
    if (request[0] != protocol_version) {
        // 版本不符: 仍按请求类型回明确结果码, 让客户端能区分"版本错"而不是超时.
        return ack(MessageType::ack, 0, 0, ResultCode::bad_version, 0, response);
    }
    const auto type = static_cast<MessageType>(request[1]);
    request_validated_ = false;
    // 非幂等请求的同一序号重传: 先看缓存, 回放原完整响应 (含代次/生效值等元数据),
    // 不重新执行; 同序号不同内容按重放拒绝, 不拿旧结果冒充本次成功.
    std::uint32_t request_session = 0;
    std::uint32_t request_seq = 0;
    const bool cacheable = non_idempotent_request(type) && request.size() >= 10;
    if (cacheable) {
        PayloadReader r{request};
        static_cast<void>(r.u8());
        static_cast<void>(r.u8());
        request_session = r.u32();
        request_seq = r.u32();
        // 会话到期必须先于同序号缓存回放判定: 缓存短路不能绕过会话时效, 否则到期后的
        // 重传仍能拿到缓存里的 ACCEPTED/REPLAY, 与"到期后一律 NO_SESSION"的契约不一致.
        // 只有当前会话自己的请求才触发本会话的到期副作用, 未知身份不干扰它.
        if (has_session_ && request_session == session_id_) {
            static_cast<void>(expire_if_stale(now));
            if (link_expired_) {
                return ack(MessageType::ack, request_session, request_seq, ResultCode::no_session,
                    0, response);
            }
        }
        if (has_session_ && request_session == session_id_ && has_seq_ && request_seq <= last_seq_) {
            const auto& cached = mutation_request(type) ? mutation_result_ : last_result_;
            const bool eligible = mutation_request(type) || request_seq == last_seq_;
            if (eligible && cached.seq == request_seq && cached.matches(request)) {
                if (cached.response_size > response.size()) { return 0; }
                ++duplicate_requests_;
                std::memcpy(response.data(), cached.response.data(), cached.response_size);
                return cached.response_size; // 不更新 link/target/permit 租期, 不重新生效.
            }
            ++replay_rejects_;
            return ack(MessageType::ack, request_session, request_seq, ResultCode::replay, 0, response);
        }
    }
    std::size_t size = 0;
    switch (type) {
    case MessageType::hello: return handle_hello(response, now, request);
    case MessageType::authorize: size = handle_authorize(response, request, now); break;
    case MessageType::targets: size = handle_targets(response, request, now); break;
    case MessageType::align: size = handle_align(response, request, now); break;
    case MessageType::param_read: size = handle_param_read(response, request, now); break;
    case MessageType::param_write: size = handle_param_write(response, request, now); break;
    case MessageType::permit: size = handle_permit(response, request, now); break;
    case MessageType::status: size = handle_status(response, request, now); break;
    case MessageType::diagnostics: size = handle_diagnostics(response, request, now); break;
    default:
        return ack(MessageType::ack, 0, 0, ResultCode::unknown_request, 0, response);
    }
    // 本请求确实通过当前会话校验并被处理后才缓存完整响应;
    // 未处理/未通过校验的错误路径 (bad_length/no_session/重放) 不得改变另一请求的缓存.
    if (cacheable && request_validated_ && size != 0) {
        last_result_.store(request_seq, request, {response.data(), size});
        if (mutation_request(type)) { mutation_result_ = last_result_; }
    }
    return size;
}
} // 命名空间 ec::sentinel::app
