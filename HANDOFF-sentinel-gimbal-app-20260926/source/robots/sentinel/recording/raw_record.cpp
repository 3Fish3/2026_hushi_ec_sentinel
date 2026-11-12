#include "raw_record.hpp"
#include <algorithm>
#include <bit>

namespace ec::sentinel::recording {
void Recorder::push(Record r, MicroTick now) noexcept {
    advance(now);
    if (state_ != State::recording) { return; }
    // Profile4 exclusively admits atomic pairs via push_bundle().
    if (profile_ == Profile::autotune) { return; }
    // 连续 control 流只承载 Pitch 关节: IMU 记录在入口即剔除, 不挤占 200 Hz 控制带宽.
    const bool pitch_stream = continuous_ && profile_ == Profile::control;
    if (pitch_stream && r.kind != Kind::can && r.kind != Kind::control) { return; }
    std::size_t key{};
    std::uint32_t period{};
    switch (r.kind) {
    case Kind::gyro: key = 0; period = profile_ == Profile::rotation ? (continuous_ ? 4000 : 2000) : 1000; break;
    case Kind::accel: key = 1; period = profile_ == Profile::rotation ? 10000 : 5000; break;
    case Kind::temperature: key = 2; break;
    case Kind::can:
        if (profile_ == Profile::stationary || (profile_ == Profile::rotation && r.stream != 4 && r.stream != 5 && r.stream != 9)) { return; }
        if (profile_ == Profile::control) {
            constexpr std::array<std::uint8_t, 10> mapping{0, 1, 2, 3, 4, 5, 9, 6, 7, 8};
            if (r.stream != mapping[selected_domain_]) { return; }
        }
        key = 3 + r.stream; period = pitch_stream ? pitch_stream_period_us
            : (profile_ == Profile::rotation ? (continuous_ ? 4000 : 2000) : 1000); break;
    case Kind::control:
        if (profile_ != Profile::control || r.stream != selected_domain_) { return; }
        key = 13 + r.stream; period = pitch_stream ? pitch_stream_period_us : 1000; break;
    case Kind::autotune_state:
    case Kind::autotune_aux:
    default:
        return; // Bundles are never admitted as individual records.
    }
    if (key >= last_.size()) { return; }
    if (seen_[key] && (r.source_sequence == sources_[key] || r.sampled_us - last_[key].us < period)) { return; }
    last_[key] = {r.sampled_us}; sources_[key] = r.source_sequence; seen_[key] = true;
    r.sequence = sequence_++; r.run = run_;
    if (continuous_ && size_ == capacity) { ++lost_; return; }
    records_[(head_+size_)%capacity] = r; ++size_;
    if (!continuous_ && size_ == capacity) { state_ = State::full; end_ = now; }
}
void Recorder::push_bundle(Record state, Record aux, MicroTick now) noexcept {
    advance(now);
    if (state_ != State::recording) { return; }
    // Reject malformed pairs outright; never fall back to legacy single-record admission.
    if (state.kind != Kind::autotune_state || aux.kind != Kind::autotune_aux) { return; }
    if (profile_ != Profile::autotune || state.stream != selected_domain_) { return; }
    // pair 一致性: state 与 aux 必须同 stream/source_sequence/sampled_us, 否则拒绝 (不写任何一条).
    if (aux.stream != state.stream || aux.source_sequence != state.source_sequence ||
        aux.sampled_us != state.sampled_us) { return; }
    // 兜底去重: 相同 source_sequence 的 bundle 重复 (board 每 fastcycle 调 trace 而本周期未再计算)
    // 只写一次, 不增加槽. 按 selected domain 分槽, 与 legacy 单条 key(Domain) 不冲突.
    const std::size_t bkey = 20 + selected_domain_;
    if (bkey < last_.size() && seen_[bkey] && state.source_sequence == sources_[bkey]) { return; }
    // 成对不按去重周期丢弃: 每个控制周期都要一对, 靠序列单调而非时间去重.
    const std::size_t room = capacity - size_;
    if (room < 2) {
        if (continuous_) { lost_ += 2; return; }
        // 短窗: 整 bundle 放不下 -> 不写任何一条, 标记满并记录丢弃, 无孤立尾部.
        lost_ += 2; state_ = State::full; end_ = now; return;
    }
    if (bkey < last_.size()) { sources_[bkey] = state.source_sequence; seen_[bkey] = true; }
    state.sequence = sequence_++;
    state.run = run_;
    aux.sequence = sequence_++;
    aux.run = run_;
    records_[(head_ + size_) % capacity] = state; ++size_;
    records_[(head_ + size_) % capacity] = aux; ++size_;
    if (!continuous_ && size_ == capacity) { state_ = State::full; end_ = now; }
}
void Recorder::imu(const drivers::Bmi088Reading& sample, MicroTick now) noexcept {
    const auto& t = sample.times;
    const auto vec = [&](Kind kind, std::uint32_t seq, MicroTick at, MicroTick arrival,
        std::array<float, 3> values, std::uint32_t lost, std::uint32_t error) {
        Record r{kind, 0, 3, 0, seq, at.us, arrival.us, lost, error, 0};
        for (std::size_t i = 0; i < values.size(); ++i) { r.data[i] = std::bit_cast<std::uint32_t>(values[i]); }
        if (kind == Kind::gyro) { r.data[3] = t.gyro_overruns; }
        push(r, now);
    };
    if (t.gyro_updated && t.gyro_valid) { vec(Kind::gyro, t.gyro_sequence, t.gyro_estimated, t.gyro,
        {sample.sample.gyro_rad_s[0], sample.sample.gyro_rad_s[1], sample.sample.gyro_rad_s[2]}, t.gyro_discarded, t.gyro_time_error_us); }
    if (t.accel_updated && t.accel_valid) { vec(Kind::accel, t.accel_sequence, t.accel, t.accel,
        {sample.sample.acc_mps2[0], sample.sample.acc_mps2[1], sample.sample.acc_mps2[2]}, 0, 1250); }
    if (t.temperature_updated && t.temperature_valid) { vec(Kind::temperature, t.temperature_sequence,
        t.temperature, t.temperature, {sample.sample.temp_c, 0, 0}, 0, 1280000); }
}
void Recorder::can(std::uint8_t bus, const transport::CanFrame& frame, MicroTick at, MicroTick now,
    std::uint32_t lost) noexcept {
    // 权威规范物理 CAN 流映射:
    // stream 0: CAN1 0x201 LF
    // stream 1: CAN1 0x202 RF
    // stream 2: CAN1 0x203 LB
    // stream 3: CAN1 0x204 RB
    // stream 4: CAN1 0x208 main_yaw
    // stream 5: CAN2 0x20a fire_yaw
    // stream 6: CAN2 0x205 frictionL
    // stream 7: CAN2 0x206 frictionR
    // stream 8: CAN2 0x207 feeder
    // stream 9: CAN2 0x011 Pitch (DM MST_ID)
    // 严格校验 bus 与 ID, 错误总线或未知 ID 均不映射, 严禁静默强制转换.
    // 记录库不依赖 capture 库以避免库依赖循环; 使用本地验证过的 constexpr 映射.
    struct CanonicalStream {
        std::uint8_t bus;
        std::uint16_t id;
    };
    constexpr std::array<CanonicalStream, 10> streams{{
        {1, 0x201}, {1, 0x202}, {1, 0x203}, {1, 0x204}, {1, 0x208},
        {2, 0x20a}, {2, 0x205}, {2, 0x206}, {2, 0x207}, {2, 0x011}
    }};
    for (std::size_t i = 0; i < streams.size(); ++i) {
        if (bus != streams[i].bus || frame.id != streams[i].id) { continue; }
        Record r{Kind::can, static_cast<std::uint8_t>(i), 4, 0, ++can_sequences_[i], at.us, at.us, lost, 0xffffffffu, 0};
        r.data[0] = bus | (static_cast<std::uint32_t>(frame.size) << 8) | (static_cast<std::uint32_t>(frame.id) << 16);
        for (std::size_t n = 0; n < 8; ++n) { r.data[1 + n / 4] |= static_cast<std::uint32_t>(frame.data[n]) << ((n % 4) * 8); }
        push(r, now); return;
    }
}
std::size_t Recorder::request(std::span<const std::uint8_t> in, std::span<std::uint8_t> out, MicroTick now) noexcept {
    if (in.size() != 12 || out.size() < reply_header || in[0] != version || in[1] > 3) { return 0; }
    advance(now);
    // 连续队列未结束时不允许短窗请求重置它; 老协议仍只处理独立冻结窗口.
    if (continuous_ && (state_ == State::recording || size_ != 0)) { return 0; }
    const auto op = in[1]; const auto run = get32(in, 4); const auto arg = get32(in, 8);
    std::uint8_t error{};
    if (op == 1) {
        // 短窗开放 control 域 6 (Pitch DM 关节域), 移除了历史排除 (in[2] == 3 && in[3] == 6)
        const bool autotune = in[2] == static_cast<std::uint8_t>(Profile::autotune);
        const bool bad_profile = autotune ? (in[3] != 5 && in[3] != 6)
            : (in[2] < 1 || in[2] > 3 || in[3] > 9 || (in[2] != 3 && in[3] != 0));
        if (!run || bad_profile || arg == 0 || arg > 2000000u) { error = 1; }
        else if (run == run_) {
            if (in[2] != static_cast<std::uint8_t>(profile_) || in[3] != selected_domain_ || arg != duration_) { error = 2; }
        } else if (state_ == State::recording) { error = 3; }
        else {
            run_ = run; profile_ = static_cast<Profile>(in[2]); selected_domain_ = in[3]; start_ = end_ = now; duration_ = arg;
            size_ = head_ = 0; continuous_ = false;
            sequence_ = 0; lost_ = 0; seen_.fill(false); state_ = State::recording;
        }
    } else if (op && run != run_) { error = 2; }
    else if (op == 2 && state_ == State::recording) { state_ = State::frozen; end_ = now; }
    std::size_t count{};
    if (op == 3 && !error) {
        if (state_ == State::recording || arg > size_) { error = 3; }
        else { count = std::min<std::size_t>({3, size_ - static_cast<std::size_t>(arg), (out.size() - reply_header) / record_bytes}); }
    }
    std::fill(out.begin(), out.begin() + reply_header, 0);
    out[0] = version; out[1] = op; out[2] = error; out[3] = static_cast<std::uint8_t>(state_);
    put32(out, 4, run_); out[8] = static_cast<std::uint8_t>(profile_); out[9] = static_cast<std::uint8_t>(count);
    out[10] = static_cast<std::uint8_t>(size_); out[11] = static_cast<std::uint8_t>(size_ >> 8);
    put32(out, 12, start_.us); put32(out, 16, state_ == State::recording ? now.us : end_.us);
    put32(out, 20, lost_); put32(out, 24, layout_id); put32(out, 28, capacity);
    for (std::size_t i = 0; i < count; ++i) { (void)encode(records_[arg + i], out.subspan(reply_header + i * record_bytes)); }
    return reply_header + count * record_bytes;
}
void Recorder::stream_header(std::span<std::uint8_t> out, std::uint8_t op, std::uint8_t error,
    MicroTick now, std::size_t remaining) const noexcept {
    std::fill_n(out.begin(),32,0);
    out[0]=1; out[1]=op; out[2]=error;
    out[3]=!continuous_ ? 0 : state_==State::recording ? 1 : remaining ? 2 : 3;
    put32(out,4,run_); put32(out,8,sequence_); put32(out,12,lost_);
    put32(out,16,static_cast<std::uint32_t>(remaining)); put32(out,20,start_.us);
    put32(out,24,state_==State::recording ? now.us : end_.us); put32(out,28,layout_id);
}
std::size_t Recorder::stream_request(std::span<const std::uint8_t> in,
    std::span<std::uint8_t> out, MicroTick now) noexcept {
    if (in.size()!=12 || out.size()<32 || in[0]!=1 || in[1]>2) { return 0; }
    advance(now);
    const auto op=in[1]; const auto run=get32(in,4); const auto duration=get32(in,8);
    std::uint8_t error{};
    if (op==1) {
        // 连续流支持 Profile 1(stationary)/2(rotation) 以及 Profile 3(control) 的域 6 (Pitch 达妙 DM).
        // continuous control 仅开放域 6, 且 stream_packet 对 control 记录按完整 64B 承载 (不截断),
        // 两条流各限速 200 Hz, 使 UART 带宽维持在物理预算内.
        const bool autotune = in[2] == static_cast<std::uint8_t>(Profile::autotune);
        const bool bad_profile = autotune ? (in[3] != 5 && in[3] != 6)
            : (in[2]<1 || in[2]>3 || (in[2]==3 ? in[3]!=6 : in[3]!=0));
        if (!run || bad_profile || !duration || duration>1800000000u) { error=1; }
        else if (continuous_ && run==run_) {
            if (static_cast<std::uint8_t>(profile_)!=in[2] || selected_domain_!=in[3] || duration_!=duration) { error=2; }
        } else if (state_==State::recording || (continuous_ && size_)) { error=3; }
        else {
            continuous_=true;stream_finished_=false;head_=size_=0;
            run_=run;profile_=static_cast<Profile>(in[2]);selected_domain_=in[3];duration_=duration;
            start_=end_=now;sequence_=lost_=0;seen_.fill(false);state_=State::recording;
        }
    } else if (op==2) {
        if (!continuous_ || run!=run_) { error=2; }
        else if (state_==State::recording) { state_=State::frozen;end_=now; }
    }
    stream_header(out,op,error,now,size_);return 32;
}
std::size_t Recorder::stream_packet(std::span<std::uint8_t> out, MicroTick now,
    std::size_t& count) noexcept {
    count=0;advance(now);
    if (!continuous_ || stream_finished_ || out.size()<32 || (size_==0 && state_==State::recording)) { return 0; }
    std::size_t length=32;
    // 最多3条/帧; 仅省去未使用的payload尾部, 主机无损恢复标准64B记录.
    // 每条按自身类型定长: gyro 48B, CAN 44B, control/Kind6/7 完整 64B; 混合3条仍 <= 224B.
    for (;count<std::min<std::size_t>(3,size_);++count) {
        const auto& r=records_[(head_+count)%capacity];
        const std::size_t bytes=r.kind==Kind::gyro ? 48
            : (r.kind==Kind::control || r.kind==Kind::autotune_state || r.kind==Kind::autotune_aux) ? record_bytes : 44;
        if (out.size()-length<bytes) { break; }
        std::array<std::uint8_t,record_bytes> full{};
        (void)encode(r,full);std::copy_n(full.begin(),bytes,out.begin()+length);length+=bytes;
    }
    stream_header(out,3,0,now,size_-count);return length;
}
void Recorder::stream_consume(std::size_t count) noexcept {
    if (!continuous_ || count>size_) { return; }
    head_=(head_+count)%capacity;size_-=count;
    if (state_!=State::recording && size_==0) { stream_finished_=true; }
}
bool Recorder::pitch_stream_healthy(std::uint32_t run, MicroTick now) const noexcept {
    if (!continuous_ || state_ != State::recording) { return false; }
    // control 域 4/5/6 = main_yaw/fire_yaw/pitch, 三者都支持连续激励采集.
    if (profile_ != Profile::control ||
        (selected_domain_ != 4 && selected_domain_ != 5 && selected_domain_ != 6)) { return false; }
    // run 精确匹配: 不匹配意味着已被新采集替换, 或已被停止/结束.
    if (!run || run != run_) { return false; }
    // 累计丢条或队列已满都表示数据链已不完整, 不允许再对外声称健康.
    if (lost_ || size_ == capacity) { return false; }
    const std::uint32_t elapsed = now.us - start_.us;
    // 半个 32 位回绕窗口之外属于时间戳倒退或歧义区, 无法无歧义判定.
    if (elapsed >= 0x80000000u) { return false; }
    // 越过截止时刻即到期 (查询不推进状态机, 因此在此自行判定).
    return elapsed < duration_;
}
bool Recorder::pitch_stream_ready(std::uint32_t run, std::uint64_t required_us,
    MicroTick now) const noexcept {
    if (!pitch_stream_healthy(run, now)) { return false; }
    const std::uint64_t elapsed = now.us - start_.us;
    const std::uint64_t remaining = static_cast<std::uint64_t>(duration_) - elapsed;
    return remaining >= required_us;
}
} // 命名空间 ec::sentinel::recording
