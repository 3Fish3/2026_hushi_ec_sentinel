#include "raw_record.hpp"
#include <algorithm>

namespace ec::sentinel::recording {
void Recorder::push(Record r, MicroTick now) noexcept {
    advance(now);
    if (state_ != State::recording) { return; }
    std::size_t key{};
    std::uint32_t period{};
    switch (r.kind) {
    case Kind::gyro: key = 0; period = profile_ == Profile::rotation ? 2000 : 1000; break;
    case Kind::accel: key = 1; period = profile_ == Profile::rotation ? 10000 : 5000; break;
    case Kind::temperature: key = 2; break;
    case Kind::can:
        if (profile_ == Profile::stationary || (profile_ == Profile::rotation && r.stream != 4 && r.stream != 5 && r.stream != 9)) { return; }
        if (profile_ == Profile::control) {
            constexpr std::array<std::uint8_t, 10> mapping{0, 1, 2, 3, 4, 5, 9, 6, 7, 8};
            if (r.stream != mapping[selected_domain_]) { return; }
        }
        key = 3 + r.stream; period = profile_ == Profile::rotation ? 2000 : 1000; break;
    case Kind::control:
        if (profile_ != Profile::control || r.stream != selected_domain_) { return; }
        key = 13 + r.stream; period = 1000; break;
    }
    if (key >= last_.size()) { return; }
    if (seen_[key] && (r.source_sequence == sources_[key] || r.sampled_us - last_[key].us < period)) { return; }
    last_[key] = {r.sampled_us}; sources_[key] = r.source_sequence; seen_[key] = true;
    r.sequence = sequence_++; r.run = run_;
    records_[size_++] = r;
    if (size_ == capacity) { state_ = State::full; end_ = now; }
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
    // 固定候选安装映射, 不由 CAN ID 猜 C610/C620 比例. 未匹配帧仍由原有 collector 统计.
    constexpr std::array<std::uint16_t, 10> ids{0x201, 0x202, 0x203, 0x204, 0x20a, 0x208, 0x205, 0x206, 0x207, 0};
    for (std::size_t i = 0; i < ids.size(); ++i) {
        const auto expected_bus = i == 4 || i == 5 || i == 9 ? 1 : 2;
        if (bus != expected_bus || frame.id != ids[i]) { continue; }
        Record r{Kind::can, static_cast<std::uint8_t>(i), 4, 0, ++can_sequences_[i], at.us, at.us, lost, 0xffffffffu, 0};
        r.data[0] = bus | (static_cast<std::uint32_t>(frame.size) << 8) | (static_cast<std::uint32_t>(frame.id) << 16);
        for (std::size_t n = 0; n < 8; ++n) { r.data[1 + n / 4] |= static_cast<std::uint32_t>(frame.data[n]) << ((n % 4) * 8); }
        push(r, now); return;
    }
}
std::size_t Recorder::request(std::span<const std::uint8_t> in, std::span<std::uint8_t> out, MicroTick now) noexcept {
    if (in.size() != 12 || out.size() < reply_header || in[0] != version || in[1] > 3) { return 0; }
    advance(now);
    const auto op = in[1]; const auto run = get32(in, 4); const auto arg = get32(in, 8);
    std::uint8_t error{};
    if (op == 1) {
        if (!run || in[2] < 1 || in[2] > 3 || in[3] > 9 || (in[2] == 3 && in[3] == 6) || (in[2] != 3 && in[3] != 0) || arg == 0 || arg > 2000000u) { error = 1; }
        else if (run == run_) {
            if (in[2] != static_cast<std::uint8_t>(profile_) || in[3] != selected_domain_ || arg != duration_) { error = 2; }
        } else if (state_ == State::recording) { error = 3; }
        else {
            run_ = run; profile_ = static_cast<Profile>(in[2]); selected_domain_ = in[3]; start_ = end_ = now; duration_ = arg;
            size_ = 0; sequence_ = 0; lost_ = 0; seen_.fill(false); state_ = State::recording;
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
} // 命名空间 ec::sentinel::recording
