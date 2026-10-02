#pragma once
#include "gm6020_bench_protocol.hpp"
#include <algorithm>
#include <array>
#include <string_view>

namespace ec::sentinel {
inline constexpr std::uint8_t bench_trace_request=0x26, bench_trace_response=0xa1;
inline constexpr std::uint32_t bench_trace_layout=0x42545231; // BTR1
inline constexpr std::size_t bench_trace_capacity=1024, bench_trace_bytes=64, bench_trace_header=32;
inline constexpr std::uint32_t bench_ident_layout=0x42494431; // BID1
inline constexpr std::size_t bench_ident_bytes=24, bench_ident_capacity=65536/bench_ident_bytes;
enum TraceFlags : std::uint8_t {
    trace_active=1, trace_feedback=2, trace_position=4, trace_pid=8, trace_saturated=16,
    trace_target_event=32, trace_reference_event=64, trace_tx_attempted=128
};
// 一个真实控制周期一条. 两类序号低16位用于短窗内判缺口; 元数据另含32位反馈序号.
// mode_reason 的低2位是模式, bit2 外环饱和, bit3 斜坡, 高4位是停止原因.
struct BenchTraceSample {
    std::uint32_t control_us{},feedback_us{},submit_us{},dt_us{};
    std::uint16_t feedback_sequence{},encoder{};
    std::int16_t rpm{},effort_raw{},command_raw{};
    std::uint8_t flags{},mode_reason{};
    float target{},position{},velocity_used{},velocity_target{},p_norm{},i_norm{},d_norm{},unconstrained_norm{};
    std::uint8_t tx_status{},axis_status{};
    std::uint16_t cycle_sequence{};
};
static_assert(sizeof(BenchTraceSample)==bench_trace_bytes);

// 只拍摄刚计算过的内环. 开环/无反馈/拒绝周期用有效位区分, 不伪造零测量.
inline BenchTraceSample bench_trace_sample(const Gm6020Bench& bench,ec::Tick now) noexcept {
    const auto state=bench.device().state(now);
    BenchTraceSample r{};
    r.control_us=bench.control_us().us; r.feedback_us=bench.feedback_us().us; r.dt_us=bench.dt_us();
    r.feedback_sequence=static_cast<std::uint16_t>(bench.feedback_sequence());
    r.cycle_sequence=static_cast<std::uint16_t>(bench.cycle_sequence());
    r.encoder=bench.encoder(); r.rpm=bench.rpm(); r.effort_raw=state.rotor.value.effort_raw;
    r.flags=(bench.active()?trace_active:0) |
        (state.velocity_rad_s.fresh_at(now,Gm6020Bench::feedback_ttl)?trace_feedback:0) |
        (state.position_rad.fresh_at(now,Gm6020Bench::feedback_ttl)?trace_position:0) |
        ((bench.events()&1)?trace_target_event:0) | ((bench.events()&2)?trace_reference_event:0);
    r.mode_reason=static_cast<std::uint8_t>(bench.settings().mode) |
        (static_cast<std::uint8_t>(bench.reason())<<4) | (bench.ramping()?8:0);
    r.target=bench.applied_target(); r.position=state.position_rad.value;
    r.velocity_used=state.velocity_rad_s.value; r.axis_status=static_cast<std::uint8_t>(bench.axis_status());
    if (const auto* pid=bench.velocity_diagnostics(); pid && pid->valid) {
        r.flags|=trace_pid;
        r.velocity_target=pid->target; r.velocity_used=pid->measured;
        // 原样保存控制器的归一化分项; A 换算基准由同窗参数元数据给出.
        r.p_norm=pid->p; r.i_norm=pid->i; r.d_norm=pid->d; r.unconstrained_norm=pid->unconstrained;
        if (pid->unconstrained!=pid->output) { r.flags|=trace_saturated; }
        if (const auto* outer=bench.position_diagnostics(); outer && outer->valid && outer->unconstrained!=outer->output) {
            r.mode_reason|=4;
        }
    } else if (bench.active() && bench.settings().mode==BenchMode::current) { r.unconstrained_norm=bench.current()/bench.settings().current_limit; }
    return r;
}
inline void encode_bench_trace(const BenchTraceSample& r,std::span<std::uint8_t> out) noexcept {
    using namespace bench_wire;
    u32(out,0,r.control_us);u32(out,4,r.feedback_us);u32(out,8,r.submit_us);u32(out,12,r.dt_us);
    u16(out,16,r.feedback_sequence);u16(out,18,r.encoder);u16(out,20,static_cast<std::uint16_t>(r.rpm));
    u16(out,22,static_cast<std::uint16_t>(r.effort_raw));u16(out,24,static_cast<std::uint16_t>(r.command_raw));
    out[26]=r.flags;out[27]=r.mode_reason;
    std::size_t at=28;
    for (const auto v:{r.target,r.position,r.velocity_used,r.velocity_target,r.p_norm,r.i_norm,r.d_norm,r.unconstrained_norm}) {
        f32(out,at,v);at+=4;
    }
    out[60]=r.tx_status;out[61]=r.axis_status;u16(out,62,r.cycle_sequence);
}

// 辨识长窗保留每周期实际目标/命令/原始运动和时间, 不抽样、不存并未观测的PID分项.
inline void encode_bench_ident(const BenchTraceSample& r,std::span<std::uint8_t> out) noexcept {
    using namespace bench_wire;
    u32(out,0,r.control_us);u32(out,4,r.feedback_us);f32(out,8,r.target);
    u16(out,12,r.encoder);u16(out,14,static_cast<std::uint16_t>(r.rpm));
    u16(out,16,static_cast<std::uint16_t>(r.command_raw));out[18]=r.flags;out[19]=r.mode_reason;
    u16(out,20,r.cycle_sequence);out[22]=r.tx_status;out[23]=r.axis_status;
}
// 记录传给真实 CanTx 的载荷与调用时刻, 而不是在观测端再编码一次来充当发送证据.
class BenchObservedTx final : public ec::transport::CanTx {
public:
    using Clock=ec::MicroTick(*)();
    BenchObservedTx(ec::transport::CanTx& output,Clock clock) noexcept : output_(output),clock_(clock) {}
    void reset() noexcept { attempted_=false; }
    ec::transport::SendResult try_send(const ec::transport::CanFrame& frame) noexcept override {
        at_=clock_(); frame_=frame; attempted_=true; return output_.try_send(frame);
    }
    void annotate(BenchTraceSample& r,const Gm6020Bench& bench,ec::drivers::DjiGroupStatus result) const noexcept {
        r.tx_status=static_cast<std::uint8_t>(result);
        r.flags=static_cast<std::uint8_t>((r.flags & ~trace_active) | (bench.active()?trace_active:0));
        r.mode_reason=static_cast<std::uint8_t>((r.mode_reason&15) | (static_cast<std::uint8_t>(bench.reason())<<4));
        if (attempted_) {
            r.flags|=trace_tx_attempted; r.submit_us=at_.us;
            const auto at=2*bench.slot();
            r.command_raw=static_cast<std::int16_t>((static_cast<unsigned>(frame_.data[at])<<8)|frame_.data[at+1]);
        }
    }
private:
    ec::transport::CanTx& output_;
    Clock clock_;
    ec::transport::CanFrame frame_{};
    ec::MicroTick at_{};
    bool attempted_{};
};

enum class BenchTraceState : std::uint8_t { idle,armed,recording,frozen };
// S5 独占的固定 RAM. 采集与命令共享一个任务; 记录器从不改变控制目标或使能.
class BenchTrace {
public:
    [[nodiscard]] BenchTraceState state() const noexcept { return state_; }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    void push(const BenchTraceSample& sample,const Gm6020Bench& bench,ec::Tick now,std::uint32_t tx_errors) noexcept {
        if (state_==BenchTraceState::armed && (!trigger_ || (sample.flags&trace_target_event))) {
            state_=BenchTraceState::recording; settings_epoch_=bench.settings_epoch();
            first_us_=sample.control_us;
            (void)encode_bench_status(bench,now,sample.tx_status,tx_errors,start_);
        }
        if (state_!=BenchTraceState::recording) { return; }
        // 参数集变了只结束旧日志, 绝不阻止新测试. 元数据标出边界, 不混用两组增益.
        if (settings_epoch_!=bench.settings_epoch()) { freeze(bench,now,sample.tx_status,tx_errors,2); return; }
        const auto bytes=record_bytes();
        auto out=std::span<std::uint8_t>{samples_}.subspan(size_*bytes,bytes);
        if (compact_) { encode_bench_ident(sample,out); } else { encode_bench_trace(sample,out); }
        ++size_;last_us_=sample.control_us;
        if (size_==goal_) { freeze(bench,now,sample.tx_status,tx_errors,0); }
    }
    [[nodiscard]] std::size_t request(std::span<const std::uint8_t> in,std::span<std::uint8_t> out,
        const Gm6020Bench& bench,ec::Tick now,std::uint8_t tx_status,std::uint32_t tx_errors,std::string_view build) noexcept {
        if (out.size()<bench_trace_header) { return 0; }
        using namespace bench_wire;
        const std::uint8_t op=in.size()>1 ? in[1] : 0;
        std::uint8_t error{};
        std::uint32_t arg{};
        if (in.size()!=12 || (in[0]!=1 && in[0]!=2) || op>4 ||
            (in[0]==1 ? in[3]!=0 : in[3]>1)) { error=1; }
        else {
            const auto run=get32(in,4);arg=get32(in,8);
            if (op==1) {
                const bool compact=in[0]==2 && in[3]==1;
                if (!run || !arg || arg>(compact?bench_ident_capacity:bench_trace_capacity) || in[2]>1) { error=1; }
                else if (run==run_) { if (goal_!=arg || trigger_!=in[2] || compact_!=compact) { error=2; } }
                else if (state_==BenchTraceState::armed || state_==BenchTraceState::recording) { error=3; }
                else {
                    compact_=compact;run_=run;goal_=static_cast<std::uint16_t>(arg);trigger_=in[2];size_=0;
                    first_us_=last_us_=0;reason_=0;start_.fill(0);end_.fill(0);state_=BenchTraceState::armed;
                }
            } else if (op && run!=run_) { error=2; }
            else if (op==2) { freeze(bench,now,tx_status,tx_errors,1); }
            else if (op>=3 && state_!=BenchTraceState::frozen) { error=3; }
            else if ((op==3 && arg>size_) || (op==4 && arg>1)) { error=1; }
        }
        std::fill_n(out.begin(),bench_trace_header,0);
        out[0]=compact_?2:1;out[1]=op;out[2]=error;out[3]=static_cast<std::uint8_t>(state_);
        u32(out,4,run_);u16(out,8,static_cast<std::uint16_t>(size_));
        u16(out,12,compact_?bench_ident_capacity:bench_trace_capacity);u16(out,14,goal_);
        u32(out,16,first_us_);u32(out,20,last_us_);u32(out,24,arg);out[28]=trigger_;out[29]=reason_;
        std::size_t n=bench_trace_header;
        if (!error && op==3) {
            const auto bytes=record_bytes();
            const auto count=std::min<std::size_t>({compact_?8u:3u,size_-arg,(out.size()-n)/bytes});
            u16(out,10,static_cast<std::uint16_t>(count));
            std::copy_n(samples_.begin()+arg*bytes,count*bytes,out.begin()+n);n+=count*bytes;
        } else if (!error && op==4) {
            constexpr std::size_t extra=bench_status_size+4+32;
            if (out.size()<n+extra) { out[2]=1;return n; }
            const auto& status=arg==0 ? start_ : end_;
            std::copy(status.begin(),status.end(),out.begin()+n);n+=status.size();
            u32(out,n,compact_?bench_ident_layout:bench_trace_layout);n+=4;
            std::fill_n(out.begin()+n,32,0);std::copy_n(build.begin(),std::min<std::size_t>(31,build.size()),out.begin()+n);n+=32;
            u16(out,30,extra);
        }
        return n;
    }
private:
    void freeze(const Gm6020Bench& bench,ec::Tick now,std::uint8_t tx_status,std::uint32_t tx_errors,std::uint8_t reason) noexcept {
        if (state_==BenchTraceState::idle || state_==BenchTraceState::frozen) { return; }
        (void)encode_bench_status(bench,now,tx_status,tx_errors,end_);
        state_=BenchTraceState::frozen;reason_=reason;
    }
    [[nodiscard]] std::size_t record_bytes() const noexcept { return compact_?bench_ident_bytes:bench_trace_bytes; }
    std::array<std::uint8_t,bench_trace_capacity*bench_trace_bytes> samples_{};
    bool compact_{};
    std::array<std::uint8_t,bench_status_size> start_{},end_{};
    BenchTraceState state_{};
    std::uint8_t trigger_{},reason_{};
    std::uint16_t goal_{};
    std::uint32_t run_{},first_us_{},last_us_{},settings_epoch_{};
    std::size_t size_{};
};
} // 命名空间 ec::sentinel
