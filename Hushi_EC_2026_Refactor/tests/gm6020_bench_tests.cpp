#include "gm6020_bench_trace.hpp"
#include <iomanip>
#include <charconv>
#include <string_view>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>
// 失败时以非零码退出且不触发 CRT 交互弹窗, ctest 要求非交互
#define CHECK(x) do{if(!(x)){std::cerr<<__LINE__<<": " #x "\n";std::_Exit(3);}}while(false)
using namespace ec::sentinel;
// 既有毫秒用例显式生成合成微秒时钟; 新用例直接覆盖亚毫秒与回绕.
struct Bench : Gm6020Bench {
    using Gm6020Bench::Gm6020Bench;
    using Gm6020Bench::start;
    using Gm6020Bench::step;
    bool start(BenchSettings c,std::uint16_t token,ec::Tick at) { return Gm6020Bench::start(c,token,at,{at.ms*1000u}); }
    float step(ec::Tick at) { return Gm6020Bench::step(at,{at.ms*1000u}); }
};
static void feedback(Gm6020Bench& b,std::uint32_t ms,float angle=2.0f,float speed=0) {
    auto a=std::fmod(angle,2*std::numbers::pi_v<float>);if(a<0){a+=2*std::numbers::pi_v<float>;}
    const auto enc=static_cast<unsigned>(a*8192/(2*std::numbers::pi_v<float>))%8192;
    const auto rpm=static_cast<std::int16_t>(speed*60/(2*std::numbers::pi_v<float>));
    ec::transport::CanFrame f{};f.id=static_cast<std::uint16_t>(0x204+b.device().config().device_id);f.size=8;
    f.data={static_cast<std::uint8_t>(enc>>8),static_cast<std::uint8_t>(enc),
        static_cast<std::uint8_t>(rpm>>8),static_cast<std::uint8_t>(rpm),0,0,31,0};
    b.receive(1,f,{ms},{ms*1000u});
}
static void contracts() {
    Bench b;BenchSettings c{};c.target=0.1f;
    // 旧主 yaw 换算参数仍是台架默认值.
    CHECK(std::abs(c.current_limit-3000.0f*3/16384)<1e-6f);
    CHECK(std::abs(c.velocity_kp-73.0f*180/std::numbers::pi_v<float>*3/16384)<1e-6f);
    BenchSettings open=c;open.current_limit=3;open.target=2;open.duration_s=0;CHECK(Gm6020Bench::valid(open));
    open.mode=BenchMode::position;open.target=20;CHECK(Gm6020Bench::valid(open));
    // 没有反馈就不能开始.
    CHECK(!b.start(c,1,{100}));CHECK(b.request_result()==BenchReason::feedback_lost);
    feedback(b,100);
    CHECK(b.encoder()!=0);
    CHECK(b.start(c,1,{100}));CHECK(b.active());
    // 同一毫秒内重复调用不发旧命令, 也不中止本次运行.
    CHECK(b.step({100})==0);CHECK(b.active());
    feedback(b,110);CHECK(std::abs(b.step({110})-0.1f)<1e-6f);
    // 运行中拒绝重启.
    CHECK(!b.start(c,2,{110}));CHECK(b.request_result()==BenchReason::busy);CHECK(b.active());
    // 令牌不符的心跳不改状态.
    CHECK(!b.heartbeat(2,{120}));CHECK(b.active());CHECK(b.request_result()==BenchReason::invalid_request);
    CHECK(b.heartbeat(1,{120}));CHECK(b.request_result()==BenchReason::running);
    b.stop();CHECK(b.step({120})==0);CHECK(!b.heartbeat(1,{120}));
    c.target=std::numeric_limits<float>::quiet_NaN();CHECK(!b.start(c,3,{120}));
    c.target=1;CHECK(!Gm6020Bench::valid(c));c.target=0.1f;
    // 心跳到期: 严格小于有效期, 到期即停止且不能靠下一次心跳复活.
    feedback(b,200);CHECK(b.start(c,4,{200}));
    for(unsigned t=210;t<=700;t+=10){feedback(b,t);(void)b.step({t});}
    CHECK(!b.active());CHECK(b.reason()==BenchReason::command_timeout);
    feedback(b,710);CHECK(b.step({710})==0);CHECK(!b.heartbeat(4,{710}));
    // 反馈丢失.
    feedback(b,800);CHECK(b.start(c,5,{800}));
    for(unsigned t=810;t<=850;t+=10){(void)b.step({t});}
    CHECK(b.reason()==BenchReason::feedback_lost);
    // 控制周期异常: 单次间隔超过 30 ms.
    feedback(b,900);c.duration_s=0;CHECK(b.start(c,10,{900}));
    feedback(b,910);(void)b.step({910});
    feedback(b,960);(void)b.step({960});
    CHECK(!b.active());CHECK(b.reason()==BenchReason::timing);
    // 运行时长.
    feedback(b,1000);c.duration_s=0.1f;CHECK(b.start(c,6,{1000}));
    for(unsigned t=1010;t<=1100;t+=10){feedback(b,t);(void)b.step({t});}
    CHECK(b.reason()==BenchReason::duration);
    // 电流模式不再以目标速度上限隐式中止.
    feedback(b,1200);c.duration_s=3;CHECK(b.start(c,7,{1200}));feedback(b,1210,2,4);
    CHECK(b.step({1210})>0);CHECK(b.active());
    b.stop();
    // 运行中不心跳: 有效期到期即停止, 下一次心跳不能把它复活.
    feedback(b,1300);c.duration_s=0;CHECK(b.start(c,8,{1300}));
    CHECK(!b.heartbeat(8,{1800}));CHECK(!b.active());CHECK(b.reason()==BenchReason::command_timeout);
    // 位置模式: 没有显式参考就不允许开始.
    Bench pos;feedback(pos,100,0.0f);
    c.mode=BenchMode::position;c.target=0.2f;c.duration_s=0;
    CHECK(!pos.start(c,9,{100}));CHECK(pos.request_result()==BenchReason::position_lost);
    CHECK(pos.set_reference({100}));
    CHECK(pos.start(c,9,{100}));CHECK(pos.active());
    // 转过一点后停止: 参考与累计角保留.
    feedback(pos,110,0.4f);(void)pos.step({110});
    const float accumulated=pos.device().state({110}).position_rad.value;
    CHECK(std::abs(accumulated-0.4f)<0.001f);
    pos.stop();
    feedback(pos,120,0.4f);
    CHECK(pos.start(c,10,{120}));
    CHECK(std::abs(pos.device().state({120}).position_rad.value-accumulated)<1e-6f);
    // 不可信位置跳变使参考失效, 并报成位置丢失而不是速度反馈丢失.
    feedback(pos,130,0.4f+std::numbers::pi_v<float>);
    CHECK(pos.step({130})==0);CHECK(pos.reason()==BenchReason::position_lost);
    CHECK(!pos.device().state({130}).position_rad.fresh_at({130},{50}));
    // 控制时基回绕 (回到电流模式, 回绕本身与模式无关).
    c.mode=BenchMode::current;c.target=0.1f;c.duration_s=0;
    Bench wrap;feedback(wrap,0xfffffff0u);CHECK(wrap.start(c,1,{0xfffffff0u}));
    feedback(wrap,4);CHECK(wrap.step({4})>0);
}
static void closed_loop(BenchMode mode,float target) {
    Bench b;BenchSettings c{};c.mode=mode;c.target=target;c.duration_s=5;
    float angle=6.2f,speed=0;feedback(b,100,angle,speed);
    if(mode==BenchMode::position){CHECK(b.set_reference({100}));}
    CHECK(b.start(c,42,{100}));
    for(unsigned t=101;t<=4100;++t){
        feedback(b,t,angle,speed);if(t%100==0){CHECK(b.heartbeat(42,{t}));}
        const auto current=b.step({t});
        CHECK(b.active());CHECK(std::abs(current)<=c.current_limit+1e-6f);
        speed+=(40*current-speed)*(0.001f/0.15f);angle+=speed*0.001f;
    }
    if(mode==BenchMode::velocity){CHECK(std::abs(speed-target)<0.2f);}
    else {CHECK(std::abs((angle-6.2f)-target)<0.12f);}
    b.stop();CHECK(b.step({4110})==0);
}
static void status_vector(std::array<std::uint8_t,bench_status_size>& data) {
    Bench b;feedback(b,100);BenchSettings c{};c.target=0.1f;
    CHECK(b.set_reference({100})); // 报告里的相对角需要有效参考
    CHECK(b.start(c,42,{100}));
    feedback(b,110);(void)b.step({110});
    CHECK(encode_bench_status(b,{110},1,0,data)==bench_status_size);
    CHECK(!decode_bench_settings({}));
}
static ec::MicroTick submit_clock{};
static ec::MicroTick clock_us() { return submit_clock; }
struct Bus final : ec::transport::CanTx {
    ec::transport::CanFrame frame{};
    ec::transport::SendResult result{ec::transport::SendResult::accepted};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& f) noexcept override { frame=f;return result; }
};
static std::array<std::uint8_t,12> trace_request(std::uint8_t op,std::uint32_t run,std::uint32_t arg=0,std::uint8_t trigger=0) {
    std::array<std::uint8_t,12> q{1,op,trigger,0};bench_wire::u32(q,4,run);bench_wire::u32(q,8,arg);return q;
}
static void mapping_and_diagnostics() {
    // 每个合法 ID 都检查真实组编码字节, 不只检查设备上的逻辑电流.
    for(std::uint8_t id=1;id<=7;++id) {
        Bench b{id};Bus bus;BenchObservedTx tx{bus,clock_us};
        ec::drivers::DjiTxGroup group{tx,1,b.group_number(),{20},b.group_devices()};CHECK(group.configured());
        feedback(b,100);BenchSettings c{};c.target=-0.1f;CHECK(b.start(c,5,{100}));
        feedback(b,101);const auto amps=b.step({101});auto sample=bench_trace_sample(b,{101});
        std::array<std::optional<ec::drivers::DjiOutput>,4> out{};out[b.slot()]=ec::drivers::DjiOutput{amps/3,{101}};
        submit_clock={101017};const auto sent=group.send({101},out);tx.annotate(sample,b,sent.status);
        CHECK(sent.status==ec::drivers::DjiGroupStatus::accepted);CHECK(sample.submit_us==101017);
        CHECK(sample.flags&trace_tx_attempted);CHECK(!(sample.flags&trace_pid));CHECK(sample.command_raw==-546);
        CHECK(bus.frame.id==(id<=4?0x1fe:0x2fe));
        for(std::size_t slot=0;slot<4;++slot) {
            const auto raw=static_cast<std::int16_t>((bus.frame.data[2*slot]<<8)|bus.frame.data[2*slot+1]);
            CHECK(raw==(slot==b.slot()?-546:0));
        }
    }
    Bench invalid{8};Bus bus;ec::drivers::DjiTxGroup bad{bus,1,invalid.group_number(),{20},invalid.group_devices()};CHECK(!bad.configured());
    Bench b;feedback(b,100,0,0);CHECK(b.set_reference({100}));
    BenchSettings c{};c.mode=BenchMode::velocity;c.target=1;c.current_limit=1;
    c.velocity_kp=.2f;c.velocity_ki=.1f;c.velocity_kd=.001f;c.feedback_alpha=.25f;
    CHECK(b.start(c,8,{100},{0xffffff00u}));
    CHECK(b.step({100},{0xffffff00u})==0);CHECK(!b.velocity_diagnostics());
    (void)b.step({101},{0x1ee});CHECK(b.dt_us()==750);CHECK(b.active());
    auto r=bench_trace_sample(b,{101});CHECK(r.flags&trace_pid);CHECK(!(r.flags&trace_reference_event));
    CHECK(std::abs(r.p_norm-.2f)<1e-6f);CHECK(std::abs(r.i_norm-.000075f)<1e-7f);CHECK(r.d_norm==0);
    const auto epoch=b.reference_epoch();const auto config_epoch=b.settings_epoch();
    CHECK(b.update_target(8,2,0,{101},{0x1ef}));
    feedback(b,102,0,1);(void)b.step({102},{0x5d6});
    const auto d=bench_trace_sample(b,{102});CHECK(d.flags&trace_target_event);CHECK(d.d_norm<0);
    CHECK(d.i_norm>r.i_norm);CHECK(d.velocity_target==2);
    CHECK(std::abs(d.velocity_used-b.device().state({102}).velocity_rad_s.value*.75f)<1e-6f);
    CHECK(b.reference_epoch()==epoch);CHECK(b.settings_epoch()==config_epoch);
    CHECK(!b.update_target(9,1,0,{102},{0x5d6}));CHECK(b.settings().target==2);
    CHECK(!b.update_target(8,1,0x80000000u,{102},{0x5d6}));
    CHECK(b.update_target(8,-2,2000,{102},{0x5d6}));CHECK(b.heartbeat(8,{103}));
    feedback(b,103,0,0);(void)b.step({103},{0x9be});CHECK(std::abs(b.applied_target())<1e-5f);
    feedback(b,104,0,0);(void)b.step({104},{0xda6});CHECK(b.applied_target()==-2);CHECK(!b.ramping());
    b.stop();(void)b.step({105},{0x118e});r=bench_trace_sample(b,{105});
    CHECK(!(r.flags&trace_pid));CHECK(r.p_norm==0 && r.i_norm==0 && r.d_norm==0);
    CHECK(!b.update_target(8,0,0,{105},{0x118e}));
    // 位置模式读取级联内环, 它与独立速度环不是同一 PID 实例.
    c.mode=BenchMode::position;c.target=.4f;feedback(b,106,0,0);CHECK(b.start(c,9,{106}));
    feedback(b,107,0,0);(void)b.step({107});r=bench_trace_sample(b,{107});
    CHECK(r.flags&trace_pid);CHECK(std::abs(r.velocity_target-c.position_kp*.4f)<1e-5f);
    CHECK(r.i_norm>0);CHECK(!b.update_target(9,0,0,{606},{606000}));CHECK(!b.active());
}
static void print_hex(std::string_view name,std::span<const std::uint8_t> bytes) {
    std::cout<<name<<' ';
    for(auto value:bytes){std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(value);}std::cout<<'\n';
}
static void trace_contracts(bool vectors) {
    Bench b{4};BenchTrace trace;Bus bus;BenchObservedTx tx{bus,clock_us};
    ec::drivers::DjiTxGroup group{tx,1,b.group_number(),{20},b.group_devices()};
    std::array<std::uint8_t,252> reply{};
    auto n=trace.request(trace_request(1,55,6,1),reply,b,{99},0,0,"native-b0");CHECK(n==32);
    CHECK(trace.state()==BenchTraceState::armed);
    feedback(b,100,0,0);(void)b.step({100});trace.push(bench_trace_sample(b,{100}),b,{100},0);
    CHECK(trace.size()==0);CHECK(b.set_reference({100}));
    BenchSettings c{};c.mode=BenchMode::velocity;c.target=1;CHECK(b.start(c,8,{100}));
    for(std::uint32_t now=101;now<=106;++now) {
        feedback(b,now,0,now==102?.3f:0);const auto current=b.step({now});auto sample=bench_trace_sample(b,{now});
        tx.reset();submit_clock={now*1000u+21};
        std::array<std::optional<ec::drivers::DjiOutput>,4> outputs{};
        outputs[b.slot()]=ec::drivers::DjiOutput{current/3,{now}};
        if(now==104){bus.result=ec::transport::SendResult::busy;}
        else{bus.result=ec::transport::SendResult::accepted;}
        const auto result=group.send({now},outputs);
        if(result.status!=ec::drivers::DjiGroupStatus::accepted){b.stop(BenchReason::tx_failed);}
        tx.annotate(sample,b,result.status);const auto active=b.active();const auto amps=b.current();
        trace.push(sample,b,{now},now>=104?1:0);CHECK(active==b.active() && amps==b.current());
    }
    CHECK(trace.state()==BenchTraceState::frozen);CHECK(trace.size()==6);
    n=trace.request(trace_request(3,55,0),reply,b,{106},1,1,"native-b0");CHECK(n==224);
    CHECK(bench_wire::get16(reply,10)==3);CHECK(reply[32+26]&trace_pid);
    CHECK(reply[32+26]&trace_target_event);CHECK(reply[32+26]&trace_reference_event);
    CHECK(bench_wire::get32(reply,32)==101000);CHECK(bench_wire::get32(reply,40)==101021);
    CHECK(bench_wire::get16(reply,32+24)==3000);
    if(vectors){print_hex("data0",{reply.data(),n});}
    n=trace.request(trace_request(3,55,3),reply,b,{106},1,1,"native-b0");CHECK(n==224);
    CHECK(!(reply[32+26]&trace_active));CHECK(reply[32+26]&trace_pid); // 本周期算过, 但发送失败后停止.
    CHECK(reply[32+60]==static_cast<unsigned>(ec::drivers::DjiGroupStatus::busy));
    CHECK(!(reply[32+64+26]&trace_pid));CHECK(bench_wire::get16(reply,32+64+24)==0);
    if(vectors){print_hex("data3",{reply.data(),n});}
    for(std::uint32_t i=0;i<2;++i){
        n=trace.request(trace_request(4,55,i),reply,b,{106},1,1,"native-b0");CHECK(n==194);
        CHECK(bench_wire::get32(reply,32)==bench_magic);CHECK(bench_wire::get32(reply,32+bench_status_size)==bench_trace_layout);
        if(vectors){print_hex(i==0?"meta0":"meta1",{reply.data(),n});}
    }
    const auto frozen=trace.size();(void)trace.request(trace_request(1,55,6,1),reply,b,{106},1,1,"native-b0");CHECK(trace.size()==frozen);
    (void)trace.request(trace_request(3,55,7),reply,b,{106},1,1,"native-b0");CHECK(reply[2]==1);
    (void)trace.request(trace_request(3,54,0),reply,b,{106},1,1,"native-b0");CHECK(reply[2]==2);
    (void)trace.request({},reply,b,{106},1,1,"native-b0");CHECK(reply[2]==1);
    (void)trace.request(trace_request(1,56,1024),reply,b,{106},1,1,"native-b0");
    for(std::uint32_t now=107;now<1132;++now){feedback(b,now);(void)b.step({now});trace.push(bench_trace_sample(b,{now}),b,{now},1);}
    CHECK(trace.size()==1024);CHECK(trace.state()==BenchTraceState::frozen);
    n=trace.request(trace_request(3,56,1023),reply,b,{1132},1,1,"native-b0");CHECK(n==96);
    // 参数切换只截止旧日志, 不引入执行阻塞. armed/空窗结束也不启停电机.
    (void)trace.request(trace_request(1,57,5),reply,b,{1132},1,1,"native-b0");
    (void)b.step({1132});trace.push(bench_trace_sample(b,{1132}),b,{1132},1);CHECK(trace.size()==1);
    feedback(b,1133);CHECK(b.start(c,9,{1133}));(void)b.step({1134});trace.push(bench_trace_sample(b,{1134}),b,{1134},1);
    CHECK(trace.state()==BenchTraceState::frozen && trace.size()==1 && b.active());
    (void)trace.request(trace_request(0,0),reply,b,{1134},1,1,"native-b0");CHECK(reply[29]==2);
    (void)trace.request(trace_request(1,58,5,1),reply,b,{1134},1,1,"native-b0");
    (void)trace.request(trace_request(1,59,5,1),reply,b,{1134},1,1,"native-b0");CHECK(reply[2]==3);
    (void)trace.request(trace_request(2,58),reply,b,{1134},1,1,"native-b0");CHECK(trace.size()==0 && b.active());
}
static void ident_trace_contracts(bool vectors) {
    Bench b{4};BenchTrace trace;std::array<std::uint8_t,252> reply{};
    auto request=trace_request(1,71,bench_ident_capacity);request[0]=2;request[3]=1;
    (void)trace.request(request,reply,b,{0},1,0,"native-ident");CHECK(reply[2]==0 && reply[0]==2);
    for (std::uint32_t t=0;t<bench_ident_capacity+2;++t) {
        feedback(b,t);(void)b.step({t});auto sample=bench_trace_sample(b,{t});
        sample.tx_status=1;sample.flags|=trace_tx_attempted;sample.command_raw=-123;
        trace.push(sample,b,{t},0);
    }
    CHECK(trace.size()==bench_ident_capacity && trace.state()==BenchTraceState::frozen);
    auto n=trace.request(trace_request(3,71,0),reply,b,{3000},1,0,"native-ident");
    CHECK(n==224 && bench_wire::get16(reply,10)==8 && bench_wire::get16(reply,12)==bench_ident_capacity);
    CHECK(bench_wire::get16(reply,32+16)==static_cast<std::uint16_t>(-123));
    if(vectors){print_hex("ident_data",std::span<const std::uint8_t>{reply}.first(n));}
    n=trace.request(trace_request(4,71,0),reply,b,{3000},1,0,"native-ident");
    CHECK(bench_wire::get32(reply,32+bench_status_size)==bench_ident_layout);
    if(vectors){print_hex("ident_meta",std::span<const std::uint8_t>{reply}.first(n));}
    n=trace.request(trace_request(3,71,bench_ident_capacity-1),reply,b,{3000},1,0,"native-ident");
    CHECK(n==56 && bench_wire::get16(reply,10)==1);
    request=trace_request(1,71,bench_ident_capacity); // 同run不能换布局.
    (void)trace.request(request,reply,b,{3000},1,0,"native-ident");CHECK(reply[2]!=0);
    request=trace_request(1,72,1);
    (void)trace.request(request,reply,b,{3000},1,0,"native-ident");CHECK(reply[2]==0 && reply[0]==1);
}
static void encoder_velocity_contracts() {
    BenchEncoderVelocity estimate;
    constexpr float count_rad=2*std::numbers::pi_v<float>/8192;
    for(std::uint32_t i=0;i<8;++i){
        estimate.receive(static_cast<std::uint16_t>((8190+i*3)%8192),{100+i},{0xfffff000u+i*1000u});
        if(i<4){CHECK(estimate.value().validity!=ec::Validity::valid);}else{
            CHECK(estimate.value().fresh_at({100+i},{50}));
            CHECK(std::abs(estimate.value().value-3000*count_rad)<1e-4f);
            CHECK(estimate.value().sampled_at.ms<100+i);
        }
    }
    const auto prior=estimate.value();estimate.receive(0,{107},{0xfffff000u+7000u});
    CHECK(estimate.value().value==prior.value); // 同时刻重复不伪造导数.
    CHECK(!estimate.value().fresh_at({160},{50}));
    estimate.receive(100,{200},{200000});CHECK(estimate.value().validity!=ec::Validity::valid);
    for(std::uint32_t i=1;i<=4;++i){estimate.receive(static_cast<std::uint16_t>(100-2*i),{200+i},{200000+i*1000});}
    CHECK(std::abs(estimate.value().value+2000*count_rad)<1e-4f);
    estimate.receive(100,{204+4294967u},{204000});CHECK(estimate.value().validity!=ec::Validity::valid);
    // 定速+量化+实际抖动时间, 以及一计数静止噪声; 不依赖未来样本.
    BenchEncoderVelocity jitter,stationary;float worst=0,noise=0;
    for(std::uint32_t i=0;i<1000;++i){
        const auto us=100000+i*1000+(i%2)*100;
        const auto counts=static_cast<std::uint16_t>(std::lround((us-100000)*1e-6f/count_rad));
        jitter.receive(counts,{us/1000},{us});stationary.receive(static_cast<std::uint16_t>(100+(i%2)),{us/1000},{us});
        if(i>=4){worst=std::max(worst,std::abs(jitter.value().value-1));noise=std::max(noise,std::abs(stationary.value().value));}
    }
    CHECK(worst<.25f);CHECK(noise<.1f);
    Bench b{4};BenchSettings c{};c.mode=BenchMode::velocity;c.target=1;c.velocity_kp=.04f;c.encoder_velocity=true;
    feedback(b,100,2,10);CHECK(!b.start(c,1,{100}));
    for(std::uint32_t i=101;i<=104;++i){feedback(b,i,2,10);}
    CHECK(b.start(c,1,{104}));feedback(b,105,2,10);CHECK(b.step({105})>0);
    CHECK(b.velocity_diagnostics()->measured==0);CHECK(b.rpm()>0); // 原生RPM仍保留, 控制实际采用另一测量.
    b.stop();CHECK(b.set_reference({105}));c.mode=BenchMode::position;c.target=.1f;
    CHECK(b.start(c,2,{105}));feedback(b,106,2,10);CHECK(b.step({106})>0);
    CHECK(b.velocity_diagnostics()->measured==0);
    feedback(b,170,2,10);CHECK(b.step({170})==0);CHECK(!b.active());
    CHECK(!b.start(c,3,{170})); // 新原生RPM不替代尚未重建的编码器窗口.
}
static void derivative_filter_contracts() {
    Bench b{4};BenchSettings c{};c.mode=BenchMode::velocity;c.target=1;
    c.velocity_kp=.02f;c.velocity_kd=.001f;c.derivative_tau_s=.003f;
    feedback(b,100);CHECK(b.start(c,1,{100}));feedback(b,101);(void)b.step({101});
    feedback(b,102,2,1.0473f);(void)b.step({102});
    const auto* d=b.velocity_diagnostics();CHECK(d && d->valid);
    const float speed=b.device().state({102}).velocity_rad_s.value;
    CHECK(std::abs(d->d*c.current_limit+c.velocity_kd*speed/.004f)<1e-5f);
    CHECK(std::abs(d->p*c.current_limit-c.velocity_kp*(1-speed))<1e-5f);
    std::array<std::uint8_t,bench_status_size> status{};
    CHECK(encode_bench_status(b,{102},1,0,status)==126);
    CHECK(bench_wire::get_float(status,114)==c.derivative_tau_s);
    b.stop();feedback(b,103);CHECK(b.start(c,2,{103}));feedback(b,104);(void)b.step({104});
    CHECK(b.velocity_diagnostics()->d==0);
    c.derivative_tau_s=-1;CHECK(!Gm6020Bench::valid(c));
}
static void excitation_contracts() {
    BenchExcitation wave{BenchExcitation::Kind::sine,0.1f,0.05f,10,100000};
    CHECK(wave.valid()); CHECK(std::abs(wave.value(25000)-0.15f)<1e-6f);
    CHECK(std::abs(wave.value(75000)-0.05f)<1e-6f); CHECK(wave.value(100000)==0.1f);
    wave.kind=BenchExcitation::Kind::square;
    CHECK(wave.value(0)==0.15f); CHECK(wave.value(50000)==0.05f);
    auto invalid=wave;invalid.frequency_hz=std::numeric_limits<float>::quiet_NaN();CHECK(!invalid.valid());
    invalid=wave;invalid.duration_us=0x80000000u;CHECK(!invalid.valid());
    Bench b{4}; BenchSettings c{};c.target=0.1f;feedback(b,100);
    const std::uint32_t anchor=0xfffff000u;
    CHECK(b.start(c,77,{100},{anchor}));
    CHECK(!b.excite(78,wave,{100},{anchor}));
    invalid=wave;invalid.amplitude=1;CHECK(!b.excite(77,invalid,{100},{anchor}));
    CHECK(b.excite(77,wave,{100},{anchor}));
    for (std::uint32_t t=1;t<=110;++t) {
        feedback(b,100+t);
        const float output=b.step({100+t},{anchor+t*1000});
        CHECK(std::abs(output-wave.value(t*1000))<1e-6f);
    }
    CHECK(b.active()); CHECK(b.settings_epoch()==1);
    CHECK(b.excite(77,wave,{210},{anchor+110000}));
    CHECK(b.update_target(77,0.02f,0,{210},{anchor+110000}));
    feedback(b,211);CHECK(std::abs(b.step({211},{anchor+111000})-0.02f)<1e-6f);
    std::array<std::uint8_t,19> packet{};
    bench_wire::u16(packet,0,77);packet[2]=1;bench_wire::f32(packet,3,0.1f);
    bench_wire::f32(packet,7,0.05f);bench_wire::f32(packet,11,10);bench_wire::u32(packet,15,100000);
    CHECK(apply_bench_excitation(b,packet,{211},{anchor+111000}));
    CHECK(!apply_bench_excitation(b,std::span<const std::uint8_t>{packet}.first(18),{211},{anchor+111000}));
    b.stop();feedback(b,212);CHECK(b.start(c,79,{212},{anchor+112000}));
    feedback(b,213);CHECK(std::abs(b.step({213},{anchor+113000})-c.target)<1e-6f);
    CHECK(b.excite(79,wave,{213},{anchor+113000}));
    CHECK(!b.excite(79,wave,{713},{anchor+613000}));CHECK(!b.active());
}
int main(int argc,char** argv){
    if(argc==2 && std::string_view(argv[1])=="--encoder-replay") {
        BenchEncoderVelocity estimate;std::uint32_t us,ms;unsigned encoder;
        std::cout<<std::setprecision(9);
        while(std::cin>>us){
            if(!(std::cin>>ms>>encoder) || encoder>=8192){return 4;}
            estimate.receive(static_cast<std::uint16_t>(encoder),{ms},{us});
            const auto value=estimate.value();
            std::cout<<value.value<<' '<<static_cast<unsigned>(value.validity)<<' '<<value.sampled_at.ms<<'\n';
        }
        return 0;
    }
    if(argc==3 && std::string_view(argv[1])=="--settings") {
        const std::string_view text=argv[2];if(text.size()!=bench_settings_size*2){return 2;}
        std::array<std::uint8_t,bench_settings_size> bytes{};
        for(std::size_t i=0;i<bytes.size();++i){
            unsigned v{};const auto r=std::from_chars(text.data()+2*i,text.data()+2*i+2,v,16);
            if(r.ec!=std::errc{} || r.ptr!=text.data()+2*i+2){return 2;}bytes[i]=static_cast<std::uint8_t>(v);
        }
        const auto c=decode_bench_settings(bytes);if(!c){return 3;}
        std::cout<<unsigned(c->mode)<<' '<<c->target<<' '<<c->duration_s<<' '<<c->current_limit<<' '
                 <<c->speed_limit<<' '<<c->velocity_kp<<' '<<c->velocity_ki<<' '<<c->position_kp<<' '
                 <<c->velocity_kd<<' '<<c->feedback_alpha<<' '<<c->derivative_tau_s<<' '<<c->encoder_velocity<<'\n';return 0;
    }
    ident_trace_contracts(argc==2 && std::string_view(argv[1])=="--trace-vectors");
    encoder_velocity_contracts();derivative_filter_contracts();excitation_contracts();mapping_and_diagnostics();trace_contracts(argc==2 && std::string_view(argv[1])=="--trace-vectors");
    if(argc==2 && std::string_view(argv[1])=="--trace-vectors"){return 0;}
    contracts();closed_loop(BenchMode::velocity,1);closed_loop(BenchMode::velocity,-1);
    closed_loop(BenchMode::position,0.5f);closed_loop(BenchMode::position,-0.5f);
    std::array<std::uint8_t,bench_status_size> data{};status_vector(data);
    if(argc==2 && std::string_view(argv[1])=="--vector") {
        for(auto v:data){std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(v);}std::cout<<'\n';
    } else {std::cout<<"GM6020 bench control contracts passed (not physical tuning evidence)\n";}
}
