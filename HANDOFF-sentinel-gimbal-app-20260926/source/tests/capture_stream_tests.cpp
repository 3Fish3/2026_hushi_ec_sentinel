#include "raw_record.hpp"
#include "ec/transport/uart_frame.hpp"
#include <array>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string_view>
using namespace ec::sentinel::recording;
#define CHECK(x) do { if (!(x)) { std::cerr<<__LINE__<<": " #x "\n";std::exit(1); } } while(false)
std::array<std::uint8_t,12> command(unsigned op,unsigned run=9,unsigned duration=1000000) {
    std::array<std::uint8_t,12> a{1,static_cast<std::uint8_t>(op),2,0};put32(a,4,run);put32(a,8,duration);return a;
}
Record sample(Kind kind,unsigned stream,unsigned at,unsigned seq) {
    Record r{kind,static_cast<std::uint8_t>(stream),1,0,seq,at,at,0,0xffffffffu,0};
    // 密集转义字节覆盖满云台流的保守传输负载, 不作为物理测量样本.
    for(unsigned i=0;i<(kind==Kind::gyro ? 4u : 3u);++i) { r.data[i]=0x3e7d7e7du; }
    if(kind==Kind::can) { r.data[0]=1u|(8u<<8)|(0x208u<<16); }
    return r;
}
// 连续 Pitch 控制流 (Profile 3 / 域 6) 的请求与控制记录构造器.
std::array<std::uint8_t,12> control_command(unsigned op,unsigned run,unsigned duration,unsigned domain=6) {
    std::array<std::uint8_t,12> a{1,static_cast<std::uint8_t>(op),3,static_cast<std::uint8_t>(domain)};
    put32(a,4,run);put32(a,8,duration);return a;
}
Record pitch_can(unsigned at,unsigned seq) {
    Record r{Kind::can,9,4,0,seq,at,at,0,0xffffffffu,0};
    r.data[0]=2u|(8u<<8)|(0x011u<<16);r.data[1]=0x11223344u;r.data[2]=0x55667788u;
    return r;
}
Record pitch_control(unsigned at,unsigned seq) {
    Record r{Kind::control,6,4,0,seq,at,at,0,0xffffffffu,0};
    for(unsigned i=0;i<8;++i) { r.data[i]=0xa0000000u+i; }
    return r;
}
void hex(std::span<const std::uint8_t> data) {
    constexpr char digits[]="0123456789abcdef";
    for (auto b:data) { std::cout<<digits[b>>4]<<digits[b&15]; }std::cout<<'\n';
}
int main(int argc,char** argv) {
    std::array<std::uint8_t,256> out{};std::size_t count{};
    if (argc==2 && std::string_view(argv[1])=="--golden") {
        Recorder r;CHECK(r.stream_request(command(1),out,{0})==32);
        r.push(sample(Kind::gyro,0,0,1),{0});r.push(sample(Kind::accel,0,0,1),{0});
        r.push(sample(Kind::can,4,0,1),{0});
        CHECK(r.stream_request(command(2),out,{1000})==32);
        const auto n=r.stream_packet(out,{1000},count);CHECK(count==3);hex({out.data(),n});return 0;
    }
    if (argc==2 && std::string_view(argv[1])=="--pitch-golden") {
        // 真实产物字节: 供主机解码器做互操作验证; 不改动历史 --golden 输出.
        Recorder r;
        // 1) 开始应答 (op1/state1)
        CHECK(r.stream_request(control_command(1,4242,1000000,6),out,{5000})==32);
        hex({out.data(),32});
        // 2) 混合记录: CAN 44B + control 64B + CAN 44B = 184B
        r.push(pitch_can(10000,1),{10000});
        r.push(pitch_control(10000,1),{10000});
        r.push(pitch_can(15000,2),{15000});
        const auto mixed=r.stream_packet(out,{15000},count);
        CHECK(count==3 && mixed==32+44+record_bytes+44);
        hex({out.data(),mixed});
        r.stream_consume(count);
        // 3) 状态查询 (op0): generated 3, buffered 0
        CHECK(r.stream_request(control_command(0,4242,0,6),out,{15000})==32);
        CHECK(get32(out,8)==3 && get32(out,16)==0 && get32(out,12)==0);
        hex({out.data(),32});
        // 4) 停止应答 (op2): 积压已清零 -> state 3
        CHECK(r.stream_request(control_command(2,4242,0,6),out,{20000})==32);
        CHECK(out[2]==0 && out[3]==3 && get32(out,8)==3 && get32(out,16)==0);
        hex({out.data(),32});
        // 5) 最终仅头部 op3 帧: 停止且积压为 0, 不得再携带记录
        CHECK(r.stream_packet(out,{20000},count)==32 && count==0);
        CHECK(out[1]==3 && out[2]==0 && out[3]==3 && get32(out,8)==3 && get32(out,16)==0);
        hex({out.data(),32});
        return 0;
    }
    Recorder r;
    CHECK(r.stream_request(command(1),out,{0})==32 && out[2]==0);
    CHECK(r.stream_request(command(1,9,2),out,{0})==32 && out[2]==2);
    CHECK(r.stream_request(command(1,10),out,{0})==32 && out[2]==3);
    for(unsigned i=0;i<520;++i) { r.push(sample(Kind::gyro,0,i*4000,i+1),{i*1000}); }
    CHECK(r.size()==512);
    CHECK(r.stream_request(command(0),out,{520000})==32 && get32(out,8)==520 && get32(out,12)==8);
    auto n=r.stream_packet(out,{520000},count);CHECK(n==176 && count==3 && get32(out,32+4)==0);
    auto copy=out;n=r.stream_packet(out,{520000},count);CHECK(out==copy); // busy不丢未提交帧.
    r.stream_consume(3);CHECK(r.size()==509);
    CHECK(r.stream_request(command(2,10),out,{520000})==32 && out[2]==2);
    CHECK(r.stream_request(command(2),out,{520000})==32 && out[3]==2);
    while(r.size()) { n=r.stream_packet(out,{520000},count);CHECK(n>32);r.stream_consume(count); }
    CHECK(out[3]==3);CHECK(r.stream_packet(out,{520000},count)==0);
    CHECK(r.stream_request(command(1,11,5000),out,{0xfffffff0u})==32 && out[2]==0);
    n=r.stream_packet(out,{6000},count);CHECK(n==32 && out[3]==3);r.stream_consume(count);
    CHECK(r.stream_request(command(1,11,5000),out,{7000})==32 && out[3]==3); // 同run不重新开始.

    // 按真实UART编码长度模拟1 Mbps出口与1 ms服务, 包括900/1100 us抖动.
    Recorder load;CHECK(load.stream_request(command(1,55,10000000),out,{0})==32);
    std::array<std::uint8_t,ec::transport::uart_wire_max> wire{};
    unsigned now=0,busy_until=0,high=0,sent=0;
    for(unsigned i=0;i<5000;++i) {
        now+=i%2 ? 1100:900;
        if(i%4==0) {
            load.push(sample(Kind::gyro,0,now,i+1),{now});
            for(unsigned axis:{4u,5u,9u}) { load.push(sample(Kind::can,axis,now,i+1),{now}); }
        }
        if(i%10==0) { load.push(sample(Kind::accel,0,now,i+1),{now}); }
        high=std::max(high,static_cast<unsigned>(load.size()));
        const auto size=load.stream_packet(out,{now},count);
        if(size && now>=busy_until) {
            const auto bytes=ec::transport::encode_uart_frame(0xb3,static_cast<std::uint16_t>(i),{out.data(),size},wire);
            CHECK(bytes>0);busy_until=now+static_cast<unsigned>(bytes)*10;
            sent+=static_cast<unsigned>(count);load.stream_consume(count);
        }
    }
    CHECK(load.stream_request(command(0),out,{now})==32 && get32(out,12)==0);
    CHECK(get32(out,8)==5500 && high<100);
    std::cout<<"PASS simulated 1Mbps: generated="<<get32(out,8)<<" sent="<<sent<<" queued="<<load.size()<<" high="<<high<<"\n";

    // ---- 连续 Pitch 控制流: Profile 3 仅开放域 6, IMU 剔除, 双流各 200 Hz, 完整 64B ----
    {
        Recorder pitch;
        // 域 6 之外的组合均拒绝: 不得静默改写请求方域号.
        CHECK(pitch.stream_request(control_command(1,21,1000000,0),out,{0})==32 && out[2]==1);
        CHECK(pitch.stream_request(control_command(1,21,1000000,7),out,{0})==32 && out[2]==1);
        CHECK(pitch.stream_request(control_command(1,0,1000000,6),out,{0})==32 && out[2]==1);
        CHECK(pitch.stream_request(control_command(1,21,0,6),out,{0})==32 && out[2]==1);
        CHECK(pitch.stream_request(control_command(1,21,1000000,6),out,{1000})==32);
        CHECK(out[2]==0 && out[3]==1 && get32(out,4)==21 && get32(out,28)==layout_id);
        // 同 run 重复启动: 参数不同->2, 参数相同->无副作用重复应答.
        CHECK(pitch.stream_request(control_command(1,21,999000,6),out,{1000})==32 && out[2]==2);
        CHECK(pitch.stream_request(control_command(1,21,1000000,6),out,{1000})==32 && out[2]==0);
        // 连续 control 流入口剔除全部 IMU 记录.
        pitch.push(sample(Kind::gyro,0,1000,1),{1000});
        pitch.push(sample(Kind::accel,0,1000,1),{1000});
        pitch.push(sample(Kind::temperature,0,1000,1),{1000});
        CHECK(pitch.size()==0);
        // 非域 6 的 CAN 流与错误控制域均不入队.
        pitch.push(sample(Kind::can,4,1000,1),{1000});
        CHECK(pitch.size()==0);
        pitch.push(sample(Kind::control,5,1000,1),{1000});
        CHECK(pitch.size()==0);
        // 200 Hz 边界: 4999us 拒绝, 5000us 接受; 两条流各自独立计时.
        pitch.push(pitch_can(10000,1),{10000});CHECK(pitch.size()==1);
        pitch.push(pitch_can(14999,2),{14999});CHECK(pitch.size()==1);
        pitch.push(pitch_can(15000,3),{15000});CHECK(pitch.size()==2);
        pitch.push(pitch_control(10000,1),{10000});CHECK(pitch.size()==3);
        pitch.push(pitch_control(14999,2),{14999});CHECK(pitch.size()==3);
        pitch.push(pitch_control(15000,3),{15000});CHECK(pitch.size()==4);
        // ready/healthy: 精确 run, 无丢条, 未满, 未过截止.
        CHECK(pitch.pitch_stream_healthy(21,{1000}));
        CHECK(!pitch.pitch_stream_healthy(22,{1000}));
        CHECK(!pitch.pitch_stream_healthy(0,{1000}));
        CHECK(pitch.pitch_stream_ready(21,1000000,{1000}));      // 恰等于剩余时长
        CHECK(!pitch.pitch_stream_ready(21,1000001,{1000}));
        CHECK(pitch.pitch_stream_ready(21,500001,{500999}));     // elapsed 499999
        CHECK(!pitch.pitch_stream_ready(21,500002,{500999}));
        CHECK(pitch.pitch_stream_healthy(21,{1000999}));         // elapsed 999999 < duration
        CHECK(!pitch.pitch_stream_healthy(21,{1001000}));        // 到达截止
        CHECK(pitch.state()==State::recording);                  // 查询不推进状态机
        // 逐条按自身类型定长: CAN 44B / control 完整 64B; 混合 3 条仍 <= 224B.
        std::array<std::uint8_t,256> packet{};
        std::size_t taken{};
        const auto bytes=pitch.stream_packet(packet,{16000},taken);
        CHECK(bytes==32+44+44+record_bytes && bytes<=stream_packet_bytes && taken==3);
        CHECK(packet[32+1]==4 && packet[32+2]==9);                // 记录0: Kind::can, stream 9, 44B
        CHECK(packet[32+44+1]==4);                               // 记录1: Kind::can, 44B
        CHECK(packet[32+88+1]==5 && packet[32+88+2]==6);         // 记录2: Kind::control, 全 64B
        CHECK(get32(packet,32+88+60)==0xa0000007u);              // control data[7] 未被截断
        CHECK(get32(packet,16)==4-3 && get32(packet,8)==4);      // buffered / generated
    }

    // ---- 溢出: 固定 512 队列丢新条 + 计数 + 序号间隙, 绝不覆盖旧条 ----
    {
        Recorder over;
        CHECK(over.stream_request(control_command(1,31,60000000,6),out,{0})==32 && out[2]==0);
        static_assert(capacity==512);
        std::uint32_t at=0, seq=0;
        for(std::size_t i=0;i<capacity;++i) { at+=pitch_stream_period_us;++seq;over.push(pitch_can(at,seq),{at}); }
        CHECK(over.size()==capacity);
        CHECK(!over.pitch_stream_healthy(31,{at}));   // 队列已满 -> 不健康
        CHECK(!over.pitch_stream_ready(31,1,{at}));
        at+=pitch_stream_period_us;++seq;over.push(pitch_can(at,seq),{at});   // 该条被丢弃
        CHECK(over.size()==capacity);
        CHECK(over.stream_request(command(0,31),out,{at})==32 && get32(out,8)==513 && get32(out,12)==1);
        CHECK(!over.pitch_stream_healthy(31,{at}));   // 已丢条 -> 不健康
        over.stream_consume(3);                       // 腾出空间后新条重新入队
        CHECK(over.size()==509);
        at+=pitch_stream_period_us;++seq;over.push(pitch_can(at,seq),{at});
        CHECK(over.size()==510);
        std::array<std::uint8_t,256> packet{};
        std::size_t taken{}, total{};
        std::array<std::uint32_t,capacity> order{};
        while(over.size()) {
            const auto n=over.stream_packet(packet,{at},taken);
            CHECK(n==32+taken*44);   // 纯 CAN 记录保持 44B 紧凑承载
            for(std::size_t i=0;i<taken;++i) { order[total++]=get32(packet,32+i*44+4); }
            over.stream_consume(taken);
        }
        CHECK(total==510);
        for(std::size_t i=0;i<509;++i) { CHECK(order[i]==i+3); }
        CHECK(order[509]==513);   // 被丢弃的序号 512 缺口保留, 未被 513 覆盖
    }

    // ---- 多秒连续 Pitch 流: 1 kHz 上报 -> 200 Hz x 2 流, 1 Mbps 出口无丢条 ----
    {
        Recorder multi;
        constexpr std::uint32_t pitch_run=77, window=3000000;
        CHECK(multi.stream_request(control_command(1,pitch_run,window,6),out,{0})==32 && out[2]==0);
        CHECK(multi.pitch_stream_ready(pitch_run,3000000,{0}));
        CHECK(!multi.pitch_stream_ready(pitch_run,3000001,{0}));
        std::array<std::uint8_t,256> packet{};
        std::array<std::uint8_t,ec::transport::uart_wire_max> wire{};
        std::size_t taken{};
        std::uint32_t now=0, busy_until=0;
        std::size_t widest=0;
        for(std::uint32_t step=0;step*1000<window;++step) {
            now=step*1000;
            // 实机 1 kHz 反馈: Pitch CAN 与控制 trace 各 1 kHz, 外加 IMU.
            multi.push(pitch_can(now,step+1),{now});
            multi.push(pitch_control(now,step+1),{now});
            multi.push(sample(Kind::gyro,0,now,step+1),{now});
            CHECK(multi.pitch_stream_healthy(pitch_run,{now}));
            if(now<busy_until) { continue; }
            const auto n=multi.stream_packet(packet,{now},taken);
            if(!n) { continue; }
            CHECK(n<=stream_packet_bytes);
            const auto wire_bytes=ec::transport::encode_uart_frame(0xb3,0,{packet.data(),n},wire);
            CHECK(wire_bytes>0);
            busy_until=now+static_cast<std::uint32_t>(wire_bytes)*10;
            if(n>widest) { widest=n; }
            multi.stream_consume(taken);
        }
        CHECK(multi.stream_request(command(0,pitch_run),out,{now})==32);
        CHECK(get32(out,8)==1200);   // 3000ms / 5ms = 600 条 x 2 流
        CHECK(get32(out,12)==0);     // 无丢条
        CHECK(get32(out,8)==600+600);
        CHECK(widest>32 && widest<=stream_packet_bytes);
        CHECK(!multi.pitch_stream_healthy(pitch_run,{window}));   // 到期
    }

    // ---- 背压边界: 积压 7 条时单包严格封顶 3 条 = 224B, 控制字完整不截断 ----
    {
        Recorder back;
        CHECK(back.stream_request(control_command(1,91,10000000,6),out,{0})==32 && out[2]==0);
        std::uint32_t at=0;
        for(unsigned i=0;i<7;++i) { at+=pitch_stream_period_us;back.push(pitch_control(at,i+1),{at}); }
        CHECK(back.size()==7);
        std::array<std::uint8_t,256> packet{};
        std::size_t taken{};
        CHECK(back.stream_packet(packet,{at},taken)==stream_packet_bytes && taken==3);
        CHECK(get32(packet,32+2*record_bytes+60)==0xa0000007u);   // 第三条 data[7] 完整保留
        CHECK(get32(packet,16)==7-3);
        back.stream_consume(taken);
        CHECK(back.stream_packet(packet,{at},taken)==stream_packet_bytes && taken==3);
        back.stream_consume(taken);
        CHECK(back.stream_packet(packet,{at},taken)==32+record_bytes && taken==1 && get32(packet,16)==0);
    }

    // ---- 32 位微秒回绕: 半回绕窗口内有效, 歧义区失效, 查询不变更状态 ----
    {
        Recorder wrap;
        constexpr std::uint32_t begin=0xfffff000u, span=1000000;
        CHECK(wrap.stream_request(control_command(1,88,span,6),out,{begin})==32 && out[2]==0);
        wrap.push(pitch_can(begin+1000,1),{begin+1000});
        CHECK(wrap.size()==1);
        const std::uint32_t after=begin+500000;   // 跨过 2^32 回绕
        CHECK(after<begin);
        CHECK(wrap.pitch_stream_healthy(88,{after}));
        CHECK(wrap.pitch_stream_ready(88,500000,{after}));
        CHECK(!wrap.pitch_stream_ready(88,500001,{after}));
        CHECK(!wrap.pitch_stream_healthy(88,{begin+0x80000000u}));   // 半个回绕 -> 歧义
        CHECK(!wrap.pitch_stream_ready(88,1,{begin+0x80000000u}));
        CHECK(wrap.size()==1 && wrap.state()==State::recording);      // 查询零副作用
        // 停止后 old run 立即失效; 新 run 接管.
        CHECK(wrap.stream_request(command(2,88),out,{after})==32 && out[2]==0);
        CHECK(!wrap.pitch_stream_healthy(88,{after}));
        std::array<std::uint8_t,256> packet{};
        std::size_t taken{};
        while(wrap.size()) { (void)wrap.stream_packet(packet,{after},taken);wrap.stream_consume(taken); }
        CHECK(wrap.stream_request(control_command(1,89,span,6),out,{after})==32 && out[2]==0);
        CHECK(!wrap.pitch_stream_healthy(88,{after}));
        CHECK(wrap.pitch_stream_healthy(89,{after}));
    }

    // ---- 短窗 control 速率与 64B 完整性不受连续流 200 Hz 节流影响 ----
    {
        Recorder window_rec;
        std::array<std::uint8_t,12> start{1,1,3,6};put32(start,4,44);put32(start,8,1000000);
        CHECK(window_rec.request(start,out,{0})==32 && out[2]==0);
        window_rec.push(pitch_control(0,1),{0});CHECK(window_rec.size()==1);
        window_rec.push(pitch_control(1000,2),{1000});CHECK(window_rec.size()==2);   // 历史 1 kHz 保持
        window_rec.push(pitch_can(0,1),{0});CHECK(window_rec.size()==3);
        window_rec.push(pitch_can(1000,2),{1000});CHECK(window_rec.size()==4);
        std::array<std::uint8_t,12> stop{1,2,0,0};put32(stop,4,44);
        CHECK(window_rec.request(stop,out,{1000})==32 && out[2]==0 && out[3]==2);
        std::array<std::uint8_t,12> page{1,3,0,0};put32(page,4,44);put32(page,8,0);
        const auto n=window_rec.request(page,out,{1000});
        CHECK(n==224 && out[2]==0 && out[9]==3);
    }

    std::cout<<"PASS pitch stream: profile3/domain6 200Hz, 64B records, 224B packets, ready/healthy, wrap, overflow\n";
}
