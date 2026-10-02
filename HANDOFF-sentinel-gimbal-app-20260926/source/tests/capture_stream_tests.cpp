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
}
