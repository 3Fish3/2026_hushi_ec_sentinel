#include "ec/transport/uart_frame.hpp"
#include "board_monitor_protocol.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string_view>

// 失败时以非零码退出且不触发 CRT 交互弹窗, ctest 要求非交互
#define CHECK(x) do { if (!(x)) { std::cerr << "line " << __LINE__ << ": " #x "\n"; std::_Exit(3); } } while (false)
using namespace ec::transport;

static void contracts() {
    const std::array<std::uint8_t,9> digits{'1','2','3','4','5','6','7','8','9'};
    CHECK(uart_crc16(digits) == 0x29b1);
    std::array<std::uint8_t, uart_payload_max> payload{};
    for (std::size_t i=0;i<payload.size();++i) { payload[i]=static_cast<std::uint8_t>(i); }
    std::array<std::uint8_t, uart_wire_max> wire{};
    const auto length = encode_uart_frame(0x81, 0x7e7d, payload, wire);
    CHECK(length > 0);
    UartFrameDecoder decoder;
    CHECK(!decoder.push(0x33));
    std::optional<UartFrame> frame;
    for (std::size_t i=0;i<length;++i) { if (auto f=decoder.push(wire[i])) { frame=f; } }
    CHECK(frame && frame->sequence==0x7e7d && frame->payload==payload && frame->size==256);
    CHECK(decoder.rejected()==0);
    // 粘包、重复边界、分字节输入均不依赖物理 IDLE.
    std::size_t accepted=0;
    for(int round=0;round<3;++round) {
        for(std::size_t i=0;i<length;++i) { if(decoder.push(wire[i])){++accepted;} }
    }
    CHECK(accepted==3);
    auto broken=wire; broken[8]^=1;
    for(std::size_t i=0;i<length;++i){CHECK(!decoder.push(broken[i]));}
    CHECK(decoder.rejected()==1);
    CHECK(!decoder.push(0x7d)); CHECK(!decoder.push(0x7e));
    CHECK(decoder.rejected()==2);
    for(std::size_t i=0;i<600;++i){CHECK(!decoder.push(0x11));}
    CHECK(!decoder.push(0x7e)); CHECK(decoder.rejected()==3);
    // 中途重启/丢字节后必须先重新看到边界, 不能拼接旧半帧.
    for(std::size_t i=0;i<8;++i){CHECK(!decoder.push(wire[i]));}
    decoder.reset();
    accepted=0;
    for(std::size_t i=0;i<length;++i){if(decoder.push(wire[i])){++accepted;}}
    CHECK(accepted==1 && decoder.rejected()==3);
    CHECK(encode_uart_frame(1,0,{},std::span<std::uint8_t>{wire.data(),1})==0);
    std::array<std::uint8_t,257> oversized{};
    CHECK(encode_uart_frame(1,0,oversized,wire)==0);
    const auto empty=encode_uart_frame(1,0x1234,{},wire);
    CHECK(empty>0);
    frame.reset();
    for(std::size_t i=0;i<empty;++i){if(auto f=decoder.push(wire[i])){frame=f;}}
    CHECK(frame && frame->size==0 && frame->sequence==0x1234);
}

int main(int argc, char** argv) {
    contracts();
    EcS1Diagnostics s{}; EcBoardIo io{};
    s.stage=6; s.build_id="fixture"; s.cycles=123; s.max_elapsed_ms=10;
    io.remote.frames=99; io.remote.rejected=1; io.remote.last_ms=0xfffffff0;
    io.remote.ch0=-660; io.remote.ch1=660; io.remote.ch2=-1; io.remote.ch3=42;
    io.remote.ch4=1234;
    io.remote.switch_left=2; io.remote.switch_right=3; io.remote.online=1;
    io.imu.samples=123; io.imu.last_ms=16; io.imu.ready=1;
    io.imu.ax=1.25f; io.imu.az=9.8f; io.imu.temp_c=27.25f;
    io.can.rx_frames=5000; io.can.last_id=0x208; io.can.last_speed_rad_s=104.71976f;
    io.usart3.noise=7;
    std::array<std::uint32_t,14> link{}; link[0]=111; link[13]=9;
    std::array<std::uint8_t,ec::sentinel::monitor_payload_size> payload{};
    CHECK(ec::sentinel::encode_monitor_snapshot(s,io,16,link,{payload.data(),1})==0);
    CHECK(ec::sentinel::encode_monitor_snapshot(s,io,16,link,payload)==ec::sentinel::monitor_payload_size);
    if(argc==2 && std::string_view(argv[1])=="--vector") {
        std::array<std::uint8_t,uart_wire_max> wire{};
        const auto n=encode_uart_frame(0x81,0x7e7d,payload,wire);
        for(std::size_t i=0;i<n;++i){std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(wire[i]);}
        std::cout<<'\n';
    } else { std::cout<<"UART framing and observation contracts passed\n"; }
}
