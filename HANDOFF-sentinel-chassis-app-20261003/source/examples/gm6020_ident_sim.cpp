// 主机侧辨识模型闭环: 使用实际台架/轴/PID/组编码, 不向硬件发送数据.
#include "gm6020_bench.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <numbers>
#include <sstream>
#include <string>

namespace {
struct Bus final : ec::transport::CanTx {
    float command{};
    ec::transport::SendResult try_send(const ec::transport::CanFrame& f) noexcept override {
        const auto bits=static_cast<std::uint16_t>((unsigned(f.data[6])<<8)|f.data[7]);
        const int raw=bits<32768?bits:static_cast<int>(bits)-65536;
        command=static_cast<float>(raw)*3/16384;
        return ec::transport::SendResult::accepted;
    }
};
void feedback(ec::sentinel::Gm6020Bench& bench,float angle,float velocity,std::uint32_t ms) {
    constexpr float tau=2*std::numbers::pi_v<float>;
    auto turns=std::fmod(angle,tau);if(turns<0){turns+=tau;}
    const auto encoder=static_cast<unsigned>(turns*8192/tau)%8192;
    const auto rpm=static_cast<std::int16_t>(std::clamp(std::round(velocity*60/tau),-32768.0f,32767.0f));
    ec::transport::CanFrame f{};f.id=0x208;f.size=8;
    f.data={static_cast<std::uint8_t>(encoder>>8),static_cast<std::uint8_t>(encoder),
        static_cast<std::uint8_t>(rpm>>8),static_cast<std::uint8_t>(rpm),0,0,30,0};
    bench.receive(1,f,{ms},{ms*1000});
}
}
int main() {
    // 每行独立运行: a b c delay_s Kp Ki position_Kp mode target seconds alpha.
    // 标准输入接口便于Python批量调用同一进程, 避免另写一套控制器.
    std::string line;
    while(std::getline(std::cin,line)) {
        std::istringstream input(line);
        double a,b,c,delay,kp,ki,pk,target,seconds,alpha;int mode;
        double kd=0,d_tau=0;int encoder_velocity=0;
        if(!(input>>a>>b>>c>>delay>>kp>>ki>>pk>>mode>>target>>seconds>>alpha) ||
            !std::isfinite(a+b+c+delay+kp+ki+pk+target+seconds+alpha) ||
            a<=0 || b<0 || c<0 || delay<0 || delay>.1 || seconds<=0 || seconds>60 ||
            mode<2 || mode>3) {std::cerr<<"invalid simulation input\n";return 2;}
        if (input>>kd) {
            if (!(input>>d_tau) || !std::isfinite(kd+d_tau) || kd<0 || d_tau<0) {return 2;}
            if (input>>encoder_velocity && encoder_velocity!=0 && encoder_velocity!=1) {return 2;}
        }
        ec::sentinel::Gm6020Bench bench{4};Bus bus;
        ec::drivers::DjiTxGroup group{bus,1,1,{20},bench.group_devices()};
        for(std::uint32_t ms=0;ms<5;++ms){feedback(bench,0,0,ms);}
        if(!bench.set_reference({4})){return 3;}
        ec::sentinel::BenchSettings cfg{};
        cfg.mode=static_cast<ec::sentinel::BenchMode>(mode);cfg.target=static_cast<float>(target);
        cfg.velocity_kp=static_cast<float>(kp);cfg.velocity_ki=static_cast<float>(ki);
        cfg.position_kp=static_cast<float>(pk);cfg.feedback_alpha=static_cast<float>(alpha);
        cfg.velocity_kd=static_cast<float>(kd);cfg.derivative_tau_s=static_cast<float>(d_tau);
        cfg.encoder_velocity=encoder_velocity!=0;
        if(!bench.start(cfg,1,{4},{4000})){ std::cerr<<"invalid controller config\n";return 4;}
        double velocity=0,position=0,sum=0,squared=0,peak=0,variation=0,previous=0;
        double minimum=1e30,maximum=-1e30;unsigned n=0,saturated=0,inactive=0;
        const auto steps=static_cast<unsigned>(seconds*1000);
        std::deque<double> commands(static_cast<std::size_t>(std::ceil(delay*1000))+1,0);
        for(unsigned ms=1;ms<=steps;++ms) {
            // 统一1ms步长; 命令延迟以相邻ZOH抽头插值近似小数部分, 不声称精确物理分解.
            const double lag=delay*1000;const auto whole=static_cast<std::size_t>(lag);
            const double fraction=lag-whole;
            const double u=commands[commands.size()-1-whole]*(1-fraction)+commands.front()*fraction;
            const double force=a*u;
            if(!(std::abs(velocity)<1e-9 && std::abs(force)<=c)) {
                const double sign=velocity!=0?std::copysign(1.0,velocity):std::copysign(1.0,force);
                double next=velocity*std::exp(-b*.001)+(force-c*sign)*(b>1e-9?-std::expm1(-b*.001)/b:.001);
                if(next*velocity<0 && std::abs(force)<=c){next=0;}
                velocity=next;
            }
            position+=velocity*.001;
            const auto now=ms+4;
            feedback(bench,static_cast<float>(position),static_cast<float>(velocity),now);
            if(ms%100==0){(void)bench.heartbeat(1,{now});}
            const float current=bench.step({now},{now*1000});
            std::array<std::optional<ec::drivers::DjiOutput>,4> outputs{};
            outputs[3]=ec::drivers::DjiOutput{current/3,{now}};
            if(group.send({now},outputs).status!=ec::drivers::DjiGroupStatus::accepted){return 5;}
            commands.pop_front();commands.push_back(bus.command);
            inactive+=!bench.active();
            const double measurement=mode==2?velocity:position;
            peak=std::max(peak,std::abs(measurement));
            variation+=std::abs(bus.command-previous);previous=bus.command;
            if(ms>steps/2) {
                const double error=measurement-target;sum+=error;squared+=error*error;++n;
                minimum=std::min(minimum,measurement);maximum=std::max(maximum,measurement);
                saturated+=std::abs(bus.command)>.999*cfg.current_limit;
            }
        }
        std::cout<<sum/n<<' '<<std::sqrt(squared/n)<<' '<<maximum-minimum<<' '<<peak<<' '
                 <<static_cast<double>(saturated)/n<<' '<<variation/seconds<<' '<<inactive<<'\n';
    }
}
