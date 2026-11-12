// 仅主机侧小信号模拟: 模型不是实车, 控制器复用正式PositionLoop串级内环.
#include "ec/control/position_loop.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
int main() {
    double a,b,delay,kp,ki,target,seconds,outer;
    while(std::cin>>a>>b>>delay>>kp>>ki>>target>>seconds>>outer) {
        if(!std::isfinite(a+b+delay+kp+ki+target+seconds+outer) || a<=0 || b<0 || delay<0 || delay>.2 ||
            kp<0 || ki<0 || target==0 || seconds<=1 || seconds>60 || outer<0) { return 2; }
        ec::control::PositionLoopConfig cfg{};
        cfg.position_pid={static_cast<float>(outer),0,0,0,0,0,0,false};
        cfg.velocity={{static_cast<float>(kp/3),static_cast<float>(ki/3),0,1,-1,1},{100},0,.005f,0};
        ec::control::PositionLoop controller{cfg};const auto grant=controller.arm(ec::CommandSource::maintenance,true,true);
        if(!grant) { return 3; }
        const auto whole=static_cast<unsigned>(delay*1000);const double fraction=delay*1000-whole;
        std::deque<double> history(whole+2,0);
        double velocity=0,angle=0,peak=0,peak_i=0,last_outside=0,sum=0;unsigned count=0,limited=0;
        const auto steps=static_cast<unsigned>(seconds*1000);
        for(unsigned n=0;n<steps;++n) {
            const double input=history[history.size()-1-whole]*(1-fraction)+history[history.size()-2-whole]*fraction;
            velocity=velocity*std::exp(-b*.001)+a*input*(b>1e-9 ? -std::expm1(-b*.001)/b : .001);
            angle+=velocity*.001;
            const ec::Tick now{n};const ec::Command<float> command{static_cast<float>(target),now,{100},*grant};
            std::optional<float> u;
            if(outer==0) { u=controller.step_inner(now,.001f,command,{static_cast<float>(velocity),now,ec::Validity::valid}).output; }
            else { u=controller.step_split(now,.001f,n%2==0?std::optional{.002f}:std::nullopt,command,
                ec::Sample<float>{static_cast<float>(angle),now,ec::Validity::valid},
                ec::Sample<float>{static_cast<float>(velocity),now,ec::Validity::valid},1).output; }
            if(!u) { return 4; }
            const double current=*u*3;
            history.pop_front();history.push_back(current);
            const double measured=outer==0?velocity:angle,error=measured-target;
            peak=std::max(peak,measured/target);peak_i=std::max(peak_i,std::fabs(current));
            if(std::fabs(error)>.05*std::fabs(target)) { last_outside=n*.001; }
            if(n>steps*3/4) { sum+=error;++count; }
            limited+=std::fabs(current)>=2.999;
        }
        std::cout<<sum/count<<' '<<std::max(0.0,peak-1)<<' '<<last_outside<<' '<<peak_i<<' '<<double(limited)/steps<<'\n';
    }
}
