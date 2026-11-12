#include "ec/control/position_loop.hpp"
#include "board_app_integration_site.hpp"
#include "sentinel_kinematics.hpp"
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>

struct Row { double t{}; float joint{}; ec::estimation::Quaternion q{}; ec::sentinel::app::Vector3 rate{}; };
int main() {
    float kp{},ki{},kd{},inner_kp{},inner_ki{},target{};
    if (!(std::cin>>kp>>ki>>kd>>inner_kp>>inner_ki>>target)) return 2;
    std::vector<Row> rows;
    Row r;
    while (std::cin>>r.t>>r.joint>>r.q.w>>r.q.x>>r.q.y>>r.q.z>>r.rate[0]>>r.rate[1]>>r.rate[2]) rows.push_back(r);
    if (rows.size()<3) return 3;
    const auto model=*ec::sentinel::board::integration_serial_geometry;
    ec::control::PidConfig outer{kp,ki,kd,15,0,0,0,false};
    ec::control::PidConfig inner{inner_kp,inner_ki,0,1,-1,1};
    ec::control::PositionLoop loop{{outer,{inner,{50},0,.005f,0},ec::control::ArcMode::continuous}};
    const auto grant=loop.arm(ec::CommandSource::remote,true,true);
    if (!grant || !loop.configured()) return 4;
    std::cout<<"t_s,command_a,position_p_a,rate_p_a,outer_i_a,inner_i_a,heading_rad,heading_rate_rad_s,heading_gain\n"<<std::setprecision(10);
    std::size_t idx=0;
    float previous_heading=0,continuous_heading=0;
    for (unsigned n=0;;++n) {
        const double t=rows.front().t+n*.001;
        if (t>rows.back().t) break;
        while (idx+1<rows.size()-1 && rows[idx+1].t<t) ++idx;
        const auto& a=rows[idx];const auto& b=rows[idx+1];
        const float weight=static_cast<float>((t-a.t)/(b.t-a.t));
        const auto lerp=[&](float x,float y){return x+(y-x)*weight;};
        const float sign=a.q.w*b.q.w+a.q.x*b.q.x+a.q.y*b.q.y+a.q.z*b.q.z<0 ? -1.0f:1.0f;
        const auto q=ec::estimation::normalized({lerp(a.q.w,sign*b.q.w),lerp(a.q.x,sign*b.q.x),lerp(a.q.y,sign*b.q.y),lerp(a.q.z,sign*b.q.z)});
        const ec::sentinel::app::Vector3 rate{lerp(a.rate[0],b.rate[0]),lerp(a.rate[1],b.rate[1]),lerp(a.rate[2],b.rate[2])};
        const auto p=ec::sentinel::app::reconstruct(model,q,rate,model.fire_reference,0,model.pitch_reference,0);
        if (!p.valid) return 5;
        if (n==0) continuous_heading=p.heading;
        else continuous_heading+=std::remainder(p.heading-previous_heading,2.0f*3.14159265358979323846f);
        previous_heading=p.heading;
        const float joint=lerp(a.joint,b.joint);
        const auto resolved=ec::sentinel::app::reachable_heading(model,q,joint,target);
        if (!resolved) return 6;
        const auto now=ec::Tick{n};
        const float adjusted_target=continuous_heading+p.heading_gain*(*resolved-joint);
        const ec::Command<float> command{adjusted_target,now,{100},*grant};
        const ec::Sample<float> position{continuous_heading,now,ec::Validity::valid};
        const ec::Sample<float> velocity{p.heading_rate,now,ec::Validity::valid};
        const auto result=loop.step_split(now,.001f,n%2==0 ? std::optional<float>{.002f}:std::nullopt,command,position,velocity,p.heading_gain);
        if (!result.output) return 7;
        const auto& o=loop.outer_diagnostics();const auto& i=loop.inner_diagnostics();
        std::cout<<t<<','<<*result.output*3<<','<<inner_kp*o.p/p.heading_gain*3<<','<<-inner_kp*p.heading_rate/p.heading_gain*3<<','<<inner_kp*o.i/p.heading_gain*3<<','<<i.i*3<<','<<continuous_heading<<','<<p.heading_rate<<','<<p.heading_gain<<'\n';
    }
}
