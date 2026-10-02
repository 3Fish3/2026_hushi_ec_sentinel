#pragma once
// 串联机构原生 SIL 夹具: 真值只生成传感器与评分, 不注入 APP 的姿态/速率结果.
// 独立 double Rodrigues 正向矩阵, 不调用生产 reconstruct/reachable_heading/propagate.
// 固定采用 synthetic_config 的槽4=main/槽5=fire 装配, 不是任意安装表模拟器.
// 参数、惯量、阻尼与一周期延迟均为合成工况, 不代表实车稳定性或调参依据.
#include "sim_plant.hpp"
#include "ec/drivers/bmi088_device.hpp"
#include <algorithm>

namespace ec::sentinel::host {
using Matrix64 = std::array<double, 9>;
using Vector64 = std::array<double, 3>;
inline Matrix64 multiply64(const Matrix64& a, const Matrix64& b) {
    Matrix64 r{};
    for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) for(unsigned k=0;k<3;++k) r[i*3+j]+=a[i*3+k]*b[k*3+j];
    return r;
}
inline Matrix64 transpose64(const Matrix64& a) { return {a[0],a[3],a[6],a[1],a[4],a[7],a[2],a[5],a[8]}; }
inline Vector64 transform64(const Matrix64& a, const Vector64& v) {
    return {a[0]*v[0]+a[1]*v[1]+a[2]*v[2],a[3]*v[0]+a[4]*v[1]+a[5]*v[2],a[6]*v[0]+a[7]*v[1]+a[8]*v[2]};
}
inline Matrix64 rotate64(const app::Vector3& a, double angle) {
    const double c=std::cos(angle),s=std::sin(angle),d=1-c,x=a[0],y=a[1],z=a[2];
    return {c+x*x*d,x*y*d-z*s,x*z*d+y*s,y*x*d+z*s,c+y*y*d,y*z*d-x*s,z*x*d-y*s,z*y*d+x*s,c+z*z*d};
}
inline Vector64 wide(const app::Vector3& v) { return {v[0],v[1],v[2]}; }
inline app::Vector3 narrow(const Vector64& v) { return {static_cast<float>(v[0]),static_cast<float>(v[1]),static_cast<float>(v[2])}; }

class SerialPlant {
public:
    SimPlant load{};
    double base_yaw{}, base_rate{}, base_roll{}, base_pitch{};
    bool delayed_commands{true};

    struct ImuSample {
        app::Vector3 gyro{}, accel{};
        MicroTick at{};
        std::uint32_t sequence{};
    };
    [[nodiscard]] Matrix64 platform(const app::SerialGeometry& m, double h=0) const {
        const auto base=multiply64(rotate64({0,0,1},base_yaw+base_rate*h),
            multiply64(rotate64({0,1,0},base_pitch),rotate64({1,0,0},base_roll)));
        return multiply64(base,rotate64(m.main_axis,load.dji[4].position_rad+load.dji[4].velocity_rad_s*h));
    }
    [[nodiscard]] Matrix64 sensor(const app::SerialGeometry& m, double h=0) const {
        return multiply64(platform(m,h),multiply64(
            rotate64(m.fire_axis,load.dji[5].position_rad-m.fire_reference+load.dji[5].velocity_rad_s*h),
            rotate64(m.pitch_axis,load.dm.position_rad-m.pitch_reference+load.dm.velocity_rad_s*h)));
    }
    [[nodiscard]] float heading(const app::SerialGeometry& m) const {
        const auto f=transform64(platform(m),wide(m.forward));
        return static_cast<float>(std::atan2(f[1],f[0]));
    }
    [[nodiscard]] ImuSample sample(const app::SerialGeometry& m, MicroTick at, std::uint32_t sequence) const {
        const auto current=sensor(m), before=sensor(m,-1e-5), after=sensor(m,1e-5);
        Matrix64 derivative{};
        for(unsigned i=0;i<9;++i) derivative[i]=(after[i]-before[i])/2e-5;
        const auto skew=multiply64(transpose64(current),derivative);
        const Vector64 rate{(skew[7]-skew[5])/2,(skew[2]-skew[6])/2,(skew[3]-skew[1])/2};
        Matrix64 install{};
        for(unsigned i=0;i<9;++i) install[i]=m.install.m[i];
        ImuSample out;
        out.gyro=narrow(transform64(transpose64(install),rate));
        for(unsigned i=0;i<3;++i) out.gyro[i]+=m.gyro_bias_sensor[i];
        out.accel=narrow(transform64(transpose64(install),transform64(transpose64(current),{0,0,9.81})));
        out.at=at; out.sequence=sequence;
        return out;
    }
    template<typename Route>
    void emit_feedback(const SentinelConfig& config, Route&& route, Tick now) {
        load.emit_feedback(config,[&](std::uint8_t bus, const transport::CanFrame& frame, Tick at) {
            // DM 只对已到达的 MIT/管理/状态问询作反馈; 失能不等于无反馈.
            if (bus==config.dm[0].bus && frame.id==config.dm[0].master_id && !dm_reply_) return;
            route(bus,frame,at);
        },now);
        dm_reply_=false;
    }
    void integrate(const SentinelConfig& config, const SoftwareBus& bus1, const SoftwareBus& bus2, float dt_s) {
        latch(config, delayed_commands ? pending1_ : bus1, 1, held1_);
        latch(config, delayed_commands ? pending2_ : bus2, 2, held2_);
        pending1_.frames=bus1.frames; pending2_.frames=bus2.frames;
        load.integrate(config,held1_,held2_,dt_s);
        base_yaw+=base_rate*dt_s;
    }
private:
    SoftwareBus held1_{},held2_{},pending1_{},pending2_{};
    bool dm_reply_{};
    void latch(const SentinelConfig& config, const SoftwareBus& received, std::uint8_t bus, SoftwareBus& held) {
        for(const auto& f:received.frames) {
            auto found=std::find_if(held.frames.begin(),held.frames.end(),[&](const auto& old){return old.id==f.id;});
            if(found==held.frames.end()) held.frames.push_back(f); else *found=f;
            const auto& dm=config.dm[0];
            if(bus!=dm.bus) continue;
            if(f.id==dm.can_id && f.size==8) {
                dm_reply_=true;
                bool special=true;
                for(unsigned i=0;i<7;++i) special=special && f.data[i]==0xff;
                if(special && f.data[7]==0xfc) load.dm_reports_enabled=true;
                if(special && f.data[7]==0xfd) load.dm_reports_enabled=false;
            } else if(f.id==0x7ff && f.size==4 && f.data[2]==0xcc) {
                dm_reply_=true;
            }
        }
    }
};
} // 命名空间 ec::sentinel::host
