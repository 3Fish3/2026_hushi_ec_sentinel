// N3 独立正向矩阵/数值微分与真实分时采样链. expected 不调用串联反解/四元数传播.
#include "sentinel_perception.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
using namespace ec::sentinel::app;
using Matrix = std::array<double, 9>;
using Vec = std::array<double, 3>;
void check(bool value, int line) { if (!value) { std::fprintf(stderr, "serial line %d\n", line); std::_Exit(3); } }
#define CHECK(...) check(static_cast<bool>((__VA_ARGS__)), __LINE__)
Matrix product(const Matrix& a, const Matrix& b) {
    Matrix result{};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) for(int k=0;k<3;++k) result[i*3+j]+=a[i*3+k]*b[k*3+j];
    return result;
}
Matrix transposed(const Matrix& a) { return {a[0],a[3],a[6],a[1],a[4],a[7],a[2],a[5],a[8]}; }
Vec transform_vector(const Matrix& a, const Vec& v) {
    return {a[0]*v[0]+a[1]*v[1]+a[2]*v[2], a[3]*v[0]+a[4]*v[1]+a[5]*v[2], a[6]*v[0]+a[7]*v[1]+a[8]*v[2]};
}
Matrix rotation(Vector3 a, double angle) {
    // 独立 double Rodrigues 正向模型.
    const double c=std::cos(angle), s=std::sin(angle), d=1-c;
    const double x=a[0], y=a[1], z=a[2];
    return {c+x*x*d,x*y*d-z*s,x*z*d+y*s,y*x*d+z*s,c+y*y*d,y*z*d-x*s,z*x*d-y*s,z*y*d+x*s,c+z*z*d};
}
Vector3 unit(Vector3 v) {
    const float norm=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    for (auto& x:v) x/=norm;
    return v;
}
Quaternion as_quaternion(const Matrix& m) {
    const auto root=[](double v){return static_cast<float>(std::sqrt(std::max(0.0,v))*0.5);};
    return {root(1+m[0]+m[4]+m[8]), std::copysign(root(1+m[0]-m[4]-m[8]),static_cast<float>(m[7]-m[5])),
        std::copysign(root(1-m[0]+m[4]-m[8]),static_cast<float>(m[2]-m[6])),
        std::copysign(root(1-m[0]-m[4]+m[8]),static_cast<float>(m[3]-m[1]))};
}
Vector3 fvec(Vec v) { return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])}; }
Vec dvec(Vector3 v) { return {v[0],v[1],v[2]}; }
Vector3 body_rate(const Matrix& before, const Matrix& after, const Matrix& current, double dt) {
    Matrix derivative{};
    for(std::size_t i=0;i<9;++i) derivative[i]=(after[i]-before[i])/(2*dt);
    const auto skew=product(transposed(current),derivative);
    return {static_cast<float>((skew[7]-skew[5])/2),static_cast<float>((skew[2]-skew[6])/2),static_cast<float>((skew[3]-skew[1])/2)};
}
SerialGeometry geometry() {
    auto model=*synthetic_config().serial;
    model.fire_axis=unit({0.12f,-0.06f,1}); model.pitch_axis=unit({0.14f,1,0.05f});
    model.main_axis=unit({0.04f,0.08f,1}); model.forward=unit({1,0.04f,0.05f});
    return model;
}
void mathematics() {
    const auto model=geometry();
    CHECK(model.valid());
    for(int i=0;i<2000;++i) {
        const double t=i*0.013;
        const auto platform=[&](double h) { return product(rotation({0,0,1},t+h*0.37),
            product(rotation({0,1,0},0.45*std::sin(t)),rotation({1,0,0},0.3*std::cos(t)))); };
        const double fire=0.9*std::sin(t), pitch=0.6*std::cos(t), vf=1.3*std::cos(t), vp=-0.8*std::sin(t);
        const auto sensor=[&](double h) { return product(platform(h),product(rotation(model.fire_axis,fire+h*vf),rotation(model.pitch_axis,pitch+h*vp))); };
        const auto rs=sensor(0);
        const auto rate=body_rate(sensor(-1e-5),sensor(1e-5),rs,1e-5);
        const auto out=reconstruct(model,as_quaternion(rs),rate,static_cast<float>(fire),static_cast<float>(vf),static_cast<float>(pitch),static_cast<float>(vp));
        CHECK(out.valid);
        const auto expected=transform_vector(platform(0),dvec(model.forward));
        CHECK(std::fabs(std::remainder(out.heading-std::atan2(expected[1],expected[0]),2*3.141592653589793)) < 0.002);
        CHECK(std::fabs(out.heading_rate-0.37f) < 0.002f);
        const auto expected_rate=body_rate(platform(-1e-5),platform(1e-5),platform(0),1e-5);
        for (std::size_t k=0;k<3;++k) CHECK(std::fabs(out.rate_V[k]-expected_rate[k])<0.002f);
        const auto target_vector=transform_vector(product(platform(0),rotation(model.main_axis,0.15)),dvec(model.forward));
        const auto target=static_cast<float>(std::atan2(target_vector[1],target_vector[0]));
        const auto result=reachable_heading(model,as_quaternion(platform(0)),0.2f,target,-1,1);
        CHECK(result && std::fabs(*result-0.35f)<0.003f);
        CHECK(!reachable_heading(model,as_quaternion(platform(0)),0.2f,target,0.1f,0.25f));
    }
    auto vertical=*synthetic_config().serial;
    const auto q=as_quaternion(rotation({0,1,0},3.141592653589793/2));
    CHECK(!reconstruct(vertical,q,{},0,0,0,0).valid);
    vertical.main_axis=vertical.forward;
    CHECK(!reconstruct(vertical,{}, {},0,0,0,0).valid);
    vertical.install.m[0]=-1;
    CHECK(!vertical.valid());
    // body-rate 必须右乘; 非零姿态下独立矩阵指数核对传播.
    const auto start=product(rotation({1,0,0},0.6),rotation({0,1,0},0.8));
    const auto result=propagate(as_quaternion(start),{0.2f,0,0},0.5f);
    const auto expected=transform_vector(product(start,rotation({1,0,0},0.1)),{0,0,1});
    const auto actual=ec::estimation::rotate_body_to_reference(result,{0,0,1});
    for (std::size_t i=0;i<3;++i) CHECK(std::fabs(actual[i]-expected[i])<0.001f);
}
void history() {
    JointHistory h;
    CHECK(h.push({{0xfffffff0u},1,1000}));
    CHECK(h.push({{0x10u},1.032f,1000}));
    const auto middle=h.at({0},{100});
    CHECK(middle && std::fabs(middle->position-1.016f)<1e-6f);
    CHECK(!h.push({{0x10u},2,1000}));
    CHECK(!h.push({{0xfffffff5u},2,1000}));
    CHECK(h.at({0x20},{100})); CHECK(!h.at({0x80},{100}));
    h.clear(); CHECK(!h.at({0x20},{100}));
    CHECK(h.push({{1000},1,0})); CHECK(h.push({{11000},2,0}));
    CHECK(!h.at({6000},{5000})); // 不跨未知大间隙插值
}
void sampled_pipeline() {
    auto config=synthetic_config(); auto model=geometry();
    model.gyro_bias_sensor={0.013f,-0.024f,0.007f};
    model.install=ec::estimation::rotation_zyx(0.2f,-0.3f,0.5f);
    config.serial=model;
    const Matrix install{model.install.m[0],model.install.m[1],model.install.m[2],model.install.m[3],model.install.m[4],model.install.m[5],model.install.m[6],model.install.m[7],model.install.m[8]};
    SentinelPerception p{config,ConfigAudience::offline_test};
    ec::drivers::ImuSamplingTimes times;
    Vector3 last_accel{};
    const auto sensor=[&](double t) { return product(product(rotation({0,0,1},0.7*t),
        product(rotation({0,1,0},-0.3),rotation({1,0,0},0.2))),product(rotation(model.fire_axis,0.6*std::sin(1.3*t)),rotation(model.pitch_axis,0.4*std::sin(0.8*t)))); };
    float initial{}; std::uint32_t corrections{};
    for(unsigned i=0;i<=11000;++i) {
        const double t=i*0.001;
        const std::uint32_t us=0xffb00000u+i*1000u;
        const ec::Tick ms{10000+i};
        const auto rs=sensor(t);
        auto gyro=fvec(transform_vector(transposed(install),dvec(body_rate(sensor(t-1e-5),sensor(t+1e-5),rs,1e-5))));
        for(std::size_t k=0;k<3;++k) gyro[k]+=model.gyro_bias_sensor[k];
        times.gyro_valid=times.gyro_updated=true; times.gyro_sequence=i+1;
        times.gyro=times.gyro_estimated={us};
        times.accel_updated=i%5==0;
        if(times.accel_updated) {
            last_accel=fvec(transform_vector(transposed(install),transform_vector(transposed(rs),{0,0,9.81})));
            if(i>400 && i<650) for(auto& x:last_accel) x*=1.4f; // 明确动态比力, 不当作新重力
            times.accel={us}; times.accel_valid=true; ++times.accel_sequence; ++corrections;
        }
        // 1 kHz fire / 500 Hz pitch; 每 7 个周期引入 200 µs 收帧延迟与状态错位.
        for(unsigned j=0;j<2;++j) {
            if(j==1 && i%2) continue;
            const auto age=i && i%7==0 ? 200u : 0u;
            const auto jt=t-age*1e-6;
            DomainFeedback fb;
            fb.position_rad={static_cast<float>(j ? 0.4*std::sin(0.8*jt) : 0.6*std::sin(1.3*jt)),ms,ec::Validity::valid};
            fb.velocity_rad_s={static_cast<float>(j ? 0.32*std::cos(0.8*jt) : 0.78*std::cos(1.3*jt)),ms,ec::Validity::valid};
            p.submit_joint(j ? Domain::pitch : Domain::fire_yaw,fb,{us-age},0);
        }
        // 有界单次缺样不重新建立参考; age 仍按真实采样时刻.
        if(i!=1200) p.submit_timed_imu(gyro,last_accel,times,ms,{us});
        p.update(ms,ec::MicroTick{us});
        CHECK(p.platform().validity==DomainValidity::valid);
        if(i==450 || i==453) {
            const auto& measured=p.inertial().accel_mps2;
            CHECK(std::fabs(std::sqrt(dot(measured.value,measured.value))-9.81f*1.4f)<0.002f);
            CHECK(measured.sampled_at.ms==10450); // 新鲜原始比力与重力融合质量是两件事
        }
        if(!i) initial=p.platform().continuous_heading;
        if(i>5) {
            CHECK(std::fabs(p.platform().continuous_heading-initial-static_cast<float>(0.7*t))<0.025f);
            CHECK(std::fabs(p.platform().value.heading_rate-0.7f)<0.025f);
        }
        CHECK(p.local_epoch()==1); CHECK(!p.alignment_valid());
    }
    CHECK(p.stats().accel_updates <= corrections);
    CHECK(p.attitude_state().measurement_skips>0);
    CHECK(p.stats().reprime_events==0);
    const auto last_us=times.gyro_estimated.us;
    p.update({21005},ec::MicroTick{last_us+5001});
    CHECK(p.platform().validity!=DomainValidity::valid); CHECK(p.local_epoch()==1);
    times.gyro_sequence++; times.gyro=times.gyro_estimated=times.accel={last_us+30000};
    times.accel_sequence++; times.accel_updated=true;
    p.set_yaw_alignment(0.3f,5);
    p.submit_timed_imu(model.gyro_bias_sensor,last_accel,times,{21030},{last_us+30000});
    CHECK(p.local_epoch()==2 && !p.alignment_valid());
    DomainFeedback invalid;
    p.submit_joint(Domain::pitch,invalid,{last_us+30000},0);
    p.update({21030},ec::MicroTick{last_us+30000});
    CHECK(p.platform().validity!=DomainValidity::valid);
    // 静态偏置在非单位安装前只减一次, 失效不污染其它机构.
    CHECK(std::fabs(p.inertial().gyro_rad_s.value[2])<1e-5f);
}
} // 测试辅助
int main() { mathematics(); history(); sampled_pipeline(); std::puts("sentinel serial contracts passed"); }
