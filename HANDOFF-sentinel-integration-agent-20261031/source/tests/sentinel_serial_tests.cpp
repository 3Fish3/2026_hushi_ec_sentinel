// N3 独立正向矩阵/数值微分与真实分时采样链. expected 不调用串联反解/四元数传播.
#include "sentinel_perception.hpp"
#include "heading_geometry.hpp"
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
    CHECK(p.tilt_diagnostics().counts[static_cast<unsigned>(ec::estimation::TiltResult::rejected_norm)]>0);
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
// 真实关节记录在gyro之后2ms到达, 正/负lag分别由历史插值处理, 不以now/命令角作锚点.
void anchored_acceleration() {
    auto config=synthetic_config();
    auto model=*ec::sentinel::test::heading_geometry;
    model.position_lag_us={1000,-1000}; model.velocity_lag_us=model.position_lag_us;
    config.serial=model;
    SentinelPerception p{config,ConfigAudience::offline_test};
    const auto main_angle=[](double t) { return 0.7*std::min(t,1.0); };
    const auto joint=[&](unsigned j,double t) { return j ? model.pitch_reference-0.33+0.35*std::sin(0.8*t)
        : model.fire_reference+0.42+0.55*std::sin(1.3*t); };
    const auto sensor=[&](double t) { return product(product(rotation({0,0,1},main_angle(t)),
        product(rotation({0,1,0},-0.31),rotation({1,0,0},0.19))),
        product(rotation(model.fire_axis,joint(0,t)-model.fire_reference),rotation(model.pitch_axis,joint(1,t)-model.pitch_reference))); };
    const auto joints=[&](int i) {
        for(unsigned j=0;j<2;++j) {
            const double t=i*.001+model.position_lag_us[j]*1e-6;
            DomainFeedback f;
            f.position_rad={static_cast<float>(joint(j,t)),{static_cast<std::uint32_t>(1000+i)},ec::Validity::valid};
            f.velocity_rad_s={static_cast<float>(j ? .28*std::cos(.8*t) : .715*std::cos(1.3*t)),f.position_rad.sampled_at,ec::Validity::valid};
            p.submit_joint(j ? Domain::pitch : Domain::fire_yaw,f,{static_cast<std::uint32_t>(1000000+i*1000)},1);
        }
    };
    for(int i=-5;i<2;++i) { joints(i); }
    ec::drivers::ImuSamplingTimes times{}; Vector3 accel{}; float initial{}, peak{};
    for(unsigned i=0;i<=14000;++i) {
        const double t=i*.001; const auto rs=sensor(t); const std::uint32_t us=1000000+i*1000;
        auto gyro=body_rate(sensor(t-1e-5),sensor(t+1e-5),rs,1e-5);
        for(unsigned j=0;j<3;++j) { gyro[j]+=model.gyro_bias_sensor[j]; }
        times.gyro_valid=times.gyro_updated=true; times.gyro_sequence=i+1; times.gyro=times.gyro_estimated={us};
        times.accel_updated=i%2==0;
        if(times.accel_updated) {
            const Vec force=t>2 ? Vec{2*std::sin(6*(t-2)),2*std::cos(6*(t-2)),9.81} : Vec{0,0,9.81};
            accel=fvec(transform_vector(transposed(rs),force)); times.accel={us}; times.accel_valid=true; ++times.accel_sequence;
        }
        joints(static_cast<int>(i)+2);
        const ec::Tick now{1002+i}; const ec::MicroTick at{us+2000};
        p.submit_timed_imu(gyro,accel,times,now,at); p.update(now,at);
        CHECK(p.platform().validity==DomainValidity::valid);
        if(!i) { initial=p.platform().continuous_heading-static_cast<float>(main_angle(t+.002)); }
        if(i>2000) { peak=std::max(peak,std::fabs(p.platform().continuous_heading-initial-.7f)); }
        if(times.accel_updated) {
            const auto& d=p.tilt_diagnostics();
            CHECK(d.queried_at[0].us==us-1000 && d.queried_at[1].us==us+1000);
            CHECK(d.extrapolated_mask==0);
        }
    }
    CHECK(peak<.004f && p.tilt_diagnostics().max_abs_heading_injection_rad<2e-6f);
    CHECK(p.stats().accel_updates>1000 && p.local_epoch()==1);
    CHECK(p.tilt_diagnostics().counts[TiltFusionDiagnostics::missing_anchor]==0);
    std::printf("APP_ANCHORED_ACCEL peak_rad=%.9f injection_rad=%.9f\n",peak,p.tilt_diagnostics().max_abs_heading_injection_rad);
    // 参考变化清旧历史; 缺锚点继续gyro预测, 不缓存量测等待下一代参考重放.
    p.invalidate_joint_reference(Domain::fire_yaw);
    ++times.gyro_sequence; ++times.accel_sequence; times.accel_updated=true;
    times.gyro=times.gyro_estimated=times.accel={15002000};
    p.submit_timed_imu(model.gyro_bias_sensor,accel,times,{15002},{15002000}); p.update({15002},ec::MicroTick{15002000});
    CHECK(p.tilt_diagnostics().result==TiltFusionDiagnostics::missing_anchor);
    CHECK(p.platform().validity!=DomainValidity::valid);
}
void prime_alignment_and_invalid_inputs() {
    auto config=synthetic_config(); SentinelPerception p{config,ConfigAudience::offline_test};
    ec::drivers::ImuSamplingTimes t{};
    t.gyro_valid=t.gyro_updated=t.accel_valid=t.accel_updated=true;
    t.gyro_sequence=t.accel_sequence=1; t.gyro=t.gyro_estimated={100000};t.accel={90000};
    p.submit_timed_imu({}, {0,0,9.81f},t,{100},{100000});p.update({100},ec::MicroTick{100000});
    CHECK(p.local_epoch()==0 && p.inertial().validity!=DomainValidity::valid);
    // 首次prime也以有界陀螺速率对齐accel到gyro时刻, 不是直接拿旧重力建姿态.
    t.gyro_sequence=t.accel_sequence=2;t.accel={96000};t.gyro=t.gyro_estimated={101000};
    const Vector3 gyro{1,0,0}, old_accel{0,0,9.81f};
    p.submit_timed_imu(gyro,old_accel,t,{101},{101000});p.update({101},ec::MicroTick{101000});
    const auto expected=transform_vector(rotation({1,0,0},-.005),{0,0,1});
    const auto measured=p.inertial().gravity_direction_body.value;
    for(unsigned j=0;j<3;++j) { CHECK(std::fabs(measured[j]-expected[j])<1e-6); }
    CHECK(p.local_epoch()==1);
    // 长gap的新gyro不可用尚在TTL内、但错开10ms的accel重新prime.
    t.gyro_sequence=t.accel_sequence=3;t.accel={121000};t.gyro=t.gyro_estimated={131000};
    p.submit_timed_imu(gyro,old_accel,t,{131},{131000});p.update({131},ec::MicroTick{131000});
    CHECK(p.local_epoch()==1 && p.inertial().validity!=DomainValidity::valid);
    ++t.gyro_sequence;++t.accel_sequence;t.accel=t.gyro=t.gyro_estimated={132000};
    p.submit_timed_imu(gyro,old_accel,t,{132},{132000});p.update({132},ec::MicroTick{132000});
    CHECK(p.local_epoch()==2 && p.stats().reprime_events==1);
    CHECK(!reconstruct(*config.serial,{0,0,0,0},{},0,0,0,0).valid);
    CHECK(!reconstruct(*config.serial,{std::nanf(""),0,0,0},{},0,0,0,0).valid);
    // 无update介入而跨完整微秒回绕: 校正入口本身必须先淘汰陈旧关节.
    SentinelPerception stale{config,ConfigAudience::offline_test};
    t={};t.gyro_valid=t.gyro_updated=t.accel_valid=t.accel_updated=true;
    t.gyro_sequence=t.accel_sequence=1;t.gyro=t.gyro_estimated=t.accel={1000000};
    for (const auto domain : {Domain::fire_yaw,Domain::pitch}) {
        DomainFeedback f;f.position_rad={0,{1000},ec::Validity::valid};f.velocity_rad_s={0,{1000},ec::Validity::valid};
        stale.submit_joint(domain,f,{1000000},1);
    }
    stale.submit_timed_imu({},old_accel,t,{1000},{1000000});
    ++t.gyro_sequence;++t.accel_sequence;
    stale.submit_timed_imu({},old_accel,t,{4295968},{1000000});
    CHECK(stale.local_epoch()==2);
    CHECK(stale.tilt_diagnostics().result==TiltFusionDiagnostics::missing_anchor);
    // 负lag可请求now以后的有限预测, 来源仍必须已到达, 显式标记外推.
    config.serial->position_lag_us={-1000,-5000};
    SentinelPerception extrapolated{config,ConfigAudience::offline_test};
    for (const auto domain : {Domain::fire_yaw,Domain::pitch}) {
        DomainFeedback f;f.position_rad={0,{1000},ec::Validity::valid};f.velocity_rad_s={0,{1000},ec::Validity::valid};
        extrapolated.submit_joint(domain,f,{1000000},1);
    }
    extrapolated.submit_timed_imu({},old_accel,t,{1000},{1000000});
    CHECK(extrapolated.tilt_diagnostics().queried_at[0].us==1001000);
    CHECK(extrapolated.tilt_diagnostics().extrapolated_mask==1);
    CHECK(extrapolated.tilt_diagnostics().result==TiltFusionDiagnostics::missing_anchor); // 另一轴恰好5000us, 严格拒绝.
}
} // 测试辅助
int main() {
    mathematics(); history(); sampled_pipeline(); anchored_acceleration(); prime_alignment_and_invalid_inputs();
    std::puts("sentinel serial contracts passed");
}
