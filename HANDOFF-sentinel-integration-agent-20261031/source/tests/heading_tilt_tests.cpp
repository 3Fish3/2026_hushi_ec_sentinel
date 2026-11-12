// 改编自20261108定向研究原生夹具: 独立double解析真值, 检验本仓库float核心.
// 覆盖直接注入、真实旋转、关节组合、坐标等价和零偏边界; 数值容差不是实车验收精度.
#include "ec/estimation/heading_tilt.hpp"
#include "heading_geometry.hpp"
#include "sentinel_kinematics.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>

namespace hr=ec::estimation;
namespace es=ec::estimation;
namespace app=ec::sentinel::app;
using DVec=std::array<double,3>;
constexpr double pi=std::numbers::pi, deg=180.0/pi;
int checks=0, failures=0;
void check(bool ok,const char* name) { ++checks; if(!ok) {++failures;std::printf("ASSERT_FAIL %s\n",name);} }
struct DQuat {double w=1,x=0,y=0,z=0;};
DQuat mul(DQuat a,DQuat b){return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}
DQuat con(DQuat q){return {q.w,-q.x,-q.y,-q.z};}
DQuat unit(DQuat q){double n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);return {q.w/n,q.x/n,q.y/n,q.z/n};}
DQuat dq(hr::Quaternion q){return unit({q.w,q.x,q.y,q.z});}
hr::Quaternion fq(DQuat q){return {float(q.w),float(q.x),float(q.y),float(q.z)};}
DVec dv(std::array<float,3> a){return {a[0],a[1],a[2]};}
std::array<float,3> fv(DVec a){return {float(a[0]),float(a[1]),float(a[2])};}
DVec add(DVec a,DVec b){return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
DVec sub(DVec a,DVec b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
DVec sc(DVec a,double s){return {a[0]*s,a[1]*s,a[2]*s};}
double dot(DVec a,DVec b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
DVec cr(DVec a,DVec b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
double norm(DVec a){return std::sqrt(dot(a,a));}
DVec unit(DVec a){return sc(a,1/norm(a));}
DVec rot(DQuat q,DVec v){auto r=mul(mul(q,{0,v[0],v[1],v[2]}),con(q));return {r.x,r.y,r.z};}
DQuat axis(DVec a,double t){a=unit(a);double s=std::sin(t/2);return {std::cos(t/2),a[0]*s,a[1]*s,a[2]*s};}
DQuat zyx(double r,double p,double y){return mul(mul(axis({0,0,1},y),axis({0,1,0},p)),axis({1,0,0},r));}
double heading(DQuat q,DVec f){auto w=rot(q,f);return std::atan2(w[1],w[0]);}
double angle_diff(double a,double b){return std::remainder(a-b,2*pi);}
double tilt_err(DQuat a,DQuat b){auto x=unit(rot(con(a),{0,0,1})),y=unit(rot(con(b),{0,0,1}));return std::atan2(norm(cr(x,y)),dot(x,y));}
double attitude_err(DQuat a,DQuat b){auto d=unit(mul(con(a),b));return 2*std::atan2(std::hypot(d.x,d.y,d.z),std::fabs(d.w));}
double hrate(DQuat q,DVec w,DVec f){auto a=rot(q,f),v=cr(rot(q,w),a);return (a[0]*v[1]-a[1]*v[0])/(a[0]*a[0]+a[1]*a[1]);}
hr::RotationMatrix3 matrix(DQuat q){auto x=rot(q,{1,0,0}),y=rot(q,{0,1,0}),z=rot(q,{0,0,1});return {{{float(x[0]),float(y[0]),float(z[0]),float(x[1]),float(y[1]),float(z[1]),float(x[2]),float(y[2]),float(z[2])}}};}
const auto model=*ec::sentinel::test::heading_geometry;
DQuat chain(double f,double p){return mul(axis(dv(model.fire_axis),f-model.fire_reference),axis(dv(model.pitch_axis),p-model.pitch_reference));}
DVec anchor(DQuat k){return rot(con(k),dv(model.forward));}
hr::Quaternion baseline_pose(float yaw){return es::multiply(app::axis_rotation({0,0,1},yaw),es::quaternion_from_gravity({-0.673284836f,0.168003842f,9.751691717f}));}


// 保持原始10.37度反例的float输入生成与计分顺序, 仅替换被测估计核心.
float original_input_probe(float amplitude,bool correction,float& rate){
    std::array<float,3> gravity{-.673284836f,.168003842f,9.751691717f};
    const float length=std::sqrt(app::dot(gravity,gravity));for(auto& x:gravity)x*=9.81f/length;
    const auto pose=es::multiply(app::axis_rotation({0,0,1},.7f),es::quaternion_from_gravity(gravity));
    hr::HeadingTilt filter;check(filter.reset(pose,model.gyro_bias_sensor),"original-input reset");
    const float wanted=app::reconstruct(model,pose,{},model.fire_reference,0,model.pitch_reference,0).heading;
    float peak=0;rate=0;
    for(unsigned i=1;i<=14000;++i){
        const float t=i*.001f,phase=6.0f*(t-2.0f);
        const std::array<float,3> a=t>2?std::array<float,3>{amplitude*std::sin(phase),amplitude*std::cos(phase),0}:std::array<float,3>{};
        const auto measured=es::rotate_reference_to_body(pose,{a[0],a[1],9.81f});
        (void)filter.predict(model.gyro_bias_sensor,.001f);
        if(i%2==0&&correction)(void)filter.correct_accel_aligned(measured,model.forward,.002f);
        const auto state=app::reconstruct(model,filter.orientation(),filter.gyro_rad_s(),model.fire_reference,0,model.pitch_reference,0);
        if(t>2){peak=std::max(peak,std::fabs(std::remainder(state.heading-wanted,2*std::numbers::pi_v<float>)));rate=std::max(rate,std::fabs(state.heading_rate));}
    }
    check(peak<2e-5f,"original-input stationary heading numerical tolerance");
    return peak*180/std::numbers::pi_v<float>;
}

void exact_counterexamples(){
    for(float yaw:{0.0f,0.7f,-1.1f,2.6f}) for(int sign:{1,-1}) for(int kind:{0,1,2}) {
        auto pose=baseline_pose(yaw); pose={pose.w*sign,pose.x*sign,pose.y*sign,pose.z*sign};
        hr::HeadingTilt filter;
        check(filter.reset(pose,model.gyro_bias_sensor),"counterexample reset");
        double wanted=heading(dq(pose),dv(model.forward)),peak=0,inject=0,tilt=0,rate=0;
        for(unsigned i=1;i<=14000;++i){
            const float t=i*.001f,phase=6*(t-2);
            DVec a{};
            if(t>2 && kind==1)a={2*std::sin(phase),2*std::cos(phase),0};
            if(t>2 && kind==2)a={2,0,0};
            auto measured=fv(rot(con(dq(pose)),add(a,{0,0,9.81})));
            (void)filter.predict(model.gyro_bias_sensor,.001f);
            if(i%2==0){(void)filter.correct_accel_aligned(measured,model.forward,.002f);inject=std::max(inject,std::fabs(double(filter.diagnostics().heading_injection_rad)));}
            auto state=app::reconstruct(model,filter.orientation(),filter.gyro_rad_s(),model.fire_reference,0,model.pitch_reference,0);
            peak=std::max(peak,std::fabs(angle_diff(state.heading,wanted)));
            tilt=std::max(tilt,tilt_err(dq(filter.orientation()),dq(pose)));
            rate=std::max(rate,std::fabs(double(state.heading_rate)));
        }
        std::printf("STATIC yaw=%+.1f sign=%+d accel=%d peak_heading_deg=%.9f peak_tilt_deg=%.6f injection_rad=%.3g peak_rate=%.3g\n",yaw,sign,kind,peak*deg,tilt*deg,inject,rate);
        check(peak<2e-5,"stationary heading (numerical, not field acceptance)");
        check(inject<1e-6,"per-correction heading invariant");
        check(rate<1e-7,"static bias subtracted exactly once");
        if(kind!=0){
            check(filter.diagnostics().corrections>1000,"acceleration fusion was active, not disabled");
            check(tilt>pi/180,"nonzero tilt response proves heading was isolated from active fusion");
        }
    }
}

struct Motion {DQuat qm{},k{},qs{};DVec wm{},ws{};double f=0,p=0,fd=0,pd=0;};
Motion motion(double t,int kind,double yaw){
    double roll=.19,pitch=-.31,yaw_t=yaw,rd=0,td=0,yd=0;
    if(kind==1){yaw_t+=.5*t;yd=.5;}
    if(kind==2||kind==6){roll+=.28*std::sin(1.1*t);rd=.308*std::cos(1.1*t);pitch+=.22*std::sin(.7*t);td=.154*std::cos(.7*t);yaw_t+=.4*t+.17*std::sin(.9*t);yd=.4+.153*std::cos(.9*t);}
    Motion m;
    m.qm=zyx(roll,pitch,yaw_t);
    m.wm={rd-yd*std::sin(pitch),td*std::cos(roll)+yd*std::sin(roll)*std::cos(pitch),-td*std::sin(roll)+yd*std::cos(roll)*std::cos(pitch)};
    m.f=model.fire_reference+.42; m.p=model.pitch_reference-.33;
    if(kind==3||kind==5||kind==6){m.f+=.55*std::sin(1.3*t);m.fd=.715*std::cos(1.3*t);}
    if(kind==4||kind==5||kind==6){m.p+=.35*std::sin(.8*t);m.pd=.28*std::cos(.8*t);}
    m.k=chain(m.f,m.p);m.qs=mul(m.qm,m.k);
    auto rf=axis(dv(model.fire_axis),m.f-model.fire_reference);
    m.ws=rot(con(m.k),add(m.wm,add(sc(dv(model.fire_axis),m.fd),sc(rot(rf,dv(model.pitch_axis)),m.pd))));
    return m;
}

struct Metrics {double heading=0,tilt=0,attitude=0,rate_truth=0,rate_difference=0,injection=0,norm_error=0;};
Metrics trajectory(int kind,int acceleration,int period=2,int skew_us=0,bool install_test=false,bool correction=true){
    const double dt=.001;
    const DQuat installq=install_test ? zyx(.61,-.43,.82) : DQuat{};
    const DVec bias{.012,-.019,.027};
    hr::HeadingTilt filter({},matrix(installq));
    const auto initial=motion(0,kind,.7);
    check(filter.reset(fq(initial.qs),fv(bias)),"trajectory reset");
    Metrics out;
    double previous_h=heading(initial.qm,dv(model.forward)),previous_rate=hrate(initial.qm,initial.wm,dv(model.forward));
    for(int i=1;i<=20000;++i){
        double t=i*dt;
        auto truth=motion(t,kind,.7);
        const auto raw=fv(add(rot(con(installq),truth.ws),bias));
        (void)filter.predict(raw,float(dt));
        if(i%period==0 && correction){
            double ta=t+skew_us*1e-6;
            auto at_accel=motion(ta,kind,.7);
            DVec a{};
            if(acceleration==1 && t>2 && t<14)a={2*std::sin(6*(ta-2)),2*std::cos(6*(ta-2)),0};
            if(acceleration==2 && t>2 && t<14)a={2,0,0};
            DVec measured=rot(con(at_accel.qs),add(a,{0,0,9.81}));

            const double w=norm(truth.ws);
            if(w>1e-12)measured=rot(axis(truth.ws,w*skew_us*1e-6),measured);
            (void)filter.correct_accel_aligned(fv(measured),fv(anchor(truth.k)),float(period*dt));
            out.injection=std::max(out.injection,std::fabs(double(filter.diagnostics().heading_injection_rad)));
        }
        auto state=app::reconstruct(model,filter.orientation(),filter.gyro_rad_s(),float(truth.f),float(truth.fd),float(truth.p),float(truth.pd));
        check(state.valid,"trajectory valid");
        const double h=heading(truth.qm,dv(model.forward));
        out.heading=std::max(out.heading,std::fabs(angle_diff(state.heading,h)));
        out.tilt=std::max(out.tilt,tilt_err(dq(filter.orientation()),truth.qs));
        out.attitude=std::max(out.attitude,attitude_err(dq(filter.orientation()),truth.qs));
        const double expected_rate=hrate(truth.qm,truth.wm,dv(model.forward));
        out.rate_truth=std::max(out.rate_truth,std::fabs(state.heading_rate-expected_rate));
        const double derivative=angle_diff(state.heading,previous_h)/dt;
        out.rate_difference=std::max(out.rate_difference,std::fabs(derivative-(previous_rate+state.heading_rate)*.5));
        const auto q=filter.orientation();
        out.norm_error=std::max(out.norm_error,std::fabs(double(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z)-1));
        previous_h=state.heading;previous_rate=state.heading_rate;
    }
    std::printf("TRAJECTORY kind=%d accel=%d accel_hz=%.0f skew_us=%d install=%d correction=%d heading_deg=%.6f tilt_deg=%.6f quaternion_deg=%.6f rate_truth=%.6g diff_vs_rate=%.6g inject=%.3g norm2_error=%.3g\n",kind,acceleration,1000.0/period,skew_us,install_test,correction,out.heading*deg,out.tilt*deg,out.attitude*deg,out.rate_truth,out.rate_difference,out.injection,out.norm_error);
    check(out.injection<2e-6,"dynamic correction invariant");
    check(out.norm_error<1e-6,"unit quaternion");
    if(acceleration==0){
        check(out.heading<.003,"true rotation response (endpoint integration tolerance)");
        check(out.tilt<.002&&out.attitude<.004,"roll/pitch and full attitude response");
    }
    if(kind==3||kind==4||kind==5)check(out.heading<.003,"fixed main and moving joints");
    return out;
}

void equivariance(){
    const auto change=zyx(.8,-.6,1.2);
    hr::HeadingTilt a,b;
    const auto q0=zyx(.2,-.3,.7), k=chain(.6,-.4);
    check(a.reset(fq(q0))&&b.reset(fq(mul(q0,con(change)))),"equivariance reset");
    double qerr=0,herr=0;
    for(int i=0;i<16000;++i){
        const double t=i*.001;DVec w{.1*std::sin(t),.17*std::cos(.8*t),.23};
        (void)a.predict(fv(w),.001f);(void)b.predict(fv(rot(change,w)),.001f);
        if(i%2==0){

            DVec acc=rot(con(q0),{2*std::sin(6*t),2*std::cos(6*t),9.81});
            DVec an=anchor(k);
            (void)a.correct_accel_aligned(fv(acc),fv(an),.002f);
            (void)b.correct_accel_aligned(fv(rot(change,acc)),fv(rot(change,an)),.002f);
        }
        const auto bS=mul(dq(b.orientation()),change);
        qerr=std::max(qerr,attitude_err(dq(a.orientation()),bS));
        herr=std::max(herr,std::fabs(angle_diff(heading(dq(a.orientation()),anchor(k)),heading(bS,anchor(k)))));
    }
    std::printf("COORDINATE_EQUIVARIANCE quaternion_deg=%.8f heading_deg=%.8f\n",qerr*deg,herr*deg);
    check(qerr<.0003&&herr<.0003,"arbitrary body-coordinate rotation equivalence");
}

// 冻结在线学习后如实保留未补偿零偏的漂移, 显式已知静止标定不得归零旧航向.
void tilt_recovery_and_bias(){
    auto truth=dq(baseline_pose(.7f)); const auto f=dv(model.forward);
    auto start=mul(zyx(.18,-.13,0),truth);

    start=mul(axis({0,0,1},angle_diff(heading(truth,f),heading(start,f))),start);
    hr::HeadingTilt filter;check(filter.reset(fq(start)),"tilt recovery reset");
    const double initial=tilt_err(start,truth);
    double hpeak=0;
    for(int i=1;i<=6000;++i){(void)filter.predict({},.001f);if(i%2==0)(void)filter.correct_accel_aligned(fv(rot(con(truth),{0,0,9.81})),model.forward,.002f);hpeak=std::max(hpeak,std::fabs(angle_diff(heading(dq(filter.orientation()),f),heading(start,f))));}
    std::printf("TILT_RECOVERY initial_deg=%.6f final_deg=%.8f heading_shift_deg=%.8f\n",initial*deg,tilt_err(dq(filter.orientation()),truth)*deg,hpeak*deg);
    check(tilt_err(dq(filter.orientation()),truth)<2e-5,"tilt converges without heading update");


    DVec residual=sc(rot(con(truth),{0,0,1}),.001);
    hr::HeadingTilt drift;check(drift.reset(fq(truth),model.gyro_bias_sensor),"bias drift reset");
    double unwrapped=0,prev=heading(truth,f),tilt=0;
    for(int i=1;i<=600000;++i){
        (void)drift.predict(fv(add(dv(model.gyro_bias_sensor),residual)),.001f);
        if(i%2==0)(void)drift.correct_accel_aligned(fv(rot(con(truth),{0,0,9.81})),model.forward,.002f);
        double h=heading(dq(drift.orientation()),f);unwrapped+=angle_diff(h,prev);prev=h;
        tilt=std::max(tilt,tilt_err(dq(drift.orientation()),truth));
    }
    std::printf("BIAS_UNOBSERVABLE seconds=600 bias_rad_s=.001 drift_deg=%.6f expected_deg=%.6f tilt_deg=%.6f\n",unwrapped*deg,.6*deg,tilt*deg);
    check(std::fabs(unwrapped-.6)<.004,"unobservable bias drift retained and reported");

    es::GyroBiasEstimator known_rest{200};
    const auto raw_rest=fv(add(dv(model.gyro_bias_sensor),residual));
    for(int i=0;i<1000;++i)known_rest.add(fv(sub(dv(raw_rest),dv(model.gyro_bias_sensor))));
    check(known_rest.estimate().has_value(),"known-rest mean ready");
    check(drift.set_residual_bias_sensor(*known_rest.estimate()),"full 3D residual installation");
    const double before=heading(dq(drift.orientation()),f);
    double recovery_peak=0;
    for(int i=1;i<=10000;++i){(void)drift.predict(fv(add(dv(model.gyro_bias_sensor),residual)),.001f);if(i%2==0)(void)drift.correct_accel_aligned(fv(rot(con(truth),{0,0,9.81})),model.forward,.002f);recovery_peak=std::max(recovery_peak,std::fabs(angle_diff(heading(dq(drift.orientation()),f),before)));}
    std::printf("CONFIRMED_REST_BIAS residual_z=%.9f subsequent_heading_change_deg=%.9f old_heading_error_is_not_reset=1\n",drift.residual_bias_sensor_rad_s()[2],recovery_peak*deg);
    check(recovery_peak<1e-4,"bias recovery stops new drift without erasing accumulated heading");
    check(std::fabs(drift.residual_bias_sensor_rad_s()[2])>0,"body Z is not forcibly zero");


    hr::HeadingTilt fixed_bias;check(fixed_bias.reset(fq(truth))&&fixed_bias.set_residual_bias_sensor({.004f,-.003f,.002f}),"explicit bias reset");
    for(int i=1;i<=20000;++i){
        (void)fixed_bias.predict({.004f,-.003f,.002f},.001f);
        if(i%2==0){const double t=i*.001;DVec aw=(t>2&&t<14)?DVec{2,0,0}:DVec{};(void)fixed_bias.correct_accel_aligned(fv(rot(con(truth),add(aw,{0,0,9.81}))),model.forward,.002f);}
    }
    std::printf("ACCEL_RECOVERY final_tilt_deg=%.8f unchanged_residual_xyz=%.6f,%.6f,%.6f\n",tilt_err(dq(fixed_bias.orientation()),truth)*deg,fixed_bias.residual_bias_sensor_rad_s()[0],fixed_bias.residual_bias_sensor_rad_s()[1],fixed_bias.residual_bias_sensor_rad_s()[2]);
    check(tilt_err(dq(fixed_bias.orientation()),truth)<3e-5,"tilt recovers after persistent acceleration");
    check(fixed_bias.residual_bias_sensor_rad_s()==std::array<float,3>{.004f,-.003f,.002f},"acceleration cannot contaminate residual bias");


    const auto tilted_truth=zyx(.35,-.55,.7);
    const auto unknown=rot(con(tilted_truth),{.006,0,0});
    hr::HeadingTilt transverse;check(transverse.reset(fq(tilted_truth)),"transverse bias reset");
    for(int i=1;i<=30000;++i){(void)transverse.predict(fv(unknown),.001f);if(i%2==0)(void)transverse.correct_accel_aligned(fv(rot(con(tilted_truth),{0,0,9.81})),model.forward,.002f);}
    const double bias_tilt=tilt_err(dq(transverse.orientation()),tilted_truth);
    std::printf("UNCOMPENSATED_TRANSVERSE_BIAS seconds=30 bias_norm=.006 tilt_final_deg=%.6f heading_error_deg=%.6f online_bias_norm=%.6f\n",bias_tilt*deg,angle_diff(heading(dq(transverse.orientation()),f),heading(tilted_truth,f))*deg,norm(dv(transverse.residual_bias_sensor_rad_s())));
    check(bias_tilt>.002&&bias_tilt<.01,"tilt feedback bounds transverse drift but does not learn bias");
}

void invalid_and_history(){
    hr::HeadingTilt f;check(f.reset({}),"guard reset");const auto initial=f.orientation();
    check(f.predict({},0)==hr::TiltResult::invalid_dt,"zero dt");
    check(f.predict({},.021f)==hr::TiltResult::invalid_dt,"long gyro gap must reprime outside core");
    check(f.predict({std::numeric_limits<float>::quiet_NaN(),0,0},.001f)==hr::TiltResult::invalid_input,"NaN gyro");
    check(f.correct_accel_aligned({0,0,9.81f},{0,0,1},.002f)==hr::TiltResult::rejected_projection,"vertical forward rejected");
    check(f.correct_accel_aligned({0,0,20},{1,0,0},.002f)==hr::TiltResult::rejected_norm,"norm gate");
    check(f.correct_accel_aligned({0,0,-9.81f},{1,0,0},.002f)==hr::TiltResult::rejected_antipodal,"antipodal gravity not arbitrary axis");
    check(f.correct_accel_aligned({0,0,9.81f},{1,0,0},.03f)==hr::TiltResult::invalid_dt,"long accel gap not huge correction");
    check(attitude_err(dq(initial),dq(f.orientation()))==0,"rejected inputs do not mutate orientation");
    check(!f.reset({0,0,0,0}),"zero quaternion rejected");
    for(int i=0;i<300;++i){
        auto q=zyx(.01*i,-.007*i,.013*i);hr::HeadingTilt anti;
        check(anti.reset(fq(q)),"nonidentity antipodal reset");
        check(anti.correct_accel_aligned(fv(rot(con(q),{0,0,-9.81})),fv(rot(con(q),{1,0,0})),.002f)==hr::TiltResult::rejected_antipodal,"nonidentity antipodal rejected");
    }
    app::JointHistory history;
    check(history.push({{1000},.1f,1})&&history.push({{3000},.102f,1}),"actual joint samples accepted");
    const auto mid=history.at({2000},{5000}),ext=history.at({7999},{5000}),boundary=history.at({8000},{5000}),bad=history.at({8001},{5000});
    check(mid&&std::fabs(mid->position-.101f)<1e-7,"bounded actual sample interpolation");
    check(ext.has_value()&&!boundary&&!bad,"strict extrapolation boundary retained");
    check(!history.push({{2500},.1f,1}),"regressed joint sample rejected");
    history.clear();check(!history.at({3000},{5000}),"reference change clears old anchor history");
    std::printf("GUARDS input_dt_projection_antipodal_history=checked (full Perception integration not compiled)\n");
}

int main(){
    std::printf("heading_validation v1 | double analytic truth, float estimator | numerical tolerances are not field acceptance\n");
    float rate=0;const auto gravity=original_input_probe(0,true,rate),accel=original_input_probe(2,true,rate);
    std::printf("ORIGINAL_INPUT_CANDIDATE gravity_only_deg=%.9f acceleration_corrected_deg=%.9f estimated_rate_peak=%.9f\n",gravity,accel,rate);
    std::printf("ORIGINAL_INPUT_CANDIDATE gyro_only_deg=%.9f\n",original_input_probe(2,false,rate));
    exact_counterexamples();
    for(int k=0;k<=6;++k){(void)trajectory(k,0);if(k>=3)(void)trajectory(k,1);}
    (void)trajectory(1,1);(void)trajectory(2,1);(void)trajectory(2,2);(void)trajectory(6,2);
    for(int period:{1,4,10}){(void)trajectory(6,0,period);}
    for(int skew:{-5000,-1000,1000,5000}){(void)trajectory(6,0,2,skew);}
    (void)trajectory(6,0,2,0,true);(void)trajectory(6,1,2,0,true);(void)trajectory(6,0,2,0,false,false);
    equivariance();tilt_recovery_and_bias();invalid_and_history();
    std::printf("CHECKS=%d FAILURES=%d RESULT=%s\n",checks,failures,failures?"FAIL":"PASS");
    return failures?1:0;
}
