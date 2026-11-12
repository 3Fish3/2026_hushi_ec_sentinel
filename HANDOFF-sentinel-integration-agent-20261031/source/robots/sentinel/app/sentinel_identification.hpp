#pragma once
#include "ec/time.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace ec::sentinel::app {
// V2仅诊断使用: 平滑带入固定电流偏置, 确认持续旋转后在闭环输出端叠加独立电流正弦.
// bias/amplitude单位为A, 不是PID或比赛限额. 0.18A偏置取自首轮带载命令量级, 仍属试验值.
struct IdentificationPlan {
    static constexpr std::uint8_t schema=2;
    enum class Profile : std::uint8_t { sweep, velocity_validation };
    Profile profile{Profile::sweep};
    static constexpr float validation_duration=15, stop_current_a=.35f, stop_speed=.4f;
    float speed{1.0f}, amplitude_a{0.06f}, bias_a{0.18f}, ramp_s{5.0f};
    float settle_timeout_s{15.0f}, zero_s{3.0f}, acceleration{0.8f};
    std::array<float,6> frequencies{0.25f,0.5f,1.0f,2.0f,4.0f,0};
    std::uint8_t count{5}, cycles{4};
    [[nodiscard]] float scan_duration() const noexcept {
        float value=0;
        for(unsigned i=0;i<count && i<frequencies.size();++i) { value+=(cycles+2)/frequencies[i]; }
        return value;
    }
    [[nodiscard]] float duration() const noexcept { return profile==Profile::velocity_validation ? validation_duration : 2*ramp_s+settle_timeout_s+scan_duration()+zero_s; }
    [[nodiscard]] bool valid() const noexcept {
        if(profile==Profile::velocity_validation) { return true; } // 固定profile不接受可调波形参数.
        if(profile!=Profile::sweep) { return false; }
        if(!std::isfinite(speed+amplitude_a+bias_a+ramp_s+settle_timeout_s+zero_s+acceleration) || speed==0 ||
            amplitude_a<=0 || bias_a<0 || amplitude_a+bias_a>=3 || ramp_s<=0 || settle_timeout_s<2 ||
            zero_s<=0 || acceleration<=0 || count==0 || count>frequencies.size() || cycles<3 || cycles>16) { return false; }
        for(unsigned i=0;i<count;++i) {
            if(!std::isfinite(frequencies[i]) || frequencies[i]<=0 || frequencies[i]>20 ||
                (i && frequencies[i]<=frequencies[i-1])) { return false; }
        }
        return std::isfinite(duration()) && duration()<300 && 1.875f*std::fabs(speed)/ramp_s<=acceleration;
    }
    struct Point { float rate{}, bias_a{}, excitation_a{}; std::uint8_t stage{}; bool measuring{}, done{}; };
    [[nodiscard]] static float smooth(float x) noexcept { return x*x*x*(10+x*(-15+6*x)); }
    [[nodiscard]] float signed_bias() const noexcept { return std::copysign(bias_a,speed); }
    [[nodiscard]] static Point validation(float t) noexcept {
        constexpr float pi=3.14159265358979323846f;
        if(t<2) { return {0,0,0,0,false,false}; }
        if(t<2.5f) { return {.125f*(1-std::cos(pi*(t-2)/.5f)),0,0,1,false,false}; }
        if(t<5.5f) { return {.25f,0,0,2,true,false}; }
        if(t<6) { return {.125f*(1+std::cos(pi*(t-5.5f)/.5f)),0,0,3,false,false}; }
        if(t<8) { return {0,0,0,4,false,false}; }
        if(t<8.5f) { return {-.125f*(1-std::cos(pi*(t-8)/.5f)),0,0,5,false,false}; }
        if(t<11.5f) { return {-.25f,0,0,6,true,false}; }
        if(t<12) { return {-.125f*(1+std::cos(pi*(t-11.5f)/.5f)),0,0,7,false,false}; }
        return {0,0,0,8,false,t>=validation_duration};
    }
    [[nodiscard]] Point ramp(float t) const noexcept {
        const auto scale=smooth(std::clamp(t/ramp_s,0.0f,1.0f));
        return {speed*scale,signed_bias()*scale,0,0,false,false};
    }
    // t从实际完成工作点建立时开始计, 不从固定3秒保持段推测相位.
    [[nodiscard]] Point scan(float t) const noexcept {
        constexpr float pi=3.14159265358979323846f;
        for(unsigned i=0;i<count;++i) {
            const float f=frequencies[i], period=1/f, length=(cycles+2)*period;
            if(t<length) {
                float envelope=1;
                if(t<period) { envelope=0.5f*(1-std::cos(pi*t/period)); }
                else if(t>(cycles+1)*period) { envelope=0.5f*(1-std::cos(pi*(length-t)/period)); }
                return {speed,signed_bias(),amplitude_a*envelope*std::sin(2*pi*f*t),
                    static_cast<std::uint8_t>(2+i),t>=period && t<(cycles+1)*period,false};
            }
            t-=length;
        }
        if(t<ramp_s) {
            const auto scale=1-smooth(t/ramp_s);
            return {speed*scale,signed_bias()*scale,0,8,false,false};
        }
        return {0,0,0,9,false,t>=ramp_s+zero_s};
    }
};
// 48字节: 记录总编码命令及独立bias/excitation, 不把同一个闭环信号冒充独立激励.
struct IdentificationSample {
    std::uint32_t at_us{}, submit_us{};
    float reference{}, feedback{}, joint{}, heading_gain{}, p{}, i{};
    std::int16_t command_raw{}, effort_raw{};
    std::uint16_t source_age_us{}, flags{};
    float bias_a{}, excitation_a{};
};
static_assert(sizeof(IdentificationSample)==48);
class MainYawIdentification {
public:
    enum class State : std::uint8_t { idle, armed, running, complete, aborted };
    enum class Reason : std::uint8_t { none, cancelled, execution_lost, controller, reference_changed, tx_failure, working_point, current_stop, speed_stop, parameters_changed };
    static constexpr std::uint32_t capacity=200, page_size=4;
    IdentificationPlan plan{};
    State state{State::idle}; Reason reason{};
    std::uint32_t session{}, run_id{}, config_epoch{}, reference_epoch{}, start_us{}, produced{}, consumed{}, dropped{};
    std::uint32_t excitation_start_us{};
    float raw_to_axis_amp{};
    bool reset_inner{};
    [[nodiscard]] bool engaged() const noexcept { return state!=State::idle; }
    [[nodiscard]] bool active() const noexcept { return state==State::armed || state==State::running; }
    [[nodiscard]] bool validating() const noexcept { return plan.profile==IdentificationPlan::Profile::velocity_validation; }
    void start(const IdentificationPlan& p, std::uint32_t owner, std::uint32_t run,
        std::uint32_t config, std::uint32_t reference, float scale) noexcept {
        plan=p;session=owner;run_id=run;config_epoch=config;reference_epoch=reference;
        produced=consumed=dropped=start_us=excitation_start_us=0;raw_to_axis_amp=scale;reason=Reason::none;
        scan_started_=previous_valid_=false;sum_=square_sum_=window_s_=previous_mean_=0;
        state=State::armed;reset_inner=true;
    }
    void abort(Reason why) noexcept { if(active()) { state=State::aborted;reason=why; } }
    [[nodiscard]] IdentificationPlan::Point point(MicroTick now, float feedback, float dt_s) noexcept {
        if(state==State::armed) { start_us=now.us;state=State::running; }
        const float elapsed=(now.us-start_us)*1e-6f;
        if(validating()) {
            const auto result=IdentificationPlan::validation(elapsed);
            if(result.done) { state=State::complete; }
            return result;
        }
        if(elapsed<plan.ramp_s) { return plan.ramp(elapsed); }
        if(!scan_started_) {
            // 质量条件而非新增行车保护: 连续两个1s窗口同向均速>=目标一半,
            // 窗内标准差<=目标20%, 相邻均值差<=目标10%. 不要求先手调到零稳态误差.
            if(std::isfinite(feedback) && std::isfinite(dt_s) && dt_s>0) {
                window_s_+=dt_s;sum_+=feedback*dt_s;square_sum_+=feedback*feedback*dt_s;
            }
            if(window_s_>=1.0f) {
                const float mean=sum_/window_s_, variance=std::max(0.0f,square_sum_/window_s_-mean*mean);
                const float magnitude=std::fabs(plan.speed);
                const bool good=mean*std::copysign(1.0f,plan.speed)>=0.5f*magnitude && variance<=0.04f*magnitude*magnitude;
                if(good && previous_valid_ && std::fabs(mean-previous_mean_)<=0.1f*magnitude) {
                    scan_started_=true;excitation_start_us=now.us;
                }
                previous_valid_=good;previous_mean_=mean;window_s_=sum_=square_sum_=0;
            }
            if(!scan_started_) {
                if(elapsed>=plan.ramp_s+plan.settle_timeout_s) {
                    abort(Reason::working_point);return {0,0,0,1,false,true};
                }
                return {plan.speed,plan.signed_bias(),0,1,false,false};
            }
        }
        const auto result=plan.scan((now.us-excitation_start_us)*1e-6f);
        if(result.done) { state=State::complete; }
        return result;
    }
    void append(const IdentificationSample& sample) noexcept {
        if(produced-consumed==capacity) { ++consumed; ++dropped; }
        data_[produced%capacity]=sample; ++produced;
    }
    [[nodiscard]] const IdentificationSample& sample(std::uint32_t sequence) const noexcept { return data_[sequence%capacity]; }
    [[nodiscard]] std::uint32_t pending() const noexcept { return produced-consumed; }
    void clear() noexcept { state=State::idle;reason=Reason::none;produced=consumed=dropped=0; }
private:
    bool scan_started_{},previous_valid_{};
    float window_s_{},sum_{},square_sum_{},previous_mean_{};
    std::array<IdentificationSample,capacity> data_{};
};
} // 命名空间 ec::sentinel::app
