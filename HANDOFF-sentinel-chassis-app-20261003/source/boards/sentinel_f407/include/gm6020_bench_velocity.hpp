#pragma once
#include "ec/time.hpp"
#include <array>
#include <cmath>
#include <cstdint>

namespace ec::sentinel {
// S5/直驱GM6020对照测速: 对最近5个已到达编码器样本作线性最小二乘.
// 不读未来样本、不外推, 时间戳落在窗口中心, 1kHz时约2ms估计延迟.
// 单圈差值在既有50ms反馈契约内解圈; 不建立/改变机械位置参考.
class BenchEncoderVelocity {
public:
    static constexpr std::size_t window=5;
    void receive(std::uint16_t encoder,ec::Tick at,ec::MicroTick at_us) noexcept {
        if (size_) {
            const auto dt=at_us.us-points_[size_-1].at.us;
            if (at.ms-last_ms_.ms>=50 || dt>=50000u) { size_=0;value_={}; }
            else if (dt==0) { return; }
        }
        int position=0;
        if (size_) {
            int delta=static_cast<int>(encoder)-static_cast<int>(previous_encoder_);
            if(delta>4095){delta-=8192;}else if(delta< -4096){delta+=8192;}
            position=points_[size_-1].position+delta;
        }
        if(size_==window){for(std::size_t i=1;i<window;++i){points_[i-1]=points_[i];}--size_;}
        points_[size_++]={position,at_us};previous_encoder_=encoder;last_ms_=at;
        // 局部计数始终以窗口首样本为原点, 不累积长时间多圈计数或float大角度误差.
        const int origin=points_[0].position;
        for(std::size_t i=0;i<size_;++i){points_[i].position-=origin;}
        if(size_<window){return;}
        std::array<float,window> t{};
        float mean_t=0,mean_q=0;
        for(std::size_t i=0;i<window;++i){
            t[i]=-static_cast<float>(at_us.us-points_[i].at.us)*1e-6f;
            mean_t+=t[i]/window;mean_q+=static_cast<float>(points_[i].position)/window;
        }
        float numerator=0,denominator=0;
        for(std::size_t i=0;i<window;++i){
            const float centered=t[i]-mean_t;
            numerator+=centered*(static_cast<float>(points_[i].position)-mean_q);
            denominator+=centered*centered;
        }
        const float velocity=numerator/denominator*0.0007669903939428206f;
        const auto lag_ms=static_cast<std::uint32_t>(std::ceil(-mean_t*1000.0f));
        value_={velocity,{at.ms-lag_ms},std::isfinite(velocity)?ec::Validity::valid:ec::Validity::invalid};
    }
    [[nodiscard]] ec::Sample<float> value() const noexcept {return value_;}
private:
    struct Point {int position{};ec::MicroTick at{};};
    std::array<Point,window> points_{};
    std::size_t size_{};
    std::uint16_t previous_encoder_{};
    ec::Tick last_ms_{};
    ec::Sample<float> value_{};
};
} // 命名空间 ec::sentinel
