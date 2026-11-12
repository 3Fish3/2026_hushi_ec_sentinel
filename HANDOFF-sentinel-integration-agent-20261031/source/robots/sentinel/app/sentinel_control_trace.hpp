#pragma once
#include "ec/time.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace ec::sentinel::app {
// 纯观测: 每次真实计算/提交后累计, 约30ms形成一条, 峰值/触限不会因主机低频读取丢掉.
// command是协议槽归一化请求, effort是电调原始反馈, 不将两者假定为相同电流量纲.
struct ControlTraceSample {
    std::uint32_t at_us{};
    float target{}, command{}, command_min{}, command_max{};
    std::int16_t effort{}, effort_min{}, effort_max{};
    std::uint16_t flags{}, samples{}, limited_samples{};
};
static_assert(sizeof(ControlTraceSample)==32);
class ControlTrace {
public:
    static constexpr std::uint32_t period_us=30000, capacity=32, page_size=6;
    // bits0/1:末样本命令/反馈有效; 2:执行; 3:段内提交失败; 4/5:段内命令/反馈缺失;
    // bit6:段内无有效命令, bit7:段内无有效反馈; bits8..9:末样本采用参考系.
    void observe(MicroTick at, float target, std::optional<float> command,
        std::optional<std::int16_t> effort, bool executing, unsigned frame,
        bool limited, bool submitted) noexcept {
        if (!started_) { started_=true; published_at_=at; }
        if (!pending_.samples) {
            pending_={};
            pending_.command_min=std::numeric_limits<float>::infinity();
            pending_.command_max=-std::numeric_limits<float>::infinity();
            pending_.effort_min=std::numeric_limits<std::int16_t>::max();
            pending_.effort_max=std::numeric_limits<std::int16_t>::min();
            pending_.flags=64u|128u;
        }
        pending_.at_us=at.us; pending_.target=target;
        pending_.flags=static_cast<std::uint16_t>((pending_.flags&0xf8u)|(executing?4u:0u)|(frame<<8));
        ++pending_.samples;
        if (command && std::isfinite(*command)) {
            pending_.flags=static_cast<std::uint16_t>((pending_.flags|1u)&~64u);
            pending_.command=*command;
            pending_.command_min=std::min(pending_.command_min,*command);
            pending_.command_max=std::max(pending_.command_max,*command);
            if (limited) { ++pending_.limited_samples; }
        } else { pending_.command=0; pending_.flags|=16u; }
        if (effort) {
            pending_.flags=static_cast<std::uint16_t>((pending_.flags|2u)&~128u);
            pending_.effort=*effort;
            pending_.effort_min=std::min(pending_.effort_min,*effort);
            pending_.effort_max=std::max(pending_.effort_max,*effort);
        } else { pending_.effort=0; pending_.flags|=32u; }
        if (!submitted) { pending_.flags|=8u; }
        if (at.us-published_at_.us<period_us) { return; }
        if (pending_.flags&64u) { pending_.command_min=pending_.command_max=0; }
        if (pending_.flags&128u) { pending_.effort_min=pending_.effort_max=0; }
        data_[next_%capacity]=pending_; ++next_; size_=std::min(size_+1,capacity);
        pending_.samples=0; published_at_=at;
    }
    struct Page { std::uint32_t first{}, next{}, count{}; bool future{}; };
    [[nodiscard]] Page page(std::optional<std::uint32_t> cursor={}) const noexcept {
        const auto distance=cursor ? next_-*cursor : std::min(size_,page_size);
        if (cursor && static_cast<std::int32_t>(distance)<0) { return {0,next_,0,true}; }
        const auto available=std::min(distance,size_);
        return {next_-available,next_,std::min(available,page_size),false};
    }
    // 仅接受page返回范围; 无动态分配, 同一appControl任务读写, 读取不清除记录.
    [[nodiscard]] const ControlTraceSample& at(std::uint32_t sequence) const noexcept { return data_[sequence%capacity]; }
private:
    std::array<ControlTraceSample,capacity> data_{};
    ControlTraceSample pending_{};
    std::uint32_t next_{},size_{};
    MicroTick published_at_{};
    bool started_{};
};
} // 命名空间 ec::sentinel::app
