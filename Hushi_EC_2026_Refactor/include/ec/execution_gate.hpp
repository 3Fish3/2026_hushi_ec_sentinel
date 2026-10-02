#pragma once
#include <cstdint>
#include <limits>
#include <optional>

namespace ec {
// 控制来源. remote = 遥控; host = 上位机应用链; maintenance = PC 直连互斥维护会话
// (仅由应用层仲裁, 不参与遥控/上位机的正常来源竞争).
enum class CommandSource : std::uint8_t { none, remote, host, maintenance };
[[nodiscard]] constexpr bool valid_command_source(CommandSource source) noexcept {
    return source == CommandSource::remote || source == CommandSource::host ||
        source == CommandSource::maintenance;
}
struct Grant {
    std::uint64_t generation{};
    CommandSource source{CommandSource::none};
    bool operator==(const Grant&) const = default;
};

// 仅由一个控制任务操作. 本组件提供可撤销授权, 不替代机器人安全状态机或通信握手.
// 应用层先完成命令来源仲裁; MCU 重启时必须一起重置通道和授权门.
// 不得直接把线协议中的 generation 反序列化为本地有效授权.
//
// 作用域: 只承诺“启动内”撤销 —— 每次 arm/invalidate 都使之前的 token 失效, 且耗尽后不回绕.
// 本地 token 从不跨重启保存, 线协议也不携带本地 generation, 因此本组件不用随机初值伪装
// 跨重启隔离; 跨重启身份由会话层 (SessionId.boot) 与本地显式重建流程负责.
class ExecutionGate {
public:
    ExecutionGate() noexcept = default;
    ExecutionGate(const ExecutionGate&) = delete;
    ExecutionGate& operator=(const ExecutionGate&) = delete;
    [[nodiscard]] std::optional<Grant> arm(CommandSource source,
                                          bool explicitly_authorized,
                                          bool locally_ready) noexcept {
        invalidate(); // 失败或重复使能不能保留先前授权.
        if (!explicitly_authorized || !locally_ready || exhausted_ ||
            !valid_command_source(source)) {
            return std::nullopt;
        }
        grant_.source = source;
        enabled_ = true;
        return grant_;
    }
    void invalidate() noexcept {
        enabled_ = false;
        grant_.source = CommandSource::none;
        if (grant_.generation == std::numeric_limits<std::uint64_t>::max()) {
            exhausted_ = true; // 耗尽后禁止回绕, 防止旧授权重新有效.
        } else {
            ++grant_.generation;
        }
    }
    [[nodiscard]] bool accepts(Grant token) const noexcept {
        return enabled_ && token == grant_;
    }
private:
    Grant grant_{};
    bool enabled_{false};
    bool exhausted_{false};
};
} // 命名空间 ec
