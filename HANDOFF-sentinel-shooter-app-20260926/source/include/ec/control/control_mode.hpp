#pragma once
#include <cstdint>

namespace ec::control {
// 控制模式只描述目标语义, 不承载权限、故障动作或恢复条件; 安全状态另由授权门
// 与每周期拒绝原因表达, 不合并成单一整车 mode 枚举.
// none 表示尚未选定目标语义, 不是一种执行模式.
enum class ControlMode : std::uint8_t { none, current, velocity, position };
} // 命名空间 ec::control
