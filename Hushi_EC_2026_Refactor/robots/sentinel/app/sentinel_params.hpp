#pragma once
// 参数存储 (O5): 固件默认值 + RAM 覆盖, 不写 MCU Flash, 不改电调保存设置.
//  - 热调参数 (hot): 允许在线更新, 更新后立即影响控制, 读回取自实际控制配置;
//  - 参考参数 (reference): 改变零位/机械范围等参考语义, 必须在该机构退出执行后修改.
// 参数按 (域, 名称) 寻址; 未列出的参数不存在, 不接受任意内存/寄存器写入.
#include "sentinel_config.hpp"
#include "sentinel_types.hpp"
#include <array>
#include <cstdint>
#include <optional>

namespace ec::sentinel::app {
enum class ParamKind : std::uint8_t { hot = 0, reference = 1 };

// 参数域: 一个参数属于某个机构域, 或整车级 (底盘几何/时序/对齐).
inline constexpr std::size_t param_capacity = 96;

struct ParamSpec {
    std::uint16_t id{};
    ParamKind kind{ParamKind::hot};
    Domain domain{Domain::chassis_lf};
    bool whole_vehicle{}; // true 表示不属于单个机构域
    float default_value{};
    float min_value{};
    float max_value{};
    const char* name{};
    const char* unit{};
};

struct ParamEntry {
    ParamSpec spec{};
    float value{};      // 当前生效值 (RAM 覆盖后的值)
    bool overridden{};  // 是否已被 RAM 覆盖 (false = 仍是固件默认值)
};

// 参数写入结果. 与业务协议的结果码一一对应, 不在协议层另造一套语义.
enum class ParamWriteResult : std::uint8_t {
    accepted = 0,
    unknown_param,
    invalid_value,
    reference_in_use, // 参考参数: 受影响机构仍在执行
    not_configured,
    // 该参数尚未实现落到运行对象的生效路径. 明确上报, 不用 ParamStore 里的新数值
    // 冒充"已生效": 读回等于实际控制配置是参数接口的全部意义.
    unsupported,
};

class ParamStore {
public:
    explicit ParamStore(const SentinelConfig& config) noexcept;
    [[nodiscard]] const ParamSpec* find(std::uint16_t id) const noexcept;
    [[nodiscard]] const ParamEntry* entry(std::uint16_t id) const noexcept;
    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    [[nodiscard]] const ParamEntry& at(std::size_t index) const noexcept { return entries_[index]; }
    [[nodiscard]] bool has(std::uint16_t id) const noexcept { return find(id) != nullptr; }
    // 恢复某个参数到固件默认值; 参考类同样要求该机构已退出执行.
    [[nodiscard]] ParamWriteResult reset_to_default(std::uint16_t id, bool domain_idle) noexcept;
    // 写入: reference 类参数必须由调用方确认该域已退出执行 (idle 为 true).
    [[nodiscard]] ParamWriteResult write(
        std::uint16_t id, float value, bool domain_idle) noexcept;
    // 读回: 热调参数的实际生效值由执行层回填, 保证"读回 == 实际控制配置".
    void set_effective(std::uint16_t id, float value) noexcept;

private:
    [[nodiscard]] ParamEntry* find_mutable(std::uint16_t id) noexcept;
    [[nodiscard]] ParamWriteResult write_entry(ParamEntry& entry, float value,
        bool domain_idle) noexcept;

    std::array<ParamEntry, param_capacity> entries_{};
    std::size_t count_{};
};

// 参数 id 编码: 高位是域索引, 低位是该域的参数序号, 便于 Python 端构造与核对.
[[nodiscard]] constexpr std::uint16_t param_id(std::size_t domain_slot, std::uint16_t local) noexcept {
    return static_cast<std::uint16_t>((domain_slot << 8) | local);
}
} // 命名空间 ec::sentinel::app
