#pragma once
#include "ec/time.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ec::monitor {
// 监控记录表: 固定容量、无动态分配、可被串口和 SWD 读到同一份逻辑状态.
// 记录只表达"谁、在什么时刻采到了什么、是否有效、为什么被拒绝";
// 不设计万能字段字典, 也不为未实现的传感来源编造健康零值.
// 写入与发布分离: 写者 (find/records) 改暂存视图, 读者 (read_page/records() const/
// published_records/描述符地址) 在 end_update 之前一直看到上一份已提交视图;
// end_update 把暂存整体复制到发布区(容量固定 48×24 B, 不依赖 dirty 区间), 再推进代次.
inline constexpr std::uint32_t layout_version = 1;
inline constexpr std::size_t record_capacity = 48;
inline constexpr std::size_t page_capacity = 8;
// 单条记录在内存与线协议上都是固定 24 字节, 字段不含指针/双精度,
// 因此 ARM32 与主机的布局一致 (见 static_assert).
inline constexpr std::size_t record_wire_size = 24;

enum class RecordKind : std::uint8_t {
    device = 1,      // 电机/设备: 反馈有效性、执行状态、目标/测量
    axis = 2,        // 轴: 模式、使能、目标与实际
    source = 3,      // 控制来源/输入: 遥控、链路
    period = 4,      // 周期诊断: 周期、超期、跳过
    environment = 5, // 环境/传感: IMU、温度
    unassigned = 0,
};

// unknown: 该来源本机还没有实现, 调用方不得当成"正常".
// unavailable: 来源存在但当前不可用 (未接线/未安装).
enum class Validity : std::uint8_t { unknown = 0, valid = 1, stale = 2, invalid = 3, unavailable = 4 };

enum class RejectReason : std::uint8_t {
    none = 0, no_feedback = 1, stale_feedback = 2, no_target = 3, no_grant = 4, not_ready = 5,
    source_mismatch = 6, target_stale = 7, link_stale = 8, policy_missing = 9,
    controller_error = 10, output_rejected = 11, tx_busy = 12, tx_error = 13, unknown = 255,
};

struct Record {
    std::uint16_t id{};
    RecordKind kind{RecordKind::unassigned};
    Validity validity{Validity::unknown};
    std::uint8_t mode{};     // control::ControlMode 的数值; 无控制语义时保持 0
    std::uint8_t enabled{};  // 独立的执行状态 (0/1), 不与 mode 合并
    RejectReason reject{RejectReason::none};
    std::uint8_t detail{};   // 由 kind+id 解释的补充位
    Tick sampled_at{};       // 该记录最后一次真实采样/判定时刻
    Tick published_at{};     // 本条记录写入发布区的时刻
    float target{};
    float measurement{};
};
static_assert(sizeof(Record) == record_wire_size, "Record 必须是 24 字节的定长布局");

// 整组发布代次: 写者每次发布把代次推进**两次** —— 复制前变奇数(发布中), 复制完成后变偶数(可读).
// 读取协议 (与工具端一致): 读 generation → 读 clock/used/记录 → 再读 generation;
// 只有“两次相同且为偶数”才是完整发布. 两次读取都落在复制窗口内时会读到同一个奇数, 因此
// 不会被误认为一致快照 —— 这是“复制后才 ++generation”做不到的.
// 代次变化本身不代表“数据错”, 也不保证跨重启唯一; 它只用于检测一次读取内的撕裂.
struct SnapshotHeader {
    std::uint32_t layout = layout_version;
    std::uint32_t generation{};
    std::uint32_t clock_ms{};
    std::uint32_t flags{}; // bit0: 监控启用 (静态属性, 不是发布状态快照)
    std::uint16_t used{};
    std::uint16_t reserved{};
};

// 分页: 只保证"本页内每条记录来自它自己的采样时刻", 不声称全车原子快照.
struct Page {
    SnapshotHeader header{};
    std::uint16_t first_index{};
    std::uint16_t count{};
    std::uint16_t total{};
    std::array<Record, page_capacity> records{};
};

// SWD 侧只读定位用的固定描述符. 全部字段为固定宽度整数, 不含指针,
// 因此主机与目标机布局一致; 地址字段在初始化时填入 (见 descriptor_init).
struct Descriptor {
    std::uint32_t magic{0x4d4f4e31u}; // "MON1"
    std::uint32_t layout{layout_version};
    std::uint32_t record_size{static_cast<std::uint32_t>(record_wire_size)};
    std::uint32_t capacity{static_cast<std::uint32_t>(record_capacity)};
    std::uint32_t records_address{};
    std::uint32_t generation_address{};
    std::uint32_t clock_address{};
    std::uint32_t build_id_address{};
    std::uint32_t used_address{};   // 已认领记录数 (uint32, 便于按同一布局读取)
    std::uint32_t flags{}; // bit0: 监控启用 (静态属性; 发布状态由 generation 奇偶携带)
};
static_assert(sizeof(Descriptor) == 40, "Descriptor 必须是 40 字节的定长布局");

// 单写者注册表. 注册在初始化阶段完成; 每周期只更新已有记录并推进代次.
// 发布模型: 写入总是针对暂存视图 (find/records 返回暂存), 读者 (read_page/records() const/
// published_records/描述符地址) 在 end_update 之前继续看到上一份已提交视图;
// end_update 只把本窗口改动过的记录复制到发布区, 然后推进代次.
// 跨上下文读取 (例如监控任务与发送任务不同) 必须走 copy_snapshot, 由调用方提供的
// 临界区完成正式同步; 同一任务内读取可直接用 published_records().
class Registry {
public:
    // 认领一个记录槽; 容量用尽返回空 (不动态扩容).
    // 认领会同时写入暂存与发布区: 注册发生在初始化阶段, 未开始发布前也必须能被读者看到.
    [[nodiscard]] std::optional<std::uint16_t> claim(RecordKind kind, std::uint16_t id) noexcept;
    // 写者视角: 返回暂存记录. 发布在 end_update 一次性完成, 不依赖“哪些槽被 find 过”.
    [[nodiscard]] Record* find(RecordKind kind, std::uint16_t id) noexcept;
    // 写者视角的暂存记录表.
    [[nodiscard]] std::span<Record> records() noexcept;
    // 读者视角: 最近一次已提交的发布视图.
    [[nodiscard]] std::span<const Record> records() const noexcept { return published_records(); }
    [[nodiscard]] std::span<const Record> published_records() const noexcept {
        return {published_.data(), used_};
    }
    [[nodiscard]] std::size_t used() const noexcept { return used_; }
    [[nodiscard]] std::uint32_t generation() const noexcept { return generation_; }
    [[nodiscard]] bool updating() const noexcept { return updating_; }
    // 更新窗口: begin 后的修改只进暂存; end 一次性把暂存整体发布并推进代次.
    void begin_update() noexcept;
    void end_update(Tick now) noexcept;
    // 填充固定描述符的地址字段; 只在初始化阶段调用一次.
    void describe(Descriptor& out, const char* build_id) noexcept;
    [[nodiscard]] Page read_page(std::uint16_t first_index) const noexcept;
    // 用临界区做一次一致复制; CriticalSection 需提供 enter()/exit(token).
    template <typename CriticalSection>
    void copy_snapshot(CriticalSection& lock, Page& out, std::uint16_t first_index) noexcept {
        const auto token = lock.enter();
        out = read_page(first_index);
        lock.exit(token);
    }
private:
    std::array<Record, record_capacity> records_{};   // 暂存视图 (写者)
    std::array<Record, record_capacity> published_{}; // 发布视图 (读者)
    std::size_t used_{};
    std::uint32_t used_count_{}; // 与 used_ 同步, 供 SWD 侧按定长字段读取
    std::uint32_t generation_{}; // 奇 = 发布中, 偶 = 可读 (见 end_update)
    Tick clock_{};
    bool updating_{};
};

// 线协议: 显式小端逐字段编码, 不复用结构体布局.
[[nodiscard]] std::size_t encode_page(const Page& page, std::span<std::uint8_t> out) noexcept;
[[nodiscard]] std::optional<Page> decode_page(std::span<const std::uint8_t> in) noexcept;
// 线协议里一条记录需要的字节数: 固定 8 字节头 + 8 字节记录头 + 时间/数值.
[[nodiscard]] constexpr std::size_t page_wire_size(std::size_t records) noexcept {
    return 24 + records * 24;
}
} // 命名空间 ec::monitor
