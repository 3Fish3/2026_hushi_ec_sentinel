#include "board_assembly.hpp"
#include "ec/drivers/bus_layout.hpp"
#include <span>

// 安装表: APP 阶段按实车机械/电调配置填写这里 (总线, 协议模式, 设备 ID, 传动比,
// 方向, 反馈有效期, 已知速度上界, 输出编码上限). 默认两张表都为空, 表示
// "没有装配任何电机", 不是"装配了但不知道参数".
//
// 表一旦填写, assembly_init() 会做全总线布局检查: 重复槽位、同总线反馈 ID 冲突、
// DJI 与 DM 接收 ID 归属冲突都会让状态变成对应错误且不安装任何设备.
namespace ec::sentinel {
namespace {
constexpr std::size_t kDjiMotorCount = 0;
constexpr std::array<ec::drivers::DjiDeviceConfig, kDjiMotorCount> kDjiMotors{};
constexpr std::size_t kDmMotorCount = 0;
constexpr std::array<ec::drivers::DmDeviceConfig, kDmMotorCount> kDmMotors{};

enum class RouteKind : std::uint8_t { dji, dm };
struct Route {
    std::uint8_t bus{};
    std::uint16_t feedback_id{};
    RouteKind kind{RouteKind::dji};
    std::size_t index{};
};

std::array<std::optional<ec::drivers::DjiDevice>, dji_install_max> dji_devices{};
std::array<std::optional<ec::drivers::DmDevice>, dm_install_max> dm_devices{};
std::array<Route, dji_install_max + dm_install_max> routes{};
AssemblyState state{};

void fail(AssemblyStatus status) noexcept {
    // 保留已读到的条目数, 只重置装配结果: 错误状态也要能告诉调用方看了几张表.
    const auto dji_count = state.dji_count;
    const auto dm_count = state.dm_count;
    state = {};
    state.dji_count = dji_count;
    state.dm_count = dm_count;
    state.status = status;
}
} // 安装表与路由内部

const AssemblyState& assembly_state() noexcept { return state; }

void assembly_init() noexcept {
    assembly_init({kDjiMotors.data(), kDjiMotorCount}, {kDmMotors.data(), kDmMotorCount});
}

void assembly_init(std::span<const ec::drivers::DjiDeviceConfig> dji,
    std::span<const ec::drivers::DmDeviceConfig> dm) noexcept {
    for (auto& device : dji_devices) { device.reset(); }
    for (auto& device : dm_devices) { device.reset(); }
    state = {};
    state.dji_count = dji.size();
    state.dm_count = dm.size();
    if (dji.empty() && dm.empty()) {
        state.status = AssemblyStatus::empty;
        return;
    }
    if (dji.size() > dji_install_max || dm.size() > dm_install_max) {
        fail(AssemblyStatus::too_many);
        return;
    }
    if (!dji.empty() && !ec::drivers::valid_dji_layout(dji)) {
        fail(AssemblyStatus::invalid_dji);
        return;
    }
    if (!dm.empty() && !ec::drivers::valid_dm_layout(dm)) {
        fail(AssemblyStatus::invalid_dm);
        return;
    }
    if (!ec::drivers::valid_mixed_bus(dm, dji)) {
        fail(AssemblyStatus::id_conflict);
        return;
    }
    for (std::size_t i = 0; i < dji.size(); ++i) {
        if (!dji[i].bus || !dji[i].feedback_ttl.valid()) {
            fail(AssemblyStatus::invalid_dji);
            return;
        }
        const auto protocol = ec::drivers::dji_protocol(dji[i].mode);
        if (!protocol) {
            fail(AssemblyStatus::invalid_dji);
            return;
        }
        dji_devices[i].emplace(dji[i]);
        routes[state.routes++] = Route{dji[i].bus,
            static_cast<std::uint16_t>(protocol->rx_base + dji[i].device_id), RouteKind::dji, i};
    }
    for (std::size_t i = 0; i < dm.size(); ++i) {
        if (!dm[i].bus || !dm[i].feedback_ttl.valid()) {
            fail(AssemblyStatus::invalid_dm);
            return;
        }
        dm_devices[i].emplace(dm[i]);
        routes[state.routes++] = Route{dm[i].bus, dm[i].master_id, RouteKind::dm, i};
    }
    state.status = AssemblyStatus::ready;
}

ec::drivers::DjiDevice* dji_device(std::size_t index) noexcept {
    if (index >= dji_install_max || !dji_devices[index]) { return nullptr; }
    return &*dji_devices[index];
}

ec::drivers::DmDevice* dm_device(std::size_t index) noexcept {
    if (index >= dm_install_max || !dm_devices[index]) { return nullptr; }
    return &*dm_devices[index];
}

void assembly_route(std::uint8_t bus, const ec::transport::CanFrame& frame, ec::Tick at) noexcept {
    bool hit = false;
    // 同一 Master ID 上可以挂多个 DM 设备 (按反馈载荷的节点字段区分), 因此遍历全部匹配项,
    // 不因第一个匹配就返回; DJI 侧同一 (总线, 反馈 ID) 只允许一个设备.
    for (std::size_t i = 0; i < state.routes; ++i) {
        const auto& route = routes[i];
        if (route.bus != bus || route.feedback_id != frame.id) { continue; }
        hit = true;
        if (route.kind == RouteKind::dji) {
            (void)dji_devices[route.index]->receive(bus, frame, at);
        } else {
            (void)dm_devices[route.index]->receive(bus, frame, at);
        }
    }
    if (hit) {
        ++state.routed;
    } else {
        ++state.unrouted;
    }
}

void assembly_step(std::uint32_t now_ms) noexcept {
    ec_board_app_control(now_ms);
    ec_board_app_commit(now_ms);
}
} // 命名空间 ec::sentinel

// 默认钩子: 不产生任何执行器输出, 也不触碰 CAN 发送路径.
extern "C" EC_BOARD_WEAK void ec_board_app_control(std::uint32_t now_ms) { (void)now_ms; }
extern "C" EC_BOARD_WEAK void ec_board_app_commit(std::uint32_t now_ms) { (void)now_ms; }
