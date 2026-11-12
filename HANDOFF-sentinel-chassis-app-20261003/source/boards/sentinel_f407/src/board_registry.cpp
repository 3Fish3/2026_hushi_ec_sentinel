#include "board_registry.h"
#include "board_app_observation.h"
#include "board_assembly.hpp"
#include "board_io.h"
#include "board_inputs.hpp"
#include "ec/monitor/registry.hpp"
#include "ec/time.hpp"
#include "s1_diagnostics.h"
#include <bit>
#include <limits>
#if EC_SENTINEL_APP
#include "sentinel_types.hpp"
#endif

#if EC_MONITOR_REGISTRY
namespace {
// 与 board_io.cpp 相同的来源有效期; 记录里的"年龄"用本机毫秒时钟计算.
constexpr ec::Lifetime kRemoteTtl{100};
#if !EC_SENTINEL_APP && !EC_SENTINEL_CAPTURE
constexpr ec::Lifetime kImuTtl{50};
#endif
ec::monitor::Registry registry{};
ec::monitor::Descriptor descriptor{};
bool ready{};

constexpr float no_value = std::numeric_limits<float>::quiet_NaN();

void set(ec::monitor::RecordKind kind, std::uint16_t id, ec::monitor::Validity validity,
    ec::monitor::RejectReason reject, ec::Tick sampled_at, ec::Tick published_at, float target,
    float measurement, std::uint8_t detail = 0) noexcept {
    auto* record = registry.find(kind, id);
    if (record == nullptr) { return; }
    record->validity = validity;
    record->reject = reject;
    record->sampled_at = sampled_at;
    record->published_at = published_at;
    record->target = target;
    record->measurement = measurement;
    record->detail = detail;
}

void publish_period(ec::Tick now) noexcept {
    // period 记录: target = 配置周期, measurement = 观测到的最大周期耗时, detail = 跳过的周期数.
    set(ec::monitor::RecordKind::period, 0, ec::monitor::Validity::valid, ec::monitor::RejectReason::none,
        ec::Tick{g_ec_s1.hal_ms}, now, static_cast<float>(g_ec_s1.period_ms),
        static_cast<float>(g_ec_s1.max_elapsed_ms),
        static_cast<std::uint8_t>(g_ec_s1.skipped_periods > 255 ? 255 : g_ec_s1.skipped_periods));
}

void publish_remote(ec::Tick now) noexcept {
    // source 记录: enabled = 遥控在线, measurement = 最近一帧年龄, target = 无 (未接策略).
    const bool online = g_ec_io.remote.online != 0;
    const ec::Tick sampled{g_ec_io.remote.last_ms};
    const bool fresh = online && ec::fresh(now, sampled, kRemoteTtl);
    set(ec::monitor::RecordKind::source, 0, fresh ? ec::monitor::Validity::valid
            : (sampled.ms == 0 ? ec::monitor::Validity::unknown : ec::monitor::Validity::stale),
        fresh ? ec::monitor::RejectReason::none : ec::monitor::RejectReason::link_stale,
        sampled, now, no_value, static_cast<float>(now.ms - sampled.ms));
    if (auto* record = registry.find(ec::monitor::RecordKind::source, 0)) {
        record->enabled = online ? 1 : 0;
    }
}

void publish_imu(ec::Tick now) noexcept {
    // environment 0: 温度有自己的低频采样时间, 不能用新 gyro 给旧温度续期.
#if EC_SENTINEL_APP || EC_SENTINEL_CAPTURE
    const auto reading = ec::sentinel::input_imu_reading(now);
    if (!reading || !reading->value.times.temperature_valid) {
        set(ec::monitor::RecordKind::environment, 0, ec::monitor::Validity::unavailable,
            ec::monitor::RejectReason::stale_feedback, {}, now, no_value, no_value);
        return;
    }
    const auto age_us = ec_board_micros() - reading->value.times.temperature.us;
    const ec::Tick sampled{now.ms - age_us / 1000u};
    const bool fresh = age_us < 2560000u; // 两个 1.28 s 温度周期, 仅观测时效
    set(ec::monitor::RecordKind::environment, 0,
        fresh ? ec::monitor::Validity::valid : ec::monitor::Validity::stale,
        fresh ? ec::monitor::RejectReason::none : ec::monitor::RejectReason::stale_feedback,
        sampled, now, no_value, reading->value.sample.temp_c);
#else
    const ec::Tick sampled{g_ec_io.imu.last_ms};
    const bool ready_now = g_ec_io.imu.ready != 0;
    const bool fresh = ready_now && ec::fresh(now, sampled, kImuTtl);
    set(ec::monitor::RecordKind::environment, 0,
        fresh ? ec::monitor::Validity::valid
              : (ready_now ? ec::monitor::Validity::stale : ec::monitor::Validity::unavailable),
        fresh ? ec::monitor::RejectReason::none : ec::monitor::RejectReason::stale_feedback,
        sampled, now, no_value, g_ec_io.imu.temp_c);
#endif
}

#if !EC_SENTINEL_APP
void publish_devices(ec::Tick now) noexcept {
    for (std::size_t i = 0; i < ec::sentinel::dji_install_max; ++i) {
        const auto id = static_cast<std::uint16_t>(i);
        const auto* device = ec::sentinel::dji_device(i);
        if (device == nullptr) {
            // 未安装: 明确不可用, 不填健康零值.
            set(ec::monitor::RecordKind::device, id, ec::monitor::Validity::unavailable,
                ec::monitor::RejectReason::no_feedback, ec::Tick{}, now, no_value, no_value);
            continue;
        }
        const auto device_state = device->state(now);
        const bool fresh = device_state.rotor.fresh_at(now, device->config().feedback_ttl);
        set(ec::monitor::RecordKind::device, id,
            fresh ? ec::monitor::Validity::valid : ec::monitor::Validity::stale,
            fresh ? ec::monitor::RejectReason::none : ec::monitor::RejectReason::stale_feedback,
            device_state.rotor.sampled_at, now, no_value, device_state.velocity_rad_s.value);
    }
    for (std::size_t i = 0; i < ec::sentinel::dm_install_max; ++i) {
        const auto id = static_cast<std::uint16_t>(ec::sentinel::dji_install_max + i);
        const auto* device = ec::sentinel::dm_device(i);
        if (device == nullptr) {
            set(ec::monitor::RecordKind::device, id, ec::monitor::Validity::unavailable,
                ec::monitor::RejectReason::no_feedback, ec::Tick{}, now, no_value, no_value);
            continue;
        }
        const auto device_state = device->state();
        const bool online = device->online(now);
        const bool faulted = device->faulted(now);
        set(ec::monitor::RecordKind::device, id,
            online ? (faulted ? ec::monitor::Validity::invalid : ec::monitor::Validity::valid)
                   : ec::monitor::Validity::stale,
            faulted ? ec::monitor::RejectReason::controller_error
                    : (online ? ec::monitor::RejectReason::none : ec::monitor::RejectReason::stale_feedback),
            device_state.feedback.sampled_at, now, no_value, device_state.feedback.value.velocity_rad_s);
        if (auto* record = registry.find(ec::monitor::RecordKind::device, id)) {
            record->enabled = device->enabled(now) ? 1 : 0;
            record->detail = static_cast<std::uint8_t>(device_state.feedback.value.status);
        }
    }
}
#endif // !EC_SENTINEL_APP
#if EC_SENTINEL_APP
namespace app = ec::sentinel::app;
const EcAppObservation* app_observation{};

// 应用层枚举到监控词汇的显式映射: 不做数值强转, 也不在监控层另造一套语义.
// 完整应用原因码始终写入 detail, 读方可直接得到权威原因.
[[nodiscard]] constexpr ec::monitor::Validity map_validity(std::uint8_t code) noexcept {
    switch (static_cast<app::DomainValidity>(code)) {
    case app::DomainValidity::valid: return ec::monitor::Validity::valid;
    case app::DomainValidity::stale: return ec::monitor::Validity::stale;
    case app::DomainValidity::invalid: return ec::monitor::Validity::invalid;
    case app::DomainValidity::unavailable: return ec::monitor::Validity::unavailable;
    // 配置缺项: 不可用但也不是设备故障.
    case app::DomainValidity::incomplete: return ec::monitor::Validity::unavailable;
    case app::DomainValidity::unknown: break;
    }
    return ec::monitor::Validity::unknown;
}

[[nodiscard]] constexpr ec::monitor::RejectReason map_reject(std::uint8_t code) noexcept {
    switch (static_cast<app::Reason>(code)) {
    case app::Reason::none: return ec::monitor::RejectReason::none;
    case app::Reason::no_target: return ec::monitor::RejectReason::no_target;
    case app::Reason::no_authority: return ec::monitor::RejectReason::no_grant;
    case app::Reason::no_feedback: return ec::monitor::RejectReason::no_feedback;
    case app::Reason::stale_feedback: return ec::monitor::RejectReason::stale_feedback;
    case app::Reason::stale_target: return ec::monitor::RejectReason::target_stale;
    case app::Reason::not_configured:
    case app::Reason::incomplete_config: return ec::monitor::RejectReason::policy_missing;
    case app::Reason::controller_error: return ec::monitor::RejectReason::controller_error;
    case app::Reason::output_rejected: return ec::monitor::RejectReason::output_rejected;
    default: break;
    }
    return ec::monitor::RejectReason::unknown;
}

void publish_device_slot(const EcAppDeviceObservation& slot, std::uint16_t id,
    ec::Tick now) noexcept {
    if (slot.installed == 0) {
        set(ec::monitor::RecordKind::device, id, ec::monitor::Validity::unavailable,
            ec::monitor::RejectReason::no_feedback, ec::Tick{}, now, no_value, no_value);
        return;
    }
    set(ec::monitor::RecordKind::device, id,
        slot.online != 0 ? ec::monitor::Validity::valid : ec::monitor::Validity::stale,
        slot.online != 0 ? ec::monitor::RejectReason::none
                         : ec::monitor::RejectReason::stale_feedback,
        ec::Tick{slot.sampled_at_ms}, now, no_value, slot.velocity_rad_s,
        slot.temperature_valid != 0 ? static_cast<std::uint8_t>(slot.temperature_c) : 0);
    if (auto* record = registry.find(ec::monitor::RecordKind::device, id)) {
        record->enabled = slot.enabled;
    }
}

// 应用状态发布: 来源/权限 (source id 1)、每域执行结论 (axis id = 域序号) 与设备视图.
// 设备 ID 与板级静态装配层一致: DJI 0..dji_install_max-1, DM 从 dji_install_max 开始.
void publish_app(ec::Tick now) noexcept {
    const auto* obs = app_observation;
    if (obs == nullptr) { return; }
    // source id 1 的 detail 位: 0-1 控制来源, 2 人工停止, 3 本地就绪, 4 上位机会话,
    // 5 维护会话, 6 配置齐备, 7 允许启动. measurement = 配置缺项数.
    const auto detail = static_cast<std::uint8_t>(obs->source | (obs->manual_stop << 2) |
        (obs->locally_ready << 3) | (obs->host_session << 4) |
        (obs->maintenance_session << 5) | (obs->config_complete << 6) |
        (obs->config_startable << 7));
    set(ec::monitor::RecordKind::source, 1, ec::monitor::Validity::valid,
        obs->authority != 0 ? ec::monitor::RejectReason::none
                            : ec::monitor::RejectReason::no_grant,
        ec::Tick{obs->now_ms}, now, no_value, static_cast<float>(obs->config_missing_items), detail);
    if (auto* record = registry.find(ec::monitor::RecordKind::source, 1)) {
        record->enabled = obs->authority;
    }
    for (std::size_t i = 0; i < EC_APP_DOMAIN_COUNT; ++i) {
        const auto& domain = obs->domains[i];
        set(ec::monitor::RecordKind::axis, static_cast<std::uint16_t>(i),
            map_validity(domain.validity), map_reject(domain.reason),
            ec::Tick{domain.sampled_at_ms}, now, domain.target, domain.measurement, domain.reason);
        if (auto* record = registry.find(ec::monitor::RecordKind::axis,
                static_cast<std::uint16_t>(i))) {
            record->mode = domain.mode;
            record->enabled = domain.execute;
        }
    }
    for (std::size_t i = 0; i < EC_APP_DJI_SLOT_COUNT; ++i) {
        publish_device_slot(obs->dji[i], static_cast<std::uint16_t>(i), now);
    }
    for (std::size_t i = 0; i < EC_APP_DM_SLOT_COUNT; ++i) {
        publish_device_slot(obs->dm[i],
            static_cast<std::uint16_t>(EC_APP_DJI_SLOT_COUNT + i), now);
    }
}
#endif
} // 注册表内部

// 描述符必须保留在镜像里供 SWD 侧按符号名定位.
extern "C" {
__attribute__((used)) ec::monitor::Descriptor g_ec_monitor_desc{};
}

extern "C" void ec_registry_init(void) {
    (void)registry.claim(ec::monitor::RecordKind::period, 0);
    (void)registry.claim(ec::monitor::RecordKind::source, 0);
    (void)registry.claim(ec::monitor::RecordKind::environment, 0);
    for (std::size_t i = 0; i < ec::sentinel::dji_install_max; ++i) {
        (void)registry.claim(ec::monitor::RecordKind::device, static_cast<std::uint16_t>(i));
    }
    for (std::size_t i = 0; i < ec::sentinel::dm_install_max; ++i) {
        (void)registry.claim(ec::monitor::RecordKind::device,
            static_cast<std::uint16_t>(ec::sentinel::dji_install_max + i));
    }
#if EC_SENTINEL_APP
    // 正式 APP 额外发布应用来源/权限 (source id 1) 与每域执行结论 (axis id = 域序号).
    (void)registry.claim(ec::monitor::RecordKind::source, 1);
    for (std::size_t i = 0; i < EC_APP_DOMAIN_COUNT; ++i) {
        (void)registry.claim(ec::monitor::RecordKind::axis, static_cast<std::uint16_t>(i));
    }
#endif
    registry.describe(descriptor, g_ec_s1_build_id);
    g_ec_monitor_desc = descriptor;
    ready = true;
}

extern "C" void ec_registry_publish(uint32_t now_ms) {
    if (!ready) { return; }
    const ec::Tick now{now_ms};
    registry.begin_update();
    publish_period(now);
    publish_remote(now);
    publish_imu(now);
#if EC_SENTINEL_APP
    // 正式 APP 的设备属于应用执行层 (板级静态装配表在 APP 角色为空), 由 publish_app 发布.
    publish_app(now);
#else
    publish_devices(now);
#endif
    registry.end_update(now);
}

#if EC_SENTINEL_APP
extern "C" void ec_registry_publish_app(const struct EcAppObservation* observation) {
    app_observation = observation;
}
#endif

extern "C" size_t ec_registry_encode_page(uint16_t first_index, uint8_t* out, size_t capacity) {
    if (!ready || out == nullptr) { return 0; }
    const auto page = registry.read_page(first_index);
    return ec::monitor::encode_page(page, {out, capacity});
}
#else
extern "C" void ec_registry_init(void) {}
extern "C" void ec_registry_publish(uint32_t) {}
extern "C" size_t ec_registry_encode_page(uint16_t, uint8_t*, size_t) { return 0; }
#if EC_SENTINEL_APP
extern "C" void ec_registry_publish_app(const struct EcAppObservation*) {}
#endif
#endif
