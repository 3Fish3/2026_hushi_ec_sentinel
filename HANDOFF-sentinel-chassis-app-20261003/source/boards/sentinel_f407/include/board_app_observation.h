#ifndef EC_BOARD_APP_OBSERVATION_H
#define EC_BOARD_APP_OBSERVATION_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// 正式 APP 的应用状态观测视图 (O6). 固定布局、无指针; 由 1 kHz 控制任务写, 注册表发布
// 与串口/SWD 观测读同一份已提交视图, 因此这里只描述"应用观察到了什么", 不代表实机结论.
// 数值字段的语义由 board_app.hpp 的映射注释给出, 不在本文件重复定义枚举.
// 域数与应用层 ec::sentinel::app::domain_count 一致: 四轮 + 主/从 yaw + pitch + 两摩擦 + 拨弹.
#define EC_APP_DOMAIN_COUNT 10
#define EC_APP_DJI_SLOT_COUNT 9
#define EC_APP_DM_SLOT_COUNT 1
struct EcAppDomainObservation {
    uint8_t validity;   // app::DomainValidity
    uint8_t mode;       // control::ControlMode (与执行状态分开)
    uint8_t execute;    // 应用层执行结论 (0/1)
    uint8_t reason;     // app::Reason
    uint8_t fault;      // app::FaultState
    uint8_t reserved[3];
    uint32_t sampled_at_ms; // 该域反馈样本时刻; 无样本时为 0
    float target;       // 本周期采用的目标; 无目标时保持 0 并由 reason 说明
    float measurement;  // 本周期测量 (轴侧 SI); 不可用时保持 0 并由 validity 说明
};
struct EcAppDeviceObservation {
    uint8_t installed;
    uint8_t online;      // 反馈在 TTL 内
    uint8_t enabled;     // 使能已确认 (DM); DJI 无使能确认语义时保持 0
    uint8_t faulted;
    uint8_t temperature_valid;
    int8_t temperature_c;
    uint8_t bus;
    uint8_t reserved;
    uint32_t sampled_at_ms;
    float velocity_rad_s; // 轴侧角速度 (SI)
};
struct EcAppObservation {
    uint32_t now_ms;
    uint8_t source;      // app::Source
    uint8_t authority;   // 是否持有执行授权
    uint8_t manual_stop; // 人工整车停止挡
    uint8_t locally_ready;
    uint8_t host_session;
    uint8_t maintenance_session;
    uint8_t config_complete;
    uint8_t config_startable;
    uint32_t config_epoch, reference_epoch, config_missing_items;
    uint32_t accepted_packets, rejected_packets, link_losses, link_dropped;
    uint32_t frame_rejects, replay_rejects;
    uint32_t group_sent, group_refused, group_tx_error, dm_tx_error;
    struct EcAppDomainObservation domains[EC_APP_DOMAIN_COUNT];
    // 设备视图与板级静态装配层使用同一套监控 ID: DJI 0..8, DM 从 9 开始.
    struct EcAppDeviceObservation dji[EC_APP_DJI_SLOT_COUNT];
    struct EcAppDeviceObservation dm[EC_APP_DM_SLOT_COUNT];
    // UART v3 快状态的相同载荷; 已提交的固定存储快照, 不另算一份近似反馈.
    uint16_t fast_size;
    uint8_t fast_payload[240];
};
#ifdef __cplusplus
}
#endif
#endif
