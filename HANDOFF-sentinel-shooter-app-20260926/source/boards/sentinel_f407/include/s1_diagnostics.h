#ifndef EC_S1_DIAGNOSTICS_H
#define EC_S1_DIAGNOSTICS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#define EC_S1_NORETURN [[noreturn]]
#else
#define EC_S1_NORETURN _Noreturn
#endif

enum EcS1Stage {
    EC_S1_BOOT = 1, EC_S1_HAL_READY = 2, EC_S1_CLOCK_READY = 3,
    EC_S1_GPIO_READY = 4, EC_S1_TASKS_READY = 5, EC_S1_RUNNING = 6, EC_S1_FATAL = 255
};
// 事件记录里的 stage 同时充当事件类型: 7 表示一秒窗口内最差的一次控制周期
// 执行耗时 (微秒, 由 DWT 短窗口计时得到; 不是长期时钟也不是跨机同步时间).
enum EcS1EventKind { EC_S1_CYCLE_COST = 7, EC_S1_APP_CONFIG = 8 };
enum EcS1Fault {
    EC_S1_HAL_ERROR = 1, EC_S1_CLOCK_MISMATCH = 2, EC_S1_TASK_CREATE = 3,
    EC_S1_SCHEDULER_RETURN = 4, EC_S1_ASSERT = 5, EC_S1_STACK_OVERFLOW = 6,
    EC_S1_ALLOC_FAILURE = 7, EC_S1_HARDFAULT = 8, EC_S1_MEMFAULT = 9,
    EC_S1_BUSFAULT = 10, EC_S1_USAGEFAULT = 11, EC_S1_PERIOD_ERROR = 12,
    EC_S1_NMI = 13
};
struct EcS1Event { uint32_t sequence, hal_ms, stage, value; };
struct EcS1Diagnostics {
    uint32_t magic, abi, stage, reset_flags, hal_version;
    uint32_t sysclk_hz, hclk_hz, pclk1_hz, pclk2_hz, rtos_hz;
    uint32_t hal_ms, rtos_tick, cycles, period_ms, last_elapsed_ms, max_elapsed_ms;
    uint32_t skipped_periods, stack_free_words, heap_free_bytes, heap_min_free_bytes;
    uint32_t fault, fault_line, cfsr, hfsr, bfar, mmfar;
    const char* fault_file;
    const char* build_id;
    uint32_t event_count;
    struct EcS1Event events[16];
};
// 固定 RAM 诊断区, 由调试器停核读取; volatile 不提供运行中多字段一致性.
// 不是持久故障存储, 复位/掉电后不保留本次事件.
extern volatile struct EcS1Diagnostics g_ec_s1;
extern const char g_ec_s1_build_id[];
void ec_s1_boot(void);
void ec_s1_stage(enum EcS1Stage stage);
// 记录一条固定格式事件 (环形缓冲, 最多 16 条): 用于启动/配置结论等不改变当前阶段的记录.
void ec_s1_event(uint32_t kind, uint32_t value);
EC_S1_NORETURN void ec_s1_task(void* argument);
EC_S1_NORETURN void ec_s1_fail(enum EcS1Fault fault, const char* file, uint32_t line);
#ifdef __cplusplus
}
#endif
#undef EC_S1_NORETURN
#endif
