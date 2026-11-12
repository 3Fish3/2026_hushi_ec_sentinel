#include "s1_diagnostics.h"
#include "board_io.h"
#include "board_capture.hpp"
#include "ec/platform/stm32f407.hpp"

extern "C" {
volatile EcS1Diagnostics g_ec_s1{};
extern const char g_ec_s1_build_id[] = EC_FIRMWARE_ID;
}

namespace {
// S1 诊断周期不是机器人控制周期. 只在启动上下文或 S1 任务写事件.
constexpr std::uint32_t period_ms = EC_S5_GM6020_BENCH ? 1 : 10;
static_assert(configTICK_RATE_HZ == 1000);
static_assert(sizeof(StackType_t) == 4);
void event(std::uint32_t stage, std::uint32_t value) noexcept {
    const auto n = g_ec_s1.event_count;
    auto& slot = g_ec_s1.events[n % 16];
    slot.sequence = n;
    slot.hal_ms = HAL_GetTick();
    slot.stage = stage;
    slot.value = value;
    g_ec_s1.event_count = n + 1;
}
} // 诊断内部实现

extern "C" void ec_s1_boot(void) {
    g_ec_s1.magic = 0x45435331u;
    g_ec_s1.abi = 1;
    g_ec_s1.build_id = g_ec_s1_build_id;
    g_ec_s1.reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    g_ec_s1.stage = EC_S1_BOOT;
    event(EC_S1_BOOT, g_ec_s1.reset_flags);
}

extern "C" void ec_s1_stage(EcS1Stage stage) {
    if (stage == EC_S1_CLOCK_READY) {
        g_ec_s1.hal_version = HAL_GetHalVersion();
        g_ec_s1.sysclk_hz = HAL_RCC_GetSysClockFreq();
        g_ec_s1.hclk_hz = HAL_RCC_GetHCLKFreq();
        g_ec_s1.pclk1_hz = HAL_RCC_GetPCLK1Freq();
        g_ec_s1.pclk2_hz = HAL_RCC_GetPCLK2Freq();
        if (g_ec_s1.sysclk_hz != 168000000 || g_ec_s1.hclk_hz != 168000000 ||
            g_ec_s1.pclk1_hz != 42000000 || g_ec_s1.pclk2_hz != 84000000) {
            ec_s1_fail(EC_S1_CLOCK_MISMATCH, __FILE__, __LINE__);
        }
    }
    g_ec_s1.stage = stage;
    event(stage, 0);
}

extern "C" void ec_s1_event(uint32_t kind, uint32_t value) { event(kind, value); }

extern "C" [[noreturn]] void ec_s1_task(void*) {
    g_ec_s1.rtos_hz = configTICK_RATE_HZ;
    g_ec_s1.period_ms = period_ms;
    ec_s1_stage(EC_S1_RUNNING);
    ec::stm32f407::PeriodicWait wait(pdMS_TO_TICKS(period_ms));
    ec::stm32f407::CycleTimer cycle_timer{};
#if EC_SENTINEL_CAPTURE
    // CAN 接收已使用 CYCCNT 打点, 此处不能重新清零时钟.
    const bool cycle_timing = ec::stm32f407::CycleTimer::available();
    ec_capture_start_task();
#else
    const bool cycle_timing = ec::stm32f407::CycleTimer::enable();
#endif
    std::uint32_t worst_cycle_us{};
    auto previous = HAL_GetTick();
    for (;;) {
        const auto skipped = wait.wait_next();
        if (!skipped) { ec_s1_fail(EC_S1_PERIOD_ERROR, __FILE__, __LINE__); }
        const auto now = HAL_GetTick();
        const auto elapsed = now - previous;
        previous = now;
        g_ec_s1.hal_ms = now;
        g_ec_s1.rtos_tick = xTaskGetTickCount();
        g_ec_s1.cycles = g_ec_s1.cycles + 1;
        g_ec_s1.last_elapsed_ms = elapsed;
        if (elapsed > g_ec_s1.max_elapsed_ms) { g_ec_s1.max_elapsed_ms = elapsed; }
        g_ec_s1.skipped_periods = g_ec_s1.skipped_periods + *skipped;
        cycle_timer.reset();
        ec_board_io_poll(now);
        if (cycle_timing) {
            const auto cost = cycle_timer.elapsed_us();
            if (cost > worst_cycle_us) { worst_cycle_us = cost; }
        }
        // 每秒扫描一次栈余量, 避免诊断本身占用每个控制周期.
        if (g_ec_s1.cycles % (1000 / period_ms) == 0) {
            g_ec_s1.stack_free_words = uxTaskGetStackHighWaterMark(nullptr);
            g_ec_s1.heap_free_bytes = xPortGetFreeHeapSize();
            g_ec_s1.heap_min_free_bytes = xPortGetMinimumEverFreeHeapSize();
            event(EC_S1_RUNNING, g_ec_s1.cycles);
            // 最差周期耗时也只在秒边界记录, 不占用每周期路径.
            if (cycle_timing) {
                event(EC_S1_CYCLE_COST, worst_cycle_us);
                worst_cycle_us = 0;
            }
        }
    }
}

extern "C" [[noreturn]] void ec_s1_fail(EcS1Fault fault, const char* file, uint32_t line) {
    __disable_irq();
    // 不依赖 RTOS、格式化输出或堆. 堆栈严重损坏时此 C 级快照仍可能无法执行.
    g_ec_s1.fault = fault;
    g_ec_s1.fault_file = file;
    g_ec_s1.fault_line = line;
    g_ec_s1.cfsr = SCB->CFSR;
    g_ec_s1.hfsr = SCB->HFSR;
    g_ec_s1.bfar = SCB->BFAR;
    g_ec_s1.mmfar = SCB->MMFAR;
    __DMB();
    g_ec_s1.stage = EC_S1_FATAL;
    for (;;) { __NOP(); }
}

extern "C" void vApplicationMallocFailedHook(void) {
    ec_s1_fail(EC_S1_ALLOC_FAILURE, __FILE__, __LINE__);
}
extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char*) {
    ec_s1_fail(EC_S1_STACK_OVERFLOW, __FILE__, __LINE__);
}
