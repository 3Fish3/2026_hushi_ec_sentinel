enable_language(ASM)
# 板级验证脚本 (check_firmware.py / check_link_verify.py) 是构建流程的一部分, 不是可选项;
# 缺解释器时配置直接失败, 而不是让验证步骤静默消失.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
option(EC_S1_UART6_MONITOR "Enable the USART6 board monitor" ON)
# 监控注册表 (设备/来源/周期记录的静态表). 关闭时不保留每周期发布开销.
option(EC_MONITOR_REGISTRY "Enable the static monitoring registry" ON)
option(EC_S5_GM6020_BENCH "Enable actuating single GM6020 bench (NOT an S1-S4 image)" OFF)
set(EC_S5_GM6020_ID "4" CACHE STRING "S5 CAN1 motor ID, verify against passive feedback before motion")
if(EC_S5_GM6020_BENCH AND NOT EC_S5_GM6020_ID MATCHES "^[1-7]$")
    message(FATAL_ERROR "EC_S5_GM6020_ID must be 1..7")
endif()
# 正式 APP 角色: 九 DJI + 一 DM 的完整应用路径与 1 kHz 控制周期. 与其它角色互斥,
# 必须使用独立构建目录 (同一目录里混用会被下面的 FATAL_ERROR 拒绝).
option(EC_BUILD_SENTINEL_APP "Build the Sentinel production APP image (9 DJI + 1 DM, 1 kHz)" OFF)
option(EC_APP_GM6020_BENCH "APP maintenance-only physical bench: one CAN1 GM6020 ID4" OFF)
option(EC_APP_FOUR_MOTOR_BENCH "APP maintenance-only four-motor CAN1 bench" OFF)
option(EC_APP_CHASSIS_COMMISSIONING "APP vehicle chassis commissioning with DT7 and maintenance" OFF)
option(EC_APP_GIMBAL_COMMISSIONING "APP vehicle gimbal mechanical commissioning" OFF)
option(EC_APP_GIMBAL_REMOTE_VERIFY "Allow DT7 validation with the same live gimbal tuning gains" OFF)
if(EC_APP_GIMBAL_REMOTE_VERIFY AND (NOT EC_BUILD_SENTINEL_APP OR NOT EC_APP_GIMBAL_COMMISSIONING))
    message(FATAL_ERROR "Gimbal remote verification requires the gimbal APP role")
endif()
option(EC_APP_SHOOTER_COMMISSIONING "APP vehicle friction and feeder commissioning" OFF)
option(EC_APP_PITCH_IDENTIFICATION "Enable explicitly bounded Pitch experiments (separate audited gimbal image)" OFF)
set(EC_APP_PITCH_POLICY_HEADER "" CACHE FILEPATH "Trusted header defining ec::sentinel::board::trusted_pitch_identification_policy; no physical defaults")
if(EC_APP_PITCH_IDENTIFICATION)
    if(NOT EC_BUILD_SENTINEL_APP OR NOT EC_APP_GIMBAL_COMMISSIONING)
        message(FATAL_ERROR "Pitch identification requires the gimbal APP deployment role")
    endif()
    if(NOT EXISTS "${EC_APP_PITCH_POLICY_HEADER}" OR IS_DIRECTORY "${EC_APP_PITCH_POLICY_HEADER}")
        message(FATAL_ERROR "Pitch identification needs an explicit trusted physical-bound policy header")
    endif()
    get_filename_component(EC_APP_PITCH_POLICY_HEADER "${EC_APP_PITCH_POLICY_HEADER}" REALPATH)
endif()
set(_app_profiles)
foreach(_profile EC_APP_GM6020_BENCH EC_APP_FOUR_MOTOR_BENCH EC_APP_CHASSIS_COMMISSIONING EC_APP_GIMBAL_COMMISSIONING EC_APP_SHOOTER_COMMISSIONING)
    if(${_profile})
        list(APPEND _app_profiles ${_profile})
    endif()
endforeach()
list(LENGTH _app_profiles _app_profile_count)
if(_app_profile_count GREATER 1)
    message(FATAL_ERROR "Select only one APP deployment profile")
endif()
if(_app_profile_count GREATER 0 AND NOT EC_BUILD_SENTINEL_APP)
    message(FATAL_ERROR "APP deployment profiles require EC_BUILD_SENTINEL_APP")
endif()
if(EC_BUILD_SENTINEL_CAPTURE AND EC_S5_GM6020_BENCH)
    message(FATAL_ERROR "Capture and S5 must use separate build directories")
endif()
if(EC_BUILD_SENTINEL_APP AND EC_S5_GM6020_BENCH)
    message(FATAL_ERROR "The production APP and the S5 bench must use separate build directories")
endif()
if(EC_BUILD_SENTINEL_APP AND EC_BUILD_SENTINEL_CAPTURE)
    message(FATAL_ERROR "The production APP and the capture role must use separate build directories")
endif()
if(EC_S5_GM6020_BENCH AND NOT EC_S1_UART6_MONITOR)
    message(FATAL_ERROR "S5 bench requires the USART6 operator interface")
endif()
set(_mx "${CMAKE_CURRENT_SOURCE_DIR}/cubemx/sentinel_s1")
set(_hal "${EC_STM32CUBE_F4_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src")
set(_rtos "${EC_STM32CUBE_F4_ROOT}/Middlewares/Third_Party/FreeRTOS/Source")
set(EC_FIRMWARE_ID "s1-development" CACHE STRING "Human-readable firmware label, not a substitute for the ELF hash")
if(NOT EC_FIRMWARE_ID MATCHES "^[A-Za-z0-9._-]+$")
    message(FATAL_ERROR "EC_FIRMWARE_ID must contain only letters, digits, dot, underscore or hyphen")
endif()

# HAL/RTOS 是独立厂商目标, 不对第三方代码施加手写代码的警告门禁.
add_library(ec_s1_vendor STATIC
    "${_hal}/stm32f4xx_hal.c"
    "${_hal}/stm32f4xx_hal_rcc.c" "${_hal}/stm32f4xx_hal_rcc_ex.c"
    "${_hal}/stm32f4xx_hal_gpio.c" "${_hal}/stm32f4xx_hal_cortex.c"
    "${_hal}/stm32f4xx_hal_dma.c" "${_hal}/stm32f4xx_hal_dma_ex.c"
    "${_hal}/stm32f4xx_hal_flash.c" "${_hal}/stm32f4xx_hal_flash_ex.c"
    "${_hal}/stm32f4xx_hal_flash_ramfunc.c"
    "${_hal}/stm32f4xx_hal_pwr.c" "${_hal}/stm32f4xx_hal_pwr_ex.c"
    "${_hal}/stm32f4xx_hal_exti.c"
    "${_hal}/stm32f4xx_hal_tim.c" "${_hal}/stm32f4xx_hal_tim_ex.c"
    "${_hal}/stm32f4xx_hal_uart.c" "${_hal}/stm32f4xx_hal_can.c"
    "${_hal}/stm32f4xx_hal_spi.c"
    "${_rtos}/tasks.c" "${_rtos}/queue.c" "${_rtos}/list.c"
    "${_rtos}/timers.c" "${_rtos}/event_groups.c" "${_rtos}/stream_buffer.c"
    "${_rtos}/portable/GCC/ARM_CM4F/port.c"
    "${_rtos}/portable/MemMang/heap_4.c"
    "${_rtos}/CMSIS_RTOS_V2/cmsis_os2.c")
target_link_libraries(ec_s1_vendor PUBLIC ec::stm32f407)
target_include_directories(ec_s1_vendor SYSTEM PUBLIC "${_rtos}/CMSIS_RTOS_V2")
# FreeRTOSConfig.h 的 USER CODE 区使用板级诊断钩子, 所有包含者使用同一声明.
target_include_directories(ec_stm32f407 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# 非部署链接验证产物: 用极小的假 sink 引用各通用能力的目标代码, 证明它们在
# --gc-sections 与真实链接脚本下确实被链接 (而不是"编进静态库就算通过").
# 它不驱动任何外设、不写发送队列、不含 APP 钩子实现, 因此不是可烧录镜像;
# 默认固件 sentinel_s1 的无输出审计规则不因它放宽.
add_executable(ec_link_verify
    "${_mx}/startup_stm32f407xx.s"
    "${_mx}/Core/Src/system_stm32f4xx.c"
    src/board_assembly.cpp
    link_verify/main.cpp)
set_target_properties(ec_link_verify PROPERTIES SUFFIX ".elf")
target_compile_definitions(ec_link_verify PRIVATE EC_FIRMWARE_ID="link-verify"
    EC_MONITOR_REGISTRY=$<BOOL:${EC_MONITOR_REGISTRY}> EC_S1_UART6_MONITOR=OFF EC_S5_GM6020_BENCH=OFF)
target_link_libraries(ec_link_verify PRIVATE ec_s1_vendor)
target_link_options(ec_link_verify PRIVATE
    "-T${_mx}/STM32F407xx_FLASH.ld"
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/ec_link_verify.map")
set_property(TARGET ec_link_verify APPEND PROPERTY LINK_DEPENDS "${_mx}/STM32F407xx_FLASH.ld")
# 链接验证 TU 必须保持"按名字可达": Release 的 -O2 会把 inline 组件内联掉, 使符号在
# --gc-sections 后消失, 这属于正常内联而不是"能力未覆盖". 因此只对这个非部署产物禁用内联
# 并保留 inline 函数的独立副本; 库本身仍按该配置正常优化.
set_source_files_properties(link_verify/main.cpp PROPERTIES COMPILE_OPTIONS
    "-Wall;-Wextra;-Wpedantic;-Werror;-fno-inline;-fkeep-inline-functions")
# 符号证据: 目标架构上必须存在这些真实组件的符号, 否则说明它们被 GC 掉了.
# find_program 必须在任何 ${EC_NM} 展开**之前**执行: add_custom_command 在配置阶段就展开变量,
# 干净构建目录里后面才找到工具不会追溯修改已生成的命令.
find_program(EC_NM arm-none-eabi-nm HINTS "${ARM_GNU_ROOT}/bin" REQUIRED)
add_custom_command(TARGET ec_link_verify POST_BUILD
    COMMAND "${EC_NM}" -C "$<TARGET_FILE:ec_link_verify>" > "${CMAKE_CURRENT_BINARY_DIR}/ec_link_verify.syms"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_link_verify.py"
        "${CMAKE_CURRENT_BINARY_DIR}/ec_link_verify.syms"
    VERBATIM)

set(_board_sources
    "${_mx}/startup_stm32f407xx.s"
    "${_mx}/Core/Src/main.c" "${_mx}/Core/Src/gpio.c"
    "${_mx}/Core/Src/freertos.c" "${_mx}/Core/Src/system_stm32f4xx.c"
    "${_mx}/Core/Src/stm32f4xx_hal_msp.c"
    "${_mx}/Core/Src/stm32f4xx_hal_timebase_tim.c"
    "${_mx}/Core/Src/stm32f4xx_it.c"
    "${_mx}/Core/Src/dma.c" "${_mx}/Core/Src/usart.c"
    "${_mx}/Core/Src/can.c" "${_mx}/Core/Src/spi.c"
    src/s1_diagnostics.cpp src/board_io.cpp src/board_monitor.cpp src/board_bench.cpp
    src/board_assembly.cpp src/board_registry.cpp src/board_inputs.cpp)
add_executable(sentinel_s1 ${_board_sources})
set_target_properties(sentinel_s1 PROPERTIES SUFFIX ".elf")
target_compile_definitions(sentinel_s1 PRIVATE EC_FIRMWARE_ID="${EC_FIRMWARE_ID}"
    EC_S1_UART6_MONITOR=$<BOOL:${EC_S1_UART6_MONITOR}>
    EC_MONITOR_REGISTRY=$<BOOL:${EC_MONITOR_REGISTRY}>
    EC_S5_GM6020_BENCH=$<BOOL:${EC_S5_GM6020_BENCH}> EC_S5_GM6020_ID=${EC_S5_GM6020_ID})
target_link_libraries(sentinel_s1 PRIVATE ec_s1_vendor)
# S1 不使用 libc 堆或主机 I/O, 不依赖 CubeMX 的 sysmem/syscalls 模板.
# FreeRTOS 使用其独立有界 heap_4, 仅在调度前分配任务对象.
target_link_options(sentinel_s1 PRIVATE
    "-T${_mx}/STM32F407xx_FLASH.ld"
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/sentinel_s1.map")
set_property(TARGET sentinel_s1 APPEND PROPERTY LINK_DEPENDS "${_mx}/STM32F407xx_FLASH.ld")
set_source_files_properties(src/s1_diagnostics.cpp src/board_io.cpp src/board_monitor.cpp src/board_bench.cpp src/board_assembly.cpp src/board_registry.cpp src/board_inputs.cpp PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic;-Werror")
find_program(EC_OBJCOPY arm-none-eabi-objcopy HINTS "${ARM_GNU_ROOT}/bin" REQUIRED)
find_program(EC_SIZE arm-none-eabi-size HINTS "${ARM_GNU_ROOT}/bin" REQUIRED)
add_custom_command(TARGET sentinel_s1 POST_BUILD
    COMMAND "${EC_OBJCOPY}" -O ihex "$<TARGET_FILE:sentinel_s1>" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_s1.hex"
    COMMAND "${EC_OBJCOPY}" -O binary "$<TARGET_FILE:sentinel_s1>" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_s1.bin"
    COMMAND "${EC_SIZE}" "$<TARGET_FILE:sentinel_s1>"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/sentinel_s1.hex" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_s1.bin"
    VERBATIM)

if(EC_BUILD_SENTINEL_CAPTURE)
    # 独立角色: DJI 纯接收, DM 仅固定参数/状态读取. 不改变默认 S1/S5 的符号审计.
    add_executable(sentinel_capture ${_board_sources} src/board_capture.cpp src/board_raw.cpp)
    set_target_properties(sentinel_capture PROPERTIES SUFFIX ".elf")
    target_compile_definitions(sentinel_capture PRIVATE EC_FIRMWARE_ID="capture-${EC_FIRMWARE_ID}"
        EC_SENTINEL_CAPTURE=1 EC_S1_UART6_MONITOR=1 EC_MONITOR_REGISTRY=0 EC_S5_GM6020_BENCH=0)
    target_link_libraries(sentinel_capture PRIVATE ec_s1_vendor ec_sentinel_capture)
    set_source_files_properties(src/board_capture.cpp PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic;-Werror")
    target_link_options(sentinel_capture PRIVATE "-T${_mx}/STM32F407xx_FLASH.ld"
        --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/sentinel_capture.map")
    set_property(TARGET sentinel_capture APPEND PROPERTY LINK_DEPENDS "${_mx}/STM32F407xx_FLASH.ld")
    add_custom_command(TARGET sentinel_capture POST_BUILD
        COMMAND "${EC_OBJCOPY}" -O ihex "$<TARGET_FILE:sentinel_capture>" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_capture.hex"
        COMMAND "${EC_OBJCOPY}" -O binary "$<TARGET_FILE:sentinel_capture>" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_capture.bin"
        COMMAND "${EC_SIZE}" "$<TARGET_FILE:sentinel_capture>"
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_capture.py"
            --elf "$<TARGET_FILE:sentinel_capture>" --nm "${EC_NM}"
        BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/sentinel_capture.hex" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_capture.bin"
        VERBATIM)
endif()

if(EC_BUILD_SENTINEL_APP)
    # 正式 APP 角色: 完整应用路径 (配置/仲裁/感知/执行/协议) + 1 kHz 控制任务.
    # 不是"只编静态库": 任务、钩子、协议、真实 CAN 发送组与设备控制路径全部链接进镜像.
    if(NOT EC_S1_UART6_MONITOR)
        message(FATAL_ERROR "The production APP keeps the USART6 observation service enabled")
    endif()
    add_executable(sentinel_app ${_board_sources} src/board_app.cpp src/board_raw.cpp)
    set_target_properties(sentinel_app PROPERTIES SUFFIX ".elf")
    set(_app_id "app-${EC_FIRMWARE_ID}")
    set(_app_audit_args)
    if(EC_APP_GIMBAL_COMMISSIONING)
        set(_app_id "app-gimbal-${EC_FIRMWARE_ID}")
        list(APPEND _app_audit_args --gimbal-commissioning)
    elseif(EC_APP_SHOOTER_COMMISSIONING)
        set(_app_id "app-shooter-${EC_FIRMWARE_ID}")
        list(APPEND _app_audit_args --shooter-commissioning)
    elseif(EC_APP_CHASSIS_COMMISSIONING)
        set(_app_id "app-chassis-${EC_FIRMWARE_ID}")
        list(APPEND _app_audit_args --chassis-commissioning)
    elseif(EC_APP_FOUR_MOTOR_BENCH)
        set(_app_id "app-four-motor-${EC_FIRMWARE_ID}")
        list(APPEND _app_audit_args --four-motor-bench)
    elseif(EC_APP_GM6020_BENCH)
        set(_app_id "app-gm6020-id4-${EC_FIRMWARE_ID}")
        list(APPEND _app_audit_args --gm6020-bench)
    endif()
    if(EC_APP_PITCH_IDENTIFICATION)
        list(APPEND _app_audit_args --pitch-identification)
        target_compile_definitions(sentinel_app PRIVATE EC_APP_PITCH_POLICY_HEADER="${EC_APP_PITCH_POLICY_HEADER}")
    endif()
    target_compile_definitions(sentinel_app PRIVATE EC_FIRMWARE_ID="${_app_id}"
        EC_APP_GIMBAL_REMOTE_VERIFY=$<BOOL:${EC_APP_GIMBAL_REMOTE_VERIFY}>
        EC_APP_PITCH_IDENTIFICATION=$<BOOL:${EC_APP_PITCH_IDENTIFICATION}>
        EC_SENTINEL_APP=1 EC_SENTINEL_CAPTURE=0 EC_S5_GM6020_BENCH=0
        EC_APP_GM6020_BENCH=$<BOOL:${EC_APP_GM6020_BENCH}>
        EC_APP_FOUR_MOTOR_BENCH=$<BOOL:${EC_APP_FOUR_MOTOR_BENCH}>
        EC_APP_CHASSIS_COMMISSIONING=$<BOOL:${EC_APP_CHASSIS_COMMISSIONING}>
        EC_APP_GIMBAL_COMMISSIONING=$<BOOL:${EC_APP_GIMBAL_COMMISSIONING}>
        EC_APP_SHOOTER_COMMISSIONING=$<BOOL:${EC_APP_SHOOTER_COMMISSIONING}>
        EC_S1_UART6_MONITOR=$<BOOL:${EC_S1_UART6_MONITOR}>
        EC_MONITOR_REGISTRY=$<BOOL:${EC_MONITOR_REGISTRY}>)
    target_link_libraries(sentinel_app PRIVATE ec_s1_vendor ec_sentinel_app)
    set_source_files_properties(src/board_app.cpp PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic;-Werror")
    target_link_options(sentinel_app PRIVATE "-T${_mx}/STM32F407xx_FLASH.ld"
        --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/sentinel_app.map")
    set_property(TARGET sentinel_app APPEND PROPERTY LINK_DEPENDS "${_mx}/STM32F407xx_FLASH.ld")
    add_custom_command(TARGET sentinel_app POST_BUILD
        COMMAND "${EC_OBJCOPY}" -O ihex "$<TARGET_FILE:sentinel_app>" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_app.hex"
        COMMAND "${EC_OBJCOPY}" -O binary "$<TARGET_FILE:sentinel_app>" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_app.bin"
        COMMAND "${EC_SIZE}" "$<TARGET_FILE:sentinel_app>"
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_sentinel_app.py"
            --elf "$<TARGET_FILE:sentinel_app>" --nm "${EC_NM}" ${_app_audit_args}
        BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/sentinel_app.hex" "${CMAKE_CURRENT_BINARY_DIR}/sentinel_app.bin"
        VERBATIM)
endif()
