cmake_minimum_required(VERSION 3.22)
project(ec_core VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(EC_BUILD_STM32F407 "Compile HAL/FreeRTOS adapters (external SDK required)" OFF)
option(EC_BUILD_SENTINEL_S1 "Build the non-actuating S1-S4 Sentinel board firmware" OFF)
option(EC_BUILD_SENTINEL_CAPTURE "Build the Sentinel acquisition image (DM parameter/status reads only)" OFF)
option(EC_BUILD_SENTINEL_APP "Build the Sentinel production APP image (9 DJI + 1 DM, 1 kHz, business protocol)" OFF)
if(EC_BUILD_SENTINEL_S1 OR EC_BUILD_SENTINEL_CAPTURE OR EC_BUILD_SENTINEL_APP)
    include(boards/sentinel_f407/sdk.cmake)
    set(EC_BUILD_STM32F407 ON)
endif()
if(CMAKE_CROSSCOMPILING)
    set(_host_default OFF)
else()
    set(_host_default ON)
endif()
option(EC_BUILD_TESTS "Build host contract tests" ${_host_default})
option(EC_BUILD_EXAMPLES "Build host assembly example" ${_host_default})

add_library(ec_core STATIC src/control/pid.cpp src/control/velocity_loop.cpp src/control/position_loop.cpp
    src/control/thermal.cpp src/control/authority.cpp src/drivers/bmi088_device.cpp src/drivers/dji.cpp
    src/drivers/dji_device.cpp src/drivers/dm_device.cpp src/estimation/attitude.cpp src/motion/dji_axis.cpp
    src/motion/omni_kinematics.cpp src/monitor/registry.cpp src/session.cpp)
add_library(ec::core ALIAS ec_core)
target_include_directories(ec_core PUBLIC include)
target_compile_features(ec_core PUBLIC cxx_std_20)
# MSVC 必须按 UTF-8 解析中文注释, 避免系统代码页误吞后续行.
target_compile_options(ec_core PUBLIC $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ec_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti>)
    target_compile_options(ec_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_library(ec_sentinel_recording STATIC robots/sentinel/recording/raw_record.cpp)
target_include_directories(ec_sentinel_recording PUBLIC robots/sentinel/recording)
target_link_libraries(ec_sentinel_recording PUBLIC ec::core)
target_compile_options(ec_sentinel_recording PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)

if(EC_BUILD_TESTS OR EC_BUILD_SENTINEL_CAPTURE)
    add_library(ec_sentinel_capture STATIC robots/sentinel/capture/capture.cpp)
    target_include_directories(ec_sentinel_capture PUBLIC robots/sentinel/capture)
    target_link_libraries(ec_sentinel_capture PUBLIC ec::core ec_sentinel_recording)
    target_compile_options(ec_sentinel_capture PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
endif()

add_library(ec_sentinel_app STATIC robots/sentinel/app/sentinel_config.cpp
    robots/sentinel/app/sentinel_app.cpp robots/sentinel/app/sentinel_authority.cpp
    robots/sentinel/app/sentinel_faults.cpp robots/sentinel/app/sentinel_execution.cpp
    robots/sentinel/app/sentinel_perception.cpp robots/sentinel/app/sentinel_remote.cpp
    robots/sentinel/app/sentinel_params.cpp robots/sentinel/app/sentinel_protocol.cpp
    robots/sentinel/app/sentinel_telemetry.cpp
    robots/sentinel/app/sentinel_param_apply.cpp robots/sentinel/app/sentinel_cycle.cpp
    robots/sentinel/app/sentinel_kinematics.cpp)
target_include_directories(ec_sentinel_app PUBLIC robots/sentinel/app)
target_link_libraries(ec_sentinel_app PUBLIC ec::core ec_sentinel_recording)
target_compile_options(ec_sentinel_app PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)

if(EC_BUILD_STM32F407)
    add_subdirectory(platform/stm32f407)
endif()
if(EC_BUILD_SENTINEL_S1 OR EC_BUILD_SENTINEL_CAPTURE OR EC_BUILD_SENTINEL_APP)
    add_subdirectory(boards/sentinel_f407)
endif()
if(EC_BUILD_TESTS OR EC_BUILD_EXAMPLES)
    if(CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR "Host tests/examples must be disabled in a cross build")
    endif()
    find_package(Threads REQUIRED)
endif()
if(EC_BUILD_TESTS)
    enable_testing()
    add_executable(ec_capture_tests tests/capture_tests.cpp)
    target_link_libraries(ec_capture_tests PRIVATE ec_sentinel_capture)
    target_compile_options(ec_capture_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_capture_contracts COMMAND ec_capture_tests)
    set_tests_properties(sentinel_capture_contracts PROPERTIES TIMEOUT 30)
    add_executable(ec_core_tests tests/core_tests.cpp)
    target_link_libraries(ec_core_tests PRIVATE ec::core Threads::Threads)
    target_include_directories(ec_core_tests PRIVATE boards/sentinel_f407/include)
    target_compile_options(ec_core_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME core_contracts COMMAND ec_core_tests)
    set_tests_properties(core_contracts PROPERTIES TIMEOUT 30)
    add_executable(ec_control_tests tests/control_tests.cpp)
    target_link_libraries(ec_control_tests PRIVATE ec::core)
    target_compile_options(ec_control_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME control_contracts COMMAND ec_control_tests)
    set_tests_properties(control_contracts PROPERTIES TIMEOUT 30)
    add_executable(ec_axis_tests tests/axis_tests.cpp)
    target_link_libraries(ec_axis_tests PRIVATE ec::core)
    target_compile_options(ec_axis_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME axis_contracts COMMAND ec_axis_tests)
    set_tests_properties(axis_contracts PROPERTIES TIMEOUT 30)
    add_executable(ec_dm_tests tests/dm_tests.cpp)
    target_link_libraries(ec_dm_tests PRIVATE ec::core)
    target_compile_options(ec_dm_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME dm_contracts COMMAND ec_dm_tests)
    set_tests_properties(dm_contracts PROPERTIES TIMEOUT 30)
    add_executable(ec_session_tests tests/session_tests.cpp)
    target_link_libraries(ec_session_tests PRIVATE ec::core)
    target_compile_options(ec_session_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME session_authority_contracts COMMAND ec_session_tests)
    set_tests_properties(session_authority_contracts PROPERTIES TIMEOUT 30)
    # 哨兵正式 APP 配置边界 (O0): 与板级无关, 直接在主机构建.
    add_executable(ec_sentinel_app_tests tests/sentinel_app_tests.cpp)
    target_link_libraries(ec_sentinel_app_tests PRIVATE ec_sentinel_app)
    target_compile_options(ec_sentinel_app_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_config_contracts COMMAND ec_sentinel_app_tests)
    set_tests_properties(sentinel_app_config_contracts PROPERTIES TIMEOUT 30)
    # O1 来源/权限/目标/恢复状态机.
    add_executable(ec_sentinel_authority_tests tests/sentinel_app_authority_tests.cpp)
    target_link_libraries(ec_sentinel_authority_tests PRIVATE ec_sentinel_app)
    target_compile_options(ec_sentinel_authority_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_authority_contracts COMMAND ec_sentinel_authority_tests)
    set_tests_properties(sentinel_app_authority_contracts PROPERTIES TIMEOUT 30)
    # O2 机构执行与真实 CAN 编组.
    add_executable(ec_sentinel_execution_tests tests/sentinel_app_execution_tests.cpp)
    target_link_libraries(ec_sentinel_execution_tests PRIVATE ec_sentinel_app)
    target_compile_options(ec_sentinel_execution_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_execution_contracts COMMAND ec_sentinel_execution_tests)
    set_tests_properties(sentinel_app_execution_contracts PROPERTIES TIMEOUT 30)
    # O3 感知/反馈重构与时钟.
    add_executable(ec_sentinel_serial_tests tests/sentinel_serial_tests.cpp)
    target_link_libraries(ec_sentinel_serial_tests PRIVATE ec_sentinel_app)
    target_compile_options(ec_sentinel_serial_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_serial_contracts COMMAND ec_sentinel_serial_tests)
    set_tests_properties(sentinel_serial_contracts PROPERTIES TIMEOUT 30)

    add_executable(ec_sentinel_inertial_tests tests/sentinel_inertial_tests.cpp)
    target_link_libraries(ec_sentinel_inertial_tests PRIVATE ec_sentinel_app)
    target_include_directories(ec_sentinel_inertial_tests PRIVATE robots/sentinel/host)
    target_compile_options(ec_sentinel_inertial_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_inertial_contracts COMMAND ec_sentinel_inertial_tests)
    set_tests_properties(sentinel_inertial_contracts PROPERTIES TIMEOUT 30)

    add_executable(ec_sentinel_replay examples/sentinel_replay.cpp)
    target_link_libraries(ec_sentinel_replay PRIVATE ec_sentinel_app)
    target_compile_options(ec_sentinel_replay PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)

    add_executable(ec_sentinel_perception_tests tests/sentinel_app_perception_tests.cpp)
    target_link_libraries(ec_sentinel_perception_tests PRIVATE ec_sentinel_app)
    target_compile_options(ec_sentinel_perception_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_perception_contracts COMMAND ec_sentinel_perception_tests)
    set_tests_properties(sentinel_app_perception_contracts PROPERTIES TIMEOUT 30)
    # O4 DT7/ch4 全链路与本地操纵映射.
    add_executable(ec_sentinel_remote_tests tests/sentinel_app_remote_tests.cpp)
    target_link_libraries(ec_sentinel_remote_tests PRIVATE ec_sentinel_app)
    target_compile_options(ec_sentinel_remote_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_remote_contracts COMMAND ec_sentinel_remote_tests)
    set_tests_properties(sentinel_app_remote_contracts PROPERTIES TIMEOUT 30)
    # O5 业务协议与会话契约.
    add_executable(ec_sentinel_protocol_tests tests/sentinel_app_protocol_tests.cpp)
    target_link_libraries(ec_sentinel_protocol_tests PRIVATE ec_sentinel_app)
    target_compile_options(ec_sentinel_protocol_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_protocol_contracts COMMAND ec_sentinel_protocol_tests)
    set_tests_properties(sentinel_app_protocol_contracts PROPERTIES TIMEOUT 30)
    # O6 正式 APP 部署配置: 直接使用板级头文件的同一份表, 核对"无实车参数就不给执行资格".
    add_executable(ec_sentinel_board_config_tests tests/sentinel_app_board_config_tests.cpp)
    target_link_libraries(ec_sentinel_board_config_tests PRIVATE ec_sentinel_app)
    target_include_directories(ec_sentinel_board_config_tests PRIVATE boards/sentinel_f407/include)
    target_compile_options(ec_sentinel_board_config_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_board_config_contracts COMMAND ec_sentinel_board_config_tests)
    set_tests_properties(sentinel_app_board_config_contracts PROPERTIES TIMEOUT 30)
    # O7 全 APP 闭环组合: 真实应用/执行/感知/协议 + 合成负载与软件总线 (主机侧合成夹具).
    add_executable(ec_sentinel_system_tests tests/sentinel_app_system_tests.cpp)
    target_link_libraries(ec_sentinel_system_tests PRIVATE ec_sentinel_app)
    target_include_directories(ec_sentinel_system_tests PRIVATE robots/sentinel/host)
    target_compile_options(ec_sentinel_system_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_system_contracts COMMAND ec_sentinel_system_tests)
    set_tests_properties(sentinel_app_system_contracts PROPERTIES TIMEOUT 60)
    # APP06 周期编排: 板级与主机共用的薄周期 (原始 DT7 帧/IMU 样本 → 真实 CAN 槽位).
    add_executable(ec_sentinel_cycle_tests tests/sentinel_app_cycle_tests.cpp)
    target_link_libraries(ec_sentinel_cycle_tests PRIVATE ec_sentinel_app)
    target_include_directories(ec_sentinel_cycle_tests PRIVATE robots/sentinel/host)
    target_compile_options(ec_sentinel_cycle_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME sentinel_app_cycle_contracts COMMAND ec_sentinel_cycle_tests)
    set_tests_properties(sentinel_app_cycle_contracts PROPERTIES TIMEOUT 30)
    # 装配层与板级无关: 用同一份源文件在主机上验证安装表/路由/布局拒绝.
    add_executable(ec_assembly_tests tests/assembly_tests.cpp boards/sentinel_f407/src/board_assembly.cpp
        boards/sentinel_f407/src/board_inputs.cpp)
    target_include_directories(ec_assembly_tests PRIVATE boards/sentinel_f407/include)
    target_link_libraries(ec_assembly_tests PRIVATE ec::core)
    target_compile_options(ec_assembly_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME assembly_routing_contracts COMMAND ec_assembly_tests)
    set_tests_properties(assembly_routing_contracts PROPERTIES TIMEOUT 30)
    add_executable(ec_dr16_input_tests tests/dr16_input_tests.cpp)
    target_link_libraries(ec_dr16_input_tests PRIVATE ec::core)
    target_compile_options(ec_dr16_input_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME dr16_input_contracts COMMAND ec_dr16_input_tests)
    set_tests_properties(dr16_input_contracts PROPERTIES TIMEOUT 30)
    add_executable(ec_attitude_tests tests/attitude_tests.cpp)
    target_link_libraries(ec_attitude_tests PRIVATE ec::core)
    target_compile_options(ec_attitude_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME attitude_contracts COMMAND ec_attitude_tests)
    set_tests_properties(attitude_contracts PROPERTIES TIMEOUT 60)
    add_executable(ec_kinematics_tests tests/kinematics_tests.cpp)
    target_link_libraries(ec_kinematics_tests PRIVATE ec::core)
    target_compile_options(ec_kinematics_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME omni_kinematics_contracts COMMAND ec_kinematics_tests)
    set_tests_properties(omni_kinematics_contracts PROPERTIES TIMEOUT 30)
    add_executable(ec_sil_tests tests/sil_tests.cpp)
    target_link_libraries(ec_sil_tests PRIVATE ec::core Threads::Threads)
    target_compile_options(ec_sil_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_executable(ec_gm6020_ident_sim examples/gm6020_ident_sim.cpp)
    target_link_libraries(ec_gm6020_ident_sim PRIVATE ec::core)
    target_include_directories(ec_gm6020_ident_sim PRIVATE boards/sentinel_f407/include)
    add_executable(ec_gm6020_bench_tests tests/gm6020_bench_tests.cpp)
    target_link_libraries(ec_gm6020_bench_tests PRIVATE ec::core)
    target_include_directories(ec_gm6020_bench_tests PRIVATE boards/sentinel_f407/include)
    add_test(NAME gm6020_bench_contracts COMMAND ec_gm6020_bench_tests)
    add_executable(ec_uart_frame_tests tests/uart_frame_tests.cpp)
    target_link_libraries(ec_uart_frame_tests PRIVATE ec::core)
    target_include_directories(ec_uart_frame_tests PRIVATE boards/sentinel_f407/include)
    target_compile_options(ec_uart_frame_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME uart_frame_contracts COMMAND ec_uart_frame_tests)
    # Python 侧的契约测试是验收的一部分: 缺解释器时必须显式失败或明确报告 partial,
    # 不能让注册数量静默缩小后仍被当成"全部通过".
    option(EC_REQUIRE_PYTHON_TESTS "Fail configuration when the Python interpreter is missing" ON)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(NOT Python3_Interpreter_FOUND)
        if(EC_REQUIRE_PYTHON_TESTS)
            message(FATAL_ERROR "Python3 interpreter is required for the acceptance tests. "
                "Install it, or configure with -DEC_REQUIRE_PYTHON_TESTS=OFF to accept a partial "
                "(C++ only) test set.")
        endif()
        message(WARNING "Python3 not found: board_monitor/motor_bench/monitor_swd contracts are "
            "NOT registered. This configuration is PARTIAL and must not be reported as full pass.")
    endif()
    if(Python3_Interpreter_FOUND)
        add_test(NAME sentinel_capture_client_contracts COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/capture_client_tests.py" "$<TARGET_FILE:ec_capture_tests>")
        set_tests_properties(sentinel_capture_client_contracts PROPERTIES TIMEOUT 30)
        add_test(NAME sentinel_raw_calibration_contracts COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/sentinel_raw_tests.py" "$<TARGET_FILE:ec_capture_tests>" "$<TARGET_FILE:ec_sentinel_replay>")
        set_tests_properties(sentinel_raw_calibration_contracts PROPERTIES TIMEOUT 60)
        add_test(NAME board_monitor_contracts COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/board_monitor_tests.py" "$<TARGET_FILE:ec_uart_frame_tests>")
        add_test(NAME sentinel_host_client_contracts COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/sentinel_host_client_tests.py"
            "$<TARGET_FILE:ec_sentinel_app_host>")
        set_tests_properties(sentinel_host_client_contracts PROPERTIES TIMEOUT 120)
        add_test(NAME motor_bench_client_contracts COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/motor_bench_tests.py" "$<TARGET_FILE:ec_gm6020_bench_tests>")
        set_tests_properties(motor_bench_client_contracts PROPERTIES TIMEOUT 20 SKIP_RETURN_CODE 77)
    endif()
    add_executable(ec_monitor_tests tests/monitor_tests.cpp)
    target_link_libraries(ec_monitor_tests PRIVATE ec::core)
    target_compile_options(ec_monitor_tests PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME monitor_registry_contracts COMMAND ec_monitor_tests
        "${CMAKE_CURRENT_BINARY_DIR}/monitor_artifacts")
    if(Python3_Interpreter_FOUND)
        # 串口载荷与假 SWD 内存必须得到同一份逻辑值; 另含只读自检 (不连物理探针).
        add_test(NAME monitor_swd_contracts COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/boards/sentinel_f407/tools/monitor_swd.py"
            --verify "${CMAKE_CURRENT_BINARY_DIR}/monitor_artifacts" --json)
        set_tests_properties(monitor_swd_contracts PROPERTIES DEPENDS monitor_registry_contracts TIMEOUT 30)
        add_test(NAME monitor_swd_selftest COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/boards/sentinel_f407/tools/monitor_swd.py" --self-test --json)
        set_tests_properties(monitor_swd_selftest PROPERTIES TIMEOUT 30)
        # 文档一致性: 被跟踪 Markdown 里的本地链接必须指向被跟踪文件.
        add_test(NAME gm6020_identification_contracts COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/gm6020_identify_tests.py" "$<TARGET_FILE:ec_gm6020_ident_sim>")
        set_tests_properties(gm6020_identification_contracts PROPERTIES TIMEOUT 120)
        add_test(NAME doc_link_contracts COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/check_doc_links.py")
        set_tests_properties(doc_link_contracts PROPERTIES TIMEOUT 30)
    endif()
    add_test(NAME sil_contracts COMMAND ec_sil_tests)
    set_tests_properties(sil_contracts PROPERTIES TIMEOUT 30)
endif()
if(EC_BUILD_EXAMPLES)
    add_executable(ec_assembly_example examples/host_assembly.cpp)
    target_link_libraries(ec_assembly_example PRIVATE ec::core Threads::Threads)
    if(EC_BUILD_TESTS)
        add_test(NAME assembly_example COMMAND ec_assembly_example)
    endif()
    # 最小单轴入门示例与合成多设备 APP 装配示例 (均离线运行).
    add_executable(ec_single_axis_example examples/host_single_axis.cpp)
    target_link_libraries(ec_single_axis_example PRIVATE ec::core)
    target_compile_options(ec_single_axis_example PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_executable(ec_app_example examples/host_app_example.cpp)
    target_link_libraries(ec_app_example PRIVATE ec::core)
    target_compile_options(ec_app_example PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    # O5 主机软件后端: 离线运行正式 APP 并说业务协议, 不打开串口/CAN.
    add_executable(ec_sentinel_app_host examples/sentinel_app_host_link.cpp)
    target_link_libraries(ec_sentinel_app_host PRIVATE ec_sentinel_app)
    target_include_directories(ec_sentinel_app_host PRIVATE robots/sentinel/host)
    target_compile_options(ec_sentinel_app_host PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    if(EC_BUILD_TESTS)
        add_test(NAME single_axis_example COMMAND ec_single_axis_example)
        add_test(NAME app_assembly_example COMMAND ec_app_example)
        set_tests_properties(app_assembly_example PROPERTIES TIMEOUT 30)
    endif()
    add_executable(ec_link_example examples/host_link_example.cpp)
    target_link_libraries(ec_link_example PRIVATE ec::core Threads::Threads)
    target_compile_options(ec_link_example PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Werror>)
    if(EC_BUILD_TESTS)
        add_test(NAME link_example COMMAND ec_link_example)
    endif()
endif()

if(EC_BUILD_TESTS)
    # 注册清单自检: 期望的测试名一个都不能少 (漏注册会让"通过"看起来依然完整).
    # 放在文件末尾, 因此必须等示例测试也注册完成后再核对.
    set(_ec_expected_tests sentinel_capture_contracts core_contracts control_contracts axis_contracts dm_contracts
        session_authority_contracts sentinel_app_config_contracts
        sentinel_app_authority_contracts sentinel_app_execution_contracts
        sentinel_app_perception_contracts sentinel_serial_contracts sentinel_app_remote_contracts sentinel_app_cycle_contracts
        sentinel_app_protocol_contracts sentinel_app_board_config_contracts sentinel_app_system_contracts
        assembly_routing_contracts dr16_input_contracts
        attitude_contracts omni_kinematics_contracts gm6020_bench_contracts uart_frame_contracts
        monitor_registry_contracts sil_contracts)
    if(Python3_Interpreter_FOUND)
        list(APPEND _ec_expected_tests sentinel_capture_client_contracts sentinel_raw_calibration_contracts board_monitor_contracts
            sentinel_host_client_contracts motor_bench_client_contracts
            monitor_swd_contracts monitor_swd_selftest doc_link_contracts)
    endif()
    if(EC_BUILD_EXAMPLES)
        list(APPEND _ec_expected_tests assembly_example app_assembly_example link_example
            single_axis_example)
    endif()
    get_property(_ec_registered_tests DIRECTORY PROPERTY TESTS)
    foreach(_ec_expected IN LISTS _ec_expected_tests)
        if(NOT _ec_expected IN_LIST _ec_registered_tests)
            message(FATAL_ERROR "expected test '${_ec_expected}' was not registered")
        endif()
    endforeach()
endif()
