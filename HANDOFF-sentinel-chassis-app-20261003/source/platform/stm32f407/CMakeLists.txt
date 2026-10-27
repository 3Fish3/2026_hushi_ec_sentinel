if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    message(FATAL_ERROR "STM32F407 adapters require the Arm cross toolchain")
endif()
# 只读取外部 SDK 与板级头文件, 不引入旧项目业务、启动代码或隐式链接资料路径.
set(EC_CUBE_PROJECT "" CACHE PATH "Cube SDK or project with Drivers and Middlewares")
set(EC_BOARD_CONFIG_DIR "" CACHE PATH "Board configuration headers (default: EC_CUBE_PROJECT/Inc)")
if(NOT EC_BOARD_CONFIG_DIR)
    set(EC_BOARD_CONFIG_DIR "${EC_CUBE_PROJECT}/Inc")
endif()
# 外部 SDK 若有追踪补丁, 由调用者显式补充头文件路径; 核心不依赖追踪实现.
set(EC_SDK_EXTRA_INCLUDE_DIRS "" CACHE STRING "Additional include directories required by a patched SDK")
foreach(_required
    Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h
    Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h
    Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h)
    if(NOT EXISTS "${EC_CUBE_PROJECT}/${_required}")
        message(FATAL_ERROR "Missing external SDK/config file: ${EC_CUBE_PROJECT}/${_required}")
    endif()
endforeach()
foreach(_header stm32f4xx_hal_conf.h FreeRTOSConfig.h)
    if(NOT EXISTS "${EC_BOARD_CONFIG_DIR}/${_header}")
        message(FATAL_ERROR "Missing board configuration: ${EC_BOARD_CONFIG_DIR}/${_header}")
    endif()
endforeach()
add_library(ec_stm32f407 STATIC src/adapter.cpp src/contract_compile.cpp)
add_library(ec::stm32f407 ALIAS ec_stm32f407)
target_link_libraries(ec_stm32f407 PUBLIC ec::core)
target_include_directories(ec_stm32f407 PUBLIC include)
target_include_directories(ec_stm32f407 SYSTEM PUBLIC
    "${EC_BOARD_CONFIG_DIR}"
    "${EC_CUBE_PROJECT}/Drivers/STM32F4xx_HAL_Driver/Inc"
    "${EC_CUBE_PROJECT}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
    "${EC_CUBE_PROJECT}/Drivers/CMSIS/Include"
    "${EC_CUBE_PROJECT}/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
    "${EC_CUBE_PROJECT}/Middlewares/Third_Party/FreeRTOS/Source/include"
    "${EC_CUBE_PROJECT}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    ${EC_SDK_EXTRA_INCLUDE_DIRS})
target_compile_definitions(ec_stm32f407 PUBLIC STM32F407xx USE_HAL_DRIVER)
target_compile_options(ec_stm32f407 PRIVATE -Wall -Wextra -Wpedantic -Werror)
