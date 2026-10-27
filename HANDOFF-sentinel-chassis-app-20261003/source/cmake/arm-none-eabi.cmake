set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 显式传入 ARM_GNU_ROOT 或把工具链加入 PATH, 不硬编码用户路径.
set(ARM_GNU_ROOT "" CACHE PATH "Arm GNU toolchain root")
# CMake 的 ABI 探测会重新加载本文件, 必须向探测子项目传递工具链根目录.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_GNU_ROOT)
find_program(CMAKE_C_COMPILER arm-none-eabi-gcc HINTS "${ARM_GNU_ROOT}/bin" REQUIRED)
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++ HINTS "${ARM_GNU_ROOT}/bin" REQUIRED)
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
find_program(CMAKE_AR arm-none-eabi-ar HINTS "${ARM_GNU_ROOT}/bin" REQUIRED)
find_program(CMAKE_RANLIB arm-none-eabi-ranlib HINTS "${ARM_GNU_ROOT}/bin" REQUIRED)
set(_cpu "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_ASM_FLAGS_INIT "${_cpu} -x assembler-with-cpp")
set(CMAKE_C_FLAGS_INIT "${_cpu} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT "${_cpu} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti")
