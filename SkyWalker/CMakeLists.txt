cmake_minimum_required(VERSION 3.22)

# 定义SkyWalker库
project(SkyWalker)

# 设置C Standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 创建静态库
add_library(SkyWalker STATIC)

# 添加源文件
target_sources(SkyWalker PRIVATE
    Algorithm/Src/alg_conversion.c
    Algorithm/Src/alg_crc.c
    Algorithm/Src/alg_imu_ekf.c
    Algorithm/Src/alg_euler.c
    Algorithm/Src/alg_kf.c
    Algorithm/Src/alg_lpf.c
    Algorithm/Src/alg_lqr.c
    Algorithm/Src/alg_pid.c
    Algorithm/Src/alg_quaternion.c
    Algorithm/Src/alg_median.c
    Algorithm/Src/alg_notch.c
    Algorithm/Src/alg_smc.c
    Algorithm/Src/alg_td.c
    BSP/Src/bsp_fdcan.c
    BSP/Src/bsp_pwm.c
    BSP/Src/bsp_spi.c
    BSP/Src/bsp_uart.c
    BSP/Src/bsp_usb.c
)

# 添加DSP源文件
file(GLOB_RECURSE CMSIS_DSP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/DSP/Source/MatrixFunctions/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/DSP/Source/CommonTables/*.c"
)

target_sources(SkyWalker PRIVATE ${CMSIS_DSP_SOURCES})

# 添加头文件目录
target_include_directories(SkyWalker PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/Inc
    # 添加DSP头文件目录
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/ST/ARM/DSP/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/DSP/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/Include
)

# 添加DSP编译定义
target_compile_definitions(SkyWalker PRIVATE
    ARM_MATH_CM7
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# 添加依赖的库
target_link_libraries(SkyWalker PUBLIC
    stm32cubemx
)

# SkyWalker本地格式化目标
find_program(CLANG_FORMAT_BIN NAMES clang-format)
if(CLANG_FORMAT_BIN)
    file(GLOB_RECURSE SKYWALKER_FORMAT_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/Inc/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/BSP/Src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/Device/Inc/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Device/Src/*.c"
    )
    add_custom_target(skywalker-format
        COMMAND ${CLANG_FORMAT_BIN} -i ${SKYWALKER_FORMAT_FILES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Run clang-format for SkyWalker files only"
        VERBATIM
    )
    add_custom_target(skywalker-format-check
        COMMAND ${CLANG_FORMAT_BIN} --dry-run --Werror ${SKYWALKER_FORMAT_FILES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Check clang-format for SkyWalker files only"
        VERBATIM
    )
else()
    message(STATUS "clang-format not found: skywalker-format targets are disabled")
endif()