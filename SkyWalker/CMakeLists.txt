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
    Algorithm/Src/alg_euler.c
    Algorithm/Src/alg_pid.c
    Algorithm/Src/alg_quaternion.c
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