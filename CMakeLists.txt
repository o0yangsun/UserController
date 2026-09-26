#此文件从模板自动生成! 请勿更改!

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

cmake_minimum_required(VERSION 4.0)


# ============================================================
# Cross compiler and tools
# ============================================================

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ============================================================
# Project settings
# ============================================================

project(STM32H7232_UserController C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# MCU architecture
# ============================================================

set(MCPU "cortex-m7")

set(CPU_FLAGS
        -mcpu=cortex-m7
        -mthumb
)

# 根据 Cortex 内核选择浮点参数
if ("${MCPU}" STREQUAL "cortex-m7")

    message(STATUS "Cortex-M7 hardware FPU enabled: FPv5-D16")

    set(FPU_FLAGS
            -mfpu=fpv5-d16
            -mfloat-abi=hard
    )

elseif ("${MCPU}" STREQUAL "cortex-m4")

    message(STATUS "Cortex-M4 hardware FPU enabled: FPv4-SP-D16")

    set(FPU_FLAGS
            -mfpu=fpv4-sp-d16
            -mfloat-abi=hard
    )

else ()

    message(STATUS "Hardware FPU is not enabled for ${MCPU}")

    set(FPU_FLAGS
            -mfloat-abi=soft
    )

endif ()

# ============================================================
# Compiler options
# ============================================================

add_compile_options(
        ${CPU_FLAGS}
        ${FPU_FLAGS}

        -ffunction-sections
        -fdata-sections
        -fno-common
        -fmessage-length=0
)

# Enable assembler file preprocessing
add_compile_options(
        $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
)

# Uncomment to suppress C++17 register keyword warnings
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# ============================================================
# Optimization settings
# ============================================================

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)

elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")

    message(STATUS "Maximum optimization for speed with debug information")
    add_compile_options(-Ofast -g)

elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")

    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)

else ()

    message(STATUS "Minimal optimization with debug information")
    add_compile_options(-Og -g)

endif ()

# ============================================================
# Include directories and preprocessor definitions
# ============================================================

include_directories(
        Core/Inc Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include
        User/Algorithm/Crc8_Crc16
        User/Module/Dwt
        User/Module/FTSservo
        User/Module/TIM_Delay
        User/Module/Usb_Pc
        User/Task/Algorithm
        User/Task/Robot_Config
        User/Task/Usart_Send
        # --- USB CDC 虚拟串口（PC ↔ 主臂板），移植自车端 Engineering_Robot_H723_ ---
        USB_DEVICE/App USB_DEVICE/Target
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

add_definitions(
        -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx
)

# ============================================================
# Source files
# ============================================================

# CONFIGURE_DEPENDS：让 CMake 每次构建都重新扫描目录。
# 否则 GLOB 结果在 configure 时就被固化了 —— 新增 .c 文件不会进构建，
# 表现为"undefined reference to 新函数"，必须手工重新 configure 才能发现。
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "Core/*.*" "Middlewares/*.*" "Drivers/*.*"
        "USB_DEVICE/*.*"
        "User/*.*"
)

# ============================================================
# Linker settings
# ============================================================

set(LINKER_SCRIPT
        ${CMAKE_SOURCE_DIR}/STM32H723VGTX_FLASH.ld
)

add_link_options(
        ${CPU_FLAGS}
        ${FPU_FLAGS}

        -T${LINKER_SCRIPT}

        -Wl,-gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)

# ============================================================
# Executable target
# ============================================================

add_executable(
        ${PROJECT_NAME}.elf
        ${SOURCES}
        ${LINKER_SCRIPT}
)

# ============================================================
# HEX and BIN generation
# ============================================================

set(HEX_FILE
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
)

set(BIN_FILE
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
)

add_custom_command(
        TARGET ${PROJECT_NAME}.elf
        POST_BUILD

        COMMAND ${CMAKE_OBJCOPY}
                -Oihex
                $<TARGET_FILE:${PROJECT_NAME}.elf>
                ${HEX_FILE}

        COMMAND ${CMAKE_OBJCOPY}
                -Obinary
                $<TARGET_FILE:${PROJECT_NAME}.elf>
                ${BIN_FILE}

        COMMAND ${CMAKE_SIZE}
                $<TARGET_FILE:${PROJECT_NAME}.elf>

        COMMENT
                "Building ${HEX_FILE}
Building ${BIN_FILE}"
)