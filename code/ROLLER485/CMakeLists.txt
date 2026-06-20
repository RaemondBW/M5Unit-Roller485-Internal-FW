cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# Bare-metal GCC/CMake build for the ROLLER485 application (STM32G431CBUx).
# Reconstructed to build with STM32CubeCLT (the repo only shipped a Keil
# MDK-ARM project). FreeRTOS is intentionally excluded: the application runs
# as a superloop (main() never starts the scheduler).
# ---------------------------------------------------------------------------

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain from STM32CubeCLT (set on PATH by `setup-stm32`).
set(TOOLCHAIN_DIR /opt/ST/STM32CubeCLT_1.17.0/GNU-tools-for-STM32/bin)
set(CMAKE_C_COMPILER   ${TOOLCHAIN_DIR}/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_DIR}/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/arm-none-eabi-g++)
set(CMAKE_OBJCOPY      ${TOOLCHAIN_DIR}/arm-none-eabi-objcopy)
set(CMAKE_SIZE         ${TOOLCHAIN_DIR}/arm-none-eabi-size)

# Don't let CMake try to build/run a host test executable.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(ROLLER485 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXECUTABLE_SUFFIX .elf)

set(MCU_FLAGS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

add_compile_definitions(
    USE_HAL_DRIVER
    USE_FULL_LL_DRIVER
    STM32G431xx
    ARM_MATH_CM4
)

include_directories(
    Core/Inc
    MyFile/inc
    U8g2_lib
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/Include
)

# Application + BSP sources.
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Core/Src/*.c
    ${CMAKE_SOURCE_DIR}/MyFile/src/*.c
    ${CMAKE_SOURCE_DIR}/U8g2_lib/*.c
)

# STM32 HAL/LL drivers (skip the *_template.c sample files).
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/*.c
)
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*_template\\.c$")

# Restored bare-metal support sources.
set(SUPPORT_SOURCES
    ${CMAKE_SOURCE_DIR}/syscalls.c
    ${CMAKE_SOURCE_DIR}/sysmem.c
)

set(STARTUP_ASM
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g431xx.s
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32G431CBUX_FLASH.ld)

add_executable(${PROJECT_NAME}
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${SUPPORT_SOURCES}
    ${STARTUP_ASM}
)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -Wall
    -g3
    $<$<CONFIG:Debug>:-Og>
    $<$<NOT:$<CONFIG:Debug>>:-O2>
)

target_link_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    --specs=nano.specs
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--print-memory-usage
)

# Link math/c libs via target_link_libraries so CMake places them AFTER the
# object files (target_link_options are emitted before objects, which would
# leave libm symbols like sinf/cosf unresolved).
target_link_libraries(${PROJECT_NAME} PRIVATE m c)

# Emit .hex and .bin alongside the .elf, and print the size summary.
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)
