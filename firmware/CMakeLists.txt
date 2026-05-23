cmake_minimum_required(VERSION 3.13)

# Pico SDK bootstrap — set PICO_SDK_PATH in your environment
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(LFR_PID C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# RP2350 target (Pico 2)
set(PICO_BOARD pico2)

pico_sdk_init()

add_executable(LFR_PID
    LFR_PID_Pico2.cpp
)

# Link required Pico SDK libraries
target_link_libraries(LFR_PID
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_gpio
    hardware_timer
    hardware_flash
    hardware_sync
    pico_time
    m
)

# Enable USB serial output for debug printf (optional)
pico_enable_stdio_usb(LFR_PID 1)
pico_enable_stdio_uart(LFR_PID 0)

# Generate UF2 file for drag-and-drop flashing
pico_add_extra_outputs(LFR_PID)
