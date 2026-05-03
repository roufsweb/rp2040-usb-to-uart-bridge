# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(pico_sdk_import.cmake)

project(pico_uart_bridge)

pico_sdk_init()

add_executable(uart_bridge 
    src/main.c 
    src/usb_descriptors.c
)

target_include_directories(uart_bridge PUBLIC
    src/
    include/
)

target_link_libraries(uart_bridge
    hardware_flash
    pico_multicore
    hardware_pio
    pico_stdlib
    tinyusb_device
    pico_util
)

pico_add_extra_outputs(uart_bridge)
