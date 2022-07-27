cmake_minimum_required(VERSION 3.13)

include(../pico-sdk/pico_sdk_init.cmake)
pico_sdk_init()

project(robot-arm)

add_executable(${PROJECT_NAME} main.c)

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_gpio hardware_pwm hardware_pio hardware_dma hardware_clocks)

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
