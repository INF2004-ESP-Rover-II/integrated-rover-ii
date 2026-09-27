cmake_minimum_required(VERSION 3.13)

# Set board type to pico_w BEFORE including SDK
set(PICO_BOARD pico_w)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(mecanum_robot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(mecanum_robot
    src/main.c
    src/motor_driver.c
    src/wifi_mqtt.c
    src/uart_protocol.c
)

target_include_directories(mecanum_robot PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(mecanum_robot
    pico_stdlib
    pico_cyw43_arch_lwip_poll    # WiFi + lwIP for Pico W
    hardware_pwm
    hardware_gpio
    hardware_uart
    pico_lwip_mqtt               # MQTT support
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(mecanum_robot 1)
pico_enable_stdio_uart(mecanum_robot 0)

pico_add_extra_outputs(mecanum_robot)
