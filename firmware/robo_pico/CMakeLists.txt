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
    src/motor_control.c          # Subsystem 6 Motion Control logic
    src/motor_hw.c               # Subsystem 6 L298N / arm / claw pins
    src/wifi_mqtt.c
    src/uart_protocol.c
    src/safety_interlock.c
)

target_include_directories(mecanum_robot PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# Subsystem 6: allow manual moves while the Sensor Pico heading is missing or
# late (heading hold then pauses). Set to 0 once heading arrives every 50 ms.
target_compile_definitions(mecanum_robot PRIVATE MC_ALLOW_MANUAL_WITHOUT_HEADING=1)
# The Sensor Pico currently sends about 4 packets/s (not 20), so a heading is
# up to ~250 ms old between packets. Allow 600 ms before "heading too old"
# stops an auto move. Back to 150 once the Sensor Pico sends at 20 Hz.
target_compile_definitions(mecanum_robot PRIVATE MC_HEADING_STALE_MS=600u)

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
