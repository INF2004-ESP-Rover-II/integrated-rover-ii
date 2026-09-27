cmake_minimum_required(VERSION 3.13)

# Set default board type (can be overridden by subdirectories)
set(PICO_BOARD pico_w)

# Import Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Top-level project
project(sensor_node C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize SDK once at the top level
pico_sdk_init()

include(${CMAKE_CURRENT_LIST_DIR}/lib/vl53l1x/import.cmake)
add_subdirectory(lib/fatfs/FatFs_SPI build_fatfs)

add_executable(sensor_pico
    src/main.c
    src/uart_protocol.c
    src/ultrasonic.c
    src/accelero.c
    src/sd_card.c
    src/sd_config.c
    src/rplidar.c
    src/tof.c
)

target_include_directories(sensor_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(sensor_pico
    pico_stdlib          # Standard library
    hardware_uart        # UART communication
    hardware_gpio        # GPIO control
    hardware_i2c         # I2C for accelerometer
    hardware_timer       # Timers for ultrasonic
    hardware_pwm
    FatFs_SPI            # Access for SD card 
    VL53L1X_pico_api     # Three ToF distance sensors
)

# Enable USB serial output for debugging (disable UART stdio)
pico_enable_stdio_usb(sensor_pico 1)
pico_enable_stdio_uart(sensor_pico 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(sensor_pico)
