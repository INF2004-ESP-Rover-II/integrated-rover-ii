# Motor bench app: Motor Control task only, controlled over USB serial.
# Build:
#   export PICO_SDK_PATH=/path/to/pico-sdk
#   export FREERTOS_KERNEL_PATH=/path/to/FreeRTOS-Kernel   (raspberrypi/FreeRTOS-Kernel)
#   mkdir build && cd build && cmake .. && make
# Flash motor_bench.uf2, open a serial monitor, type 'q'.
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(motor_bench C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(ROBO ${CMAKE_CURRENT_LIST_DIR}/..)

add_executable(motor_bench
    main.c
    ${ROBO}/src/motor_control.c
    ${ROBO}/src/motor_hw.c
    ${ROBO}/src/motor_task.c
)

target_include_directories(motor_bench PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}          # FreeRTOSConfig.h
    ${ROBO}/include
)

target_compile_options(motor_bench PRIVATE -Wall -Wextra)

# Wheel PWM frequency, fixed per build (changing it at run time crashed the Pico).
#   cmake ..                     -> 10 kHz (default)
#   cmake -DMOTOR_PWM_HZ=1000 .. -> 1 kHz (more power at low speed on the L298N)
if(DEFINED MOTOR_PWM_HZ)
    target_compile_definitions(motor_bench PRIVATE PWM_FREQ_HZ=${MOTOR_PWM_HZ})
endif()

target_link_libraries(motor_bench
    pico_stdlib
    hardware_pwm
    hardware_uart
    FreeRTOS-Kernel-Heap4
)

# GP0/GP1 are motor direction pins: printf over USB only.
pico_enable_stdio_usb(motor_bench 1)
pico_enable_stdio_uart(motor_bench 0)

pico_add_extra_outputs(motor_bench)
