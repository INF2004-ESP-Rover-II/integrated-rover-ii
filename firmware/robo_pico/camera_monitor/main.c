/**
 * @file main.c
 * @brief Main firmware module.
 */
#include "camera_uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Camera receiver bench image: no motors, sensors, Wi-Fi or MQTT initialised.
int
main (void)
{
    stdio_init_all();
    sleep_ms(2000);
    printf("\nRoboPico camera UART monitor (USB serial output)\n");
    camera_uart_init();
    while (true)
    {
        camera_uart_poll();
        sleep_ms(1);
    }
}

/* End of file. */
