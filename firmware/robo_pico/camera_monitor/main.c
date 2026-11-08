#include <stdio.h>
#include "pico/stdlib.h"
#include "camera_uart.h"

// Camera receiver bench image: no motors, sensors, Wi-Fi or MQTT initialised.
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    printf("\nRoboPico camera UART monitor (USB serial output)\n");
    camera_uart_init();
    while (true) {
        camera_uart_poll();
        sleep_ms(1);
    }
}
