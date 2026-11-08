#ifndef ROVER_CAMERA_UART_H
#define ROVER_CAMERA_UART_H

#include "camera_protocol.h"

typedef struct {
    camera_frame_t frame;
    uint32_t received_ms;
    uint32_t generation; // Advances for every valid UART packet, including health.
} camera_observation_t;

// Dedicated UART1 RX on GP21, 115200 8N1. Does not configure a TX pin.
void camera_uart_init(void);
// Call regularly from the main loop. Decode/print only; never commands motors.
void camera_uart_poll(void);
// Main-loop snapshot only. The camera-only monitor needs no MQTT dependency.
bool camera_uart_latest(camera_observation_t *observation);

#endif
