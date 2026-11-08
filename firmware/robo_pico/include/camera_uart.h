#ifndef ROVER_CAMERA_UART_H
#define ROVER_CAMERA_UART_H

// Dedicated UART1 RX on GP21, 115200 8N1. Does not configure a TX pin.
void camera_uart_init(void);
// Call regularly from the main loop. Decode/print only; never commands motors.
void camera_uart_poll(void);

#endif
