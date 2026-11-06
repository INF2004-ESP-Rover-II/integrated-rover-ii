#ifndef CAMERA_UART_H
#define CAMERA_UART_H

#include "vision_protocol.h"

#define CAMERA_UART_RX_PIN 21u

typedef struct {
    vision_frame_t frame;
    uint32_t received_ms;
} camera_packet_t;

typedef struct {
    vision_parser_stats_t parser;
    uint32_t uart_errors;
    uint32_t mailbox_overwrites; /* Complete packets replaced before consumption. */
} camera_uart_stats_t;

/* UART1, GP21 RX, 115200 8N1; UART0 and motor pins are untouched.
 * GP21 shares the Robo Pico button: reserve it for UART, do not press it. */
void camera_uart_init(void);
/* Main-loop calls; latest-value mailbox, NOT a navigation/event queue.
 * Packets include duplicates/health reports; downstream code must distinguish
 * a new image from a repeated sequence. No motor or map actions happen here. */
void camera_uart_poll(void);
bool camera_uart_take_packet(camera_packet_t *packet);
void camera_uart_get_stats(camera_uart_stats_t *stats);

#endif
