#include "camera_uart.h"

#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#define CAMERA_UART uart1

static vision_parser_t parser;
static camera_packet_t latest;
static bool pending;
static uint32_t uart_errors;
static uint32_t mailbox_overwrites;

static void publish_packet(const vision_frame_t *frame, uint32_t now_ms) {
    if (pending) ++mailbox_overwrites;
    latest.frame = *frame;
    latest.received_ms = now_ms;
    pending = true;
}

static void camera_rx_irq(void) {
    while (uart_is_readable(CAMERA_UART)) {
        uint32_t data = uart_get_hw(CAMERA_UART)->dr;
        if ((data & (UART_UARTDR_OE_BITS | UART_UARTDR_BE_BITS |
                     UART_UARTDR_PE_BITS | UART_UARTDR_FE_BITS)) != 0u) {
            ++uart_errors;
            vision_parser_reset(&parser);
            uart_get_hw(CAMERA_UART)->rsr = 0;
            continue;
        }
        vision_frame_t frame;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (vision_parser_feed(&parser, (uint8_t)data, now, &frame)) {
            publish_packet(&frame, parser.frame_received_ms);
        }
    }
}

void camera_uart_init(void) {
    irq_set_enabled(UART1_IRQ, false);
    vision_parser_init(&parser);
    pending = false;
    uart_errors = 0;
    mailbox_overwrites = 0;
    uart_init(CAMERA_UART, VISION_BAUD);
    gpio_set_function(CAMERA_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(CAMERA_UART_RX_PIN);
    uart_set_format(CAMERA_UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(CAMERA_UART, false, false);
    uart_set_fifo_enabled(CAMERA_UART, true);
    irq_set_exclusive_handler(UART1_IRQ, camera_rx_irq);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(CAMERA_UART, true, false);
}

void camera_uart_poll(void) {
    uint32_t irq = save_and_disable_interrupts();
    vision_frame_t frame;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (vision_parser_poll(&parser, now, &frame)) {
        /* A salvaged suffix was fully received at frame_received_ms, not now.
         * Keeping that receive time ensures recovery cannot rejuvenate it. */
        publish_packet(&frame, parser.frame_received_ms);
    }
    restore_interrupts(irq);
}

bool camera_uart_take_packet(camera_packet_t *packet) {
    uint32_t irq = save_and_disable_interrupts();
    bool available = pending;
    if (available) {
        *packet = latest;
        pending = false;
    }
    restore_interrupts(irq);
    return available;
}

void camera_uart_get_stats(camera_uart_stats_t *stats) {
    uint32_t irq = save_and_disable_interrupts();
    stats->parser = parser.stats;
    stats->uart_errors = uart_errors;
    stats->mailbox_overwrites = mailbox_overwrites;
    restore_interrupts(irq);
}
