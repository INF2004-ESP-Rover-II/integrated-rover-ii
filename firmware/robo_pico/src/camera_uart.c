#include "camera_uart.h"
#include "camera_protocol.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/regs/uart.h"
#include "hardware/sync.h"
#include "hardware/uart.h"

#define CAMERA_UART uart1
#define CAMERA_RX_PIN 21u
#define CAMERA_BAUD 115200u
#define CAMERA_RX_CAPACITY 512u
#define CAMERA_POLL_BYTE_LIMIT 128u
#define CAMERA_STATS_INTERVAL_MS 5000u

typedef struct {
    uint32_t receive_ms;
    uint8_t byte;
} received_byte_t;

static received_byte_t receive_ring[CAMERA_RX_CAPACITY];
static volatile uint16_t receive_head, receive_tail;
static volatile bool receive_loss;
static volatile uint32_t received_bytes, dropped_bytes, uart_errors;
static camera_parser_t parser;
static uint32_t last_stats_ms;
static camera_observation_t latest_observation;
static bool have_observation;

static void camera_uart_irq(void)
{
    // Only buffer bytes in the ISR. In particular, never printf here.
    while (uart_is_readable(CAMERA_UART)) {
        uint32_t data = uart_get_hw(CAMERA_UART)->dr;
        received_bytes++;
        if (data & (UART_UARTDR_OE_BITS | UART_UARTDR_BE_BITS |
                    UART_UARTDR_PE_BITS | UART_UARTDR_FE_BITS)) {
            uart_errors++;
            receive_loss = true;
            uart_get_hw(CAMERA_UART)->rsr = 0; // Clear sticky UART error flags.
        }
        uint16_t next = (uint16_t)((receive_head + 1u) % CAMERA_RX_CAPACITY);
        if (receive_loss || next == receive_tail) {
            receive_loss = true;
            dropped_bytes++;
            continue;
        }
        receive_ring[receive_head].byte = (uint8_t)data;
        receive_ring[receive_head].receive_ms = to_ms_since_boot(get_absolute_time());
        receive_head = next;
    }
}

void camera_uart_init(void)
{
    camera_parser_init(&parser);
    have_observation = false;
    latest_observation.generation = 0;
    receive_head = receive_tail = 0;
    receive_loss = false;
    received_bytes = dropped_bytes = uart_errors = 0;
    last_stats_ms = to_ms_since_boot(get_absolute_time());
    uart_init(CAMERA_UART, CAMERA_BAUD);
    gpio_set_function(CAMERA_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(CAMERA_RX_PIN); // UART idle level when the cable is absent.
    uart_set_format(CAMERA_UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(CAMERA_UART, false, false);
    uart_set_fifo_enabled(CAMERA_UART, true);
    irq_set_exclusive_handler(UART1_IRQ, camera_uart_irq);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(CAMERA_UART, true, false);
    printf("[CAMERA] UART1 RX=GP%u, %u baud 8N1; ESP32 GPIO17 -> GP21, common GND\n",
           CAMERA_RX_PIN, CAMERA_BAUD);
}

static void print_frame(const camera_frame_t *frame, uint32_t receive_ms)
{
    uint32_t queue_age_ms = to_ms_since_boot(get_absolute_time()) - receive_ms;
    printf("[CAMERA] seq=%u status=%s(%u) age_ms=%u markers=%u queue_ms=%lu\n",
           frame->seq, camera_status_name(frame->status), frame->status,
           frame->age_ms, frame->count, (unsigned long)queue_age_ms);
    for (uint8_t i = 0; i < frame->count; ++i) {
        const camera_marker_t *marker = &frame->markers[i];
        int bearing = marker->bearing_cdeg;
        unsigned magnitude = (unsigned)(bearing < 0 ? -bearing : bearing);
        printf("[CAMERA]   id=%u range_mm=%u bearing_deg=%c%u.%02u quality=%s(%u)\n",
               marker->id, marker->range_mm, bearing < 0 ? '-' : '+',
               magnitude / 100, magnitude % 100,
               camera_quality_name(marker->quality), marker->quality);
    }
}

void camera_uart_poll(void)
{
    // Bound work per main-loop tick, even if a sender transmits continuously.
    for (unsigned i = 0; i < CAMERA_POLL_BYTE_LIMIT; ++i) {
        received_byte_t received;
        uint32_t irq_state = save_and_disable_interrupts();
        if (receive_loss) {
            // Do not splice bytes from opposite sides of an overflow/error.
            receive_tail = receive_head;
            receive_loss = false;
            camera_parser_reset(&parser);
        }
        if (receive_tail == receive_head) {
            restore_interrupts(irq_state);
            break;
        }
        received = receive_ring[receive_tail];
        receive_tail = (uint16_t)((receive_tail + 1u) % CAMERA_RX_CAPACITY);
        restore_interrupts(irq_state);

        camera_frame_t frame;
        if (camera_parser_feed(&parser, received.byte, received.receive_ms, &frame)) {
            latest_observation.frame = frame;
            latest_observation.received_ms = received.receive_ms;
            latest_observation.generation++;
            have_observation = true;
            // commented out cuz it spams the serial monitor with prints, uncomment to see the camera data
            // print_frame(&frame, received.receive_ms);
        }
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((uint32_t)(now - last_stats_ms) >= CAMERA_STATS_INTERVAL_MS) {
        uint32_t irq_state = save_and_disable_interrupts();
        uint32_t bytes = received_bytes, drops = dropped_bytes, errors = uart_errors;
        restore_interrupts(irq_state);
        printf("[CAMERA] stats: bytes=%lu frames=%lu crc_errors=%lu format_errors=%lu "
               "gap_resets=%lu dropped_bytes=%lu uart_errors=%lu\n",
               (unsigned long)bytes, (unsigned long)parser.valid_frames,
               (unsigned long)parser.crc_errors, (unsigned long)parser.format_errors,
               (unsigned long)parser.gap_resets, (unsigned long)drops,
               (unsigned long)errors);
        last_stats_ms = now;
    }
}

bool camera_uart_latest(camera_observation_t *observation)
{
    if (!have_observation) return false;
    *observation = latest_observation;
    return true;
}
