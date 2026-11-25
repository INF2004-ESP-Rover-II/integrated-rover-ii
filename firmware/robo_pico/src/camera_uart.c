/**
 * @file camera_uart.c
 * @brief Camera uart firmware module.
 */
#ifndef CAMERA_UART_PRINT_FRAMES
#define CAMERA_UART_PRINT_FRAMES 0
#endif

#include "camera_uart.h"
#include "camera_protocol.h"
#include <inttypes.h>

#include "hardware/irq.h"
#include "hardware/regs/uart.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define CAMERA_UART uart1
#define CAMERA_RX_PIN 21u
#define CAMERA_BAUD 115200u
#define CAMERA_RX_CAPACITY 512u
#define CAMERA_POLL_BYTE_LIMIT 128u
#define CAMERA_STATS_INTERVAL_MS 5000u

typedef struct
{
    uint32_t receive_ms;
    uint8_t  byte;
} received_byte_t;

static received_byte_t      g_receive_ring[CAMERA_RX_CAPACITY];
static volatile uint16_t    g_receive_head;
static volatile uint16_t    g_receive_tail;
static volatile bool        g_b_receive_loss;
static volatile uint32_t    g_received_bytes;
static volatile uint32_t    g_dropped_bytes;
static volatile uint32_t    g_uart_errors;
static camera_parser_t      g_parser;
static uint32_t             g_last_stats_ms;
static camera_observation_t g_latest_observation;
static bool                 g_b_have_observation;

static void
camera_uart_isr (void)
{
    // Only buffer bytes in the ISR. In particular, never printf here.
    while (uart_is_readable(CAMERA_UART))
    {
        uint32_t data = uart_get_hw(CAMERA_UART)->dr;
        g_received_bytes++;
        if (data & (UART_UARTDR_OE_BITS | UART_UARTDR_BE_BITS |
                    UART_UARTDR_PE_BITS | UART_UARTDR_FE_BITS))
        {
            g_uart_errors++;
            g_b_receive_loss              = true;
            uart_get_hw(CAMERA_UART)->rsr = 0; // Clear sticky UART error flags.
        }
        uint16_t next = (uint16_t) ((g_receive_head + 1u) % CAMERA_RX_CAPACITY);
        if (g_b_receive_loss || next == g_receive_tail)
        {
            g_b_receive_loss = true;
            g_dropped_bytes++;
            continue;
        }
        g_receive_ring[g_receive_head].byte = (uint8_t) data;

        g_receive_ring[g_receive_head].receive_ms =
            to_ms_since_boot(get_absolute_time());

        g_receive_head = next;
    }
}

void
camera_uart_init (void)
{
    camera_parser_init(&g_parser);
    g_b_have_observation            = false;
    g_latest_observation.generation = 0;
    g_receive_head = g_receive_tail = 0;
    g_b_receive_loss                = false;
    g_received_bytes = g_dropped_bytes = g_uart_errors = 0;
    g_last_stats_ms = to_ms_since_boot(get_absolute_time());
    uart_init(CAMERA_UART, CAMERA_BAUD);
    gpio_set_function(CAMERA_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(CAMERA_RX_PIN); // UART idle level when the cable is absent.
    uart_set_format(CAMERA_UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(CAMERA_UART, false, false);
    uart_set_fifo_enabled(CAMERA_UART, true);
    irq_set_exclusive_handler(UART1_IRQ, camera_uart_isr);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(CAMERA_UART, true, false);
    printf("[CAMERA] UART1 RX=GP%u, %u baud 8N1; ESP32 GPIO17 -> GP21, common "
           "GND\n",
           CAMERA_RX_PIN, CAMERA_BAUD);
}

#if CAMERA_UART_PRINT_FRAMES
static void
print_frame (const camera_frame_t * p_frame, uint32_t receive_ms)
{
    uint32_t queue_age_ms = to_ms_since_boot(get_absolute_time()) - receive_ms;
    printf(
        "[CAMERA] seq=%u status=%s(%u) age_ms=%u markers=%u queue_ms=%" PRIu32
        "\n",
        p_frame->seq, camera_status_name(p_frame->status), p_frame->status,
        p_frame->age_ms, p_frame->count, (uint32_t) queue_age_ms);
    for (uint8_t index = 0; index < p_frame->count; ++index)
    {
        const camera_marker_t * p_marker = &p_frame->markers[index];
        int                     bearing  = p_marker->bearing_cdeg;
        unsigned magnitude = (unsigned) (bearing < 0 ? -bearing : bearing);
        printf("[CAMERA]   id=%u range_mm=%u bearing_deg=%c%u.%02u "
               "quality=%s(%u)\n",
               p_marker->id, p_marker->range_mm, bearing < 0 ? '-' : '+',
               magnitude / 100, magnitude % 100,
               camera_quality_name(p_marker->quality), p_marker->quality);
    }
}

#endif

void
camera_uart_poll (void)
{
    // Bound work per main-loop tick, even if a sender transmits continuously.
    for (unsigned index = 0; index < CAMERA_POLL_BYTE_LIMIT; ++index)
    {
        received_byte_t received;
        uint32_t        irq_state = save_and_disable_interrupts();
        if (g_b_receive_loss)
        {
            // Do not splice bytes from opposite sides of an overflow/error.
            g_receive_tail   = g_receive_head;
            g_b_receive_loss = false;
            camera_parser_reset(&g_parser);
        }
        if (g_receive_tail == g_receive_head)
        {
            restore_interrupts(irq_state);
            break;
        }
        received = g_receive_ring[g_receive_tail];

        g_receive_tail =
            (uint16_t) ((g_receive_tail + 1u) % CAMERA_RX_CAPACITY);

        restore_interrupts(irq_state);

        camera_frame_t frame;
        if (camera_parser_feed(&g_parser, received.byte, received.receive_ms,
                               &frame))
        {
            g_latest_observation.frame       = frame;
            g_latest_observation.received_ms = received.receive_ms;
            g_latest_observation.generation++;
            g_b_have_observation = true;
            // commented out cuz it spams the serial monitor with prints,
            // uncomment to see the camera data print_frame(&frame,
            // received.receive_ms);
        }
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((uint32_t) (now - g_last_stats_ms) >= CAMERA_STATS_INTERVAL_MS)
    {
        uint32_t irq_state = save_and_disable_interrupts();
        uint32_t bytes     = g_received_bytes;
        uint32_t drops     = g_dropped_bytes;
        uint32_t errors    = g_uart_errors;
        restore_interrupts(irq_state);
        printf("[CAMERA] stats: bytes=%" PRIu32 " frames=%" PRIu32
               " crc_errors=%" PRIu32 " format_errors=%" PRIu32 " "
               "gap_resets=%" PRIu32 " dropped_bytes=%" PRIu32
               " uart_errors=%" PRIu32 "\n",
               (uint32_t) bytes, (uint32_t) g_parser.valid_frames,
               (uint32_t) g_parser.crc_errors,
               (uint32_t) g_parser.format_errors,
               (uint32_t) g_parser.gap_resets, (uint32_t) drops,
               (uint32_t) errors);
        g_last_stats_ms = now;
    }
}

bool
camera_uart_latest (camera_observation_t * p_observation)
{
    if (!g_b_have_observation)
    {
        return false;
    }
    *p_observation = g_latest_observation;
    return true;
}

/* End of file. */
