#include <inttypes.h>
/**
 * @file uart_protocol.c
 * @brief Uart protocol firmware module.
 */
#include "uart_protocol.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include <string.h>
#include <stdio.h>

// Sync bytes for packet alignment
#define PACKET_SYNC_BYTE1 0xAA
#define PACKET_SYNC_BYTE2 0x55

// UART receiver state
static uart_inst_t *          gp_uart_instance = NULL;
static uart_rx_state_t        g_rx_state       = UART_SYNC1;
static uint8_t                g_rx_buffer[sizeof(sensor_packet_t)];
static size_t                 g_rx_index         = 0;
static uart_packet_callback_t gp_packet_callback = NULL;

// Double buffer for thread-safe packet access
static sensor_packet_t  g_packet_buffer[2];
static volatile uint8_t g_write_buffer           = 0;
static volatile uint8_t g_read_buffer            = 0;
static volatile bool    g_b_new_packet_available = false;

// Statistics
static uint32_t g_total_packets = 0;
static uint32_t g_valid_packets = 0;
static uint32_t g_error_packets = 0;

// Forward declarations

static void uart_rx_interrupt_handler (void);


uint8_t
calculate_packet_checksum (const sensor_packet_t * p_packet)
{
    uint8_t         checksum = 0;
    const uint8_t * p_data   = (const uint8_t *) p_packet;

    // XOR all bytes except the checksum field itself
    for (size_t index = 0; index < sizeof(sensor_packet_t) - 1; index++)
    {
        checksum ^= (uint32_t) (p_data[index]);
    }

    return checksum;
}

bool
validate_packet_checksum (const sensor_packet_t * p_packet)
{
    return (calculate_packet_checksum(p_packet) == p_packet->checksum);
}

void
uart_set_packet_callback (uart_packet_callback_t p_callback)
{
    uint32_t irq_state = save_and_disable_interrupts();
    gp_packet_callback = p_callback;
    restore_interrupts(irq_state);
}

void
uart_receiver_init (uart_inst_t * p_uart_id, uint32_t baud_rate, int rx_pin,
                    int tx_pin)
{
    gp_uart_instance = p_uart_id;

    // Initialize UART
    uart_init(gp_uart_instance, baud_rate);

    // Set up pins
    if (rx_pin >= 0)
    {
        gpio_set_function(rx_pin, GPIO_FUNC_UART);
    }
    if (tx_pin >= 0)
    {
        gpio_set_function(tx_pin, GPIO_FUNC_UART);
    }

    // Configure UART
    uart_set_format(gp_uart_instance, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(gp_uart_instance, true);

    // Initialize the mailbox and parser before enabling reception interrupts.
    g_rx_state               = UART_SYNC1;
    g_rx_index               = 0;
    g_b_new_packet_available = false;

    // Set up interrupt
    int uart_irq = (gp_uart_instance == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, uart_rx_interrupt_handler);
    irq_set_enabled(uart_irq, true);

    // Enable RX interrupt
    uart_set_irq_enables(gp_uart_instance, true, false);

    printf("[UART] Receiver initialized at %" PRIu32 " baud (RX=GP%d)\n",
           baud_rate, rx_pin);
}

// UART RX interrupt handler - called when data arrives
static void
uart_rx_interrupt_handler (void)
{
    while (uart_is_readable(gp_uart_instance))
    {
        uint8_t byte = uart_getc(gp_uart_instance);

        switch (g_rx_state)
        {
            case UART_SYNC1:
                if (byte == PACKET_SYNC_BYTE1)
                {
                    g_rx_buffer[0] = byte;
                    g_rx_index     = 1;
                    g_rx_state     = UART_SYNC2;
                }
                break;

            case UART_SYNC2:
                if (byte == PACKET_SYNC_BYTE2)
                {
                    g_rx_buffer[1] = byte;
                    g_rx_index     = 2;
                    g_rx_state     = UART_DATA;
                }
                else
                {
                    // Bad sync, restart
                    g_rx_state = UART_SYNC1;
                }
                break;

            case UART_DATA:
                g_rx_buffer[g_rx_index++] = byte;

                // Check if we've received a complete packet
                if (g_rx_index >= sizeof(sensor_packet_t))
                {
                    g_total_packets++;

                    // Validate checksum

                    sensor_packet_t * p_packet =
                        (sensor_packet_t *) g_rx_buffer;

                    if (validate_packet_checksum(p_packet))
                    {
                        if (gp_packet_callback != NULL)
                        {
                            gp_packet_callback(
                                p_packet,
                                to_ms_since_boot(get_absolute_time()));
                        }
                        // Copy to write buffer
                        memcpy(&g_packet_buffer[g_write_buffer], p_packet,
                               sizeof(sensor_packet_t));

                        // Swap buffers
                        g_read_buffer  = g_write_buffer;
                        g_write_buffer = (g_write_buffer + 1) % 2;

                        g_b_new_packet_available = true;
                        g_valid_packets++;
                    }
                    else
                    {
                        g_error_packets++;
                    }

                    // Reset state machine
                    g_rx_state = UART_SYNC1;
                    g_rx_index = 0;
                }
                break;
            default:
                /* All defined receiver states are handled above. */
                break;
        }
    }
}

bool
uart_packet_available (void)
{
    return g_b_new_packet_available;
}

bool
uart_get_packet (sensor_packet_t * p_packet)
{
    uint32_t irq_state = save_and_disable_interrupts();
    if (!g_b_new_packet_available)
    {
        restore_interrupts(irq_state);
        return false;
    }

    // Copy from read buffer
    memcpy(p_packet, &g_packet_buffer[g_read_buffer], sizeof(sensor_packet_t));

    g_b_new_packet_available = false;
    restore_interrupts(irq_state);
    return true;
}

void
uart_get_stats (uint32_t * p_total_received, uint32_t * p_valid_pkts,
                uint32_t * p_error_pkts)
{
    if (p_total_received)
    {
        *p_total_received = g_total_packets;
    }
    if (p_valid_pkts)
    {
        *p_valid_pkts = g_valid_packets;
    }
    if (p_error_pkts)
    {
        *p_error_pkts = g_error_packets;
    }
}

/* End of file. */
