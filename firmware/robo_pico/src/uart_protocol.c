#include "uart_protocol.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <string.h>
#include <stdio.h>

// Sync bytes for packet alignment
#define PACKET_SYNC_BYTE1  0xAA
#define PACKET_SYNC_BYTE2  0x55

// UART receiver state
static uart_inst_t* uart_instance = NULL;
static uart_rx_state_t rx_state = UART_SYNC1;
static uint8_t rx_buffer[sizeof(sensor_packet_t)];
static size_t rx_index = 0;

// Double buffer for thread-safe packet access
static sensor_packet_t packet_buffer[2];
static volatile uint8_t write_buffer = 0;
static volatile uint8_t read_buffer = 0;
static volatile bool new_packet_available = false;

// Statistics
static uint32_t total_packets = 0;
static uint32_t valid_packets = 0;
static uint32_t error_packets = 0;

// Forward declarations
static void uart_rx_interrupt_handler(void);

uint8_t calculate_packet_checksum(const sensor_packet_t* packet) {
    uint8_t checksum = 0;
    const uint8_t* data = (const uint8_t*)packet;
    
    // XOR all bytes except the checksum field itself
    for (size_t i = 0; i < sizeof(sensor_packet_t) - 1; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

bool validate_packet_checksum(const sensor_packet_t* packet) {
    return (calculate_packet_checksum(packet) == packet->checksum);
}

void uart_receiver_init(uart_inst_t* uart_id, uint32_t baud_rate, int rx_pin, int tx_pin) {
    uart_instance = uart_id;
    
    // Initialize UART
    uart_init(uart_instance, baud_rate);
    
    // Set up pins
    if (rx_pin >= 0) {
        gpio_set_function(rx_pin, GPIO_FUNC_UART);
    }
    if (tx_pin >= 0) {
        gpio_set_function(tx_pin, GPIO_FUNC_UART);
    }
    
    // Configure UART
    uart_set_format(uart_instance, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_instance, false);
    
    // Set up interrupt
    int uart_irq = (uart_instance == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, uart_rx_interrupt_handler);
    irq_set_enabled(uart_irq, true);
    
    // Enable RX interrupt
    uart_set_irq_enables(uart_instance, true, false);
    
    // Initialize state
    rx_state = UART_SYNC1;
    rx_index = 0;
    new_packet_available = false;
    
    printf("[UART] Receiver initialized at %d baud (RX=GP%d)\n", baud_rate, rx_pin);
}

// UART RX interrupt handler - called when data arrives
static void uart_rx_interrupt_handler(void) {
    while (uart_is_readable(uart_instance)) {
        uint8_t byte = uart_getc(uart_instance);
        
        switch (rx_state) {
            case UART_SYNC1:
                if (byte == PACKET_SYNC_BYTE1) {
                    rx_buffer[0] = byte;
                    rx_index = 1;
                    rx_state = UART_SYNC2;
                }
                break;
                
            case UART_SYNC2:
                if (byte == PACKET_SYNC_BYTE2) {
                    rx_buffer[1] = byte;
                    rx_index = 2;
                    rx_state = UART_DATA;
                } else {
                    // Bad sync, restart
                    rx_state = UART_SYNC1;
                }
                break;
                
            case UART_DATA:
                rx_buffer[rx_index++] = byte;
                
                // Check if we've received a complete packet
                if (rx_index >= sizeof(sensor_packet_t)) {
                    total_packets++;
                    
                    // Validate checksum
                    sensor_packet_t* packet = (sensor_packet_t*)rx_buffer;
                    if (validate_packet_checksum(packet)) {
                        // Copy to write buffer
                        memcpy(&packet_buffer[write_buffer], packet, sizeof(sensor_packet_t));
                        
                        // Swap buffers
                        read_buffer = write_buffer;
                        write_buffer = (write_buffer + 1) % 2;
                        
                        new_packet_available = true;
                        valid_packets++;
                    } else {
                        error_packets++;
                    }
                    
                    // Reset state machine
                    rx_state = UART_SYNC1;
                    rx_index = 0;
                }
                break;
        }
    }
}

bool uart_packet_available(void) {
    return new_packet_available;
}

bool uart_get_packet(sensor_packet_t* packet) {
    if (!new_packet_available) {
        return false;
    }
    
    // Copy from read buffer
    memcpy(packet, &packet_buffer[read_buffer], sizeof(sensor_packet_t));
    
    new_packet_available = false;
    return true;
}

void uart_get_stats(uint32_t* total_received, uint32_t* valid_pkts, uint32_t* error_pkts) {
    if (total_received) *total_received = total_packets;
    if (valid_pkts) *valid_pkts = valid_packets;
    if (error_pkts) *error_pkts = error_packets;
}