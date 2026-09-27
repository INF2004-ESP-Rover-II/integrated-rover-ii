#include "rplidar.h"
#include "sensor_config.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"  // Add this include for interrupt functions
#include <stdio.h>
#include <string.h>

// RPLIDAR A1 Commands
#define RPLIDAR_CMD_SYNC_BYTE           0xA5
#define RPLIDAR_CMD_STOP                0x25
#define RPLIDAR_CMD_RESET               0x40
#define RPLIDAR_CMD_SCAN                0x20

// IRQ-based circular buffer for LIDAR measurements
#define RPLIDAR_IRQ_BUFFER_SIZE 2000  // Buffer capacity: 2000 points (~5-6 full 360° scans)

static struct {
    rplidar_measurement_t points[RPLIDAR_IRQ_BUFFER_SIZE];
    volatile uint32_t head;           // Write index (IRQ context)
    volatile uint32_t tail;           // Read index (main loop)
    volatile uint32_t overflow_count; // Dropped points due to buffer full
    volatile uint32_t total_points;   // Total points received since init
    volatile uint32_t valid_points;   // Points passing quality filter
} lidar_buffer;

// IRQ packet parser state machine
static volatile struct {
    uint8_t packet[5];     // RPLIDAR A1 standard measurement packet (5 bytes)
    uint8_t packet_idx;    // Current byte position in packet
    bool synced;           // Sync bit alignment flag
} irq_state;

// Static functions
static void rplidar_uart_irq_handler(void);
static bool parse_measurement_packet(const uint8_t* packet, rplidar_measurement_t* measurement);  // Add const
static void send_simple_command(uint8_t cmd);

// Send simple command (no payload)
static void send_simple_command(uint8_t cmd) {
    uint8_t packet[2] = {RPLIDAR_CMD_SYNC_BYTE, cmd};
    uart_write_blocking(RPLIDAR_UART_ID, packet, 2);
    printf("[RPLIDAR] Sent: 0xA5 0x%02X\n", cmd);
}

// IRQ Handler - This runs automatically when data arrives
static void rplidar_uart_irq_handler(void) {
    while (uart_is_readable(RPLIDAR_UART_ID)) {
        uint8_t byte = uart_getc(RPLIDAR_UART_ID);
        
        // Sync detection
        if (!irq_state.synced && irq_state.packet_idx == 0) {
            bool s_bit = (byte & 0x01);
            bool s_inv_bit = (byte & 0x02) >> 1;
            
            if (s_bit == s_inv_bit) {
                continue; // Invalid sync, skip
            }
            irq_state.synced = true;
        }
        
        // Store byte in packet buffer
        irq_state.packet[irq_state.packet_idx++] = byte;
        
        // Complete packet received
        if (irq_state.packet_idx >= 5) {
            rplidar_measurement_t measurement;
            
            // Cast away volatile for the function call
            if (parse_measurement_packet((const uint8_t*)irq_state.packet, &measurement)) {
                // Add to circular buffer
                uint32_t next_head = (lidar_buffer.head + 1) % RPLIDAR_IRQ_BUFFER_SIZE;
                
                if (next_head != lidar_buffer.tail) {
                    lidar_buffer.points[lidar_buffer.head] = measurement;
                    lidar_buffer.head = next_head;
                    lidar_buffer.valid_points++;
                } else {
                    lidar_buffer.overflow_count++;
                }
                lidar_buffer.total_points++;
            }
            
            // Reset for next packet
            irq_state.packet_idx = 0;
            irq_state.synced = false;
        }
    }
}

// Fix the function signature to accept const pointer
static bool parse_measurement_packet(const uint8_t* packet, rplidar_measurement_t* measurement) {
    uint8_t quality = packet[0] >> 2;
    bool start = (packet[0] & 0x01);
    bool check_bit = (packet[0] & 0x02) >> 1;
    
    // Validate sync bits
    if (start == check_bit) {
        return false;
    }
    
    // Parse angle
    uint16_t angle_raw = ((uint16_t)packet[2] << 8) | packet[1];
    angle_raw = angle_raw >> 1;
    float angle = (angle_raw / 64.0f);
    
    // Parse distance
    uint16_t distance_raw = ((uint16_t)packet[4] << 8) | packet[3];
    float distance = (distance_raw / 4.0f);
    
    // Validate data
    if (angle > 360.0f || quality > 63) {
        return false;
    }
    
    measurement->angle = angle;
    measurement->distance = distance;
    measurement->quality = quality;
    measurement->start_flag = start;
    
    return true;
}

// IRQ Initialization
void rplidar_irq_init(void) {
    // Initialize circular buffer
    lidar_buffer.head = 0;
    lidar_buffer.tail = 0;
    lidar_buffer.overflow_count = 0;
    lidar_buffer.total_points = 0;
    lidar_buffer.valid_points = 0;
    
    // Initialize IRQ state
    irq_state.packet_idx = 0;
    irq_state.synced = false;
    
    // Set up UART with IRQ
    uart_init(RPLIDAR_UART_ID, RPLIDAR_BAUD_RATE);
    gpio_set_function(RPLIDAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RPLIDAR_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(RPLIDAR_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(RPLIDAR_UART_ID, true);
    
    // Clear any pending interrupts and enable RX interrupt
    uart_set_irq_enables(RPLIDAR_UART_ID, true, false);
    irq_set_exclusive_handler(UART1_IRQ, rplidar_uart_irq_handler);
    irq_set_enabled(UART1_IRQ, true);
    
    printf("[RPLIDAR] IRQ system initialized\n");
}

// Get measurement from IRQ buffer (non-blocking)
bool rplidar_irq_get_measurement(rplidar_measurement_t* measurement) {
    if (!measurement) return false;
    
    // Disable interrupts temporarily for thread-safe access
    uint32_t interrupts = save_and_disable_interrupts();
    
    if (lidar_buffer.head == lidar_buffer.tail) {
        // Buffer empty
        restore_interrupts(interrupts);
        return false;
    }
    
    *measurement = lidar_buffer.points[lidar_buffer.tail];
    lidar_buffer.tail = (lidar_buffer.tail + 1) % RPLIDAR_IRQ_BUFFER_SIZE;
    
    restore_interrupts(interrupts);
    return true;
}

// Get number of available points in buffer
uint32_t rplidar_irq_get_available_points(void) {
    uint32_t interrupts = save_and_disable_interrupts();
    uint32_t available = (lidar_buffer.head >= lidar_buffer.tail) ? 
                        (lidar_buffer.head - lidar_buffer.tail) : 
                        (RPLIDAR_IRQ_BUFFER_SIZE - lidar_buffer.tail + lidar_buffer.head);
    restore_interrupts(interrupts);
    return available;
}

// Clear IRQ buffer
void rplidar_irq_clear_buffer(void) {
    uint32_t interrupts = save_and_disable_interrupts();
    lidar_buffer.head = 0;
    lidar_buffer.tail = 0;
    lidar_buffer.overflow_count = 0;
    lidar_buffer.total_points = 0;
    lidar_buffer.valid_points = 0;
    restore_interrupts(interrupts);
}

// Get buffer statistics
void rplidar_irq_get_buffer_stats(uint32_t* available, uint32_t* overflow, uint32_t* total) {
    uint32_t interrupts = save_and_disable_interrupts();
    
    if (available) {
        *available = (lidar_buffer.head >= lidar_buffer.tail) ? 
                    (lidar_buffer.head - lidar_buffer.tail) : 
                    (RPLIDAR_IRQ_BUFFER_SIZE - lidar_buffer.tail + lidar_buffer.head);
    }
    if (overflow) {
        *overflow = lidar_buffer.overflow_count;
    }
    if (total) {
        *total = lidar_buffer.total_points;
    }
    
    restore_interrupts(interrupts);
}

// Modified existing functions to use IRQ system
bool rplidar_init(void) {
    printf("\n[RPLIDAR] ========================================\n");
    printf("[RPLIDAR] Initializing RPLIDAR A1 with IRQ\n");
    printf("[RPLIDAR] ========================================\n");
    
    if (RPLIDAR_UART_ID != uart1) {
        printf("[RPLIDAR] ERROR: Must use UART1 for GP8/GP9\n");
        return false;
    }
    
    printf("[RPLIDAR] UART: uart1\n");
    printf("[RPLIDAR] TX Pin: GP%d → RPLIDAR RX (White)\n", RPLIDAR_TX_PIN);
    printf("[RPLIDAR] RX Pin: GP%d ← RPLIDAR TX (Green)\n", RPLIDAR_RX_PIN);
    printf("[RPLIDAR] Baud: %d\n", RPLIDAR_BAUD_RATE);
    
    // Initialize IRQ system
    rplidar_irq_init();
    
    // Wait for motor
    printf("[RPLIDAR] Waiting for motor (3 seconds)...\n");
    sleep_ms(3000);
    
    // Stop any ongoing operation
    printf("[RPLIDAR] Sending STOP...\n");
    send_simple_command(RPLIDAR_CMD_STOP);
    sleep_ms(100);
    
    // Clear buffer
    rplidar_irq_clear_buffer();
    
    // Reset device
    printf("[RPLIDAR] Sending RESET...\n");
    send_simple_command(RPLIDAR_CMD_RESET);
    sleep_ms(2000);
    
    // Clear buffer after reset
    rplidar_irq_clear_buffer();
    
    printf("[RPLIDAR] IRQ-based initialization complete\n");
    printf("[RPLIDAR] ========================================\n\n");
    
    return true;
}

bool rplidar_start_scan(void) {
    printf("[RPLIDAR] Starting scan mode...\n");
    
    // Clear buffer before starting
    rplidar_irq_clear_buffer();
    
    // Send SCAN command
    printf("[RPLIDAR] Sending SCAN command...\n");
    send_simple_command(RPLIDAR_CMD_SCAN);
    
    printf("[RPLIDAR] ✓ Scan started (IRQ mode)\n");
    printf("[RPLIDAR] Ready to receive measurement data via interrupts\n\n");
    
    return true;
}

bool rplidar_stop_scan(void) {
    printf("[RPLIDAR] Stopping scan...\n");
    send_simple_command(RPLIDAR_CMD_STOP);
    sleep_ms(100);
    
    printf("[RPLIDAR] Scan stopped\n");
    return true;
}

// Use IRQ system for measurement polling
bool rplidar_get_measurement(rplidar_measurement_t* measurement) {
    return rplidar_irq_get_measurement(measurement);
}

void rplidar_get_stats(uint32_t* points_per_scan, uint32_t* valid_pts, uint32_t* invalid_pts) {
    uint32_t interrupts = save_and_disable_interrupts();
    
    if (valid_pts) {
        *valid_pts = lidar_buffer.valid_points;
    }
    if (invalid_pts) {
        *invalid_pts = lidar_buffer.total_points - lidar_buffer.valid_points;
    }
    if (points_per_scan) {
        // Simple approximation
        *points_per_scan = (lidar_buffer.valid_points > 0) ? 360 : 0;
    }
    
    restore_interrupts(interrupts);
}