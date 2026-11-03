#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H
#define PACKET_SYNC_BYTE1 0xAA
#define PACKET_SYNC_BYTE2 0x55

#include <stdint.h>
#include <stdbool.h>
#include "hardware/uart.h"

// Sensor data packet structure (must match Sensor Pico!)
typedef struct {
    // Sync bytes for frame alignment
    uint8_t sync1;              // 0xAA
    uint8_t sync2;              // 0x55
    
    // Timestamp
    uint32_t timestamp_ms;      // Milliseconds since boot
    
    // IMU - Accelerometer (m/s²)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // IMU - Gyroscope (deg/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // MPU6500 relative heading (degrees, 0-360 clockwise from startup)
    float heading;
    float pitch;               // Degrees, +X forward/+Z up: negative nose-up
    float roll;                // Degrees, rotation about +X
    
    // Ultrasonic distances (cm, 65535 = offline/disconnected)
    uint16_t ultrasonic_front;
    uint16_t ultrasonic_back;
    uint16_t ultrasonic_left;
    uint16_t ultrasonic_right;
    
    // Status flags
    uint8_t imu_status;         // 0=OK, 1=ERROR
    uint8_t ultrasonic_status;  // Bitmask: bit0=front, bit1=back, bit2=left, bit3=right

    // ToF obstacle snapshot. Direction bits: left=1, front=2, right=4.
    // Skip 0 (unavailable) and 1023 (no target) when using numerical distances.
    // Left/front/right mm; 0 unavailable, 1023 no target.
    // uint16_t tof_mm[3];         
    // Change requirement for Tribunal 1, minimise the number of bits needed
    // Send all 3 readings in a single field, each reading uses 10 bits
    uint32_t tof_bits;
    
    // uint8_t blocked_mask;
    // uint8_t urgent_mask;
    // uint8_t fault_mask;         // Now packed: offline/accepted-result timeout.
    // uint8_t stop_required;      // 0 or 1; a status flag, not a motor command.
    // Blocked, Urgent and Fault masks need only 3 bits each (1 for each ToF)
    // stop_required only needs 1 bit
    /* 
    Change made for Tribunal 1: Combine all blocked, urgent and stop into two bytes (10 bits needed)
        bits 0-2: blocked left/front/right
        bits 3-5: urgent left/front/right
        bits 6-8: fault left/front/right
        bit 9: stop request
        bits 10-15: spare
    */
    
    uint16_t obstacle_flags;
    
    // Checksum (XOR of all bytes before checksum)
    uint8_t checksum;
    
} __attribute__((packed)) sensor_packet_t;

// UART receiver state
typedef enum {
    UART_SYNC1,     // Waiting for 0xAA
    UART_SYNC2,     // Waiting for 0x55
    UART_DATA       // Receiving packet data
} uart_rx_state_t;

// Invoked in RX interrupt context for every checksum-valid frame, before the
// latest-packet mailbox can overwrite it. Must be short, nonblocking, and not log.
typedef void (*uart_packet_callback_t)(const sensor_packet_t *packet, uint32_t receive_ms);
void uart_set_packet_callback(uart_packet_callback_t callback);

/**
 * @brief Initialize UART receiver
 * @param uart_id UART instance (uart0 or uart1)
 * @param baud_rate Baud rate (must match Sensor Pico)
 * @param rx_pin RX GPIO pin
 * @param tx_pin TX GPIO pin (optional, set to -1 if not used)
 */
void uart_receiver_init(uart_inst_t* uart_id, uint32_t baud_rate, int rx_pin, int tx_pin);

/**
 * @brief Check if new sensor packet is available
 * @return true if valid packet received, false otherwise
 */
bool uart_packet_available(void);

/**
 * @brief Get latest sensor packet
 * @param packet Pointer to structure to fill with packet data
 * @return true if valid packet copied, false if no new data
 */
bool uart_get_packet(sensor_packet_t* packet);

/**
 * @brief Calculate checksum for sensor packet
 * @param packet Pointer to packet (checksum field will be ignored)
 * @return Calculated checksum value
 */
uint8_t calculate_packet_checksum(const sensor_packet_t* packet);

/**
 * @brief Validate packet checksum
 * @param packet Pointer to received packet
 * @return true if checksum is valid, false otherwise
 */
bool validate_packet_checksum(const sensor_packet_t* packet);

/**
 * @brief Get packet reception statistics
 * @param total_received Total packets received
 * @param valid_packets Valid packets (good checksum)
 * @param error_packets Error packets (bad checksum)
 */
void uart_get_stats(uint32_t* total_received, uint32_t* valid_packets, uint32_t* error_packets);

#endif // UART_PROTOCOL_H
