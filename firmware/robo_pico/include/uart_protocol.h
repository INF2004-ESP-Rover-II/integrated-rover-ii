#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H
#define PACKET_SYNC_BYTE1 0xAA
#define PACKET_SYNC_BYTE2 0x55
#define MAX_LIDAR_POINTS  100

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
    
    // IMU - Magnetometer (gauss)
    float mag_x;
    float mag_y;
    float mag_z;
    
    // Calculated heading (degrees, 0-360)
    float heading;
    float pitch;
    
    // Ultrasonic distances (cm, 65535 = offline/disconnected)
    uint16_t ultrasonic_front;
    uint16_t ultrasonic_back;
    uint16_t ultrasonic_left;
    uint16_t ultrasonic_right;
    
    // Status flags
    uint8_t imu_status;         // 0=OK, 1=ERROR
    uint8_t ultrasonic_status;  // Bitmask: bit0=front, bit1=back, bit2=left, bit3=right

    // ToF obstacle snapshot. Direction bits: left=1, front=2, right=4.
    // Skip 0 (unavailable) and 65535 (no target) when using numerical distances.
    uint16_t tof_mm[3];         // Left/front/right mm; 0 unavailable, 65535 no target.
    uint8_t blocked_mask;
    uint8_t urgent_mask;
    uint8_t fault_mask;         // Reserved; Sensor Pico sends zero.
    uint8_t stop_required;      // 0 or 1; a status flag, not a motor command.

    // LiDAR data
    uint8_t lidar_count;  // Number of LiDAR points (<= MAX_LIDAR_POINTS)
    struct {
        float angle;      // degrees
        float distance;   // mm
    } lidar_points[MAX_LIDAR_POINTS];
    
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
