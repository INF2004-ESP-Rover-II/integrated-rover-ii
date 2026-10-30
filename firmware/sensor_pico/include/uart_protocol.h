#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H
#define PACKET_SYNC_BYTE1 0xAA
#define PACKET_SYNC_BYTE2 0x55

#include <stdint.h>
#include <stdbool.h>

// Sensor data packet structure (sent from Sensor Pico to Robo Pico)
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
    
    // Ultrasonic distances (cm)
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
    // Change made for Tribunal 1: Combine all blocked, urgent and stop into two bytes (10 bits needed)
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
 * @brief Send sensor packet via UART
 * @param packet Pointer to packet to send
 */
void send_sensor_packet(const sensor_packet_t* packet);

#endif // UART_PROTOCOL_H
