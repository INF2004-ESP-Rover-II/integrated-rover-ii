#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H
#define PACKET_SYNC_BYTE1 0xAA
#define PACKET_SYNC_BYTE2 0x55
#define MAX_LIDAR_POINTS  100

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

    // LiDAR data
    uint8_t lidar_count;  // Number of LiDAR points (<= MAX_LIDAR_POINTS)
    struct {
        float angle;      // degrees
        float distance;   // mm
    } lidar_points[MAX_LIDAR_POINTS];
    
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