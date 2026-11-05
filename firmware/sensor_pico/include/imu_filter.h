#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "mpu6500.h"

typedef struct {
    float pitch_deg;
    float roll_deg;
    float heading_deg;
    float startup_up[3]; // Unit gravity direction in sensor axes at startup.
    uint64_t last_sample_us;
    bool seeded;
    bool timing_fault;
} imu_filter_t;

// Zero-initialize state before use. Heading is relative to startup, clockwise
// positive for yaw_sign = -1. Yaw projects all three gyro axes onto the
// startup gravity direction, so the sensor can be mounted on its side.
// Keep the rover level and stationary at startup; pitch/roll use sensor axes.
// A gap above 500 ms latches a fault: lost rotation cannot be reconstructed.
bool imu_filter_update(imu_filter_t *state, const mpu6500_reading_t *reading,
                       uint64_t now_us, float yaw_sign);

#endif
