#ifndef MPU6500_H
#define MPU6500_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
} mpu6500_reading_t;

// Uses IMU_I2C_PORT / IMU_SDA_PIN / IMU_SCL_PIN from sensor_config.h.
bool mpu6500_init(void);
bool mpu6500_read(mpu6500_reading_t *reading);
// Keep stationary: accept a stable 100-sample window, retry up to ten times.
// Its gyro mean is subtracted by read(); constant yaw cannot be distinguished
// from bias. Suspicious offsets are reported by mpu6500_last_error().
bool mpu6500_calibrate(void);
// Retained startup diagnostics, available even if USB attached after init.
const char *mpu6500_last_error(void);
uint8_t mpu6500_device_id(void);

#endif
