#ifndef ACCELERO_H
#define ACCELERO_H

#include <stdint.h>
#include <stdbool.h>

// Note: I2C_PORT, SDA_PIN, SCL_PIN are defined in sensor_config.h
#ifndef I2C_PORT
#include "sensor_config.h"
#endif

// Device addresses
#define LSM303_ACCEL_ADDR 0x19
#define LSM303_MAG_ADDR   0x1E

// Accelerometer registers
#define CTRL_REG1_A 0x20
#define OUT_X_L_A   0x28

// Magnetometer registers
#define CRA_REG_M   0x00    // Control Register A
#define CRB_REG_M   0x01    // Control Register B
#define MR_REG_M    0x02    // Mode Register
#define OUT_X_H_M   0x03    // Mag data starts here (X high byte)

// Settings
#define ACCEL_SCALE 16384.0f
#define ACCEL_THRESHOLD 0.2f
#define ACCEL_TIMEOUT_MS 50  // max wait for I2C read

// Magnetometer gain settings (±1.3 gauss default)
#define MAG_GAIN_1_3  0x20  // ±1.3 gauss
#define MAG_GAIN_1_9  0x40  // ±1.9 gauss
#define MAG_GAIN_2_5  0x60  // ±2.5 gauss
#define MAG_GAIN_4_0  0x80  // ±4.0 gauss
#define MAG_GAIN_4_7  0xA0  // ±4.7 gauss
#define MAG_GAIN_5_6  0xC0  // ±5.6 gauss
#define MAG_GAIN_8_1  0xE0  // ±8.1 gauss

// Magnetometer scale factor (LSB/gauss for ±1.3 gauss range)
#define MAG_SCALE_XY  1100.0f  // XY axes
#define MAG_SCALE_Z   980.0f   // Z axis

// Return codes
typedef enum {
    ACCEL_OK = 0,
    ACCEL_EINVAL,
    ACCEL_EBUSY,
    ACCEL_ETIMEOUT,
    ACCEL_EHW
} accel_status_t;

// Functions
accel_status_t accel_init(void);
accel_status_t read_accel(float *ax, float *ay, float *az);
accel_status_t compute_tilt(float ax, float ay, float az, float *roll, float *pitch);

// Magnetometer functions
accel_status_t mag_init(void);
accel_status_t read_mag(float *mx, float *my, float *mz);
accel_status_t compute_heading(float mx, float my, float mz, float *heading);

// Combined heading calculation (compensated for tilt)
accel_status_t compute_tilt_compensated_heading(
    float ax, float ay, float az,
    float mx, float my, float mz,
    float* heading_deg
);

#endif // ACCELERO_H