#include "accelero.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdint.h>

#define IMU_HEADING_OFFSET 52.5f  // Heading calibration offset (52.5°) - adjust to align robot forward with 0°

static bool accel_busy = false;
static bool mag_initialized = false;

// Write a single byte to a register
static accel_status_t i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    int ret = i2c_write_blocking(I2C_PORT, addr, (uint8_t[]){reg, value}, 2, false);
    return (ret == 2) ? ACCEL_OK : ACCEL_EHW;
}

// Read multiple bytes from consecutive registers with timeout
static accel_status_t i2c_read_registers(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    absolute_time_t start = get_absolute_time();

    while (true) {
        int ret = i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
        if (ret == 1) break;
        if (absolute_time_diff_us(start, get_absolute_time()) / 1000 > ACCEL_TIMEOUT_MS) {
            return ACCEL_ETIMEOUT;
        }
        sleep_ms(1);
    }

    start = get_absolute_time();
    while (true) {
        int ret = i2c_read_blocking(I2C_PORT, addr, buf, len, false);
        if (ret == len) return ACCEL_OK;
        if (absolute_time_diff_us(start, get_absolute_time()) / 1000 > ACCEL_TIMEOUT_MS) {
            return ACCEL_ETIMEOUT;
        }
        sleep_ms(1);
    }
}

// Read one axis from accelerometer
static accel_status_t read_accel_axis(uint8_t low_reg, int16_t *val) {
    uint8_t buf[2];
    accel_status_t status = i2c_read_registers(LSM303_ACCEL_ADDR, low_reg | 0x80, buf, 2);
    if (status != ACCEL_OK) return status;

    *val = (int16_t)(buf[0] | (buf[1] << 8)) >> 4; // 12-bit signed
    return ACCEL_OK;
}

// Initialize accelerometer and I2C
accel_status_t accel_init(void) {
    if (accel_busy) return ACCEL_EBUSY;
    accel_busy = true;

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Enable accelerometer: 50Hz, normal mode, all axes enabled
    accel_status_t status = i2c_write_register(LSM303_ACCEL_ADDR, CTRL_REG1_A, 0x27);

    accel_busy = false;
    return status;
}

// Read accelerometer axes in g
accel_status_t read_accel(float *ax, float *ay, float *az) {
    if (!ax || !ay || !az) return ACCEL_EINVAL;
    if (accel_busy) return ACCEL_EBUSY;
    accel_busy = true;

    int16_t x, y, z;
    accel_status_t status;

    // Read X
    status = read_accel_axis(OUT_X_L_A, &x);
    if (status != ACCEL_OK) goto cleanup;

    // Read Y
    status = read_accel_axis(OUT_X_L_A + 2, &y);
    if (status != ACCEL_OK) goto cleanup;

    // Read Z
    status = read_accel_axis(OUT_X_L_A + 4, &z);
    if (status != ACCEL_OK) goto cleanup;

    *ax = x / ACCEL_SCALE;
    *ay = y / ACCEL_SCALE;
    *az = z / ACCEL_SCALE;

cleanup:
    accel_busy = false;
    return status;
}

// Compute roll and pitch
accel_status_t compute_tilt(float ax, float ay, float az, float *roll, float *pitch) {
    if (!roll || !pitch) return ACCEL_EINVAL;

    *roll  = atan2(ay, az) * 180.0f / M_PI;
    *pitch = -atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;

    return ACCEL_OK;
}

// ==================== MAGNETOMETER FUNCTIONS ====================

// Initialize magnetometer
accel_status_t mag_init(void) {
    if (accel_busy) return ACCEL_EBUSY;
    accel_busy = true;

    accel_status_t status;

    // CRA_REG_M: 15Hz output rate, normal measurement mode
    status = i2c_write_register(LSM303_MAG_ADDR, CRA_REG_M, 0x10);
    if (status != ACCEL_OK) goto cleanup;

    // CRB_REG_M: Gain = ±1.3 gauss
    status = i2c_write_register(LSM303_MAG_ADDR, CRB_REG_M, MAG_GAIN_1_3);
    if (status != ACCEL_OK) goto cleanup;

    // MR_REG_M: Continuous conversion mode
    status = i2c_write_register(LSM303_MAG_ADDR, MR_REG_M, 0x00);
    if (status != ACCEL_OK) goto cleanup;

    mag_initialized = true;

cleanup:
    accel_busy = false;
    return status;
}

// Read magnetometer axes in gauss
accel_status_t read_mag(float *mx, float *my, float *mz) {
    if (!mx || !my || !mz) return ACCEL_EINVAL;
    if (!mag_initialized) return ACCEL_EHW;
    if (accel_busy) return ACCEL_EBUSY;
    accel_busy = true;

    uint8_t buf[6];
    accel_status_t status;

    // Read all 6 bytes (X, Z, Y order in LSM303DLHC!)
    status = i2c_read_registers(LSM303_MAG_ADDR, OUT_X_H_M, buf, 6);
    if (status != ACCEL_OK) goto cleanup;

    // LSM303DLHC magnetometer data is in X, Z, Y order (big-endian)
    int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t z = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t y = (int16_t)((buf[4] << 8) | buf[5]);

    // Convert to gauss
    *mx = x / MAG_SCALE_XY;
    *my = y / MAG_SCALE_XY;
    *mz = z / MAG_SCALE_Z;

cleanup:
    accel_busy = false;
    return status;
}

// Compute heading from magnetometer (0-360 degrees)
accel_status_t compute_heading(float mx, float my, float mz, float *heading) {
    if (!heading) return ACCEL_EINVAL;

    // Simple 2D heading (not tilt-compensated)
    float heading_rad = atan2(my, mx);
    
    // Convert to degrees
    *heading = heading_rad * 180.0f / M_PI;
    
    // Normalize to 0-360
    if (*heading < 0) {
        *heading += 360.0f;
    }

    return ACCEL_OK;
}

// Compute tilt-compensated heading
accel_status_t compute_tilt_compensated_heading(
    float ax, float ay, float az,
    float mx, float my, float mz,
    float* heading_deg
) {
    if (!heading_deg) return ACCEL_EINVAL;

    // Calculate roll and pitch
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay*ay + az*az));

    // Tilt compensation
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);

    // Compensate magnetometer readings
    float mx_comp = mx * cos_pitch + mz * sin_pitch;
    float my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

    // Calculate heading
    float heading = atan2f(my_comp, mx_comp);
    
    // Convert to degrees
    *heading_deg = heading * 180.0f / M_PI;
    
    // Make sure heading is positive 0-360
    if (*heading_deg < 0) {
        *heading_deg += 360.0f;
    }

    // Apply calibration offset
    *heading_deg = fmodf(*heading_deg - IMU_HEADING_OFFSET + 360.0f, 360.0f);

    return ACCEL_OK;
}