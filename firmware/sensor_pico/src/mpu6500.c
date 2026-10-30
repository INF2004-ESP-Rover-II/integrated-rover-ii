#include "mpu6500.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "sensor_config.h"

#define REG_WHO_AM_I     0x75
#define REG_PWR_MGMT_1   0x6B
#define REG_PWR_MGMT_2   0x6C
#define REG_CONFIG       0x1A
#define REG_SMPLRT_DIV   0x19
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_ACCEL_XOUT_H 0x3B
#define I2C_TIMEOUT_US   5000u

static float gyro_bias[3];
static const char *last_error = "not initialized";
static uint8_t device_id;

const char *mpu6500_last_error(void) { return last_error; }
uint8_t mpu6500_device_id(void) { return device_id; }

static bool write_register(uint8_t reg, uint8_t value) {
    const uint8_t bytes[] = {reg, value};
    int result = i2c_write_timeout_us(IMU_I2C_PORT, MPU6500_ADDR, bytes, 2, false,
                                      I2C_TIMEOUT_US);
    if (result != 2) {
        last_error = "configuration register write failed";
        printf("[IMU] Config write failed: addr=0x%02X reg=0x%02X transfer=%d/2\n",
               MPU6500_ADDR, reg, result);
    }
    return result == 2;
}

static bool read_registers(uint8_t reg, uint8_t *bytes, size_t count) {
    if (i2c_write_timeout_us(IMU_I2C_PORT, MPU6500_ADDR, &reg, 1, true,
                             I2C_TIMEOUT_US) != 1) return false;
    return i2c_read_timeout_us(IMU_I2C_PORT, MPU6500_ADDR, bytes, count, false,
                               I2C_TIMEOUT_US) == (int)count;
}

bool mpu6500_init(void) {
    last_error = "WHO_AM_I read failed; check SDA/SCL, power and AD0/address";
    device_id = 0;
    for (int axis = 0; axis < 3; ++axis) gyro_bias[axis] = 0.0f;
    i2c_init(IMU_I2C_PORT, 100000);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    uint8_t who = 0;
    if (!read_registers(REG_WHO_AM_I, &who, 1)) {
        printf("[IMU] WHO_AM_I read failed at 0x%02X on GP%d/GP%d; check bus/address/power\n",
               MPU6500_ADDR, IMU_SDA_PIN, IMU_SCL_PIN);
        return false;
    }
    printf("[IMU] WHO_AM_I at 0x%02X = 0x%02X (expected 0x70)\n", MPU6500_ADDR, who);
    device_id = who;
    if (who != 0x70) {
        last_error = "WHO_AM_I mismatch; device is not the expected MPU6500";
        return false;
    }
    // The IMU may remain powered across Pico resets. Start from a known state.
    if (!write_register(REG_PWR_MGMT_1, 0x80)) return false;
    sleep_ms(100);
    if (!write_register(REG_PWR_MGMT_1, 0x01) ||
        !write_register(REG_PWR_MGMT_2, 0x00)) return false;
    sleep_ms(100);
    // 1 kHz filtered gyro output / (9 + 1) = 100 Hz sensor sampling.
    // Core 1 reads/fuses independently of the main-loop packet cadence.
    bool configured = write_register(REG_CONFIG, 0x03) &&
           write_register(REG_SMPLRT_DIV, 9) &&
           write_register(REG_GYRO_CONFIG, 0x18) &&  // +/-2000 deg/s for fast turns
           write_register(REG_ACCEL_CONFIG, 0x08) && // +/-4 g
           write_register(REG_ACCEL_CONFIG2, 0x03); // Filter accel output too
    if (!configured) return false;

    uint8_t accel_config = 0, gyro_config = 0;
    if (!read_registers(REG_ACCEL_CONFIG, &accel_config, 1) ||
        !read_registers(REG_GYRO_CONFIG, &gyro_config, 1)) {
        last_error = "range configuration readback failed";
        return false;
    }
    printf("[IMU] Range readback: ACCEL_CONFIG=0x%02X GYRO_CONFIG=0x%02X (expected 0x08/0x18)\n",
           accel_config, gyro_config);
    if (accel_config != 0x08 || gyro_config != 0x18) {
        last_error = "range configuration readback mismatch (expected 0x08/0x18)";
        return false;
    }
    // Do not decode an output latched at the previous range with the new scale.
    // At 100 Hz this allows ten sample periods after the final config writes.
    sleep_ms(100);
    last_error = "OK (not yet calibrated)";
    return true;
}

bool mpu6500_read(mpu6500_reading_t *reading) {
    if (reading == NULL) return false;
    uint8_t bytes[14];
    if (!read_registers(REG_ACCEL_XOUT_H, bytes, sizeof(bytes))) return false;
    int16_t raw[7];
    for (int i = 0; i < 7; ++i)
        raw[i] = (int16_t)(((uint16_t)bytes[2 * i] << 8) | bytes[2 * i + 1]);
    for (int axis = 0; axis < 3; ++axis) {
        reading->accel_g[axis] = raw[axis] / 8192.0f;
        reading->gyro_dps[axis] = raw[axis + 4] / 16.4f - gyro_bias[axis];
    }
    return true;
}

// These are calibration quality gates, not sensor accuracy specifications.
#define CAL_SAMPLES 100u
#define CAL_ATTEMPTS 10u
#define CAL_MAX_BIAS_DPS 20.0f
#define CAL_MAX_STD_DPS 0.35f
#define CAL_MAX_GYRO_SPAN_DPS 2.0f
#define CAL_MAX_ACCEL_SPAN_G 0.08f
static char calibration_detail[256];

bool mpu6500_calibrate(void) {
    for (int axis = 0; axis < 3; ++axis) gyro_bias[axis] = 0.0f;
    sleep_ms(1000); // Allow startup handling to settle before sampling windows.
    for (unsigned attempt = 1; attempt <= CAL_ATTEMPTS; ++attempt) {
        float mean[3] = {0}, m2[3] = {0};
        float gyro_min[3] = {0}, gyro_max[3] = {0};
        float accel_min[3] = {0}, accel_max[3] = {0};
        float norm_min = 100.0f, norm_max = 0.0f;
        for (unsigned sample = 0; sample < CAL_SAMPLES; ++sample) {
            mpu6500_reading_t reading;
            if (!mpu6500_read(&reading)) {
                snprintf(calibration_detail, sizeof(calibration_detail),
                         "calibration data transfer failed: window=%u sample=%u",
                         attempt, sample + 1);
                last_error = calibration_detail;
                printf("[IMU] %s\n", last_error);
                return false;
            }
            float norm2 = 0.0f;
            for (int axis = 0; axis < 3; ++axis) {
                float gyro = reading.gyro_dps[axis], accel = reading.accel_g[axis];
                float delta = gyro - mean[axis];
                mean[axis] += delta / (float)(sample + 1);
                m2[axis] += delta * (gyro - mean[axis]);
                if (sample == 0 || gyro < gyro_min[axis]) gyro_min[axis] = gyro;
                if (sample == 0 || gyro > gyro_max[axis]) gyro_max[axis] = gyro;
                if (sample == 0 || accel < accel_min[axis]) accel_min[axis] = accel;
                if (sample == 0 || accel > accel_max[axis]) accel_max[axis] = accel;
                norm2 += accel * accel;
            }
            float norm = sqrtf(norm2);
            if (norm < norm_min) norm_min = norm;
            if (norm > norm_max) norm_max = norm;
            sleep_ms(10);
        }
        float max_std = 0.0f, max_gyro_span = 0.0f, max_accel_span = 0.0f;
        bool bias_plausible = true, above_nominal_bias = false;
        for (int axis = 0; axis < 3; ++axis) {
            float std = sqrtf(fmaxf(m2[axis] / (CAL_SAMPLES - 1), 0.0f));
            max_std = fmaxf(max_std, std);
            max_gyro_span = fmaxf(max_gyro_span, gyro_max[axis] - gyro_min[axis]);
            max_accel_span = fmaxf(max_accel_span, accel_max[axis] - accel_min[axis]);
            if (fabsf(mean[axis]) > CAL_MAX_BIAS_DPS) bias_plausible = false;
            if (fabsf(mean[axis]) > 5.0f) above_nominal_bias = true;
        }
        const char *reason = NULL;
        if (norm_min < 0.85f || norm_max > 1.15f) reason = "acceleration magnitude outside 0.85..1.15 g";
        else if (!bias_plausible) reason = "gyro bias exceeds 20 dps quality limit";
        else if (max_std > CAL_MAX_STD_DPS || max_gyro_span > CAL_MAX_GYRO_SPAN_DPS ||
                 max_accel_span > CAL_MAX_ACCEL_SPAN_G) reason = "motion/noise in calibration window";

        printf("[IMU] Calibration window %u: mean X=%.3f Y=%.3f Z=%.3f dps "
               "max_std=%.3f gyro_span=%.3f accel_span=%.3f norm=%.3f..%.3f g\n",
               attempt, mean[0], mean[1], mean[2], max_std, max_gyro_span,
               max_accel_span, norm_min, norm_max);
        if (reason != NULL) {
            snprintf(calibration_detail, sizeof(calibration_detail),
                     "calibration rejected: %s; mean=%.2f/%.2f/%.2f dps std=%.2f "
                     "gyro_span=%.2f accel_span=%.3f norm=%.2f..%.2f g",
                     reason, mean[0], mean[1], mean[2], max_std, max_gyro_span,
                     max_accel_span, norm_min, norm_max);
            last_error = calibration_detail;
            printf("[IMU] %s%s\n", last_error, attempt < CAL_ATTEMPTS ? "; retrying, keep stationary" : "");
            continue;
        }
        for (int axis = 0; axis < 3; ++axis) gyro_bias[axis] = mean[axis];
        if (above_nominal_bias) {
            last_error = "OK; calibrated gyro bias >5 dps is suspicious; verify stillness and heading";
            printf("[WARN] %s\n", last_error);
        } else {
            last_error = "OK";
        }
        printf("[IMU] Calibration OK: gyro bias X=%.3f Y=%.3f Z=%.3f dps\n",
               gyro_bias[0], gyro_bias[1], gyro_bias[2]);
        return true;
    }
    return false;
}
