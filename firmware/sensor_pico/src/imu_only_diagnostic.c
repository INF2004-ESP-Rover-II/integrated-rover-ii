// Standalone Sensor Pico diagnostic. Only I2C1 IMU + USB serial are used.
// No calibration, heading, UART telemetry, motors, ToF, ultrasonic or SD setup.
#include <math.h>
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "mpu6500.h"
#include "sensor_config.h"

static bool read_config_byte(uint8_t reg, uint8_t *value) {
    if (i2c_write_timeout_us(IMU_I2C_PORT, MPU6500_ADDR, &reg, 1, true, 5000) != 1)
        return false;
    return i2c_read_timeout_us(IMU_I2C_PORT, MPU6500_ADDR, value, 1, false, 5000) == 1;
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("\n[IMU ONLY DIAG v1] SDA=GP%d SCL=GP%d addr=0x%02X\n",
           IMU_SDA_PIN, IMU_SCL_PIN, MPU6500_ADDR);
    while (!mpu6500_init()) {
        printf("[IMU ONLY] Init failed: %s; retrying\n", mpu6500_last_error());
        sleep_ms(1000);
    }
    printf("[IMU ONLY] Uncalibrated readings: rest sensor on table, leave untouched\n");
    uint32_t window = 0;
    while (true) {
        float mean[3] = {0}, m2[3] = {0};
        float minimum[3] = {0}, maximum[3] = {0};
        float accel_min[3] = {0}, accel_max[3] = {0};
        float norm_min = 100.0f, norm_max = 0.0f;
        unsigned accepted = 0, failures = 0;
        uint64_t started_us = time_us_64(), next_us = started_us;
        for (unsigned attempt = 0; attempt < 100; ++attempt) {
            uint64_t now_us = time_us_64();
            if (now_us < next_us) sleep_us(next_us - now_us);
            mpu6500_reading_t reading;
            if (mpu6500_read(&reading)) {
                accepted++;
                float norm2 = 0.0f;
                for (int axis = 0; axis < 3; ++axis) {
                    float gyro = reading.gyro_dps[axis], accel = reading.accel_g[axis];
                    float delta = gyro - mean[axis];
                    mean[axis] += delta / accepted;
                    m2[axis] += delta * (gyro - mean[axis]);
                    if (accepted == 1 || gyro < minimum[axis]) minimum[axis] = gyro;
                    if (accepted == 1 || gyro > maximum[axis]) maximum[axis] = gyro;
                    if (accepted == 1 || accel < accel_min[axis]) accel_min[axis] = accel;
                    if (accepted == 1 || accel > accel_max[axis]) accel_max[axis] = accel;
                    norm2 += accel * accel;
                }
                float norm = sqrtf(norm2);
                norm_min = fminf(norm_min, norm); norm_max = fmaxf(norm_max, norm);
                if (attempt % 10 == 0)
                    printf("[IMU ONLY SAMPLE] accel_g=%.3f/%.3f/%.3f gyro_dps=%.3f/%.3f/%.3f\n",
                           reading.accel_g[0], reading.accel_g[1], reading.accel_g[2],
                           reading.gyro_dps[0], reading.gyro_dps[1], reading.gyro_dps[2]);
            } else {
                failures++;
            }
            next_us += 10000u;
            now_us = time_us_64();
            if (next_us < now_us) next_us = now_us;
        }
        float elapsed_s = (time_us_64() - started_us) / 1000000.0f;
        uint8_t who = 0, accel_config = 0, gyro_config = 0;
        bool config_ok = read_config_byte(0x75, &who) &&
                         read_config_byte(0x1C, &accel_config) &&
                         read_config_byte(0x1B, &gyro_config);
        printf("[IMU ONLY WINDOW %lu] accepted=%u errors=%u hz=%.1f cfg_read_ok=%u "
               "WHO=0x%02X ACCEL=0x%02X GYRO=0x%02X\n",
               (unsigned long)++window, accepted, failures,
               elapsed_s > 0 ? accepted / elapsed_s : 0,
               (unsigned)config_ok, who, accel_config, gyro_config);
        for (int axis = 0; axis < 3; ++axis) {
            float std = accepted > 1 ? sqrtf(fmaxf(m2[axis] / (accepted - 1), 0)) : 0;
            printf("[IMU ONLY AXIS %c] mean=%.3f std=%.3f min=%.3f max=%.3f "
                   "span=%.3f dps accel_span=%.3f g\n", 'X' + axis, mean[axis], std,
                   minimum[axis], maximum[axis], maximum[axis] - minimum[axis],
                   accel_max[axis] - accel_min[axis]);
        }
        if (accepted)
            printf("[IMU ONLY NORM] %.3f..%.3f g\n", norm_min, norm_max);
        // No bias is subtracted and no pass/fail heading is invented here.
    }
}
