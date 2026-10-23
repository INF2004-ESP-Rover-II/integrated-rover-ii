#include "tof.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "sensor_config.h"
#include "VL53L1X_api.h"
#include "VL53L1X_platform.h"
#include "VL53L1X_types.h"

#define TOF_DEFAULT_ADDR 0x29
#define TOF_DISTANCE_MODE_LONG 2
#define TOF_TIMING_BUDGET_MS 50
#define TOF_INTERMEASUREMENT_MS 60

static const uint8_t sensor_addresses[TOF_SENSOR_COUNT] = {0x30, 0x31, 0x32};
static uint8_t sensor_xshut_pins[TOF_SENSOR_COUNT];
static uint8_t online_mask;

static bool configure_sensor(uint8_t address) {
    // The API accepts the new address in 8-bit form; subsequent calls use 7-bit form.
    if (VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDR, address << 1) != 0) return false;
    if (VL53L1X_SensorInit(address) != 0) return false;
    if (VL53L1X_SetDistanceMode(address, TOF_DISTANCE_MODE_LONG) != 0) return false;
    if (VL53L1X_SetTimingBudgetInMs(address, TOF_TIMING_BUDGET_MS) != 0) return false;
    if (VL53L1X_SetInterMeasurementInMs(address, TOF_INTERMEASUREMENT_MS) != 0) return false;
    if (VL53L1X_StartRanging(address) != 0) return false;
    return true;
}

uint8_t tof_init(const uint8_t xshut_pins[TOF_SENSOR_COUNT]) {
    online_mask = 0;
    if (xshut_pins == NULL) return 0;

    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        sensor_xshut_pins[i] = xshut_pins[i];
        gpio_init(sensor_xshut_pins[i]);
        gpio_put(sensor_xshut_pins[i], 0);
        gpio_set_dir(sensor_xshut_pins[i], GPIO_OUT);
    }
    sleep_ms(100);

    bool platform_ready = false;
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        gpio_put(sensor_xshut_pins[i], 1);
        sleep_ms(100);

        // Unlike raw i2c_init(), this also connects the library's I2C pointer.
        // It validates the first sensor while that sensor is at its default address.
        if (!platform_ready) {
            if (VL53L1X_I2C_Init(TOF_DEFAULT_ADDR, I2C_PORT) != 0) {
                gpio_put(sensor_xshut_pins[i], 0);
                continue;
            }
            platform_ready = true;

        }

        if (!configure_sensor(sensor_addresses[i])) {
            // Take a failed device off the bus before starting the next one.
            gpio_put(sensor_xshut_pins[i], 0);
            continue;
        }
    printf("ToF sensor initialised 1!\n");
    printf("online_mask = 0x%02X\n", online_mask);

        online_mask |= (uint8_t)(1u << i);
    }

    return online_mask;
}

uint8_t tof_online_mask(void) {
    return online_mask;
}

void tof_poll(tof_readings_t *readings) {
    if (readings == NULL) return;
    memset(readings, 0, sizeof(*readings));

    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        const uint8_t bit = (uint8_t)(1u << i);
        if ((online_mask & bit) == 0) continue;

        uint8_t data_ready = 0;
        if (VL53L1X_CheckForDataReady(sensor_addresses[i], &data_ready) != 0 || !data_ready) {
            continue;
        }

        VL53L1X_Result_t result;
        if (VL53L1X_GetResult(sensor_addresses[i], &result) != 0) continue;
        if (VL53L1X_ClearInterrupt(sensor_addresses[i]) != 0) continue;

        readings->fresh_mask |= bit;
        readings->timestamp_ms[i] = to_ms_since_boot(get_absolute_time());
        if (result.status == 0) {
            readings->valid_mask |= bit;
            readings->distance_mm[i] = result.distance;
        } else if (result.status == 2 || result.status == 4) {
            // Project policy: a fresh weak-signal/no-target (2) or out-of-bounds
            // (4) result is no detected obstacle, not a failed sensor. These
            // statuses do not guarantee empty space (e.g. weak target returns).
            // Other range errors and I2C failures remain unavailable readings.
            readings->valid_mask |= bit;
            readings->distance_mm[i] = TOF_NO_TARGET_MM;
        }
    }
}
