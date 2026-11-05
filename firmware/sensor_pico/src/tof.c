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

// The bundled ToF platform initializes board-default I2C pins. Keep its bus
// separate from the IMU and fail compilation if the configured pins diverge.
#if TOF_SDA_PIN != PICO_DEFAULT_I2C_SDA_PIN || TOF_SCL_PIN != PICO_DEFAULT_I2C_SCL_PIN
#error "ToF pin config must match the bundled platform's board-default I2C pins"
#endif

#define TOF_DEFAULT_ADDR 0x29
#define TOF_DISTANCE_MODE_LONG 2
#define TOF_TIMING_BUDGET_MS 50
#define TOF_INTERMEASUREMENT_MS 60

static const uint8_t sensor_addresses[TOF_SENSOR_COUNT] = {0x30, 0x31, 0x32};
static uint8_t sensor_xshut_pins[TOF_SENSOR_COUNT];
static uint8_t online_mask;

// Rate-limit diagnostics per sensor so persistent faults do not flood USB.
static uint32_t last_diagnostic_ms[TOF_SENSOR_COUNT];
static uint8_t diagnostic_reported_mask;

static void report_diagnostic(int sensor, const char *stage, int status, unsigned distance)
{
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    uint8_t bit = (uint8_t)(1u << sensor);
    if ((diagnostic_reported_mask & bit) &&
        (uint32_t)(now_ms - last_diagnostic_ms[sensor]) < 2000u) return;
    diagnostic_reported_mask |= bit;
    last_diagnostic_ms[sensor] = now_ms;
    const char *names[TOF_SENSOR_COUNT] = {"L", "F", "R"};
    printf("[TOF DIAG %s] %s status=%d raw_mm=%u\n", names[sensor], stage, status, distance);
}

static bool configure_sensor(uint8_t address)
{
    // The API accepts the new address in 8-bit form; subsequent calls use 7-bit form.
    if (VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDR, address << 1) != 0)
        return false;
    if (VL53L1X_SensorInit(address) != 0)
        return false;
    if (VL53L1X_SetDistanceMode(address, TOF_DISTANCE_MODE_LONG) != 0)
        return false;
    if (VL53L1X_SetTimingBudgetInMs(address, TOF_TIMING_BUDGET_MS) != 0)
        return false;
    if (VL53L1X_SetInterMeasurementInMs(address, TOF_INTERMEASUREMENT_MS) != 0)
        return false;
    if (VL53L1X_StartRanging(address) != 0)
        return false;
    return true;
}

uint8_t tof_init(const uint8_t xshut_pins[TOF_SENSOR_COUNT])
{
    online_mask = 0;
    diagnostic_reported_mask = 0;
    if (xshut_pins == NULL)
        return 0;

    for (int i = 0; i < TOF_SENSOR_COUNT; ++i)
    {
        sensor_xshut_pins[i] = xshut_pins[i];
        gpio_init(sensor_xshut_pins[i]);
        gpio_put(sensor_xshut_pins[i], 0);
        gpio_set_dir(sensor_xshut_pins[i], GPIO_OUT);
    }
    sleep_ms(100);

    bool platform_ready = false;
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i)
    {
        gpio_put(sensor_xshut_pins[i], 1);
        sleep_ms(100);

        // Unlike raw i2c_init(), this also connects the library's I2C pointer.
        // It validates the first sensor while that sensor is at its default address.
        if (!platform_ready)
        {
            if (VL53L1X_I2C_Init(TOF_DEFAULT_ADDR, TOF_I2C_PORT) != 0)
            {
                gpio_put(sensor_xshut_pins[i], 0);
                continue;
            }
            platform_ready = true;
        }

        if (!configure_sensor(sensor_addresses[i]))
        {
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

uint8_t tof_online_mask(void) 
{
    return online_mask;
}

void tof_poll(tof_readings_t *readings)
{
    if (readings == NULL)
        return;
    memset(readings, 0, sizeof(*readings));

    for (int i = 0; i < TOF_SENSOR_COUNT; ++i)
    {
        const uint8_t bit = (uint8_t)(1u << i);
        if ((online_mask & bit) == 0)
            continue;

        uint8_t data_ready = 0;
        int status = VL53L1X_CheckForDataReady(sensor_addresses[i], &data_ready);
        if (status != 0)
        {
            report_diagnostic(i, "ready-read failed", status, 0);
            continue;
        }
        if (!data_ready) continue;

        VL53L1X_Result_t result;
        status = VL53L1X_GetResult(sensor_addresses[i], &result);
        if (status != 0) {
            report_diagnostic(i, "result-read failed", status, 0);
            continue;
        }
        status = VL53L1X_ClearInterrupt(sensor_addresses[i]);
        if (status != 0) {
            report_diagnostic(i, "interrupt-clear failed", status, 0);
            continue;
        }

        readings->fresh_mask |= bit;
        readings->timestamp_ms[i] = to_ms_since_boot(get_absolute_time());
        uint16_t code = tof_result_code(result.status, result.distance);
        if (code == 0)
            report_diagnostic(i, "range rejected", result.status, result.distance);
        if (code != 0)
        {
            readings->valid_mask |= bit;
            tof_distance_set(&readings->distance_bits, (tof_sensor_t)i, code);
        }
    }
}

uint16_t tof_result_code(uint8_t range_status, uint16_t distance_mm)
{
    if (range_status == 0) {
        // Keep 1000 as an exact measured value. 1023 is a category, not 1023 mm.
        return distance_mm > TOF_MAX_DISTANCE_MM ? TOF_NO_TARGET_MM : distance_mm;
    }
    // Weak signal/out-of-bounds and other warnings/errors do not prove clearance.
    return 0;
}

uint16_t tof_distance_get(uint32_t packed, tof_sensor_t sensor)
{
    unsigned shift = 10u * (unsigned)sensor;
    return (uint16_t)((packed >> shift) & TOF_DISTANCE_MASK);
}

void tof_distance_set(uint32_t *packed, tof_sensor_t sensor, uint16_t distance_mm)
{
    unsigned shift = 10u * (unsigned)sensor;

    // Only pack ranges whose status has already been checked by tof_result_code().
    // Preserve zero/unavailable; successful values beyond 1 m use 1023/no-target.
    uint32_t code = distance_mm <= TOF_MAX_DISTANCE_MM ? distance_mm : TOF_NO_TARGET_MM;

    // Clear this sensor's field, then insert its new value.
    *packed = (*packed & ~(TOF_DISTANCE_MASK << shift)) | (code << shift);
}
