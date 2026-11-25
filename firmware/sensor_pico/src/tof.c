/**
 * @file tof.c
 * @brief Tof firmware module.
 */
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
#if TOF_SDA_PIN != PICO_DEFAULT_I2C_SDA_PIN ||                                 \
    TOF_SCL_PIN != PICO_DEFAULT_I2C_SCL_PIN
#error "ToF pin config must match the bundled platform's board-default I2C pins"
#endif

#define TOF_DEFAULT_ADDR 0x29
#define TOF_DISTANCE_MODE_LONG 2
#define TOF_TIMING_BUDGET_MS 50
#define TOF_INTERMEASUREMENT_MS 60

static const uint8_t g_sensor_addresses[TOF_SENSOR_COUNT] = { 0x30, 0x31,
                                                              0x32 };
static uint8_t       g_sensor_xshut_pins[TOF_SENSOR_COUNT];
static uint8_t       g_online_mask;

// Rate-limit diagnostics per sensor so persistent faults do not flood USB.
static uint32_t g_last_diagnostic_ms[TOF_SENSOR_COUNT];
static uint8_t  g_diagnostic_reported_mask;

static void
report_diagnostic (int sensor, const char * p_stage, int status,
                   unsigned distance)
{
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    uint8_t  bit    = (uint8_t) (1u << (uint32_t) (sensor));
    if (((uint32_t) (g_diagnostic_reported_mask) & (uint32_t) (bit)) &&
        (uint32_t) (now_ms - g_last_diagnostic_ms[sensor]) < 2000u)
    {
        return;
    }
    g_diagnostic_reported_mask |= (uint32_t) (bit);
    g_last_diagnostic_ms[sensor]         = now_ms;
    const char * names[TOF_SENSOR_COUNT] = { "L", "F", "R" };
    printf("[TOF DIAG %s] %s status=%d raw_mm=%u\n", names[sensor], p_stage,
           status, distance);
}

static bool
configure_sensor (uint8_t address)
{
    // The API accepts the new address in 8-bit form; subsequent calls use 7-bit
    // form.
    if (VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDR, (uint32_t) (address)
                                                    << (uint32_t) (1)) != 0)
    {
        return false;
    }
    if (VL53L1X_SensorInit(address) != 0)
    {
        return false;
    }
    if (VL53L1X_SetDistanceMode(address, TOF_DISTANCE_MODE_LONG) != 0)
    {
        return false;
    }
    if (VL53L1X_SetTimingBudgetInMs(address, TOF_TIMING_BUDGET_MS) != 0)
    {
        return false;
    }
    if (VL53L1X_SetInterMeasurementInMs(address, TOF_INTERMEASUREMENT_MS) != 0)
    {
        return false;
    }
    if (VL53L1X_StartRanging(address) != 0)
    {
        return false;
    }
    return true;
}

uint8_t
tof_init (const uint8_t xshut_pins[TOF_SENSOR_COUNT])
{
    g_online_mask              = 0;
    g_diagnostic_reported_mask = 0;
    if (xshut_pins == NULL)
    {
        return 0;
    }

    for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
    {
        g_sensor_xshut_pins[index] = xshut_pins[index];
        gpio_init(g_sensor_xshut_pins[index]);
        gpio_put(g_sensor_xshut_pins[index], 0);
        gpio_set_dir(g_sensor_xshut_pins[index], GPIO_OUT);
    }
    sleep_ms(100);

    bool b_platform_ready = false;
    for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
    {
        printf("initialising tof sensor..!\n");

        gpio_put(g_sensor_xshut_pins[index], 1);
        sleep_ms(100);

        // Unlike raw i2c_init(), this also connects the library's I2C pointer.
        // It validates the first sensor while that sensor is at its default
        // address.
        if (!b_platform_ready)
        {
            if (VL53L1X_I2C_Init(TOF_DEFAULT_ADDR, TOF_I2C_PORT) != 0)
            {
                gpio_put(g_sensor_xshut_pins[index], 0);
                continue;
            }
            b_platform_ready = true;
        }

        if (!configure_sensor(g_sensor_addresses[index]))
        {
            // Take a failed device off the bus before starting the next one.
            gpio_put(g_sensor_xshut_pins[index], 0);
            continue;
        }
        printf("ToF sensor initialised 1!\n");
        printf("online_mask = 0x%02X\n", g_online_mask);

        g_online_mask |= (uint32_t) ((uint8_t) (1u << (uint32_t) (index)));
    }

    return g_online_mask;
}

uint8_t
tof_online_mask (void)
{
    return g_online_mask;
}

void
tof_poll (tof_readings_t * p_readings)
{
    if (p_readings == NULL)
    {
        return;
    }
    memset(p_readings, 0, sizeof(*p_readings));

    for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
    {
        const uint8_t bit = (uint8_t) (1u << (uint32_t) (index));
        if (((uint32_t) (g_online_mask) & (uint32_t) (bit)) == 0)
        {
            continue;
        }

        uint8_t data_ready = 0;

        int status =
            VL53L1X_CheckForDataReady(g_sensor_addresses[index], &data_ready);

        if (status != 0)
        {
            report_diagnostic(index, "ready-read failed", status, 0);
            continue;
        }
        if (!data_ready)
        {
            continue;
        }

        VL53L1X_Result_t result;
        status = VL53L1X_GetResult(g_sensor_addresses[index], &result);
        if (status != 0)
        {
            report_diagnostic(index, "result-read failed", status, 0);
            continue;
        }
        status = VL53L1X_ClearInterrupt(g_sensor_addresses[index]);
        if (status != 0)
        {
            report_diagnostic(index, "interrupt-clear failed", status, 0);
            continue;
        }

        p_readings->fresh_mask |= (uint32_t) (bit);
        p_readings->timestamp_ms[index] = to_ms_since_boot(get_absolute_time());
        uint16_t code = tof_result_code(result.status, result.distance);
        if (code == 0)
        {
            report_diagnostic(index, "range rejected", result.status,
                              result.distance);
        }
        if (code != 0)
        {
            p_readings->valid_mask |= (uint32_t) (bit);
            tof_distance_set(&p_readings->distance_bits, (tof_sensor_t) index,
                             code);
        }
    }
}

uint16_t
tof_result_code (uint8_t range_status, uint16_t distance_mm)
{
    if (range_status == 0)
    {
        // Keep 1000 as an exact measured value. 1023 is a category, not 1023
        // mm.
        return distance_mm > TOF_MAX_DISTANCE_MM ? TOF_NO_TARGET_MM
                                                 : distance_mm;
    }
    // Weak signal/out-of-bounds and other warnings/errors do not prove
    // clearance.
    return 0;
}

uint16_t
tof_distance_get (uint32_t packed, tof_sensor_t sensor)
{
    unsigned shift = 10u * (unsigned) sensor;
    return (uint16_t) ((packed >> shift) & TOF_DISTANCE_MASK);
}

void
tof_distance_set (uint32_t * p_packed, tof_sensor_t sensor,
                  uint16_t distance_mm)
{
    unsigned shift = 10u * (unsigned) sensor;

    // Only pack ranges whose status has already been checked by
    // tof_result_code(). Preserve zero/unavailable; successful values beyond 1
    // m use 1023/no-target.

    uint32_t code =
        distance_mm <= TOF_MAX_DISTANCE_MM ? distance_mm : TOF_NO_TARGET_MM;


    // Clear this sensor's field, then insert its new value.
    *p_packed = (*p_packed & ~(TOF_DISTANCE_MASK << shift)) | (code << shift);
}

/* End of file. */
