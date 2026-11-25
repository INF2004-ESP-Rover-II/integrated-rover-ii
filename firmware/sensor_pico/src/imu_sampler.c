/**
 * @file imu_sampler.c
 * @brief Imu sampler firmware module.
 */
#include "imu_sampler.h"

#include <math.h>
#include <string.h>
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "sensor_config.h"

/* Fixed dimensions and iteration limits used by this module. */
#define IMU_AXIS_COUNT 3

#define IMU_PERIOD_US 10000u

static critical_section_t g_snapshot_lock;
static imu_snapshot_t     g_shared_snapshot;
static bool               g_b_started;

static void
imu_core1 (void)
{
    imu_snapshot_t current     = { 0 };
    uint64_t       next_us     = time_us_64();
    uint64_t       previous_us = 0;
    while (true)
    {
        uint64_t now_us = time_us_64();
        if (now_us < next_us)
        {
            sleep_us(next_us - now_us);
        }
        bool     b_read_ok = mpu6500_read(&current.reading);
        uint64_t sample_us = time_us_64();
        if (previous_us != 0)
        {
            uint64_t gap = sample_us - previous_us;

            uint32_t bounded_gap =
                gap > UINT32_MAX ? UINT32_MAX : (uint32_t) gap;

            if (bounded_gap > current.max_gap_us)
            {
                current.max_gap_us = bounded_gap;
            }
        }
        previous_us   = sample_us;
        current.valid = false;
        if (b_read_ok)
        {
            bool b_clipped = false;
            for (int axis = 0; axis < IMU_AXIS_COUNT; ++axis)
            {
                if (fabsf(current.reading.gyro_dps[axis]) >= 1970.0f)
                {
                    b_clipped = true;
                }
            }
            if (b_clipped)
            {
                current.clipped_samples++;
                // Rotation lost to saturation cannot be reconstructed.
                current.orientation.timing_fault = true;
            }

            current.valid =
                !b_clipped &&
                imu_filter_update(&current.orientation, &current.reading,
                                  sample_us, MPU6500_YAW_SIGN);

            current.sample_us = sample_us;
            current.samples++;
        }
        else
        {
            current.read_errors++;
        }
        // Never hold this lock during I2C, math, sleep or printing.
        critical_section_enter_blocking(&g_snapshot_lock);
        g_shared_snapshot = current;
        critical_section_exit(&g_snapshot_lock);

        next_us += IMU_PERIOD_US;
        now_us = time_us_64();
        if (next_us <= now_us)
        {
            current.overruns++;
            next_us = now_us + IMU_PERIOD_US;
        }
    }
}

void
imu_sampler_start (void)
{
    if (g_b_started)
    {
        return;
    }
    critical_section_init(&g_snapshot_lock);
    memset(&g_shared_snapshot, 0, sizeof(g_shared_snapshot));
    g_b_started = true;
    multicore_launch_core1(imu_core1);
}

void
imu_sampler_get_snapshot (imu_snapshot_t * p_snapshot)
{
    if (!g_b_started)
    {
        memset(p_snapshot, 0, sizeof(*p_snapshot));
        return;
    }
    critical_section_enter_blocking(&g_snapshot_lock);
    *p_snapshot = g_shared_snapshot;
    critical_section_exit(&g_snapshot_lock);
}

/* End of file. */
