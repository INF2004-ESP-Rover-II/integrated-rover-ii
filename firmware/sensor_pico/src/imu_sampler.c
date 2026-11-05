#include "imu_sampler.h"

#include <math.h>
#include <string.h>
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "sensor_config.h"

#define IMU_PERIOD_US 10000u

static critical_section_t snapshot_lock;
static imu_snapshot_t shared_snapshot;
static bool started;

static void imu_core1(void) {
    imu_snapshot_t current = {0};
    uint64_t next_us = time_us_64();
    uint64_t previous_us = 0;
    while (true) {
        uint64_t now_us = time_us_64();
        if (now_us < next_us) sleep_us(next_us - now_us);
        bool read_ok = mpu6500_read(&current.reading);
        uint64_t sample_us = time_us_64();
        if (previous_us != 0) {
            uint64_t gap = sample_us - previous_us;
            uint32_t bounded_gap = gap > UINT32_MAX ? UINT32_MAX : (uint32_t)gap;
            if (bounded_gap > current.max_gap_us) current.max_gap_us = bounded_gap;
        }
        previous_us = sample_us;
        current.valid = false;
        if (read_ok) {
            bool clipped = false;
            for (int axis = 0; axis < 3; ++axis)
                if (fabsf(current.reading.gyro_dps[axis]) >= 1970.0f) clipped = true;
            if (clipped) {
                current.clipped_samples++;
                // Rotation lost to saturation cannot be reconstructed.
                current.orientation.timing_fault = true;
            }
            current.valid = !clipped && imu_filter_update(&current.orientation,
                &current.reading, sample_us, MPU6500_YAW_SIGN);
            current.sample_us = sample_us;
            current.samples++;
        } else {
            current.read_errors++;
        }
        // Never hold this lock during I2C, math, sleep or printing.
        critical_section_enter_blocking(&snapshot_lock);
        shared_snapshot = current;
        critical_section_exit(&snapshot_lock);

        next_us += IMU_PERIOD_US;
        now_us = time_us_64();
        if (next_us <= now_us) {
            current.overruns++;
            next_us = now_us + IMU_PERIOD_US;
        }
    }
}

void imu_sampler_start(void) {
    if (started) return;
    critical_section_init(&snapshot_lock);
    memset(&shared_snapshot, 0, sizeof(shared_snapshot));
    started = true;
    multicore_launch_core1(imu_core1);
}

void imu_sampler_get_snapshot(imu_snapshot_t *snapshot) {
    if (!started) {
        memset(snapshot, 0, sizeof(*snapshot));
        return;
    }
    critical_section_enter_blocking(&snapshot_lock);
    *snapshot = shared_snapshot;
    critical_section_exit(&snapshot_lock);
}
