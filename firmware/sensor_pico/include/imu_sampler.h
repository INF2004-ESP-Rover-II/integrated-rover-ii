#ifndef IMU_SAMPLER_H
#define IMU_SAMPLER_H

#include "imu_filter.h"

typedef struct {
    mpu6500_reading_t reading;
    imu_filter_t orientation;
    uint64_t sample_us;
    uint32_t samples;
    uint32_t read_errors;
    uint32_t clipped_samples;
    uint32_t max_gap_us;
    uint32_t overruns;
    bool valid;
} imu_snapshot_t;

// Call once after successful init/calibration. Thereafter core 1 exclusively
// owns MPU6500 I2C1 reads and fusion. Core 0 keeps ToF I2C0, UART and SD work.
void imu_sampler_start(void);
// Short synchronized copy; never reads I2C or waits for a sensor.
void imu_sampler_get_snapshot(imu_snapshot_t *snapshot);

#endif
