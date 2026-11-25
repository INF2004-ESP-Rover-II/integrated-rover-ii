/**
 * @file imu_filter.c
 * @brief Imu filter firmware module.
 */
#include "imu_filter.h"

#include <math.h>
#include <stddef.h>

/* Fixed dimensions and iteration limits used by this module. */
#define IMU_AXIS_COUNT 3

#define RAD_TO_DEG 57.2957795131f

static float
wrap180 (float degrees)
{
    float wrapped = fmodf(degrees + 180.0f, 360.0f);
    if (wrapped < 0.0f)
    {
        wrapped += 360.0f;
    }
    return wrapped - 180.0f;
}

bool
imu_filter_update (imu_filter_t * p_state, const mpu6500_reading_t * p_reading,
                   uint64_t now_us, float yaw_sign)
{
    if (p_state == NULL || p_reading == NULL || p_state->timing_fault ||
        !isfinite(yaw_sign))
    {
        return false;
    }
    for (int axis = 0; axis < IMU_AXIS_COUNT; ++axis)
    {
        if (!isfinite(p_reading->accel_g[axis]) ||
            !isfinite(p_reading->gyro_dps[axis]))
        {
            return false;
        }
    }

    const float accel_x = p_reading->accel_g[0];
    const float accel_y = p_reading->accel_g[1];
    const float accel_z = p_reading->accel_g[2];
    if (accel_x * accel_x + accel_y * accel_y + accel_z * accel_z < 0.01f)
    {
        return false;
    }

    const float accel_pitch =
        atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) *
        RAD_TO_DEG;

    const float accel_roll = atan2f(accel_y, accel_z) * RAD_TO_DEG;
    if (!p_state->seeded)
    {

        const float norm =
            sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

        p_state->startup_up[0]  = accel_x / norm;
        p_state->startup_up[1]  = accel_y / norm;
        p_state->startup_up[2]  = accel_z / norm;
        p_state->pitch_deg      = accel_pitch;
        p_state->roll_deg       = accel_roll;
        p_state->last_sample_us = now_us;
        p_state->seeded         = true;
        return true;
    }
    if (now_us <= p_state->last_sample_us)
    {
        return false;
    }
    const uint64_t elapsed = now_us - p_state->last_sample_us;
    if (elapsed > 500000u)
    {
        p_state->timing_fault = true;
        return false;
    }
    const float delta_time = (float) elapsed / 1000000.0f;
    // Equivalent to the sketch's 0.98 at 10 ms, adjusted for variable loop
    // time.
    const float alpha = powf(0.98f, delta_time / 0.01f);

    const float pitch =
        p_state->pitch_deg + p_reading->gyro_dps[1] * delta_time;

    const float roll = p_state->roll_deg + p_reading->gyro_dps[0] * delta_time;

    p_state->pitch_deg =
        wrap180(pitch + (1.0f - alpha) * wrap180(accel_pitch - pitch));


    p_state->roll_deg =
        wrap180(roll + (1.0f - alpha) * wrap180(accel_roll - roll));

    float yaw_rate = 0.0f;
    for (int axis = 0; axis < IMU_AXIS_COUNT; ++axis)
    {
        yaw_rate += p_reading->gyro_dps[axis] * p_state->startup_up[axis];
    }

    p_state->heading_deg =
        fmodf(p_state->heading_deg + yaw_sign * yaw_rate * delta_time, 360.0f);

    if (p_state->heading_deg < 0.0f)
    {
        p_state->heading_deg += 360.0f;
    }
    p_state->last_sample_us = now_us;
    return true;
}

/* End of file. */
