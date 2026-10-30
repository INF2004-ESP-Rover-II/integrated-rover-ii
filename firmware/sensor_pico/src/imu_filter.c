#include "imu_filter.h"

#include <math.h>
#include <stddef.h>

#define RAD_TO_DEG 57.2957795131f

static float wrap180(float degrees) {
    float wrapped = fmodf(degrees + 180.0f, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    return wrapped - 180.0f;
}

bool imu_filter_update(imu_filter_t *state, const mpu6500_reading_t *reading,
                       uint64_t now_us, float yaw_sign) {
    if (state == NULL || reading == NULL || state->timing_fault ||
        !isfinite(yaw_sign)) return false;
    for (int axis = 0; axis < 3; ++axis)
        if (!isfinite(reading->accel_g[axis]) ||
            !isfinite(reading->gyro_dps[axis])) return false;

    const float ax = reading->accel_g[0], ay = reading->accel_g[1];
    const float az = reading->accel_g[2];
    if (ax * ax + ay * ay + az * az < 0.01f) return false;
    const float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    const float accel_roll = atan2f(ay, az) * RAD_TO_DEG;
    if (!state->seeded) {
        const float norm = sqrtf(ax * ax + ay * ay + az * az);
        state->startup_up[0] = ax / norm;
        state->startup_up[1] = ay / norm;
        state->startup_up[2] = az / norm;
        state->pitch_deg = accel_pitch;
        state->roll_deg = accel_roll;
        state->last_sample_us = now_us;
        state->seeded = true;
        return true;
    }
    if (now_us <= state->last_sample_us) return false;
    const uint64_t elapsed = now_us - state->last_sample_us;
    if (elapsed > 500000u) {
        state->timing_fault = true;
        return false;
    }
    const float dt = (float)elapsed / 1000000.0f;
    // Equivalent to the sketch's 0.98 at 10 ms, adjusted for variable loop time.
    const float alpha = powf(0.98f, dt / 0.01f);
    const float pitch = state->pitch_deg + reading->gyro_dps[1] * dt;
    const float roll = state->roll_deg + reading->gyro_dps[0] * dt;
    state->pitch_deg = wrap180(pitch + (1.0f - alpha) * wrap180(accel_pitch - pitch));
    state->roll_deg = wrap180(roll + (1.0f - alpha) * wrap180(accel_roll - roll));
    float yaw_rate = 0.0f;
    for (int axis = 0; axis < 3; ++axis)
        yaw_rate += reading->gyro_dps[axis] * state->startup_up[axis];
    state->heading_deg = fmodf(state->heading_deg + yaw_sign * yaw_rate * dt,
                              360.0f);
    if (state->heading_deg < 0.0f) state->heading_deg += 360.0f;
    state->last_sample_us = now_us;
    return true;
}
