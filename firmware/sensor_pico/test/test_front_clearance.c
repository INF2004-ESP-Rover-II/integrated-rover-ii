#include <assert.h>
#include <stdio.h>
#include "obstacle.h"

// Wire-format helpers only; run the real obstacle detector on controlled inputs.
uint16_t tof_distance_get(uint32_t bits, tof_sensor_t sensor) {
    return (uint16_t)((bits >> (10u * sensor)) & 1023u);
}
void tof_distance_set(uint32_t *bits, tof_sensor_t sensor, uint16_t distance) {
    unsigned shift = 10u * sensor;
    *bits = (*bits & ~(1023u << shift)) | ((uint32_t)distance << shift);
}

int main(void) {
    obstacle_state_t state;
    obstacle_init(&state, 0);
    const uint16_t ranges[] = {149, 150, 151, 1023, 1, 0, 200};
    for (unsigned i = 0; i < sizeof ranges / sizeof ranges[0]; ++i) {
        tof_readings_t sample = {0};
        sample.fresh_mask = 2;
        sample.valid_mask = ranges[i] != 0 ? 2 : 0;
        sample.timestamp_ms[TOF_FRONT] = i + 1;
        tof_distance_set(&sample.distance_bits, TOF_FRONT, ranges[i]);
        obstacle_update(&state, &sample, 2, i + 1);
        assert(state.stop_required == (ranges[i] > 0 && ranges[i] < 150));
        if (ranges[i] == 151) {
            assert(state.urgent_mask & 2); // Diagnostic hysteresis may remain.
            assert(!state.stop_required); // Forward permission must already clear.
        }
    }
    puts("Sensor front clearance tests passed");
    return 0;
}
