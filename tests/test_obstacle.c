#include <assert.h>
#include <stdio.h>
#include "obstacle.h"
#include "sensor_config.h"

static tof_readings_t readings_at(uint16_t left, uint16_t front, uint16_t right,
                                 uint32_t now) {
    tof_readings_t readings = {0};
    const uint16_t distances[3] = {left, front, right};
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        tof_distance_set(&readings.distance_bits, (tof_sensor_t)i, distances[i]);
        readings.timestamp_ms[i] = now;
    }
    readings.fresh_mask = OBSTACLE_ALL_MASK;
    readings.valid_mask = OBSTACLE_ALL_MASK;
    return readings;
}

static void sample(obstacle_state_t *state, uint16_t left, uint16_t front,
                   uint16_t right, uint32_t now) {
    tof_readings_t readings = readings_at(left, front, right, now);
    obstacle_update(state, &readings, OBSTACLE_ALL_MASK, now);
}

static void startup_and_initialization_faults(void) {
    obstacle_state_t state;
    tof_readings_t empty = {0};
    const uint32_t start = 100000u; // Startup grace must not be based on boot zero.
    obstacle_init(&state, start);
    obstacle_update(&state, &empty, OBSTACLE_ALL_MASK, start);
    assert(state.valid_mask == 0 && state.fault_mask == 0 && !state.stop_required);
    obstacle_update(&state, &empty, OBSTACLE_ALL_MASK, start + OBSTACLE_STALE_MS);
    assert(state.fault_mask == 0);
    obstacle_update(&state, &empty, OBSTACLE_ALL_MASK, start + OBSTACLE_STALE_MS + 1u);
    assert(state.fault_mask == OBSTACLE_ALL_MASK && state.valid_mask == 0);

    for (uint8_t online = 0; online <= OBSTACLE_ALL_MASK; ++online) {
        obstacle_init(&state, start);
        obstacle_update(&state, &empty, online, start);
        assert(state.fault_mask == ((~online) & OBSTACLE_ALL_MASK));
        assert(state.valid_mask == 0 && !state.stop_required);
        // Changing online alone is not evidence of a recovered result.
        obstacle_update(&state, &empty, OBSTACLE_ALL_MASK, start + 1u);
        assert(state.fault_mask == ((~online) & OBSTACLE_ALL_MASK));
        sample(&state, 1000, 1000, 1000, start + 2u);
        assert(state.fault_mask == 0 && state.valid_mask == OBSTACLE_ALL_MASK);
    }
    obstacle_init(&state, start);
    tof_readings_t readings = readings_at(100, 100, 100, start);
    obstacle_update(&state, &readings, 0, start);
    assert(state.fault_mask == 7 && state.valid_mask == 0 && !state.stop_required);
}

static void proximity_thresholds_and_hysteresis(void) {
    obstacle_state_t state;
    const uint8_t front = 1u << TOF_FRONT;
    obstacle_init(&state, 0);
    sample(&state, 1000, 1000, 1000, 100);
    sample(&state, 150, 1000, 1000, 101);
    assert(state.urgent_mask == 1 && !state.stop_required);
    sample(&state, 1000, 1000, 150, 102);
    assert(state.urgent_mask == 4 && !state.stop_required);
    sample(&state, 150, 1000, 150, 103);
    assert(state.urgent_mask == 5 && !state.stop_required);
    sample(&state, 1000, OBSTACLE_BLOCKED_MM, 1000, 110);
    assert(state.blocked_mask == front && !state.stop_required);
    sample(&state, 1000, OBSTACLE_BLOCKED_MM + 1u, 1000, 120);
    assert(state.blocked_mask == front);
    sample(&state, 1000, OBSTACLE_BLOCKED_MM + OBSTACLE_HYSTERESIS_MM, 1000, 130);
    assert(state.blocked_mask == 0);
    sample(&state, 1000, OBSTACLE_URGENT_MM - 1u, 1000, 140);
    assert(state.urgent_mask == front && state.stop_required);
    sample(&state, 1000, OBSTACLE_URGENT_MM + 1u, 1000, 150);
    assert(state.urgent_mask == front && !state.stop_required);
    sample(&state, 1000, OBSTACLE_URGENT_MM + OBSTACLE_HYSTERESIS_MM, 1000, 160);
    assert(state.urgent_mask == 0 && !state.stop_required && state.fault_mask == 0);
}

static void expiry_and_replayed_results(void) {
    obstacle_state_t state;
    tof_readings_t empty = {0};
    obstacle_init(&state, 0);
    tof_readings_t old = readings_at(1000, 149, 1000, 100);
    obstacle_update(&state, &old, 7, 100);
    obstacle_update(&state, &empty, 7, 100 + OBSTACLE_STALE_MS);
    assert(state.fault_mask == 0 && state.valid_mask == 7 && state.stop_required);
    obstacle_update(&state, &empty, 7, 101 + OBSTACLE_STALE_MS);
    assert(state.fault_mask == 7 && state.valid_mask == 0 && !state.stop_required);
    assert(state.urgent_mask == 0 && state.blocked_mask == 0);
    assert(tof_distance_get(state.distance_bits, TOF_FRONT) == 0);
    obstacle_update(&state, &old, 7, 102 + OBSTACLE_STALE_MS);
    assert(state.fault_mask == 7 && state.valid_mask == 0 && !state.stop_required);
    sample(&state, 1000, 1000, 1000, 103 + OBSTACLE_STALE_MS);
    assert(state.fault_mask == 0 && state.valid_mask == 7);
}

static void rejected_ranges_are_unavailable_but_device_remains_healthy(void) {
    obstacle_state_t state;
    const uint8_t front = 1u << TOF_FRONT;
    obstacle_init(&state, 0);
    sample(&state, 1000, 149, 1000, 500);
    for (uint32_t now = 501; now <= 10000; ++now) {
        tof_readings_t readings = readings_at(1000, 100, 1000, now);
        readings.valid_mask &= (uint8_t)~front;
        obstacle_update(&state, &readings, 7, now);
        assert(state.fault_mask == 0);
        assert(state.valid_mask == 5 && !state.stop_required);
        assert(state.timestamp_ms[TOF_FRONT] == now);
        assert(tof_distance_get(state.distance_bits, TOF_FRONT) == 0);
    }
    sample(&state, 1000, 0, 1000, 10001);
    assert(state.fault_mask == 0 && !state.stop_required);
    tof_readings_t unsupported = readings_at(1000, 1000, 1000, 10002);
    unsupported.distance_bits = (unsupported.distance_bits & ~(0x3FFu << 10u)) |
                                (1001u << 10u); // Bypass the encoder deliberately.
    obstacle_update(&state, &unsupported, 7, 10002);
    assert(state.fault_mask == 0 && state.valid_mask == 5);
    sample(&state, 1000, 149, 1000, 10003); // Hand appears: usable close range immediately.
    assert(state.fault_mask == 0 && state.valid_mask == 7 && state.stop_required);
    sample(&state, 1000, TOF_NO_TARGET_MM, 1000, 10004);
    assert(state.fault_mask == 0 && state.valid_mask == 7 && !state.stop_required);
}

static void independent_fault_bits_and_recovery(void) {
    for (int sensor = 0; sensor < TOF_SENSOR_COUNT; ++sensor) {
        obstacle_state_t state;
        const uint8_t bit = (uint8_t)(1u << sensor);
        obstacle_init(&state, 0);
        sample(&state, 1000, 1000, 1000, 100);
        tof_readings_t readings = readings_at(1000, 1000, 1000, 401);
        readings.fresh_mask &= (uint8_t)~bit;
        obstacle_update(&state, &readings, 7, 401);
        assert(state.fault_mask == bit && state.valid_mask == (7u & ~bit));
        assert(!state.stop_required);
        // A completed but rejected result restores device health, not range validity.
        readings = readings_at(1000, 1000, 1000, 402);
        readings.valid_mask &= (uint8_t)~bit;
        obstacle_update(&state, &readings, 7, 402);
        assert(state.fault_mask == 0 && state.valid_mask == (7u & ~bit));
        readings = readings_at(1000, 1000, 1000, 403);
        tof_distance_set(&readings.distance_bits, (tof_sensor_t)sensor, TOF_NO_TARGET_MM);
        obstacle_update(&state, &readings, 7, 403);
        assert(state.fault_mask == 0 && state.valid_mask == 7);
    }
}

static void warnings_after_initialization_do_not_require_an_initial_good_range(void) {
    const uint8_t statuses[] = {1, 2, 4, 7};
    for (unsigned status = 0; status < sizeof statuses / sizeof statuses[0]; ++status) {
        obstacle_state_t state;
        obstacle_init(&state, 0);
        for (uint32_t now = 60; now <= 6000; now += 60) {
            const uint16_t code = tof_result_code(statuses[status], 2000);
            tof_readings_t readings = readings_at(code, code, code, now);
            readings.valid_mask = 0;
            obstacle_update(&state, &readings, 7, now);
            assert(state.fault_mask == 0 && state.valid_mask == 0);
            assert(state.distance_bits == 0 && !state.stop_required);
        }
        // Actual loss of result reads still faults each sensor after the grace period.
        tof_readings_t empty = {0};
        obstacle_update(&state, &empty, 7, 6000 + OBSTACLE_STALE_MS + 1);
        assert(state.fault_mask == 7 && state.valid_mask == 0);
        tof_readings_t recovered = readings_at(0, 0, 0, 6400);
        recovered.valid_mask = 0;
        obstacle_update(&state, &recovered, 7, 6400);
        assert(state.fault_mask == 0 && state.valid_mask == 0);
    }
}

static void valid_far_ranges_and_uncertain_results(void) {
    const uint16_t ranges[] = {0, 1, 150, 999, 1000, 1001, 1023, 1024, UINT16_MAX};
    for (unsigned i = 0; i < sizeof(ranges) / sizeof(ranges[0]); ++i) {
        const uint16_t raw = ranges[i];
        const uint16_t expected = raw > 1000 ? TOF_NO_TARGET_MM : raw;
        assert(tof_result_code(0, raw) == expected);
        for (unsigned status = 1; status <= 255; ++status) {
            assert(tof_result_code((uint8_t)status, raw) == 0);
        }
    }
    obstacle_state_t state;
    obstacle_init(&state, 0);
    sample(&state, 100, 100, 100, 1);
    assert(state.stop_required && state.blocked_mask == 7);
    tof_readings_t readings = {0};
    readings.fresh_mask = readings.valid_mask = 7;
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        // Successful long ranges become the category code, including raw 1023 mm.
        const uint16_t raw[] = {1001, 1023, 4000};
        tof_distance_set(&readings.distance_bits, (tof_sensor_t)i, tof_result_code(0, raw[i]));
        readings.timestamp_ms[i] = 2;
        assert(tof_distance_get(readings.distance_bits, (tof_sensor_t)i) == TOF_NO_TARGET_MM);
    }
    obstacle_update(&state, &readings, 7, 2);
    assert(state.valid_mask == 7 && state.fault_mask == 0);
    assert(state.blocked_mask == 0 && state.urgent_mask == 0 && !state.stop_required);
}

static void no_target_and_clock_rollover(void) {
    obstacle_state_t state;
    tof_readings_t empty = {0};
    obstacle_init(&state, 0);
    for (uint32_t now = 0; now <= 1000; now += 100) {
        sample(&state, TOF_NO_TARGET_MM, TOF_NO_TARGET_MM, TOF_NO_TARGET_MM, now);
        assert(state.fault_mask == 0 && state.valid_mask == 7 && !state.stop_required);
        assert(state.blocked_mask == 0 && state.urgent_mask == 0);
    }
    // Packing assumes status was already checked. Raw measurement results must
    // pass tof_result_code() first; warnings/errors return unavailable (0).
    sample(&state, 1001, 2000, UINT16_MAX, 1001);
    assert(state.fault_mask == 0 && state.valid_mask == 7);
    obstacle_init(&state, UINT32_MAX - 99u);
    sample(&state, 1000, 1000, 1000, UINT32_MAX - 99u);
    obstacle_update(&state, &empty, 7, 200);
    assert(state.fault_mask == 0 && state.valid_mask == 7);
    obstacle_update(&state, &empty, 7, 201);
    assert(state.fault_mask == 7 && state.valid_mask == 0 && !state.stop_required);
    sample(&state, 1000, 1000, 1000, 202);
    assert(state.fault_mask == 0 && state.valid_mask == 7);
}

int main(void) {
    startup_and_initialization_faults();
    proximity_thresholds_and_hysteresis();
    expiry_and_replayed_results();
    rejected_ranges_are_unavailable_but_device_remains_healthy();
    warnings_after_initialization_do_not_require_an_initial_good_range();
    independent_fault_bits_and_recovery();
    valid_far_ranges_and_uncertain_results();
    no_target_and_clock_rollover();
    puts("Obstacle detector/fault tests passed");
    return 0;
}
