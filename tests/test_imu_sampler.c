// Runs the real core-1 loop against a simulated clock and IMU. The simulated
// main loop never polls during the turn, reproducing blocked core-0 sensor work.
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "imu_sampler.h"
#include "pico/critical_section.h"

static void (*core1_entry)(void);
static jmp_buf worker_done;
static bool worker_running;
static unsigned reads, launches, mounting_axis, error_at, clip_at;
static uint64_t clock_us, read_cost_us = 100;

void multicore_launch_core1(void (*entry)(void)) { core1_entry = entry; launches++; }
uint64_t time_us_64(void) { return clock_us; }
void sleep_us(uint64_t us) { clock_us += us; }
void critical_section_init(critical_section_t *section) { section->held = false; }
void critical_section_enter_blocking(critical_section_t *section) {
    assert(!section->held); section->held = true;
}
void critical_section_exit(critical_section_t *section) {
    assert(section->held); section->held = false;
    if (worker_running && reads == 201) longjmp(worker_done, 1);
}
bool mpu6500_read(mpu6500_reading_t *reading) {
    reads++; clock_us += read_cost_us;
    if (reads == error_at) return false;
    memset(reading, 0, sizeof(*reading));
    reading->accel_g[mounting_axis] = 1.0f;
    // 100 samples * 10 ms * 180 deg/s = a half turn. No telemetry consumer
    // runs until the turn has finished and another full second has elapsed.
    if (reads >= 2 && reads <= 101) reading->gyro_dps[mounting_axis] = -180;
    if (reads == clip_at) reading->gyro_dps[2] = 1998;
    return true;
}
static imu_snapshot_t run_worker(void) {
    clock_us = 10000; reads = 0;
    worker_running = true;
    if (setjmp(worker_done) == 0) core1_entry();
    worker_running = false;
    imu_snapshot_t snapshot;
    imu_sampler_get_snapshot(&snapshot);
    return snapshot;
}
int main(void) {
    imu_snapshot_t snapshot;
    imu_sampler_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.samples == 0);
    imu_sampler_start(); imu_sampler_start(); assert(launches == 1);
    for (mounting_axis = 0; mounting_axis < 3; ++mounting_axis) {
        snapshot = run_worker();
        assert(snapshot.valid && snapshot.samples == 201);
        assert(fabsf(snapshot.orientation.heading_deg - 180) < .02f);
        assert(snapshot.max_gap_us == 10000 && snapshot.overruns == 0);
    }
    mounting_axis = 2;
    error_at = 50;
    snapshot = run_worker();
    assert(snapshot.read_errors == 1 && snapshot.valid);
    error_at = 0; clip_at = 50;
    snapshot = run_worker();
    assert(snapshot.clipped_samples == 1 && !snapshot.valid);
    assert(snapshot.orientation.timing_fault);
    clip_at = 0; read_cost_us = 30000;
    snapshot = run_worker();
    assert(snapshot.overruns > 0 && snapshot.max_gap_us == 40000);
    puts("Core-1 IMU half-turn, sideways mounting, snapshot, read error, saturation and overrun checks passed");
    return 0;
}
