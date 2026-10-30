// Native tests run the real driver and filter against simulated I2C transfers.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "hardware/i2c.h"
#include "imu_filter.h"
#include "sensor_config.h"

i2c_inst_t test_i2c;
i2c_inst_t test_other_i2c;
_Static_assert(IMU_SDA_PIN == 6 && IMU_SCL_PIN == 7, "IMU wiring must match GP6/GP7");
_Static_assert(TOF_SDA_PIN == 4 && TOF_SCL_PIN == 5, "ToF stays on GP4/GP5");
_Static_assert(!ULTRA_LEFT_ENABLED, "Legacy left ultrasonic must release IMU pins");
static uint8_t registers[256], selected;
static bool fail_write, fail_read;
static unsigned writes, pins, pulls;
static uint32_t simulated_ms, accel_configured_ms;
static bool simulate_range_transition;
static bool ignore_accel_config_write;
static unsigned calibration_script, script_samples;
static void raw(unsigned word, int16_t value);

unsigned i2c_init(i2c_inst_t *port, unsigned baud) {
    assert(port == IMU_I2C_PORT && baud == 100000); return baud;
}
void gpio_set_function(unsigned pin, unsigned function) {
    assert((pin == IMU_SDA_PIN || pin == IMU_SCL_PIN) && function == 3);
    pins++;
}
void gpio_pull_up(unsigned pin) { assert(pin == IMU_SDA_PIN || pin == IMU_SCL_PIN); pulls++; }
void sleep_ms(uint32_t ms) { assert(ms == 10 || ms == 100 || ms == 1000); simulated_ms += ms; }
int i2c_write_timeout_us(i2c_inst_t *port, uint8_t addr, const uint8_t *bytes,
                         size_t count, bool nostop, unsigned timeout_us) {
    assert(port == IMU_I2C_PORT && addr == MPU6500_ADDR && timeout_us == 5000);
    if (fail_write) return -1;
    selected = bytes[0];
    if (count == 1) { assert(nostop); return 1; }
    assert(count == 2 && !nostop);
    if (!(ignore_accel_config_write && selected == 0x1C)) registers[selected] = bytes[1];
    if (selected == 0x1C) accel_configured_ms = simulated_ms;
    writes++;
    return 2;
}
int i2c_read_timeout_us(i2c_inst_t *port, uint8_t addr, uint8_t *bytes,
                        size_t count, bool nostop, unsigned timeout_us) {
    assert(port == IMU_I2C_PORT && addr == MPU6500_ADDR && !nostop && timeout_us == 5000);
    if (fail_read) return (int)count - 1; // Short read must be rejected.
    if (simulate_range_transition && selected == 0x3B && count == 14) {
        // A latched 1 g output from the old +/-2 g range is 16384 counts.
        // New +/-4 g samples become available one 100 Hz sample period later.
        bool fresh = (uint32_t)(simulated_ms - accel_configured_ms) >= 10;
        memset(bytes, 0, count);
        bytes[4] = fresh ? 0x20 : 0x40;
        return (int)count;
    }
    if (calibration_script && selected == 0x3B && count == 14) {
        ++script_samples;
        raw(0, 0); raw(1, 0); raw(2, 8192);
        raw(4, 0); raw(5, 0); raw(6, 0);
        if (calibration_script == 1 || (calibration_script == 3 && script_samples <= 500))
            raw(4, (script_samples & 1u) ? 131 : -131);
        if (calibration_script == 2) {
            raw(0, (script_samples & 1u) ? 819 : -819);
            raw(2, 8151); // About 1 g magnitude, but changing direction.
        }
    }
    memcpy(bytes, registers + selected, count); return (int)count;
}
static void near(float actual, float expected, float tolerance) {
    assert(fabsf(actual - expected) < tolerance);
}
static void raw(unsigned word, int16_t value) {
    registers[0x3B + 2 * word] = (uint8_t)((uint16_t)value >> 8);
    registers[0x3C + 2 * word] = (uint8_t)value;
}
static void test_driver(void) {
    assert(IMU_I2C_PORT == i2c1 && TOF_I2C_PORT == i2c0);
    assert(IMU_I2C_PORT != TOF_I2C_PORT);
    registers[0x75] = 0x71;
    assert(!mpu6500_init()); // Do not silently accept MPU9250/other devices.
    assert(mpu6500_device_id() == 0x71);
    assert(strstr(mpu6500_last_error(), "mismatch") != NULL);
    registers[0x75] = 0x70;
    fail_write = true; assert(!mpu6500_init()); fail_write = false;
    assert(strstr(mpu6500_last_error(), "read failed") != NULL);
    assert(mpu6500_init());
    assert(mpu6500_device_id() == 0x70);
    assert(writes == 8 && pins == 6 && pulls == 6);
    assert(registers[0x6B] == 1 && registers[0x1A] == 3 && registers[0x19] == 9);
    assert(registers[0x1B] == 0x18 && registers[0x1C] == 8);
    assert(registers[0x6C] == 0 && registers[0x1D] == 3);
    raw(0, -8192); raw(1, 4096); raw(2, 8192);
    raw(3, 12345); // Temperature must not become gyro X.
    raw(4, 164); raw(5, -328); raw(6, 492);
    mpu6500_reading_t reading;
    assert(mpu6500_read(&reading));
    near(reading.accel_g[0], -1, 0.0001f); near(reading.accel_g[1], .5f, 0.0001f);
    near(reading.gyro_dps[0], 10, .0001f); near(reading.gyro_dps[1], -20, .0001f);
    near(reading.gyro_dps[2], 30, .0001f);
    fail_read = true; assert(!mpu6500_read(&reading)); fail_read = false;
    assert(!mpu6500_read(NULL));
    raw(0, 0); raw(1, 0); raw(2, 8192);
    raw(4, 33); raw(5, -16); raw(6, 49);
    assert(mpu6500_calibrate());
    assert(strcmp(mpu6500_last_error(), "OK") == 0);
    assert(mpu6500_read(&reading));
    for (int axis = 0; axis < 3; ++axis) near(reading.gyro_dps[axis], 0, .0001f);
    raw(6, 410); assert(!mpu6500_calibrate()); // Implausibly large 25 deg/s mean.
    assert(strstr(mpu6500_last_error(), "gyro") != NULL);
    raw(6, 0); raw(2, 0); assert(!mpu6500_calibrate()); // No gravity vector.
    assert(strstr(mpu6500_last_error(), "magnitude") != NULL);
    fail_read = true; assert(!mpu6500_calibrate()); fail_read = false;
    assert(strstr(mpu6500_last_error(), "transfer failed") != NULL);
}
static void test_calibration_windows(void) {
    raw(0, 0); raw(1, 0); raw(2, 8192);
    raw(4, 131); raw(5, -98); raw(6, 0); // Stable +8/-6 deg/s offset.
    assert(mpu6500_calibrate());
    assert(strstr(mpu6500_last_error(), "suspicious") != NULL);
    mpu6500_reading_t reading;
    assert(mpu6500_read(&reading));
    near(reading.gyro_dps[0], 0, .001f); near(reading.gyro_dps[1], 0, .001f);

    calibration_script = 1; script_samples = 0;
    assert(!mpu6500_calibrate()); // Noisy rotation with a zero mean must fail.
    assert(script_samples == 1000);
    assert(strstr(mpu6500_last_error(), "motion/noise") != NULL);

    calibration_script = 2; script_samples = 0;
    assert(!mpu6500_calibrate()); // Changing gravity direction also means motion.
    assert(strstr(mpu6500_last_error(), "motion/noise") != NULL);

    calibration_script = 3; script_samples = 0;
    assert(mpu6500_calibrate()); // Five windows move, then a stable window is accepted.
    assert(script_samples == 600);
    assert(strcmp(mpu6500_last_error(), "OK") == 0);
    calibration_script = 0;
}
static void test_range_setup_and_settling(void) {
    registers[0x75] = 0x70;
    registers[0x1C] = 0;
    ignore_accel_config_write = true;
    assert(!mpu6500_init());
    assert(strstr(mpu6500_last_error(), "readback mismatch") != NULL);
    ignore_accel_config_write = false;

    simulate_range_transition = true;
    assert(mpu6500_init());
    assert((uint32_t)(simulated_ms - accel_configured_ms) >= 10);
    mpu6500_reading_t reading;
    assert(mpu6500_read(&reading));
    near(reading.accel_g[2], 1, .0001f);
    assert(mpu6500_calibrate());
    simulate_range_transition = false;

    // A persistent 2 g value must still fail, rather than widening the limits
    // or silently dividing every measurement by two.
    raw(0, 0); raw(1, 0); raw(2, 16384);
    raw(4, 0); raw(5, 0); raw(6, 0);
    assert(!mpu6500_calibrate());
    assert(strstr(mpu6500_last_error(), "magnitude") != NULL);
}
static void test_filter(void) {
    imu_filter_t state = {0};
    mpu6500_reading_t reading = {{-.5f, 0, .8660254f}, {0, 0, 0}};
    assert(imu_filter_update(&state, &reading, 1000, -1));
    near(state.pitch_deg, 30, .001f); near(state.roll_deg, 0, .001f);
    state = (imu_filter_t){0};
    reading = (mpu6500_reading_t){{0, 0, 1}, {0, 0, 0}};
    assert(imu_filter_update(&state, &reading, 1000, -1));
    reading.gyro_dps[2] = -90;
    for (unsigned i = 1; i <= 100; ++i)
        assert(imu_filter_update(&state, &reading, 1000 + i * 10000u, -1));
    near(state.heading_deg, 90, .001f);
    reading.gyro_dps[2] = 180;
    for (unsigned i = 1; i <= 100; ++i)
        assert(imu_filter_update(&state, &reading, 1001000 + i * 10000u, -1));
    near(state.heading_deg, 270, .001f);

    // Elapsed-time filter weighting is equivalent at 100 Hz and 20 Hz.
    imu_filter_t fast = {0}, slow = {0};
    reading = (mpu6500_reading_t){{0, 0, 1}, {0, 0, 0}};
    assert(imu_filter_update(&fast, &reading, 1, -1)); slow = fast;
    reading.accel_g[0] = -.5f; reading.accel_g[2] = .8660254f;
    for (unsigned i = 1; i <= 5; ++i)
        assert(imu_filter_update(&fast, &reading, 1 + i * 10000u, -1));
    assert(imu_filter_update(&slow, &reading, 50001, -1));
    near(fast.pitch_deg, slow.pitch_deg, .001f);

    // Crossing the +/-180 roll boundary must not pull the estimate toward zero.
    state = (imu_filter_t){0};
    reading = (mpu6500_reading_t){{0, .0174524f, -.9998477f}, {0, 0, 0}};
    assert(imu_filter_update(&state, &reading, 1, -1));
    reading.accel_g[1] *= -1;
    assert(imu_filter_update(&state, &reading, 10001, -1));
    near(state.roll_deg, 179.04f, .01f);

    // 64-bit timestamps continue across the 32-bit microsecond boundary.
    state = (imu_filter_t){0};
    assert(imu_filter_update(&state, &reading, UINT64_C(0xFFFFFF00), -1));
    assert(imu_filter_update(&state, &reading, UINT64_C(0x100002610), -1));
    uint64_t last = state.last_sample_us;
    assert(!imu_filter_update(&state, &reading, last, -1));
    reading.gyro_dps[0] = NAN;
    assert(!imu_filter_update(&state, &reading, last + 10000, -1));
    assert(state.last_sample_us == last);
    reading.gyro_dps[0] = 0;
    assert(!imu_filter_update(&state, &reading, last + 500001, -1));
    assert(state.timing_fault);
    assert(!imu_filter_update(&state, &reading, last + 510001, -1));
}
static void test_heading_mounting_and_half_turn(void) {
    for (int axis = 0; axis < 3; ++axis) {
        for (int sign = -1; sign <= 1; sign += 2) {
            imu_filter_t state = {0};
            mpu6500_reading_t reading = {{0}, {0}};
            reading.accel_g[axis] = (float)sign;
            assert(imu_filter_update(&state, &reading, 1000, -1));
            reading.gyro_dps[axis] = -90.0f * sign;
            for (unsigned sample = 1; sample <= 200; ++sample)
                assert(imu_filter_update(&state, &reading, 1000 + sample * 10000u, -1));
            near(state.heading_deg, 180, .01f);
        }
    }
}
int main(void) {
    test_driver(); test_range_setup_and_settling(); test_calibration_windows(); test_filter(); test_heading_mounting_and_half_turn();
    puts("MPU6500 driver, calibration, tilt, heading, wrap and fault tests passed");
    return 0;
}
