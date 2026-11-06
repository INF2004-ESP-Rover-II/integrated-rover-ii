/* Real motor driver/control/interlock, with only the Pico and GPIO mocked.
 * gcc -std=c11 -Wall -Wextra -Werror -DMC_ALLOW_MANUAL_WITHOUT_HEADING=1
 *   -Istubs -I../include test_motor_front_stop.c ../src/motor_driver.c
 *   ../src/motor_control.c ../src/safety_interlock.c -lm -o <test executable>
 */
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "motor_config.h"
#include "motor_driver.h"
#include "motor_hw.h"
#include "safety_interlock.h"
#include "../../common/obstacle_threshold.h"

static uint32_t clock_ms = 100;
static unsigned int applied;
static mc_out_t output;
static bool (*tick)(struct repeating_timer *);

uint32_t get_absolute_time(void) { return clock_ms; }
uint32_t to_ms_since_boot(uint32_t value) { return value; }
uint32_t save_and_disable_interrupts(void) { return 0; }
void restore_interrupts(uint32_t state) { (void)state; }
bool add_repeating_timer_ms(int32_t delay,
                            bool (*callback)(struct repeating_timer *),
                            void *user_data, struct repeating_timer *timer) {
    (void)delay; (void)user_data; (void)timer;
    tick = callback;
    return true;
}
void motor_hw_safe_boot(void) {}
void motor_hw_init(void) {}
void motor_hw_apply(const mc_out_t *out) { output = *out; ++applied; }
void motor_hw_arm(mh_dir_t dir) { (void)dir; }
void motor_hw_claw(mh_dir_t dir) { (void)dir; }

int main(void) {
    assert(motor_init() == MOTOR_OK);
    assert(motor_zero_heading() == MOTOR_ERR_NO_HEADING);
    assert(!motor_heading_sample_since(clock_ms));
    // No packets or IMU at startup must not lock manual movement.
    assert(!motor_safety_is_latched());
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);
    tick(NULL);

    const uint16_t ranges[] = {0, 1, 149, 150, 151, 199, 200, 1000, 1023, 65535};
    for (unsigned i = 0; i < sizeof ranges / sizeof ranges[0]; ++i) {
        for (int fault = 0; fault <= 1; ++fault) {
            bool detected = front_obstacle_detected(ranges[i], fault != 0);
            bool expected = !fault && ranges[i] > 0 && ranges[i] < 150;
            assert(detected == expected);
            motor_safety_receive(detected, clock_ms);
            assert(!motor_safety_is_latched());
            assert(motor_set_direction(DIRECTION_FORWARD, 50) ==
                   (expected ? MOTOR_ERR_FRONT_BLOCKED : MOTOR_OK));
            for (int dir = DIRECTION_BACKWARD; dir < DIRECTION_STOP; ++dir)
                assert(motor_set_direction((motor_direction_t)dir, 50) == MOTOR_OK);
        }
    }
    motor_safety_receive(false, clock_ms);
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);
    tick(NULL);
    motor_safety_receive(true, clock_ms);
    assert(motor_motion_state() == MS_IDLE);
    for (int i = 0; i < MC_WHEELS; ++i) assert(output.wheel[i] == 0);
    assert(motor_set_direction(DIRECTION_BACKWARD, 50) == MOTOR_OK);
    unsigned before = applied;
    motor_safety_receive(false, clock_ms);
    motor_safety_receive(true, clock_ms);
    assert(applied == before); // New front detections never interrupt reverse.

    // Sensor silence removes old forward inhibition without creating a latch.
    clock_ms += SAFETY_PACKET_TIMEOUT_MS + 1;
    motor_safety_poll();
    assert(!motor_safety_is_latched());
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);
    // An expired command stops, and the next command works without reset.
    clock_ms += 1000;
    tick(NULL);
    assert(motor_motion_state() == MS_IDLE);
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);

    // Only explicit manual E-STOP locks every direction; readings cannot release it.
    motor_manual_estop();
    assert(motor_safety_is_latched());
    motor_safety_receive(false, clock_ms);
    assert(motor_safety_is_latched());
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_ERR_SAFETY);
    assert(motor_set_direction(DIRECTION_BACKWARD, 50) == MOTOR_ERR_SAFETY);
    assert(motor_safety_reset());
    assert(!motor_safety_is_latched());
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);
    motor_link_lost();
    assert(!motor_safety_is_latched());
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);

    motor_emergency_stop();
    assert(motor_auto_rotate_to(90) == MOTOR_ERR_NO_HEADING);
    motor_heading_update(0, true);
    assert(motor_heading_sample_since(clock_ms));
    assert(!motor_heading_sample_since(clock_ms + 1));
    tick(NULL);
    assert(motor_auto_rotate_to(90) == MOTOR_OK);
    tick(NULL);
    assert(motor_auto_status() == MOTOR_AUTO_RUNNING);
    clock_ms += 20;
    motor_heading_update(89, true);
    tick(NULL);
    assert(motor_auto_status() == MOTOR_AUTO_HEADING_REACHED);
    assert(motor_motion_state() == MS_IDLE && output.brake);
    tick(NULL);
    assert(motor_auto_status() == MOTOR_AUTO_HEADING_REACHED);
    assert(motor_motion_state() == MS_IDLE);
    motor_manual_estop();
    assert(motor_auto_rotate_to(180) == MOTOR_ERR_SAFETY);
    assert(motor_zero_heading() == MOTOR_OK);
    assert(motor_safety_is_latched()); // Zeroing must not release manual E-STOP.
    assert(motor_heading_epoch() == 1);
    assert(fabsf(motor_heading_relative(89)) < 0.01f);
    assert(fabsf(motor_heading_relative(179) - 90) < 0.01f);
    assert(motor_safety_reset());
    motor_heading_update(179, true);
    tick(NULL);
    assert(motor_auto_rotate_to(90) == MOTOR_OK);
    tick(NULL);
    assert(motor_auto_status() == MOTOR_AUTO_HEADING_REACHED);
    assert(motor_zero_heading() == MOTOR_OK); // A repeated zero uses the new raw heading.
    assert(fabsf(motor_heading_offset() - 179) < 0.01f);
    assert(fabsf(motor_heading_relative(179)) < 0.01f);
    motor_heading_update(350, true);
    assert(motor_zero_heading() == MOTOR_OK);
    assert(fabsf(motor_heading_relative(10) - 20) < 0.01f);
    assert(fabsf(motor_heading_relative(349) - 359) < 0.01f);
    uint32_t epoch = motor_heading_epoch();
    clock_ms += MC_HEADING_STALE_MS + 1;
    motor_heading_update(NAN, true); // Invalid readings cannot refresh the reference.
    assert(motor_zero_heading() == MOTOR_ERR_NO_HEADING);
    assert(motor_heading_epoch() == epoch);
    motor_emergency_stop();motor_safety_receive(false,clock_ms);motor_heading_update(0,true);
    motor_executor_require(true);
    assert(motor_auto_drive(30)==MOTOR_OK);tick(NULL);
    clock_ms+=301;motor_heading_update(0,true);tick(NULL);
    assert(motor_executor_expired()&&motor_auto_status()!=MOTOR_AUTO_RUNNING);
    motor_executor_renew();clock_ms+=20;tick(NULL);
    assert(motor_executor_expired()); /* Renewal cannot erase an expired lease. */
    motor_executor_require(false);motor_executor_require(true);
    assert(!motor_executor_expired());
    motor_emergency_stop();motor_executor_require(false);
    puts("Front range, timeout and manual E-STOP tests passed");
    return 0;
}
