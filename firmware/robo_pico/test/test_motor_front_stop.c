/* Real motor driver/control/interlock, with only the Pico and GPIO mocked.
 * gcc -std=c11 -Wall -Wextra -Werror -DMC_ALLOW_MANUAL_WITHOUT_HEADING=1
 *   -Istubs -I../include test_motor_front_stop.c ../src/motor_driver.c
 *   ../src/motor_control.c ../src/safety_interlock.c -lm -o <test executable>
 */
#include <assert.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "motor_driver.h"
#include "motor_hw.h"
#include "safety_interlock.h"

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
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_ERR_SAFETY);
    motor_safety_receive(false, clock_ms);
    assert(motor_safety_reset());
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);
    tick(NULL);
    assert(motor_motion_state() == MS_MANUAL);

    motor_safety_receive(true, clock_ms);
    assert(!motor_safety_is_latched());
    assert(motor_motion_state() == MS_IDLE);
    for (int i = 0; i < MC_WHEELS; ++i) assert(output.wheel[i] == 0);
    assert(motor_take_front_stop());
    assert(!motor_take_front_stop());
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_ERR_FRONT_BLOCKED);

    for (int dir = DIRECTION_BACKWARD; dir < DIRECTION_STOP; ++dir) {
        assert(motor_set_direction((motor_direction_t)dir, 50) == MOTOR_OK);
        unsigned int before = applied;
        motor_safety_receive(true, clock_ms);
        assert(applied == before);  /* Continuing detection must not stop escape. */
        assert(motor_motion_state() == MS_MANUAL);
        assert(!motor_take_front_stop());
    }
    assert(arm_set_direction(ARM_UP) == MOTOR_OK);
    assert(claw_set_direction(CLAW_OPEN) == MOTOR_OK);

    motor_safety_receive(false, clock_ms);
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);
    motor_safety_receive(true, clock_ms);
    motor_safety_receive(false, clock_ms);
    assert(motor_take_front_stop());  /* Report even a brief detection. */
    assert(motor_motion_state() == MS_IDLE);
    tick(NULL);
    assert(motor_motion_state() == MS_IDLE); /* Clearing never resumes an old move. */
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_OK);

    clock_ms += SAFETY_PACKET_TIMEOUT_MS + 1;
    motor_safety_poll();
    assert(motor_safety_is_latched());
    assert(motor_set_direction(DIRECTION_BACKWARD, 50) == MOTOR_ERR_SAFETY);
    motor_safety_receive(true, clock_ms);
    assert(motor_safety_reset());
    assert(motor_set_direction(DIRECTION_FORWARD, 50) == MOTOR_ERR_FRONT_BLOCKED);
    assert(motor_set_direction(DIRECTION_BACKWARD, 50) == MOTOR_OK);
    puts("Front stop driver tests passed");
    return 0;
}
