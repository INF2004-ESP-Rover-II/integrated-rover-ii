/**
 * @file motor_arm_claw.c
 * @brief Arm and claw: plain on/off GPIO.
 *
 * Part of the motor driver; see motor_driver.c for the file overview.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_driver.h"
#include "motor_driver_internal.h"

#include "motor_hw.h"

#include "hardware/sync.h"

/* ---------------- arm / claw: plain on/off GPIO ---------------- */

motor_error_t arm_set_direction (arm_direction_t direction)
{
    uint32_t      irq = save_and_disable_interrupts();
    motor_error_t r   = MOTOR_ERR_SAFETY;
    if (!g_motor.manual_estop || direction == ARM_STOP)
    {
        motor_hw_arm(direction == ARM_DOWN ? MH_FWD
                     : direction == ARM_UP ? MH_REV
                                           : MH_OFF);
        r = MOTOR_OK;
    }
    restore_interrupts(irq);
    return r;
}

motor_error_t claw_set_direction (claw_direction_t direction)
{
    uint32_t      irq = save_and_disable_interrupts();
    motor_error_t r   = MOTOR_ERR_SAFETY;
    if (!g_motor.manual_estop || direction == CLAW_STOP)
    {
        motor_hw_claw(direction == CLAW_CLOSE  ? MH_FWD
                      : direction == CLAW_OPEN ? MH_REV
                                               : MH_OFF);
        r = MOTOR_OK;
    }
    restore_interrupts(irq);
    return r;
}
