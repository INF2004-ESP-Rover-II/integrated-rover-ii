/**
 * @file motor_manual.c
 * @brief Dashboard (manual) moves and parallel park.
 *
 * Part of the motor driver; see motor_driver.c for the file overview.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_driver.h"
#include "motor_driver_internal.h"

#include "motor_config.h"
#include "motor_control.h"
#include "motor_types.h"

#include "hardware/sync.h"

static int32_t clamp100 (int32_t v)
{
    return v < 0 ? 0 : (v > 100 ? 100 : v);
}

/* Direction -> body motion (vx forward, vy right, w clockwise).
 * Diagonals use half on each axis, so the two pushing wheels get 's'
 * and the other two stop. */
static bool dir_to_motion (motor_direction_t d, int32_t s, int32_t * vx,
                           int32_t * vy, int32_t * w)
{
    int32_t h = s / 2;
    *vx = *vy = *w = 0;
    switch (d)
    {
    case DIRECTION_FORWARD:
        *vx = s;
        break;
    case DIRECTION_BACKWARD:
        *vx = -s;
        break;
    case DIRECTION_LEFT:
        *vy = -s;
        break;
    case DIRECTION_RIGHT:
        *vy = s;
        break;
    case DIRECTION_ROTATE_LEFT:
        *w = -s;
        break;
    case DIRECTION_ROTATE_RIGHT:
        *w = s;
        break;
    case DIRECTION_FRONT_LEFT:
        *vx = h;
        *vy = -h;
        break;
    case DIRECTION_FRONT_RIGHT:
        *vx = h;
        *vy = h;
        break;
    case DIRECTION_BACK_LEFT:
        *vx = -h;
        *vy = -h;
        break;
    case DIRECTION_BACK_RIGHT:
        *vx = -h;
        *vy = h;
        break;
    default:
        return false;
    }
    return true;
}

int32_t motor_move_speed (motor_direction_t direction)
{
    switch (direction)
    {
    case DIRECTION_FORWARD:
    case DIRECTION_BACKWARD:
        return MC_DASH_STRAIGHT_PCT;
    case DIRECTION_LEFT:
    case DIRECTION_RIGHT:
        return MC_DASH_STRAFE_PCT;
    case DIRECTION_ROTATE_LEFT:
    case DIRECTION_ROTATE_RIGHT:
        return MC_DASH_ROTATE_PCT;
    default:
        return MC_DASH_DIAGONAL_PCT;
    }
}

motor_error_t motor_set_direction (motor_direction_t direction,
                                   int32_t           speed_percent)
{
    if (direction == DIRECTION_STOP)
    {
        return motor_emergency_stop();
    }
    if (speed_percent < 0 || speed_percent > 100)
    {
        return MOTOR_ERR_PIN;
    }

    int32_t vx, vy, w;
    if (!dir_to_motion(direction, speed_percent, &vx, &vy, &w))
    {
        return MOTOR_ERR_PIN;
    }

    uint32_t      irq = save_and_disable_interrupts();
    motor_error_t r   = MOTOR_ERR_SAFETY;
    if (!g_motor.manual_estop && direction == DIRECTION_FORWARD &&
        g_motor.front_blocked)
    {
        r = MOTOR_ERR_FRONT_BLOCKED;
    }
    else if (!g_motor.manual_estop)
    {
        /* Same move again = refresh (same word). A different move gets a
         * new sequence number. */
        uint32_t word = 0;
        if (g_motor.word && mc_state(&g_motor.mc) == MS_MANUAL)
        {
            word = mc_pack_manual_drive(mc_seq(g_motor.word), (int8_t)vx,
                                        (int8_t)vy, (int8_t)w);
        }
        if (word == 0 || word != g_motor.word)
        {
            word = mc_pack_manual_drive(motor_core_next_seq(), (int8_t)vx,
                                        (int8_t)vy, (int8_t)w);
        }
        r = motor_core_submit(word);
        if (r == MOTOR_OK)
        {
            g_motor.word            = word;
            g_motor.forward_driving = direction == DIRECTION_FORWARD;
            g_motor.last_call_ms    = motor_now_ms();
        }
    }
    restore_interrupts(irq);
    return r;
}

/* Parallel park (strafe for MC_PARK_DURATION_MS, then stops by itself).
 * direction: -1 = left, +1 = right. power 10..100, 0 = default. */
motor_error_t motor_parallel_park (int32_t direction, int32_t power_percent)
{
    int32_t       p = power_percent > 0 ? clamp100(power_percent) : MC_PARK_PCT;
    uint32_t      irq = save_and_disable_interrupts();
    motor_error_t r   = MOTOR_ERR_SAFETY;
    if (!g_motor.manual_estop)
    {
        uint32_t word = mc_pack_manual_park(motor_core_next_seq(),
                                            (int8_t)(direction < 0 ? -p : p));
        r             = motor_core_submit(word);
        if (r == MOTOR_OK)
        {
            g_motor.forward_driving = false;
        }
        g_motor.word = 0; /* park ends by itself: no auto refresh */
    }
    restore_interrupts(irq);
    return r;
}
