/**
 * @file motor_auto.c
 * @brief Autonomous drive and rotate, and their lease (FR-40, FR-41).
 *
 * Part of the motor driver; see motor_driver.c for the file overview.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_driver.h"
#include "motor_driver_internal.h"

#include "motor_control.h"
#include "motor_lease.h"
#include "motor_types.h"

#include "hardware/sync.h"

/* ---------------- autonomous moves (FR-40, FR-41) ----------------
 * Called by the mapping / Exploration side. One call starts the move; it
 * keeps going (the driver refreshes it) until motor_auto_stop(), an operator
 * STOP, or a safety stop. Needs a fresh heading from the Sensor Pico. */

static bool heading_ready (void)
{
    return g_motor.mc.heading_seen && !g_motor.mc.stale_latched;
}

static int32_t heading_deg_now (void)
{
    return (int32_t)((g_motor.mc.heading + 5u) / 10u) % 360;
}

static motor_error_t start_auto (uint32_t word)
{
    g_motor.auto_lease.duration_ms = 0;
    g_motor.word                   = 0; /* a manual move can't be held now */
    g_motor.auto_seq               = mc_seq(word);
    g_motor.auto_status            = MOTOR_AUTO_RUNNING;
    mc_out_t out;
    mc_out_clear(&out);
    mc_command(&g_motor.mc, word, motor_now_ms(), g_motor.heading_word, &out);
    motor_core_note_result(&out);
    if (g_motor.auto_status == MOTOR_AUTO_REJECTED)
    {
        return MOTOR_ERR_SAFETY;
    }
    g_motor.auto_word = word;
    return MOTOR_OK;
}

static motor_error_t auto_checks (void)
{
    if (g_motor.manual_estop)
    {
        return MOTOR_ERR_SAFETY;
    }
    if (!heading_ready())
    {
        return MOTOR_ERR_NO_HEADING;
    }
    return MOTOR_OK;
}

motor_error_t motor_auto_drive (int32_t speed_percent)
{
    if (speed_percent == 0)
    {
        return motor_auto_stop();
    }
    if (speed_percent < -100 || speed_percent > 100)
    {
        return MOTOR_ERR_PIN;
    }
    uint32_t      irq = save_and_disable_interrupts();
    motor_error_t r   = auto_checks();
    if (r == MOTOR_OK && speed_percent > 0 && g_motor.front_blocked)
    {
        r = MOTOR_ERR_FRONT_BLOCKED; /* only forward is blocked */
    }
    if (r == MOTOR_OK && g_motor.auto_word &&
        mc_state(&g_motor.mc) == MS_AUTO_DRIVE &&
        mc_s8(g_motor.auto_word, 8) == speed_percent)
    {
        /* Same move sent again (the laptop repeats it): keep going, no
         * restart. */
    }
    else if (r == MOTOR_OK)
    {
        /* Hold the heading the rover has now; if it is already driving
         * straight, keep the same target so a speed change doesn't re-aim. */
        int32_t hold =
            (mc_state(&g_motor.mc) == MS_AUTO_DRIVE && g_motor.auto_word)
                ? (int32_t)mc_hdg9(g_motor.auto_word, 16)
                : heading_deg_now();
        r = start_auto(mc_pack_auto_drive(
            motor_core_next_seq(), (int8_t)speed_percent, (uint16_t)hold));
        if (r == MOTOR_OK)
        {
            /* A front obstacle stops a forward move. */
            g_motor.forward_driving = speed_percent > 0;
        }
    }
    restore_interrupts(irq);
    return r;
}

motor_error_t motor_auto_drive_leased (int32_t speed_percent, uint16_t lease_ms)
{
    if (speed_percent == 0 || speed_percent < -100 || speed_percent > 100 ||
        !lease_ms || lease_ms > 500u)
    {
        return MOTOR_ERR_PIN;
    }
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = motor_auto_drive(speed_percent);
    if (result == MOTOR_OK)
    {
        g_motor.auto_lease = (motor_lease_t){.refreshed_ms = motor_now_ms(),
                                             .duration_ms  = lease_ms};
    }
    restore_interrupts(irq);
    return result;
}

bool motor_auto_renew_lease (void)
{
    uint32_t irq = save_and_disable_interrupts();
    bool     ok  = g_motor.auto_word && g_motor.auto_lease.duration_ms &&
              g_motor.auto_status == MOTOR_AUTO_RUNNING &&
              !motor_lease_expired(&g_motor.auto_lease, motor_now_ms());
    if (ok)
    {
        g_motor.auto_lease.refreshed_ms = motor_now_ms();
    }
    else if (g_motor.auto_lease.duration_ms)
    {
        motor_core_hard_stop(MC_STOP_OPERATOR);
    }
    restore_interrupts(irq);
    return ok;
}

motor_error_t motor_auto_rotate_to (int32_t heading_deg)
{
    heading_deg %= 360;
    if (heading_deg < 0)
    {
        heading_deg += 360;
    }
    uint32_t      irq = save_and_disable_interrupts();
    motor_error_t r   = auto_checks();
    if (r == MOTOR_OK)
    {
        r = start_auto(
            mc_pack_auto_rotate(motor_core_next_seq(), (uint16_t)heading_deg));
        if (r == MOTOR_OK)
        {
            /* Turning is not stopped by a front obstacle. */
            g_motor.forward_driving = false;
        }
    }
    restore_interrupts(irq);
    return r;
}

motor_error_t motor_auto_rotate_leased (int32_t heading_deg, uint16_t lease_ms)
{
    if (!lease_ms || lease_ms > 500u)
    {
        return MOTOR_ERR_PIN;
    }
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = motor_auto_rotate_to(heading_deg);
    if (result == MOTOR_OK)
    {
        g_motor.auto_lease = (motor_lease_t){.refreshed_ms = motor_now_ms(),
                                             .duration_ms  = lease_ms};
    }
    restore_interrupts(irq);
    return result;
}

motor_error_t motor_auto_rotate_by (int32_t delta_deg)
{
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t r      = auto_checks();
    int32_t       target = heading_deg_now() + delta_deg;
    restore_interrupts(irq);
    return r == MOTOR_OK ? motor_auto_rotate_to(target) : r;
}

motor_error_t motor_auto_stop (void)
{
    uint32_t irq                   = save_and_disable_interrupts();
    g_motor.auto_lease.duration_ms = 0;
    if (g_motor.auto_word)
    {
        g_motor.auto_word = 0; /* stop refreshing the move */
        mc_out_t out;
        mc_out_clear(&out);
        /* Normal stop: ramp down, then DONE. */
        mc_command(&g_motor.mc,
                   mc_pack_simple(MC_STOP, MC_SRC_AUTO, g_motor.auto_seq),
                   motor_now_ms(), g_motor.heading_word, &out);
        motor_core_note_result(&out);
    }
    restore_interrupts(irq);
    return MOTOR_OK;
}

motor_auto_status_t motor_auto_status (void)
{
    return (motor_auto_status_t)g_motor.auto_status;
}

const char * motor_auto_status_name (motor_auto_status_t s)
{
    switch (s)
    {
    case MOTOR_AUTO_RUNNING:
        return "RUNNING";
    case MOTOR_AUTO_DONE:
        return "DONE";
    case MOTOR_AUTO_HEADING_REACHED:
        return "HEADING_REACHED";
    case MOTOR_AUTO_STOPPED:
        return "STOPPED";
    case MOTOR_AUTO_REJECTED:
        return "REJECTED";
    default:
        return "IDLE";
    }
}
