/**
 * @file motor_safety.c
 * @brief Stops, E-STOP latch, front-obstacle guard and SAFE STOP reason (FR-42,
 * NFR-03).
 *
 * Part of the motor driver; see motor_driver.c for the file overview.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_driver.h"
#include "motor_driver_internal.h"
#include "safety_interlock.h"

#include "motor_control.h"
#include "motor_hw.h"
#include "motor_types.h"

#include "hardware/sync.h"

/* Front guard state used from interrupts, so volatile (BARR-C 1.8.c). */
static volatile uint32_t s_last_sensor_ms; /* last Sensor Pico packet */
static volatile bool     s_front_stop_pending;

/* Called from motor_init(). */
void
motor_safety_init (void)
{
    s_front_stop_pending = false;
}

/* ---------------- stops ---------------- */

/* Dashboard STOP / AUTO_STOP / end of a turn: operator stop.
 * PWM 0 at once + active brake for MC_BRAKE_HOLD_MS, then IDLE. */
motor_error_t
motor_emergency_stop (void)
{
    uint32_t irq = save_and_disable_interrupts();
    motor_core_hard_stop(MC_STOP_OPERATOR);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    restore_interrupts(irq);
    return MOTOR_OK;
}

void
motor_manual_estop (void)
{
    uint32_t irq         = save_and_disable_interrupts();
    g_motor.manual_estop = true;
    motor_core_hard_stop(MC_STOP_ESTOP);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    restore_interrupts(irq);
}

/* Sensor packets only control the temporary forward guard. */
void
motor_safety_receive (bool front_blocked, uint32_t receive_ms)
{
    uint32_t irq = save_and_disable_interrupts();
    if (front_blocked && !g_motor.front_blocked)
    {
        if (g_motor.forward_driving)
        {
            motor_core_hard_stop(MC_STOP_OPERATOR);
        }
        s_front_stop_pending = true;
    }
    g_motor.front_blocked = front_blocked;
    s_last_sensor_ms      = receive_ms;
    restore_interrupts(irq);
}

bool
motor_take_front_stop (void)
{
    uint32_t irq         = save_and_disable_interrupts();
    bool     pending     = s_front_stop_pending;
    s_front_stop_pending = false;
    restore_interrupts(irq);
    return pending;
}

void
motor_safety_poll (void)
{
    uint32_t irq = save_and_disable_interrupts();
    /* Missing readings mean no detected obstacle, never a safety latch. */
    if ((uint32_t) (motor_now_ms() - s_last_sensor_ms) >
        SAFETY_PACKET_TIMEOUT_MS)
    {
        g_motor.front_blocked = false;
    }
    restore_interrupts(irq);
}

void
motor_link_lost (void)
{
    /* Communications loss stops motion without requiring a safety reset. */
    motor_emergency_stop();
}

const char *
motor_safe_stop_reason (void)
{
    switch (g_motor.safe_reason)
    {
    case MOTOR_SAFE_CMD_TIMEOUT:
        return "no command for 0.8 s (link lost?)";
    case MOTOR_SAFE_LINK_LOST:
        return "laptop link lost";
    case MOTOR_SAFE_MANUAL_ESTOP:
        return "manual dashboard E-STOP";
    case MOTOR_SAFE_HEADING_STALE:
        return "heading too old";
    default:
        return "unknown";
    }
}

bool
motor_safety_is_latched (void)
{
    uint32_t irq = save_and_disable_interrupts();
    bool     l   = g_motor.manual_estop;
    restore_interrupts(irq);
    return l;
}

bool
motor_safety_reset (void)
{
    uint32_t irq         = save_and_disable_interrupts();
    g_motor.manual_estop = false;
    motor_core_hard_stop(
        MC_STOP_OPERATOR); /* a reset never restarts an old move */

    mc_out_t out;
    mc_out_clear(&out);
    /* SAFE STOP -> IDLE. */
    mc_command(&g_motor.mc,
               mc_pack_simple(MC_RESET, MC_SRC_MANUAL, motor_core_next_seq()),
               motor_now_ms(), g_motor.heading_word, &out);
    mc_set_system_ready(&g_motor.mc, true);
    if (mc_state(&g_motor.mc) != MS_SAFE_STOP)
    {
        g_motor.safe_reason = MOTOR_SAFE_NONE;
    }
    restore_interrupts(irq);
    return true;
}

/* End of file. */
