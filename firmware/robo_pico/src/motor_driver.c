/**
 * @file motor_driver.c
 * @brief Core of the motor driver: shared state, the 20 ms update timer and
 *        the helpers the other driver files use.
 *
 * The old motor_driver.h API now runs on the Subsystem 6 Motion Control
 * module (motor_control.c + motor_hw.c). main.c keeps calling
 * motor_set_direction(), motor_emergency_stop(), etc. exactly as before.
 * Underneath, every call becomes a packed command word for the Motion
 * Control state machine (Week 6 Section 6.6), which adds:
 *   - one fixed 20 ms update (repeating timer) for ramping + heading hold
 *   - inverse kinematics for all omni moves (FR-38)
 *   - ramp up / ramp down, start boost, deadband (FR-42)
 *   - heading hold from the Sensor Pico heading (FR-41)
 *   - active brake (IN1 = IN2, EN on) instead of reverse thrust
 *   - the arm and claw as plain on/off GPIO, so they no longer share
 *     PWM slice 5 with the back wheels (prof feedback)
 *
 * The driver is split over these files (public API in motor_driver.h):
 *   motor_driver.c    state, 20 ms timer interrupt, helpers, motor_init
 *   motor_manual.c    dashboard moves and parallel park
 *   motor_auto.c      autonomous drive / rotate and the lease (FR-40, FR-41)
 *   motor_heading.c   heading input, offset and zeroing
 *   motor_safety.c    stops, E-STOP latch, front guard, SAFE STOP reason
 *   motor_arm_claw.c  arm and claw GPIO
 *
 * This build has no FreeRTOS, so motor_task.c is NOT used here (it is used
 * by motor_bench/). Shared state is protected the same way the old driver
 * did it: interrupts off around every access.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_driver.h"
#include "motor_driver_internal.h"
#include "config.h"

#include "motor_control.h"
#include "motor_config.h"
#include "motor_hw.h"
#include "motor_types.h"
#include "motor_lease.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"

/* Send the movement-direction byte to the Sensor Pico (GP12 -> GP17).
 * Off until the Sensor Pico side reads it (agree with the Collision
 * Detection owner first), so nothing unexpected arrives on that line. */
#ifndef MOTOR_SEND_DIR_BYTE
#define MOTOR_SEND_DIR_BYTE 0
#endif

/* main.c and the dashboard resend a held move every 200-400 ms, but Motion
 * Control times out after MC_CMD_TIMEOUT_MS (300 ms). Until they resend
 * every 100 ms, this driver keeps refreshing the last move itself for
 * REFRESH_WINDOW_MS after each call, so the total timeout stays at the
 * team's TIMEOUT_MS (800 ms, config.h), the same as the old driver. */
#define REFRESH_WINDOW_MS (TIMEOUT_MS - MC_CMD_TIMEOUT_MS)

/* Used from interrupts, so volatile (BARR-C 1.8.c; see
 * motor_driver_internal.h). Only the timer handle is not. */
motor_ctx_t g_motor;

static struct repeating_timer s_timer;

/* Turn a move result for the running auto command into its status. */
void motor_core_note_result (const mc_out_t * o)
{
    if (!o->result_valid || g_motor.auto_status != MOTOR_AUTO_RUNNING)
    {
        return;
    }
    if (((o->result_byte >> 2) & 0xFu) != g_motor.auto_seq)
    {
        return; /* not the auto move */
    }
    switch ((move_result_t)(o->result_byte & 0x3u))
    {
    case MR_DONE:
        g_motor.auto_status = MOTOR_AUTO_DONE;
        break;
    case MR_HEADING_REACHED:
        g_motor.auto_status = MOTOR_AUTO_HEADING_REACHED;
        break;
    case MR_STOPPED:
        g_motor.auto_status = MOTOR_AUTO_STOPPED;
        break;
    case MR_REJECTED:
        g_motor.auto_status = MOTOR_AUTO_REJECTED;
        break;
    default:
        break;
    }
    if (g_motor.auto_status != MOTOR_AUTO_RUNNING)
    {
        g_motor.auto_word = 0;
    }
}

void motor_core_apply (const mc_out_t * o)
{
#if MOTOR_SEND_DIR_BYTE
    if (o->dir_send)
    {
        motor_hw_send_dir(o->dir_byte); /* direction before motion */
    }
#endif
    motor_hw_apply(o);
}

/* ---------------- fixed 20 ms update (timer interrupt) ------------- */

static bool motion_tick_isr (struct repeating_timer * t)
{
    (void)t;
    uint32_t irq = save_and_disable_interrupts();
    uint32_t nm  = motor_now_ms();
    uint32_t hw  = g_motor.heading_word;
    mc_out_t out;

    if (g_motor.word)
    {
        if ((uint32_t)(nm - g_motor.last_call_ms) <= REFRESH_WINDOW_MS)
        {
            mc_out_clear(&out);
            mc_command(&g_motor.mc, g_motor.word, nm, hw,
                       &out); /* same word = refresh */
        }
        else
        {
            /* Caller went quiet: Motion Control times out (FR-42). */
            g_motor.word = 0;
        }
    }
    if (g_motor.auto_word && motor_lease_expired(&g_motor.auto_lease, nm))
    {
        motor_core_hard_stop(MC_STOP_OPERATOR);
    }
    if (g_motor.auto_word)
    {
        /* Keep the auto move alive. */
        mc_out_clear(&out);
        mc_command(&g_motor.mc, g_motor.auto_word, nm, hw, &out);
        motor_core_note_result(&out);
    }
    motor_state_t before = mc_state(&g_motor.mc);
    mc_out_clear(&out);
    mc_step(&g_motor.mc, nm, hw, &out);
    motor_core_note_result(&out);
    if (before != MS_SAFE_STOP && mc_state(&g_motor.mc) == MS_SAFE_STOP)
    {
        g_motor.safe_reason = (out.fault_set & FAULT_CMD_TIMEOUT)
                                  ? MOTOR_SAFE_CMD_TIMEOUT
                                  : MOTOR_SAFE_HEADING_STALE;
    }
    motor_core_apply(&out);
    restore_interrupts(irq);
    return true;
}

/* ---------------- helpers (call with interrupts off) -------------- */

uint8_t motor_core_next_seq (void)
{
    g_motor.seq = (uint8_t)((g_motor.seq + 1u) & 0xFu);
    return g_motor.seq;
}

motor_error_t motor_core_submit (uint32_t word)
{
    mc_out_t out;
    mc_out_clear(&out);
    mc_command(&g_motor.mc, word, motor_now_ms(), g_motor.heading_word, &out);
    if (out.result_valid && (out.result_byte & 0x3u) == MR_REJECTED)
    {
        return MOTOR_ERR_SAFETY;
    }
    return MOTOR_OK;
}

void motor_core_hard_stop (mc_stop_kind_t kind)
{
    motor_state_t before = mc_state(&g_motor.mc);
    mc_out_t      out;
    mc_out_clear(&out);
    mc_immediate_stop(&g_motor.mc, kind, motor_now_ms(), &out);
    motor_core_note_result(&out);
    motor_hw_apply(&out);
    g_motor.word                   = 0;
    g_motor.auto_word              = 0;
    g_motor.auto_lease.duration_ms = 0;
    if (g_motor.auto_status == MOTOR_AUTO_RUNNING)
    {
        g_motor.auto_status = MOTOR_AUTO_STOPPED;
    }
    if (before != MS_SAFE_STOP && mc_state(&g_motor.mc) == MS_SAFE_STOP)
    {
        g_motor.safe_reason = MOTOR_SAFE_MANUAL_ESTOP;
    }
}

/* ---------------- public API ---------------- */

motor_error_t motor_init (void)
{
    motor_hw_safe_boot(); /* claw, arm and wheels low first */
    motor_hw_init();
#if MOTOR_SEND_DIR_BYTE
    motor_hw_dir_uart_init();
#endif
    mc_init(&g_motor.mc);
    mc_set_system_ready(&g_motor.mc, true);
    g_motor.manual_estop    = false;
    g_motor.forward_driving = false;
    g_motor.front_blocked   = false;
    motor_safety_init(); /* front-stop pending flag */
    g_motor.word = 0;
    motor_heading_reset(); /* offset, epoch, have-heading */
    g_motor.auto_word  = 0;
    g_motor.auto_lease = (motor_lease_t){0};

    if (!add_repeating_timer_ms(-(int32_t)MC_PERIOD_MS, motion_tick_isr, NULL,
                                &s_timer))
    {
        return MOTOR_ERR_INIT;
    }
    printf("[MOTOR] Motion Control ready (%lu ms update, active brake)\n",
           (unsigned long)MC_PERIOD_MS);
    return MOTOR_OK;
}

/* ---------------- status ---------------- */

int32_t motor_motion_state (void)
{
    return (int32_t)mc_state(&g_motor.mc);
}
