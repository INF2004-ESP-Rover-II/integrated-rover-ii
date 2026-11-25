/**
 * @file motor_driver.c
 * @brief Old motor_driver.h API, now running on the Subsystem 6 Motion
 *        Control module (motor_control.c + motor_hw.c).
 *
 * main.c keeps calling motor_set_direction(), motor_emergency_stop(), etc.
 * exactly as before. Underneath, every call becomes a packed command word
 * for the Motion Control state machine (Week 6 Section 6.6), which adds:
 *   - one fixed 20 ms update (repeating timer) for ramping + heading hold
 *   - inverse kinematics for all omni moves (FR-38)
 *   - ramp up / ramp down, start boost, deadband (FR-42)
 *   - heading hold from the Sensor Pico heading (FR-41)
 *   - active brake (IN1 = IN2, EN on) instead of reverse thrust
 *   - the arm and claw as plain on/off GPIO, so they no longer share
 *     PWM slice 5 with the back wheels (prof feedback)
 *
 * This build has no FreeRTOS, so motor_task.c is NOT used here (it is used
 * by motor_bench/). Shared state is protected the same way the old driver
 * did it: interrupts off around every access.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_driver.h"
#include "config.h"
#include "safety_interlock.h"

#include "motor_control.h"
#include "motor_config.h"
#include "motor_hw.h"
#include "motor_types.h"
#include "motor_lease.h"

#include <stdio.h>
#include <math.h>
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

static mc_t                   g_s_mc;
static bool                   g_b_s_manual_estop;
static uint32_t               g_s_last_sensor_ms;
static bool                   g_b_s_forward_driving;
static bool                   g_b_s_front_blocked;
static bool                   g_b_s_front_stop_pending;
static struct repeating_timer g_s_timer;
static volatile uint32_t      g_s_heading_word; /* one 32-bit store / load */
static uint8_t                g_s_hdg_seq;
static uint32_t               g_s_word; /* move being held (0 = none) */
static uint32_t               g_s_last_call_ms;
static uint8_t                g_s_seq;
static float                  g_s_raw_heading;
static float                  g_s_heading_offset;
static uint32_t               g_s_raw_heading_ms;
static uint32_t               g_s_heading_epoch;
static bool                   g_b_s_have_raw_heading;

/* Why Motion Control last entered SAFE STOP (state 7), for the dashboard. */
static volatile uint8_t g_s_safe_reason = MOTOR_SAFE_NONE;

/* Autonomous move (mapping / Exploration). The driver refreshes it itself,
 * so it keeps going until motor_auto_stop(), an operator STOP or a safety
 * stop. A rotation also ends by itself when the heading is reached. */
static uint32_t         g_s_auto_word; /* auto move being held (0 = none) */
static uint8_t          g_s_auto_seq;
static volatile uint8_t g_s_auto_status = MOTOR_AUTO_IDLE;
static motor_lease_t    g_s_auto_lease;

static void hard_stop (mc_stop_kind_t kind);


static inline uint32_t
now_ms (void)
{
    return to_ms_since_boot(get_absolute_time());
}

/* Turn a move result for the running auto command into its status. */
static void
note_result (const mc_out_t * p_o)
{
    if (!p_o->result_valid || g_s_auto_status != MOTOR_AUTO_RUNNING)
    {
        return;
    }
    if ((((uint32_t) (p_o->result_byte) >> (uint32_t) (2)) & 0xFu) !=
        g_s_auto_seq)
    {
        return; /* not the auto move */
    }
    switch ((move_result_t) (p_o->result_byte & 0x3u))
    {
        case MR_DONE:
            g_s_auto_status = MOTOR_AUTO_DONE;
            break;
        case MR_HEADING_REACHED:
            g_s_auto_status = MOTOR_AUTO_HEADING_REACHED;
            break;
        case MR_STOPPED:
            g_s_auto_status = MOTOR_AUTO_STOPPED;
            break;
        case MR_REJECTED:
            g_s_auto_status = MOTOR_AUTO_REJECTED;
            break;
        default:
            /* The two-bit result field covers every defined result. */
            break;
    }
    if (g_s_auto_status != MOTOR_AUTO_RUNNING)
    {
        g_s_auto_word = 0;
    }
}

static void
apply (const mc_out_t * p_o)
{
#if MOTOR_SEND_DIR_BYTE
    if (p_o->dir_send)
    {
        motor_hw_send_dir(p_o->dir_byte); /* direction before motion */
    }
#endif
    motor_hw_apply(p_o);
}

/* ---------------- fixed 20 ms update (timer interrupt) ---------------- */

static bool
motion_tick (struct repeating_timer * p_time_value)
{
    (void) p_time_value;
    uint32_t irq          = save_and_disable_interrupts();
    uint32_t current_ms   = now_ms();
    uint32_t heading_word = g_s_heading_word;
    mc_out_t out;

    if (g_s_word)
    {
        if ((uint32_t) (current_ms - g_s_last_call_ms) <= REFRESH_WINDOW_MS)
        {
            mc_out_clear(&out);
            mc_command(&g_s_mc, g_s_word, current_ms, heading_word,
                       &out); /* same word = refresh */
        }
        else
        {

            g_s_word = 0;
            /* caller went quiet: Motion Control times out (FR-42) */
        }
    }
    if (g_s_auto_word && motor_lease_expired(&g_s_auto_lease, current_ms))
    {
        hard_stop(MC_STOP_OPERATOR);
    }
    if (g_s_auto_word)
    {
        mc_out_clear(&out);
        mc_command(&g_s_mc, g_s_auto_word, current_ms, heading_word,
                   &out); /* keep the auto move alive */
        note_result(&out);
    }
    motor_state_t before = mc_state(&g_s_mc);
    mc_out_clear(&out);
    mc_step(&g_s_mc, current_ms, heading_word, &out);
    note_result(&out);
    if (before != MS_SAFE_STOP && mc_state(&g_s_mc) == MS_SAFE_STOP)
    {

        g_s_safe_reason = (out.fault_set & FAULT_CMD_TIMEOUT)
                              ? MOTOR_SAFE_CMD_TIMEOUT
                              : MOTOR_SAFE_HEADING_STALE;
    }
    apply(&out);
    restore_interrupts(irq);
    return true;
}

/* ---------------- helpers (call with interrupts off) ---------------- */

static uint8_t
next_seq (void)
{
    g_s_seq = (uint8_t) ((g_s_seq + 1u) & 0xFu);
    return g_s_seq;
}

static motor_error_t
submit (uint32_t word)
{
    mc_out_t out;
    mc_out_clear(&out);
    mc_command(&g_s_mc, word, now_ms(), g_s_heading_word, &out);
    if (out.result_valid && (out.result_byte & 0x3u) == MR_REJECTED)
    {
        return MOTOR_ERR_SAFETY;
    }
    return MOTOR_OK;
}

static void
hard_stop (mc_stop_kind_t kind)
{
    motor_state_t before = mc_state(&g_s_mc);
    mc_out_t      out;
    mc_out_clear(&out);
    mc_immediate_stop(&g_s_mc, kind, now_ms(), &out);
    note_result(&out);
    motor_hw_apply(&out);
    g_s_word                   = 0;
    g_s_auto_word              = 0;
    g_s_auto_lease.duration_ms = 0;
    if (g_s_auto_status == MOTOR_AUTO_RUNNING)
    {
        g_s_auto_status = MOTOR_AUTO_STOPPED;
    }
    if (before != MS_SAFE_STOP && mc_state(&g_s_mc) == MS_SAFE_STOP)
    {
        g_s_safe_reason = MOTOR_SAFE_MANUAL_ESTOP;
    }
}

static int
clamp100 (int value)
{
    return value < 0 ? 0 : (value > 100 ? 100 : value);
}

/* Direction -> body motion (vx forward, vy right, w clockwise).
 * Diagonals use half on each axis, so the two pushing wheels get 's'
 * and the other two stop. */
static bool
dir_to_motion (motor_direction_t data, int state, int * p_velocity_x,
               int * p_velocity_y, int * p_value_w)
{
    int heading   = state / 2;
    *p_velocity_x = *p_velocity_y = *p_value_w = 0;
    switch (data)
    {
        case DIRECTION_FORWARD:
            *p_velocity_x = state;
            break;
        case DIRECTION_BACKWARD:
            *p_velocity_x = -state;
            break;
        case DIRECTION_LEFT:
            *p_velocity_y = -state;
            break;
        case DIRECTION_RIGHT:
            *p_velocity_y = state;
            break;
        case DIRECTION_ROTATE_LEFT:
            *p_value_w = -state;
            break;
        case DIRECTION_ROTATE_RIGHT:
            *p_value_w = state;
            break;
        case DIRECTION_FRONT_LEFT:
            *p_velocity_x = heading;
            *p_velocity_y = -heading;
            break;
        case DIRECTION_FRONT_RIGHT:
            *p_velocity_x = heading;
            *p_velocity_y = heading;
            break;
        case DIRECTION_BACK_LEFT:
            *p_velocity_x = -heading;
            *p_velocity_y = -heading;
            break;
        case DIRECTION_BACK_RIGHT:
            *p_velocity_x = -heading;
            *p_velocity_y = heading;
            break;
        default:
            return false;
    }
    return true;
}

/* ============================== public API ============================== */

motor_error_t
motor_init (void)
{
    motor_hw_safe_boot(); /* claw, arm and wheels low first */
    motor_hw_init();
#if MOTOR_SEND_DIR_BYTE
    motor_hw_dir_uart_init();
#endif
    mc_init(&g_s_mc);
    mc_set_system_ready(&g_s_mc, true);
    g_b_s_manual_estop       = false;
    g_b_s_forward_driving    = false;
    g_b_s_front_blocked      = false;
    g_b_s_front_stop_pending = false;
    g_s_word                 = 0;
    g_s_heading_offset       = 0;
    g_s_auto_word            = 0;
    g_s_auto_lease           = (motor_lease_t) { 0 };
    g_s_heading_epoch        = 0;
    g_b_s_have_raw_heading   = false;

    if (!add_repeating_timer_ms(-(int32_t) MC_PERIOD_MS, motion_tick, NULL,
                                &g_s_timer))
    {
        return MOTOR_ERR_INIT;
    }
    printf("[MOTOR] Motion Control ready (%u ms update, active brake)\n",
           (unsigned) MC_PERIOD_MS);
    return MOTOR_OK;
}

static float
wrap_heading (float heading_deg)
{
    float wrapped = fmodf(heading_deg, 360.0f);
    return wrapped < 0 ? wrapped + 360.0f : wrapped;
}

static void
store_heading (float heading_deg)
{
    uint16_t tenths = (uint16_t) (heading_deg * 10.0f + 0.5f);
    if (tenths >= 3600u)
    {
        tenths = 0;
    }
    g_s_hdg_seq = (uint8_t) ((g_s_hdg_seq + 1u) & 0xFu);

    g_s_heading_word = heading_word_pack(tenths, g_s_hdg_seq, now_ms());
    /* one store */
}

void
motor_heading_update (float heading_deg, bool b_valid)
{
    if (!b_valid || !isfinite(heading_deg))
    {
        return;
    }
    uint32_t irq           = save_and_disable_interrupts();
    g_s_raw_heading        = wrap_heading(heading_deg);
    g_s_raw_heading_ms     = now_ms();
    g_b_s_have_raw_heading = true;
    store_heading(wrap_heading(g_s_raw_heading - g_s_heading_offset));
    restore_interrupts(irq);
}

float
motor_heading_relative (float raw_heading_deg)
{
    uint32_t irq      = save_and_disable_interrupts();
    float    relative = wrap_heading(raw_heading_deg - g_s_heading_offset);
    restore_interrupts(irq);
    return relative;
}

float
motor_heading_offset (void)
{
    uint32_t irq    = save_and_disable_interrupts();
    float    offset = g_s_heading_offset;
    restore_interrupts(irq);
    return offset;
}

uint32_t
motor_heading_epoch (void)
{
    uint32_t irq   = save_and_disable_interrupts();
    uint32_t epoch = g_s_heading_epoch;
    restore_interrupts(irq);
    return epoch;
}

bool
motor_heading_sample_since (uint32_t since_ms)
{
    uint32_t irq = save_and_disable_interrupts();

    bool b_received = g_b_s_have_raw_heading &&
                      (int32_t) (g_s_raw_heading_ms - since_ms) >= 0;

    restore_interrupts(irq);
    return b_received;
}

motor_error_t
motor_zero_heading (void)
{
    uint32_t irq = save_and_disable_interrupts();
    if (!g_b_s_have_raw_heading ||
        (uint32_t) (now_ms() - g_s_raw_heading_ms) > MC_HEADING_STALE_MS)
    {
        restore_interrupts(irq);
        return MOTOR_ERR_NO_HEADING;
    }
    hard_stop(MC_STOP_OPERATOR);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    g_s_heading_offset = g_s_raw_heading;
    ++g_s_heading_epoch;
    store_heading(0);
    mc_out_t out;
    mc_out_clear(&out);
    mc_step(&g_s_mc, now_ms(), g_s_heading_word, &out);
    apply(&out);
    restore_interrupts(irq);
    return MOTOR_OK;
}

int
motor_move_speed (motor_direction_t direction)
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

motor_error_t
motor_set_direction (motor_direction_t direction, int speed_percent)
{
    if (direction == DIRECTION_STOP)
    {
        return motor_emergency_stop();
    }
    if (speed_percent < 0 || speed_percent > 100)
    {
        return MOTOR_ERR_PIN;
    }

    int velocity_x = 0;
    int velocity_y = 0;
    int value_w    = 0;
    if (!dir_to_motion(direction, speed_percent, &velocity_x, &velocity_y,
                       &value_w))
    {
        return MOTOR_ERR_PIN;
    }

    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = MOTOR_ERR_SAFETY;
    if (!g_b_s_manual_estop && direction == DIRECTION_FORWARD &&
        g_b_s_front_blocked)
    {
        result = MOTOR_ERR_FRONT_BLOCKED;
    }
    else if (!g_b_s_manual_estop)
    {
        /* Same move again = refresh (same word). A different move gets a
         * new sequence number. */
        uint32_t word = 0;
        if (g_s_word && mc_state(&g_s_mc) == MS_MANUAL)
        {

            word = mc_pack_manual_drive(mc_seq(g_s_word), (int8_t) velocity_x,
                                        (int8_t) velocity_y, (int8_t) value_w);
        }
        if (word == 0 || word != g_s_word)
        {

            word = mc_pack_manual_drive(next_seq(), (int8_t) velocity_x,
                                        (int8_t) velocity_y, (int8_t) value_w);
        }
        result = submit(word);
        if (result == MOTOR_OK)
        {
            g_s_word              = word;
            g_b_s_forward_driving = direction == DIRECTION_FORWARD;
            g_s_last_call_ms      = now_ms();
        }
    }
    restore_interrupts(irq);
    return result;
}

/* ======================= autonomous moves (FR-40, FR-41)
 * ======================= Called by the mapping / Exploration side. One call
 * starts the move; it keeps going (the driver refreshes it) until
 * motor_auto_stop(), an operator STOP, or a safety stop. Needs a fresh heading
 * from the Sensor Pico. */

static bool
heading_ready (void)
{
    return g_s_mc.heading_seen && !g_s_mc.stale_latched;
}
static int
heading_deg_now (void)
{
    return (int) ((g_s_mc.heading + 5u) / 10u) % 360;
}

static motor_error_t
start_auto (uint32_t word)
{
    g_s_auto_lease.duration_ms = 0;
    g_s_word                   = 0; /* a manual move can't be held now */
    g_s_auto_seq               = mc_seq(word);
    g_s_auto_status            = MOTOR_AUTO_RUNNING;
    mc_out_t out;
    mc_out_clear(&out);
    mc_command(&g_s_mc, word, now_ms(), g_s_heading_word, &out);
    note_result(&out);
    if (g_s_auto_status == MOTOR_AUTO_REJECTED)
    {
        return MOTOR_ERR_SAFETY;
    }
    g_s_auto_word = word;
    return MOTOR_OK;
}

static motor_error_t
auto_checks (void)
{
    if (g_b_s_manual_estop)
    {
        return MOTOR_ERR_SAFETY;
    }
    if (!heading_ready())
    {
        return MOTOR_ERR_NO_HEADING;
    }
    return MOTOR_OK;
}

motor_error_t
motor_auto_drive (int speed_percent)
{
    if (speed_percent == 0)
    {
        return motor_auto_stop();
    }
    if (speed_percent < -100 || speed_percent > 100)
    {
        return MOTOR_ERR_PIN;
    }
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = auto_checks();
    if (result == MOTOR_OK && speed_percent > 0 && g_b_s_front_blocked)
    {
        result = MOTOR_ERR_FRONT_BLOCKED; /* obstacle ahead: only forward is
                                             blocked */
    }
    if (result == MOTOR_OK && g_s_auto_word &&
        mc_state(&g_s_mc) == MS_AUTO_DRIVE &&
        mc_s8(g_s_auto_word, 8) == speed_percent)
    {
        /* Same move sent again (the laptop repeats it): keep going, no restart.
         */
    }
    else if (result == MOTOR_OK)
    {
        /* Hold the heading the rover has now; if it is already driving
         * straight, keep the same target so a speed change doesn't re-aim. */

        int hold = (mc_state(&g_s_mc) == MS_AUTO_DRIVE && g_s_auto_word)
                       ? (int) mc_hdg9(g_s_auto_word, 16)
                       : heading_deg_now();


        result = start_auto(mc_pack_auto_drive(
            next_seq(), (int8_t) speed_percent, (uint16_t) hold));

        if (result == MOTOR_OK)
        {

            g_b_s_forward_driving = speed_percent > 0;
            /* a front obstacle stops it */
        }
    }
    restore_interrupts(irq);
    return result;
}

motor_error_t
motor_auto_drive_leased (int speed_percent, uint16_t lease_ms)
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

        g_s_auto_lease = (motor_lease_t) { .refreshed_ms = now_ms(),
                                           .duration_ms  = lease_ms };
    }
    restore_interrupts(irq);
    return result;
}

bool
motor_auto_renew_lease (void)
{
    uint32_t irq = save_and_disable_interrupts();

    bool b_is_valid = g_s_auto_word && g_s_auto_lease.duration_ms &&
                      g_s_auto_status == MOTOR_AUTO_RUNNING &&
                      !motor_lease_expired(&g_s_auto_lease, now_ms());

    if (b_is_valid)
    {
        g_s_auto_lease.refreshed_ms = now_ms();
    }
    else if (g_s_auto_lease.duration_ms)
    {
        hard_stop(MC_STOP_OPERATOR);
    }
    restore_interrupts(irq);
    return b_is_valid;
}

motor_error_t
motor_auto_rotate_to (int heading_deg)
{
    heading_deg %= 360;
    if (heading_deg < 0)
    {
        heading_deg += 360;
    }
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = auto_checks();
    if (result == MOTOR_OK)
    {

        result =
            start_auto(mc_pack_auto_rotate(next_seq(), (uint16_t) heading_deg));

        if (result == MOTOR_OK)
        {

            g_b_s_forward_driving = false;
            /* turning is not stopped by a front obstacle */
        }
    }
    restore_interrupts(irq);
    return result;
}

motor_error_t
motor_auto_rotate_leased (int heading_deg, uint16_t lease_ms)
{
    if (!lease_ms || lease_ms > 500u)
    {
        return MOTOR_ERR_PIN;
    }
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = motor_auto_rotate_to(heading_deg);
    if (result == MOTOR_OK)
    {

        g_s_auto_lease = (motor_lease_t) { .refreshed_ms = now_ms(),
                                           .duration_ms  = lease_ms };
    }
    restore_interrupts(irq);
    return result;
}

motor_error_t
motor_auto_rotate_by (int delta_deg)
{
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = auto_checks();
    int           target = heading_deg_now() + delta_deg;
    restore_interrupts(irq);
    return result == MOTOR_OK ? motor_auto_rotate_to(target) : result;
}

motor_error_t
motor_auto_stop (void)
{
    uint32_t irq               = save_and_disable_interrupts();
    g_s_auto_lease.duration_ms = 0;
    if (g_s_auto_word)
    {
        g_s_auto_word = 0; /* stop refreshing the move */
        mc_out_t out;
        mc_out_clear(&out);
        mc_command(&g_s_mc, mc_pack_simple(MC_STOP, MC_SRC_AUTO, g_s_auto_seq),
                   now_ms(), g_s_heading_word,
                   &out); /* normal stop: ramp down, then DONE */
        note_result(&out);
    }
    restore_interrupts(irq);
    return MOTOR_OK;
}

motor_auto_status_t
motor_auto_status (void)
{
    return (motor_auto_status_t) g_s_auto_status;
}

const char *
motor_auto_status_name (motor_auto_status_t state)
{
    switch (state)
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

/* Parallel park (strafe for MC_PARK_DURATION_MS, then stops by itself).
 * direction: -1 = left, +1 = right. power 10..100, 0 = default. */
motor_error_t
motor_parallel_park (int direction, int power_percent)
{
    int pointer = power_percent > 0 ? clamp100(power_percent) : MC_PARK_PCT;
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = MOTOR_ERR_SAFETY;
    if (!g_b_s_manual_estop)
    {

        uint32_t word = mc_pack_manual_park(
            next_seq(), (int8_t) (direction < 0 ? -pointer : pointer));

        result = submit(word);
        if (result == MOTOR_OK)
        {
            g_b_s_forward_driving = false;
        }
        g_s_word = 0; /* park ends by itself: no auto refresh */
    }
    restore_interrupts(irq);
    return result;
}

/* ---------------- arm / claw: plain on/off GPIO ---------------- */

motor_error_t
arm_set_direction (arm_direction_t direction)
{
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = MOTOR_ERR_SAFETY;
    if (!g_b_s_manual_estop || direction == ARM_STOP)
    {
        motor_hw_arm(direction == ARM_DOWN ? MH_FWD
                     : direction == ARM_UP ? MH_REV
                                           : MH_OFF);
        result = MOTOR_OK;
    }
    restore_interrupts(irq);
    return result;
}

motor_error_t
claw_set_direction (claw_direction_t direction)
{
    uint32_t      irq    = save_and_disable_interrupts();
    motor_error_t result = MOTOR_ERR_SAFETY;
    if (!g_b_s_manual_estop || direction == CLAW_STOP)
    {
        motor_hw_claw(direction == CLAW_CLOSE  ? MH_FWD
                      : direction == CLAW_OPEN ? MH_REV
                                               : MH_OFF);
        result = MOTOR_OK;
    }
    restore_interrupts(irq);
    return result;
}

/* ---------------- stops ---------------- */

/* Dashboard STOP / AUTO_STOP / end of a turn: operator stop.
 * PWM 0 at once + active brake for MC_BRAKE_HOLD_MS, then IDLE. */
motor_error_t
motor_emergency_stop (void)
{
    uint32_t irq = save_and_disable_interrupts();
    hard_stop(MC_STOP_OPERATOR);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    restore_interrupts(irq);
    return MOTOR_OK;
}

void
motor_manual_estop (void)
{
    uint32_t irq       = save_and_disable_interrupts();
    g_b_s_manual_estop = true;
    hard_stop(MC_STOP_ESTOP);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    restore_interrupts(irq);
}

/* Sensor packets only control the temporary forward guard. */
void
motor_safety_receive (bool b_front_blocked, uint32_t receive_ms)
{
    uint32_t irq = save_and_disable_interrupts();
    if (b_front_blocked && !g_b_s_front_blocked)
    {
        if (g_b_s_forward_driving)
        {
            hard_stop(MC_STOP_OPERATOR);
        }
        g_b_s_front_stop_pending = true;
    }
    g_b_s_front_blocked = b_front_blocked;
    g_s_last_sensor_ms  = receive_ms;
    restore_interrupts(irq);
}

bool
motor_take_front_stop (void)
{
    uint32_t irq             = save_and_disable_interrupts();
    bool     b_pending       = g_b_s_front_stop_pending;
    g_b_s_front_stop_pending = false;
    restore_interrupts(irq);
    return b_pending;
}

void
motor_safety_poll (void)
{
    uint32_t irq = save_and_disable_interrupts();
    // Missing readings mean no detected obstacle, never a safety latch.
    if ((uint32_t) (now_ms() - g_s_last_sensor_ms) > SAFETY_PACKET_TIMEOUT_MS)
    {
        g_b_s_front_blocked = false;
    }
    restore_interrupts(irq);
}

void
motor_link_lost (void)
{
    // Communications loss stops motion without requiring a safety reset.
    motor_emergency_stop();
}

const char *
motor_safe_stop_reason (void)
{
    switch (g_s_safe_reason)
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
    bool     b_l = g_b_s_manual_estop;
    restore_interrupts(irq);
    return b_l;
}

bool
motor_safety_reset (void)
{
    uint32_t irq       = save_and_disable_interrupts();
    bool     b_ready   = true;
    g_b_s_manual_estop = false;
    hard_stop(MC_STOP_OPERATOR); /* a reset never restarts an old move */
    if (b_ready)
    {
        mc_out_t out;
        mc_out_clear(&out);
        mc_command(&g_s_mc, mc_pack_simple(MC_RESET, MC_SRC_MANUAL, next_seq()),
                   now_ms(), g_s_heading_word, &out); /* SAFE STOP -> IDLE */
        mc_set_system_ready(&g_s_mc, true);
        if (mc_state(&g_s_mc) != MS_SAFE_STOP)
        {
            g_s_safe_reason = MOTOR_SAFE_NONE;
        }
    }
    restore_interrupts(irq);
    return b_ready;
}

/* ---------------- status ---------------- */

int
motor_motion_state (void)
{
    return (int) mc_state(&g_s_mc);
}

/* End of file. */
