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

static mc_t               s_mc;
static safety_interlock_t s_safety = {.latched = true, .stop_requested = true};
static struct repeating_timer s_timer;
static volatile uint32_t  s_heading_word;   /* one 32-bit store / load */
static uint8_t            s_hdg_seq;
static uint32_t           s_word;           /* move being held (0 = none) */
static uint32_t           s_last_call_ms;
static uint8_t            s_seq;

/* Why Motion Control last entered SAFE STOP (state 7), for the dashboard. */
static volatile uint8_t   s_safe_reason = MOTOR_SAFE_NONE;

/* Autonomous move (mapping / Exploration). The driver refreshes it itself,
 * so it keeps going until motor_auto_stop(), an operator STOP or a safety
 * stop. A rotation also ends by itself when the heading is reached. */
static uint32_t           s_auto_word;      /* auto move being held (0 = none) */
static uint8_t            s_auto_seq;
static volatile uint8_t   s_auto_status = MOTOR_AUTO_IDLE;

static inline uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

/* Turn a move result for the running auto command into its status. */
static void note_result(const mc_out_t *o) {
    if (!o->result_valid || s_auto_status != MOTOR_AUTO_RUNNING) return;
    if (((o->result_byte >> 2) & 0xFu) != s_auto_seq) return;   /* not the auto move */
    switch ((move_result_t)(o->result_byte & 0x3u)) {
    case MR_DONE:            s_auto_status = MOTOR_AUTO_DONE;            break;
    case MR_HEADING_REACHED: s_auto_status = MOTOR_AUTO_HEADING_REACHED; break;
    case MR_STOPPED:         s_auto_status = MOTOR_AUTO_STOPPED;         break;
    case MR_REJECTED:        s_auto_status = MOTOR_AUTO_REJECTED;        break;
    }
    if (s_auto_status != MOTOR_AUTO_RUNNING) s_auto_word = 0;
}

static void apply(const mc_out_t *o) {
#if MOTOR_SEND_DIR_BYTE
    if (o->dir_send) motor_hw_send_dir(o->dir_byte);   /* direction before motion */
#endif
    motor_hw_apply(o);
}

/* ---------------- fixed 20 ms update (timer interrupt) ---------------- */

static bool motion_tick(struct repeating_timer *t) {
    (void)t;
    uint32_t irq = save_and_disable_interrupts();
    uint32_t nm = now_ms();
    uint32_t hw = s_heading_word;
    mc_out_t out;

    if (s_word) {
        if ((uint32_t)(nm - s_last_call_ms) <= REFRESH_WINDOW_MS) {
            mc_out_clear(&out);
            mc_command(&s_mc, s_word, nm, hw, &out);   /* same word = refresh */
        } else {
            s_word = 0;   /* caller went quiet: Motion Control times out (FR-42) */
        }
    }
    if (s_auto_word) {
        mc_out_clear(&out);
        mc_command(&s_mc, s_auto_word, nm, hw, &out);  /* keep the auto move alive */
        note_result(&out);
    }
    motor_state_t before = mc_state(&s_mc);
    mc_out_clear(&out);
    mc_step(&s_mc, nm, hw, &out);
    note_result(&out);
    if (before != MS_SAFE_STOP && mc_state(&s_mc) == MS_SAFE_STOP)
        s_safe_reason = (out.fault_set & FAULT_CMD_TIMEOUT) ? MOTOR_SAFE_CMD_TIMEOUT
                                                            : MOTOR_SAFE_HEADING_STALE;
    apply(&out);
    restore_interrupts(irq);
    return true;
}

/* ---------------- helpers (call with interrupts off) ---------------- */

static uint8_t next_seq(void) { s_seq = (uint8_t)((s_seq + 1u) & 0xFu); return s_seq; }

static motor_error_t submit(uint32_t word) {
    mc_out_t out;
    mc_out_clear(&out);
    mc_command(&s_mc, word, now_ms(), s_heading_word, &out);
    if (out.result_valid && (out.result_byte & 0x3u) == MR_REJECTED) return MOTOR_ERR_SAFETY;
    return MOTOR_OK;
}

static void hard_stop(mc_stop_kind_t kind) {
    motor_state_t before = mc_state(&s_mc);
    mc_out_t out;
    mc_out_clear(&out);
    mc_immediate_stop(&s_mc, kind, now_ms(), &out);
    note_result(&out);
    motor_hw_apply(&out);
    s_word = 0;
    s_auto_word = 0;
    if (s_auto_status == MOTOR_AUTO_RUNNING) s_auto_status = MOTOR_AUTO_STOPPED;
    if (before != MS_SAFE_STOP && mc_state(&s_mc) == MS_SAFE_STOP)
        s_safe_reason = (kind == MC_STOP_LINK_LOST) ? MOTOR_SAFE_LINK_LOST : MOTOR_SAFE_SENSOR_LOCK;
}

static int clamp100(int v) { return v < 0 ? 0 : (v > 100 ? 100 : v); }

/* Direction -> body motion (vx forward, vy right, w clockwise).
 * Diagonals use half on each axis, so the two pushing wheels get 's'
 * and the other two stop. */
static bool dir_to_motion(motor_direction_t d, int s, int *vx, int *vy, int *w) {
    int h = s / 2;
    *vx = *vy = *w = 0;
    switch (d) {
    case DIRECTION_FORWARD:      *vx =  s; break;
    case DIRECTION_BACKWARD:     *vx = -s; break;
    case DIRECTION_LEFT:         *vy = -s; break;
    case DIRECTION_RIGHT:        *vy =  s; break;
    case DIRECTION_ROTATE_LEFT:  *w  = -s; break;
    case DIRECTION_ROTATE_RIGHT: *w  =  s; break;
    case DIRECTION_FRONT_LEFT:   *vx =  h; *vy = -h; break;
    case DIRECTION_FRONT_RIGHT:  *vx =  h; *vy =  h; break;
    case DIRECTION_BACK_LEFT:    *vx = -h; *vy = -h; break;
    case DIRECTION_BACK_RIGHT:   *vx = -h; *vy =  h; break;
    default: return false;
    }
    return true;
}

/* ============================== public API ============================== */

motor_error_t motor_init(void) {
    motor_hw_safe_boot();              /* claw, arm and wheels low first */
    motor_hw_init();
#if MOTOR_SEND_DIR_BYTE
    motor_hw_dir_uart_init();
#endif
    mc_init(&s_mc);
    mc_set_system_ready(&s_mc, false); /* locked until the safety interlock is ready */
    safety_interlock_init(&s_safety);
    s_word = 0;

    if (!add_repeating_timer_ms(-(int32_t)MC_PERIOD_MS, motion_tick, NULL, &s_timer))
        return MOTOR_ERR_INIT;
    printf("[MOTOR] Motion Control ready (%u ms update, active brake)\n", (unsigned)MC_PERIOD_MS);
    return MOTOR_OK;
}

void motor_heading_update(float heading_deg, bool valid) {
    if (!valid) return;                /* no new frame -> heading goes stale by itself */
    while (heading_deg < 0.0f)    heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    uint16_t tenths = (uint16_t)(heading_deg * 10.0f + 0.5f);
    if (tenths >= 3600u) tenths = 0;
    s_hdg_seq = (uint8_t)((s_hdg_seq + 1u) & 0xFu);
    s_heading_word = heading_word_pack(tenths, s_hdg_seq, now_ms());   /* one store */
}

int motor_move_speed(motor_direction_t direction) {
    switch (direction) {
    case DIRECTION_FORWARD: case DIRECTION_BACKWARD:          return MC_DASH_STRAIGHT_PCT;
    case DIRECTION_LEFT:    case DIRECTION_RIGHT:             return MC_DASH_STRAFE_PCT;
    case DIRECTION_ROTATE_LEFT: case DIRECTION_ROTATE_RIGHT:  return MC_DASH_ROTATE_PCT;
    default:                                                  return MC_DASH_DIAGONAL_PCT;
    }
}

motor_error_t motor_set_direction(motor_direction_t direction, int speed_percent) {
    if (direction == DIRECTION_STOP) return motor_emergency_stop();
    if (speed_percent < 0 || speed_percent > 100) return MOTOR_ERR_PIN;

    int vx, vy, w;
    if (!dir_to_motion(direction, speed_percent, &vx, &vy, &w)) return MOTOR_ERR_PIN;

    uint32_t irq = save_and_disable_interrupts();
    motor_error_t r = MOTOR_ERR_SAFETY;
    if (!s_safety.latched) {
        /* Same move again = refresh (same word). A different move gets a
         * new sequence number. */
        uint32_t word = 0;
        if (s_word && mc_state(&s_mc) == MS_MANUAL)
            word = mc_pack_manual_drive(mc_seq(s_word), (int8_t)vx, (int8_t)vy, (int8_t)w);
        if (word == 0 || word != s_word)
            word = mc_pack_manual_drive(next_seq(), (int8_t)vx, (int8_t)vy, (int8_t)w);
        r = submit(word);
        if (r == MOTOR_OK) {
            s_word = word;
            s_last_call_ms = now_ms();
        }
    }
    restore_interrupts(irq);
    return r;
}

/* ======================= autonomous moves (FR-40, FR-41) =======================
 * Called by the mapping / Exploration side. One call starts the move; it
 * keeps going (the driver refreshes it) until motor_auto_stop(), an operator
 * STOP, or a safety stop. Needs a fresh heading from the Sensor Pico. */

static bool heading_ready(void) { return s_mc.heading_seen && !s_mc.stale_latched; }
static int  heading_deg_now(void) { return (int)((s_mc.heading + 5u) / 10u) % 360; }

static motor_error_t start_auto(uint32_t word) {
    s_word = 0;                              /* a manual move can't be held now */
    s_auto_seq = mc_seq(word);
    s_auto_status = MOTOR_AUTO_RUNNING;
    mc_out_t out;
    mc_out_clear(&out);
    mc_command(&s_mc, word, now_ms(), s_heading_word, &out);
    note_result(&out);
    if (s_auto_status == MOTOR_AUTO_REJECTED) return MOTOR_ERR_SAFETY;
    s_auto_word = word;
    return MOTOR_OK;
}

static motor_error_t auto_checks(void) {
    if (s_safety.latched) return MOTOR_ERR_SAFETY;
    if (!heading_ready()) return MOTOR_ERR_NO_HEADING;
    return MOTOR_OK;
}

motor_error_t motor_auto_drive(int speed_percent) {
    if (speed_percent == 0) return motor_auto_stop();
    if (speed_percent < -100 || speed_percent > 100) return MOTOR_ERR_PIN;
    uint32_t irq = save_and_disable_interrupts();
    motor_error_t r = auto_checks();
    if (r == MOTOR_OK) {
        /* Hold the heading the rover has now; if it is already driving
         * straight, keep the same target so a speed change doesn't re-aim. */
        int hold = (mc_state(&s_mc) == MS_AUTO_DRIVE && s_auto_word)
                   ? (int)mc_hdg9(s_auto_word, 16) : heading_deg_now();
        r = start_auto(mc_pack_auto_drive(next_seq(), (int8_t)speed_percent, (uint16_t)hold));
    }
    restore_interrupts(irq);
    return r;
}

motor_error_t motor_auto_rotate_to(int heading_deg) {
    heading_deg %= 360;
    if (heading_deg < 0) heading_deg += 360;
    uint32_t irq = save_and_disable_interrupts();
    motor_error_t r = auto_checks();
    if (r == MOTOR_OK) r = start_auto(mc_pack_auto_rotate(next_seq(), (uint16_t)heading_deg));
    restore_interrupts(irq);
    return r;
}

motor_error_t motor_auto_rotate_by(int delta_deg) {
    uint32_t irq = save_and_disable_interrupts();
    motor_error_t r = auto_checks();
    int target = heading_deg_now() + delta_deg;
    restore_interrupts(irq);
    return r == MOTOR_OK ? motor_auto_rotate_to(target) : r;
}

motor_error_t motor_auto_stop(void) {
    uint32_t irq = save_and_disable_interrupts();
    if (s_auto_word) {
        s_auto_word = 0;                     /* stop refreshing the move */
        mc_out_t out;
        mc_out_clear(&out);
        mc_command(&s_mc, mc_pack_simple(MC_STOP, MC_SRC_AUTO, s_auto_seq), now_ms(),
                   s_heading_word, &out);    /* normal stop: ramp down, then DONE */
        note_result(&out);
    }
    restore_interrupts(irq);
    return MOTOR_OK;
}

motor_auto_status_t motor_auto_status(void) { return (motor_auto_status_t)s_auto_status; }

const char *motor_auto_status_name(motor_auto_status_t s) {
    switch (s) {
    case MOTOR_AUTO_RUNNING:         return "RUNNING";
    case MOTOR_AUTO_DONE:            return "DONE";
    case MOTOR_AUTO_HEADING_REACHED: return "HEADING_REACHED";
    case MOTOR_AUTO_STOPPED:         return "STOPPED";
    case MOTOR_AUTO_REJECTED:        return "REJECTED";
    default:                         return "IDLE";
    }
}

/* Parallel park (strafe for MC_PARK_DURATION_MS, then stops by itself).
 * direction: -1 = left, +1 = right. power 10..100, 0 = default. */
motor_error_t motor_parallel_park(int direction, int power_percent) {
    int p = power_percent > 0 ? clamp100(power_percent) : MC_PARK_PCT;
    uint32_t irq = save_and_disable_interrupts();
    motor_error_t r = MOTOR_ERR_SAFETY;
    if (!s_safety.latched) {
        uint32_t word = mc_pack_manual_park(next_seq(), (int8_t)(direction < 0 ? -p : p));
        r = submit(word);
        s_word = 0;                    /* park ends by itself: no auto refresh */
    }
    restore_interrupts(irq);
    return r;
}

/* ---------------- arm / claw: plain on/off GPIO ---------------- */

motor_error_t arm_set_direction(arm_direction_t direction) {
    uint32_t irq = save_and_disable_interrupts();
    motor_error_t r = MOTOR_ERR_SAFETY;
    if (!s_safety.latched || direction == ARM_STOP) {
        motor_hw_arm(direction == ARM_DOWN ? MH_FWD : direction == ARM_UP ? MH_REV : MH_OFF);
        r = MOTOR_OK;
    }
    restore_interrupts(irq);
    return r;
}

motor_error_t claw_set_direction(claw_direction_t direction) {
    uint32_t irq = save_and_disable_interrupts();
    motor_error_t r = MOTOR_ERR_SAFETY;
    if (!s_safety.latched || direction == CLAW_STOP) {
        motor_hw_claw(direction == CLAW_CLOSE ? MH_FWD : direction == CLAW_OPEN ? MH_REV : MH_OFF);
        r = MOTOR_OK;
    }
    restore_interrupts(irq);
    return r;
}

/* ---------------- stops ---------------- */

/* Dashboard STOP / AUTO_STOP / end of a turn: operator stop.
 * PWM 0 at once + active brake for MC_BRAKE_HOLD_MS, then IDLE. */
motor_error_t motor_emergency_stop(void) {
    uint32_t irq = save_and_disable_interrupts();
    hard_stop(MC_STOP_OPERATOR);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    restore_interrupts(irq);
    return MOTOR_OK;
}

static void on_latched(void) {
    /* Collision Detection STOP or lost sensor link: SAFE STOP, no ramp. */
    hard_stop(MC_STOP_ESTOP);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    mc_set_system_ready(&s_mc, false);
}

/* Called from the UART interrupt: short, no printf. */
void motor_safety_receive(bool stop_requested, uint32_t receive_ms) {
    uint32_t irq = save_and_disable_interrupts();
    bool was = s_safety.latched;
    safety_interlock_receive(&s_safety, stop_requested, receive_ms);
    if (s_safety.latched && !was) on_latched();
    restore_interrupts(irq);
}

void motor_safety_poll(void) {
    uint32_t irq = save_and_disable_interrupts();
    bool was = s_safety.latched;
    safety_interlock_poll(&s_safety, now_ms());
    if (s_safety.latched && !was) on_latched();
    restore_interrupts(irq);
}

/* Laptop link lost (FR-08): stop at once. If the rover was moving this is
 * SAFE STOP (statechart 6.6): it will not move again until Reset Safety. */
void motor_link_lost(void) {
    uint32_t irq = save_and_disable_interrupts();
    hard_stop(MC_STOP_LINK_LOST);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    restore_interrupts(irq);
}

const char *motor_safe_stop_reason(void) {
    switch (s_safe_reason) {
    case MOTOR_SAFE_CMD_TIMEOUT:   return "no command for 0.8 s (link lost?)";
    case MOTOR_SAFE_LINK_LOST:     return "laptop link lost";
    case MOTOR_SAFE_SENSOR_LOCK:   return "collision stop / sensor link lost";
    case MOTOR_SAFE_HEADING_STALE: return "heading too old";
    default:                       return "unknown";
    }
}

bool motor_safety_is_latched(void) {
    uint32_t irq = save_and_disable_interrupts();
    bool l = s_safety.latched;
    restore_interrupts(irq);
    return l;
}

bool motor_safety_reset(void) {
    uint32_t irq = save_and_disable_interrupts();
    bool ready = safety_interlock_reset(&s_safety, now_ms());
    hard_stop(MC_STOP_OPERATOR);       /* a reset never restarts an old move */
    if (ready) {
        mc_out_t out;
        mc_out_clear(&out);
        mc_command(&s_mc, mc_pack_simple(MC_RESET, MC_SRC_MANUAL, next_seq()),
                   now_ms(), s_heading_word, &out);          /* SAFE STOP -> IDLE */
        mc_set_system_ready(&s_mc, true);
        if (mc_state(&s_mc) != MS_SAFE_STOP) s_safe_reason = MOTOR_SAFE_NONE;
    }
    restore_interrupts(irq);
    return ready;
}

/* ---------------- status ---------------- */

int motor_motion_state(void) { return (int)mc_state(&s_mc); }
