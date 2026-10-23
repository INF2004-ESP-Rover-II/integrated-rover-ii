/**
 * @file motor_control.c
 * @brief Motor Control logic for Subsystem 6 (FR-38 to FR-42).
 *
 * Pure C: no hardware, no RTOS. See motor_control.h for the statechart.
 * Calculations use normal-size local ints; values are packed only when
 * stored in mc_t or sent out (Change Requirement 1).
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_control.h"
#include "motor_config.h"

#include <string.h>

static int8_t k_deadband[MC_WHEELS] = MC_DEADBAND_PCT;   /* adjustable on the bench */
static int     k_kick_pct = MC_KICK_PCT;                  /* adjustable on the bench */
static int     k_kick_ms  = (int)MC_KICK_MS;
static const int8_t k_trim[MC_WHEELS]     = MC_TRIM_PCT;
static const int8_t k_invert[MC_WHEELS]   = MC_INVERT;

/* ============================== helpers ============================== */

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline int absi(int v) { return v < 0 ? -v : v; }
static inline bool in_range_pct(int v) { return v >= -100 && v <= 100; }

int16_t mc_heading_error(uint16_t target, uint16_t current) {
    /* shortest signed difference, so 359.0 -> 1.0 deg is +2.0, not -358 */
    int e = ((int)target - (int)current) % 3600;
    if (e >= 1800) e -= 3600;
    if (e < -1800) e += 3600;
    return (int16_t)e;
}

/* Inverse kinematics, mecanum "X" rollers, seen from above.
 * vx + = forward, vy + = right, w + = clockwise.
 * Matches the old motor_driver.c patterns (DIRECTION_RIGHT, ROTATE_RIGHT). */
void mc_mix(int vx, int vy, int w, int out[MC_WHEELS]) {
    out[0] = vx + vy + w;   /* front-left  */
    out[1] = vx - vy - w;   /* front-right */
    out[2] = vx - vy + w;   /* back-left   */
    out[3] = vx + vy - w;   /* back-right  */

    int peak = 0;
    for (int i = 0; i < MC_WHEELS; i++) if (absi(out[i]) > peak) peak = absi(out[i]);
    if (peak > 100) {                 /* scale all wheels together, keep the ratio */
        for (int i = 0; i < MC_WHEELS; i++) out[i] = out[i] * 100 / peak;
    }
}

uint8_t mc_dir_mask(int vx, int vy, int w) {
    const int th = 3;  /* ignore tiny heading-hold trims */
    if (absi(w) >= th && absi(vx) < th && absi(vy) < th) return (uint8_t)DIR_ALL; /* rotate */
    uint8_t m = 0;
    if (vx >=  th) m |= DIR_FRONT;
    if (vx <= -th) m |= DIR_BACK;
    if (vy >=  th) m |= DIR_RIGHT;
    if (vy <= -th) m |= DIR_LEFT;
    if (absi(w) > MC_HOLD_MAX_PCT && m) m = (uint8_t)DIR_ALL;  /* arc turn sweeps the corners */
    return m;
}

/* Heading hold, PI controller (FR-41). The I part removes the steady
 * offset a constant pull (weaker wheel, uneven floor) would leave. */
static int hold_trim(mc_t *m) {
    int err = mc_heading_error((uint16_t)m->hold_target, (uint16_t)m->heading);
#if !MC_HEADING_CW_POSITIVE
    err = -err;
#endif
    int integ = clampi(m->hold_integ + err, -MC_HOLD_INTEG_LIMIT, MC_HOLD_INTEG_LIMIT);
    m->hold_integ = (int16_t)integ;
    int trim = err * MC_HOLD_KP_NUM / MC_HOLD_KP_DEN + integ * MC_HOLD_KI_NUM / MC_HOLD_KI_DEN;
    return clampi(trim, -MC_HOLD_MAX_PCT, MC_HOLD_MAX_PCT);
}

static void set_hold_target(mc_t *m, unsigned tenths) {
    m->hold_target = tenths & 0xFFFu;
    m->hold_integ = 0;
}

/* Normal stop: ease down. The finished command is remembered so a late
 * refresh of it can't restart the move. */
static void begin_ramp_down(mc_t *m) {
    m->tvx = m->tvy = m->tw = 0;
    m->parking = 0;
    if (m->last_cmd_word) m->done_word = m->last_cmd_word;
    m->last_cmd_word = 0;
    m->state = MS_RAMP_DOWN;
}

static bool is_moving_state(motor_state_t s) {
    return s == MS_MANUAL || s == MS_AUTO_ROTATE || s == MS_AUTO_DRIVE || s == MS_RAMP_DOWN;
}

static bool duties_zero(const mc_t *m) {
    for (int i = 0; i < MC_WHEELS; i++) if (m->duty[i] != 0) return false;
    return true;
}

static void set_result(mc_t *m, mc_out_t *out, move_result_t r, uint8_t seq) {
    out->result_valid = 1;
    out->result_byte = result_byte_pack(r, seq);
    if (r != MR_REJECTED) {
        m->result_sent = 1;
        if (m->last_cmd_word) m->done_word = m->last_cmd_word; /* ignore late refreshes of it */
    }
}

static void clear_targets(mc_t *m) {
    m->tvx = m->tvy = m->tw = 0;
    m->parking = 0;
}

/* Heading freshness (tribunal decision):
 * - age uses a rolling 10-bit ms stamp written by the UART side;
 * - once older than MC_HEADING_STALE_MS the stale flag LATCHES, so the
 *   10-bit wrap (every 1024 ms) can never make old data look fresh again;
 * - the latch clears only when a NEW sequence number arrives that is fresh.
 * Returns true if the latch was just set. */
static bool update_heading(mc_t *m, uint32_t now_ms, uint32_t hw) {
    if (!heading_word_valid(hw)) return false;

    uint16_t age  = (uint16_t)(((now_ms & HW_TIME_MASK) - heading_word_time(hw)) & HW_TIME_MASK);
    uint8_t  seq  = heading_word_seq(hw);
    bool was_stale = m->stale_latched;

    if (!m->heading_seen) {
        m->heading_seen  = 1;
        m->last_hdg_seq  = seq & 0xFu;
        m->stale_latched = (age > MC_HEADING_STALE_MS);
    } else if (seq != m->last_hdg_seq) {
        m->last_hdg_seq = seq & 0xFu;
        if (age <= MC_HEADING_STALE_MS) m->stale_latched = 0;
    }
    if (age > MC_HEADING_STALE_MS) m->stale_latched = 1;

    if (!m->stale_latched) m->heading = heading_word_tenths(hw) & 0xFFFu;
    return m->stale_latched && !was_stale;
}

static bool heading_ok(const mc_t *m) { return m->heading_seen && !m->stale_latched; }

/* Enter SAFE STOP or IDLE with PWM forced to 0 immediately (no ramp). */
static void hard_stop(mc_t *m, motor_state_t next, uint32_t now_ms, mc_out_t *out) {
    bool was_moving = is_moving_state((motor_state_t)m->state);
    if (was_moving && !m->result_sent) set_result(m, out, MR_STOPPED, (uint8_t)m->active_seq);
    clear_targets(m);
    memset(m->duty, 0, sizeof m->duty);
    if (m->last_cmd_word) m->done_word = m->last_cmd_word;
    m->last_cmd_word = 0;
    m->brake_until_ms = now_ms + MC_BRAKE_HOLD_MS;
    m->state = (unsigned)next & 0x7u;
    out->outputs_valid = 1;
    out->brake = 1;
    memset(out->wheel, 0, sizeof out->wheel);
    out->state = m->state;
}

/* ============================== public API ============================== */

void mc_init(mc_t *m) {
    memset(m, 0, sizeof *m);
#if MC_ALLOW_MANUAL_WITHOUT_HEADING
    m->state = MS_IDLE;
#else
    m->state = MS_ZERO_GYRO;   /* until the Sensor Pico finishes zeroing the gyro */
#endif
}

void mc_out_clear(mc_out_t *o) { memset(o, 0, sizeof *o); }

void mc_set_deadband(int wheel, int pct) {
    pct = clampi(pct, 0, 90);
    for (int i = 0; i < MC_WHEELS; i++) if (wheel < 0 || wheel == i) k_deadband[i] = (int8_t)pct;
}
int mc_get_deadband(int wheel) { return k_deadband[clampi(wheel, 0, MC_WHEELS - 1)]; }

void mc_set_kick(int pct, int ms) { k_kick_pct = clampi(pct, 0, 100); k_kick_ms = clampi(ms, 0, 1000); }
int  mc_get_kick_pct(void) { return k_kick_pct; }
int  mc_get_kick_ms(void)  { return k_kick_ms; }

void mc_set_system_ready(mc_t *m, bool ready) { m->system_ready = ready ? 1u : 0u; }

void mc_command(mc_t *m, uint32_t w, uint32_t now_ms, uint32_t heading_word, mc_out_t *out) {
    mc_type_t type = mc_type(w);
    mc_src_t  src  = mc_src(w);
    uint8_t   seq  = mc_seq(w);
    motor_state_t s = (motor_state_t)m->state;

    if (update_heading(m, now_ms, heading_word)) out->fault_set |= FAULT_HEADING_STALE;

    /* A late refresh of a move that already finished is ignored silently,
     * so it can never restart the move or produce a second result. */
    if (w != 0 && w == m->done_word) return;

    /* Refresh: the identical word again just keeps the move alive. */
    if (w == m->last_cmd_word && (s == MS_MANUAL || s == MS_AUTO_ROTATE || s == MS_AUTO_DRIVE)) {
        m->last_cmd_ms = now_ms;
        return;
    }

    bool ok = false;
    switch (type) {
    case MC_STOP:
        /* Normal stop from any source: ease down, report DONE when stopped. */
        if (s == MS_MANUAL || s == MS_AUTO_DRIVE || s == MS_AUTO_ROTATE) {
            begin_ramp_down(m);
        } else if (s == MS_IDLE) {
            set_result(m, out, MR_DONE, seq);
        }
        out->state = m->state;
        return;

    case MC_RESET:
        /* SAFE STOP -> IDLE only on an operator reset with the fault cleared. */
        if (s == MS_SAFE_STOP && src == MC_SRC_MANUAL &&
            (heading_ok(m) || MC_ALLOW_MANUAL_WITHOUT_HEADING)) {
            m->state = MS_IDLE;
            out->fault_clear |= FAULT_MOTOR_OWNED;
            ok = true;
        }
        break;

    case MC_MANUAL_DRIVE:
    case MC_MANUAL_PARK: {
        int vx = (type == MC_MANUAL_DRIVE) ? mc_s8(w, 8) : 0;
        int vy = mc_s8(w, 16);
        int wz = (type == MC_MANUAL_DRIVE) ? mc_s8(w, 24) : 0;
        bool heading_needed_ok = heading_ok(m) || MC_ALLOW_MANUAL_WITHOUT_HEADING;
        ok = src == MC_SRC_MANUAL                         /* FR-40: not from autonomous logic */
          && m->system_ready                              /* FR-40: manual only in READY      */
          && (s == MS_IDLE || s == MS_MANUAL || s == MS_RAMP_DOWN)
          && in_range_pct(vx) && in_range_pct(vy) && in_range_pct(wz)
          && (type != MC_MANUAL_PARK || vy != 0)
          && heading_needed_ok;
        if (ok) {
            if (s != MS_MANUAL || m->tw != 0) set_hold_target(m, m->heading);
            m->tvx = (int8_t)vx; m->tvy = (int8_t)vy; m->tw = (int8_t)wz;
            m->parking = (type == MC_MANUAL_PARK);
            if (m->parking) m->park_end_ms = now_ms + MC_PARK_DURATION_MS;
            m->state = MS_MANUAL;
        }
        break;
    }

    case MC_AUTO_ROTATE: {
        uint16_t hdg = mc_hdg9(w, 8);
        ok = src == MC_SRC_AUTO && hdg <= 359 && heading_ok(m)
          && (s == MS_IDLE || s == MS_AUTO_ROTATE || s == MS_AUTO_DRIVE || s == MS_RAMP_DOWN);
        if (ok) {
            clear_targets(m);
            m->rot_target = (hdg * 10u) & 0xFFFu;
            m->state = MS_AUTO_ROTATE;
        }
        break;
    }

    case MC_AUTO_DRIVE: {
        int speed = mc_s8(w, 8);
        uint16_t hdg = mc_hdg9(w, 16);
        ok = src == MC_SRC_AUTO && in_range_pct(speed) && hdg <= 359 && heading_ok(m)
          && (s == MS_IDLE || s == MS_AUTO_ROTATE || s == MS_AUTO_DRIVE || s == MS_RAMP_DOWN);
        if (ok) {
            clear_targets(m);
            m->tvx = (int8_t)speed;        /* forward/back only: vy = 0 always (FR-40) */
            set_hold_target(m, hdg * 10u);
            m->state = MS_AUTO_DRIVE;
        }
        break;
    }

    default:
        ok = false;
        break;
    }

    if (!ok) {
        set_result(m, out, MR_REJECTED, seq);
        out->state = m->state;
        return;
    }

    if (type != MC_RESET) {
        m->last_cmd_word  = w;
        m->last_cmd_ms    = now_ms;
        m->active_seq     = seq & 0xFu;
        m->active_src     = src;
        m->result_sent    = 0;
        m->brake_until_ms = now_ms;   /* release any brake for the new move */
    }
    out->state = m->state;
}

void mc_immediate_stop(mc_t *m, mc_stop_kind_t kind, uint32_t now_ms, mc_out_t *out) {
    motor_state_t s = (motor_state_t)m->state;
    bool moving = is_moving_state(s);

    if (kind == MC_STOP_OPERATOR) {
        /* Operator stop: PWM 0 at once, back to IDLE, map/mission state untouched. */
        hard_stop(m, (s == MS_SAFE_STOP || s == MS_ZERO_GYRO) ? s : MS_IDLE, now_ms, out);
        return;
    }
    /* E-stop or link lost: SAFE STOP if we were moving; if already still,
     * just make sure the outputs are off (no latch needed). */
    if (moving) {
        hard_stop(m, MS_SAFE_STOP, now_ms, out);
        if (kind == MC_STOP_ESTOP) out->fault_set |= FAULT_ESTOP;
        /* FAULT_LINK_LOST is set by Robo Comms, which detected it. */
    } else {
        hard_stop(m, s, now_ms, out);
    }
}

void mc_step(mc_t *m, uint32_t now_ms, uint32_t heading_word, mc_out_t *out) {
    if (update_heading(m, now_ms, heading_word)) out->fault_set |= FAULT_HEADING_STALE;

    motor_state_t s = (motor_state_t)m->state;

    /* ---- ZERO GYRO: wait for the first good heading ---- */
    if (s == MS_ZERO_GYRO && heading_ok(m)) { m->state = MS_IDLE; s = MS_IDLE; }

    /* ---- faults while moving -> SAFE STOP (no ramp) ---- */
    if (s == MS_MANUAL || s == MS_AUTO_ROTATE || s == MS_AUTO_DRIVE) {
        bool needs_heading = (s != MS_MANUAL) || !MC_ALLOW_MANUAL_WITHOUT_HEADING;
        if (needs_heading && !heading_ok(m)) {
            out->fault_set |= FAULT_HEADING_STALE;
            hard_stop(m, MS_SAFE_STOP, now_ms, out);
            return;
        }
        if ((uint32_t)(now_ms - m->last_cmd_ms) > MC_CMD_TIMEOUT_MS) {   /* FR-42 */
            out->fault_set |= FAULT_CMD_TIMEOUT;
            hard_stop(m, MS_SAFE_STOP, now_ms, out);
            return;
        }
    }
    /* Mission left READY while the operator was driving: ease down. */
    if (s == MS_MANUAL && !m->system_ready) { begin_ramp_down(m); s = MS_RAMP_DOWN; }

    /* ---- requested body motion for this tick ---- */
    int vx = 0, vy = 0, wz = 0;
    switch (s) {
    case MS_MANUAL:
        vx = m->tvx; vy = m->tvy; wz = m->tw;
        if (wz == 0 && heading_ok(m)) wz = hold_trim(m);              /* FR-41 hold      */
        else if (heading_ok(m))       set_hold_target(m, m->heading);  /* turning: re-aim */
        if (m->parking && (int32_t)(now_ms - m->park_end_ms) >= 0) {
            begin_ramp_down(m); s = MS_RAMP_DOWN; vx = vy = wz = 0;
        }
        break;

    case MS_AUTO_DRIVE:
        vx = m->tvx;
        wz = hold_trim(m);                                          /* FR-41 hold */
        break;

    case MS_AUTO_ROTATE: {
        int err = mc_heading_error((uint16_t)m->rot_target, (uint16_t)m->heading);
#if !MC_HEADING_CW_POSITIVE
        err = -err;
#endif
        if (absi(err) <= MC_HEADING_TOL_TENTHS) {                   /* FR-41 tolerance */
            set_result(m, out, MR_HEADING_REACHED, (uint8_t)m->active_seq);
            begin_ramp_down(m); s = MS_RAMP_DOWN;
        } else {
            int mag = absi(err) > MC_ROT_SLOW_ZONE_TENTHS ? MC_ROT_FAST_PCT : MC_ROT_SLOW_PCT;
            wz = err > 0 ? mag : -mag;
        }
        break;
    }
    default:
        break;   /* IDLE, RAMP DOWN, SAFE STOP, ZERO GYRO: no motion requested */
    }

    /* ---- inverse kinematics + ramp (FR-38, FR-42) ---- */
    int target[MC_WHEELS];
    mc_mix(vx, vy, wz, target);
    for (int i = 0; i < MC_WHEELS; i++) {
        int d = m->duty[i];
        int before = d;
        if      (target[i] > d) d = (target[i] - d > MC_RAMP_STEP_PCT) ? d + MC_RAMP_STEP_PCT : target[i];
        else if (target[i] < d) d = (d - target[i] > MC_RAMP_STEP_PCT) ? d - MC_RAMP_STEP_PCT : target[i];
        m->duty[i] = (int8_t)d;
        /* start boost when the wheel starts from 0 or changes direction */
        if (d != 0 && (before == 0 || (before > 0) != (d > 0)))
            m->kick_until_ms[i] = now_ms + (uint32_t)k_kick_ms;
        if (d == 0) m->kick_until_ms[i] = now_ms;
    }

    /* ---- RAMP DOWN finished -> IDLE ---- */
    if (s == MS_RAMP_DOWN && duties_zero(m)) {
        if (!m->result_sent) set_result(m, out, MR_DONE, (uint8_t)m->active_seq);
        m->state = MS_IDLE; s = MS_IDLE;
    }

    /* ---- movement direction for Collision Detection ----
     * Union of where we are asked to go and where the wheels still push us,
     * so a switch from forward to strafe keeps FRONT set until it has ramped.
     * While braking after a hard stop, keep the last mask: the rover may still roll. */
    int evx = (m->duty[0] + m->duty[1] + m->duty[2] + m->duty[3]) / 4;
    int evy = (m->duty[0] - m->duty[1] - m->duty[2] + m->duty[3]) / 4;
    int ewz = (m->duty[0] - m->duty[1] + m->duty[2] - m->duty[3]) / 4;
    uint8_t mask = (uint8_t)(mc_dir_mask(vx, vy, wz) | mc_dir_mask(evx, evy, ewz));
    bool braking = (int32_t)(m->brake_until_ms - now_ms) > 0;
    if (braking && mask == 0) mask = (uint8_t)m->dir_mask;
    if (mask != m->dir_mask || (uint32_t)(now_ms - m->dir_sent_ms) >= MC_DIR_REFRESH_MS) {
        m->dir_mask = mask & 0xFu;
        m->dir_seq  = (m->dir_seq + 1u) & 0xFu;
        m->dir_sent_ms = now_ms;
        out->dir_send = 1;
        out->dir_byte = dir_byte_pack(mask, (uint8_t)m->dir_seq);
    }

    /* ---- final duty per wheel: trim, invert, deadband ---- */
    for (int i = 0; i < MC_WHEELS; i++) {
        int d = m->duty[i] * k_trim[i] / 100;
        if (k_invert[i]) d = -d;
        if (d != 0) {
            int mag = k_deadband[i] + (100 - k_deadband[i]) * absi(d) / 100;
            if ((int32_t)(m->kick_until_ms[i] - now_ms) > 0 && mag < k_kick_pct) mag = k_kick_pct;
            d = d > 0 ? mag : -mag;
        }
        out->wheel[i] = (int8_t)clampi(d, -100, 100);
    }
    out->brake = braking && duties_zero(m);
    out->outputs_valid = 1;
    out->state = m->state;
}
