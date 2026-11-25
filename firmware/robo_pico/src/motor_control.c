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

static int8_t g_k_deadband[MC_WHEELS] =
    MC_DEADBAND_PCT;                   /* adjustable on the bench */
static int g_k_kick_pct = MC_KICK_PCT; /* adjustable on the bench */
static int g_k_kick_ms  = (int) MC_KICK_MS;
static int g_k_zigzag_ms =
    (int) MC_STRAFE_ZIGZAG_MS; /* adjustable on the bench */
static int8_t g_k_trim_fwd[MC_WHEELS] =
    MC_TRIM_FWD_PCT; /* adjustable on the bench */
static int8_t g_k_trim_rev[MC_WHEELS] =
    MC_TRIM_REV_PCT; /* adjustable on the bench */
static const int8_t g_k_invert[MC_WHEELS] = MC_INVERT;

/* ============================== helpers ============================== */

static inline int
clampi (int value, int lower_bound, int upper_bound)
{
    return value < lower_bound ? lower_bound
                               : (value > upper_bound ? upper_bound : value);
}
static inline int
absi (int value)
{
    return value < 0 ? -value : value;
}
static inline bool
in_range_pct (int value)
{
    return value >= -100 && value <= 100;
}

int16_t
mc_heading_error (uint16_t target, uint16_t current)
{
    /* shortest signed difference, so 359.0 -> 1.0 deg is +2.0, not -358 */
    int error_value = ((int) target - (int) current) % 3600;
    if (error_value >= 1800)
    {
        error_value -= 3600;
    }
    if (error_value < -1800)
    {
        error_value += 3600;
    }
    return (int16_t) error_value;
}

/* Inverse kinematics, mecanum "X" rollers, seen from above.
 * vx + = forward, vy + = right, w + = clockwise.
 * Matches the old motor_driver.c patterns (DIRECTION_RIGHT, ROTATE_RIGHT). */
void
mc_mix (int velocity_x, int velocity_y, int command_word, int out[MC_WHEELS])
{
    out[0] = velocity_x + velocity_y + command_word; /* front-left  */
    out[1] = velocity_x - velocity_y - command_word; /* front-right */
    out[2] = velocity_x - velocity_y + command_word; /* back-left   */
    out[3] = velocity_x + velocity_y - command_word; /* back-right  */

    int peak = 0;
    for (int index = 0; index < MC_WHEELS; index++)
    {
        if (absi(out[index]) > peak)
        {
            peak = absi(out[index]);
        }
    }
    if (peak > 100)
    { /* scale all wheels together, keep the ratio */
        for (int index = 0; index < MC_WHEELS; index++)
        {
            out[index] = out[index] * 100 / peak;
        }
    }
}

uint8_t
mc_dir_mask (int velocity_x, int velocity_y, int command_word)
{
    const int th = 3; /* ignore tiny heading-hold trims */
    if (absi(command_word) >= th && absi(velocity_x) < th &&
        absi(velocity_y) < th)
    {
        return (uint8_t) DIR_ALL; /* rotate */
    }
    uint8_t value_m = 0;
    if (velocity_x >= th)
    {
        value_m |= DIR_FRONT;
    }
    if (velocity_x <= -th)
    {
        value_m |= DIR_BACK;
    }
    if (velocity_y >= th)
    {
        value_m |= DIR_RIGHT;
    }
    if (velocity_y <= -th)
    {
        value_m |= DIR_LEFT;
    }
    if (absi(command_word) > MC_HOLD_MAX_PCT && value_m)
    {
        value_m = (uint8_t) DIR_ALL; /* arc turn sweeps the corners */
    }
    return value_m;
}

/* Heading hold, PI controller (FR-41). The I part removes the steady
 * offset a constant pull (weaker wheel, uneven floor) would leave. */
static int
hold_trim (mc_t * p_control)
{

    int err = mc_heading_error((uint16_t) p_control->hold_target,
                               (uint16_t) p_control->heading);

#if !MC_HEADING_CW_POSITIVE
    err = -err;
#endif

    int integ = clampi(p_control->hold_integ + err, -MC_HOLD_INTEG_LIMIT,
                       MC_HOLD_INTEG_LIMIT);

    p_control->hold_integ = (int16_t) integ;

    int trim = err * MC_HOLD_KP_NUM / MC_HOLD_KP_DEN +
               integ * MC_HOLD_KI_NUM / MC_HOLD_KI_DEN;

    return clampi(trim, -MC_HOLD_MAX_PCT, MC_HOLD_MAX_PCT);
}

static void
set_hold_target (mc_t * p_control, unsigned tenths)
{
    p_control->hold_target = tenths & 0xFFFu;
    p_control->hold_integ  = 0;
}

/* Normal stop: ease down. The finished command is remembered so a late
 * refresh of it can't restart the move. */
static void
begin_ramp_down (mc_t * p_control)
{
    p_control->tvx = p_control->tvy = p_control->tw = 0;
    p_control->parking                              = 0;
    if (p_control->last_cmd_word)
    {
        p_control->done_word = p_control->last_cmd_word;
    }
    p_control->last_cmd_word = 0;
    p_control->state         = MS_RAMP_DOWN;
}

static bool
is_moving_state (motor_state_t state)
{
    return state == MS_MANUAL || state == MS_AUTO_ROTATE ||
           state == MS_AUTO_DRIVE || state == MS_RAMP_DOWN;
}

static bool
duties_zero (const mc_t * p_control)
{
    for (int index = 0; index < MC_WHEELS; index++)
    {
        if (p_control->duty[index] != 0)
        {
            return false;
        }
    }
    return true;
}

static void
set_result (mc_t * p_control, mc_out_t * p_out, move_result_t result,
            uint8_t seq)
{
    p_out->result_valid = 1;
    p_out->result_byte  = result_byte_pack(result, seq);
    if (result != MR_REJECTED)
    {
        p_control->result_sent = 1;
        if (p_control->last_cmd_word)
        {

            p_control->done_word = p_control->last_cmd_word;
            /* ignore late refreshes of it */
        }
    }
}

static void
clear_targets (mc_t * p_control)
{
    p_control->tvx = p_control->tvy = p_control->tw = 0;
    p_control->parking                              = 0;
}

/* Heading freshness (tribunal decision):
 * - age uses a rolling 10-bit ms stamp written by the UART side;
 * - once older than MC_HEADING_STALE_MS the stale flag LATCHES, so the
 *   10-bit wrap (every 1024 ms) can never make old data look fresh again;
 * - the latch clears only when a NEW sequence number arrives that is fresh.
 * Returns true if the latch was just set. */
static bool
update_heading (mc_t * p_control, uint32_t now_ms, uint32_t heading_word)
{
    if (!heading_word_valid(heading_word))
    {
        return false;
    }


    uint16_t age = (uint16_t) (((now_ms & HW_TIME_MASK) -
                                heading_word_time(heading_word)) &
                               HW_TIME_MASK);

    uint8_t seq         = heading_word_seq(heading_word);
    bool    b_was_stale = p_control->stale_latched;

    if (!p_control->heading_seen)
    {
        p_control->heading_seen  = 1;
        p_control->last_hdg_seq  = seq & 0xFu;
        p_control->stale_latched = (age > MC_HEADING_STALE_MS);
    }
    else if (seq != p_control->last_hdg_seq)
    {
        p_control->last_hdg_seq = seq & 0xFu;
        if (age <= MC_HEADING_STALE_MS)
        {
            p_control->stale_latched = 0;
        }
    }
    if (age > MC_HEADING_STALE_MS)
    {
        p_control->stale_latched = 1;
    }

    if (!p_control->stale_latched)
    {
        p_control->heading = heading_word_tenths(heading_word) & 0xFFFu;
    }
    return p_control->stale_latched && !b_was_stale;
}

static bool
heading_ok (const mc_t * p_control)
{
    return p_control->heading_seen && !p_control->stale_latched;
}

/* Enter SAFE STOP or IDLE with PWM forced to 0 immediately (no ramp). */
static void
hard_stop (mc_t * p_control, motor_state_t next, uint32_t now_ms,
           mc_out_t * p_out)
{
    bool b_was_moving = is_moving_state((motor_state_t) p_control->state);
    if (b_was_moving && !p_control->result_sent)
    {
        set_result(p_control, p_out, MR_STOPPED,
                   (uint8_t) p_control->active_seq);
    }
    clear_targets(p_control);
    memset(p_control->duty, 0, sizeof p_control->duty);
    if (p_control->last_cmd_word)
    {
        p_control->done_word = p_control->last_cmd_word;
    }
    p_control->last_cmd_word  = 0;
    p_control->brake_until_ms = now_ms + MC_BRAKE_HOLD_MS;
    p_control->state          = (unsigned) next & 0x7u;
    p_out->outputs_valid      = 1;
    p_out->brake              = 1;
    memset(p_out->wheel, 0, sizeof p_out->wheel);
    p_out->state = p_control->state;
}

/* ============================== public API ============================== */

void
mc_init (mc_t * p_control)
{
    memset(p_control, 0, sizeof *p_control);
#if MC_ALLOW_MANUAL_WITHOUT_HEADING
    p_control->state = MS_IDLE;
#else
    p_control->state =
        MS_ZERO_GYRO; /* until the Sensor Pico finishes zeroing the gyro */
#endif
}

void
mc_out_clear (mc_out_t * p_o)
{
    memset(p_o, 0, sizeof *p_o);
}

void
mc_set_deadband (int wheel, int pct)
{
    pct = clampi(pct, 0, 90);
    for (int index = 0; index < MC_WHEELS; index++)
    {
        if (wheel < 0 || wheel == index)
        {
            g_k_deadband[index] = (int8_t) pct;
        }
    }
}
int
mc_get_deadband (int wheel)
{
    return g_k_deadband[clampi(wheel, 0, MC_WHEELS - 1)];
}

void
mc_set_trim (int wheel, int fwd_pct, int rev_pct)
{
    fwd_pct = clampi(fwd_pct, 0, 100);
    rev_pct = clampi(rev_pct, 0, 100);
    for (int index = 0; index < MC_WHEELS; index++)
    {
        if (wheel < 0 || wheel == index)
        {
            g_k_trim_fwd[index] = (int8_t) fwd_pct;
            g_k_trim_rev[index] = (int8_t) rev_pct;
        }
    }
}
int
mc_get_trim_fwd (int wheel)
{
    return g_k_trim_fwd[clampi(wheel, 0, MC_WHEELS - 1)];
}
int
mc_get_trim_rev (int wheel)
{
    return g_k_trim_rev[clampi(wheel, 0, MC_WHEELS - 1)];
}

void
mc_set_kick (int pct, int milliseconds)
{
    g_k_kick_pct = clampi(pct, 0, 100);
    g_k_kick_ms  = clampi(milliseconds, 0, 1000);
}
int
mc_get_kick_pct (void)
{
    return g_k_kick_pct;
}
int
mc_get_kick_ms (void)
{
    return g_k_kick_ms;
}

void
mc_set_zigzag (int milliseconds)
{
    g_k_zigzag_ms = clampi(milliseconds, 0, 2000);
}
int
mc_get_zigzag (void)
{
    return g_k_zigzag_ms;
}

void
mc_set_system_ready (mc_t * p_control, bool b_ready)
{
    p_control->system_ready = b_ready ? 1u : 0u;
}

void
mc_command (mc_t * p_control, uint32_t command_word, uint32_t now_ms,
            uint32_t heading_word, mc_out_t * p_out)
{
    mc_type_t     type  = mc_type(command_word);
    mc_src_t      src   = mc_src(command_word);
    uint8_t       seq   = mc_seq(command_word);
    motor_state_t state = (motor_state_t) p_control->state;

    if (update_heading(p_control, now_ms, heading_word))
    {
        p_out->fault_set |= FAULT_HEADING_STALE;
    }

    /* A late refresh of a move that already finished is ignored silently,
     * so it can never restart the move or produce a second result. */
    if (command_word != 0 && command_word == p_control->done_word)
    {
        return;
    }

    /* Refresh: the identical word again just keeps the move alive. */
    if (command_word == p_control->last_cmd_word &&
        (state == MS_MANUAL || state == MS_AUTO_ROTATE ||
         state == MS_AUTO_DRIVE))
    {
        p_control->last_cmd_ms = now_ms;
        return;
    }

    bool b_is_valid = false;
    switch (type)
    {
        case MC_STOP:
            /* Normal stop from any source: ease down, report DONE when stopped.
             */
            if (state == MS_MANUAL || state == MS_AUTO_DRIVE ||
                state == MS_AUTO_ROTATE)
            {
                begin_ramp_down(p_control);
            }
            else if (state == MS_IDLE)
            {
                set_result(p_control, p_out, MR_DONE, seq);
            }
            p_out->state = p_control->state;
            return;

        case MC_RESET:
            /* SAFE STOP -> IDLE only on an operator reset with the fault
             * cleared. */
            if (state == MS_SAFE_STOP && src == MC_SRC_MANUAL &&
                (heading_ok(p_control) || MC_ALLOW_MANUAL_WITHOUT_HEADING))
            {
                p_control->state = MS_IDLE;
                p_out->fault_clear |= FAULT_MOTOR_OWNED;
                b_is_valid = true;
            }
            break;

        case MC_MANUAL_DRIVE:
        case MC_MANUAL_PARK:
        {

            int velocity_x =
                (type == MC_MANUAL_DRIVE) ? mc_s8(command_word, 8) : 0;

            int velocity_y = mc_s8(command_word, 16);

            int yaw_rate =
                (type == MC_MANUAL_DRIVE) ? mc_s8(command_word, 24) : 0;


            bool b_heading_needed_ok =
                heading_ok(p_control) || MC_ALLOW_MANUAL_WITHOUT_HEADING;


            b_is_valid =
                src == MC_SRC_MANUAL /* FR-40: not from autonomous logic */
                && p_control->system_ready /* FR-40: manual only in READY */
                && (state == MS_IDLE || state == MS_MANUAL ||
                    state == MS_RAMP_DOWN) &&
                in_range_pct(velocity_x) && in_range_pct(velocity_y) &&
                in_range_pct(yaw_rate) &&
                (type != MC_MANUAL_PARK || velocity_y != 0) &&
                b_heading_needed_ok;

            if (b_is_valid)
            {
                if (state != MS_MANUAL || p_control->tw != 0)
                {
                    set_hold_target(p_control, p_control->heading);
                }
                p_control->tvx     = (int8_t) velocity_x;
                p_control->tvy     = (int8_t) velocity_y;
                p_control->tw      = (int8_t) yaw_rate;
                p_control->parking = (type == MC_MANUAL_PARK);
                if (p_control->parking)
                {
                    p_control->park_end_ms = now_ms + MC_PARK_DURATION_MS;
                }
                p_control->state = MS_MANUAL;
            }
            break;
        }

        case MC_AUTO_ROTATE:
        {
            uint16_t hdg = mc_hdg9(command_word, 8);

            b_is_valid = src == MC_SRC_AUTO && hdg <= 359 &&
                         heading_ok(p_control) &&
                         (state == MS_IDLE || state == MS_AUTO_ROTATE ||
                          state == MS_AUTO_DRIVE || state == MS_RAMP_DOWN);

            if (b_is_valid)
            {
                clear_targets(p_control);
                p_control->rot_target = (hdg * 10u) & 0xFFFu;
                p_control->state      = MS_AUTO_ROTATE;
            }
            break;
        }

        case MC_AUTO_DRIVE:
        {
            int      speed = mc_s8(command_word, 8);
            uint16_t hdg   = mc_hdg9(command_word, 16);

            b_is_valid = src == MC_SRC_AUTO && in_range_pct(speed) &&
                         hdg <= 359 && heading_ok(p_control) &&
                         (state == MS_IDLE || state == MS_AUTO_ROTATE ||
                          state == MS_AUTO_DRIVE || state == MS_RAMP_DOWN);

            if (b_is_valid)
            {
                clear_targets(p_control);

                p_control->tvx = (int8_t) speed;
                /* forward/back only: vy = 0 always (FR-40) */
                set_hold_target(p_control, hdg * 10u);
                p_control->state = MS_AUTO_DRIVE;
            }
            break;
        }

        default:
            b_is_valid = false;
            break;
    }

    if (!b_is_valid)
    {
        set_result(p_control, p_out, MR_REJECTED, seq);
        p_out->state = p_control->state;
        return;
    }

    if (type != MC_RESET)
    {
        p_control->last_cmd_word = command_word;
        p_control->last_cmd_ms   = now_ms;
        p_control->active_seq    = seq & 0xFu;
        p_control->active_src    = src;
        p_control->result_sent   = 0;

        p_control->brake_until_ms = now_ms;
        /* release any brake for the new move */
    }
    p_out->state = p_control->state;
}

void
mc_immediate_stop (mc_t * p_control, mc_stop_kind_t kind, uint32_t now_ms,
                   mc_out_t * p_out)
{
    motor_state_t state    = (motor_state_t) p_control->state;
    bool          b_moving = is_moving_state(state);

    if (kind != MC_STOP_ESTOP)
    {
        /* Operator stop: PWM 0 at once, back to IDLE, map/mission state
         * untouched. */
        hard_stop(p_control,
                  (state == MS_SAFE_STOP || state == MS_ZERO_GYRO) ? state
                                                                   : MS_IDLE,
                  now_ms, p_out);
        return;
    }
    /* E-stop or link lost: SAFE STOP if we were moving; if already still,
     * just make sure the outputs are off (no latch needed). */
    if (b_moving)
    {
        hard_stop(p_control, MS_SAFE_STOP, now_ms, p_out);
        if (kind == MC_STOP_ESTOP)
        {
            p_out->fault_set |= FAULT_ESTOP;
        }
        /* FAULT_LINK_LOST is set by Robo Comms, which detected it. */
    }
    else
    {
        hard_stop(p_control, state, now_ms, p_out);
    }
}

typedef struct
{
    motor_state_t state;
    int           velocity_x;
    int           velocity_y;
    int           yaw_rate;
    _Bool         b_zigzag;
    int           target[4];
    int           evx;
    int           evy;
    int           ewz;
    uint8_t       mask;
    _Bool         b_braking;
} mc_step_context_t;

/** @brief Execute the prepare stage of mc_step. */
static bool
mc_step_prepare (mc_t * p_control, uint32_t now_ms, uint32_t heading_word,
                 mc_out_t * p_out, mc_step_context_t * p_context)
{

    if (update_heading(p_control, now_ms, heading_word))
    {
        p_out->fault_set |= FAULT_HEADING_STALE;
    }


    p_context->state = (motor_state_t) p_control->state;

    /* ---- ZERO GYRO: wait for the first good heading ---- */
    if (p_context->state == MS_ZERO_GYRO && heading_ok(p_control))
    {
        p_control->state = MS_IDLE;
        p_context->state = MS_IDLE;
    }

    /* ---- faults while moving -> SAFE STOP (no ramp) ---- */
    if (p_context->state == MS_MANUAL || p_context->state == MS_AUTO_ROTATE ||
        p_context->state == MS_AUTO_DRIVE)
    {

        bool b_needs_heading =
            (p_context->state != MS_MANUAL) || !MC_ALLOW_MANUAL_WITHOUT_HEADING;

        if (b_needs_heading && !heading_ok(p_control))
        {
            p_out->fault_set |= FAULT_HEADING_STALE;
            hard_stop(p_control, MS_IDLE, now_ms, p_out);
            return true;
        }
        if ((uint32_t) (now_ms - p_control->last_cmd_ms) > MC_CMD_TIMEOUT_MS)
        { /* FR-42 */
            p_out->fault_set |= FAULT_CMD_TIMEOUT;
            hard_stop(p_control, MS_IDLE, now_ms, p_out);
            return true;
        }
    }
    /* Mission left READY while the operator was driving: ease down. */
    if (p_context->state == MS_MANUAL && !p_control->system_ready)
    {
        begin_ramp_down(p_control);
        p_context->state = MS_RAMP_DOWN;
    }

    /* ---- requested body motion for this tick ---- */

    p_context->velocity_x = 0;
    return false;
}

/** @brief Execute the inputs stage of mc_step. */
static bool
mc_step_inputs (mc_step_context_t * p_context)
{


    p_context->velocity_y = 0;

    p_context->yaw_rate = 0;

    p_context->b_zigzag = false;
    return false;
}

/** @brief Execute the motion stage of mc_step. */
static bool
mc_step_motion (mc_t * p_control, uint32_t now_ms, mc_out_t * p_out,
                mc_step_context_t * p_context)
{

    switch (p_context->state)
    {
        case MS_MANUAL:
            p_context->velocity_x = p_control->tvx;
            p_context->velocity_y = p_control->tvy;
            p_context->yaw_rate   = p_control->tw;
            if (p_context->yaw_rate == 0 && heading_ok(p_control))
            {
                p_context->yaw_rate = hold_trim(p_control); /* FR-41 hold */
            }
            else if (heading_ok(p_control))
            {
                set_hold_target(p_control,
                                p_control->heading); /* turning: re-aim */
            }
            /* Zigzag strafe: a pure sideways move is done as the two diagonals
             * in turn (e.g. right = front-right, back-right, front-right ...).
             * Each half uses 2 wheels at full effort and the forward/back parts
             * cancel, so the rover still ends up moving sideways. */
            if (g_k_zigzag_ms > 0 && p_context->velocity_x == 0 &&
                p_context->velocity_y != 0 && p_control->tw == 0)
            {
                int  half    = absi(p_context->velocity_y) / 2;
                bool b_front = ((now_ms / (uint32_t) g_k_zigzag_ms) & 1u) != 0u;
                p_context->velocity_x = b_front ? half : -half;

                p_context->velocity_y =
                    p_context->velocity_y > 0 ? half : -half;

                p_context->b_zigzag = true;
            }
            if (p_control->parking &&
                (int32_t) (now_ms - p_control->park_end_ms) >= 0)
            {
                begin_ramp_down(p_control);
                p_context->state = MS_RAMP_DOWN;

                p_context->velocity_x   = p_context->velocity_y =
                    p_context->yaw_rate = 0;

                p_context->b_zigzag = false; /* ease down */
            }
            break;

        case MS_AUTO_DRIVE:
            p_context->velocity_x = p_control->tvx;
            p_context->yaw_rate   = hold_trim(p_control); /* FR-41 hold */
            break;

        case MS_AUTO_ROTATE:
        {

            int err = mc_heading_error((uint16_t) p_control->rot_target,
                                       (uint16_t) p_control->heading);

#if !MC_HEADING_CW_POSITIVE
            err = -err;
#endif
            if (absi(err) <= MC_HEADING_TOL_TENTHS)
            { /* FR-41 tolerance */
                set_result(p_control, p_out, MR_HEADING_REACHED,
                           (uint8_t) p_control->active_seq);
                // Brake locally at the target instead of continuing to rotate
                // through a ramp-down or waiting for a laptop STOP over MQTT.
                hard_stop(p_control, MS_IDLE, now_ms, p_out);
                return true;
            }
            else
            {

                int mag = absi(err) > MC_ROT_SLOW_ZONE_TENTHS ? MC_ROT_FAST_PCT
                                                              : MC_ROT_SLOW_PCT;

                p_context->yaw_rate = err > 0 ? mag : -mag;
            }
            break;
        }
        default:
            break; /* IDLE, RAMP DOWN, SAFE STOP, ZERO GYRO: no motion requested
                    */
    }
    return false;
}

/** @brief Execute the camera stage of mc_step. */
static bool
mc_step_camera (mc_t * p_control, uint32_t now_ms,
                mc_step_context_t * p_context)
{


    /* ---- inverse kinematics + ramp (FR-38, FR-42) ---- */


    mc_mix(p_context->velocity_x, p_context->velocity_y, p_context->yaw_rate,
           p_context->target);
    for (int index = 0; index < MC_WHEELS; index++)
    {
        int data   = p_control->duty[index];
        int before = data;
        if (p_context->b_zigzag)
        {
            data = p_context->target[index]; /* no ramp: each half is short */
        }
        else if (p_context->target[index] > data)
        {

            data = (p_context->target[index] - data > MC_RAMP_STEP_PCT)
                       ? data + MC_RAMP_STEP_PCT
                       : p_context->target[index];
        }
        else if (p_context->target[index] < data)
        {

            data = (data - p_context->target[index] > MC_RAMP_STEP_PCT)
                       ? data - MC_RAMP_STEP_PCT
                       : p_context->target[index];
        }
        p_control->duty[index] = (int8_t) data;
        /* start boost when the wheel starts from 0 or changes direction */
        if (data != 0 && (before == 0 || (before > 0) != (data > 0)))
        {
            p_control->kick_until_ms[index] = now_ms + (uint32_t) g_k_kick_ms;
        }
        if (data == 0)
        {
            p_control->kick_until_ms[index] = now_ms;
        }
    }
    return false;
}

/** @brief Execute the tracking stage of mc_step. */
static bool
mc_step_tracking (mc_t * p_control, uint32_t now_ms, mc_out_t * p_out,
                  mc_step_context_t * p_context)
{


    /* ---- RAMP DOWN finished -> IDLE ---- */
    if (p_context->state == MS_RAMP_DOWN && duties_zero(p_control))
    {
        if (!p_control->result_sent)
        {
            set_result(p_control, p_out, MR_DONE,
                       (uint8_t) p_control->active_seq);
        }
        p_control->state = MS_IDLE;
        p_context->state = MS_IDLE;
    }

    /* ---- movement direction for Collision Detection ----
     * Union of where we are asked to go and where the wheels still push us,
     * so a switch from forward to strafe keeps FRONT set until it has ramped.
     * While braking after a hard stop, keep the last mask: the rover may still
     * roll. */


    p_context->evx = (p_control->duty[0] + p_control->duty[1] +
                      p_control->duty[2] + p_control->duty[3]) /
                     4;


    p_context->evy = (p_control->duty[0] - p_control->duty[1] -
                      p_control->duty[2] + p_control->duty[3]) /
                     4;


    p_context->ewz = (p_control->duty[0] - p_control->duty[1] +
                      p_control->duty[2] - p_control->duty[3]) /
                     4;


    p_context->mask =
        (uint8_t) ((uint32_t) (mc_dir_mask(p_context->velocity_x,
                                           p_context->velocity_y,
                                           p_context->yaw_rate)) |
                   (uint32_t) (mc_dir_mask(p_context->evx, p_context->evy,
                                           p_context->ewz)));


    p_context->b_braking = (int32_t) (p_control->brake_until_ms - now_ms) > 0;
    if (p_context->b_braking && p_context->mask == 0)
    {
        p_context->mask = (uint8_t) p_control->dir_mask;
    }
    return false;
}

/** @brief Execute the reference stage of mc_step. */
static bool
mc_step_reference (mc_t * p_control, uint32_t now_ms, mc_out_t * p_out,
                   mc_step_context_t * p_context)
{

    if (p_context->mask != p_control->dir_mask ||
        (uint32_t) (now_ms - p_control->dir_sent_ms) >= MC_DIR_REFRESH_MS)
    {
        p_control->dir_mask    = p_context->mask & 0xFu;
        p_control->dir_seq     = (p_control->dir_seq + 1u) & 0xFu;
        p_control->dir_sent_ms = now_ms;
        p_out->dir_send        = 1;

        p_out->dir_byte =
            dir_byte_pack(p_context->mask, (uint8_t) p_control->dir_seq);
    }

    /* ---- final duty per wheel: trim, invert, deadband ---- */
    for (int index = 0; index < MC_WHEELS; index++)
    {

        int data = p_control->duty[index] *
                   (p_control->duty[index] > 0 ? g_k_trim_fwd[index]
                                               : g_k_trim_rev[index]) /
                   100;

        if (g_k_invert[index])
        {
            data = -data;
        }
        if (data != 0)
        {

            int mag = g_k_deadband[index] +
                      (100 - g_k_deadband[index]) * absi(data) / 100;

            if ((int32_t) (p_control->kick_until_ms[index] - now_ms) > 0 &&
                mag < g_k_kick_pct)
            {
                mag = g_k_kick_pct;
            }
            data = data > 0 ? mag : -mag;
        }
        p_out->wheel[index] = (int8_t) clampi(data, -100, 100);
    }
    p_out->brake         = p_context->b_braking && duties_zero(p_control);
    p_out->outputs_valid = 1;
    p_out->state         = p_control->state;
    return false;
}

void
mc_step (mc_t * p_control, uint32_t now_ms, uint32_t heading_word,
         mc_out_t * p_out)
{
    mc_step_context_t context = { 0 };
    if (mc_step_prepare(p_control, now_ms, heading_word, p_out, &context))
    {
        return;
    }
    if (mc_step_inputs(&context))
    {
        return;
    }
    if (mc_step_motion(p_control, now_ms, p_out, &context))
    {
        return;
    }
    if (mc_step_camera(p_control, now_ms, &context))
    {
        return;
    }
    if (mc_step_tracking(p_control, now_ms, p_out, &context))
    {
        return;
    }
    if (mc_step_reference(p_control, now_ms, p_out, &context))
    {
        return;
    }
}

/* End of file. */
