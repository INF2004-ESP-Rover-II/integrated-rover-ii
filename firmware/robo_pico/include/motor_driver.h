/**
 * @file motor_driver.h
 * @brief Motor API used by main.c (dashboard keys, autonomous mode, safety).
 *        Runs on the Subsystem 6 Motion Control module underneath.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MOTOR_OK         =  0,
    MOTOR_ERR_INIT   = -1,
    MOTOR_ERR_PIN    = -2,   /* bad direction or speed */
    MOTOR_ERR_SAFETY = -5,   /* refused: safety lock, SAFE STOP, or wrong state */
    MOTOR_ERR_NO_HEADING = -6 /* auto move refused: no fresh heading from the IMU */
} motor_error_t;

typedef enum {
    DIRECTION_FORWARD,
    DIRECTION_BACKWARD,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_ROTATE_LEFT,
    DIRECTION_ROTATE_RIGHT,
    DIRECTION_FRONT_LEFT,
    DIRECTION_FRONT_RIGHT,
    DIRECTION_BACK_LEFT,
    DIRECTION_BACK_RIGHT,
    DIRECTION_STOP
} motor_direction_t;

typedef enum { ARM_UP, ARM_DOWN, ARM_STOP } arm_direction_t;
typedef enum { CLAW_OPEN, CLAW_CLOSE, CLAW_STOP } claw_direction_t;

/** Set up pins, PWM and the 20 ms Motion Control update. Call once. */
motor_error_t motor_init(void);

/** Start or keep a move. Calling again with the same move and speed
 *  keeps it going; stop with motor_emergency_stop().
 *  @param speed_percent 0..100 */
motor_error_t motor_set_direction(motor_direction_t direction, int speed_percent);

/** Speed for a dashboard key, from motor_config.h (MC_DASH_*_PCT). */
int motor_move_speed(motor_direction_t direction);

/** Parallel park: strafe for a fixed time, then stop by itself.
 *  @param direction -1 = left, +1 = right
 *  @param power_percent 10..100, or 0 for the default (MC_PARK_PCT) */
motor_error_t motor_parallel_park(int direction, int power_percent);

/* ---------------- Autonomous moves (for the mapping / Exploration side) ----------------
 * One call starts the move and it KEEPS GOING until motor_auto_stop(), an
 * operator STOP (motor_emergency_stop) or a safety stop (Collision Detection,
 * link lost). No need to resend. Heading hold keeps it straight (FR-41).
 * Needs a fresh heading from the Sensor Pico, else MOTOR_ERR_NO_HEADING.
 * Manual dashboard moves are refused while an auto move runs (FR-40). */

/** Drive straight, holding the current heading. speed: +1..100 forward,
 *  -1..-100 backward (0 = motor_auto_stop). */
motor_error_t motor_auto_drive(int speed_percent);

/** Rotate on the spot to an absolute heading 0..359 (shortest way).
 *  Stops by itself within +/-3 deg; status becomes HEADING_REACHED. */
motor_error_t motor_auto_rotate_to(int heading_deg);

/** Rotate on the spot by an angle: +90 = turn right 90, -90 = left 90. */
motor_error_t motor_auto_rotate_by(int delta_deg);

/** Normal stop of the auto move: ramps down, status becomes DONE. */
motor_error_t motor_auto_stop(void);

typedef enum {
    MOTOR_AUTO_IDLE = 0,          /* no auto move yet                         */
    MOTOR_AUTO_RUNNING,           /* moving                                   */
    MOTOR_AUTO_DONE,              /* stopped normally (motor_auto_stop)       */
    MOTOR_AUTO_HEADING_REACHED,   /* rotation finished within +/-3 deg        */
    MOTOR_AUTO_STOPPED,           /* ended by operator STOP / safety stop     */
    MOTOR_AUTO_REJECTED           /* refused (state, range, FR-40)            */
} motor_auto_status_t;

/** How the last auto move is doing / how it ended. Poll it, or watch the
 *  "AUTO_RESULT:" status the rover publishes. */
motor_auto_status_t motor_auto_status(void);
const char *motor_auto_status_name(motor_auto_status_t s);

/** Arm and claw: on/off. */
motor_error_t arm_set_direction(arm_direction_t direction);
motor_error_t claw_set_direction(claw_direction_t direction);

/** Stop now: PWM 0 + active brake, then idle (dashboard STOP, end of a turn). */
motor_error_t motor_emergency_stop(void);

/** Safety lock from the Sensor Pico (Collision Detection).
 *  motor_safety_receive() is called from the UART interrupt. */
void motor_safety_receive(bool stop_requested, uint32_t receive_ms);
void motor_safety_poll(void);
bool motor_safety_is_latched(void);
bool motor_safety_reset(void);

/** Latest heading from the Sensor Pico, once per packet (safe from the
 *  UART interrupt). valid = false if the IMU reported an error. */
void motor_heading_update(float heading_deg, bool valid);

/** Motion Control state (motor_state_t in motor_types.h), for telemetry.
 *  0 ZERO_GYRO, 1 IDLE, 3 MANUAL, 4 AUTO_ROTATE, 5 AUTO_DRIVE, 6 RAMP_DOWN,
 *  7 SAFE_STOP (moves refused until Reset Safety). */
int motor_motion_state(void);
#define MOTOR_STATE_SAFE_STOP 7

/** Laptop link lost (FR-08): stop now; SAFE STOP if the rover was moving. */
void motor_link_lost(void);

/** Why the rover is in SAFE STOP, as short text for the dashboard. */
typedef enum {
    MOTOR_SAFE_NONE = 0,
    MOTOR_SAFE_CMD_TIMEOUT,    /* moving, then no command for 0.8 s (FR-42) */
    MOTOR_SAFE_LINK_LOST,      /* laptop heartbeat lost while moving (FR-08) */
    MOTOR_SAFE_SENSOR_LOCK,    /* Collision Detection STOP / sensor link lost */
    MOTOR_SAFE_HEADING_STALE   /* heading too old (auto moves)               */
} motor_safe_reason_t;
const char *motor_safe_stop_reason(void);

#endif /* MOTOR_DRIVER_H */
