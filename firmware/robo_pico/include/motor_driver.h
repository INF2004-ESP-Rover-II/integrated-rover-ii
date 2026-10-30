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
    MOTOR_ERR_SAFETY = -5    /* refused: safety lock or SAFE STOP */
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
