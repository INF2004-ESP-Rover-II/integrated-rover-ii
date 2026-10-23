#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

// Motor driver error codes (as per Section 6.3.5)
typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERR_INIT = -1,
    MOTOR_ERR_PIN = -2,
    MOTOR_ERR_TIMEOUT = -3,
    MOTOR_ERR_OVERCURRENT = -4,
    MOTOR_ERR_SAFETY = -5
} motor_error_t;

// Motor movement directions
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

// Arm movement directions
typedef enum {
    ARM_UP,
    ARM_DOWN,
    ARM_STOP
} arm_direction_t;

// Claw movement directions
typedef enum {
    CLAW_OPEN,
    CLAW_CLOSE,
    CLAW_STOP
} claw_direction_t;

/**
 * @brief Initialize motor driver subsystem
 * Configures GPIO, PWM for all motor pins
 * @return MOTOR_OK on success, error code otherwise
 */
motor_error_t motor_init(void);

/**
 * @brief Set motor movement direction and speed
 * @param direction Movement direction (forward, backward, etc.)
 * @param speed_percent Speed as percentage (0-100)
 * @return MOTOR_OK on success, error code otherwise
 */
motor_error_t motor_set_direction(motor_direction_t direction, int speed_percent);

/**
 * @brief Set motor movement with individual wheel speed adjustments
 * @param direction Movement direction (forward, backward, etc.)
 * @param speed_percent Base speed as percentage (0-100)
 * @param speed_fl Front Left wheel adjustment (0-100%)
 * @param speed_fr Front Right wheel adjustment (0-100%)
 * @param speed_bl Back Left wheel adjustment (0-100%)
 * @param speed_br Back Right wheel adjustment (0-100%)
 * @return MOTOR_OK on success, error code otherwise
 */
motor_error_t motor_set_direction_individual(motor_direction_t direction, int speed_percent,
                                             int speed_fl, int speed_fr, int speed_bl, int speed_br);

/**
 * @brief Control arm movement
 * @param direction ARM_UP, ARM_DOWN, or ARM_STOP
 * @return MOTOR_OK on success, error code otherwise
 */
motor_error_t arm_set_direction(arm_direction_t direction);

/**
 * @brief Control claw movement
 * @param direction CLAW_OPEN, CLAW_CLOSE, or CLAW_STOP
 * @return MOTOR_OK on success, error code otherwise
 */
motor_error_t claw_set_direction(claw_direction_t direction);

/**
 * @brief Emergency stop - applies reverse thrust braking
 * Implements FR-003 (E-Stop requirement)
 * @return MOTOR_OK on success, error code otherwise
 */
motor_error_t motor_emergency_stop(void);

// IRQ-safe safety path: latch and cut outputs without reverse thrust or logging.
void motor_safety_receive(bool stop_requested, uint32_t receive_ms);
void motor_safety_poll(void);
bool motor_safety_is_latched(void);
// Only succeeds with a recent healthy UART frame. Does not start any motor.
bool motor_safety_reset(void);

/**
 * @brief Check if motors are currently active
 * @return true if motors are running, false otherwise
 */
bool motor_is_active(void);

/**
 * @brief Update motor activity timestamp (called by movement functions)
 * Used for timeout detection
 */
void motor_update_activity(void);

/**
 * @brief Check for motor timeout and stop if needed
 * Should be called periodically in main loop
 * Implements NFR-002 safety requirement
 */
void motor_check_timeout(void);

/**
 * @brief Check for brake timeout and stop if needed
 * Should be called periodically in main loop
 */
void motor_check_brake_timeout(void);

void arm_set_pwm_duty(float duty_up, float duty_down);

/**
 * @brief Get last error code
 * @return Last motor error that occurred
 */
motor_error_t motor_get_last_error(void);

/* ---------------- Subsystem 6 additions (Buddy 4) ---------------- */

/**
 * @brief Give Motion Control the latest heading from the Sensor Pico.
 * Call once per sensor packet (safe from the UART interrupt).
 * @param heading_deg 0..360 degrees
 * @param valid false if the IMU reported an error (heading is then ignored)
 */
void motor_heading_update(float heading_deg, bool valid);

/**
 * @brief Parallel park: strafe for a fixed time, then stop by itself.
 * @param direction -1 = left, +1 = right
 * @param power_percent 10..100, or 0 for the default (MC_PARK_PCT)
 */
motor_error_t motor_parallel_park(int direction, int power_percent);

/** @brief Motion Control state (motor_state_t in motor_types.h), for telemetry. */
int motor_motion_state(void);

#endif // MOTOR_DRIVER_H
