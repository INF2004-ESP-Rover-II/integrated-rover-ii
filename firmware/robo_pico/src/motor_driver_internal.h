/**
 * @file motor_driver_internal.h
 * @brief Private header shared by the motor driver source files
 *        (motor_driver.c, motor_manual.c, motor_auto.c, motor_heading.c,
 *        motor_safety.c, motor_arm_claw.c). Not part of the public API:
 *        other modules include motor_driver.h only.
 *
 * The driver state below is used by the 20 ms timer interrupt and by the
 * public calls, so every member is volatile (BARR-C 1.8.c). Multi-step
 * updates run with interrupts disabled, as before the split.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#ifndef MOTOR_DRIVER_INTERNAL_H
#define MOTOR_DRIVER_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_control.h"
#include "motor_driver.h"
#include "motor_lease.h"
#include "pico/stdlib.h"

/** Driver state shared by the source files (one instance: g_motor). */
typedef struct
{
    volatile mc_t     mc;              /* Motion Control instance */
    volatile bool     manual_estop;    /* dashboard E-STOP latch */
    volatile bool     forward_driving; /* front obstacle stops it */
    volatile bool     front_blocked;
    volatile uint8_t  safe_reason;  /* why SAFE STOP (motor_safe_reason_t) */
    volatile uint32_t heading_word; /* one 32-bit store / load */
    volatile uint32_t word;         /* manual move being held (0 = none) */
    volatile uint32_t last_call_ms;
    volatile uint8_t  seq;
    volatile uint32_t auto_word; /* auto move being held (0 = none) */
    volatile uint8_t  auto_seq;
    volatile uint8_t  auto_status; /* motor_auto_status_t */
    volatile motor_lease_t auto_lease;
} motor_ctx_t;

extern motor_ctx_t g_motor;

/** Milliseconds since boot. */
static inline uint32_t
motor_now_ms (void)
{
    return to_ms_since_boot(get_absolute_time());
}

/* motor_driver.c: helpers. Call them with interrupts disabled. */
void          motor_core_note_result (const mc_out_t * o);
void          motor_core_apply (const mc_out_t * o);
uint8_t       motor_core_next_seq (void);
motor_error_t motor_core_submit (uint32_t word);
void          motor_core_hard_stop (mc_stop_kind_t kind);

/* motor_heading.c: reset at motor_init(). */
void motor_heading_reset (void);

/* motor_safety.c: reset at motor_init(). */
void motor_safety_init (void);

#endif /* MOTOR_DRIVER_INTERNAL_H */

/* End of file. */
