/**
 * @file motor_control.h
 * @brief Motor Control logic for Subsystem 6 (FR-38 to FR-42).
 *
 * This file has NO hardware and NO FreeRTOS calls, so it can be unit-tested
 * on a laptop (see test/test_motor_control.c). motor_task.c runs it on the
 * Robo Pico every MC_PERIOD_MS and motor_hw.c turns the output into PWM.
 *
 * Statechart (Week 6 Section 6.6):
 *   ZERO GYRO -> IDLE -> VALIDATE -> MANUAL MODE | AUTO ROTATE | AUTO DRIVE
 *   -> RAMP DOWN -> IDLE
 *   e-stop / command timeout / stale heading / link lost -> SAFE STOP
 *   SAFE STOP -> (fault cleared + operator reset) -> IDLE
 *   operator stop -> IDLE with PWM 0 at once
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"

#define MC_WHEELS 4

/** Private state of the Motor Control task. Only the Motor Control task
 *  writes it (single writer), so packing it with bit-fields is race-free. */
typedef struct {
    unsigned state         : 3;  /* motor_state_t                          */
    unsigned active_src    : 1;  /* mc_src_t of the running move            */
    unsigned active_seq    : 4;  /* sequence number of the running move     */
    unsigned result_sent   : 1;  /* result for the running move reported    */
    unsigned parking       : 1;  /* MANUAL is a parallel park               */
    unsigned system_ready  : 1;  /* Mission Control says READY (FR-40)      */
    unsigned heading_seen  : 1;  /* at least one valid heading received     */
    unsigned stale_latched : 1;  /* heading too old; cleared by a new frame */
    unsigned last_hdg_seq  : 4;
    unsigned dir_mask      : 4;  /* last direction mask sent                */
    unsigned dir_seq       : 4;
    unsigned heading       : 12; /* last heading, tenths of a degree        */
    unsigned hold_target   : 12; /* heading to hold, tenths                 */
    unsigned rot_target    : 12; /* rotation target, tenths                 */
    int8_t   tvx, tvy, tw;       /* requested efforts, -100..100 %          */
    int8_t   duty[MC_WHEELS];    /* ramped wheel efforts, -100..100 %       */
    int16_t  hold_integ;         /* heading-hold integral, tenths x ticks   */
    uint32_t kick_until_ms[MC_WHEELS]; /* start boost end time per wheel  */
    uint32_t last_cmd_word;      /* for refresh detection                    */
    uint32_t done_word;          /* last finished move (late refreshes ignored) */
    uint32_t last_cmd_ms;
    uint32_t brake_until_ms;
    uint32_t dir_sent_ms;
    uint32_t park_end_ms;
} mc_t;

/** What the task must do after each call. Clear with mc_out_clear(). */
typedef struct {
    int8_t   wheel[MC_WHEELS]; /* final signed duty -100..100 (deadband, trim, invert applied) */
    uint8_t  dir_byte;         /* movement direction byte for the Sensor Pico */
    uint8_t  result_byte;      /* move result for the Exploration Task        */
    uint8_t  fault_set;        /* bits to SET in the shared fault mask        */
    uint8_t  fault_clear;      /* bits to CLEAR in the shared fault mask      */
    unsigned brake        : 1; /* hold the active brake (IN1 = IN2, EN on)    */
    unsigned dir_send     : 1; /* dir_byte must be sent now                   */
    unsigned result_valid : 1; /* result_byte must be delivered               */
    unsigned outputs_valid: 1; /* wheel[] / brake must be applied             */
    unsigned state        : 3; /* motor_state_t, for telemetry                */
} mc_out_t;

typedef enum {
    MC_STOP_ESTOP     = 0, /* Collision Detection STOP over UART -> SAFE STOP */
    MC_STOP_OPERATOR  = 1, /* dashboard STOP button          -> IDLE          */
    MC_STOP_LINK_LOST = 2  /* Robo Comms link timeout (FR-08) -> SAFE STOP    */
} mc_stop_kind_t;

void mc_init(mc_t *m);
void mc_out_clear(mc_out_t *o);

/** Mission Control state gate for FR-40 (manual only in READY). */
void mc_set_system_ready(mc_t *m, bool ready);

/** VALIDATE COMMAND: check one packed command word and start the move,
 *  refresh it, or reject it (result MR_REJECTED). */
void mc_command(mc_t *m, uint32_t cmd_word, uint32_t now_ms,
                uint32_t heading_word, mc_out_t *out);

/** Immediate stop: PWM to 0 at once, no ramp (FR-42, NFR-03). */
void mc_immediate_stop(mc_t *m, mc_stop_kind_t kind, uint32_t now_ms, mc_out_t *out);

/** Periodic update, call every MC_PERIOD_MS. */
void mc_step(mc_t *m, uint32_t now_ms, uint32_t heading_word, mc_out_t *out);

/** Minimum starting power per wheel (wheel -1 = all). Bench tuning only:
 *  call it while the rover is stopped. */
void mc_set_deadband(int wheel, int pct);
int  mc_get_deadband(int wheel);

/** Start boost: pct of power for ms when a wheel starts or reverses (0 = off). */
void mc_set_kick(int pct, int ms);
int  mc_get_kick_pct(void);
int  mc_get_kick_ms(void);

/* ---- helpers, exposed for unit tests ---- */
int16_t mc_heading_error(uint16_t target_tenths, uint16_t current_tenths); /* -1800..1799 */
void    mc_mix(int vx, int vy, int w, int out[MC_WHEELS]);                 /* inverse kinematics */
uint8_t mc_dir_mask(int vx, int vy, int w);

static inline motor_state_t mc_state(const mc_t *m) { return (motor_state_t)m->state; }

#endif /* MOTOR_CONTROL_H */
