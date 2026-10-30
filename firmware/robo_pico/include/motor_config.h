/**
 * @file motor_config.h
 * @brief Tuning values for Subsystem 6 (Motion Control & Heading).
 *
 * Every number here is PROVISIONAL (agreed at Tribunal 1 so the FRs are
 * testable) and must be tuned on the real rover. See Week 6 Section 7:
 *   A5  marker-to-marker accuracy, A6 manual omni moves,
 *   A7  gyro drift, A12 command timeout + heading tolerance.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* ---------------- Timing ---------------- */
#define MC_PERIOD_MS            20u   /* Motor Control fixed period (vTaskDelayUntil-style) */
#define MC_CMD_TIMEOUT_MS       300u  /* FR-42 / A12: no command or refresh -> SAFE STOP   */
#ifndef MC_HEADING_STALE_MS
#define MC_HEADING_STALE_MS     150u  /* ~3 missed 20 Hz heading frames -> stop            */
#endif                                /* (must stay below 1024 ms: 10-bit receive time)    */
#define MC_BRAKE_HOLD_MS        300u  /* active brake time after an immediate stop          */
#define MC_DIR_REFRESH_MS       500u  /* resend movement direction to the Sensor Pico      */

/* Senders (Exploration, dashboard) must refresh the active command faster
 * than MC_CMD_TIMEOUT_MS. Recommended refresh: every 100 ms. */
#define MC_RECOMMENDED_REFRESH_MS 100u

/* ---------------- Heading (FR-41) ---------------- */
/* Heading is carried in tenths of a degree (0..3599) = 12 bits. */
#define MC_HEADING_TOL_TENTHS   30    /* +/-3.0 deg: rotation counts as done      */
#define MC_ROT_SLOW_ZONE_TENTHS 250   /* within 25 deg of target: rotate slowly   */
#define MC_ROT_FAST_PCT         70    /* rotate-on-spot effort, far from target (floor test: 70 turns, 45 too weak) */
#define MC_ROT_SLOW_PCT         40    /* rotate-on-spot effort, near target        */

/* Heading hold: turn trim = KP * error, error in tenths of a degree.
 * KP_NUM/KP_DEN = 2/10 -> 2 % turn effort per 1 deg of error. */
#define MC_HOLD_KP_NUM          2
#define MC_HOLD_KP_DEN          10
#define MC_HOLD_MAX_PCT         20    /* clamp on the heading-hold correction      */
/* Integral part: removes the steady offset from a constant pull (weak wheel).
 * KI = 1/500 per tick -> about 1 % more turn per second per 1 deg error. */
#define MC_HOLD_KI_NUM          1
#define MC_HOLD_KI_DEN          500
#define MC_HOLD_INTEG_LIMIT     10000 /* anti-windup: I part at most 10 %        */

/* Set to 1 if the IMU heading INCREASES when the rover turns clockwise
 * (seen from above). Confirm with Buddy 1's IMU code before first run. */
#define MC_HEADING_CW_POSITIVE  1

/* ---------------- Speed (A5) ---------------- */
/* Provisional cruise 0.3 m/s. The % that gives 0.3 m/s must be measured;
 * these are effort percentages, not m/s. Exploration chooses the speed. */
#define MC_AUTO_CRUISE_PCT      50
#define MC_AUTO_APPROACH_PCT    30    /* slower near a marker (NFR-05 overshoot) */
#define MC_PARK_PCT             60    /* parallel park strafe effort (35 too weak on the floor) */
#define MC_PARK_DURATION_MS     1500u /* parallel park strafe time               */

/* ---------------- Dashboard key speeds (motor_driver.c) ----------------
 * Power for each dashboard key, 0..100 %. Starting values from the bench
 * tests: strafe and diagonal need more (only 2 wheels push on a diagonal,
 * and mecanum rollers slip sideways). Forward also gets the team's pitch
 * boost on a slope. Tune on the floor, then rebuild. */
#define MC_DASH_STRAIGHT_PCT    50    /* W / S      forward, backward  */
#define MC_DASH_STRAFE_PCT      100   /* A / D      strafe left, right */
#define MC_DASH_DIAGONAL_PCT    80    /* R T F G    diagonals          */
#define MC_DASH_ROTATE_PCT      70    /* Q / E      rotate on the spot */

/* ---------------- Start boost ----------------
 * Motors need more power to START turning than to keep turning (static
 * friction, mecanum rollers sliding sideways). When a wheel starts from 0 or
 * reverses, it gets KICK_PCT for KICK_MS, then drops to the requested speed.
 * Set MC_KICK_PCT to 0 to switch it off. Adjustable on the bench ('kick'). */
#define MC_KICK_PCT             90
#define MC_KICK_MS              150u

/* ---------------- Zigzag strafe ----------------
 * Floor tests: diagonals (2 wheels) move easily but a 4-wheel strafe stalls.
 * With this on, strafe and parallel park alternate the two diagonals every
 * ZIGZAG_MS (e.g. right = front-right, back-right, ...), with no ramp.
 * The rover wobbles slightly forward/back but moves sideways.
 * 0 = off (normal strafe). Adjustable on the bench ('zz MS'). */
#ifndef MC_STRAFE_ZIGZAG_MS
#define MC_STRAFE_ZIGZAG_MS     200u
#endif

/* ---------------- Ramp (FR-42) ---------------- */
#define MC_RAMP_STEP_PCT        5     /* per 20 ms tick -> 0 to 100 % in 400 ms  */

/* ---------------- Per-wheel calibration ----------------
 * Wheel order everywhere: 0 = front-left, 1 = front-right,
 *                         2 = back-left,  3 = back-right.
 * DEADBAND: smallest duty that makes that wheel turn (measure each wheel alone).
 * TRIM:     scales a stronger wheel down so all four match (100 = no change).
 *           Separate values for each wheel spinning FORWARD and BACKWARD,
 *           because a DC motor is often weaker in one direction (floor test:
 *           drifts right going forward, straight going backward).
 *           Tune live on the bench with 'trim W FWD REV'.
 * INVERT:   set to 1 if that wheel spins backwards for a positive command. */
#define MC_DEADBAND_PCT   { 20, 20, 20, 20 }
#define MC_TRIM_FWD_PCT   { 100, 100, 100, 100 }
#define MC_TRIM_REV_PCT   { 100, 100, 100, 100 }
#define MC_INVERT         { 0, 0, 0, 0 }

/* Bench testing only: allow manual moves while no heading is arriving
 * (no Sensor Pico connected). Heading hold is skipped. Keep 0 on the rover. */
#ifndef MC_ALLOW_MANUAL_WITHOUT_HEADING
#define MC_ALLOW_MANUAL_WITHOUT_HEADING 0
#endif

#endif /* MOTOR_CONFIG_H */
