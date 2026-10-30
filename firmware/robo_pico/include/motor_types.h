/**
 * @file motor_types.h
 * @brief Minimum-bit data layouts for Subsystem 6 (Change Requirement 1).
 *
 * Every field below uses only the bits its valid range needs. Anything that
 * crosses a link (queue, UART, MQTT) uses EXPLICIT shifts and masks, not C
 * bit-fields, so the Sensor Pico, the Robo Pico and the Python dashboard all
 * read the same bit positions. Bit-fields are only used for data private to
 * the Motor Control task.
 *
 * Data representation table (Week 13 evidence):
 *
 *  Field                    Range                       Bits  Where
 *  -----------------------  --------------------------  ----  ------------------------
 *  command type             7 types (1..7)              3     motor command word
 *  command source           auto / manual               1     motor command word
 *  command sequence         0..15                       4     motor command word
 *  vx, vy, w (manual)       -100..+100 %                8 ea  motor command word
 *  drive speed (auto)       -100..+100 %                8     motor command word
 *  target heading           0..359 deg (1 deg)          9     motor command word
 *  measured heading         0..359.9 deg (0.1 deg)      12    heading word
 *  heading sequence         0..15                       4     heading word
 *  heading receive time     0..1023 ms (rolling)        10    heading word
 *  heading valid            yes / no                    1     heading word
 *  motor state              8 states                    3     telemetry / private
 *  movement direction       front/back/left/right mask  4     direction byte (UART)
 *  direction sequence       0..15                       4     direction byte (UART)
 *  move result              4 results                   2     result byte
 *  result sequence          0..15                       4     result byte
 *  fault flags              1 bit each                  1 ea  shared fault mask
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ======================= Motor state (3 bits) ======================= */
/* Matches the Section 6.6 motor statechart. */
typedef enum {
    MS_ZERO_GYRO   = 0, /* waiting for the first heading (gyro being zeroed) */
    MS_IDLE        = 1, /* all PWM 0, waiting for a command                   */
    MS_VALIDATE    = 2, /* transient: command being checked                   */
    MS_MANUAL      = 3, /* MANUAL MODE (dashboard, only in READY)             */
    MS_AUTO_ROTATE = 4, /* AUTO MODE: ROTATE TO HEADING                       */
    MS_AUTO_DRIVE  = 5, /* AUTO MODE: DRIVE FORWARD/BACK with heading hold    */
    MS_RAMP_DOWN   = 6, /* easing PWM to 0 after a normal stop                */
    MS_SAFE_STOP   = 7  /* PWM forced 0, commands ignored until reset         */
} motor_state_t;
#define MOTOR_STATE_BITS 3u

/* ===================== Motor command word (32 bits) =====================
 *
 *  bit  31            24 23            16 15             8 7    4 3   2  0
 *      +----------------+----------------+----------------+------+---+----+
 *      |   payload hi   |  payload mid   |  payload lo    | seq  |src|type|
 *      +----------------+----------------+----------------+------+---+----+
 *
 *  type MC_MANUAL_DRIVE : lo = vx (int8), mid = vy (int8), hi = w (int8)
 *  type MC_MANUAL_PARK  : mid = vy (int8)  sign = side, size = speed
 *  type MC_AUTO_ROTATE  : bits 8..16  = target heading 0..359 (9 bits)
 *  type MC_AUTO_DRIVE   : lo = speed (int8, + forward / - back),
 *                         bits 16..24 = heading to hold 0..359 (9 bits)
 *  type MC_STOP, MC_RESET : no payload
 *
 *  Conventions: vx + = forward, vy + = right, w + = clockwise (turn right).
 *  Resending the identical word is a REFRESH (keeps the command alive).
 *  A new move with the same intent must use a new sequence number.
 */
typedef enum {
    MC_INVALID      = 0,
    MC_MANUAL_DRIVE = 1, /* forward/back, strafe, diagonal, arc, rotate (FR-39) */
    MC_MANUAL_PARK  = 2, /* parallel park: strafe into gap holding heading      */
    MC_AUTO_ROTATE  = 3, /* rotate on the spot to a heading (FR-40, FR-41)      */
    MC_AUTO_DRIVE   = 4, /* drive forward/back holding a heading (FR-40, FR-41) */
    MC_STOP         = 5, /* normal stop -> RAMP DOWN (e.g. marker reached)      */
    MC_RESET        = 6  /* operator reset from SAFE STOP                       */
} mc_type_t;

typedef enum { MC_SRC_AUTO = 0, MC_SRC_MANUAL = 1 } mc_src_t;

#define MC_TYPE_MASK   0x7u
#define MC_SRC_SHIFT   3u
#define MC_SEQ_SHIFT   4u
#define MC_SEQ_MASK    0xFu
#define MC_HDG9_MASK   0x1FFu

static inline uint32_t mc_pack_header(mc_type_t t, mc_src_t s, uint8_t seq) {
    return ((uint32_t)t & MC_TYPE_MASK) |
           (((uint32_t)s & 1u) << MC_SRC_SHIFT) |
           (((uint32_t)seq & MC_SEQ_MASK) << MC_SEQ_SHIFT);
}

static inline uint32_t mc_pack_manual_drive(uint8_t seq, int8_t vx, int8_t vy, int8_t w) {
    return mc_pack_header(MC_MANUAL_DRIVE, MC_SRC_MANUAL, seq) |
           ((uint32_t)(uint8_t)vx << 8) | ((uint32_t)(uint8_t)vy << 16) |
           ((uint32_t)(uint8_t)w << 24);
}

static inline uint32_t mc_pack_manual_park(uint8_t seq, int8_t vy) {
    return mc_pack_header(MC_MANUAL_PARK, MC_SRC_MANUAL, seq) |
           ((uint32_t)(uint8_t)vy << 16);
}

static inline uint32_t mc_pack_auto_rotate(uint8_t seq, uint16_t heading_deg) {
    return mc_pack_header(MC_AUTO_ROTATE, MC_SRC_AUTO, seq) |
           (((uint32_t)heading_deg & MC_HDG9_MASK) << 8);
}

static inline uint32_t mc_pack_auto_drive(uint8_t seq, int8_t speed, uint16_t heading_deg) {
    return mc_pack_header(MC_AUTO_DRIVE, MC_SRC_AUTO, seq) |
           ((uint32_t)(uint8_t)speed << 8) |
           (((uint32_t)heading_deg & MC_HDG9_MASK) << 16);
}

static inline uint32_t mc_pack_simple(mc_type_t t, mc_src_t s, uint8_t seq) {
    return mc_pack_header(t, s, seq);
}

static inline mc_type_t mc_type(uint32_t w)   { return (mc_type_t)(w & MC_TYPE_MASK); }
static inline mc_src_t  mc_src(uint32_t w)    { return (mc_src_t)((w >> MC_SRC_SHIFT) & 1u); }
static inline uint8_t   mc_seq(uint32_t w)    { return (uint8_t)((w >> MC_SEQ_SHIFT) & MC_SEQ_MASK); }
static inline int8_t    mc_s8(uint32_t w, unsigned shift) { return (int8_t)(uint8_t)(w >> shift); }
static inline uint16_t  mc_hdg9(uint32_t w, unsigned shift) { return (uint16_t)((w >> shift) & MC_HDG9_MASK); }

/* ===================== Heading word (32 bits, shared) =====================
 * Written by the UART receive side in ONE 32-bit store, read by the Motor
 * Control task in ONE 32-bit load (atomic on the Cortex-M0+), so the reader
 * can never see half an old value and half a new one.
 *
 *  bits 0..11  heading, tenths of a degree, 0..3599
 *  bits 12..15 sequence number from the Sensor Pico frame
 *  bits 16..25 receive time, ms since boot & 0x3FF (rolling 10-bit)
 *  bit  26     valid (at least one heading received)
 */
#define HW_HDG_MASK    0xFFFu
#define HW_SEQ_SHIFT   12u
#define HW_SEQ_MASK    0xFu
#define HW_TIME_SHIFT  16u
#define HW_TIME_MASK   0x3FFu
#define HW_VALID_BIT   (1u << 26)

static inline uint32_t heading_word_pack(uint16_t tenths, uint8_t seq, uint32_t now_ms) {
    if (tenths > 3599u) tenths = (uint16_t)(tenths % 3600u);
    return ((uint32_t)tenths & HW_HDG_MASK) |
           (((uint32_t)seq & HW_SEQ_MASK) << HW_SEQ_SHIFT) |
           ((now_ms & HW_TIME_MASK) << HW_TIME_SHIFT) |
           HW_VALID_BIT;
}
static inline uint16_t heading_word_tenths(uint32_t w) { return (uint16_t)(w & HW_HDG_MASK); }
static inline uint8_t  heading_word_seq(uint32_t w)    { return (uint8_t)((w >> HW_SEQ_SHIFT) & HW_SEQ_MASK); }
static inline uint16_t heading_word_time(uint32_t w)   { return (uint16_t)((w >> HW_TIME_SHIFT) & HW_TIME_MASK); }
static inline bool     heading_word_valid(uint32_t w)  { return (w & HW_VALID_BIT) != 0u; }

/* ============== Movement direction byte (Robo Pico -> Sensor Pico) ==============
 * Sent on the return UART line (Robo GP12 TX -> Sensor GP17 RX) so Collision
 * Detection knows which sensors to watch. One bit per side the rover is
 * moving TOWARD. Rotating sets all four. Stopped = 0.
 *  bits 0..3 mask, bits 4..7 sequence.
 * If no byte arrives for ~300 ms, Collision Detection should watch ALL sensors. */
#define DIR_FRONT 0x1u
#define DIR_BACK  0x2u
#define DIR_LEFT  0x4u
#define DIR_RIGHT 0x8u
#define DIR_ALL   0xFu

static inline uint8_t dir_byte_pack(uint8_t mask, uint8_t seq) {
    return (uint8_t)((mask & 0xFu) | ((seq & 0xFu) << 4));
}

/* ================= Move result (to the Exploration Task) ================= */
typedef enum {
    MR_DONE            = 0, /* move ended normally (ramp-down finished)         */
    MR_HEADING_REACHED = 1, /* rotation finished within the heading tolerance   */
    MR_STOPPED         = 2, /* ended by e-stop, operator stop, timeout or fault */
    MR_REJECTED        = 3  /* command refused (FR-40 rules, range, state)      */
} move_result_t;

/* result byte: bits 0..1 result, bits 2..5 sequence of the command it answers */
static inline uint8_t result_byte_pack(move_result_t r, uint8_t seq) {
    return (uint8_t)(((uint8_t)r & 0x3u) | ((seq & 0xFu) << 2));
}

/* ================= Shared fault mask (one bit each) =================
 * Lives in a FreeRTOS event group so several tasks can set/clear bits safely.
 * Only TASKS write it. The STOP interrupt never writes it directly: it only
 * notifies the Motor Control task, which stops the motors and then sets
 * FAULT_ESTOP. Bit numbers must be agreed team-wide (Buddy 5 owns the list). */
#define FAULT_ESTOP          (1u << 0)  /* set by Motor Control after a UART STOP  */
#define FAULT_CMD_TIMEOUT    (1u << 1)  /* set by Motor Control (FR-42)            */
#define FAULT_HEADING_STALE  (1u << 2)  /* set by Motor Control                    */
#define FAULT_IMU            (1u << 3)  /* set by IMU side (Sensor Pico report)    */
#define FAULT_LINK_LOST      (1u << 4)  /* set by Robo Comms (FR-08)               */
#define FAULT_MOTOR_STALL    (1u << 5)  /* RESERVED: no current sensing to detect  */
#define FAULT_ENCODER        (1u << 6)  /* RESERVED: rover has no wheel encoders   */
#define FAULT_MOTOR_OWNED    (FAULT_ESTOP | FAULT_CMD_TIMEOUT | FAULT_HEADING_STALE)

#endif /* MOTOR_TYPES_H */
