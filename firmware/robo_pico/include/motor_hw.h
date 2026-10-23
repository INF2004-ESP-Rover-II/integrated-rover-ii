/**
 * @file motor_hw.h
 * @brief Robo Pico motor hardware: 2 x L298N, claw/arm pins, direction UART TX.
 *
 * Only the Motor Control task calls motor_hw_apply(), so the motor pins have
 * exactly one writer.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_control.h"

/** Call FIRST in main(), before anything else: drives the claw/arm pins
 *  (GP8-GP11) and all L298N inputs to a safe level so nothing twitches at
 *  power-up (prof feedback). */
void motor_hw_safe_boot(void);

/** PWM on the four enable pins (10 kHz), direction pins as GPIO. */
void motor_hw_init(void);

/** Apply one Motor Control output: signed duty per wheel, or active brake. */
void motor_hw_apply(const mc_out_t *out);

/** Change the wheel PWM frequency (100..20000 Hz). Bench tuning only. */
void     motor_hw_set_pwm_freq(uint32_t hz);
uint32_t motor_hw_pwm_freq(void);

/** Coast all four wheels (enable off). */
void motor_hw_all_off(void);

/** Return-line UART TX (Robo GP12 -> Sensor GP17) for the direction byte.
 *  Uses uart0, which Robo Comms already opened for RX on GP13. */
void motor_hw_dir_uart_init(void);
void motor_hw_send_dir(uint8_t dir_byte);

/** Claw / arm: plain on/off GPIO (see motor_hw.c for why not PWM). */
typedef enum { MH_OFF = 0, MH_FWD = 1, MH_REV = 2 } mh_dir_t;
void motor_hw_claw(mh_dir_t d);   /* FWD = close, REV = open */
void motor_hw_arm(mh_dir_t d);    /* FWD = down,  REV = up   */

#endif /* MOTOR_HW_H */
