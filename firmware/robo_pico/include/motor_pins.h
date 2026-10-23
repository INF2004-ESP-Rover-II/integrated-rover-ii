/**
 * @file motor_pins.h
 * @brief Robo Pico motor, claw and arm pins (Week 6 Section 2, unchanged wiring).
 *
 * Same values as the team's config.h. Each one is guarded, so if config.h is
 * also included the two files never clash. Kept separate so the motor code
 * does not need config.h (which also holds WiFi settings).
 */
#ifndef MOTOR_PINS_H
#define MOTOR_PINS_H

/* Front L298N */
#ifndef F_ENA_PIN
#define F_ENA_PIN   4
#endif
#ifndef F_ENB_PIN
#define F_ENB_PIN   5
#endif
#ifndef F_IN1_PIN
#define F_IN1_PIN   3
#endif
#ifndef F_IN2_PIN
#define F_IN2_PIN   2
#endif
#ifndef F_IN3_PIN
#define F_IN3_PIN   16
#endif
#ifndef F_IN4_PIN
#define F_IN4_PIN   17
#endif

/* Back L298N */
#ifndef B_ENA_PIN
#define B_ENA_PIN   27
#endif
#ifndef B_ENB_PIN
#define B_ENB_PIN   26
#endif
#ifndef B_IN1_PIN
#define B_IN1_PIN   28
#endif
#ifndef B_IN2_PIN
#define B_IN2_PIN   7
#endif
#ifndef B_IN3_PIN
#define B_IN3_PIN   0
#endif
#ifndef B_IN4_PIN
#define B_IN4_PIN   1
#endif

/* Claw (GP8 close, GP9 open) and arm (GP11 down, GP10 up) */
#ifndef CLAW_M1A_PIN
#define CLAW_M1A_PIN 8
#endif
#ifndef CLAW_M1B_PIN
#define CLAW_M1B_PIN 9
#endif
#ifndef ARM_M1A_PIN
#define ARM_M1A_PIN  11
#endif
#ifndef ARM_M1B_PIN
#define ARM_M1B_PIN  10
#endif

/* Wheel PWM frequency */
#ifndef PWM_FREQ_HZ
#define PWM_FREQ_HZ  10000
#endif

#endif /* MOTOR_PINS_H */
