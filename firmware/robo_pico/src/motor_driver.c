#include "motor_driver.h"
#include "config.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "safety_interlock.h"

// =================== PRIVATE STATE ===================
static uint16_t wrapFA, wrapFB, wrapBA, wrapBB;
static volatile uint32_t last_command_time = 0;
static volatile bool motors_active = false;
static motor_error_t last_error = MOTOR_OK;
static uint16_t wrapArmA = 0;
static uint16_t wrapArmB = 0;
static uint16_t wrapClawA = 0;
static uint16_t wrapClawB = 0;

// Braking state variables
static bool braking_active = false;
static uint32_t brake_start_time = 0;
static const uint32_t BRAKE_DURATION_MS = 300; // Brake for 300ms
static safety_interlock_t safety = {.latched = true, .stop_requested = true};

// =================== PRIVATE FUNCTION DECLARATIONS ====================
static void arm_off(void);
static void claw_off(void);
static void stop_all_motors_final(void);
static void apply_emergency_brake(void);
static void arm_up_on(void);
static void arm_down_on(void);
static void claw_open_on(void);
static void claw_close_on(void);

// =================== PWM HELPERS ====================
static uint16_t pwm_wrap_for_freq(uint32_t sys_hz, float clkdiv, uint32_t freq_hz) {
    float fwrap = (float)sys_hz / (clkdiv * (float)freq_hz) - 1.0f;
    if (fwrap < 1) fwrap = 1;
    if (fwrap > 65535) fwrap = 65535;
    return (uint16_t)(fwrap + 0.5f);
}

static void pwm_setup_pin(uint pin, uint32_t freq_hz, float clkdiv, uint16_t *out_wrap) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    uint16_t wrap = pwm_wrap_for_freq(125000000u, clkdiv, freq_hz);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clkdiv);
    pwm_config_set_wrap(&cfg, wrap);
    pwm_init(slice, &cfg, true);
    pwm_set_chan_level(slice, channel, 0);

    if (out_wrap) *out_wrap = wrap;
}

static void pwm_set_duty_percent(uint pin, uint16_t wrap, float duty_percent) {
    if (duty_percent < 0) duty_percent = 0;
    if (duty_percent > 100) duty_percent = 100;
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    uint16_t level = (uint16_t)((duty_percent / 100.0f) * wrap);
    pwm_set_chan_level(slice, channel, level);
}

// ================== MOTOR PRIMITIVES =================
static void frontA_set(int speed_percent, uint16_t wrap) {
    if (speed_percent > 0) {
        gpio_put(F_IN1_PIN, 1);
        gpio_put(F_IN2_PIN, 0);
        pwm_set_duty_percent(F_ENA_PIN, wrap, speed_percent);
    } else if (speed_percent < 0) {
        gpio_put(F_IN1_PIN, 0);
        gpio_put(F_IN2_PIN, 1);
        pwm_set_duty_percent(F_ENA_PIN, wrap, -speed_percent);
    } else {
        pwm_set_duty_percent(F_ENA_PIN, wrap, 0);
    }
}

static void frontB_set(int speed_percent, uint16_t wrap) {
    if (speed_percent > 0) {
        gpio_put(F_IN3_PIN, 1);
        gpio_put(F_IN4_PIN, 0);
        pwm_set_duty_percent(F_ENB_PIN, wrap, speed_percent);
    } else if (speed_percent < 0) {
        gpio_put(F_IN3_PIN, 0);
        gpio_put(F_IN4_PIN, 1);
        pwm_set_duty_percent(F_ENB_PIN, wrap, -speed_percent);
    } else {
        pwm_set_duty_percent(F_ENB_PIN, wrap, 0);
    }
}

static void backA_set(int speed_percent, uint16_t wrap) {
    if (speed_percent > 0) {
        gpio_put(B_IN1_PIN, 1);
        gpio_put(B_IN2_PIN, 0);
        pwm_set_duty_percent(B_ENA_PIN, wrap, speed_percent);
    } else if (speed_percent < 0) {
        gpio_put(B_IN1_PIN, 0);
        gpio_put(B_IN2_PIN, 1);
        pwm_set_duty_percent(B_ENA_PIN, wrap, -speed_percent);
    } else {
        pwm_set_duty_percent(B_ENA_PIN, wrap, 0);
    }
}

static void backB_set(int speed_percent, uint16_t wrap) {
    if (speed_percent > 0) {
        gpio_put(B_IN3_PIN, 1);
        gpio_put(B_IN4_PIN, 0);
        pwm_set_duty_percent(B_ENB_PIN, wrap, speed_percent);
    } else if (speed_percent < 0) {
        gpio_put(B_IN3_PIN, 0);
        gpio_put(B_IN4_PIN, 1);
        pwm_set_duty_percent(B_ENB_PIN, wrap, -speed_percent);
    } else {
        pwm_set_duty_percent(B_ENB_PIN, wrap, 0);
    }
}

// ================== BRAKING FUNCTIONS =================
static void apply_emergency_brake(void) {
    // Apply reverse thrust to all wheels
    // Front wheels: spin backward, Back wheels: spin forward
    int brake_power = 100; // 60% power for braking
    
    frontA_set(-brake_power, wrapFA);  // Front left: backward
    frontB_set(-brake_power, wrapFB);  // Front right: backward  
    backA_set(brake_power, wrapBA);    // Back left: forward
    backB_set(brake_power, wrapBB);    // Back right: forward
    
    braking_active = true;
    brake_start_time = to_ms_since_boot(get_absolute_time());
    
    // Logging is done after restoring interrupts in motor_emergency_stop().
}

static void stop_all_motors_final(void) {
    // Completely stop all motors
    frontA_set(0, wrapFA);
    frontB_set(0, wrapFB);
    backA_set(0, wrapBA);
    backB_set(0, wrapBB);
    arm_off();
    claw_off();
    
    braking_active = false;
    motors_active = false;
}

// ================== ARM CONTROL ======================
static void arm_down_on(void) {
    gpio_set_function(ARM_M1B_PIN, GPIO_FUNC_SIO);
    gpio_put(ARM_M1B_PIN, 0);
    pwm_set_duty_percent(ARM_M1B_PIN, wrapArmB, 0);

    gpio_set_function(ARM_M1A_PIN, GPIO_FUNC_PWM);
    pwm_set_duty_percent(ARM_M1A_PIN, wrapArmA, 80.0f);
}

static void arm_up_on(void) {
    gpio_set_function(ARM_M1A_PIN, GPIO_FUNC_SIO);
    gpio_put(ARM_M1A_PIN, 0);
    pwm_set_duty_percent(ARM_M1A_PIN, wrapArmA, 0);

    gpio_set_function(ARM_M1B_PIN, GPIO_FUNC_PWM);
    pwm_set_duty_percent(ARM_M1B_PIN, wrapArmB, 100.0f);
}

static void arm_off(void) {
    gpio_set_function(ARM_M1A_PIN, GPIO_FUNC_SIO);
    gpio_set_function(ARM_M1B_PIN, GPIO_FUNC_SIO);
    pwm_set_duty_percent(ARM_M1A_PIN, wrapArmA, 0);
    pwm_set_duty_percent(ARM_M1B_PIN, wrapArmB, 0);
#if ARM_BRAKE_ON_STOP
    gpio_put(ARM_M1A_PIN, 1);
    gpio_put(ARM_M1B_PIN, 1);
#else
    gpio_put(ARM_M1A_PIN, 0);
    gpio_put(ARM_M1B_PIN, 0);
#endif
}

void arm_set_pwm_duty(float duty_up, float duty_down) {
    uint32_t irq_state = save_and_disable_interrupts();
    if (safety.latched) {
        restore_interrupts(irq_state);
        return;
    }
    if (duty_up < 0) duty_up = 0;
    if (duty_up > 100) duty_up = 100;
    if (duty_down < 0) duty_down = 0;
    if (duty_down > 100) duty_down = 100;
    pwm_set_duty_percent(ARM_M1A_PIN, wrapArmA, duty_up);
    pwm_set_duty_percent(ARM_M1B_PIN, wrapArmB, duty_down);
    restore_interrupts(irq_state);
}

// ================== CLAW CONTROL ======================
static void claw_open_on(void) {
    // Per wiring: CLAW_M1B (PIN9) opens
    gpio_set_function(CLAW_M1A_PIN, GPIO_FUNC_SIO);
    gpio_put(CLAW_M1A_PIN, 0);
    pwm_set_duty_percent(CLAW_M1A_PIN, wrapClawA, 0);

    gpio_set_function(CLAW_M1B_PIN, GPIO_FUNC_PWM);
    pwm_set_duty_percent(CLAW_M1B_PIN, wrapClawB, 80.0f);
}

static void claw_close_on(void) {
    // Per wiring: CLAW_M1A (PIN8) closes
    gpio_set_function(CLAW_M1B_PIN, GPIO_FUNC_SIO);
    gpio_put(CLAW_M1B_PIN, 0);
    pwm_set_duty_percent(CLAW_M1B_PIN, wrapClawB, 0);

    gpio_set_function(CLAW_M1A_PIN, GPIO_FUNC_PWM);
    pwm_set_duty_percent(CLAW_M1A_PIN, wrapClawA, 80.0f);
}

static void claw_off(void) {
    gpio_set_function(CLAW_M1A_PIN, GPIO_FUNC_SIO);
    gpio_set_function(CLAW_M1B_PIN, GPIO_FUNC_SIO);
    pwm_set_duty_percent(CLAW_M1A_PIN, wrapClawA, 0);
    pwm_set_duty_percent(CLAW_M1B_PIN, wrapClawB, 0);
#if CLAW_BRAKE_ON_STOP
    gpio_put(CLAW_M1A_PIN, 1);
    gpio_put(CLAW_M1B_PIN, 1);
#else
    gpio_put(CLAW_M1A_PIN, 0);
    gpio_put(CLAW_M1B_PIN, 0);
#endif
}

// ================= PUBLIC API IMPLEMENTATION =========
motor_error_t motor_init(void) {
    // Initialize wheel direction pins
    gpio_init(F_IN1_PIN); gpio_set_dir(F_IN1_PIN, GPIO_OUT); gpio_put(F_IN1_PIN, 0);
    gpio_init(F_IN2_PIN); gpio_set_dir(F_IN2_PIN, GPIO_OUT); gpio_put(F_IN2_PIN, 0);
    gpio_init(F_IN3_PIN); gpio_set_dir(F_IN3_PIN, GPIO_OUT); gpio_put(F_IN3_PIN, 0);
    gpio_init(F_IN4_PIN); gpio_set_dir(F_IN4_PIN, GPIO_OUT); gpio_put(F_IN4_PIN, 0);

    gpio_init(B_IN1_PIN); gpio_set_dir(B_IN1_PIN, GPIO_OUT); gpio_put(B_IN1_PIN, 0);
    gpio_init(B_IN2_PIN); gpio_set_dir(B_IN2_PIN, GPIO_OUT); gpio_put(B_IN2_PIN, 0);
    gpio_init(B_IN3_PIN); gpio_set_dir(B_IN3_PIN, GPIO_OUT); gpio_put(B_IN3_PIN, 0);
    gpio_init(B_IN4_PIN); gpio_set_dir(B_IN4_PIN, GPIO_OUT); gpio_put(B_IN4_PIN, 0);

    // Initialize arm pins
    gpio_init(ARM_M1A_PIN); gpio_set_dir(ARM_M1A_PIN, GPIO_OUT); gpio_put(ARM_M1A_PIN, 0);
    gpio_init(ARM_M1B_PIN); gpio_set_dir(ARM_M1B_PIN, GPIO_OUT); gpio_put(ARM_M1B_PIN, 0);
    
    // Initialize claw pins
    gpio_init(CLAW_M1A_PIN); gpio_set_dir(CLAW_M1A_PIN, GPIO_OUT); gpio_put(CLAW_M1A_PIN, 0);
    gpio_init(CLAW_M1B_PIN); gpio_set_dir(CLAW_M1B_PIN, GPIO_OUT); gpio_put(CLAW_M1B_PIN, 0);

    // Setup PWM for wheel enable pins
    pwm_setup_pin(F_ENA_PIN, PWM_FREQ_HZ, PWM_CLKDIV, &wrapFA);
    pwm_setup_pin(F_ENB_PIN, PWM_FREQ_HZ, PWM_CLKDIV, &wrapFB);
    pwm_setup_pin(B_ENA_PIN, PWM_FREQ_HZ, PWM_CLKDIV, &wrapBA);
    pwm_setup_pin(B_ENB_PIN, PWM_FREQ_HZ, PWM_CLKDIV, &wrapBB);

    // Arm PWM setup
    pwm_setup_pin(ARM_M1A_PIN, PWM_FREQ_HZ, PWM_CLKDIV, &wrapArmA);
    pwm_setup_pin(ARM_M1B_PIN, PWM_FREQ_HZ, PWM_CLKDIV, &wrapArmB);

    // Claw PWM setup
    pwm_setup_pin(CLAW_M1A_PIN, PWM_FREQ_HZ, PWM_CLKDIV, &wrapClawA);
    pwm_setup_pin(CLAW_M1B_PIN, PWM_FREQ_HZ, PWM_CLKDIV, &wrapClawB);

    motors_active = false;
    braking_active = false;
    last_error = MOTOR_OK;
    safety_interlock_init(&safety);
    
    printf("[MOTOR] Initialization complete (with reverse thrust braking)\n");
    return MOTOR_OK;
}

void motor_update_activity(void) {
    motors_active = true;
    last_command_time = to_ms_since_boot(get_absolute_time());
}

void motor_check_brake_timeout(void) {
    uint32_t irq_state = save_and_disable_interrupts();
    uint32_t now = to_ms_since_boot(get_absolute_time());
    bool completed = braking_active && (now - brake_start_time) > BRAKE_DURATION_MS;
    if (completed) {
        stop_all_motors_final();
    }
    restore_interrupts(irq_state);
    if (completed) printf("[MOTOR] Braking period completed - all motors stopped\n");
}

motor_error_t motor_set_direction(motor_direction_t direction, int speed_percent) {
    // Call individual function with 100% for all wheels (no adjustment)
    return motor_set_direction_individual(direction, speed_percent, 100, 100, 100, 100);
}

static motor_error_t set_direction_unlocked(motor_direction_t direction, int speed_percent,
                                             int speed_fl, int speed_fr, int speed_bl, int speed_br) {
    if (speed_percent < 0 || speed_percent > 100) {
        last_error = MOTOR_ERR_PIN;
        return MOTOR_ERR_PIN;
    }

    // If we're currently braking, cancel it for new movement commands
    if (braking_active) {
        stop_all_motors_final();
    }

    // Apply individual wheel adjustments to base speed
    int sFL = (speed_percent * speed_fl) / 100;
    int sFR = (speed_percent * speed_fr) / 100;
    int sBL = (speed_percent * speed_bl) / 100;
    int sBR = (speed_percent * speed_br) / 100;

    switch (direction) {
        case DIRECTION_FORWARD:
            frontA_set(sFL, wrapFA);
            frontB_set(sFR, wrapFB);
            backA_set(sBL, wrapBA);
            backB_set(sBR, wrapBB);
            break;

        case DIRECTION_BACKWARD:
            frontA_set(-sFL, wrapFA);
            frontB_set(-sFR, wrapFB);
            backA_set(-sBL, wrapBA);
            backB_set(-sBR, wrapBB);
            break;

        case DIRECTION_LEFT:
            frontA_set(-sFL, wrapFA);
            frontB_set(sFR, wrapFB);
            backA_set(sBL, wrapBA);
            backB_set(-sBR, wrapBB);
            break;

        case DIRECTION_RIGHT:
            frontA_set(sFL, wrapFA);
            frontB_set(-sFR, wrapFB);
            backA_set(-sBL, wrapBA);
            backB_set(sBR, wrapBB);
            break;

        case DIRECTION_ROTATE_LEFT:
            frontA_set(-sFL, wrapFA);
            frontB_set(sFR, wrapFB);
            backA_set(-sBL, wrapBA);
            backB_set(sBR, wrapBB);
            break;

        case DIRECTION_ROTATE_RIGHT:
            frontA_set(sFL, wrapFA);
            frontB_set(-sFR, wrapFB);
            backA_set(sBL, wrapBA);
            backB_set(-sBR, wrapBB);
            break;

        case DIRECTION_FRONT_LEFT:
            frontA_set(0, wrapFA);
            frontB_set(sFR, wrapFB);
            backA_set(sBL, wrapBA);
            backB_set(0, wrapBB);
            break;

        case DIRECTION_FRONT_RIGHT:
            frontA_set(sFL, wrapFA);
            frontB_set(0, wrapFB);
            backA_set(0, wrapBA);
            backB_set(sBR, wrapBB);
            break;

        case DIRECTION_BACK_LEFT:
            frontA_set(-sFL, wrapFA);
            frontB_set(0, wrapFB);
            backA_set(0, wrapBA);
            backB_set(-sBR, wrapBB);
            break;

        case DIRECTION_BACK_RIGHT:
            frontA_set(0, wrapFA);
            frontB_set(-sFR, wrapFB);
            backA_set(-sBL, wrapBA);
            backB_set(0, wrapBB);
            break;

        default:
            last_error = MOTOR_ERR_PIN;
            return MOTOR_ERR_PIN;
    }

    motor_update_activity();
    last_error = MOTOR_OK;
    return MOTOR_OK;
}

motor_error_t motor_set_direction_individual(motor_direction_t direction, int speed_percent,
                                             int speed_fl, int speed_fr, int speed_bl, int speed_br) {
    if (direction == DIRECTION_STOP) return motor_emergency_stop();
    uint32_t irq_state = save_and_disable_interrupts();
    motor_error_t result = MOTOR_ERR_SAFETY;
    if (!safety.latched) {
        result = set_direction_unlocked(direction, speed_percent,
                                        speed_fl, speed_fr, speed_bl, speed_br);
    }
    last_error = result;
    restore_interrupts(irq_state);
    return result;
}

static motor_error_t set_arm_unlocked(arm_direction_t direction) {
    switch (direction) {
        case ARM_UP:
            arm_up_on();
            break;
        case ARM_DOWN:
            arm_down_on();
            break;
        case ARM_STOP:
            arm_off();
            motors_active = false;
            return MOTOR_OK;
        default:
            last_error = MOTOR_ERR_PIN;
            return MOTOR_ERR_PIN;
    }

    motor_update_activity();
    last_error = MOTOR_OK;
    return MOTOR_OK;
}

motor_error_t arm_set_direction(arm_direction_t direction) {
    uint32_t irq_state = save_and_disable_interrupts();
    motor_error_t result = MOTOR_ERR_SAFETY;
    if (!safety.latched || direction == ARM_STOP) result = set_arm_unlocked(direction);
    last_error = result;
    restore_interrupts(irq_state);
    return result;
}

static motor_error_t set_claw_unlocked(claw_direction_t direction) {
    switch (direction) {
        case CLAW_OPEN:
            claw_open_on();
            break;
        case CLAW_CLOSE:
            claw_close_on();
            break;
        case CLAW_STOP:
            claw_off();
            motors_active = false;
            return MOTOR_OK;
        default:
            last_error = MOTOR_ERR_PIN;
            return MOTOR_ERR_PIN;
    }

    motor_update_activity();
    last_error = MOTOR_OK;
    return MOTOR_OK;
}

motor_error_t claw_set_direction(claw_direction_t direction) {
    uint32_t irq_state = save_and_disable_interrupts();
    motor_error_t result = MOTOR_ERR_SAFETY;
    if (!safety.latched || direction == CLAW_STOP) result = set_claw_unlocked(direction);
    last_error = result;
    restore_interrupts(irq_state);
    return result;
}

void motor_safety_receive(bool stop_requested, uint32_t receive_ms) {
    uint32_t irq_state = save_and_disable_interrupts();
    safety_interlock_receive(&safety, stop_requested, receive_ms);
    if (safety.latched) stop_all_motors_final();
    restore_interrupts(irq_state);
}

void motor_safety_poll(void) {
    uint32_t irq_state = save_and_disable_interrupts();
    safety_interlock_poll(&safety, to_ms_since_boot(get_absolute_time()));
    if (safety.latched) stop_all_motors_final();
    restore_interrupts(irq_state);
}

bool motor_safety_is_latched(void) {
    uint32_t irq_state = save_and_disable_interrupts();
    bool latched = safety.latched;
    restore_interrupts(irq_state);
    return latched;
}

bool motor_safety_reset(void) {
    uint32_t irq_state = save_and_disable_interrupts();
    bool ready = safety_interlock_reset(&safety, to_ms_since_boot(get_absolute_time()));
    // Reset itself cannot energize outputs or resume a previous command.
    stop_all_motors_final();
    restore_interrupts(irq_state);
    return ready;
}

motor_error_t motor_emergency_stop(void) {
    uint32_t irq_state = save_and_disable_interrupts();
    bool started_braking = false;
    if (safety.latched) {
        stop_all_motors_final();
    }
    // Instead of immediately stopping, apply reverse thrust braking
    else if (!braking_active) {
        apply_emergency_brake();
        started_braking = true;
    }
    last_error = MOTOR_OK;
    restore_interrupts(irq_state);
    if (started_braking) printf("[MOTOR] Emergency brake applied: reverse thrust braking\n");
    return MOTOR_OK;
}

bool motor_is_active(void) {
    return motors_active || braking_active;
}

void motor_check_timeout(void) {
    if (!motors_active && !braking_active) return;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - last_command_time) > TIMEOUT_MS) {
        motor_emergency_stop();
        printf("[MOTOR] Timeout - emergency braking activated\n");
    }
}

motor_error_t motor_get_last_error(void) {
    return last_error;
}
