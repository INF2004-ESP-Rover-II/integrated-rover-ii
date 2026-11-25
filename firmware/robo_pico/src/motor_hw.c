/**
 * @file motor_hw.c
 * @brief Robo Pico motor hardware layer (Pico SDK).
 *
 * Wheel index -> L298N pins (unchanged wiring, Week 6 Section 2):
 *   0 front-left : ENA GP4  IN1 GP3  IN2 GP2     (front L298N, channel A)
 *   1 front-right: ENB GP5  IN3 GP16 IN4 GP17    (front L298N, channel B)
 *   2 back-left  : ENA GP27 IN1 GP28 IN2 GP7     (back  L298N, channel A)
 *   3 back-right : ENB GP26 IN3 GP0  IN4 GP1     (back  L298N, channel B)
 *
 * PWM slices on the RP2040: GPn uses slice (n / 2) % 8, channel n % 2.
 *   GP4/GP5   -> slice 2 A/B
 *   GP26/GP27 -> slice 5 A/B
 *   GP10/GP11 -> slice 5 A/B   (arm)   <-- SAME slice AND SAME channels
 *   GP8/GP9   -> slice 4 A/B   (claw)
 * Two pins on the same slice channel always output the SAME duty. If the arm
 * pins were switched to PWM, driving the arm would also change the back
 * wheels' speed. So the arm and claw are driven as plain on/off GPIO here and
 * slice 5 is owned by the wheels at one fixed frequency (prof feedback).
 *
 * L298N truth table: EN high + IN1 != IN2 -> drive; EN high + IN1 == IN2 ->
 * fast stop (brake); EN low -> coast. Active brake = IN1 = IN2 = 0, EN on.
 * No drive current flows while braking; the motor terminals are shorted.
 *
 * GP0/GP1 are motor direction pins, so stdio must stay on USB
 * (CMake: pico_enable_stdio_uart 0). GP28 as an output: check the Robo Pico
 * VBAT jumper is open (prof feedback) before the first run.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_hw.h"
#include "motor_pins.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"

#define DIR_UART uart0
#define DIR_UART_TX_PIN 12u
#define DIR_UART_BAUD 115200u /* must match the Sensor Pico RX */

typedef struct
{
    uint8_t en;
    uint8_t in_a;
    uint8_t in_b;
} wheel_pins_t;

static const wheel_pins_t g_k_wheel[MC_WHEELS] = {
    { F_ENA_PIN, F_IN1_PIN, F_IN2_PIN }, /* front-left  */
    { F_ENB_PIN, F_IN3_PIN, F_IN4_PIN }, /* front-right */
    { B_ENA_PIN, B_IN1_PIN, B_IN2_PIN }, /* back-left   */
    { B_ENB_PIN, B_IN3_PIN, B_IN4_PIN }, /* back-right  */
};

static uint16_t g_s_wrap = 0;

static void
out_low (uint pin)
{
    gpio_init(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

void
motor_hw_safe_boot (void)
{
    /* claw + arm first, they are the ones that can move something on power-up
     */
    out_low(CLAW_M1A_PIN);
    out_low(CLAW_M1B_PIN);
    out_low(ARM_M1A_PIN);
    out_low(ARM_M1B_PIN);
    for (int index = 0; index < MC_WHEELS; index++)
    {
        out_low(g_k_wheel[index].en); /* coast until PWM is set up */
        out_low(g_k_wheel[index].in_a);
        out_low(g_k_wheel[index].in_b);
    }
}

static uint32_t g_s_freq_hz = PWM_FREQ_HZ;

static void
pwm_setup (void)
{
    /* clkdiv keeps wrap <= 65535 at low frequencies (e.g. 1 kHz -> div 2) */
    uint32_t sys_hz = clock_get_hz(clk_sys);
    uint32_t div    = 1u;
    while (sys_hz / (div * g_s_freq_hz) > 65536u && div < 255u)
    {
        div++;
    }
    g_s_wrap = (uint16_t) (sys_hz / (div * g_s_freq_hz) - 1u);

    for (int index = 0; index < MC_WHEELS; index++)
    {
        uint       slice = pwm_gpio_to_slice_num(g_k_wheel[index].en);
        pwm_config cfg   = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&cfg, div);
        pwm_config_set_wrap(&cfg, g_s_wrap);
        pwm_init(slice, &cfg, false);
        pwm_set_gpio_level(g_k_wheel[index].en, 0);
        gpio_set_function(g_k_wheel[index].en, GPIO_FUNC_PWM);
        pwm_set_enabled(slice, true);
    }
}

void
motor_hw_init (void)
{
    pwm_setup();
}

void
motor_hw_set_pwm_freq (uint32_t hz)
{
    if (hz < 100u)
    {
        hz = 100u;
    }
    if (hz > 20000u)
    {
        hz = 20000u;
    }
    g_s_freq_hz = hz;
    pwm_setup(); /* wheels stop (level 0) until the next update */
}

uint32_t
motor_hw_pwm_freq (void)
{
    return g_s_freq_hz;
}

static void
wheel_set (int index, int duty, bool b_brake)
{
    const wheel_pins_t * p_pointer = &g_k_wheel[index];
    if (b_brake)
    {
        gpio_put(p_pointer->in_a, 0);
        gpio_put(p_pointer->in_b, 0);
        pwm_set_gpio_level(p_pointer->en,
                           g_s_wrap); /* EN fully on -> short brake */
        return;
    }
    if (duty == 0)
    {
        pwm_set_gpio_level(p_pointer->en, 0); /* coast */
        return;
    }
    uint32_t mag = (uint32_t) (duty < 0 ? -duty : duty);
    if (mag > 100u)
    {
        mag = 100u;
    }
    /* set direction first, then speed */
    gpio_put(p_pointer->in_a, duty > 0);
    gpio_put(p_pointer->in_b, duty < 0);
    pwm_set_gpio_level(p_pointer->en,
                       (uint16_t) ((uint32_t) g_s_wrap * mag / 100u));
}

void
motor_hw_apply (const mc_out_t * p_out)
{
    if (!p_out->outputs_valid)
    {
        return;
    }
    for (int index = 0; index < MC_WHEELS; index++)
    {
        wheel_set(index, p_out->wheel[index],
                  p_out->brake && p_out->wheel[index] == 0);
    }
}

void
motor_hw_all_off (void)
{
    for (int index = 0; index < MC_WHEELS; index++)
    {
        pwm_set_gpio_level(g_k_wheel[index].en, 0);
    }
}

void
motor_hw_dir_uart_init (void)
{
    /* uart_init is safe to call if Robo Comms already did (same baud). */
    if (!uart_is_enabled(DIR_UART))
    {
        uart_init(DIR_UART, DIR_UART_BAUD);
    }
    gpio_set_function(DIR_UART_TX_PIN, GPIO_FUNC_UART);
}

void
motor_hw_send_dir (uint8_t dir_byte)
{
    /* one byte; only the Motor Control task writes this TX line */
    if (uart_is_writable(DIR_UART))
    {
        uart_putc_raw(DIR_UART, (char) dir_byte);
    }
}

static void
pair_set (uint pin_fwd, uint pin_rev, mh_dir_t data)
{
    gpio_put(pin_fwd, data == MH_FWD);
    gpio_put(pin_rev, data == MH_REV);
}

void
motor_hw_claw (mh_dir_t data)
{
    pair_set(CLAW_M1A_PIN, CLAW_M1B_PIN, data);
}
void
motor_hw_arm (mh_dir_t data)
{
    pair_set(ARM_M1A_PIN, ARM_M1B_PIN, data);
}

/* End of file. */
