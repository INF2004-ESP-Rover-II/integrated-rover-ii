/**
 * @file ultrasonic.c
 * @brief Ultrasonic firmware module.
 */
#include "ultrasonic.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"

// Local implementation constants (don’t leak into the API)
#define TRIGGER_PULSE_US 10
#define ECHO_TIMEOUT_US 30000 // 30 ms ~ >4 m round-trip
#define SPEED_OF_SOUND_CM_S 34300.0f
#define US_PER_S 1000000.0f

void
ultrasonic_init (uint32_t trig_pin, uint32_t echo_pin)
{
    gpio_init(trig_pin);
    gpio_init(echo_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_set_dir(echo_pin, GPIO_IN);
    gpio_put(trig_pin, 0); // ensure trigger starts low
}

ultrasonic_status_t
ultrasonic_get_pulse (uint32_t trig_pin, uint32_t echo_pin,
                      uint32_t * p_pulse_us)
{
    // Send 10 µs trigger
    gpio_put(trig_pin, 1);
    sleep_us(TRIGGER_PULSE_US);
    gpio_put(trig_pin, 0);

    // Wait for echo to go HIGH (start), with timeout
    absolute_time_t t0 = get_absolute_time();
    while (gpio_get(echo_pin) == 0)
    {
        if (absolute_time_diff_us(t0, get_absolute_time()) > ECHO_TIMEOUT_US)
        {
            return ULTRASONIC_NO_ECHO_START;
        }
        tight_loop_contents();
    }

    // Measure HIGH pulse width, with timeout
    absolute_time_t start = get_absolute_time();
    while (gpio_get(echo_pin) == 1)
    {
        if (absolute_time_diff_us(start, get_absolute_time()) > ECHO_TIMEOUT_US)
        {
            return ULTRASONIC_ECHO_STUCK_HIGH;
        }
        tight_loop_contents();
    }
    absolute_time_t end = get_absolute_time();

    *p_pulse_us = (uint32_t) absolute_time_diff_us(start, end);
    return ULTRASONIC_OK;
}

ultrasonic_status_t
ultrasonic_get_cm (uint32_t trig_pin, uint32_t echo_pin, float * p_distance_cm)
{
    uint32_t pulse_us = 0;

    ultrasonic_status_t st =
        ultrasonic_get_pulse(trig_pin, echo_pin, &pulse_us);

    if (st != ULTRASONIC_OK)
    {
        return st;
    }

    // distance = (c * t) / 2, with t in seconds and c in cm/s
    *p_distance_cm = (SPEED_OF_SOUND_CM_S * (pulse_us / US_PER_S)) / 2.0f;
    // (equivalently: *distance_cm = pulse_us / 58.0f)
    return ULTRASONIC_OK;
}

/* End of file. */
