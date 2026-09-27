#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Result codes for non-blocking / timeout-aware reads
typedef enum {
    ULTRASONIC_OK = 0,
    ULTRASONIC_NO_ECHO_START,   // Echo never went HIGH (no return from sensor or wiring issue)
    ULTRASONIC_ECHO_STUCK_HIGH, // Echo never went LOW (bad pulse or out-of-range reflection)
} ultrasonic_status_t;

/**
 * @brief Initialise trigger and echo pins.
 * @param trigPin GPIO number for trigger (output).
 * @param echoPin GPIO number for echo (input).
 */
void ultrasonic_init(uint32_t trigPin, uint32_t echoPin);

/**
 * @brief Emit trigger and measure echo HIGH pulse width.
 * @param trigPin Trigger pin.
 * @param echoPin Echo pin.
 * @param pulse_us Out: echo HIGH duration in microseconds (valid if return == ULTRASONIC_OK).
 * @return ultrasonic_status_t
 */
ultrasonic_status_t ultrasonic_get_pulse(uint32_t trigPin, uint32_t echoPin, uint32_t *pulse_us);

/**
 * @brief Get distance (cm). Applies distance = c * t / 2 with c ≈ 34300 cm/s.
 * @param trigPin Trigger pin.
 * @param echoPin Echo pin.
 * @param distance_cm Out: distance in centimeters (valid if return == ULTRASONIC_OK).
 * @return ultrasonic_status_t
 */
ultrasonic_status_t ultrasonic_get_cm(uint32_t trigPin, uint32_t echoPin, float *distance_cm);

/* -------- Optional: lightweight compatibility shim --------
   If you have legacy callers that expect a float and no status,
   you can keep them working like this (define ULTRASONIC_ENABLE_LEGACY before including the header):
*/
#ifdef ULTRASONIC_ENABLE_LEGACY
#include <math.h>
static inline float ultrasonic_get_cm_legacy(uint32_t trigPin, uint32_t echoPin) {
    float d = NAN;
    return (ultrasonic_get_cm(trigPin, echoPin, &d) == ULTRASONIC_OK) ? d : NAN;
}
#endif

#ifdef __cplusplus
}
#endif
#endif // ULTRASONIC_H
