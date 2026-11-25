/**
 * @file motor_lease.h
 * @brief Motor lease firmware module.
 */
#ifndef ROVER_MOTOR_LEASE_H
#define ROVER_MOTOR_LEASE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t refreshed_ms;
    uint16_t duration_ms;
} motor_lease_t;
static inline bool
motor_lease_expired (const motor_lease_t * p_lease, uint32_t now)
{
    return p_lease->duration_ms &&
           (uint32_t) (now - p_lease->refreshed_ms) > p_lease->duration_ms;
}
#endif

/* End of file. */
