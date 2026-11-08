#ifndef ROVER_MOTOR_LEASE_H
#define ROVER_MOTOR_LEASE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint32_t refreshed_ms; uint16_t duration_ms; } motor_lease_t;
static inline bool motor_lease_expired(const motor_lease_t *lease, uint32_t now)
{
    return lease->duration_ms && (uint32_t)(now - lease->refreshed_ms) > lease->duration_ms;
}
#endif
