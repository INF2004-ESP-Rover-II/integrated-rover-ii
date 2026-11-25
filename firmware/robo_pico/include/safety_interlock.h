/**
 * @file safety_interlock.h
 * @brief Safety interlock firmware module.
 */
#ifndef SAFETY_INTERLOCK_H
#define SAFETY_INTERLOCK_H

#include <stdbool.h>
#include <stdint.h>

// Provisional link watchdog; independent of the 1 Hz MQTT telemetry rate.
#define SAFETY_PACKET_TIMEOUT_MS 500u

typedef struct
{
    bool     latched;
    bool     have_packet;
    bool     stop_requested;
    uint32_t last_packet_ms;
} safety_interlock_t;

// Callers must serialize access when used from interrupt and main contexts.

void safety_interlock_init (safety_interlock_t * p_state);


void safety_interlock_receive (safety_interlock_t * p_state,
                               bool b_stop_requested, uint32_t now_ms);


void safety_interlock_poll (safety_interlock_t * p_state, uint32_t now_ms);


bool safety_interlock_reset (safety_interlock_t * p_state, uint32_t now_ms);


#endif

/* End of file. */
