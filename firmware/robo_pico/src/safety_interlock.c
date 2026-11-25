/**
 * @file safety_interlock.c
 * @brief Safety interlock firmware module.
 */
#include "safety_interlock.h"

void
safety_interlock_init (safety_interlock_t * p_state)
{
    *p_state = (safety_interlock_t) { .latched = true, .stop_requested = true };
}

void
safety_interlock_receive (safety_interlock_t * p_state, bool b_stop_requested,
                          uint32_t now_ms)
{
    p_state->have_packet    = true;
    p_state->last_packet_ms = now_ms;
    p_state->stop_requested = b_stop_requested;
    if (b_stop_requested)
    {
        p_state->latched = true;
    }
    // A later healthy frame must never release an earlier stop automatically.
}

void
safety_interlock_poll (safety_interlock_t * p_state, uint32_t now_ms)
{
    if (!p_state->have_packet || p_state->stop_requested ||
        (uint32_t) (now_ms - p_state->last_packet_ms) >
            SAFETY_PACKET_TIMEOUT_MS)
    {
        p_state->latched = true;
    }
}

bool
safety_interlock_reset (safety_interlock_t * p_state, uint32_t now_ms)
{
    safety_interlock_poll(p_state, now_ms);
    if (!p_state->have_packet || p_state->stop_requested ||
        (uint32_t) (now_ms - p_state->last_packet_ms) >
            SAFETY_PACKET_TIMEOUT_MS)
    {
        return false;
    }
    p_state->latched = false;
    return true;
}

/* End of file. */
