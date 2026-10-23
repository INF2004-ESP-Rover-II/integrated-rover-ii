#include "safety_interlock.h"

void safety_interlock_init(safety_interlock_t *state) {
    *state = (safety_interlock_t){.latched = true, .stop_requested = true};
}

void safety_interlock_receive(safety_interlock_t *state, bool stop_requested,
                              uint32_t now_ms) {
    state->have_packet = true;
    state->last_packet_ms = now_ms;
    state->stop_requested = stop_requested;
    if (stop_requested) state->latched = true;
    // A later healthy frame must never release an earlier stop automatically.
}

void safety_interlock_poll(safety_interlock_t *state, uint32_t now_ms) {
    if (!state->have_packet || state->stop_requested ||
        (uint32_t)(now_ms - state->last_packet_ms) > SAFETY_PACKET_TIMEOUT_MS) {
        state->latched = true;
    }
}

bool safety_interlock_reset(safety_interlock_t *state, uint32_t now_ms) {
    safety_interlock_poll(state, now_ms);
    if (!state->have_packet || state->stop_requested ||
        (uint32_t)(now_ms - state->last_packet_ms) > SAFETY_PACKET_TIMEOUT_MS) {
        return false;
    }
    state->latched = false;
    return true;
}
