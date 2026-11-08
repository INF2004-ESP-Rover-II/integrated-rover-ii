#ifndef ROVER_TARGET_SELECTOR_H
#define ROVER_TARGET_SELECTOR_H

#include "camera_uart.h"
#include "mission_control.h"
#include "rover_map.h"

// Stationary optical-candidate thresholds, NOT driving/arrival tolerances.
#define TARGET_CAMERA_MAX_AGE_MS 1500u
#define TARGET_UART_MAX_AGE_MS 300u
#define TARGET_MAX_BEARING_CDEG 1000
#define TARGET_REFERENCE_MAX_DELTA_CDEG 500
#define TARGET_REFERENCE_MIN_GAP_MM 100u
#define TARGET_CONFIRM_FRAMES 3u

typedef enum { TARGET_NONE, TARGET_WAITING, TARGET_LOCKED, TARGET_CANCELLED } target_selection_state_t;
typedef struct {
    bool enabled;
    uint8_t expected_id;
    int16_t x, y;
    leg_direction_t direction;
} target_route_t;
typedef struct {
    target_route_t route;
    target_selection_state_t state;
    const char *detail;
    uint32_t leg_id, started_ms, map_revision, last_generation, last_image_received_ms;
    uint32_t sample_generation, sample_received_ms;
    uint16_t sample_age_ms;
    uint8_t last_seq, marker, confirmations, reference_marker, reference_candidate, reference_confirmations;
    bool have_seq, available, reference_available;
    uint16_t range_mm, reference_range_mm;
    int16_t bearing_cdeg, reference_bearing_cdeg;
} target_selector_t;

void target_selector_init(target_selector_t *selector);
void target_selector_set_route(target_selector_t *selector, target_route_t route);
// Automatically arms at a stopped confirmed node. No dashboard selection,
// hardware calls, map writes, dynamic allocation, waits or arrival commits.
// Unknown target/reference associations are provisional optical candidates.
void target_selector_tick(target_selector_t *selector, mission_t *mission,
    const rover_map_t *map, const camera_observation_t *latest,
    bool stopped_and_safe, uint32_t now_ms);
size_t target_selector_format(const target_selector_t *selector, char *out, size_t capacity);

#endif
