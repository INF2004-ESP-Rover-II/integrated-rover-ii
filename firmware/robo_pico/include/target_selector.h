/**
 * @file target_selector.h
 * @brief Target selector firmware module.
 */
#ifndef ROVER_TARGET_SELECTOR_H
#define ROVER_TARGET_SELECTOR_H

#include "camera_uart.h"
#include "mission_control.h"
#include "rover_map.h"

// Stationary optical-candidate thresholds, NOT driving/arrival tolerances.
#define TARGET_CAMERA_MAX_AGE_MS 1500u
#define TARGET_UART_MAX_AGE_MS 300u
#define TARGET_MAX_BEARING_CDEG 1000
// Only an already locked target gets the wider tracking window.
#define TARGET_TRACKING_MAX_BEARING_CDEG 3000
// Wider visibility for an already locked recovery landmark; driving still
// requires alignment within TARGET_MAX_BEARING_CDEG.
#define TARGET_RECOVERY_MAX_BEARING_CDEG 4500
#define TARGET_REFERENCE_MAX_DELTA_CDEG 500
#define TARGET_REFERENCE_MIN_GAP_MM 100u
#define TARGET_CONFIRM_FRAMES 3u

typedef enum
{
    TARGET_NONE,
    TARGET_WAITING,
    TARGET_LOCKED,
    TARGET_CANCELLED
} target_selection_state_t;
typedef struct
{
    bool            enabled;
    uint8_t         expected_id;
    int16_t         x;
    int16_t         y;
    leg_direction_t direction;
} target_route_t;
typedef struct
{
    target_route_t           route;
    target_selection_state_t state;
    const char *             detail;
    uint32_t                 leg_id;
    uint32_t                 started_ms;
    uint32_t                 map_revision;
    uint32_t                 last_generation;
    uint32_t                 last_image_received_ms;
    uint32_t                 sample_generation;
    uint32_t                 sample_received_ms;
    uint16_t                 sample_age_ms;
    uint8_t                  last_seq;
    uint8_t                  marker;
    uint8_t                  confirmations;
    uint8_t                  reference_marker;
    uint8_t                  reference_candidate;
    uint8_t                  reference_confirmations;
    bool                     have_seq;
    bool                     available;
    bool                     reference_available;
    bool                     allow_late_reference;
    uint16_t                 range_mm;
    uint16_t                 reference_range_mm;
    int16_t                  bearing_cdeg;
    int16_t                  reference_bearing_cdeg;
} target_selector_t;


void target_selector_init (target_selector_t * p_selector);


void target_selector_set_route (target_selector_t * p_selector,
                                target_route_t      route);

// Automatically arms at a stopped confirmed node. No dashboard selection,
// hardware calls, map writes, dynamic allocation, waits or arrival commits.
// Unknown target/reference associations are provisional optical candidates.

void target_selector_tick (target_selector_t * p_selector,
                           mission_t * p_mission, const rover_map_t * p_map,
                           const camera_observation_t * p_latest,
                           bool b_stopped_and_safe, uint32_t now_ms);


size_t target_selector_format (const target_selector_t * p_selector,
                               char * p_out, size_t capacity);


#endif

/* End of file. */
