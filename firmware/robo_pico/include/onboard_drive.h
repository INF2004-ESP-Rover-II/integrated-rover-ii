#ifndef ROVER_ONBOARD_DRIVE_H
#define ROVER_ONBOARD_DRIVE_H
#include "target_selector.h"

#define ONBOARD_DRIVE_SENSOR_MAX_AGE_MS 600u
#define ONBOARD_DRIVE_IMAGE_MAX_AGE_MS 750u
#define ONBOARD_DRIVE_MAX_BEARING_CDEG 500
#define ONBOARD_DRIVE_NEAR_RANGE_MM 300u
#define ONBOARD_DRIVE_MAX_MS 3000u
#define ONBOARD_DRIVE_LEASE_MS 250u
#define ONBOARD_DRIVE_REFRESH_MS 100u
#define ONBOARD_DRIVE_ARRIVAL_STOP_MM 50
#define ONBOARD_DRIVE_VERIFY_ERROR_MM 100
#define ONBOARD_DRIVE_VERIFY_FRAMES 3u
#define ONBOARD_DRIVE_SETTLE_MS 300u
#define ONBOARD_DRIVE_VERIFY_TIMEOUT_MS 3500u
#define ONBOARD_DRIVE_REFERENCE_MAX_STEP_MM 250
#define ONBOARD_DRIVE_REFERENCE_BACKWARDS_MM 60
#define ONBOARD_DRIVE_CAMERA_RECOVERY_MS 1500u
#define ONBOARD_DRIVE_RECOVERY_FRAMES 2u
#define ONBOARD_DRIVE_MAX_WALL_MS 10000u
// One timed attempt per leg; this is not calibrated reverse odometry.
#define ONBOARD_DRIVE_REFERENCE_MISSING_MS 2000u
#define ONBOARD_DRIVE_BACKUP_MS 250u
#define ONBOARD_DRIVE_BACKUP_PCT 100
#define ONBOARD_DRIVE_REACQUIRE_MS 2500u
#define ONBOARD_DRIVE_RECOVERY_WALL_MS 6000u
#define ONBOARD_DRIVE_RECOVERY_BACKWARDS_MM 250

typedef struct {
    bool stopped, safety_latched, link_ok, sensor_valid, imu_ok, front_fault, front_urgent, camera_fresh;
    uint16_t front_mm;
    uint32_t sensor_received_ms;
    float heading_deg; // Sensor Pico raw startup-relative heading; unaffected by manual zero.
    bool camera_present;
    uint8_t camera_status;
    uint32_t camera_packet_age_ms, camera_image_age_ms;
    bool reference_absent; // Explicit absence from a fresh OK/no-marker camera packet, not a stale stream.
} onboard_drive_inputs_t;
typedef struct {
    bool active;
    uint32_t leg_id, started_ms, last_refresh_ms;
    const char *result;
    bool frame_valid;
    float north_heading_deg;
    uint32_t map_revision, braked_ms, last_sample_generation, last_verify_generation;
    uint8_t reference_id, verification_frames;
    bool reference_model_valid, estimate_valid, using_reference;
    int32_t reference_anchor_mm, target_anchor_mm, reference_origin_mm, last_reference_mm;
    int32_t remaining_mm, reference_progress_mm;
    uint32_t hold_started_ms, recovery_generation, driving_ms, last_tick_ms;
    uint8_t recovery_frames;
    const char *input_reason;
    bool camera_present;
    uint8_t camera_status;
    uint32_t camera_packet_age_ms, camera_image_age_ms, accepted_image_age_ms;
    bool reference_recovery, backup_used, reference_missing;
    uint32_t reference_wait_ms, reference_missing_ms, backup_started_ms, backup_stopped_ms;
} onboard_drive_t;

void onboard_drive_init(onboard_drive_t *drive);
// One bounded heading-held leg with empirical reference-range arrival.
const char *onboard_drive_start(onboard_drive_t *drive, mission_t *mission,
    rover_map_t *map, const target_selector_t *selection, const onboard_drive_inputs_t *inputs, uint32_t now);
void onboard_drive_tick(onboard_drive_t *drive, mission_t *mission,
    rover_map_t *map, const target_selector_t *selection, const onboard_drive_inputs_t *inputs, uint32_t now);
void onboard_drive_stop(onboard_drive_t *drive, mission_t *mission, mission_reason_t reason,
    const char *result, uint32_t now);
size_t onboard_drive_format(const onboard_drive_t *drive, const mission_t *mission, char *out, size_t capacity);
#endif
