/**
 * @file onboard_drive.h
 * @brief Onboard drive firmware module.
 */
#ifndef ROVER_ONBOARD_DRIVE_H
#define ROVER_ONBOARD_DRIVE_H
#include "target_selector.h"

#define ONBOARD_DRIVE_SENSOR_MAX_AGE_MS 600u
#define ONBOARD_DRIVE_IMAGE_MAX_AGE_MS 750u
#define ONBOARD_DRIVE_MAX_BEARING_CDEG TARGET_MAX_BEARING_CDEG
#define ONBOARD_DRIVE_NEAR_RANGE_MM 300u
#define ONBOARD_DRIVE_MAX_MS 3000u
#define ONBOARD_DRIVE_LEASE_MS 250u
#define ONBOARD_DRIVE_REFRESH_MS 100u
#define ONBOARD_DRIVE_ARRIVAL_STOP_MM 50
#define ONBOARD_DRIVE_VERIFY_ERROR_MM 100
#define ONBOARD_DRIVE_VERIFY_FRAMES 3u
#define ONBOARD_DRIVE_SETTLE_MS 300u
#define ONBOARD_DRIVE_VERIFY_TIMEOUT_MS 3500u
#define ONBOARD_DRIVE_CAMERA_RECOVERY_MS 1500u
#define ONBOARD_DRIVE_RECOVERY_FRAMES 2u
#define ONBOARD_DRIVE_MAX_WALL_MS 10000u
// Renewable reverse-search lease, not a total reverse duration.
#define ONBOARD_DRIVE_REFERENCE_MISSING_MS 2000u
#define ONBOARD_DRIVE_BACKUP_MS ONBOARD_DRIVE_LEASE_MS
#define ONBOARD_DRIVE_BACKUP_PCT 30
#define ONBOARD_DRIVE_REACQUIRE_MS 2500u
#define ONBOARD_DRIVE_RECOVERY_WALL_MS 6000u
#define ONBOARD_DRIVE_REFERENCE_ARRIVAL_MM 600
#define ONBOARD_DRIVE_SEARCH_CLEAR_MM 600u
#define ONBOARD_DRIVE_SEARCH_PCT 15
#define ONBOARD_DRIVE_SEARCH_MAX_MS 10000u
#define ONBOARD_DRIVE_SEARCH_WALL_MS 30000u
#define ONBOARD_DRIVE_OBSTACLE_CONFIRM_FRAMES 2u
#define ONBOARD_DRIVE_ALIGNMENT_STEP_CDEG 1000
#define ONBOARD_DRIVE_ALIGNMENT_CONFIRM_FRAMES 3u
#define ONBOARD_DRIVE_CORRECTION_MS 150u
#define ONBOARD_DRIVE_CORRECTION_PCT 15

typedef struct
{
    bool     stopped;
    bool     safety_latched;
    bool     link_ok;
    bool     sensor_valid;
    bool     imu_ok;
    bool     front_fault;
    bool     front_urgent;
    bool     camera_fresh;
    uint16_t front_mm;
    uint32_t sensor_received_ms;
    float heading_deg; // Sensor Pico raw startup-relative heading; unaffected
                       // by manual zero.
    float heading_offset_deg; // Convert a raw alignment heading to the motor's
                              // current zero.
    bool     camera_present;
    uint8_t  camera_status;
    uint32_t camera_packet_age_ms;
    uint32_t camera_image_age_ms;
    bool reference_absent; // Explicit absence from a fresh OK/no-marker camera
                           // packet, not a stale stream.
    uint32_t        camera_generation;
    uint8_t         camera_seq;
    uint8_t         recovery_marker_count;
    camera_marker_t recovery_markers[VISION_MAX_MARKERS];
} onboard_drive_inputs_t;
typedef struct
{
    bool         active;
    bool         b_departure_pending;
    uint32_t     leg_id;
    uint32_t     started_ms;
    uint32_t     last_refresh_ms;
    const char * result;
    bool         frame_valid;
    float        north_heading_deg;
    uint32_t     map_revision;
    uint32_t     braked_ms;
    uint32_t     last_sample_generation;
    uint32_t     last_verify_generation;
    uint8_t      reference_id;
    uint8_t      verification_frames;
    bool         reference_model_valid;
    bool         estimate_valid;
    bool         using_reference;
    int32_t      reference_anchor_mm;
    int32_t      target_anchor_mm;
    int32_t      reference_origin_mm;
    int32_t      remaining_mm;
    int32_t      reference_progress_mm;
    uint32_t     hold_started_ms;
    uint32_t     recovery_generation;
    uint32_t     driving_ms;
    uint32_t     last_tick_ms;
    uint8_t      recovery_frames;
    const char * input_reason;
    bool         camera_present;
    uint8_t      camera_status;
    uint32_t     camera_packet_age_ms;
    uint32_t     camera_image_age_ms;
    uint32_t     accepted_image_age_ms;
    bool         reference_recovery;
    bool         backup_used;
    bool         reference_missing;
    uint32_t     reference_wait_ms;
    uint32_t     reference_missing_ms;
    uint32_t     backup_started_ms;
    uint32_t     backup_stopped_ms;
    bool         search_active;
    bool         search_used;
    bool         front_arrival_pending;
    bool         using_obstacle;
    uint16_t     last_target_range_mm;
    uint32_t     search_driving_ms;
    uint32_t     search_hold_ms;
    uint32_t     search_generation;
    uint32_t     last_obstacle_ms;
    uint8_t      obstacle_frames;
    bool         aligning;
    bool         b_alignment_reference;
    bool         alignment_rotating;
    uint32_t     alignment_stopped_ms;
    uint32_t     alignment_generation;
    uint32_t     alignment_wait_ms;
    uint8_t      alignment_frames;
    bool         camera_waiting;
    leg_phase_t  camera_resume_phase;
    uint32_t     camera_wait_started_ms;
    uint32_t     camera_wait_pose_ms;
    uint32_t     camera_wait_generation;
    uint32_t     camera_waited_ms; // Stopped camera outages, not driving time.
    uint8_t      camera_wait_frames;
    uint16_t     reference_arrival_mm;
    uint8_t      recovery_marker_id;
    uint8_t      mapped_recovery_frames;
    uint8_t      mapped_recovery_seq;
    bool         recovery_marker_mapped;
    bool         mapped_recovery_have_seq;
    uint32_t     mapped_recovery_generation;
    bool         input_waiting;
    bool         correcting;
    leg_phase_t  input_resume_phase;
    uint32_t     input_wait_started_ms;
    uint32_t     correction_started_ms;
} onboard_drive_t;

void onboard_drive_init (onboard_drive_t * p_drive);

// Bounded target leg: reference at the configured 600 mm arrival range, or
// confirmed front obstacle arrival. Empty camera views can enter a slow search.
// A valid off-centre locked target aligns in place before departure.

const char * onboard_drive_start (onboard_drive_t * p_drive,
                                  mission_t * p_mission, rover_map_t * p_map,
                                  target_selector_t *            p_selection,
                                  const onboard_drive_inputs_t * p_inputs,
                                  uint32_t                       now);

void onboard_drive_tick (onboard_drive_t * p_drive, mission_t * p_mission,
                         rover_map_t * p_map, target_selector_t * p_selection,
                         const onboard_drive_inputs_t * p_inputs, uint32_t now);

// Called in the main loop after the independent front motor guard has braked.
// Manual E-STOP/fault handling is separate and never commits an arrival.

bool onboard_drive_note_front_stop (onboard_drive_t * p_drive,
                                    const mission_t * p_mission);

void onboard_drive_stop (onboard_drive_t * p_drive, mission_t * p_mission,
                         mission_reason_t reason, const char * p_result,
                         uint32_t now);

size_t onboard_drive_format (const onboard_drive_t * p_drive,
                             const mission_t * p_mission, char * p_out,
                             size_t capacity);

#endif

/* End of file. */
