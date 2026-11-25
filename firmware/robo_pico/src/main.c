/**
 * @file main.c
 * @brief Main firmware module.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/rand.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "config.h"
#include "motor_driver.h"
#include "../../common/obstacle_threshold.h"
#include "wifi_mqtt.h"
#include "mqtt_publish_policy.h"
#include "uart_protocol.h"
#include "camera_uart.h"
#include "camera_telemetry.h"
#include "camera_map.h"
#include "mission_control.h"
#include "rover_map.h"
#include "target_selector.h"
#include "onboard_drive.h"
#include "onboard_dfs.h"
#include "lwip/timeouts.h"
#include "lwip/apps/mqtt.h"

/* Fixed dimensions and iteration limits used by this module. */
#define TOF_SCAN_COUNT 3
#define MAP_REQUEST_HEX_LENGTH 32

// UART configuration
#define SENSOR_UART_ID uart0
#define SENSOR_UART_BAUD 115200
#define SENSOR_UART_RX_PIN 13
#define SENSOR_UART_TX_PIN (-1)
#define SENSOR_TELEMETRY_INTERVAL_MS                                           \
    500u // 10 Hz feedback for laptop navigation.

// Buzzer configuration (piezo buzzer - PWM)
#define BUZZER_PIN 22
#define BUZZER_FREQ_HZ 2500 // 2.5kHz tone

// Latest sensor data
static sensor_packet_t g_latest_sensors;
static bool            g_b_sensor_data_valid = false;
static uint32_t        g_last_sensor_packet_ms =
    0; // Local receive time, not Sensor Pico time.

// Autonomous mode state machine
typedef enum
{
    AUTO_STATE_IDLE,
    AUTO_STATE_MOVING_FORWARD,
    AUTO_STATE_OBSTACLE_DETECTED,
    AUTO_STATE_SCANNING,
    AUTO_STATE_TURNING,
    AUTO_STATE_RESUMING
} auto_state_t;

static bool         g_b_autonomous_mode = false;
static auto_state_t g_autonomous_state  = AUTO_STATE_IDLE;
// Forward boost handling
static bool g_b_forward_boost_active = false; // whether boost is active
// Track latest control command time
static uint32_t g_last_command_time_ms = 0;

// Proximity buzzer state
static uint32_t g_last_beep_time = 0;
static bool     g_b_beep_state   = false;
static uint     g_buzzer_slice_num;

// Turn memory and dead-end detection
static bool g_b_safety_lock_reported = false;
static bool g_b_link_lost =
    true; // FR-08: true until the first laptop heartbeat
static motor_auto_status_t g_last_auto_reported = MOTOR_AUTO_IDLE;
static bool                g_b_heading_zero_pending;
static uint32_t            g_heading_zero_ready_ms;
static mission_t           g_mission;
static target_selector_t   g_target_selector;
static onboard_drive_t     g_onboard_drive;
static onboard_dfs_t       g_onboard_dfs;
static rover_map_t         g_rover_map;
static camera_map_t        g_camera_map;
static char                g_map_last_request[33];
static uint8_t             g_last_start_marker;
static bool                g_b_map_publish_requested;
_Static_assert(
    ROVER_MAP_JSON_CAPACITY + 32u < MQTT_OUTPUT_RINGBUF_SIZE,
    "Increase MQTT ring or add snapshot chunking for this map capacity");

static void
service_map_publish (void)
{
    static char     payload[ROVER_MAP_JSON_CAPACITY];
    static uint32_t last_attempt_ms;
    static uint32_t last_sent_ms;
    static uint32_t sent_revision;
    static bool     b_sent;
    uint32_t        now = to_ms_since_boot(get_absolute_time());
    if (!mqtt_is_connected() ||
        (uint32_t) (now - last_attempt_ms) < MQTT_REPORT_RETRY_MS)
    {
        return;
    }
    if (b_sent && !g_b_map_publish_requested &&
        sent_revision == g_rover_map.revision &&
        (uint32_t) (now - last_sent_ms) < 2000u)
    {
        return;
    }
    last_attempt_ms = now;
    if (!rover_map_format(&g_rover_map, g_map_last_request, payload,
                          sizeof(payload)))
    {
        return;
    }
    if (mqtt_publish_map(payload) == WIFI_OK)
    {
        b_sent                    = true;
        sent_revision             = g_rover_map.revision;
        last_sent_ms              = now;
        g_b_map_publish_requested = false;
    }
}

static void
map_note_translation (const char * p_command, motor_error_t result)
{
    if (result != MOTOR_OK)
    {
        return;
    }

    const char * translations[] = { "FORWARD",      "BACKWARD",
                                    "LEFT",         "RIGHT",
                                    "FRONT_LEFT",   "FRONT_RIGHT",
                                    "BACK_LEFT",    "BACK_RIGHT",
                                    "MOVE_FORWARD", "MOVE_BACKWARD" };

    for (size_t index = 0;
         index < sizeof(translations) / sizeof(translations[0]); ++index)
    {
        if (strcmp(p_command, translations[index]) == 0)
        {
            mission_clear_leg(&g_mission,
                              to_ms_since_boot(get_absolute_time()));
            rover_map_depart(&g_rover_map);
            return;
        }
    }
    if (strcmp(p_command, "ROTATE_LEFT") == 0 ||
        strcmp(p_command, "ROTATE_RIGHT") == 0 ||
        strcmp(p_command, "TURN_TO") == 0 || strcmp(p_command, "TURN_BY") == 0)
    {
        mission_clear_leg(&g_mission, to_ms_since_boot(get_absolute_time()));
    }
}

static void
service_camera_relay (void)
{
    static uint32_t last_attempt_ms;
    static uint32_t last_sent_ms;
    static uint32_t sent_generation;
    static bool     b_sent;
    uint32_t        now = to_ms_since_boot(get_absolute_time());
    if (!mqtt_is_connected() ||
        (uint32_t) (now - last_attempt_ms) < MQTT_REPORT_RETRY_MS)
    {
        return;
    }
    camera_observation_t observation;
    if (!camera_uart_latest(&observation))
    {
        return;
    }
    if (b_sent && observation.generation == sent_generation &&
        (uint32_t) (now - last_sent_ms) < 1000u)
    {
        return;
    }
    last_attempt_ms = now;
    char payload[768];
    if (!camera_telemetry_format(payload, sizeof(payload), &observation.frame,
                                 observation.received_ms, now))
    {
        return;
    }
    if (mqtt_publish_camera(payload) == WIFI_OK)
    {
        b_sent          = true;
        sent_generation = observation.generation;
        last_sent_ms    = now;
    }
    // Backpressure: retry the latest observation later; never queue old frames.
}

static void
report_mission (bool b_force)
{
    static uint32_t serial_revision = UINT32_MAX;
    static uint32_t last_publish_ms;
    static uint32_t last_attempt_ms;
    static uint32_t published_revision;
    static bool     b_sent;
    static bool     b_requested;
    uint32_t        now       = to_ms_since_boot(get_absolute_time());
    bool            b_changed = serial_revision != g_mission.revision;
    b_requested |= b_force;

    bool b_due = b_requested || !b_sent ||
                 published_revision != g_mission.revision ||
                 (uint32_t) (now - last_publish_ms) >= 1000u;

    if (!b_force && !b_changed && !b_due)
    {
        return;
    }
    static char status[512];
    static char leg_status[256];
    if (!travel_leg_format_status(&g_mission.leg, leg_status,
                                  sizeof(leg_status)))
    {
        return;
    }

    int length = snprintf(
        status, sizeof(status),
        "MISSION_STATE:%s test=%u motion=%s reason=%s revision=%" PRIu32 " %s",
        mission_state_name(g_mission.state), (unsigned) g_mission.test_mode,
        mission_motion_enabled(&g_mission) ? "enabled" : "disabled",
        mission_reason_name(g_mission.reason), (uint32_t) g_mission.revision,
        leg_status);

    if (length < 0 || (size_t) length >= sizeof(status))
    {
        return;
    }
    if (b_force || b_changed)
    {
        printf("[MISSION] %s\n", status);
        serial_revision = g_mission.revision;
    }
    if (!mqtt_is_connected() || !b_due ||
        (uint32_t) (now - last_attempt_ms) < MQTT_REPORT_RETRY_MS)
    {
        return;
    }
    last_attempt_ms = now;
    if (mqtt_publish_report(MQTT_REPORT_MISSION, status) == WIFI_OK)
    {
        last_publish_ms    = now;
        published_revision = g_mission.revision;
        b_sent             = true;
        b_requested        = false;
    }
}

static void
stop_mission_outputs (void)
{
    g_b_autonomous_mode      = false;
    g_autonomous_state       = AUTO_STATE_IDLE;
    g_b_forward_boost_active = false;
    g_b_heading_zero_pending = false;
    motor_emergency_stop();
    arm_set_direction(ARM_STOP);
    claw_set_direction(CLAW_STOP);
}

static bool
target_selection_safe (void)
{
    // Selection is local and does not require a dashboard heartbeat.
    return !g_b_autonomous_mode && !g_b_heading_zero_pending &&
           !motor_safety_is_latched() && motor_motion_state() == 1;
}

static void
service_target_selection (void)
{
    static char          previous[768];
    static uint32_t      last_sent_ms;
    static uint32_t      last_attempt_ms;
    uint32_t             now = to_ms_since_boot(get_absolute_time());
    camera_observation_t observation;

    const camera_observation_t * p_latest =
        camera_uart_latest(&observation) ? &observation : NULL;

    if (!g_onboard_dfs.active || g_onboard_drive.active ||
        onboard_dfs_selecting(&g_onboard_dfs, now))
    {
        target_selector_tick(&g_target_selector, &g_mission, &g_rover_map,
                             p_latest, target_selection_safe(), now);
    }
    if (!mqtt_is_connected() ||
        (uint32_t) (now - last_attempt_ms) < MQTT_REPORT_RETRY_MS)
    {
        return;
    }
    static char payload[768];
    static char status[800];
    if (!target_selector_format(&g_target_selector, payload, sizeof(payload)))
    {
        return;
    }
    if (strcmp(previous, payload) == 0 &&
        (uint32_t) (now - last_sent_ms) < 1000u)
    {
        return;
    }
    snprintf(status, sizeof(status), "TARGET_SELECTION:%s", payload);
    last_attempt_ms = now;
    if (mqtt_publish_report(MQTT_REPORT_TARGET, status) == WIFI_OK)
    {
        memcpy(previous, payload, strlen(payload) + 1u);
        last_sent_ms = now;
    }
}

static bool
handle_target_command (const char * p_command)
{
    if (strcmp(p_command, "CLEAR_TARGET") == 0)
    {
        if (g_onboard_drive.active || g_onboard_dfs.active)
        {
            mqtt_publish_status("TARGET_REFUSED:STOP_DFS_FIRST");
            return true;
        }
        mission_clear_leg(&g_mission, to_ms_since_boot(get_absolute_time()));
        mqtt_publish_status("TARGET_ACK:CLEARED");
        report_mission(true);
        return true;
    }
    if (strncmp(p_command, "SELECT_NEXT", 11) != 0)
    {
        return false;
    }
    mqtt_publish_status("TARGET_REFUSED:SELECTION_IS_AUTOMATIC");
    return true;
}

static onboard_drive_inputs_t
drive_inputs (uint32_t now)
{
    uint32_t relative_x = mqtt_last_link_rx_ms();

    onboard_drive_inputs_t input = {
        .stopped        = target_selection_safe(),
        .safety_latched = motor_safety_is_latched(),
        .link_ok =
            relative_x != 0 && (uint32_t) (now - relative_x) <= LINK_TIMEOUT_MS,
        .sensor_valid       = g_b_sensor_data_valid,
        .sensor_received_ms = g_last_sensor_packet_ms,
        .heading_deg        = g_latest_sensors.heading,
        .heading_offset_deg = motor_heading_offset(),
        .imu_ok =
            g_b_sensor_data_valid && g_latest_sensors.imu_status == 0 &&
            motor_heading_sample_since(now - ONBOARD_DRIVE_SENSOR_MAX_AGE_MS),
        .front_mm    = (uint16_t) ((g_latest_sensors.tof_bits >> 10u) & 0x3ffu),
        .front_fault = (g_latest_sensors.obstacle_flags & (2u << 6u)) != 0,
        .front_urgent = (g_latest_sensors.obstacle_flags & (2u << 3u)) != 0
    };

    camera_observation_t observation;
    if (camera_uart_latest(&observation))
    {
        input.camera_present    = true;
        input.camera_generation = observation.generation;
        input.camera_seq        = observation.frame.seq;
        if (observation.frame.count <= VISION_MAX_MARKERS)
        {
            input.recovery_marker_count = observation.frame.count;
            memcpy(input.recovery_markers, observation.frame.markers,
                   (size_t) observation.frame.count *
                       sizeof(observation.frame.markers[0]));
        }
        input.camera_status        = observation.frame.status;
        input.camera_packet_age_ms = now - observation.received_ms;

        input.camera_image_age_ms =
            input.camera_packet_age_ms + observation.frame.age_ms;


        input.camera_fresh =
            observation.frame.status == VISION_STATUS_OK &&
            input.camera_packet_age_ms <= TARGET_UART_MAX_AGE_MS &&
            input.camera_image_age_ms <= ONBOARD_DRIVE_IMAGE_MAX_AGE_MS;

        if (g_mission.leg.arrival_reference_node &&
            observation.frame.count <= VISION_MAX_MARKERS &&
            (observation.frame.status == VISION_STATUS_OK ||
             observation.frame.status == VISION_STATUS_NO_MARKER) &&
            input.camera_packet_age_ms <= TARGET_UART_MAX_AGE_MS &&
            input.camera_image_age_ms <= ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
        {
            input.reference_absent = true;
            for (uint8_t index = 0; index < observation.frame.count; ++index)
            {
                if (observation.frame.markers[index].id ==
                        g_mission.leg.arrival_reference_node &&
                    !observation.frame.markers[index].quality &&
                    observation.frame.markers[index].range_mm &&
                    observation.frame.markers[index].range_mm != UINT16_MAX)
                {
                    input.reference_absent = false;
                }
            }
        }
    }
    return input;
}

static void
service_camera_map (uint32_t now)
{
    // An active leg owns its map snapshot, including stationary alignment
    // before departure. Project new camera cells again at the next node.
    if (g_onboard_drive.active)
    {
        return;
    }
    bool b_stopped = target_selection_safe();

    bool b_heading_valid =
        g_b_sensor_data_valid && g_latest_sensors.imu_status == 0 &&
        (uint32_t) (now - g_last_sensor_packet_ms) <=
            CAMERA_MAP_SENSOR_MAX_AGE_MS &&
        motor_heading_sample_since(now - CAMERA_MAP_SENSOR_MAX_AGE_MS) &&
        isfinite(g_latest_sensors.heading) && g_latest_sensors.heading >= 0 &&
        g_latest_sensors.heading < 360;

    // The first reliable stopped pose after Set start node establishes grid
    // north. DFS and camera projection must share this frame, including before
    // Start DFS.
    if (!g_onboard_drive.frame_valid && g_rover_map.count == 1u &&
        g_rover_map.current_node && b_stopped && b_heading_valid)
    {
        g_onboard_drive.north_heading_deg = g_latest_sensors.heading;
        g_onboard_drive.frame_valid       = true;
    }
    camera_observation_t observation;

    const camera_observation_t * p_latest =
        camera_uart_latest(&observation) ? &observation : NULL;

    camera_map_tick(&g_camera_map, &g_rover_map, p_latest, b_stopped,
                    b_heading_valid && g_onboard_drive.frame_valid,
                    g_latest_sensors.heading, g_onboard_drive.north_heading_deg,
                    now);
}

static void
service_onboard_drive (uint32_t now)
{
    bool                   b_was_active = g_onboard_drive.active;
    onboard_drive_inputs_t inputs       = drive_inputs(now);
    onboard_drive_tick(&g_onboard_drive, &g_mission, &g_rover_map,
                       &g_target_selector, &inputs, now);
    if (b_was_active && !g_onboard_drive.active)
    {
        char reply[96];
        snprintf(reply, sizeof(reply), "DFS_EVENT:%s",
                 g_onboard_dfs.active &&
                         strcmp(g_onboard_drive.result, "ARRIVED") == 0
                     ? "NODE_ARRIVED"
                     : g_onboard_drive.result);
        mqtt_publish_status(reply);
        printf("[DFS] %s\n", reply);
        g_b_map_publish_requested = true;
    }
    static char     previous[1024];
    static char     payload[1024];
    static char     packet[1056];
    static uint32_t last_sent;
    static uint32_t last_attempt;
    if (mqtt_is_connected() &&
        (uint32_t) (now - last_attempt) >= MQTT_REPORT_RETRY_MS &&
        onboard_drive_format(&g_onboard_drive, &g_mission, payload,
                             sizeof(payload)) &&
        (strcmp(previous, payload) != 0 ||
         (uint32_t) (now - last_sent) >= 1000u))
    {
        snprintf(packet, sizeof(packet), "DFS_TRACKING:%s", payload);
        last_attempt = now;
        if (mqtt_publish_report(MQTT_REPORT_TRACKING, packet) == WIFI_OK)
        {
            memcpy(previous, payload, strlen(payload) + 1u);
            last_sent = now;
        }
    }
}

static onboard_dfs_inputs_t
dfs_inputs (uint32_t now)
{

    onboard_dfs_inputs_t inputs = {
        .drive      = drive_inputs(now),
        .fault_mask = (uint8_t) ((g_latest_sensors.obstacle_flags >> 6u) & 7u),
        .heading_offset_deg = motor_heading_offset()
    };

    for (unsigned index = 0; index < TOF_SCAN_COUNT; ++index)
    {

        inputs.tof_mm[index] =
            (uint16_t) ((g_latest_sensors.tof_bits >> (10u * index)) & 1023u);
    }
    return inputs;
}

static void
service_onboard_dfs (uint32_t now)
{
    bool                 b_was_active = g_onboard_dfs.active;
    onboard_dfs_inputs_t inputs       = dfs_inputs(now);
    camera_observation_t observation;

    const camera_observation_t * p_camera =
        camera_uart_latest(&observation) ? &observation : NULL;

    onboard_dfs_tick(&g_onboard_dfs, &g_mission, &g_rover_map, &g_onboard_drive,
                     &g_target_selector, &inputs, p_camera, now);
    if (b_was_active && !g_onboard_dfs.active)
    {
        if (strcmp(g_onboard_dfs.detail, "CANCELLED") != 0)
        {
            char event[96];
            snprintf(event, sizeof(event), "DFS_EVENT:%s",
                     g_onboard_dfs.stage == DFS_FINISHED
                         ? "COMPLETE"
                         : g_onboard_dfs.detail);
            mqtt_publish_status(event);
        }
        g_b_map_publish_requested = true;
    }
    static char     previous[384];
    static char     payload[384];
    static char     packet[400];
    static uint32_t last;
    static uint32_t last_attempt;
    if (mqtt_is_connected() &&
        (uint32_t) (now - last_attempt) >= MQTT_REPORT_RETRY_MS &&
        onboard_dfs_format(&g_onboard_dfs, &g_rover_map, payload,
                           sizeof(payload)) &&
        (strcmp(previous, payload) != 0 || (uint32_t) (now - last) >= 1000u))
    {
        snprintf(packet, sizeof(packet), "DFS_STATE:%s", payload);
        last_attempt = now;
        if (mqtt_publish_report(MQTT_REPORT_DFS, packet) == WIFI_OK)
        {
            memcpy(previous, payload, strlen(payload) + 1u);
            last = now;
        }
    }
}

static bool
handle_map_command (const char * p_command)
{
    if (strcmp(p_command, "MAP_GET") == 0)
    {
        g_b_map_publish_requested = true;
        return true;
    }
    if (strncmp(p_command, "MAP_RESET:", 10) == 0)
    {
        const char * p_request = p_command + 10;
        if (strlen(p_request) != 32u ||
            strspn(p_request, "0123456789abcdef") != 32u)
        {
            mqtt_publish_status("MAP_REFUSED:RESET_EXPECTED_REQUEST32HEX");
            return true;
        }
        if (strcmp(p_request, g_map_last_request) == 0)
        {
            if (g_last_start_marker)
            {
                mqtt_publish_status("MAP_REFUSED:REQUEST_ID_REUSED");
            }
            else
            {
                g_b_map_publish_requested = true;
                mqtt_publish_status("MAP_ACK:RESET_DUPLICATE");
            }
            return true;
        }
        // Stop before discarding the active route. Never clear the safety
        // latch.
        stop_mission_outputs();
        uint32_t now = to_ms_since_boot(get_absolute_time());
        mission_test_event(&g_mission, "RESET", motor_safety_is_latched(), now);
        onboard_drive_init(&g_onboard_drive);
        camera_map_init(&g_camera_map);
        onboard_dfs_init(&g_onboard_dfs);
        target_selector_init(&g_target_selector);
        rover_map_clear(&g_rover_map);
        memcpy(g_map_last_request, p_request, sizeof(g_map_last_request));
        g_last_start_marker       = 0;
        g_b_map_publish_requested = true;
        mqtt_publish_status("MAP_ACK:RESET_ACCEPTED");
        report_mission(true);
        return true;
    }
    if (strncmp(p_command, "SET_START:", 10) != 0)
    {
        return false;
    }
    const char * p_token = p_command + 10;
    size_t       length  = strlen(p_token);
    if (length < 34 || length > 36 || p_token[32] != ':')
    {
        goto invalid;
    }
    for (size_t index = 0; index < MAP_REQUEST_HEX_LENGTH; ++index)
    {
        if (!strchr("0123456789abcdef", p_token[index]))
        {
            goto invalid;
        }
    }
    unsigned marker_id = 0;
    for (size_t index = 33; index < length; ++index)
    {
        if (p_token[index] < '0' || p_token[index] > '9')
        {
            goto invalid;
        }
        marker_id = marker_id * 10u + (unsigned) (p_token[index] - '0');
    }
    if (!marker_id || marker_id > UINT8_MAX)
    {
        goto invalid;
    }
    char p_request[33];
    memcpy(p_request, p_token, 32);
    p_request[32] = '\0';
    if (strcmp(p_request, g_map_last_request) == 0)
    {
        if (g_last_start_marker != marker_id)
        {
            mqtt_publish_status("MAP_REFUSED:REQUEST_ID_REUSED");
            return true;
        }
        g_b_map_publish_requested = true;
        mqtt_publish_status("MAP_ACK:SET_START_DUPLICATE");
        return true;
    }
    if ((g_mission.state != MISSION_IDLE &&
         (g_mission.state != MISSION_PAUSED || g_mission.test_mode)) ||
        motor_safety_is_latched() || g_b_heading_zero_pending ||
        (motor_motion_state() != 0 && motor_motion_state() != 1))
    {
        mqtt_publish_status(
            "MAP_REFUSED:STOP_RESET_MISSION_AND_CLEAR_SAFETY_FIRST");
        return true;
    }
    stop_mission_outputs();
    onboard_drive_init(&g_onboard_drive); // A new asserted origin also
                                          // establishes a new grid frame.
    camera_map_init(&g_camera_map);
    onboard_dfs_init(&g_onboard_dfs);
    target_selector_init(&g_target_selector);
    mission_test_event(&g_mission, "RESET", false,
                       to_ms_since_boot(get_absolute_time()));
    mission_clear_leg(&g_mission, to_ms_since_boot(get_absolute_time()));
    rover_map_start(&g_rover_map, (uint8_t) marker_id);
    memcpy(g_map_last_request, p_request, sizeof(g_map_last_request));
    g_last_start_marker       = (uint8_t) marker_id;
    g_b_map_publish_requested = true;
    printf("[MAP] Start node %u at (0,0), current_node=%u, revision=%" PRIu32
           "\n",
           marker_id, (unsigned) g_rover_map.current_node,
           (uint32_t) g_rover_map.revision);
    mqtt_publish_status("MAP_ACK:SET_START_ACCEPTED");
    return true;
invalid:
    mqtt_publish_status(
        "MAP_REFUSED:EXPECTED_SET_START_REQUEST32HEX_ID1_TO_255");
    return true;
}

static bool
handle_mission_command (const char * p_command)
{
    if (strcmp(p_command, "MISSION_STATUS") == 0)
    {
        report_mission(true);
        return true;
    }
    if (strcmp(p_command, "START_DFS") == 0 ||
        strncmp(p_command, "START_DFS:", 10) == 0)
    {
        if (p_command[9] == ':')
        {
            char *   p_end   = NULL;
            uint32_t opening = strtoul(p_command + 10, &p_end, 10);
            if (p_command[10] < '0' || p_command[10] > '9' || *p_end ||
                opening <= FRONT_STOP_DISTANCE_MM || opening > 1000u)
            {
                mqtt_publish_status("DFS_REFUSED:INVALID_OPENING_THRESHOLD");
                return true;
            }
            if (g_onboard_dfs.active &&
                opening != g_onboard_dfs.open_distance_mm)
            {
                mqtt_publish_status(
                    "DFS_REFUSED:STOP_DFS_BEFORE_CONFIG_CHANGE");
                return true;
            }
            g_onboard_dfs.open_distance_mm = (uint16_t) opening;
        }
        service_target_selection(); // Validate the latest camera packet before
                                    // authorising motion.
        uint32_t             now    = to_ms_since_boot(get_absolute_time());
        onboard_dfs_inputs_t inputs = dfs_inputs(now);

        const char * p_result = onboard_dfs_start(
            &g_onboard_dfs, &g_mission, &g_rover_map, &g_onboard_drive,
            &g_target_selector, &inputs, now);


        bool b_is_valid = strcmp(p_result, "STARTED") == 0 ||
                          strcmp(p_result, "ALREADY_RUNNING") == 0;

        char reply[96];
        snprintf(reply, sizeof(reply), "DFS_%s:%s",
                 b_is_valid ? "ACK" : "REFUSED", p_result);
        mqtt_publish_status(reply);
        printf("[DFS] %s\n", reply);
        report_mission(true);
        g_b_map_publish_requested = true;
        return true;
    }
    if (strcmp(p_command, "STOP_DFS") == 0)
    {
        g_onboard_dfs.active = false;
        g_onboard_dfs.stage  = DFS_STOPPED;
        g_onboard_dfs.detail = "OPERATOR_STOP";
        onboard_drive_stop(&g_onboard_drive, &g_mission,
                           MISSION_REASON_OPERATOR_STOP, "OPERATOR_STOP",
                           to_ms_since_boot(get_absolute_time()));
        stop_mission_outputs();
        mqtt_publish_status("DFS_ACK:STOPPED");
        report_mission(true);
        return true;
    }
    const char * p_event = NULL;
    if (strncmp(p_command, "MISSION_TEST:", 13) == 0)
    {
        p_event = p_command + 13;
    }
    else if (strcmp(p_command, "RESET_MISSION") == 0)
    {
        p_event = "RESET";
    }
    else
    {
        return false;
    }


    mission_result_t p_result =
        mission_test_event(&g_mission, p_event, motor_safety_is_latched(),
                           to_ms_since_boot(get_absolute_time()));

    if (p_result == MISSION_ACCEPTED)
    {
        stop_mission_outputs();
    }
    if (p_result == MISSION_ACCEPTED && strcmp(p_event, "RESET") == 0)
    {
        onboard_dfs_init(&g_onboard_dfs);
        target_selector_init(&g_target_selector);
        g_onboard_drive.active         = false;
        g_onboard_drive.estimate_valid = false;
        g_onboard_drive.result         = "MISSION_RESET";
    }
    char reply[96];
    snprintf(reply, sizeof(reply), "MISSION_TEST:%s state=%s",
             mission_result_name(p_result),
             mission_state_name(g_mission.state));
    printf("[MISSION] %s\n", reply);
    mqtt_publish_status(reply);
    report_mission(true);
    return true;
}

static void
service_heading_zero (void)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!g_b_heading_zero_pending ||
        (int32_t) (now - g_heading_zero_ready_ms) < 0)
    {
        return;
    }
    // Capture a new healthy sample after braking, rather than zeroing a cached
    // heading measured while the rover was still rotating.
    if (!motor_heading_sample_since(g_heading_zero_ready_ms))
    {
        if ((uint32_t) (now - g_heading_zero_ready_ms) <= 1000u)
        {
            return;
        }
        g_b_heading_zero_pending = false;
        mqtt_publish_status("HEADING_ZERO_REFUSED");
        return;
    }
    g_b_heading_zero_pending = false;
    mqtt_publish_status(motor_zero_heading() == MOTOR_OK
                            ? "HEADING_ZEROED"
                            : "HEADING_ZERO_REFUSED");
}

static void
on_sensor_packet (const sensor_packet_t * p_packet, uint32_t receive_ms)
{
    // Runs in the UART ISR: no MQTT, printf, sleeps, or reverse-thrust braking.
    uint16_t front_mm      = (uint16_t) ((p_packet->tof_bits >> 10u) & 0x3FFu);
    bool     b_front_fault = (p_packet->obstacle_flags & (2u << 6u)) != 0;
    // Read the actual range; old urgent/STOP flags may retain hysteresis.
    motor_safety_receive(front_obstacle_detected(front_mm, b_front_fault),
                         receive_ms);
    motor_heading_update(p_packet->heading,
                         p_packet->imu_status == 0); // heading hold (FR-41)
}

static bool
safety_watchdog_tick (struct repeating_timer * p_timer)
{
    (void) p_timer;
    motor_safety_poll();
    return true;
}

static void
service_safety_stop (void)
{
    motor_safety_poll();
    if (motor_take_front_stop())
    {
        g_b_autonomous_mode      = false;
        g_autonomous_state       = AUTO_STATE_IDLE;
        g_b_forward_boost_active = false;

        bool b_planned_arrival =
            onboard_drive_note_front_stop(&g_onboard_drive, &g_mission);

        if (!b_planned_arrival &&
            (!g_onboard_dfs.active || g_onboard_drive.active ||
             g_mission.test_mode))
        {
            mission_pause(&g_mission, MISSION_REASON_FRONT_STOP,
                          to_ms_since_boot(get_absolute_time()));
        }
        mqtt_publish_status("FRONT_STOPPED");
    }
    if (motor_safety_is_latched())
    {
        mission_fault(&g_mission, MISSION_REASON_ESTOP,
                      to_ms_since_boot(get_absolute_time()));
        g_b_autonomous_mode      = false;
        g_autonomous_state       = AUTO_STATE_IDLE;
        g_b_forward_boost_active = false;
        if (!g_b_safety_lock_reported)
        {
            g_b_safety_lock_reported = true;
            printf("[SAFETY] Manual dashboard E-STOP active\n");
            mqtt_publish_status("SAFETY_LOCKED");
        }
    }
    else
    {
        g_b_safety_lock_reported = false;
    }
}

// Autonomous navigation parameters
#define AUTO_OBSTACLE_DISTANCE_MM 600.0f // Stop if obstacle within 600mm
#define AUTO_FORWARD_SECTOR_MIN 160.0f // Front sector: 160° to 200° (40° cone)
#define AUTO_FORWARD_SECTOR_MAX 200.0f
#define AUTO_LEFT_SECTOR_MIN 90.0f // Left sector: 90° to 150° (60° cone)
#define AUTO_LEFT_SECTOR_MAX 150.0f
#define AUTO_RIGHT_SECTOR_MIN 210.0f // Right sector: 210° to 270° (60° cone)
#define AUTO_RIGHT_SECTOR_MAX 270.0f
#define AUTO_TURN_DURATION_MS 400  // Turn execution time (400ms)
#define AUTO_SCAN_DURATION_MS 2500 // Gap detection scan period (2500ms)
#define AUTO_TURN_BIAS_PENALTY_MM                                              \
    300.0f // Penalty to avoid turn direction oscillation (300mm)
#define AUTO_DEAD_END_TIME_MS 2000 // Obstacle hit within 2000ms = dead end
#define AUTO_SUCCESSFUL_FORWARD_TIME_MS                                        \
    5000 // Clear turn bias after 5000ms forward progress
// Gap-aware navigation thresholds
#define AUTO_FREE_DISTANCE_MM 1000.0f // Ray considered free if >= 1000mm
#define AUTO_MIN_FORWARD_CLEAR_RATIO                                           \
    0.40f // Require 40% of front rays free to resume

// Pitch compensation for upslope detection
#define PITCH_BASELINE                                                         \
    0.0f // MPU6500 mounted +X forward, +Z up: flat = 0 degrees
#define PITCH_BOOST_PER_15_DEG 10 // Speed boost: +10% per 15 degrees nose-up
#define PITCH_MAX_BOOST 30        // Maximum boost capped at +30%

static int
apply_pitch_compensation (int base_speed, float current_pitch)
{
    // Sketch convention with +X forward/+Z up: nose-up pitch is negative.
    float pitch_drop = PITCH_BASELINE - current_pitch;

    if (pitch_drop > 0)
    {
        // Apply boost: every 15 degrees nose-up adds 10% speed
        int boost_percent = (int) (pitch_drop / 15.0f) * PITCH_BOOST_PER_15_DEG;

        // Apply boost cap at 30%
        if (boost_percent > PITCH_MAX_BOOST)
        {
            boost_percent = PITCH_MAX_BOOST;
        }

        int adjusted_speed = base_speed + boost_percent;

        // Hard limit at 100%
        if (adjusted_speed > 100)
        {
            adjusted_speed = 100;
        }

        return adjusted_speed;
    }

    return base_speed;
}

static void
process_command (const char * p_command)
{
    printf("[CMD] Processing: %s\n", p_command);

    motor_error_t result = MOTOR_OK;

    // Dashboard keys: one word per move (e.g. "FORWARD"). Anything after a
    // ':' (old speed / wheel values) is ignored; each move has its own speed
    // in motor_config.h (MC_DASH_*_PCT).
    char cmd_buffer[32];
    strncpy(cmd_buffer, p_command, sizeof(cmd_buffer) - 1);
    cmd_buffer[sizeof(cmd_buffer) - 1] = '\0';
    char * p_colon                     = strchr(cmd_buffer, ':');
    if (p_colon != NULL)
    {
        *p_colon = '\0';
    }

    service_safety_stop();
    if (handle_map_command(p_command))
    {
        return;
    }
    if (handle_mission_command(p_command))
    {
        return;
    }
    if (handle_target_command(p_command))
    {
        return;
    }
    if (!mission_command_allowed(&g_mission, p_command))
    {
        mqtt_publish_status(
            g_mission.test_mode
                ? "MISSION_REFUSED:TEST_MODE_MOVEMENT_DISABLED"
                : "MISSION_REFUSED:STOP_DFS_BEFORE_MANUAL_CONTROL");
        return;
    }
    if (strcmp(cmd_buffer, "ZERO_HEADING") == 0)
    {
        mission_clear_leg(&g_mission, to_ms_since_boot(get_absolute_time()));
        g_b_autonomous_mode = false;
        g_autonomous_state  = AUTO_STATE_IDLE;
        motor_emergency_stop();
        g_b_heading_zero_pending = true;
        g_heading_zero_ready_ms  = to_ms_since_boot(get_absolute_time()) + 400u;
        mqtt_publish_status("HEADING_ZERO_PENDING");
        return;
    }
    if (g_b_heading_zero_pending)
    {
        if (strcmp(cmd_buffer, "STOP") == 0 ||
            strcmp(cmd_buffer, "ESTOP") == 0 ||
            strcmp(cmd_buffer, "RESET_SAFETY") == 0 ||
            strcmp(cmd_buffer, "AUTO_STOP") == 0)
        {
            g_b_heading_zero_pending = false;
            mqtt_publish_status("HEADING_ZERO_CANCELLED");
        }
        else
        {
            mqtt_publish_status("HEADING_ZERO_BUSY");
            return;
        }
    }
    if (strcmp(cmd_buffer, "ESTOP") == 0)
    {
        motor_manual_estop();
        service_safety_stop();
        return;
    }
    if (strcmp(cmd_buffer, "RESET_SAFETY") == 0)
    {
        mission_clear_leg(&g_mission, to_ms_since_boot(get_absolute_time()));
        g_b_autonomous_mode = false;
        g_autonomous_state  = AUTO_STATE_IDLE;
        if (motor_safety_reset())
        {
            g_b_safety_lock_reported = false;
            mqtt_publish_status("SAFETY_READY");
            printf("[SAFETY] Reset accepted; a new movement command is "
                   "required\n");
        }
        else
        {
            mqtt_publish_status("SAFETY_RESET_REFUSED");
            printf("[SAFETY] Reset refused: missing, stale, or unsafe sensor "
                   "packet\n");
        }
        return;
    }
    if (motor_safety_is_latched() && strcmp(cmd_buffer, "STOP") != 0 &&
        strcmp(cmd_buffer, "AUTO_STOP") != 0)
    {
        mqtt_publish_status("SAFETY_LOCKED");
        return;
    }

    if (strncmp(cmd_buffer, "FORWARD", 7) == 0)
    {
        // Apply pitch compensation for upslope
        int speed = motor_move_speed(DIRECTION_FORWARD);

        int compensated_speed =
            (g_b_sensor_data_valid && g_latest_sensors.imu_status == 0)
                ? apply_pitch_compensation(speed, g_latest_sensors.pitch)
                : speed;

        if (compensated_speed != speed)
        {
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "PITCH: Upslope %.1f° - boost %d%% -> %d%%",
                     g_latest_sensors.pitch, speed, compensated_speed);
            printf("[%s]\n", msg);
            mqtt_publish_status(msg);
        }
        result = motor_set_direction(DIRECTION_FORWARD, compensated_speed);
    }
    else if (strncmp(cmd_buffer, "BACKWARD", 8) == 0)
    {

        result = motor_set_direction(DIRECTION_BACKWARD,
                                     motor_move_speed(DIRECTION_BACKWARD));
    }
    else if (strncmp(cmd_buffer, "LEFT", 4) == 0)
    {

        result = motor_set_direction(DIRECTION_LEFT,
                                     motor_move_speed(DIRECTION_LEFT));
    }
    else if (strncmp(cmd_buffer, "RIGHT", 5) == 0)
    {

        result = motor_set_direction(DIRECTION_RIGHT,
                                     motor_move_speed(DIRECTION_RIGHT));
    }
    else if (strncmp(cmd_buffer, "ROTATE_LEFT", 11) == 0)
    {

        result = motor_set_direction(DIRECTION_ROTATE_LEFT,
                                     motor_move_speed(DIRECTION_ROTATE_LEFT));
    }
    else if (strncmp(cmd_buffer, "ROTATE_RIGHT", 12) == 0)
    {

        result = motor_set_direction(DIRECTION_ROTATE_RIGHT,
                                     motor_move_speed(DIRECTION_ROTATE_RIGHT));
    }
    else if (strncmp(cmd_buffer, "FRONT_LEFT", 10) == 0)
    {

        result = motor_set_direction(DIRECTION_FRONT_LEFT,
                                     motor_move_speed(DIRECTION_FRONT_LEFT));
    }
    else if (strncmp(cmd_buffer, "FRONT_RIGHT", 11) == 0)
    {

        result = motor_set_direction(DIRECTION_FRONT_RIGHT,
                                     motor_move_speed(DIRECTION_FRONT_RIGHT));
    }
    else if (strncmp(cmd_buffer, "BACK_LEFT", 9) == 0)
    {

        result = motor_set_direction(DIRECTION_BACK_LEFT,
                                     motor_move_speed(DIRECTION_BACK_LEFT));
    }
    else if (strncmp(cmd_buffer, "BACK_RIGHT", 10) == 0)
    {

        result = motor_set_direction(DIRECTION_BACK_RIGHT,
                                     motor_move_speed(DIRECTION_BACK_RIGHT));
    }
    else if (strncmp(cmd_buffer, "ARM_UP", 6) == 0)
    {
        // Direct mapping per corrected wiring
        result = arm_set_direction(ARM_UP);
    }
    else if (strncmp(cmd_buffer, "ARM_DOWN", 8) == 0)
    {
        // Direct mapping per corrected wiring
        result = arm_set_direction(ARM_DOWN);
    }
    else if (strncmp(cmd_buffer, "CLAW_OPEN", 9) == 0)
    {
        // Direct mapping per corrected wiring
        result = claw_set_direction(CLAW_OPEN);
    }
    else if (strncmp(cmd_buffer, "CLAW_CLOSE", 10) == 0)
    {
        // Direct mapping per corrected wiring
        result = claw_set_direction(CLAW_CLOSE);
    }
    // ---- Autonomous moves for the mapping / Exploration side (Subsystem 6)
    // ---- One command starts the move; it keeps going until MOVE_STOP, STOP
    // (operator) or a safety stop. Rotations end by themselves.
    //   MOVE_FORWARD:30   MOVE_BACKWARD:30   (speed %, default = W/S key speed)
    //   TURN_TO:90        absolute heading 0..359
    //   TURN_BY:-90       relative: + right, - left
    //   MOVE_STOP         normal stop (ramp down)
    // Replies on robot/status: "AUTO_MOVE: ..." now, "AUTO_RESULT: ..." when it
    // ends.
    else if (strncmp(cmd_buffer, "MOVE_FORWARD", 12) == 0 ||
             strncmp(cmd_buffer, "MOVE_BACKWARD", 13) == 0 ||
             strncmp(cmd_buffer, "TURN_TO", 7) == 0 ||
             strncmp(cmd_buffer, "TURN_BY", 7) == 0 ||
             strncmp(cmd_buffer, "MOVE_STOP", 9) == 0)
    {
        const char * p_arg     = strchr(p_command, ':');
        bool         b_has_arg = p_arg != NULL && p_arg[1] != '\0';
        int          value     = b_has_arg ? atoi(p_arg + 1) : 0;

        g_b_autonomous_mode = false;
        // the old LIDAR auto mode must not drive too
        g_autonomous_state = AUTO_STATE_IDLE;

        if (strncmp(cmd_buffer, "MOVE_FORWARD", 12) == 0)
        {

            result = motor_auto_drive(
                b_has_arg ? abs(value) : motor_move_speed(DIRECTION_FORWARD));
        }
        else if (strncmp(cmd_buffer, "MOVE_BACKWARD", 13) == 0)
        {

            result = motor_auto_drive(-(
                b_has_arg ? abs(value) : motor_move_speed(DIRECTION_BACKWARD)));
        }
        else if (strncmp(cmd_buffer, "TURN_TO", 7) == 0)
        {
            result = b_has_arg ? motor_auto_rotate_to(value) : MOTOR_ERR_PIN;
        }
        else if (strncmp(cmd_buffer, "TURN_BY", 7) == 0)
        {
            result = b_has_arg ? motor_auto_rotate_by(value) : MOTOR_ERR_PIN;
        }
        else
        {
            mission_pause(&g_mission, MISSION_REASON_OPERATOR_STOP,
                          to_ms_since_boot(get_absolute_time()));
            result = motor_auto_stop();
        }
        map_note_translation(cmd_buffer, result);

        char        msg[80];
        static char last_move[32] = "";
        if (result == MOTOR_OK && strncmp(cmd_buffer, "MOVE_", 5) == 0 &&
            strcmp(last_move, p_command) == 0 &&
            motor_auto_status() == MOTOR_AUTO_RUNNING)
        {
            return; // the laptop repeats MOVE_FORWARD: still running, no new
                    // reply
        }
        strncpy(last_move, p_command, sizeof(last_move) - 1);
        if (result == MOTOR_OK)
        {
            // Also report consecutive short turns that finish between loop
            // polls.
            g_last_auto_reported = MOTOR_AUTO_RUNNING;
            snprintf(msg, sizeof(msg), "AUTO_MOVE: %s", p_command);
        }
        else if (result == MOTOR_ERR_FRONT_BLOCKED)
        {
            last_move[0] = '\0';
            snprintf(msg, sizeof(msg),
                     "FORWARD_BLOCKED"); // same reply as the W key
        }
        else if (result == MOTOR_ERR_NO_HEADING)
        {
            snprintf(msg, sizeof(msg),
                     "AUTO_REFUSED: %s (no fresh heading from IMU)",
                     cmd_buffer);
        }
        else if (result == MOTOR_ERR_PIN)
        {
            snprintf(msg, sizeof(msg),
                     "AUTO_REFUSED: %s (bad or missing value)", cmd_buffer);
        }
        else
        {
            snprintf(msg, sizeof(msg),
                     "AUTO_REFUSED: %s (safety lock / SAFE STOP / busy)",
                     cmd_buffer);
        }
        printf("[AUTO] %s\n", msg);
        mqtt_publish_status(msg);
        return;
    }
    else if (strncmp(cmd_buffer, "AUTO_START", 10) == 0)
    {
        // Exploration decisions belong to Python. RoboPico executes movement
        // commands and retains its local heading hold and collision interlock.
        mqtt_publish_status("AUTO_START_DISABLED_USE_LAPTOP_DFS");
        return;
    }
    else if (strncmp(cmd_buffer, "AUTO_STOP", 9) == 0)
    {
        mission_pause(&g_mission, MISSION_REASON_OPERATOR_STOP,
                      to_ms_since_boot(get_absolute_time()));
        g_b_autonomous_mode = false;
        g_autonomous_state  = AUTO_STATE_IDLE;
        motor_emergency_stop();
        printf("[AUTO] Autonomous mode DISABLED\n");
        mqtt_publish_status("AUTO_DISABLED");
        return;
    }
    else if (strncmp(cmd_buffer, "STOP", 4) == 0)
    {
        mission_pause(&g_mission, MISSION_REASON_OPERATOR_STOP,
                      to_ms_since_boot(get_absolute_time()));
        g_b_autonomous_mode = false; // Stop also disables auto
        g_autonomous_state  = AUTO_STATE_IDLE;
        result              = motor_emergency_stop();
        mqtt_publish_status("STOPPED");
        return;
    }
    else
    {
        printf("[CMD] Unknown command: %s\n", cmd_buffer);
        return;
    }
    printf("[CMD] %s -> result %d (motion state %d)\n", cmd_buffer,
           (int) result, motor_motion_state());
    map_note_translation(cmd_buffer, result);

    if (result == MOTOR_OK)
    {
        static char last_reported[16] = "";
        if (strncmp(last_reported, cmd_buffer, sizeof(last_reported)) != 0)
        {
            mqtt_publish_status("RUNNING");
            strncpy(last_reported, cmd_buffer, sizeof(last_reported) - 1);
        }
    }
    else if (result == MOTOR_ERR_FRONT_BLOCKED)
    {
        mqtt_publish_status("FORWARD_BLOCKED");
    }
    else
    {
        mqtt_publish_status("ERROR");
    }
    // Record command activity time for MQTT load adaptation
    g_last_command_time_ms = to_ms_since_boot(get_absolute_time());
}

static void
publish_sensor_data (void)
{

    if (!g_b_sensor_data_valid)
    {
        return;
    }

    static char telemetry[2048]; // ← Reduced size since no LIDAR
    static char leg_json[384];
    if (!travel_leg_format_json(&g_mission.leg, leg_json, sizeof(leg_json)))
    {
        return;
    }

    memset(telemetry, 0, sizeof(telemetry));

    snprintf(
        telemetry, sizeof(telemetry),
        "{\"timestamp\":%" PRIu32 ","
        "\"heading_epoch\":%" PRIu32
        ",\"heading_offset_deg\":%.3f,\"mission\":{\"state\":\"%s\",\"reason\":"
        "\"%s\",\"test_mode\":%s,"
        "\"motion_enabled\":%s,\"revision\":%" PRIu32 ",\"leg\":%s},\"imu\":{"
        "\"heading\":%.1f,\"heading_reference\":\"%s\","
        "\"status\":%d},\"obstacle\":{"
        "\"distance_mm\":{\"left\":%u,\"front\":%u,\"right\":%u},\"blocked_"
        "mask\":%u,"
        "\"urgent_mask\":%u,\"fault_mask\":%u,"
        "\"stop_required\":%s,\"age_ms\":%" PRIu32 ","
        "\"forward_threshold_mm\":%u}"
        "}",
        (uint32_t) g_latest_sensors.timestamp_ms,
        (uint32_t) motor_heading_epoch(), motor_heading_offset(),
        mission_state_name(g_mission.state),
        mission_reason_name(g_mission.reason),
        g_mission.test_mode ? "true" : "false",
        mission_motion_enabled(&g_mission) ? "true" : "false",
        (uint32_t) g_mission.revision, leg_json,
        motor_heading_relative(g_latest_sensors.heading),
        motor_heading_epoch() ? "manual_zero" : "startup",
        g_latest_sensors.imu_status,
        // (unsigned)latest_sensors.tof_mm[0],
        // (unsigned)latest_sensors.tof_mm[1],
        // (unsigned)latest_sensors.tof_mm[2],
        (unsigned) (g_latest_sensors.tof_bits & 0x3FFu), // Left (bit 0-9)
        (unsigned) ((g_latest_sensors.tof_bits >> 10u) &
                    0x3FFu), // Front (bit 10-19)
        (unsigned) ((g_latest_sensors.tof_bits >> 20u) &
                    0x3FFu), // Right (bit 20-29)
        //  (unsigned)latest_sensors.blocked_mask,
        //  (unsigned)latest_sensors.urgent_mask,
        //  (unsigned)latest_sensors.fault_mask,
        //  latest_sensors.stop_required ? "true" : "false",
        (unsigned) (g_latest_sensors.obstacle_flags & 0x07u),
        (unsigned) ((g_latest_sensors.obstacle_flags >> 3u) & 0x07u),
        (unsigned) ((g_latest_sensors.obstacle_flags >> 6u) & 0x07u),
        (g_latest_sensors.obstacle_flags & (1u << 9u)) ? "true" : "false",
        (uint32_t) (to_ms_since_boot(get_absolute_time()) -
                    g_last_sensor_packet_ms),
        (unsigned) FRONT_STOP_DISTANCE_MM);

    mqtt_publish_telemetry(telemetry);
}

int
main (void)
{
    stdio_init_all();
    sleep_ms(2000);
    mission_init(&g_mission, to_ms_since_boot(get_absolute_time()));
    target_selector_init(&g_target_selector);
    onboard_drive_init(&g_onboard_drive);
    onboard_dfs_init(&g_onboard_dfs);
    rover_map_init(&g_rover_map, get_rand_64());
    report_mission(true);

    printf("\n========================================\n");
    printf("  Mecanum Robot Controller\n");
    printf("========================================\n\n");

    if (wifi_init() != WIFI_OK)
    {
        printf("[ERROR] WiFi init failed\n");
        return -1;
    }

    if (wifi_connect(WIFI_SSID, WIFI_PASSWORD, 30000) != WIFI_OK)
    {
        printf("[ERROR] WiFi connection failed\n");
        return -1;
    }

    if (motor_init() != MOTOR_OK)
    {
        printf("[ERROR] Motor init failed\n");
        return -1;
    }

    struct repeating_timer safety_timer;
    if (!add_repeating_timer_ms(-10, safety_watchdog_tick, NULL, &safety_timer))
    {
        printf("[ERROR] Cannot start safety watchdog; motors remain locked\n");
        return -1;
    }
    uart_set_packet_callback(on_sensor_packet);
    uart_receiver_init(SENSOR_UART_ID, SENSOR_UART_BAUD, SENSOR_UART_RX_PIN,
                       SENSOR_UART_TX_PIN);
    camera_uart_init();

    // Initialize piezo buzzer with PWM
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    g_buzzer_slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Set PWM frequency to 2.5kHz
    float    clock_freq = 125000000.0f; // Pico system clock
    uint32_t divider    = clock_freq / (BUZZER_FREQ_HZ * 65536); // 16-bit wrap
    if (divider < 1)
    {
        divider = 1;
    }
    pwm_set_clkdiv(g_buzzer_slice_num, (float) divider);
    pwm_set_wrap(g_buzzer_slice_num, 65535); // 16-bit resolution
    pwm_set_chan_level(g_buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN),
                       0); // Start off
    pwm_set_enabled(g_buzzer_slice_num, true);
    printf("[BUZZER] PWM initialized on GP%d at %dHz\n", BUZZER_PIN,
           BUZZER_FREQ_HZ);

    if (mqtt_init(MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_CLIENT_ID) != WIFI_OK)
    {
        printf("[ERROR] MQTT init failed\n");
        return -1;
    }

    if (mqtt_connect() != WIFI_OK)
    {
        printf("[ERROR] MQTT connection failed\n");
        return -1;
    }

    printf("[MQTT] Waiting for connection...\n");
    int retries = 0;
    while (!mqtt_is_connected() && retries < 100)
    {
        cyw43_arch_poll();
        sys_check_timeouts();
        sleep_ms(50);
        retries++;
    }

    if (!mqtt_is_connected())
    {
        printf("[ERROR] MQTT connection timeout\n");
        return -1;
    }

    if (mqtt_subscribe_commands(process_command) != WIFI_OK)
    {
        printf("[ERROR] MQTT subscribe failed\n");
        return -1;
    }

    mqtt_publish_status("READY");

    printf("\n========================================\n");
    printf("  System Ready!\n");
    printf("  IP: %s\n", wifi_get_ip_address());
    printf("  MQTT Broker: %s:%d\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    printf("  UART Sensor: Listening on GP%d\n", SENSOR_UART_RX_PIN);
    printf("========================================\n\n");

    uint32_t last_timeout_check  = 0;
    uint32_t last_sensor_publish = 0;
    uint32_t last_stats_print    = 0;
    uint32_t sensor_packet_count = 0;

#if ROVER_MQTT_DIAGNOSTICS
    printf("[NETDIAG] Diagnostic build enabled: 2-second USB reports; normal "
           "navigation and timeouts retained.\n");
#endif

    while (true)
    {
#if ROVER_MQTT_DIAGNOSTICS
        mqtt_diagnostics_loop_begin();
#endif
        service_safety_stop();
        camera_uart_poll(); // Decode UART1; selection below never commands
                            // motors.
#if ROVER_MQTT_DIAGNOSTICS
        mqtt_diagnostics_poll_begin();
#endif
        cyw43_arch_poll();
#if ROVER_MQTT_DIAGNOSTICS
        mqtt_diagnostics_poll_end();
#endif
        service_safety_stop();
        service_heading_zero();
        uint32_t now             = to_ms_since_boot(get_absolute_time());
        uint32_t mission_last_rx = mqtt_last_link_rx_ms();

        bool b_mission_link_ok =
            mission_last_rx != 0 &&
            (uint32_t) (now - mission_last_rx) <= LINK_TIMEOUT_MS;


        if (now - last_timeout_check >= 100)
        {
            sys_check_timeouts();
            last_timeout_check = now;
        }

        if (uart_packet_available())
        {
            if (uart_get_packet(&g_latest_sensors))
            {
                g_b_sensor_data_valid   = true;
                g_last_sensor_packet_ms = to_ms_since_boot(get_absolute_time());
                sensor_packet_count++;
            }
        }

        now = to_ms_since_boot(get_absolute_time());
        // Admit due telemetry before optional display reports and large
        // snapshots.
        if (g_b_sensor_data_valid &&
            (now - last_sensor_publish >= SENSOR_TELEMETRY_INTERVAL_MS))
        {
            publish_sensor_data();
            last_sensor_publish = now;
        }
        mqtt_service_status_queue();
        mission_tick(&g_mission, motor_safety_is_latched(), b_mission_link_ok,
                     now);
        service_camera_map(now);
        service_onboard_dfs(now);
        service_target_selection();
        service_onboard_drive(now);
        report_mission(false);
        service_camera_relay();
        service_map_publish();

        // --- Proximity buzzer (disabled during autonomous) ---
        if (!g_b_autonomous_mode && g_b_sensor_data_valid)
        {
            // Find closest obstacle from all 4 ultrasonic sensors
            int closest = g_latest_sensors.ultrasonic_front;
            if (g_latest_sensors.ultrasonic_back > 0 &&
                g_latest_sensors.ultrasonic_back < closest)
            {
                closest = g_latest_sensors.ultrasonic_back;
            }
            if (g_latest_sensors.ultrasonic_left > 0 &&
                g_latest_sensors.ultrasonic_left < closest)
            {
                closest = g_latest_sensors.ultrasonic_left;
            }
            if (g_latest_sensors.ultrasonic_right > 0 &&
                g_latest_sensors.ultrasonic_right < closest)
            {
                closest = g_latest_sensors.ultrasonic_right;
            }

            // Proximity beeping logic (car-like behavior)
            if (closest <= 30)
            {
                // Very close (< 25cm) - continuous beep (25% duty cycle for
                // softer sound)
                pwm_set_chan_level(g_buzzer_slice_num,
                                   pwm_gpio_to_channel(BUZZER_PIN), 3277);
            }
            else if (closest < 45)
            {
                // Close (< 40cm) - beep every 1 second
                if (now - g_last_beep_time >= 500)
                {
                    g_b_beep_state = !g_b_beep_state;
                    pwm_set_chan_level(
                        g_buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN),
                        g_b_beep_state ? 3277 : 0); // 25% duty or off
                    g_last_beep_time = now;
                }
            }
            else
            {
                // Safe - buzzer off
                pwm_set_chan_level(g_buzzer_slice_num,
                                   pwm_gpio_to_channel(BUZZER_PIN), 0);
                g_b_beep_state = false;
            }
        }
        else if (g_b_autonomous_mode)
        {
            // Disable buzzer during autonomous mode
            pwm_set_chan_level(g_buzzer_slice_num,
                               pwm_gpio_to_channel(BUZZER_PIN), 0);
            g_b_beep_state = false;
        }

        // IMU-tied arm PID removed (manual control only)

        // Subsystem 6: tell the mapping side once when an auto move ends
        {
            motor_auto_status_t st = motor_auto_status();
            if (st != g_last_auto_reported && st != MOTOR_AUTO_RUNNING &&
                st != MOTOR_AUTO_IDLE)
            {
                char msg[48];
                snprintf(msg, sizeof(msg), "AUTO_RESULT: %s",
                         motor_auto_status_name(st));
                printf("[AUTO] %s\n", msg);
                mqtt_publish_status(msg);
            }
            g_last_auto_reported = st;
        }

        if (now - last_stats_print >= 5000)
        {
            uint32_t total  = 0;
            uint32_t valid  = 0;
            uint32_t errors = 0;
            uart_get_stats(&total, &valid, &errors);

            printf("[STATS] Packets: %" PRIu32 " total, %" PRIu32
                   " valid, %" PRIu32 " errors | ",
                   total, valid, errors);

            if (g_b_sensor_data_valid)
            {
                printf("Heading=%.1f° Pitch=%.1f°", g_latest_sensors.heading,
                       g_latest_sensors.pitch);
                printf(" | Ultrasonic: F=%dcm B=%dcm L=%dcm R=%dcm",
                       g_latest_sensors.ultrasonic_front,
                       g_latest_sensors.ultrasonic_back,
                       g_latest_sensors.ultrasonic_left,
                       g_latest_sensors.ultrasonic_right);
            }
            else
            {
                printf("No sensor data");
            }
            printf("\n");
            last_stats_print = now;
        }
        // FR-08: stop on our own if the laptop goes quiet
        {
            uint32_t last_rx = mqtt_last_link_rx_ms();

            bool b_link_ok = (last_rx != 0) &&
                             ((int32_t) (now - last_rx) <= LINK_TIMEOUT_MS);


            if (!b_link_ok && !g_b_link_lost)
            {
                g_b_link_lost       = true;
                g_b_autonomous_mode = false;
                g_autonomous_state  = AUTO_STATE_IDLE;
                motor_emergency_stop();
                printf("[LINK] LOST at t=%" PRIu32
                       " ms (last laptop msg at t=%" PRIu32
                       " ms), MOTORS STOPPED\n",
                       (uint32_t) now, (uint32_t) last_rx);
                mqtt_publish_status("STOPPED: LINK LOST");
            }
            else if (b_link_ok && g_b_link_lost)
            {
                g_b_link_lost = false;
                printf("[LINK] Laptop link up\n");
            }
        }

        // FR-12: if the broker connection died, retry every 3 s and resubscribe
        {
            static uint32_t last_reconnect_try = 0;
            static bool     b_need_resubscribe = false;

            if (!mqtt_is_connected())
            {
                if (now - last_reconnect_try >= 3000)
                {
                    last_reconnect_try = now;
                    printf("[MQTT] Connection down, trying to reconnect...\n");
                    mqtt_connect();
                    b_need_resubscribe = true;
                }
            }
            else if (b_need_resubscribe)
            {
                b_need_resubscribe = false;
                if (mqtt_subscribe_commands(process_command) == WIFI_OK)
                {
                    printf("[MQTT] Reconnected and resubscribed\n");
                    mqtt_publish_status("RECONNECTED");
                }
            }
        }
#if ROVER_MQTT_DIAGNOSTICS
        mqtt_diagnostics_report(g_b_sensor_data_valid
                                    ? to_ms_since_boot(get_absolute_time()) -
                                          g_last_sensor_packet_ms
                                    : UINT32_MAX);
#endif
        sleep_ms(10);
    }
    return 0;
}

/* End of file. */
