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
#include "uart_protocol.h"
#include "camera_uart.h"
#include "camera_telemetry.h"
#include "mission_control.h"
#include "rover_map.h"
#include "lwip/timeouts.h"
#include "lwip/apps/mqtt.h"

// UART configuration
#define SENSOR_UART_ID uart0
#define SENSOR_UART_BAUD 115200
#define SENSOR_UART_RX_PIN 13
#define SENSOR_UART_TX_PIN -1
#define SENSOR_TELEMETRY_INTERVAL_MS 500u // 10 Hz feedback for laptop navigation.

// Buzzer configuration (piezo buzzer - PWM)
#define BUZZER_PIN 22
#define BUZZER_FREQ_HZ 2500 // 2.5kHz tone

// Latest sensor data
static sensor_packet_t latest_sensors;
static bool sensor_data_valid = false;
static uint32_t last_sensor_packet_ms = 0; // Local receive time, not Sensor Pico time.

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

static bool autonomous_mode = false;
static auto_state_t autonomous_state = AUTO_STATE_IDLE;
static uint32_t state_start_time = 0;
static int turn_direction = 0;           // -1 = left, 1 = right
static uint32_t last_auto_motor_cmd = 0; // Track last motor command for keep-alive
// Forward boost handling
static uint32_t forward_boost_start_time = 0; // when forward boost started
static bool forward_boost_active = false;     // whether boost is active
// Track latest control command time 
static uint32_t last_command_time_ms = 0;

// Proximity buzzer state
static uint32_t last_beep_time = 0;
static bool beep_state = false;
static uint buzzer_slice_num;

// Turn memory and dead-end detection
static int last_turn_direction = 0;               // Remember last turn to bias against reversing
static uint32_t last_successful_forward_time = 0; // Track when we last moved forward successfully
static uint32_t entered_moving_forward_time = 0;  // When we entered MOVING_FORWARD state
static bool safety_lock_reported = false;
static bool link_lost = true; // FR-08: true until the first laptop heartbeat
static motor_auto_status_t last_auto_reported = MOTOR_AUTO_IDLE;
static bool heading_zero_pending;
static uint32_t heading_zero_ready_ms;
static mission_t mission;
static rover_map_t rover_map;
static char map_last_request[33];
static uint8_t last_start_marker;
static bool map_publish_requested;
_Static_assert(ROVER_MAP_JSON_CAPACITY + 32u < MQTT_OUTPUT_RINGBUF_SIZE,
               "Increase MQTT ring or add snapshot chunking for this map capacity");

static void service_map_publish(void)
{
    static char payload[ROVER_MAP_JSON_CAPACITY];
    static uint32_t last_attempt_ms, last_sent_ms, sent_revision;
    static bool sent;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!mqtt_is_connected() || (uint32_t)(now - last_attempt_ms) < 100u) return;
    if (sent && !map_publish_requested && sent_revision == rover_map.revision &&
        (uint32_t)(now - last_sent_ms) < 2000u) return;
    last_attempt_ms = now;
    if (!rover_map_format(&rover_map, map_last_request, payload, sizeof(payload))) return;
    if (mqtt_publish_map(payload) == WIFI_OK) {
        sent = true;
        sent_revision = rover_map.revision;
        last_sent_ms = now;
        map_publish_requested = false;
    }
}

static void map_note_translation(const char *command, motor_error_t result)
{
    if (result != MOTOR_OK) return;
    const char *translations[] = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "FRONT_LEFT", "FRONT_RIGHT",
                                  "BACK_LEFT", "BACK_RIGHT", "MOVE_FORWARD", "MOVE_BACKWARD"};
    for (size_t i = 0; i < sizeof(translations) / sizeof(translations[0]); ++i)
        if (strcmp(command, translations[i]) == 0) { rover_map_depart(&rover_map); return; }
}

static void service_camera_relay(void)
{
    static uint32_t last_attempt_ms, last_sent_ms, sent_generation;
    static bool sent;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!mqtt_is_connected() || (uint32_t)(now - last_attempt_ms) < 100u) return;
    camera_observation_t observation;
    if (!camera_uart_latest(&observation)) return;
    if (sent && observation.generation == sent_generation &&
        (uint32_t)(now - last_sent_ms) < 1000u) return;
    last_attempt_ms = now;
    char payload[768];
    if (!camera_telemetry_format(payload, sizeof(payload), &observation.frame,
                                 observation.received_ms, now)) return;
    if (mqtt_publish_camera(payload) == WIFI_OK) {
        sent = true;
        sent_generation = observation.generation;
        last_sent_ms = now;
    }
    // Backpressure: retry the latest observation later; never queue old frames.
}

static void report_mission(bool force)
{
    static uint32_t serial_revision = UINT32_MAX;
    static uint32_t last_publish_ms;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    bool changed = serial_revision != mission.revision;
    if (!force && !changed && (uint32_t)(now - last_publish_ms) < 1000u) return;
    char status[192];
    snprintf(status, sizeof(status),
             "MISSION_STATE:%s test=%u motion=disabled reason=%s revision=%lu",
             mission_state_name(mission.state), (unsigned)mission.test_mode,
             mission_reason_name(mission.reason), (unsigned long)mission.revision);
    if (force || changed) {
        printf("[MISSION] %s\n", status);
        serial_revision = mission.revision;
    }
    if (mqtt_is_connected()) mqtt_publish_status(status);
    last_publish_ms = now;
}

static void stop_mission_outputs(void)
{
    autonomous_mode = false;
    autonomous_state = AUTO_STATE_IDLE;
    forward_boost_active = false;
    heading_zero_pending = false;
    motor_emergency_stop();
    arm_set_direction(ARM_STOP);
    claw_set_direction(CLAW_STOP);
}

static bool handle_map_command(const char *command)
{
    if (strcmp(command, "MAP_GET") == 0) { map_publish_requested = true; return true; }
    if (strncmp(command, "SET_START:", 10) != 0) return false;
    const char *token = command + 10;
    size_t length = strlen(token);
    if (length < 34 || length > 36 || token[32] != ':') goto invalid;
    for (size_t i = 0; i < 32; ++i)
        if (!strchr("0123456789abcdef", token[i])) goto invalid;
    unsigned marker_id = 0;
    for (size_t i = 33; i < length; ++i) {
        if (token[i] < '0' || token[i] > '9') goto invalid;
        marker_id = marker_id * 10u + (unsigned)(token[i] - '0');
    }
    if (!marker_id || marker_id > UINT8_MAX) goto invalid;
    char request[33];
    memcpy(request, token, 32);
    request[32] = '\0';
    if (strcmp(request, map_last_request) == 0) {
        if (last_start_marker != marker_id) { mqtt_publish_status("MAP_REFUSED:REQUEST_ID_REUSED"); return true; }
        map_publish_requested = true;
        mqtt_publish_status("MAP_ACK:SET_START_DUPLICATE");
        return true;
    }
    if (mission.state != MISSION_IDLE || motor_safety_is_latched() || heading_zero_pending ||
        (motor_motion_state() != 0 && motor_motion_state() != 1)) {
        mqtt_publish_status("MAP_REFUSED:STOP_RESET_MISSION_AND_CLEAR_SAFETY_FIRST");
        return true;
    }
    stop_mission_outputs();
    rover_map_start(&rover_map, (uint8_t)marker_id);
    memcpy(map_last_request, request, sizeof(map_last_request));
    last_start_marker = (uint8_t)marker_id;
    map_publish_requested = true;
    printf("[MAP] Start node %u at (0,0), current_node=%u, revision=%lu\n",
        marker_id, (unsigned)rover_map.current_node, (unsigned long)rover_map.revision);
    mqtt_publish_status("MAP_ACK:SET_START_ACCEPTED");
    return true;
invalid:
    mqtt_publish_status("MAP_REFUSED:EXPECTED_SET_START_REQUEST32HEX_ID1_TO_255");
    return true;
}

static bool handle_mission_command(const char *command)
{
    if (strcmp(command, "MISSION_STATUS") == 0) {
        report_mission(true);
        return true;
    }
    if (strcmp(command, "START_DFS") == 0) {
        mqtt_publish_status("MISSION_REFUSED:ONBOARD_DFS_NOT_IMPLEMENTED");
        printf("[MISSION] START_DFS refused: onboard DFS/executor not implemented\n");
        return true;
    }
    const char *event;
    if (strncmp(command, "MISSION_TEST:", 13) == 0) event = command + 13;
    else if (strcmp(command, "RESET_MISSION") == 0) event = "RESET";
    else return false;

    mission_result_t result = mission_test_event(&mission, event,
        motor_safety_is_latched(), to_ms_since_boot(get_absolute_time()));
    if (result == MISSION_ACCEPTED) stop_mission_outputs();
    char reply[96];
    snprintf(reply, sizeof(reply), "MISSION_TEST:%s state=%s",
             mission_result_name(result), mission_state_name(mission.state));
    printf("[MISSION] %s\n", reply);
    mqtt_publish_status(reply);
    report_mission(true);
    return true;
}

static void service_heading_zero(void)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!heading_zero_pending || (int32_t)(now - heading_zero_ready_ms) < 0) return;
    // Capture a new healthy sample after braking, rather than zeroing a cached
    // heading measured while the rover was still rotating.
    if (!motor_heading_sample_since(heading_zero_ready_ms)) {
        if ((uint32_t)(now - heading_zero_ready_ms) <= 1000u) return;
        heading_zero_pending = false;
        mqtt_publish_status("HEADING_ZERO_REFUSED");
        return;
    }
    heading_zero_pending = false;
    mqtt_publish_status(motor_zero_heading() == MOTOR_OK ? "HEADING_ZEROED" : "HEADING_ZERO_REFUSED");
}

static void on_sensor_packet(const sensor_packet_t *packet, uint32_t receive_ms)
{
    // Runs in the UART ISR: no MQTT, printf, sleeps, or reverse-thrust braking.
    uint16_t front_mm = (uint16_t)((packet->tof_bits >> 10u) & 0x3FFu);
    bool front_fault = (packet->obstacle_flags & (2u << 6u)) != 0;
    // Read the actual range; old urgent/STOP flags may retain hysteresis.
    motor_safety_receive(front_obstacle_detected(front_mm, front_fault), receive_ms);
    motor_heading_update(packet->heading, packet->imu_status == 0); // heading hold (FR-41)
}

static bool safety_watchdog_tick(struct repeating_timer *timer)
{
    (void)timer;
    motor_safety_poll();
    return true;
}

static void service_safety_stop(void)
{
    motor_safety_poll();
    if (motor_take_front_stop())
    {
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        forward_boost_active = false;
        mission_pause(&mission, MISSION_REASON_FRONT_STOP, to_ms_since_boot(get_absolute_time()));
        mqtt_publish_status("FRONT_STOPPED");
    }
    if (motor_safety_is_latched())
    {
        mission_fault(&mission, MISSION_REASON_ESTOP, to_ms_since_boot(get_absolute_time()));
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        forward_boost_active = false;
        if (!safety_lock_reported)
        {
            safety_lock_reported = true;
            printf("[SAFETY] Manual dashboard E-STOP active\n");
            mqtt_publish_status("SAFETY_LOCKED");
        }
    }
    else
    {
        safety_lock_reported = false;
    }
}

// Autonomous navigation parameters
#define AUTO_OBSTACLE_DISTANCE_MM 600.0f // Stop if obstacle within 600mm
#define AUTO_FORWARD_SECTOR_MIN 160.0f   // Front sector: 160° to 200° (40° cone)
#define AUTO_FORWARD_SECTOR_MAX 200.0f
#define AUTO_LEFT_SECTOR_MIN 90.0f // Left sector: 90° to 150° (60° cone)
#define AUTO_LEFT_SECTOR_MAX 150.0f
#define AUTO_RIGHT_SECTOR_MIN 210.0f // Right sector: 210° to 270° (60° cone)
#define AUTO_RIGHT_SECTOR_MAX 270.0f
#define AUTO_TURN_DURATION_MS 400            // Turn execution time (400ms)
#define AUTO_SCAN_DURATION_MS 2500           // Gap detection scan period (2500ms)
#define AUTO_TURN_BIAS_PENALTY_MM 300.0f     // Penalty to avoid turn direction oscillation (300mm)
#define AUTO_DEAD_END_TIME_MS 2000           // Obstacle hit within 2000ms = dead end
#define AUTO_SUCCESSFUL_FORWARD_TIME_MS 5000 // Clear turn bias after 5000ms forward progress
// Gap-aware navigation thresholds
#define AUTO_FREE_DISTANCE_MM 1000.0f      // Ray considered free if >= 1000mm
#define AUTO_MIN_FORWARD_CLEAR_RATIO 0.40f // Require 40% of front rays free to resume

// Pitch compensation for upslope detection
#define PITCH_BASELINE 0.0f       // MPU6500 mounted +X forward, +Z up: flat = 0 degrees
#define PITCH_BOOST_PER_15_DEG 10 // Speed boost: +10% per 15 degrees nose-up
#define PITCH_MAX_BOOST 30        // Maximum boost capped at +30%

static int apply_pitch_compensation(int base_speed, float current_pitch)
{
    // Sketch convention with +X forward/+Z up: nose-up pitch is negative.
    float pitch_drop = PITCH_BASELINE - current_pitch;

    if (pitch_drop > 0)
    {
        // Apply boost: every 15 degrees nose-up adds 10% speed
        int boost_percent = (int)(pitch_drop / 15.0f) * PITCH_BOOST_PER_15_DEG;

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

static void process_command(const char *command)
{
    printf("[CMD] Processing: %s\n", command);

    motor_error_t result = MOTOR_OK;

    // Dashboard keys: one word per move (e.g. "FORWARD"). Anything after a
    // ':' (old speed / wheel values) is ignored; each move has its own speed
    // in motor_config.h (MC_DASH_*_PCT).
    char cmd_buffer[32];
    strncpy(cmd_buffer, command, sizeof(cmd_buffer) - 1);
    cmd_buffer[sizeof(cmd_buffer) - 1] = '\0';
    char *colon = strchr(cmd_buffer, ':');
    if (colon != NULL)
        *colon = '\0';

    service_safety_stop();
    if (handle_map_command(command)) return;
    if (handle_mission_command(command)) return;
    if (!mission_command_allowed(&mission, command)) {
        mqtt_publish_status("MISSION_REFUSED:TEST_MODE_MOVEMENT_DISABLED");
        return;
    }
    if (strcmp(cmd_buffer, "ZERO_HEADING") == 0)
    {
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        motor_emergency_stop();
        heading_zero_pending = true;
        heading_zero_ready_ms = to_ms_since_boot(get_absolute_time()) + 400u;
        mqtt_publish_status("HEADING_ZERO_PENDING");
        return;
    }
    if (heading_zero_pending)
    {
        if (strcmp(cmd_buffer, "STOP") == 0 || strcmp(cmd_buffer, "ESTOP") == 0 ||
            strcmp(cmd_buffer, "RESET_SAFETY") == 0 || strcmp(cmd_buffer, "AUTO_STOP") == 0) {
            heading_zero_pending = false;
            mqtt_publish_status("HEADING_ZERO_CANCELLED");
        } else {
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
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        if (motor_safety_reset())
        {
            safety_lock_reported = false;
            mqtt_publish_status("SAFETY_READY");
            printf("[SAFETY] Reset accepted; a new movement command is required\n");
        }
        else
        {
            mqtt_publish_status("SAFETY_RESET_REFUSED");
            printf("[SAFETY] Reset refused: missing, stale, or unsafe sensor packet\n");
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
        int compensated_speed = (sensor_data_valid && latest_sensors.imu_status == 0)
                                    ? apply_pitch_compensation(speed, latest_sensors.pitch) : speed;
        if (compensated_speed != speed)
        {
            char msg[128];
            snprintf(msg, sizeof(msg), "PITCH: Upslope %.1f° - boost %d%% -> %d%%",
                     latest_sensors.pitch, speed, compensated_speed);
            printf("[%s]\n", msg);
            mqtt_publish_status(msg);
        }
        result = motor_set_direction(DIRECTION_FORWARD, compensated_speed);
    }
    else if (strncmp(cmd_buffer, "BACKWARD", 8) == 0)
    {
        result = motor_set_direction(DIRECTION_BACKWARD, motor_move_speed(DIRECTION_BACKWARD));
    }
    else if (strncmp(cmd_buffer, "LEFT", 4) == 0)
    {
        result = motor_set_direction(DIRECTION_LEFT, motor_move_speed(DIRECTION_LEFT));
    }
    else if (strncmp(cmd_buffer, "RIGHT", 5) == 0)
    {
        result = motor_set_direction(DIRECTION_RIGHT, motor_move_speed(DIRECTION_RIGHT));
    }
    else if (strncmp(cmd_buffer, "ROTATE_LEFT", 11) == 0)
    {
        result = motor_set_direction(DIRECTION_ROTATE_LEFT, motor_move_speed(DIRECTION_ROTATE_LEFT));
    }
    else if (strncmp(cmd_buffer, "ROTATE_RIGHT", 12) == 0)
    {
        result = motor_set_direction(DIRECTION_ROTATE_RIGHT, motor_move_speed(DIRECTION_ROTATE_RIGHT));
    }
    else if (strncmp(cmd_buffer, "FRONT_LEFT", 10) == 0)
    {
        result = motor_set_direction(DIRECTION_FRONT_LEFT, motor_move_speed(DIRECTION_FRONT_LEFT));
    }
    else if (strncmp(cmd_buffer, "FRONT_RIGHT", 11) == 0)
    {
        result = motor_set_direction(DIRECTION_FRONT_RIGHT, motor_move_speed(DIRECTION_FRONT_RIGHT));
    }
    else if (strncmp(cmd_buffer, "BACK_LEFT", 9) == 0)
    {
        result = motor_set_direction(DIRECTION_BACK_LEFT, motor_move_speed(DIRECTION_BACK_LEFT));
    }
    else if (strncmp(cmd_buffer, "BACK_RIGHT", 10) == 0)
    {
        result = motor_set_direction(DIRECTION_BACK_RIGHT, motor_move_speed(DIRECTION_BACK_RIGHT));
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
    // ---- Autonomous moves for the mapping / Exploration side (Subsystem 6) ----
    // One command starts the move; it keeps going until MOVE_STOP, STOP
    // (operator) or a safety stop. Rotations end by themselves.
    //   MOVE_FORWARD:30   MOVE_BACKWARD:30   (speed %, default = W/S key speed)
    //   TURN_TO:90        absolute heading 0..359
    //   TURN_BY:-90       relative: + right, - left
    //   MOVE_STOP         normal stop (ramp down)
    // Replies on robot/status: "AUTO_MOVE: ..." now, "AUTO_RESULT: ..." when it ends.
    else if (strncmp(cmd_buffer, "MOVE_FORWARD", 12) == 0 ||
             strncmp(cmd_buffer, "MOVE_BACKWARD", 13) == 0 ||
             strncmp(cmd_buffer, "TURN_TO", 7) == 0 ||
             strncmp(cmd_buffer, "TURN_BY", 7) == 0 ||
             strncmp(cmd_buffer, "MOVE_STOP", 9) == 0) {
        const char *arg = strchr(command, ':');
        bool has_arg = arg != NULL && arg[1] != '\0';
        int value = has_arg ? atoi(arg + 1) : 0;
        autonomous_mode = false;           // the old LIDAR auto mode must not drive too
        autonomous_state = AUTO_STATE_IDLE;

        if (strncmp(cmd_buffer, "MOVE_FORWARD", 12) == 0)
            result = motor_auto_drive(has_arg ? abs(value) : motor_move_speed(DIRECTION_FORWARD));
        else if (strncmp(cmd_buffer, "MOVE_BACKWARD", 13) == 0)
            result = motor_auto_drive(-(has_arg ? abs(value) : motor_move_speed(DIRECTION_BACKWARD)));
        else if (strncmp(cmd_buffer, "TURN_TO", 7) == 0)
            result = has_arg ? motor_auto_rotate_to(value) : MOTOR_ERR_PIN;
        else if (strncmp(cmd_buffer, "TURN_BY", 7) == 0)
            result = has_arg ? motor_auto_rotate_by(value) : MOTOR_ERR_PIN;
        else
        {
            mission_pause(&mission, MISSION_REASON_OPERATOR_STOP, to_ms_since_boot(get_absolute_time()));
            result = motor_auto_stop();
        }
        map_note_translation(cmd_buffer, result);

        char msg[80];
        static char last_move[32] = "";
        if (result == MOTOR_OK && strncmp(cmd_buffer, "MOVE_", 5) == 0 &&
            strcmp(last_move, command) == 0 && motor_auto_status() == MOTOR_AUTO_RUNNING) {
            return;   // the laptop repeats MOVE_FORWARD: still running, no new reply
        }
        strncpy(last_move, command, sizeof(last_move) - 1);
        if (result == MOTOR_OK) {
            // Also report consecutive short turns that finish between loop polls.
            last_auto_reported = MOTOR_AUTO_RUNNING;
            snprintf(msg, sizeof(msg), "AUTO_MOVE: %s", command);
        }
        else if (result == MOTOR_ERR_FRONT_BLOCKED) {
            last_move[0] = '\0';
            snprintf(msg, sizeof(msg), "FORWARD_BLOCKED");   // same reply as the W key
        }
        else if (result == MOTOR_ERR_NO_HEADING)
            snprintf(msg, sizeof(msg), "AUTO_REFUSED: %s (no fresh heading from IMU)", cmd_buffer);
        else if (result == MOTOR_ERR_PIN)
            snprintf(msg, sizeof(msg), "AUTO_REFUSED: %s (bad or missing value)", cmd_buffer);
        else
            snprintf(msg, sizeof(msg), "AUTO_REFUSED: %s (safety lock / SAFE STOP / busy)", cmd_buffer);
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
        mission_pause(&mission, MISSION_REASON_OPERATOR_STOP, to_ms_since_boot(get_absolute_time()));
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        motor_emergency_stop();
        printf("[AUTO] Autonomous mode DISABLED\n");
        mqtt_publish_status("AUTO_DISABLED");
        return;
    }
    else if (strncmp(cmd_buffer, "STOP", 4) == 0)
    {
        mission_pause(&mission, MISSION_REASON_OPERATOR_STOP, to_ms_since_boot(get_absolute_time()));
        autonomous_mode = false; // Stop also disables auto
        autonomous_state = AUTO_STATE_IDLE;
        result = motor_emergency_stop();
        mqtt_publish_status("STOPPED");
        return;
    }
    else
    {
        printf("[CMD] Unknown command: %s\n", cmd_buffer);
        return;
    }
    printf("[CMD] %s -> result %d (motion state %d)\n",
           cmd_buffer, (int)result, motor_motion_state());
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
    last_command_time_ms = to_ms_since_boot(get_absolute_time());
}

static void publish_sensor_data(void)
{

    if (!sensor_data_valid)
        return;

    static char telemetry[2048]; // ← Reduced size since no LIDAR

    memset(telemetry, 0, sizeof(telemetry));

    snprintf(telemetry, sizeof(telemetry),
             "{"
             "\"timestamp\":%lu,"
             "\"heading_epoch\":%lu,\"heading_offset_deg\":%.3f,"
             "\"mission\":{\"state\":\"%s\",\"reason\":\"%s\",\"test_mode\":%s,"
             "\"motion_enabled\":false,\"revision\":%lu},"
             "\"imu\":{"
             "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"heading\":%.1f,"
             "\"pitch\":%.1f,"
             "\"roll\":%.1f,"
             "\"heading_reference\":\"%s\","
             "\"status\":%d"
             "},"
             "\"ultrasonic\":{"
             "\"front\":%d,"
             "\"back\":%d,"
             "\"left\":%d,"
             "\"right\":%d,"
             "\"status\":%d"
             "},"
             "\"obstacle\":{"
             "\"distance_mm\":{\"left\":%u,\"front\":%u,\"right\":%u},"
             "\"blocked_mask\":%u,"
             "\"urgent_mask\":%u,"
             "\"fault_mask\":%u,"
             "\"stop_required\":%s,"
             "\"age_ms\":%lu,"
             "\"forward_threshold_mm\":%u"
             "}"
             "}",
             (unsigned long)latest_sensors.timestamp_ms,
             (unsigned long)motor_heading_epoch(), motor_heading_offset(),
             mission_state_name(mission.state), mission_reason_name(mission.reason),
             mission.test_mode ? "true" : "false", (unsigned long)mission.revision,
             latest_sensors.accel_x, latest_sensors.accel_y, latest_sensors.accel_z,
             latest_sensors.gyro_x, latest_sensors.gyro_y, latest_sensors.gyro_z,
             motor_heading_relative(latest_sensors.heading), latest_sensors.pitch, latest_sensors.roll,
             motor_heading_epoch() ? "manual_zero" : "startup", latest_sensors.imu_status,
             latest_sensors.ultrasonic_front, latest_sensors.ultrasonic_back,
             latest_sensors.ultrasonic_left, latest_sensors.ultrasonic_right,
             latest_sensors.ultrasonic_status,
             // (unsigned)latest_sensors.tof_mm[0],
             // (unsigned)latest_sensors.tof_mm[1],
             // (unsigned)latest_sensors.tof_mm[2],
             (unsigned)(latest_sensors.tof_bits & 0x3FFu),          // Left (bit 0-9)
             (unsigned)((latest_sensors.tof_bits >> 10u) & 0x3FFu), // Front (bit 10-19)
             (unsigned)((latest_sensors.tof_bits >> 20u) & 0x3FFu), // Right (bit 20-29)
            //  (unsigned)latest_sensors.blocked_mask,
            //  (unsigned)latest_sensors.urgent_mask,
            //  (unsigned)latest_sensors.fault_mask,
            //  latest_sensors.stop_required ? "true" : "false",
            (unsigned)(latest_sensors.obstacle_flags & 0x07u),
            (unsigned)((latest_sensors.obstacle_flags >> 3u) & 0x07u),
            (unsigned)((latest_sensors.obstacle_flags >> 6u) & 0x07u),
            (latest_sensors.obstacle_flags & (1u << 9u)) ? "true" : "false",
             (unsigned long)(to_ms_since_boot(get_absolute_time()) - last_sensor_packet_ms),
             (unsigned)FRONT_STOP_DISTANCE_MM);

    mqtt_publish_telemetry(telemetry);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    mission_init(&mission, to_ms_since_boot(get_absolute_time()));
    rover_map_init(&rover_map, get_rand_64());
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
    uart_receiver_init(SENSOR_UART_ID, SENSOR_UART_BAUD, SENSOR_UART_RX_PIN, SENSOR_UART_TX_PIN);
    camera_uart_init();

    // Initialize piezo buzzer with PWM
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Set PWM frequency to 2.5kHz
    float clock_freq = 125000000.0f;                          // Pico system clock
    uint32_t divider = clock_freq / (BUZZER_FREQ_HZ * 65536); // 16-bit wrap
    if (divider < 1)
        divider = 1;
    pwm_set_clkdiv(buzzer_slice_num, (float)divider);
    pwm_set_wrap(buzzer_slice_num, 65535);                                    // 16-bit resolution
    pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 0); // Start off
    pwm_set_enabled(buzzer_slice_num, true);
    printf("[BUZZER] PWM initialized on GP%d at %dHz\n", BUZZER_PIN, BUZZER_FREQ_HZ);

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

    uint32_t last_timeout_check = 0;
    uint32_t last_sensor_publish = 0;
    uint32_t last_stats_print = 0;
    uint32_t sensor_packet_count = 0;

    while (true)
    {
        service_safety_stop();
        camera_uart_poll(); // UART1 diagnostics only; no navigation/motor actions.
        cyw43_arch_poll();
        service_safety_stop();
        service_camera_relay();
        service_map_publish();
        service_heading_zero();
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint32_t mission_last_rx = mqtt_last_link_rx_ms();
        bool mission_link_ok = mission_last_rx != 0 &&
            (uint32_t)(now - mission_last_rx) <= LINK_TIMEOUT_MS;
        mission_tick(&mission, motor_safety_is_latched(), mission_link_ok, now);
        report_mission(false);

        if (now - last_timeout_check >= 100)
        {
            sys_check_timeouts();
            last_timeout_check = now;
        }

        if (uart_packet_available())
        {
            if (uart_get_packet(&latest_sensors))
            {
                sensor_data_valid = true;
                last_sensor_packet_ms = to_ms_since_boot(get_absolute_time());
                sensor_packet_count++;
            }
        }

        // --- Proximity buzzer (disabled during autonomous) ---
        if (!autonomous_mode && sensor_data_valid)
        {
            // Find closest obstacle from all 4 ultrasonic sensors
            int closest = latest_sensors.ultrasonic_front;
            if (latest_sensors.ultrasonic_back > 0 && latest_sensors.ultrasonic_back < closest)
            {
                closest = latest_sensors.ultrasonic_back;
            }
            if (latest_sensors.ultrasonic_left > 0 && latest_sensors.ultrasonic_left < closest)
            {
                closest = latest_sensors.ultrasonic_left;
            }
            if (latest_sensors.ultrasonic_right > 0 && latest_sensors.ultrasonic_right < closest)
            {
                closest = latest_sensors.ultrasonic_right;
            }

            // Proximity beeping logic (car-like behavior)
            if (closest <= 30)
            {
                // Very close (< 25cm) - continuous beep (25% duty cycle for softer sound)
                pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 3277);
            }
            else if (closest < 45)
            {
                // Close (< 40cm) - beep every 1 second
                if (now - last_beep_time >= 500)
                {
                    beep_state = !beep_state;
                    pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN),
                                       beep_state ? 3277 : 0); // 25% duty or off
                    last_beep_time = now;
                }
            }
            else
            {
                // Safe - buzzer off
                pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 0);
                beep_state = false;
            }
        }
        else if (autonomous_mode)
        {
            // Disable buzzer during autonomous mode
            pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 0);
            beep_state = false;
        }

        // IMU-tied arm PID removed (manual control only)

        // Subsystem 6: tell the mapping side once when an auto move ends
        {
            motor_auto_status_t st = motor_auto_status();
            if (st != last_auto_reported && st != MOTOR_AUTO_RUNNING && st != MOTOR_AUTO_IDLE) {
                char msg[48];
                snprintf(msg, sizeof(msg), "AUTO_RESULT: %s", motor_auto_status_name(st));
                printf("[AUTO] %s\n", msg);
                mqtt_publish_status(msg);
            }
            last_auto_reported = st;
        }

        // --- Sensor & status publishing ---
        if (sensor_data_valid && (now - last_sensor_publish >= SENSOR_TELEMETRY_INTERVAL_MS))
        {
            publish_sensor_data();
            last_sensor_publish = now;
        }

        if (now - last_stats_print >= 5000)
        {
            uint32_t total, valid, errors;
            uart_get_stats(&total, &valid, &errors);

            printf("[STATS] Packets: %lu total, %lu valid, %lu errors | ",
                   total, valid, errors);

            if (sensor_data_valid)
            {
                printf("Heading=%.1f° Pitch=%.1f°", latest_sensors.heading, latest_sensors.pitch);
                printf(" | Ultrasonic: F=%dcm B=%dcm L=%dcm R=%dcm",
                       latest_sensors.ultrasonic_front,
                       latest_sensors.ultrasonic_back,
                       latest_sensors.ultrasonic_left,
                       latest_sensors.ultrasonic_right);
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
            bool link_ok = (last_rx != 0) &&
                           ((int32_t)(now - last_rx) <= LINK_TIMEOUT_MS);

            if (!link_ok && !link_lost)
            {
                link_lost = true;
                autonomous_mode = false;
                autonomous_state = AUTO_STATE_IDLE;
                motor_emergency_stop();
                printf("[LINK] LOST at t=%lu ms (last laptop msg at t=%lu ms), MOTORS STOPPED\n",
                       (unsigned long)now, (unsigned long)last_rx);
                mqtt_publish_status("STOPPED: LINK LOST");
            }
            else if (link_ok && link_lost)
            {
                link_lost = false;
                printf("[LINK] Laptop link up\n");
            }
        }

        // FR-12: if the broker connection died, retry every 3 s and resubscribe
        {
            static uint32_t last_reconnect_try = 0;
            static bool need_resubscribe = false;

            if (!mqtt_is_connected())
            {
                if (now - last_reconnect_try >= 3000)
                {
                    last_reconnect_try = now;
                    printf("[MQTT] Connection down, trying to reconnect...\n");
                    mqtt_connect();
                    need_resubscribe = true;
                }
            }
            else if (need_resubscribe)
            {
                need_resubscribe = false;
                if (mqtt_subscribe_commands(process_command) == WIFI_OK)
                {
                    printf("[MQTT] Reconnected and resubscribed\n");
                    mqtt_publish_status("RECONNECTED");
                }
            }
        }
        sleep_ms(10);
    }
    return 0;
}
