#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "config.h"
#include "motor_driver.h"
#include "../../common/obstacle_threshold.h"
#include "wifi_mqtt.h"
#include "uart_protocol.h"
#include "camera_uart.h"
#include "onboard_dfs.h"
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
static onboard_dfs_t onboard;
static char map_message[ONBOARD_SNAPSHOT_BYTES];
_Static_assert(ONBOARD_SNAPSHOT_BYTES+32u<=MQTT_OUTPUT_RINGBUF_SIZE,"Map snapshot must fit MQTT ring");
_Static_assert(MEM_SIZE>=MQTT_OUTPUT_RINGBUF_SIZE+2u*TCP_SND_BUF+2048u,"Reserve network heap beyond MQTT ring");
static bool map_requested=true, mapping_reference_invalid;
static char command_queue[4][256];
static unsigned command_head, command_count;
static bool command_stop, command_estop;
static volatile bool sensor_source_seen, sensor_restart_pending;
static volatile uint32_t sensor_source_stamp;


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
    if (!sensor_source_seen || packet->timestamp_ms != sensor_source_stamp) {
        if(sensor_source_seen && (int32_t)(packet->timestamp_ms-sensor_source_stamp)<0) sensor_restart_pending=true;
        sensor_source_stamp=packet->timestamp_ms;sensor_source_seen=true;
        motor_heading_update(packet->heading,packet->imu_status==0);
    }
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
        if(onboard_dfs_busy(&onboard))onboard_dfs_fault(&onboard,"FRONT_OBSTACLE_STOP",to_ms_since_boot(get_absolute_time()));
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        forward_boost_active = false;
        mqtt_publish_status("FRONT_STOPPED");
    }
    if (motor_safety_is_latched())
    {
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
            result = motor_auto_stop();

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
        mqtt_publish_status("USE_DFS_START_COUNT_THRESHOLD");
        return;
    }
    else if (strncmp(cmd_buffer, "AUTO_STOP", 9) == 0)
    {
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        motor_emergency_stop();
        printf("[AUTO] Autonomous mode DISABLED\n");
        mqtt_publish_status("AUTO_DISABLED");
        return;
    }
    else if (strncmp(cmd_buffer, "STOP", 4) == 0)
    {
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


static bool onboard_drive(void *ctx,int speed) {(void)ctx;return motor_auto_drive(speed)==MOTOR_OK;}
static bool onboard_turn(void *ctx,int heading) {(void)ctx;return motor_auto_rotate_to(heading)==MOTOR_OK;}
static bool onboard_turn_done(void *ctx) {(void)ctx;return motor_auto_status()==MOTOR_AUTO_HEADING_REACHED;}
static void onboard_stop_motor(void *ctx) {(void)ctx;motor_emergency_stop();}
static const onboard_motor_io_t onboard_io={NULL,onboard_drive,onboard_turn,onboard_turn_done,onboard_stop_motor};

static void queue_command(const char *command) {
    if(!strcmp(command,"STOP")||!strcmp(command,"DFS_STOP")||!strcmp(command,"AUTO_STOP")){command_stop=true;return;}
    if(!strcmp(command,"ESTOP")){command_estop=true;return;}
    if(command_count>=4 || strlen(command)>=sizeof(command_queue[0]))return;
    unsigned slot=(command_head+command_count)%4;
    snprintf(command_queue[slot],sizeof(command_queue[slot]),"%s",command);++command_count;
}
static bool uint_argument(const char *text,unsigned *value) {
    if(*text<'0'||*text>'9')return false;
    char *end;unsigned long n=strtoul(text,&end,10);
    if(*end||n>65535)return false;
    *value=(unsigned)n;return true;
}
static bool handle_onboard_command(const char *command,uint32_t now) {
    if(!strcmp(command,"MAP_REQUEST")){map_requested=true;return true;}
    if(!strncmp(command,"CAMERA_GEOMETRY:",16)) {
        float height,forward,down;char extra;
        bool ok=sscanf(command+16,"%f:%f:%f%c",&height,&forward,&down,&extra)==3 &&
                onboard_dfs_set_geometry(&onboard,height,forward,down,now);
        mqtt_publish_status(ok?"CAMERA_GEOMETRY_SET":"GEOMETRY_REFUSED_RESET_MAP_AND_CHECK_VALUES");map_requested=true;return true;
    }
    if(!strncmp(command,"SET_START:",10)) {
        unsigned id;
        bool ok=!heading_zero_pending&&motor_motion_state()==1&&uint_argument(command+10,&id)&&
                id<=ROVER_MAX_MARKER_ID&&onboard_dfs_set_start(&onboard,(uint8_t)id,now);
        if(ok)mapping_reference_invalid=false;
        mqtt_publish_status(ok?"START_NODE_SET":"START_REFUSED_STOP_AND_CHECK_SENSOR");map_requested=true;return true;
    }
    if(!strncmp(command,"DFS_START:",10)||!strcmp(command,"AUTO_START")) {
        unsigned count=ROVER_DEFAULT_GOAL,opening=500;bool valid=true;
        if(!strncmp(command,"DFS_START:",10)) {
            char args[64];snprintf(args,sizeof(args),"%s",command+10);char *colon=strchr(args,':');
            if(colon){*colon++=0;valid=uint_argument(colon,&opening);}
            valid=valid&&uint_argument(args,&count);
        }
        bool ok=valid&&count<=ROVER_MAX_NODES&&!mapping_reference_invalid&&!motor_safety_is_latched()&&
                onboard_dfs_start(&onboard,(uint8_t)count,(uint16_t)opening,now);
        mqtt_publish_status(ok?"ONBOARD_DFS_STARTED":!onboard.config.camera_calibrated?"DFS_REFUSED_MEASURE_AND_SET_CAMERA_GEOMETRY":"DFS_REFUSED_CHECK_START_AND_FRESH_INPUTS");
        map_requested=true;return true;
    }
    if(!strcmp(command,"DFS_RESUME")) {
        bool ok=!mapping_reference_invalid&&!motor_safety_is_latched()&&onboard_dfs_resume(&onboard,now);
        mqtt_publish_status(ok?"DFS_RESUMED":"RESUME_REFUSED_CHECK_REFERENCE_AND_INPUTS");return true;
    }
    if(!strcmp(command,"DFS_RESET")) {
        if(onboard.phase==DFS_FAULT)onboard_dfs_stop(&onboard,"FAULT_RESET_PAUSED",now);
        map_requested=true;return true;
    }
    if(!strcmp(command,"RESET_MAP")) {
        if(onboard_dfs_busy(&onboard)){mqtt_publish_status("RESET_MAP_REFUSED_STOP_FIRST");return true;}
        motor_emergency_stop();onboard_config_t config=onboard.config;
        onboard_dfs_init(&onboard,&onboard_io);onboard.config=config;
        mapping_reference_invalid=false;map_requested=true;mqtt_publish_status("MAP_RESET");return true;
    }
    return false;
}
static void dispatch_commands(uint32_t now) {
    if(command_stop||command_estop) {
        bool emergency=command_estop;command_stop=command_estop=false;command_head=command_count=0;
        heading_zero_pending=false;
        if(emergency){motor_manual_estop();onboard_dfs_fault(&onboard,"OPERATOR_ESTOP",now);}
        else {motor_emergency_stop();onboard_dfs_stop(&onboard,"OPERATOR_STOP",now);}
        mqtt_publish_status(emergency?"SAFETY_LOCKED":"STOPPED");map_requested=true;return;
    }
    while(command_count) {
        char command[256];snprintf(command,sizeof(command),"%s",command_queue[command_head]);
        command_head=(command_head+1)%4;--command_count;
        if(handle_onboard_command(command,now))continue;
        if(!strcmp(command,"RESET_SAFETY"))onboard_dfs_stop(&onboard,"SAFETY_RESET_PAUSED",now);
        if(!strncmp(command,"MOVE_",5)||!strncmp(command,"TURN_",5)) {
            mqtt_publish_status("EXTERNAL_AUTO_REFUSED_USE_ONBOARD_DFS");continue;
        }
        /* No second controller may move the chassis behind an anchored map. */
        bool chassis=!strncmp(command,"FORWARD",7)||!strncmp(command,"BACK",4)||
            !strncmp(command,"LEFT",4)||!strncmp(command,"RIGHT",5)||!strncmp(command,"FRONT_",6)||
            !strncmp(command,"ROTATE_",7)||!strncmp(command,"MOVE_",5)||!strncmp(command,"TURN_",5)||
            !strncmp(command,"ZERO_HEADING",12);
        if(chassis&&onboard.map.start){mqtt_publish_status("MANUAL_CHASSIS_REFUSED_RESET_MAP");continue;}
        process_command(command);
    }
}
static void service_camera(void) {
    camera_uart_poll();camera_packet_t packet;
    if(camera_uart_take_packet(&packet)) {
        uint32_t now=to_ms_since_boot(get_absolute_time());
        onboard_dfs_camera(&onboard,&packet.frame,packet.received_ms,now);
        static uint32_t last_print;
        if(now-last_print>=1000u) {
            const vision_frame_t *f=&packet.frame;
            printf("[CAMERA] seq=%u status=%u age=%lums N=%u",f->sequence,(unsigned)f->status,
                   (unsigned long)vision_effective_age_ms(f,packet.received_ms,now),f->count);
            for(unsigned i=0;i<f->count;++i)printf(" | id=%u range=%umm bearing=%dcdeg quality=%u",f->markers[i].id,f->markers[i].range_mm,f->markers[i].bearing_cdeg,f->markers[i].quality);
            puts("");last_print=now;
        }
    }
}
int main(void) {
    stdio_init_all();sleep_ms(2000);
    puts("\nMecanum Robot Controller - ONBOARD DFS / CAMERA UART1 GP21 [NETFIX-1]");
    if(motor_init()!=MOTOR_OK)return -1;
    onboard_dfs_init(&onboard,&onboard_io);
    struct repeating_timer safety_timer;
    if(!add_repeating_timer_ms(-10,safety_watchdog_tick,NULL,&safety_timer)){motor_manual_estop();return -1;}
    uart_set_packet_callback(on_sensor_packet);
    uart_receiver_init(SENSOR_UART_ID,SENSOR_UART_BAUD,SENSOR_UART_RX_PIN,SENSOR_UART_TX_PIN);
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


    bool network_ready=wifi_init()==WIFI_OK;
    bool mqtt_ready=network_ready&&mqtt_init(MQTT_BROKER_IP,MQTT_BROKER_PORT,MQTT_CLIENT_ID)==WIFI_OK;
    puts("[NAV] Set measured camera geometry before movement; SET_START anchors the physical starting marker.");
    uint32_t boot_now=to_ms_since_boot(get_absolute_time());
    uint32_t last_wifi=boot_now-30000u,last_mqtt=boot_now-3000u,last_publish=0,last_telemetry=0,last_timeouts=0,last_run=0;
    uint32_t published_revision=UINT32_MAX,published_state=UINT32_MAX;
    bool subscribed=false,have_sensor_stamp=false,executor_was_required=false;uint32_t sensor_stamp=0;
    puts("[RUN] Main loop started; Wi-Fi connection runs in the background.");
    while(true) {
        uint32_t now=to_ms_since_boot(get_absolute_time());
        service_safety_stop();wifi_mqtt_poll();
        if(network_ready&&now-last_timeouts>=100){sys_check_timeouts();last_timeouts=now;}
        now=to_ms_since_boot(get_absolute_time());
        if(uart_packet_available()&&uart_get_packet(&latest_sensors)) {
            bool changed=!have_sensor_stamp||latest_sensors.timestamp_ms!=sensor_stamp;
            if(changed){last_sensor_packet_ms=now;sensor_stamp=latest_sensors.timestamp_ms;have_sensor_stamp=true;}
            sensor_data_valid=true;
            onboard_sensor_t sensor={0};
            sensor.heading_deg=motor_heading_relative(latest_sensors.heading);
            sensor.imu_valid=latest_sensors.imu_status==0;
            sensor.tof_mm[0]=(uint16_t)(latest_sensors.tof_bits&1023u);
            sensor.tof_mm[1]=(uint16_t)((latest_sensors.tof_bits>>10u)&1023u);
            sensor.tof_mm[2]=(uint16_t)((latest_sensors.tof_bits>>20u)&1023u);
            sensor.tof_faults=(uint8_t)((latest_sensors.obstacle_flags>>6u)&7u);
            sensor.rear_valid=!(latest_sensors.ultrasonic_status&2u)&&latest_sensors.ultrasonic_back>0&&latest_sensors.ultrasonic_back<400;
            sensor.rear_mm=sensor.rear_valid?(uint16_t)(latest_sensors.ultrasonic_back*10u):0;
            sensor.received_ms=last_sensor_packet_ms;onboard_dfs_sensor(&onboard,&sensor);
        }
        if(sensor_restart_pending) {
            sensor_restart_pending=false;
            if(onboard.map.start){mapping_reference_invalid=true;onboard_dfs_fault(&onboard,"SENSOR_PICO_RESTARTED_REANCHOR",now);}
        }
        service_camera();now=to_ms_since_boot(get_absolute_time());
        dispatch_commands(now);service_heading_zero();
        bool was_busy=onboard_dfs_busy(&onboard);
        if(was_busy&&((executor_was_required&&motor_executor_expired())||motor_safety_is_latched()||motor_motion_state()==MOTOR_STATE_SAFE_STOP))
            onboard_dfs_fault(&onboard,"MOTOR_SAFETY_OR_EXECUTOR_STOP",now);
        onboard_dfs_tick(&onboard,now);
        executor_was_required=onboard_dfs_busy(&onboard);
        motor_executor_require(executor_was_required);
        if(onboard_dfs_busy(&onboard))motor_executor_renew();
        service_safety_stop();
        if(!onboard_dfs_busy(&onboard)) {
            uint32_t last_rx=mqtt_last_link_rx_ms();bool link_ok=last_rx&&now-last_rx<=LINK_TIMEOUT_MS;
            if(!link_ok&&!link_lost){motor_link_lost();link_lost=true;}
            else if(link_ok)link_lost=false;
        }
        if(mqtt_ready) {
            if(!wifi_is_connected()) {
                subscribed=false;
                if(now-last_wifi>=30000u){last_wifi=now;wifi_connect_async(WIFI_SSID,WIFI_PASSWORD);}
            } else if(!mqtt_is_connected()) {
                subscribed=false;
                if(now-last_mqtt>=3000u){last_mqtt=now;mqtt_connect();}
            } else {
                if(!subscribed&&mqtt_subscribe_commands(queue_command)==WIFI_OK){subscribed=true;map_requested=true;mqtt_publish_status("ONBOARD_READY_SET_START");}
                bool dirty=map_requested||published_revision!=onboard.map.revision||published_state!=onboard.state_revision;
                if((dirty&&now-last_publish>=200u)||now-last_publish>=2000u) {
                    if(onboard_dfs_snapshot(&onboard,map_message,sizeof(map_message))&&mqtt_publish_map(map_message)==WIFI_OK) {
                        published_revision=onboard.map.revision;published_state=onboard.state_revision;
                        map_requested=false;last_publish=now;
                        printf("[MAP] queued %u bytes on robot/map\n",(unsigned)strlen(map_message));
                    }
                }
                if(sensor_data_valid&&now-last_telemetry>=SENSOR_TELEMETRY_INTERVAL_MS){publish_sensor_data();last_telemetry=now;}
            }
        }
        if(now-last_run>=2000u) {
            camera_uart_stats_t stats;camera_uart_get_stats(&stats);
            printf("[RUN] t=%lums wifi=%d mqtt=%s camera_bytes=%lu camera_frames=%lu sensor=%s nav=%s\n",
                (unsigned long)now,wifi_link_status(),mqtt_is_connected()?"up":"down",
                (unsigned long)stats.parser.bytes_received,(unsigned long)stats.parser.frames_valid,
                sensor_data_valid?"seen":"none",onboard_dfs_phase_name(onboard.phase));
            last_run=now;
        }
        sleep_ms(10);
    }
}
