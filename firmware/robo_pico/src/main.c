#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "config.h"
#include "motor_driver.h"
#include "wifi_mqtt.h"
#include "uart_protocol.h"
#include "lwip/timeouts.h"
#include "lwip/apps/mqtt.h"

// UART configuration
#define SENSOR_UART_ID uart0
#define SENSOR_UART_BAUD 115200
#define SENSOR_UART_RX_PIN 13
#define SENSOR_UART_TX_PIN -1
#define SENSOR_TELEMETRY_INTERVAL_MS 100u // 10 Hz feedback for laptop navigation.

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

static void on_sensor_packet(const sensor_packet_t *packet, uint32_t receive_ms)
{
    // Runs in the UART ISR: no MQTT, printf, sleeps, or reverse-thrust braking.
    const uint8_t front_mask = 0x02u; // UART bits: left=1, front=2, right=4.
    // Only front proximity or an explicit Sensor-side stop requests this lock.
    // Unavailable/zero readings and reported ToF faults do not request this lock.
    
    // bool stop_requested = packet->stop_required != 0 || (packet->urgent_mask & front_mask) != 0;
    uint8_t urgent_mask = (uint8_t)((packet->obstacle_flags >> 3u) & 0x07u);

    bool explicit_stop = (packet->obstacle_flags & (1u << 9u)) != 0;

    bool stop_requested = explicit_stop || (urgent_mask & front_mask) != 0;
        
    motor_safety_receive(stop_requested, receive_ms);
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
    if (motor_safety_is_latched())
    {
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        forward_boost_active = false;
        if (!safety_lock_reported)
        {
            safety_lock_reported = true;
            printf("[SAFETY] Motors locked: clear sensors and send RESET_SAFETY\n");
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
#define PITCH_BASELINE 77.0f      // Flat ground pitch from MPU6050 (77.0°) - calibrate per robot
#define PITCH_BOOST_PER_15_DEG 10 // Speed boost: +10% per 15° pitch drop
#define PITCH_MAX_BOOST 30        // Maximum boost capped at +30%

static int apply_pitch_compensation(int base_speed, float current_pitch)
{
    // Calculate pitch drop from baseline (upslope detected when pitch < baseline)
    float pitch_drop = PITCH_BASELINE - current_pitch;

    if (pitch_drop > 0)
    {
        // Apply boost: Every 15° drop adds 10% speed
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

    // Manual mode safety checks for ultrasonic obstacles
    if (!autonomous_mode && sensor_data_valid)
    {
        if (strncmp(cmd_buffer, "FORWARD", 7) == 0 && latest_sensors.ultrasonic_front <= 30)
        {
            printf("[SAFETY] Forward blocked - front obstacle at %dmm\n", latest_sensors.ultrasonic_front);
            return;
        }
        if (strncmp(cmd_buffer, "BACKWARD", 8) == 0 && latest_sensors.ultrasonic_back <= 30)
        {
            printf("[SAFETY] Backward blocked - back obstacle at %dmm\n", latest_sensors.ultrasonic_back);
            return;
        }
    }

    if (strncmp(cmd_buffer, "FORWARD", 7) == 0)
    {
        // Apply pitch compensation for upslope
        int speed = motor_move_speed(DIRECTION_FORWARD);
        int compensated_speed = apply_pitch_compensation(speed, latest_sensors.pitch);
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
    else if (strncmp(cmd_buffer, "AUTO_START", 10) == 0)
    {
        // Exploration decisions belong to Python. RoboPico executes movement
        // commands and retains its local heading hold and collision interlock.
        mqtt_publish_status("AUTO_START_DISABLED_USE_LAPTOP_DFS");
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

    if (result == MOTOR_OK)
    {
        mqtt_publish_status("RUNNING");
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
             "\"imu\":{"
             "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"mag\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"heading\":%.1f,"
             "\"pitch\":%.1f,"
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
             "\"age_ms\":%lu"
             "}"
             "}",
             (unsigned long)latest_sensors.timestamp_ms,
             latest_sensors.accel_x, latest_sensors.accel_y, latest_sensors.accel_z,
             latest_sensors.mag_x, latest_sensors.mag_y, latest_sensors.mag_z,
             latest_sensors.heading, latest_sensors.pitch, latest_sensors.imu_status,
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
             (unsigned long)(to_ms_since_boot(get_absolute_time()) - last_sensor_packet_ms));

    mqtt_publish_telemetry(telemetry);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);

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
        cyw43_arch_poll();
        service_safety_stop();
        uint32_t now = to_ms_since_boot(get_absolute_time());

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

            last_stats_print = now;
        }

        sleep_ms(100);
    }

    return 0;
}
