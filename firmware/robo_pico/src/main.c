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
#define SENSOR_UART_ID      uart0
#define SENSOR_UART_BAUD    115200
#define SENSOR_UART_RX_PIN  13
#define SENSOR_UART_TX_PIN  -1

// Buzzer configuration (piezo buzzer - PWM)
#define BUZZER_PIN          22
#define BUZZER_FREQ_HZ      2500  // 2.5kHz tone

// MQTT Topics
// #define MQTT_TOPIC_LIDAR    "robot/lidar"

// Latest sensor data
static sensor_packet_t latest_sensors;
static bool sensor_data_valid = false;
static uint32_t last_sensor_packet_ms = 0;  // Local receive time, not Sensor Pico time.

// Current speed setting
static int current_speed = 90;

// Individual wheel speed adjustments (0-100%)
static int wheel_speed_fl = 100;
static int wheel_speed_fr = 100;
static int wheel_speed_bl = 100;
static int wheel_speed_br = 100;

// LIDAR parsing state
static uint32_t lidar_points_received = 0;

// Autonomous mode state machine
typedef enum {
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
static int turn_direction = 0;  // -1 = left, 1 = right
static uint32_t last_auto_motor_cmd = 0;  // Track last motor command for keep-alive
// Forward boost handling
static uint32_t forward_boost_start_time = 0; // when forward boost started
static bool forward_boost_active = false;     // whether boost is active
// Track latest control command time to adapt LIDAR publish load
static uint32_t last_command_time_ms = 0;

// Proximity buzzer state
static uint32_t last_beep_time = 0;
static bool beep_state = false;
static uint buzzer_slice_num;

// Turn memory and dead-end detection
static int last_turn_direction = 0;  // Remember last turn to bias against reversing
static uint32_t last_successful_forward_time = 0;  // Track when we last moved forward successfully
static uint32_t entered_moving_forward_time = 0;  // When we entered MOVING_FORWARD state
static bool safety_lock_reported = false;

static void dfs_test_output(const char *message) {
    printf("%s\n", message);
    mqtt_publish_status(message);
}

static void on_sensor_packet(const sensor_packet_t *packet, uint32_t receive_ms) {
    // Runs in the UART ISR: no MQTT, printf, sleeps, or reverse-thrust braking.
    const uint8_t front_mask = 0x02u; // UART bits: left=1, front=2, right=4.
    // Only front proximity or an explicit Sensor-side stop requests this lock.
    // Unavailable/zero ToF readings and legacy fault bits do not request a stop.
    bool stop_requested = packet->stop_required != 0 ||
                          (packet->urgent_mask & front_mask) != 0;
    motor_safety_receive(stop_requested, receive_ms);
}

static bool safety_watchdog_tick(struct repeating_timer *timer) {
    (void)timer;
    motor_safety_poll();
    return true;
}

static void service_safety_stop(void) {
    motor_safety_poll();
    if (motor_safety_is_latched()) {
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        forward_boost_active = false;
        if (!safety_lock_reported) {
            safety_lock_reported = true;
            printf("[SAFETY] Motors locked: clear sensors and send RESET_SAFETY\n");
            mqtt_publish_status("SAFETY_LOCKED");
        }
    } else {
        safety_lock_reported = false;
    }
}

// Autonomous navigation parameters
#define AUTO_OBSTACLE_DISTANCE_MM 600.0f  // Stop if obstacle within 600mm
#define AUTO_FORWARD_SECTOR_MIN 160.0f    // Front sector: 160° to 200° (40° cone)
#define AUTO_FORWARD_SECTOR_MAX 200.0f
#define AUTO_LEFT_SECTOR_MIN 90.0f        // Left sector: 90° to 150° (60° cone)
#define AUTO_LEFT_SECTOR_MAX 150.0f
#define AUTO_RIGHT_SECTOR_MIN 210.0f      // Right sector: 210° to 270° (60° cone)
#define AUTO_RIGHT_SECTOR_MAX 270.0f
#define AUTO_TURN_DURATION_MS 400         // Turn execution time (400ms)
#define AUTO_SCAN_DURATION_MS 2500        // Gap detection scan period (2500ms)
#define AUTO_TURN_BIAS_PENALTY_MM 300.0f  // Penalty to avoid turn direction oscillation (300mm)
#define AUTO_DEAD_END_TIME_MS 2000        // Obstacle hit within 2000ms = dead end
#define AUTO_SUCCESSFUL_FORWARD_TIME_MS 5000  // Clear turn bias after 5000ms forward progress
// Gap-aware navigation thresholds
#define AUTO_FREE_DISTANCE_MM 1000.0f     // Ray considered free if >= 1000mm
#define AUTO_MIN_FORWARD_CLEAR_RATIO 0.40f// Require 40% of front rays free to resume

// Pitch compensation for upslope detection
#define PITCH_BASELINE 77.0f              // Flat ground pitch from MPU6050 (77.0°) - calibrate per robot
#define PITCH_BOOST_PER_15_DEG 10         // Speed boost: +10% per 15° pitch drop
#define PITCH_MAX_BOOST 30                // Maximum boost capped at +30%

static int apply_pitch_compensation(int base_speed, float current_pitch) {
    // Calculate pitch drop from baseline (upslope detected when pitch < baseline)
    float pitch_drop = PITCH_BASELINE - current_pitch;
    
    if (pitch_drop > 0) {
        // Apply boost: Every 15° drop adds 10% speed
        int boost_percent = (int)(pitch_drop / 15.0f) * PITCH_BOOST_PER_15_DEG;
        
        // Apply boost cap at 30%
        if (boost_percent > PITCH_MAX_BOOST) {
            boost_percent = PITCH_MAX_BOOST;
        }
        
        int adjusted_speed = base_speed + boost_percent;
        
        // Hard limit at 100%
        if (adjusted_speed > 100) {
            adjusted_speed = 100;
        }
        
        return adjusted_speed;
    }
    
    return base_speed;
}

static void process_command(const char *command) {
    printf("[CMD] Processing: %s\n", command);
    if (handle_dfs_test_command(command)) return;

    motor_error_t result = MOTOR_OK;
    
    char cmd_buffer[32];
    int speed = current_speed;
    
    strncpy(cmd_buffer, command, sizeof(cmd_buffer) - 1);
    cmd_buffer[sizeof(cmd_buffer) - 1] = '\0';
    
    // Parse format: COMMAND:SPEED:FL:FR:BL:BR
    char *colon = strchr(cmd_buffer, ':');
    if (colon != NULL) {
        *colon = '\0';
        speed = atoi(colon + 1);
        
        if (speed < 20) speed = 20;
        if (speed > 100) speed = 100;
        
        current_speed = speed;
        
        // Parse wheel speed adjustments if present
        char *colon2 = strchr(colon + 1, ':');
        if (colon2 != NULL) {
            wheel_speed_fl = atoi(colon2 + 1);
            
            char *colon3 = strchr(colon2 + 1, ':');
            if (colon3 != NULL) {
                wheel_speed_fr = atoi(colon3 + 1);
                
                char *colon4 = strchr(colon3 + 1, ':');
                if (colon4 != NULL) {
                    wheel_speed_bl = atoi(colon4 + 1);
                    
                    char *colon5 = strchr(colon4 + 1, ':');
                    if (colon5 != NULL) {
                        wheel_speed_br = atoi(colon5 + 1);
                    }
                }
            }
            
            // Clamp values
            if (wheel_speed_fl < 0) wheel_speed_fl = 0;
            if (wheel_speed_fl > 100) wheel_speed_fl = 100;
            if (wheel_speed_fr < 0) wheel_speed_fr = 0;
            if (wheel_speed_fr > 100) wheel_speed_fr = 100;
            if (wheel_speed_bl < 0) wheel_speed_bl = 0;
            if (wheel_speed_bl > 100) wheel_speed_bl = 100;
            if (wheel_speed_br < 0) wheel_speed_br = 0;
            if (wheel_speed_br > 100) wheel_speed_br = 100;
            
            printf("[CMD] Wheel speeds: FL=%d%% FR=%d%% BL=%d%% BR=%d%%\n",
                   wheel_speed_fl, wheel_speed_fr, wheel_speed_bl, wheel_speed_br);
        }
        
        printf("[CMD] Speed set to: %d%%\n", speed);
    }
    
    service_safety_stop();
    if (strcmp(cmd_buffer, "RESET_SAFETY") == 0) {
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        if (motor_safety_reset()) {
            safety_lock_reported = false;
            mqtt_publish_status("SAFETY_READY");
            printf("[SAFETY] Reset accepted; a new movement command is required\n");
        } else {
            mqtt_publish_status("SAFETY_RESET_REFUSED");
            printf("[SAFETY] Reset refused: missing, stale, or unsafe sensor packet\n");
        }
        return;
    }
    if (motor_safety_is_latched() && strcmp(cmd_buffer, "STOP") != 0 &&
        strcmp(cmd_buffer, "AUTO_STOP") != 0) {
        mqtt_publish_status("SAFETY_LOCKED");
        return;
    }

    // Manual mode safety checks for ultrasonic obstacles
    if (!autonomous_mode && sensor_data_valid) {
        if (strncmp(cmd_buffer, "FORWARD", 7) == 0 && latest_sensors.ultrasonic_front <= 30) {
            printf("[SAFETY] Forward blocked - front obstacle at %dmm\n", latest_sensors.ultrasonic_front);
            return;
        }
        if (strncmp(cmd_buffer, "BACKWARD", 8) == 0 && latest_sensors.ultrasonic_back <= 30) {
            printf("[SAFETY] Backward blocked - back obstacle at %dmm\n", latest_sensors.ultrasonic_back);
            return;
        }
    }

    if (strncmp(cmd_buffer, "FORWARD", 7) == 0) {
        // Apply pitch compensation for upslope
        int compensated_speed = apply_pitch_compensation(speed, latest_sensors.pitch);
        if (compensated_speed != speed) {
            char msg[128];
            snprintf(msg, sizeof(msg), "PITCH: Upslope %.1f° - boost %d%% -> %d%%", 
                     latest_sensors.pitch, speed, compensated_speed);
            printf("[%s]\n", msg);
            mqtt_publish_status(msg);
        }
        result = motor_set_direction_individual(DIRECTION_FORWARD, compensated_speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "BACKWARD", 8) == 0) {
        result = motor_set_direction_individual(DIRECTION_BACKWARD, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "LEFT", 4) == 0) {
        result = motor_set_direction_individual(DIRECTION_LEFT, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "RIGHT", 5) == 0) {
        result = motor_set_direction_individual(DIRECTION_RIGHT, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "ROTATE_LEFT", 11) == 0) {
        result = motor_set_direction_individual(DIRECTION_ROTATE_LEFT, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "ROTATE_RIGHT", 12) == 0) {
        result = motor_set_direction_individual(DIRECTION_ROTATE_RIGHT, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "FRONT_LEFT", 10) == 0) {
        result = motor_set_direction_individual(DIRECTION_FRONT_LEFT, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "FRONT_RIGHT", 11) == 0) {
        result = motor_set_direction_individual(DIRECTION_FRONT_RIGHT, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "BACK_LEFT", 9) == 0) {
        result = motor_set_direction_individual(DIRECTION_BACK_LEFT, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "BACK_RIGHT", 10) == 0) {
        result = motor_set_direction_individual(DIRECTION_BACK_RIGHT, speed,
                                                wheel_speed_fl, wheel_speed_fr,
                                                wheel_speed_bl, wheel_speed_br);
    }
    else if (strncmp(cmd_buffer, "ARM_UP", 6) == 0) {
        // Direct mapping per corrected wiring
        result = arm_set_direction(ARM_UP);
    }
    else if (strncmp(cmd_buffer, "ARM_DOWN", 8) == 0) {
        // Direct mapping per corrected wiring
        result = arm_set_direction(ARM_DOWN);
    }
    else if (strncmp(cmd_buffer, "CLAW_OPEN", 9) == 0) {
        // Direct mapping per corrected wiring
        result = claw_set_direction(CLAW_OPEN);
    }
    else if (strncmp(cmd_buffer, "CLAW_CLOSE", 10) == 0) {
        // Direct mapping per corrected wiring
        result = claw_set_direction(CLAW_CLOSE);
    }
    else if (strncmp(cmd_buffer, "AUTO_START", 10) == 0) {
        autonomous_mode = true;
        autonomous_state = AUTO_STATE_IDLE;
        last_turn_direction = 0;
        last_successful_forward_time = 0;
        entered_moving_forward_time = 0;
        printf("[AUTO] Autonomous mode ENABLED\n");
        mqtt_publish_status("AUTO_ENABLED");
        return;
    }
    else if (strncmp(cmd_buffer, "AUTO_STOP", 9) == 0) {
        autonomous_mode = false;
        autonomous_state = AUTO_STATE_IDLE;
        motor_emergency_stop();
        printf("[AUTO] Autonomous mode DISABLED\n");
        mqtt_publish_status("AUTO_DISABLED");
        return;
    }
    else if (strncmp(cmd_buffer, "STOP", 4) == 0) {
        autonomous_mode = false;  // Stop also disables auto
        autonomous_state = AUTO_STATE_IDLE;
        result = motor_emergency_stop();
        mqtt_publish_status("STOPPED");
        return;
    }
    else {
        printf("[CMD] Unknown command: %s\n", cmd_buffer);
        return;
    }

    if (result == MOTOR_OK) {
        mqtt_publish_status("RUNNING");
    } else {
        mqtt_publish_status("ERROR");
    }
    // Record command activity time for MQTT load adaptation
    last_command_time_ms = to_ms_since_boot(get_absolute_time());
}

static void publish_sensor_data(void) {
    
    if (!sensor_data_valid) return;

    static char telemetry[2048];  // ← Reduced size since no LIDAR
    
    memset(telemetry, 0, sizeof(telemetry));
    
    // Remove all LIDAR JSON building code

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
        (unsigned)latest_sensors.tof_mm[0],
        (unsigned)latest_sensors.tof_mm[1],
        (unsigned)latest_sensors.tof_mm[2],
        (unsigned)latest_sensors.blocked_mask,
        (unsigned)latest_sensors.urgent_mask,
        (unsigned)latest_sensors.fault_mask,
        latest_sensors.stop_required ? "true" : "false",
        (unsigned long)(to_ms_since_boot(get_absolute_time()) - last_sensor_packet_ms)
    );

    mqtt_publish_telemetry(telemetry);

    // Keep only the dedicated LIDAR publish to robot/lidar
    if (latest_sensors.lidar_count > 0 && mqtt_is_connected()) {
        // LIMIT points to prevent buffer overflow; reduce when recent commands
        uint32_t tnow = to_ms_since_boot(get_absolute_time());
        bool recent_cmd = (tnow - last_command_time_ms) < 600; // last 600ms
        int max_points_to_publish = recent_cmd ? 90 : 150;
        int points_to_send = (latest_sensors.lidar_count < max_points_to_publish) ? 
                    latest_sensors.lidar_count : max_points_to_publish;

        // Bigger buffer and safer JSON assembly to avoid trailing commas
        static char lidar_only[8192];
        memset(lidar_only, 0, sizeof(lidar_only));

        int pos = snprintf(lidar_only, sizeof(lidar_only), "{\"points\":[");
        bool first = true;
        for (int i = 0; i < points_to_send; i++) {
            char item[96];
            int item_len = snprintf(item, sizeof(item),
                "{\"angle\":%.1f,\"distance\":%.1f,\"quality\":%d}",
                latest_sensors.lidar_points[i].angle,
                latest_sensors.lidar_points[i].distance,
                15);

            if (item_len < 0) break;

            // Reserve space for optional comma and closing "]}"
            int needed = item_len + (first ? 0 : 1) + 2; // 2 for ]}
            if (pos + needed >= (int)sizeof(lidar_only)) {
                printf("[WARN] LIDAR JSON buffer full at %d points\n", i);
                break;
            }

            if (!first) {
                lidar_only[pos++] = ',';
            }
            memcpy(lidar_only + pos, item, (size_t)item_len);
            pos += item_len;
            first = false;
        }

        lidar_only[pos++] = ']';
        lidar_only[pos++] = '}';
        lidar_only[pos] = '\0';

        mqtt_client_t *client = get_mqtt_client();
        if (client && mqtt_client_is_connected(client)) {
            // Check if MQTT is busy (previous message still in queue)
            // This prevents buffer reuse while MQTT is still transmitting
            static uint32_t last_lidar_publish_attempt = 0;
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            
            // Only attempt publish if enough time passed since last attempt
            uint32_t required_gap = recent_cmd ? 1200 : 800; // backoff under command activity
            if (current_time - last_lidar_publish_attempt >= required_gap) {
                err_t err = mqtt_publish(
                    client,
                    "robot/lidar",             // must match TOPIC_LIDAR in the dashboard
                    lidar_only,
                    strlen(lidar_only),
                    0,                         // QoS 0
                    0,                         // not retained
                    NULL, NULL
                );
                
                if (err == ERR_OK) {
                    printf("[MQTT] Published %d LIDAR points (%d bytes) to robot/lidar\n",
                        points_to_send, (int)strlen(lidar_only));
                    last_lidar_publish_attempt = current_time;
                } else if (err == ERR_MEM) {
                    printf("[WARN] MQTT queue full, skipping LIDAR publish\n");
                    // Don't update timestamp - will retry next cycle
                } else {
                    printf("[MQTT] LIDAR publish failed (err=%d)\n", err);
                    last_lidar_publish_attempt = current_time;
                }
            } else {
                // Silently skip if too soon after last attempt
            }
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n========================================\n");
    printf("  Mecanum Robot Controller\n");
    printf("  (With LIDAR Visualization)\n");
    printf("========================================\n\n");

    if (wifi_init() != WIFI_OK) {
        printf("[ERROR] WiFi init failed\n");
        return -1;
    }

    if (wifi_connect(WIFI_SSID, WIFI_PASSWORD, 30000) != WIFI_OK) {
        printf("[ERROR] WiFi connection failed\n");
        return -1;
    }

    if (motor_init() != MOTOR_OK) {
        printf("[ERROR] Motor init failed\n");
        return -1;
    }

    struct repeating_timer safety_timer;
    if (!add_repeating_timer_ms(-10, safety_watchdog_tick, NULL, &safety_timer)) {
        printf("[ERROR] Cannot start safety watchdog; motors remain locked\n");
        return -1;
    }
    uart_set_packet_callback(on_sensor_packet);
    uart_receiver_init(SENSOR_UART_ID, SENSOR_UART_BAUD, SENSOR_UART_RX_PIN, SENSOR_UART_TX_PIN);

    // Initialize piezo buzzer with PWM
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    
    // Set PWM frequency to 2.5kHz
    float clock_freq = 125000000.0f;  // Pico system clock
    uint32_t divider = clock_freq / (BUZZER_FREQ_HZ * 65536);  // 16-bit wrap
    if (divider < 1) divider = 1;
    pwm_set_clkdiv(buzzer_slice_num, (float)divider);
    pwm_set_wrap(buzzer_slice_num, 65535);  // 16-bit resolution
    pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 0);  // Start off
    pwm_set_enabled(buzzer_slice_num, true);
    printf("[BUZZER] PWM initialized on GP%d at %dHz\n", BUZZER_PIN, BUZZER_FREQ_HZ);

    if (mqtt_init(MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_CLIENT_ID) != WIFI_OK) {
        printf("[ERROR] MQTT init failed\n");
        return -1;
    }

    if (mqtt_connect() != WIFI_OK) {
        printf("[ERROR] MQTT connection failed\n");
        return -1;
    }

    printf("[MQTT] Waiting for connection...\n");
    int retries = 0;
    while (!mqtt_is_connected() && retries < 100) {
        cyw43_arch_poll();
        sys_check_timeouts();
        sleep_ms(50);
        retries++;
    }

    if (!mqtt_is_connected()) {
        printf("[ERROR] MQTT connection timeout\n");
        return -1;
    }

    if (mqtt_subscribe_commands(process_command) != WIFI_OK) {
        printf("[ERROR] MQTT subscribe failed\n");
        return -1;
    }

    mqtt_publish_status("READY");

    printf("\n========================================\n");
    printf("  System Ready!\n");
    printf("  IP: %s\n", wifi_get_ip_address());
    printf("  MQTT Broker: %s:%d\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    printf("  UART Sensor: Listening on GP%d\n", SENSOR_UART_RX_PIN);
    printf("  LIDAR: Forwarding to robot/lidar\n");
    printf("========================================\n\n");

    uint32_t last_timeout_check = 0;
    uint32_t last_sensor_publish = 0;
    uint32_t last_stats_print = 0;
    uint32_t sensor_packet_count = 0;
    
    while (true) {
        service_safety_stop();
        cyw43_arch_poll();
        service_safety_stop();
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (now - last_timeout_check >= 100) {
            sys_check_timeouts();
            last_timeout_check = now;
        }
        
        motor_check_brake_timeout();
    
        // check_for_lidar_data();

        if (uart_packet_available()) {
            if (uart_get_packet(&latest_sensors)) {
                sensor_data_valid = true;
                last_sensor_packet_ms = to_ms_since_boot(get_absolute_time());
                sensor_packet_count++;

                // Count received LIDAR points
                lidar_points_received += latest_sensors.lidar_count;
                
                // Debug print only when new packet arrives
                if (latest_sensors.lidar_count > 0) {
                    printf("[LIDAR] %d points received. First angle=%.1f dist=%.1fmm\n",
                        latest_sensors.lidar_count,
                        latest_sensors.lidar_points[0].angle,
                        latest_sensors.lidar_points[0].distance);
                }
            }
        }

        // --- Proximity buzzer (disabled during autonomous) ---
        if (!autonomous_mode && sensor_data_valid) {
            // Find closest obstacle from all 4 ultrasonic sensors
            int closest = latest_sensors.ultrasonic_front;
            if (latest_sensors.ultrasonic_back > 0 && latest_sensors.ultrasonic_back < closest) {
                closest = latest_sensors.ultrasonic_back;
            }
            if (latest_sensors.ultrasonic_left > 0 && latest_sensors.ultrasonic_left < closest) {
                closest = latest_sensors.ultrasonic_left;
            }
            if (latest_sensors.ultrasonic_right > 0 && latest_sensors.ultrasonic_right < closest) {
                closest = latest_sensors.ultrasonic_right;
            }

            // Proximity beeping logic (car-like behavior)
            if (closest <= 30) {
                // Very close (< 25cm) - continuous beep (25% duty cycle for softer sound)
                pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 3277);
            } else if (closest < 45) {
                // Close (< 40cm) - beep every 1 second
                if (now - last_beep_time >= 500) {
                    beep_state = !beep_state;
                    pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 
                                      beep_state ? 3277 : 0);  // 25% duty or off
                    last_beep_time = now;
                }
            } else {
                // Safe - buzzer off
                pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 0);
                beep_state = false;
            }
        } else if (autonomous_mode) {
            // Disable buzzer during autonomous mode
            pwm_set_chan_level(buzzer_slice_num, pwm_gpio_to_channel(BUZZER_PIN), 0);
            beep_state = false;
        }

        // --- Autonomous mode state machine ---
        if (autonomous_mode && sensor_data_valid && latest_sensors.lidar_count > 0) {
            // State-based sector scanning (optimized to only check necessary sectors)
            float min_forward_distance = 5000.0f;
            float min_left_distance = 5000.0f;
            float min_right_distance = 5000.0f;
            int forward_count = 0, left_count = 0, right_count = 0;
            int left_free_count = 0, right_free_count = 0;  // Track free points
            
            // Determine which sectors to scan based on current state
            bool scan_forward = (autonomous_state == AUTO_STATE_MOVING_FORWARD || 
                                autonomous_state == AUTO_STATE_RESUMING || 
                                autonomous_state == AUTO_STATE_SCANNING ||
                                autonomous_state == AUTO_STATE_IDLE);
            bool scan_sides = (autonomous_state == AUTO_STATE_SCANNING);
            
            for (int i = 0; i < latest_sensors.lidar_count; i++) {
                float angle = latest_sensors.lidar_points[i].angle;
                float distance = latest_sensors.lidar_points[i].distance;
                
                // Normalize angle to 0-360
                while (angle < 0) angle += 360.0f;
                while (angle >= 360.0f) angle -= 360.0f;
                
                if (distance < 10.0f) continue;  // Ignore invalid readings
                
                // Forward sector (160-200°) - only when needed
                if (scan_forward && angle >= AUTO_FORWARD_SECTOR_MIN && angle <= AUTO_FORWARD_SECTOR_MAX) {
                    if (distance < min_forward_distance) {
                        min_forward_distance = distance;
                    }
                    forward_count++;
                }
                // Left sector (90-150°) - only during scanning
                else if (scan_sides && angle >= AUTO_LEFT_SECTOR_MIN && angle <= AUTO_LEFT_SECTOR_MAX) {
                    if (distance < min_left_distance) {
                        min_left_distance = distance;
                    }
                    if (distance >= AUTO_FREE_DISTANCE_MM) {
                        left_free_count++;
                    }
                    left_count++;
                }
                // Right sector (210-270°) - only during scanning
                else if (scan_sides && angle >= AUTO_RIGHT_SECTOR_MIN && angle <= AUTO_RIGHT_SECTOR_MAX) {
                    if (distance < min_right_distance) {
                        min_right_distance = distance;
                    }
                    if (distance >= AUTO_FREE_DISTANCE_MM) {
                        right_free_count++;
                    }
                    right_count++;
                }
            }
            
            // State machine logic
            switch (autonomous_state) {
                case AUTO_STATE_IDLE: {
                    // Start moving forward at moderate speed
                    // Apply startup boost: 90% for first 150ms, then 40%
                    int startup_speed = apply_pitch_compensation(90, latest_sensors.pitch);
                    motor_set_direction(DIRECTION_FORWARD, startup_speed);
                    autonomous_state = AUTO_STATE_MOVING_FORWARD;
                    state_start_time = now;
                    last_auto_motor_cmd = now;
                    entered_moving_forward_time = now;
                    forward_boost_start_time = now;
                    forward_boost_active = true;
                    printf("[AUTO] Starting forward movement\n");
                    mqtt_publish_status("AUTO: Moving forward");
                    break;
                }
                    
                case AUTO_STATE_MOVING_FORWARD:
                    // Keep-alive: Refresh motor command every 300ms for faster obstacle detection
                    if (now - last_auto_motor_cmd >= 200) {
                        // Maintain current intended speed: 90% during boost, else 50%
                        int intended_speed = (forward_boost_active ? 90 : 50);
                        int compensated_speed = apply_pitch_compensation(intended_speed, latest_sensors.pitch);
                        motor_set_direction(DIRECTION_FORWARD, compensated_speed);
                        last_auto_motor_cmd = now;
                        printf("[AUTO] Motor keep-alive - forward refresh\n");
                    }

                    // End boost after 150ms and drop to 50%
                    if (forward_boost_active && (now - forward_boost_start_time >= 175)) {
                        int reduced_speed = apply_pitch_compensation(50, latest_sensors.pitch);
                        motor_set_direction(DIRECTION_FORWARD, reduced_speed);
                        last_auto_motor_cmd = now;
                        forward_boost_active = false;
                        printf("[AUTO] Forward boost complete, reducing to 50%% speed\n");
                        mqtt_publish_status("AUTO: Forward speed reduced to 50%");
                    }
                    
                    // Check for obstacles ahead
                    if (forward_count > 0 && min_forward_distance < AUTO_OBSTACLE_DISTANCE_MM) {
                        motor_emergency_stop();
                        autonomous_state = AUTO_STATE_OBSTACLE_DETECTED;
                        state_start_time = now;
                        
                        // Dead-end detection: if we hit obstacle within 2 seconds of entering forward state
                        uint32_t time_in_forward = now - entered_moving_forward_time;
                        if (time_in_forward < AUTO_DEAD_END_TIME_MS && last_turn_direction != 0) {
                            printf("[AUTO] DEAD END detected! Hit obstacle after only %lums. Will reverse turn direction.\n", time_in_forward);
                            mqtt_publish_status("AUTO: Dead end detected, reversing strategy");
                            // Force opposite turn by negating last turn direction
                            last_turn_direction = -last_turn_direction;
                        }
                        
                        printf("[AUTO] Obstacle detected at %.1fmm - stopping\n", min_forward_distance);
                        mqtt_publish_status("AUTO: Obstacle detected, stopping");
                    }
                    // If no forward data for extended period, continue anyway (LIDAR might have dropout)
                    // This prevents false stops due to temporary sensor issues
                    break;
                    
                case AUTO_STATE_OBSTACLE_DETECTED:
                    // Transition to scanning immediately
                    autonomous_state = AUTO_STATE_SCANNING;
                    state_start_time = now;
                    printf("[AUTO] Scanning left/right for clear path\n");
                    mqtt_publish_status("AUTO: Scanning for clear path");
                    break;
                    
                case AUTO_STATE_SCANNING:
                    // Wait for scan duration, then decide turn direction
                    if (now - state_start_time >= AUTO_SCAN_DURATION_MS) {
                        // Calculate percentage of free points in each sector
                        float left_free_ratio = 0.0f;
                        float right_free_ratio = 0.0f;
                        
                        // Require minimum point count for confidence (avoid decisions on sparse data)
                        const int MIN_POINTS_FOR_CONFIDENCE = 5;
                        
                        if (left_count >= MIN_POINTS_FOR_CONFIDENCE) {
                            left_free_ratio = (float)left_free_count / (float)left_count;
                        }
                        if (right_count >= MIN_POINTS_FOR_CONFIDENCE) {
                            right_free_ratio = (float)right_free_count / (float)right_count;
                        }

                        // Apply bias penalty against reversing last turn (reduce ratio by 0.15)
                        if (last_turn_direction == 1) { // last RIGHT
                            left_free_ratio = (left_free_ratio > 0.15f) ? (left_free_ratio - 0.15f) : 0.0f;
                            printf("[AUTO] Bias: penalizing LEFT (was %.2f)\n", left_free_ratio + 0.15f);
                        } else if (last_turn_direction == -1) { // last LEFT
                            right_free_ratio = (right_free_ratio > 0.15f) ? (right_free_ratio - 0.15f) : 0.0f;
                            printf("[AUTO] Bias: penalizing RIGHT (was %.2f)\n", right_free_ratio + 0.15f);
                        }

                        // Decision logic with safety check
                        if (left_count == 0 && right_count == 0) {
                            turn_direction = (last_turn_direction != 0) ? -last_turn_direction : -1;
                            printf("[AUTO] No side scan data, defaulting to %s turn\n",
                                   (turn_direction < 0) ? "LEFT" : "RIGHT");
                        } else if (left_count < MIN_POINTS_FOR_CONFIDENCE) {
                            turn_direction = 1;  // Insufficient left data, go right
                            printf("[AUTO] Insufficient LEFT data (%d points), turning RIGHT\n", left_count);
                        } else if (right_count < MIN_POINTS_FOR_CONFIDENCE) {
                            turn_direction = -1;  // Insufficient right data, go left
                            printf("[AUTO] Insufficient RIGHT data (%d points), turning LEFT\n", right_count);
                        } else {
                            // Both sides have enough data - compare free ratios
                            if (fabs(left_free_ratio - right_free_ratio) > 0.2f) {
                                // Significant difference - pick more open side
                                turn_direction = (left_free_ratio > right_free_ratio) ? -1 : 1;
                                printf("[AUTO] Choosing by FREE RATIO: L=%.1f%% (%d/%d free) R=%.1f%% (%d/%d free) minDist: L=%.0fmm R=%.0fmm\n",
                                       left_free_ratio * 100, left_free_count, left_count,
                                       right_free_ratio * 100, right_free_count, right_count,
                                       min_left_distance, min_right_distance);
                            } else {
                                // Similar openness - use min distance as tie-breaker with safety check
                                if (min_left_distance < 600.0f && min_right_distance < 600.0f) {
                                    // Both unsafe - pick lesser evil
                                    turn_direction = (min_left_distance > min_right_distance) ? -1 : 1;
                                    printf("[AUTO] Both sides UNSAFE, choosing least bad: L=%.0fmm R=%.0fmm\n",
                                           min_left_distance, min_right_distance);
                                } else {
                                    turn_direction = (min_left_distance > min_right_distance) ? -1 : 1;
                                    printf("[AUTO] Similar free ratios (L=%.1f%% R=%.1f%%), tie-breaker by DISTANCE: L=%.0fmm R=%.0fmm\n",
                                           left_free_ratio * 100, right_free_ratio * 100,
                                           min_left_distance, min_right_distance);
                                }
                            }
                        }
                        
                        // Start turning
                        if (turn_direction < 0) {
                            motor_set_direction(DIRECTION_ROTATE_LEFT, 87);
                            printf("[AUTO] Turning LEFT\n");
                            mqtt_publish_status("AUTO: Turning left");
                        } else {
                            motor_set_direction(DIRECTION_ROTATE_RIGHT, 87);
                            printf("[AUTO] Turning RIGHT\n");
                            mqtt_publish_status("AUTO: Turning right");
                        }
                        
                        // Remember this turn direction
                        last_turn_direction = turn_direction;
                        
                        autonomous_state = AUTO_STATE_TURNING;
                        state_start_time = now;
                        last_auto_motor_cmd = now;
                    }
                    break;
                    
                case AUTO_STATE_TURNING:
                    // Keep-alive: Refresh turn command every 300ms
                    if (now - last_auto_motor_cmd >= 300) {
                        if (turn_direction < 0) {
                            motor_set_direction(DIRECTION_ROTATE_LEFT, 87);
                        } else {
                            motor_set_direction(DIRECTION_ROTATE_RIGHT, 87);
                        }
                        last_auto_motor_cmd = now;
                        printf("[AUTO] Motor keep-alive - turn refresh\n");
                    }
                    
                    // Turn for fixed duration
                    if (now - state_start_time >= AUTO_TURN_DURATION_MS) {
                        motor_emergency_stop();
                        autonomous_state = AUTO_STATE_RESUMING;
                        state_start_time = now;
                        printf("[AUTO] Turn complete, checking forward path\n");
                        mqtt_publish_status("AUTO: Turn complete");
                    }
                    break;
                    
                case AUTO_STATE_RESUMING:
                    // Check if forward is clear, then resume
                    if (forward_count > 0) {
                        // We have forward data - check if clear
                        // Gap-aware gating: require distance and a minimum clear ratio
                        float forward_clear_ratio = (min_forward_distance >= AUTO_FREE_DISTANCE_MM) ? 1.0f : 0.0f;
                        if (min_forward_distance >= AUTO_OBSTACLE_DISTANCE_MM && forward_clear_ratio >= AUTO_MIN_FORWARD_CLEAR_RATIO) {
                            // Resume forward with startup boost pattern
                            int resume_speed = apply_pitch_compensation(90, latest_sensors.pitch);
                            motor_set_direction(DIRECTION_FORWARD, resume_speed);
                            autonomous_state = AUTO_STATE_MOVING_FORWARD;
                            state_start_time = now;
                            last_auto_motor_cmd = now;
                            entered_moving_forward_time = now;
                            last_successful_forward_time = now;
                            forward_boost_start_time = now;
                            forward_boost_active = true;
                            
                            // Clear turn bias if we've been navigating successfully for a while
                            if (last_successful_forward_time > 0 && 
                                (now - last_successful_forward_time) > AUTO_SUCCESSFUL_FORWARD_TIME_MS) {
                                printf("[AUTO] Clearing turn bias after successful navigation\n");
                                last_turn_direction = 0;
                            }
                            
                            printf("[AUTO] Path clear, resuming forward (%.1fmm)\n", min_forward_distance);
                            mqtt_publish_status("AUTO: Resuming forward");
                        } else if (now - state_start_time >= 2000) {
                            // Still blocked after 2s, perform a full rescan
                            autonomous_state = AUTO_STATE_SCANNING;
                            state_start_time = now;
                            printf("[AUTO] Still blocked, scanning again\n");
                            mqtt_publish_status("AUTO: Path blocked, scanning again");
                        }
                    } else {
                        // No forward data - assume clear after timeout to avoid getting stuck (aligned to 2s)
                        if (now - state_start_time >= 2000) {
                            // Resume forward with startup boost pattern
                            int resume_speed = apply_pitch_compensation(90, latest_sensors.pitch);
                            motor_set_direction(DIRECTION_FORWARD, resume_speed);
                            last_auto_motor_cmd = now;
                            entered_moving_forward_time = now;
                            autonomous_state = AUTO_STATE_MOVING_FORWARD;
                            state_start_time = now;
                            forward_boost_start_time = now;
                            forward_boost_active = true;
                            printf("[AUTO] No forward data, assuming clear and resuming\n");
                            mqtt_publish_status("AUTO: No sensor data, resuming forward");
                        }
                    }
                    break;
            }
        } else if (autonomous_mode && autonomous_state != AUTO_STATE_IDLE) {
            // Autonomous enabled but state machine not running (likely no LIDAR data)
            static uint32_t last_no_lidar_warning = 0;
            if (now - last_no_lidar_warning >= 2000) {
                printf("[AUTO] WARNING: State machine paused - no LIDAR data (state=%d)\n", autonomous_state);
                mqtt_publish_status("AUTO: Waiting for LIDAR data");
                last_no_lidar_warning = now;
            }
        } else if (!autonomous_mode && autonomous_state != AUTO_STATE_IDLE) {
            // Auto mode disabled - reset state
            motor_emergency_stop();
            autonomous_state = AUTO_STATE_IDLE;
        }

        // IMU-tied arm PID removed (manual control only)

        // --- Sensor & status publishing ---
        // Increased interval to 1000ms to prevent MQTT queue overflow with large LIDAR messages
        if (sensor_data_valid && (now - last_sensor_publish >= 1000)) {
            publish_sensor_data();
            last_sensor_publish = now;
        }

        if (now - last_stats_print >= 5000) {
            uint32_t total, valid, errors;
            uart_get_stats(&total, &valid, &errors);

            printf("[STATS] Packets: %lu total, %lu valid, %lu errors | ",
                total, valid, errors);

            if (sensor_data_valid) {
                printf("Heading=%.1f° Pitch=%.1f°", latest_sensors.heading, latest_sensors.pitch);
                printf(" | Ultrasonic: F=%dcm B=%dcm L=%dcm R=%dcm", 
                       latest_sensors.ultrasonic_front,
                       latest_sensors.ultrasonic_back,
                       latest_sensors.ultrasonic_left, 
                       latest_sensors.ultrasonic_right);
            } else {
                printf("No sensor data");
            }

            printf(" | LIDAR points recv: %lu\n", lidar_points_received);
            last_stats_print = now;
        }

        motor_check_timeout();
        sleep_ms(100);
    }

    return 0;
}
