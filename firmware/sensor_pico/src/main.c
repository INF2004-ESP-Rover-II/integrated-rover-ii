/**
 * @file main.c
 * @brief Main firmware module.
 */
#include "hardware/uart.h"
#include "imu_sampler.h"
#include "mpu6500.h"
#include "obstacle.h"
#include "pico/stdlib.h"
#include "sd_card.h"
#include "sensor_config.h"
#include "tof.h"
#include "uart_protocol.h"
#include "ultrasonic.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Fixed dimensions and iteration limits used by this module. */
#define TOF_SCAN_COUNT 3

#define M_PI 3.14159265358979323846

static bool g_b_imu_available        = false;
static bool g_b_ultrasonic_available = true;

// SD Card logging
static bool     g_b_sd_card_available = false;
static uint32_t g_log_file_number     = 1;
static char     g_log_filename[32];

// Diagnostic only: count delivered fresh results, not calls to tof_poll().
// Both polling sites share this state. A cycle requires a new result from all
// three sensors since the previous cycle, even if they arrive in different
// polls.
#define TOF_TEST_REPORT_MS 10000u
#define TOF_TEST_MAX_GAP_MS 100u
#define TOF_TEST_ALL_MASK ((uint8_t) ((1u << TOF_SENSOR_COUNT) - 1u))

typedef struct
{
    uint32_t started_ms;
    uint32_t window_started_ms;
    uint32_t total_cycles;
    uint32_t window_cycles;
    uint32_t fresh[TOF_SENSOR_COUNT];
    uint32_t valid[TOF_SENSOR_COUNT];
    uint32_t no_target[TOF_SENSOR_COUNT];
    uint32_t invalid[TOF_SENSOR_COUNT];
    uint32_t last_fresh_ms[TOF_SENSOR_COUNT];
    uint32_t max_gap_ms[TOF_SENSOR_COUNT];
    uint8_t  pending_mask;
} tof_test_stats_t;

static void
tof_test_observe (tof_test_stats_t * p_stats, const tof_readings_t * p_readings)
{
    for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
    {
        const uint8_t bit = (uint8_t) (1u << (uint32_t) (index));
        if (((uint32_t) (p_readings->fresh_mask) & (uint32_t) (bit)) == 0)
        {
            continue;
        }

        uint32_t gap_ms =
            p_readings->timestamp_ms[index] - p_stats->last_fresh_ms[index];

        p_stats->last_fresh_ms[index] = p_readings->timestamp_ms[index];
        if (gap_ms > p_stats->max_gap_ms[index])
        {
            p_stats->max_gap_ms[index] = gap_ms;
        }

        p_stats->fresh[index]++;

        uint16_t distance =
            tof_distance_get(p_readings->distance_bits, (tof_sensor_t) index);

        if (((uint32_t) (p_readings->valid_mask) & (uint32_t) (bit)) &&
            distance == TOF_NO_TARGET_MM)
        {
            p_stats->no_target[index]++;
        }
        else if (((uint32_t) (p_readings->valid_mask) & (uint32_t) (bit)) &&
                 distance > 0)
        {
            p_stats->valid[index]++;
        }
        else
        {
            p_stats->invalid[index]++;
        }
    }

    p_stats->pending_mask |= (uint32_t) ((uint32_t) (p_readings->fresh_mask) &
                                         (uint32_t) (TOF_TEST_ALL_MASK));
    if (p_stats->pending_mask == TOF_TEST_ALL_MASK)
    {
        p_stats->window_cycles++;
        p_stats->total_cycles++;
        p_stats->pending_mask = 0;
    }
}

static void
tof_test_report (tof_test_stats_t * p_stats, uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - p_stats->window_started_ms;
    if (elapsed_ms < TOF_TEST_REPORT_MS)
    {
        return;
    }

    // Use actual elapsed time: blocking sensor/UART/SD work may delay reports.
    // RATE checks fresh-result throughput only; validity and gaps are separate.
    double full_hz = p_stats->window_cycles * 1000.0 / elapsed_ms;
    printf("[TOF RATE] elapsed_ms=%" PRIu32 " cycles=%" PRIu32
           " full_hz=%.2f rate=%s total_ms=%" PRIu32 " total_cycles=%" PRIu32
           "\n",
           (uint32_t) elapsed_ms, (uint32_t) p_stats->window_cycles, full_hz,
           full_hz >= 10.0 ? "PASS" : "FAIL",
           (uint32_t) (now_ms - p_stats->started_ms),
           (uint32_t) p_stats->total_cycles);

    const char * names[TOF_SENSOR_COUNT] = {"L", "F", "R"};
    for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
    {
        // Include an ongoing outage, even if no new result ever finishes it.
        uint32_t age_ms = now_ms - p_stats->last_fresh_ms[index];
        if (age_ms > p_stats->max_gap_ms[index])
        {
            p_stats->max_gap_ms[index] = age_ms;
        }
        printf("[TOF %s] fresh=%" PRIu32 " valid=%" PRIu32 " no_target=%" PRIu32
               " invalid=%" PRIu32 " fresh_hz=%.2f "
               "valid_hz=%.2f usable_hz=%.2f max_gap_ms=%" PRIu32 " gap=%s\n",
               names[index], (uint32_t) p_stats->fresh[index],
               (uint32_t) p_stats->valid[index],
               (uint32_t) p_stats->no_target[index],
               (uint32_t) p_stats->invalid[index],
               p_stats->fresh[index] * 1000.0 / elapsed_ms,
               p_stats->valid[index] * 1000.0 / elapsed_ms,
               (p_stats->valid[index] + p_stats->no_target[index]) * 1000.0 /
                   elapsed_ms,
               (uint32_t) p_stats->max_gap_ms[index],
               p_stats->max_gap_ms[index] <= TOF_TEST_MAX_GAP_MS ? "OK"
                                                                 : "SLOW");
        p_stats->fresh[index]      = 0;
        p_stats->valid[index]      = 0;
        p_stats->no_target[index]  = 0;
        p_stats->invalid[index]    = 0;
        p_stats->max_gap_ms[index] = 0;
    }

    // Retain last-result times and pending bits across reporting windows.
    // Cross-window gaps remain visible; a partial cycle is counted only once.
    p_stats->window_cycles     = 0;
    p_stats->window_started_ms = now_ms;
}

static void
blink_error (void)
{
    for (int index = 0; index < TOF_SCAN_COUNT; index++)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }
}

static void
blink_ok (void)
{
    gpio_put(LED_PIN, 1);
    sleep_ms(50);
    gpio_put(LED_PIN, 0);
}

static uint16_t
read_ultrasonic_safe (uint32_t trig_pin, uint32_t echo_pin)
{
    float distance_cm = 0.0f;

    ultrasonic_status_t status =
        ultrasonic_get_cm(trig_pin, echo_pin, &distance_cm);

    if (status == ULTRASONIC_OK)
    {
        if (distance_cm < 2.0f)
        {
            distance_cm = 2.0f;
        }
        if (distance_cm > 400.0f)
        {
            distance_cm = 400.0f;
        }
        return (uint16_t) distance_cm;
    }
    else if (status == ULTRASONIC_NO_ECHO_START)
    {
        // No echo = sensor disconnected or wiring issue
        return 500; // Special value: disconnected
    }
    else
    {
        // ULTRASONIC_ECHO_STUCK_HIGH = out of range or bad reflection
        return 400; // Normal max range
    }
}

int
main (void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n========================================\n");
    printf("  Sensor Pico - Full System with IRQ\n");
    printf("========================================\n\n");

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, false);

    printf("[UART] Initialized at %d baud (TX=GP%d, RX=GP%d)\n", UART_BAUD_RATE,
           UART_TX_PIN, UART_RX_PIN);

    printf("[ULTRASONIC] Initializing sensors...\n");
    ultrasonic_init(ULTRA_FRONT_TRIG_PIN, ULTRA_FRONT_ECHO_PIN);
    ultrasonic_init(ULTRA_BACK_TRIG_PIN, ULTRA_BACK_ECHO_PIN);
#if ULTRA_LEFT_ENABLED
    ultrasonic_init(ULTRA_LEFT_TRIG_PIN, ULTRA_LEFT_ECHO_PIN);
#endif
    ultrasonic_init(ULTRA_RIGHT_TRIG_PIN, ULTRA_RIGHT_ECHO_PIN);
    printf("[ULTRASONIC] Front/back/right initialized; left %s\n",
           ULTRA_LEFT_ENABLED ? "enabled" : "disabled (GP6/GP7 used by IMU)");

    printf("[IMU] MPU6500 on GP%d/GP%d, address 0x%02X\n", IMU_SDA_PIN,
           IMU_SCL_PIN, MPU6500_ADDR);
    g_b_imu_available = mpu6500_init();
    if (g_b_imu_available)
    {
        printf("[IMU] Keep rover stationary: calibrating gyro (1 s settling, "
               "up to 10 windows)...\n");
        g_b_imu_available = mpu6500_calibrate();
    }
    if (!g_b_imu_available)
    {
        printf("[WARN] MPU6500 init/calibration failed; see IMU startup "
               "diagnostic below\n");
        blink_error();
    }

    const uint8_t tof_xshut_pins[TOF_SENSOR_COUNT] = {
        TOF_LEFT_XSHUT_PIN, TOF_FRONT_XSHUT_PIN, TOF_RIGHT_XSHUT_PIN};

    uint8_t tof_online = tof_init(tof_xshut_pins);
    printf("[TOF] Initialized sensors (L/F/R mask: 0x%02X)\n", tof_online);

    printf("[SD] Initializing SD card...\n");
    sd_init();
    if (sd_mount())
    {
        g_b_sd_card_available = true;
        printf("[SD] ✓ SD card ready\n");
    }
    else
    {
        g_b_sd_card_available = false;
        printf("[SD] ❌ SD card not available\n");
    }

    printf("\n========================================\n");
    if (g_b_imu_available)
    {
        imu_sampler_start();
        printf("[IMU] Core 1 sampler started: target 100 Hz, gravity-aligned "
               "yaw\n");
    }
    printf("  System Ready - Sensor Status:\n");
    printf("  Ultrasonic: %s\n",
           g_b_ultrasonic_available ? "ONLINE" : "OFFLINE");
    printf("  MPU6500: %s (relative heading, no magnetometer)\n",
           g_b_imu_available ? "ONLINE" : "OFFLINE");
    printf("  IMU startup: %s\n", mpu6500_last_error());
    printf("  IMU bus: SDA=GP%d SCL=GP%d addr=0x%02X WHO_AM_I=0x%02X (expected "
           "0x70)\n",
           IMU_SDA_PIN, IMU_SCL_PIN, MPU6500_ADDR, mpu6500_device_id());
    printf("  SD Card: %s \n", g_b_sd_card_available ? "ONLINE" : "OFFLINE");
    printf("  Update Rate: %d Hz\n", SENSOR_UPDATE_RATE_HZ);
    printf("  SD Save: Every 30 seconds\n");
    printf("========================================\n\n");

    blink_ok();

    uint32_t       last_update_ms            = 0;
    uint32_t       packet_count              = 0;
    uint32_t       last_stats_ms             = 0;
    uint32_t       last_sd_save_ms           = 0;
    uint32_t       last_packet_print         = 0;
    uint32_t       last_tof_print_ms         = 0;
    bool           b_imu_read_fault_reported = false;
    uint64_t       last_imu_diag_us          = time_us_64();
    imu_snapshot_t imu_baseline;
    imu_sampler_get_snapshot(&imu_baseline);
    uint32_t         last_imu_diag_samples       = imu_baseline.samples;
    const char *     tof_names[TOF_SENSOR_COUNT] = {"L", "F", "R"};
    obstacle_state_t obstacles;
    obstacle_init(&obstacles, to_ms_since_boot(get_absolute_time()));
    const uint32_t update_interval_ms = 1000 / SENSOR_UPDATE_RATE_HZ;

    const uint32_t stats_interval_ms = 5000;
    // Console stats report interval (5000ms)

    const uint32_t sd_save_interval_ms = 30000;
    // SD card write interval (30000ms)

    printf("[SYSTEM] Starting main loop...\n");

    // tof_test_stats_t tof_test;
    // tof_test_init(&tof_test, to_ms_since_boot(get_absolute_time()));

    while (true)
    {
        tof_readings_t tof_readings;
        tof_poll(&tof_readings);
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // Evaluate every loop so old measurements expire and are omitted.
        // The snapshot is also included in UART telemetry; motor handling is
        // separate.
        obstacle_update(&obstacles, &tof_readings, tof_online, now_ms);
        // tof_test_observe(&tof_test, &tof_readings);

        if (now_ms - last_tof_print_ms >= 100)
        {
            last_tof_print_ms = now_ms;
            printf("[TOF] ");
            for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
            {
                uint8_t bit = (uint8_t) (1u << (uint32_t) (index));

                uint16_t distance = tof_distance_get(obstacles.distance_bits,
                                                     (tof_sensor_t) index);

                if (((uint32_t) (obstacles.valid_mask) & (uint32_t) (bit)) == 0)
                {
                    printf("%s=-- ", tof_names[index]);
                }
                else if (distance == TOF_NO_TARGET_MM)
                {
                    printf("%s=>1000 mm ", tof_names[index]);
                }
                else
                {
                    printf("%s=%u mm ", tof_names[index], distance);
                }
            }
            printf("| blocked=0x%02X urgent=0x%02X fault=0x%02X stop=%s\n",
                   obstacles.blocked_mask, obstacles.urgent_mask,
                   obstacles.fault_mask,
                   obstacles.stop_required ? "YES" : "NO");
        }

        // Send regular sensor packets
        if (now_ms - last_update_ms >= update_interval_ms)
        {
            last_update_ms = now_ms;

            sensor_packet_t packet;
            memset(&packet, 0, sizeof(packet));

            packet.sync1        = PACKET_SYNC_BYTE1;
            packet.sync2        = PACKET_SYNC_BYTE2;
            packet.timestamp_ms = now_ms;

            packet.ultrasonic_status = 0;

            packet.ultrasonic_front = read_ultrasonic_safe(
                ULTRA_FRONT_TRIG_PIN, ULTRA_FRONT_ECHO_PIN);

            if (packet.ultrasonic_front >= 400)
            {
                packet.ultrasonic_status |=
                    (uint32_t) (((uint32_t) (1) << (uint32_t) (0)));
            }

            sleep_ms(15);

            packet.ultrasonic_back =
                read_ultrasonic_safe(ULTRA_BACK_TRIG_PIN, ULTRA_BACK_ECHO_PIN);

            if (packet.ultrasonic_back >= 400)
            {
                packet.ultrasonic_status |=
                    (uint32_t) (((uint32_t) (1) << (uint32_t) (1)));
            }

            sleep_ms(15);

#if ULTRA_LEFT_ENABLED
            packet.ultrasonic_left =
                read_ultrasonic_safe(ULTRA_LEFT_TRIG_PIN, ULTRA_LEFT_ECHO_PIN);
#else

            packet.ultrasonic_left = 500;
            // Unavailable; no GPIO access to IMU pins.
#endif
            if (packet.ultrasonic_left >= 400)
            {
                packet.ultrasonic_status |=
                    (uint32_t) (((uint32_t) (1) << (uint32_t) (2)));
            }

            sleep_ms(15);

            packet.ultrasonic_right = read_ultrasonic_safe(
                ULTRA_RIGHT_TRIG_PIN, ULTRA_RIGHT_ECHO_PIN);

            if (packet.ultrasonic_right >= 400)
            {
                packet.ultrasonic_status |=
                    (uint32_t) (((uint32_t) (1) << (uint32_t) (3)));
            }

            // Refresh ToF after the blocking legacy sensor reads and before
            // sending. Recheck age so an expired cached range is not reported
            // as healthy.
            tof_poll(&tof_readings);
            // Copy the latest independently sampled/fused IMU result. Core 0
            // never reads I2C1 while the core 1 sampler owns the IMU.
            imu_snapshot_t imu;
            imu_sampler_get_snapshot(&imu);
            uint64_t imu_now_us = time_us_64();

            bool b_imu_healthy = g_b_imu_available && imu.valid &&
                                 imu.sample_us != 0 &&
                                 imu_now_us - imu.sample_us <= 50000u;

            packet.imu_status = b_imu_healthy ? 0 : 1;
            if (b_imu_healthy)
            {
                if (b_imu_read_fault_reported)
                {
                    printf("[IMU] Fresh samples recovered\n");
                }
                b_imu_read_fault_reported = false;
                packet.accel_x            = imu.reading.accel_g[0] * 9.81f;
                packet.accel_y            = imu.reading.accel_g[1] * 9.81f;
                packet.accel_z            = imu.reading.accel_g[2] * 9.81f;
                packet.gyro_x             = imu.reading.gyro_dps[0];
                packet.gyro_y             = imu.reading.gyro_dps[1];
                packet.gyro_z             = imu.reading.gyro_dps[2];
                packet.heading            = imu.orientation.heading_deg;
                packet.pitch              = imu.orientation.pitch_deg;
                packet.roll               = imu.orientation.roll_deg;
            }
            else if (g_b_imu_available && !b_imu_read_fault_reported)
            {
                printf("[WARN] IMU unavailable: read_errors=%" PRIu32
                       " near_full_scale=%" PRIu32 " "
                       "timing_fault=%u (latched faults require restart)\n",
                       (uint32_t) imu.read_errors,
                       (uint32_t) imu.clipped_samples,
                       (unsigned) imu.orientation.timing_fault);
                b_imu_read_fault_reported = true;
            }
            packet.timestamp_ms = to_ms_since_boot(get_absolute_time());
            obstacle_update(&obstacles, &tof_readings, tof_online,
                            packet.timestamp_ms);
            // tof_test_observe(&tof_test, &tof_readings);
            // memcpy(packet.tof_mm, obstacles.distance_mm,
            // sizeof(packet.tof_mm));
            packet.tof_bits = obstacles.distance_bits;
            // packet.blocked_mask = obstacles.blocked_mask;
            // packet.urgent_mask = obstacles.urgent_mask;
            // packet.fault_mask = obstacles.fault_mask;
            // packet.stop_required = obstacles.stop_required ? 1u : 0u;

            /*
                Change made for Tribunal 1: pack the obstacle flags into a 2
               byte field instead of using 4 separate bytes to represent each
               mask bit 0-2: blocked bit 3-5: urgent bit 6-8: fault bit 9: STOP
                bit 10-15: spare
            */

            packet.obstacle_flags =
                (uint16_t) (((uint16_t) obstacles.blocked_mask & 0x07u) |
                            (((uint16_t) obstacles.urgent_mask & 0x07u) << 3u) |
                            (((uint16_t) obstacles.fault_mask & 0x07u) << 6u) |
                            ((obstacles.stop_required ? 1u : 0u) << 9u));

            packet.checksum = calculate_packet_checksum(&packet);

            send_sensor_packet(&packet);

            packet_count++;

            // Print status every second
            if (packet_count != last_packet_print &&
                packet_count % SENSOR_UPDATE_RATE_HZ == 0)
            {
                last_packet_print = packet_count;
                printf("[PKT %" PRIu32 "] ", packet_count);

                if (packet.imu_status == 0)
                {
                    printf("Heading(rel): %.1f | Pitch: %.1f | Roll: %.1f | "
                           "GyroZ: %.2f dps | ",
                           packet.heading, packet.pitch, packet.roll,
                           packet.gyro_z);
                }
                else
                {
                    printf("IMU: OFFLINE | ");
                }

                printf("Ultra: F=%d B=%d L=%d R=%d | ", packet.ultrasonic_front,
                       packet.ultrasonic_back, packet.ultrasonic_left,
                       packet.ultrasonic_right);

                if (g_b_sd_card_available)
                {
                    printf(" | SD: ✓");
                }
                else
                {
                    printf(" | SD: ✗");
                }

                if (packet.imu_status != 0 || packet.ultrasonic_status != 0)
                {
                    printf(" [IMU:%d US:0x%02X]", packet.imu_status,
                           packet.ultrasonic_status);
                }

                printf("\n");

                blink_ok();
            }
        }

        uint64_t imu_diag_us = time_us_64();
        if (g_b_imu_available && imu_diag_us - last_imu_diag_us >= 1000000u)
        {
            imu_snapshot_t diagnostic;
            imu_sampler_get_snapshot(&diagnostic);

            float sample_hz = (diagnostic.samples - last_imu_diag_samples) *
                              1000000.0f /
                              (float) (imu_diag_us - last_imu_diag_us);

            printf(
                "[IMU RATE] hz=%.1f max_gap_ms=%.2f read_errors=%" PRIu32
                " overruns=%" PRIu32 " "
                "near_full_scale=%" PRIu32 " heading=%.1f gyro=%.1f/%.1f/%.1f "
                "dps startup_up=%.2f/%.2f/%.2f\n",
                sample_hz, diagnostic.max_gap_us / 1000.0f,
                (uint32_t) diagnostic.read_errors,
                (uint32_t) diagnostic.overruns,
                (uint32_t) diagnostic.clipped_samples,
                diagnostic.orientation.heading_deg,
                diagnostic.reading.gyro_dps[0], diagnostic.reading.gyro_dps[1],
                diagnostic.reading.gyro_dps[2],
                diagnostic.orientation.startup_up[0],
                diagnostic.orientation.startup_up[1],
                diagnostic.orientation.startup_up[2]);
            last_imu_diag_us      = imu_diag_us;
            last_imu_diag_samples = diagnostic.samples;
        }

        // Report after normal processing so diagnostics do not precede STOP
        // data. tof_test_report(&tof_test,
        // to_ms_since_boot(get_absolute_time()));
        sleep_ms(1);
    }

    return 0;
}

/* End of file. */
