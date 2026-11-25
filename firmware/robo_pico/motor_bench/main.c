#include <inttypes.h>
/**
 * @file main.c
 * @brief Motor bench app: runs ONLY the Motor Control task on the Robo Pico,
 *        driven from a USB serial terminal. Use it to test Subsystem 6 on the
 *        real rover before integration (wheels off the ground first!).
 *
 * Type a command and press Enter (115200, any serial monitor):
 *   ready 1|0        Mission state gate (manual moves need ready 1, FR-40)
 *   m vx vy w        manual drive, -100..100 each (held until 's')
 *                      m 40 0 0 forward | m 0 -40 0 strafe left
 *                      m 30 30 0 diagonal | m 40 0 20 arc | m 0 0 40 rotate
 *   park -1|1 [pwr]  parallel park left / right, optional power 10..100
 *                    (default 60; ends by itself after 1.5 s)
 *   r deg            AUTO rotate to heading 0..359
 *   d speed deg      AUTO drive (speed -100..100) holding heading deg
 *   s                normal stop (ramp down)
 *   x                operator stop (PWM 0 at once -> IDLE)
 *   e                emergency stop (as if Collision Detection sent STOP)
 *   l                link lost (as if Robo Comms timed out)
 *   reset            operator reset from SAFE STOP
 *   h deg | h off    bench heading: publish a fixed heading at 20 Hz, or stop
 *                    publishing (tests the stale-heading stop)
 *   t [duty]         wheel test: each wheel alone, forward then back
 *                    (duty 10..100 %, default 60; try 't 90' if nothing moves)
 *   db N             minimum starting power for all wheels (0..90 %)
 *   db W N           ... for one wheel (W = 0 FL, 1 FR, 2 BL, 3 BR)
 *   trim W FWD REV   scale one wheel down, separately forward / backward
 *                    (100 = full; e.g. 'trim 0 90 100' slows FL forward only)
 *   pwm              shows the PWM frequency (fixed per build: 1 kHz or 10 kHz)
 *   kick PCT MS      start boost: PCT power for MS ms when a wheel starts
 *                    (default kick 90 150; 'kick 0 0' turns it off)
 *   zz MS            zigzag strafe: switch diagonal every MS ms (0 = off)
 *   q                print status now
 *
 * The held command is refreshed every 100 ms, like the dashboard / Exploration
 * must do (command timeout 300 ms). Without a Sensor Pico, use 'h' to give
 * the rover a heading: e.g. 'r 90' starts rotating, 'h 89' finishes it.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "motor_task.h"
#include "motor_hw.h"
#include "motor_control.h"
#include "motor_config.h"

static volatile int     g_s_last_result = -1;
static volatile uint8_t g_s_last_result_seq;

static void
on_result (uint8_t second)
{ /* runs in the motor task: keep short */
    g_s_last_result     = second & 0x3;
    g_s_last_result_seq = (uint8_t) ((second >> 2) & 0xF);
}

static const char *
state_name (motor_state_t state)
{
    static const char * count[] = { "ZERO_GYRO", "IDLE",        "VALIDATE",
                                    "MANUAL",    "AUTO_ROTATE", "AUTO_DRIVE",
                                    "RAMP_DOWN", "SAFE_STOP" };
    return count[state & 7];
}
static const char *
result_name (int result)
{
    static const char * count[] = { "DONE", "HEADING_REACHED", "STOPPED",
                                    "REJECTED" };
    return (result >= 0 && result < 4) ? count[result] : "-";
}

static void
print_status (int heading_tenths)
{
    EventBits_t f = xEventGroupGetBits(motor_task_faults());
    printf("[motor] state=%s faults=0x%02" PRIx32
           " last_result=%s(seq %u) loop=%" PRIu32 "us wcet=%" PRIu32
           "us heading=%s",
           state_name(motor_task_state()), (uint32_t) f,
           result_name(g_s_last_result), g_s_last_result_seq,
           (uint32_t) motor_task_last_us(), (uint32_t) motor_task_wcet_us(),
           heading_tenths >= 0 ? "" : "off");
    if (heading_tenths >= 0)
    {
        printf("%d.%d", heading_tenths / 10, heading_tenths % 10);
    }
    printf(" pwm=%" PRIu32 "Hz minpower=%d/%d/%d/%d%% kick=%d%%/%dms",
           (uint32_t) motor_hw_pwm_freq(), mc_get_deadband(0),
           mc_get_deadband(1), mc_get_deadband(2), mc_get_deadband(3),
           mc_get_kick_pct(), mc_get_kick_ms());
    printf(" trim fwd=%d/%d/%d/%d rev=%d/%d/%d/%d", mc_get_trim_fwd(0),
           mc_get_trim_fwd(1), mc_get_trim_fwd(2), mc_get_trim_fwd(3),
           mc_get_trim_rev(0), mc_get_trim_rev(1), mc_get_trim_rev(2),
           mc_get_trim_rev(3));
    printf(" stack_free=%" PRIu32 " words\n",
           (uint32_t) uxTaskGetStackHighWaterMark(motor_task_handle()));
}

static void
wheel_test (int duty)
{
    static const char * names[4] = { "front-left", "front-right", "back-left",
                                     "back-right" };
    motor_task_operator_stop();
    vTaskDelay(pdMS_TO_TICKS(400));
    vTaskSuspend(motor_task_handle()); /* single writer while testing */
    mc_out_t o;
    for (int value_w = 0; value_w < 4; value_w++)
    {
        for (int dir = 1; dir >= -1; dir -= 2)
        {
            memset(&o, 0, sizeof o);
            o.outputs_valid  = 1;
            o.wheel[value_w] = (int8_t) (duty * dir);
            printf("[test] %s %s %d%%: check it turns %s\n", names[value_w],
                   dir > 0 ? "forward" : "backward", duty,
                   dir > 0 ? "FORWARD" : "BACKWARD");
            motor_hw_apply(&o);
            vTaskDelay(pdMS_TO_TICKS(1000));
            motor_hw_all_off();
            vTaskDelay(pdMS_TO_TICKS(400));
        }
    }
    printf(
        "[test] done. If a wheel turned the wrong way, set it in MC_INVERT.\n");
    vTaskResume(motor_task_handle());
}

static void
bench_task (void * p_arg)
{
    (void) p_arg;
    char       line[48];
    int        len = 0;
    uint8_t    seq = 0, hseq = 0;
    uint32_t   held           = 0;  /* command refreshed every 100 ms */
    int        heading_tenths = -1; /* -1 = not publishing            */
    TickType_t last_refresh = 0, last_hdg = 0, last_status = 0;

    printf("\n[bench] Motor Control bench ready. Type 'q' for status.\n");
    for (;;)
    {
        int value_c = getchar_timeout_us(0);
        /* Serial monitors set to "no line ending" never send Enter:
         * treat a 150 ms pause after typing as the end of the command. */
        static TickType_t last_char = 0;
        if (value_c == PICO_ERROR_TIMEOUT && len > 0 &&
            xTaskGetTickCount() - last_char >= pdMS_TO_TICKS(150))
        {
            value_c = '\n';
        }
        if (value_c != PICO_ERROR_TIMEOUT)
        {
            last_char = xTaskGetTickCount();
            if (value_c == '\r' || value_c == '\n')
            {
                line[len] = 0;
                len       = 0;
                if (line[0] == 0)
                {
                    continue;
                }
                printf("[bench] > %s\n", line); /* echo: command received */
                seq            = (uint8_t) ((seq + 1) & 0xF);
                int      first = 0, second = 0, data = 0;
                uint32_t value_w = 0;
                if (sscanf(line, "ready %d", &first) == 1)
                {
                    motor_task_set_system_ready(first != 0);
                    printf("[bench] ready=%d\n", first != 0);
                }
                else if (sscanf(line, "m %d %d %d", &first, &second, &data) ==
                         3)
                {
                    value_w = mc_pack_manual_drive(
                        seq, (int8_t) first, (int8_t) second, (int8_t) data);
                }
                else if (sscanf(line, "park %d %d", &first, &second) == 2)
                {
                    second  = second < 10 ? 10 : (second > 100 ? 100 : second);
                    value_w = mc_pack_manual_park(
                        seq, (int8_t) (first < 0 ? -second : second));
                }
                else if (sscanf(line, "park %d", &first) == 1)
                {
                    value_w = mc_pack_manual_park(
                        seq, (int8_t) (first < 0 ? -MC_PARK_PCT : MC_PARK_PCT));
                }
                else if (sscanf(line, "zz %d", &first) == 1)
                {
                    mc_set_zigzag(first);
                    printf("[bench] zigzag strafe = %d ms%s\n", mc_get_zigzag(),
                           mc_get_zigzag() ? "" : " (off)");
                }
                else if (sscanf(line, "kick %d %d", &first, &second) == 2)
                {
                    mc_set_kick(first, second);
                    printf("[bench] start boost = %d%% for %d ms\n",
                           mc_get_kick_pct(), mc_get_kick_ms());
                }
                else if (sscanf(line, "r %d", &first) == 1)
                {
                    value_w = mc_pack_auto_rotate(seq, (uint16_t) first);
                }
                else if (sscanf(line, "d %d %d", &first, &second) == 2)
                {
                    value_w = mc_pack_auto_drive(seq, (int8_t) first,
                                                 (uint16_t) second);
                }
                else if (!strcmp(line, "s"))
                {
                    held = 0;
                    motor_task_send_cmd(
                        mc_pack_simple(MC_STOP, MC_SRC_MANUAL, seq));
                }
                else if (!strcmp(line, "x"))
                {
                    held = 0;
                    motor_task_operator_stop();
                }
                else if (!strcmp(line, "e"))
                {
                    held = 0;
                    motor_task_estop();
                }
                else if (!strcmp(line, "l"))
                {
                    held = 0;
                    motor_task_link_lost();
                }
                else if (!strcmp(line, "reset"))
                {
                    held = 0;
                    motor_task_send_cmd(
                        mc_pack_simple(MC_RESET, MC_SRC_MANUAL, seq));
                }
                else if (!strcmp(line, "h off"))
                {
                    heading_tenths = -1;
                    printf("[bench] heading publishing OFF\n");
                }
                else if (sscanf(line, "h %d", &first) == 1)
                {
                    heading_tenths = (first % 360 + 360) % 360 * 10;
                }
                else if (line[0] == 't' && (line[1] == 0 || line[1] == ' '))
                {
                    int duty = 60;
                    if (sscanf(line, "t %d", &first) == 1)
                    {
                        duty = first < 10 ? 10 : (first > 100 ? 100 : first);
                    }
                    held = 0;
                    wheel_test(duty);
                }
                else if (!strcmp(line, "q"))
                {
                    print_status(heading_tenths);
                }
                else if (sscanf(line, "trim %d %d %d", &first, &second,
                                &data) == 3)
                {
                    mc_set_trim(first, second, data);
                    printf(
                        "[bench] wheel %d trim: forward %d%%, backward %d%%\n",
                        first, mc_get_trim_fwd(first < 0 ? 0 : first),
                        mc_get_trim_rev(first < 0 ? 0 : first));
                }
                else if (sscanf(line, "db %d %d", &first, &second) == 2)
                {
                    held = 0;
                    motor_task_operator_stop();
                    vTaskDelay(pdMS_TO_TICKS(50));
                    mc_set_deadband(first, second);
                    printf("[bench] wheel %d min power = %d%%\n", first,
                           mc_get_deadband(first));
                }
                else if (sscanf(line, "db %d", &first) == 1)
                {
                    held = 0;
                    motor_task_operator_stop();
                    vTaskDelay(pdMS_TO_TICKS(50));
                    mc_set_deadband(-1, first);
                    printf("[bench] all wheels min power = %d%%\n",
                           mc_get_deadband(0));
                }
                else if (sscanf(line, "pwm %d", &first) == 1)
                {
                    /* Changing PWM at run time crashed the Pico on the rover,
                     * so the frequency is fixed per firmware build instead. */
                    printf("[bench] PWM is fixed at %" PRIu32
                           " Hz in this firmware. Flash motor_bench_1k.uf2 or "
                           "motor_bench_10k.uf2 to "
                           "change it.\n",
                           (uint32_t) motor_hw_pwm_freq());
                }
                else
                {
                    printf("[bench] unknown: %s\n", line);
                }

                if (value_w)
                {
                    held = value_w; /* refreshed until it ends; late refreshes
                                       are ignored */
                    if (!motor_task_send_cmd(value_w))
                    {
                        printf("[bench] queue full\n");
                    }
                }
            }
            else if (len < (int) sizeof line - 1)
            {
                line[len++] = (char) value_c;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (held &&
            now - last_refresh >= pdMS_TO_TICKS(MC_RECOMMENDED_REFRESH_MS))
        {
            motor_task_send_cmd(held); /* refresh, same word */
            last_refresh = now;
        }
        if (heading_tenths >= 0 && now - last_hdg >= pdMS_TO_TICKS(50))
        { /* 20 Hz */
            hseq = (uint8_t) ((hseq + 1) & 0xF);
            motor_heading_publish((uint16_t) heading_tenths, hseq);
            last_hdg = now;
        }
        if (now - last_status >= pdMS_TO_TICKS(1000))
        {
            print_status(heading_tenths);
            last_status = now;
        }
        if (held &&
            (g_s_last_result == MR_STOPPED || g_s_last_result == MR_REJECTED ||
             g_s_last_result == MR_HEADING_REACHED ||
             g_s_last_result == MR_DONE) &&
            g_s_last_result_seq == mc_seq(held))
        {
            printf("[bench] move %u ended: %s\n", mc_seq(held),
                   result_name(g_s_last_result));
            held = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

int
main (void)
{
    motor_hw_safe_boot(); /* FIRST: claw/arm and wheels off */
    stdio_init_all();

    EventGroupHandle_t faults = xEventGroupCreate();
    motor_task_set_result_cb(on_result);
    motor_task_start(configMAX_PRIORITIES - 2, faults); /* high priority */
    xTaskCreate(bench_task, "bench", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;)
    {
    }
}

/* FreeRTOS hooks */
void
vApplicationMallocFailedHook (void)
{
    panic("malloc failed");
}
void
vApplicationStackOverflowHook (TaskHandle_t time_value, char * p_name)
{
    (void) time_value;
    panic("stack overflow: %s", p_name);
}

/* End of file. */
