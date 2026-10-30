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

static volatile int     s_last_result = -1;
static volatile uint8_t s_last_result_seq;

static void on_result(uint8_t b) {           /* runs in the motor task: keep short */
    s_last_result = b & 0x3;
    s_last_result_seq = (uint8_t)((b >> 2) & 0xF);
}

static const char *state_name(motor_state_t s) {
    static const char *n[] = { "ZERO_GYRO", "IDLE", "VALIDATE", "MANUAL",
                               "AUTO_ROTATE", "AUTO_DRIVE", "RAMP_DOWN", "SAFE_STOP" };
    return n[s & 7];
}
static const char *result_name(int r) {
    static const char *n[] = { "DONE", "HEADING_REACHED", "STOPPED", "REJECTED" };
    return (r >= 0 && r < 4) ? n[r] : "-";
}

static void print_status(int heading_tenths) {
    EventBits_t f = xEventGroupGetBits(motor_task_faults());
    printf("[motor] state=%s faults=0x%02lx last_result=%s(seq %u) loop=%luus wcet=%luus heading=%s",
           state_name(motor_task_state()), (unsigned long)f, result_name(s_last_result),
           s_last_result_seq, (unsigned long)motor_task_last_us(),
           (unsigned long)motor_task_wcet_us(), heading_tenths >= 0 ? "" : "off");
    if (heading_tenths >= 0) printf("%d.%d", heading_tenths / 10, heading_tenths % 10);
    printf(" pwm=%luHz minpower=%d/%d/%d/%d%% kick=%d%%/%dms", (unsigned long)motor_hw_pwm_freq(),
           mc_get_deadband(0), mc_get_deadband(1), mc_get_deadband(2), mc_get_deadband(3),
           mc_get_kick_pct(), mc_get_kick_ms());
    printf(" trim fwd=%d/%d/%d/%d rev=%d/%d/%d/%d",
           mc_get_trim_fwd(0), mc_get_trim_fwd(1), mc_get_trim_fwd(2), mc_get_trim_fwd(3),
           mc_get_trim_rev(0), mc_get_trim_rev(1), mc_get_trim_rev(2), mc_get_trim_rev(3));
    printf(" stack_free=%lu words\n",
           (unsigned long)uxTaskGetStackHighWaterMark(motor_task_handle()));
}

static void wheel_test(int duty) {
    static const char *names[4] = { "front-left", "front-right", "back-left", "back-right" };
    motor_task_operator_stop();
    vTaskDelay(pdMS_TO_TICKS(400));
    vTaskSuspend(motor_task_handle());           /* single writer while testing */
    mc_out_t o;
    for (int w = 0; w < 4; w++) {
        for (int dir = 1; dir >= -1; dir -= 2) {
            memset(&o, 0, sizeof o);
            o.outputs_valid = 1;
            o.wheel[w] = (int8_t)(duty * dir);
            printf("[test] %s %s %d%%: check it turns %s\n", names[w],
                   dir > 0 ? "forward" : "backward", duty, dir > 0 ? "FORWARD" : "BACKWARD");
            motor_hw_apply(&o);
            vTaskDelay(pdMS_TO_TICKS(1000));
            motor_hw_all_off();
            vTaskDelay(pdMS_TO_TICKS(400));
        }
    }
    printf("[test] done. If a wheel turned the wrong way, set it in MC_INVERT.\n");
    vTaskResume(motor_task_handle());
}

static void bench_task(void *arg) {
    (void)arg;
    char line[48];
    int len = 0;
    uint8_t seq = 0, hseq = 0;
    uint32_t held = 0;                 /* command refreshed every 100 ms */
    int heading_tenths = -1;           /* -1 = not publishing            */
    TickType_t last_refresh = 0, last_hdg = 0, last_status = 0;

    printf("\n[bench] Motor Control bench ready. Type 'q' for status.\n");
    for (;;) {
        int c = getchar_timeout_us(0);
        /* Serial monitors set to "no line ending" never send Enter:
         * treat a 150 ms pause after typing as the end of the command. */
        static TickType_t last_char = 0;
        if (c == PICO_ERROR_TIMEOUT && len > 0 &&
            xTaskGetTickCount() - last_char >= pdMS_TO_TICKS(150)) c = '\n';
        if (c != PICO_ERROR_TIMEOUT) {
            last_char = xTaskGetTickCount();
            if (c == '\r' || c == '\n') {
                line[len] = 0;
                len = 0;
                if (line[0] == 0) continue;
                printf("[bench] > %s\n", line);          /* echo: command received */
                seq = (uint8_t)((seq + 1) & 0xF);
                int a = 0, b = 0, d = 0;
                uint32_t w = 0;
                if (sscanf(line, "ready %d", &a) == 1) { motor_task_set_system_ready(a != 0); printf("[bench] ready=%d\n", a != 0); }
                else if (sscanf(line, "m %d %d %d", &a, &b, &d) == 3) w = mc_pack_manual_drive(seq, (int8_t)a, (int8_t)b, (int8_t)d);
                else if (sscanf(line, "park %d %d", &a, &b) == 2) {
                    b = b < 10 ? 10 : (b > 100 ? 100 : b);
                    w = mc_pack_manual_park(seq, (int8_t)(a < 0 ? -b : b));
                }
                else if (sscanf(line, "park %d", &a) == 1) w = mc_pack_manual_park(seq, (int8_t)(a < 0 ? -MC_PARK_PCT : MC_PARK_PCT));
                else if (sscanf(line, "zz %d", &a) == 1) {
                    mc_set_zigzag(a);
                    printf("[bench] zigzag strafe = %d ms%s\n", mc_get_zigzag(), mc_get_zigzag() ? "" : " (off)");
                }
                else if (sscanf(line, "kick %d %d", &a, &b) == 2) {
                    mc_set_kick(a, b);
                    printf("[bench] start boost = %d%% for %d ms\n", mc_get_kick_pct(), mc_get_kick_ms());
                }
                else if (sscanf(line, "r %d", &a) == 1)    w = mc_pack_auto_rotate(seq, (uint16_t)a);
                else if (sscanf(line, "d %d %d", &a, &b) == 2) w = mc_pack_auto_drive(seq, (int8_t)a, (uint16_t)b);
                else if (!strcmp(line, "s"))     { held = 0; motor_task_send_cmd(mc_pack_simple(MC_STOP, MC_SRC_MANUAL, seq)); }
                else if (!strcmp(line, "x"))     { held = 0; motor_task_operator_stop(); }
                else if (!strcmp(line, "e"))     { held = 0; motor_task_estop(); }
                else if (!strcmp(line, "l"))     { held = 0; motor_task_link_lost(); }
                else if (!strcmp(line, "reset")) { held = 0; motor_task_send_cmd(mc_pack_simple(MC_RESET, MC_SRC_MANUAL, seq)); }
                else if (!strcmp(line, "h off")) { heading_tenths = -1; printf("[bench] heading publishing OFF\n"); }
                else if (sscanf(line, "h %d", &a) == 1) { heading_tenths = (a % 360 + 360) % 360 * 10; }
                else if (line[0] == 't' && (line[1] == 0 || line[1] == ' ')) {
                    int duty = 60;
                    if (sscanf(line, "t %d", &a) == 1) duty = a < 10 ? 10 : (a > 100 ? 100 : a);
                    held = 0; wheel_test(duty);
                }
                else if (!strcmp(line, "q"))     { print_status(heading_tenths); }
                else if (sscanf(line, "trim %d %d %d", &a, &b, &d) == 3) {
                    mc_set_trim(a, b, d);
                    printf("[bench] wheel %d trim: forward %d%%, backward %d%%\n",
                           a, mc_get_trim_fwd(a < 0 ? 0 : a), mc_get_trim_rev(a < 0 ? 0 : a));
                }
                else if (sscanf(line, "db %d %d", &a, &b) == 2) {
                    held = 0; motor_task_operator_stop(); vTaskDelay(pdMS_TO_TICKS(50));
                    mc_set_deadband(a, b);
                    printf("[bench] wheel %d min power = %d%%\n", a, mc_get_deadband(a));
                }
                else if (sscanf(line, "db %d", &a) == 1) {
                    held = 0; motor_task_operator_stop(); vTaskDelay(pdMS_TO_TICKS(50));
                    mc_set_deadband(-1, a);
                    printf("[bench] all wheels min power = %d%%\n", mc_get_deadband(0));
                }
                else if (sscanf(line, "pwm %d", &a) == 1) {
                    /* Changing PWM at run time crashed the Pico on the rover,
                     * so the frequency is fixed per firmware build instead. */
                    printf("[bench] PWM is fixed at %lu Hz in this firmware. Flash motor_bench_1k.uf2 or motor_bench_10k.uf2 to change it.\n",
                           (unsigned long)motor_hw_pwm_freq());
                }
                else printf("[bench] unknown: %s\n", line);

                if (w) {
                    held = w;   /* refreshed until it ends; late refreshes are ignored */
                    if (!motor_task_send_cmd(w)) printf("[bench] queue full\n");
                }
            } else if (len < (int)sizeof line - 1) {
                line[len++] = (char)c;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (held && now - last_refresh >= pdMS_TO_TICKS(MC_RECOMMENDED_REFRESH_MS)) {
            motor_task_send_cmd(held);                    /* refresh, same word */
            last_refresh = now;
        }
        if (heading_tenths >= 0 && now - last_hdg >= pdMS_TO_TICKS(50)) {   /* 20 Hz */
            hseq = (uint8_t)((hseq + 1) & 0xF);
            motor_heading_publish((uint16_t)heading_tenths, hseq);
            last_hdg = now;
        }
        if (now - last_status >= pdMS_TO_TICKS(1000)) {
            print_status(heading_tenths);
            last_status = now;
        }
        if (held && (s_last_result == MR_STOPPED || s_last_result == MR_REJECTED ||
                     s_last_result == MR_HEADING_REACHED || s_last_result == MR_DONE) &&
            s_last_result_seq == mc_seq(held)) {
            printf("[bench] move %u ended: %s\n", mc_seq(held), result_name(s_last_result));
            held = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

int main(void) {
    motor_hw_safe_boot();              /* FIRST: claw/arm and wheels off */
    stdio_init_all();

    EventGroupHandle_t faults = xEventGroupCreate();
    motor_task_set_result_cb(on_result);
    motor_task_start(configMAX_PRIORITIES - 2, faults);   /* high priority */
    xTaskCreate(bench_task, "bench", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;) { }
}

/* FreeRTOS hooks */
void vApplicationMallocFailedHook(void) { panic("malloc failed"); }
void vApplicationStackOverflowHook(TaskHandle_t t, char *name) { (void)t; panic("stack overflow: %s", name); }
