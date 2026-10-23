/**
 * @file motor_task.c
 * @brief FreeRTOS Motor Control task (Subsystem 6).
 *
 * Timing (prof feedback): the task runs on a FIXED period of MC_PERIOD_MS,
 * like vTaskDelayUntil, so heading hold and ramping run at a steady rate.
 * The difference: it waits with xTaskNotifyWait() until the next period, so
 * a STOP notification wakes it IMMEDIATELY instead of up to 20 ms later.
 *
 * STOP path (NFR-03): UART RX interrupt -> motor_task_estop_from_isr()
 * -> task notification -> this task wakes -> PWM off + brake FIRST, then
 * the fault bit is set. The interrupt never touches the fault mask, and no
 * STOP ever waits behind MQTT or the command queue.
 *
 * Shared data (Change Requirement 1, race-free):
 *   - command queue: 4-byte packed words, FreeRTOS queue (safe by design)
 *   - heading word: one aligned 32-bit store / load (atomic on Cortex-M0+)
 *   - fault mask: FreeRTOS event group, set/cleared only from tasks
 *   - motor state: 3-bit value, single writer (this task)
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_task.h"
#include "motor_control.h"
#include "motor_config.h"
#include "motor_hw.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "pico/time.h"
#include "hardware/timer.h"

#define NOTIFY_ESTOP      (1u << 0)
#define NOTIFY_OP_STOP    (1u << 1)
#define NOTIFY_LINK_LOST  (1u << 2)
#define CMD_QUEUE_LEN     4u
#define MOTOR_STACK_WORDS 512u

static TaskHandle_t        s_task;
static QueueHandle_t       s_queue;
static EventGroupHandle_t  s_faults;
static motor_result_cb_t   s_result_cb;
static mc_t                s_mc;               /* private to the task */

static volatile uint32_t   s_heading_word;     /* one 32-bit store/load    */
static volatile uint8_t    s_ready;            /* written by Robo Comms    */
static volatile uint8_t    s_state;            /* written by this task     */
static volatile uint32_t   s_wcet_us, s_last_us;

static inline uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

/* ------------------------------ publishing ------------------------------ */

static void publish(const mc_out_t *o) {
    if (o->fault_set)   xEventGroupSetBits(s_faults, o->fault_set);
    if (o->fault_clear) xEventGroupClearBits(s_faults, o->fault_clear);
    if (o->result_valid && s_result_cb) s_result_cb(o->result_byte);
    s_state = (uint8_t)mc_state(&s_mc);
}

/* ------------------------------ the task ------------------------------ */

static void motor_task(void *arg) {
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(MC_PERIOD_MS);
    TickType_t next_wake = xTaskGetTickCount() + period;
    mc_out_t out;

    for (;;) {
        /* Sleep until the next period, but wake at once on a STOP. */
        TickType_t now_t = xTaskGetTickCount();
        TickType_t wait = (TickType_t)(next_wake - now_t);
        if ((int32_t)wait < 0 || wait > period) wait = 0;     /* running late: go now */

        uint32_t bits = 0;
        if (xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, wait) == pdTRUE && bits != 0) {
            uint32_t t0 = time_us_32();
            mc_stop_kind_t kind = (bits & NOTIFY_ESTOP)     ? MC_STOP_ESTOP
                                : (bits & NOTIFY_LINK_LOST) ? MC_STOP_LINK_LOST
                                                            : MC_STOP_OPERATOR;
            mc_out_clear(&out);
            mc_immediate_stop(&s_mc, kind, now_ms(), &out);
            motor_hw_apply(&out);          /* PWM off + brake before anything else */
            publish(&out);                 /* then fault bit + result               */
            s_last_us = time_us_32() - t0;
            continue;                      /* keep the same period grid             */
        }

        /* ---------------- periodic update ---------------- */
        next_wake += period;
        if ((int32_t)(xTaskGetTickCount() - next_wake) > (int32_t)period) {
            next_wake = xTaskGetTickCount() + period;   /* resync after a long stall */
        }

        uint32_t t0 = time_us_32();
        uint32_t nm = now_ms();
        uint32_t hw = s_heading_word;      /* single 32-bit load */
        mc_set_system_ready(&s_mc, s_ready != 0);

        uint32_t word;
        while (xQueueReceive(s_queue, &word, 0) == pdTRUE) {   /* VALIDATE each command */
            mc_out_clear(&out);
            mc_command(&s_mc, word, nm, hw, &out);
            publish(&out);
        }

        mc_out_clear(&out);
        mc_step(&s_mc, nm, hw, &out);
        if (out.dir_send) motor_hw_send_dir(out.dir_byte);   /* direction BEFORE motion */
        motor_hw_apply(&out);
        publish(&out);

        uint32_t dt = time_us_32() - t0;
        s_last_us = dt;
        if (dt > s_wcet_us) s_wcet_us = dt;                   /* worst case, for evidence */
    }
}

/* ------------------------------ public API ------------------------------ */

void motor_task_start(UBaseType_t priority, EventGroupHandle_t fault_group) {
    mc_init(&s_mc);
    s_faults = fault_group ? fault_group : xEventGroupCreate();
    s_queue  = xQueueCreate(CMD_QUEUE_LEN, sizeof(uint32_t));   /* 4-byte items */
    motor_hw_init();
    motor_hw_dir_uart_init();
    xTaskCreate(motor_task, "motor", MOTOR_STACK_WORDS, NULL, priority, &s_task);
}

bool motor_task_send_cmd(uint32_t cmd_word) {
    return s_queue && xQueueSend(s_queue, &cmd_word, 0) == pdTRUE;
}

void motor_task_estop_from_isr(void) {
    if (!s_task) return;
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(s_task, NOTIFY_ESTOP, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
}

void motor_task_estop(void)         { if (s_task) xTaskNotify(s_task, NOTIFY_ESTOP, eSetBits); }
void motor_task_operator_stop(void) { if (s_task) xTaskNotify(s_task, NOTIFY_OP_STOP, eSetBits); }
void motor_task_link_lost(void)     { if (s_task) xTaskNotify(s_task, NOTIFY_LINK_LOST, eSetBits); }

void motor_task_set_system_ready(bool ready) { s_ready = ready ? 1u : 0u; }

void motor_heading_publish(uint16_t heading_tenths, uint8_t seq) {
    s_heading_word = heading_word_pack(heading_tenths, seq, now_ms());   /* one store */
}

void motor_task_set_result_cb(motor_result_cb_t cb) { s_result_cb = cb; }

motor_state_t      motor_task_state(void)   { return (motor_state_t)s_state; }
uint32_t           motor_task_wcet_us(void) { return s_wcet_us; }
uint32_t           motor_task_last_us(void) { return s_last_us; }
EventGroupHandle_t motor_task_faults(void)  { return s_faults; }
TaskHandle_t       motor_task_handle(void)  { return s_task; }
