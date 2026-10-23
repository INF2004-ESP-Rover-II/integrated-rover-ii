/**
 * @file motor_task.h
 * @brief FreeRTOS Motor Control task (Subsystem 6) and its interfaces.
 *
 * Who calls what:
 *   Exploration Task (Buddy 1)  motor_task_send_cmd()  autonomous moves only
 *                               motor_task_set_result_cb()  receives results
 *   Robo Comms Task (Buddy 5)   motor_task_send_cmd()  manual moves (dashboard)
 *                               motor_task_operator_stop()  dashboard STOP button
 *                               motor_task_link_lost()      FR-08 link timeout
 *                               motor_task_set_system_ready()  READY gate (FR-40)
 *   UART STOP (ISR)             motor_task_estop_from_isr()  Collision Detection STOP
 *   UART heading (task or ISR)  motor_heading_publish()      20 Hz heading
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "motor_types.h"

/** Called from the Motor Control task with each move result byte
 *  (bits 0..1 move_result_t, bits 2..5 command sequence). Keep it short,
 *  e.g. push the byte into the Exploration Task's queue. */
typedef void (*motor_result_cb_t)(uint8_t result_byte);

/** Create the task. fault_group = the team's shared fault mask (event group);
 *  pass NULL to let the task create its own. */
void motor_task_start(UBaseType_t priority, EventGroupHandle_t fault_group);

/** Queue one packed 4-byte command word. Non-blocking; false if the queue is full. */
bool motor_task_send_cmd(uint32_t cmd_word);

/** Immediate stops (bypass the queue, wake the task at once). */
void motor_task_estop_from_isr(void);   /* from the UART STOP interrupt */
void motor_task_estop(void);            /* same, from a task            */
void motor_task_operator_stop(void);    /* from a task                  */
void motor_task_link_lost(void);        /* from a task                  */

/** Mission Control state gate: manual moves only while READY (FR-40). */
void motor_task_set_system_ready(bool ready);

/** Store the latest heading (tenths of a degree, 0..3599) and its frame
 *  sequence number. One 32-bit store: safe from a task or an ISR. */
void motor_heading_publish(uint16_t heading_tenths, uint8_t seq);

void motor_task_set_result_cb(motor_result_cb_t cb);

/** Telemetry / evidence. */
motor_state_t motor_task_state(void);          /* 3-bit state, single writer */
uint32_t      motor_task_wcet_us(void);        /* worst loop time so far     */
uint32_t      motor_task_last_us(void);        /* last loop time             */
EventGroupHandle_t motor_task_faults(void);

/** Bench testing only (wheel-by-wheel test): the bench suspends the task so
 *  the motor pins still have a single writer. */
TaskHandle_t motor_task_handle(void);

#endif /* MOTOR_TASK_H */
