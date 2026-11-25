/**
 * @file motor_heading.c
 * @brief Heading input, offset and zeroing (FR-37, FR-41).
 *
 * Part of the motor driver; see motor_driver.c for the file overview.
 *
 * Owner: Buddy 4 (Lim Zi Ying)
 */
#include "motor_driver.h"
#include "motor_driver_internal.h"

#include "motor_config.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "motor_types.h"

#include <math.h>
#include "hardware/sync.h"

/* Heading input from the Sensor Pico. Used from interrupts, so volatile
 * (BARR-C 1.8.c). The packed heading word shared with the timer is
 * g_motor.heading_word. */
static volatile uint8_t  s_hdg_seq;
static volatile float    s_raw_heading;
static volatile float    s_heading_offset;
static volatile uint32_t s_raw_heading_ms;
static volatile uint32_t s_heading_epoch;
static volatile bool     s_have_raw_heading;

/* Called from motor_init(). */
void motor_heading_reset (void)
{
    s_heading_offset   = 0;
    s_heading_epoch    = 0;
    s_have_raw_heading = false;
}

static float wrap_heading (float heading_deg)
{
    float wrapped = fmodf(heading_deg, 360.0f);
    return wrapped < 0 ? wrapped + 360.0f : wrapped;
}

static void store_heading (float heading_deg)
{
    uint16_t tenths = (uint16_t)(heading_deg * 10.0f + 0.5f);
    if (tenths >= 3600u)
    {
        tenths = 0;
    }
    s_hdg_seq = (uint8_t)((s_hdg_seq + 1u) & 0xFu);
    /* One 32-bit store, so the timer never sees half an update. */
    g_motor.heading_word = heading_word_pack(tenths, s_hdg_seq, motor_now_ms());
}

void motor_heading_update (float heading_deg, bool valid)
{
    if (!valid || !isfinite(heading_deg))
    {
        return;
    }
    uint32_t irq       = save_and_disable_interrupts();
    s_raw_heading      = wrap_heading(heading_deg);
    s_raw_heading_ms   = motor_now_ms();
    s_have_raw_heading = true;
    store_heading(wrap_heading(s_raw_heading - s_heading_offset));
    restore_interrupts(irq);
}

float motor_heading_relative (float raw_heading_deg)
{
    uint32_t irq      = save_and_disable_interrupts();
    float    relative = wrap_heading(raw_heading_deg - s_heading_offset);
    restore_interrupts(irq);
    return relative;
}

float motor_heading_offset (void)
{
    uint32_t irq    = save_and_disable_interrupts();
    float    offset = s_heading_offset;
    restore_interrupts(irq);
    return offset;
}

uint32_t motor_heading_epoch (void)
{
    uint32_t irq   = save_and_disable_interrupts();
    uint32_t epoch = s_heading_epoch;
    restore_interrupts(irq);
    return epoch;
}

bool motor_heading_sample_since (uint32_t since_ms)
{
    uint32_t irq = save_and_disable_interrupts();
    bool     received =
        s_have_raw_heading && (int32_t)(s_raw_heading_ms - since_ms) >= 0;
    restore_interrupts(irq);
    return received;
}

motor_error_t motor_zero_heading (void)
{
    uint32_t irq = save_and_disable_interrupts();
    if (!s_have_raw_heading ||
        (uint32_t)(motor_now_ms() - s_raw_heading_ms) > MC_HEADING_STALE_MS)
    {
        restore_interrupts(irq);
        return MOTOR_ERR_NO_HEADING;
    }
    motor_core_hard_stop(MC_STOP_OPERATOR);
    motor_hw_arm(MH_OFF);
    motor_hw_claw(MH_OFF);
    s_heading_offset = s_raw_heading;
    ++s_heading_epoch;
    store_heading(0);
    mc_out_t out;
    mc_out_clear(&out);
    mc_step(&g_motor.mc, motor_now_ms(), g_motor.heading_word, &out);
    motor_core_apply(&out);
    restore_interrupts(irq);
    return MOTOR_OK;
}
