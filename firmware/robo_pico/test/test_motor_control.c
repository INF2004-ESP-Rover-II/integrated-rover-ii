/**
 * @file test_motor_control.c
 * @brief Laptop unit tests for the Motor Control logic (no Pico needed).
 *
 *   cd firmware/robo_pico/test && make
 *
 * A simple simulated rover turns the wheel efforts into a heading change
 * (yaw), and a simulated Sensor Pico publishes that heading at 20 Hz,
 * so rotation and heading hold can be checked end to end.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "motor_control.h"
#include "motor_config.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
    printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ------------------------- simulated rover ------------------------- */
typedef struct {
    double heading_deg;      /* true heading                                  */
    double yaw_per_pct;      /* deg/s per 1 % of turn effort                  */
    double bias_dps;         /* constant yaw disturbance (e.g. weak wheel)    */
    uint32_t now;            /* ms                                            */
    uint32_t hw;             /* heading word as the UART side would write it  */
    uint8_t  hseq;
    bool     imu_alive;      /* false = Sensor Pico stopped sending           */
    mc_t     mc;
    mc_out_t out;
    int      results[8];     /* count per move_result_t                       */
    int      last_result;
} sim_t;

static void sim_publish_heading(sim_t *s) {
    double h = fmod(s->heading_deg, 360.0); if (h < 0) h += 360.0;
    uint16_t tenths = (uint16_t)lround(h * 10.0) % 3600;
    s->hseq = (s->hseq + 1) & 0xF;
    s->hw = heading_word_pack(tenths, s->hseq, s->now);
}

static void sim_collect(sim_t *s) {
    if (s->out.result_valid) {
        int r = s->out.result_byte & 0x3;
        s->results[r]++;
        s->last_result = r;
    }
}

static void sim_init(sim_t *s, double start_heading) {
    memset(s, 0, sizeof *s);
    s->heading_deg = start_heading;
    s->yaw_per_pct = 2.0;    /* 100 % rotate effort ~ 200 deg/s */
    s->imu_alive = true;
    s->last_result = -1;
    mc_init(&s->mc);
    s->now = 1000;
    sim_publish_heading(s);
}

static void sim_cmd(sim_t *s, uint32_t word) {
    mc_out_clear(&s->out);
    mc_command(&s->mc, word, s->now, s->hw, &s->out);
    sim_collect(s);
}

/* advance one 20 ms tick: physics, 20 Hz heading publish, motor step */
static void sim_tick(sim_t *s) {
    s->now += MC_PERIOD_MS;
    int ewz = (s->mc.duty[0] - s->mc.duty[1] + s->mc.duty[2] - s->mc.duty[3]) / 4;
    s->heading_deg += (ewz * s->yaw_per_pct + s->bias_dps) * (MC_PERIOD_MS / 1000.0);
    if (s->imu_alive && (s->now % 50u) < MC_PERIOD_MS) sim_publish_heading(s);
    mc_out_clear(&s->out);
    mc_step(&s->mc, s->now, s->hw, &s->out);
    sim_collect(s);
}

static double wrap_err(double target, double h) {
    double e = fmod(target - h, 360.0);
    if (e > 180) e -= 360;
    if (e < -180) e += 360;
    return e;
}

/* ------------------------------ tests ------------------------------ */

static void test_packing(void) {
    puts("packing (Change Requirement 1)");
    uint32_t w = mc_pack_manual_drive(9, -100, 55, -7);
    CHECK(mc_type(w) == MC_MANUAL_DRIVE && mc_src(w) == MC_SRC_MANUAL && mc_seq(w) == 9, "header");
    CHECK(mc_s8(w, 8) == -100 && mc_s8(w, 16) == 55 && mc_s8(w, 24) == -7, "signed speeds");
    w = mc_pack_auto_drive(15, -30, 359);
    CHECK(mc_s8(w, 8) == -30 && mc_hdg9(w, 16) == 359 && mc_src(w) == MC_SRC_AUTO, "auto drive");
    w = mc_pack_auto_rotate(3, 271);
    CHECK(mc_hdg9(w, 8) == 271, "auto rotate heading");
    uint32_t h = heading_word_pack(3599, 12, 123456);
    CHECK(heading_word_tenths(h) == 3599 && heading_word_seq(h) == 12 &&
          heading_word_time(h) == (123456 & 0x3FF) && heading_word_valid(h), "heading word");
    CHECK(sizeof(uint32_t) == 4, "command word is 4 bytes");
    const char *old_text = "FORWARD:90:100:100:100:100";
    printf("  motor command: old text %zu bytes -> packed %zu bytes\n", strlen(old_text), sizeof(uint32_t));
    printf("  heading: old float %zu bytes -> 12-bit field (2 bytes with 4-bit seq)\n", sizeof(float));
    printf("  sizeof(mc_t) private state = %zu bytes\n", sizeof(mc_t));
}

static void test_helpers(void) {
    puts("heading error wrap + kinematics");
    CHECK(mc_heading_error(100, 3590) == 110, "359 -> 10 deg is +11 deg (got %d)", mc_heading_error(100, 3590));
    CHECK(mc_heading_error(3590, 100) == -110, "10 -> 359 deg is -11 deg");
    CHECK(mc_heading_error(1800, 0) == -1800, "180 deg edge");
    int o[4];
    mc_mix(50, 0, 0, o);  CHECK(o[0] == 50 && o[1] == 50 && o[2] == 50 && o[3] == 50, "forward");
    mc_mix(0, 50, 0, o);  CHECK(o[0] == 50 && o[1] == -50 && o[2] == -50 && o[3] == 50, "strafe right");
    mc_mix(0, 0, 50, o);  CHECK(o[0] == 50 && o[1] == -50 && o[2] == 50 && o[3] == -50, "rotate right");
    mc_mix(80, 80, 0, o); CHECK(o[0] == 100 && o[1] == 0 && o[2] == 0 && o[3] == 100, "diagonal scaled to 100");
    CHECK(mc_dir_mask(50, 0, 0) == DIR_FRONT, "dir forward");
    CHECK(mc_dir_mask(0, -50, 0) == DIR_LEFT, "dir strafe left");
    CHECK(mc_dir_mask(0, 0, 40) == DIR_ALL, "dir rotate = all");
    CHECK(mc_dir_mask(-40, 40, 0) == (DIR_BACK | DIR_RIGHT), "dir diagonal back-right");
}

static void test_zero_gyro_and_ready_gate(void) {
    puts("ZERO GYRO -> IDLE, FR-40 manual only in READY");
    sim_t s; sim_init(&s, 0);
    s.hw = 0;                                   /* no heading yet */
    mc_out_clear(&s.out); mc_step(&s.mc, s.now, s.hw, &s.out);
    CHECK(mc_state(&s.mc) == MS_ZERO_GYRO, "waits in ZERO GYRO without heading");
    sim_publish_heading(&s); sim_tick(&s);
    CHECK(mc_state(&s.mc) == MS_IDLE, "IDLE once heading arrives");

    sim_cmd(&s, mc_pack_manual_drive(1, 40, 0, 0));
    CHECK(s.last_result == MR_REJECTED && mc_state(&s.mc) == MS_IDLE, "manual rejected when not READY");
    mc_set_system_ready(&s.mc, true);
    sim_cmd(&s, mc_pack_manual_drive(2, 40, 0, 0));
    CHECK(mc_state(&s.mc) == MS_MANUAL, "manual accepted in READY");
}

static void test_fr40_source_rules(void) {
    puts("FR-40 source rules");
    sim_t s; sim_init(&s, 0); sim_tick(&s); mc_set_system_ready(&s.mc, true);
    /* autonomous logic may not strafe / diagonal / arc / park */
    uint32_t sneaky = mc_pack_header(MC_MANUAL_DRIVE, MC_SRC_AUTO, 1) | ((uint32_t)(uint8_t)40 << 16);
    sim_cmd(&s, sneaky);
    CHECK(s.last_result == MR_REJECTED && mc_state(&s.mc) == MS_IDLE, "strafe from autonomous rejected");
    uint32_t park_auto = mc_pack_header(MC_MANUAL_PARK, MC_SRC_AUTO, 2) | ((uint32_t)(uint8_t)30 << 16);
    sim_cmd(&s, park_auto);
    CHECK(s.last_result == MR_REJECTED, "park from autonomous rejected");
    /* manual source may not send autonomous moves */
    uint32_t rot_manual = mc_pack_header(MC_AUTO_ROTATE, MC_SRC_MANUAL, 3) | (90u << 8);
    sim_cmd(&s, rot_manual);
    CHECK(s.last_result == MR_REJECTED, "auto rotate from manual source rejected");
    /* manual during autonomous, and autonomous during manual */
    sim_cmd(&s, mc_pack_auto_drive(4, 40, 0));
    CHECK(mc_state(&s.mc) == MS_AUTO_DRIVE, "auto drive accepted");
    sim_cmd(&s, mc_pack_manual_drive(5, 0, 40, 0));
    CHECK(s.last_result == MR_REJECTED && mc_state(&s.mc) == MS_AUTO_DRIVE, "manual during auto rejected");
    sim_cmd(&s, mc_pack_simple(MC_STOP, MC_SRC_AUTO, 6));
    for (int i = 0; i < 30; i++) sim_tick(&s);
    sim_cmd(&s, mc_pack_manual_drive(7, 30, 0, 0));
    CHECK(mc_state(&s.mc) == MS_MANUAL, "manual accepted after stop");
    sim_cmd(&s, mc_pack_auto_rotate(8, 90));
    CHECK(s.last_result == MR_REJECTED && mc_state(&s.mc) == MS_MANUAL, "auto during manual rejected");
    /* out of range */
    uint32_t bad = mc_pack_header(MC_MANUAL_DRIVE, MC_SRC_MANUAL, 9) | ((uint32_t)(uint8_t)(int8_t)120 << 8);
    sim_cmd(&s, bad);
    CHECK(s.last_result == MR_REJECTED, "speed 120 %% rejected");
}

static void test_ramp_and_normal_stop(void) {
    puts("FR-42 ramp up, normal stop ramps down to IDLE");
    sim_t s; sim_init(&s, 0); sim_tick(&s); mc_set_system_ready(&s.mc, true);
    sim_cmd(&s, mc_pack_manual_drive(1, 50, 0, 0));
    sim_tick(&s);
    CHECK(s.mc.duty[0] == MC_RAMP_STEP_PCT, "first tick only +%d %% (got %d)", MC_RAMP_STEP_PCT, s.mc.duty[0]);
    for (int i = 0; i < 4; i++) { sim_tick(&s); if (i % 4 == 0) sim_cmd(&s, mc_pack_manual_drive(1, 50, 0, 0)); }
    CHECK(s.mc.duty[0] == 25, "ramping, 25 %% after 5 ticks (got %d)", s.mc.duty[0]);
    for (int i = 0; i < 10; i++) { sim_tick(&s); if (i % 4 == 0) sim_cmd(&s, mc_pack_manual_drive(1, 50, 0, 0)); }
    CHECK(s.mc.duty[0] == 50, "reaches 50 %%");
    CHECK(s.out.wheel[0] == 20 + (80 * 50) / 100, "deadband applied: 50 %% effort -> %d %% duty (got %d)",
          20 + (80 * 50) / 100, s.out.wheel[0]);
    sim_cmd(&s, mc_pack_simple(MC_STOP, MC_SRC_MANUAL, 2));
    CHECK(mc_state(&s.mc) == MS_RAMP_DOWN, "STOP -> RAMP DOWN");
    sim_tick(&s);
    CHECK(s.mc.duty[0] == 45 && !s.out.brake, "ramping down gradually, no brake");
    for (int i = 0; i < 12; i++) sim_tick(&s);
    CHECK(mc_state(&s.mc) == MS_IDLE && s.results[MR_DONE] == 1, "IDLE + DONE after ramp");
}

static void test_command_timeout(void) {
    puts("FR-42 command timeout -> SAFE STOP, refresh keeps it alive");
    sim_t s; sim_init(&s, 0); sim_tick(&s); mc_set_system_ready(&s.mc, true);
    uint32_t w = mc_pack_manual_drive(1, 60, 0, 0);
    sim_cmd(&s, w);
    for (int i = 0; i < 50; i++) { sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, w); } /* refresh every 100 ms */
    CHECK(mc_state(&s.mc) == MS_MANUAL, "still driving with 100 ms refresh (1 s)");
    int ticks = 0;
    while (mc_state(&s.mc) == MS_MANUAL && ticks < 100) { sim_tick(&s); ticks++; }
    uint32_t stopped_after = (uint32_t)ticks * MC_PERIOD_MS;
    CHECK(mc_state(&s.mc) == MS_SAFE_STOP, "SAFE STOP after refresh stops");
    CHECK(stopped_after > MC_CMD_TIMEOUT_MS - 100 && stopped_after <= MC_CMD_TIMEOUT_MS + 2 * MC_PERIOD_MS + 100,
          "stopped about %u ms after last refresh window (got %u)", MC_CMD_TIMEOUT_MS, stopped_after);
    CHECK((s.out.fault_set & FAULT_CMD_TIMEOUT) && s.out.brake, "timeout fault + brake");
    CHECK(s.out.wheel[0] == 0 && s.mc.duty[0] == 0, "PWM 0 at once, no ramp");
    CHECK(s.results[MR_STOPPED] == 1, "Exploration told STOPPED");
}

static void test_estop_and_reset(void) {
    puts("e-stop -> SAFE STOP latched, operator reset -> IDLE");
    sim_t s; sim_init(&s, 0); sim_tick(&s);
    sim_cmd(&s, mc_pack_auto_drive(1, 60, 0));
    for (int i = 0; i < 15; i++) { sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, mc_pack_auto_drive(1, 60, 0)); }
    CHECK(s.mc.duty[0] > 40, "driving before e-stop");
    mc_out_clear(&s.out);
    mc_immediate_stop(&s.mc, MC_STOP_ESTOP, s.now, &s.out); sim_collect(&s);
    CHECK(mc_state(&s.mc) == MS_SAFE_STOP && s.out.wheel[0] == 0 && s.out.brake &&
          (s.out.fault_set & FAULT_ESTOP), "immediate SAFE STOP, PWM 0, brake, FAULT_ESTOP");
    CHECK(s.last_result == MR_STOPPED, "result STOPPED");
    sim_cmd(&s, mc_pack_auto_drive(2, 60, 0));
    CHECK(s.last_result == MR_REJECTED && mc_state(&s.mc) == MS_SAFE_STOP, "commands ignored in SAFE STOP");
    sim_cmd(&s, mc_pack_simple(MC_RESET, MC_SRC_AUTO, 3));
    CHECK(mc_state(&s.mc) == MS_SAFE_STOP, "reset from autonomous source not accepted");
    sim_cmd(&s, mc_pack_simple(MC_RESET, MC_SRC_MANUAL, 4));
    CHECK(mc_state(&s.mc) == MS_IDLE && (s.out.fault_clear & FAULT_ESTOP), "operator reset -> IDLE, fault cleared");
    for (int i = 0; i < 20; i++) sim_tick(&s);
    CHECK(!s.out.brake, "brake released after hold time");
}

static void test_operator_stop(void) {
    puts("operator stop -> IDLE, PWM 0 at once");
    sim_t s; sim_init(&s, 0); sim_tick(&s); mc_set_system_ready(&s.mc, true);
    sim_cmd(&s, mc_pack_manual_drive(1, 0, -60, 0));
    for (int i = 0; i < 10; i++) sim_tick(&s);
    mc_out_clear(&s.out);
    mc_immediate_stop(&s.mc, MC_STOP_OPERATOR, s.now, &s.out);
    CHECK(mc_state(&s.mc) == MS_IDLE && s.out.wheel[1] == 0 && s.out.brake && s.out.fault_set == 0,
          "IDLE, no fault latched");
}

static void test_heading_stale_and_wrap(void) {
    puts("stale heading -> SAFE STOP; 10-bit wrap can't make old heading look fresh");
    sim_t s; sim_init(&s, 0); sim_tick(&s);
    uint32_t w = mc_pack_auto_drive(1, 50, 0);
    sim_cmd(&s, w);
    for (int i = 0; i < 10; i++) { sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, w); }
    s.imu_alive = false;                                   /* UART goes quiet */
    int ticks = 0;
    while (mc_state(&s.mc) == MS_AUTO_DRIVE && ticks < 50) { sim_tick(&s); if (ticks % 5 == 0) sim_cmd(&s, w); ticks++; }
    CHECK(mc_state(&s.mc) == MS_SAFE_STOP, "SAFE STOP on stale heading");
    CHECK(ticks * (int)MC_PERIOD_MS <= (int)MC_HEADING_STALE_MS + 60, "within ~%u ms (got %d)", MC_HEADING_STALE_MS, ticks * (int)MC_PERIOD_MS);
    /* keep the old word for > 1024 ms: the rolling stamp wraps, latch must hold */
    bool ever_fresh = false;
    for (int i = 0; i < 80; i++) { sim_tick(&s); if (!s.mc.stale_latched) ever_fresh = true; }
    CHECK(!ever_fresh, "still stale after 1.6 s (wrap ignored)");
    s.imu_alive = true;
    for (int i = 0; i < 5; i++) sim_tick(&s);
    CHECK(!s.mc.stale_latched, "fresh again once new frames arrive");
}

static void test_rotate_to_heading(void) {
    puts("FR-41 rotate to heading (simulated rover, heading at 20 Hz)");
    double cases[][2] = { {350, 20}, {0, 90}, {90, 0}, {10, 190}, {180, 179} };
    for (unsigned c = 0; c < sizeof cases / sizeof cases[0]; c++) {
        sim_t s; sim_init(&s, cases[c][0]); sim_tick(&s);
        uint32_t w = mc_pack_auto_rotate((uint8_t)c, (uint16_t)cases[c][1]);
        sim_cmd(&s, w);
        double first_dir = 0;
        int ticks = 0;
        while (s.results[MR_HEADING_REACHED] == 0 && ticks < 400) {
            sim_tick(&s); ticks++;
            if (ticks % 5 == 0) sim_cmd(&s, w);               /* Exploration refreshes */
            if (first_dir == 0 && s.mc.duty[0] != 0) first_dir = s.mc.duty[0];
        }
        for (int i = 0; i < 40; i++) { sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, w); } /* late refreshes */
        double err = wrap_err(cases[c][1], s.heading_deg);
        double expect_dir = wrap_err(cases[c][1], cases[c][0]);
        CHECK(s.results[MR_HEADING_REACHED] == 1, "%.0f->%.0f: HEADING_REACHED once (got %d)",
              cases[c][0], cases[c][1], s.results[MR_HEADING_REACHED]);
        CHECK(fabs(expect_dir) < 1 || fabs(expect_dir) > 179 || (first_dir > 0) == (expect_dir > 0), "%.0f->%.0f turns the short way", cases[c][0], cases[c][1]);
        CHECK(mc_state(&s.mc) == MS_IDLE, "%.0f->%.0f back to IDLE", cases[c][0], cases[c][1]);
        printf("  %5.0f -> %5.0f deg: done in %4d ms, final error %+.1f deg (incl. ramp-down overshoot)\n",
               cases[c][0], cases[c][1], ticks * (int)MC_PERIOD_MS, err);
        CHECK(fabs(err) <= 5.0, "final error within 5 deg");
    }
}

static void test_heading_hold(void) {
    puts("FR-41 heading hold against a weak wheel (yaw disturbance)");
    sim_t s; sim_init(&s, 45); sim_tick(&s);
    s.bias_dps = 8.0;                       /* rover keeps yawing right 8 deg/s on its own */
    uint32_t w = mc_pack_auto_drive(1, MC_AUTO_CRUISE_PCT, 45);
    sim_cmd(&s, w);
    double worst = 0, steady = 0;
    for (int i = 0; i < 250; i++) {          /* 5 s */
        sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, w);
        double e = fabs(wrap_err(45, s.heading_deg));
        if (e > worst) worst = e;
        if (i >= 200 && e > steady) steady = e;
    }
    printf("  worst error %.1f deg, last second %.1f deg (8 deg/s disturbance)\n", worst, steady);
    CHECK(worst <= 3.0, "held within +/-3 deg the whole time");
    CHECK(steady <= 1.0, "settles within 1 deg (integral removes the offset)");

    sim_t n; sim_init(&n, 45); sim_tick(&n); n.bias_dps = 8.0;
    /* same disturbance, no hold: drive via manual with w=0 but no heading -> compare */
    double drift = 8.0 * 4.0;
    printf("  without heading hold the same rover would drift ~%.0f deg in 4 s\n", drift);
}

static void test_direction_byte(void) {
    puts("direction byte for Collision Detection");
    sim_t s; sim_init(&s, 0); sim_tick(&s); mc_set_system_ready(&s.mc, true);
    uint32_t fwd = mc_pack_manual_drive(1, 60, 0, 0);
    sim_cmd(&s, fwd);
    sim_tick(&s);
    CHECK(s.out.dir_send && (s.out.dir_byte & 0xF) == DIR_FRONT, "FRONT sent on the first tick");
    for (int i = 0; i < 15; i++) { sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, fwd); }
    uint32_t left = mc_pack_manual_drive(2, 0, -60, 0);
    sim_cmd(&s, left);
    sim_tick(&s);
    CHECK((s.out.dir_byte & 0xF) == (DIR_FRONT | DIR_LEFT), "switching: FRONT|LEFT while forward ramps out (got 0x%X)", s.out.dir_byte & 0xF);
    for (int i = 0; i < 30; i++) { sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, left); }
    int sends = 0, last = -1;
    for (int i = 0; i < 25; i++) { sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, left); if (s.out.dir_send) { sends++; last = s.out.dir_byte & 0xF; } }
    CHECK(last == DIR_LEFT, "LEFT once settled");
    CHECK(sends >= 4 && sends <= 6, "refreshed about every 100 ms (%d sends in 500 ms)", sends);
    sim_cmd(&s, mc_pack_manual_drive(3, 0, 0, 40));
    sim_tick(&s);
    CHECK((s.out.dir_byte & 0xF) == DIR_ALL, "rotate -> ALL");
}

static void test_start_boost(void) {
    puts("start boost: full kick when a wheel starts, then the requested speed");
    sim_t s; sim_init(&s, 0); sim_tick(&s); mc_set_system_ready(&s.mc, true);
    uint32_t w = mc_pack_manual_drive(1, 30, 0, 0);
    sim_cmd(&s, w);
    sim_tick(&s);
    CHECK(s.out.wheel[0] == MC_KICK_PCT, "first tick gets the boost (%d, got %d)", MC_KICK_PCT, s.out.wheel[0]);
    for (int i = 0; i < 20; i++) { sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, w); }
    int expect = 20 + 80 * 30 / 100;
    CHECK(s.out.wheel[0] == expect, "after the boost: normal %d %% (got %d)", expect, s.out.wheel[0]);
    mc_set_kick(0, 0);
    sim_t n; sim_init(&n, 0); sim_tick(&n); mc_set_system_ready(&n.mc, true);
    sim_cmd(&n, w); sim_tick(&n);
    CHECK(n.out.wheel[0] == 20 + 80 * 5 / 100, "kick 0 turns the boost off (got %d)", n.out.wheel[0]);
    mc_set_kick(MC_KICK_PCT, (int)MC_KICK_MS);
}

static void test_park(void) {
    puts("FR-39 parallel park ends by itself");
    sim_t s; sim_init(&s, 0); sim_tick(&s); mc_set_system_ready(&s.mc, true);
    uint32_t w = mc_pack_manual_park(1, -MC_PARK_PCT);
    sim_cmd(&s, w);
    int ticks = 0;
    while (mc_state(&s.mc) != MS_IDLE && ticks < 200) { sim_tick(&s); if (ticks % 5 == 0) sim_cmd(&s, w); ticks++; }
    CHECK(mc_state(&s.mc) == MS_IDLE && s.results[MR_DONE] == 1, "park -> RAMP DOWN -> IDLE + DONE");
}

static void test_zigzag_strafe(void) {
    puts("zigzag strafe: diagonals in turn, 2 wheels at full, no forward drift on average");
    mc_set_zigzag(200);
    sim_t s; sim_init(&s, 0); sim_tick(&s); mc_set_system_ready(&s.mc, true);
    uint32_t w = mc_pack_manual_drive(1, 0, 100, 0);          /* strafe right */
    sim_cmd(&s, w);
    int fr_phase = 0, br_phase = 0, other = 0;
    for (int i = 0; i < 100; i++) {                           /* 2 s */
        sim_tick(&s); if (i % 5 == 0) sim_cmd(&s, w);
        int8_t *d = s.mc.duty;
        if (d[0] == 100 && d[3] == 100 && d[1] == 0 && d[2] == 0) fr_phase++;
        else if (d[1] == -100 && d[2] == -100 && d[0] == 0 && d[3] == 0) br_phase++;
        else other++;
    }
    CHECK(fr_phase > 30 && br_phase > 30, "both diagonals used (front-right %d, back-right %d ticks)", fr_phase, br_phase);
    CHECK(other == 0, "always exactly 2 wheels at full (other %d)", other);
    CHECK(abs(fr_phase - br_phase) <= 10, "about equal time each way, so no net forward drift");
    sim_cmd(&s, mc_pack_simple(MC_STOP, MC_SRC_MANUAL, 2));
    for (int i = 0; i < 40; i++) sim_tick(&s);
    CHECK(mc_state(&s.mc) == MS_IDLE, "normal stop still ramps down to IDLE");
    mc_set_zigzag(0);
}

int main(void) {
    mc_set_zigzag(0);   /* the older tests check the normal 4-wheel strafe */
    test_packing();
    test_helpers();
    test_zero_gyro_and_ready_gate();
    test_fr40_source_rules();
    test_ramp_and_normal_stop();
    test_command_timeout();
    test_estop_and_reset();
    test_operator_stop();
    test_heading_stale_and_wrap();
    test_rotate_to_heading();
    test_heading_hold();
    test_direction_byte();
    test_start_boost();
    test_park();
    test_zigzag_strafe();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
