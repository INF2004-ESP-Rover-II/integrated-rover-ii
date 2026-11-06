/* test_map_port.c - replays the vectors made by rover_mapping/tools_gen_c_vectors.py on the C code.
 * The vectors record what the PYTHON mapping/planner did, so this proves the C port behaves the same.
 *   make map     (from firmware/robo_pico/test)
 * Also has a few hand-written checks at the end. Host only (uses stdio); the Pico build does not include it. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "map_fsm.h"

static map_fsm_t fsm;
static int last_ret = -1;
static int failures = 0, checks = 0, line_no = 0;
static char line[512];

#define FAIL(...) do { failures++; printf("MISMATCH line %d: ", line_no); printf(__VA_ARGS__); printf("\n  %s\n", line); \
                       if (failures >= 10) { printf("too many failures\n"); exit(1); } } while (0)

static int cur_id(void)
{
    return fsm.graph.current == MAP_NONE ? -1 : fsm.graph.markers[fsm.graph.current].id;
}

static void check_expect(void)
{
    char state[32], res[16], route[200], hop[64];
    int n, count, cur, conf, tgt, hopi, resT, inc;
    int got = sscanf(line, "EXPECT %31s n=%d count=%d cur=%d conf=%d tgt=%d route=%199s hop=%d cur_hop=%63s res=%15s resT=%d inc=%d",
                     state, &n, &count, &cur, &conf, &tgt, route, &hopi, hop, res, &resT, &inc);
    if (got != 12) { FAIL("cannot parse EXPECT (%d fields)", got); return; }
    checks++;
    if (strcmp(state, mapfsm_state_name(fsm.state))) FAIL("state: python %s, C %s", state, mapfsm_state_name(fsm.state));
    if (n != fsm.n) FAIL("n: python %d, C %d", n, fsm.n);
    if (count != map_count(&fsm.graph)) FAIL("count: python %d, C %d", count, map_count(&fsm.graph));
    if (cur != cur_id()) FAIL("current: python %d, C %d", cur, cur_id());
    if (conf != (int)fsm.position_confirmed) FAIL("confirmed: python %d, C %d", conf, fsm.position_confirmed);
    if (tgt != fsm.target_id) FAIL("target: python %d, C %d", tgt, fsm.target_id);
    char mine[200] = "-";
    if (fsm.path_len) {
        mine[0] = 0;
        for (int i = 0; i < fsm.path_len; i++) { char t[8]; snprintf(t, sizeof t, i ? ",%d" : "%d", fsm.path[i]); strcat(mine, t); }
    }
    if (strcmp(route, mine)) FAIL("route: python %s, C %s", route, mine);
    if (hopi != fsm.hop) FAIL("hop index: python %d, C %d", hopi, fsm.hop);
    map_hop_t h;
    char mh[64] = "-";
    if (mapfsm_current_hop(&fsm, &h)) snprintf(mh, sizeof mh, "%d:%d:%d", h.heading, h.target_id, h.last);
    if (strcmp(hop, mh)) FAIL("current hop: python %s, C %s", hop, mh);
    const char *r = fsm.last_result == RESULT_REACHED ? "REACHED" : fsm.last_result == RESULT_NO_ROUTE ? "NO_ROUTE" : "NONE";
    if (strcmp(res, r)) FAIL("last result: python %s, C %s", res, r);
    if (resT != fsm.last_result_target) FAIL("last result target: python %d, C %d", resT, fsm.last_result_target);
    if (inc != fsm.graph.inconsistencies) FAIL("inconsistencies: python %d, C %d", inc, fsm.graph.inconsistencies);
}

static void check_plan(void)
{
    int a, b, hd, up;
    char want[200];
    if (sscanf(line, "PLAN %d %d %d %d %199s", &a, &b, &hd, &up, want) != 5) { FAIL("cannot parse PLAN"); return; }
    checks++;
    uint8_t path[PLAN_MAX_PATH];
    int n = plan_route(&fsm.graph, (uint8_t)a, (uint8_t)b, hd, up != 0, path, (int)PLAN_MAX_PATH);
    char mine[200] = "-";
    if (n > 0) {
        mine[0] = 0;
        for (int i = 0; i < n; i++) { char t[8]; snprintf(t, sizeof t, i ? ",%d" : "%d", path[i]); strcat(mine, t); }
    }
    if (strcmp(want, mine)) FAIL("path %d->%d heading %d probable %d: python %s, C %s", a, b, hd, up, want, mine);
}

static void check_headings(void)
{
    /* uses the path of the previous PLAN line: recompute and compare */
    static char prev_plan[512];
    (void)prev_plan;
}

static void run(const char *file)
{
    FILE *fp = fopen(file, "r");
    if (!fp) { perror(file); exit(2); }
    int plan_a = 0, plan_b = 0, plan_h = -1, plan_up = 0;
    while (fgets(line, sizeof line, fp)) {
        line_no++;
        line[strcspn(line, "\r\n")] = 0;
        int a, b, c, d, e;
        if (!strcmp(line, "FSM_INIT")) { mapfsm_init(&fsm); last_ret = -1; }
        else if (sscanf(line, "START %d %d", &a, &b) == 2) last_ret = mapfsm_start(&fsm, (uint8_t)a, (uint8_t)b);
        else if (sscanf(line, "REACHED %d %d", &a, &b) == 2) (void)mapfsm_on_marker_reached(&fsm, (uint8_t)a, (uint8_t)b);
        else if (sscanf(line, "EXITREP %d %d %d %d %d", &a, &b, &c, &d, &e) == 5) {
            uint8_t ex[4] = { (uint8_t)b, (uint8_t)c, (uint8_t)d, (uint8_t)e };
            (void)mapfsm_on_exit_report(&fsm, (uint8_t)a, ex);
        }
        else if (sscanf(line, "EXITEXP %d %d", &a, &b) == 2) (void)mapfsm_on_exit_explored(&fsm, (uint8_t)a, (uint8_t)b);
        else if (sscanf(line, "EXITBLK %d %d", &a, &b) == 2) (void)mapfsm_on_exit_blocked(&fsm, (uint8_t)a, (uint8_t)b);
        else if (!strcmp(line, "EXPLDONE")) (void)mapfsm_on_explore_done(&fsm);
        else if (!strcmp(line, "STOP")) mapfsm_stop(&fsm);
        else if (!strcmp(line, "FAULT")) mapfsm_fault(&fsm);
        else if (!strcmp(line, "RESUME")) last_ret = mapfsm_resume(&fsm);
        else if (!strcmp(line, "RESETFAULT")) last_ret = mapfsm_reset_fault(&fsm);
        else if (!strcmp(line, "RESETMAP")) last_ret = mapfsm_reset_map(&fsm);
        else if (!strcmp(line, "FINISH")) last_ret = mapfsm_finish_early(&fsm);
        else if (sscanf(line, "CONFIRM %d", &a) == 1) last_ret = mapfsm_confirm_marker(&fsm, (uint8_t)a);
        else if (sscanf(line, "RESUMESTART %d %d", &a, &b) == 2) last_ret = mapfsm_resume_start(&fsm, (uint8_t)a, (uint8_t)b);
        else if (sscanf(line, "GOTO %d", &a) == 1) last_ret = mapfsm_goto(&fsm, (uint8_t)a);
        else if (!strcmp(line, "HOPDONE") || !strcmp(line, "HOPDONE_STALE")) {
            map_hop_t h;
            if (mapfsm_current_hop(&fsm, &h))
                (void)mapfsm_hop_done(&fsm, (uint8_t)(h.seq + (line[7] ? 1 : 0)));
            else FAIL("HOPDONE with no outstanding hop in C");
        }
        else if (sscanf(line, "HOPBLOCKED %d %d", &a, &b) == 2) {
            map_hop_t h;
            if (mapfsm_current_hop(&fsm, &h)) (void)mapfsm_hop_blocked(&fsm, h.seq, (uint8_t)a, (uint8_t)b);
            else FAIL("HOPBLOCKED with no outstanding hop in C");
        }
        else if (sscanf(line, "RET %d", &a) == 1) { checks++; if (a != last_ret) FAIL("return value: python %d, C %d", a, last_ret); }
        else if (!strncmp(line, "EXPECT ", 7)) check_expect();
        else if (!strncmp(line, "PLAN ", 5)) {
            check_plan();
            sscanf(line, "PLAN %d %d %d %d", &plan_a, &plan_b, &plan_h, &plan_up);
        }
        else if (!strncmp(line, "HEADINGS ", 9)) {
            uint8_t path[PLAN_MAX_PATH], hd[PLAN_MAX_PATH], tg[PLAN_MAX_PATH];
            int n = plan_route(&fsm.graph, (uint8_t)plan_a, (uint8_t)plan_b, plan_h, plan_up != 0, path, (int)PLAN_MAX_PATH);
            int hops = n > 1 ? plan_path_headings(&fsm.graph, path, n, hd, tg) : -1;
            char mine[400] = "";
            for (int i = 0; i < hops; i++) { char t[16]; snprintf(t, sizeof t, i ? ",%d:%d" : "%d:%d", hd[i], tg[i]); strcat(mine, t); }
            checks++;
            if (strcmp(line + 9, mine)) FAIL("hop headings: python %s, C %s", line + 9, mine);
        }
        else if (!strcmp(line, "END_SCENARIO")) { /* nothing */ }
        else FAIL("unknown line");
    }
    fclose(fp);
    (void)check_headings;
}

/* ---- a few hand-written checks (worked example) ---- */
static void hand_checks(void)
{
    /* Straight corridor 12 - 18 - 14 - 15 - 9 - 2 going north, like the real 6-marker run. */
    map_fsm_t f;
    mapfsm_init(&f);
    int ok = mapfsm_start(&f, 6, 12);
    (void)mapfsm_on_marker_reached(&f, 18, DIR_N);
    (void)mapfsm_on_marker_reached(&f, 14, DIR_N);
    (void)mapfsm_on_marker_reached(&f, 15, DIR_N);
    (void)mapfsm_on_marker_reached(&f, 9, DIR_N);
    (void)mapfsm_on_marker_reached(&f, 2, DIR_N);
    checks++; if (!ok || f.state != MS_MAP_COMPLETE || map_count(&f.graph) != 6) { failures++; printf("hand: map not complete\n"); }
    checks++; if (!mapfsm_goto(&f, 15)) { failures++; printf("hand: goto refused\n"); }
    map_hop_t h;
    checks++; if (!mapfsm_current_hop(&f, &h) || h.heading != DIR_S || h.target_id != 9 || h.last) { failures++; printf("hand: first hop wrong\n"); }
    checks++; if (!mapfsm_hop_done(&f, h.seq)) { failures++; printf("hand: hop_done refused\n"); }
    checks++; if (!mapfsm_current_hop(&f, &h) || h.target_id != 15 || !h.last) { failures++; printf("hand: second hop wrong\n"); }
    checks++; if (mapfsm_hop_done(&f, (uint8_t)(h.seq + 1))) { failures++; printf("hand: stale hop_done accepted\n"); }
    (void)mapfsm_hop_done(&f, h.seq);
    checks++; if (f.state != MS_MAP_COMPLETE || f.last_result != RESULT_REACHED) { failures++; printf("hand: not reached\n"); }
    checks++; if (map_count(&f.graph) != 6 || f.graph.markers[f.graph.current].id != 15) { failures++; printf("hand: wrong marker\n"); }
}

int main(int argc, char **argv)
{
    if (argc < 2) { printf("usage: %s vectors.txt\n", argv[0]); return 2; }
    run(argv[1]);
    hand_checks();
    printf("%d checks, %d mismatches\n", checks, failures);
    return failures ? 1 : 0;
}
