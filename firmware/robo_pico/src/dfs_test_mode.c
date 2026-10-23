#include "dfs_test_mode.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DFS_TEST_COMMAND_SIZE 128u
#define DFS_TEST_MANUAL_TIMEOUT_MS 180000u

bool dfs_test_is_command(const char *command) {
    return command != NULL && strncmp(command, "DFS_TEST_", 9) == 0;
}

static bool number(const char *text, uint32_t *value) {
    if (text == NULL || text[0] < '0' || text[0] > '9') return false;
    int base = text[0] == '0' && (text[1] == 'x' || text[1] == 'X') ? 16 : 10;
    char *end;
    errno = 0;
    unsigned long parsed = strtoul(text, &end, base);
    if (errno == ERANGE || end == text || *end != '\0' || parsed > UINT32_MAX) return false;
    *value = (uint32_t)parsed;
    return true;
}

static bool heading(const char *text, dfs_direction_t *value) {
    if (text == NULL || strlen(text) != 1) return false;
    static const char names[] = "NESW";
    const char *match = strchr(names, text[0]);
    if (match == NULL) return false;
    *value = (dfs_direction_t)(match - names);
    return true;
}

static bool marker_type(const char *text, dfs_marker_type_t *value) {
    if (text == NULL || strlen(text) != 1 || (text[0] != 'J' && text[0] != 'P')) return false;
    *value = text[0] == 'J' ? DFS_JUNCTION : DFS_PLAIN;
    return true;
}

static const char *move_name(dfs_move_kind_t kind) {
    switch (kind) {
        case DFS_EXPLORE: return "EXPLORE";
        case DFS_BACKTRACK: return "BACKTRACK";
        case DFS_LOOP_RETURN: return "LOOP_RETURN";
        case DFS_BLOCKED_RETURN: return "BLOCKED_RETURN";
        default: return "NONE";
    }
}

static void summary(const dfs_test_mode_t *test, dfs_test_output_t output) {
    static const char *states[] = {"IDLE", "READY", "MOVING", "PAUSED", "COMPLETE", "EXHAUSTED", "FAULT"};
    char message[224];
    const dfs_explorer_t *dfs = &test->explorer;
    snprintf(message, sizeof(message),
        "DFS_TEST %s active=%u session=%lu marker=%lu confirmed=%u count=%u/%u depth=%u pending=%lu fault=%u DRY_RUN",
        states[dfs->state], (unsigned)test->active, (unsigned long)dfs->config.session_id,
        (unsigned long)(dfs->node_count ? dfs->nodes[dfs->current].marker_id : 0),
        (unsigned)dfs->current_confirmed, (unsigned)dfs->node_count,
        (unsigned)dfs->config.target_markers, (unsigned)dfs->depth,
        (unsigned long)dfs->pending.segment_id, (unsigned)dfs->fault);
    output(message);
}

void dfs_test_tick(dfs_test_mode_t *test, uint32_t now_ms, dfs_test_output_t output) {
    if (test == NULL || output == NULL || !test->active) return;
    dfs_state_t before = test->explorer.state;
    dfs_tick(&test->explorer, now_ms);
    if (before != test->explorer.state) summary(test, output);
}

void dfs_test_command(dfs_test_mode_t *test, const char *command,
                       uint32_t now_ms, dfs_test_output_t output) {
    if (test == NULL || output == NULL || !dfs_test_is_command(command)) return;
    size_t length = strlen(command);
    if (length >= DFS_TEST_COMMAND_SIZE) {
        output("DFS_TEST ERROR command too long");
        return;
    }
    char buffer[DFS_TEST_COMMAND_SIZE];
    memcpy(buffer, command, length + 1u);
    char *args[7] = {buffer};
    unsigned count = 1;
    for (char *cursor = buffer; *cursor != '\0'; ++cursor) {
        if (*cursor == ':') {
            if (count == 7 || cursor == buffer || cursor[1] == '\0' || cursor[1] == ':') {
                output("DFS_TEST ERROR invalid arguments");
                return;
            }
            *cursor = '\0';
            args[count++] = cursor + 1;
        }
    }
    uint32_t a = 0, b = 0, c = 0;
    dfs_direction_t direction;
    dfs_marker_type_t type;
    if (strcmp(args[0], "DFS_TEST_START") == 0 && count == 6 &&
        number(args[1], &a) && number(args[2], &b) && number(args[3], &c) &&
        marker_type(args[4], &type) && heading(args[5], &direction) &&
        b > 0 && b <= DFS_MAX_MARKERS && a > test->last_session) {
        dfs_config_t config = dfs_default_config(a, (uint16_t)b);
        config.segment_timeout_ms = DFS_TEST_MANUAL_TIMEOUT_MS;
        if (dfs_start(&test->explorer, &config, c, type, direction)) {
            test->active = true;
            test->last_session = a;
            summary(test, output);
            return;
        }
    } else if (strcmp(args[0], "DFS_TEST_HELP") == 0 && count == 1) {
        output("DFS_TEST START:session:N:marker:J/P:N/E/S/W; SCAN:open:blocked; NEXT; ARRIVE:session:segment:marker:J/P:heading; BLOCKED:session:segment; STATUS; PAUSE; RESUME:marker:heading; EXIT (all commands prefixed DFS_TEST_)");
        return;
    } else if (strcmp(args[0], "DFS_TEST_EXIT") == 0 && count == 1) {
        if (test->active) dfs_pause(&test->explorer);
        test->active = false;
        output("DFS_TEST EXITED motors remain locked; explicit RESET_SAFETY required");
        return;
    } else if (strcmp(args[0], "DFS_TEST_STATUS") == 0 && count == 1) {
        summary(test, output);
        return;
    } else if (test->active) {
        dfs_test_tick(test, now_ms, output);
        bool accepted = false;
        if (strcmp(args[0], "DFS_TEST_SCAN") == 0 && count == 3 &&
            number(args[1], &a) && number(args[2], &b) && a <= DFS_ALL_EXITS && b <= DFS_ALL_EXITS) {
            accepted = dfs_observe_exits(&test->explorer,
                test->explorer.nodes[test->explorer.current].marker_id, (uint8_t)a, (uint8_t)b);
        } else if (strcmp(args[0], "DFS_TEST_NEXT") == 0 && count == 1) {
            dfs_action_t action = dfs_next_action(&test->explorer, now_ms);
            if (action.kind == DFS_ACTION_MOVE) {
                char message[224];
                snprintf(message, sizeof(message),
                    "DFS_TEST MOVE session=%lu segment=%lu from=%lu from_confirmed=%u target_known=%u target=%lu dir=%c kind=%s DRY_RUN",
                    (unsigned long)action.segment.session_id, (unsigned long)action.segment.segment_id,
                    (unsigned long)action.segment.from_marker, (unsigned)action.segment.from_confirmed,
                    (unsigned)action.segment.target_known, (unsigned long)action.segment.target_marker,
                    "NESW"[action.segment.direction], move_name(action.segment.kind));
                output(message);
            } else if (action.kind == DFS_ACTION_SCAN) {
                output("DFS_TEST SCAN_REQUIRED supply confirmed map-frame exits with DFS_TEST_SCAN");
            } else {
                summary(test, output);
            }
            return;
        } else if (strcmp(args[0], "DFS_TEST_ARRIVE") == 0 && count == 6 &&
            number(args[1], &a) && number(args[2], &b) && number(args[3], &c) &&
            marker_type(args[4], &type) && heading(args[5], &direction)) {
            accepted = dfs_arrive(&test->explorer, a, b, c, type, direction, now_ms);
        } else if (strcmp(args[0], "DFS_TEST_BLOCKED") == 0 && count == 3 &&
            number(args[1], &a) && number(args[2], &b)) {
            accepted = dfs_segment_blocked(&test->explorer, a, b, now_ms);
        } else if (strcmp(args[0], "DFS_TEST_PAUSE") == 0 && count == 1) {
            accepted = dfs_pause(&test->explorer);
        } else if (strcmp(args[0], "DFS_TEST_RESUME") == 0 && count == 3 &&
            number(args[1], &a) && heading(args[2], &direction)) {
            accepted = dfs_resume(&test->explorer, a, direction);
        }
        if (accepted) {
            summary(test, output);
            return;
        }
    }
    output("DFS_TEST REJECTED check syntax, session/segment IDs and state; send DFS_TEST_HELP");
}
