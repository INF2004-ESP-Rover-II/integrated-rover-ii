#ifndef DFS_TEST_MODE_H
#define DFS_TEST_MODE_H

#include "dfs_explorer.h"

// Manual on-board decision test only. The firmware adapter MUST lock motors
// before handling test commands and reject movement/reset while active.
typedef void (*dfs_test_output_t)(const char *message);
typedef struct {
    dfs_explorer_t explorer;
    bool active;
    uint32_t last_session;
} dfs_test_mode_t;

bool dfs_test_is_command(const char *command);
void dfs_test_command(dfs_test_mode_t *test, const char *command,
                       uint32_t now_ms, dfs_test_output_t output);
void dfs_test_tick(dfs_test_mode_t *test, uint32_t now_ms, dfs_test_output_t output);

#endif
