/* map_cmd.h - text commands and JSON snapshot for the mapping / path-planning FSM (map_fsm.h).
 * Plain C11 + snprintf, no Pico SDK includes (so it also builds on a laptop for tests).
 *
 * Commands arrive on robot/command like every other command. They never move the motors:
 * they only feed the mapping FSM (while the navigator is not yet on the Pico, YOU play the navigator
 * by typing the events; later the navigator calls the mapfsm_* functions directly).
 *
 *   MAP_START:<n>,<start_id>        start mapping, want n markers, rover is at start_id
 *   MAP_REACHED:<id>,<heading>      rover arrived at marker id travelling heading (0=N 1=E 2=S 3=W)
 *   MAP_EXITS:<id>,<n>,<e>,<s>,<w>  exit status per direction (0 unchecked 1 open 2 explored 3 blocked)
 *   MAP_EXPLORED:<id>,<dir>         exit travelled
 *   MAP_BLOCKED:<id>,<dir>          exit blocked (phase 1)
 *   MAP_DONE                        nothing left to explore
 *   MAP_STOP / MAP_FAULT / MAP_RESUME / MAP_RESET_FAULT / MAP_RESET / MAP_FINISH
 *   MAP_CONFIRM:<id>                camera confirms the rover is at marker id
 *   MAP_RESUME_START:<n>,<id>       restart mapping at a known marker, keeping the map
 *   MAP_GOTO:<id>                   plan a route to marker id (A*) and issue the first hop
 *   MAP_HOP_DONE                    the outstanding hop finished at its target marker
 *   MAP_HOP_BLOCKED:<id>,<dir>      the outstanding hop is blocked at marker id, direction dir; re-plan
 *   MAP_STATUS                      one-line summary
 *   MAP_DUMP                        publish the full JSON snapshot on robot/map now
 *   MAP_SELFTEST                    run the built-in checks and an A* timing run on the board
 *
 * Reply text goes to robot/status (starts with "MAP_") and to the USB serial log. */
#ifndef MAP_CMD_H
#define MAP_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "map_fsm.h"

map_fsm_t *map_cmd_fsm(void);                       /* the one FSM instance (also used by the navigator) */
void map_cmd_init(void);                            /* call once at start-up */
void map_cmd_set_clock_us(uint32_t (*clock_us)(void));   /* optional: lets MAP_SELFTEST time the planner */

/* Returns true if `command` was a MAP_ command (reply is then filled), false otherwise. */
bool map_cmd_handle(const char *command, char *reply, size_t reply_size);

/* Full snapshot as JSON (same keys as rover_mapping/mapping/snapshot.py). Returns length, 0 if it did not fit. */
size_t map_snapshot_json(const map_fsm_t *f, char *out, size_t out_size);

/* Call from the main loop with a clock in ms. Fills `out` and returns true when the map changed and at least
 * min_interval_ms has passed since the last snapshot (or when MAP_DUMP asked for one). */
bool map_cmd_snapshot_due(uint32_t now_ms, uint32_t min_interval_ms, char *out, size_t out_size);

/* Call if publishing the snapshot failed (not connected / busy): it will be offered again after the interval. */
void map_cmd_snapshot_retry(void);

/* Built-in checks; clock_us may be NULL (no timing). Returns true on PASS; text goes into reply. */
bool map_selftest(char *reply, size_t reply_size, uint32_t (*clock_us)(void));

#endif
