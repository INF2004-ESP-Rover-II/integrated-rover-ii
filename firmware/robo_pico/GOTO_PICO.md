# GOTO on the RoboPico (Subsystem 4, on top of Buddy 1's map)

My earlier mapping port (own graph + FSM) is **not** used here. The map is Buddy 1's `rover_map_t`
(read only). Three modules (all in BARR-C:2018 style, see the end of this file):

| File | What it does |
|---|---|
| `src/route_planner.c`, `include/route_planner.h` | Shortest route over the verified graph (cost-to-go relaxation over node and heading states, exact, no heuristic). Edge usable only if both ends are `MAP_EXPLORED` and the neighbour exists. Cost = 1000 per hop + 1 per 90 degree turn; ties go to the smallest marker-ID path. No malloc, works on node slots (IDs up to 255). `route_planner_plan()` gives the whole route, `route_planner_next_hop()` gives just the first hop (for the DFS). |
| `src/goto_ctrl.c`, `include/goto_ctrl.h` | `GOTO:<id>`: plan, send ONE hop with `onboard_dfs_begin_hop`, wait for `onboard_dfs_take_result`, re-plan from the confirmed current node, repeat. Never drives motors, never edits the map. |

| `src/goto_dry.c`, `include/goto_dry.h` | `DEMO_MAP` and `GOTO_DRY:<id>`: plan-only test helpers, no motors, real map never changed. |

`src/main.c` only gets a small hook (adapter functions, the GOTO commands, one `goto_ctrl_tick` after `onboard_dfs_tick`).

Renamed in the BARR-C conversion (module prefix rule): `route_plan` -> `route_planner_plan`, `route_t` ->
`route_planner_route_t`, `goto_start/cancel/tick/status/take_event/init` -> `goto_ctrl_...`, `goto_io_t` ->
`goto_ctrl_io_t`, `goto_hop_result_t` -> `goto_ctrl_hop_result_t`, `GOTO_IDLE` -> `GOTO_CTRL_IDLE`,
`demo_map_build` -> `goto_dry_build_demo_map`. The commands on `robot/command` did not change.

## Commands (send on `robot/command`; replies on `robot/status`, also printed as `[GOTO] ...`)

| Command | Meaning |
|---|---|
| `GOTO:<id>` | go to marker `<id>`. Refused if bad ID, a route is running, safety latched or reference invalid |
| `GOTO_CANCEL` | stop the route and the rover (same as STOP for a route) |
| `GOTO_STATUS` | one status line |
| `GOTO_BLOCK:<id>:<dir>` | operator override: mark the edge leaving `<id>` in `<dir>` (0=N 1=E 2=S 3=W) blocked, both ends. Only when idle. For testing re-planning |

STOP / ESTOP also cancel a route. Status text looks like
`GOTO RUNNING target=36 hop=1/3 next=9 last=NONE` or `GOTO IDLE last=REACHED target=36`.
Results: `REACHED, NO_ROUTE, UNKNOWN_TARGET, NOT_READY, HOP_FAILED (with reason), CANCELLED`.

## No camera? Test planning with a fake map (plan-only, nothing can move)

The camera link currently decodes no frames, so DFS cannot run. These two commands let you test the planner and the
GOTO controller on the Pico anyway:

| Command | What it does |
|---|---|
| `DEMO_MAP` | loads a fake 3 x 2 arena (markers 1..6, a ring plus a middle link, all driven) into the map. Refused if a real map exists or DFS is busy. `RESET_MAP` clears it |
| `GOTO_DRY:<id>` | plans from the map's current node, then walks the route on a COPY of the map with a simulated executor through the real GOTO controller. Reply, e.g. `GOTO_DRY 1>2>3>4 hops=3 plan_us=420 result=REACHED` |

Try: `DEMO_MAP`, `GOTO_DRY:4` (expect `1>2>3>4`), `GOTO_BLOCK:1:1` (block 1's east edge), `GOTO_DRY:4` (expect `1>6>5>4`),
`GOTO_BLOCK:1:0`, `GOTO_DRY:4` (expect `none ... NO_ROUTE`), then `RESET_MAP`. `plan_us` is the real planning time on
the Pico. The dashboard map view also shows the fake map, because it is in B1's map.

This does NOT test driving. A real `GOTO` still needs B1's camera, sensor and position checks and will not move
without them. The fake map only exists for planning/visual tests: do `RESET_MAP` before real mapping.

## Rules it follows (from B1's handover)

* Only after DFS is complete and the current node is confirmed (`phase COMPLETE`, `current_confirmed`).
* One hop at a time, strictly increasing sequence numbers, result checked even when `begin_hop` returns false.
* Arrival is trusted only if `map.current` equals the hop target.
* **A failed hop stops the route.** The executor is in FAULT and the rover may be between nodes, so nothing is
  re-planned from a guessed position. Recover with `DFS_RESET`, then `DFS_RESUME`, then send `GOTO:<id>` again.
* If the executor refuses a start (stale camera or sensor) it retries for 3 s, then reports `NOT_READY`.

## Checked on the laptop (no Pico needed)

```sh
cd firmware/robo_pico/test
make route goto dry dfs   # or: make check (also runs the other tests)
make barrc                # mechanical BARR-C checks on the Subsystem 4 files
```
* `test_route_planner`: B1's fixture cases (7->36, wall changes, 40 open, 255 disconnected, 99 unknown, 7->7) plus
  about 125,000 checks against a brute force reference on random maps (hops, then turns, then smallest ID path), and a
  64-node line. Three deliberately broken versions of the planner were caught.
* `test_goto_ctrl`: mock executor with B1's contract: refused start, failure queued with a refused start, stale
  sequence, cancel with a late result, wall appearing mid-route, arrival at the wrong node, second route after the first.

* `test_goto_dry`: DEMO_MAP and GOTO_DRY text, real map untouched, blocked edge detour, no route.
* `test_dfs_backtrack`: see the next section.

Ten deliberately broken versions of the planner and controller (open edges routable, one-sided edge accepted, no turn
cost, wrong tie-break, wrong first-hop direction, arrival not checked, second GOTO accepted, failed hop ignored, late
results kept, no admission timeout) were each caught by at least one test.

NOT checked: the Pico build (no SDK here), and anything on the real rover.

## Test on the rover, in this order

1. Build and flash the RoboPico as usual. Nothing changes until you send a `GOTO` command.
2. Do your normal mapping (set geometry, set start, `DFS_START`) until `phase=COMPLETE`.
3. `mosquitto_sub -h <broker-ip> -t robot/status -v` in one terminal.
4. `mosquitto_pub -h <broker-ip> -t robot/command -m "GOTO_STATUS"` -> `GOTO IDLE last=NONE target=0`.
5. `GOTO:99` (an ID not on the map) -> `last=UNKNOWN_TARGET`. Nothing moves.
6. `GOTO:<id of the node you are on>` -> `last=REACHED`. Nothing moves.
7. **One hop first:** `GOTO:<id of a direct neighbour with a driven (explored) edge>`. Expect `RUNNING ... hop=0/1`,
   then `IDLE last=REACHED`. Keep a hand near the rover / on a stand for the first try.
8. A longer route, then `GOTO_BLOCK:<id>:<dir>` on the shortest route and send it again to see the detour.
9. During a route send `STOP` -> `last=CANCELLED` and the rover stops.

If a hop fails, send me the `GOTO IDLE last=HOP_FAILED ... reason=...` line and the `[RUN]` lines around it.

## For the DFS: backtracking with the planner (Buddy 1)

`route_planner_next_hop()` is a pure read of `rover_map_t`: it never moves the rover and never writes the map. Today
`onboard_dfs.c` goes back one node with `stack[depth - 2]` and `direction_to()`. To go back to ANY earlier node
(for example the most recent stack node that still has an open exit) over the shortest verified path:

```c
route_planner_step_t step;
route_planner_status_t status;

status = route_planner_next_hop(&n->map, n->map.current, target_id,
                                (int32_t)n->direction, &step);
if (ROUTE_PLANNER_OK == status)
{
    /* Drive step.direction to marker step.next_id with the DFS's own leg
     * function. Call again after each CONFIRMED arrival, because the map may
     * have changed. */
}
```

Rules to keep in mind:
* Only EXPLORED edges (driven, both ends) are used. A node seen but never driven to gives `ROUTE_PLANNER_NO_ROUTE`:
  fall back to your own handling (`BACKTRACK_BLOCKED` etc.).
* `ROUTE_PLANNER_ALREADY_THERE` means no move. Unknown start or target gives a status, never a move.
* The start must be the confirmed current node (`map.current`). Do not plan from a guessed position.
* If the DFS takes a shortcut (not the stack path), cut its own stack back to the target node afterwards, as the test
  model does. Going back ONE node is always the same single hop as the stack gives.
* This is separate from `GOTO`: `onboard_dfs_begin_hop` only accepts a hop when the phase is COMPLETE, so GOTO cannot
  be used while exploring. The DFS calls the planner directly.

`test/test_dfs_backtrack.c` is the worked example. It reads the three 6 x 6 DFS test maps from the drawing (20, 10 and
15 inside wall segments, 36 reachable, dead ends B2/C6/D5, none, E4/E5; all checked from the wall lists), explores each with a host model
of the DFS (FR-19 exit order), and at every return compares the stack path with the planner path. Result:

| Map | Returns | Shorter by planner | Back-hops stack / planner | Total hops stack run / planner run |
|---|---|---|---|---|
| current layout | 4 | 1 | 8 / 6 | 53 / 51 |
| S layout | 11 | 1 | 20 / 16 | 85 / 81 |
| loop and dead ends | 9 | 0 | 20 / 20 | 75 / 75 |

The planner path was never longer than the stack path, and every cell was explored in both runs. The totals are what
this model produced, not a guarantee for another exit order. The model is NOT `onboard_dfs.c`: it scans all four exits on
arrival and counts a loop-closing exit as driven there and back.

## Coding standard (BARR-C:2018)

The three modules and their tests follow BARR-C:2018: C99, Allman braces always, 80 columns, no tabs, 4-space indent,
module-prefixed public names, `uint32_t` etc. instead of `int`/`unsigned`, `p_` for pointers, `g_` for file-scope
variables, names of 3 or more characters, no `continue` or `goto`, constants on the left of `==`, `default` in every
`switch`, `else` after every `else if`, a comment on every cast, Doxygen-style comments on every function, an
end-of-file comment. Built with `-std=c99 -pedantic -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Wcast-qual
-Wstrict-prototypes` (gcc and clang, no warnings). `make barrc` runs `tools/barrc_check.py`, a partial mechanical
check (it can find rule breaks, it cannot certify compliance).

Known deviations: plain `int` where an API forces it (`rover_map_at/place` take `int`, `snprintf` returns `int`,
`main` returns `int`); the other team's files (`main.c` outside the GOTO glue, `rover_map.c`, `onboard_dfs.c`, motor and camera code,
`fixtures/`) are NOT converted.
