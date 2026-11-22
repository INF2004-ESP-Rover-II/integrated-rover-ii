# GOTO on the RoboPico (Subsystem 4, on top of Buddy 1's map)

My earlier mapping port (own graph + FSM) is **not** used here. The map is Buddy 1's `rover_map_t`
(read only). Two new modules:

| File | What it does |
|---|---|
| `src/route_planner.c`, `include/route_planner.h` | A* over the verified graph. Edge usable only if both ends are `MAP_EXPLORED` and the neighbour exists. Cost = 1000 per hop + 1 per 90 degree turn; ties go to the smallest marker-ID path. No malloc, works on node slots (IDs up to 255). |
| `src/goto_ctrl.c`, `include/goto_ctrl.h` | `GOTO:<id>`: plan, send ONE hop with `onboard_dfs_begin_hop`, wait for `onboard_dfs_take_result`, re-plan from the confirmed current node, repeat. Never drives motors, never edits the map. |

`src/main.c` only gets a small hook (adapter functions, 4 commands, one `goto_tick` after `onboard_dfs_tick`).

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
make route goto        # or: make check
```
* `test_route_planner`: B1's fixture cases (7->36, wall changes, 40 open, 255 disconnected, 99 unknown, 7->7) plus
  about 125,000 checks against a brute force reference on random maps (hops, then turns, then smallest ID path), and a
  64-node line. Three deliberately broken versions of the planner were caught.
* `test_goto_ctrl`: mock executor with B1's contract: refused start, failure queued with a refused start, stale
  sequence, cancel with a late result, wall appearing mid-route, arrival at the wrong node, second route after the first.

NOT checked: the Pico build (no SDK here), and anything on the real rover.

## Test on the rover, in this order

1. Build and flash the RoboPico as usual. Nothing changes until you send a `GOTO` command.
2. Do your normal mapping (set geometry, set start, `DFS_START`) until `phase=COMPLETE`.
3. `mosquitto_sub -h localhost -t robot/status -v` in one terminal.
4. `mosquitto_pub -h localhost -t robot/command -m "GOTO_STATUS"` -> `GOTO IDLE last=NONE target=0`.
5. `GOTO:99` (an ID not on the map) -> `last=UNKNOWN_TARGET`. Nothing moves.
6. `GOTO:<id of the node you are on>` -> `last=REACHED`. Nothing moves.
7. **One hop first:** `GOTO:<id of a direct neighbour with a driven (explored) edge>`. Expect `RUNNING ... hop=0/1`,
   then `IDLE last=REACHED`. Keep a hand near the rover / on a stand for the first try.
8. A longer route, then `GOTO_BLOCK:<id>:<dir>` on the shortest route and send it again to see the detour.
9. During a route send `STOP` -> `last=CANCELLED` and the rover stops.

If a hop fails, send me the `GOTO IDLE last=HOP_FAILED ... reason=...` line and the `[RUN]` lines around it.
