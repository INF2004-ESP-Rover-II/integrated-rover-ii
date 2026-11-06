# Onboard DFS: implemented flow and bring-up

Production camera/map/DFS integration has been restored after the pull. The user
confirmed real S3-to-Pico UART reception. The normal and diagnostic firmware are
built separately. The assistant has not flashed a board or commanded motion.

## Next hardware test: stationary, with unmeasured camera geometry

1. Keep motor power OFF; close the old LAPTOP DFS dashboard and laptop camera
   publisher. Leave the broker running and preserve your working Wi-Fi setup.
2. Deliberately flash `build/firmware/robo_pico/mecanum_robot.uf2` this time. The
   normal target now contains the UART1 receiver and local navigation. Its banner
   includes `ONBOARD DFS / CAMERA UART1 GP21`; Sensor UART0 stays on GP13.
3. Open `dashboard/onboard_dashboard.py --broker <your-broker-IP>`, or run
   `start_rover_mapping.ps1` (it reads the current firmware broker IP). Click
   Connect. The viewer subscribes to robot/map, robot/status and robot/telemetry;
   it never starts a laptop detector or planner. Do not run two controllers.
4. Wait for live S3 ID/range/bearing/quality/age and Sensor Pico heading/ToF.
   The camera row is raw diagnostics, not proof of a valid navigation fix.
5. Keep the rover still, centred over a known starting marker, with its forward
   direction defining map north. Enter that **underlying** ID and Set start node.
   Expect a single visited node at (0,0). A marker visible ahead is not the start.
6. With geometry unmeasured, leave its three fields blank. Start DFS stays
   disabled and direct DFS_START/Resume/B2 hop requests are refused by firmware.
   Raw camera data still updates, but guessed geometry cannot place other nodes.
7. Move a printed marker in front of the stationary camera; verify the ID,
   lens range in mm, signed bearing and quality. Remove it -> status 1/no marker.
   Unplug only the webcam -> status 2/3. No motors should run in this test.

Before a moving test, measure lens height above the floor, forward lens offset
from the chassis reference/turn centre, and downward optical-axis tilt. Reset
map if a start is already anchored, enter these **measured** values, click Set
camera geometry, then re-anchor the start and select two visits for a first hop.
Geometry is stored in RAM per boot and preserved by Reset map. Editing geometry
with an anchored map is refused. Do not enter the simulator's mount values as if
they were hardware measurements. Spacing remains 600 mm unless configured.

Updated 6 November 2026. The production `mecanum_robot` target now uses S3 UART1
observations and Sensor Pico UART0 readings to execute DFS on the Robo Pico.
The default dashboard is a receiver/viewer with high-level mission buttons.
No board has been flashed or physically tested by this implementation task.

## Startup appears to stop at NAV: NETFIX-1

A corrected Wi-Fi readiness check now uses `cyw43_tcpip_link_status`, like the
Pico SDK's own connect routine. The previous low-level `cyw43_wifi_link_status`
reports JOIN (1) after association; comparing it against TCP/IP UP (3) could never
start MQTT. The NAV message is informational, not a wait for camera geometry.

Rebuild/flash the normal `mecanum_robot.uf2`; the first banner includes
`[NETFIX-1]`. It prints `[RUN] Main loop started`, the asynchronous connection
request, and state changes: 0 down, 1 joining, 2 waiting for an IP, 3 ready,
negative failure. It then attempts MQTT. A periodic `[RUN]` line continues even
without S3 frames or a broker and shows camera bytes/valid frames and nav phase.
Successful map queueing prints `[MAP] queued ... bytes on robot/map`.

A native regression compiles the production Wi-Fi/MQTT module with a fake SDK,
covering JOIN/NOIP/failure/UP, broker connection, robot/map publication and link
loss. It fails with the previous readiness expression and passes with the fix.
Run `make check_wifi` in `firmware/robo_pico/test` with a working host compiler.
This does not replace a real board connectivity test. Reconnection allows 30 s
for joining/DHCP. Source data is serviced independently of broker availability.

## Run the actual C controller in a virtual arena

From the repository root in PowerShell:

```powershell
.\tools\run_onboard_simulation.ps1
```

This builds and tests the portable C controller, runs an 8x8 (64-marker) virtual maze,
then opens the dashboard replay. `-NoGui` runs tests and generates
`build/onboard-simulation.jsonl` without opening a window. Dimensions are optional:

```powershell
.\tools\run_onboard_simulation.ps1 -Columns 2 -Rows 20
.\tools\run_onboard_simulation.ps1 -Columns 6 -Rows 6 -NoGui
```

The dimensions select the **test arena**, not a firmware map size. Actual firmware
learns its occupied bounds from marker coordinates. The test arena moves a
virtual chassis at 200 mm/s, supplies 50 Hz labelled sensors and ~5.6 fps encoded
camera packets, and hides floor tags below 500 mm lens range. These are simulated
conditions, not measured hardware rates. The production arrival tolerance is
100 mm; each committed simulated arrival checks the physical virtual position.

Tests also cover 1x1, 8x3, 3x8, 2x20, 8x8 and 64x1 arenas, negative coordinates,
64-node capacity/atomic rejection, IDs through 255, dense full-map serialization,
a branching dead end, revisits, a loop, arbitrary IDs, exhausted
search, missing camera/position, healthy NO_MARKER, duplicate-image confirmation,
STOP, and B2 hop replay/results. The replay is a viewer, not a Python controller.
It does not issue rover commands, including after replay finishes.

Validation in this checkout: the C virtual-arena, camera protocol and motor
lease/stop checks pass; seven receiver/command-model Python tests pass; the Tk
viewer is checked against the dynamic full-map trace and long/negative layouts.
The normal and receiver-only Pico targets build.
Broader legacy Python discovery has the same 17 failures and 63 errors out of
260 tests with both the original and modified dashboard entry point; those old
laptop-navigation failures are not claimed resolved by this migration.

To replay a saved trace separately:

```powershell
.\.venv\Scripts\python.exe dashboard/onboard_dashboard.py --replay build/onboard-simulation.jsonl
```

## Hardware build and operation

```powershell
cmake --build build --target mecanum_robot
.\.venv\Scripts\python.exe dashboard/robot_dashboard.py --broker 192.168.50.11
```

Use the actual broker IP configured in the existing local firmware `config.h`.
Alternatively use `start_rover_mapping.ps1 -Broker <laptop-IP>` for the existing
local broker workflow. The launcher no longer starts laptop camera detection.
The old laptop controller is available only with `--legacy-laptop` for offline
reference/explicit legacy use; do not run it as a second autonomous controller.

After independently verifying wiring, calibrated sensor/motor behaviour and the
local stop path on the actual chassis:

1. Flash `build/firmware/robo_pico/mecanum_robot.uf2` deliberately. The separate
   `camera_uart_diagnostic.uf2` is still receiver-only and does not execute DFS.
2. S3 GPIO17 -> series resistor -> Robo Pico GP21 RX, common ground, 3.3 V logic,
   115200 8N1. The S3 must match `vision_protocol.h`; GP21's button is reserved.
3. Enter measured camera geometry in the viewer first (see stationary test
   above); geometry starts unconfigured each boot. Keep the chassis stationary on the starting marker and calibrate the IMU at
   boot. Connect the dashboard, enter the actual underlying marker ID, and click
   **Set start node**. The current heading becomes map north and start cell (0,0).
4. Enter the number of distinct nodes to visit, normally **36**, and the exit
   clearance threshold, initially 500 mm. Wait for live camera and Sensor Pico
   data, then click **Start DFS**.
5. The rover scans exits, chooses an open edge, turns, settles, acquires a fresh
   reference, drives, brakes, confirms arrival and updates the local map/node.
   Repeated detections update measurements, not visit counts.
6. At a dead end, it returns along the DFS stack. Loop crossings return to their
   source before continuing the stack. A new node counts once; revisits remain
   real arrival events. At N visited nodes it stops in COMPLETE for B2.

## Dynamic map storage and limits

The RAM-only `rover_map_t` holds a bounded collection of coordinate-based nodes:

```c
rover_node_t nodes[ROVER_MAX_NODES]; // Current capacity: 64 discovered nodes
uint8_t node_count;
int16_t min_x, max_x, min_y, max_y;
```

Each node has `marker_id`, `x`, `y`, `visited` and four exits (0 unchecked,
1 open/untraversed, 2 explored, 3 blocked). The start is (0,0); north increases y
and east increases x. Marker IDs do not determine positions. A reliably located
marker adds one node and expands the occupied bounds in any direction. Seen
nodes can expand the bounds before being visited; only confirmed arrivals count
towards the mission target. Missing cells inside the bounds stay unknown.

- `rover_map_columns()` = max_x - min_x + 1; rows use y. An empty map is 0x0.
- No arena width/height is configured in production. A 2x20 or 8x8 layout works.
- Capacity is **64 discovered nodes**, including seen-but-unvisited nodes, not
  a rectangle of 64 cells. All storage is reserved at boot; no heap allocation
  or repeated resizing is required.
- IDs are 1..255; ID 0 remains reserved. The one-byte S3 wire format is unchanged.
  B3 must enable the desired IDs in the S3 detector (its previous 1..40 filter
  will not emit IDs above 40).
- Goal is 1..64 distinct visits; the dashboard still defaults to 36. Select 64
  for a 64-marker mission. This target does not prescribe dimensions.
- Coordinates are bounded to +/-63 relative to the start, sufficient for any
  connected 64-node grid. Every move remains one adjacent 600 mm hop.
- A new ID beyond capacity stops active navigation with `MAP_CAPACITY_REACHED`;
  no old nodes are overwritten. Duplicate-ID/different-position and occupied-cell
  conflicts stop with `MAP_CONFLICT`. Out-of-range coordinates are rejected.
- Change `ROVER_MAX_NODES` in `rover_map.h` to change the capacity and rebuild.
  Snapshot storage scales with it; a firmware static assertion requires the
  MQTT ring to be large enough. The current ring is 16384 bytes; the lwIP heap
  is 32768 bytes to retain space for TCP/network work beyond that ring. Revalidate RAM,
  packet sizing and tests for a different capacity. Count/ID fields limit this
  implementation to at most 255 nodes.

STOP retains this map; Reset map, Set start or a power cycle clears it. Bounds
are discovery extents, not proof of arena boundaries or complete exploration.

The node is committed only after braking/settling and three independent fresh
visual fixes, or three fresh readings of a previously anchored static wall.
The target tag need not remain visible: another known forward tag can locate the
rover over it. A static front-wall reference is anchored only while stationary
at a confirmed source, with measured wall range 750..1000 mm (beyond the next
600 mm cell). It cannot be invented from an arbitrary new obstacle mid-leg.
The initial wall model assumes the ToF origin is the rover reference origin;
mounting offsets must be calibrated before relying on its arrival accuracy.

## State, safety and recovery

- No usable position reference for 500 ms during travel stops translation.
- No fresh camera images stops motion and faults after about 5 s from the last
  image. Repeated sequences and health-only packets do not keep image data fresh.
- Healthy NO_MARKER can trigger a protected in-place search, bounded to 180
  degrees cumulative rotation and 15 s. A candidate reference must produce
  consistent fresh fixes while stopped. The overall segment deadline still wins.
- Turns, including ordinary DFS turns, require valid front/left/right ToF and
  valid rear ultrasonic readings all >=200 mm. This is a sampled clearance
  check, not proof of complete swept-area coverage; validate the actual footprint
  and sensor coverage. If rear data is unavailable, turning is refused.
- Sensor/heading receive age is provisionally limited to 600 ms to accommodate
  the existing Sensor Pico implementation. The legacy wire packet lacks explicit
  per-sample age; reception freshness does not certify acquisition freshness.
  Meeting original >=20 Hz UART / >=10 Hz ToF and <=200 ms physical halt criteria
  still requires Sensor Pico scheduling work and hardware measurements.
- A 300 ms executor lease is checked by the independent motor timer; a stalled
  main loop cannot leave the stored auto-drive refreshing forever.
- Segment timeout is 30 s; turn timeout 5 s; scan timeout 10 s. Invalid sensors,
  off-path/conflicting fixes, an emergency stop or failed recovery fault locally.
- A very-close obstacle during a leg stops and faults while retaining the map.
  It does not pretend to be back at the source or reverse blindly. Dead-end
  backtracking at a confirmed node is implemented; arbitrary mid-edge obstacle
  return/replanning is not treated as a confirmed-node backtrack.
- STOP retains the map/stack and pauses. Resume from a confirmed node rescans;
  between nodes it first requires fresh relocalisation. Reset fault never starts
  motion; use explicit Resume after health/pose checks. Emergency reset also
  needs the motor latch cleared. Reset map or Set start clears the old map.
- Dashboard/network loss does not stop an already accepted healthy onboard
  mission. STOP sent over a disconnected network cannot be guaranteed; retain a
  local stop/power-isolation method. Manual commands cannot move an anchored map
  behind the navigator: reset the map first for legacy manual motion.

The flat-floor camera model uses lens range, optical bearing, measured height,
forward offset and mounting pitch. Mounting fields start unconfigured/zero with
camera_calibrated=false. The Pico requires CAMERA_GEOMETRY before autonomous
movement. Only quality=0 can localise or place landmarks; quality=1..4/unknown
remain diagnostic-only. A processed frame containing failed poses still counts
as an image for camera health; it cannot refresh position or confirm arrival.
B3's age_ms excludes camera/USB delay; effective_age_ms adds wire and local elapsed
time but cannot reconstruct that delay. The 500 ms reported-age guard is retained.
The simulator's 325 mm height/120 mm offset/0 degree tilt are virtual fixtures. Slopes, camera roll and
uncalibrated chassis slip are not claimed solved by these virtual tests. Quality
is passed through without an invented confidence metric.

## Dashboard contract

Commands on `robot/command`, non-retained:

```text
CAMERA_GEOMETRY:<measured-height-mm>:<measured-forward-mm>:<measured-down-deg>
SET_START:7
DFS_START:36:500
STOP
ESTOP
RESET_SAFETY
DFS_RESET
DFS_RESUME
RESET_MAP
MAP_REQUEST
```

Every accepted change advances map/state revision. `robot/map` carries a complete
JSON snapshot including current node, physical-node confirmation flag, visited
count, seen/visited markers, coordinates, exit states, target, pose and reason.
Schema 1 also carries `capacity`, `id_max`, `pitch_mm`, and `bounds` with min/max
x/y, rows and columns. The dashboard validates bounds against received nodes and
scales its axes to them (with pan/zoom controls for long maps); it does not infer missing nodes or navigation decisions.
Snapshots also carry geometry_ready/geometry and raw camera measurements, including
reported/effective/local receive ages and quality. The viewer disables Start until
measured geometry is acknowledged and the map is READY. Firmware independently
checks the same requirement, including Resume and B2 hops.
Snapshots are coalesced to at most 5 Hz and repeated every 2 s; failed/blocked
network sends do not block control. A fresh full snapshot is requested on connect.
Cells seen ahead stay blue/unvisited until confirmed arrival; green means visited.
Closing the viewer does not send STOP or cancel DFS. High-level commands only:
the default viewer never constructs LaptopNavigator, DFS or a planner instance.

## B2 integration after DFS

External legacy MOVE_/TURN_ commands are refused in the normal application.
Manual chassis moves/heading zero are refused once a map is anchored. STOP/ESTOP
have priority over queued commands. A Sensor Pico timestamp restart requires
re-anchoring; replayed timestamps cannot keep sensor or IMU freshness alive.

No A* implementation has been added. B2 can include `onboard_dfs.h` and reuse the
authoritative map instead of constructing another map:

```c
const rover_map_t *map = onboard_dfs_map(&navigator);
// Iterate map->nodes[0..map->node_count-1] for IDs, coordinates and exits.
// Use rover_map_at(map,x,y) / rover_map_find(map,id,&x,&y) for lookups.
// Plan locally using MAP_EXPLORED edges; there is no cells[6][6] array.
// For each verified adjacent route edge, only when ready and input guards pass:
bool accepted = onboard_dfs_begin_hop(&navigator, target_id, heading, hop_sequence, now_ms);
// Continue normal sensor/camera/tick servicing; don't restart the same hop.
onboard_hop_result_t result;
if (onboard_dfs_take_result(&navigator, &result)) {
    // One terminal result. Advance/replan based on result, current confirmation.
}
```

Hop sequences must be nonzero and strictly increase within the boot/session.
Blocked/unchecked/open-but-untraversed edges are not accepted route shortcuts.
The API verifies an adjacent target, prevents repeated execution and uses the
same executor/arrival rules. A result must be consumed before another hop.

Completion here means **N confirmed visited IDs**, as requested. It does not
prove every possible connection was traversed or that an A* route is globally
shortest in an incompletely explored arena. The finer occupancy map from original
FR-24 remains a separate B2 deliverable; it is not implemented by this marker grid.
