"""Join forward-camera observations and MQTT sensor snapshots on the laptop.

The live dashboard configures GridLocalizer: a user-supplied start node, two
consecutive markers ahead, cross-junction side readings, and terminal wall stops.
The stand-off adapter remains available for existing simulations/clients which
do not configure a grid anchor.

RoboPico receives TURN_TO:<heading>/FORWARD/STOP commands. It closes the turn
loop locally; the laptop waits for acknowledgement and HEADING_REACHED.
Feed B3's JSON into vision_received(), or supply lens distance and bearing through
marker_seen(). Detection alone is not arrival. All methods have one thread owner.
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

from dfs_explorer import DFSExplorer, Direction, MarkerType, State
from vision_feed import VisionFeed, finite_number
from vision_feed import MarkerObservation
from grid_localization import GridLocalizer, VECTORS


def heading_error(target, actual):
    return (target - actual + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class SensorSnapshot:
    heading: float
    distances: dict
    faults: int
    stop_required: bool
    received_at: float
    age_s: float

    @classmethod
    def decode(cls, data, now):
        """Decode RoboPico's robot/telemetry relay of the UART sensor packet."""
        imu, obstacle = data["imu"], data["obstacle"]
        heading, age = imu["heading"], obstacle["age_ms"]
        if (imu.get("status") != 0 or isinstance(heading, bool) or
                not isinstance(heading, (int, float)) or not math.isfinite(heading) or
                isinstance(age, bool) or not isinstance(age, (int, float)) or
                not math.isfinite(age) or age < 0):
            raise ValueError("Valid heading and sensor age are required")
        distances = obstacle["distance_mm"]
        if not isinstance(distances, dict):
            raise ValueError("distance_mm must be an object")
        stop, faults = obstacle["stop_required"], obstacle["fault_mask"]
        if type(stop) is not bool or type(faults) is not int or not 0 <= faults <= 7:
            raise ValueError("Invalid sensor stop/fault flags")
        return cls(heading % 360, dict(distances), faults, stop, now, age / 1000)

    def exits(self, heading, threshold_mm, front_threshold_mm=None):
        """Rotate L/F/R sensor directions into the fixed map frame.

        1023 is confirmed beyond the 1 m reporting range and establishes an open exit.
        0/unavailable, faulted sensors and legacy ambiguous 65535 stay unknown.
        Rear is unknown until a traversal establishes the entry connection.
        """
        opened = blocked = 0
        for sensor_bit, (name, offset) in enumerate((("left", 3), ("front", 0), ("right", 1))):
            distance = self.distances.get(name)
            if self.faults & (1 << sensor_bit) or type(distance) is not int:
                continue
            bit = 1 << ((heading + offset) % 4)
            if distance == 1023:
                opened |= bit
            elif not 1 <= distance <= 1000:
                continue
            elif name == "front" and self.stop_required and front_threshold_mm is None:
                blocked |= bit
            elif distance >= (front_threshold_mm if name == 'front' and
                              front_threshold_mm is not None else threshold_mm):
                opened |= bit
            else:
                blocked |= bit
        return opened, blocked


class LaptopNavigator:
    SENSOR_MAX_AGE_S = 0.75
    CAMERA_MAX_AGE_S = 1.0
    COMMAND_REFRESH_S = 0.2
    TURN_TIMEOUT_S = 15.0
    POST_TURN_INPUT_WAIT_S = 3.0
    INPUT_RECOVERY_WAIT_S = 5.0
    TURN_TOLERANCE_DEG = 5.0  # Pico reaches 3 deg; allow a small post-braking margin.
    SCAN_TOLERANCE_DEG = 20.0
    STOP_SETTLE_S = 0.4  # Exceeds RoboPico's current 300 ms active brake hold.
    ARRIVAL_DISTANCE_MM = 500.0  # Camera lens to marker centre.
    ARRIVAL_TOLERANCE_MM = 30.0  # Confirmation band after stopping; tune on hardware.
    ARRIVAL_BEARING_DEG = 30.0  # shortcut: loosened from 5.0 for mapping tests, restore before real runs.
    ARRIVAL_CONFIRM_FRAMES = 5
    ALIGN_TRIGGER_DEG = 5.0
    ALIGN_TARGET_DEG = 5.0
    ALIGN_CONFIRM_FRAMES = 3
    ALIGN_PULSE_S = .4  # One default Q/E zigzag cycle (two 200 ms halves).
    ALIGN_SIDE_CLEARANCE_MM = 150

    def __init__(self, send_command, report=lambda message: None, clock=time.monotonic,
                 wall_clock=time.time):
        self.send_command = send_command  # Must return True on successful publish.
        self.report = report
        self.clock = clock
        self.wall_clock = wall_clock
        self.vision_feed = VisionFeed()
        self.explorer = DFSExplorer()
        self.sensor: Optional[SensorSnapshot] = None
        # Stable dashboard map reference, independent of DFS mission restarts.
        self.initial_sensor_heading: Optional[float] = None
        self.last_camera_at: Optional[float] = None
        self.visible_marker: Optional[int] = None
        self.visible_distance_mm: Optional[float] = None
        self.visible_bearing_deg: Optional[float] = None
        # All accepted camera sightings for display, independent of movement/arrival.
        self.camera_markers = ()
        self.camera_heading = None
        self.camera_grid_cell = None
        # Latest confirmed stand-off visit, including manual driving outside DFS.
        self.last_reached_marker: Optional[int] = None
        self.grid = None
        self.grid_required = False
        self.grid_mission_origin = (0, 0)
        self._grid_pending = None
        self._grid_reason = ''
        self._input_hold = None
        self._shimmy = None
        self.last_frame = -1
        self.active = False
        self.phase = "idle"
        self.status = "Waiting for marker and sensors"
        self.map_zero_deg = 0.0
        self.threshold_mm = 0
        self.last_command: Optional[str] = None
        self.last_command_at = -float("inf")
        self.ready_at = 0.0
        self.departed = False
        self.scan_attempted: Dict[int, int] = {}
        self.scan_direction: Optional[Direction] = None
        self.scan_turn_deadline = 0.0
        self._drive_turn_corrections = 0
        self._turn_budget_at = None
        self._session = 0
        self._sensor_timestamp: Optional[int] = None
        self._heading_epoch = 0
        self._heading_offset_deg = 0.0
        self._arrival_marker: Optional[int] = None
        self._arrival_count = 0
        self._arrival_at = -float("inf")
        self._hold_marker: Optional[int] = None
        self._reset_turn()

    def _reset_turn(self):
        self._turn_target: Optional[int] = None
        self._turn_acked = False
        self._turn_reached = False
        self._turn_deadline = 0.0

    def _account_turn_time(self, now):
        """Alignment retries do not consume the pending segment's travel budget."""
        if self.phase in ('turn', 'settle') and self.explorer.state == State.MOVING:
            if self._turn_budget_at is not None:
                self.explorer.issued_at += max(0.0, now - self._turn_budget_at)
            self._turn_budget_at = now
        else:
            self._turn_budget_at = None

    def _brake_after_turn(self, now, message):
        """Stop one attempt, then verify fresh settled heading before retrying."""
        scan_turn = self.phase == 'scan_turn'
        if not self._send('STOP', now, force=True):
            return
        self.phase = 'scan_settle' if scan_turn else 'settle'
        self.ready_at = now + self.STOP_SETTLE_S
        self._reset_turn()
        self._status(message)

    def _retry_turn(self, target, actual, now, scan=False):
        self._drive_turn_corrections += 1
        self.phase = 'scan_turn' if scan else 'turn'
        self.scan_turn_deadline = now + self.TURN_TIMEOUT_S
        self._reset_turn()
        self._status('Post-braking heading %.1f°, target %d°, error %+.1f°; '
                     'correction %d (retrying until aligned or DFS is stopped)' %
                     (actual, target, heading_error(target, actual), self._drive_turn_corrections))

    def configure_grid_start(self, marker_id, camera_height_mm=325.0, camera_forward_mm=120.0):
        if self.active:
            raise ValueError("Stop the mission before setting the starting node")
        sensor = self.sensor
        if sensor is None or self.clock() - sensor.received_at + sensor.age_s > self.SENSOR_MAX_AGE_S:
            raise ValueError("Fresh IMU/ToF telemetry is required to set map north")
        self.grid = GridLocalizer(marker_id, sensor.heading, camera_height_mm, camera_forward_mm)
        self.camera_markers = ()
        self.camera_heading = self.camera_grid_cell = None
        self.map_zero_deg = self.grid.zero
        self.last_reached_marker = marker_id
        self.explorer = DFSExplorer()
        self._grid_pending = None
        self._shimmy = None
        self._status("Starting node %d set; current direction is grid north" % marker_id)

    def motion_received(self, command):
        if self.grid is not None:
            if self._shimmy is not None and command in ('LEFT', 'RIGHT'):
                # This is a bounded lateral adjustment on the current leg, not
                # a new DFS direction or a node arrival.
                self.grid.forward = False
                self.grid.camera_lost()
                self.grid.confirmation_note = 'Lateral centring active; node confirmation is paused'
                return
            heading = (self.sensor.heading if self.sensor is not None else
                       self.grid.zero + (self.grid.direction or 0) * 90)
            self.grid.motion(command, heading)
            if command in ('FORWARD', 'ESTOP', 'RESET_SAFETY', 'BACKWARD', 'LEFT', 'RIGHT',
                           'ROTATE_LEFT', 'ROTATE_RIGHT', 'FRONT_LEFT', 'FRONT_RIGHT',
                           'BACK_LEFT', 'BACK_RIGHT', 'ZERO_HEADING') or command.startswith('TURN_TO:'):
                self._grid_pending = None

    def _grid_camera(self, markers, age_ms):
        now = self.clock()
        self.last_camera_at = now - age_ms / 1000
        self.camera_markers = tuple(markers) if age_ms <= VisionFeed.MAX_AGE_MS else ()
        self.camera_heading = None
        self.camera_grid_cell = self.grid.cells[self.grid.current]
        selected = min(markers, key=lambda m: m.camera_distance_mm, default=None)
        self.visible_marker = selected.marker_id if selected else None
        self.visible_distance_mm = selected.camera_distance_mm if selected else None
        self.visible_bearing_deg = selected.camera_bearing_deg if selected else None
        if (self.sensor is not None and now - self.sensor.received_at + self.sensor.age_s <= self.SENSOR_MAX_AGE_S
                and age_ms <= VisionFeed.MAX_AGE_MS):
            self.camera_heading = self.sensor.heading
            if self._shimmy is None:
                self.grid.observe(markers, now - age_ms / 1000, self.sensor.heading)
            if self.grid.fault is not None and self.active:
                self.stop(self.grid.fault)
        else:
            self.grid.camera_lost()
            self.grid.confirmation_note = 'Confirmation held: IMU/ToF or camera data is stale'
        return True

    def _bearing_target(self, marker_id=None):
        """Prefer the nearest marker ahead on the pending travel line."""
        now = self.clock()
        if self.last_camera_at is None or now - self.last_camera_at > VisionFeed.MAX_AGE_MS / 1000:
            return None
        segment = self.explorer.pending
        if segment is None:
            return None
        markers = list(self.camera_markers)
        if (not markers and self.visible_marker is not None and
                finite_number(self.visible_distance_mm) and finite_number(self.visible_bearing_deg)):
            markers = [MarkerObservation(self.visible_marker, self.visible_distance_mm, self.visible_bearing_deg)]
        candidates, unknown = [], []
        for marker in markers:
            if marker_id is not None:
                if marker.marker_id == marker_id and abs(marker.camera_bearing_deg) <= 60:
                    return marker
                continue
            if marker.marker_id == segment.from_marker or abs(marker.camera_bearing_deg) > 60:
                continue
            if self.grid is None or marker.marker_id not in self.grid.cells:
                unknown.append(marker)
                continue
            origin = self.grid.cells.get(segment.from_marker)
            cell = self.grid.cells[marker.marker_id]
            vx, vy = VECTORS[int(segment.direction)]
            if origin is not None:
                dx, dy = cell[0] - origin[0], cell[1] - origin[1]
                if dx * vy == dy * vx and dx * vx + dy * vy > 0:
                    candidates.append(marker)
        if marker_id is not None:
            return None  # Never switch anchors in the middle of a correction.
        return min(candidates or unknown, key=lambda m: m.camera_distance_mm, default=None)

    def _start_shimmy_if_needed(self, now):
        marker = self._bearing_target()
        if marker is None or abs(marker.camera_bearing_deg) <= self.ALIGN_TRIGGER_DEG:
            return False
        self._shimmy = dict(marker=marker.marker_id, started=now,
                            paused=0.0,
                            stage='brake', pulses=0, centred=0, last_frame=-1)
        if not self._send('STOP', now, force=True):
            self._shimmy = None
            return True
        self.phase = 'align'
        self.ready_at = now + self.STOP_SETTLE_S
        if self.grid is not None:
            self.grid.camera_lost()
            self.grid.confirmation_note = 'Lateral centring active; node confirmation is paused'
        self._status("Centring marker %d: bearing %+.1f° outside ±%.0f°; "
                     "strafing until within ±%.0f°" %
                     (marker.marker_id, marker.camera_bearing_deg,
                      self.ALIGN_TRIGGER_DEG, self.ALIGN_TARGET_DEG))
        return True

    def _shimmy_tick(self, now):
        state = self._shimmy
        if state is None:
            self.stop('Missing bearing-correction state')
            return
        marker = self._bearing_target(state['marker'])
        if state['stage'] == 'pulse':
            side = 'right' if state['command'] == 'RIGHT' else 'left'
            clearance = GridLocalizer.reading(self.sensor, side)
            if clearance is None or clearance < self.ALIGN_SIDE_CLEARANCE_MM:
                self.stop('Lateral centring stopped: %s side clearance is %s' %
                          (side, 'unavailable' if clearance is None else '%d mm' % clearance))
                return
            # End a pulse immediately if the target disappears or reaches the band.
            if (now >= self.ready_at or marker is None or
                    abs(marker.camera_bearing_deg) <= self.ALIGN_TARGET_DEG):
                if self._send('STOP', now, force=True):
                    state['stage'] = 'brake'
                    state['centred'] = 0
                    self.ready_at = now + self.STOP_SETTLE_S
                return
            # LEFT/RIGHT are manual primitives with the Pico's 300 ms watchdog.
            self._send(state['command'], now, force=True)
            return
        if now < self.ready_at:
            return
        if marker is None:
            # Repeated polling of an ageing frame is not a new failed reading.
            # A fresh frame without the anchor does break consecutive confirmation.
            if state['last_frame'] != self.last_frame:
                state['centred'] = 0
                state['last_frame'] = self.last_frame
            self._status('Centring waiting: need a fresh view of marker %d; holding STOP' % state['marker'])
            return
        if self.last_camera_at < self.ready_at or self.sensor.received_at < self.ready_at:
            return  # Require a post-braking observation, not a queued pre-strafe frame.
        bearing = marker.camera_bearing_deg
        if abs(bearing) <= self.ALIGN_TARGET_DEG:
            if state['last_frame'] != self.last_frame:
                state['last_frame'] = self.last_frame
                state['centred'] += 1
            if state['centred'] < self.ALIGN_CONFIRM_FRAMES:
                return
            self.explorer.issued_at += now - state['started'] - state['paused']
            self._shimmy = None
            self.phase = 'settle'
            self.ready_at = now + self.STOP_SETTLE_S
            self._status('Marker %d centred at %+.1f°; checking inputs before forward travel' %
                         (marker.marker_id, bearing))
            return
        state['centred'] = 0
        command = 'RIGHT' if bearing > 0 else 'LEFT'
        side = 'right' if bearing > 0 else 'left'
        distance = GridLocalizer.reading(self.sensor, side)
        if distance is None or distance < self.ALIGN_SIDE_CLEARANCE_MM:
            self.stop('Cannot centre %s: %s ToF %s; need valid side clearance ≥%d mm' %
                      (command, side, 'unavailable' if distance is None else '%d mm' % distance,
                       self.ALIGN_SIDE_CLEARANCE_MM))
            return
        state['command'] = command
        state['pulses'] += 1
        if self._send(command, now, force=True):
            state['stage'] = 'pulse'
            self.ready_at = now + self.ALIGN_PULSE_S
            self._status('Centring marker %d: %s pulse %d, bearing %+.1f°' %
                         (marker.marker_id, command, state['pulses'], bearing))

    def handle_front_obstacle(self):
        """An expected terminal wall is an arrival; other obstacles still pause."""
        if self.grid is None or not self.active or self.phase not in ('drive', 'settle'):
            return False
        marker, reason = self.grid.candidate(self.clock())
        if reason != 'terminal wall stop' or marker is None:
            return False
        return self._begin_grid_arrival(marker, reason)

    def _begin_grid_arrival(self, marker, reason):
        path = self.grid.path_to(marker)
        if not path:
            return False
        if self.active and not self._send('STOP', self.clock(), force=True):
            return False
        self._grid_pending = (marker, path)
        self._grid_reason = reason
        self.ready_at = self.clock() + self.STOP_SETTLE_S
        if self.active:
            self.phase = 'arrival_settle'
        self._status("Grid node %d: %s; braking before confirmation" % (marker, reason))
        return True

    def _grid_tick(self, now):
        grid, sensor = self.grid, self.sensor
        if sensor is None or now - sensor.received_at + sensor.age_s > grid.MAX_AGE_S:
            return
        if self._grid_pending is not None:
            if now < self.ready_at or sensor.received_at < self.ready_at:
                return
            if self._grid_reason == 'terminal wall stop' and not grid.wall_stop:
                self._grid_pending = None
                if self.active:
                    self.stop("Wall-stop reading disappeared before node confirmation")
                return
            marker, path = self._grid_pending
            if self.active:
                self._commit_grid_path(path, now)
                if not self.active and self.explorer.state != State.COMPLETE:
                    self._grid_pending = None
                    return
            grid.current = marker
            grid.visited.update(path)
            self.last_reached_marker = marker
            self._grid_pending = None
            grid.message = "At node %d: %s" % (marker, self._grid_reason)
            if self.active:
                self.phase = 'scan'
                self._reset_turn()
                self.scan_attempted.pop(marker, None)
                self._status("Arrived at grid node %d; continuing DFS" % marker)
            return
        marker, reason = grid.candidate(now)
        if marker is None or marker == grid.current:
            if self.active and self.phase == 'drive' and grid.wall_stop:
                self.stop("Front obstacle: no confirmed terminal marker/path")
            return
        if self.active:
            if self.phase == 'drive' and not self._begin_grid_arrival(marker, reason):
                self.stop("Grid arrival has an unknown or conflicting path")
        elif reason == 'terminal wall stop':
            if not grid.forward:
                self._begin_grid_arrival(marker, reason)
        elif (grid.forward or reason == 'position from fixed forward marker') and grid.path_to(marker):
            grid.visited.update(grid.path_to(marker))
            grid.current = marker
            self.last_reached_marker = marker
            grid.message = "At node %d: %s" % (marker, reason)

    def _commit_grid_path(self, path, now):
        """Record every known intermediate node when a wall confirms a longer hop."""
        desired = self.explorer.target_markers
        for index, marker in enumerate(path):
            segment = self.explorer.pending
            if segment is None or int(segment.direction) != self.grid.direction:
                self.stop("Grid arrival disagrees with the pending DFS direction")
                return
            # Keep a multi-cell terminal confirmation contiguous in the DFS graph.
            self.explorer.target_markers = 36 if index < len(path) - 1 else desired
            if not self.explorer.arrive(segment.session_id, segment.segment_id, marker,
                                        MarkerType.JUNCTION, segment.direction, now):
                self.explorer.target_markers = desired
                self.stop("Grid arrival rejected: " + (self.explorer.fault or 'topology'))
                return
            if index < len(path) - 1:
                self.explorer.observe_exits(marker, 1 << self.grid.direction, 0)
                action = self.explorer.next_action(now)
                if action.kind != 'MOVE' or int(action.segment.direction) != self.grid.direction:
                    self.explorer.target_markers = desired
                    self.stop("Intermediate grid path could not be recorded")
                    return
        self.explorer.target_markers = desired
        if self.explorer.state == State.COMPLETE:
            self._finish("Complete: %d distinct markers" % len(self.explorer.nodes), now)

    def _local_turn_active(self):
        return self.active and self.phase in ("turn", "scan_turn")

    def _hold_for_inputs(self, reason):
        if self.grid is None:
            self.stop(reason)
            return
        if self._input_hold is not None:
            return
        now = self.clock()
        previous = self.phase
        remaining = max(0.0, self.ready_at - now)
        if not self._send('STOP', now, force=True):
            return
        self._input_hold = (previous, now, remaining)
        self._turn_budget_at = None
        self.phase = 'input_wait'
        if self._shimmy is not None:
            self._shimmy['stage'] = 'brake'
            self._shimmy['centred'] = 0
        self.grid.camera_lost()
        self._status("DFS waiting for inputs: %s. STOP sent; will resume automatically "
                     "if data recovers within %.0fs" % (reason, self.INPUT_RECOVERY_WAIT_S))

    def _recover_inputs(self, now):
        previous, started, remaining = self._input_hold
        elapsed = now - started
        if elapsed >= self.INPUT_RECOVERY_WAIT_S:
            self.stop("Inputs unavailable for %.0fs; automatic recovery expired: %s" %
                      (self.INPUT_RECOVERY_WAIT_S, self._input_problem(now) or 'recovery arrived too late'))
            return
        if self._input_problem(now):
            return
        self._input_hold = None
        if self._shimmy is not None:
            self._shimmy['paused'] += elapsed
        if self.explorer.state == State.MOVING:
            self.explorer.issued_at += elapsed
        self.scan_turn_deadline += elapsed
        self._turn_deadline += elapsed
        self.phase = 'settle' if previous == 'drive' else previous
        self._turn_budget_at = now if self.phase in ('turn', 'settle') else None
        self.ready_at = now + max(self.STOP_SETTLE_S, remaining)
        self._status("Inputs restored; resuming the same DFS mission after braking/input checks")

    def robot_status_received(self, status):
        """Consume robot/status on the same Tk thread as tick()."""
        if not self.active or self.phase not in ("turn", "scan_turn") or self._turn_target is None:
            return
        command = "TURN_TO:%d" % self._turn_target
        if status == "AUTO_MOVE: " + command:
            self._turn_acked = True
        elif status.startswith("AUTO_REFUSED: TURN_TO"):
            self.stop("RoboPico refused turn: " + status)
        elif status.startswith("AUTO_RESULT: ") and self._turn_acked:
            if status == "AUTO_RESULT: HEADING_REACHED":
                self._turn_reached = True
            else:
                self.stop("RoboPico turn ended: " + status)

    def _status(self, message):
        if message != self.status:
            self.status = message
            self.report(message)

    def sensor_received(self, data):
        try:
            timestamp = data["timestamp"]
            if type(timestamp) is not int or not 0 <= timestamp <= 0xFFFFFFFF:
                raise ValueError("Invalid sensor timestamp")
            epoch = data.get("heading_epoch", self._heading_epoch)
            offset = data.get("heading_offset_deg", self._heading_offset_deg)
            if (type(epoch) is not int or not 0 <= epoch <= 0xFFFFFFFF or
                    not finite_number(offset) or not 0 <= offset < 360):
                raise ValueError("Invalid heading reference")
            if epoch != self._heading_epoch or offset != self._heading_offset_deg:
                if self.active:
                    self.stop("IMU heading zero changed; start a new mission")
                # Keep existing maps in their original physical orientation as
                # the absolute IMU reference moves. Old arrival frames are reset.
                delta = offset - self._heading_offset_deg
                if self.initial_sensor_heading is not None:
                    self.initial_sensor_heading = (self.initial_sensor_heading - delta) % 360
                self.map_zero_deg = (self.map_zero_deg - delta) % 360
                if self.grid is not None:
                    self.grid.rebase_heading(delta)
                self._heading_epoch, self._heading_offset_deg = epoch, offset
                self._sensor_timestamp = None
                self._reset_arrival()
                self._hold_marker = self.visible_marker = None
                self.camera_markers = ()
                self.camera_heading = None
                self._reset_turn()
            if timestamp == self._sensor_timestamp:
                return  # Duplicate delivery must not refresh input health.
            if self._sensor_timestamp is not None and timestamp < self._sensor_timestamp and self.active:
                self.stop("Sensor clock restarted or rolled over")
            self._sensor_timestamp = timestamp
            self.sensor = SensorSnapshot.decode(data, self.clock())
            if self.grid is not None:
                self.grid.sensors(self.sensor, self.clock(), self.threshold_mm or 500)
            if self.initial_sensor_heading is None and self.sensor.age_s <= self.SENSOR_MAX_AGE_S:
                self.initial_sensor_heading = self.sensor.heading
        except (KeyError, TypeError, ValueError):
            self.sensor = None
            if self.active and not self._local_turn_active() and self.phase not in ("settle", "scan_settle", "arrival_settle"):
                self._hold_for_inputs("Invalid sensor data")

    def _reset_arrival(self):
        self._arrival_marker = None
        self._arrival_count = 0

    def vision_received(self, data):
        """Consume B3's full JSON envelope; choose one valid marker per frame."""
        try:
            frame = self.vision_feed.decode(data, self.wall_clock() * 1000)
        except (KeyError, TypeError, ValueError):
            self._reset_arrival()
            self.last_camera_at = None
            self.camera_markers = ()
            if self.active and not self._local_turn_active() and self.phase not in ("settle", "scan_settle", "arrival_settle"):
                self._hold_for_inputs("Invalid camera observation")
            return False
        if frame is None:
            return False
        if frame.restarted:
            self._reset_arrival()
            self._hold_marker = None
            self.visible_marker = None
            if self.active:
                self.stop("Camera detector restarted; start a new mission")
        if frame.status in ("stalled", "disconnected", "stale"):
            self.camera_markers = ()
            if self.grid is not None:
                self.grid.camera_lost()
            self._reset_arrival()
            self.last_camera_at = None
            self.visible_marker = None
            if self.active and not self._local_turn_active() and self.phase not in ("settle", "scan_settle", "arrival_settle"):
                self._hold_for_inputs("Camera " + frame.status)
            return False  # Health messages never refresh camera freshness.
        if self.grid is not None:
            self.last_frame += 1
            return self._grid_camera(frame.markers, frame.age_ms)
        candidates = frame.markers
        segment = self.explorer.pending
        if self.active and self.phase == "arrival_hold":
            # Confirm the marker we stopped at even if a side marker appears.
            candidates = [marker for marker in candidates if marker.marker_id == self._hold_marker]
        elif self.active and segment is not None:
            candidates = [marker for marker in candidates if marker.marker_id != segment.from_marker]
        # Nearest first: on the 60 cm grid a far marker straight ahead can hide the one about to be hit.
        selected = min(candidates, key=lambda marker: (marker.camera_distance_mm,
                       abs(marker.camera_bearing_deg), marker.marker_id), default=None)
        return self.marker_seen(selected.marker_id if selected else None,
                                self.last_frame + 1, frame.age_ms,
                                selected.camera_distance_mm if selected else None,
                                selected.camera_bearing_deg if selected else None)

    def marker_seen(self, marker_id, frame_id, age_ms=0.0, distance_mm=None, bearing_deg=None):
        """Lens-relative observation. IDs alone cannot start/complete a visit.

        frame_id must increase for the lifetime of this navigator. A stale or
        repeated frame cannot refresh camera health or confirm another arrival.
        """
        if (type(frame_id) is not int or frame_id <= self.last_frame or
                (marker_id is not None and (type(marker_id) is not int or not 1 <= marker_id <= 36)) or
                isinstance(age_ms, bool) or not isinstance(age_ms, (int, float)) or
                not math.isfinite(age_ms) or not 0 <= age_ms <= self.CAMERA_MAX_AGE_S * 1000):
            return False
        now = self.clock()
        self.last_frame = frame_id
        if self.grid is not None:
            markers = ([MarkerObservation(marker_id, float(distance_mm), float(bearing_deg))]
                       if marker_id is not None and finite_number(distance_mm) and distance_mm > 0
                       and finite_number(bearing_deg) else [])
            return self._grid_camera(markers, age_ms)
        self.last_camera_at = now - age_ms / 1000
        self.visible_marker = marker_id
        self.visible_distance_mm = (float(distance_mm) if marker_id is not None and
                                    finite_number(distance_mm) and distance_mm > 0 else None)
        self.visible_bearing_deg = (float(bearing_deg) if marker_id is not None and
                                    finite_number(bearing_deg) else None)
        if marker_id is None:
            self._reset_arrival()
            if self.phase == "drive":
                self.departed = True
            return True
        if self.active and self.phase not in ("drive", "arrival_hold"):
            self._reset_arrival()
            return True
        segment = self.explorer.pending
        if self.active and segment is None:
            self.stop("Missing pending movement at marker arrival")
            return False
        if self.active and segment is not None and marker_id == segment.from_marker:
            self._reset_arrival()
            return True
        if (not finite_number(distance_mm) or distance_mm <= 0 or not finite_number(bearing_deg) or
                age_ms > VisionFeed.MAX_AGE_MS):
            self._reset_arrival()
            return True
        aligned = abs(bearing_deg) <= self.ARRIVAL_BEARING_DEG
        in_band = abs(distance_mm - self.ARRIVAL_DISTANCE_MM) <= self.ARRIVAL_TOLERANCE_MM
        if self.active and self.phase == "drive":
            if distance_mm > self.ARRIVAL_DISTANCE_MM:
                self._reset_arrival()
                return True  # Visible ahead; keep approaching, do not add a node.
            if not aligned or not in_band:
                self.stop("Marker reached stop distance outside arrival alignment/band")
                return False
            if segment is not None and segment.target_marker is not None and marker_id != segment.target_marker:
                self.stop("Unexpected marker at arrival distance")
                return False
            # Stop on the first qualifying <=500 mm observation. Do not continue
            # driving while accumulating the five confirmation frames.
            if not self._send("STOP", now, force=True):
                return False
            self.phase = "arrival_hold"
            self._hold_marker = marker_id
            self._reset_arrival()
        if self.active and self.phase == "arrival_hold" and marker_id != self._hold_marker:
            self.stop("Marker changed while confirming arrival")
            return False
        if not aligned or not in_band:
            self._reset_arrival()
            return True  # Keep holding STOP; wait for stable observations.
        if now - self._arrival_at > VisionFeed.MAX_AGE_MS / 1000:
            self._reset_arrival()
        if self._arrival_marker == marker_id:
            self._arrival_count += 1
        else:
            self._arrival_marker, self._arrival_count = marker_id, 1
        self._arrival_at = now - age_ms / 1000
        if not self.active:
            if self._arrival_count >= self.ARRIVAL_CONFIRM_FRAMES:
                self.last_reached_marker = marker_id
            return True
        if self._arrival_count < self.ARRIVAL_CONFIRM_FRAMES:
            return True
        if segment is None:
            self.stop("Missing movement while confirming arrival")
            return False
        # STOP was sent on the first arrival frame. Finish braking before
        # committing this visit and choosing the next DFS branch.
        self.phase = "arrival_settle"
        self.ready_at = now + self.STOP_SETTLE_S
        self._status("Marker %d confirmed; settling before the next DFS decision" % marker_id)
        return True

    def _complete_arrival(self, now):
        segment = self.explorer.pending
        marker_id = self._hold_marker
        if segment is None or marker_id is None:
            self.stop("Missing confirmed marker or movement at arrival")
            return
        heading = self._map_heading()
        if not self.explorer.arrive(segment.session_id, segment.segment_id, marker_id,
                                    MarkerType.JUNCTION, heading, now):
            self.stop("Marker arrival rejected: " + (self.explorer.fault or "state mismatch"))
            return
        self.last_reached_marker = marker_id
        self._hold_marker = None
        self._reset_arrival()
        self._reset_turn()
        self.scan_attempted.pop(marker_id, None)
        self.phase = "scan"
        if self.explorer.state == State.COMPLETE:
            self._finish("Complete: %d distinct markers" % len(self.explorer.nodes), now)
        else:
            self._status("Arrived at marker %d (%d/%d); continuing DFS exit scan" %
                         (marker_id, len(self.explorer.nodes), self.explorer.target_markers))

    def _inputs_fresh(self, now):
        return self._input_problem(now) is None

    def _input_problem(self, now):
        sensor = self.sensor
        last_camera_at = self.last_camera_at
        problems = []
        if sensor is None:
            problems.append("sensor telemetry unavailable")
        else:
            age = now - sensor.received_at + sensor.age_s
            if age > self.SENSOR_MAX_AGE_S:
                problems.append("sensor telemetry age %.2fs > %.2fs" % (age, self.SENSOR_MAX_AGE_S))
        if last_camera_at is None:
            problems.append("camera frames unavailable")
        elif now - last_camera_at > self.CAMERA_MAX_AGE_S:
            problems.append("camera frame age %.2fs > %.2fs" %
                            (now - last_camera_at, self.CAMERA_MAX_AGE_S))
        return "; ".join(problems) if problems else None

    def _map_heading(self):
        sensor = self.sensor
        if sensor is None:
            raise ValueError("Cannot calculate map heading without a sensor snapshot")
        return Direction(int(((sensor.heading - self.map_zero_deg) % 360 + 45) // 90) % 4)

    def start(self, target_markers, threshold_mm):
        now = self.clock()
        sensor = self.sensor
        marker_id = self.grid.current if self.grid is not None else self.visible_marker
        if self.grid_required and self.grid is None:
            raise ValueError("Enter the node under the rover in Start node ID, then click Set start node")
        if type(threshold_mm) is not int or not 1 <= threshold_mm <= 1000:
            raise ValueError("Set the open-path threshold to 1..1000 mm")
        if sensor is None or not self._inputs_fresh(now) or marker_id is None:
            raise ValueError("Require a current marker and fresh camera/sensor data")
        if self.grid is None and (self._arrival_marker != marker_id or
                self._arrival_count < self.ARRIVAL_CONFIRM_FRAMES or
                now - self._arrival_at > VisionFeed.MAX_AGE_MS / 1000):
            raise ValueError("Position at 500 mm from the camera lens with five aligned valid frames")
        if self.active:
            raise ValueError("Stop the existing DFS mission first")
        # These existing commands stop any inherited Pico AUTO routine too.
        if not self._send("STOP", now, force=True):
            raise ValueError("Cannot publish STOP to RoboPico")
        self._session += 1
        self.explorer.start(self._session, target_markers, marker_id,
                            MarkerType.JUNCTION,
                            (Direction(round(((sensor.heading - self.grid.zero) % 360) / 90) % 4)
                             if self.grid is not None else Direction.NORTH), segment_timeout_s=30.0)
        self.last_reached_marker = marker_id
        self.map_zero_deg = self.grid.zero if self.grid is not None else sensor.heading
        self.grid_mission_origin = self.grid.cells[marker_id] if self.grid is not None else (0, 0)
        self._grid_pending = None
        self.threshold_mm = threshold_mm
        self.active = True
        self.phase = "scan"
        self._drive_turn_corrections = 0
        self._turn_budget_at = None
        self._shimmy = None
        self.scan_attempted = {}
        self._hold_marker = None
        self._reset_arrival()
        self._reset_turn()
        self.ready_at = now + self.STOP_SETTLE_S
        self._status("Laptop DFS started at marker %d" % marker_id)

    def _send(self, command, now, force=False):
        if not force and command == self.last_command and now - self.last_command_at < self.COMMAND_REFRESH_S:
            return True
        try:
            published = self.send_command(command) is True
        except Exception:
            published = False
        if not published:
            self._shimmy = None
            self._turn_budget_at = None
            self.motion_received('STOP')
            self.active = False
            self.explorer.pause()
            self.phase = "idle"
            if command != "STOP":
                try:
                    self.send_command("STOP")
                except Exception:
                    pass  # RoboPico's local command timeout is the final stop.
            self._status("DFS paused: command not sent (link guard, connection or MQTT publish failure)")
            return False
        self.last_command, self.last_command_at = command, now
        self.motion_received(command)
        return True

    def stop(self, reason="Stopped by operator"):
        was_active = self.active
        self.active = False
        self.explorer.pause()
        self.phase = "idle"
        self._hold_marker = None
        self._grid_pending = None
        self._input_hold = None
        self._shimmy = None
        self._turn_budget_at = None
        self._reset_arrival()
        self._reset_turn()
        if was_active:
            self._send("STOP", self.clock(), force=True)
        self.motion_received('STOP')
        self._status("DFS paused: " + reason)

    def tick(self):
        if not self.active:
            if self.grid is not None:
                self._grid_tick(self.clock())
            return
        now = self.clock()
        self._account_turn_time(now)
        if self._input_hold is not None:
            self._recover_inputs(now)
            return
        sensor = self.sensor
        local_turn = self._local_turn_active()
        # Rotation is closed locally against UART IMU data. Laptop camera/MQTT
        # gaps must not abort it halfway; Pico's heading/link watchdogs remain.
        if not local_turn:
            problem = self._input_problem(now)
            if problem:
                if self.grid is not None:
                    self._hold_for_inputs(problem)
                elif (self.phase in ("settle", "scan_settle", "arrival_settle") and
                        now <= self.ready_at + self.POST_TURN_INPUT_WAIT_S):
                    self._status("Holding STOP while waiting for fresh camera/sensor data")
                else:
                    self.stop("Inputs unavailable: " + problem)
                return
        if self.phase == 'align':
            self._shimmy_tick(now)
            return
        if (self.phase == 'drive' and
                not (self.grid is not None and self.grid.wall_stop) and
                self._start_shimmy_if_needed(now)):
            return
        if self.grid is not None and self.phase in ('drive', 'arrival_settle'):
            self._grid_tick(now)
            if not self.active or self.phase != 'drive':
                return
        if self.grid is None and sensor is not None and sensor.stop_required and self.phase == "drive":
            self.stop("Sensor stop request")
            return
        self.explorer.tick(now)
        if self.explorer.state == State.FAULT:
            self.stop("DFS fault: " + (self.explorer.fault or "unknown"))
            return
        if self.phase == "scan_turn" and now >= self.scan_turn_deadline:
            self._brake_after_turn(now, 'Exit turn attempt timed out; braking and checking fresh heading before retrying')
            return
        if local_turn and self._turn_target is not None and now >= self._turn_deadline:
            self._brake_after_turn(now, 'Turn to %d attempt timed out; braking and checking fresh heading before retrying' %
                                   self._turn_target)
            return
        if now < self.ready_at:
            return
        if self.phase == "arrival_hold":
            return  # Wait stationary for camera confirmation, still checking timeouts.
        if self.phase == "arrival_settle":
            if sensor.received_at >= self.ready_at:
                self._complete_arrival(now)
            return
        if self.phase == "scan_settle":
            if sensor.received_at < self.ready_at:
                return
            target = (self.map_zero_deg + int(self.scan_direction) * 90) % 360
            error = abs(heading_error(target, sensor.heading))
            if error <= self.SCAN_TOLERANCE_DEG:
                self.phase = 'scan'
            else:
                self._retry_turn(int(target + .5) % 360, sensor.heading, now, scan=True)
            return
        if self.phase == "scan":
            if self.explorer.state == State.COMPLETE:
                self._finish("Complete: %d distinct markers" % len(self.explorer.nodes), now)
                return
            current = self.explorer.current
            if current is None:
                self.stop("Missing current marker for exit scan")
                return
            if sensor.received_at < self.ready_at:
                return  # Never classify exits using the approach/rotation snapshot.
            heading = self._map_heading()
            expected_heading = self.map_zero_deg + int(heading) * 90
            if abs(heading_error(expected_heading, sensor.heading)) > self.SCAN_TOLERANCE_DEG:
                self.stop("Heading is not aligned to the map axes")
                return
            self.explorer.heading = heading
            # In grid mode 60 cm is an approach warning, not a closed forward
            # exit. Keep approaching a wall until the existing 15 cm motor stop.
            opened, blocked = sensor.exits(heading, self.threshold_mm,
                front_threshold_mm=self.grid.WALL_STOP_MM if self.grid is not None else None)
            self.explorer.observe_exits(current, opened, blocked)
            attempted = self.scan_attempted.get(current, 0)
            for offset in (3, 0, 1):
                attempted |= 1 << ((heading + offset) % 4)
            self.scan_attempted[current] = attempted
            action = self.explorer.next_action(now)
            if action.kind == "MOVE":
                segment = action.segment
                if segment is None:
                    self.stop("DFS move is missing its segment")
                    return
                self.phase = "turn"
                self._drive_turn_corrections = 0
                self._turn_budget_at = now
                self._reset_turn()
                self.departed = False
                self._reset_arrival()
                self._status("Marker %d: %s %s" % (current,
                             segment.kind.value, segment.direction.name))
            elif action.kind == "SCAN":
                node = self.explorer.nodes[current]
                unseen = 15 & ~node.known_exits & ~attempted
                if unseen:
                    if self.grid is not None and self.grid.reading(sensor, 'front') is None:
                        self._status("Marker %d: waiting for a valid front ToF before rear inspection "
                                     "(front=%s, faults=0x%02x)" %
                                     (current, sensor.distances.get('front'), sensor.faults))
                        return
                    # The three ToFs do not look behind the rover. Inspect an
                    # unobserved rear exit by turning in place, without advancing.
                    scan_direction = next(direction for direction in Direction if unseen & (1 << direction))
                    self.scan_direction = scan_direction
                    self.phase = "scan_turn"
                    self._drive_turn_corrections = 0
                    self._reset_turn()
                    self.scan_turn_deadline = now + 15.0
                    self._status("Marker %d: turning to inspect %s | ToF front=%s, left=%s, "
                                 "right=%s mm; faults=0x%02x; forward stop < %d mm" %
                                 (current, scan_direction.name, sensor.distances.get('front'),
                                  sensor.distances.get('left'), sensor.distances.get('right'),
                                  sensor.faults, self.grid.WALL_STOP_MM if self.grid is not None
                                  else self.threshold_mm))
                else:
                    self._status("Marker %d: waiting for usable exit measurements" % current)
            elif action.kind in ("COMPLETE", "EXHAUSTED", "FAULT"):
                self._finish(action.kind + (": " + action.fault if action.fault else ""), now)
            return
        segment = self.explorer.pending
        if self.phase in ("turn", "scan_turn"):
            scan_turn = self.phase == "scan_turn"
            if scan_turn:
                direction = self.scan_direction
                if direction is None:
                    self.stop("Missing direction for exit scan turn")
                    return
            else:
                if segment is None:
                    self.stop("Missing pending movement for turn")
                    return
                direction = segment.direction
            target = int((self.map_zero_deg + int(direction) * 90) % 360 + 0.5) % 360
            if self._turn_target != target:
                if (not scan_turn and sensor is not None and
                        now - sensor.received_at + sensor.age_s <= self.SENSOR_MAX_AGE_S and
                        abs(heading_error(target, sensor.heading)) <= self.TURN_TOLERANCE_DEG):
                    # Straight-ahead moves should not start a needless local
                    # rotation. Still brake and recheck fresh inputs before driving.
                    if self._send('STOP', now, force=True):
                        self.phase = 'settle'
                        self.ready_at = now + self.STOP_SETTLE_S
                        self._reset_turn()
                        self._status("Already aligned for %s: heading %.1f°, target %d°; "
                                     "skipping rotation, checking inputs before forward travel" %
                                     (direction.name, sensor.heading, target))
                    return
                self._turn_target = target
                self._turn_acked = self._turn_reached = False
                self._turn_deadline = now + self.TURN_TIMEOUT_S
                if scan_turn:
                    self.scan_turn_deadline = self._turn_deadline
                # One command per turn. Reissuing would create a new Pico move
                # and could restart a completed turn. Its driver refreshes locally.
                self._send("TURN_TO:%d" % target, now, force=True)
            elif self._turn_reached:
                self._brake_after_turn(now, "Turn to %d reached; braking before checking fresh inputs" % target)
        elif self.phase in ("settle", "drive"):
            if segment is None:
                self.stop("Missing pending movement for forward travel")
                return
            if self.phase == "settle":
                if sensor.received_at < self.ready_at:
                    return  # Verify a post-braking heading before driving.
                target = int((self.map_zero_deg + int(segment.direction) * 90) % 360 + .5) % 360
                error = heading_error(target, sensor.heading)
                if abs(error) > self.TURN_TOLERANCE_DEG:
                    self._retry_turn(target, sensor.heading, now)
                    return  # Braking drift must be corrected before driving.
                if self.grid is not None:
                    marker, reason = self.grid.candidate(now)
                    if (marker is not None and marker != self.grid.current and
                            reason == 'position from fixed forward marker' and
                            self._begin_grid_arrival(marker, reason)):
                        return
                if sensor.stop_required:
                    if self.handle_front_obstacle():
                        return
                    self.stop("Forward path blocked after turn")
                    return
                if self._start_shimmy_if_needed(now):
                    return
            starting_drive = self.phase == 'settle'
            self.phase = "drive"
            if self._send("FORWARD", now) and starting_drive:
                self._status("Driving %s: FORWARD sent (heading %.1f°)" %
                             (segment.direction.name, sensor.heading))

    def _finish(self, message, now):
        self.active = False
        self.phase = "idle"
        self._send("STOP", now, force=True)
        self._status("DFS " + message)
