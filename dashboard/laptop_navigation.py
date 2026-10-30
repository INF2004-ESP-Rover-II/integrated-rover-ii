"""Join forward-camera observations and MQTT sensor snapshots on the laptop.

RoboPico receives its existing ROTATE_LEFT/ROTATE_RIGHT/FORWARD/STOP commands.
Feed B3's JSON into vision_received(), or supply lens distance and bearing through
marker_seen(). Detection alone is not arrival. All methods have one thread owner.
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

from dfs_explorer import DFSExplorer, Direction, MarkerType, State
from vision_feed import VisionFeed, finite_number


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

    def exits(self, heading, threshold_mm):
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
            elif distance >= threshold_mm:
                opened |= bit
            else:
                blocked |= bit
        return opened, blocked


class LaptopNavigator:
    SENSOR_MAX_AGE_S = 0.75
    CAMERA_MAX_AGE_S = 1.0
    COMMAND_REFRESH_S = 0.2
    TURN_TOLERANCE_DEG = 10.0
    SCAN_TOLERANCE_DEG = 20.0
    STOP_SETTLE_S = 0.4  # Exceeds RoboPico's current 300 ms active brake hold.
    ARRIVAL_DISTANCE_MM = 450.0  # Agreed: camera lens to marker centre.
    ARRIVAL_TOLERANCE_MM = 30.0  # Confirmation band after stopping; tune on hardware.
    ARRIVAL_BEARING_DEG = 30.0  # shortcut: loosened from 5.0 for mapping tests, restore before real runs.
    ARRIVAL_CONFIRM_FRAMES = 5

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
        self._session = 0
        self._sensor_timestamp: Optional[int] = None
        self._arrival_marker: Optional[int] = None
        self._arrival_count = 0
        self._arrival_at = -float("inf")
        self._hold_marker: Optional[int] = None

    def _status(self, message):
        if message != self.status:
            self.status = message
            self.report(message)

    def sensor_received(self, data):
        try:
            timestamp = data["timestamp"]
            if type(timestamp) is not int or not 0 <= timestamp <= 0xFFFFFFFF:
                raise ValueError("Invalid sensor timestamp")
            if timestamp == self._sensor_timestamp:
                return  # Duplicate delivery must not refresh input health.
            if self._sensor_timestamp is not None and timestamp < self._sensor_timestamp and self.active:
                self.stop("Sensor clock restarted or rolled over")
            self._sensor_timestamp = timestamp
            self.sensor = SensorSnapshot.decode(data, self.clock())
            if self.initial_sensor_heading is None and self.sensor.age_s <= self.SENSOR_MAX_AGE_S:
                self.initial_sensor_heading = self.sensor.heading
        except (KeyError, TypeError, ValueError):
            self.sensor = None
            if self.active:
                self.stop("Invalid sensor data")

    def _reset_arrival(self):
        self._arrival_marker = None
        self._arrival_count = 0

    def vision_received(self, data):
        """Consume B3's full JSON envelope; choose one valid marker per frame."""
        try:
            frame = self.vision_feed.decode(data, self.wall_clock() * 1000)
        except (KeyError, TypeError, ValueError):
            self._reset_arrival()
            if self.active:
                self.stop("Invalid camera observation")
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
            self._reset_arrival()
            if self.active:
                self.stop("Camera " + frame.status)
            return False  # Health messages never refresh camera freshness.
        candidates = frame.markers
        segment = self.explorer.pending
        if self.active and segment is not None:
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
        self.last_camera_at = now - age_ms / 1000
        self.visible_marker = marker_id
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
            # Stop on the first qualifying <=450 mm observation. Do not continue
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
        if not self.active or self._arrival_count < self.ARRIVAL_CONFIRM_FRAMES:
            return True
        if segment is None:
            self.stop("Missing movement while confirming arrival")
            return False
        if not self._inputs_fresh(now):
            self.stop("Stale inputs at marker arrival")
            return False
        if not self._send("STOP", now, force=True):
            return False
        heading = self._map_heading()
        if not self.explorer.arrive(segment.session_id, segment.segment_id, marker_id,
                                    MarkerType.JUNCTION, heading, now):
            self.stop("Marker arrival rejected: " + (self.explorer.fault or "state mismatch"))
            return False
        self.phase = "scan"
        self._hold_marker = None
        self._reset_arrival()
        self.ready_at = now + self.STOP_SETTLE_S
        self._status("Arrived at marker %d; waiting for a fresh sensor scan" % marker_id)
        return True

    def _inputs_fresh(self, now):
        sensor = self.sensor
        last_camera_at = self.last_camera_at
        return (sensor is not None and
                now - sensor.received_at + sensor.age_s <= self.SENSOR_MAX_AGE_S and
                last_camera_at is not None and now - last_camera_at <= self.CAMERA_MAX_AGE_S)

    def _map_heading(self):
        sensor = self.sensor
        if sensor is None:
            raise ValueError("Cannot calculate map heading without a sensor snapshot")
        return Direction(int(((sensor.heading - self.map_zero_deg) % 360 + 45) // 90) % 4)

    def start(self, target_markers, threshold_mm):
        now = self.clock()
        sensor = self.sensor
        marker_id = self.visible_marker
        if type(threshold_mm) is not int or not 1 <= threshold_mm <= 1000:
            raise ValueError("Set the open-path threshold to 1..1000 mm")
        if sensor is None or not self._inputs_fresh(now) or marker_id is None or sensor.stop_required:
            raise ValueError("Require a current marker, fresh healthy sensors and no stop request")
        if (self._arrival_marker != marker_id or self._arrival_count < self.ARRIVAL_CONFIRM_FRAMES or
                now - self._arrival_at > VisionFeed.MAX_AGE_MS / 1000):
            raise ValueError("Position at 450 mm from the camera lens with five aligned valid frames")
        if self.active:
            raise ValueError("Stop the existing DFS mission first")
        # These existing commands stop any inherited Pico AUTO routine too.
        if not self._send("STOP", now, force=True):
            raise ValueError("Cannot publish STOP to RoboPico")
        self._session += 1
        self.explorer.start(self._session, target_markers, marker_id,
                            MarkerType.JUNCTION, Direction.NORTH, segment_timeout_s=30.0)
        self.map_zero_deg = sensor.heading
        self.threshold_mm = threshold_mm
        self.active = True
        self.phase = "scan"
        self.scan_attempted = {}
        self._hold_marker = None
        self._reset_arrival()
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
            self.active = False
            self.explorer.pause()
            self.phase = "idle"
            if command != "STOP":
                try:
                    self.send_command("STOP")
                except Exception:
                    pass  # RoboPico's local command timeout is the final stop.
            self._status("DFS paused: command publication failed")
            return False
        self.last_command, self.last_command_at = command, now
        return True

    def stop(self, reason="Stopped by operator"):
        was_active = self.active
        self.active = False
        self.explorer.pause()
        self.phase = "idle"
        self._hold_marker = None
        self._reset_arrival()
        if was_active:
            self._send("STOP", self.clock(), force=True)
        self._status("DFS paused: " + reason)

    def tick(self):
        if not self.active:
            return
        now = self.clock()
        sensor = self.sensor
        if sensor is None or not self._inputs_fresh(now):
            self.stop("Camera or sensor stream lost")
            return
        if sensor.stop_required:
            self.stop("Sensor stop request")
            return
        self.explorer.tick(now)
        if self.explorer.state == State.FAULT:
            self.stop("DFS fault: " + (self.explorer.fault or "unknown"))
            return
        if self.phase == "scan_turn" and now >= self.scan_turn_deadline:
            self.stop("Timed out turning to inspect an exit")
            return
        if now < self.ready_at:
            return
        if self.phase == "arrival_hold":
            return  # Wait stationary for camera confirmation, still checking timeouts.
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
            opened, blocked = sensor.exits(heading, self.threshold_mm)
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
                self.departed = False
                self._reset_arrival()
                self._status("Marker %d: %s %s" % (current,
                             segment.kind.value, segment.direction.name))
            elif action.kind == "SCAN":
                node = self.explorer.nodes[current]
                unseen = 15 & ~node.known_exits & ~attempted
                if unseen:
                    # The three ToFs do not look behind the rover. Inspect an
                    # unobserved rear exit by turning in place, without advancing.
                    scan_direction = next(direction for direction in Direction if unseen & (1 << direction))
                    self.scan_direction = scan_direction
                    self.phase = "scan_turn"
                    self.scan_turn_deadline = now + 15.0
                    self._status("Marker %d: turning to inspect %s" % (current, scan_direction.name))
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
            error = heading_error(self.map_zero_deg + int(direction) * 90, sensor.heading)
            if abs(error) <= self.TURN_TOLERANCE_DEG:
                if self._send("STOP", now, force=True):
                    self.phase = "scan" if scan_turn else "settle"
                    self.ready_at = now + self.STOP_SETTLE_S
            else:
                self._send("ROTATE_RIGHT" if error > 0 else "ROTATE_LEFT", now)
        elif self.phase in ("settle", "drive"):
            if segment is None:
                self.stop("Missing pending movement for forward travel")
                return
            if self.phase == "settle":
                error = heading_error(self.map_zero_deg + int(segment.direction) * 90, sensor.heading)
                if abs(error) > self.TURN_TOLERANCE_DEG:
                    self.phase = "turn"
                    return  # Braking drift must be corrected before driving.
            self.phase = "drive"
            self._send("FORWARD", now)

    def _finish(self, message, now):
        self.active = False
        self.phase = "idle"
        self._send("STOP", now, force=True)
        self._status("DFS " + message)
