"""Where is the rover, in cm, for placing ToF readings in the occupancy grid?

There are no wheel encoders, so the position comes from the two things we do know:
  * stopped: the rover stands at the stand-off point of the marker it last arrived at
             (marker position minus STANDOFF_CM along the direction it travelled to get there);
  * driving: the forward camera sees the destination marker at some range, so the position is the
             destination minus (range - ARRIVAL_RANGE_CM + STANDOFF_CM) along the travel direction.
Sideways offset and the exact camera mounting are not modelled (see config.STANDOFF_CM).
The heading is the compass heading relative to the map north set when the mission started.

PoseFeed hooks Buddy 1's LaptopNavigator from the outside (it wraps two methods on the instance, her
files are not edited) so every telemetry sample is paired with the newest camera observation."""
import math
from typing import Callable, Optional, Tuple

from . import config as cfg
from .config import DIR_VECTORS
from .route_exec import RouteExplorer

Pose = Tuple[float, float, float]


class PoseTracker:
    CAMERA_MAX_AGE_S = 0.5

    def __init__(self, graph_provider: Callable):
        self._graph = graph_provider            # () -> MarkerGraph
        self.cam = None                         # (marker_id, range_mm, bearing_deg, time)
        self._session = None
        self._cur = None
        self._travel_dir = None
        self.arrival_dir = 0

    def observe_camera(self, marker_id, distance_mm, now) -> None:
        self.cam = (marker_id, distance_mm, now)

    def pose(self, nav) -> Optional[Pose]:
        sensor, explorer = nav.sensor, nav.explorer
        if sensor is None or not nav.active or explorer.current is None:
            return None
        if explorer.session_id != self._session:
            self._session, self._cur, self._travel_dir = explorer.session_id, None, None
            if not isinstance(explorer, RouteExplorer):
                self.arrival_dir = 0                # a new mission starts facing map north
        seg = explorer.pending
        if seg is not None:
            self._travel_dir = int(seg.direction)
        if explorer.current != self._cur:
            if self._cur is not None and self._travel_dir is not None:
                self.arrival_dir = self._travel_dir
            self._cur = explorer.current
        if seg is not None and getattr(seg.kind, "value", seg.kind) == "BLOCKED_RETURN":
            return None                         # backing out of a blocked hop: no pose estimate
        g = self._graph()
        pitch = cfg.MARKER_PITCH_CM
        heading = (sensor.heading - nav.map_zero_deg) % 360
        if nav.phase in ("drive", "arrival_hold") and seg is not None:
            cam = self.cam
            src = g.markers.get(seg.from_marker)
            if (cam is None or src is None or cam[0] == seg.from_marker or cam[1] is None
                    or nav.clock() - cam[2] > self.CAMERA_MAX_AGE_S):
                return None
            vx, vy = DIR_VECTORS[int(seg.direction)]
            dest_x, dest_y = (src.pos[0] + vx) * pitch, (src.pos[1] + vy) * pitch
            # floor distance still to go to the stop point, from the camera's slant range (lens height removed)
            dist = (cfg.camera_ground_cm(cam[1]) - cfg.camera_ground_cm(cfg.ARRIVAL_RANGE_CM * 10)
                    + cfg.STANDOFF_CM)
            return dest_x - dist * vx, dest_y - dist * vy, heading
        here = g.markers.get(explorer.current)
        if here is None:
            return None                         # the map has not caught up with a new arrival yet
        vx, vy = DIR_VECTORS[self.arrival_dir]
        return (here.pos[0] * pitch - cfg.STANDOFF_CM * vx,
                here.pos[1] * pitch - cfg.STANDOFF_CM * vy, heading)


class PoseFeed:
    """Pairs every sensor sample with a pose and feeds the occupancy grid."""

    def __init__(self, nav, bridge):
        self.nav, self.bridge = nav, bridge
        self.tracker = PoseTracker(lambda: bridge.ctrl.graph)
        self._last_sensor = None
        self.skipped = 0

    def attach(self) -> None:
        nav, orig_marker, orig_sensor = self.nav, self.nav.marker_seen, self.nav.sensor_received

        def marker_seen(marker_id, frame_id, age_ms=0.0, distance_mm=None, bearing_deg=None):
            ok = orig_marker(marker_id, frame_id, age_ms, distance_mm, bearing_deg)
            if ok and marker_id is not None:
                self.tracker.observe_camera(marker_id, distance_mm, nav.clock())
            return ok

        def sensor_received(data):
            orig_sensor(data)
            s = nav.sensor
            if s is not None and s is not self._last_sensor:
                self._last_sensor = s
                self.on_sample(s)

        nav.marker_seen, nav.sensor_received = marker_seen, sensor_received

    def on_sample(self, sensor) -> None:
        pose = self.tracker.pose(self.nav)
        if pose is None:
            self.skipped += 1
            return
        x, y, heading = pose
        fwd = math.radians(heading)
        # ToF sensors sit TOF_FORWARD_CM ahead of the rover centre (all three on the front)
        origin = (x + math.sin(fwd) * cfg.TOF_FORWARD_CM, y + math.cos(fwd) * cfg.TOF_FORWARD_CM, heading)
        with self.bridge.lock:
            self.bridge.grid.integrate(origin, sensor.distances, sensor.faults)
