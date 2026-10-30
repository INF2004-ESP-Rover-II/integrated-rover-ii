"""A small physical model of Rover II in a marker maze, speaking the data formats of Buddy 1 / 3.

It is deliberately simple (topological, no real geometry): enough to drive Buddy 1's actual
LaptopNavigator + DFS and my mapping code end to end without hardware.

  in : command strings as the dashboard sends them (ROTATE_LEFT / ROTATE_RIGHT / FORWARD / STOP)
  out: robot/telemetry JSON  (imu heading, ToF distances, obstacle flags)
       camera/marker  JSON   (schema 1, lens-relative range/bearing)

Conventions copied from her code: compass heading INCREASES with ROTATE_RIGHT; map north is the
heading the rover had when the mission started; markers are seen at 450 mm when standing at one.
Marker IDs are the maze node numbers + 1 (live IDs are 1..36)."""
import math
import random
from typing import Optional

from mapping.config import opposite


class RoverPhysics:
    OPEN_MM, WALL_MM = 800, 250        # ToF reading for an open exit / a wall
    STAND_OFF_MM = 450
    SEE_MM = 1000                      # camera range limit

    def __init__(self, maze, start_node: int, start_heading_deg: float = 0.0,
                 rotate_dps: float = 45.0, drive_mm_s: float = 200.0,
                 world=None, standoff_cm: float = 0.0, tof_noise_mm: float = 0.0, seed: int = 1,
                 no_target_code: int = 1000, dropout: float = 0.0):
        """world=None: stylised ToF (open exit 800 mm, wall 250 mm). With a sim.world.World the ToF
        values come from real ray casts; standoff_cm is how far BEFORE the marker the rover really
        stands when stopped (the real rover stops 45 cm from the marker; 0 = standing on the marker)."""
        self.world, self.standoff_cm = world, standoff_cm
        self.noise = tof_noise_mm
        self.dropout = dropout                 # chance a reading is lost (weak return -> 0 = unknown)
        self.no_target_code = no_target_code   # spec says 1023; 1000 lets the DFS see open corridors
        self.rng = random.Random(seed)
        self.arrival_dir = 0
        self.maze = maze
        self.at = start_node                       # node whose stand-off point the rover is at
        self.heading = float(start_heading_deg) % 360
        self.zero = self.heading                   # map north
        self.rotate_dps, self.drive_mm_s = rotate_dps, drive_mm_s
        self.cmd = "STOP"
        self.drive: Optional[dict] = None          # {"to": node, "range": mm} while between markers
        self.blocked_edges = set()                 # frozenset({a, b}) the rover cannot pass
        self.obstacles = {}                        # frozenset({a, b}) -> progress (mm from the start of the hop) of an obstacle
        self.frame = 0
        self.stamp = 0

    # ---- helpers ---------------------------------------------------------------------
    def axis(self) -> int:
        """Nearest map direction (0=N,1=E,2=S,3=W) of the current heading."""
        return int(((self.heading - self.zero) % 360 + 45) // 90) % 4

    def marker_id(self, node: int) -> int:
        return node + 1

    # ---- physics ---------------------------------------------------------------------
    def command(self, text: str) -> None:
        self.cmd = text.split(":")[0]

    def step(self, dt: float) -> None:
        if self.cmd == "ROTATE_RIGHT":
            self.heading = (self.heading + self.rotate_dps * dt) % 360
        elif self.cmd == "ROTATE_LEFT":
            self.heading = (self.heading - self.rotate_dps * dt) % 360
        elif self.cmd == "FORWARD":
            if self.drive is None:
                nxt = self.maze.neighbour(self.at, self.axis())
                if nxt is not None and frozenset((self.at, nxt)) not in self.blocked_edges:
                    self.drive = {"to": nxt, "range": self.SEE_MM + 50.0, "dir": self.axis(), "from": self.at}
            if self.drive is not None:
                d = self.drive
                if self.axis() == d["dir"]:                       # driving on towards the target marker
                    d["range"] -= self.drive_mm_s * dt
                    limit = self._obstacle_progress()
                    if limit is not None:                          # an obstacle stops it ~10 cm short
                        d["range"] = max(d["range"], self.SEE_MM + 50.0 - (limit - 100.0))
                elif self.axis() == opposite(d["dir"]):          # backing out: turned round, driving to the source
                    d["range"] = min(d["range"] + self.drive_mm_s * dt, self.SEE_MM + 50.0)
        # stopped at (or inside) the stand-off distance: we are now at the target marker
        if self.drive is not None and self.cmd == "STOP":
            d = self.drive
            if d["range"] <= self.STAND_OFF_MM + 1e-6 and self.axis() == d["dir"]:
                self.at, self.arrival_dir, self.drive = d["to"], d["dir"], None
            elif self._progress() <= 30.0 and self.axis() == opposite(d["dir"]):
                self.at, self.drive = d["from"], None             # back at the source marker

    def _progress(self) -> float:
        """How far along the current hop (mm from where it started)."""
        return self.SEE_MM + 50.0 - self.drive["range"]

    def _obstacle_progress(self):
        if self.drive is None:
            return None
        return self.obstacles.get(frozenset((self.drive["from"], self.drive["to"])))

    # ---- true position (only used with a World) -------------------------------------
    @staticmethod
    def _vec(axis: int):
        return ((0, 1), (1, 0), (0, -1), (-1, 0))[axis % 4]

    def xy_cm(self):
        """True rover position in cm (map frame: x east, y north)."""
        if self.drive is not None:
            tx, ty = self.world.marker_xy(self.drive["to"])
            vx, vy = self._vec(self.drive["dir"])
            # the camera range reads 450 mm when the rover has stopped at its stand-off point
            r = self.drive["range"] / 10 - self.STAND_OFF_MM / 10 + self.standoff_cm
            return tx - r * vx, ty - r * vy
        mx, my = self.world.marker_xy(self.at)
        vx, vy = self._vec(self.arrival_dir)
        return mx - self.standoff_cm * vx, my - self.standoff_cm * vy

    def _geometric_tof(self) -> dict:
        x, y = self.xy_cm()
        out = {}
        for name, off in (("left", -90), ("front", 0), ("right", 90)):
            d = self.world.raycast(x, y, (self.heading - self.zero) + off, 100.0)
            if self.dropout and self.rng.random() < self.dropout:
                out[name] = 0
                continue
            if d is None:
                out[name] = self.no_target_code        # nothing within 1 m
            else:
                mm = d * 10 + (self.rng.gauss(0, self.noise) if self.noise else 0)
                out[name] = max(1, min(1000, int(round(mm))))
        return out

    # ---- sensors ---------------------------------------------------------------------
    def telemetry(self) -> dict:
        self.stamp += 1
        a = self.axis()
        dist = {}
        if self.world is not None:
            dist = self._geometric_tof()
        else:
            for name, d in (("left", (a - 1) % 4), ("front", a), ("right", (a + 1) % 4)):
                dist[name] = self.OPEN_MM if self.maze.neighbour(self.at, d) is not None else self.WALL_MM
        blocked = urgent = 0
        limit = self._obstacle_progress()
        if limit is not None and self.axis() == self.drive["dir"]:     # obstacle straight ahead
            front = max(1, int(limit - self._progress()))
            dist["front"] = front
            blocked = 2 if front <= 400 else 0
            urgent = 2 if front <= 150 else 0
        return {"timestamp": self.stamp,
                "imu": {"heading": round(self.heading, 1), "pitch": 77.0, "status": 0},
                "obstacle": {"distance_mm": dist, "blocked_mask": blocked, "urgent_mask": urgent,
                             "fault_mask": 0, "stop_required": bool(urgent), "age_ms": 0}}

    def camera_frame(self, wall_ms: float) -> dict:
        """B3 schema-1 envelope for the current instant."""
        self.frame += 1
        markers = []
        if self.drive is not None:
            if self.axis() == opposite(self.drive["dir"]):       # facing back: the source marker is ahead
                if 450.0 + self._progress() <= self.SEE_MM:
                    markers.append((self.marker_id(self.drive["from"]), 450.0 + self._progress()))
            elif self.drive["range"] <= self.SEE_MM:
                markers.append((self.marker_id(self.drive["to"]), max(self.drive["range"], 1.0)))
        else:
            markers.append((self.marker_id(self.at), float(self.STAND_OFF_MM)))
        return {"schema": 1, "source_session_id": "sim-camera", "frame_seq": self.frame,
                "capture_time_ms": wall_ms, "capture_time_source": "laptop_receive",
                "status": "ok" if markers else "no_marker", "last_frame_age_ms": 0,
                "markers": [{"marker_id": m, "valid": True, "camera_range_mm": r,
                             "camera_bearing_deg": 0.0} for m, r in markers]}
