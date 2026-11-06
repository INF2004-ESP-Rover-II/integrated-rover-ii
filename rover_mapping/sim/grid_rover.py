"""Continuous-geometry fake rover for the NEW grid-localisation DFS (Start node + TURN_TO).

Unlike RoverPhysics (which hops marker to marker) this rover has a true position inside a World,
drives wherever its heading points until a wall stops it, turns with TURN_TO:<deg> like the
RoboPico, reads ToFs by ray casting (1023 = nothing within 1 m, the Buddy-1 contract) and sees every
marker that is inside the camera's field of view and not hidden behind a wall (so the DFS gets the
look-ahead pair it needs). All lengths in cm, headings in degrees clockwise from map north."""
import math
from typing import Dict, List

from sim.world import World

CAMERA_HEIGHT_CM = 32.5
CAMERA_FORWARD_CM = 12.0
FOV_DEG = float(__import__("os").environ.get("SIM_FOV_DEG", "30"))   # camera half angle
SEE_CM = 150.0
BODY_CM = 9.0                      # stops this far from a wall


def _clear(world: World, x0, y0, x1, y1) -> bool:
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-6:
        return True
    ang = math.degrees(math.atan2(x1 - x0, y1 - y0))
    hit = world.raycast(x0, y0, ang, d)
    return hit is None or hit >= d - 1.0


class GridRover:
    def __init__(self, maze, start_node: int, world: World, start_heading_deg: float = 0.0,
                 drive_cm_s: float = 20.0, rotate_dps: float = 45.0, seed: int = 1):
        self.maze, self.world = maze, world
        self.x, self.y = world.marker_xy(start_node)
        self.heading = float(start_heading_deg) % 360
        self.drive_cm_s, self.rotate_dps = drive_cm_s, rotate_dps
        self.cmd, self.turn_target = "STOP", 0.0
        self.stamp, self.frame = 0, 0
        self.log: List[str] = []

    # ---- commands / motion -------------------------------------------------------------
    def command(self, text: str) -> None:
        self.cmd = text.split(":")[0]
        if self.cmd == "TURN_TO":
            try:
                self.turn_target = float(text.split(":")[1]) % 360
            except (IndexError, ValueError):
                self.cmd = "STOP"

    def _front(self, deg=0.0):
        return self.world.raycast(self.x, self.y, self.heading + deg, 1000.0)

    def step(self, dt: float) -> None:
        if self.cmd == "TURN_TO":
            err = (self.turn_target - self.heading + 540.0) % 360.0 - 180.0
            move = self.rotate_dps * dt
            self.heading = self.turn_target if abs(err) <= move else (self.heading + math.copysign(move, err)) % 360
        elif self.cmd in ("ROTATE_RIGHT", "ROTATE_LEFT"):
            self.heading = (self.heading + (1 if self.cmd == "ROTATE_RIGHT" else -1) * self.rotate_dps * dt) % 360
        elif self.cmd in ("FORWARD", "BACKWARD", "LEFT", "RIGHT"):
            off = {"FORWARD": 0, "BACKWARD": 180, "RIGHT": 90, "LEFT": -90}[self.cmd]
            ang = math.radians(self.heading + off)
            step = self.drive_cm_s * dt
            ahead = self.world.raycast(self.x, self.y, self.heading + off, BODY_CM + step)
            if ahead is None or ahead > BODY_CM + step:
                self.x += step * math.sin(ang)
                self.y += step * math.cos(ang)

    # ---- sensors -----------------------------------------------------------------------
    def telemetry(self) -> dict:
        self.stamp += 1
        dist: Dict[str, int] = {}
        for name, off in (("left", -90), ("front", 0), ("right", 90)):
            d = self._front(off)
            dist[name] = 1023 if d is None else max(1, min(1000, int(round(d * 10))))
        blocked = urgent = 0
        for bit, name in ((1, "left"), (2, "front"), (4, "right")):
            if 1 <= dist[name] <= 1000:
                blocked |= bit if dist[name] <= 400 else 0
                urgent |= bit if dist[name] <= 150 else 0
        return {"timestamp": self.stamp * 100,
                "imu": {"heading": round(self.heading, 1), "pitch": 0.0, "roll": 0.0, "status": 0},
                "obstacle": {"distance_mm": dist, "blocked_mask": blocked, "urgent_mask": urgent,
                             "fault_mask": 0, "stop_required": bool(urgent & 2), "age_ms": 0}}

    def camera_frame(self, wall_ms: float) -> dict:
        self.frame += 1
        a = math.radians(self.heading)
        lx, ly = self.x + CAMERA_FORWARD_CM * math.sin(a), self.y + CAMERA_FORWARD_CM * math.cos(a)
        markers = []
        for node in self.maze.pos:
            mx, my = self.world.marker_xy(node)
            dx, dy = mx - lx, my - ly
            ground = math.hypot(dx, dy)
            if ground < 20.0 or ground > SEE_CM:
                continue
            bearing = (math.degrees(math.atan2(dx, dy)) - self.heading + 540.0) % 360.0 - 180.0
            if abs(bearing) > FOV_DEG or not _clear(self.world, lx, ly, mx, my):
                continue
            rng = math.hypot(ground, CAMERA_HEIGHT_CM) * 10.0
            markers.append({"marker_id": node + 1, "valid": True, "camera_range_mm": round(rng, 1),
                            "camera_bearing_deg": round(bearing, 1)})
        return {"schema": 1, "source_session_id": "sim-camera", "frame_seq": self.frame,
                "capture_time_ms": wall_ms, "capture_time_source": "laptop_receive",
                "status": "ok" if markers else "no_marker", "last_frame_age_ms": 0, "markers": markers}

    def true_node(self):
        best = min(self.maze.pos, key=lambda n: math.hypot(*(a - b for a, b in zip(self.world.marker_xy(n), (self.x, self.y)))))
        return best + 1
