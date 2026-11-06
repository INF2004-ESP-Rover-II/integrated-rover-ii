"""Receiver-only model. No localisation, graph construction or path decisions."""
import math
import time


MAX_NODES = 64
MAX_MARKER_ID = 255


class OnboardState:
    def __init__(self):
        self.snapshot = None
        self.received_at = None

    def update(self, value, now=None):
        if not isinstance(value, dict) or value.get("schema") != 1:
            raise ValueError("Unsupported onboard map schema")
        capacity = value.get("capacity", MAX_NODES)
        id_max = value.get("id_max", MAX_MARKER_ID)
        if type(capacity) is not int or not 1 <= capacity <= 255 or type(id_max) is not int or not 1 <= id_max <= MAX_MARKER_ID:
            raise ValueError("Invalid map capacity/ID limit")
        markers = value.get("markers")
        if not isinstance(markers, list) or len(markers) > capacity:
            raise ValueError("Invalid marker list")
        if value.get("state") not in {"IDLE", "READY", "SCAN", "TURN", "SETTLE", "ACQUIRE", "DRIVE", "CONFIRM", "HOLD", "RECOVERY_TURN", "RECOVERY_SETTLE", "COMPLETE", "PAUSED", "FAULT"}:
            raise ValueError("Invalid navigation state")
        if type(value.get("count")) is not int or not 0 <= value["count"] <= capacity:
            raise ValueError("Invalid visited count")
        ids, cells = set(), set()
        for marker in markers:
            if (not isinstance(marker, dict) or type(marker.get("id")) is not int or
                    not 1 <= marker["id"] <= id_max or marker["id"] in ids or
                    type(marker.get("x")) is not int or type(marker.get("y")) is not int or
                    abs(marker["x"]) >= capacity or abs(marker["y"]) >= capacity or
                    (marker["x"], marker["y"]) in cells or type(marker.get("visited")) is not bool):
                raise ValueError("Invalid marker/cell association")
            exits = marker.get("exits")
            if not isinstance(exits, list) or len(exits) != 4 or any(type(x) is not int or not 0 <= x <= 3 for x in exits):
                raise ValueError("Invalid exit states")
            ids.add(marker["id"]); cells.add((marker["x"], marker["y"]))
        if value["count"] != sum(m["visited"] for m in markers):
            raise ValueError("Visited count disagrees with cells")
        if value.get("current", 0) and value["current"] not in ids:
            raise ValueError("Current node missing from map")
        if "n" in value and (type(value["n"]) is not int or not 0 <= value["n"] <= capacity):
            raise ValueError("Invalid marker target")
        bounds = value.get("bounds")
        if bounds is not None:
            keys = ("min_x", "max_x", "min_y", "max_y", "rows", "columns")
            if not isinstance(bounds, dict) or any(type(bounds.get(k)) is not int for k in keys):
                raise ValueError("Invalid map bounds")
            if markers:
                expected = dict(min_x=min(m["x"] for m in markers), max_x=max(m["x"] for m in markers),
                                min_y=min(m["y"] for m in markers), max_y=max(m["y"] for m in markers))
                expected.update(rows=expected["max_y"]-expected["min_y"]+1,
                                columns=expected["max_x"]-expected["min_x"]+1)
            else:
                expected = dict.fromkeys(keys, 0)
            if any(bounds[k] != expected[k] for k in keys):
                raise ValueError("Map bounds disagree with nodes")
        pitch = value.get("pitch_mm", 600)
        if isinstance(pitch, bool) or not isinstance(pitch, (int, float)) or not math.isfinite(pitch) or pitch <= 0:
            raise ValueError("Invalid grid pitch")
        pose = value.get("pose", {})
        for key in ("x_mm", "y_mm"):
            number = pose.get(key)
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
                raise ValueError("Invalid pose")
        if "geometry_ready" in value and type(value["geometry_ready"]) is not bool:
            raise ValueError("Invalid geometry readiness")
        camera = value.get("camera")
        if camera is not None:
            if (not isinstance(camera, dict) or type(camera.get("received")) is not bool or
                type(camera.get("status")) is not int or not 0 <= camera["status"] <= 3 or
                type(camera.get("sequence")) is not int or not 0 <= camera["sequence"] <= 255 or
                any(type(camera.get(k)) is not int or not 0 <= camera[k] <= 0xFFFFFFFF
                    for k in ("reported_age_ms", "received_age_ms", "effective_age_ms")) or
                not isinstance(camera.get("markers"), list) or len(camera["markers"]) > 4):
                raise ValueError("Invalid camera diagnostic")
            for m in camera["markers"]:
                if (not isinstance(m, dict) or any(type(m.get(k)) is not int for k in ("id", "range_mm", "bearing_cdeg", "quality")) or
                    not 1 <= m["id"] <= id_max or not 0 <= m["range_mm"] <= 65535 or
                    not -18000 <= m["bearing_cdeg"] <= 18000 or not 0 <= m["quality"] <= 255):
                    raise ValueError("Invalid camera measurement")
        # One complete rover snapshot replaces the cache; no local inference.
        self.snapshot = value
        self.received_at = time.monotonic() if now is None else now

    def stale(self, now=None):
        now = time.monotonic() if now is None else now
        return self.received_at is None or now - self.received_at > 3.0


def mission_command(action, marker=None, count=36, opening_mm=500, capacity=MAX_NODES, id_max=MAX_MARKER_ID):
    if type(capacity) is not int or not 1 <= capacity <= 255 or type(id_max) is not int or not 1 <= id_max <= MAX_MARKER_ID:
        raise ValueError("Invalid rover limits")
    if action == "start_node":
        if type(marker) is not int or not 1 <= marker <= id_max:
            raise ValueError(f"Start node must be 1..{id_max}")
        return f"SET_START:{marker}"
    if action == "start":
        if type(count) is not int or not 1 <= count <= capacity or type(opening_mm) is not int or not 200 <= opening_mm <= 1000:
            raise ValueError(f"Require 1..{capacity} markers and opening threshold 200..1000 mm")
        return f"DFS_START:{count}:{opening_mm}"
    return {"stop": "STOP", "estop": "ESTOP", "resume": "DFS_RESUME", "reset_fault": "DFS_RESET",
            "reset_map": "RESET_MAP", "snapshot": "MAP_REQUEST"}[action]


def geometry_command(height_mm, forward_mm, down_deg):
    values = (height_mm, forward_mm, down_deg)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError("Enter finite measured camera geometry values")
    if not (0 < height_mm <= 2000 and -1000 <= forward_mm <= 1000 and -45 <= down_deg <= 75):
        raise ValueError("Height must be 0..2000 mm, offset +/-1000 mm, tilt -45..75 degrees")
    return f"CAMERA_GEOMETRY:{height_mm:.3f}:{forward_mm:.3f}:{down_deg:.3f}"
