"""Decode B3's schema-1 camera feed"""

import math
from dataclasses import dataclass
from typing import List, Optional, Set, TypeGuard, Union


def finite_number(value: object) -> TypeGuard[Union[int, float]]:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    camera_distance_mm: float
    camera_bearing_deg: float


@dataclass(frozen=True)
class VisionFrame:
    status: str
    age_ms: float
    markers: List[MarkerObservation]
    restarted: bool


class VisionFeed:
    """Sequence numbers are compared within a publisher session, not globally."""

    MAX_AGE_MS = 500.0

    def __init__(self):
        self.session: Optional[str] = None
        self.last_seq = -1
        self.retired_sessions: Set[str] = set()

    def decode(self, data, wall_time_ms) -> Optional[VisionFrame]:
        if not isinstance(data, dict) or type(data.get("schema")) is not int or data["schema"] != 1:
            raise ValueError("Expected camera schema 1")
        session, sequence = data.get("source_session_id"), data.get("frame_seq")
        status, markers = data.get("status"), data.get("markers")
        if (not isinstance(session, str) or not session or type(sequence) is not int or sequence < 0 or
                not isinstance(status, str) or status not in ("ok", "no_marker", "stalled", "disconnected") or
                not isinstance(markers, list)):
            raise ValueError("Invalid camera frame envelope")
        if session in self.retired_sessions:
            return None
        capture, reported_age = data.get("capture_time_ms"), data.get("last_frame_age_ms")
        if (data.get("capture_time_source") != "laptop_receive" or not finite_number(capture) or
                not finite_number(reported_age) or reported_age < 0 or capture > wall_time_ms):
            raise ValueError("Camera timestamps must use the laptop clock")
        # Include time spent in pipes, MQTT and the Tk callback queue.
        age = max(reported_age, wall_time_ms - capture)
        restarted = self.session is not None and session != self.session
        if session != self.session:
            if self.session is not None:
                self.retired_sessions.add(self.session)
            self.session, self.last_seq = session, -1
        # Health packets intentionally repeat the last real frame sequence.
        if status in ("stalled", "disconnected"):
            return VisionFrame(status, age, [], restarted)
        if sequence <= self.last_seq:
            return None
        self.last_seq = sequence
        if age > self.MAX_AGE_MS:
            return VisionFrame("stale", age, [], restarted)
        if status == "no_marker" and markers:
            raise ValueError("no_marker frame must have an empty markers list")
        usable = []
        for marker in markers:
            if not isinstance(marker, dict) or marker.get("valid") is not True:
                continue
            marker_id = marker.get("marker_id")
            if type(marker_id) is not int or not 1 <= marker_id <= 36:
                continue
            # Always use lens-relative measurements, even if the publisher's
            # primary distance is converted to the rover centre by --cam-*.
            distance = marker.get("camera_range_mm")
            bearing = marker.get("camera_bearing_deg")
            if distance is None or bearing is None:
                if marker.get("distance_ref") != "camera_lens":
                    continue
                distance, bearing = marker.get("distance_mm"), marker.get("bearing_deg")
            if (not finite_number(distance) or distance <= 0 or not finite_number(bearing) or
                    not -180 <= bearing <= 180):
                continue
            usable.append(MarkerObservation(marker_id, float(distance), float(bearing)))
        return VisionFrame(status, age, usable, restarted)


GRID_CELL_MM = 600.0
GRID_SIZE = 6
LENS_AHEAD_MM = 120.0  # Camera lens ahead of the rover centre.
AT_WALL_MM = 450.0  # Front ToF this close: the rover has driven up to the end of the line.


def grid_cell(position_mm):
    """Cell (col, row) of a map position in mm, with cell (0, 0) centred on the origin."""
    return round(position_mm[0] / GRID_CELL_MM), round(position_mm[1] / GRID_CELL_MM)


def place_markers(markers: List[MarkerObservation], found, start_cell=(0, 0), heading_deg=0.0, rover=None,
                  front_mm=None):
    """Add first sightings to found {marker_id: (x_mm, y_mm)}; return the new IDs.

    heading_deg is clockwise from the starting IMU heading: 0 faces +row,
    90 faces +col. Rotate the camera offset along with each observation.
    A visible mapped marker anchors the rover's position in the map frame;
    without one the rover stays where it was last anchored (else start_cell).
    Corridor ends: no markers lie past the wall, so once the front ToF
    (front_mm) reads AT_WALL_MM or less the rover is at the last marker
    recorded; when the heading then turns to a new quarter it is placed in
    that marker's cell. Turning before reaching the wall keeps the position.
    rover keeps this state between frames. Signed cells allow observations on
    every side of the start. The dashboard expands its grid to display them.
    """
    if not finite_number(heading_deg):
        raise ValueError("A finite map-relative heading is required")
    rover = {} if rover is None else rover
    rover.setdefault("mm", (start_cell[0] * GRID_CELL_MM, start_cell[1] * GRID_CELL_MM))
    quarter = round(heading_deg % 360 / 90) % 4
    if rover.setdefault("quarter", quarter) != quarter:  # Turned left or right.
        if rover.get("wall") and "last" in rover:
            col, row = grid_cell(rover["last"])
            rover["mm"] = (col * GRID_CELL_MM, row * GRID_CELL_MM)
        rover["quarter"], rover["wall"] = quarter, False
    if finite_number(front_mm) and 0 < front_mm <= AT_WALL_MM:
        rover["wall"] = True
    heading = math.radians(heading_deg % 360)
    cos_heading, sin_heading = math.cos(heading), math.sin(heading)
    seen = {}
    for marker in sorted(markers, key=lambda marker: marker.camera_distance_mm):
        bearing = math.radians(marker.camera_bearing_deg)
        right = marker.camera_distance_mm * math.sin(bearing)
        forward = LENS_AHEAD_MM + marker.camera_distance_mm * math.cos(bearing)
        seen[marker.marker_id] = (right * cos_heading + forward * sin_heading,
                                  forward * cos_heading - right * sin_heading)
    anchor = next((marker_id for marker_id in seen if marker_id in found), None)
    if anchor is None:
        origin = rover["mm"]
    else:
        origin = rover["mm"] = (found[anchor][0] - seen[anchor][0], found[anchor][1] - seen[anchor][1])
    new = []
    for marker_id, (x, y) in seen.items():  # Nearest first, so "last" ends on the farthest new marker.
        position = (origin[0] + x, origin[1] + y)
        if marker_id not in found:
            found[marker_id] = rover["last"] = position
            new.append(marker_id)
    return new
