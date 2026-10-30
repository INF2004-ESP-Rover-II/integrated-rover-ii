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
