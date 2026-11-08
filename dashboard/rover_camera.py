"""Display-only cache for RoboPico's UART camera relay, separate from laptop DFS."""
from dataclasses import dataclass


QUALITY_NAMES = ("OK", "POSE_FAILED", "BAD_SHAPE", "TOO_SMALL", "TOO_FAR")
CAMERA_MAX_AGE_MS = 1000
RELAY_TIMEOUT_S = 1.5


def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("Invalid camera integer")
    return value


@dataclass(frozen=True)
class CameraMarker:
    id: int
    range_mm: int
    bearing_cdeg: int
    quality: int


class RoverCameraCache:
    def __init__(self):
        self.seen = False
        self.clear()

    def clear(self):
        # Once selected, never silently fall back to old laptop-camera data.
        self.snapshot = None

    def update(self, data, now):
        try:
            if not isinstance(data, dict) or type(data.get("schema")) is not int or data["schema"] != 1:
                return False
            if data.get("source") != "robo_pico_uart":
                return False
            status = data["status"]
            if not isinstance(status, str) or status not in {"OK", "NO_MARKER", "STALLED", "CAMERA_LOST", "UART_TIMEOUT"}:
                return False
            seq = integer(data["seq"], 0, 255)
            age = integer(data["age_ms"], 0, 0xffffffff)
            uart_age = integer(data["uart_age_ms"], 0, 0xffffffff)
            if age < uart_age or not isinstance(data["markers"], list) or len(data["markers"]) > 4:
                return False
            if (status == "OK") != bool(data["markers"]):
                return False
            markers = tuple(CameraMarker(integer(item["id"], 1, 255),
                integer(item["range_mm"], 0, 65535), integer(item["bearing_cdeg"], -32768, 32767),
                integer(item["quality"], 0, 4)) for item in data["markers"])
        except (KeyError, TypeError, ValueError):
            return False
        self.snapshot = status, seq, age, uart_age, markers, now
        self.seen = True
        return True

    def markers_in_view(self, now):
        """All entries in the latest fresh OK frame, including rejected poses."""
        if self.snapshot is None:
            return ()
        status, _, age, uart_age, markers, received_at = self.snapshot
        elapsed = max(0.0, now - received_at)
        if (status != "OK" or elapsed > RELAY_TIMEOUT_S or
                age + elapsed * 1000 > CAMERA_MAX_AGE_MS or
                uart_age + elapsed * 1000 > CAMERA_MAX_AGE_MS):
            return ()
        return markers

    def display(self, now):
        """Return selected marker, health text, detail text and colour."""
        if self.snapshot is None:
            return None, "Waiting for RoboPico camera observations", "RoboPico camera: waiting", '#95a5a6'
        status, seq, age, uart_age, markers, received_at = self.snapshot
        elapsed = max(0.0, now - received_at)
        age += elapsed * 1000
        uart_age += elapsed * 1000
        details = "RoboPico UART camera: %s | seq=%d | age=%.0f ms | markers=%d" % (status, seq, age, len(markers))
        if elapsed > RELAY_TIMEOUT_S:
            return None, "RoboPico camera relay reports stopped", details, '#f39c12'
        if status != "OK":
            colour = '#95a5a6' if status == "NO_MARKER" else '#e74c3c'
            return None, "RoboPico camera: " + status, details, colour
        if not self.markers_in_view(now):
            return None, "RoboPico camera observation is stale", details, '#f39c12'
        usable = [marker for marker in markers if marker.quality == 0 and marker.range_mm > 0]
        marker = min(usable, key=lambda item: (abs(item.bearing_cdeg), item.range_mm, item.id)) if usable else None
        message = "RoboPico camera observation (display only; arrival not confirmed)" if marker else "RoboPico camera: no usable marker pose"
        return marker, message, details, '#3498db' if marker else '#f39c12'
