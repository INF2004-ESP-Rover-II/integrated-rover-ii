"""Provisional constants. Everything here is a decision or assumption, not a measurement.
Change values here only; do not hard-code them elsewhere."""

# Arena and grids (team decisions, 21 Oct)
ARENA_CM = (240, 360)          # width x length
MARKER_PITCH_CM = 60           # distance between adjacent markers (marker grid cell)
OCC_CELL_CM = 5                # occupancy grid cell (not used by routing)
TOF_MAX_CM = 100               # provisional cap (sensor reads about 200 cm)

# Wire-format related (provisional, see Version B notes)
MARKER_ID_BITS = 6             # live IDs 1-36 (6x6 arena), field holds 0-63
SEQ_BITS = 8                   # sequence number width for rover/event and rover/segment
JUNCTION_IDS = range(1, 37)    # agreed live set: IDs 1..36, ALL are junction/decision nodes
PLAIN_IDS = range(0)           # none in the live maze

# Grid convention: start marker is (0, 0); heading 0 deg = +y, 90 deg = +x.
N, E, S, W = 0, 1, 2, 3
DIR_VECTORS = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}
DIR_NAMES = {N: "N", E: "E", S: "S", W: "W"}


def opposite(d: int) -> int:
    return (d + 2) % 4


def turn_steps(a: int, b: int) -> int:
    """Number of 90-degree turns needed to change heading a -> b (0, 1 or 2)."""
    diff = (a - b) % 4
    return min(diff, 4 - diff)

# ---- Occupancy grid and rover pose (see mapping/occupancy.py, mapping/pose.py) ----------------
ARRIVAL_RANGE_CM = 45.0      # camera lens -> marker centre when the rover has stopped (Buddy 1: 450 mm)
import math as _math
import os as _os

# ---- Rover geometry (centre of the rover = the reference point; all PROVISIONAL, measure them) ----
# Buddy 1, 29 Oct: camera height about 30-35 cm, camera about 12 cm ahead of the rover centre, ToF sensors
# on the front of the rover at 0 / 90 / 270 deg. Camera tilt not measured (see camera_ground_cm).
CAMERA_HEIGHT_CM = float(_os.environ.get("MAPPING_CAMERA_HEIGHT_CM", 32.5))   # lens height above the floor (env: simulations)
CAMERA_FORWARD_CM = 12.0     # lens is this far ahead of the rover centre
TOF_FORWARD_CM = 0.0         # ToF sensors this far ahead of the rover centre. UNMEASURED, 0 = at the centre.


def camera_ground_cm(range_mm: float) -> float:
    """Floor distance from the point under the lens to a floor marker, from the camera's range.
    ASSUMES `range_mm` is the straight-line lens-to-marker distance (it then does not depend on the camera
    tilt). If Buddy 3's range is the depth along the optical axis instead, the tilt matters and this is wrong."""
    r = range_mm / 10.0
    return _math.sqrt(max(r * r - CAMERA_HEIGHT_CM ** 2, 0.0))


# Rover centre -> marker centre when it has stopped. From the geometry above (450 mm lens range, 32.5 cm up,
# camera 12 cm ahead of the centre) this is about 43 cm, which matches the "about 45 cm" seen on the rover.
# Replace with a MEASURED value; the occupancy map shifts by exactly that amount. The arrival rule itself
# (450 mm) is being replaced by Buddy 1, see MAPPING_INTEGRATION.md. Env override is for simulations.
STANDOFF_CM = float(_os.environ.get("MAPPING_STANDOFF_CM", camera_ground_cm(ARRIVAL_RANGE_CM * 10) + CAMERA_FORWARD_CM))
