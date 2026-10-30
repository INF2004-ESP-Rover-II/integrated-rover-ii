"""Internal event and message objects. The wire codec (binary) will map to and from these;
the logic never sees bytes. Names follow Section 6.4."""
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

from .config import opposite


class ExitStatus(IntEnum):
    UNCHECKED = 0    # not scanned yet
    UNEXPLORED = 1   # open, not travelled
    EXPLORED = 2     # travelled, leads to a known marker
    BLOCKED = 3      # wall / far-limit / EXIT_BLOCKED


# ---- rover -> laptop (rover/event, rover/marker) -------------------------------------
@dataclass(frozen=True)
class MarkerReached:
    marker_id: int
    heading: int                 # grid direction of travel on arrival (0=N,1=E,2=S,3=W)
    seq: Optional[int] = None


@dataclass(frozen=True)
class ExitExplored:
    marker_id: int
    direction: int               # absolute grid direction
    seq: Optional[int] = None


@dataclass(frozen=True)
class ExitBlocked:
    marker_id: int
    direction: int
    seq: Optional[int] = None        # event sequence (Phase 1 de-duplication)
    ref_seq: Optional[int] = None    # Phase 2: sequence of the segment that failed


@dataclass(frozen=True)
class Backtrack:
    marker_id: int                   # marker the rover is heading back to
    seq: Optional[int] = None


@dataclass(frozen=True)
class ExitReport:
    """All four exit statuses of one junction in one event (agreed at Zoom)."""
    marker_id: int
    exits: Tuple[ExitStatus, ExitStatus, ExitStatus, ExitStatus]   # index = absolute direction
    seq: Optional[int] = None


@dataclass(frozen=True)
class ExploreDoneLocal:
    seq: Optional[int] = None


@dataclass(frozen=True)
class SegmentDone:
    ref_seq: int


@dataclass(frozen=True)
class MarkerSeen:
    """Vision confirmed the rover's current marker (rover/marker)."""
    marker_id: int


# ---- laptop -> rover / Mission -------------------------------------------------------
@dataclass(frozen=True)
class Segment:
    seq: int
    target_marker: int
    heading: int                 # absolute grid direction to drive
    last: bool


@dataclass(frozen=True)
class RouteResult:
    result: str                  # "REACHED" or "NO_ROUTE"
    target: int


@dataclass(frozen=True)
class CountReached:
    n: int


@dataclass(frozen=True)
class MapExhausted:
    count: int
    n: int


def exits_from_relative(heading: int, front: ExitStatus, left: ExitStatus,
                        right: ExitStatus, back: ExitStatus = ExitStatus.EXPLORED):
    """Convert exits reported relative to the rover to absolute grid directions.
    Use this if Buddy 1 sends front/left/right instead of N/E/S/W."""
    out = [ExitStatus.UNCHECKED] * 4
    out[heading % 4] = front
    out[(heading - 1) % 4] = left
    out[(heading + 1) % 4] = right
    out[opposite(heading)] = back
    return tuple(out)
