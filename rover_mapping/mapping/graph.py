"""Marker graph (FR-24, first map). Nodes are markers; edges are travelled connections.
Marker grid positions are inferred from the heading of each hop: pos = previous + step."""
from typing import Dict, List, Optional, Set, Tuple

from .config import DIR_VECTORS, opposite
from .events import ExitStatus


class Marker:
    def __init__(self, marker_id: int, pos: Tuple[int, int]):
        self.id = marker_id
        self.pos = pos
        self.exits: List[ExitStatus] = [ExitStatus.UNCHECKED] * 4


class MarkerGraph:
    def __init__(self):
        self.markers: Dict[int, Marker] = {}
        self.neighbours: Dict[int, Dict[int, int]] = {}   # id -> {direction: other id}
        self.blocked: Set[frozenset] = set()              # blocked edges
        self.start_id: Optional[int] = None
        self.current: Optional[int] = None
        self.inconsistencies: List[str] = []

    # ---- construction ------------------------------------------------------------
    def start(self, marker_id: int) -> None:
        self.markers[marker_id] = Marker(marker_id, (0, 0))
        self.neighbours[marker_id] = {}
        self.start_id = marker_id
        self.current = marker_id

    def marker_reached(self, marker_id: int, heading: int) -> bool:
        """Rover travelled one hop from the current marker in grid direction `heading`
        and arrived at `marker_id`. Returns True if the marker is new."""
        if self.current is None:
            raise RuntimeError("graph not started")
        cur = self.markers[self.current]
        dx, dy = DIR_VECTORS[heading]
        pos = (cur.pos[0] + dx, cur.pos[1] + dy)
        is_new = marker_id not in self.markers
        if is_new:
            clash = self._marker_at(pos)
            if clash is not None:
                self.inconsistencies.append(
                    f"marker {marker_id} inferred at {pos} but marker {clash} is already there")
                return False
            self.markers[marker_id] = Marker(marker_id, pos)
            self.neighbours[marker_id] = {}
        elif self.markers[marker_id].pos != pos:
            self.inconsistencies.append(
                f"marker {marker_id} known at {self.markers[marker_id].pos}, "
                f"but hop from {self.current} heading {heading} implies {pos}")
            return False
        self._link(self.current, heading, marker_id)
        self.current = marker_id
        return is_new

    def _marker_at(self, pos) -> Optional[int]:
        for m in self.markers.values():
            if m.pos == pos:
                return m.id
        return None

    def _link(self, a: int, d: int, b: int) -> None:
        self.neighbours[a][d] = b
        self.neighbours[b][opposite(d)] = a
        self.markers[a].exits[d] = ExitStatus.EXPLORED
        self.markers[b].exits[opposite(d)] = ExitStatus.EXPLORED

    # ---- exit statuses -----------------------------------------------------------
    def set_exit_status(self, marker_id: int, d: int, status: ExitStatus) -> None:
        """Never downgrade EXPLORED (a travelled exit stays explored)."""
        m = self.markers.get(marker_id)
        if m is None:
            return
        if m.exits[d] == ExitStatus.EXPLORED and status != ExitStatus.BLOCKED:
            return
        m.exits[d] = status

    def apply_exit_report(self, marker_id: int, exits) -> None:
        for d, status in enumerate(exits):
            self.set_exit_status(marker_id, d, status)

    def mark_blocked(self, marker_id: int, d: int) -> None:
        """Phase 1: a wall/obstacle exit. Phase 2: a travelled (or probable) edge is now blocked."""
        m = self.markers.get(marker_id)
        if m is None:
            return
        m.exits[d] = ExitStatus.BLOCKED
        other = self.neighbour_at(marker_id, d)
        if other is not None:
            self.blocked.add(frozenset((marker_id, other)))
            self.markers[other].exits[opposite(d)] = ExitStatus.BLOCKED

    # ---- queries -----------------------------------------------------------------
    def count(self) -> int:
        """Distinct markers found (FR-25)."""
        return len(self.markers)

    def unexplored_exits(self) -> List[Tuple[int, int]]:
        return [(m.id, d) for m in self.markers.values()
                for d, s in enumerate(m.exits) if s == ExitStatus.UNEXPLORED]

    def has_unexplored(self) -> bool:
        return bool(self.unexplored_exits())

    def usable_neighbours(self, marker_id: int, use_probable: bool = False) -> List[Tuple[int, int]]:
        """(direction, neighbour id) over explored, non-blocked edges, sorted by id.
        With use_probable the probable (never driven) edges are included too."""
        out = [(d, n) for d, n in self.neighbours.get(marker_id, {}).items()
               if frozenset((marker_id, n)) not in self.blocked]
        if use_probable:
            out += [(d, n) for d, n, _ in self.probable_from(marker_id)]
        return sorted(out, key=lambda t: t[1])

    # ---- probable links ----------------------------------------------------------
    # Two markers one pitch apart on the grid that were never driven between are *probably*
    # connected: in this arena every marker is a junction on a regular grid. This is only a guess
    # (a wall may sit between them), so it is drawn dashed, the planner charges extra for it, and
    # a wall report (ExitBlocked) removes it. A real drive over it turns it into an explored link.
    def neighbour_at(self, marker_id: int, d: int) -> Optional[int]:
        """Explored neighbour in direction d, else the marker one pitch away on the grid."""
        n = self.neighbours.get(marker_id, {}).get(d)
        if n is not None:
            return n
        m = self.markers.get(marker_id)
        if m is None:
            return None
        dx, dy = DIR_VECTORS[d]
        return self._marker_at((m.pos[0] + dx, m.pos[1] + dy))

    def is_probable(self, a: int, d: int) -> Optional[bool]:
        """None = not probable; False = possible (exit unknown); True = likely (ToF saw it open)."""
        if d in self.neighbours.get(a, {}):
            return None
        b = self.neighbour_at(a, d)
        if b is None or frozenset((a, b)) in self.blocked:
            return None
        sa, sb = self.markers[a].exits[d], self.markers[b].exits[opposite(d)]
        if ExitStatus.BLOCKED in (sa, sb):
            return None
        return ExitStatus.UNEXPLORED in (sa, sb)

    def probable_from(self, marker_id: int) -> List[Tuple[int, int, bool]]:
        out = []
        for d in range(4):
            flag = self.is_probable(marker_id, d)
            if flag is not None:
                out.append((d, self.neighbour_at(marker_id, d), flag))
        return out

    def probable_links(self) -> List[Tuple[int, int, bool]]:
        seen, out = set(), []
        for a in sorted(self.markers):
            for d, b, strong in self.probable_from(a):
                key = frozenset((a, b))
                if key not in seen:
                    seen.add(key)
                    out.append((min(a, b), max(a, b), strong))
        return out

    def direction_between(self, a: int, b: int) -> Optional[int]:
        for d, n in self.neighbours.get(a, {}).items():
            if n == b:
                return d
        if a in self.markers and b in self.markers:        # probable edge: use the grid geometry
            pa, pb = self.markers[a].pos, self.markers[b].pos
            for d, (dx, dy) in DIR_VECTORS.items():
                if (pa[0] + dx, pa[1] + dy) == pb:
                    return d
        return None

    def link(self, a: int, d: int, b: int) -> None:
        """Record a driven connection (used when a probable edge has been travelled)."""
        self._link(a, d, b)

    def to_ascii(self) -> str:
        if not self.markers:
            return "(empty)"
        xs = [m.pos[0] for m in self.markers.values()]
        ys = [m.pos[1] for m in self.markers.values()]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        w, h = (x1 - x0) * 4 + 2, (y1 - y0) * 2 + 1
        canvas = [[" "] * w for _ in range(h)]
        for m in self.markers.values():
            cx, cy = (m.pos[0] - x0) * 4, (y1 - m.pos[1]) * 2
            for i, ch in enumerate(f"{m.id:02d}"):
                canvas[cy][cx + i] = ch
            for d, n in self.neighbours[m.id].items():
                if frozenset((m.id, n)) in self.blocked:
                    continue
                if d == 1:
                    canvas[cy][cx + 2] = canvas[cy][cx + 3] = "-"
                if d == 2:
                    canvas[cy + 1][cx] = "|"
        return "\n".join("".join(r).rstrip() for r in canvas)
