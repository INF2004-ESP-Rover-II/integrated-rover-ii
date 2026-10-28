"""Laptop-owned marker DFS. No GUI, MQTT, camera or motor dependencies.

The camera/mission adapter supplies confirmed arrivals, marker types and heading.
The sensor adapter supplies confirmed exits in a fixed N/E/S/W map frame.
An exploration move deliberately has no destination ID until arrival is reported.
"""

from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Dict, List, Optional, Tuple


class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    @property
    def opposite(self):
        return Direction((self + 2) % 4)


class MarkerType(str, Enum):
    PLAIN = "P"
    JUNCTION = "J"


class State(str, Enum):
    IDLE = "IDLE"
    READY = "READY"
    MOVING = "MOVING"
    PAUSED = "PAUSED"
    COMPLETE = "COMPLETE"
    EXHAUSTED = "EXHAUSTED"
    FAULT = "FAULT"


class MoveKind(str, Enum):
    EXPLORE = "EXPLORE"
    BACKTRACK = "BACKTRACK"
    LOOP_RETURN = "LOOP_RETURN"
    BLOCKED_RETURN = "BLOCKED_RETURN"


@dataclass
class Node:
    marker_id: int
    marker_type: MarkerType
    known_exits: int = 0
    open_exits: int = 0
    blocked_exits: int = 0
    traversed_exits: int = 0
    # Direction -> actual marker ID, not an index or an assumed grid coordinate.
    neighbors: Dict[Direction, int] = field(default_factory=dict)


@dataclass(frozen=True)
class Segment:
    session_id: int
    segment_id: int
    from_marker: int
    from_confirmed: bool
    direction: Direction
    kind: MoveKind
    target_marker: Optional[int] = None


@dataclass(frozen=True)
class Action:
    kind: str
    segment: Optional[Segment] = None
    fault: Optional[str] = None


def _marker_id(value):
    return type(value) is int and 1 <= value <= 36


def _bit(direction):
    return 1 << int(direction)


class DFSExplorer:
    """Single-owner event-driven DFS; keep all calls on the dashboard Tk thread.

    Time arguments use monotonic seconds. Polling produces MOVE once, then WAIT
    until a matching arrival/blockage or timeout. This class never drives motors.
    """

    def __init__(self):
        self.state = State.IDLE
        self.nodes: Dict[int, Node] = {}
        self.stack: List[int] = []
        self.current: Optional[int] = None
        self.current_confirmed = False
        self.heading = Direction.NORTH
        self.session_id = 0
        self.target_markers = 0
        self.segment_timeout_s = 10.0
        self.pending: Optional[Segment] = None
        self.issued_at = 0.0
        self.next_segment_id = 1
        self.return_move: Optional[Tuple[MoveKind, Direction, int]] = None
        self.fault: Optional[str] = None

    def start(self, session_id, target_markers, marker_id, marker_type, heading,
              segment_timeout_s=10.0):
        if (type(session_id) is not int or session_id <= 0 or
                type(target_markers) is not int or not 1 <= target_markers <= 36 or
                not _marker_id(marker_id) or not 0 < segment_timeout_s < float("inf")):
            raise ValueError("Require a positive session, 1..36 markers and a finite timeout")
        marker_type, heading = MarkerType(marker_type), Direction(heading)
        self.__init__()
        self.session_id = session_id
        self.target_markers = target_markers
        self.segment_timeout_s = segment_timeout_s
        self.nodes[marker_id] = Node(marker_id, marker_type)
        self.stack = [marker_id]
        self.current = marker_id
        self.current_confirmed = True
        self.heading = heading
        self.state = State.COMPLETE if target_markers == 1 else State.READY

    def _fail(self, reason):
        self.state = State.FAULT
        self.fault = reason

    def _block_exit(self, marker_id, direction):
        node = self.nodes[marker_id]
        bit = _bit(direction)
        node.known_exits |= bit
        node.blocked_exits |= bit
        node.open_exits &= ~bit
        neighbor_id = node.neighbors.get(direction)
        if neighbor_id is not None:
            other = self.nodes[neighbor_id]
            reverse = _bit(direction.opposite)
            other.known_exits |= reverse
            other.blocked_exits |= reverse
            other.open_exits &= ~reverse

    def observe_exits(self, marker_id, open_exits, blocked_exits):
        """Four-bit N/E/S/W masks; bits omitted from both remain unknown."""
        current = self.current
        if (current is None or self.state != State.READY or not self.current_confirmed or
                self.return_move is not None or marker_id != current or
                type(open_exits) is not int or type(blocked_exits) is not int or
                not 0 <= open_exits <= 15 or not 0 <= blocked_exits <= 15 or
                open_exits & blocked_exits):
            return False
        node = self.nodes[current]
        node.known_exits |= open_exits | blocked_exits
        node.open_exits |= open_exits
        for direction in Direction:
            if blocked_exits & _bit(direction):
                self._block_exit(current, direction)
        node.open_exits &= ~node.blocked_exits
        return True

    def tick(self, now):
        if self.state == State.MOVING and now - self.issued_at >= self.segment_timeout_s:
            self._fail("TIMEOUT")

    def _issue(self, kind, direction, target, now):
        current = self.current
        if current is None:
            raise RuntimeError("Cannot issue a DFS move before starting at a marker")
        self.pending = Segment(self.session_id, self.next_segment_id, current,
                               self.current_confirmed, direction, kind, target)
        self.next_segment_id += 1
        self.issued_at = now
        self.current_confirmed = False
        self.state = State.MOVING
        return Action("MOVE", self.pending)

    def next_action(self, now):
        self.tick(now)
        if self.state == State.READY:
            current = self.current
            if current is None:
                raise RuntimeError("Ready DFS must have a current marker")
            if self.return_move is not None:
                kind, direction, target = self.return_move
                return self._issue(kind, direction, target, now)
            node = self.nodes[current]
            allowed = _bit(self.heading) if node.marker_type == MarkerType.PLAIN else 15
            candidates = node.open_exits & allowed & ~node.blocked_exits & ~node.traversed_exits
            for offset in (0, 3, 1, 2):  # front, left, right, back
                direction = Direction((self.heading + offset) % 4)
                if candidates & _bit(direction):
                    return self._issue(MoveKind.EXPLORE, direction, None, now)
            if allowed & ~node.known_exits:
                return Action("SCAN")
            if len(self.stack) == 1:
                self.state = State.EXHAUSTED
            else:
                parent = self.stack[-2]
                for direction, neighbor in node.neighbors.items():
                    bit = _bit(direction)
                    if neighbor == parent and node.open_exits & bit and not node.blocked_exits & bit:
                        return self._issue(MoveKind.BACKTRACK, direction, parent, now)
                self._fail("RETURN_BLOCKED")
        if self.state in (State.COMPLETE, State.EXHAUSTED, State.PAUSED, State.FAULT):
            return Action(self.state.value, fault=self.fault)
        return Action("WAIT")

    def _matches(self, session_id, segment_id, now):
        self.tick(now)
        return (self.state == State.MOVING and self.pending is not None and
                session_id == self.session_id and segment_id == self.pending.segment_id)

    def arrive(self, session_id, segment_id, marker_id, marker_type, heading, now):
        """Report a confirmed stopped arrival, not every visible camera marker."""
        if not _marker_id(marker_id):
            return False
        try:
            marker_type, heading = MarkerType(marker_type), Direction(heading)
        except (ValueError, TypeError):
            return False
        if not self._matches(session_id, segment_id, now):
            return False
        segment = self.pending
        current = self.current
        if segment is None or current is None:
            return False
        if segment.target_marker is not None and marker_id != segment.target_marker:
            self._fail("TOPOLOGY")
            return False
        existing = self.nodes.get(marker_id)
        if existing is not None and existing.marker_type != marker_type:
            self._fail("TOPOLOGY")
            return False
        source = self.nodes[current]
        target = existing or Node(marker_id, marker_type)
        direction, reverse = segment.direction, segment.direction.opposite
        if segment.kind in (MoveKind.EXPLORE, MoveKind.BACKTRACK):
            if (marker_id == current or
                    source.neighbors.get(direction, marker_id) != marker_id or
                    target.neighbors.get(reverse, current) != current):
                self._fail("TOPOLOGY")
                return False
            source.neighbors[direction] = marker_id
            target.neighbors[reverse] = current
            for node, exit_direction in ((source, direction), (target, reverse)):
                bit = _bit(exit_direction)
                node.known_exits |= bit
                node.open_exits |= bit
                node.traversed_exits |= bit
        self.nodes[marker_id] = target
        self.return_move = None
        if segment.kind == MoveKind.EXPLORE:
            if existing is None:
                self.stack.append(marker_id)
            else:
                self.return_move = (MoveKind.LOOP_RETURN, reverse, current)
        elif segment.kind == MoveKind.BACKTRACK:
            self.stack.pop()
        self.current = marker_id
        self.current_confirmed = True
        self.heading = heading
        self.pending = None
        self.state = State.COMPLETE if len(self.nodes) >= self.target_markers else State.READY
        return True

    def segment_blocked(self, session_id, segment_id, now):
        if not self._matches(session_id, segment_id, now):
            return False
        segment = self.pending
        current = self.current
        if segment is None or current is None:
            return False
        if segment.kind != MoveKind.BLOCKED_RETURN:
            self._block_exit(current, segment.direction)
        if segment.kind != MoveKind.EXPLORE:
            self._fail("RETURN_BLOCKED")
            return True
        self.return_move = (MoveKind.BLOCKED_RETURN, segment.direction.opposite, current)
        self.pending = None
        self.state = State.READY
        self.current_confirmed = False
        return True

    def pause(self):
        if self.state not in (State.READY, State.MOVING):
            return False
        self.pending = None
        self.return_move = None
        self.current_confirmed = False
        self.state = State.PAUSED
        return True

    def resume(self, marker_id, heading):
        try:
            heading = Direction(heading)
        except (ValueError, TypeError):
            return False
        if self.state != State.PAUSED or not self.stack or marker_id != self.stack[-1]:
            return False
        self.current = marker_id
        self.heading = heading
        self.current_confirmed = True
        self.state = State.READY
        return True
