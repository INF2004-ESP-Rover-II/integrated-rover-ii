"""Fake rover for tests: DFS exploration (Phase 1) and segment execution (Phase 2).

It only knows what a real rover would know: its own visited set, and which exits are open
(ToF) or walled. It never reads the laptop's map."""
from typing import Iterator, List

from mapping import events as ev
from mapping.config import opposite
from mapping.events import ExitStatus
from mapping.seqnum import next_seq
from mapping.controller import MappingController, State, PHASE2
from sim.maze import Maze


class SimRover:
    def __init__(self, maze: Maze, start_id: int, duplicate_events: bool = False):
        self.maze = maze
        self.start_id = start_id
        self.cur = start_id
        self.heading = 0
        self.duplicate_events = duplicate_events
        self._seq = 0

    def _stamp(self, event):
        """Give a Phase 1 event the next sequence number."""
        self._seq = next_seq(self._seq)
        return type(event)(**{**event.__dict__, "seq": self._seq})

    def _statuses(self, node: int, tried: set, came_from):
        out = []
        for d in range(4):
            if d == came_from or d in tried:
                out.append(ExitStatus.EXPLORED)
            elif self.maze.neighbour(node, d) is not None:
                out.append(ExitStatus.UNEXPLORED)
            else:
                out.append(ExitStatus.BLOCKED)
        return tuple(out)

    def explore(self) -> Iterator:
        """Depth-first exploration. Exits tried in the order front, left, right, back."""
        maze = self.maze
        visited = {self.cur}
        tried = {self.cur: set()}
        stack = []
        came_from = {self.cur: None}
        yield self._stamp(ev.ExitReport(self.cur, self._statuses(self.cur, tried[self.cur], None)))
        while True:
            cur = self.cur
            order = [self.heading, (self.heading - 1) % 4, (self.heading + 1) % 4,
                     opposite(self.heading)]
            choice = next((d for d in order if d not in tried[cur]
                           and d != came_from[cur] and maze.neighbour(cur, d) is not None), None)
            if choice is not None:
                nxt = maze.neighbour(cur, choice)
                tried[cur].add(choice)
                self.heading = choice
                yield self._stamp(ev.ExitExplored(cur, choice))
                self.cur = nxt                        # the rover is physically at nxt now
                yield self._stamp(ev.MarkerReached(nxt, choice))
                if nxt in visited:
                    # loop edge: travel back along it
                    tried[nxt].add(opposite(choice))
                    yield self._stamp(ev.Backtrack(cur))
                    self.heading = opposite(choice)
                    self.cur = cur
                    yield self._stamp(ev.MarkerReached(cur, self.heading))
                    continue
                visited.add(nxt)
                tried[nxt] = {opposite(choice)}
                came_from[nxt] = opposite(choice)
                stack.append((cur, choice))
                yield self._stamp(ev.ExitReport(nxt, self._statuses(nxt, tried[nxt], came_from[nxt])))
            else:
                if not stack:
                    yield self._stamp(ev.ExploreDoneLocal())
                    return
                prev, travelled = stack.pop()
                yield self._stamp(ev.Backtrack(prev))
                self.heading = opposite(travelled)
                self.cur = prev
                yield self._stamp(ev.MarkerReached(prev, self.heading))

    def execute(self, seg: ev.Segment):
        """Drive one segment; return SegmentDone or ExitBlocked (obstacle on the edge)."""
        nxt = self.maze.neighbour(self.cur, seg.heading)
        if nxt is None:                       # a wall: the rover stops at the junction and reports it
            return ev.ExitBlocked(self.cur, seg.heading, ref_seq=seg.seq)
        if nxt != seg.target_marker:
            raise AssertionError(f"segment {seg} does not match the maze from {self.cur}")
        if self.maze.is_obstructed(self.cur, nxt):
            return ev.ExitBlocked(self.cur, seg.heading, ref_seq=seg.seq)
        self.cur, self.heading = nxt, seg.heading
        return ev.SegmentDone(seg.seq)


def run_phase1(controller: MappingController, rover: SimRover, limit: int = 10000) -> None:
    for i, event in enumerate(rover.explore()):
        controller.on_rover_event(event)
        if rover.duplicate_events:
            controller.on_rover_event(event)      # same message delivered twice
        if controller.state != State.MAPPING or i > limit:
            return


def run_phase2(controller: MappingController, rover: SimRover, limit: int = 1000) -> List:
    """Run until the controller leaves the Phase 2 states. Returns segments sent."""
    sent = []
    for _ in range(limit):
        if controller.state not in PHASE2:
            break
        msgs = [m for m in controller.drain() if isinstance(m, ev.Segment)]
        if not msgs:
            break
        for seg in msgs:
            sent.append(seg)
            result = rover.execute(seg)
            controller.on_rover_event(result)
            if rover.duplicate_events:
                controller.on_rover_event(result)
    return sent
