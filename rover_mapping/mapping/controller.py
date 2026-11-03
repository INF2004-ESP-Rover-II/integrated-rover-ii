"""Mapping & Path Planning state machine (Section 6.4 statechart).

Inputs are plain method calls / event objects; outputs are collected in `outbox`
(Segment, RouteResult, CountReached, MapExhausted). No MQTT or bytes in here."""
from enum import Enum
from typing import List, Optional

from . import events as ev
from .config import SEQ_BITS
from .graph import MarkerGraph
from .planner import path_headings, plan_route
from .seqnum import is_newer, next_seq


class State(Enum):
    IDLE = "IDLE"
    MAPPING = "MAPPING"
    PAUSED = "PAUSED"
    MAP_COMPLETE = "MAP_COMPLETE"
    MAP_EXHAUSTED = "MAP_EXHAUSTED"
    PLANNING = "PLANNING"
    SENDING_SEGMENT = "SENDING_SEGMENT"
    WAITING_SEGMENT = "WAITING_SEGMENT"
    REPLANNING = "REPLANNING"
    TARGET_REACHED = "TARGET_REACHED"
    NO_ROUTE = "NO_ROUTE"
    FAULT_WAIT = "FAULT_WAIT"
    ENDED = "ENDED"          # final state (operator exits dashboard)


PHASE2 = (State.PLANNING, State.SENDING_SEGMENT, State.WAITING_SEGMENT, State.REPLANNING)


class MappingController:
    def __init__(self):
        self.history: List[State] = []
        self.outbox: list = []
        self.sent_log: list = []      # everything ever published, never drained
        self.state = State.IDLE
        self.history.append(State.IDLE)
        self._clear_map()

    # ---- helpers -----------------------------------------------------------------
    def _clear_map(self) -> None:
        self.graph = MarkerGraph()
        self.n: Optional[int] = None
        self.heading = 0
        self.position_confirmed = False
        self._last_event_seq: Optional[int] = None
        self._reset_route()
        self._seg_seq = 0

    def _reset_route(self) -> None:
        self.target: Optional[int] = None
        self.path: List[int] = []
        self.hop = 0
        self.outstanding: Optional[ev.Segment] = None

    def _goto(self, state: State) -> None:
        self.state = state
        self.history.append(state)

    def _emit(self, msg) -> None:
        self.outbox.append(msg)
        self.sent_log.append(msg)

    def drain(self) -> list:
        out, self.outbox = self.outbox, []
        return out

    def _fresh(self, seq: Optional[int]) -> bool:
        """Drop repeated or older rover events (wrap-safe)."""
        if seq is None or self._last_event_seq is None:
            self._last_event_seq = seq if seq is not None else self._last_event_seq
            return True
        if seq == self._last_event_seq or not is_newer(seq, self._last_event_seq, SEQ_BITS):
            return False
        self._last_event_seq = seq
        return True

    # ---- operator / Mission inputs -----------------------------------------------
    def start(self, n: int, start_marker_id: int) -> bool:
        """'N set and start marker seen': IDLE -> MAPPING."""
        if self.state != State.IDLE or n < 1:
            return False
        self.n = n
        self.graph.start(start_marker_id)
        self.position_confirmed = True
        self._goto(State.MAPPING)
        self._check_count()
        return True

    def operator_stop(self) -> None:
        if self.state == State.MAPPING:
            self.position_confirmed = False
            self._goto(State.PAUSED)
        elif self.state in PHASE2:
            self._reset_route()
            self.position_confirmed = False
            self._goto(State.MAP_COMPLETE)

    def mission_fault(self) -> None:
        """Fault reported by Mission Control (not detected locally)."""
        if self.state == State.MAPPING:
            self.position_confirmed = False
            self._goto(State.PAUSED)
        elif self.state in PHASE2:
            self._reset_route()
            self.position_confirmed = False
            self._goto(State.FAULT_WAIT)

    def operator_resume(self, self_check_ok: bool) -> bool:
        if self.state == State.PAUSED and self_check_ok and self.position_confirmed:
            self._goto(State.MAPPING)
            return True
        return False

    def operator_reset(self, self_check_ok: bool = True) -> bool:
        """FAULT_WAIT -> MAP_COMPLETE (map kept); MAP_EXHAUSTED -> IDLE (map cleared)."""
        if self.state == State.FAULT_WAIT and self_check_ok:
            self._goto(State.MAP_COMPLETE)
            return True
        if self.state == State.MAP_EXHAUSTED:
            self._clear_map()
            self._goto(State.IDLE)
            return True
        return False

    def reset_map(self) -> bool:
        if self.state in (State.PAUSED, State.MAP_COMPLETE):
            self._clear_map()
            self._goto(State.IDLE)
            return True
        return False

    def exit_dashboard(self) -> bool:
        if self.state == State.IDLE:
            self._goto(State.ENDED)
            return True
        return False

    def on_marker_seen(self, marker_id: int) -> bool:
        """Vision confirms the rover's current marker."""
        if marker_id in self.graph.markers:
            self.graph.current = marker_id
            self.position_confirmed = True
            return True
        return False

    def set_target(self, target_id: int) -> bool:
        """MAP_COMPLETE -> PLANNING, only when the current marker is camera-confirmed."""
        if (self.state != State.MAP_COMPLETE or not self.position_confirmed
                or target_id not in self.graph.markers):
            return False
        self.target = target_id
        self._goto(State.PLANNING)
        self._plan()
        return True

    # ---- rover events ------------------------------------------------------------
    def on_rover_event(self, event) -> bool:
        """Returns True if the event was used, False if ignored (wrong state, repeat, stale)."""
        if isinstance(event, ev.MarkerSeen):
            return self.on_marker_seen(event.marker_id)
        if self.state == State.MAPPING:
            return self._mapping_event(event)
        if self.state == State.WAITING_SEGMENT:
            return self._waiting_event(event)
        return False

    def _mapping_event(self, e) -> bool:
        if isinstance(e, ev.SegmentDone):
            return False
        if not self._fresh(getattr(e, "seq", None)):
            return False
        g = self.graph
        if isinstance(e, ev.MarkerReached):
            g.marker_reached(e.marker_id, e.heading)
            self.heading = e.heading
            self.position_confirmed = True
            self._check_count()
        elif isinstance(e, ev.ExitExplored):
            g.set_exit_status(e.marker_id, e.direction, ev.ExitStatus.EXPLORED)
        elif isinstance(e, ev.ExitBlocked):
            g.mark_blocked(e.marker_id, e.direction)
        elif isinstance(e, ev.ExitReport):
            g.apply_exit_report(e.marker_id, e.exits)
        elif isinstance(e, ev.Backtrack):
            pass   # position is updated by the MarkerReached that follows
        elif isinstance(e, ev.ExploreDoneLocal):
            if g.count() < self.n:
                self._emit(ev.MapExhausted(g.count(), self.n))
                self._goto(State.MAP_EXHAUSTED)
        else:
            return False
        return True

    def _check_count(self) -> None:
        if self.state == State.MAPPING and self.graph.count() == self.n:
            self._emit(ev.CountReached(self.n))
            self._goto(State.MAP_COMPLETE)

    def _waiting_event(self, e) -> bool:
        seg = self.outstanding
        if seg is None:
            return False
        if isinstance(e, ev.SegmentDone) and e.ref_seq == seg.seq:
            if self.graph.direction_between(self.graph.current, seg.target_marker) is not None:
                # a probable edge was driven: it is a real link now
                self.graph.link(self.graph.current, seg.heading, seg.target_marker)
            self.graph.current = seg.target_marker
            self.heading = seg.heading
            self.position_confirmed = True
            self.outstanding = None
            if seg.last:
                self._finish("REACHED")
            else:
                self.hop += 1
                self._send_next()
            return True
        if isinstance(e, ev.ExitBlocked) and e.ref_seq == seg.seq:
            self.graph.mark_blocked(e.marker_id, e.direction)
            self.outstanding = None
            self._goto(State.REPLANNING)
            self._plan()
            return True
        return False   # stale or repeated SEGMENT_DONE / EXIT_BLOCKED

    # ---- planning ----------------------------------------------------------------
    def _plan(self) -> None:
        path = plan_route(self.graph, self.graph.current, self.target, self.heading)
        if path is None:
            self._finish("NO_ROUTE")
            return
        self.path, self.hop = path, 0
        if len(path) == 1:
            self._finish("REACHED")
            return
        self._send_next()

    def _send_next(self) -> None:
        self._goto(State.SENDING_SEGMENT)
        hops = path_headings(self.graph, self.path)
        heading, target = hops[self.hop]
        self._seg_seq = next_seq(self._seg_seq, SEQ_BITS)
        seg = ev.Segment(self._seg_seq, target, heading, last=(self.hop == len(hops) - 1))
        self.outstanding = seg
        self._emit(seg)
        self._goto(State.WAITING_SEGMENT)

    def _finish(self, result: str) -> None:
        self._goto(State.TARGET_REACHED if result == "REACHED" else State.NO_ROUTE)
        self._emit(ev.RouteResult(result, self.target))
        self._reset_route()
        self._goto(State.MAP_COMPLETE)
