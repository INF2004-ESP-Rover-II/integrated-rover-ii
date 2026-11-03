"""Phase 2: drive the planned route with Buddy 1's existing turn / drive / arrival code.

Her LaptopNavigator (dashboard/laptop_navigation.py) already knows how to turn to a map heading,
drive forward, stop 450 mm from the camera lens to the marker, and confirm arrival with 5 frames.
It gets its next move from `navigator.explorer`. For a route we temporarily plug in a
`RouteExplorer`, an object with the same interface as her DFSExplorer whose "next move" is the next
segment of my A* route and whose "arrival" reports SegmentDone to my controller. None of her files
change; when the route ends the original DFS explorer is plugged back in.

Needs her dashboard directory on sys.path (the dashboard does that itself; tests add it)."""
from dataclasses import replace
from enum import Enum
from typing import Optional

from dfs_explorer import Action, Direction, MoveKind, Segment, State

from . import events as ev
from .bridge import MappingBridge
from .controller import PHASE2, State as MState


class RouteKind(str, Enum):
    ROUTE = "ROUTE"          # shows as "Marker 5: ROUTE EAST" in her status line


class RouteExplorer:
    """Duck-typed stand-in for DFSExplorer while a route is being driven."""

    def __init__(self, bridge: MappingBridge):
        self.bridge = bridge
        self.state = State.IDLE
        self.current: Optional[int] = None
        self.current_confirmed = False
        self.heading = Direction.NORTH
        self.session_id = 0
        self.target_markers = 0
        self.segment_timeout_s = 30.0
        self.pending: Optional[Segment] = None
        self.issued_at = 0.0
        self.fault: Optional[str] = None
        self._next_id = 1
        self._planned: Optional[ev.Segment] = None   # the controller segment being driven
        self._return: Optional[tuple] = None         # (direction, marker) while backing out of a blocked hop
        self.recovering = False                      # True from a blockage until the next confirmed arrival
        self.blocked_log = []                        # (source marker, direction) of every blockage reported

    @property
    def nodes(self):
        return self.bridge.ctrl.graph.markers

    # ---- the interface LaptopNavigator calls ---------------------------------------
    def start(self, session_id, target_markers, marker_id, marker_type, heading,
              segment_timeout_s=10.0):
        here = self.bridge.ctrl.graph.current
        if marker_id != here:
            raise ValueError(f"Camera sees marker {marker_id} but the map says the rover is at {here}")
        self.__init__(self.bridge)
        self.session_id, self.segment_timeout_s = session_id, segment_timeout_s
        self.current, self.current_confirmed, self.state = marker_id, True, State.READY

    def observe_exits(self, marker_id, open_exits, blocked_exits):
        return True              # the map already exists; scans during a route are not used

    def tick(self, now):
        if self.state == State.MOVING and now - self.issued_at >= self.segment_timeout_s:
            self._fail("TIMEOUT")

    def next_action(self, now):
        self.tick(now)
        if self.state == State.READY and self._return is not None:
            direction, marker = self._return
            # from_marker=0 (not a real marker ID): her navigator ignores any sighting of segment.from_marker
            # ("still the marker we left"), and here the marker we are driving back to IS the one we left.
            self.pending = Segment(self.session_id, self._next_id, 0, False,
                                   direction, MoveKind.BLOCKED_RETURN, marker)
            self._next_id += 1
            self.issued_at, self.current_confirmed, self.state = now, False, State.MOVING
            return Action("MOVE", self.pending)
        if self.state == State.READY:
            planned = self.bridge.peek_segment()
            if planned is None:
                return Action("WAIT")
            self._planned = planned
            self.pending = Segment(self.session_id, self._next_id, self.current, True,
                                   Direction(planned.heading), RouteKind.ROUTE, planned.target_marker)
            self._next_id += 1
            self.issued_at, self.current_confirmed, self.state = now, False, State.MOVING
            return Action("MOVE", self.pending)
        if self.state in (State.COMPLETE, State.PAUSED, State.FAULT):
            return Action(self.state.value, fault=self.fault)
        return Action("WAIT")

    def arrive(self, session_id, segment_id, marker_id, marker_type, heading, now):
        self.tick(now)
        seg, planned = self.pending, self._planned
        if (self.state != State.MOVING or seg is None or
                session_id != self.session_id or segment_id != seg.segment_id):
            return False
        if seg.kind == MoveKind.BLOCKED_RETURN:          # backed out of a blocked hop: position known again
            if marker_id != seg.target_marker:
                self._fail("TOPOLOGY")
                return False
            self.bridge.confirm_marker(marker_id)
            self._return, self.pending, self.recovering = None, None, False
            self.current, self.current_confirmed = marker_id, True
            self.state = State.READY if self.bridge.ctrl.state in PHASE2 else State.COMPLETE
            return True
        if planned is None:
            return False
        if marker_id != planned.target_marker:
            self._fail("TOPOLOGY")
            return False
        self.bridge.pop_segment()
        self.bridge.on_event(ev.SegmentDone(planned.seq))
        self.bridge.confirm_marker(marker_id)
        self.current, self.current_confirmed = marker_id, True
        self.pending = self._planned = None
        self.recovering = False
        finished = self.bridge.ctrl.state not in PHASE2
        self.state = State.COMPLETE if finished else State.READY
        return True

    def segment_blocked(self, session_id, segment_id, now, moved=True):
        """The hop in progress is blocked (obstacle in front). Same signature as the DFS explorer's.

        1. the blocked edge is marked blocked in the map (both directions) and the route is re-planned;
        2. if the rover had already left the source marker (`moved`, the safe default), it first backs
           out: turn round and drive to the source marker, where the camera confirms the position;
        3. then the new route continues (or the route ends with NO_ROUTE, rover stopped at the marker)."""
        if (self.state != State.MOVING or self.pending is None or self._planned is None or
                session_id != self.session_id or segment_id != self.pending.segment_id or
                self.pending.kind == MoveKind.BLOCKED_RETURN):
            return False
        planned, source = self._planned, self.current
        direction = Direction(planned.heading)
        self.blocked_log.append((source, int(direction)))
        self.bridge.pop_segment()                         # the hop that cannot be driven
        self.bridge.on_event(ev.ExitBlocked(source, int(direction), ref_seq=planned.seq))
        self.pending = self._planned = None
        self.recovering = True
        if moved:
            self._return, self.current_confirmed = (direction.opposite, source), False
        else:
            self.current_confirmed = True
        self.state = State.READY
        if not moved and self.bridge.ctrl.state not in PHASE2:
            self.state, self.recovering = State.COMPLETE, False      # NO_ROUTE and still at the marker
        return True

    def pause(self):
        if self.state not in (State.READY, State.MOVING):
            return False
        self.pending = self._planned = self._return = None
        self.current_confirmed = False
        self.state = State.PAUSED
        return True

    def _fail(self, reason):
        self.state, self.fault = State.FAULT, reason


class RouteRunner:
    """Starts a route through the navigator and puts the DFS explorer back afterwards."""

    def __init__(self, bridge: MappingBridge):
        self.bridge = bridge
        self.active = False
        self._saved_explorer = None

    def begin(self, nav, target_id: int, threshold_mm: int = 500) -> None:
        """Raises ValueError with a readable reason if the route cannot start."""
        if self.active or nav.active:
            raise ValueError("Stop the running mission first")
        b = self.bridge
        if b.ctrl.state != MState.MAP_COMPLETE:
            raise ValueError(f"Map is not complete (state {b.ctrl.state.value})")
        self._hook_blockage(nav)
        saved_explorer, saved_zero = nav.explorer, nav.map_zero_deg
        nav.explorer = RouteExplorer(b)
        try:
            nav.start(1, threshold_mm or nav.threshold_mm or 500)   # camera/sensor checks live here
        except ValueError:
            nav.explorer = saved_explorer
            raise
        nav.map_zero_deg = saved_zero                    # keep the map frame of the exploration
        if not b.set_target(target_id):
            nav.stop("Route could not be planned")
            nav.explorer = saved_explorer
            raise ValueError(f"Cannot route to marker {target_id} (see the [Mapping] message above)")
        self._saved_explorer, self.active = saved_explorer, True

    def sync(self, nav) -> None:
        """Call each tick: when the navigator stops, restore the DFS explorer."""
        if not self.active or nav.active:
            return
        self.active = False
        nav.explorer, self._saved_explorer = self._saved_explorer, None
        if self.bridge.ctrl.state in PHASE2:             # stopped before arriving
            self.bridge.stop()

    # ---- blockage hook -----------------------------------------------------------------
    def _hook_blockage(self, nav) -> None:
        """Wrap nav.tick (on the instance, her file is not edited) so a sensor stop request during a route
        hop becomes `explorer.segment_blocked(...)` instead of a plain pause.

        Her navigator pauses on `sensor.stop_required` (front ToF <= the urgent distance) at the top of every
        tick. During a route that is now: stop the motors, report the blockage (map edge blocked, route
        re-planned), back out to the source marker if the rover had left it, continue. While the rover is
        backing out, the stop request is hidden from her tick for the in-place turn only, because the
        obstacle is still in front of it until it has turned round; any stop request while driving forward
        still stops it. Only active while a RouteExplorer is plugged in; Phase 1 behaviour is unchanged."""
        if getattr(nav, "_mapping_tick_hooked", False):
            return
        original = nav.tick

        def tick():
            ex, sensor = nav.explorer, nav.sensor
            if not (isinstance(ex, RouteExplorer) and nav.active and sensor is not None
                    and sensor.stop_required):
                return original()
            now = nav.clock()
            if (ex.state == State.MOVING and ex.pending is not None and ex._planned is not None
                    and nav.phase in ("turn", "settle", "drive")):
                moved = nav.phase == "drive"             # still turning/settling = never left the marker
                nav._send("STOP", now, force=True)
                if ex.segment_blocked(ex.session_id, ex.pending.segment_id, now, moved=moved):
                    nav.phase, nav.departed = "scan", False
                    nav.ready_at = now + nav.STOP_SETTLE_S
                    nav._reset_arrival()
                    nav._status("Route blocked: %s" % ("backing out to marker %s" % ex.current
                                                      if moved else "re-planning at marker %s" % ex.current))
            if ex.recovering and nav.phase in ("turn", "scan", "settle"):
                nav.sensor = replace(sensor, stop_required=False)
                try:
                    return original()
                finally:
                    nav.sensor = sensor
            return original()

        nav.tick = tick
        nav._mapping_tick_hooked = True
