"""Thread-safe wrapper around MappingController for the dashboard (in-process, no MQTT needed).

Buddy 1's laptop DFS and navigation run in the same Python process as the dashboard, so the
mapping code is fed by plain method calls from `dfs_adapter.DfsAdapter` (Phase 1) and
`route_exec.RouteExplorer` (Phase 2). Optionally pass `publish(topic, text)` to also publish the
laptop-only JSON topics robot/map (snapshot) and robot/route (result) for other subsystems."""
import json
import threading
from collections import deque
from typing import Callable, Optional

from . import events as ev
from .controller import MappingController, State
from .snapshot import build_snapshot
from .occupancy import OccupancyGrid

TOPIC_MAP = "robot/map"
TOPIC_ROUTE = "robot/route"


class MappingBridge:
    def __init__(self, notify: Optional[Callable[[str], None]] = None,
                 publish: Optional[Callable[[str, str], None]] = None):
        self.ctrl = MappingController()
        self.lock = threading.RLock()
        self.version = 0                       # bumps on every change; the UI polls this
        self.notify = notify or (lambda msg: None)
        self.publish = publish
        self.segments = deque()                # Segments the planner wants driven, oldest first
        self.grid = OccupancyGrid()            # 5 cm ToF occupancy grid (see mapping/occupancy.py)

    # ---- session handling ------------------------------------------------------------
    def _force_idle(self) -> None:
        """Whatever state we were in, get back to IDLE with an empty map (new mission)."""
        c = self.ctrl
        for _ in range(4):
            if c.state == State.IDLE:
                return
            if c.state == State.MAPPING or c.state in (State.PLANNING, State.SENDING_SEGMENT,
                                                       State.WAITING_SEGMENT, State.REPLANNING):
                c.operator_stop()
            elif c.state in (State.PAUSED, State.MAP_COMPLETE):
                c.reset_map()
            elif c.state in (State.FAULT_WAIT, State.MAP_EXHAUSTED):
                c.operator_reset(self_check_ok=True)
            else:
                return

    def begin_session(self, n: int, start_marker_id: int) -> bool:
        with self.lock:
            self._force_idle()
            self.segments.clear()
            self.grid.clear()
            ok = self.ctrl.start(n, start_marker_id)
            self._flush()
        if not ok:
            self.notify(f"Cannot start mapping in state {self.ctrl.state.value}")
        return ok

    # ---- inputs ----------------------------------------------------------------------
    def on_event(self, event) -> bool:
        with self.lock:
            used = self.ctrl.on_rover_event(event)
            self._flush()
        return used

    def confirm_marker(self, marker_id: int) -> bool:
        """Camera-confirmed arrival at marker_id."""
        with self.lock:
            ok = self.ctrl.on_marker_seen(marker_id)
            self._bump()
        return ok

    def set_target(self, target_id: int) -> bool:
        with self.lock:
            c = self.ctrl
            if c.state != State.MAP_COMPLETE:
                self.notify(f"Cannot route in state {c.state.value} (the map must be complete)")
                return False
            if target_id not in c.graph.markers:
                self.notify(f"Marker {target_id} is not on the map "
                            f"(mapped: {sorted(c.graph.markers)})")
                return False
            if not c.position_confirmed:
                self.notify("Cannot route: the rover's marker has not been confirmed by the camera")
                return False
        return self._do(lambda c: c.set_target(target_id), "set target")

    def stop(self) -> None:
        self._do(lambda c: c.operator_stop(), "stop", quiet=True)

    def fault(self) -> None:
        self._do(lambda c: c.mission_fault(), "fault", quiet=True)

    def resume(self) -> bool:
        return self._do(lambda c: c.operator_resume(self_check_ok=True), "resume")

    def reset_fault(self) -> bool:
        return self._do(lambda c: c.operator_reset(self_check_ok=True), "reset")

    def reset_map(self) -> bool:
        ok = self._do(lambda c: c.reset_map(), "reset map")
        if ok:
            self.grid.clear()
        return ok

    def _do(self, fn, what: str, quiet: bool = False) -> bool:
        with self.lock:
            before = self.ctrl.state
            result = fn(self.ctrl)
            if result is False and not quiet:
                self.notify(f"Cannot {what} in state {before.value}")
            self._flush()
        return result is not False

    # ---- outputs ---------------------------------------------------------------------
    def peek_segment(self) -> Optional[ev.Segment]:
        with self.lock:
            return self.segments[0] if self.segments else None

    def pop_segment(self) -> Optional[ev.Segment]:
        with self.lock:
            return self.segments.popleft() if self.segments else None

    def _flush(self) -> None:
        for msg in self.ctrl.drain():
            if isinstance(msg, ev.Segment):
                self.segments.append(msg)
                self.notify(f"Route: segment {msg.seq} to marker {msg.target_marker}"
                            f"{' (last)' if msg.last else ''}")
            elif isinstance(msg, ev.RouteResult):
                self.segments.clear()
                self._publish(TOPIC_ROUTE, json.dumps({"result": msg.result, "target": msg.target}))
                self.notify(f"Route to marker {msg.target}: {msg.result}")
            elif isinstance(msg, ev.CountReached):
                self.notify(f"All {msg.n} markers found; map complete")
            elif isinstance(msg, ev.MapExhausted):
                self.notify(f"FAULT: exploration exhausted with {msg.count} of {msg.n} markers")
        if self.ctrl.state not in (State.SENDING_SEGMENT, State.WAITING_SEGMENT, State.REPLANNING,
                                   State.PLANNING):
            self.segments.clear()
        self._bump()

    def _bump(self) -> None:
        self.version += 1
        self._publish(TOPIC_MAP, json.dumps(build_snapshot(self.ctrl)))

    def _publish(self, topic: str, payload: str) -> None:
        if self.publish is not None:
            try:
                self.publish(topic, payload)
            except Exception as exc:         # never let a network error break the UI thread
                self.notify(f"Publish failed: {exc}")

    def snapshot(self) -> dict:
        with self.lock:
            return build_snapshot(self.ctrl)
