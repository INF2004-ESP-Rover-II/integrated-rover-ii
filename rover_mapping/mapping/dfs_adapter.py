"""Phase 1: feed Buddy 1's laptop DFS graph into the MappingController.

Buddy 1's `DFSExplorer` (dashboard/dfs_explorer.py) owns exploration. This adapter only READS its
public state once per dashboard tick (no changes to her files) and turns what changed into the
events my controller understands:

  new mission (session_id changed)   -> controller.start(n, start marker)
  explorer.current changed + confirmed -> MarkerReached(marker, direction travelled)
  a node's exit masks changed        -> ExitReport (blocked / travelled / open / unchecked)
  explorer.state EXHAUSTED           -> ExploreDoneLocal
  explorer.state PAUSED / FAULT      -> operator stop / mission fault

Directions are the same in both programs: NORTH=0, EAST=1, SOUTH=2, WEST=3 (map frame fixed at
mission start). Duck-typed on purpose: anything with the same attributes works (used in tests)."""
from typing import Optional

from . import events as ev
from .events import ExitStatus
from .bridge import MappingBridge

PAUSED, FAULT, EXHAUSTED = "PAUSED", "FAULT", "EXHAUSTED"


def _val(x):
    return getattr(x, "value", x)


def exit_statuses(node) -> tuple:
    """Her four-bit N/E/S/W masks -> my per-direction ExitStatus (index = direction)."""
    out = []
    for d in range(4):
        bit = 1 << d
        if node.blocked_exits & bit:
            out.append(ExitStatus.BLOCKED)
        elif node.traversed_exits & bit:
            out.append(ExitStatus.EXPLORED)
        elif node.open_exits & bit:
            out.append(ExitStatus.UNEXPLORED)
        else:
            out.append(ExitStatus.UNCHECKED)          # unknown is never guessed to be a wall
    return tuple(out)


class DfsAdapter:
    def __init__(self, bridge: MappingBridge):
        self.bridge = bridge
        self.session = None
        self.prev: Optional[int] = None
        self._sig = {}
        self._state = None

    def sync(self, explorer) -> None:
        """Call once per tick with navigator.explorer (Phase 1 explorer)."""
        if explorer.current is None or not explorer.nodes:
            return
        if explorer.session_id != self.session:
            self._new_mission(explorer)
        if self.session is None:
            return
        cur = explorer.current
        if explorer.current_confirmed and cur != self.prev and self.prev is not None:
            d = self._travelled_direction(explorer, self.prev, cur)
            if d is None:
                self.bridge.notify(f"Mapping: arrived at {cur} from {self.prev} but no link is recorded")
            else:
                self.bridge.on_event(ev.MarkerReached(cur, d))
                self.bridge.confirm_marker(cur)
            self.prev = cur
        for marker_id, node in list(explorer.nodes.items()):
            sig = (node.known_exits, node.open_exits, node.blocked_exits, node.traversed_exits)
            if sig != self._sig.get(marker_id):
                self._sig[marker_id] = sig
                if node.known_exits:
                    self.bridge.on_event(ev.ExitReport(marker_id, exit_statuses(node)))
        state = _val(explorer.state)
        if state != self._state:
            self._state = state
            if state == EXHAUSTED:
                self.bridge.on_event(ev.ExploreDoneLocal())
            elif state == PAUSED:
                self.bridge.stop()
            elif state == FAULT:
                self.bridge.fault()

    def _new_mission(self, explorer) -> None:
        self.session = explorer.session_id
        self._sig, self._state = {}, None
        n = explorer.target_markers or len(explorer.nodes)
        if self.bridge.begin_session(n, explorer.current):
            self.prev = explorer.current
            self.bridge.confirm_marker(explorer.current)
        else:
            self.session = None

    @staticmethod
    def _travelled_direction(explorer, a: int, b: int) -> Optional[int]:
        for d, neighbour in explorer.nodes[a].neighbors.items():
            if neighbour == b:
                return int(d)
        return None
