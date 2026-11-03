"""Plain-dict view of the controller for the dashboard and for robot/map."""
from . import events as ev
from .config import JUNCTION_IDS


def build_snapshot(ctrl) -> dict:
    g = ctrl.graph
    markers = [{"id": m.id, "x": m.pos[0], "y": m.pos[1],
                "junction": m.id in JUNCTION_IDS,
                "exits": [int(s) for s in m.exits]} for m in g.markers.values()]
    edges, seen = [], set()
    for a, nbrs in g.neighbours.items():
        for b in nbrs.values():
            key = frozenset((a, b))
            if key not in seen:
                seen.add(key)
                edges.append([min(a, b), max(a, b), key in g.blocked])
    last_result = next((m for m in reversed(ctrl.sent_log) if isinstance(m, ev.RouteResult)), None)
    return {
        "state": ctrl.state.value,
        "n": ctrl.n,
        "count": g.count(),
        "start": g.start_id,
        "current": g.current,
        "confirmed": ctrl.position_confirmed,
        "target": ctrl.target,
        "route": list(ctrl.path),
        "hop": ctrl.hop,
        "markers": markers,
        "edges": edges,
        "probable": [[a, b, strong] for a, b, strong in g.probable_links()],
        "last_result": None if last_result is None else last_result.result,
        "inconsistencies": list(g.inconsistencies),
    }
