"""A* over the marker graph (FR-27), explored and non-blocked edges only.

Cost = hops * W + 90-degree turns, so the route has the fewest hops first, then the fewest
turns; remaining ties go to the lexicographically smallest marker-ID path (deterministic).
With a uniform marker pitch, hops equal grid distance, so the Manhattan heuristic is
admissible and consistent."""
import heapq
from typing import List, Optional, Tuple

from .config import turn_steps
from .graph import MarkerGraph

W = 1000   # larger than any possible turn count (turns <= edges <= 2N)
PROBABLE_PENALTY = 200   # an unproven edge costs this much extra: explored routes win unless clearly shorter


def _manhattan(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def plan_route(graph: MarkerGraph, start: int, goal: int,
               start_heading: Optional[int] = None, use_probable: bool = True) -> Optional[List[int]]:
    """Return a list of marker IDs from start to goal, or None if no route exists."""
    if start not in graph.markers or goal not in graph.markers:
        return None
    if start == goal:
        return [start]
    gpos = graph.markers[goal].pos
    h = lambda n: _manhattan(graph.markers[n].pos, gpos) * W
    best = {(start, start_heading): 0}
    heap = [(h(start), (start,), 0, start, start_heading)]
    closed = set()
    while heap:
        f, path, g, node, hd = heapq.heappop(heap)
        state = (node, hd)
        if state in closed:
            continue
        closed.add(state)
        if node == goal:
            return list(path)
        for d, nxt in graph.usable_neighbours(node, use_probable):
            extra = PROBABLE_PENALTY if d not in graph.neighbours.get(node, {}) else 0
            ng = g + W + extra + (turn_steps(hd, d) if hd is not None else 0)
            nstate = (nxt, d)
            if nstate in closed or ng > best.get(nstate, 10**12):
                continue
            best[nstate] = ng
            heapq.heappush(heap, (ng + h(nxt), path + (nxt,), ng, nxt, d))
    return None


def path_headings(graph: MarkerGraph, path: List[int]) -> List[Tuple[int, int]]:
    """[(heading, target marker)] for each hop of a path."""
    hops = []
    for a, b in zip(path, path[1:]):
        d = graph.direction_between(a, b)
        if d is None:
            raise ValueError(f"no edge {a}->{b}")
        hops.append((d, b))
    return hops
