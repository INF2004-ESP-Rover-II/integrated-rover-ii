"""Ground-truth geometry of the maze in centimetres, for the simulated ToF sensors and for scoring
the occupancy grid. Markers sit on a pitch-cm grid; a wall stands half a pitch from a marker wherever
the maze has no link (and around the outside); extra boxes are simulated obstacles.

Coordinates: x to the east, y to the north, angles in degrees clockwise from north (the same
convention as the rover's map heading)."""
import math
from typing import List, Optional, Set, Tuple

Segment = Tuple[float, float, float, float]


class World:
    def __init__(self, maze, pitch: float = 60.0, boxes: Optional[List[Tuple[float, float, float, float]]] = None):
        self.maze, self.pitch = maze, pitch
        self.segments: List[Segment] = []
        h = pitch / 2
        at = {pos: n for n, pos in maze.pos.items()}
        for n, (ix, iy) in maze.pos.items():
            cx, cy = ix * pitch, iy * pitch
            for d, (dx, dy) in enumerate(((0, 1), (1, 0), (0, -1), (-1, 0))):
                other = at.get((ix + dx, iy + dy))
                linked = other is not None and frozenset((n, other)) in maze.links
                if linked:
                    continue
                if other is not None and other < n:
                    continue                      # the shared wall is added from the lower id only
                # wall segment perpendicular to direction d, half a pitch away, one pitch long
                if dx == 0:
                    y = cy + dy * h
                    self.segments.append((cx - h, y, cx + h, y))
                else:
                    x = cx + dx * h
                    self.segments.append((x, cy - h, x, cy + h))
        self.boxes = []
        for b in boxes or []:
            self.add_box(*b)

    def add_box(self, x0, y0, x1, y1) -> None:
        x0, x1, y0, y1 = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
        self.boxes.append((x0, y0, x1, y1))
        self.segments += [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]

    def marker_xy(self, node: int) -> Tuple[float, float]:
        ix, iy = self.maze.pos[node]
        return ix * self.pitch, iy * self.pitch

    def raycast(self, x: float, y: float, angle_deg: float, max_cm: float) -> Optional[float]:
        """Distance to the nearest wall/box along the ray, or None if nothing within max_cm."""
        a = math.radians(angle_deg)
        dx, dy = math.sin(a), math.cos(a)
        best = None
        for x1, y1, x2, y2 in self.segments:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((x1 - x) * ey - (y1 - y) * ex) / den       # along the ray
            u = ((x1 - x) * dy - (y1 - y) * dx) / den       # along the segment
            if t >= 0 and 0 <= u <= 1 and (best is None or t < best):
                best = t
        return best if best is not None and best <= max_cm else None

    def occupied_cells(self, cell_cm: float = 5.0) -> Set[Tuple[int, int]]:
        """Grid cells touched by a wall or box (ground truth for scoring)."""
        cells = set()
        for x1, y1, x2, y2 in self.segments:
            steps = int(max(abs(x2 - x1), abs(y2 - y1)) / (cell_cm / 4)) + 1
            for k in range(steps + 1):
                t = k / steps
                cells.add((math.floor((x1 + (x2 - x1) * t) / cell_cm), math.floor((y1 + (y2 - y1) * t) / cell_cm)))
        return cells


def add_corridor_boxes(world: "World", maze, count: int = 10, seed: int = 5, depth: float = 12.0,
                       length: float = 12.0, lateral: float = 18.0):
    """Small simulated obstacles standing against the side wall at the middle of some corridors.
    They never block the centre line (the rover still fits) but the side ToF sensors see them."""
    import random
    rng = random.Random(seed)
    links = [tuple(sorted(l)) for l in maze.links]
    rng.shuffle(links)
    boxes = []
    for a, b in links[:count]:
        (ax, ay), (bx, by) = world.marker_xy(a), world.marker_xy(b)
        mx, my = (ax + bx) / 2, (ay + by) / 2
        side, h = rng.choice([-1, 1]), length / 2
        if ax == bx:
            boxes.append((mx + side * lateral, my - h, mx + side * (lateral + depth), my + h))
        else:
            boxes.append((mx - h, my + side * lateral, mx + h, my + side * (lateral + depth)))
    for b in boxes:
        world.add_box(*b)
    return boxes
