"""2D occupancy grid (5 cm cells) built from the three ToF readings and the rover pose.

Log-odds per cell (sparse dict, so the arena can be any size). A usable reading of d mm marks the
cells along the ray as more likely free and the cell at d as more likely occupied.

What counts as a reading (Buddy 1's telemetry rules):
  * 1..999 mm  usable distance -> free cells along the ray, an occupied cell at the end
  * 0          unavailable      -> ignored
  * 1000+      range limit / no-target codes (1000, 1023, 65535) -> ignored: nothing proves the ray
               is free, so we neither clear nor mark cells from it
  * a sensor whose fault bit is set -> ignored
Frame: centimetres, x east, y north, origin = the start marker; angles clockwise from north."""
import math
from typing import Dict, Optional, Tuple

import numpy as np

from . import config as cfg

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
SENSORS = (("left", -90.0, 1), ("front", 0.0, 2), ("right", 90.0, 4))


class OccupancyGrid:
    def __init__(self, cell_cm: float = float(cfg.OCC_CELL_CM), hit: float = 0.9, miss: float = 0.35,
                 clamp: float = 4.0, occ_threshold: float = 1.0, free_threshold: float = -0.7,
                 max_range_mm: int = 999, min_range_mm: int = 20):
        self.cell = cell_cm
        self.hit, self.miss, self.clamp = hit, miss, clamp
        self.occ_threshold, self.free_threshold = occ_threshold, free_threshold
        self.max_range_mm, self.min_range_mm = max_range_mm, min_range_mm
        self.cells: Dict[Tuple[int, int], float] = {}
        self.samples = 0
        self.version = 0

    def clear(self) -> None:
        self.cells.clear()
        self.samples = 0
        self.version += 1

    # ---- updates -----------------------------------------------------------------
    def _ij(self, x: float, y: float) -> Tuple[int, int]:
        return math.floor(x / self.cell), math.floor(y / self.cell)

    def _add(self, ij, delta: float) -> None:
        v = self.cells.get(ij, 0.0) + delta
        self.cells[ij] = max(-self.clamp, min(self.clamp, v))

    def integrate_ray(self, x: float, y: float, angle_deg: float, dist_cm: float) -> None:
        a = math.radians(angle_deg)
        dx, dy = math.sin(a), math.cos(a)
        end = self._ij(x + dx * dist_cm, y + dy * dist_cm)
        step = self.cell / 3
        seen = set()
        t = 0.0
        while t < dist_cm:
            ij = self._ij(x + dx * t, y + dy * t)
            if ij != end and ij not in seen:
                seen.add(ij)
                self._add(ij, -self.miss)
            t += step
        self._add(end, self.hit)

    def integrate(self, pose: Tuple[float, float, float], distances_mm: dict, fault_mask: int = 0) -> int:
        """pose = (x_cm, y_cm, heading_deg). Returns how many readings were used."""
        x, y, heading = pose
        used = 0
        for name, offset, bit in SENSORS:
            d = distances_mm.get(name)
            if (fault_mask & bit or type(d) is not int or not self.min_range_mm <= d <= self.max_range_mm):
                continue
            self.integrate_ray(x, y, heading + offset, d / 10.0)
            used += 1
        if used:
            self.samples += 1
            self.version += 1
        return used

    # ---- queries -----------------------------------------------------------------
    def state(self, ij) -> int:
        v = self.cells.get(ij)
        if v is None:
            return UNKNOWN
        if v >= self.occ_threshold:
            return OCCUPIED
        if v <= self.free_threshold:
            return FREE
        return UNKNOWN

    def occupied_cells(self):
        return {ij for ij, v in self.cells.items() if v >= self.occ_threshold}

    def free_cells(self):
        return {ij for ij, v in self.cells.items() if v <= self.free_threshold}

    def stats(self) -> dict:
        occ, free = len(self.occupied_cells()), len(self.free_cells())
        return {"occupied": occ, "free": free, "samples": self.samples}

    def render(self, margin_cm: float = 0.0):
        """(rgba float array [ny, nx, 4], extent [x0, x1, y0, y1] in cm) or None if empty."""
        if not self.cells:
            return None
        xs = [i for i, _ in self.cells]
        ys = [j for _, j in self.cells]
        i0, i1, j0, j1 = min(xs), max(xs), min(ys), max(ys)
        img = np.zeros((j1 - j0 + 1, i1 - i0 + 1, 4), dtype=float)
        for (i, j), v in self.cells.items():
            if v >= self.occ_threshold:
                img[j - j0, i - i0] = (0.95, 0.35, 0.25, 0.9)       # occupied
            elif v <= self.free_threshold:
                img[j - j0, i - i0] = (0.75, 0.85, 0.95, 0.16)      # free
        c = self.cell
        return img, [i0 * c, (i1 + 1) * c, j0 * c, (j1 + 1) * c]
