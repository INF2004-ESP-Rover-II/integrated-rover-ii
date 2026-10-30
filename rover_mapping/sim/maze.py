"""Ground-truth maze for tests. Markers sit on a grid at integer positions.

ASCII format: node letters A..Z (id = letter index) at columns 0,4,8..; '-' joins nodes in a
row, '|' joins nodes in adjacent rows. Top row has the highest y.

    A - B - C
    |       |
    D - E   F
"""
import textwrap
from typing import Dict, Optional, Set, Tuple

from mapping.config import DIR_VECTORS, opposite


class Maze:
    def __init__(self):
        self.pos: Dict[int, Tuple[int, int]] = {}
        self.at: Dict[Tuple[int, int], int] = {}
        self.links: Set[frozenset] = set()
        self.obstacles: Set[frozenset] = set()   # edges that become blocked during Phase 2

    def add_node(self, node_id: int, pos: Tuple[int, int]) -> None:
        self.pos[node_id] = pos
        self.at[pos] = node_id

    def add_link(self, a: int, b: int) -> None:
        self.links.add(frozenset((a, b)))

    def neighbour(self, node_id: int, d: int) -> Optional[int]:
        """Static neighbour in absolute direction d (what the ToF/walls show)."""
        dx, dy = DIR_VECTORS[d]
        x, y = self.pos[node_id]
        other = self.at.get((x + dx, y + dy))
        if other is not None and frozenset((node_id, other)) in self.links:
            return other
        return None

    def block(self, a: int, b: int) -> None:
        self.obstacles.add(frozenset((a, b)))

    def is_obstructed(self, a: int, b: int) -> bool:
        return frozenset((a, b)) in self.obstacles

    @classmethod
    def from_ascii(cls, text: str) -> "Maze":
        lines = textwrap.dedent(text).strip("\n").split("\n")
        rows = (len(lines) + 1) // 2
        m = cls()
        for r, line in enumerate(lines):
            if r % 2:
                continue
            for c, ch in enumerate(line):
                if ch.isalpha():
                    m.add_node(ord(ch) - ord("A"), (c // 4, rows - 1 - r // 2))
        for r, line in enumerate(lines):
            for c, ch in enumerate(line):
                if ch == "-" and r % 2 == 0:
                    x, y = (c - 2) // 4, rows - 1 - r // 2
                    m.add_link(m.at[(x, y)], m.at[(x + 1, y)])
                elif ch == "|" and r % 2 == 1:
                    x, y = c // 4, rows - 1 - (r - 1) // 2
                    m.add_link(m.at[(x, y)], m.at[(x, y - 1)])
        return m

    @classmethod
    def random(cls, width: int, height: int, extra_links: int, seed: int) -> "Maze":
        """Full grid of markers, random spanning tree plus `extra_links` loop edges."""
        import random
        rnd = random.Random(seed)
        m = cls()
        for y in range(height):
            for x in range(width):
                m.add_node(y * width + x, (x, y))
        seen, stack = {0}, [0]
        while stack:
            cur = stack[-1]
            options = [n for d in range(4) if (n := _step(m, cur, d)) is not None and n not in seen]
            if not options:
                stack.pop()
                continue
            nxt = rnd.choice(options)
            m.add_link(cur, nxt)
            seen.add(nxt)
            stack.append(nxt)
        candidates = [frozenset((a, b)) for a in m.pos for d in (1, 0)
                      if (b := _step(m, a, d)) is not None and frozenset((a, b)) not in m.links]
        rnd.shuffle(candidates)
        m.links.update(candidates[:extra_links])
        return m


def _step(m: Maze, node_id: int, d: int) -> Optional[int]:
    dx, dy = DIR_VECTORS[d]
    x, y = m.pos[node_id]
    return m.at.get((x + dx, y + dy))
