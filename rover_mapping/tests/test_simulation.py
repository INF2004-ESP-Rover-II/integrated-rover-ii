import unittest
from collections import deque

from mapping import events as ev
from mapping.config import DIR_VECTORS
from mapping.controller import MappingController, State
from mapping.planner import plan_route
from sim.maze import Maze
from sim.sim_rover import SimRover, run_phase1, run_phase2

LOOP_MAZE = """
A - B - C
|       |
D - E   F
    |   |
G - H - I
"""


# Square A-B-C-D with the last marker (E) off A: DFS closes the loop before finding E.
SQUARE_TAIL = """
B - C
|   |
A - D
|
E
"""


def bfs_hops(maze, start):
    dist, q = {start: 0}, deque([start])
    while q:
        cur = q.popleft()
        for d in range(4):
            nxt = maze.neighbour(cur, d)
            if nxt is not None and nxt not in dist:
                dist[nxt] = dist[cur] + 1
                q.append(nxt)
    return dist


def explore(maze, n, start=0, dup=False):
    c = MappingController()
    c.start(n, start)
    r = SimRover(maze, start, duplicate_events=dup)
    run_phase1(c, r)
    return c, r


def c_id(c, letter):
    return ord(letter) - ord('A')            # Maze.from_ascii numbers markers in reading order


def true_edges(graph):
    return {frozenset((a, b)) for a, n in graph.neighbours.items() for b in n.values()}


def true_links(maze):
    return {frozenset((a, b)) for a, b in (tuple(l) for l in maze.links)}


def learned_links(graph):
    out = set()
    for a, nbrs in graph.neighbours.items():
        for b in nbrs.values():
            out.add(frozenset((a, b)))
    return out


class TestMazeParser(unittest.TestCase):
    def test_parse(self):
        m = Maze.from_ascii(LOOP_MAZE)
        self.assertEqual(len(m.pos), 9)
        self.assertEqual(m.pos[0], (0, 2))          # A: top-left
        self.assertEqual(m.pos[8], (2, 0))          # I: bottom-right
        self.assertEqual(len(m.links), 9)           # 8 tree links + E-H closing a loop


class TestPhase1Simulation(unittest.TestCase):
    def test_full_exploration_matches_ground_truth(self):
        m = Maze.from_ascii(LOOP_MAZE)
        c, _ = explore(m, n=10)                      # more than exist -> rover exhausts every exit
        self.assertEqual(c.state, State.MAP_EXHAUSTED)
        self.assertEqual(c.graph.count(), 9)
        self.assertEqual(c.graph.inconsistencies, [])
        # positions relative to the start marker
        ox, oy = m.pos[0]
        for node, mk in c.graph.markers.items():
            self.assertEqual(mk.pos, (m.pos[node][0] - ox, m.pos[node][1] - oy))
        self.assertEqual(learned_links(c.graph), true_links(m))
        self.assertFalse(c.graph.has_unexplored())    # consistent with EXPLORE_DONE_LOCAL

    def test_stops_when_n_found(self):
        m = Maze.from_ascii(LOOP_MAZE)
        c, _ = explore(m, n=5)
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertEqual(c.graph.count(), 5)
        self.assertIn(ev.CountReached(5), c.sent_log)

    def test_duplicate_deliveries_change_nothing(self):
        m = Maze.from_ascii(LOOP_MAZE)
        a, _ = explore(m, n=10)
        b, _ = explore(m, n=10, dup=True)
        self.assertEqual(learned_links(a.graph), learned_links(b.graph))
        self.assertEqual(a.graph.count(), b.graph.count())

    def test_dead_end_start_and_single_corridor(self):
        m = Maze.from_ascii("A - B - C")
        c, _ = explore(m, n=3)
        self.assertEqual(c.state, State.MAP_COMPLETE)
        c, _ = explore(m, n=4)
        self.assertEqual(c.state, State.MAP_EXHAUSTED)


class TestPhase2Simulation(unittest.TestCase):
    def _complete(self, n=9, dup=False):
        m = Maze.from_ascii(LOOP_MAZE)
        c, r = explore(m, n=n, dup=dup)
        assert c.state == State.MAP_COMPLETE, c.state
        c.drain()
        return m, c, r

    def test_route_executes_and_reaches_target(self):
        m, c, r = self._complete()
        self.assertTrue(c.set_target(6))                      # G
        sent = run_phase2(c, r)
        self.assertEqual(r.cur, 6)
        self.assertEqual(c.sent_log[-1], ev.RouteResult("REACHED", 6))
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertTrue(all(isinstance(s, ev.Segment) for s in sent))

    def test_segment_count_equals_shortest_path(self):
        m, c, r = self._complete()
        hops = bfs_hops(m, r.cur)
        self.assertTrue(c.set_target(8))
        sent = run_phase2(c, r)
        self.assertEqual(len(sent), hops[8])

    def test_duplicate_replies_do_not_skip_segments(self):
        m, c, r = self._complete(dup=True)
        hops = bfs_hops(m, r.cur)
        c.set_target(8)
        sent = run_phase2(c, r)
        self.assertEqual(len(sent), hops[8])
        self.assertEqual(r.cur, 8)

    def test_blocked_edge_triggers_replan_around_loop(self):
        m = Maze.from_ascii(SQUARE_TAIL)
        c, r = explore(m, n=5)
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertEqual(learned_links(c.graph), true_links(m))     # loop was closed before E was found
        c.drain()
        path = plan_route(c.graph, r.cur, 2, c.heading)             # E -> ... -> C (3 hops)
        self.assertEqual(len(path), 4)
        m.block(path[1], path[2])                                   # obstacle appears after the first hop
        c.set_target(2)
        sent = run_phase2(c, r)
        self.assertIn(State.REPLANNING, c.history)
        self.assertEqual(c.sent_log[-1], ev.RouteResult("REACHED", 2))
        self.assertEqual(r.cur, 2)
        self.assertEqual(len(sent), 4)                              # hop to A, failed hop, then A-D-C
        self.assertEqual(c.graph.current, 2)

    def test_probable_link_recovers_a_route_the_exploration_never_drove(self):
        """Week 6 gap #9, fixed: D-A was never travelled, but the markers are grid neighbours, so the
        planner tries it once the only known route is blocked. It exists, so the rover gets there
        and the link becomes a real one."""
        m, c, r = self._complete()
        self.assertTrue(c.graph.has_unexplored())
        start = r.cur
        path = plan_route(c.graph, start, 8, c.heading, use_probable=False)
        m.block(path[0], path[1])
        self.assertIn(8, bfs_hops(m, start))                         # a physical route exists
        before = set(true_edges(c.graph))
        c.set_target(8)
        run_phase2(c, r)
        self.assertEqual(c.sent_log[-1], ev.RouteResult("REACHED", 8))
        self.assertGreater(len(true_edges(c.graph)), len(before))    # a probable edge was promoted

    def test_probable_link_that_is_really_a_wall_gives_no_route(self):
        m = Maze.from_ascii("A - B\n|   |\nC   D\n")             # C-D are grid neighbours with a wall
        c, r = explore(m, n=4)
        self.assertEqual(c.state, State.MAP_COMPLETE)
        c.drain()
        # model "ToF said 1023 (unknown)" for the wall between C and D
        c.graph.markers[c_id(c, "C")].exits[1] = ev.ExitStatus.UNCHECKED
        c.graph.markers[c_id(c, "D")].exits[3] = ev.ExitStatus.UNCHECKED
        self.assertIn(frozenset((c_id(c, "C"), c_id(c, "D"))), {frozenset((a, b)) for a, b, _ in c.graph.probable_links()})
        m.block(c_id(c, "B"), c_id(c, "D"))                            # the only real way to D closes
        c.set_target(c_id(c, "D"))
        run_phase2(c, r)
        self.assertIn(State.REPLANNING, c.history)                    # it tried the guessed edge first...
        self.assertEqual(c.sent_log[-1], ev.RouteResult("NO_ROUTE", c_id(c, "D")))   # ...found the wall
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertNotIn(frozenset((c_id(c, "C"), c_id(c, "D"))), {frozenset((a, b)) for a, b, _ in c.graph.probable_links()})

    def test_all_routes_blocked_gives_no_route_and_keeps_map(self):
        m, c, r = self._complete()
        # cut the corridor next to the rover in every direction it can go
        for d in range(4):
            nb = m.neighbour(r.cur, d)
            if nb is not None:
                m.block(r.cur, nb)
        c.set_target(0 if r.cur != 0 else 8)
        run_phase2(c, r)
        self.assertEqual(c.sent_log[-1], ev.RouteResult("NO_ROUTE", c.sent_log[-1].target))
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertEqual(c.graph.count(), 9)


class TestRandomMazes(unittest.TestCase):
    """After full exploration, A* hop counts equal BFS on the true maze, for every target."""

    def test_astar_matches_bfs_on_random_mazes(self):
        for seed in range(25):
            m = Maze.random(4, 6, extra_links=seed % 6, seed=seed)      # 4 x 6 = 24 markers, as in the 240x360 arena at 60 cm
            c, r = explore(m, n=25)                                      # 25 > 24 -> full exploration
            self.assertEqual(c.state, State.MAP_EXHAUSTED, seed)
            self.assertEqual(c.graph.count(), 24, seed)
            self.assertEqual(c.graph.inconsistencies, [], seed)
            self.assertEqual(learned_links(c.graph), true_links(m), seed)
            hops = bfs_hops(m, 0)
            for target, expected in hops.items():
                path = plan_route(c.graph, 0, target)
                self.assertEqual(len(path) - 1, expected, (seed, target))

    def test_partial_map_routes_use_only_explored_edges(self):
        for seed in range(10):
            m = Maze.random(4, 6, extra_links=4, seed=seed)
            c, r = explore(m, n=12)
            self.assertEqual(c.state, State.MAP_COMPLETE, seed)
            known = c.graph
            for target in known.markers:
                path = plan_route(known, c.graph.current, target)
                self.assertIsNotNone(path, (seed, target))
                for a, b in zip(path, path[1:]):
                    self.assertIn(frozenset((a, b)), true_links(m))


if __name__ == "__main__":
    unittest.main()
