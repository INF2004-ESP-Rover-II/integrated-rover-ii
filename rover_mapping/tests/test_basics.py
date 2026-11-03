import unittest

from mapping import events as ev
from mapping.config import E, N, S, W, opposite, turn_steps
from mapping.events import ExitStatus, exits_from_relative
from mapping.graph import MarkerGraph
from mapping.planner import plan_route
from mapping.seqnum import is_newer, next_seq


class TestSeq(unittest.TestCase):
    def test_newer_and_wrap(self):
        self.assertTrue(is_newer(5, 4))
        self.assertFalse(is_newer(4, 4))            # repeat
        self.assertFalse(is_newer(3, 4))            # older
        self.assertTrue(is_newer(0, 255))           # wraps: 0 is newer than 255
        self.assertTrue(is_newer(130, 3))           # ahead by 127 is still newer
        self.assertFalse(is_newer(131, 3))          # ahead by 128 counts as older
        self.assertEqual(next_seq(255), 0)


class TestConfigHelpers(unittest.TestCase):
    def test_turns(self):
        self.assertEqual(turn_steps(N, N), 0)
        self.assertEqual(turn_steps(N, E), 1)
        self.assertEqual(turn_steps(N, W), 1)
        self.assertEqual(turn_steps(N, S), 2)
        self.assertEqual(opposite(E), W)

    def test_relative_exits(self):
        # facing east: front=E, left=N, right=S, back=W
        out = exits_from_relative(E, ExitStatus.UNEXPLORED, ExitStatus.BLOCKED, ExitStatus.UNEXPLORED)
        self.assertEqual(out[E], ExitStatus.UNEXPLORED)
        self.assertEqual(out[N], ExitStatus.BLOCKED)
        self.assertEqual(out[S], ExitStatus.UNEXPLORED)
        self.assertEqual(out[W], ExitStatus.EXPLORED)


class TestGraph(unittest.TestCase):
    def make(self):
        g = MarkerGraph()
        g.start(0)
        return g

    def test_positions_inferred_from_headings(self):
        g = self.make()
        self.assertTrue(g.marker_reached(1, E))
        self.assertTrue(g.marker_reached(2, N))
        self.assertEqual(g.markers[1].pos, (1, 0))
        self.assertEqual(g.markers[2].pos, (1, 1))
        self.assertEqual(g.count(), 3)
        self.assertEqual(g.neighbours[1][W], 0)

    def test_revisit_not_counted_twice(self):
        g = self.make()
        g.marker_reached(1, E)
        g.marker_reached(0, W)            # back to start
        self.assertFalse(g.marker_reached(1, E))
        self.assertEqual(g.count(), 2)

    def test_loop_closure_consistent(self):
        g = self.make()
        for mid, h in ((1, E), (2, N), (3, W)):
            g.marker_reached(mid, h)      # 0 -> (1,0) -> (1,1) -> (0,1)
        g.marker_reached(0, S)            # (0,1) -> (0,0): closes the loop
        self.assertEqual(g.inconsistencies, [])
        self.assertEqual(g.neighbours[0][N], 3)

    def test_inconsistent_hop_is_recorded(self):
        g = self.make()
        g.marker_reached(1, E)
        g.marker_reached(2, E)
        g.marker_reached(0, W)            # claims (2,0) -> west reaches 0, but 0 is at (0,0)
        self.assertEqual(len(g.inconsistencies), 1)

    def test_exit_status_never_downgrades_explored(self):
        g = self.make()
        g.marker_reached(1, E)
        g.set_exit_status(0, E, ExitStatus.UNEXPLORED)
        self.assertEqual(g.markers[0].exits[E], ExitStatus.EXPLORED)

    def test_blocked_edge_removed_from_usable(self):
        g = self.make()
        g.marker_reached(1, E)
        g.mark_blocked(0, E)
        self.assertEqual(g.usable_neighbours(0), [])
        self.assertEqual(g.markers[1].exits[W], ExitStatus.BLOCKED)


def line_graph():
    g = MarkerGraph()
    g.start(0)
    g.marker_reached(1, E)
    g.marker_reached(2, E)
    return g


class TestPlanner(unittest.TestCase):
    def test_simple_and_same_node(self):
        g = line_graph()
        self.assertEqual(plan_route(g, 0, 2), [0, 1, 2])
        self.assertEqual(plan_route(g, 1, 1), [1])

    def test_unknown_or_unreachable(self):
        g = line_graph()
        self.assertIsNone(plan_route(g, 0, 99))
        g.mark_blocked(1, E)
        self.assertIsNone(plan_route(g, 0, 2))

    def test_unexplored_edge_is_not_used(self):
        # A square 0-1-2-3 where the closing edge 3-0 was never travelled
        g = MarkerGraph(); g.start(0)
        g.marker_reached(1, E); g.marker_reached(2, N); g.marker_reached(3, W)
        self.assertEqual(plan_route(g, 3, 0, use_probable=False), [3, 2, 1, 0])   # long way round
        g.marker_reached(0, S)                                      # now travelled
        self.assertEqual(plan_route(g, 3, 0, use_probable=False), [3, 0])

    def _square_without_closing_edge(self):
        g = MarkerGraph(); g.start(0)
        g.marker_reached(1, E); g.marker_reached(2, N); g.marker_reached(3, W)
        return g

    def test_probable_link_is_inferred_from_grid_adjacency(self):
        g = self._square_without_closing_edge()
        self.assertEqual(g.probable_links(), [(0, 3, False)])
        self.assertEqual(g.direction_between(3, 0), S)
        self.assertEqual(plan_route(g, 3, 0), [3, 0])               # short cut over the guessed edge

    def test_probable_link_is_likely_when_tof_saw_it_open(self):
        g = self._square_without_closing_edge()
        g.set_exit_status(3, S, ExitStatus.UNEXPLORED)
        self.assertEqual(g.probable_links(), [(0, 3, True)])

    def test_wall_on_either_side_removes_probable_link(self):
        g = self._square_without_closing_edge()
        g.set_exit_status(0, N, ExitStatus.BLOCKED)
        self.assertEqual(g.probable_links(), [])
        self.assertEqual(plan_route(g, 3, 0), [3, 2, 1, 0])

    def test_blocked_report_on_probable_edge_kills_it_for_both_ends(self):
        g = self._square_without_closing_edge()
        g.mark_blocked(3, S)
        self.assertEqual(g.probable_links(), [])
        self.assertEqual(g.markers[0].exits[N], ExitStatus.BLOCKED)

    def test_explored_route_preferred_over_equal_length_probable(self):
        g = self._square_without_closing_edge()
        g.marker_reached(0, S)                                      # closing edge now explored
        self.assertEqual(g.probable_links(), [])
        self.assertEqual(plan_route(g, 3, 0), [3, 0])

    def test_probable_penalty_prefers_explored_path_when_hops_tie(self):
        # 2x2 block 0(0,0) 1(1,0) 2(1,1) 3(0,1); explored 0-1-2 and 0-3 is probable; 0->2 is 2 hops either way
        g = MarkerGraph(); g.start(0)
        g.marker_reached(1, E); g.marker_reached(2, N)
        g.markers[3] = __import__("mapping.graph", fromlist=["Marker"]).Marker(3, (0, 1)); g.neighbours[3] = {}
        g.link(2, W, 3)        # 3 reached from 2; 0-3 stays probable
        self.assertEqual(plan_route(g, 0, 2), [0, 1, 2])

    def _grid_2x2_loop(self):
        g = MarkerGraph(); g.start(0)
        g.marker_reached(1, E); g.marker_reached(3, N)      # 0(0,0) 1(1,0) 3(1,1)
        g.marker_reached(2, W); g.marker_reached(0, S)      # 2(0,1) closes loop to 0
        return g

    def test_equal_hops_prefers_fewest_turns_then_lowest_id(self):
        g = self._grid_2x2_loop()
        # 0 -> 3 is 2 hops both ways (via 1 or via 2). Via 1: E then N (1 turn).
        # Via 2: N then E (1 turn). Equal -> lowest-ID path wins: [0,1,3].
        self.assertEqual(plan_route(g, 0, 3), [0, 1, 3])
        # Heading north at 0 already: via 2 means going straight first (0 turns), so it wins.
        self.assertEqual(plan_route(g, 0, 3, start_heading=N), [0, 2, 3])
        self.assertEqual(plan_route(g, 0, 3, start_heading=E), [0, 1, 3])

    def test_straight_beats_zigzag_with_same_hops(self):
        # 0-1-2 along x; 0-3-4-2 is longer: hops decide first
        g = MarkerGraph(); g.start(0)
        g.marker_reached(1, E); g.marker_reached(2, E)
        self.assertEqual(plan_route(g, 0, 2), [0, 1, 2])


if __name__ == "__main__":
    unittest.main()
