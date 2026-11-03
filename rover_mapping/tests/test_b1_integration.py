"""End to end with the REAL laptop DFS / navigator from the dashboard folder:
simulated 6x6 maze -> RoverPhysics -> LaptopNavigator + DFSExplorer -> DfsAdapter -> my controller,
then Phase 2 through RouteRunner. No hardware, no broker, no Tk."""
import copy
import unittest

from tests import _paths  # noqa: F401  (puts dashboard/ on sys.path)
from laptop_navigation import LaptopNavigator
from dfs_explorer import State as DState

from mapping.bridge import MappingBridge
from mapping.controller import State
from mapping.dfs_adapter import DfsAdapter
from mapping.planner import plan_route
from mapping.pose import PoseFeed
from mapping.route_exec import RouteRunner, RouteExplorer
from sim.maze import Maze
from sim.rover_physics import RoverPhysics

WALL0 = 1_700_000_000.0


class Rig:
    """One simulated rover wired exactly like the dashboard wires it (100 ms ticks)."""

    def __init__(self, maze, start_node=0, heading=0.0, occupancy=False, **phys_kw):
        self.now = 0.0
        self.commands = []
        self.phys = RoverPhysics(maze, start_node, heading, **phys_kw)
        self.notes = []
        self.bridge = MappingBridge(notify=self.notes.append)
        self.adapter = DfsAdapter(self.bridge)
        self.runner = RouteRunner(self.bridge)

        def send(cmd):
            self.commands.append(cmd)
            self.phys.command(cmd)
            return True
        self.nav = LaptopNavigator(send, clock=lambda: self.now, wall_clock=lambda: WALL0 + self.now)
        self.sync = self._sync_default           # tests may swap in MappingPanel.sync
        self.pose_feed = None
        if occupancy:
            self.pose_feed = PoseFeed(self.nav, self.bridge)
            self.pose_feed.attach()

    def _sync_default(self):
        self.runner.sync(self.nav)
        if not isinstance(self.nav.explorer, RouteExplorer):
            self.adapter.sync(self.nav.explorer)

    def step(self):
        self.now += 0.1
        self.phys.step(0.1)
        self.nav.sensor_received(self.phys.telemetry())
        self.nav.vision_received(self.phys.camera_frame((WALL0 + self.now) * 1000))
        self.nav.tick()
        self.sync()

    def run(self, until, limit=40000):
        for _ in range(limit):
            if until():
                return True
            self.step()
        return False

    def start_dfs(self, n, threshold=500):
        for _ in range(self.nav.ARRIVAL_CONFIRM_FRAMES):
            self.step()
        self.nav.start(n, threshold)


def maze36(seed=3, extra=8):
    return Maze.random(6, 6, extra_links=extra, seed=seed)


class Phase1(unittest.TestCase):
    def test_dfs_maps_the_whole_maze_and_my_controller_agrees(self):
        maze = maze36()
        rig = Rig(maze)
        rig.start_dfs(36)
        self.assertTrue(rig.run(lambda: rig.nav.explorer.state == DState.COMPLETE),
                        rig.nav.status)
        rig.step(); rig.step()
        snap = rig.bridge.snapshot()
        self.assertEqual(rig.bridge.ctrl.state, State.MAP_COMPLETE)
        self.assertEqual(snap["count"], 36)
        self.assertEqual(snap["inconsistencies"], [])
        # every link my graph holds really exists in the maze (ids are node + 1)
        for a, b, blocked in snap["edges"]:
            self.assertIn(frozenset((a - 1, b - 1)), maze.links)
        # and my positions match the maze's true geometry
        for m in snap["markers"]:
            self.assertEqual((m["x"], m["y"]), maze.pos[m["id"] - 1])

    def test_new_mission_restarts_the_map(self):
        rig = Rig(maze36())
        rig.start_dfs(6)
        rig.run(lambda: rig.nav.explorer.state == DState.COMPLETE)
        rig.step()
        self.assertEqual(rig.bridge.ctrl.state, State.MAP_COMPLETE)
        rig.nav.stop("operator")                       # her toggle starts a fresh mission next time
        rig.start_dfs(3)
        rig.run(lambda: rig.bridge.ctrl.n == 3)
        self.assertEqual(rig.bridge.ctrl.n, 3)

    def test_stop_mid_mission_pauses_the_map(self):
        rig = Rig(maze36())
        rig.start_dfs(36)
        rig.run(lambda: rig.bridge.snapshot()["count"] >= 4)
        rig.nav.stop("Emergency stop")
        rig.step(); rig.step()
        self.assertEqual(rig.bridge.ctrl.state, State.PAUSED)


class Phase2Base(unittest.TestCase):
    EXTRA_LINKS = 8

    def setUp(self):
        self.maze = maze36(extra=self.EXTRA_LINKS)
        self.rig = Rig(self.maze)
        self.rig.start_dfs(36)
        self.rig.run(lambda: self.rig.nav.explorer.state == DState.COMPLETE)
        self.rig.run(lambda: not self.rig.nav.active)
        self.rig.step()
        self.assertEqual(self.rig.bridge.ctrl.state, State.MAP_COMPLETE)

    def drive_to(self, target):
        rig = self.rig
        for _ in range(rig.nav.ARRIVAL_CONFIRM_FRAMES):          # camera re-confirms the marker
            rig.step()
        rig.runner.begin(rig.nav, target)
        self.assertTrue(rig.run(lambda: not rig.runner.active, limit=60000), rig.nav.status)



class Phase2(Phase2Base):
    def test_route_reaches_target_and_restores_dfs_explorer(self):
        rig = self.rig
        here = rig.bridge.ctrl.graph.current
        target = 1 if here != 1 else 36
        self.drive_to(target)
        self.assertEqual(rig.bridge.ctrl.state, State.MAP_COMPLETE)
        self.assertEqual(rig.bridge.snapshot()["last_result"], "REACHED")
        self.assertEqual(rig.bridge.ctrl.graph.current, target)
        self.assertEqual(rig.phys.marker_id(rig.phys.at), target)     # the rover really is there
        self.assertNotIsInstance(rig.nav.explorer, RouteExplorer)     # DFS explorer is back

    def test_two_routes_in_a_row(self):
        for target in (1, 36, 18):
            if self.rig.bridge.ctrl.graph.current == target:
                continue
            self.drive_to(target)
            self.assertEqual(self.rig.phys.marker_id(self.rig.phys.at), target)

    def test_route_refused_if_camera_disagrees_with_map(self):
        rig = self.rig
        rig.bridge.ctrl.graph.current = 5 if rig.bridge.ctrl.graph.current != 5 else 6
        for _ in range(5):
            rig.step()
        with self.assertRaises(ValueError):
            rig.runner.begin(rig.nav, 1)
        self.assertNotIsInstance(rig.nav.explorer, RouteExplorer)     # nothing left half-swapped

    def test_stop_during_route_restores_everything(self):
        rig = self.rig
        for _ in range(5):
            rig.step()
        far = 1 if rig.bridge.ctrl.graph.current != 1 else 36
        rig.runner.begin(rig.nav, far)
        rig.run(lambda: rig.phys.drive is not None)
        rig.nav.stop("Emergency stop")
        rig.step(); rig.step()
        self.assertFalse(rig.runner.active)
        self.assertNotIsInstance(rig.nav.explorer, RouteExplorer)
        self.assertEqual(rig.bridge.ctrl.state, State.MAP_COMPLETE)   # route cancelled, map kept


class Phase2Blockage(Phase2Base):
    """An obstacle appears in a corridor mid-route. Her sensor stop request must become: stop, mark that edge
    blocked, back out to the source marker, re-plan (or report NO_ROUTE and stay at the marker)."""

    def _first_hop(self, target):
        g = self.rig.bridge.ctrl.graph
        src = g.current
        path = plan_route(g, src, target, self.rig.bridge.ctrl.heading)
        return src, path[1], path

    def _target_with_alternative(self):
        """(source, first hop, target): a target whose first hop can be avoided by another driven route."""
        g = self.rig.bridge.ctrl.graph
        for target in sorted(g.markers):
            if target == g.current:
                continue
            src, nxt, path = self._first_hop(target)
            trial = copy.deepcopy(g)
            trial.mark_blocked(src, trial.direction_between(src, nxt))
            if plan_route(trial, src, target, use_probable=False) is not None and len(path) > 2:
                return src, nxt, target
        self.fail("no suitable target in this maze")

    def _arm(self, src, nxt, at_mm=300):
        self.rig.phys.obstacles[frozenset((src - 1, nxt - 1))] = at_mm   # physics nodes are marker ids - 1

    def _count_blocked_calls(self):
        calls = []
        original = RouteExplorer.segment_blocked
        def spy(me, *a, **k):
            r = original(me, *a, **k)
            calls.append((a, k, r))
            return r
        RouteExplorer.segment_blocked = spy
        self.addCleanup(setattr, RouteExplorer, "segment_blocked", original)
        return calls

    def test_obstacle_midway_backs_out_replans_and_still_arrives(self):
        rig = self.rig
        for _ in range(5):
            rig.step()
        g = rig.bridge.ctrl.graph
        src, nxt, target = self._target_with_alternative()
        calls = self._count_blocked_calls()
        self._arm(src, nxt)
        for _ in range(5):
            rig.step()
        rig.runner.begin(rig.nav, target)
        self.assertTrue(rig.run(lambda: not rig.runner.active, limit=80000), rig.nav.status)
        self.assertEqual([c[2] for c in calls], [True])                # reported exactly once, accepted
        self.assertIn(frozenset((src, nxt)), g.blocked)                # that edge, both ways, not a random one
        self.assertEqual(rig.bridge.snapshot()["last_result"], "REACHED")
        self.assertEqual(rig.phys.marker_id(rig.phys.at), target)
        self.assertEqual(g.current, target)
        self.assertNotIsInstance(rig.nav.explorer, RouteExplorer)

    def test_no_alternative_gives_no_route_and_rover_waits_at_a_confirmed_marker(self):
        maze = maze36(extra=0)                              # a tree: plenty of dead ends
        rig = self.rig = Rig(maze)
        rig.start_dfs(36)
        rig.run(lambda: rig.nav.explorer.state == DState.COMPLETE)
        rig.run(lambda: not rig.nav.active)
        rig.step()
        for _ in range(5):
            rig.step()
        g = rig.bridge.ctrl.graph
        dead_ends = [m for m in g.markers if len(g.neighbours[m]) == 1 and not g.probable_from(m)
                     and m != g.current]
        self.assertTrue(dead_ends, "maze has no dead end")
        target = dead_ends[0]
        (only,) = g.neighbours[target].values()
        self._arm(only, target, at_mm=250)
        rig.runner.begin(rig.nav, target)
        self.assertTrue(rig.run(lambda: not rig.runner.active, limit=80000), rig.nav.status)
        self.assertEqual(rig.bridge.snapshot()["last_result"], "NO_ROUTE")
        self.assertEqual(rig.bridge.ctrl.state, State.MAP_COMPLETE)
        self.assertEqual(rig.phys.marker_id(rig.phys.at), only)        # backed out to the marker before it
        self.assertEqual(g.current, only)
        self.assertTrue(rig.bridge.ctrl.position_confirmed)
        self.assertFalse(rig.nav.active)

    def test_segment_blocked_rejects_the_wrong_segment_or_session(self):
        rig = self.rig
        for _ in range(5):
            rig.step()
        far = 1 if rig.bridge.ctrl.graph.current != 1 else 36
        rig.runner.begin(rig.nav, far)
        rig.run(lambda: rig.nav.explorer.pending is not None)
        ex = rig.nav.explorer
        seg = ex.pending
        self.assertFalse(ex.segment_blocked(ex.session_id, seg.segment_id + 1, rig.now))
        self.assertFalse(ex.segment_blocked(ex.session_id + 1, seg.segment_id, rig.now))
        self.assertIs(ex.state, DState.MOVING)                          # nothing changed

    def test_obstacle_before_leaving_the_marker_replans_without_backing_out(self):
        rig = self.rig
        for _ in range(5):
            rig.step()
        src, nxt, target = self._target_with_alternative()
        calls = self._count_blocked_calls()
        stop_flag = {"on": False}
        real = rig.phys.telemetry
        def telemetry():                                    # an obstacle 10 cm in front while still at the marker
            t = real()
            if stop_flag["on"]:
                t["obstacle"].update(stop_required=True, urgent_mask=2, blocked_mask=2)
                t["obstacle"]["distance_mm"]["front"] = 100
            return t
        rig.phys.telemetry = telemetry
        rig.runner.begin(rig.nav, target)
        self.assertTrue(rig.run(lambda: rig.nav.phase in ("turn", "settle") and rig.nav.explorer.pending is not None))
        issued = []
        explorer = rig.nav.explorer
        stop_flag["on"] = True
        rig.run(lambda: calls, limit=200)
        stop_flag["on"] = False                             # (moving back would not clear a real obstacle; this
        self.assertEqual(len(calls), 1)                     #  model only needs it gone for the re-plan)
        self.assertIs(calls[0][1]["moved"], False)
        self.assertIsNone(explorer._return)                 # no back-out: it never left
        self.assertTrue(rig.run(lambda: not rig.runner.active, limit=80000), rig.nav.status)
        self.assertEqual(rig.bridge.snapshot()["last_result"], "REACHED")
        self.assertEqual(rig.phys.marker_id(rig.phys.at), target)
        self.assertIn(frozenset((src, nxt)), rig.bridge.ctrl.graph.blocked)


if __name__ == "__main__":
    unittest.main()
