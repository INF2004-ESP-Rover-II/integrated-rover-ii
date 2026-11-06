"""Grid-localisation mode (teammates' main): Phase 1 DFS then a Phase 2 route, on the 500 mm / TURN_TO simulator."""
import os
import sys
import unittest

from tests import _paths  # noqa: F401

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from dfs_explorer import State as DState  # noqa: E402
from laptop_navigation import LaptopNavigator  # noqa: E402
from mapping.bridge import MappingBridge  # noqa: E402
from mapping.dfs_adapter import DfsAdapter  # noqa: E402
from mapping.pose import PoseFeed  # noqa: E402
from mapping.route_exec import RouteExplorer, RouteRunner  # noqa: E402
from sim.grid_rover import GridRover  # noqa: E402
from sim.maze import Maze  # noqa: E402
from sim.world import World  # noqa: E402

WALL0 = 1.8e9


class StallingRover(GridRover):
    """Wheels stop turning after `stall_after` s of driving until a STOP is received (what the real rover does)."""
    stall_after = 2.0

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.driven = 0.0

    def command(self, text):
        if text.split(":")[0] != "FORWARD":
            self.driven = 0.0
        super().command(text)

    def step(self, dt):
        if self.cmd == "FORWARD":
            self.driven += dt
            if self.driven > self.stall_after:
                return
        super().step(dt)


class GridModeTest(unittest.TestCase):
    def test_route_after_stop_without_camera_confirmation(self):
        """Operator stop / Finish mapping leaves position unconfirmed; Go to target must still work."""
        self._run(8, finish_early_at=None, unconfirm=True)

    def test_recovers_from_stalled_wheels(self):
        self._run(8, finish_early_at=None, rover_cls=StallingRover)

    def test_phase1_then_route(self):
        self._run(8, finish_early_at=None)

    def test_finish_early_then_route(self):
        """Target count left at 36 but only 5 markers wanted: Finish mapping makes Go to target usable."""
        self._run(36, finish_early_at=5)

    def test_restart_after_fault_keeps_map(self):
        from mapping.controller import State as MState
        bridge = MappingBridge(notify=lambda m: None)
        self.assertTrue(bridge.begin_session(36, 1))
        from mapping import events as ev
        bridge.on_event(ev.MarkerReached(2, 1))
        bridge.on_event(ev.MarkerReached(3, 1))
        bridge.fault()
        self.assertEqual(bridge.ctrl.state, MState.PAUSED)
        self.assertTrue(bridge.begin_session(36, 3))          # DFS restarted where the rover stands
        self.assertEqual(bridge.snapshot()["count"], 3)
        self.assertEqual(bridge.ctrl.state, MState.MAPPING)
        bridge.stop()
        self.assertTrue(bridge.begin_session(36, 9))          # unknown marker -> fresh map
        self.assertEqual(bridge.snapshot()["count"], 1)

    def _run(self, n, finish_early_at, rover_cls=GridRover, unconfirm=False):
        os.environ["SIM_FOV_DEG"] = "20"
        maze = Maze.random(6, 6, extra_links=8, seed=3)
        rov = rover_cls(maze, 0, World(maze))
        now = [0.0]

        def send(cmd):
            rov.command(cmd)
            nav.motion_received(cmd)
            return True

        nav = LaptopNavigator(send, clock=lambda: now[0], wall_clock=lambda: WALL0 + now[0])
        bridge = MappingBridge(notify=lambda m: None)
        adapter = DfsAdapter(bridge)
        runner = RouteRunner(bridge)
        PoseFeed(nav, bridge).attach()

        def step():
            now[0] += 0.1
            rov.step(0.1)
            nav.sensor_received(rov.telemetry())
            nav.vision_received(rov.camera_frame((WALL0 + now[0]) * 1000))
            nav.tick()
            runner.sync(nav)
            if not isinstance(nav.explorer, RouteExplorer):
                adapter.sync(nav.explorer)

        for _ in range(10):
            step()
        nav.grid_required = True
        nav.configure_grid_start(1)
        for _ in range(10):
            step()
        nav.start(n, 500)
        for _ in range(30000):
            step()
            if finish_early_at and bridge.snapshot()["count"] >= finish_early_at:
                self.assertTrue(bridge.finish_early())
                nav.stop("test finish")
                break
            if nav.explorer.state == DState.COMPLETE or not nav.active:
                break
        for _ in range(20):
            step()
        snap = bridge.snapshot()
        if not finish_early_at:
            self.assertEqual(snap["count"], len(nav.explorer.nodes))
        self.assertEqual(snap["state"], "MAP_COMPLETE")
        # route to the most recently discovered marker that is not where the rover stands
        cur = snap["current"]
        target = [m["id"] for m in snap["markers"] if m["id"] != cur][-1]
        if unconfirm:
            bridge.ctrl.position_confirmed = False
        runner.begin(nav, target, 500)
        for _ in range(30000):
            step()
            if not runner.active:
                break
        self.assertEqual(bridge.snapshot()["last_result"], "REACHED")


if __name__ == "__main__":
    unittest.main()
