"""Occupancy grid: unit tests, then an end-to-end run of the real laptop DFS in a geometric simulation
(walls from the maze, small box obstacles beside the corridors) with the grid scored against truth."""
import math
import unittest

from tests import _paths  # noqa: F401
from dfs_explorer import State as DState

from mapping import config as cfg
from mapping.occupancy import OccupancyGrid, UNKNOWN, FREE, OCCUPIED
from tests.test_b1_integration import Rig, maze36
from sim.world import World, add_corridor_boxes


class GridUnit(unittest.TestCase):
    def test_front_wall_marks_cells_free_then_occupied(self):
        g = OccupancyGrid()
        for _ in range(3):
            g.integrate((0, 0, 0), {"front": 500})                 # wall 50 cm to the north
        self.assertEqual(g.state((0, 10)), OCCUPIED)               # y = 50..55 cm
        self.assertEqual(g.state((0, 4)), FREE)
        self.assertEqual(g.state((0, 12)), UNKNOWN)                # behind the wall is not known

    def test_heading_rotates_the_rays(self):
        g = OccupancyGrid()
        for _ in range(3):
            g.integrate((0, 0, 90), {"front": 300, "left": 300, "right": 300})
        self.assertEqual(g.state((6, 0)), OCCUPIED)                # front -> east
        self.assertEqual(g.state((0, 6)), OCCUPIED)                # left of east = north
        self.assertEqual(g.state((0, -6)), OCCUPIED)               # right of east = south

    def test_unknown_codes_and_faults_change_nothing(self):
        g = OccupancyGrid()
        used = g.integrate((0, 0, 0), {"front": 1023, "left": 0, "right": 1000})
        self.assertEqual(used, 0)
        self.assertEqual(g.cells, {})
        self.assertEqual(g.integrate((0, 0, 0), {"front": 300, "left": 300, "right": 300}, fault_mask=7), 0)
        self.assertEqual(g.cells, {})
        self.assertEqual(g.integrate((0, 0, 0), {"front": 65535, "left": True, "right": None}), 0)

    def test_a_single_stray_reading_does_not_become_an_obstacle_of_certainty(self):
        g = OccupancyGrid()
        g.integrate((0, 0, 0), {"front": 400})
        for _ in range(6):
            g.integrate((0, 0, 0), {"front": 800})                 # later the way is clear
        self.assertNotEqual(g.state((0, 8)), OCCUPIED)

    def test_clear_and_render(self):
        g = OccupancyGrid()
        self.assertIsNone(g.render())
        g.integrate((0, 0, 0), {"front": 300})
        g.integrate((0, 0, 0), {"front": 300})
        img, ext = g.render()
        self.assertEqual(img.shape[2], 4)
        self.assertEqual(ext[2], 0)
        g.clear()
        self.assertEqual(g.stats(), {"occupied": 0, "free": 0, "samples": 0})


def _boxes(world, maze, count=10, seed=5):
    return add_corridor_boxes(world, maze, count, seed)


def _score(grid, world, boxes):
    truth = world.occupied_cells(grid.cell)
    occ = grid.occupied_cells()
    near = lambda c: any((c[0] + i, c[1] + j) in truth for i in (-1, 0, 1) for j in (-1, 0, 1))
    precision = sum(near(c) for c in occ) / max(1, len(occ))
    found = 0
    for x0, y0, x1, y1 in boxes:
        cells = {(i, j) for i in range(math.floor(x0 / 5) - 1, math.floor(x1 / 5) + 2)
                 for j in range(math.floor(y0 / 5) - 1, math.floor(y1 / 5) + 2)}
        found += bool(cells & occ)
    return precision, found


import contextlib


@contextlib.contextmanager
def sim_camera(standoff_cm):
    """The simulated camera sits on the floor (range = floor distance), so the lens height is 0 here."""
    saved = cfg.STANDOFF_CM, cfg.CAMERA_HEIGHT_CM
    cfg.STANDOFF_CM, cfg.CAMERA_HEIGHT_CM = standoff_cm, 0.0
    try:
        yield
    finally:
        cfg.STANDOFF_CM, cfg.CAMERA_HEIGHT_CM = saved


def _map_with_obstacles(config_standoff, true_standoff=0.0):
    with sim_camera(config_standoff):
        maze = maze36(3)
        world = World(maze)
        boxes = _boxes(world, maze)
        rig = Rig(maze, occupancy=True, world=world, standoff_cm=true_standoff, no_target_code=1000)
        rig.start_dfs(36)
        assert rig.run(lambda: rig.nav.explorer.state == DState.COMPLETE, limit=80000), rig.nav.status
        return rig, world, boxes


class SimulatedObstacles(unittest.TestCase):
    """Geometric simulation: ToF values come from ray casts against real wall/box geometry.
    (No-target code 1000 here: with the spec's 1023-as-unknown Buddy 1's DFS does not get past the first
    marker in simulation, see MAPPING_INTEGRATION.md. The occupancy grid itself ignores both codes.)"""

    def test_grid_finds_the_boxes_and_walls_with_the_right_standoff(self):
        rig, world, boxes = _map_with_obstacles(config_standoff=0.0, true_standoff=0.0)
        precision, found = _score(rig.bridge.grid, world, boxes)
        self.assertGreaterEqual(precision, 0.95, "occupied cells that are not near any real wall/box")
        self.assertGreaterEqual(found, 9, "boxes with at least one occupied cell on them")
        self.assertGreater(rig.bridge.grid.stats()["free"], 1000)

    def test_wrong_standoff_constant_visibly_smears_the_map(self):
        """The 45 cm stop-short distance matters: if the constant disagrees with the rover, walls land in the
        wrong place. This is why STANDOFF_CM must be measured on the real rover."""
        rig, world, boxes = _map_with_obstacles(config_standoff=45.0, true_standoff=0.0)
        precision, found = _score(rig.bridge.grid, world, boxes)
        self.assertLess(precision, 0.85)

    def test_new_mission_clears_the_grid(self):
        rig, world, boxes = _map_with_obstacles(config_standoff=0.0)
        self.assertGreater(len(rig.bridge.grid.cells), 0)
        rig.bridge.reset_map()
        self.assertEqual(len(rig.bridge.grid.cells), 0)


class CameraGeometry(unittest.TestCase):
    def test_ground_distance_removes_the_lens_height(self):
        self.assertAlmostEqual(cfg.camera_ground_cm(450), 31.1, delta=0.1)      # 450 mm slant, 32.5 cm up
        self.assertAlmostEqual(cfg.camera_ground_cm(682), 60.0, delta=0.2)      # a marker 60 cm away on the floor
        self.assertEqual(cfg.camera_ground_cm(100), 0.0)                         # closer than the lens height
        self.assertAlmostEqual(cfg.STANDOFF_CM, 43.1, delta=0.2)                 # 31.1 + 12 cm camera offset

    def test_tof_forward_offset_moves_the_rays(self):
        from mapping.pose import PoseFeed
        rig = Rig(maze36(3), occupancy=True)
        saved = cfg.TOF_FORWARD_CM
        cfg.TOF_FORWARD_CM = 10.0
        try:
            rig.pose_feed.tracker.pose = lambda nav: (0.0, 0.0, 90.0)           # at the origin facing east
            from types import SimpleNamespace
            for _ in range(2):
                rig.pose_feed.on_sample(SimpleNamespace(distances={"front": 300, "left": 0, "right": 0}, faults=0))
        finally:
            cfg.TOF_FORWARD_CM = saved
        grid = rig.bridge.grid
        self.assertEqual(grid.state((8, 0)), OCCUPIED)       # wall at x = 10 + 30 = 40 cm  -> cell 8


class PoseWithStandoff(unittest.TestCase):
    def test_pose_matches_the_simulated_rover_with_a_45cm_standoff(self):
        maze = maze36(3)
        world = World(maze)
        with sim_camera(45.0):
            rig = Rig(maze, occupancy=True, world=world, standoff_cm=45.0, no_target_code=1000)
            rig.start_dfs(36)
            worst, n = 0.0, 0
            for _ in range(3000):
                rig.step()
                pose = rig.pose_feed.tracker.pose(rig.nav)
                if pose is None or not rig.nav.active:
                    continue
                tx, ty = rig.phys.xy_cm()
                worst, n = max(worst, math.hypot(pose[0] - tx, pose[1] - ty)), n + 1
        self.assertGreater(n, 20)   # the DFS faults at a 45 cm standoff after a hop or two (see docs)
        self.assertLess(worst, 8.0, f"pose off by up to {worst:.1f} cm over {n} samples")


if __name__ == "__main__":
    unittest.main()
