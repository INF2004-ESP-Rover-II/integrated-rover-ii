"""One LAPTOP DFS click through real arrivals, branch exploration and backtracking."""
import ast
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dashboard"))
from dfs_explorer import Direction as D, MoveKind, State
from laptop_navigation import LaptopNavigator

source = ROOT / "dashboard/robot_dashboard.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
gui_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RobotGUI")
methods = [n for n in gui_class.body if isinstance(n, ast.FunctionDef) and
           n.name in ("toggle_auto_mode", "navigation_tick")]
namespace = {"time": time, "TOF_UART_STALE_MS": 500}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.frame = self.stamp = 0
        self.at = 1
        self.heading = 0
        self.commands, self.messages, self.draws = [], [], []
        self.graph = {1: {D.NORTH, D.EAST}, 2: {D.NORTH, D.SOUTH},
                      3: {D.SOUTH}, 4: {D.WEST}}
        self.nav = LaptopNavigator(self.send, self.messages.append, clock=lambda: self.now,
                                   wall_clock=lambda: 1790000000 + self.now)
        self.gui = SimpleNamespace(navigator=self.nav, auto_mode=Value(False),
            dfs_target=Value("4"), dfs_threshold=Value("500"),
            stop_hold=lambda: None, cancel_heartbeat=lambda: None, log=self.messages.append,
            front_obstacle_snapshot=None, last_tof_rx=None, set_front_blocked=lambda blocked: None,
            update_current_marker=lambda: None,
            mapping=SimpleNamespace(sync=lambda: self.draws.append(self.nav.explorer.current)),
            root=SimpleNamespace(after=lambda *args: None))
        self.gui.navigation_tick = self.tick

    def send(self, command):
        self.commands.append(command)
        return True

    def inputs(self, marker=None, distance=500, seconds=0.05, fault=0, front=None):
        self.now += seconds
        self.frame += 1
        self.stamp += 1
        heading = D(round(self.heading / 90) % 4)
        ranges = {name: 700 if D((heading + offset) % 4) in self.graph[self.at] else 200
                  for name, offset in (("left", 3), ("front", 0), ("right", 1))}
        if front is not None:
            ranges["front"] = front
        self.nav.sensor_received({"timestamp": self.stamp,
            "imu": {"status": 0, "heading": self.heading},
            "obstacle": {"age_ms": 0, "distance_mm": ranges, "fault_mask": fault,
                         "stop_required": 0 < ranges['front'] < 150}})
        self.nav.marker_seen(marker, self.frame, distance_mm=distance, bearing_deg=0)

    def tick(self):
        namespace["navigation_tick"](self.gui)

    def click_start(self, target=4, front=None):
        self.gui.dfs_target.set(str(target))
        for _ in range(5):
            self.inputs(1, front=front)
        self.gui.auto_mode.set(True)
        namespace["toggle_auto_mode"](self.gui)
        self.assertTrue(self.nav.active)
        self.inputs(1, seconds=0.41, front=front)
        self.tick()

    def drive_selected_segment(self):
        segment = self.nav.explorer.pending
        self.assertIsNotNone(segment)
        self.assertEqual(self.nav.phase, "turn")
        self.inputs()
        self.tick()
        target = int(segment.direction) * 90
        if self.nav.phase == 'turn':
            self.assertEqual(self.commands[-1], "TURN_TO:%d" % target)
            self.nav.robot_status_received("AUTO_MOVE: TURN_TO:%d" % target)
            self.heading = target
            self.inputs()
            self.nav.robot_status_received("AUTO_RESULT: HEADING_REACHED")
            self.tick()
        self.assertEqual(self.nav.phase, "settle")
        self.inputs(seconds=0.41)
        self.tick()
        self.assertEqual(self.nav.phase, "drive")
        self.assertEqual(self.commands[-1], "FORWARD")
        return segment

    def arrive(self, marker):
        self.inputs(marker, distance=700)
        self.tick()
        self.assertEqual(self.nav.phase, "drive")  # Seeing an ID is not arrival.
        self.at = marker
        for _ in range(5):
            self.inputs(marker)
            self.tick()
        self.assertEqual(self.nav.phase, "arrival_settle")
        self.assertEqual(self.commands[-1], "STOP")
        old_current = self.nav.explorer.current
        self.inputs(marker, seconds=0.41)
        self.tick()
        self.assertEqual(self.nav.explorer.current, marker)
        self.assertNotEqual(old_current, marker)
        if self.nav.active:
            self.assertTrue(self.gui.auto_mode.get())
            self.inputs(marker)
            self.tick()  # Select the next DFS branch without touching the checkbox.

    def test_one_click_explores_backtracks_and_continues_to_next_branch(self):
        self.click_start()
        actions = []
        for marker in (2, 3, 2, 1, 4):
            segment = self.drive_selected_segment()
            actions.append((segment.from_marker, segment.kind, segment.direction))
            self.arrive(marker)
        self.assertEqual(actions, [(1, MoveKind.EXPLORE, D.NORTH),
            (2, MoveKind.EXPLORE, D.NORTH), (3, MoveKind.BACKTRACK, D.SOUTH),
            (2, MoveKind.BACKTRACK, D.SOUTH), (1, MoveKind.EXPLORE, D.EAST)])
        self.assertEqual(self.nav.explorer.state, State.COMPLETE)
        self.assertEqual(self.nav.explorer.nodes[1].neighbors, {D.NORTH: 2, D.EAST: 4})
        self.assertFalse(self.nav.active)
        self.assertFalse(self.gui.auto_mode.get())
        self.assertEqual(self.commands[-1], "STOP")
        self.assertGreater(len(self.draws), 5)

    def test_front_obstacle_at_stationary_start_still_allows_other_exits(self):
        self.graph[1] = {D.WEST}
        self.click_start(front=100)
        self.assertEqual(self.nav.explorer.pending.direction, D.WEST)
        self.assertNotIn("FORWARD", self.commands)

    def test_stops_when_reachable_branches_are_exhausted_before_target_count(self):
        self.click_start(target=36)
        for marker in (2, 3, 2, 1, 4, 1):
            self.drive_selected_segment()
            self.arrive(marker)
        self.assertEqual(self.nav.explorer.state, State.EXHAUSTED)
        self.assertEqual(len(self.nav.explorer.nodes), 4)
        self.assertFalse(self.gui.auto_mode.get())
        self.assertEqual(self.commands[-1], "STOP")

    def test_loop_returns_to_previous_branch_without_counting_repeat_markers(self):
        self.graph = {1: {D.NORTH, D.EAST}, 2: {D.SOUTH, D.EAST},
                      3: {D.WEST, D.SOUTH}, 4: {D.NORTH, D.WEST}}
        self.click_start(target=36)
        kinds = []
        for marker in (2, 3, 4, 1, 4, 3, 2, 1):
            kinds.append(self.drive_selected_segment().kind)
            self.arrive(marker)
        self.assertIn(MoveKind.LOOP_RETURN, kinds)
        self.assertEqual(self.nav.explorer.state, State.EXHAUSTED)
        self.assertEqual(len(self.nav.explorer.nodes), 4)
        self.assertEqual(self.nav.explorer.stack, [1])

    def test_camera_confirmation_stays_on_arrival_marker_when_side_marker_appears(self):
        self.click_start()
        self.drive_selected_segment()
        self.at = 2
        self.inputs(2)
        self.assertEqual(self.nav.phase, "arrival_hold")
        for seq in range(1, 5):
            self.now += 0.05
            self.nav.vision_received({"schema": 1, "source_session_id": "camera",
                "frame_seq": seq, "capture_time_ms": int(self.nav.wall_clock() * 1000),
                "capture_time_source": "laptop_receive", "last_frame_age_ms": 0, "status": "ok",
                "markers": [{"marker_id": marker, "valid": True, "distance_ref": "camera_lens",
                             "distance_mm": distance, "bearing_deg": 0}
                            for marker, distance in ((3, 480), (2, 500))]})
        self.assertEqual(self.nav.phase, "arrival_settle")
        self.assertEqual(self.nav.explorer.current, 1)  # Braking not yet complete.
        self.frame = self.nav.last_frame
        self.inputs(seconds=0.41)
        self.tick()
        self.assertEqual(self.nav.explorer.current, 2)
        self.assertNotIn(3, self.nav.explorer.nodes)

    def test_source_marker_flicker_during_travel_does_not_restart_or_finish_mission(self):
        self.click_start()
        self.drive_selected_segment()
        for _ in range(6):
            self.inputs(1)
            self.tick()
        self.assertEqual(self.nav.phase, "drive")
        self.assertEqual(len(self.nav.explorer.nodes), 1)
        self.assertTrue(self.gui.auto_mode.get())

    def test_unchecking_dfs_stops_instead_of_starting_another_segment(self):
        self.click_start()
        self.drive_selected_segment()
        self.gui.auto_mode.set(False)
        namespace["toggle_auto_mode"](self.gui)
        self.assertFalse(self.nav.active)
        self.assertEqual(self.commands[-1], "STOP")
        self.inputs(2)
        self.tick()
        self.assertFalse(self.gui.auto_mode.get())


if __name__ == "__main__":
    unittest.main()
