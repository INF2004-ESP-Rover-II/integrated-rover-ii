"""DFS owns the route; TURN_TO completion and braking belong to RoboPico."""
from pathlib import Path
import ast
from datetime import datetime
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dashboard"))
from dfs_explorer import Direction, MarkerType
from laptop_navigation import LaptopNavigator


class TurnTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.stamp = 0
        self.commands = []
        self.nav = LaptopNavigator(self.send, clock=lambda: self.now)

    def send(self, command):
        self.commands.append(command)
        return True

    def sensor(self, heading, stop=False):
        self.stamp += 1
        self.nav.sensor_received({"timestamp": self.stamp,
            "imu": {"heading": heading, "status": 0},
            "obstacle": {"distance_mm": {"left": 700, "front": 700, "right": 700},
                         "stop_required": stop, "fault_mask": 0, "age_ms": 0}})
        self.nav.last_camera_at = self.now

    def turn(self, direction=Direction.EAST, zero=0, scan=False, actual=None):
        self.nav.explorer.start(1, 4, 1, MarkerType.JUNCTION, Direction.NORTH,
                                segment_timeout_s=30)
        if not scan:
            self.nav.explorer.observe_exits(1, 1 << direction, 0)
            self.nav.explorer.next_action(self.now)
        self.nav.map_zero_deg = zero
        self.nav.active = True
        self.nav.phase = "scan_turn" if scan else "turn"
        self.nav.scan_direction = direction
        self.nav.scan_turn_deadline = 15
        self.sensor(zero if actual is None else actual)
        self.nav.tick()

    def reached(self):
        self.nav.robot_status_received("AUTO_MOVE: TURN_TO:%d" % self.nav._turn_target)
        self.nav.robot_status_received("AUTO_RESULT: HEADING_REACHED")
        self.nav.tick()

    def test_cardinal_targets_wrap_and_command_is_sent_once(self):
        for direction, zero, target in ((Direction.EAST, 287.3, 17),
                                        (Direction.SOUTH, 287.3, 107),
                                        (Direction.WEST, 287.3, 197),
                                        (Direction.NORTH, 359.7, 0)):
            with self.subTest(direction=direction):
                self.setUp()
                self.turn(direction, zero, actual=zero - 30 if direction == Direction.NORTH else zero)
                self.assertEqual(self.commands, ["TURN_TO:%d" % target])
                for _ in range(8):
                    self.now += 0.1
                    self.sensor(zero - 30 if direction == Direction.NORTH else zero)
                    self.nav.tick()
                self.assertEqual(len(self.commands), 1)

    def test_waits_for_matching_ack_and_completion_even_if_heading_is_close(self):
        self.turn()
        self.sensor(90)
        self.nav.robot_status_received("AUTO_RESULT: HEADING_REACHED")
        self.nav.robot_status_received("AUTO_MOVE: TURN_TO:180")
        self.nav.robot_status_received("AUTO_RESULT: HEADING_REACHED")
        self.nav.tick()
        self.assertEqual(self.nav.phase, "turn")
        self.assertEqual(self.commands, ["TURN_TO:90"])
        self.reached()
        self.assertEqual(self.nav.phase, "settle")
        self.assertEqual(self.commands[-1], "STOP")

    def test_no_forward_until_post_braking_heading_is_fresh(self):
        self.turn()
        self.reached()
        self.now = 0.41
        self.nav.tick()
        self.assertNotIn("FORWARD", self.commands)
        self.sensor(90)
        self.nav.tick()
        self.assertEqual(self.commands[-1], "FORWARD")

    def test_settled_drift_triggers_another_local_turn(self):
        self.turn()
        self.reached()
        self.now = 0.41
        self.sensor(99)
        self.nav.tick()
        self.nav.tick()
        self.assertEqual(self.commands, ["TURN_TO:90", "STOP", "TURN_TO:90"])

    def test_forward_turn_keeps_correcting_beyond_travel_timeout_then_drives(self):
        self.turn()
        for correction in range(1, 7):
            self.now += 8
            self.sensor(99)
            self.reached()
            self.now += .41
            self.sensor(99)
            self.nav.tick()
            self.assertTrue(self.nav.active)
            self.assertEqual(self.nav.phase, 'turn')
            self.assertIn('heading 99.0°', self.nav.status)
            self.assertIn('error -9.0°', self.nav.status)
            self.assertIn('correction %d ' % correction, self.nav.status)
            self.assertNotIn('FORWARD', self.commands)
            self.nav.tick()
            self.assertEqual(self.commands.count('TURN_TO:90'), correction + 1)
        self.assertGreater(self.now, self.nav.explorer.segment_timeout_s)
        self.now += 8
        self.sensor(90)
        self.reached()
        self.now += .41
        self.sensor(90)
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertEqual(self.commands[-1], 'FORWARD')
        # Alignment is exempt; actual forward travel still has a finite budget.
        self.now += self.nav.explorer.segment_timeout_s + .1
        self.sensor(90)
        self.nav.tick()
        self.assertFalse(self.nav.active)
        self.assertIn('TIMEOUT', self.nav.status)

    def test_dashboard_stop_cancels_all_further_corrections(self):
        self.turn()
        for _ in range(3):
            self.reached()
            self.now += .41
            self.sensor(99)
            self.nav.tick()
            self.nav.tick()
        self.nav.stop('Stopped from dashboard')
        count = len(self.commands)
        for _ in range(5):
            self.now += .2
            self.sensor(90)
            self.nav.robot_status_received('AUTO_RESULT: HEADING_REACHED')
            self.nav.tick()
        self.assertFalse(self.nav.active)
        self.assertEqual(self.commands[-1], 'STOP')
        self.assertEqual(len(self.commands), count)

    def test_reported_46_degree_target_keeps_retrying_at_31_point_three(self):
        self.turn(Direction.NORTH, zero=46, actual=0)
        for correction in range(1, 4):
            self.reached()
            self.now += .41
            self.sensor(31.3)
            self.nav.tick()
            self.assertTrue(self.nav.active)
            self.assertIn('error +14.7°', self.nav.status)
            self.assertIn('correction %d ' % correction, self.nav.status)
            self.nav.tick()
        self.assertEqual(self.commands.count('TURN_TO:46'), 4)
        self.assertNotIn('FORWARD', self.commands)

    def test_successful_single_correction_can_drive(self):
        self.turn()
        self.reached()
        self.now = .41
        self.sensor(99)
        self.nav.tick()
        self.nav.tick()
        self.reached()
        self.now += .41
        self.sensor(91)
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'drive')
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_settled_heading_is_compared_with_the_integer_command_target(self):
        self.turn(Direction.EAST, zero=17.49)
        self.reached()
        self.now = .41
        self.sensor(103.1)  # Within 4° of TURN_TO:107, but not 107.49°.
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'drive')
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_straight_ahead_north_skips_rotation_but_waits_for_fresh_inputs(self):
        self.turn(Direction.NORTH, zero=20, actual=20)
        self.assertEqual(self.nav.phase, 'settle')
        self.assertEqual(self.commands, ['STOP'])
        self.assertIn('skipping rotation', self.nav.status)
        self.now = .41
        self.nav.tick()
        self.assertNotIn('FORWARD', self.commands)
        self.sensor(20)
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'drive')
        self.assertEqual(self.commands, ['STOP', 'FORWARD'])
        self.assertIn('Driving NORTH', self.nav.status)

    def test_reported_four_point_four_degree_error_allows_forward(self):
        self.turn(Direction.NORTH, zero=20, actual=29.1)
        self.assertEqual(self.commands, ['TURN_TO:20'])
        self.reached()
        self.now = .41
        self.sensor(29.1)
        self.nav.tick()
        self.nav.tick()
        self.assertEqual(self.commands.count('TURN_TO:20'), 2)
        self.reached()
        self.now += .41
        self.sensor(15.6)
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, 'drive')
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_aligned_wrapped_heading_skips_rotation(self):
        self.turn(Direction.NORTH, zero=359.7, actual=1.2)
        self.assertEqual(self.nav.phase, 'settle')
        self.assertEqual(self.commands, ['STOP'])

    def test_fresh_sensor_recovers_stale_link_label_before_navigation_command(self):
        self.sensor(90)
        source = Path(__file__).resolve().parents[1] / 'dashboard/robot_dashboard.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        gui_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RobotGUI')
        handler = next(n for n in gui_class.body if isinstance(n, ast.FunctionDef)
                       and n.name == 'send_navigation_command')
        namespace = {}
        exec(compile(ast.Module(body=[handler], type_ignores=[]), str(source), 'exec'), namespace)
        sent, links, logs = [], [], []
        gui = SimpleNamespace(navigator=self.nav, estop_active=False, robot_link='lost',
                              log=logs.append,
                              send_command=lambda command, quiet: sent.append(command) or True)
        def set_link(state):
            links.append(state)
            gui.robot_link = state
        gui.set_robot_link = set_link
        self.assertTrue(namespace['send_navigation_command'](gui, 'FORWARD'))
        self.assertEqual(links, ['live'])
        self.assertEqual(sent, ['FORWARD'])
        gui.robot_link = 'lost'
        self.now += 1
        self.assertFalse(namespace['send_navigation_command'](gui, 'FORWARD'))
        self.assertEqual(sent, ['FORWARD'])
        self.assertIn('stale', logs[-1])
        self.sensor(90)
        gui.estop_active = True
        self.assertFalse(namespace['send_navigation_command'](gui, 'FORWARD'))
        self.assertTrue(namespace['send_navigation_command'](gui, 'STOP'))

    def test_scan_turn_also_waits_for_completion_and_fresh_heading(self):
        self.turn(Direction.SOUTH, scan=True)
        self.assertEqual(self.commands, ["TURN_TO:180"])
        self.reached()
        self.assertEqual(self.nav.phase, "scan_settle")
        self.now = 0.41
        self.nav.tick()
        self.assertEqual(self.nav.phase, "scan_settle")
        self.sensor(180)
        self.nav.tick()
        self.assertEqual(self.nav.phase, "scan")
        self.assertNotIn("FORWARD", self.commands)

    def test_rear_scan_accepts_settled_heading_within_scan_tolerance_without_retry(self):
        self.turn(Direction.SOUTH, scan=True)
        self.reached()
        self.now = .41
        self.sensor(192)
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'scan')
        self.assertEqual(self.commands.count('TURN_TO:180'), 1)
        self.assertNotIn('FORWARD', self.commands)

    def test_rear_scan_keeps_retrying_excessive_settled_drift_then_scans(self):
        self.turn(Direction.SOUTH, scan=True)
        for correction in range(1, 5):
            self.now += 8
            self.sensor(205)
            self.reached()
            self.now += .41
            self.sensor(205)
            self.nav.tick()
            self.assertTrue(self.nav.active)
            self.assertEqual(self.nav.phase, 'scan_turn')
            self.assertIn('correction %d ' % correction, self.nav.status)
            self.nav.tick()
            self.assertEqual(self.commands.count('TURN_TO:180'), correction + 1)
        self.reached()
        self.now += .41
        self.sensor(180)
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, 'scan')
        self.assertNotIn('FORWARD', self.commands)

    def test_refused_or_interrupted_turn_pauses(self):
        for status in ("AUTO_REFUSED: TURN_TO (no fresh heading from IMU)",
                       "AUTO_RESULT: STOPPED", "AUTO_RESULT: REJECTED"):
            with self.subTest(status=status):
                self.setUp()
                self.turn()
                self.nav.robot_status_received("AUTO_MOVE: TURN_TO:90")
                self.nav.robot_status_received(status)
                self.assertFalse(self.nav.active)
                self.assertEqual(self.commands[-1], "STOP")

    def test_turn_timer_brakes_and_missing_inputs_eventually_pause(self):
        self.turn()
        self.now = 1.1
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertEqual(self.commands, ["TURN_TO:90"])
        self.now = 15.1
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, 'settle')
        self.assertEqual(self.commands[-1], "STOP")
        self.now = 18.6
        self.nav.tick()
        self.assertFalse(self.nav.active)
        self.assertEqual(self.commands[-1], "STOP")

    def test_turn_timer_retries_with_fresh_data_for_navigation_and_exit_scan(self):
        for scan in (False, True):
            with self.subTest(scan=scan):
                self.setUp()
                self.turn(scan=scan)
                for attempt in range(1, 5):
                    self.now += self.nav.TURN_TIMEOUT_S + .1
                    self.sensor(0)
                    self.nav.tick()
                    self.assertTrue(self.nav.active)
                    self.assertEqual(self.nav.phase, 'scan_settle' if scan else 'settle')
                    self.assertEqual(self.commands[-1], 'STOP')
                    self.now += .41
                    self.sensor(0)
                    self.nav.tick()
                    self.nav.tick()
                    self.assertEqual(self.commands.count('TURN_TO:90'), attempt + 1)
                self.assertNotIn('FORWARD', self.commands)

    def camera_stalled(self):
        self.nav.vision_received({"schema": 1, "source_session_id": "camera",
            "frame_seq": 1, "capture_time_ms": int(self.nav.wall_clock() * 1000) - 2000,
            "capture_time_source": "laptop_receive", "last_frame_age_ms": 2000,
            "status": "stalled", "markers": []})

    def test_stalled_camera_and_missing_sensor_do_not_abort_rotation(self):
        self.turn()
        self.camera_stalled()
        self.stamp += 1
        self.nav.sensor_received({"timestamp": self.stamp})
        self.now = 1.1
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.reached()
        self.assertEqual(self.nav.phase, "settle")
        self.assertEqual(self.commands[-1], "STOP")
        self.now = 1.6
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertNotIn("FORWARD", self.commands)
        self.sensor(90)
        self.nav.last_camera_at = None
        self.nav.tick()
        self.assertNotIn("FORWARD", self.commands)
        # A fresh no-marker frame is enough for camera health after turning.
        self.nav.marker_seen(None, 1)
        self.nav.tick()
        self.assertEqual(self.commands[-1], "FORWARD")

    def test_missing_post_turn_inputs_pause_with_specific_diagnostics(self):
        self.turn()
        self.reached()
        self.now = 1.5
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertNotIn("FORWARD", self.commands)
        self.now = 3.5
        self.nav.tick()
        self.assertFalse(self.nav.active)
        self.assertIn("sensor telemetry age", self.nav.status)
        self.assertIn("camera frame age", self.nav.status)

    def test_camera_timeout_still_stops_forward_travel(self):
        self.turn()
        self.nav.phase = "drive"
        self.now = 1.1
        self.sensor(90)
        self.nav.last_camera_at = 0.0
        self.nav.tick()
        self.assertFalse(self.nav.active)
        self.assertIn("camera frame age", self.nav.status)
        self.assertEqual(self.commands[-1], "STOP")

    def test_dashboard_telemetry_watchdog_does_not_abort_local_turn(self):
        self.turn()
        source = Path(__file__).resolve().parents[1] / "dashboard/robot_dashboard.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        gui_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RobotGUI")
        handler = next(n for n in gui_class.body if isinstance(n, ast.FunctionDef) and
                       n.name == "set_robot_link")
        namespace = {"datetime": datetime}
        exec(compile(ast.Module(body=[handler], type_ignores=[]), str(source), "exec"), namespace)
        gui = SimpleNamespace(navigator=self.nav, robot_link="live", active_command=None,
                              last_telemetry_rx=1, clear_tof_readings=lambda reason: None,
                              robot_connected=SimpleNamespace(set=lambda value: None),
                              robot_conn_label=SimpleNamespace(config=lambda **kwargs: None),
                              log=lambda message: None)
        namespace['set_robot_link'](gui, "lost", 2.1)
        self.assertTrue(self.nav.active)
        self.assertEqual(self.commands, ["TURN_TO:90"])
        self.nav.phase = 'input_wait'
        namespace['set_robot_link'](gui, "lost", 2.1)
        self.assertTrue(self.nav.active)
        self.assertEqual(self.commands, ["TURN_TO:90"])

    def test_dashboard_routes_mqtt_turn_results_to_navigator(self):
        self.turn()
        source = Path(__file__).resolve().parents[1] / "dashboard/robot_dashboard.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        gui_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RobotGUI")
        handler = next(n for n in gui_class.body if isinstance(n, ast.FunctionDef) and
                       n.name == "handle_mqtt_message")
        namespace = {"datetime": datetime, "TOPIC_STATUS": "robot/status"}
        exec(compile(ast.Module(body=[handler], type_ignores=[]), str(source), "exec"), namespace)
        value = SimpleNamespace(set=lambda value: None)
        gui = SimpleNamespace(navigator=self.nav, last_msg_time=value, robot_status=value,
                              msg_count=0, log=lambda message: None)
        route = namespace['handle_mqtt_message']
        route(gui, "robot/status", "AUTO_MOVE: TURN_TO:90")
        route(gui, "robot/status", "AUTO_RESULT: HEADING_REACHED")
        self.assertTrue(self.nav._turn_reached)
        self.nav.tick()
        self.assertEqual(self.nav.phase, "settle")


if __name__ == "__main__":
    unittest.main()
