"""Heading-aware marker placement and real dashboard routing, without hardware."""

import ast
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dashboard"))
from laptop_navigation import LaptopNavigator
from vision_feed import GRID_SIZE, MarkerObservation, VisionFeed, grid_cell, place_markers


class MarkerGeometryTests(unittest.TestCase):
    def test_front_marker_rotates_to_each_map_direction_including_camera_offset(self):
        for heading, expected in ((0, (0, 720)), (90, (720, 0)),
                                  (180, (0, -720)), (270, (-720, 0))):
            with self.subTest(heading=heading):
                found = {}
                self.assertEqual(place_markers([MarkerObservation(1, 600, 0)], found,
                                               heading_deg=heading), [1])
                self.assertAlmostEqual(found[1][0], expected[0])
                self.assertAlmostEqual(found[1][1], expected[1])

    def test_camera_bearing_and_nonzero_start_cell_rotate_together(self):
        found = {}
        place_markers([MarkerObservation(1, 600, 30)], found, (2, 3), heading_deg=90)
        self.assertAlmostEqual(found[1][0], 1200 + 120 + 600 * math.cos(math.radians(30)))
        self.assertAlmostEqual(found[1][1], 1800 - 300)

    def test_known_marker_anchors_new_markers_after_turning_and_translation(self):
        found = {1: (1200, 1200)}
        added = place_markers([MarkerObservation(1, 480, 0), MarkerObservation(2, 600, 90)],
                              found, heading_deg=90)
        self.assertEqual(added, [2])
        self.assertEqual(found[1], (1200, 1200))
        self.assertAlmostEqual(found[2][0], 720)
        self.assertAlmostEqual(found[2][1], 600)

    def test_invalid_heading_cannot_add_a_marker(self):
        for heading in (None, True, float("nan"), float("inf")):
            with self.subTest(heading=heading):
                found = {}
                with self.assertRaises(ValueError):
                    place_markers([MarkerObservation(1, 600, 0)], found, heading_deg=heading)
                self.assertEqual(found, {})


class DashboardMarkerMapTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.wall_start = 1790000000.0
        self.frame_seq = 0
        self.sensor_seq = 0
        source = Path(__file__).resolve().parents[1] / "dashboard/robot_dashboard.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
        names = {"update_marker_map", "handle_mqtt_message", "on_mqtt_message"}
        namespace = {"datetime": datetime, "json": json,
                     "time": SimpleNamespace(time=lambda: self.wall_start + self.now,
                                             monotonic=lambda: self.now),
                     "Rectangle": Mock(), "GRID_SIZE": GRID_SIZE,
                     "grid_cell": grid_cell, "place_markers": place_markers,
                     "TOPIC_TELEMETRY": "robot/telemetry", "TOPIC_MARKER": "camera/marker",
                     "TOPIC_STATUS": "robot/status"}
        methods = [node for node in gui.body if isinstance(node, ast.FunctionDef) and node.name in names]
        self.assertEqual(len(methods), len(names))
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)
        self.gui = type("HeadlessGUI", (), {name: namespace[name] for name in names})()
        self.gui.navigator = LaptopNavigator(lambda command: True, clock=lambda: self.now,
                                             wall_clock=lambda: self.wall_start + self.now)
        self.gui.map_feed = VisionFeed()
        self.gui.map_start_cell = (0, 0)
        self.gui.found_markers = {}
        self.gui.map_title = ""
        self.gui.map_drawn_at = -float("inf")
        self.gui.ax = Mock()
        self.gui.canvas = Mock()
        self.gui.log = Mock()
        self.gui.last_msg_time = Mock()
        self.gui.msg_count = 0
        self.gui.update_tof_readings = Mock()
        self.gui.odometry_lock = threading.Lock()
        self.gui.heading_label = Mock()
        self.gui.pitch_label = Mock()
        self.gui.roll_label = Mock()
        self.gui.ultrasonic_label = Mock()
        self.gui.ultrasonic = dict(front=0, back=0, left=0, right=0)
        self.gui.root = SimpleNamespace(after=lambda delay, callback, *args: callback(*args))

    def telemetry(self, heading=0, age_ms=0, status=0, repeat=False):
        if not repeat:
            self.sensor_seq += 1
        data = {"timestamp": self.sensor_seq, "imu": {"heading": heading, "status": status},
                "obstacle": {"distance_mm": {"left": 700, "front": 700, "right": 700},
                             "age_ms": age_ms, "fault_mask": 0, "stop_required": False}}
        self.gui.handle_mqtt_message("robot/telemetry", json.dumps(data))

    def camera(self, marker_id=1, bearing=0, age_ms=0):
        self.frame_seq += 1
        data = {"schema": 1, "source_session_id": "camera-a", "frame_seq": self.frame_seq,
                "capture_time_ms": int((self.wall_start + self.now) * 1000) - age_ms,
                "capture_time_source": "laptop_receive", "last_frame_age_ms": age_ms,
                "status": "ok", "markers": [{"marker_id": marker_id, "valid": True,
                    "distance_ref": "camera_lens", "distance_mm": 600, "bearing_deg": bearing}]}
        self.gui.on_mqtt_message(None, None, SimpleNamespace(topic="camera/marker", retain=False,
                                payload=json.dumps(data).encode()))

    def test_mqtt_uses_initial_imu_heading_even_if_first_marker_arrives_after_turn(self):
        self.telemetry(350)
        self.telemetry(80)
        self.camera()
        self.assertEqual(self.gui.navigator.initial_sensor_heading, 350)
        self.assertAlmostEqual(self.gui.found_markers[1][0], 720)
        self.assertAlmostEqual(self.gui.found_markers[1][1], 0)
        self.assertEqual(grid_cell(self.gui.found_markers[1]), (1, 0))

    def test_turns_map_all_four_sides_and_expand_the_visible_grid(self):
        self.telemetry(0)
        for marker_id, heading, cell in ((1, 0, (0, 1)), (2, 90, (1, 0)),
                                        (3, 180, (0, -1)), (4, 270, (-1, 0))):
            self.telemetry(heading)
            self.camera(marker_id)
            self.assertEqual(grid_cell(self.gui.found_markers[marker_id]), cell)
        self.gui.ax.set_xlim.assert_called_with(-1, 6)
        self.gui.ax.set_ylim.assert_called_with(-1, 6)
        self.assertEqual(self.gui.ax.add_patch.call_count, 4)

    def test_wraparound_and_camera_bearing_use_continuous_heading(self):
        self.telemetry(350)
        self.telemetry(10)
        self.camera(bearing=10)
        # The marker ray is 30 degrees from map north; the lens offset is at 20.
        self.assertAlmostEqual(self.gui.found_markers[1][0], 600 * math.sin(math.radians(30)) +
                               120 * math.sin(math.radians(20)))
        self.assertAlmostEqual(self.gui.found_markers[1][1], 600 * math.cos(math.radians(30)) +
                               120 * math.cos(math.radians(20)))

    def test_no_imu_waits_without_permanently_mapping_a_marker(self):
        self.camera()
        self.assertEqual(self.gui.found_markers, {})
        self.assertEqual(self.gui.map_title, "Waiting for fresh IMU heading")
        self.telemetry(123)
        self.camera()
        self.assertEqual(grid_cell(self.gui.found_markers[1]), (0, 1))

    def test_stale_uart_packet_cannot_establish_starting_heading(self):
        self.telemetry(0, age_ms=1000)
        self.camera()
        self.assertIsNone(self.gui.navigator.initial_sensor_heading)
        self.assertEqual(self.gui.found_markers, {})
        self.telemetry(90)
        self.camera()
        self.assertEqual(self.gui.navigator.initial_sensor_heading, 90)
        self.assertEqual(grid_cell(self.gui.found_markers[1]), (0, 1))

    def test_duplicate_packet_does_not_keep_mapping_heading_fresh(self):
        self.telemetry(0)
        self.now += 1
        self.telemetry(0, repeat=True)
        self.camera()
        self.assertEqual(self.gui.found_markers, {})
        self.telemetry(90)
        self.camera()
        self.assertEqual(grid_cell(self.gui.found_markers[1]), (1, 0))

    def test_reported_uart_age_counts_toward_mapping_freshness(self):
        self.telemetry(0, age_ms=600)
        self.now += 0.2
        self.camera()
        self.assertEqual(self.gui.found_markers, {})

    def test_invalid_imu_cannot_map_or_reset_the_starting_reference(self):
        self.telemetry(20)
        for heading, status in ((0, 1), (float("nan"), 0)):
            with self.subTest(heading=heading, status=status):
                self.telemetry(heading, status=status)
                self.camera()
                self.assertEqual(self.gui.found_markers, {})
                self.assertEqual(self.gui.navigator.initial_sensor_heading, 20)
        self.telemetry(110)
        self.camera()
        self.assertEqual(grid_cell(self.gui.found_markers[1]), (1, 0))

    def test_stale_camera_frame_cannot_place_a_marker(self):
        self.telemetry(0)
        self.camera(age_ms=600)
        self.assertEqual(self.gui.found_markers, {})
        self.assertEqual(self.gui.map_title, "Camera stale")


if __name__ == "__main__":
    unittest.main()
