"""RoboPico observations render without feeding laptop DFS, mapping or motors."""
import ast
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dashboard"))
from rover_camera import RoverCameraCache, QUALITY_NAMES
from test_dashboard_mission import FakeGUI, Variable, Label

tree = ast.parse((ROOT / "dashboard/robot_dashboard.py").read_text(encoding="utf-8"))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
names = {"update_rover_camera_display", "update_camera_marker_list", "clear_rover_camera_display", "update_current_marker", "on_mqtt_message", "on_mqtt_connect"}
namespace = {"time": time, "TOPIC_STATUS": "robot/status", "TOPIC_TELEMETRY": "robot/telemetry",
             "TOPIC_MARKER": "camera/marker", "TOPIC_ROVER_CAMERA": "robot/camera", "TOPIC_ROVER_MAP": "robot/map",
             "QUALITY_NAMES": QUALITY_NAMES}
exec(compile(ast.Module(body=[node for node in gui.body if isinstance(node, ast.FunctionDef)
                             and node.name in names], type_ignores=[]), "camera_relay_handlers", "exec"), namespace)


def observation(status="OK", age=200, markers=None, seq=194):
    if markers is None:
        markers = [{"id": 1, "range_mm": 484, "bearing_cdeg": 82, "quality": 0}] if status == "OK" else []
    return {"schema": 1, "source": "robo_pico_uart", "seq": seq, "status": status,
            "age_ms": age, "uart_age_ms": 1, "markers": markers}


class Table:
    def __init__(self):
        self.rows = {}
        self.height = 1

    def get_children(self):
        return tuple(self.rows)

    def delete(self, item):
        del self.rows[item]

    def configure(self, height):
        self.height = height

    def insert(self, parent, where, iid, values, tags):
        self.rows[iid] = values


class CameraGUI(FakeGUI):
    def __init__(self):
        super().__init__()
        self.rover_camera = RoverCameraCache()
        for name in ("current_marker_text", "marker_distance_text", "marker_bearing_text",
                     "marker_confirmation_text", "camera_relay_text", "last_reached_marker_text", "dfs_current_node_text"):
            setattr(self, name, Variable("--"))
        self.marker_confirmation_label = Label()
        self.camera_marker_count = Variable()
        self.camera_marker_rows = None
        self.camera_marker_table = Table()
        self.navigator.clock = lambda: 100.0
        self.navigator.last_reached_marker = None
        self.navigator.explorer = SimpleNamespace(current=None, current_confirmed=False)
        self.navigator.grid = None
        self.navigator.visible_marker = 99 # Legacy camera must not overwrite relay display.
        self.queued = []
        self.root.after = lambda delay, callback, *args: self.queued.append((callback, args))


for name in names:
    setattr(CameraGUI, name, namespace[name])


class CameraRelayTests(unittest.TestCase):
    def test_visible_nearest_marker_does_not_replace_onboard_locked_target(self):
        app = CameraGUI()
        app.update_mission_display({'state': 'IDLE', 'reason': 'BOOT', 'test_mode': False,
            'motion_enabled': False, 'revision': 2, 'leg': {
                'leg_id': 1, 'configured': True, 'departure_node': 7, 'next_target_node': 12,
                'arrival_reference_node': 12, 'target_cell': {'x': 1, 'y': 0},
                'direction': 'EAST', 'phase': 'CONFIGURED'}})
        app.handle_mqtt_message('robot/camera', json.dumps(observation(markers=[
            {'id': 9, 'range_mm': 100, 'bearing_cdeg': 0, 'quality': 0}])))
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '12')
        self.assertIsNone(app.onboard_map.snapshot)

    def test_live_observation_updates_existing_fields_without_navigation(self):
        app = CameraGUI()
        with patch("time.monotonic", return_value=100.0):
            app.handle_mqtt_message("robot/camera", json.dumps(observation()))
            app.update_current_marker()
        self.assertEqual(app.current_marker_text.get(), "1")
        self.assertEqual(app.marker_distance_text.get(), "484 mm")
        self.assertEqual(app.marker_bearing_text.get(), "+0.82°")
        self.assertEqual(app.last_reached_marker_text.get(), "--")
        self.assertEqual(app.dfs_current_node_text.get(), "--")
        self.assertEqual(app.sensor_events, [])
        self.assertIn("display only", app.marker_confirmation_text.get())

    def test_health_packets_immediately_clear_old_marker(self):
        app = CameraGUI()
        for status in ("NO_MARKER", "STALLED", "CAMERA_LOST", "UART_TIMEOUT"):
            app.handle_mqtt_message("robot/camera", json.dumps(observation()))
            app.handle_mqtt_message("robot/camera", json.dumps(observation(status)))
            self.assertEqual(app.current_marker_text.get(), "--")
            self.assertEqual(app.camera_marker_rows, ())
            self.assertIn(status, app.marker_confirmation_text.get())

    def test_old_ok_frames_and_silent_stream_never_leave_live_marker(self):
        cache = RoverCameraCache()
        self.assertTrue(cache.update(observation(age=3971), 10.0))
        self.assertIsNone(cache.display(10.0)[0])
        self.assertIn("stale", cache.display(10.0)[1])
        cache.update(observation(), 10.0)
        self.assertIsNotNone(cache.display(10.1)[0])
        self.assertIsNone(cache.display(11.0)[0])
        self.assertIn("stale", cache.display(11.0)[1])
        self.assertIn("stopped", cache.display(12.0)[1])

    def test_all_markers_visible_and_selection_excludes_bad_quality(self):
        markers = [{"id": 36, "range_mm": 100, "bearing_cdeg": 0, "quality": 4},
                   {"id": 2, "range_mm": 800, "bearing_cdeg": -200, "quality": 0},
                   {"id": 1, "range_mm": 484, "bearing_cdeg": 82, "quality": 0},
                   {"id": 7, "range_mm": 0, "bearing_cdeg": 0, "quality": 1}]
        cache = RoverCameraCache()
        self.assertTrue(cache.update(observation(markers=markers), 10))
        marker, _, _, _ = cache.display(10)
        self.assertEqual(marker.id, 1)
        self.assertEqual([item.id for item in cache.markers_in_view(10)], [36, 2, 1, 7])
        self.assertEqual(cache.markers_in_view(10)[0].quality, 4)
        cache.update(observation(markers=[markers[0]]), 11)
        self.assertIsNone(cache.display(11)[0])
        self.assertIn("no usable", cache.display(11)[1])

    def test_malformed_relay_cannot_refresh_the_cache(self):
        cache = RoverCameraCache()
        cache.update(observation(), 10)
        original = cache.snapshot
        for data in (None, [], {}, {**observation(), "schema": True},
                     {**observation(), "age_ms": -1}, {**observation(), "source": "laptop"},
                     {**observation(), "markers": [None]}, {**observation(), "seq": 256},
                     {**observation(), "status": "STALLED"}):
            self.assertFalse(cache.update(data, 11))
            self.assertEqual(cache.snapshot, original)

    def test_repeated_health_sequence_and_sequence_wrap_are_valid(self):
        cache = RoverCameraCache()
        for seq, status in ((255, "OK"), (255, "STALLED"), (255, "STALLED"), (0, "OK")):
            self.assertTrue(cache.update(observation(status, seq=seq), 10))
        self.assertEqual(cache.display(10)[0].id, 1)

    def test_retained_camera_messages_are_not_dispatched(self):
        app = CameraGUI()
        app.on_mqtt_message(None, None, SimpleNamespace(retain=True, topic="robot/camera",
                                                       payload=json.dumps(observation()).encode()))
        self.assertFalse(app.queued)
        self.assertFalse(app.rover_camera.seen)

    def test_retained_target_status_cannot_restore_a_live_selection(self):
        app = CameraGUI()
        app.on_mqtt_message(None, None, SimpleNamespace(retain=True, topic='robot/status',
            payload=b'TARGET_SELECTION:{}'))
        self.assertFalse(app.queued)
        self.assertIsNone(app.target_selection_report)

    def test_disconnect_clears_marker_and_does_not_restore_legacy_camera(self):
        app = CameraGUI()
        app.handle_mqtt_message("robot/camera", json.dumps(observation()))
        app.handle_mqtt_disconnect(1)
        app.update_current_marker()
        self.assertEqual(app.current_marker_text.get(), "--")
        self.assertTrue(app.rover_camera.seen)
        self.assertEqual(app.camera_marker_rows, ())

    def test_two_markers_from_rover_log_appear_as_separate_rows_and_do_not_accumulate(self):
        app = CameraGUI()
        markers = [{"id": 2, "range_mm": 576, "bearing_cdeg": -177, "quality": 0},
                   {"id": 1, "range_mm": 473, "bearing_cdeg": -418, "quality": 0}]
        with patch("time.monotonic", return_value=100.0):
            app.handle_mqtt_message("robot/camera", json.dumps(observation(markers=markers)))
            self.assertEqual(app.current_marker_text.get(), "2")
            self.assertEqual(app.camera_marker_count.get(), "Markers in view: 2")
            self.assertEqual(list(app.camera_marker_table.rows.values()), [
                (1, '473', '-4.18', 'OK'), (2, '576', '-1.77', 'OK')])
            app.handle_mqtt_message("robot/camera", json.dumps(observation(markers=markers[1:])))
            self.assertEqual(list(app.camera_marker_table.rows.values()), [(1, '473', '-4.18', 'OK')])
        with patch("time.monotonic", return_value=102.0):
            app.update_current_marker()
        self.assertEqual(app.camera_marker_table.rows, {})
        self.assertEqual(app.camera_marker_count.get(), "Markers in view: 0")

    def test_all_four_entries_are_listed_including_unusable_poses(self):
        app = CameraGUI()
        markers = [{"id": 4, "range_mm": 50, "bearing_cdeg": 0, "quality": 1},
                   {"id": 3, "range_mm": 1200, "bearing_cdeg": 350, "quality": 4},
                   {"id": 2, "range_mm": 300, "bearing_cdeg": -177, "quality": 0},
                   {"id": 1, "range_mm": 473, "bearing_cdeg": -418, "quality": 0}]
        app.handle_mqtt_message("robot/camera", json.dumps(observation(markers=markers)))
        self.assertEqual(len(app.camera_marker_table.rows), 4)
        self.assertEqual(app.camera_marker_table.height, 4)
        self.assertEqual([row[0] for row in app.camera_marker_rows], [2, 1, 3, 4])
        self.assertEqual(app.camera_marker_rows[3], (4, '--', '--', 'POSE_FAILED'))
        self.assertEqual(app.camera_marker_rows[2][3], 'TOO_FAR')


if __name__ == "__main__":
    unittest.main()
