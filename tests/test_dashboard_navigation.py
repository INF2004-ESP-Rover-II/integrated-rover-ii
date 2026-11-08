"""Real dashboard MQTT/camera entry points with no Tk window or live broker."""

import ast
from datetime import datetime
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dashboard"))
from laptop_navigation import LaptopNavigator

source = Path(__file__).resolve().parents[1] / "dashboard/robot_dashboard.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
names = {"on_mqtt_connect", "on_mqtt_message", "handle_mqtt_message", "update_marker_map", "submit_vision_observation", "submit_marker_detection", "send_navigation_command", "toggle_auto_mode"}
namespace = {"time": time, "datetime": datetime, "json": json,
             "TOPIC_STATUS": "robot/status", "TOPIC_TELEMETRY": "robot/telemetry",
             "TOPIC_MARKER": "camera/marker", "TOPIC_ROVER_CAMERA": "robot/camera"}
namespace['TOPIC_ROVER_MAP'] = 'robot/map'
exec(compile(ast.Module(body=[node for node in gui.body if isinstance(node, ast.FunctionDef)
                             and node.name in names], type_ignores=[]), str(source), "exec"), namespace)


class Value:
    def set(self, value):
        self.value = value


class DashboardNavigationTests(unittest.TestCase):
    def setUp(self):
        class FakeGUI:
            on_mqtt_connect = namespace["on_mqtt_connect"]
            on_mqtt_message = namespace["on_mqtt_message"]
            handle_mqtt_message = namespace["handle_mqtt_message"]
            update_marker_map = namespace["update_marker_map"]
            submit_marker_detection = namespace["submit_marker_detection"]
            submit_vision_observation = namespace["submit_vision_observation"]
            send_navigation_command = namespace["send_navigation_command"]
            toggle_auto_mode = namespace["toggle_auto_mode"]
        self.gui = FakeGUI()
        self.queue = []
        self.gui.root = SimpleNamespace(after=lambda delay, callback, *args: self.queue.append((callback, args)))
        self.gui.navigator = LaptopNavigator(lambda command: True)
        self.gui.last_msg_time = Value()
        self.gui.msg_count = 0
        self.gui.update_tof_readings = lambda obstacle: None
        self.gui.log = lambda message: None
        self.gui.odometry_lock = threading.Lock()
        self.gui.heading_label = SimpleNamespace(config=lambda **values: None)
        self.gui.ultrasonic_label = SimpleNamespace(config=lambda **values: None)
        self.gui.ultrasonic = dict(front=0, back=0, left=0, right=0)
        self.gui.handle_mqtt_connect = lambda rc: None

    def flush(self):
        while self.queue:
            callback, args = self.queue.pop(0)
            callback(*args)

    def test_worker_camera_input_is_queued_then_updates_actual_navigator(self):
        self.gui.submit_marker_detection(12, 1)
        self.assertIsNone(self.gui.navigator.visible_marker)
        self.flush()
        self.assertEqual(self.gui.navigator.visible_marker, 12)
        self.gui.submit_marker_detection(None, 2)
        self.flush()
        self.assertIsNone(self.gui.navigator.visible_marker)

    def camera_message(self, sequence):
        return {"schema": 1, "source_session_id": "camera-a", "frame_seq": sequence,
                "capture_time_ms": int(time.time() * 1000), "capture_time_source": "laptop_receive",
                "status": "ok", "last_frame_age_ms": 0,
                "markers": [{"marker_id": 12, "valid": True, "distance_ref": "camera_lens",
                             "distance_mm": LaptopNavigator.ARRIVAL_DISTANCE_MM, "bearing_deg": 0}]}

    def test_full_camera_json_callback_queues_range_aware_observation(self):
        self.gui.submit_vision_observation(self.camera_message(1))
        self.assertIsNone(self.gui.navigator.visible_marker)
        self.flush()
        self.assertEqual(self.gui.navigator.visible_marker, 12)
        self.assertEqual(self.gui.navigator._arrival_count, 1)

    def test_full_camera_json_over_mqtt_reaches_confirmation_gate(self):
        for sequence in range(1, 6):
            message = SimpleNamespace(topic="camera/marker", retain=False,
                                      payload=json.dumps(self.camera_message(sequence)).encode())
            self.gui.on_mqtt_message(None, None, message)
            self.flush()
        self.assertEqual(self.gui.navigator._arrival_count, 5)
        self.assertEqual(self.gui.navigator.visible_marker, 12)
        self.assertEqual(len(self.gui.navigator.explorer.nodes), 0)  # Mission not started.

    def test_mqtt_marker_input_and_retained_observation_rejection(self):
        message = SimpleNamespace(topic="camera/marker", retain=True,
                                  payload=b'{"marker_id":12,"frame_id":1}')
        self.gui.on_mqtt_message(None, None, message)
        self.assertEqual(self.queue, [])
        message.retain = False
        self.gui.on_mqtt_message(None, None, message)
        self.flush()
        self.assertEqual(self.gui.navigator.visible_marker, 12)
        self.gui.handle_mqtt_message("camera/marker", '{"marker_id":13,"frame_id":1}')
        self.assertEqual(self.gui.navigator.visible_marker, 12)

    def test_robo_telemetry_supplies_dfs_and_robot_liveness(self):
        data = {"timestamp": 1, "imu": {"heading": 0, "status": 0},
                "obstacle": {"distance_mm": {"left": 100, "front": 700, "right": 100},
                             "age_ms": 0, "stop_required": False, "fault_mask": 0}}
        self.gui.last_telemetry_rx = None
        self.gui.handle_mqtt_message("robot/telemetry", json.dumps(data))
        self.assertEqual(self.gui.navigator.sensor.distances["front"], 700)
        self.assertIsNotNone(self.gui.last_telemetry_rx)
        data["timestamp"] = 2
        data["obstacle"]["distance_mm"]["front"] = 50
        self.gui.handle_mqtt_message("robot/telemetry", json.dumps(data))
        self.assertEqual(self.gui.navigator.sensor.distances["front"], 50)

    def test_subscriptions_use_robo_telemetry_and_camera(self):
        subscriptions = []
        client = SimpleNamespace(subscribe=subscriptions.append)
        self.gui.on_mqtt_connect(client, None, None, 0)
        self.assertEqual(subscriptions, ["robot/status", "robot/telemetry", "camera/marker", "robot/camera", "robot/map"])

    def test_retained_robo_telemetry_cannot_refresh_sensor_or_robot_health(self):
        self.gui.last_telemetry_rx = None
        message = SimpleNamespace(topic="robot/telemetry", retain=True,
                                  payload=b'{"timestamp":1}')
        self.gui.on_mqtt_message(None, None, message)
        self.assertEqual(self.queue, [])
        self.assertIsNone(self.gui.navigator.sensor)
        self.assertIsNone(self.gui.last_telemetry_rx)

    def test_live_robo_with_stale_uart_data_cannot_start_dfs(self):
        data = {"timestamp": 1, "imu": {"heading": 0, "status": 0},
                "obstacle": {"distance_mm": {"left": 100, "front": 700, "right": 100},
                             "age_ms": 1000, "stop_required": False, "fault_mask": 0}}
        self.gui.navigator.marker_seen(1, 1)
        self.gui.handle_mqtt_message("robot/telemetry", json.dumps(data))
        self.assertIsNotNone(self.gui.last_telemetry_rx)
        with self.assertRaises(ValueError):
            self.gui.navigator.start(3, 500)

    def test_motor_commands_require_robo_link_and_stop_remains_allowed(self):
        commands = []
        self.gui.estop_active = False
        self.gui.robot_link = "lost"
        self.gui.send_command = lambda command, **kwargs: commands.append(command) or True
        self.assertFalse(self.gui.send_navigation_command("FORWARD"))
        self.assertTrue(self.gui.send_navigation_command("STOP"))
        self.gui.robot_link = "live"
        self.assertTrue(self.gui.send_navigation_command("ROTATE_LEFT"))
        self.gui.estop_active = True
        self.assertFalse(self.gui.send_navigation_command("FORWARD"))
        self.assertEqual(commands, ["STOP", "ROTATE_LEFT"])

    def test_dfs_toggle_sends_only_high_level_onboard_command(self):
        starts = []
        commands = []
        self.gui.navigator = SimpleNamespace(start=lambda *args: starts.append(args))
        self.gui.auto_mode = SimpleNamespace(get=lambda: True, set=lambda value: None)
        self.gui.dfs_feedback = Value()
        self.gui.dfs_feedback_label = SimpleNamespace(config=lambda **values: None)
        self.gui.send_command = lambda command: commands.append(command) or True
        self.gui.dfs_target = SimpleNamespace(get=lambda: "36")
        self.gui.dfs_threshold = SimpleNamespace(get=lambda: "500")
        self.gui.dfs_advance_speed = SimpleNamespace(get=lambda: "150")
        self.gui.stop_hold = lambda: None
        self.gui.cancel_heartbeat = lambda: None
        self.gui.toggle_auto_mode()
        self.assertEqual(starts, [])
        self.gui.dfs_advance_speed = SimpleNamespace(get=lambda: "")
        self.gui.toggle_auto_mode()
        self.assertEqual(starts, [])
        self.assertEqual(commands, ['START_DFS:500', 'START_DFS:500'])


if __name__ == "__main__":
    unittest.main()
