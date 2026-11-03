"""Exercise the real ToF display and telemetry handlers without Tk or a broker."""
import ast
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

source = Path(__file__).resolve().parents[1] / "dashboard" / "robot_dashboard.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
names = {"clear_tof_readings", "update_tof_readings", "handle_mqtt_message", "watchdog_tick"}
namespace = {
    "time": time, "math": math, "json": json, "datetime": datetime,
    "TOPIC_STATUS": "robot/status", "TOPIC_TELEMETRY": "robot/telemetry",
    "TOPIC_MARKER": "camera/marker",
    "TOF_UART_STALE_MS": 500, "ROBOT_LOST_S": 2.0, "WATCHDOG_CHECK_MS": 250,
}
methods = [node for node in gui.body if isinstance(node, ast.FunctionDef) and node.name in names]
assert len(methods) == len(names)
exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)


class Label:
    def __init__(self):
        self.values = {}

    def config(self, **values):
        self.values.update(values)


class Variable:
    def set(self, value):
        self.value = value


class Root:
    def after(self, *args):
        pass


class FakeGUI:
    clear_tof_readings = namespace["clear_tof_readings"]
    update_tof_readings = namespace["update_tof_readings"]
    handle_mqtt_message = namespace["handle_mqtt_message"]
    watchdog_tick = namespace["watchdog_tick"]

    def __init__(self):
        self.navigator = SimpleNamespace(sensor_received=lambda data: None)
        self.tof_labels = {name: Label() for name in ("left", "front", "right")}
        self.tof_state_labels = {name: Label() for name in ("left", "front", "right")}
        self.tof_summary_label = Label()
        self.tof_age_label = Label()
        self.last_tof_rx = None
        self.connected = True
        self.last_telemetry_rx = time.time()
        self.connect_time = time.time()
        self.robot_link = "live"
        self.root = Root()
        self.last_msg_time = Variable()
        self.msg_count = 0
        self.ultrasonic = {name: 0 for name in ("left", "front", "right", "back")}
        self.ultrasonic_label = Label()
        self.heading_label = Label()
        self.pitch_label = Label()
        self.roll_label = Label()
        self.odometry_lock = threading.Lock()

    def set_robot_link(self, state, age=0):
        self.robot_link = state

    def send_command(self, *args):
        raise AssertionError("Displaying ToF telemetry must never send motor commands")


def snapshot(left=740, front=100, right=65535, blocked=2, urgent=2, stop=True, age=20, faults=0):
    return {"distance_mm": {"left": left, "front": front, "right": right},
            "blocked_mask": blocked, "urgent_mask": urgent, "fault_mask": faults,
            "stop_required": stop, "age_ms": age}


class ToFDisplayTests(unittest.TestCase):
    def setUp(self):
        self.app = FakeGUI()

    def test_real_front_obstacle_and_direction_order(self):
        self.app.update_tof_readings(snapshot())
        self.assertEqual(self.app.tof_labels["left"].values["text"], "740 mm")
        self.assertEqual(self.app.tof_labels["front"].values["text"], "100 mm")
        self.assertEqual(self.app.tof_state_labels["front"].values["text"], "Urgent")
        self.assertEqual(self.app.tof_labels["right"].values["text"], "--")
        self.assertEqual(self.app.tof_state_labels["right"].values["text"], "No target")
        self.assertIn("Stop request: YES", self.app.tof_summary_label.values["text"])
        self.assertIn("Blocked: 0x02", self.app.tof_summary_label.values["text"])
        self.assertIn("20 ms", self.app.tof_age_label.values["text"])

    def test_side_urgent_is_not_presented_as_a_stop(self):
        self.app.update_tof_readings(snapshot(left=50, front=1000, blocked=1, urgent=1, stop=False))
        self.assertEqual(self.app.tof_state_labels["left"].values["text"], "Urgent")
        self.assertIn("Stop request: NO", self.app.tof_summary_label.values["text"])

    def test_exact_1000_range_is_not_labelled_as_capped(self):
        self.app.update_tof_readings(snapshot(front=1000, blocked=0, urgent=0, stop=False))
        self.assertEqual(self.app.tof_labels['front'].values['text'], '1000 mm')
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'Range')

    def test_unavailable_does_not_appear_as_zero_distance_or_fault(self):
        self.app.update_tof_readings(snapshot(left=0, front=65535, right=None, blocked=0, urgent=0, stop=False))
        for label in self.app.tof_labels.values():
            self.assertEqual(label.values["text"], "--")
        self.assertEqual(self.app.tof_state_labels["left"].values["text"], "Unavailable")
        self.assertEqual(self.app.tof_state_labels["front"].values["text"], "No target")

    def test_missing_obstacle_clears_previous_values(self):
        self.app.update_tof_readings(snapshot())
        for missing in (None, {}, {"distance_mm": None}, []):
            self.app.update_tof_readings(missing)
            self.assertEqual(self.app.tof_labels["front"].values["text"], "--")
            self.assertIsNone(self.app.last_tof_rx)

    def test_healthy_rejected_results_show_unavailable_then_detect_a_hand(self):
        self.app.update_tof_readings(snapshot(front=740, blocked=0, urgent=0, stop=False))
        for _ in range(10):
            self.app.update_tof_readings(snapshot(front=0, blocked=0, urgent=0, stop=False, faults=0))
            self.assertEqual(self.app.tof_labels['front'].values['text'], '--')
            self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'Unavailable')
        self.app.update_tof_readings(snapshot(front=100))
        self.assertEqual(self.app.tof_labels['front'].values['text'], '100 mm')
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'Urgent')
        self.assertTrue(self.app.front_obstacle_snapshot)

    def test_device_response_recovery_does_not_require_a_valid_distance(self):
        self.app.update_tof_readings(snapshot(front=0, blocked=0, urgent=0, stop=False, faults=2))
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'Sensor fault')
        self.app.update_tof_readings(snapshot(front=0, blocked=0, urgent=0, stop=False, faults=0))
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'Unavailable')
        self.app.update_tof_readings(snapshot(front=1023, blocked=0, urgent=0, stop=False, faults=0))
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'No target within 1 m')

    def test_fault_bits_override_distance_and_clear_on_recovery(self):
        for faults in range(1, 8):
            self.app.update_tof_readings(snapshot(left=740, front=740, right=740,
                                                 blocked=0, urgent=0, stop=False, faults=faults))
            for direction, bit in (('left', 1), ('front', 2), ('right', 4)):
                if faults & bit:
                    self.assertEqual(self.app.tof_labels[direction].values['text'], '--')
                    self.assertEqual(self.app.tof_state_labels[direction].values['text'], 'Sensor fault')
                else:
                    self.assertEqual(self.app.tof_labels[direction].values['text'], '740 mm')
            self.assertIn('Stop request: NO', self.app.tof_summary_label.values['text'])
        self.app.update_tof_readings(snapshot(front=740, blocked=0, urgent=0, stop=False, faults=0))
        self.assertEqual(self.app.tof_labels['front'].values['text'], '740 mm')
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'Range')

    def test_reported_fault_is_distinct_from_no_target_and_urgency(self):
        self.app.update_tof_readings(snapshot(front=1023, faults=2, blocked=0, urgent=0, stop=False))
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'Sensor fault')
        self.app.update_tof_readings(snapshot(front=100, faults=2))
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'Sensor fault')
        self.app.update_tof_readings(snapshot(front=1023, faults=0, blocked=0, urgent=0, stop=False))
        self.assertEqual(self.app.tof_labels['front'].values['text'], '>1000 mm')
        self.assertEqual(self.app.tof_state_labels['front'].values['text'], 'No target within 1 m')

    def test_uart_age_boundary_and_staleness(self):
        self.app.update_tof_readings(snapshot(age=500))
        self.assertEqual(self.app.tof_labels["front"].values["text"], "100 mm")
        self.app.update_tof_readings(snapshot(age=501))
        self.assertEqual(self.app.tof_labels["front"].values["text"], "--")
        self.assertEqual(self.app.tof_state_labels["front"].values["text"], "UART data stale")

    def test_no_telemetry_expires_display_even_if_other_telemetry_stays_live(self):
        with patch.object(time, "time", return_value=100):
            self.app.update_tof_readings(snapshot())
        with patch.object(time, "time", return_value=103):
            self.app.last_telemetry_rx = 103
            self.app.watchdog_tick()
        self.assertEqual(self.app.tof_labels["front"].values["text"], "--")
        self.assertEqual(self.app.tof_state_labels["front"].values["text"], "Telemetry stale")

    def test_missing_or_malformed_fields_are_not_coerced_to_measurements(self):
        data = snapshot(left=True, front="100", right=-1)
        data.update(blocked_mask="2", urgent_mask=2.5, stop_required="false", age_ms=float("nan"))
        self.app.update_tof_readings(data)
        for label in self.app.tof_labels.values():
            self.assertEqual(label.values["text"], "--")
        self.assertIn("Stop request: --", self.app.tof_summary_label.values["text"])
        self.assertIn("Blocked: --", self.app.tof_summary_label.values["text"])

    def test_actual_telemetry_handler_updates_tof_and_existing_fields(self):
        data = {"obstacle": snapshot(), "imu": {"heading": 45, "pitch": 2, "roll": -3, "status": 0},
                "ultrasonic": {"front": 124, "back": 400}}
        self.app.handle_mqtt_message("robot/telemetry", json.dumps(data))
        self.assertEqual(self.app.tof_labels["front"].values["text"], "100 mm")
        self.assertEqual(self.app.ultrasonic["front"], 124)
        self.assertEqual(self.app.heading_label.values["text"], "45.0°")
        self.assertEqual(self.app.pitch_label.values["text"], "2.0°")
        self.assertEqual(self.app.roll_label.values["text"], "-3.0°")
        self.app.handle_mqtt_message("robot/telemetry", json.dumps({"imu": {}}))
        self.assertEqual(self.app.tof_labels["front"].values["text"], "--")

    def test_imu_fault_hides_angles_and_preserves_last_valid_heading(self):
        self.app.handle_mqtt_message("robot/telemetry", json.dumps({
            "imu": {"heading": 45, "pitch": 2, "roll": -3, "status": 0}}))
        self.app.handle_mqtt_message("robot/telemetry", json.dumps({
            "imu": {"heading": 0, "pitch": 0, "roll": 0, "status": 1}}))
        self.assertEqual(self.app.robot_heading, 45)
        for label in (self.app.heading_label, self.app.pitch_label, self.app.roll_label):
            self.assertEqual(label.values["text"], "N/A")


if __name__ == "__main__":
    unittest.main()
