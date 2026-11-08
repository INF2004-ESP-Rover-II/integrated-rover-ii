"""Actual mission display/message handlers, without Tk, MQTT or motor output."""
import ast
from datetime import datetime
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dashboard"))
from mission_display import parse_mission_report
from onboard_map import OnboardMapCache

SOURCE = ROOT / "dashboard" / "robot_dashboard.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
names = {"update_mission_display", "clear_mission_display", "handle_mqtt_message",
         "handle_mqtt_disconnect", "watchdog_tick"}
namespace = {"parse_mission_report": parse_mission_report, "datetime": datetime, "time": time,
             "json": json, "TOPIC_STATUS": "robot/status", "TOPIC_TELEMETRY": "robot/telemetry",
             "TOPIC_MARKER": "camera/marker", "TOPIC_ROVER_CAMERA": "robot/camera",
             "TOPIC_ROVER_MAP": "robot/map",
             "ROBOT_LOST_S": 2.0, "WATCHDOG_CHECK_MS": 250}
exec(compile(ast.Module(body=[node for node in gui.body if isinstance(node, ast.FunctionDef)
                             and node.name in names], type_ignores=[]), str(SOURCE), "exec"), namespace)


class Variable:
    def __init__(self, value="Unknown"):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class Label:
    def __init__(self):
        self.values = {}

    def config(self, **values):
        self.values.update(values)


def report(state="IDLE", reason="BOOT", test=False, motion=False, revision=0):
    return {"state": state, "reason": reason, "test_mode": test,
            "motion_enabled": motion, "revision": revision}


def text(value):
    return "MISSION_STATE:%s test=%d motion=%s reason=%s revision=%d" % (
        value["state"], value["test_mode"], "enabled" if value["motion_enabled"] else "disabled",
        value["reason"], value["revision"])


class FakeGUI:
    def __init__(self):
        self.logs = []
        self.status_events = []
        self.sensor_events = []
        self.robot_status = Variable("STOPPED")
        self.mission_state = Variable()
        self.mission_reason = Variable("--")
        self.mission_mode = Variable()
        self.mission_motion = Variable("--")
        self.mission_report = None
        self.last_mission_rx = None
        self.last_mission_reply = None
        self.onboard_map = OnboardMapCache()
        self.onboard_map_status = Variable()
        self.mission_state_label = Label()
        self.robot_status_label = Label()
        self.last_msg_time = Variable()
        self.msg_count = 0
        self.stop_sent_at = None
        self.navigator = SimpleNamespace(robot_status_received=self.status_events.append,
                                         sensor_received=self.sensor_events.append, stop=lambda reason: None)
        self.manual_turn_target = None
        self.heading_zero_pending = False
        self.connected = True
        self.last_tof_rx = None
        self.last_telemetry_rx = time.time()
        self.connect_time = time.time()
        self.robot_link = "live"
        self.root = SimpleNamespace(after=lambda *args: None)
        self.heading_label, self.pitch_label, self.roll_label = Label(), Label(), Label()
        self.broker_connected, self.broker_status_label = Variable(), Label()
        self.odometry_lock = threading.Lock()
        self.ultrasonic = dict.fromkeys(("left", "front", "right", "back"), 0)
        self.ultrasonic_label = Label()

    def log(self, value):
        self.logs.append(value)

    def clear_tof_readings(self, *args):
        pass

    def clear_rover_camera_display(self):
        pass

    def update_tof_readings(self, *args):
        pass

    def set_robot_link(self, state, age=0):
        self.robot_link = state

    def send_command(self, *args, **kwargs):
        raise AssertionError("Mission visualisation must not send motor commands")


for name in names:
    setattr(FakeGUI, name, namespace[name])


class MissionDisplayTests(unittest.TestCase):
    def test_status_reports_have_separate_fields_and_do_not_drive_navigator(self):
        app = FakeGUI()
        app.handle_mqtt_message("robot/status", text(report()))
        self.assertEqual(app.robot_status.get(), "STOPPED")
        self.assertEqual(app.mission_state.get(), "IDLE")
        self.assertEqual(app.mission_reason.get(), "BOOT")
        self.assertEqual(app.mission_mode.get(), "Inactive")
        self.assertEqual(app.mission_motion.get(), "Disabled")
        self.assertEqual(app.status_events, [])

        app.handle_mqtt_message("robot/status", "RUNNING")
        self.assertEqual(app.robot_status.get(), "RUNNING")
        app.handle_mqtt_message("robot/status", text(report("READY", "TEST_READY", True, revision=1)))
        self.assertEqual(app.robot_status.get(), "RUNNING")
        self.assertEqual(app.mission_mode.get(), "Test (stationary)")
        self.assertEqual(app.status_events, ["RUNNING"])

    def test_telemetry_and_periodic_status_share_one_display_and_log_changes_once(self):
        app = FakeGUI()
        value = report("EXPLORING", "TEST_START", True, revision=2)
        for _ in range(5):
            app.handle_mqtt_message("robot/status", text(value))
            app.handle_mqtt_message("robot/telemetry", json.dumps({"mission": value}))
        self.assertEqual(len(app.logs), 1)
        self.assertEqual(app.robot_status.get(), "STOPPED")
        self.assertEqual(app.mission_state.get(), "EXPLORING")
        self.assertEqual(len(app.sensor_events), 5)
        value["revision"] = 3
        app.update_mission_display(value)
        self.assertEqual(len(app.logs), 1)
        app.update_mission_display(report("FAULT", "ESTOP", True, revision=4))
        self.assertEqual(len(app.logs), 2)
        self.assertEqual(app.mission_state_label.values["fg"], "#e74c3c")

    def test_acknowledgements_and_refusals_do_not_replace_robot_status(self):
        app = FakeGUI()
        for value in ("MISSION_TEST:ACCEPTED state=READY", "MISSION_REFUSED:ONBOARD_DFS_NOT_IMPLEMENTED"):
            app.handle_mqtt_message("robot/status", value)
            app.handle_mqtt_message("robot/status", value)
        self.assertEqual(app.robot_status.get(), "STOPPED")
        self.assertEqual(len(app.logs), 2)
        self.assertEqual(app.status_events, [])

    def test_invalid_reports_preserve_existing_display_and_normal_telemetry(self):
        app = FakeGUI()
        app.update_mission_display(report())
        invalid = [None, [], {}, {**report(), "state": "DRIVING"},
                   {**report(), "test_mode": "false"}, {**report(), "motion_enabled": 0},
                   {**report(), "revision": True}, {**report(), "revision": -1},
                   {**report(), "reason": "bad reason"}, {**report(), "state": []},
                   "MISSION_STATE:FAULT", text(report()) + " test=1"]
        for value in invalid:
            self.assertFalse(app.update_mission_display(value))
        app.handle_mqtt_message("robot/status", "MISSION_STATE:BAD")
        app.handle_mqtt_message("robot/telemetry", json.dumps({"mission": {"state": "BAD"}}))
        self.assertEqual(app.mission_state.get(), "IDLE")
        self.assertEqual(app.robot_status.get(), "STOPPED")
        self.assertEqual(len(app.logs), 1)
        self.assertEqual(len(app.sensor_events), 1)

    def test_stale_and_disconnected_views_do_not_present_old_state_as_live(self):
        app = FakeGUI()
        app.update_mission_display(report())
        now = time.time()
        with patch("time.time", return_value=now + 3):
            app.watchdog_tick()
        self.assertEqual(app.mission_state.get(), "Stale")
        self.assertIsNone(app.mission_report)
        app.update_mission_display(report("READY", "TEST_READY", True, revision=1))
        self.assertEqual(app.mission_state.get(), "READY")
        app.handle_mqtt_disconnect(1)
        self.assertEqual(app.mission_state.get(), "Unknown")
        self.assertEqual(app.mission_motion.get(), "--")


if __name__ == "__main__":
    unittest.main()
