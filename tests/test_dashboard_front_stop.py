"""Exercise the actual keyboard handlers without opening Tk or connecting MQTT."""
import ast
from pathlib import Path
from types import SimpleNamespace
import time
import math
import unittest

source = Path(__file__).resolve().parents[1] / "dashboard" / "robot_dashboard.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
names = {"set_front_blocked", "start_hold", "stop_hold", "finish_key_release", "navigation_tick",
         "update_tof_readings", "clear_tof_readings", "trigger_estop"}
methods = [node for node in gui.body if isinstance(node, ast.FunctionDef) and node.name in names]
keymap = next(node for node in tree.body if isinstance(node, ast.Assign) and
              any(isinstance(target, ast.Name) and target.id == "KEYMAP" for target in node.targets))
namespace = {"time": time, "math": math, "TOF_UART_STALE_MS": 500,
             "ARM_COMMANDS": {'CLAW_OPEN', 'CLAW_CLOSE', 'ARM_UP', 'ARM_DOWN'}}
exec(compile(ast.Module(body=[keymap] + methods, type_ignores=[]), str(source), "exec"), namespace)


class FakeGUI:
    def __init__(self):
        self.front_blocked = False
        self.front_obstacle_snapshot = None
        self.last_tof_rx = None
        self.forward_requires_release = False
        self.pressed_keys = {"w"}
        self.release_jobs = {}
        self.active_command = "FORWARD"
        self.commands = []
        self.heartbeat = True
        self.auto_mode = SimpleNamespace(set=lambda value: None, get=lambda: False)
        self.navigator = SimpleNamespace(stop=lambda reason: None, tick=lambda: None, active=False)
        self.mapping = SimpleNamespace(sync=lambda: None)
        self.root = SimpleNamespace(after=lambda *args: None)
        label = SimpleNamespace(config=lambda **kwargs: None)
        self.tof_labels = dict.fromkeys(('left', 'front', 'right'), label)
        self.tof_state_labels = dict(self.tof_labels)
        self.tof_summary_label = self.tof_age_label = label
        self.update_estop_ui = lambda: None

    def cancel_heartbeat(self):
        self.heartbeat = False

    def schedule_heartbeat(self):
        self.heartbeat = True

    def send_command(self, command, quiet=False):
        self.commands.append(command)

    def log(self, message):
        pass


for name in names:
    setattr(FakeGUI, name, namespace[name])


class FrontStopTests(unittest.TestCase):
    def test_range_overrides_old_stop_and_urgent_flags(self):
        for distance in (0, 1, 149, 150, 151, 199, 200, 1000, 1023, 65535, None):
            for fault in (0, 2):
                with self.subTest(distance=distance, fault=fault):
                    app = FakeGUI()
                    app.update_tof_readings({'distance_mm': {'front': distance},
                                             'stop_required': True, 'urgent_mask': 2,
                                             'fault_mask': fault, 'age_ms': 0})
                    app.navigation_tick()
                    self.assertEqual(app.front_blocked,
                                     distance in (1, 149) and not fault)
                    app.start_hold('FORWARD')
                    if not app.front_blocked:
                        self.assertEqual(app.active_command, 'FORWARD')

    def test_manual_estop_sends_explicit_estop_command(self):
        app = FakeGUI()
        app.trigger_estop()
        self.assertEqual(app.commands, ['ESTOP'])
        self.assertTrue(app.estop_active)
        self.assertIsNone(app.active_command)

    def test_telemetry_updates_forward_guard_on_control_tick(self):
        app = FakeGUI()
        app.front_obstacle_snapshot, app.last_tof_rx = True, time.time()
        app.navigation_tick()
        self.assertTrue(app.front_blocked)
        self.assertEqual(app.commands, ["STOP"])
        app.start_hold("BACKWARD")
        app.navigation_tick()
        self.assertEqual(app.active_command, "BACKWARD")
        app.front_obstacle_snapshot, app.last_tof_rx = False, time.time()
        app.navigation_tick()
        self.assertFalse(app.front_blocked)
        self.assertEqual(app.active_command, "BACKWARD")

    def test_stale_reading_does_not_keep_w_blocked(self):
        app = FakeGUI()
        app.set_front_blocked(True)
        app.front_obstacle_snapshot, app.last_tof_rx = False, time.time() - 1
        app.navigation_tick()
        self.assertFalse(app.front_blocked)

    def test_detection_stops_and_only_forward_is_disabled(self):
        app = FakeGUI()
        app.set_front_blocked(True)
        self.assertEqual(app.commands, ["STOP"])
        self.assertIsNone(app.active_command)
        self.assertFalse(app.heartbeat)
        app.start_hold("FORWARD")
        self.assertEqual(app.commands, ["STOP"])
        for command in namespace["KEYMAP"].values():
            if command != "FORWARD":
                app.start_hold(command)
                self.assertEqual(app.active_command, command)
                self.assertEqual(app.commands[-1], command)

    def test_repeated_detection_does_not_interrupt_reverse(self):
        app = FakeGUI()
        app.set_front_blocked(True)
        app.start_hold("BACKWARD")
        app.set_front_blocked(True)
        self.assertEqual(app.active_command, "BACKWARD")
        self.assertEqual(app.commands, ["STOP", "BACKWARD"])

    def test_releasing_reverse_does_not_fall_back_to_held_w(self):
        for clear in (False,):
            with self.subTest(clear=clear):
                app = FakeGUI()
                app.set_front_blocked(True)
                app.pressed_keys.add("s")
                app.start_hold("BACKWARD")
                if clear:
                    app.set_front_blocked(False)
                app.finish_key_release("s")
                self.assertIsNone(app.active_command)
                self.assertFalse(app.heartbeat)
                self.assertEqual(app.commands[-1], "STOP")

    def test_clear_enables_forward_without_reset(self):
        app = FakeGUI()
        app.set_front_blocked(True)
        app.set_front_blocked(False)
        app.start_hold("FORWARD")
        self.assertEqual(app.active_command, "FORWARD")
        self.assertTrue(app.heartbeat)
        self.assertNotIn("RESET_SAFETY", app.commands)

    def test_first_detection_does_not_interrupt_reverse(self):
        app = FakeGUI()
        app.start_hold("BACKWARD")
        app.commands.clear()
        app.set_front_blocked(True)
        self.assertEqual(app.active_command, "BACKWARD")
        self.assertEqual(app.commands, [])


if __name__ == "__main__":
    unittest.main()
