"""Exercise the actual keyboard handlers without opening Tk or connecting MQTT."""
import ast
from pathlib import Path
from types import SimpleNamespace
import time
import unittest

source = Path(__file__).resolve().parents[1] / "dashboard" / "robot_dashboard.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
names = {"set_front_blocked", "start_hold", "stop_hold", "finish_key_release", "navigation_tick"}
methods = [node for node in gui.body if isinstance(node, ast.FunctionDef) and node.name in names]
keymap = next(node for node in tree.body if isinstance(node, ast.Assign) and
              any(isinstance(target, ast.Name) and target.id == "KEYMAP" for target in node.targets))
namespace = {"time": time, "TOF_UART_STALE_MS": 500,
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
        self.auto_mode = SimpleNamespace(set=lambda value: None)
        self.navigator = SimpleNamespace(stop=lambda reason: None, tick=lambda: None, active=False)
        self.root = SimpleNamespace(after=lambda *args: None)

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

    def test_stale_clear_snapshot_cannot_enable_w(self):
        app = FakeGUI()
        app.set_front_blocked(True)
        app.front_obstacle_snapshot, app.last_tof_rx = False, time.time() - 1
        app.navigation_tick()
        self.assertTrue(app.front_blocked)

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
        for clear in (False, True):
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

    def test_clear_requires_release_and_new_forward_press(self):
        app = FakeGUI()
        app.set_front_blocked(True)
        app.set_front_blocked(False)
        self.assertIsNone(app.active_command)
        app.start_hold("FORWARD")
        self.assertIsNone(app.active_command)
        app.finish_key_release("w")
        app.start_hold("FORWARD")
        self.assertEqual(app.active_command, "FORWARD")
        self.assertTrue(app.heartbeat)
        self.assertNotIn("RESET_SAFETY", app.commands)


if __name__ == "__main__":
    unittest.main()
