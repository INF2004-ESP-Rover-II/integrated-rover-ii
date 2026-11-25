"""Command heartbeats preserve measured heading and authoritative map state."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

source = Path(__file__).resolve().parents[1] / 'dashboard' / 'robot_dashboard.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'RobotGUI')
names = {'heartbeat_tick', 'schedule_heartbeat', 'cancel_heartbeat'}
namespace = {'HEARTBEAT_MS': 300}
exec(compile(ast.Module(body=[node for node in gui.body if isinstance(node, ast.FunctionDef) and node.name in names],
                        type_ignores=[]), str(source), 'exec'), namespace)


class HeartbeatGUI:
    def __init__(self, command, heading):
        self.active_command = command
        self.navigator = SimpleNamespace(sensor=SimpleNamespace(heading=heading))
        self.heartbeat_job = None
        self.onboard_map = SimpleNamespace(snapshot=object())
        self.sent = []
        self.scheduled = []
        self.cancelled = []
        self.root = SimpleNamespace(after=self.schedule, after_cancel=self.cancelled.append)

    def schedule(self, delay, callback):
        self.scheduled.append((delay, callback))
        return len(self.scheduled)

    def send_command(self, command, **kwargs):
        self.sent.append((command, kwargs))
        return True


for name in names:
    setattr(HeartbeatGUI, name, namespace[name])


class HeadingTests(unittest.TestCase):
    def test_command_refresh_preserves_measured_heading_and_map(self):
        for heading in (0.0, 50.0, 180.0, 359.2):
            for command in ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT', 'ROTATE_LEFT', 'ROTATE_RIGHT'):
                with self.subTest(heading=heading, command=command):
                    app = HeartbeatGUI(command, heading)
                    snapshot = app.onboard_map.snapshot
                    app.heartbeat_tick()
                    self.assertEqual(app.sent, [(command, {'quiet': True})])
                    self.assertEqual(app.navigator.sensor.heading, heading)
                    self.assertIs(app.onboard_map.snapshot, snapshot)
                    self.assertEqual(app.scheduled, [(300, app.heartbeat_tick)])

    def test_stopped_controls_do_not_refresh_or_schedule_movement(self):
        app = HeartbeatGUI(None, 123)
        app.heartbeat_tick()
        self.assertEqual(app.sent, [])
        self.assertEqual(app.scheduled, [])
        self.assertEqual(app.navigator.sensor.heading, 123)


if __name__ == '__main__':
    unittest.main()
