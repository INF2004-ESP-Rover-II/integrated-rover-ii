"""Exercise real heading-button handlers and MQTT publishing without moving hardware."""
import ast
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
import time
import unittest

source = Path(__file__).resolve().parents[1] / 'dashboard/robot_dashboard.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
gui = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RobotGUI')
names = {'request_manual_turn', 'cancel_manual_turn', 'manual_turn_timeout',
         'handle_manual_turn_status', 'send_command'}
methods = [n for n in gui.body if isinstance(n, ast.FunctionDef) and n.name in names]
keymap = next(n for n in tree.body if isinstance(n, ast.Assign) and
              any(isinstance(t, ast.Name) and t.id == 'KEYMAP' for t in n.targets))
namespace = {'datetime': datetime, 'time': time, 'TOPIC_COMMAND': 'robot/command',
             'mqtt': SimpleNamespace(MQTT_ERR_SUCCESS=0)}
exec(compile(ast.Module(body=[keymap] + methods, type_ignores=[]), str(source), 'exec'), namespace)


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeGUI:
    def __init__(self):
        self.turn_heading = Value('90')
        self.manual_turn_target = self.manual_turn_job = None
        self.manual_turn_acked = False
        self.connected, self.robot_link, self.estop_active = True, 'live', False
        self.pressed_keys = set()
        self.active_command, self.heartbeat = 'FORWARD', True
        self.auto_mode = Value(False)
        self.jobs, self.published, self.messages, self.pause_reasons = {}, [], [], []
        self.job_count = 0
        self.fail_command = None
        self.root = SimpleNamespace(after=self.after, after_cancel=lambda job: self.jobs.pop(job))
        self.mqtt_client = SimpleNamespace(publish=self.publish)
        self.navigator = SimpleNamespace(sensor=SimpleNamespace(received_at=10.0, age_s=0.0),
            clock=lambda: 10.0, SENSOR_MAX_AGE_S=0.75, TURN_TIMEOUT_S=15, active=False,
            stop=self.pause)

    def after(self, delay, callback):
        self.job_count += 1
        self.jobs[self.job_count] = (delay, callback)
        return self.job_count

    def publish(self, topic, payload, qos):
        self.published.append(payload)
        return SimpleNamespace(rc=1 if payload == self.fail_command else 0)

    def pause(self, reason):
        self.pause_reasons.append(reason)
        self.navigator.active = False
        self.send_command('STOP', quiet=True)

    def cancel_heartbeat(self):
        self.heartbeat = False

    def log(self, message):
        self.messages.append(message)


for name in names:
    setattr(FakeGUI, name, namespace[name])


class ManualTurnTests(unittest.TestCase):
    def test_valid_heading_sends_stop_then_turn_and_starts_timeout(self):
        for value in ('0', ' 90 ', '359'):
            with self.subTest(value=value):
                app = FakeGUI()
                app.turn_heading.set(value)
                app.request_manual_turn()
                self.assertEqual(app.published, ['STOP', 'TURN_TO:%d' % int(value)])
                self.assertIsNone(app.active_command)
                self.assertFalse(app.heartbeat)
                self.assertEqual(app.jobs[app.manual_turn_job][0], 15000)

    def test_invalid_heading_never_sends_a_command(self):
        for value in ('', '-1', '360', '90.5', 'abc', '90:STOP'):
            with self.subTest(value=value):
                app = FakeGUI()
                app.turn_heading.set(value)
                app.request_manual_turn()
                self.assertEqual(app.published, [])

    def test_connection_estop_held_keys_and_heading_freshness_are_checked(self):
        for change in ('broker', 'robot', 'estop', 'key', 'no_heading', 'stale_heading'):
            with self.subTest(change=change):
                app = FakeGUI()
                if change == 'broker':
                    app.connected = False
                elif change == 'robot':
                    app.robot_link = 'lost'
                elif change == 'estop':
                    app.estop_active = True
                elif change == 'key':
                    app.pressed_keys.add('w')
                elif change == 'no_heading':
                    app.navigator.sensor = None
                else:
                    app.navigator.sensor.age_s = 0.8
                app.request_manual_turn()
                self.assertEqual(app.published, [])

    def test_manual_turn_pauses_dfs_and_releases_checkbox(self):
        app = FakeGUI()
        app.navigator.active = True
        app.auto_mode.set(True)
        app.request_manual_turn()
        self.assertFalse(app.navigator.active)
        self.assertFalse(app.auto_mode.get())
        self.assertTrue(app.pause_reasons)
        self.assertEqual(app.published[-1], 'TURN_TO:90')

    def test_failed_stop_or_turn_publish_does_not_leave_an_active_monitor(self):
        for failed in ('STOP', 'TURN_TO:90'):
            with self.subTest(failed=failed):
                app = FakeGUI()
                app.fail_command = failed
                app.request_manual_turn()
                self.assertIsNone(app.manual_turn_target)
                self.assertEqual(app.jobs, {})
                if failed == 'STOP':
                    self.assertEqual(app.published, ['STOP'])

    def test_completion_requires_matching_ack_and_cancels_timeout(self):
        app = FakeGUI()
        app.request_manual_turn()
        app.handle_manual_turn_status('AUTO_RESULT: HEADING_REACHED')
        self.assertEqual(app.manual_turn_target, 90)
        app.handle_manual_turn_status('AUTO_MOVE: TURN_TO:180')
        self.assertFalse(app.manual_turn_acked)
        app.handle_manual_turn_status('AUTO_MOVE: TURN_TO:90')
        app.handle_manual_turn_status('AUTO_RESULT: HEADING_REACHED')
        self.assertIsNone(app.manual_turn_target)
        self.assertEqual(app.jobs, {})
        self.assertTrue(any('complete' in m for m in app.messages))

    def test_timeout_stops_turn(self):
        app = FakeGUI()
        app.request_manual_turn()
        _, callback = app.jobs.pop(app.manual_turn_job)
        callback()
        self.assertEqual(app.published, ['STOP', 'TURN_TO:90', 'STOP'])
        self.assertIsNone(app.manual_turn_target)

    def test_new_target_replaces_old_timeout(self):
        app = FakeGUI()
        app.request_manual_turn()
        old_job = app.manual_turn_job
        app.turn_heading.set('180')
        app.request_manual_turn()
        self.assertNotIn(old_job, app.jobs)
        self.assertEqual(len(app.jobs), 1)
        self.assertEqual(app.manual_turn_target, 180)

    def test_stop_estop_and_reset_cancel_turn_tracking(self):
        for command in ('STOP', 'ESTOP', 'RESET_SAFETY'):
            with self.subTest(command=command):
                app = FakeGUI()
                app.request_manual_turn()
                app.send_command(command)
                self.assertIsNone(app.manual_turn_target)
                self.assertEqual(app.jobs, {})

    def test_keyboard_takeover_stops_turn_before_sending_move(self):
        app = FakeGUI()
        app.request_manual_turn()
        app.send_command('BACKWARD')
        self.assertEqual(app.published[-2:], ['STOP', 'BACKWARD'])
        self.assertIsNone(app.manual_turn_target)


if __name__ == '__main__':
    unittest.main()
