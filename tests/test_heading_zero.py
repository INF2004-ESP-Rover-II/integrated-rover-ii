"""Zero-yaw button lifecycle and heading-reference changes in the real navigator."""
import ast
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace

from test_dashboard_manual_turn import FakeGUI

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'dashboard'))
from laptop_navigation import LaptopNavigator
from dfs_explorer import Direction, MarkerType, State

source = ROOT / 'dashboard/robot_dashboard.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
gui = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RobotGUI')
names = {'request_heading_zero', 'cancel_heading_zero', 'heading_zero_timeout',
         'handle_heading_zero_status', 'start_hold', 'toggle_auto_mode'}
methods = [n for n in gui.body if isinstance(n, ast.FunctionDef) and n.name in names]
namespace = {}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), namespace)


class ZeroGUI(FakeGUI):
    def __init__(self):
        super().__init__()
        self.heading_zero_pending = False
        self.heading_zero_job = None
        self.odometry_lock = threading.Lock()
        self.robot_heading = 123.0
        self.heading_text = {}
        self.heading_label = SimpleNamespace(config=lambda **kwargs: self.heading_text.update(kwargs))
        self.arrival_resets = 0
        self.navigator._reset_arrival = self.reset_arrival

    def reset_arrival(self):
        self.arrival_resets += 1


for name in names:
    setattr(ZeroGUI, name, namespace[name])


class ZeroButtonTests(unittest.TestCase):
    def test_stop_then_zero_and_wait_for_ack_before_changing_display(self):
        app = ZeroGUI()
        app.request_heading_zero()
        self.assertEqual(app.published, ['STOP', 'ZERO_HEADING'])
        self.assertTrue(app.heading_zero_pending)
        self.assertEqual(app.robot_heading, 123)
        self.assertEqual(app.jobs[app.heading_zero_job][0], 5000)
        app.handle_heading_zero_status('HEADING_ZERO_PENDING')
        self.assertTrue(app.heading_zero_pending)
        app.handle_heading_zero_status('HEADING_ZEROED')
        self.assertFalse(app.heading_zero_pending)
        self.assertEqual(app.robot_heading, 0)
        self.assertEqual(app.heading_text['text'], '0.0°')
        self.assertIsNone(app.navigator.sensor)
        self.assertIsNone(app.navigator.visible_marker)
        self.assertEqual(app.arrival_resets, 1)
        self.assertEqual(app.jobs, {})

    def test_zero_pauses_dfs_and_cancels_manual_turn(self):
        app = ZeroGUI()
        app.request_manual_turn()
        old_job = app.manual_turn_job
        app.navigator.active = True
        app.auto_mode.set(True)
        app.request_heading_zero()
        self.assertFalse(app.navigator.active)
        self.assertFalse(app.auto_mode.get())
        self.assertIsNone(app.manual_turn_target)
        self.assertNotIn(old_job, app.jobs)
        self.assertEqual(app.published[-1], 'ZERO_HEADING')

    def test_invalid_link_keys_or_heading_never_send_zero(self):
        for condition in ('broker', 'robot', 'key', 'heading', 'stale'):
            with self.subTest(condition=condition):
                app = ZeroGUI()
                if condition == 'broker':
                    app.connected = False
                elif condition == 'robot':
                    app.robot_link = 'lost'
                elif condition == 'key':
                    app.pressed_keys.add('w')
                elif condition == 'heading':
                    app.navigator.sensor = None
                else:
                    app.navigator.sensor.age_s = 1
                app.request_heading_zero()
                self.assertEqual(app.published, [])

    def test_zero_does_not_release_estop_or_send_safety_reset(self):
        app = ZeroGUI()
        app.estop_active = True
        app.request_heading_zero()
        app.handle_heading_zero_status('HEADING_ZEROED')
        self.assertTrue(app.estop_active)
        self.assertNotIn('RESET_SAFETY', app.published)

    def test_movement_and_dfs_are_blocked_until_zero_completes(self):
        app = ZeroGUI()
        app.request_heading_zero()
        commands = list(app.published)
        app.request_heading_zero()
        app.request_manual_turn()
        app.start_hold('FORWARD')
        app.auto_mode.set(True)
        app.toggle_auto_mode()
        self.assertFalse(app.auto_mode.get())
        self.assertFalse(app.send_command('FORWARD'))
        self.assertIsNone(app.active_command)
        self.assertEqual(app.published, commands)

    def test_refusal_or_publish_failure_never_zero_display(self):
        app = ZeroGUI()
        app.request_heading_zero()
        app.handle_heading_zero_status('HEADING_ZERO_REFUSED')
        self.assertEqual(app.robot_heading, 123)
        self.assertFalse(app.heading_zero_pending)
        app.fail_command = 'ZERO_HEADING'
        app.request_heading_zero()
        self.assertFalse(app.heading_zero_pending)
        self.assertEqual(app.jobs, {})

    def test_timeout_sends_stop_and_reports_missing_firmware_ack(self):
        app = ZeroGUI()
        app.request_heading_zero()
        _, callback = app.jobs.pop(app.heading_zero_job)
        callback()
        self.assertFalse(app.heading_zero_pending)
        self.assertEqual(app.published[-1], 'STOP')
        self.assertEqual(app.robot_heading, 123)


class HeadingReferenceTests(unittest.TestCase):
    def setUp(self):
        self.commands = []
        self.nav = LaptopNavigator(lambda cmd: self.commands.append(cmd) or True, clock=lambda: 10.0)

    def packet(self, stamp=1, heading=123, epoch=0, offset=0):
        return {'timestamp': stamp, 'heading_epoch': epoch, 'heading_offset_deg': offset,
                'imu': {'status': 0, 'heading': heading},
                'obstacle': {'age_ms': 0, 'distance_mm': {}, 'stop_required': False, 'fault_mask': 0}}

    def test_zero_updates_absolute_heading_and_preserves_existing_map_direction(self):
        self.nav.sensor_received(self.packet())
        self.nav.map_zero_deg = 100
        self.nav.explorer.start(1, 4, 14, MarkerType.JUNCTION, Direction.NORTH)
        self.nav._arrival_marker, self.nav._arrival_count = 14, 5
        self.nav.sensor_received(self.packet(stamp=2, heading=0, epoch=1, offset=123))
        self.assertEqual(self.nav.sensor.heading, 0)
        self.assertEqual(self.nav.initial_sensor_heading, 0)
        self.assertEqual(self.nav.map_zero_deg, 337)
        self.assertEqual(list(self.nav.explorer.nodes), [14])
        self.assertEqual(self.nav._arrival_count, 0)
        self.nav.sensor_received(self.packet(stamp=3, heading=6, epoch=1, offset=123))
        self.assertEqual(self.nav.map_zero_deg, 337)

    def test_same_sensor_timestamp_is_accepted_when_its_reference_changes(self):
        self.nav.sensor_received(self.packet())
        self.nav.sensor_received(self.packet(heading=0, epoch=1, offset=123))
        self.assertEqual(self.nav.sensor.heading, 0)

    def test_reference_change_pauses_an_active_mission(self):
        self.nav.sensor_received(self.packet())
        self.nav.explorer.start(1, 4, 14, MarkerType.JUNCTION, Direction.NORTH)
        self.nav.active = True
        self.nav.sensor_received(self.packet(stamp=2, heading=0, epoch=1, offset=123))
        self.assertFalse(self.nav.active)
        self.assertEqual(self.nav.explorer.state, State.PAUSED)
        self.assertEqual(self.commands[-1], 'STOP')

    def test_repeated_zero_across_angle_wrap_preserves_map(self):
        self.nav.sensor_received(self.packet(heading=20, epoch=1, offset=350))
        self.nav.map_zero_deg = 15
        self.nav.sensor_received(self.packet(stamp=2, heading=0, epoch=2, offset=10))
        self.assertEqual(self.nav.initial_sensor_heading, 0)
        self.assertEqual(self.nav.map_zero_deg, 355)


if __name__ == '__main__':
    unittest.main()
