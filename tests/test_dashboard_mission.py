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
from mission_display import parse_mission_report, parse_target_selection, parse_dfs_reply, parse_dfs_tracking, parse_dfs_state, LEG_FIELDS
from onboard_map import OnboardMapCache

SOURCE = ROOT / "dashboard" / "robot_dashboard.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
names = {"update_mission_display", "clear_mission_display", "handle_mqtt_message",
         "handle_mqtt_disconnect", "watchdog_tick", "toggle_auto_mode", "navigation_tick"}
namespace = {"parse_mission_report": parse_mission_report, "datetime": datetime, "time": time,
             "parse_target_selection": parse_target_selection,
             "parse_dfs_reply": parse_dfs_reply,
             "parse_dfs_tracking": parse_dfs_tracking,
             "parse_dfs_state": parse_dfs_state,
             "json": json, "TOPIC_STATUS": "robot/status", "TOPIC_TELEMETRY": "robot/telemetry",
             "TOPIC_MARKER": "camera/marker", "TOPIC_ROVER_CAMERA": "robot/camera",
             "TOPIC_ROVER_MAP": "robot/map",
             "ROBOT_LOST_S": 2.0, "WATCHDOG_CHECK_MS": 250, "TOF_UART_STALE_MS": 500}
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
        self.mission_leg_fields = {key: Variable('--') for key in LEG_FIELDS}
        self.mission_report = None
        self.auto_mode = Variable(False)
        self.dfs_feedback = Variable('')
        self.dfs_feedback_label = Label()
        self.dfs_tracking_text = Variable()
        self.dfs_tracking_report = None
        self.dfs_planner_report = None
        self.dfs_planner_text = Variable()
        self.dfs_threshold = Variable('500')
        self.last_dfs_reply = None
        self.target_selection_report = None
        self.target_selection_status = Variable()
        self.active_command = None
        self.pressed_keys = set()
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
    def test_node_arrival_keeps_dfs_running_through_exit_scan(self):
        app=FakeGUI()
        app.handle_mqtt_message('robot/status','DFS_EVENT:NODE_ARRIVED')
        self.assertTrue(app.auto_mode.get())
        leg={'leg_id':1,'configured':False,'departure_node':None,'next_target_node':None,
             'arrival_reference_node':None,'target_cell':None,'direction':None,'phase':'SCANNING'}
        app.update_mission_display({**report('EXPLORING','TARGET_ARRIVED'), 'leg':leg})
        self.assertTrue(app.auto_mode.get())
        self.assertIn('checking ToF exits',app.dfs_feedback.get())
        state={'active':True,'stage':'SCANNING','current_node':2,'depth':2,'direction':'NORTH',
               'expected_node':0,'detail':'SAMPLING_TOF_EXITS'}
        app.handle_mqtt_message('robot/status','DFS_STATE:'+json.dumps(state))
        self.assertIn('stack depth: 2',app.dfs_planner_text.get())
        self.assertIn('node: 2',app.dfs_planner_text.get())
        self.assertEqual(app.robot_status.get(),'STOPPED')
        self.assertIsNone(app.onboard_map.snapshot)
        app.front_obstacle_snapshot=None
        app.set_front_blocked=lambda value:None
        app.update_current_marker=lambda:None
        app.navigator.tick=lambda:None
        app.navigator.active=False
        app.navigation_tick()
        self.assertTrue(app.auto_mode.get()) # Inactive laptop navigator must not uncheck onboard DFS.
        app.handle_mqtt_message('robot/status','DFS_STATE:'+json.dumps({**state,'depth':-1}))
        self.assertEqual(app.dfs_planner_report.depth,2)

    def test_only_dfs_exhaustion_reports_mission_complete(self):
        app=FakeGUI()
        app.handle_mqtt_message('robot/status','DFS_EVENT:NODE_ARRIVED')
        self.assertTrue(app.auto_mode.get())
        app.handle_mqtt_message('robot/status','DFS_EVENT:COMPLETE')
        self.assertFalse(app.auto_mode.get())
        self.assertIn('DFS complete',app.dfs_feedback.get())
        app.update_mission_display(report('COMPLETE','DFS_COMPLETE'))
        self.assertIn('reachable',app.dfs_feedback.get())
        self.assertIsNone(app.onboard_map.snapshot)
        app.clear_mission_display()
        self.assertIsNone(app.dfs_planner_report)

    def test_opening_threshold_is_a_high_level_dfs_setting(self):
        app=FakeGUI();sent=[]
        app.send_command=lambda command:sent.append(command) or True
        app.dfs_threshold.set('700');app.auto_mode.set(True);app.toggle_auto_mode()
        self.assertEqual(sent,['START_DFS:700'])
        for invalid in ('150','1001','x'):
            app.dfs_threshold.set(invalid);app.auto_mode.set(True);app.toggle_auto_mode()
        self.assertEqual(sent,['START_DFS:700'])
        self.assertIn('151 to 1000',app.dfs_feedback.get())

    def test_tracking_decisions_log_counts_and_ages_without_logging_every_repeat(self):
        app = FakeGUI()
        tracking = {'leg_id': 3, 'active': True, 'phase': 'VERIFYING', 'source': 'REFERENCE',
            'remaining_mm': 0, 'reference_progress_mm': 470, 'reference_model_valid': True,
            'verification_frames': 1, 'result': 'VERIFYING_ARRIVAL', 'camera_present': True,
            'camera_packet_age_ms': 10, 'camera_image_age_ms': 510, 'accepted_image_age_ms': 510}
        app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps(tracking))
        self.assertEqual(len(app.logs), 1)
        self.assertIn('verification: 1/3', app.logs[-1])
        self.assertIn('image=510 ms', app.logs[-1])
        for age in (20, 30, 50):
            app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps({**tracking, 'camera_packet_age_ms': age}))
        self.assertEqual(len(app.logs), 1)
        app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps({**tracking, 'verification_frames': 2}))
        self.assertEqual(len(app.logs), 2)
        self.assertIn('verification: 2/3', app.logs[-1])

    def test_exact_arrival_failure_survives_following_paused_mission_report(self):
        app = FakeGUI()
        app.update_mission_display(report('EXPLORING', 'START_DFS', motion=False))
        app.handle_mqtt_message('robot/status', 'DFS_EVENT:ARRIVAL_VERIFICATION_TIMEOUT')
        message = app.dfs_feedback.get()
        app.update_mission_display(report('PAUSED', 'INPUT_LOST'))
        self.assertEqual(app.dfs_feedback.get(), message)
        self.assertIn('fresh post-brake', app.dfs_feedback.get())
        self.assertEqual(app.dfs_feedback_label.values['fg'], '#e74c3c')

    def test_camera_hold_reports_keep_controls_active_and_show_exact_ages(self):
        app = FakeGUI()
        leg = {'leg_id': 2, 'configured': True, 'departure_node': 1, 'next_target_node': 2,
               'arrival_reference_node': 9, 'target_cell': {'x': 0, 'y': 1}, 'direction': 'NORTH', 'phase': 'HOLDING'}
        app.update_mission_display({**report('EXPLORING', 'START_DFS'), 'leg': leg})
        self.assertTrue(app.auto_mode.get())
        self.assertIn('wheels stopped', app.dfs_feedback.get())
        holding = {'leg_id': 2, 'active': True, 'phase': 'HOLDING', 'source': 'NONE', 'remaining_mm': None,
            'reference_progress_mm': 60, 'reference_model_valid': True, 'verification_frames': 0,
            'result': 'HOLDING_FOR_CAMERA', 'input_reason': 'CAMERA_PACKET_STALE', 'camera_present': True,
            'camera_status': 0, 'camera_packet_age_ms': 350, 'camera_image_age_ms': 880,
            'accepted_image_age_ms': 880, 'recovery_frames': 1, 'driving_ms': 200}
        app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps(holding))
        text = app.dfs_tracking_text.get()
        for expected in ('CAMERA_PACKET_STALE', '350 ms', '880 ms', 'recovery 1/2'):
            self.assertIn(expected, text)
        self.assertEqual(app.robot_status.get(), 'STOPPED')
        self.assertEqual(app.status_events, [])
        self.assertIsNone(app.onboard_map.snapshot)
        for bad in ({**holding, 'camera_packet_age_ms': -1}, {**holding, 'camera_present': 1},
                    {**holding, 'recovery_frames': 3}, {**holding, 'input_reason': 'bad value'}):
            app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps(bad))
            self.assertEqual(app.dfs_tracking_text.get(), text)

    def test_legacy_tracking_packet_from_report_does_not_pollute_generic_status(self):
        app = FakeGUI()
        packet = {'leg_id': 2, 'active': False, 'phase': 'NONE', 'source': 'NONE', 'remaining_mm': None,
            'reference_progress_mm': 60, 'reference_model_valid': True, 'verification_frames': 0,
            'result': 'CAMERA_STALE_OR_TARGET_LOST'}
        app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps(packet))
        self.assertIn('CAMERA_STALE_OR_TARGET_LOST', app.dfs_tracking_text.get())
        self.assertEqual(app.robot_status.get(), 'STOPPED')
        self.assertEqual(app.status_events, [])
        app.handle_mqtt_message('robot/status', 'DFS_EVENT:CAMERA_RECOVERY_TIMEOUT')
        self.assertIn('Camera recovery timed out', app.dfs_feedback.get())
        self.assertFalse(app.auto_mode.get())

    def test_reference_tracking_progress_is_display_only_and_validated(self):
        app = FakeGUI()
        tracking = {'leg_id': 2, 'active': True, 'phase': 'APPROACHING', 'source': 'REFERENCE',
            'remaining_mm': 250, 'reference_progress_mm': 250, 'reference_model_valid': True,
            'verification_frames': 0, 'result': 'APPROACHING'}
        app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps(tracking))
        self.assertIn('REFERENCE', app.dfs_tracking_text.get())
        self.assertIn('~250 mm', app.dfs_tracking_text.get())
        self.assertIsNone(app.onboard_map.snapshot)
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '--')
        before = app.dfs_tracking_text.get()
        for bad in ({}, {**tracking, 'remaining_mm': True}, {**tracking, 'source': []},
                    {**tracking, 'verification_frames': 4}, {**tracking, 'active': 1}):
            app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps(bad))
            self.assertEqual(app.dfs_tracking_text.get(), before)

    def test_verifying_is_still_cancellable_and_arrival_requires_rover_map_echo(self):
        app = FakeGUI()
        leg = {'leg_id': 2, 'configured': True, 'departure_node': 1, 'next_target_node': 2,
               'arrival_reference_node': 9, 'target_cell': {'x': 0, 'y': 1}, 'direction': 'NORTH', 'phase': 'VERIFYING'}
        app.update_mission_display({**report('EXPLORING', 'START_DFS'), 'leg': leg})
        self.assertTrue(app.auto_mode.get())
        self.assertIn('verifying', app.dfs_feedback.get())
        app.handle_mqtt_message('robot/status', 'DFS_EVENT:ARRIVED')
        self.assertFalse(app.auto_mode.get())
        self.assertIn('confirmed arrival', app.dfs_feedback.get())
        self.assertIsNone(app.onboard_map.snapshot)
        app.update_mission_display({**report('COMPLETE', 'TARGET_ARRIVED'), 'leg': {**leg, 'phase': 'COMPLETE'}})
        self.assertFalse(app.auto_mode.get())
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '2')
        self.assertEqual(app.dfs_feedback_label.values['fg'], '#2ecc71')
        self.assertIsNone(app.onboard_map.snapshot) # Only robot/map can commit the displayed position.
        app.clear_mission_display()
        self.assertEqual(app.dfs_tracking_text.get(), 'Tracking: waiting for RoboPico')

    def test_reference_reverse_and_reacquisition_are_onboard_display_only(self):
        app = FakeGUI()
        leg = {'leg_id': 2, 'configured': True, 'departure_node': 1, 'next_target_node': 2,
               'arrival_reference_node': 9, 'target_cell': {'x': 0, 'y': 1}, 'direction': 'NORTH', 'phase': 'BACKING_UP'}
        app.update_mission_display({**report('EXPLORING', 'START_DFS', motion=True), 'leg': leg})
        self.assertTrue(app.auto_mode.get())
        self.assertEqual(app.mission_motion.get(), 'Enabled')
        self.assertIn('reversing', app.dfs_feedback.get())
        tracking = {'leg_id': 2, 'active': True, 'phase': 'BACKING_UP', 'source': 'NONE',
            'remaining_mm': None, 'reference_progress_mm': 420, 'reference_model_valid': True,
            'verification_frames': 0, 'result': 'BACKING_UP_FOR_REFERENCE'}
        app.handle_mqtt_message('robot/status', 'DFS_TRACKING:' + json.dumps(tracking))
        self.assertIn('BACKING_UP_FOR_REFERENCE', app.dfs_tracking_text.get())
        app.update_mission_display({**report('EXPLORING', 'START_DFS'), 'leg': {**leg, 'phase': 'REACQUIRING'}})
        self.assertTrue(app.auto_mode.get())
        self.assertEqual(app.mission_motion.get(), 'Disabled')
        self.assertIn('reacquiring', app.dfs_feedback.get())
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '2')
        self.assertEqual(app.mission_leg_fields['arrival_reference_node'].get(), '9')
        self.assertIsNone(app.onboard_map.snapshot)
        self.assertEqual(app.status_events, [])
        app.handle_mqtt_message('robot/status', 'DFS_EVENT:REFERENCE_REACQUISITION_TIMEOUT')
        self.assertFalse(app.auto_mode.get())
        self.assertIn('Arrival remains unconfirmed', app.dfs_feedback.get())

    def test_reverse_rear_refusal_does_not_claim_arrival(self):
        app = FakeGUI()
        app.handle_mqtt_message('robot/status', 'DFS_EVENT:REAR_BLOCKED')
        self.assertFalse(app.auto_mode.get())
        self.assertIn('250 mm', app.dfs_feedback.get())
        self.assertIsNone(app.onboard_map.snapshot)

    def test_start_dfs_is_high_level_send_only_and_board_reports_missing_start_node(self):
        app = FakeGUI()
        sent = []
        app.send_command = lambda command: sent.append(command) or True
        app.auto_mode.set(True)
        app.toggle_auto_mode()
        self.assertEqual(sent, ['START_DFS:500'])
        self.assertFalse(app.auto_mode.get())
        self.assertIsNone(app.onboard_map.snapshot)
        app.handle_mqtt_message('robot/status', 'DFS_REFUSED:START_NODE_NOT_SET')
        self.assertIn('Set the start node first', app.dfs_feedback.get())
        self.assertEqual(app.dfs_feedback_label.values['fg'], '#e74c3c')
        self.assertFalse(app.auto_mode.get())
        self.assertEqual(app.robot_status.get(), 'STOPPED')
        self.assertEqual(app.status_events, [])
        self.assertEqual(sent, ['START_DFS:500'])

    def test_dfs_acknowledgements_stop_and_input_refusals_only_render(self):
        app = FakeGUI()
        app.handle_mqtt_message('robot/status', 'DFS_ACK:STARTED')
        self.assertTrue(app.auto_mode.get())
        app.handle_mqtt_message('robot/status', 'DFS_ACK:ALREADY_RUNNING')
        self.assertTrue(app.auto_mode.get())
        sent = []
        app.send_command = lambda command: sent.append(command) or True
        app.auto_mode.set(False)
        app.toggle_auto_mode()
        self.assertEqual(sent, ['STOP_DFS'])
        app.handle_mqtt_message('robot/status', 'DFS_ACK:STOPPED')
        self.assertFalse(app.auto_mode.get())
        app.handle_mqtt_message('robot/status', 'DFS_REFUSED:SENSOR_STALE')
        self.assertIn('stale', app.dfs_feedback.get())
        app.handle_mqtt_message('robot/status', 'DFS_EVENT:NEAR_TARGET_UNCONFIRMED')
        self.assertIn('Arrival is not confirmed', app.dfs_feedback.get())
        self.assertIsNone(app.onboard_map.snapshot)
        self.assertEqual(sent, ['STOP_DFS'])

    def test_real_mission_reports_confirm_dfs_running_and_staleness_clears_it(self):
        app = FakeGUI()
        leg = {'leg_id': 1, 'configured': True, 'departure_node': 7, 'next_target_node': 2,
               'arrival_reference_node': 9, 'target_cell': None, 'direction': None, 'phase': 'APPROACHING'}
        app.update_mission_display({**report('EXPLORING', 'START_DFS', motion=True), 'leg': leg})
        self.assertTrue(app.auto_mode.get())
        self.assertEqual(app.mission_motion.get(), 'Enabled')
        app.clear_mission_display('Stale')
        self.assertFalse(app.auto_mode.get())
        for bad in ('DFS_ACK:UNKNOWN', 'DFS_ACK:', 'DFS_REFUSED:bad message'):
            with self.assertRaises(ValueError):
                parse_dfs_reply(bad)

    @staticmethod
    def selection(**changes):
        return {'state': 'LOCKED', 'request': '0123456789abcdef0123456789abcdef',
                'leg_id': 1, 'role': 'DESTINATION', 'direction': 'EAST', 'marker': 12,
                'confirmations': 3, 'available': True, 'range_mm': 500,
                'bearing_cdeg': -125, 'detail': 'LOCKED', **changes}

    def test_automatic_target_and_reference_reports_do_not_issue_selection_commands(self):
        app = FakeGUI()
        automatic = self.selection(role='AUTOMATIC', request='', direction=None, marker=2,
            reference_marker=9, reference_confirmations=3, reference_available=True,
            reference_range_mm=900, reference_bearing_cdeg=150)
        app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + json.dumps(automatic))
        self.assertIn('target 2', app.target_selection_status.get())
        self.assertIn('reference 9', app.target_selection_status.get())
        self.assertIsNone(app.mission_report)
        self.assertIsNone(app.onboard_map.snapshot)
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '--')
        self.assertEqual(app.status_events, [])
        leg = {'leg_id': 1, 'configured': True, 'departure_node': 7, 'next_target_node': 2,
               'arrival_reference_node': 9, 'target_cell': None, 'direction': None, 'phase': 'CONFIGURED'}
        app.update_mission_display({**report(), 'leg': leg})
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '2')
        self.assertEqual(app.mission_leg_fields['arrival_reference_node'].get(), '9')
        self.assertEqual(app.mission_leg_fields['target_cell'].get(), '--')

    def test_automatic_missing_and_late_reference_updates_from_board(self):
        app = FakeGUI()
        automatic = self.selection(role='AUTOMATIC', request='', direction=None, marker=2)
        app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + json.dumps(automatic))
        self.assertIn('reference --', app.target_selection_status.get())
        late = {**automatic, 'reference_marker': 9, 'reference_confirmations': 3,
                'reference_available': True, 'reference_range_mm': 900, 'reference_bearing_cdeg': 100}
        app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + json.dumps(late))
        count = len(app.logs)
        app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + json.dumps({**late, 'reference_range_mm': 850}))
        self.assertEqual(len(app.logs), count)
        self.assertIn('850 mm', app.target_selection_status.get())
        app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + json.dumps({**late, 'reference_available': False}))
        self.assertEqual(app.target_selection_report.reference_marker, 9)
        self.assertIn('unavailable/unconfirmed', app.target_selection_status.get())

    def test_selection_reports_do_not_change_robot_status_map_or_leg_and_log_only_changes(self):
        app = FakeGUI()
        for value in (self.selection(), self.selection(), self.selection(range_mm=450)):
            app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + json.dumps(value))
        self.assertEqual(len(app.logs), 1)
        self.assertIn('DESTINATION marker 12', app.target_selection_status.get())
        self.assertIn('450 mm', app.target_selection_status.get())
        self.assertEqual(app.robot_status.get(), 'STOPPED')
        self.assertEqual(app.status_events, [])
        self.assertIsNone(app.onboard_map.snapshot)
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '--')
        app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + json.dumps(
            self.selection(available=False, detail='LOCKED_OBSERVATION_UNAVAILABLE')))
        self.assertEqual(app.target_selection_report.marker, 12)
        self.assertIn('observation unavailable', app.target_selection_status.get())
        self.assertEqual(len(app.logs), 2)
        app.handle_mqtt_message('robot/status', 'TARGET_REFUSED:LEG_ALREADY_PENDING')
        self.assertEqual(app.robot_status.get(), 'STOPPED')

    def test_bad_selection_reports_preserve_display(self):
        app = FakeGUI()
        app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + json.dumps(self.selection()))
        before = app.target_selection_report
        for bad in ('broken', '{}', 'null', json.dumps(self.selection(marker=True)),
                    json.dumps(self.selection(direction=[])), json.dumps(self.selection(state=[])),
                    json.dumps(self.selection(available=1)), json.dumps(self.selection(confirmations=4)),
                    json.dumps(self.selection(request='invalid')), json.dumps(self.selection(marker=256)),
                    json.dumps(self.selection(reference_marker=True)), json.dumps(self.selection(reference_confirmations=4)),
                    json.dumps(self.selection(reference_available=1))):
            app.handle_mqtt_message('robot/status', 'TARGET_SELECTION:' + bad)
            self.assertEqual(app.target_selection_report, before)

    def test_selecting_phase_and_target_changes_are_driven_only_by_mission_reports(self):
        app = FakeGUI()
        leg = {'leg_id': 1, 'configured': False, 'departure_node': 7, 'next_target_node': None,
               'arrival_reference_node': None, 'target_cell': {'x': 1, 'y': 0},
               'direction': 'EAST', 'phase': 'SELECTING'}
        self.assertTrue(app.update_mission_display({**report(), 'leg': leg}))
        self.assertEqual(app.mission_leg_fields['phase'].get(), 'SELECTING')
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '--')
        locked = {**leg, 'configured': True, 'next_target_node': 12, 'arrival_reference_node': 12, 'phase': 'CONFIGURED'}
        self.assertTrue(app.update_mission_display({**report(), 'leg': locked}))
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '12')
        app.clear_mission_display()
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '--')
        self.assertIsNone(app.target_selection_report)

    def test_unset_and_configured_leg_reports_coalesce_across_both_input_paths(self):
        app = FakeGUI()
        unset = {'leg_id': 0, 'configured': False, 'departure_node': None, 'next_target_node': None,
                 'arrival_reference_node': None, 'target_cell': None, 'direction': None, 'phase': 'NONE'}
        status = text(report()) + (' leg_id=0 leg_configured=0 departure_node=0 next_target_node=0 '
            'arrival_reference_node=0 target_cell=UNKNOWN direction=UNKNOWN leg_phase=NONE')
        app.handle_mqtt_message('robot/status', status)
        app.handle_mqtt_message('robot/telemetry', json.dumps({'mission': {**report(), 'leg': unset}}))
        self.assertEqual(len(app.logs), 1)
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '--')
        self.assertEqual(app.mission_leg_fields['phase'].get(), 'NONE')
        self.assertEqual(app.mission_leg_fields['configured'].get(), 'No')
        leg = {**unset, 'leg_id': 42, 'configured': True, 'departure_node': 7, 'next_target_node': 12,
               'arrival_reference_node': 3, 'target_cell': {'x': 1, 'y': 0}, 'direction': 'EAST', 'phase': 'CONFIGURED'}
        app.handle_mqtt_message('robot/telemetry', json.dumps({'mission': {**report(), 'leg': leg}}))
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '12')
        self.assertEqual(app.mission_leg_fields['arrival_reference_node'].get(), '3')
        self.assertEqual(app.mission_leg_fields['departure_node'].get(), '7')
        self.assertEqual(app.mission_leg_fields['target_cell'].get(), '(1, 0)')
        self.assertEqual(app.robot_status.get(), 'STOPPED')
        self.assertIsNone(app.onboard_map.snapshot) # Reporting must not create a map/arrival.
        matching_status = text(report()) + (' leg_id=42 leg_configured=1 departure_node=7 next_target_node=12 '
            'arrival_reference_node=3 target_cell=1,0 direction=EAST leg_phase=CONFIGURED')
        app.handle_mqtt_message('robot/status', matching_status)
        self.assertEqual(len(app.logs), 2)

    def test_leg_reset_origin_cell_and_older_firmware_have_distinct_displays(self):
        app = FakeGUI()
        leg = {'leg_id': 9, 'configured': True, 'departure_node': 12, 'next_target_node': 7,
               'arrival_reference_node': 3, 'target_cell': {'x': 0, 'y': 0}, 'direction': 'WEST', 'phase': 'CONFIGURED'}
        app.update_mission_display({**report(), 'leg': leg})
        self.assertEqual(app.mission_leg_fields['target_cell'].get(), '(0, 0)')
        clear = {**leg, 'configured': False, 'departure_node': None, 'next_target_node': None,
                 'arrival_reference_node': None, 'target_cell': None, 'direction': None, 'phase': 'NONE'}
        app.update_mission_display({**report(), 'leg': clear})
        self.assertEqual(app.mission_leg_fields['leg_id'].get(), '9')
        self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '--')
        app.update_mission_display(report())
        self.assertEqual(app.mission_leg_fields['phase'].get(), 'Unknown')
        self.assertEqual(app.mission_leg_fields['leg_id'].get(), '--')

    def test_invalid_or_partial_leg_data_cannot_replace_known_fields(self):
        app = FakeGUI()
        leg = {'leg_id': 42, 'configured': True, 'departure_node': 7, 'next_target_node': 12,
               'arrival_reference_node': 3, 'target_cell': {'x': -1, 'y': 2}, 'direction': 'WEST', 'phase': 'CONFIGURED'}
        app.update_mission_display({**report(), 'leg': leg})
        for bad in ({}, {**leg, 'next_target_node': 0}, {**leg, 'next_target_node': True},
                    {**leg, 'target_cell': {'x': 40000, 'y': 0}}, {**leg, 'configured': 1},
                    {**leg, 'direction': 'UP'}, {**leg, 'phase': 'MOVING'}, {**leg, 'leg_id': -1}):
            self.assertFalse(app.update_mission_display({**report(), 'leg': bad}))
            self.assertEqual(app.mission_leg_fields['next_target_node'].get(), '12')
        self.assertFalse(app.update_mission_display(text(report()) + ' leg_id=42'))
        self.assertEqual(app.mission_leg_fields['target_cell'].get(), '(-1, 2)')

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
