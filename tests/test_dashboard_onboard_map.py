"""Set-start is send-only; only validated RoboPico snapshots own map/current node."""
import ast
import copy
import json
from pathlib import Path
import sys
import time
import unittest
import uuid

from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from matplotlib.ticker import MaxNLocator
from matplotlib.colors import to_rgba

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'dashboard'))
from onboard_map import OnboardMapCache
from test_dashboard_camera_relay import CameraGUI, Variable, observation

tree = ast.parse((ROOT / 'dashboard/robot_dashboard.py').read_text(encoding='utf-8'))
gui = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RobotGUI')
names = {'set_start_node', 'receive_onboard_map', 'render_onboard_map', 'update_grid_map', 'update_marker_map', 'toggle_auto_mode'}
namespace = {'time': time, 'uuid': uuid, 'json': json, 'Rectangle': Rectangle, 'MaxNLocator': MaxNLocator}
exec(compile(ast.Module(body=[n for n in gui.body if isinstance(n, ast.FunctionDef) and n.name in names],
                        type_ignores=[]), 'onboard_map_handlers', 'exec'), namespace)


def node(marker_id, x, y, visited=False):
    return {'marker_id': marker_id, 'x': x, 'y': y, 'visited': visited, 'exits': [0, 0, 0, 0]}


def snapshot(nodes=None, current=7, revision=1, boot='0000000012345678', request=''):
    if nodes is None:
        nodes = [node(7, 0, 0, True)]
    bounds = None if not nodes else {'min_x': min(n['x'] for n in nodes), 'max_x': max(n['x'] for n in nodes),
                                   'min_y': min(n['y'] for n in nodes), 'max_y': max(n['y'] for n in nodes)}
    return {'schema': 1, 'source': 'robo_pico', 'boot_id': boot, 'revision': revision, 'capacity': 64,
            'last_request': request, 'current_node': current if nodes else None, 'nodes': nodes,
            'bounds': bounds, 'columns': bounds['max_x'] - bounds['min_x'] + 1 if nodes else 0,
            'rows': bounds['max_y'] - bounds['min_y'] + 1 if nodes else 0}


class MapGUI(CameraGUI):
    def __init__(self):
        super().__init__()
        self.pending_start_request = None
        self.start_node_id = Variable('7')
        self.start_node_status = Variable()
        self.active_command = None
        self.pressed_keys = set()
        self.auto_mode = Variable(False)
        self.sent = []
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        self.draws = []
        self.canvas = type('Canvas', (), {'draw_idle': lambda canvas: self.draws.append(True)})()

    def send_command(self, command, **kwargs):
        self.sent.append(command)
        return True


for name in names:
    setattr(MapGUI, name, namespace[name])


class OnboardMapTests(unittest.TestCase):
    def test_onboard_sightings_are_blue_and_only_arrival_snapshot_turns_node_green(self):
        app = MapGUI()
        data = snapshot([node(1, 0, 0, True)], current=1)
        data['observed_nodes'] = [{'marker_id': 2, 'x': 0, 'y': 1}, {'marker_id': 9, 'x': 0, 'y': 2}]
        app.receive_onboard_map(data)
        self.assertEqual(len(app.onboard_map.snapshot.nodes), 1)
        self.assertEqual(len(app.onboard_map.snapshot.observed_nodes), 2)
        self.assertEqual(app.dfs_current_node_text.get(), '1')
        self.assertEqual(tuple(app.ax.get_ylim()), (-0.5, 2.5))
        self.assertEqual([p.get_facecolor() for p in app.ax.patches],
                         [to_rgba('#27ae60', .65), to_rgba('#2980b9', .65), to_rgba('#2980b9', .65)])
        self.assertEqual([p.get_linestyle() for p in app.ax.patches], ['-', '--', '--'])
        before = app.onboard_map.snapshot
        app.handle_mqtt_message('robot/camera', json.dumps(observation(markers=[
            {'id': 2, 'range_mm': 10, 'bearing_cdeg': 0, 'quality': 0}])))
        self.assertIs(app.onboard_map.snapshot, before)
        app.handle_mqtt_message('robot/status', 'DFS_EVENT:NODE_ARRIVED')
        self.assertIs(app.onboard_map.snapshot, before) # Even the event must await board map data.
        data = snapshot([node(1, 0, 0, True), node(2, 0, 1, True)], current=2, revision=2)
        data['observed_nodes'] = [{'marker_id': 9, 'x': 0, 'y': 2}]
        app.receive_onboard_map(data)
        self.assertEqual(app.dfs_current_node_text.get(), '2')
        self.assertEqual([p.get_facecolor() for p in app.ax.patches],
                         [to_rgba('#27ae60', .65), to_rgba('#27ae60', .65), to_rgba('#2980b9', .65)])
        self.assertEqual(app.sent, [])

    def test_rotated_sightings_use_received_coordinates_and_do_not_move_the_start(self):
        app = MapGUI()
        data = snapshot([node(1, 0, 0, True), node(2, 0, 1, True)], current=2)
        data['observed_nodes'] = [{'marker_id': 9, 'x': 1, 'y': 1}]
        app.receive_onboard_map(data)
        self.assertEqual(app.onboard_map.snapshot.display_bounds, (0, 1, 0, 1))
        self.assertEqual([(p.get_x()+.5, p.get_y()+.5) for p in app.ax.patches], [(0, 0), (0, 1), (1, 1)])
        self.assertEqual(app.dfs_current_node_text.get(), '2')
        self.assertEqual(app.onboard_map.snapshot.bounds, (0, 0, 0, 1))

    def test_invalid_sighting_packets_cannot_overwrite_confirmed_nodes_or_visited_state(self):
        cache = OnboardMapCache()
        self.assertTrue(cache.update(snapshot(), 10))
        previous = cache.snapshot
        for observed in (None, [{'marker_id': 7, 'x': 0, 'y': 1}],
                         [{'marker_id': 2, 'x': 0, 'y': 0}],
                         [{'marker_id': 2, 'x': 0, 'y': 1, 'visited': True}],
                         [{'marker_id': 2, 'x': 0, 'y': 32768}],
                         [{'marker_id': 2, 'x': 0, 'y': 1}, {'marker_id': 3, 'x': 0, 'y': 1}],
                         [{'marker_id': 2, 'x': 0, 'y': 1}, {'marker_id': 2, 'x': 0, 'y': 2}]):
            self.assertFalse(cache.update({**snapshot(revision=2), 'observed_nodes': observed}, 11))
            self.assertIs(cache.snapshot, previous)
        self.assertFalse(cache.update({**snapshot(revision=2), 'current_node': 2,
                                      'observed_nodes': [{'marker_id': 2, 'x': 0, 'y': 1}]}, 11))

    def test_set_start_waits_for_map_echo_and_ack_alone_cannot_change_view(self):
        app = MapGUI()
        app.receive_onboard_map(snapshot([node(9, -1, 3, True)], current=9))
        previous, draw_count = app.onboard_map.snapshot, len(app.draws)
        app.set_start_node()
        self.assertEqual(len(app.sent), 1)
        command, token, marker_id = app.sent[0].split(':')
        self.assertEqual((command, marker_id), ('SET_START', '7'))
        self.assertEqual(len(token), 32)
        self.assertIs(app.onboard_map.snapshot, previous)
        self.assertEqual(len(app.draws), draw_count)
        self.assertEqual(app.dfs_current_node_text.get(), '9')
        app.handle_mqtt_message('robot/status', 'MAP_ACK:SET_START_ACCEPTED')
        self.assertIs(app.onboard_map.snapshot, previous)
        self.assertIsNotNone(app.pending_start_request)
        app.handle_mqtt_message('robot/map', json.dumps(snapshot(revision=2, request=token)))
        self.assertEqual(app.dfs_current_node_text.get(), '7')
        self.assertEqual(app.onboard_map.snapshot.columns, 1)
        self.assertEqual(tuple(app.ax.get_xlim()), (-0.5, 0.5))
        self.assertIsNone(app.pending_start_request)
        self.assertIn('confirmed', app.start_node_status.get())

    def test_retry_reuses_request_and_rejection_keeps_existing_map(self):
        app = MapGUI()
        app.receive_onboard_map(snapshot())
        previous = app.onboard_map.snapshot
        app.set_start_node()
        app.set_start_node()
        self.assertEqual(app.sent[0], app.sent[1])
        app.handle_mqtt_message('robot/status', 'MAP_REFUSED:BUSY')
        self.assertIs(app.onboard_map.snapshot, previous)
        self.assertIsNone(app.pending_start_request)
        self.assertEqual(app.dfs_current_node_text.get(), '7')

    def test_map_grows_from_received_signed_coordinates_in_all_directions(self):
        app = MapGUI()
        nodes = [node(7, 0, 0, True)]
        for revision, new, dimensions in ((1, None, (1, 1)), (2, node(12, 1, 0), (2, 1)),
                (3, node(3, 1, 1), (2, 2)), (4, node(9, -1, 0), (3, 2)), (5, node(20, 0, -1), (3, 3))):
            if new:
                nodes.append(new)
            app.receive_onboard_map(snapshot(copy.deepcopy(nodes), revision=revision))
            self.assertEqual((app.onboard_map.snapshot.columns, app.onboard_map.snapshot.rows), dimensions)
        self.assertEqual(tuple(app.ax.get_xlim()), (-1.5, 1.5))
        self.assertEqual(tuple(app.ax.get_ylim()), (-1.5, 1.5))
        self.assertEqual(len(app.ax.patches), 5)
        self.assertEqual(app.dfs_current_node_text.get(), '7')

    def test_camera_and_legacy_grid_callbacks_cannot_mutate_map_or_current_node(self):
        app = MapGUI()
        app.receive_onboard_map(snapshot())
        previous, draw_count = app.onboard_map.snapshot, len(app.draws)
        app.update_marker_map({'markers': [{'id': 12}]})
        app.update_grid_map()
        app.handle_mqtt_message('robot/camera', json.dumps(observation()))
        app.update_current_marker()
        self.assertIs(app.onboard_map.snapshot, previous)
        self.assertEqual(len(app.draws), draw_count)
        self.assertEqual(app.dfs_current_node_text.get(), '7')
        app.receive_onboard_map(snapshot(current=None, revision=2))
        self.assertEqual(app.dfs_current_node_text.get(), '--')
        self.assertEqual(app.last_reached_marker_text.get(), '7')

    def test_64_node_layout_and_extreme_sparse_bounds_have_no_six_by_six_limit(self):
        app = MapGUI()
        nodes = [node(i + 1, i % 8, i // 8) for i in range(64)]
        app.receive_onboard_map(snapshot(nodes, current=None))
        self.assertEqual(len(app.ax.patches), 64)
        self.assertEqual((app.onboard_map.snapshot.columns, app.onboard_map.snapshot.rows), (8, 8))
        app.receive_onboard_map(snapshot([node(254, -32768, 0), node(255, 32767, 0)], current=None, revision=2))
        self.assertEqual(app.onboard_map.snapshot.columns, 65536)
        self.assertLessEqual(len(app.ax.get_xticks()), 12)

    def test_invalid_conflicting_and_old_snapshots_do_not_replace_map(self):
        cache = OnboardMapCache()
        self.assertTrue(cache.update(snapshot(), 10))
        previous = cache.snapshot
        bad = snapshot(revision=2)
        invalid = [None, {}, {**bad, 'columns': 6}, {**bad, 'current_node': 12},
                   snapshot([node(7, 0, 0, True), node(12, 0, 0)], revision=2),
                   snapshot([node(7, 0, 0, True), node(7, 1, 0)], revision=2),
                   snapshot([node(7, 0, 0, False)], revision=2)]
        for data in invalid:
            self.assertFalse(cache.update(data, 11))
            self.assertIs(cache.snapshot, previous)
        self.assertTrue(cache.update(snapshot(revision=2), 12))
        self.assertFalse(cache.update(snapshot(revision=1), 13))
        self.assertFalse(cache.update(snapshot(current=None, revision=2), 13))
        self.assertTrue(cache.update(snapshot(nodes=[], boot='0000000000000099', revision=0), 14))
        self.assertFalse(cache.update(snapshot(revision=100), 15))

    def test_retained_maps_are_ignored_and_disconnect_preserves_last_snapshot(self):
        from types import SimpleNamespace
        app = MapGUI()
        app.on_mqtt_message(None, None, SimpleNamespace(retain=True, topic='robot/map', payload=b'{}'))
        self.assertFalse(app.queued)
        app.receive_onboard_map(snapshot())
        previous = app.onboard_map.snapshot
        app.handle_mqtt_disconnect(1)
        self.assertIs(app.onboard_map.snapshot, previous)
        self.assertFalse(app.onboard_map.is_fresh(time.monotonic()))

    def test_invalid_start_and_manual_motion_do_not_send_a_start_request(self):
        app = MapGUI()
        for value in ('0', '256', '-1', 'abc'):
            app.start_node_id.set(value)
            app.set_start_node()
        self.assertEqual(app.sent, [])
        app.start_node_id.set('255')
        app.active_command = 'FORWARD'
        app.set_start_node()
        self.assertEqual(app.sent, [])
        app.active_command = None
        app.set_start_node()
        self.assertTrue(app.sent[-1].endswith(':255'))


if __name__ == '__main__':
    unittest.main()
