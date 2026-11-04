"""Grid localization, wall/junction events and real DFS handoffs without hardware."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'dashboard'))
sys.path.insert(0, str(ROOT / 'rover_mapping'))
from dfs_explorer import Direction, State
from grid_localization import GridLocalizer
from laptop_navigation import LaptopNavigator, SensorSnapshot
from vision_feed import MarkerObservation
from mapping.bridge import MappingBridge
from mapping.dfs_adapter import DfsAdapter


def observation(marker, distance, bearing=0):
    return MarkerObservation(marker, distance, bearing)


class GridTests(unittest.TestCase):
    def setUp(self):
        self.grid = GridLocalizer(8, 0)
        self.now = 0
        self.heading = 0

    def pair(self, near=14, far=17, heading=0, frames=5):
        for _ in range(frames):
            self.now += .05
            self.grid.observe([observation(near, 600), observation(far, 1200)], self.now, heading)

    def sensor(self, front=1023, left=200, right=200, fault=0, age=0, heading=0):
        self.now += .05
        sensor = SensorSnapshot(heading, dict(front=front, left=left, right=right),
                                fault, 0 < front < 150, self.now, age)
        self.grid.sensors(sensor, self.now, 500)

    def test_arbitrary_ids_are_spatially_ordered_and_only_forward_updates(self):
        self.pair()
        self.assertEqual(self.grid.cells, {8: (0, 0), 14: (0, 1), 17: (0, 2)})
        self.assertEqual(self.grid.visited, {8})
        self.grid.motion('FORWARD', 0)
        self.pair(frames=4)
        self.assertIsNone(self.grid.estimate)
        self.pair(frames=1)
        self.assertEqual(self.grid.cells, {8: (0, 0), 14: (0, 1), 17: (0, 2)})
        self.assertEqual(self.grid.candidate(self.now)[0], 8)
        self.pair(17, 3)
        self.assertEqual(self.grid.candidate(self.now)[0], 14)
        self.assertEqual(self.grid.cells[3], (0, 3))

    def test_turning_strafing_backward_and_heading_drift_do_not_localize(self):
        self.grid.motion('FORWARD', 0)
        self.pair(heading=40)
        self.assertIsNone(self.grid.estimate)
        for command in ('TURN_TO:90', 'LEFT', 'BACKWARD', 'STOP'):
            self.grid.motion(command, 0)
            self.pair()
            self.assertIsNone(self.grid.estimate)

    def test_new_east_leg_rotates_grid_not_marker_ids(self):
        self.grid.motion('FORWARD', 90)
        self.pair(31, 2, heading=90)
        self.assertEqual(self.grid.cells[31], (1, 0))
        self.assertEqual(self.grid.cells[2], (2, 0))
        self.pair(2, 20, heading=90)
        self.assertEqual(self.grid.estimate, 31)

    def test_wall_approach_is_not_arrival_and_terminal_is_latched(self):
        self.grid.motion('FORWARD', 0)
        self.pair()
        self.sensor(front=600)
        self.assertFalse(self.grid.approaching_wall)
        self.sensor(front=599)
        self.assertEqual(self.grid.terminal, 17)
        self.assertEqual(self.grid.candidate(self.now)[0], 8)
        self.pair(17, 3)
        self.assertEqual(self.grid.terminal, 17)
        self.sensor(front=150)
        self.assertNotEqual(self.grid.candidate(self.now)[1], 'terminal wall stop')
        self.sensor(front=149)
        self.assertEqual(self.grid.candidate(self.now), (17, 'terminal wall stop'))
        self.assertEqual(self.grid.path_to(17), [14, 17])

    def test_invalid_faulted_stale_and_out_of_range_are_not_walls(self):
        self.grid.motion('FORWARD', 0)
        self.pair()
        for front, fault, age in ((0, 0, 0), (65535, 0, 0), (1023, 0, 0),
                                  (100, 2, 0), (100, 0, 1)):
            self.sensor(front=front, fault=fault, age=age)
            self.assertFalse(self.grid.approaching_wall)

    def test_cross_junction_requires_corridor_then_three_valid_open_samples(self):
        self.grid.motion('FORWARD', 0)
        self.pair()
        for _ in range(4):
            self.sensor(left=1023, right=1023)
        self.assertFalse(self.grid.junction)  # Starting in an open room isn't a crossing.
        self.sensor()
        self.sensor(left=1023, right=1023)
        self.sensor(left=1023, right=0)
        self.sensor(left=1023, right=1023)
        self.sensor(left=1023, right=1023)
        self.assertFalse(self.grid.junction)
        self.sensor(left=1023, right=1023)
        self.assertTrue(self.grid.junction)
        self.pair(17, 3)
        self.assertEqual(self.grid.candidate(self.now), (14, 'cross junction'))

    def test_stale_pairs_missing_overlap_and_conflicting_layout_are_not_guessed(self):
        self.grid.motion('FORWARD', 0)
        self.pair()
        self.now += 1
        self.assertIsNone(self.grid.candidate(self.now)[0])
        self.pair(30, 31)
        self.assertIsNotNone(self.grid.fault)
        self.grid.motion('TURN_TO:90', 0)
        self.grid.motion('FORWARD', 90)
        self.pair(14, 17, heading=90)
        self.assertIsNotNone(self.grid.fault)
        self.assertIsNone(self.grid.estimate)

    def test_side_markers_and_nonconsecutive_ranges_do_not_shift_the_node(self):
        self.grid.motion('FORWARD', 0)
        for pair in ([observation(14, 600, -15), observation(17, 1200, 15)],
                     [observation(14, 600), observation(17, 650)],
                     [observation(14, 600), observation(17, 1800)]):
            for _ in range(5):
                self.now += .05
                self.grid.observe(pair, self.now, 0)
            self.assertIsNone(self.grid.estimate)
            self.assertEqual(self.grid.current, 8)
            self.assertEqual(self.grid.visited, {8})

    def test_cleared_obstacle_does_not_leave_a_stale_terminal_latch(self):
        self.grid.motion('FORWARD', 0)
        self.pair()
        self.sensor(front=550)
        self.assertEqual(self.grid.terminal, 17)
        for _ in range(3):
            self.sensor(front=1023)
        self.assertFalse(self.grid.approaching_wall)
        self.assertIsNone(self.grid.terminal)


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.seq = self.stamp = 0
        self.commands, self.logs = [], []
        self.nav = LaptopNavigator(self.send, self.logs.append,
            clock=lambda: self.now, wall_clock=lambda: 1800000000 + self.now)
        self.nav.grid_required = True
        self.sensor()
        self.nav.configure_grid_start(8)

    def send(self, command):
        self.commands.append(command)
        return True

    def sensor(self, front=1023, left=200, right=200, seconds=.05):
        self.now += seconds
        self.stamp += 1
        self.nav.sensor_received({'timestamp': self.stamp, 'imu': {'heading': getattr(self, 'heading', 0), 'status': 0},
            'obstacle': {'age_ms': 0, 'distance_mm': dict(left=left, front=front, right=right),
                         'fault_mask': 0, 'stop_required': 0 < front < 150}})

    def camera(self, markers=((14, 600), (17, 1200)), bearing=0, age_ms=0):
        self.seq += 1
        self.nav.vision_received({'schema': 1, 'source_session_id': 'grid-test',
            'frame_seq': self.seq, 'capture_time_ms': self.nav.wall_clock() * 1000 - age_ms,
            'capture_time_source': 'laptop_receive', 'last_frame_age_ms': age_ms,
            'status': 'ok' if markers else 'no_marker',
            'markers': [{'marker_id': marker, 'valid': True, 'distance_ref': 'camera_lens',
                         'distance_mm': distance, 'bearing_deg': bearing} for marker, distance in markers]})

    def frames(self, markers=((14, 600), (17, 1200)), front=1023):
        for _ in range(5):
            self.sensor(front=front)
            self.camera(markers)
            self.nav.tick()

    def drive(self, target=36, markers=((14, 600), (17, 1200))):
        self.camera(markers)
        self.nav.start(target, 500)
        self.sensor(seconds=.41)
        self.camera(markers)
        self.nav.tick()  # Scan chooses north.
        self.nav.tick()  # Already facing north: no rotation command is needed.
        self.assertEqual(self.nav.phase, 'settle')
        self.assertNotIn('TURN_TO:0', self.commands)
        self.sensor(seconds=.41)
        self.camera(markers)
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'drive')
        self.assertEqual(self.commands[-1], 'FORWARD')

    def settle(self, front=1023):
        self.sensor(front=front, seconds=.41)
        self.camera(())
        self.nav.tick()

    def test_heading_retry_and_input_recovery_freeze_travel_budget_once(self):
        self.drive()
        self.heading = 12
        self.sensor()
        self.camera()
        self.nav.phase = 'settle'
        self.nav.ready_at = self.now
        self.nav.tick()
        self.nav.tick()
        self.assertEqual(self.commands[-1], 'TURN_TO:0')
        self.nav.robot_status_received('AUTO_MOVE: TURN_TO:0')
        self.nav.robot_status_received('AUTO_RESULT: HEADING_REACHED')
        self.nav.tick()
        self.sensor(seconds=1.1)  # Camera data expires during post-turn settling.
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'input_wait')
        budget_used = self.now - self.nav.explorer.issued_at
        self.sensor(seconds=2)
        self.camera()
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'settle')
        self.assertAlmostEqual(self.now - self.nav.explorer.issued_at, budget_used)
        self.sensor(seconds=.41)
        self.camera()
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, 'turn')
        self.assertAlmostEqual(self.now - self.nav.explorer.issued_at, budget_used)

    def test_manual_tracks_without_500mm_arrival_or_mutating_dfs_graph(self):
        self.nav.motion_received('FORWARD')
        self.frames()
        self.frames(((17, 600), (3, 1200)))
        self.assertEqual(self.nav.last_reached_marker, 14)
        self.assertEqual(self.nav.grid.current, 14)
        self.assertFalse(self.nav.explorer.nodes)

    def test_stationary_camera_sightings_remain_available_without_confirming_nodes(self):
        self.camera()
        self.assertEqual([m.marker_id for m in self.nav.camera_markers], [14, 17])
        self.assertEqual(self.nav.camera_heading, 0)
        self.assertEqual(self.nav.camera_grid_cell, (0, 0))
        self.assertEqual(self.nav.grid.cells, {8: (0, 0), 14: (0, 1), 17: (0, 2)})
        self.assertEqual(self.nav.grid.visited, {8})
        self.assertEqual(self.nav.last_reached_marker, 8)
        self.assertFalse(self.nav.explorer.nodes)
        self.assertFalse(self.nav.grid.forward)
        self.assertIsNone(self.nav.grid.estimate)

    def test_stale_camera_frames_do_not_supply_new_display_sightings(self):
        self.camera()
        self.seq += 1
        self.nav.vision_received({'schema': 1, 'source_session_id': 'grid-test',
            'frame_seq': self.seq, 'capture_time_ms': self.nav.wall_clock() * 1000 - 2000,
            'capture_time_source': 'laptop_receive', 'last_frame_age_ms': 2000,
            'status': 'stalled', 'markers': []})
        self.assertEqual(self.nav.camera_markers, ())
        self.assertEqual(self.nav.grid.visited, {8})

    def test_full_dfs_start_straight_arrival_and_continuation(self):
        self.drive()
        self.frames()
        self.assertEqual(self.nav.explorer.current, 8)
        self.frames(((17, 600), (3, 1200)))
        self.assertEqual(self.nav.phase, 'arrival_settle')
        self.assertEqual(self.nav.explorer.current, 8)
        self.settle()
        self.assertEqual(self.nav.explorer.current, 14)
        self.assertEqual(self.nav.last_reached_marker, 14)
        self.assertEqual(self.nav.phase, 'scan')
        self.assertEqual(self.nav.explorer.nodes[8].neighbors[0], 14)

    def test_wall_confirms_known_intermediate_nodes_and_mapping_receives_each_hop(self):
        self.drive()
        bridge = MappingBridge(notify=lambda message: None)
        adapter = DfsAdapter(bridge)
        adapter.sync(self.nav.explorer)
        self.frames()
        self.sensor(front=599)
        self.camera(())
        self.nav.tick()
        self.assertEqual(self.nav.explorer.current, 8)
        self.assertEqual(self.nav.grid.terminal, 17)
        self.sensor(front=149)
        self.camera(())
        self.assertTrue(self.nav.handle_front_obstacle())
        self.assertEqual(self.commands[-1], 'STOP')
        self.settle(front=149)
        self.assertEqual(self.nav.grid.current, 17)
        self.assertEqual(self.nav.explorer.current, 17)
        self.assertEqual(set(self.nav.explorer.nodes), {8, 14, 17})
        adapter.sync(self.nav.explorer)
        self.assertEqual(bridge.ctrl.graph.current, 17)
        self.assertEqual(bridge.ctrl.graph.markers[17].pos, (0, 2))
        self.assertEqual(bridge.ctrl.graph.neighbours[8][0], 14)

    def test_manual_wall_only_confirms_after_stop_and_settle(self):
        self.nav.motion_received('FORWARD')
        self.frames()
        self.sensor(front=550)
        self.camera(())
        self.nav.tick()
        self.assertEqual(self.nav.last_reached_marker, 8)
        self.sensor(front=140)
        self.nav.motion_received('STOP')
        self.nav.tick()
        self.assertEqual(self.nav.last_reached_marker, 8)
        self.settle(front=140)
        self.assertEqual(self.nav.last_reached_marker, 17)

    def test_unknown_terminal_stops_dfs_without_inventing_a_node(self):
        self.drive(markers=())
        self.sensor(front=100)
        self.camera(())
        self.nav.tick()
        self.assertFalse(self.nav.active)
        self.assertEqual(self.nav.explorer.current, 8)
        self.assertEqual(self.commands[-1], 'STOP')

    def test_visible_near_marker_does_not_trigger_old_500mm_stop(self):
        self.drive()
        self.frames(((14, 490),))
        self.assertEqual(self.nav.phase, 'drive')
        self.assertEqual(self.nav.explorer.current, 8)

    def test_grid_start_requires_explicit_valid_anchor_and_fresh_inputs(self):
        self.nav.grid = None
        self.camera()
        with self.assertRaisesRegex(ValueError, 'Start node ID'):
            self.nav.start(36, 500)
        with self.assertRaises(ValueError):
            self.nav.configure_grid_start(0)
        self.now += 2
        with self.assertRaisesRegex(ValueError, 'Fresh'):
            self.nav.configure_grid_start(8)

    def test_east_branch_uses_new_leg_and_continues_dfs(self):
        self.drive()
        self.frames()
        self.frames(((17, 600), (3, 1200)))
        self.settle()
        self.sensor(front=149, right=1023)
        self.camera(())
        self.nav.tick()
        self.nav.tick()
        self.assertEqual(self.commands[-1], 'TURN_TO:90')
        self.nav.robot_status_received('AUTO_MOVE: TURN_TO:90')
        self.heading = 90
        self.sensor()
        self.camera(())
        self.nav.robot_status_received('AUTO_RESULT: HEADING_REACHED')
        self.nav.tick()
        self.settle()
        self.assertNotEqual(self.commands[-1], 'FORWARD')  # Holds until the new leg is visible.
        self.sensor(seconds=.41)
        self.camera(((5, 600), (31, 1200)))
        self.nav.tick()
        self.assertEqual(self.commands[-1], 'FORWARD')
        self.frames(((5, 600), (31, 1200)))
        self.frames(((31, 600), (2, 1200)))
        self.settle()
        self.assertEqual(self.nav.explorer.current, 5)
        self.assertEqual(self.nav.grid.cells[5], (1, 1))
        self.assertEqual(self.nav.explorer.nodes[14].neighbors[1], 5)

    def test_cross_junction_event_confirms_two_grids_back(self):
        self.drive()
        self.frames()
        for _ in range(5):
            self.sensor(left=1023, right=1023)
            self.camera(((17, 600), (3, 1200)))
            self.nav.tick()
        self.assertEqual(self.nav._grid_reason, 'cross junction')
        self.settle()
        self.assertEqual(self.nav.explorer.current, 14)

    def test_new_motion_cancels_manual_wall_confirmation(self):
        self.nav.motion_received('FORWARD')
        self.frames()
        self.sensor(front=140)
        self.nav.motion_received('STOP')
        self.nav.tick()
        self.assertIsNotNone(self.nav._grid_pending)
        self.nav.motion_received('ROTATE_LEFT')
        self.settle(front=140)
        self.assertEqual(self.nav.last_reached_marker, 8)
        self.assertIsNone(self.nav._grid_pending)

    def test_terminal_batch_keeps_actual_final_node_when_count_reached_midway(self):
        self.drive(target=2)
        bridge = MappingBridge(notify=lambda message: None)
        adapter = DfsAdapter(bridge)
        adapter.sync(self.nav.explorer)
        self.frames()
        self.sensor(front=140)
        self.camera(())
        self.nav.tick()
        self.settle(front=140)
        self.assertEqual(self.nav.explorer.state, State.COMPLETE)
        self.assertEqual(self.nav.grid.current, 17)
        self.assertEqual(self.nav.explorer.current, 17)
        adapter.sync(self.nav.explorer)
        self.assertEqual(bridge.ctrl.graph.current, 17)
        self.assertEqual(set(bridge.ctrl.graph.markers), {8, 14, 17})

    def test_startup_front_150_to_499_remains_open_and_does_not_inspect_south(self):
        for front in (150, 200, 400, 499):
            with self.subTest(front=front):
                self.setUp()
                self.camera()
                self.nav.start(36, 500)
                self.sensor(front=front, seconds=.41)
                self.camera()
                self.nav.tick()
                self.assertEqual(self.nav.phase, 'turn')
                self.assertEqual(int(self.nav.explorer.pending.direction), 0)
                self.nav.tick()
                self.assertEqual(self.nav.phase, 'settle')
                self.assertNotIn('TURN_TO:0', self.commands)
                self.assertNotIn('TURN_TO:180', self.commands)

    def scan(self, samples=3, **readings):
        self.camera()
        self.nav.start(36, 500)
        for _ in range(samples):
            self.sensor(seconds=.41, **readings)
            self.camera(())
            self.nav.tick()

    def test_invalid_front_reading_does_not_trigger_blind_rear_scan(self):
        self.camera()
        self.nav.start(36, 500)
        self.sensor(front=65535, seconds=.41)
        self.camera()
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'scan')
        self.assertNotIn('TURN_TO:180', self.commands)
        self.assertIn('waiting for a valid front ToF', self.nav.status)

    def test_front_too_far_goes_straight_even_with_open_side(self):
        self.scan(samples=2, front=0, left=1023, right=300)
        self.assertEqual(self.nav.phase, 'scan')  # A short dropout must not choose the side.
        self.assertIsNone(self.nav.explorer.pending)
        self.sensor(front=0, left=1023, right=300, seconds=.41)
        self.nav.tick()
        self.assertEqual(self.nav.explorer.pending.direction, Direction.NORTH)

    def test_open_side_too_far_is_ignored_until_an_obstacle_blocks_ahead(self):
        self.scan(front=1023, left=0, right=300)
        self.assertEqual(self.nav.explorer.pending.direction, Direction.NORTH)

    def test_obstacle_turns_to_side_too_far_then_waits_for_camera(self):
        self.scan(samples=2, front=120, left=0, right=300)
        self.assertIsNone(self.nav.explorer.pending)
        self.sensor(front=120, left=0, right=300, seconds=.41)
        self.nav.tick()
        self.assertEqual(self.nav.explorer.pending.direction, Direction.WEST)
        self.nav.tick()
        self.assertEqual(self.commands[-1], 'TURN_TO:270')
        self.nav.robot_status_received('AUTO_MOVE: TURN_TO:270')
        self.nav.robot_status_received('AUTO_RESULT: HEADING_REACHED')
        self.nav.tick()
        self.heading = 270
        for _ in range(3):
            self.sensor(seconds=.41)
            self.camera(())
            self.nav.tick()
            self.assertEqual(self.nav.phase, 'settle')
        self.assertNotIn('FORWARD', self.commands)
        self.assertIn('until the camera sees a marker', self.nav.status)
        self.sensor(seconds=.41)
        self.camera(((6, 600),))
        self.nav.tick()
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_marker_seen_mid_turn_is_placed_from_the_settled_heading(self):
        self.camera(())
        self.nav.start(36, 500)
        self.sensor(front=120, left=200, right=1023, seconds=.41)
        self.camera(())
        self.nav.tick()
        self.nav.tick()
        self.assertEqual(self.commands[-1], 'TURN_TO:90')
        self.sensor(left=200, right=1023)  # Telemetry still says 0° while the rover is rotating.
        self.camera(((20, 620),), bearing=30)
        self.assertNotIn(20, self.nav.grid.cells)
        self.nav.robot_status_received('AUTO_MOVE: TURN_TO:90')
        self.nav.robot_status_received('AUTO_RESULT: HEADING_REACHED')
        self.nav.tick()
        self.camera(((20, 620),), bearing=30)  # Braking, before a fresh heading arrives.
        self.assertNotIn(20, self.nav.grid.cells)
        self.heading = 90
        self.sensor(seconds=.41)
        self.camera(((20, 600),))
        self.assertEqual(self.nav.grid.cells[20], (1, 0))
        self.nav.tick()
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_obstacle_prefers_left_and_uses_right_when_left_is_a_wall(self):
        self.scan(samples=1, front=120, left=200, right=1023)
        self.assertEqual(self.nav.explorer.pending.direction, Direction.EAST)

    def test_side_dropout_beside_a_wall_is_not_an_exit(self):
        self.scan(samples=1, front=120, left=0, right=300)
        for left in (200, 0, 0, 0):
            self.sensor(front=120, left=left, right=300, seconds=.41)
            self.camera(())
            self.nav.tick()
        self.assertFalse(self.nav.explorer.nodes[8].open_exits & (1 << Direction.WEST))

    def test_faulted_side_is_never_treated_as_too_far(self):
        sensor = SensorSnapshot(0, dict(left=0, front=120, right=0), 0b001, True, 0, 0)
        self.assertFalse(sensor.too_far('left'))
        self.assertTrue(sensor.too_far('right'))

    def test_real_front_stop_still_allows_rear_inspection_and_logs_readings(self):
        self.camera()
        self.nav.start(36, 500)
        self.sensor(front=149, seconds=.41)
        self.camera()
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'scan_turn')
        self.assertIn('front=149', self.nav.status)
        self.nav.tick()
        self.assertEqual(self.commands[-1], 'TURN_TO:180')

    def test_sensor_age_pauses_dfs_only_past_one_second(self):
        self.drive()
        self.now += .95
        self.camera()
        self.nav.tick()
        self.assertIsNone(self.nav._input_hold)
        self.now += .1
        self.camera()
        self.nav.tick()
        self.assertIsNotNone(self.nav._input_hold)
        self.assertIn('sensor telemetry age 1.05s > 1.00s', self.nav.status)

    def test_camera_gap_stops_then_resumes_same_dfs_without_reactivation(self):
        self.drive()
        segment = self.nav.explorer.pending
        forward_count = self.commands.count('FORWARD')
        self.sensor(seconds=1.02)
        self.nav.tick()
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, 'input_wait')
        self.assertEqual(self.commands[-1], 'STOP')
        self.assertIs(self.nav.explorer.pending, segment)
        self.assertFalse(self.nav.grid.forward)
        self.sensor(seconds=.2)
        self.camera()
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'settle')
        self.assertEqual(self.commands.count('FORWARD'), forward_count)
        self.settle()
        self.assertEqual(self.nav.phase, 'drive')
        self.assertIs(self.nav.explorer.pending, segment)
        self.assertEqual(self.commands.count('FORWARD'), forward_count + 1)

    def test_long_camera_loss_expires_recovery_without_moving_blind(self):
        self.drive()
        self.sensor(seconds=1.02)
        self.nav.tick()
        self.sensor(seconds=self.nav.INPUT_RECOVERY_WAIT_S + .1)
        self.nav.tick()
        self.assertFalse(self.nav.active)
        self.assertIn('automatic recovery expired', self.nav.status)
        self.assertEqual(self.commands[-1], 'STOP')
        count = self.commands.count('FORWARD')
        self.sensor()
        self.camera()
        self.nav.tick()
        self.assertEqual(self.commands.count('FORWARD'), count)

    def test_operator_stop_cancels_automatic_recovery(self):
        self.drive()
        self.sensor(seconds=1.02)
        self.nav.tick()
        self.nav.stop('Emergency stop')
        count = self.commands.count('FORWARD')
        self.sensor()
        self.camera()
        self.nav.tick()
        self.assertFalse(self.nav.active)
        self.assertIsNone(self.nav._input_hold)
        self.assertEqual(self.commands.count('FORWARD'), count)

    def test_stalled_health_packet_holds_instead_of_ending_grid_mission(self):
        self.drive()
        self.seq += 1
        self.nav.vision_received({'schema': 1, 'source_session_id': 'grid-test', 'frame_seq': self.seq,
            'capture_time_ms': self.nav.wall_clock() * 1000 - 2000,
            'capture_time_source': 'laptop_receive', 'last_frame_age_ms': 2000,
            'status': 'stalled', 'markers': []})
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, 'input_wait')
        self.assertFalse(self.nav.grid.forward)
        self.assertEqual(self.commands[-1], 'STOP')

    def test_fixed_marker_positions_do_not_slide_with_camera_range(self):
        self.camera()
        pinned = dict(self.nav.grid.cells)
        for distance in (1000, 800, 650, 600):
            self.sensor()
            self.camera(((17, distance),))
        self.assertEqual(self.nav.grid.cells, pinned)
        self.assertEqual(len(set(pinned.values())), len(pinned))
        self.assertEqual(self.nav.grid.visited, {8})

    def test_14_to_17_is_confirmed_from_remaining_fixed_marker_8(self):
        self.nav.configure_grid_start(14)
        self.camera(((17, 600), (8, 1200)))
        self.drive()
        original = dict(self.nav.grid.cells)
        self.frames(((8, 600),))
        self.assertEqual(self.nav.phase, 'arrival_settle')
        self.settle()
        self.assertEqual(self.nav.explorer.current, 17)
        self.assertEqual(self.nav.last_reached_marker, 17)
        self.assertIn(17, self.nav.grid.visited)
        self.assertEqual(self.nav.grid.cells, original)
        self.assertEqual(self.nav.grid.cells[8], (0, 2))

    def test_wall_remembers_stationary_sightings_without_five_moving_pair_frames(self):
        self.nav.configure_grid_start(14)
        self.drive(markers=((17, 600), (8, 1200)))
        self.assertEqual(self.nav.grid.latest, 8)
        self.assertEqual(self.nav.grid.count, 0)
        self.sensor(front=550)
        self.camera(((8, 600),))
        self.nav.tick()
        self.assertEqual(self.nav.grid.terminal, 8)
        self.sensor(front=140)
        self.camera(())
        self.assertTrue(self.nav.handle_front_obstacle())
        self.settle(front=140)
        self.assertIn(17, self.nav.grid.visited)
        self.assertIn(8, self.nav.grid.visited)
        self.assertEqual(self.nav.explorer.current, 8)

    def test_position_confirmation_explains_counter_and_range_rejection(self):
        self.nav.configure_grid_start(14)
        self.drive(markers=((17, 600), (8, 1200)))
        for _ in range(3):
            self.sensor()
            self.camera(((8, 600),))
        self.assertIn('Node 17: 3/5 position frames', self.nav.grid.confirmation_summary(self.now))
        self.sensor()
        self.camera(((8, 800),))
        self.assertIn('position error', self.nav.grid.confirmation_summary(self.now))

    def test_manual_confirmation_can_finish_after_releasing_forward(self):
        self.nav.configure_grid_start(14)
        self.camera(((17, 600), (8, 1200)))
        self.nav.motion_received('FORWARD')
        self.nav.motion_received('STOP')
        self.frames(((8, 600),))
        self.assertFalse(self.nav.grid.forward)
        self.assertEqual(self.nav.last_reached_marker, 17)
        self.assertIn(17, self.nav.grid.visited)
        self.assertFalse(self.nav.explorer.nodes)
        self.assertNotIn('FORWARD', self.commands)

    def test_rotation_cancels_stationary_confirmation_from_old_straight_leg(self):
        self.nav.configure_grid_start(14)
        self.camera(((17, 600), (8, 1200)))
        self.nav.motion_received('FORWARD')
        self.nav.motion_received('STOP')
        self.nav.motion_received('ROTATE_LEFT')
        self.frames(((8, 600),))
        self.assertEqual(self.nav.last_reached_marker, 14)
        self.assertNotIn(17, self.nav.grid.visited)

    def test_recovery_confirms_arrived_node_while_stopped_before_resuming_forward(self):
        self.nav.configure_grid_start(14)
        self.drive(markers=((17, 600), (8, 1200)))
        self.sensor(seconds=1.02)
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'input_wait')
        forward_count = self.commands.count('FORWARD')
        for _ in range(5):
            self.sensor()
            self.camera(((8, 600),))
            self.nav.tick()
        # Give the recovery brake wait a fresh sensor and camera sample.
        self.sensor(seconds=.41)
        self.camera(((8, 600),))
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'arrival_settle')
        self.assertEqual(self.commands.count('FORWARD'), forward_count)
        self.settle()
        self.assertEqual(self.nav.explorer.current, 17)
        self.assertIn(17, self.nav.grid.visited)


if __name__ == '__main__':
    unittest.main()
