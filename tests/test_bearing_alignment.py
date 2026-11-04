"""Exercise actual DFS steering handlers with camera bearings and no hardware."""
import unittest

import test_grid_navigation as navigation_cases


class BearingTests(unittest.TestCase):
    setUp = navigation_cases.NavigationTests.setUp
    send = navigation_cases.NavigationTests.send
    sensor = navigation_cases.NavigationTests.sensor
    camera = navigation_cases.NavigationTests.camera
    drive = navigation_cases.NavigationTests.drive
    settle = navigation_cases.NavigationTests.settle

    def step(self, bearing, seconds=.1, left=200, right=200,
             markers=((14, 600), (17, 1200)), age_ms=0):
        self.sensor(left=left, right=right, seconds=seconds)
        self.camera(markers, bearing=bearing, age_ms=age_ms)
        self.nav.tick()

    def begin(self, bearing=12):
        self.drive()
        self.step(bearing)
        self.assertEqual(self.nav.phase, 'align')
        self.assertEqual(self.commands[-1], 'STOP')

    def test_positive_bearing_calls_right_and_negative_calls_left(self):
        for bearing, expected in ((12, 'RIGHT'), (-12, 'LEFT')):
            with self.subTest(bearing=bearing):
                self.setUp()
                self.begin(bearing)
                self.step(bearing, seconds=.41)
                self.assertEqual(self.commands[-1], expected)
                self.assertFalse(any(command.startswith('ROTATE_') for command in self.commands))

    def test_trigger_and_completion_bands_are_five_degrees(self):
        self.drive()
        self.step(5)
        self.step(-5)
        self.assertEqual(self.nav.phase, 'drive')
        self.step(5.1)
        self.assertEqual(self.nav.phase, 'align')
        self.step(5.1, seconds=.41)
        self.assertEqual(self.commands[-1], 'RIGHT')
        self.step(5)
        self.assertEqual(self.commands[-1], 'STOP')

    def test_short_pulse_stops_before_another_camera_based_adjustment(self):
        self.begin()
        self.step(12, seconds=.41)
        self.assertEqual(self.commands[-1], 'RIGHT')
        self.step(8, seconds=self.nav.ALIGN_PULSE_S + .01)
        self.assertEqual(self.commands[-1], 'STOP')
        self.step(8, seconds=.2)
        self.assertEqual(self.commands[-1], 'STOP')
        self.step(8, seconds=.21)
        self.assertEqual(self.commands[-1], 'RIGHT')

    def test_three_distinct_centred_frames_then_resume_forward(self):
        self.begin()
        self.step(12, seconds=.41)
        self.step(5, seconds=.1)
        self.step(5, seconds=.41)
        for _ in range(5):
            self.nav.tick()
        self.assertEqual(self.nav.phase, 'align')  # Duplicate polling isn't confirmation.
        self.step(-5, seconds=.03)
        self.step(-4.4, seconds=.03)
        self.assertEqual(self.nav.phase, 'settle')
        self.assertIsNone(self.nav._shimmy)
        self.assertIn('centred', self.nav.status)
        self.step(-4.4, seconds=.41)
        self.assertEqual(self.nav.phase, 'drive')
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_overshoot_changes_direction_only_after_stopping_and_braking(self):
        self.begin()
        self.step(12, seconds=.41)
        self.step(-7, seconds=self.nav.ALIGN_PULSE_S + .01)
        self.assertEqual(self.commands[-1], 'STOP')
        self.step(-7, seconds=.41)
        self.assertEqual(self.commands[-2:], ['STOP', 'LEFT'])

    def test_missing_target_holds_stop_instead_of_switching_markers(self):
        self.begin()
        self.step(12, seconds=.41)
        self.step(12, markers=((17, 1200),))  # Original anchor 14 disappears.
        self.assertEqual(self.commands[-1], 'STOP')
        count = len(self.commands)
        self.step(12, seconds=.41, markers=((17, 1200),))
        self.assertEqual(len(self.commands), count)
        self.assertIn('fresh view of marker 14', self.nav.status)
        self.step(12, seconds=120, markers=())
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, 'align')
        self.assertEqual(len(self.commands), count)
        self.assertEqual(self.commands[-1], 'STOP')
        self.step(12)  # The same target returns; resume correction automatically.
        self.assertEqual(self.commands[-1], 'RIGHT')
        self.step(2)
        self.step(2, seconds=.41)
        self.step(1)
        self.step(0)
        self.assertEqual(self.nav.phase, 'settle')
        self.step(0, seconds=.41)
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_side_too_far_to_range_allows_strafe(self):
        self.begin()
        self.step(12, seconds=.41, right=0)
        self.assertEqual(self.commands[-1], 'RIGHT')

    def test_unavailable_or_close_side_reading_prevents_automatic_strafe(self):
        for distance in (65535, 100):
            with self.subTest(distance=distance):
                self.setUp()
                self.begin()
                self.step(12, seconds=.41, right=distance)
                self.assertFalse(self.nav.active)
                self.assertNotIn('RIGHT', self.commands)
                self.assertIn('Cannot centre RIGHT', self.nav.status)

    def test_side_obstacle_during_pulse_stops(self):
        self.begin()
        self.step(12, seconds=.41)
        self.step(10, seconds=.05, right=100)
        self.assertFalse(self.nav.active)
        self.assertEqual(self.commands[-1], 'STOP')

    def test_input_gap_stops_and_restarts_with_a_fresh_braking_check(self):
        self.begin()
        self.step(12, seconds=.41)
        self.sensor(seconds=1.1)
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'input_wait')
        self.assertEqual(self.commands[-1], 'STOP')
        self.sensor(seconds=.2)
        self.camera(bearing=12)
        self.nav.tick()
        self.assertEqual(self.nav.phase, 'align')
        self.assertEqual(self.commands[-1], 'STOP')
        self.step(12, seconds=.41)
        self.assertEqual(self.commands[-1], 'RIGHT')

    def test_manual_stop_cancels_alignment_and_cannot_auto_resume(self):
        self.begin()
        self.nav.stop('Emergency stop')
        count = len(self.commands)
        self.step(12)
        self.step(0)
        self.assertFalse(self.nav.active)
        self.assertIsNone(self.nav._shimmy)
        self.assertEqual(len(self.commands), count)

    def test_alignment_preserves_nodes_and_leg_without_confirming_sideways_visits(self):
        self.begin()
        cells, visited = dict(self.nav.grid.cells), set(self.nav.grid.visited)
        latest, direction = self.nav.grid.latest, self.nav.grid.direction
        self.step(12, seconds=.41)
        self.step(12, seconds=.05, markers=((14, 600), (3, 1200)))
        self.assertEqual(self.nav.grid.cells, cells)
        self.assertEqual(self.nav.grid.visited, visited)
        self.assertEqual(self.nav.grid.latest, latest)
        self.assertEqual(self.nav.grid.direction, direction)
        self.assertFalse(self.nav.grid.forward)

    def test_pre_braking_camera_frame_cannot_trigger_a_pulse(self):
        self.begin()
        self.step(12, seconds=.41, age_ms=200)
        self.assertEqual(self.commands[-1], 'STOP')
        self.step(12, seconds=.05)
        self.assertEqual(self.commands[-1], 'RIGHT')

    def test_publish_failure_clears_alignment_and_requests_stop(self):
        self.begin()
        def reject_strafe(command):
            self.commands.append(command)
            return command != 'RIGHT'
        self.nav.send_command = reject_strafe
        self.step(12, seconds=.41)
        self.assertFalse(self.nav.active)
        self.assertIsNone(self.nav._shimmy)
        self.assertEqual(self.commands[-1], 'STOP')

    def test_correction_continues_past_old_limits_and_resumes_forward(self):
        self.begin()
        forward_count = self.commands.count('FORWARD')
        for pulse in range(1, 41):
            self.step(8, seconds=.41)
            self.assertEqual(self.commands[-1], 'RIGHT')
            self.assertIn('pulse %d,' % pulse, self.nav.status)
            self.step(8, seconds=self.nav.ALIGN_PULSE_S + .01)
            self.assertEqual(self.commands[-1], 'STOP')
            self.assertTrue(self.nav.active)
            self.assertEqual(self.nav._shimmy['pulses'], pulse)
            self.assertEqual(self.commands.count('FORWARD'), forward_count)
        self.assertGreater(self.now, self.nav.explorer.segment_timeout_s)
        self.step(3, seconds=.41)
        self.step(2)
        self.step(0)
        self.assertEqual(self.nav.phase, 'settle')
        self.assertEqual(self.commands[-1], 'STOP')
        self.step(0, seconds=.41)
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, 'drive')
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_waiting_for_next_frame_does_not_erase_fresh_centred_samples(self):
        self.begin()
        self.step(2, seconds=.41)
        for count in (1, 2):
            self.assertEqual(self.nav._shimmy['centred'], count)
            self.sensor(seconds=.6)
            self.nav.tick()  # No new frame: hold STOP, don't count or reject it again.
            self.assertEqual(self.nav._shimmy['centred'], count)
            self.assertEqual(self.commands[-1], 'STOP')
            self.step(2)
        self.assertEqual(self.nav.phase, 'settle')
        self.step(0, seconds=.41)
        self.assertEqual(self.commands[-1], 'FORWARD')

    def test_fresh_frame_missing_anchor_resets_centred_confirmation(self):
        self.begin()
        self.step(2, seconds=.41)
        self.step(2)
        self.assertEqual(self.nav._shimmy['centred'], 2)
        self.step(0, markers=())
        self.assertEqual(self.nav._shimmy['centred'], 0)
        for count in (1, 2):
            self.step(2)
            self.assertEqual(self.nav._shimmy['centred'], count)
        self.step(0)
        self.assertEqual(self.nav.phase, 'settle')

    def test_refresh_keeps_strafe_command_alive_during_a_pulse(self):
        self.begin()
        self.step(12, seconds=.41)
        count = self.commands.count('RIGHT')
        self.step(10, seconds=.1)
        self.assertEqual(self.commands.count('RIGHT'), count + 1)

    def test_healthy_out_of_range_side_allows_correction(self):
        self.begin()
        self.step(12, seconds=.41, right=1023)
        self.assertEqual(self.commands[-1], 'RIGHT')


if __name__ == '__main__':
    unittest.main()
