"""Legacy standalone geometry helpers; production map ownership is on RoboPico."""

import ast
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dashboard"))
from laptop_navigation import LaptopNavigator
from vision_feed import GRID_SIZE, MarkerObservation, VisionFeed, grid_cell, place_markers


class MarkerGeometryTests(unittest.TestCase):
    def test_front_marker_rotates_to_each_map_direction_including_camera_offset(self):
        for heading, expected in ((0, (0, 720)), (90, (720, 0)),
                                  (180, (0, -720)), (270, (-720, 0))):
            with self.subTest(heading=heading):
                found = {}
                self.assertEqual(place_markers([MarkerObservation(1, 600, 0)], found,
                                               heading_deg=heading), [1])
                self.assertAlmostEqual(found[1][0], expected[0])
                self.assertAlmostEqual(found[1][1], expected[1])

    def test_camera_bearing_and_nonzero_start_cell_rotate_together(self):
        found = {}
        place_markers([MarkerObservation(1, 600, 30)], found, (2, 3), heading_deg=90)
        self.assertAlmostEqual(found[1][0], 1200 + 120 + 600 * math.cos(math.radians(30)))
        self.assertAlmostEqual(found[1][1], 1800 - 300)

    def test_known_marker_anchors_new_markers_after_turning_and_translation(self):
        found = {1: (1200, 1200)}
        added = place_markers([MarkerObservation(1, 480, 0), MarkerObservation(2, 600, 90)],
                              found, heading_deg=90)
        self.assertEqual(added, [2])
        self.assertEqual(found[1], (1200, 1200))
        self.assertAlmostEqual(found[2][0], 720)
        self.assertAlmostEqual(found[2][1], 600)

    def test_marker_seen_alone_after_a_turn_uses_last_anchored_rover_position(self):
        found, rover = {}, {}
        place_markers([MarkerObservation(15, 480, 0), MarkerObservation(18, 1080, 0)], found, rover=rover)
        # Drive up the row: 18 near, 10 new.
        place_markers([MarkerObservation(18, 480, 0), MarkerObservation(10, 1080, 0)], found, rover=rover)
        place_markers([MarkerObservation(10, 480, 0)], found, rover=rover)  # Stop in front of 10.
        # Turn right in place; only 16 is in view.
        self.assertEqual(place_markers([MarkerObservation(16, 480, 0)], found, heading_deg=90, rover=rover), [16])
        self.assertEqual(grid_cell(found[10]), (0, 3))
        self.assertEqual(grid_cell(found[16]), (1, 2))

    def test_wall_ahead_then_turn_puts_rover_on_last_recorded_marker(self):
        found, rover = {}, {}
        # Straight up column 0: 14, 15, 18, 10 recorded as the camera reaches ~2 cells ahead.
        place_markers([MarkerObservation(14, 480, 0), MarkerObservation(15, 1080, 0)], found, rover=rover)
        place_markers([MarkerObservation(15, 480, 0), MarkerObservation(18, 1080, 0)], found, rover=rover)
        place_markers([MarkerObservation(18, 480, 0), MarkerObservation(10, 1080, 0)], found, rover=rover,
                      front_mm=900)  # Wall ahead: no more markers in this line.
        place_markers([], found, rover=rover, front_mm=300)  # Drives up to the wall blind.
        # Turns right; 16 is the first marker seen on the new heading.
        self.assertEqual(place_markers([MarkerObservation(16, 480, 0)], found, heading_deg=90,
                                       rover=rover, front_mm=1023), [16])
        self.assertEqual([grid_cell(found[i]) for i in (14, 15, 18, 10)], [(0, 1), (0, 2), (0, 3), (0, 4)])
        self.assertEqual(grid_cell(found[16]), (1, 4))

    def test_turn_without_a_wall_keeps_last_anchored_position(self):
        found, rover = {}, {}
        place_markers([MarkerObservation(18, 480, 0), MarkerObservation(10, 1080, 0)], found, rover=rover,
                      front_mm=1023)  # 1023 = nothing within 1 m.
        place_markers([MarkerObservation(16, 480, 0)], found, heading_deg=90, rover=rover)
        self.assertEqual(grid_cell(found[16]), (1, 0))

    def test_invalid_heading_cannot_add_a_marker(self):
        for heading in (None, True, float("nan"), float("inf")):
            with self.subTest(heading=heading):
                found = {}
                with self.assertRaises(ValueError):
                    place_markers([MarkerObservation(1, 600, 0)], found, heading_deg=heading)
                self.assertEqual(found, {})


# Dashboard-side placement was retired. Send/echo rendering, signed bounds and
# camera non-mutation coverage live in test_dashboard_onboard_map.py.

if __name__ == "__main__":
    unittest.main()
