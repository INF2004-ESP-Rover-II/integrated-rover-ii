import copy
import unittest

from onboard_state import OnboardState, mission_command, geometry_command


def snapshot():
    return {"schema": 1, "state": "READY", "count": 1, "n": 36, "current": 7,
            "pose": {"x_mm": 0, "y_mm": 0, "valid": True},
            "markers": [{"id": 7, "x": 0, "y": 0, "visited": True, "exits": [0, 1, 2, 3]}]}


class StateTests(unittest.TestCase):
    def test_cache_and_staleness(self):
        state = OnboardState()
        self.assertTrue(state.stale(0))
        state.update(snapshot(), 10)
        self.assertFalse(state.stale(12))
        self.assertTrue(state.stale(14))

    def test_seen_is_not_visited(self):
        value = snapshot()
        value["markers"].append({"id": 1, "x": 0, "y": 1, "visited": False, "exits": [0]*4})
        state = OnboardState(); state.update(value)
        self.assertEqual(state.snapshot["count"], 1)
        self.assertEqual(state.snapshot["current"], 7)

    def test_bad_snapshots_do_not_replace_good_state(self):
        state = OnboardState(); good = snapshot(); state.update(good)
        bad_values = []
        for key, value in (("state", "DRIVE_ANYWHERE"), ("count", 36), ("current", 12)):
            bad = copy.deepcopy(good); bad[key] = value; bad_values.append(bad)
        bad = copy.deepcopy(good); bad["markers"].append(copy.deepcopy(bad["markers"][0])); bad_values.append(bad)
        bad = copy.deepcopy(good); bad["pose"]["x_mm"] = float("nan"); bad_values.append(bad)
        for bad in bad_values:
            with self.assertRaises(ValueError): state.update(bad)
            self.assertIs(state.snapshot, good)

    def test_dynamic_bounds_and_high_ids(self):
        value = snapshot()
        value.update(capacity=64, id_max=255, count=64, n=64, current=255, pitch_mm=600)
        value["markers"] = [{"id": 255-i, "x": i-32, "y": -4, "visited": True, "exits": [0]*4} for i in range(64)]
        value["bounds"] = dict(min_x=-32, max_x=31, min_y=-4, max_y=-4, rows=1, columns=64)
        state = OnboardState();state.update(value)
        self.assertEqual(len(state.snapshot["markers"]), 64)
        self.assertEqual(state.snapshot["bounds"]["columns"], 64)
        bad = copy.deepcopy(value);bad["bounds"]["columns"] = 6
        with self.assertRaises(ValueError): state.update(bad)
        bad = copy.deepcopy(value);bad["capacity"] = 63
        with self.assertRaises(ValueError): state.update(bad)
        bad = copy.deepcopy(value);bad["markers"][0]["id"] = 256
        with self.assertRaises(ValueError): state.update(bad)
        bad = copy.deepcopy(value);bad["n"] = 65
        with self.assertRaises(ValueError): state.update(bad)
        bad = snapshot();bad["markers"][0]["x"] = 64
        with self.assertRaises(ValueError): state.update(bad)
        self.assertIs(state.snapshot, value)

    def test_expanded_commands_and_advertised_limits(self):
        self.assertEqual(mission_command("start_node", marker=255), "SET_START:255")
        self.assertEqual(mission_command("start_node", marker=40), "SET_START:40")
        self.assertEqual(mission_command("start", count=64), "DFS_START:64:500")
        with self.assertRaises(ValueError): mission_command("start", count=65)
        with self.assertRaises(ValueError): mission_command("start_node", marker=0)
        with self.assertRaises(ValueError): mission_command("start", count=64, capacity=40)
        with self.assertRaises(ValueError): mission_command("start_node", marker=255, id_max=40)

    def test_measured_geometry_command(self):
        self.assertEqual(geometry_command(325,120,0),"CAMERA_GEOMETRY:325.000:120.000:0.000")
        for values in ((0,120,0),(325,120,float("nan")),(325,120,76),(True,120,0)):
            with self.assertRaises(ValueError): geometry_command(*values)

    def test_high_level_commands(self):
        self.assertEqual(mission_command("start_node", marker=7), "SET_START:7")
        self.assertEqual(mission_command("start"), "DFS_START:36:500")
        self.assertEqual(mission_command("stop"), "STOP")
        self.assertEqual(mission_command("resume"), "DFS_RESUME")
        for args in ({"marker": 256}, {"marker": True}):
            with self.assertRaises(ValueError): mission_command("start_node", **args)
        with self.assertRaises(ValueError): mission_command("start", count=0)


if __name__ == "__main__": unittest.main()
