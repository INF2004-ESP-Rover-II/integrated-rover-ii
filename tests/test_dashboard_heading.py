"""Rotation commands must not overwrite the measured IMU heading."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

source = Path(__file__).resolve().parents[1] / "dashboard" / "robot_dashboard.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
gui = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RobotGUI")
method = next(node for node in gui.body if isinstance(node, ast.FunctionDef) and node.name == "update_odometry")
namespace = {"math": math}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)


class HeadingTests(unittest.TestCase):
    def test_commands_preserve_measured_heading(self):
        for heading in (0.0, 50.0, 180.0, 359.2):
            for command in ("ROTATE_LEFT", "ROTATE_RIGHT"):
                with self.subTest(heading=heading, command=command):
                    app = SimpleNamespace(odometry_lock=threading.Lock(), robot_heading=heading,
                                          robot_x=0, robot_y=0,
                                          current_speed=SimpleNamespace(get=lambda: 50))
                    namespace["update_odometry"](app, command, 2.0)
                    self.assertEqual(app.robot_heading, heading)

    def test_strafes_move_to_the_correct_side(self):
        # Heading 0 faces +y; clockwise heading puts the rover's right on +x.
        for command, expected in (("RIGHT", (100, 0)), ("LEFT", (-100, 0)), ("FORWARD", (0, 100)),
                                  ("FRONT_RIGHT", (70.7, 70.7)), ("BACK_LEFT", (-70.7, -70.7))):
            with self.subTest(command=command):
                app = SimpleNamespace(odometry_lock=threading.Lock(), robot_heading=0.0,
                                      robot_x=0, robot_y=0, current_speed=SimpleNamespace(get=lambda: 50))
                namespace["update_odometry"](app, command, 1.0)  # 50% of 200 mm/s for 1 s
                self.assertAlmostEqual(app.robot_x, expected[0], places=1)
                self.assertAlmostEqual(app.robot_y, expected[1], places=1)


if __name__ == "__main__":
    unittest.main()
