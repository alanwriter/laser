"""Offline geometry and safety checks for the Leader-only sine experiment."""

import math
import unittest
from dataclasses import replace

from leader_formation import RoverStatus
from leader_sine_calibration import (LENGTH_MM, SCALE_DEFAULT, control, guard,
                                     reference)


ORIGIN = RoverStatus("idle", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                     0, 1, 1, 1, 0, 0)


class SineExperimentTests(unittest.TestCase):
    def test_right_left_right_and_smooth_endpoints(self):
        self.assertAlmostEqual(reference(LENGTH_MM / 6)[0], -70)
        self.assertAlmostEqual(reference(LENGTH_MM / 2)[0], 70)
        self.assertAlmostEqual(reference(5 * LENGTH_MM / 6)[0], -70)
        for x in (0, LENGTH_MM):
            y, slope, curvature = reference(x)
            self.assertAlmostEqual(y, 0, places=8)
            self.assertAlmostEqual(slope, 0, places=8)
            self.assertAlmostEqual(curvature, 0, places=8)

    def test_scale_and_lateral_guard(self):
        status = replace(ORIGIN, x_mm=100 / SCALE_DEFAULT)
        command = control(status, ORIGIN, SCALE_DEFAULT, 25)
        self.assertAlmostEqual(command.x_mm, 100)
        self.assertLess(command.target_y_mm, 0)
        guard(status, ORIGIN, command, 5)
        bad = replace(status, y_mm=250 / SCALE_DEFAULT)
        with self.assertRaisesRegex(RuntimeError, "Lateral"):
            guard(bad, ORIGIN, control(bad, ORIGIN, SCALE_DEFAULT, 25), 5)

    def test_ideal_differential_drive_reaches_endpoint(self):
        x = y = heading = 0.0
        for step in range(1250):
            status = replace(ORIGIN, x_mm=x / SCALE_DEFAULT,
                             y_mm=y / SCALE_DEFAULT,
                             heading_deg=math.degrees(heading))
            command = control(status, ORIGIN, SCALE_DEFAULT, 25)
            guard(status, ORIGIN, command, step * 0.1)
            if command.x_mm >= LENGTH_MM:
                self.assertLess(abs(y), 10)
                self.assertLess(abs(math.degrees(heading)), 5)
                return
            speed = (command.left_mm_s + command.right_mm_s) / 2
            yaw = (command.right_mm_s - command.left_mm_s) / 130
            heading += yaw * 0.1
            x += speed * math.cos(heading) * 0.1
            y += speed * math.sin(heading) * 0.1
        self.fail("Ideal model did not reach the endpoint")


if __name__ == "__main__":
    unittest.main()
