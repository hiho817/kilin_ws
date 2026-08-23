import unittest

import numpy as np

from kilin_known_terrain_controller.ramp_model import OneSidedRamp, OneSidedRampSequence


class RampModelTests(unittest.TestCase):
    def setUp(self):
        self.ramp = OneSidedRamp(0.08, 0.75, 0.30, 0.35, 0.30, 0.25, 0.34)

    def test_profile_and_one_sided_track(self):
        x = np.array([0.70, 0.90, 1.10, 1.55, 1.80])
        expected = np.array([0.0, 0.04, 0.08, 0.04, 0.0])
        np.testing.assert_allclose(self.ramp.height(x, 0.25), expected)
        np.testing.assert_allclose(self.ramp.height(x, -0.25), 0.0)

    def test_end_position(self):
        self.assertAlmostEqual(self.ramp.end_x_m, 1.70)

    def test_two_ramps_use_opposite_wheel_tracks(self):
        second = OneSidedRamp(0.08, 2.70, 0.30, 0.35, 0.30, -0.25, 0.34)
        course = OneSidedRampSequence(self.ramp, second)
        x = np.array([1.55, 2.85, 3.10, 3.50, 3.70])
        np.testing.assert_allclose(course.height(x, 0.25), [0.04, 0.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(course.height(x, -0.25), [0.0, 0.04, 0.08, 0.04, 0.0])
        self.assertAlmostEqual(course.end_x_m, 3.65)


if __name__ == "__main__":
    unittest.main()
