import math
import unittest

from kilin_known_terrain_controller.command_mapping import radians_per_second_to_rpm10


class CommandMappingTests(unittest.TestCase):
    def test_one_revolution_per_second(self):
        self.assertAlmostEqual(radians_per_second_to_rpm10(2.0 * math.pi), 600.0)


if __name__ == "__main__":
    unittest.main()
