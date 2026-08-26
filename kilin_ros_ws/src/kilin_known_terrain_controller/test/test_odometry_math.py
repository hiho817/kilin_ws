import numpy as np

from kilin_known_terrain_controller.odometry_math import relative_planar_pose


def test_relative_pose_zeros_the_measured_origin():
    result = relative_planar_pose((4.0, -2.0, 0.7), (4.0, -2.0, 0.7))
    np.testing.assert_allclose(result, [0.0, 0.0, 0.0], atol=1e-12)


def test_relative_pose_uses_initial_vehicle_heading_as_forward():
    origin = (2.0, 3.0, np.pi / 2.0)
    result = relative_planar_pose((2.0, 4.0, np.pi / 2.0), origin)
    np.testing.assert_allclose(result, [1.0, 0.0, 0.0], atol=1e-12)


def test_relative_pose_can_be_anchored_in_analytical_map():
    origin_yaw = 0.2
    result = relative_planar_pose(
        (2.0 + np.cos(origin_yaw), 1.0 + np.sin(origin_yaw), origin_yaw),
        (2.0, 1.0, origin_yaw),
        (0.5, -0.1, np.pi / 2.0),
    )
    np.testing.assert_allclose(result, [0.5, 0.9, np.pi / 2.0], atol=1e-12)
