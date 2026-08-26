import importlib.util
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "hip_center_odometry_adapter.py"
)
SPEC = importlib.util.spec_from_file_location("hip_center_odometry_adapter", MODULE_PATH)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


def matrix(translation, rpy_deg):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    result[:3, 3] = translation
    return result


def test_pose_chain_applies_parent_leveling_and_body_to_hip_offset():
    target_source = matrix([1.0, 2.0, 3.0], [0.0, 45.0, 0.0])
    source_body = matrix([0.4, -0.2, 0.1], [2.0, -4.0, 6.0])
    body_hip = matrix([-0.26, 0.001, -0.36], [0.0, -45.0, 0.0])
    actual = ADAPTER.compose_target_pose(target_source, source_body, body_hip)
    np.testing.assert_allclose(actual, target_source @ source_body @ body_hip)


def test_twist_moves_velocity_to_hip_origin_and_rotates_axes():
    body_hip = matrix([1.0, 0.0, 0.0], [0.0, 0.0, 90.0])
    linear, angular = ADAPTER.transformed_twist(
        np.zeros(3), np.asarray([0.0, 0.0, 2.0]), body_hip
    )
    np.testing.assert_allclose(linear, [2.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(angular, [0.0, 0.0, 2.0], atol=1e-12)
