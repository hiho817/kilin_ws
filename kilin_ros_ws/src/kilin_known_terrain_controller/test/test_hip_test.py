import numpy as np
import pytest

from kilin_known_terrain_controller.hip_test import (
    bounded_position_step,
    named_hip_positions,
    smoothstep_position,
    stance_wheel_speed_rad_s,
)


def test_named_hip_positions_uses_names_not_message_order():
    names = ["RR_hip", "FL_wheel", "FL_hip", "RL_hip", "FR_hip"]
    positions = [4.0, 99.0, 1.0, 3.0, 2.0]
    np.testing.assert_allclose(named_hip_positions(names, positions), [1, 2, 3, 4])


def test_named_hip_positions_rejects_missing_joint():
    with pytest.raises(ValueError, match="RR_hip"):
        named_hip_positions(["FL_hip", "FR_hip", "RL_hip"], [1, 2, 3])


def test_bounded_step_limits_each_joint_and_does_not_overshoot():
    result = bounded_position_step([0, 0, 0, 0], [-1, -0.1, 0.1, 1], 0.2)
    np.testing.assert_allclose(result, [-0.2, -0.1, 0.1, 0.2])


def test_smoothstep_position_is_continuous_and_reaches_target():
    start = np.zeros(4)
    target = np.ones(4)
    np.testing.assert_allclose(smoothstep_position(start, target, 0.0, 0.2), start)
    np.testing.assert_allclose(smoothstep_position(start, target, 0.1, 0.2), 0.5)
    np.testing.assert_allclose(smoothstep_position(start, target, 0.2, 0.2), target)


def test_stance_wheel_ik_rolls_front_forward_and_rear_backward():
    rates = np.deg2rad([-5, -5, 5, 5])
    result = stance_wheel_speed_rad_s(np.zeros(4), rates, 0.26, 0.0585)
    assert np.all(result[:2] > 0.0)
    assert np.all(result[2:] < 0.0)
    np.testing.assert_allclose(np.abs(result), 0.26 * np.deg2rad(5) / 0.0585)
