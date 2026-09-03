from threading import Lock
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from kilin_msgs.msg import MotorCmdStamped
import pytest

from kilin_known_terrain_controller.controller_node import KnownTerrainController


class _Clock:
    def __init__(self, nanoseconds: int = 1_000_000_000):
        self._now = SimpleNamespace(nanoseconds=nanoseconds, to_msg=lambda: Time())

    def now(self):
        return self._now


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


def _parameter(value):
    return SimpleNamespace(value=value)


def _controller_with_fresh_feedback():
    controller = KnownTerrainController.__new__(KnownTerrainController)
    parameters = {
        "armed": True,
        "stance.target_deg": [-45.0, -45.0, 45.0, 45.0],
        "stance.hip_rate_deg_s": 15.0,
        "stance.hip_to_wheel_m": 0.260,
        "stance.wheel_radius_m": 0.0585,
        "stance.tolerance_deg": 0.5,
        "hip_kp": 350.0,
        "hip_ki": 0.0,
        "hip_kd": 5.0,
        "angle_diff_compensation.gain": 0.0,
        "angle_diff_compensation.maximum_abs_rad": 0.10,
        "position_mode": 4,
        "velocity_mode": 5,
        "rest_mode": 0,
    }
    controller.get_parameter = lambda name: _parameter(parameters[name])
    controller.get_clock = lambda: _Clock()
    controller.get_logger = lambda: _Logger()
    controller._mode = "known_ramp"
    controller._known_ramp_stance_ready = False
    controller._known_ramp_origin_hips = None
    controller._planning_period_s = 0.1
    controller._completed = False
    controller._feedback_is_fresh = lambda: True
    controller._warned_waiting_for_feedback = False
    controller._lock = Lock()
    controller._publisher = _Publisher()
    controller._measured_hips = np.array([0.10, -0.20, 0.30, -0.40])
    controller._greatest_outward_position_diff = np.zeros(4)
    controller._commanded_hips = None
    controller._latest_command = MotorCmdStamped()
    return controller


@pytest.mark.parametrize(
    "mode",
    [
        "hip_test",
        "hip_calibration",
        "wheel_calibration",
        "stance_initialization",
        "planner_posture_test",
        "known_ramp",
    ],
)
def test_fresh_feedback_does_not_publish_cached_nominal_command_before_control_cycle(mode):
    controller = _controller_with_fresh_feedback()
    controller._mode = mode
    controller._known_ramp_stance_ready = mode in ("planner_posture_test", "known_ramp")
    controller._known_ramp_origin_hips = None

    KnownTerrainController._publish_latest(controller)

    assert controller._publisher.messages == []


def test_first_stance_command_is_derived_from_measured_hips_not_nominal_pose():
    controller = _controller_with_fresh_feedback()

    KnownTerrainController._stance_initialization_cycle(controller, continue_to_planner=True)
    KnownTerrainController._publish_latest(controller)

    assert len(controller._publisher.messages) == 1
    message = controller._publisher.messages[0]
    hips = np.array(
        [
            message.module_a.hip.position,
            message.module_b.hip.position,
            message.module_c.hip.position,
            message.module_d.hip.position,
        ]
    )
    expected = np.array([0.10, -0.20, 0.30, -0.40]) + np.deg2rad(
        [-1.5, -1.5, 1.5, 1.5]
    )
    np.testing.assert_allclose(hips, expected)
    assert not np.allclose(hips, np.deg2rad([-45.0, -45.0, 45.0, 45.0]))


def test_shutdown_stays_silent_if_no_feedback_derived_command_was_initialized():
    controller = _controller_with_fresh_feedback()
    controller._commanded_hips = None
    controller._vicon_trigger_test_start_timer = None
    controller._vicon_trigger_test_stop_timer = None
    controller._set_vicon_trigger = lambda _enabled, _reason: None
    controller._release_vicon_trigger = lambda: None

    KnownTerrainController.stop(controller)

    assert controller._publisher.messages == []


def test_outward_angle_diff_compensation_uses_greatest_outward_difference_only():
    controller = _controller_with_fresh_feedback()
    parameters = {
        "angle_diff_compensation.gain": 0.4,
        "angle_diff_compensation.maximum_abs_rad": np.deg2rad(10.0),
    }
    original_get_parameter = controller.get_parameter
    controller.get_parameter = lambda name: (
        _parameter(parameters[name]) if name in parameters else original_get_parameter(name)
    )
    controller._greatest_outward_position_diff = np.deg2rad([-2.0, -3.0, 4.0, 1.0])

    command = KnownTerrainController._motor_command(
        controller,
        np.deg2rad([-45.0, -45.0, 45.0, 45.0]),
        np.zeros(4),
        hubs_at_rest=True,
    )

    hips_deg = np.rad2deg([
        command.module_a.hip.position,
        command.module_b.hip.position,
        command.module_c.hip.position,
        command.module_d.hip.position,
    ])
    # actual_angle = motor_position + position_diff, so the motor-side target
    # must move inward to counter an outward physical-angle difference.
    np.testing.assert_allclose(hips_deg, [-44.2, -43.8, 43.4, 44.6])
