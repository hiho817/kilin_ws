from threading import Lock
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import Odometry

from kilin_known_terrain_controller.controller_node import KnownTerrainController


def parameter(value):
    return SimpleNamespace(value=value)


class Logger:
    def __init__(self):
        self.info_messages = []

    def info(self, message):
        self.info_messages.append(message)


def test_analytical_map_origin_is_captured_from_first_corrected_odometry():
    controller = KnownTerrainController.__new__(KnownTerrainController)
    values = {
        "odometry_relative_origin": True,
        "initial_x_m": 0.0,
        "initial_y_m": 0.0,
        "initial_yaw_rad": 0.0,
    }
    controller.get_parameter = lambda name: parameter(values[name])
    controller.get_logger = lambda: Logger()
    controller._lock = Lock()
    controller._odometry_origin_xy_yaw = None

    first = Odometry()
    first.header.frame_id = "map"
    first.pose.pose.position.x = 3.0
    first.pose.pose.position.y = -2.0
    first.pose.pose.orientation.z = np.sin(0.25)
    first.pose.pose.orientation.w = np.cos(0.25)
    x, y, yaw, frame = controller._planner_pose_from_odometry(first)
    np.testing.assert_allclose([x, y, yaw], [0.0, 0.0, 0.0], atol=1e-12)
    assert frame == "known_map"

    second = Odometry()
    second.header.frame_id = "map"
    second.pose.pose.position.x = 3.0 + np.cos(0.5)
    second.pose.pose.position.y = -2.0 + np.sin(0.5)
    second.pose.pose.orientation.z = np.sin(0.25)
    second.pose.pose.orientation.w = np.cos(0.25)
    x, y, yaw, frame = controller._planner_pose_from_odometry(second)
    np.testing.assert_allclose([x, y, yaw], [1.0, 0.0, 0.0], atol=1e-12)
    assert frame == "known_map"


def test_required_odometry_never_falls_back_to_time_integration():
    controller = KnownTerrainController.__new__(KnownTerrainController)
    controller.get_parameter = lambda name: parameter(True if name == "use_odometry" else None)
    controller._feedback_is_fresh = lambda: True
    controller._odometry_is_fresh = lambda: False
    calls = []
    controller._set_vicon_trigger = lambda enabled, reason: calls.append(
        ("trigger", enabled, reason)
    )
    controller._stop_for_unavailable_odometry = lambda: calls.append(("stop",))

    controller._start_plan_cycle()

    assert calls == [
        ("stop",),
        ("trigger", False, "required planner input unavailable"),
    ]


def test_required_planner_inputs_block_start_until_odometry_and_terrain_are_fresh():
    controller = KnownTerrainController.__new__(KnownTerrainController)
    values = {"use_odometry": True, "use_terrain_window": True}
    controller.get_parameter = lambda name: parameter(values[name])
    calls = []
    controller._odometry_is_fresh = lambda: False
    controller._terrain_is_fresh = lambda: True
    controller._stop_for_unavailable_odometry = lambda: calls.append("odometry")
    controller._stop_for_unavailable_terrain = lambda: calls.append("terrain")

    assert not controller._required_planner_inputs_are_fresh()
    assert calls == ["odometry"]

    controller._odometry_is_fresh = lambda: True
    controller._terrain_is_fresh = lambda: False
    controller._warned_odometry_unavailable = True

    assert not controller._required_planner_inputs_are_fresh()
    assert calls == ["odometry", "terrain"]
