from __future__ import annotations

import sys
from threading import Lock, Thread

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from kilin_msgs.msg import MotorCmdStamped, MotorStateStamped
from kilin_msgs.msg import TerrainWindow
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from .command_mapping import radians_per_second_to_rpm10, validate_four
from .hip_test import (
    bounded_position_step,
    named_hip_positions,
    smoothstep_position,
    stance_wheel_speed_rad_s,
)
from .odometry_math import relative_planar_pose
from .ramp_model import OneSidedRamp, OneSidedRampSequence
from .terrain_window_policy import (
    fill_isolated_unknown_nodes,
    infer_flat_height_ahead,
    seed_initial_flat_support,
)


class KnownTerrainController(Node):
    def __init__(self) -> None:
        super().__init__("kilin_known_terrain_controller")
        self._declare_parameters()
        self._mode = str(self.get_parameter("mode").value)
        if self._mode not in (
            "disabled",
            "hip_test",
            "hip_calibration",
            "wheel_calibration",
            "stance_initialization",
            "planner_posture_test",
            "known_ramp",
        ):
            raise ValueError(
                "mode must be disabled, hip_test, hip_calibration, "
                "wheel_calibration, stance_initialization, planner_posture_test, or known_ramp"
            )
        if self._mode in ("planner_posture_test", "known_ramp"):
            self._load_planner()

        self._command_topic = str(self.get_parameter("command_topic").value)
        self._publisher = self.create_publisher(MotorCmdStamped, self._command_topic, 10)
        self._feedback_source = str(self.get_parameter("feedback_source").value)
        if self._feedback_source == "motor_state":
            self._feedback_subscription = self.create_subscription(
                MotorStateStamped,
                str(self.get_parameter("motor_state_topic").value),
                self._motor_state_callback,
                10,
            )
        elif self._feedback_source == "joint_state":
            self._feedback_subscription = self.create_subscription(
                JointState,
                str(self.get_parameter("joint_state_topic").value),
                self._joint_state_callback,
                10,
            )
        else:
            raise ValueError("feedback_source must be motor_state or joint_state")
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._odometry_callback,
            10,
        )
        self._terrain_subscription = self.create_subscription(
            TerrainWindow, str(self.get_parameter("terrain_window_topic").value),
            self._terrain_callback, 1)
        self._speed_subscription = self.create_subscription(
            Float32, str(self.get_parameter("speed_command_topic").value),
            self._speed_callback, 10)
        self._debug_path_publisher = self.create_publisher(
            Path, "/kilin/planner/debug/horizon", 1
        )
        self._debug_footprint_publisher = self.create_publisher(
            MarkerArray, "/kilin/planner/debug/footprints", 1
        )
        self._debug_footprint_marker_count = 0
        self._lock = Lock()
        # Do not cache a nominal pose here.  The publish timer runs faster than
        # the control timer, so a cached 45-degree stance could otherwise be
        # sent for one or more cycles immediately after the first feedback.
        self._latest_command = None
        self._planning = False
        self._clock_start_ns = None
        self._completed = False
        self._last_joint_state_time = None
        self._measured_hips = None
        self._commanded_hips = None
        self._hip_target = None
        self._hip_target_reached_ns = None
        self._hip_start = None
        self._calibration_joint = 0
        self._calibration_returning = False
        self._calibration_hold_start_ns = None
        self._wheel_calibration_start_ns = None
        self._known_ramp_origin_hips = None
        self._planner_target_hips = None
        self._planner_wheel_rates = np.zeros(4)
        self._planner_hubs_at_rest = True
        self._last_interpolation_ns = None
        self._planner_transition_start_hips = None
        self._planner_transition_start_ns = None
        self._stance_hold_start_ns = None
        self._known_ramp_stance_ready = not (
            self._mode == "known_ramp"
            and bool(self.get_parameter("known_ramp.auto_initialize_stance").value)
        )
        self._warned_waiting_for_feedback = False
        self._latest_odom = None
        self._latest_terrain = None
        # Fixed map-frame patch inferred once from the verified initial flat
        # stance.  It is not translated with the robot or used as a general
        # replacement for unobserved terrain.
        self._initial_flat_support = None
        self._target_speed_m_s = None
        self._active_speed_m_s = 0.0
        self._last_speed_command_time = None
        self._warned_speed_timeout = False
        self._open_loop_x_m = None
        self._open_loop_y_m = None
        self._open_loop_yaw_rad = None
        self._last_open_loop_update_ns = None
        self._last_odom_time = None
        self._last_terrain_time = None
        self._odometry_origin_xy_yaw = None
        self._warned_odometry_unavailable = False
        self._warned_odometry_frame = False
        self._warned_terrain_unavailable = False
        self._trigger_chip = None
        self._trigger_request = None
        self._trigger_gpiod = None
        self._trigger_on = False
        self._vicon_trigger_test_start_timer = None
        self._vicon_trigger_test_stop_timer = None

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        planning_rate = float(self.get_parameter("planning_rate_hz").value)
        self._publish_timer = self.create_timer(1.0 / publish_rate, self._publish_latest)
        self._planning_period_s = 1.0 / planning_rate
        self._planning_timer = self.create_timer(self._planning_period_s, self._control_cycle)
        if (
            bool(self.get_parameter("vicon_trigger.enabled").value)
            and bool(self.get_parameter("vicon_trigger.test_mode").value)
        ):
            # Delay very slightly so normal node initialization and logging
            # complete before a safe, unarmed physical LED self-test begins.
            self._vicon_trigger_test_start_timer = self.create_timer(
                0.1, self._start_vicon_trigger_test
            )
        self.get_logger().info(
            f"Kilin controller ready; mode={self._mode}, command_topic={self._command_topic}, "
            f"armed={self.get_parameter('armed').value}. "
            "Motion modes remain silent until fresh hip feedback has been captured "
            "by a control cycle."
        )
        self.get_logger().info(
            "Active hip PID: "
            f"kp={self.get_parameter('hip_kp').value}, "
            f"ki={self.get_parameter('hip_ki').value}, "
            f"kd={self.get_parameter('hip_kd').value}"
        )
        if self._mode in ("planner_posture_test", "known_ramp"):
            hard_limit = float(self.get_parameter("hard_motion_limit_s").value)
            speed_topic_mode = (
                self._mode == "known_ramp"
                and bool(self.get_parameter("use_speed_command").value)
            )
            duration_detail = (
                f"speed_topic={self.get_parameter('speed_command_topic').value}, "
                f"speed_timeout={self.get_parameter('speed_command_timeout_s').value} s, "
                "duration=message-controlled, "
                if speed_topic_mode
                else f"requested_duration={self.get_parameter('run_duration_s').value} s, "
                f"hard_limit={hard_limit} s, "
                f"effective_duration={min(float(self.get_parameter('run_duration_s').value), hard_limit)} s, "
            )
            self.get_logger().info(
                "Effective planner parameters: "
                f"speed={self.get_parameter('speed_m_s').value} m/s, "
                f"startup_delay={self.get_parameter('startup_delay_s').value} s, "
                + duration_detail
                +
                f"horizon_steps={self.get_parameter('horizon_steps').value}, "
                f"horizon_knot_spacing={self.get_parameter('horizon_knot_spacing_m').value} m, "
                f"planner_dt={self.get_parameter('planner_dt_s').value} s, "
                f"planner_deadline={self.get_parameter('planner_deadline_s').value} s, "
                f"wheel_radius={self._session.planner.geometry.wheel_radius_m} m, "
                "position_source=odometry_or_integrated_applied_speed"
            )

    def _declare_parameters(self) -> None:
        defaults = {
            "mode": "disabled",
            "command_topic": "/kilin/motor_cmd_raw",
            "feedback_source": "motor_state",
            "motor_state_topic": "/motor/state",
            "joint_state_topic": "/kilin_joint_states",
            "use_odometry": False,
            "odometry_topic": "/Odometry",
            "odometry_timeout_s": 0.5,
            "odometry_required_frame": "",
            "odometry_relative_origin": False,
            "use_terrain_window": False,
            "terrain_window_topic": "/kilin/terrain/local_window",
            "terrain_window_timeout_s": 1.0,
            "live_terrain.initial_flat_support.enabled": True,
            "live_terrain.initial_flat_support.rear_m": 0.65,
            # Covers the initial five-knot preview plus the approximately
            # 0.6 m near-field LiDAR blind strip seen on the real MID360s.
            "live_terrain.initial_flat_support.forward_m": 0.85,
            "live_terrain.initial_flat_support.half_width_m": 0.50,
            "live_terrain.initial_flat_support.measurement_min_forward_m": 0.20,
            "live_terrain.initial_flat_support.measurement_max_forward_m": 0.80,
            "live_terrain.initial_flat_support.maximum_height_span_m": 0.05,
            "live_terrain.initial_flat_support.minimum_inlier_fraction": 0.80,
            "live_terrain.isolated_hole_fill.enabled": True,
            "live_terrain.isolated_hole_fill.maximum_height_span_m": 0.08,
            "use_speed_command": False,
            "speed_command_topic": "/kilin/control/target_speed_m_s",
            "speed_command_max_m_s": 0.30,
            "speed_command_accel_limit_m_s2": 0.15,
            "speed_command_timeout_s": 0.5,
            "armed": False,
            "initial_x_m": 0.0,
            "initial_y_m": 0.0,
            "initial_yaw_rad": 0.0,
            "speed_m_s": 0.18,
            "startup_delay_s": 1.0,
            "run_duration_s": 22.0,
            "hard_motion_limit_s": 22.0,
            "planning_rate_hz": 10.0,
            "publish_rate_hz": 50.0,
            "planner_dt_s": 0.1,
            "horizon_steps": 5,
            "horizon_knot_spacing_m": 0.05,
            "planner_deadline_s": 0.6,
            "solver_max_iterations": 400,
            "map_resolution_m": 0.025,
            "map_margin_x_m": 0.8,
            "map_half_width_m": 0.7,
            "ramp.height_m": 0.08,
            "ramp.start_x_m": 0.75,
            "ramp.up_ramp_length_m": 0.30,
            "ramp.deck_length_m": 0.35,
            "ramp.down_ramp_length_m": 0.30,
            "ramp.track_center_y_m": 0.25,
            "ramp.track_width_m": 0.34,
            "ramp.second.enabled": True,
            "ramp.second.height_m": 0.08,
            "ramp.second.start_x_m": 2.70,
            "ramp.second.up_ramp_length_m": 0.30,
            "ramp.second.deck_length_m": 0.35,
            "ramp.second.down_ramp_length_m": 0.30,
            "ramp.second.track_center_y_m": -0.25,
            "hip_kp": 350.0,
            "hip_ki": 0.0,
            "hip_kd": 5.0,
            "position_mode": 4,
            "velocity_mode": 5,
            "rest_mode": 0,
            "feedback_timeout_s": 0.5,
            "hip_test.delta_deg": [5.0, 5.0, -5.0, -5.0],
            "hip_test.rate_deg_s": 8.0,
            "hip_test.tolerance_deg": 0.5,
            "hip_test.hold_s": 2.0,
            "hip_calibration.delta_deg": [-3.0, -3.0, 3.0, 3.0],
            "hip_calibration.rate_deg_s": 6.0,
            "hip_calibration.hold_s": 1.0,
            "wheel_calibration.speed_rad_s": 0.5,
            "wheel_calibration.pre_drive_hold_s": 1.0,
            "wheel_calibration.drive_s": 1.0,
            "known_ramp.max_initial_hip_error_deg": 5.0,
            "known_ramp.auto_initialize_stance": True,
            "known_ramp.hip_rate_limit_deg_s": 144.0,
            "known_ramp.command_smoothing_s": 0.20,
            "planner_posture_test.duration_s": 4.0,
            "stance.target_deg": [-45.0, -45.0, 45.0, 45.0],
            "stance.hip_rate_deg_s": 15.0,
            "stance.hip_to_wheel_m": 0.260,
            "stance.wheel_radius_m": 0.0585,
            "stance.tolerance_deg": 0.5,
            "stance.hold_s": 1.0,
            # Emits only a small Path and horizon footprint markers. It does
            # not enable RViz or publish terrain/FAST-LIO point clouds.
            "debug_publish_enabled": False,
            # Optional physical Vicon LED trigger.  It is deliberately off by
            # default, and is intended only for the real robot.
            "vicon_trigger.enabled": False,
            "vicon_trigger.chip": "/dev/gpiochip0",
            "vicon_trigger.line": 112,
            # ROS runs with PYTHONNOUSERSITE=1 on this Orin.  The GPIO v2
            # binding is intentionally added only when the trigger is used,
            # so it cannot affect ros2cli's package discovery.
            "vicon_trigger.gpiod_site_packages": (
                "/home/biorola/.local/lib/python3.10/site-packages"
            ),
            # Safe GPIO-only self-test.  It neither arms nor commands motors.
            "vicon_trigger.test_mode": False,
            "vicon_trigger.test_duration_s": 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_planner(self) -> None:
        from kilin_motion_planner.config import PlannerConfig
        from kilin_motion_planner.live_terrain import ElevationWindow, regular_grid_coordinates
        from kilin_motion_planner.online import LivePlannerConfig, RecedingHorizonSession
        from kilin_motion_planner.path import make_straight_horizon
        from kilin_motion_planner.planner import RecedingHorizonPlanner

        config = PlannerConfig(
            horizon_steps=int(self.get_parameter("horizon_steps").value),
            dt_s=float(self.get_parameter("planner_dt_s").value),
            horizon_knot_spacing_m=float(
                self.get_parameter("horizon_knot_spacing_m").value
            ),
            solver_max_iterations=int(self.get_parameter("solver_max_iterations").value),
            use_analytic_derivatives=True,
        )
        planner = RecedingHorizonPlanner(config=config)
        planner.prepare()
        self._session = RecedingHorizonSession(
            planner=planner,
            config=LivePlannerConfig(
                planner_deadline_s=float(self.get_parameter("planner_deadline_s").value)
            ),
            initial_hip_rad=config.nominal_hip_rad,
        )
        self._planner_config = config
        self._ElevationWindow = ElevationWindow
        self._regular_grid_coordinates = regular_grid_coordinates
        self._make_straight_horizon = make_straight_horizon
        first_ramp = OneSidedRamp(
            height_m=float(self.get_parameter("ramp.height_m").value),
            start_x_m=float(self.get_parameter("ramp.start_x_m").value),
            up_ramp_length_m=float(self.get_parameter("ramp.up_ramp_length_m").value),
            deck_length_m=float(self.get_parameter("ramp.deck_length_m").value),
            down_ramp_length_m=float(self.get_parameter("ramp.down_ramp_length_m").value),
            track_center_y_m=float(self.get_parameter("ramp.track_center_y_m").value),
            track_width_m=float(self.get_parameter("ramp.track_width_m").value),
        )
        second_ramp = None
        if bool(self.get_parameter("ramp.second.enabled").value):
            second_ramp = OneSidedRamp(
                height_m=float(self.get_parameter("ramp.second.height_m").value),
                start_x_m=float(self.get_parameter("ramp.second.start_x_m").value),
                up_ramp_length_m=float(self.get_parameter("ramp.second.up_ramp_length_m").value),
                deck_length_m=float(self.get_parameter("ramp.second.deck_length_m").value),
                down_ramp_length_m=float(self.get_parameter("ramp.second.down_ramp_length_m").value),
                track_center_y_m=float(
                    self.get_parameter("ramp.second.track_center_y_m").value
                ),
                track_width_m=float(self.get_parameter("ramp.track_width_m").value),
            )
        self._ramp = OneSidedRampSequence(first=first_ramp, second=second_ramp)

    def _elapsed_motion_s(self) -> float:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            return 0.0
        if self._clock_start_ns is None:
            self._clock_start_ns = now_ns
        elapsed = (now_ns - self._clock_start_ns) * 1.0e-9
        return max(0.0, elapsed - float(self.get_parameter("startup_delay_s").value))

    def _control_cycle(self) -> None:
        if self._completed:
            self._set_vicon_trigger(False, "controller completed")
            return
        if self._mode == "known_ramp" and not self._feedback_is_fresh():
            self._set_vicon_trigger(False, "motor feedback stale")
            return
        if self._mode in ("known_ramp", "planner_posture_test"):
            if not self._required_planner_inputs_are_fresh():
                self._set_vicon_trigger(False, "required planner input unavailable")
                return
        if self._mode == "known_ramp" and not self._known_ramp_stance_ready:
            self._stance_initialization_cycle(continue_to_planner=True)
        elif self._mode in ("planner_posture_test", "known_ramp"):
            self._start_plan_cycle()
        elif self._mode == "hip_test":
            self._hip_test_cycle()
        elif self._mode == "hip_calibration":
            self._hip_calibration_cycle()
        elif self._mode == "wheel_calibration":
            self._wheel_calibration_cycle()
        elif self._mode == "stance_initialization":
            self._stance_initialization_cycle()

    def _set_vicon_trigger(self, enabled: bool, reason: str) -> None:
        """Drive the optional active-low Vicon LED trigger safely.

        GPIO is opened only when explicitly enabled for a real-robot run.  A
        failure to access it never changes motor-control behavior.
        """
        if not bool(self.get_parameter("vicon_trigger.enabled").value):
            return
        if enabled == self._trigger_on:
            return
        try:
            if self._trigger_request is None:
                gpiod_site_packages = str(
                    self.get_parameter("vicon_trigger.gpiod_site_packages").value
                )
                if gpiod_site_packages and gpiod_site_packages not in sys.path:
                    sys.path.append(gpiod_site_packages)
                import gpiod
                from gpiod.line import Direction, Value

                chip = gpiod.Chip(str(self.get_parameter("vicon_trigger.chip").value))
                line = int(self.get_parameter("vicon_trigger.line").value)
                settings = gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    active_low=True,
                    output_value=Value.INACTIVE,
                )
                self._trigger_request = chip.request_lines(
                    consumer="kilin_known_terrain_vicon_trigger",
                    config={line: settings},
                )
                self._trigger_chip = chip
                self._trigger_gpiod = (line, Value)
                self.get_logger().info(
                    f"Vicon trigger ready: {self.get_parameter('vicon_trigger.chip').value} "
                    f"line {line}, active-low"
                )
            line, value = self._trigger_gpiod
            self._trigger_request.set_value(
                line, value.ACTIVE if enabled else value.INACTIVE
            )
            self._trigger_on = enabled
            self.get_logger().info(
                f"Vicon trigger {'ON' if enabled else 'OFF'} ({reason})"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Unable to set Vicon trigger ({reason}): {exc}; continuing without it"
            )
            self._release_vicon_trigger()

    def _start_vicon_trigger_test(self) -> None:
        """Run one GPIO-only Vicon LED pulse without arming the robot."""
        if self._vicon_trigger_test_start_timer is not None:
            self._vicon_trigger_test_start_timer.cancel()
            self._vicon_trigger_test_start_timer = None
        duration_s = max(
            0.1, float(self.get_parameter("vicon_trigger.test_duration_s").value)
        )
        self.get_logger().info(
            f"Starting safe Vicon trigger self-test for {duration_s:.1f} s; no motor command is sent"
        )
        self._set_vicon_trigger(True, "safe self-test started")
        self._vicon_trigger_test_stop_timer = self.create_timer(
            duration_s, self._stop_vicon_trigger_test
        )

    def _stop_vicon_trigger_test(self) -> None:
        """End the one-shot GPIO-only Vicon LED pulse."""
        if self._vicon_trigger_test_stop_timer is not None:
            self._vicon_trigger_test_stop_timer.cancel()
            self._vicon_trigger_test_stop_timer = None
        self._set_vicon_trigger(False, "safe self-test completed")

    def _release_vicon_trigger(self) -> None:
        """Return the trigger to its inactive state and relinquish the GPIO."""
        if self._trigger_request is not None:
            try:
                line, value = self._trigger_gpiod
                self._trigger_request.set_value(line, value.INACTIVE)
                self._trigger_request.release()
            except Exception:
                pass
        if self._trigger_chip is not None:
            try:
                self._trigger_chip.close()
            except Exception:
                pass
        self._trigger_chip = None
        self._trigger_request = None
        self._trigger_gpiod = None
        self._trigger_on = False

    def _publish_debug_plan(self, path, frame_id: str) -> None:
        """Publish lightweight planner diagnostics when explicitly requested."""
        if not bool(self.get_parameter("debug_publish_enabled").value):
            return
        timestamp = self.get_clock().now().to_msg()
        path_message = Path()
        path_message.header.stamp = timestamp
        path_message.header.frame_id = frame_id
        for x_m, y_m, yaw_rad in zip(path.x_m, path.y_m, path.yaw_rad):
            pose = PoseStamped()
            pose.header = path_message.header
            pose.pose.position.x = float(x_m)
            pose.pose.position.y = float(y_m)
            pose.pose.orientation.z = float(np.sin(0.5 * yaw_rad))
            pose.pose.orientation.w = float(np.cos(0.5 * yaw_rad))
            path_message.poses.append(pose)
        self._debug_path_publisher.publish(path_message)

        geometry = self._session.planner.geometry
        local_corners = np.array([
            [geometry.body_x_max_m, geometry.body_half_width_m],
            [geometry.body_x_max_m, -geometry.body_half_width_m],
            [geometry.body_x_min_m, -geometry.body_half_width_m],
            [geometry.body_x_min_m, geometry.body_half_width_m],
            [geometry.body_x_max_m, geometry.body_half_width_m],
        ])
        footprints = MarkerArray()
        for marker_id, (x_m, y_m, yaw_rad) in enumerate(
            zip(path.x_m, path.y_m, path.yaw_rad)
        ):
            rotation = np.array([
                [np.cos(yaw_rad), -np.sin(yaw_rad)],
                [np.sin(yaw_rad), np.cos(yaw_rad)],
            ])
            corners = local_corners @ rotation.T + np.array([x_m, y_m])
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = frame_id
            marker.ns = "planned_body_footprints"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.015
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.color.a = 0.9
            marker.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in corners]
            footprints.markers.append(marker)
        for marker_id in range(path.steps, self._debug_footprint_marker_count):
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = frame_id
            marker.ns = "planned_body_footprints"
            marker.id = marker_id
            marker.action = Marker.DELETE
            footprints.markers.append(marker)
        self._debug_footprint_marker_count = path.steps
        self._debug_footprint_publisher.publish(footprints)

    def _feedback_is_fresh(self) -> bool:
        if self._last_joint_state_time is None or self._measured_hips is None:
            return False
        age_s = (self.get_clock().now() - self._last_joint_state_time).nanoseconds * 1.0e-9
        return 0.0 <= age_s <= float(self.get_parameter("feedback_timeout_s").value)

    def _odometry_is_fresh(self) -> bool:
        if self._last_odom_time is None or self._latest_odom is None:
            return False
        age_s = (self.get_clock().now() - self._last_odom_time).nanoseconds * 1.0e-9
        return 0.0 <= age_s <= float(self.get_parameter("odometry_timeout_s").value)

    def _terrain_is_fresh(self) -> bool:
        if self._last_terrain_time is None or self._latest_terrain is None:
            return False
        age_s = (self.get_clock().now() - self._last_terrain_time).nanoseconds * 1.0e-9
        return 0.0 <= age_s <= float(self.get_parameter("terrain_window_timeout_s").value)

    def _required_planner_inputs_are_fresh(self) -> bool:
        """Block planner motion until every enabled external input is fresh."""
        if bool(self.get_parameter("use_odometry").value) and not self._odometry_is_fresh():
            self._stop_for_unavailable_odometry()
            return False
        self._warned_odometry_unavailable = False
        if bool(self.get_parameter("use_terrain_window").value) and not self._terrain_is_fresh():
            self._stop_for_unavailable_terrain()
            return False
        self._warned_terrain_unavailable = False
        return True

    def _odometry_callback(self, message: Odometry) -> None:
        required_frame = str(self.get_parameter("odometry_required_frame").value)
        if required_frame and message.header.frame_id != required_frame:
            if not self._warned_odometry_frame:
                self.get_logger().error(
                    "ODOMETRY REJECTED: topic="
                    f"{self.get_parameter('odometry_topic').value}, received parent "
                    f"frame='{message.header.frame_id or '<empty>'}', expected "
                    f"frame='{required_frame}'. The controller will hold the hips and "
                    "keep hubs in REST until corrected odometry is published in the "
                    "required frame. Check the FAST-LIO hip-center adapter."
                )
                self._warned_odometry_frame = True
            return
        with self._lock:
            self._latest_odom = message
            self._last_odom_time = self.get_clock().now()
            self._warned_odometry_frame = False

    @staticmethod
    def _yaw_from_odometry(message: Odometry) -> float:
        orientation = message.pose.pose.orientation
        return float(
            np.arctan2(
                2.0
                * (
                    orientation.w * orientation.z
                    + orientation.x * orientation.y
                ),
                1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
            )
        )

    def _planner_pose_from_odometry(
        self, message: Odometry
    ) -> tuple[float, float, float, str]:
        pose = message.pose.pose
        measured = (
            float(pose.position.x),
            float(pose.position.y),
            self._yaw_from_odometry(message),
        )
        frame_id = message.header.frame_id or "known_map"
        if not bool(self.get_parameter("odometry_relative_origin").value):
            return (*measured, frame_id)
        with self._lock:
            if self._odometry_origin_xy_yaw is None:
                self._odometry_origin_xy_yaw = measured
                self.get_logger().info(
                    "Captured corrected odometry origin for analytical map: "
                    f"x={measured[0]:.4f} m, y={measured[1]:.4f} m, "
                    f"yaw={np.rad2deg(measured[2]):.3f} deg"
                )
            origin = self._odometry_origin_xy_yaw
        anchor = (
            float(self.get_parameter("initial_x_m").value),
            float(self.get_parameter("initial_y_m").value),
            float(self.get_parameter("initial_yaw_rad").value),
        )
        x, y, yaw = relative_planar_pose(measured, origin, anchor)
        return x, y, yaw, "known_map"

    def _stop_for_unavailable_odometry(self) -> None:
        with self._lock:
            if self._commanded_hips is not None:
                self._planner_target_hips = self._commanded_hips.copy()
                self._planner_transition_start_hips = self._commanded_hips.copy()
                self._planner_transition_start_ns = self.get_clock().now().nanoseconds
                self._planner_wheel_rates = np.zeros(4)
                self._planner_hubs_at_rest = True
                self._latest_command = self._motor_command(
                    self._commanded_hips, np.zeros(4), hubs_at_rest=True
                )
        if not self._warned_odometry_unavailable:
            topic = str(self.get_parameter("odometry_topic").value)
            timeout_s = float(self.get_parameter("odometry_timeout_s").value)
            with self._lock:
                last_odom_time = self._last_odom_time
            if last_odom_time is None:
                diagnostic = (
                    f"no accepted message has arrived on {topic}. Start FAST-LIO and its "
                    "hip-center odometry adapter, then verify the topic with "
                    f"'ros2 topic echo {topic} --once'."
                )
            else:
                age_s = (self.get_clock().now() - last_odom_time).nanoseconds * 1.0e-9
                diagnostic = (
                    f"the last accepted message on {topic} is {age_s:.2f} s old "
                    f"(timeout {timeout_s:.2f} s). Check FAST-LIO/adapter health and "
                    "the topic rate."
                )
            self.get_logger().error(
                "ODOMETRY SAFETY HOLD: " + diagnostic + " Hips are held and wheel hubs are REST."
            )
            self._warned_odometry_unavailable = True

    def _terrain_callback(self, message: TerrainWindow) -> None:
        with self._lock:
            self._latest_terrain = message
            self._last_terrain_time = self.get_clock().now()

    def _stop_for_unavailable_terrain(self) -> None:
        with self._lock:
            if self._commanded_hips is not None:
                self._planner_target_hips = self._commanded_hips.copy()
                self._planner_transition_start_hips = self._commanded_hips.copy()
                self._planner_transition_start_ns = self.get_clock().now().nanoseconds
                self._planner_wheel_rates = np.zeros(4)
                self._planner_hubs_at_rest = True
                self._latest_command = self._motor_command(
                    self._commanded_hips, np.zeros(4), hubs_at_rest=True
                )
        if not self._warned_terrain_unavailable:
            topic = str(self.get_parameter("terrain_window_topic").value)
            timeout_s = float(self.get_parameter("terrain_window_timeout_s").value)
            with self._lock:
                last_terrain_time = self._last_terrain_time
            if last_terrain_time is None:
                diagnostic = (
                    f"no terrain window has arrived on {topic}. Start the local terrain mapper "
                    f"and verify it with 'ros2 topic echo {topic} --once'."
                )
            else:
                age_s = (self.get_clock().now() - last_terrain_time).nanoseconds * 1.0e-9
                diagnostic = (
                    f"the last terrain window on {topic} is {age_s:.2f} s old "
                    f"(timeout {timeout_s:.2f} s). Check mapper and FAST-LIO health."
                )
            self.get_logger().error(
                "TERRAIN SAFETY HOLD: " + diagnostic + " Hips are held and wheel hubs are REST."
            )
            self._warned_terrain_unavailable = True

    def _condition_live_terrain(self, terrain, x_m: float, y_m: float, yaw_rad: float):
        """Apply bounded initial-support seeding and isolated-node repair."""

        heights = terrain.heights_m
        valid = terrain.valid_mask
        if bool(self.get_parameter("live_terrain.initial_flat_support.enabled").value):
            if self._initial_flat_support is None:
                flat_height = infer_flat_height_ahead(
                    terrain.x_coordinates_m,
                    terrain.y_coordinates_m,
                    heights,
                    valid,
                    origin_x_m=x_m,
                    origin_y_m=y_m,
                    yaw_rad=yaw_rad,
                    minimum_forward_m=float(self.get_parameter("live_terrain.initial_flat_support.measurement_min_forward_m").value),
                    maximum_forward_m=float(self.get_parameter("live_terrain.initial_flat_support.measurement_max_forward_m").value),
                    half_width_m=float(self.get_parameter("live_terrain.initial_flat_support.half_width_m").value),
                    maximum_height_span_m=float(self.get_parameter("live_terrain.initial_flat_support.maximum_height_span_m").value),
                    minimum_inlier_fraction=float(self.get_parameter("live_terrain.initial_flat_support.minimum_inlier_fraction").value),
                )
                if flat_height is not None:
                    self._initial_flat_support = (x_m, y_m, yaw_rad, flat_height, terrain.frame_id)
                    self.get_logger().info(
                        "Live terrain: seeded initial flat support patch from observed terrain ahead "
                        f"at z={flat_height:.3f} m in frame {terrain.frame_id!r}"
                    )
                else:
                    self.get_logger().warning(
                        "Live terrain: initial support is unknown and the front reference strip is not "
                        "flat/observed; retaining the terrain safety fallback",
                        throttle_duration_sec=2.0,
                    )
            if self._initial_flat_support is not None:
                origin_x, origin_y, origin_yaw, flat_height, frame_id = self._initial_flat_support
                if frame_id == terrain.frame_id:
                    heights, valid, filled = seed_initial_flat_support(
                        terrain.x_coordinates_m,
                        terrain.y_coordinates_m,
                        heights,
                        valid,
                        origin_x_m=origin_x,
                        origin_y_m=origin_y,
                        yaw_rad=origin_yaw,
                        flat_height_m=flat_height,
                        rear_m=float(self.get_parameter("live_terrain.initial_flat_support.rear_m").value),
                        forward_m=float(self.get_parameter("live_terrain.initial_flat_support.forward_m").value),
                        half_width_m=float(self.get_parameter("live_terrain.initial_flat_support.half_width_m").value),
                    )
                    if filled:
                        self.get_logger().debug(f"Live terrain: filled {filled} initial flat-support nodes")
        if bool(self.get_parameter("live_terrain.isolated_hole_fill.enabled").value):
            heights, valid, filled = fill_isolated_unknown_nodes(
                heights,
                valid,
                maximum_height_span_m=float(self.get_parameter("live_terrain.isolated_hole_fill.maximum_height_span_m").value),
            )
            if filled:
                self.get_logger().debug(f"Live terrain: repaired {filled} isolated unknown nodes")
        return self._ElevationWindow(
            x_coordinates_m=terrain.x_coordinates_m,
            y_coordinates_m=terrain.y_coordinates_m,
            heights_m=heights,
            valid_mask=valid,
            timestamp_s=terrain.timestamp_s,
            frame_id=terrain.frame_id,
        )

    def _speed_callback(self, message: Float32) -> None:
        if np.isfinite(message.data):
            with self._lock:
                self._target_speed_m_s = float(message.data)
                self._last_speed_command_time = self.get_clock().now()
                self._warned_speed_timeout = False

    def _speed_command_is_fresh(self) -> bool:
        with self._lock:
            command_time = self._last_speed_command_time
        if command_time is None:
            return False
        age_s = (self.get_clock().now() - command_time).nanoseconds * 1.0e-9
        return 0.0 <= age_s <= float(
            self.get_parameter("speed_command_timeout_s").value
        )

    def _integrated_open_loop_pose(self, speed_m_s: float) -> tuple[float, float, float]:
        now_ns = self.get_clock().now().nanoseconds
        if self._open_loop_x_m is None:
            self._open_loop_x_m = float(self.get_parameter("initial_x_m").value)
            self._open_loop_y_m = float(self.get_parameter("initial_y_m").value)
            self._open_loop_yaw_rad = float(self.get_parameter("initial_yaw_rad").value)
            self._last_open_loop_update_ns = now_ns
        else:
            delta_s = max(0.0, (now_ns - self._last_open_loop_update_ns) * 1.0e-9)
            self._open_loop_x_m += speed_m_s * delta_s * np.cos(self._open_loop_yaw_rad)
            self._open_loop_y_m += speed_m_s * delta_s * np.sin(self._open_loop_yaw_rad)
            self._last_open_loop_update_ns = now_ns
        return self._open_loop_x_m, self._open_loop_y_m, self._open_loop_yaw_rad

    def _hip_test_cycle(self) -> None:
        if self._completed or not self._feedback_is_fresh():
            return
        with self._lock:
            if self._commanded_hips is None:
                self._commanded_hips = self._measured_hips.copy()
                delta = np.deg2rad(
                    validate_four(
                        self.get_parameter("hip_test.delta_deg").value, "hip-test delta"
                    )
                )
                self._hip_target = self._commanded_hips + delta
                self.get_logger().info(
                    "Fresh FL/FR/RL/RR hip feedback acquired; beginning bounded hip test"
                )
            target = self._hip_target
            rate = np.deg2rad(float(self.get_parameter("hip_test.rate_deg_s").value))
            self._commanded_hips = bounded_position_step(
                self._commanded_hips, target, rate * self._planning_period_s
            )
            self._latest_command = self._motor_command(
                self._commanded_hips, np.zeros(4), hubs_at_rest=True
            )

        error = np.max(np.abs(target - self._commanded_hips))
        tolerance = np.deg2rad(float(self.get_parameter("hip_test.tolerance_deg").value))
        now_ns = self.get_clock().now().nanoseconds
        if error <= tolerance:
            if self._hip_target_reached_ns is None:
                self._hip_target_reached_ns = now_ns
                self.get_logger().info("Hip-test target reached; starting finite hold")
            hold_s = (now_ns - self._hip_target_reached_ns) * 1.0e-9
            if hold_s >= float(self.get_parameter("hip_test.hold_s").value):
                self._completed = True
                self.get_logger().info("Hip test complete; holding final pose with zero wheel speed")

    def _hip_calibration_cycle(self) -> None:
        if self._completed or not self._feedback_is_fresh():
            return
        now_ns = self.get_clock().now().nanoseconds
        names = ("FL_hip", "FR_hip", "RL_hip", "RR_hip")
        with self._lock:
            if self._commanded_hips is None:
                self._hip_start = self._measured_hips.copy()
                self._commanded_hips = self._hip_start.copy()
                self.get_logger().info(
                    "Fresh hip feedback acquired; starting sequential outward calibration"
                )
            deltas = np.deg2rad(
                validate_four(
                    self.get_parameter("hip_calibration.delta_deg").value,
                    "hip-calibration delta",
                )
            )
            target = self._hip_start.copy()
            if not self._calibration_returning:
                target[self._calibration_joint] += deltas[self._calibration_joint]
            rate = np.deg2rad(
                float(self.get_parameter("hip_calibration.rate_deg_s").value)
            )
            self._commanded_hips = bounded_position_step(
                self._commanded_hips, target, rate * self._planning_period_s
            )
            self._latest_command = self._motor_command(
                self._commanded_hips, np.zeros(4), hubs_at_rest=True
            )

        tolerance = np.deg2rad(float(self.get_parameter("hip_test.tolerance_deg").value))
        if np.max(np.abs(target - self._commanded_hips)) > tolerance:
            self._calibration_hold_start_ns = None
            return
        if self._calibration_hold_start_ns is None:
            self._calibration_hold_start_ns = now_ns
            action = "returned to start" if self._calibration_returning else "outward target reached"
            self.get_logger().info(f"{names[self._calibration_joint]} {action}; holding")
            return
        hold_s = (now_ns - self._calibration_hold_start_ns) * 1.0e-9
        if hold_s < float(self.get_parameter("hip_calibration.hold_s").value):
            return
        self._calibration_hold_start_ns = None
        if not self._calibration_returning:
            self._calibration_returning = True
            return
        self._calibration_returning = False
        self._calibration_joint += 1
        if self._calibration_joint >= 4:
            self._completed = True
            self.get_logger().info(
                "Sequential hip calibration complete; holding measured starting pose"
            )
        else:
            self.get_logger().info(f"Advancing to {names[self._calibration_joint]}")

    def _wheel_calibration_cycle(self) -> None:
        if self._completed or not self._feedback_is_fresh():
            return
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            if self._commanded_hips is None:
                self._commanded_hips = self._measured_hips.copy()
                self._wheel_calibration_start_ns = now_ns
                self._latest_command = self._motor_command(
                    self._commanded_hips, np.zeros(4), hubs_at_rest=True
                )
                self.get_logger().info(
                    "Fresh hip feedback acquired; wheel test begins after the rest hold"
                )
                return

            elapsed_s = (now_ns - self._wheel_calibration_start_ns) * 1.0e-9
            hold_s = float(
                self.get_parameter("wheel_calibration.pre_drive_hold_s").value
            )
            drive_s = float(self.get_parameter("wheel_calibration.drive_s").value)
            if elapsed_s < hold_s:
                self._latest_command = self._motor_command(
                    self._commanded_hips, np.zeros(4), hubs_at_rest=True
                )
                return
            if elapsed_s < hold_s + drive_s:
                speed = float(
                    self.get_parameter("wheel_calibration.speed_rad_s").value
                )
                self._latest_command = self._motor_command(
                    self._commanded_hips, np.full(4, speed)
                )
                return
            self._latest_command = self._motor_command(
                self._commanded_hips, np.zeros(4), hubs_at_rest=True
            )
            self._completed = True
            self.get_logger().info(
                "Wheel calibration pulse complete; all hubs are back in REST mode"
            )

    def _stance_initialization_cycle(
        self, *, continue_to_planner: bool = False
    ) -> None:
        if self._completed or not self._feedback_is_fresh():
            return
        now_ns = self.get_clock().now().nanoseconds
        target = np.deg2rad(
            validate_four(self.get_parameter("stance.target_deg").value, "stance target")
        )
        max_rate = np.deg2rad(float(self.get_parameter("stance.hip_rate_deg_s").value))
        with self._lock:
            if self._commanded_hips is None:
                self._commanded_hips = self._measured_hips.copy()
                self.get_logger().info(
                    "Fresh feedback acquired; beginning IK-coordinated 45-degree stance transition"
                )
            error = target - self._commanded_hips
            rates = np.clip(error / self._planning_period_s, -max_rate, max_rate)
            next_hips = bounded_position_step(
                self._commanded_hips, target, max_rate * self._planning_period_s
            )
            wheel_rates = stance_wheel_speed_rad_s(
                self._commanded_hips,
                rates,
                float(self.get_parameter("stance.hip_to_wheel_m").value),
                float(self.get_parameter("stance.wheel_radius_m").value),
            )
            self._commanded_hips = next_hips
            tolerance = np.deg2rad(float(self.get_parameter("stance.tolerance_deg").value))
            reached = np.max(np.abs(target - self._commanded_hips)) <= tolerance
            self._latest_command = self._motor_command(
                self._commanded_hips,
                np.zeros(4) if reached else wheel_rates,
                hubs_at_rest=reached,
            )

        if not reached:
            self._stance_hold_start_ns = None
            return
        if self._stance_hold_start_ns is None:
            self._stance_hold_start_ns = now_ns
            self.get_logger().info("Nominal 45-degree stance reached; hubs entering REST")
            return
        held_s = (now_ns - self._stance_hold_start_ns) * 1.0e-9
        if held_s >= float(self.get_parameter("stance.hold_s").value):
            if continue_to_planner:
                self._known_ramp_stance_ready = True
                self._clock_start_ns = None
                self._stance_hold_start_ns = None
                self.get_logger().info(
                    "Automatic stance initialization complete; beginning known-ramp startup delay"
                )
            else:
                self._completed = True
                self.get_logger().info(
                    "Stance initialization complete; holding with hubs in REST"
                )

    def _start_plan_cycle(self) -> None:
        if not self._feedback_is_fresh():
            return
        if not self._required_planner_inputs_are_fresh():
            self._set_vicon_trigger(False, "required planner input unavailable")
            return
        if self._clock_start_ns is None:
            nominal = np.asarray(self._planner_config.nominal_hip_rad, dtype=float)
            initial_error_deg = np.rad2deg(
                np.max(np.abs(self._measured_hips - nominal))
            )
            limit_deg = float(
                self.get_parameter("known_ramp.max_initial_hip_error_deg").value
            )
            if initial_error_deg > limit_deg:
                self._completed = True
                self.get_logger().error(
                    f"Planner-mode start refused: hips differ from the nominal "
                    f"45-degree stance by {initial_error_deg:.2f} deg "
                    f"(limit {limit_deg:.2f} deg); "
                    "no command will be published"
                )
                return
            with self._lock:
                self._known_ramp_origin_hips = self._measured_hips.copy()
                self._commanded_hips = self._known_ramp_origin_hips.copy()
                self._planner_target_hips = self._commanded_hips.copy()
                self._planner_wheel_rates = np.zeros(4)
                self._planner_hubs_at_rest = True
                self._last_interpolation_ns = self.get_clock().now().nanoseconds
                self._planner_transition_start_hips = self._commanded_hips.copy()
                self._planner_transition_start_ns = self._last_interpolation_ns
                self._latest_command = self._motor_command(
                    self._commanded_hips, np.zeros(4), hubs_at_rest=True
                )
        elapsed = self._elapsed_motion_s()
        if self._completed or elapsed <= 0.0 or self._planning:
            return
        if self._mode == "known_ramp":
            self._set_vicon_trigger(True, "timed ramp motion started")
        if self._mode == "planner_posture_test":
            effective_duration = float(
                self.get_parameter("planner_posture_test.duration_s").value
            )
        elif self._mode == "known_ramp" and not bool(
            self.get_parameter("use_speed_command").value
        ):
            effective_duration = min(
                float(self.get_parameter("run_duration_s").value),
                float(self.get_parameter("hard_motion_limit_s").value),
            )
        else:
            effective_duration = None
        if effective_duration is not None and elapsed >= effective_duration:
            self._completed = True
            with self._lock:
                self._planner_wheel_rates = np.zeros(4)
                self._planner_hubs_at_rest = True
                self._latest_command = self._motor_command(
                    self._commanded_hips, np.zeros(4), hubs_at_rest=True
                )
            label = "Planner posture test" if self._mode == "planner_posture_test" else "Known-ramp run"
            self.get_logger().info(f"{label} complete; wheel hubs are in REST mode")
            return
        self._planning = True
        Thread(target=self._plan_cycle, args=(elapsed,), daemon=True).start()

    def _plan_cycle(self, elapsed: float) -> None:
        try:
            speed = (
                0.0
                if self._mode == "planner_posture_test"
                else float(self.get_parameter("speed_m_s").value)
            )
            if self._mode == "known_ramp" and bool(self.get_parameter("use_speed_command").value):
                speed_message_fresh = self._speed_command_is_fresh()
                with self._lock:
                    requested = self._target_speed_m_s if speed_message_fresh else 0.0
                    requested = float(np.clip(requested, 0.0, float(self.get_parameter("speed_command_max_m_s").value)))
                    acceleration_limit = float(
                        self.get_parameter("speed_command_accel_limit_m_s2").value
                    )
                    if acceleration_limit <= 0.0:
                        self._active_speed_m_s = requested
                    else:
                        step = acceleration_limit * self._planning_period_s
                        self._active_speed_m_s += float(
                            np.clip(
                                requested - self._active_speed_m_s,
                                -step,
                                step,
                            )
                        )
                    speed = self._active_speed_m_s
                    if requested == 0.0 and not self._warned_speed_timeout:
                        self.get_logger().warning(
                            "Speed command watchdog expired; ramping wheel speed to zero"
                        )
                        self._warned_speed_timeout = True
            if bool(self.get_parameter("use_odometry").value):
                if not self._required_planner_inputs_are_fresh():
                    self._set_vicon_trigger(False, "required planner input unavailable")
                    return
                with self._lock:
                    odometry = self._latest_odom
                x, y, yaw, debug_frame_id = self._planner_pose_from_odometry(
                    odometry
                )
            else:
                x, y, yaw = self._integrated_open_loop_pose(speed)
                debug_frame_id = "known_map"
            path = self._make_straight_horizon(
                start_x_m=x,
                start_y_m=y,
                yaw_rad=yaw,
                speed_m_s=speed,
                steps=self._planner_config.horizon_steps,
                dt_s=self._planner_config.dt_s,
                knot_spacing_m=self._planner_config.horizon_knot_spacing_m,
            )
            margin = float(self.get_parameter("map_margin_x_m").value)
            half_width = float(self.get_parameter("map_half_width_m").value)
            resolution = float(self.get_parameter("map_resolution_m").value)
            grid_x = self._regular_grid_coordinates(float(path.x_m[0]) - margin, float(path.x_m[-1]) + margin, resolution)
            grid_y = self._regular_grid_coordinates(y - half_width, y + half_width, resolution)
            xx, yy = np.meshgrid(grid_x, grid_y)
            heights = self._ramp.height(xx, yy)
            terrain = self._ElevationWindow(
                x_coordinates_m=grid_x,
                y_coordinates_m=grid_y,
                heights_m=heights,
                valid_mask=np.ones_like(heights, dtype=bool),
                timestamp_s=elapsed,
                frame_id="known_map",
            )
            if bool(self.get_parameter("use_terrain_window").value):
                with self._lock:
                    window = self._latest_terrain
                if window is None or not self._terrain_is_fresh():
                    self._stop_for_unavailable_terrain()
                    return
                wx = float(window.origin.position.x) + float(window.resolution_m) * np.arange(window.width)
                wy = float(window.origin.position.y) + float(window.resolution_m) * np.arange(window.height)
                values = np.asarray(window.elevation_m, dtype=float).reshape((window.height, window.width))
                valid = np.asarray(window.valid, dtype=bool).reshape((window.height, window.width))
                terrain = self._ElevationWindow(x_coordinates_m=wx, y_coordinates_m=wy, heights_m=values, valid_mask=valid, timestamp_s=elapsed, frame_id=window.header.frame_id)
                terrain = self._condition_live_terrain(terrain, x, y, yaw)
            result = self._session.plan_cycle(path=path, terrain=terrain)
            self._publish_debug_plan(path, debug_frame_id)
            with self._lock:
                now_ns = self.get_clock().now().nanoseconds
                self._planner_transition_start_hips = self._commanded_hips.copy()
                self._planner_transition_start_ns = now_ns
                self._planner_target_hips = np.asarray(
                    result.hip_command_rad, dtype=float
                ).copy()
                self._planner_wheel_rates = (
                    np.zeros(4)
                    if self._mode == "planner_posture_test"
                    else np.asarray(result.wheel_rolling_speed_rad_s, dtype=float).copy()
                )
                self._planner_hubs_at_rest = self._mode == "planner_posture_test"
            if not result.published_new_plan:
                self.get_logger().warning(f"Planner fallback: {result.status.value}: {result.message}")
        except Exception as exc:
            with self._lock:
                if self._commanded_hips is not None:
                    self._planner_target_hips = self._commanded_hips.copy()
                    self._planner_transition_start_hips = self._commanded_hips.copy()
                    self._planner_transition_start_ns = self.get_clock().now().nanoseconds
                    self._planner_wheel_rates = np.zeros(4)
                    self._planner_hubs_at_rest = True
                    self._latest_command = self._motor_command(
                        self._commanded_hips, np.zeros(4), hubs_at_rest=True
                    )
            self.get_logger().error(f"Planning cycle failed; commanding stop: {exc}")
        finally:
            self._planning = False

    def _motor_command(
        self, hip_rad, wheel_rad_s, hubs_at_rest: bool = False
    ) -> MotorCmdStamped:
        hips = validate_four(hip_rad, "hip command")
        wheels = validate_four(wheel_rad_s, "wheel command")
        message = MotorCmdStamped()
        legs = (message.module_a, message.module_b, message.module_c, message.module_d)
        for leg, hip, wheel in zip(legs, hips, wheels):
            leg.hip.position = hip
            leg.hip.kp = float(self.get_parameter("hip_kp").value)
            leg.hip.ki = float(self.get_parameter("hip_ki").value)
            leg.hip.kd = float(self.get_parameter("hip_kd").value)
            leg.hip.motor_mode = int(self.get_parameter("position_mode").value)
            leg.steering.position = 0.0
            leg.steering.motor_mode = int(self.get_parameter("position_mode").value)
            leg.hub.velocity = radians_per_second_to_rpm10(wheel)
            hub_mode_parameter = "rest_mode" if hubs_at_rest else "velocity_mode"
            leg.hub.motor_mode = int(self.get_parameter(hub_mode_parameter).value)
        return message

    def _publish_latest(self) -> None:
        if not bool(self.get_parameter("armed").value):
            return
        if self._mode == "disabled":
            return
        feedback_modes = (
            "hip_test",
            "hip_calibration",
            "wheel_calibration",
            "stance_initialization",
            "planner_posture_test",
            "known_ramp",
        )
        if self._mode in feedback_modes and not self._feedback_is_fresh():
            if not self._warned_waiting_for_feedback:
                self.get_logger().warning(
                    "Hip-motion interlock active: waiting for fresh FL/FR/RL/RR joint feedback"
                )
                self._warned_waiting_for_feedback = True
            return
        planner_control_active = self._mode == "planner_posture_test" or (
            self._mode == "known_ramp" and self._known_ramp_stance_ready
        )
        if planner_control_active and self._known_ramp_origin_hips is None:
            return
        # Fresh feedback alone must not release a position command.  Wait for
        # the active 10 Hz control cycle to create a command from that feedback.
        with self._lock:
            startup_command_ready = (
                self._commanded_hips is not None and self._latest_command is not None
            )
        if not startup_command_ready:
            return
        self._warned_waiting_for_feedback = False
        with self._lock:
            if planner_control_active:
                now_ns = self.get_clock().now().nanoseconds
                if self._planner_target_hips is None or self._commanded_hips is None:
                    return
                if self._last_interpolation_ns is None:
                    self._last_interpolation_ns = now_ns
                dt_s = max(
                    0.0, (now_ns - self._last_interpolation_ns) * 1.0e-9
                )
                # Prevent a large target jump after pausing and resuming Isaac.
                dt_s = min(dt_s, 0.1)
                if dt_s > 0.0:
                    smoothing_s = float(
                        self.get_parameter("known_ramp.command_smoothing_s").value
                    )
                    transition_start = self._planner_transition_start_hips
                    transition_start_ns = self._planner_transition_start_ns
                    if transition_start is None or transition_start_ns is None:
                        transition_start = self._commanded_hips
                        transition_start_ns = now_ns
                    desired_hips = smoothstep_position(
                        transition_start,
                        self._planner_target_hips,
                        max(0.0, (now_ns - transition_start_ns) * 1.0e-9),
                        smoothing_s,
                    )
                    hip_rate = np.deg2rad(
                        float(
                            self.get_parameter(
                                "known_ramp.hip_rate_limit_deg_s"
                            ).value
                        )
                    )
                    self._commanded_hips = bounded_position_step(
                        self._commanded_hips,
                        desired_hips,
                        hip_rate * dt_s,
                    )
                    self._last_interpolation_ns = now_ns
                message = self._motor_command(
                    self._commanded_hips,
                    self._planner_wheel_rates,
                    hubs_at_rest=self._planner_hubs_at_rest,
                )
            else:
                message = self._latest_command
        now = self.get_clock().now().to_msg()
        message.header.seq += 1
        message.header.time = now
        message.header.frame_id = "kilin_known_terrain_controller"
        self._publisher.publish(message)

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            hips = named_hip_positions(message.name, message.position)
        except ValueError as exc:
            feedback_modes = (
                "hip_test",
                "hip_calibration",
                "wheel_calibration",
                "stance_initialization",
                "planner_posture_test",
                "known_ramp",
            )
            if self._mode in feedback_modes and not self._warned_waiting_for_feedback:
                self.get_logger().warning(str(exc))
                self._warned_waiting_for_feedback = True
            return
        with self._lock:
            self._measured_hips = hips
            self._last_joint_state_time = self.get_clock().now()

    def _motor_state_callback(self, message: MotorStateStamped) -> None:
        """Use the shared Kilin motor-state contract: A/B/C/D = FL/FR/RL/RR."""
        hips = np.asarray(
            [
                message.module_a.hip.position,
                message.module_b.hip.position,
                message.module_c.hip.position,
                message.module_d.hip.position,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(hips)):
            if not self._warned_waiting_for_feedback:
                self.get_logger().warning("MotorStateStamped contains non-finite hip positions")
                self._warned_waiting_for_feedback = True
            return
        with self._lock:
            self._measured_hips = hips
            self._last_joint_state_time = self.get_clock().now()

    def stop(self) -> None:
        if self._vicon_trigger_test_start_timer is not None:
            self._vicon_trigger_test_start_timer.cancel()
            self._vicon_trigger_test_start_timer = None
        if self._vicon_trigger_test_stop_timer is not None:
            self._vicon_trigger_test_stop_timer.cancel()
            self._vicon_trigger_test_stop_timer = None
        self._set_vicon_trigger(False, "controller shutdown")
        self._release_vicon_trigger()
        if bool(self.get_parameter("armed").value):
            with self._lock:
                hips = self._commanded_hips
            # Never emit a nominal fallback pose during shutdown.  If this
            # controller never initialized from measured feedback, it has not
            # issued a position command and must remain silent.
            if hips is None:
                return
            for _ in range(3):
                command = self._motor_command(
                    hips,
                    np.zeros(4),
                    hubs_at_rest=self._mode
                    in (
                        "hip_test",
                        "hip_calibration",
                        "wheel_calibration",
                        "stance_initialization",
                        "planner_posture_test",
                    ),
                )
                self._publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KnownTerrainController()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
