#!/usr/bin/env python3
"""Convert FAST-LIO's camera_init->body pose to map->hip_axis_center."""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


def transform_matrix(translation, quaternion_xyzw) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def message_pose_matrix(message: Odometry) -> np.ndarray:
    pose = message.pose.pose
    return transform_matrix(
        [pose.position.x, pose.position.y, pose.position.z],
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    )


def tf_matrix(message) -> np.ndarray:
    transform = message.transform
    return transform_matrix(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ],
    )


def compose_target_pose(
    target_source: np.ndarray,
    source_body: np.ndarray,
    body_target_child: np.ndarray,
) -> np.ndarray:
    """Return target_parent->target_child from the three audited transforms."""
    return target_source @ source_body @ body_target_child


def transformed_twist(
    linear_body: np.ndarray,
    angular_body: np.ndarray,
    body_target_child: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Move a body-origin twist to the fixed target-child origin and axes."""
    rotation_body_child = body_target_child[:3, :3]
    body_to_child = body_target_child[:3, 3]
    linear_at_child_body = np.asarray(linear_body) + np.cross(
        np.asarray(angular_body), body_to_child
    )
    child_from_body = rotation_body_child.T
    return (
        child_from_body @ linear_at_child_body,
        child_from_body @ np.asarray(angular_body),
    )


def rotate_covariance(covariance, rotation: np.ndarray) -> list[float]:
    values = np.asarray(covariance, dtype=float).reshape(6, 6)
    transform = np.zeros((6, 6), dtype=float)
    transform[:3, :3] = rotation
    transform[3:, 3:] = rotation
    return (transform @ values @ transform.T).ravel().tolist()


class HipCenterOdometryAdapter(Node):
    def __init__(self) -> None:
        super().__init__("kilin_fastlio_hip_center_odometry")
        defaults = {
            "input_topic": "/Odometry",
            "output_topic": "/kilin/fastlio/odometry",
            "expected_source_parent_frame": "camera_init",
            "expected_source_child_frame": "body",
            "target_parent_frame": "map",
            "target_child_frame": "hip_axis_center",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        # The two transforms required here are static calibration transforms.
        # Do not subscribe this node to FAST-LIO's dynamic /tf stream: during
        # bag replay its asynchronous output can be slightly out of timestamp
        # order, which produces TF_OLD_DATA although neither transform needs
        # that stream.
        self._tf_buffer = Buffer()
        self._static_tf_subscription = self.create_subscription(
            TFMessage, "/tf_static", self._static_tf_callback,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10
        )
        self._subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._callback,
            10,
        )
        self._warned_frames = False
        self._warned_tf = False
        self.get_logger().info(
            "FAST-LIO hip-center adapter ready: /Odometry -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def _static_tf_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self._tf_buffer.set_transform_static(transform, "kilin_fastlio_static_frames")

    def _callback(self, message: Odometry) -> None:
        expected_parent = str(
            self.get_parameter("expected_source_parent_frame").value
        )
        expected_child = str(self.get_parameter("expected_source_child_frame").value)
        if (
            message.header.frame_id != expected_parent
            or message.child_frame_id != expected_child
        ):
            if not self._warned_frames:
                self.get_logger().error(
                    "Rejecting FAST-LIO odometry with unexpected frames: "
                    f"{message.header.frame_id}->{message.child_frame_id}; expected "
                    f"{expected_parent}->{expected_child}"
                )
                self._warned_frames = True
            return
        self._warned_frames = False

        target_parent = str(self.get_parameter("target_parent_frame").value)
        target_child = str(self.get_parameter("target_child_frame").value)
        try:
            target_source = tf_matrix(
                self._tf_buffer.lookup_transform(
                    target_parent, expected_parent, Time()
                )
            )
            body_target_child = tf_matrix(
                self._tf_buffer.lookup_transform(
                    expected_child, target_child, Time()
                )
            )
        except TransformException as error:
            if not self._warned_tf:
                self.get_logger().warning(
                    f"Waiting for audited FAST-LIO frame chain: {error}"
                )
                self._warned_tf = True
            return
        self._warned_tf = False

        target_pose = compose_target_pose(
            target_source, message_pose_matrix(message), body_target_child
        )
        quaternion = Rotation.from_matrix(target_pose[:3, :3]).as_quat()
        output = Odometry()
        output.header = message.header
        output.header.frame_id = target_parent
        output.child_frame_id = target_child
        output.pose.pose.position.x = float(target_pose[0, 3])
        output.pose.pose.position.y = float(target_pose[1, 3])
        output.pose.pose.position.z = float(target_pose[2, 3])
        output.pose.pose.orientation.x = float(quaternion[0])
        output.pose.pose.orientation.y = float(quaternion[1])
        output.pose.pose.orientation.z = float(quaternion[2])
        output.pose.pose.orientation.w = float(quaternion[3])

        # FAST-LIO reports twist in its child/body axes. Move the velocity to
        # the hip-center origin and express it in hip-center axes.
        twist = message.twist.twist
        linear, angular = transformed_twist(
            np.asarray([twist.linear.x, twist.linear.y, twist.linear.z]),
            np.asarray([twist.angular.x, twist.angular.y, twist.angular.z]),
            body_target_child,
        )
        output.twist.twist.linear.x = float(linear[0])
        output.twist.twist.linear.y = float(linear[1])
        output.twist.twist.linear.z = float(linear[2])
        output.twist.twist.angular.x = float(angular[0])
        output.twist.twist.angular.y = float(angular[1])
        output.twist.twist.angular.z = float(angular[2])

        parent_rotation = target_source[:3, :3]
        output.pose.covariance = rotate_covariance(
            message.pose.covariance, parent_rotation
        )
        output.twist.covariance = rotate_covariance(
            message.twist.covariance, body_target_child[:3, :3].T
        )
        self._publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = HipCenterOdometryAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
