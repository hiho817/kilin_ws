"""Build a planner-ready local elevation window from a FAST-LIO map."""

from __future__ import annotations

import json
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from kilin_msgs.msg import TerrainWindow
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker

from .elevation import (
    elevation_from_points,
    front_roi,
    grid_coordinates,
    retained_window_roi,
    transform_points,
    voxel_downsample,
)


def _rotation_matrix(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return np.array(
        [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]],
        dtype=float,
    )


class LocalTerrain(Node):
    """Extract a robust map-frame 2.5-D window and RViz inspection topics."""

    def __init__(self):
        super().__init__("kilin_local_terrain_mapping")
        defaults = {
            "odometry_topic": "/kilin/fastlio/odometry",
            "window_topic": "/kilin/terrain/local_window",
            "terrain_source": "pointcloud",
            "pointcloud_topic": "/cloud_registered",
            "output_frame": "map",
            "forward_m": 3.0, "rear_m": 1.0, "half_width_m": 1.0,
            "resolution_m": 0.05, "rate_hz": 5.0,
            "minimum_points_per_cell": 2, "height_percentile": 50.0,
            "max_cloud_age_s": 2.0, "rolling_window_s": 1.0,
            # A terrain point is admitted only while it is observed ahead of
            # the robot.  It is then retained in map coordinates until it
            # leaves the planner window; this avoids losing an exit ramp when
            # the ramp occludes the sensor after its first observation.
            "retain_observed_terrain": True,
            "retained_terrain.max_age_s": 45.0,
            "retained_terrain.margin_m": 0.25,
            "retained_terrain.voxel_m": 0.05,
            # These are physical terrain plausibility limits, expressed from
            # the measured hip-axis height in the levelled map frame. They
            # prevent ceiling/deep outliers from entering the planner map.
            "ground_filter.enabled": True,
            "ground_filter.maximum_above_hip_axis_m": 0.10,
            "ground_filter.maximum_below_hip_axis_m": 1.50,
            "ground_filter.maximum_cell_vertical_span_m": 0.35,
            "front_roi.enabled": True, "front_roi.frame": "hip_axis_center",
            "front_roi.minimum_x_m": 0.2, "front_roi.maximum_x_m": 3.0,
            "front_roi.half_width_m": 0.9,
            "front_roi.minimum_z_m": -1.0, "front_roi.maximum_z_m": 0.2,
            "analytic_ramp.height_m": 0.08, "analytic_ramp.start_x_m": 0.75,
            "analytic_ramp.up_ramp_length_m": 0.30, "analytic_ramp.deck_length_m": 0.35,
            "analytic_ramp.down_ramp_length_m": 0.30, "analytic_ramp.track_center_y_m": 0.25,
            "analytic_ramp.track_width_m": 0.34,
            "analytic_second.enabled": True, "analytic_second.height_m": 0.08,
            "analytic_second.start_x_m": 2.70, "analytic_second.up_ramp_length_m": 0.30,
            "analytic_second.deck_length_m": 0.35, "analytic_second.down_ramp_length_m": 0.30,
            "analytic_second.track_center_y_m": -0.25, "analytic_second.track_width_m": 0.34,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.pose = None
        self.cloud_buffer = deque()
        self.cloud_stamp_ns = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(TerrainWindow, self.get_parameter("window_topic").value, 1)
        self.cells_pub = self.create_publisher(PointCloud2, "/kilin/terrain/local_window/cells", 1)
        self.bounds_pub = self.create_publisher(Marker, "/kilin/terrain/local_window/bounds", 1)
        self.overlay_pub = self.create_publisher(String, "/kilin/terrain/local_window_overlay", 1)
        self.create_subscription(Odometry, self.get_parameter("odometry_topic").value, self.odom, 10)
        self.create_subscription(PointCloud2, self.get_parameter("pointcloud_topic").value, self.cloud_callback, 2)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.publish)

    def odom(self, message):
        self.pose = message

    def cloud_callback(self, message):
        raw = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        points = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(float)
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        points = self._transform_to_frame(points, message.header.frame_id, str(self.get_parameter("output_frame").value))
        if points is None:
            return
        # Fresh sensor input keeps a previously observed, still-local terrain
        # patch usable even if this particular scan has no new ROI points.
        self.cloud_stamp_ns = stamp_ns
        # Gate points in the instantaneous body-fixed front ROI before they
        # enter memory.  This retains only causal terrain observations and
        # excludes the body/rear hemisphere from the terrain product.
        points = self._front_roi_points(points)
        if points is None or not len(points):
            return
        points = voxel_downsample(points, float(self.get_parameter("retained_terrain.voxel_m").value))
        self.cloud_buffer.append((stamp_ns, points))
        memory_s = (float(self.get_parameter("retained_terrain.max_age_s").value)
                    if bool(self.get_parameter("retain_observed_terrain").value)
                    else float(self.get_parameter("rolling_window_s").value))
        cutoff_ns = stamp_ns - int(memory_s * 1e9)
        while self.cloud_buffer and self.cloud_buffer[0][0] < cutoff_ns:
            self.cloud_buffer.popleft()

    def _transform_to_frame(self, points, source_frame, target_frame):
        if source_frame == target_frame:
            return points
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as error:
            self.get_logger().warning(f"Cannot transform terrain cloud {source_frame!r} to {target_frame!r}: {error}", throttle_duration_sec=2.0)
            return None
        rotation = _rotation_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        return transform_points(points, rotation, [translation.x, translation.y, translation.z])

    def _front_roi_points(self, points):
        if not bool(self.get_parameter("front_roi.enabled").value):
            return points
        output_frame = str(self.get_parameter("output_frame").value)
        roi_frame = str(self.get_parameter("front_roi.frame").value)
        hip_points = self._transform_to_frame(points, output_frame, roi_frame)
        if hip_points is None:
            return None
        kept = front_roi(
            hip_points,
            float(self.get_parameter("front_roi.minimum_x_m").value),
            float(self.get_parameter("front_roi.maximum_x_m").value),
            float(self.get_parameter("front_roi.half_width_m").value),
            float(self.get_parameter("front_roi.minimum_z_m").value),
            float(self.get_parameter("front_roi.maximum_z_m").value),
        )
        # ``kept`` is expressed in the hip frame; return the corresponding
        # fixed-frame points so every scan shares a stable elevation frame.
        if len(kept) == len(hip_points):
            return points
        keep = ((hip_points[:, 0] >= float(self.get_parameter("front_roi.minimum_x_m").value)) &
                (hip_points[:, 0] <= float(self.get_parameter("front_roi.maximum_x_m").value)) &
                (np.abs(hip_points[:, 1]) <= float(self.get_parameter("front_roi.half_width_m").value)) &
                (hip_points[:, 2] >= float(self.get_parameter("front_roi.minimum_z_m").value)) &
                (hip_points[:, 2] <= float(self.get_parameter("front_roi.maximum_z_m").value)))
        return points[keep]

    def _retained_window_points(self):
        if not self.cloud_buffer:
            return None
        points = np.vstack([entry[1] for entry in self.cloud_buffer])
        if not bool(self.get_parameter("retain_observed_terrain").value):
            return points
        output_frame = str(self.get_parameter("output_frame").value)
        roi_frame = str(self.get_parameter("front_roi.frame").value)
        hip_points = self._transform_to_frame(points, output_frame, roi_frame)
        if hip_points is None:
            return None
        margin = float(self.get_parameter("retained_terrain.margin_m").value)
        retained = retained_window_roi(
            hip_points,
            float(self.get_parameter("rear_m").value),
            float(self.get_parameter("forward_m").value),
            float(self.get_parameter("half_width_m").value),
            margin,
        )
        if len(retained) == len(hip_points):
            return points
        keep = ((hip_points[:, 0] >= -float(self.get_parameter("rear_m").value) - margin) &
                (hip_points[:, 0] <= float(self.get_parameter("forward_m").value) + margin) &
                (np.abs(hip_points[:, 1]) <= float(self.get_parameter("half_width_m").value) + margin))
        return points[keep]

    def _ground_height_gate(self, points):
        """Reject overhead/deep points using the current hip-axis map height."""
        if not bool(self.get_parameter("ground_filter.enabled").value):
            return points
        hip_z = float(self.pose.pose.pose.position.z)
        relative_z = points[:, 2] - hip_z
        keep = ((relative_z <= float(self.get_parameter("ground_filter.maximum_above_hip_axis_m").value)) &
                (relative_z >= -float(self.get_parameter("ground_filter.maximum_below_hip_axis_m").value)))
        return points[keep]

    def _ramp(self, x, y, prefix):
        value = lambda name: self.get_parameter(f"{prefix}.{name}").value
        height, start = float(value("height_m")), float(value("start_x_m"))
        up_length, deck_length, down_length = float(value("up_ramp_length_m")), float(value("deck_length_m")), float(value("down_ramp_length_m"))
        up_end, deck_end, down_end = start + up_length, start + up_length + deck_length, start + up_length + deck_length + down_length
        elevation = np.zeros_like(x)
        up, deck, down = (x >= start) & (x < up_end), (x >= up_end) & (x < deck_end), (x >= deck_end) & (x <= down_end)
        elevation[up] = height * (x[up] - start) / up_length
        elevation[deck] = height
        elevation[down] = height * (1.0 - (x[down] - deck_end) / down_length)
        return np.where(np.abs(y - float(value("track_center_y_m"))) <= 0.5 * float(self.get_parameter("analytic_ramp.track_width_m").value), elevation, 0.0)

    def _publish_visualization(self, header, xs, ys, elevation, valid):
        rows, columns = np.nonzero(valid)
        points = np.column_stack((xs[columns], ys[rows], elevation[rows, columns])).astype(np.float32)
        self.cells_pub.publish(point_cloud2.create_cloud_xyz32(header, points.tolist()))
        marker = Marker(header=header, ns="local_terrain_window", id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
        marker.scale.x = 0.015
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.8, 0.2, 1.0
        z = float(np.nanmedian(elevation[valid])) if np.any(valid) else 0.0
        marker.points = [Point(x=float(x), y=float(y), z=z) for x, y in [(xs[0], ys[0]), (xs[-1], ys[0]), (xs[-1], ys[-1]), (xs[0], ys[-1]), (xs[0], ys[0])]]
        self.bounds_pub.publish(marker)

    def publish(self):
        if self.pose is None:
            return
        output_frame = str(self.get_parameter("output_frame").value)
        if self.pose.header.frame_id != output_frame:
            self.get_logger().error(f"Odometry frame {self.pose.header.frame_id!r} is not {output_frame!r}", throttle_duration_sec=2.0)
            return
        position, resolution = self.pose.pose.pose.position, float(self.get_parameter("resolution_m").value)
        xs, ys = grid_coordinates(position.x, position.y, float(self.get_parameter("forward_m").value), float(self.get_parameter("rear_m").value), float(self.get_parameter("half_width_m").value), resolution)
        x, y = np.meshgrid(xs, ys)
        if str(self.get_parameter("terrain_source").value) == "analytic":
            elevation, valid = self._ramp(x, y, "analytic_ramp"), np.ones_like(x, dtype=bool)
        else:
            if self.cloud_stamp_ns is None:
                self.get_logger().warning("Waiting for a FAST-LIO terrain cloud", throttle_duration_sec=2.0)
                return
            if self.get_clock().now().nanoseconds - self.cloud_stamp_ns > int(float(self.get_parameter("max_cloud_age_s").value) * 1e9):
                self.get_logger().warning("Terrain cloud is stale; withholding terrain window", throttle_duration_sec=2.0)
                return
            points = self._retained_window_points()
            if points is None:
                return
            points = self._ground_height_gate(points)
            elevation, valid, _ = elevation_from_points(
                points,
                xs,
                ys,
                resolution,
                int(self.get_parameter("minimum_points_per_cell").value),
                float(self.get_parameter("height_percentile").value),
                float(self.get_parameter("ground_filter.maximum_cell_vertical_span_m").value),
            )
        message = TerrainWindow()
        message.header = self.pose.header
        message.header.frame_id = output_frame
        message.origin.position.x, message.origin.position.y = float(xs[0]), float(ys[0])
        message.resolution_m, message.width, message.height = resolution, len(xs), len(ys)
        message.elevation_m, message.valid = elevation.astype(np.float32).ravel().tolist(), valid.astype(np.uint8).ravel().tolist()
        self.pub.publish(message)
        self._publish_visualization(message.header, xs, ys, elevation, valid)
        self.overlay_pub.publish(String(data=json.dumps({"points": np.column_stack((x[valid], y[valid], elevation[valid])).astype(float).tolist()})))


def main():
    rclpy.init()
    node = LocalTerrain()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
