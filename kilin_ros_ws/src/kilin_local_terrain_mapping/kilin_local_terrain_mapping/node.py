import json

import numpy as np
import rclpy
from kilin_msgs.msg import TerrainWindow
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class LocalTerrain(Node):
    def __init__(self):
        super().__init__("kilin_local_terrain_mapping")
        defaults = {
            "odometry_topic": "/Odometry", "window_topic": "/kilin/terrain/local_window", "terrain_source": "analytic", "pointcloud_topic": "/cloud_registered", "forward_m": 6.0, "rear_m": 1.0, "half_width_m": 1.0, "resolution_m": 0.05, "rate_hz": 5.0,
            "analytic_ramp.height_m": 0.08, "analytic_ramp.start_x_m": 0.75, "analytic_ramp.up_ramp_length_m": 0.30, "analytic_ramp.deck_length_m": 0.35, "analytic_ramp.down_ramp_length_m": 0.30, "analytic_ramp.track_center_y_m": 0.25, "analytic_ramp.track_width_m": 0.34,
            "analytic_second.enabled": True, "analytic_second.height_m": 0.08, "analytic_second.start_x_m": 2.70, "analytic_second.up_ramp_length_m": 0.30, "analytic_second.deck_length_m": 0.35, "analytic_second.down_ramp_length_m": 0.30, "analytic_second.track_center_y_m": -0.25,
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.pose = None; self.points = []
        self.pub = self.create_publisher(TerrainWindow, self.get_parameter("window_topic").value, 1)
        self.overlay_pub = self.create_publisher(String, "/kilin/terrain/local_window_overlay", 1)
        self.create_subscription(Odometry, self.get_parameter("odometry_topic").value, self.odom, 10)
        self.create_subscription(PointCloud2, self.get_parameter("pointcloud_topic").value, self.cloud, 2)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.publish)

    def odom(self, msg): self.pose = msg
    def cloud(self, msg): self.points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    def _ramp(self, x, y, prefix):
        value = lambda name: self.get_parameter(f"{prefix}.{name}").value
        height, start = float(value("height_m")), float(value("start_x_m"))
        up_length, deck_length, down_length = float(value("up_ramp_length_m")), float(value("deck_length_m")), float(value("down_ramp_length_m"))
        up_end, deck_end, down_end = start + up_length, start + up_length + deck_length, start + up_length + deck_length + down_length
        elevation = np.zeros_like(x); up = (x >= start) & (x < up_end); deck = (x >= up_end) & (x < deck_end); down = (x >= deck_end) & (x <= down_end)
        elevation[up] = height * (x[up] - start) / up_length; elevation[deck] = height; elevation[down] = height * (1.0 - (x[down] - deck_end) / down_length)
        return np.where(np.abs(y - float(value("track_center_y_m"))) <= 0.5 * float(self.get_parameter("analytic_ramp.track_width_m").value), elevation, 0.0)

    def analytic(self, x, y):
        elevation = self._ramp(x, y, "analytic_ramp")
        return np.maximum(elevation, self._ramp(x, y, "analytic_second")) if self.get_parameter("analytic_second.enabled").value else elevation

    def publish(self):
        if self.pose is None: return
        position = self.pose.pose.pose.position; resolution = float(self.get_parameter("resolution_m").value)
        xs = np.arange(position.x - float(self.get_parameter("rear_m").value), position.x + float(self.get_parameter("forward_m").value) + resolution * 0.1, resolution)
        ys = np.arange(position.y - float(self.get_parameter("half_width_m").value), position.y + float(self.get_parameter("half_width_m").value) + resolution * 0.1, resolution)
        x, y = np.meshgrid(xs, ys); elevation = self.analytic(x, y); valid = np.ones_like(elevation, dtype=np.uint8)
        if self.get_parameter("terrain_source").value == "pointcloud":
            elevation = np.zeros_like(elevation); valid = np.zeros_like(elevation, dtype=np.uint8)
            for point_x, point_y, point_z in self.points:
                ix, iy = int((point_x - xs[0]) / resolution), int((point_y - ys[0]) / resolution)
                if 0 <= ix < len(xs) and 0 <= iy < len(ys) and (not valid[iy, ix] or point_z > elevation[iy, ix]): elevation[iy, ix] = point_z; valid[iy, ix] = 1
        msg = TerrainWindow(); msg.header = self.pose.header; msg.header.frame_id = "camera_init"; msg.origin.position.x = float(xs[0]); msg.origin.position.y = float(ys[0]); msg.resolution_m = resolution; msg.width = len(xs); msg.height = len(ys); msg.elevation_m = elevation.astype(np.float32).ravel().tolist(); msg.valid = valid.ravel().tolist(); self.pub.publish(msg)
        points = [[float(xs[ix]), float(ys[iy]), float(elevation[iy, ix])] for iy in range(0, len(ys), 2) for ix in range(0, len(xs), 2) if valid[iy, ix]]
        self.overlay_pub.publish(String(data=json.dumps({"points": points})))


def main():
    rclpy.init(); node = LocalTerrain(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
