"""Republish Isaac ground truth as a controlled FAST-LIO-like estimate."""
import math
import random
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class Emulator(Node):
    def __init__(self):
        super().__init__("kilin_fastlio_emulator")
        self.declare_parameter("ground_truth_topic", "/kilin/isaac/ground_truth/odometry")
        self.declare_parameter("output_topic", "/Odometry")
        self.declare_parameter("map_frame", "camera_init")
        self.declare_parameter("base_frame", "body")
        self.declare_parameter("position_noise_std_m", 0.0)
        self.declare_parameter("yaw_noise_std_rad", 0.0)
        self.pub = self.create_publisher(Odometry, self.get_parameter("output_topic").value, 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.get_parameter("ground_truth_topic").value, self.cb, 10)
    def cb(self, msg):
        out = Odometry(); out.header = msg.header; out.header.frame_id = self.get_parameter("map_frame").value; out.child_frame_id = self.get_parameter("base_frame").value
        out.pose = msg.pose; out.twist = msg.twist
        s = float(self.get_parameter("position_noise_std_m").value)
        out.pose.pose.position.x += random.gauss(0.0, s); out.pose.pose.position.y += random.gauss(0.0, s)
        self.pub.publish(out)
        tf = TransformStamped(); tf.header = out.header; tf.child_frame_id = out.child_frame_id; tf.transform.translation.x = out.pose.pose.position.x; tf.transform.translation.y = out.pose.pose.position.y; tf.transform.translation.z = out.pose.pose.position.z; tf.transform.rotation = out.pose.pose.orientation; self.tf.sendTransform(tf)
def main(): rclpy.init(); node=Emulator(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
