from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='kilin_imu',
			executable='imu_subscriber_logger',
			name='imu_subscriber_logger',
			parameters=[{'test_mode': False}]  # Set to True for test mode
		),
		Node(
			package='kilin_imu',
			executable='IMUpublisher',
			name='imu_publisher',
			parameters=[{'baud': 115200, 'port': '/dev/ttyACM0', 'publish_period': 100, "print_info_period": 10000}]
		)
	])