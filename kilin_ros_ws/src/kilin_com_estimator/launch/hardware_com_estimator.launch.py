from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("kilin_com_estimator")
    default_config = f"{package_share}/config/hardware_com_estimator.yaml"

    config = LaunchConfiguration("config")
    arm_joint_state_topic = LaunchConfiguration("arm_joint_state_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument(
                "arm_joint_state_topic", default_value="/joint_states"
            ),
            Node(
                package="kilin_com_estimator",
                executable="kilin_com_estimator",
                name="kilin_com_estimator",
                output="screen",
                parameters=[
                    config,
                    {"arm_joint_state_topic": arm_joint_state_topic},
                ],
            ),
        ]
    )
