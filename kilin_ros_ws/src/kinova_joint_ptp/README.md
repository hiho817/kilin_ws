# kinova_joint_ptp

Hardware-agnostic ROS 2 joint-space PTP wrapper for Kinova Gen3. The server
accepts `kinova_ptp_interfaces/action/JointPtp` goals and forwards a one-point
trajectory to a standard `control_msgs/action/FollowJointTrajectory` server.

The package does not depend on MoveIt, Kortex, or Isaac Sim. The backend is
selected by the configured FollowJointTrajectory action name:

- Hardware: `ros2_kortex` and `joint_trajectory_controller`.
- Simulation: an Isaac Sim Python FollowJointTrajectory adapter.

## Build

```bash
sudo apt install ros-humble-control-msgs
cd ~/kilin_ws/kilin_ros_ws
colcon build --packages-up-to kinova_joint_ptp
source install/setup.bash
```

## Run

```bash
ros2 launch kinova_joint_ptp joint_ptp.launch.py
```

The server rejects goals until all configured joint states have been observed.
Positions are in radians and duration is in seconds.

The default joint names are `joint_1` through `joint_7`. Change
`config/joint_ptp.yaml` when the hardware controller uses a robot-name prefix.

## Send a goal

This example commands the current Kilin standard pose in radians:

```bash
ros2 action send_goal --feedback /kinova_joint_ptp \
  kinova_ptp_interfaces/action/JointPtp \
  "{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7], \
    positions: [0.0, -1.4999, 0.0, 2.5656, 0.0, 0.4000, 0.0], \
    velocities: [], duration_sec: 3.0}"
```
