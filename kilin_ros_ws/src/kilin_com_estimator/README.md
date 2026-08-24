# Kilin hardware COM estimator

This node loads the self-contained `kilin_robot_description` model and publishes
the combined Kilin + Kinova centre of mass and nominal wheel-bottom points as
`kilin_msgs/BalanceStateStamped` on `/kilin/balance_state`.

Inputs:

- `/motor/state`: A/B/C/D are FL/FR/RL/RR. Each hip angle is computed as
  `position + position_diff`, because hardware defines
  `position_diff = actual - position`.
- `/joint_states`: Kinova positions are selected by `name[]`, not by message
  order. Extra joints such as `robotiq_85_left_knuckle_joint` are ignored.

The first hardware version intentionally uses only measured hip and Kinova
angles. Steering, wheel rotation, and the unsensed passive suspension joints
remain at zero in the URDF.

## No-IMU limitation

The robot currently has no hardware IMU. Therefore this version publishes in
`base_link_assumed_level` and is valid for flat-ground estimator and arm-motion
validation only. On stairs, base-frame XY is not the horizontal plane and the
COM gravity projection will be wrong. Do not enable stair `com_closed_loop`
from this estimator until a base-orientation source is added.

Sensor contact fields are set to false/NaN because no hardware contact sensor
source is connected. The balance monitor and stair controller use the nominal
wheel geometry, independently of these diagnostic fields.

## Build and run on Jetson

```bash
cd ~/kilin_ws/kilin_ros_ws
colcon build --packages-up-to kilin_com_estimator --symlink-install
source install/setup.bash
ros2 launch kilin_com_estimator hardware_com_estimator.launch.py
```

If Kinova publishes on `/kinova/joint_states`, override the configured topic:

```bash
ros2 launch kilin_com_estimator hardware_com_estimator.launch.py \
  arm_joint_state_topic:=/kinova/joint_states
```

Verify that both inputs are fresh and the estimator is publishing:

```bash
ros2 topic hz /motor/state
ros2 topic hz /joint_states
ros2 topic hz /kilin/balance_state
ros2 topic echo /kilin/balance_state --once
```
