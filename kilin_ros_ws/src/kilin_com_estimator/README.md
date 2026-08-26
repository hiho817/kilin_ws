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
- `/kilin/stair_terrain`: the gait-synchronized stair rise, tread index, and
  support mask for FL/FR/RL/RR.

The hardware-only `com_bias_base_m` parameter applies a force-plate calibration
to the URDF COM in `base_link` before the result is rotated into the
gravity-aligned output frame. Its default is `[0.0, 0.0, 0.0]`; the hardware
YAML currently uses `[0.0054, -0.0011, 0.0]` m from the preliminary level-floor
measurement. Simulation remains unmodified unless it explicitly sets this
parameter.

The first hardware version intentionally uses only measured hip and Kinova
angles. Steering, wheel rotation, and the unsensed passive suspension joints
remain at zero in the URDF.

## No-IMU known-stair orientation

The robot has no hardware IMU. For generated stair gaits, the controller now
publishes the known tread height of every supporting wheel. From three supports
the estimator solves

`n dot (p_i - p_0) = (level_i - level_0) * stair_rise`

with `|n|=1`, where `n` is world vertical expressed in `base_link`. The selected
solution has positive Z and remains continuous with the previous estimate. COM
and wheel points are rotated into `base_link_gravity_aligned`, and
`base_to_output_rotation` carries the same transform back to the controller.

This removes the flat-base assumption but does not measure contact. A missed
tread, wheel slip, chassis flex, or wrong stair dimension cannot be detected
without an IMU/contact sensor. The hardware controller therefore refuses to
start without fresh terrain metadata and a valid orientation, but the first
real run must still be supervised and restrained.

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
ros2 topic echo /kilin/stair_terrain --once
ros2 topic echo /kilin/balance_state --once
```

For stair use, `header.frame_id` must be `base_link_gravity_aligned` and
`orientation_valid` must be `true`.
