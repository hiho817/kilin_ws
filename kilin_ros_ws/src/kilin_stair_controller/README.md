# kilin_stair_controller

Coordinates Kilin CSV gait playback with four leg-specific Kinova joint poses for
stair climbing and center-of-mass adjustment. Kinova commands use the
`kinova_ptp_interfaces/action/JointPtp` action.

Two arm strategies are available:

- `fixed_phase` preserves the original behavior and reads one fixed pose for each
  phase from YAML.
- `com_closed_loop` treats phases 1--4 only as the identity of the future swing
  wheel. It reads geometric COM/wheel positions, computes the future three-wheel
  support triangle, and incrementally moves Kinova until the COM projection is in
  the inset safe region.

The arm normally holds `standard_pose_deg`. Before a swing phase, Kilin gait
playback pauses while Kinova first rotates joint 1 in the retracted standard
shape, then extends to that leg's compensation pose. When the CSV returns to
phase 0, the order is reversed: Kinova retracts while keeping the current joint-1
direction, then rotates joint 1 home. CSV playback resumes only after every
waypoint succeeds.

## CSV format

Existing 13-column and 17-column Kilin gait CSV files remain supported. To coordinate
the arm, append `arm_phase` as the final column:

- `0`: use `standard_pose_deg`
- `1`: front-left leg moving; use `front_left_pose_deg`
- `2`: front-right leg moving; use `front_right_pose_deg`
- `3`: rear-left leg moving; use `rear_left_pose_deg`
- `4`: rear-right leg moving; use `rear_right_pose_deg`

The phase name identifies the leg that is moving, not the direction in which the
arm or center of mass moves. This four-phase format replaces the previous
front/rear-only phase format.

An `arm_phase` value on a target row describes the arm pose required before any
motion toward that row begins. Whenever the next row's phase differs from the
active phase, including a transition back to phase 0, the controller pauses the
CSV clock and republishes the complete last Kilin command unchanged. Kinova moves
to the new pose, and only after `/kinova_joint_ptp` returns a successful result
may hip, steering, and wheel commands advance from the current row toward the target row.
Repeating the same phase across consecutive rows does not issue another arm
command.

## Geometry COM closed loop

Select the integrated controller at launch:

```bash
ros2 launch kilin_stair_controller launch.py \
  use_sim_time:=true \
  csv_name:=onestep/alex_one.csv \
  arm_control_mode:=com_closed_loop
```

Before launching, run the Isaac Sim JointTrajectory adapter and
`run_balance_state_publisher.py`, then press Play. The balance publisher supplies
the COM and the nominal wheel-bottom geometry on `/kilin/balance_state`; contact
sensor booleans are not used. `kilin_balance_monitor` is not required in this
mode because the same geometry calculation is performed inside this controller.

For each nonzero phase, playback holds the complete last Kilin command. The arm
starts from its standard shape, rotates joint 1 toward the shortest correction,
then interpolates joints 2--7 from `standard_pose_deg` toward
`full_extension_pose_deg` in `com_alpha_step` increments. After every PTP result,
the latest support geometry is evaluated again. Playback resumes only after the
margin is at least `com_safe_margin_m` for `com_safe_hold_sec` continuously. If
full extension is still unsafe, missing/stale geometry exceeds the timeout, or a
PTP goal fails, playback stops.

The input positions and COM are currently expected in the world frame.
`amr_yaw_in_world_deg` rotates the world correction into the AMR frame before J1
is selected. Keep it at zero while AMR +X and world +X are aligned; update it if
the robot begins with a nonzero heading.

For example, if the row at 2 seconds has phase 0 and the row at 5 seconds changes
the hips and has phase 2, Kinova reaches the phase-2 pose at 2 seconds before the
2-to-5-second command interpolation begins. Likewise, if a row at 15 seconds has
phase 1 and the next row at 16 seconds has phase 0 and wheel velocity 100, the
entire 15-second Kilin command is held while Kinova returns to standard; wheel
velocity 100 is released only after the arm reaches phase 0. A final phase-0 row
is pre-positioned at the start of its incoming segment and is therefore complete
before the controller shuts down.

Pose values in the YAML are degrees in `joint_1` through `joint_7` order. The
controller converts them to radians before sending a `JointPtp` goal; ROS and
Isaac Sim joint positions therefore remain in radians.

## Topics

- Publishes `/kilin/motor_cmd_raw` (`kilin_msgs/msg/MotorCmdStamped`)
- Publishes `/kilin/trigger` (`kilin_msgs/msg/TriggerStamped`)
- Subscribes `/kilin/balance_state` (`kilin_msgs/msg/BalanceStateStamped`)
- Publishes `/kilin/stability_state` (`kilin_msgs/msg/StabilityStateStamped`)
- Uses `/kinova_joint_ptp` (`kinova_ptp_interfaces/action/JointPtp`) as an action client

The controller pauses the CSV clock while a Kinova waypoint sequence is pending.
A rejected, aborted, canceled, failed, or timed-out goal stops stair playback.
The `arm_ptp_duration_sec` parameter controls each waypoint's motion duration;
`arm_timeout_sec` is the watchdog for each individual waypoint and must be larger.

## Launch

The launch file starts both `kinova_joint_ptp_server` and
`kilin_stair_controller` by default. The Isaac Sim FollowJointTrajectory
adapter must still be started inside Isaac Sim. When `start_ptp:=true`, the
launch file starts JointPtp first and waits 2 seconds of wall-clock time before
starting the stair controller. This warm-up lets JointPtp receive the initial
`/joint_states` samples before phase 0 is requested and avoids rejecting the
first goal during startup.

Simulation:

```bash
ros2 launch kilin_stair_controller launch.py \
  use_sim_time:=true \
  csv_name:=stairs.csv \
  config_name:=stair_controller.yaml
```

The warm-up duration can be changed if the simulation or robot interface starts
more slowly:

```bash
ros2 launch kilin_stair_controller launch.py \
  use_sim_time:=true \
  csv_name:=stairs.csv \
  ptp_warmup_sec:=3.0
```

Real robot:

```bash
ros2 launch kilin_stair_controller launch.py use_sim_time:=false csv_name:=stairs.csv
```

If a JointPtp server is already running, prevent a duplicate action server with:

```bash
ros2 launch kilin_stair_controller launch.py start_ptp:=false
```

With `start_ptp:=false`, the stair controller starts immediately because the
external JointPtp server is expected to be ready already. The warm-up does not
replace the requirement to start the Isaac trajectory adapter and press Play;
`/joint_states` must be publishing before the first arm goal.

Use `ptp_config:=/absolute/path/to/joint_ptp.yaml` to select another JointPtp
server configuration.

During pose tuning, place parameter files under `~/kilin_ws/config` and select
one with `config_name`. YAML changes then require only a node restart, not a
package rebuild. Use `config_dir` only when loading a file from another folder.
The package config remains the version-controlled reference; after pose values
are finalized, copy the accepted values back to it.

When `use_sim_time` is `false`, module A and B hip commands are inverted to match
the physical motor mounting direction; module C and D are unchanged. Simulation
does not apply this inversion. Set `invert_ab_hips_on_hardware:=false` in the
configuration only when testing hardware with an already corrected CSV.

Only one node that publishes `/kilin/motor_cmd_raw` should run at a time.
