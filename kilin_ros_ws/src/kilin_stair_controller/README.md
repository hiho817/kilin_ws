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

## Offline gait generator

`generate_stair_csv` generates a controller-compatible CSV from the validated
`alex_v2.csv` phase sequence. Stair rise sets the FR clearance angle using the
rigid geometry from `kilin_stairs/cars.py`, retaining the extra clearance already
validated at a 0.10 m rise. It also adds a calibrated hub approach immediately
before the second FR lift when the rise exceeds 0.10 m. The calibration is
2.8 mm of extra travel per 1 mm of added rise: the 0.12/0.35 rosbag measured a
56 mm wheel-center setback relative to the successful 0.10/0.35 baseline.
For rear-leg support, the same taller-stair calibration adds 1 degree to the
rear landing angle per additional centimeter of rise. At 0.12 m this changes
the first RR/RL stair landing from 320 to 322 degrees, moving the planted rear
wheel rearward before the opposite rear leg is unloaded. This addresses the
phase-3 full-extension margin measured at 13.86 mm versus the required 15 mm.
The 0.12/0.35 middle cycle also applies a `-50` hub command during the two-second
FL transfer. Its approximately 55 mm ideal retreat was validated in log04 and
prevents FL from catching the third riser while landing on the second tread.
Stair run scales hub-only travel, and the initial
robot-center distance determines the first approach duration. With the validated
0.10 m rise, 0.35 m run, and 0.63 m center distance, the generated CSV remains
identical to `alex_v2.csv`. This start pose puts the leading edge of the outward
40-degree front wheels about 0.1736 m before the first riser.

Hub duration uses ideal wheel travel. `--drive-scale` is the measured ratio of
actual to ideal travel and defaults to 1.0; for example, use 0.9 if a commanded
100 mm produces 90 mm of real travel. The open-loop output should still be
checked before using dimensions other than the validated baseline. Inputs that
place the initial front wheel inside the riser or require a lift beyond the
calibrated leg reach are rejected.

All continuous hip targets are constructed from three independent angles:

- `outward`: the robot's fixed 40-degree initial support configuration;
- `lift`: the conservative 90-degree alex_v2 swing, increased only when the
  wheel rim would otherwise fail to clear the tread by 10 mm;
- `transition`: `atan2(rise, run)` plus the transition clearance calibrated by
  the successful 10/35 gait.

The generator then composes landing, preload, centered, and next-support angles
from those values and adds full 360-degree turns without wrapping. Consequently
values such as 320, 340, 370, 380, 400, and the stage-3 angles are generated
expressions rather than independent copied constants. At the validated 0.10 m
rise and 0.35 m run, the independent values are 40/90/20 degrees and reconstruct
the complete `alex_v2.csv` exactly.

Rise-dependent entry, rear-landing, and middle-retreat corrections are measured
calibrations from the successful 0.12/0.35 gait. They are capped at their 0.12 m
values instead of being extrapolated into untested taller-stair commands.

Stage 3 always uses the validated flat-top continuous targets, independent of
stair rise and run. In the one-cycle three-step gait the split-front sequence is
1040/320, 1090/370, then 1140/320 degrees, and the final level configuration is
exactly 1080/360/1080/1080 degrees.

After building and sourcing the workspace:

```bash
ros2 run kilin_stair_controller generate_stair_csv \
  --rise 0.10 \
  --run 0.35 \
  --center-to-first-riser 0.63 \
  --middle-cycles 1 \
  --output ~/kilin_ws/csv/generated/stair_35_10_cycle1.csv
```

The generated one-cycle gait is identical to `alex_v2.csv` from 0 through 52 s.
Additional middle cycles repeat the validated 35--52 s motion, add 20 s per
cycle, and add 360 degrees to every continuous hip target. Hip angles are never
normalized modulo 360 degrees; in particular, the FR 10-to-320-degree swing
remains a continuous +310-degree motion.

The final row is phase 0 at the canonical middle-cycle end pose. The controller
holds that last command after playback finishes. To append the final-approach
bridge and validated stage-3 gait, add `--include-stage3`:

```bash
ros2 run kilin_stair_controller generate_stair_csv \
  --rise 0.10 \
  --run 0.35 \
  --center-to-first-riser 0.63 \
  --middle-cycles 1 \
  --include-stage3 \
  --output ~/kilin_ws/csv/generated/stair_35_10_full.csv
```

With one middle cycle, this output is identical to the complete `alex_v2.csv`
through t=75 s. Each extra middle cycle shifts stage 3 by 20 s and adds 360
degrees to every stage-3 continuous hip target.

The output command refuses to overwrite an existing CSV unless `--force` is
given. Run it with the existing controller using, for example:

```bash
ros2 launch kilin_stair_controller launch.py \
  use_sim_time:=true \
  csv_name:=generated/stair_35_10_cycle1.csv \
  arm_control_mode:=com_closed_loop
```

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
