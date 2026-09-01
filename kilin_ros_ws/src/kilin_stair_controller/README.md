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
`full_extension_pose_deg` in `com_alpha_step` increments. When
`com_use_inverse_initial_alpha` is enabled, the controller first rotates joint 1
at alpha zero, recomputes the measured COM, and uses the fitted URDF/FK Delta-COM
model to command a model-based initial alpha. If the measured margin is still
insufficient, it continues with the existing `com_alpha_step` increments.
When `com_inverse_target_relative_to_release` is true, the feedforward target is
the active phase release margin minus `com_inverse_target_margin_offset_m`.
Hardware inverse experiments use a 1 mm offset, producing FL/FR/RL/RR targets
of 24/22/14/24 mm. The older absolute `com_inverse_target_margin_m` remains
available when relative mode is false. `com_inverse_min_refinement_steps`
requires a minimum number of measured fine-tune steps before release. Full
extension is exempt from the mandatory-step count, so a phase such as RL can
still release when it satisfies its configured safe margin even if no further
alpha step is possible. After every PTP result, the latest support geometry is
evaluated again. Playback resumes only after the
margin is at least the current entry in `com_safe_margin_by_phase_m` (FL, FR,
RL, RR), or the backward-compatible `com_safe_margin_m` fallback when the array
is empty, for `com_safe_hold_sec` continuously. If
full extension is still unsafe, missing/stale geometry exceeds the timeout, or a
PTP goal fails, playback stops.

`com_min_alpha_by_phase` optionally sets a minimum extension fraction for FL,
FR, RL, and RR. Once COM correction starts, playback resumes only when both the
phase's minimum alpha and its safe margin are satisfied.

In simulation, input positions and COM are expected in the world frame. The
Isaac balance-state publisher supplies the simulated base-to-world orientation,
which is used as the IMU-equivalent attitude for both J1 direction selection and
the model-based Delta-COM projection. `amr_yaw_in_world_deg` is only the fallback
when that orientation is unavailable; keep it at zero while AMR +X and world +X
are aligned.

`arm_base_yaw_offset_deg` then rotates the AMR-frame correction into the Kinova
base mounting frame. Use `0.0` when Kinova base +X points toward AMR +X, as in
the current simulation. Use `180.0` on hardware when the Kinova base is mounted
backward, so an AMR-forward correction commands J1 to 180 degrees. This mounting
offset is independent of `amr_yaw_in_world_deg`; do not use the AMR heading
parameter to compensate for the arm installation.

On no-IMU hardware, generate the CSV with `--with-terrain-metadata`. The added
columns contain stair rise, tread indices, and the current three/four-wheel
support mask. `kilin_com_estimator` uses these values to publish gravity-aligned
COM geometry and a base-to-output quaternion. When that quaternion is valid,
the controller transforms the horizontal correction back into the physical
base/arm XY plane before selecting Kinova J1. Hardware configuration sets
`require_balance_orientation: true`; an assumed-level balance state is rejected.

Hardware generation also applies the CAD-derived same-side leg clearance limit
and separates the final left-leg normalization. The FL level target is 1080
degrees; its Stage-3 inward support pose is limited to 40 degrees, giving a
1120-degree command and five degrees of angular reserve before the observed
approximately 45-degree FL/RL interference pose. FL stays at 1120 degrees while
RL completes its motion, then FL returns to 1080 degrees in a separate
two-second segment. Arm phase 3 remains active until RL reaches 1080, so the
Kinova correction is not retracted during this final RL motion. Simulation CSVs
without `--with-terrain-metadata` retain the validated original Stage 3.

Hardware configuration for the backward-mounted base:

```yaml
amr_yaw_in_world_deg: 0.0
arm_base_yaw_offset_deg: 180.0
standard_pose_deg: [180.0, -85.94, 0.0, 147.0, 0.0, 22.92, 0.0]
full_extension_pose_deg: [180.0, 90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

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
  --with-terrain-metadata \
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
- Publishes `/kilin/stair_terrain` (`kilin_msgs/msg/StairTerrainStamped`)
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

Real robot uses the integrated launch, which starts the COM estimator, Kinova
PTP server, and gated stair controller. The hardware controller publishes to
`/kilin/motor_cmd_raw`; put `kilin_panel` in Manual mode so it forwards those
commands to `/motor/command`. Do not run another raw-command publisher at the
same time.

```bash
ros2 launch kilin_stair_controller hardware_stair.launch.py \
  csv_path:=$HOME/kilin_ws/csv/stairs_v3/stair_35_10_hardware.csv
```

The inverse hardware experiment has a separate launch and configuration, so the
incremental hardware baseline remains available unchanged:

```bash
ros2 launch kilin_stair_controller hardware_inverse_stair.launch.py \
  csv_path:=$HOME/kilin_ws/csv/stairs_v3/stair_35_10_hardware.csv
```

At startup, confirm that the controller reports inverse initial alpha enabled
and inverse target margins `[24.0, 22.0, 14.0, 24.0]` mm before calling the
start service.

Launching does not move the robot. Verify fresh inputs and gravity-aligned
output, then explicitly start:

```bash
ros2 topic hz /motor/state
ros2 topic hz /joint_states
ros2 topic echo /kilin/stair_terrain --once
ros2 topic echo /kilin/balance_state --once
ros2 service call /kilin/start_stair std_srvs/srv/Trigger "{}"
```

Record the hardware inverse experiment without `/imu`:

```bash
ros2 bag record -o inverse_refine_hw_35_10_log01 \
  /kilin/trigger /kilin/stair_phase /kilin/stability_state \
  /kilin/balance_state /kilin/stair_terrain \
  /motor/state /joint_states /kilin/motor_cmd_raw
```

The start service rejects a missing `/kilin/motor_cmd_raw` subscriber (the
panel forwarder), unavailable
Kinova PTP, stale balance data, and invalid base orientation. Stop with
`ros2 service call /kilin/stop_stair std_srvs/srv/Trigger "{}"` or Ctrl-C; the
controller publishes zero hub velocity before shutdown.

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

The current hardware was verified with all four physical hips at zero and no
A/B sign mismatch. Its configuration therefore sets
`invert_ab_hips_on_hardware: false`. The parameter remains available only for
older hardware wiring that still requires command-side A/B inversion.

Only one node should publish the selected motor command topic at a time.
