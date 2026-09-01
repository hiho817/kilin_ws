# Kilin known-terrain controller

This ROS 2 node runs the migrated Version 2 receding-horizon planner online
against two parameterized, already-known one-sided ramps on opposite wheel
tracks. It does not replay CSV
commands and does not use live terrain-map feedback.

`kilin_motion_planner` is an explicit ROS workspace dependency. After pulling
changes, build both packages before launching; no planner checkout under a
user-specific `Documents` path is required.

The planner requires the Ubuntu ROS packages `python3-numpy` and
`python3-scipy`. Their versions must be compatible; a user-installed NumPy 2.x
can shadow Ubuntu's older NumPy and break the system SciPy package.

```bash
colcon build --packages-select kilin_motion_planner kilin_known_terrain_controller
```

For `target:=isaac`, the node publishes complete A/B/C/D motor commands to
`/kilin/motor_cmd_raw`; `isaac_bridge` maps them to the Isaac Sim action graph.
For `target:=real`, it publishes directly to `/motor/command` and reads
`/motor/state`; it never starts Isaac bridge. Its wheel field uses the existing
RPM-times-ten convention.

The generic launch is disarmed, selects stationary `hip_test`, and defaults to
the safe real target (no Isaac process is started):

```bash
ros2 launch kilin_known_terrain_controller one_sided_ramp_control.launch.py
```

For Isaac simulation, select its target explicitly:

```bash
ros2 launch kilin_known_terrain_controller one_sided_ramp_control.launch.py \
  target:=isaac
```

To run the bounded hip-only test, explicitly enable publication:

```bash
ros2 launch kilin_known_terrain_controller one_sided_ramp_control.launch.py \
  target:=isaac armed:=true
```

The test waits for fresh named FL/FR/RL/RR hip feedback, starts at the measured
pose, moves the FL/FR/RL/RR hips by `[+5, +5, -5, -5]` degrees at 8
degrees/second, keeps all wheel velocities and steering positions zero, then
reports completion after a two-second hold. If feedback becomes stale,
publication stops.

The online Version 2 ramp controller cannot be selected accidentally; it needs
the explicit launch argument `mode:=known_ramp`.

For sign verification, `mode:=hip_calibration` moves one joint at a time:
FL and FR by -3 degrees, then RL and RR by +3 degrees. Each joint should move
its wheel module outward, hold for one second, return to its measured starting
pose, and only then advance to the next joint. Hub motors use REST mode (0) and
zero velocity; steering stays at zero in position mode. The current Isaac
converter ignores motor modes, so simulation still receives zero wheel speed,
while the real robot bridge preserves REST mode.

`mode:=wheel_calibration` holds the measured hip pose and zero steering, keeps
the hubs in REST for one second, commands all four wheels at 0.5 rad/s for one
second, and then automatically returns every hub to REST. It is intended only
to verify the forward sign before enabling `known_ramp`.

The `known_ramp` launch defaults to 0.18 m/s for up to twelve seconds. The
first ramp begins at x=0.75 m and the second at x=2.70 m (a 1 m gap); both are 80 mm high,
with 0.30 m rise, 0.35 m deck, and 0.30 m descent. With
`known_ramp.auto_initialize_stance` enabled (the default), this
single mode first moves from the measured pose into the planner nominal
`[-45, -45, 45, 45]` degree stance using the same wheel-coordinated IK motion,
holds in REST, and then begins the configured one-second startup delay and ramp motion.
Planning and publication require fresh named hip feedback, and hubs use REST
mode before and after active motion. Planner hip angles are sent directly to Isaac, with applied hip
motion limited to the measured actuator limit of 0.4 revolutions/s
(144 degrees/s), matching the Version 2 planner constraint.

If the robot is already in that exact nominal stance, skip the transition with
`auto_initialize_stance:=false` in the launch command. Fresh hip feedback is
still required; this option only skips the commanded repositioning step.

`mode:=planner_posture_test` uses that same planner and direct hip mapping at a
stationary flat-ground position for four seconds. All hub motors remain in REST
mode for the entire test; no forward wheel command is generated.

The shared ROS/web model uses a measured 58.5 mm wheel radius, a 14-step,
0.15-second preview horizon (1.95 seconds from first to last knot), and
collision bounds measured from the supplied base-link mesh. The ROS node logs
every effective motion parameter and enforces a
12-second hard motion limit. Base position remains a speed-times-simulation-
time estimate because the current Isaac graph publishes no odometry/base pose.

Planner results are actuator targets rather than step commands. The 50 Hz ROS
publisher interpolates hips toward each target using actual Isaac simulation
time and the measured 144 degree/s actuator limit. This keeps actuation smooth
even though the current SLSQP planner completes at roughly 3 Hz.

`mode:=stance_initialization` remains available as a standalone diagnostic and
moves from the vertical zero-degree pose to the planner nominal
`[-45, -45, 45, 45]` degree stance at 15 degrees/s. Differential
kinematics commands the front wheels forward and rear wheels backward so the
wheel contact points follow the spreading legs without translating the body.
At the target, all hubs enter REST mode and the pose is held.

Only `target:=isaac` starts `isaac_bridge` with `start_cmd_converter:=false`,
so there is only one publisher of `/kilin/motor_cmd_raw`. `target:=real` starts
neither Isaac bridge nor an Isaac simulation clock; use
`real_kilin_known_ramp.launch.py` for the documented real-robot ramp procedure.

For Vicon validation, `real_kilin_known_ramp.launch.py` accepts
`vicon_trigger:=true`. It drives the existing active-low GPIO trigger on
`/dev/gpiochip0`, line 112, only while the timed known-ramp motion is active.
It returns the line inactive on completion, stale feedback, Ctrl-C, or normal
node shutdown. Leave it disabled for simulation and for runs without Vicon.

## Real Kilin: analytical map with corrected FAST-LIO2 odometry

FAST-LIO keeps its raw `/Odometry` interface. `kilin_fastlio_bringup` also
publishes `/kilin/fastlio/odometry`, transformed through
`map -> camera_init -> body -> base_link -> hip_axis_center`. The known-ramp
controller must use only this corrected topic. At the first active planning
cycle it rebases the corrected pose to `initial_x_m`, `initial_y_m`, and
`initial_yaw_rad`, so the surveyed ramp still begins at the analytical
profile's configured `ramp.start_x_m`.

Start FAST-LIO first:

```bash
ros2 launch kilin_fastlio_bringup mid360s_fastlio.launch.py
```

Verify the corrected topic before arming anything:

```bash
ros2 topic hz /kilin/fastlio/odometry
ros2 topic echo /kilin/fastlio/odometry --once
```

The message must report `frame_id: map`, `child_frame_id: hip_axis_center`, and
approximately 10 Hz. The 2026-08-26 experiment repeats the same analytical
150 mm profile twice, as `ramp_test01` and `ramp_test02`. Both runs use corrected
odometry progress, no live terrain window, 0.1 m/s commanded speed, 30.0 s run
duration, and a 35.0 s hard limit.

Record the complete ROS graph in a separate terminal. This intentionally keeps
the large accumulated map and all point-cloud/debug topics for post-run frame,
mapping, and planner analysis.

Prepare both trial directories once:

```bash
mkdir -p ~/kilin_ws/logs/2026-08-26/ramp_test01
mkdir -p ~/kilin_ws/logs/2026-08-26/ramp_test02
```

For the first run, record into its dedicated bag directory:

```bash
ros2 bag record -a \
  -o ~/kilin_ws/logs/2026-08-26/ramp_test01/bag
```

For the second run, use:

```bash
ros2 bag record -a \
  -o ~/kilin_ws/logs/2026-08-26/ramp_test02/bag
```

Start recording only after FAST-LIO and `/kilin/fastlio/odometry` are healthy,
but before launching the controller. Save the corresponding Vicon exports as
`ramp_test01/vicon_ramp_test01.csv` and `ramp_test02/vicon_ramp_test02.csv`.
Do not rename the generated `bag` directory, its `.db3` file, or
`metadata.yaml` after recording.

Then launch the experiment:

```bash
PYTHONNOUSERSITE=1 ros2 launch kilin_known_terrain_controller \
  real_kilin_known_ramp.launch.py \
  armed:=true mode:=known_ramp \
  terrain_profile:=terrain_150mm_20deg_single_600mm.yaml \
  use_odometry:=true odometry_relative_origin:=true \
  use_speed_command:=false speed_m_s:=0.1 \
  run_duration_s:=30.0 hard_motion_limit_s:=35.0 \
  vicon_trigger:=true debug_publish:=true
```

Use this identical controller command for both runs. Stop the bag recorder
cleanly with Ctrl-C after each controller run, confirm `metadata.yaml` exists,
reset the robot and physical starting pose, then start the second bag recorder.

`use_terrain_window` remains hard-disabled in the real analytical-map launch.
If corrected odometry is absent, is in the wrong frame, or becomes older than
`odometry_timeout_s`, the controller holds the current hips and commands all
wheel hubs to REST. It does not fall back to time-integrated position.

## Live terrain-window startup policy

For a live FAST-LIO terrain window, start Kilin on a verified flat approach
area.  The MID360s front ROI cannot observe the ground immediately underneath
the chassis, so the controller infers one fixed flat support patch from the
observed strip 0.20--0.80 m ahead.  It then fills only the initial support
envelope from 0.65 m behind to 0.85 m ahead of that starting pose.  This covers
the current five-knot preview and the measured near-field LiDAR blind strip;
it is not translated with the robot and is never used to fill later unknown
terrain.

The reference strip must contain one dominant flat surface: at least 80% of its
observed nodes must fit within a 50 mm height span.  This tolerates an isolated
misregistered return but does not accept a ramp covering a material part of the
strip.  If the strip is missing or non-flat, the controller deliberately retains the
`terrain_unavailable` stop.  Place the robot so that the first ramp edge is
more than 0.85 m ahead at startup.  The separate isolated-hole repair only
fills one grid node when all four cardinal neighbours agree within 80 mm; it
does not bridge a multi-cell gap, a window boundary, or a step.

For a first check, use `armed:=false` and confirm the log reports
`seeded initial flat support patch` without a planner fallback before repeating
the same launch with `armed:=true`.

To verify the LED and Vicon capture path before a run, use the separate safe,
GPIO-only self-test. It does not arm the controller or publish a motor command:

```zsh
PYTHONNOUSERSITE=1 ros2 launch kilin_known_terrain_controller real_kilin_known_ramp.launch.py \
  armed:=false vicon_trigger:=true vicon_trigger_test:=true
```

The LED is active for three seconds by default and then returns dark. Override
the pulse length with `vicon_trigger_test_duration_s:=<seconds>`; Ctrl-C also
returns the line inactive immediately. Keep `PYTHONNOUSERSITE=1`: the
controller adds the required local gpiod 2.x binding only when it opens the
physical trigger, so unrelated user Python packages cannot affect `ros2`.

Set `debug_publish:=true` when launching either controller launch file to
publish the lightweight `/kilin/planner/debug/horizon` (`nav_msgs/Path`) and
`/kilin/planner/debug/footprints` (`visualization_msgs/MarkerArray`) topics.
They contain only the planned horizon and body rectangles; they do not publish
or render the terrain grid, FAST-LIO cloud, or RViz itself.

For `target:=isaac`, the collected Isaac graph intentionally publishes feedback
below the `/kilin/isaac` prefix. The legacy Isaac bridge converts its JointState
into the same `/motor/state` contract used by the real robot. This launch
remaps the controller's simulation clock and the bridge's IMU/JointState
subscriptions to:

- `/kilin/isaac/clock`
- `/kilin/isaac/imu`
- `/kilin/isaac/joint_states` (bridge input)
- `/motor/state` (published by `isaac_bridge` from Isaac JointState)
