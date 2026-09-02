# Kilin known-terrain controller

This ROS 2 node runs the migrated Version 2 receding-horizon planner online
against parameterized analytical ramps or, when `use_terrain_window:=true`, a
live local elevation window. It does not replay CSV commands. Unknown live
terrain remains a safety stop rather than an analytical-map fallback.

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

## Reproducible real-robot trial runner

For a real-Kilin experiment, prefer `run_real_terrain_trial` over starting
FAST-LIO2, recording, the mapper, and the controller in separate terminals.
It reads `trial_config.yaml` from one trial directory, so the copied terrain,
FAST-LIO2, hip-PID, mapper, controller, and recording settings stay next to the
bag. It starts FAST-LIO2 first, then the optional terrain mapper, rosbag, and
finally the controller. Every child process has a retained console file under
`console/<UTC timestamp>/`; that same stdout/stderr is also shown live in the
invoking terminal.

Before it starts the mapper, recorder, or controller, the runner requires one
message on `fastlio.required_odometry_topic` (default:
`/kilin/fastlio/odometry`). If that does not arrive within the configured
`fastlio.required_odometry_wait_s`, it exits without arming and points to both
`fastlio.log` and `fastlio_readiness.log` in the trial directory.

When `controller.use_terrain_window: true`, the runner also requires one
message on `terrain_mapper.required_terrain_topic` (default:
`/kilin/terrain/local_window`) before it starts recording or the controller.
This applies whether the runner starts the mapper itself or the mapper is an
already-running external process. A missing required topic is therefore a
startup refusal, not a condition that can reach a moving controller.

`real_kilin_trial_stack.launch.py` reads the same manifest and provides an
interactive selectable stack: FAST-LIO2 → optional mapper → optional rosbag →
controller. Every component's `startup_wait_s` is its absolute offset from the
main command, not a delay relative to the preceding component. The supplied
template schedules FAST-LIO2 at 0 s, mapper at 4 s, recorder at 6 s, and
controller at 8 s. Use the runner for an armed traversal: it additionally
waits for actual required-topic messages before starting the controller.

```bash
PYTHONNOUSERSITE=1 ros2 run kilin_known_terrain_controller run_real_terrain_trial \
  --trial-dir /home/biorola/kilin_ws/logs/2026-09-02/analytical_map_test05 \
  --allow-armed
```

The runner refuses to arm unless both `controller.armed: true` in the local
YAML and `--allow-armed` are present. It also refuses to overwrite an existing
bag directory. Set `recording.mode: all` for complete ROS graph capture, or
set `recording.mode: selected` and list only absolute ROS topics under
`recording.topics`. The 2026-09-02 campaign README contains the copyable YAML
template and exact operating instructions.

### Trial manifest example

This abbreviated `trial_config.yaml` shows the stack and logging controls. The
full copyable example, including the selected-topic list, is in the campaign
template.

```yaml
console_logging:
  enabled: true
  directory: console          # creates console/<UTC timestamp>/ in this trial

fastlio:
  enabled: true
  config: fastlio_config.yaml
  startup_wait_s: 0.0
  required_odometry_topic: /kilin/fastlio/odometry
  required_odometry_wait_s: 8.0

terrain_mapper:
  enabled: true
  profile: terrain_mapper_profile.yaml
  startup_wait_s: 4.0
  required_terrain_topic: /kilin/terrain/local_window
  required_terrain_wait_s: 5.0

recording:
  enabled: true
  mode: all                   # or selected with an explicit topics list
  output_directory: bag
  startup_wait_s: 6.0

controller:
  startup_wait_s: 8.0
  armed: true
  mode: known_ramp
  terrain_profile: terrain_profile.yaml
  hip_pid_profile: hip_pid.yaml
  use_odometry: true
  odometry_relative_origin: true
  use_terrain_window: true
  terrain_window_timeout_s: 1.0
  use_speed_command: false
  speed_m_s: 0.10
  run_duration_s: 30.0
  hard_motion_limit_s: 35.0
  vicon_trigger: true
  debug_publish: true
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

If the robot is already in a manually verified physical nominal stance, skip
the transition with `known_ramp_auto_initialize_stance:=false`. Fresh hip
feedback is still required; this option only skips the commanded repositioning
step. The subsequent motor-side acceptance threshold is
`known_ramp_max_initial_hip_error_deg` (default 5.0 degrees). This is useful
when the physical 45-degree posture has a known `position_diff`.

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

For every planner mode, the controller has a second independent input gate:
when `use_odometry: true`, it publishes no startup/stance motion until fresh
accepted odometry exists; when `use_terrain_window: true`, it likewise waits
for a fresh TerrainWindow. Loss of either stream after motion has begun causes
an `ODOMETRY SAFETY HOLD` or `TERRAIN SAFETY HOLD`, holding hips and setting
wheel hubs to REST. The default terrain timeout is 1.0 s, intentionally larger
than the normal 5 Hz terrain-window interval.

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

## Complete parameter reference

This section is the authoritative parameter contract for this package.  The
**Default** column is the node's declared default unless an entry says
otherwise. `one_sided_ramp.yaml` and a launch argument can override it.  The
real-robot launch overlays its own defaults where noted below.  **Set for an
experiment?** means whether an operator normally needs to state a non-default
value in the command or per-trial YAML; it is not permission to bypass a safety
check.

For reproducible real trials, pass absolute per-trial paths for
`terrain_profile` and `hip_pid_profile`.  The latter is deliberately PID-only:
it may contain `hip_kp`, `hip_ki`, and `hip_kd`, but not characterization
strategy, torque feed-forward, or angle-difference compensation.

### Launch arguments

Both `real_kilin_known_ramp.launch.py` and `one_sided_ramp_control.launch.py`
accept package-relative YAML names or absolute YAML paths for the two profile
arguments.  The generic launch also has the two arguments marked `generic`.

| Argument | Default (real / generic) | Possible values | Description | Set for an experiment? |
| --- | --- | --- | --- | --- |
| `armed` | `false` / `false` | `true`, `false` | Enables motor command publication. | **Yes**: keep `false` for preflight, explicitly use `true` only when ready. |
| `mode` | `known_ramp` / `hip_test` | `disabled`, `hip_test`, `hip_calibration`, `wheel_calibration`, `stance_initialization`, `planner_posture_test`, `known_ramp` | Selects controller behaviour. | **Yes**. |
| `terrain_profile` | package `terrain_150mm_20deg_single.yaml` / `terrain_80mm_two_ramps.yaml` | package YAML name or absolute YAML path | Analytical ramp geometry overlay. In live mode it is retained for trial provenance but is not the terrain source. | **Yes**: absolute per-trial path for a recorded trial. |
| `hip_pid_profile` | package `one_sided_ramp.yaml` / `one_sided_ramp.yaml` | package YAML name or absolute PID-only YAML path | PID overlay loaded after base and terrain profiles. | **Yes**: absolute per-trial `hip_pid.yaml`. |
| `target` (generic) | — / `real` | `real`, `isaac` | Chooses real motor contract or Isaac bridge/time. | **Yes** for generic launch; use the real-specific launch on hardware. |
| `use_speed_command` | `false` / `true` | `true`, `false` | Uses the live `Float32` speed topic instead of fixed `speed_m_s`. | **Yes**. |
| `speed_m_s` | `0.05` / `0.18` | non-negative m/s, capped by `speed_command_max_m_s` only in topic mode | Fixed forward speed. | **Yes** for fixed-speed trials. |
| `run_duration_s` | `30.0` / `22.0` | positive seconds | Fixed-speed requested duration; ignored in live-speed mode. | **Yes** for fixed-speed trials. |
| `hard_motion_limit_s` | `35.0` / `22.0` | positive seconds | Upper timeout; effective fixed-speed duration is the lower of this and `run_duration_s`. | **Yes**. |
| `vicon_trigger` | `false` | `true`, `false` | Enables the physical active-low Vicon GPIO trigger during timed ramp motion. | Only with Vicon. |
| `vicon_trigger_test` | `false` | `true`, `false` | GPIO-only LED self-test; requires `armed:=false`. | Only for trigger preflight. |
| `vicon_trigger_test_duration_s` | `3.0` | positive seconds | Trigger self-test pulse duration. | No. |
| `debug_publish` | `false` | `true`, `false` | Publishes planner horizon and footprints. | Recommended for recorded trials. |
| `use_terrain_window` | `false` | `true`, `false` | Uses live `TerrainWindow`; unknown terrain is a stop, not analytical fallback. | **Yes**: `false` analytical, `true` live. |
| `use_odometry` | `false` | `true`, `false` | Uses corrected odometry for progress; stale odometry stops motion. | **Yes** on real terrain runs. |
| `odometry_relative_origin` | `true` | `true`, `false` | Rebase corrected odometry at planner start for an analytical map. | **Yes**: current live trials use `false`; choose deliberately. |
| `auto_initialize_stance` (generic) | — / `true` | `true`, `false` | Automatically move to nominal 45-degree stance before `known_ramp`. | No unless already verified at nominal stance. |

### ROS node: transport, feedback, and safety

| Parameter | Default | Possible values | Description | Set for an experiment? |
| --- | --- | --- | --- | --- |
| `use_sim_time` | ROS standard `false`; base YAML `true`; real launch forces `false` | `true`, `false` | ROS clock source. | No; real must remain `false`. |
| `mode` | `disabled` | launch-mode list above | Node mode if no launch overlay is used. | Use launch argument. |
| `armed` | `false` | `true`, `false` | Motor publication interlock. | Use launch argument. |
| `command_topic` | `/kilin/motor_cmd_raw` | ROS topic | `MotorCmdStamped` output. Real launch forces `/motor/command`. | No. |
| `feedback_source` | `motor_state` | `motor_state`, `joint_state` | Selects canonical real feedback or legacy joint state. | No for real Kilin. |
| `motor_state_topic` | `/motor/state` | ROS topic | `MotorStateStamped` feedback. | No for real Kilin. |
| `joint_state_topic` | `/kilin_joint_states` | ROS topic | Legacy `JointState` feedback if selected. | Isaac/legacy only. |
| `feedback_timeout_s` | `0.5` s | positive seconds | Fresh-feedback interlock timeout. | No without safety review. |
| `position_mode` | `4` | motor-driver position-mode integer | Hip and steering motor mode. | No. |
| `velocity_mode` | `5` | motor-driver velocity-mode integer | Active hub motor mode. | No. |
| `rest_mode` | `0` | motor-driver rest-mode integer | Stopped hub motor mode. | No. |
| `debug_publish_enabled` | `false` | `true`, `false` | Enables horizon/footprint debug topics only. | Recommended for bags. |

### ROS node: odometry, live terrain, and speed input

| Parameter | Default | Possible values | Description | Set for an experiment? |
| --- | --- | --- | --- | --- |
| `use_odometry` | `false` | `true`, `false` | Planner position comes from odometry rather than integrated applied speed. | **Yes** on real runs. |
| `odometry_topic` | `/Odometry` | ROS topic | Real launch forces `/kilin/fastlio/odometry`. | No with real launch. |
| `odometry_timeout_s` | `0.5` s | positive seconds | Maximum odometry age before safe hold/REST. | No without safety review. |
| `odometry_required_frame` | empty | frame ID or empty | Rejects odometry from another parent frame. Real launch forces `map`. | No with real launch. |
| `odometry_relative_origin` | `false` | `true`, `false` | Captures an origin at planner start when enabled. | See launch table. |
| `use_terrain_window` | `false` | `true`, `false` | Replaces analytical terrain with latest live window. | **Yes**. |
| `terrain_window_topic` | `/kilin/terrain/local_window` | ROS topic | `TerrainWindow` input. | No. |
| `terrain_window_timeout_s` | `1.0` s | positive seconds | Maximum accepted TerrainWindow age before terrain safety hold. | No without safety review. |
| `angle_diff_compensation.gain` | `0.0` | non-negative scalar | Adds this fraction of each hip's greatest observed outward `position_diff` to outward motor targets. | Set explicitly only for a documented transmission experiment. |
| `angle_diff_compensation.maximum_abs_rad` | `0.10` rad | positive radians | Hard per-hip bound on the added compensation. | No without safety review. |
| `live_terrain.initial_flat_support.enabled` | `true` | `true`, `false` | Enables fixed initial support seed only. | No without safety review. |
| `live_terrain.initial_flat_support.rear_m` | `0.65` m | non-negative m | Rear extent of seeded support envelope. | No. |
| `live_terrain.initial_flat_support.forward_m` | `0.85` m | non-negative m | Forward extent of seeded support envelope. | No. |
| `live_terrain.initial_flat_support.half_width_m` | `0.50` m | positive m | Lateral half width of support envelope. | No. |
| `live_terrain.initial_flat_support.measurement_min_forward_m` | `0.20` m | non-negative m | Near edge of observed flat-strip test. | No. |
| `live_terrain.initial_flat_support.measurement_max_forward_m` | `0.80` m | greater than minimum | Far edge of observed flat-strip test. | No. |
| `live_terrain.initial_flat_support.maximum_height_span_m` | `0.05` m | positive m | Flat-cluster height tolerance. | No. |
| `live_terrain.initial_flat_support.minimum_inlier_fraction` | `0.80` | `(0, 1]` | Required dominant-flat fraction. | No. |
| `live_terrain.isolated_hole_fill.enabled` | `true` | `true`, `false` | Allows only one isolated, neighbour-consistent unknown node. | No. |
| `live_terrain.isolated_hole_fill.maximum_height_span_m` | `0.08` m | positive m | Cardinal-neighbour agreement tolerance. | No. |
| `use_speed_command` | `false` | `true`, `false` | Enables speed-topic mode. | **Yes**. |
| `speed_command_topic` | `/kilin/control/target_speed_m_s` | ROS topic | `Float32` speed request input. | Only topic mode. |
| `speed_command_max_m_s` | `0.30` m/s | non-negative m/s | Clamps live speed request. | No without review. |
| `speed_command_accel_limit_m_s2` | `0.15` m/s²; base YAML `0.0` | non-negative; `0` is direct steps | Rate limit for live speed request. | Only topic mode. |
| `speed_command_timeout_s` | `0.5` s | positive seconds | Missing speed topic decelerates/stops wheels. | Only topic mode. |

### ROS node: planner, analytical terrain, and motion limits

| Parameter | Default | Possible values | Description | Set for an experiment? |
| --- | --- | --- | --- | --- |
| `initial_x_m`, `initial_y_m`, `initial_yaw_rad` | `0`, `0`, `0` | finite m, m, rad | Integrated-position start when odometry is disabled. | Only open-loop/Isaac. |
| `speed_m_s` | `0.18` m/s | non-negative m/s | Fixed-speed planner input. | Use launch argument. |
| `startup_delay_s` | `1.0` s | non-negative seconds | Wait after planner readiness before timed motion. | No. |
| `run_duration_s`, `hard_motion_limit_s` | `22`, `22` s | positive seconds | Fixed-speed requested duration and hard timeout. | Use launch arguments. |
| `planning_rate_hz`, `publish_rate_hz` | `10`, `50` Hz | positive Hz | Planning timer and motor publication timer. | No without profiling. |
| `planner_dt_s` | `0.1` s | positive seconds | Optimizer knot time step. | No without planner validation. |
| `horizon_steps` | `5` | integer ≥ 2 | Number of horizon knots. | No without planner validation. |
| `horizon_knot_spacing_m` | `0.05` m | positive m | Distance between knots; independent of speed. | No without planner validation. |
| `planner_deadline_s` | `0.6` s | positive seconds | Rejects late solve and holds safely. | No without timing evidence. |
| `solver_max_iterations` | `400` | positive integer | SLSQP iteration cap. | No without timing evidence. |
| `map_resolution_m` | `0.025` m | positive m | Analytical grid resolution around horizon. | Analytical only. |
| `map_margin_x_m`, `map_half_width_m` | `0.8`, `0.7` m | positive m | Analytical terrain sampling bounds. | Analytical only. |
| `ramp.height_m` | `0.08` m | non-negative m | First ramp height. | **Yes** in terrain YAML. |
| `ramp.start_x_m` | `0.75` m | finite m | First up-ramp start in analytical map. | **Yes** in terrain YAML. |
| `ramp.up_ramp_length_m`, `ramp.deck_length_m`, `ramp.down_ramp_length_m` | `0.30`, `0.35`, `0.30` m | positive m | First ramp segment lengths. | **Yes** in terrain YAML. |
| `ramp.track_center_y_m`, `ramp.track_width_m` | `0.25`, `0.34` m | finite m, positive m | First ramp lateral centre and width. | **Yes** in terrain YAML. |
| `ramp.second.enabled` | `true` | `true`, `false` | Enables second analytical ramp. | **Yes** in terrain YAML. |
| `ramp.second.height_m`, `ramp.second.start_x_m` | `0.08`, `2.70` m | non-negative/finite m | Second-ramp height and start. | If enabled. |
| `ramp.second.up_ramp_length_m`, `ramp.second.deck_length_m`, `ramp.second.down_ramp_length_m` | `0.30`, `0.35`, `0.30` m | positive m | Second-ramp segment lengths. | If enabled. |
| `ramp.second.track_center_y_m` | `-0.25` m | finite m | Second-ramp lateral centre; width is shared from `ramp.track_width_m`. | If enabled. |
| `known_ramp.max_initial_hip_error_deg` | `5.0` deg | non-negative deg | Motor-side acceptance tolerance when planner mode starts; launch name `known_ramp_max_initial_hip_error_deg`. | Set explicitly for a documented manually initialized stance. |
| `known_ramp.auto_initialize_stance` | `true` | `true`, `false` | Initializes nominal stance before known-ramp planning; launch name `known_ramp_auto_initialize_stance`. | Set `false` only after verified physical nominal stance. |
| `known_ramp.hip_rate_limit_deg_s` | `144` deg/s | positive deg/s | Applied hip interpolation rate limit. | No without actuator evidence. |
| `known_ramp.command_smoothing_s` | `0.20` s | non-negative seconds | Smooth transition to each new planner target. | No without tracking evidence. |
| `planner_posture_test.duration_s` | `4.0` s | positive seconds | Stationary planner-posture test duration. | Only that mode. |

### ROS node: hip PID and diagnostic motion modes

| Parameter | Default | Possible values | Description | Set for an experiment? |
| --- | --- | --- | --- | --- |
| `hip_kp`, `hip_ki`, `hip_kd` | `350`, `0`, `5` | finite motor-controller gains | Gains included in every hip position command. | **Yes**: record in per-trial PID YAML. |
| `hip_test.delta_deg` | `[5, 5, -5, -5]` | four finite degrees (A/B/C/D) | Bounded hip-test displacement. | Only hip test. |
| `hip_test.rate_deg_s` | `8` deg/s | positive deg/s | Hip-test command rate. | Only hip test. |
| `hip_test.tolerance_deg` | `0.5` deg | non-negative deg | Hip-test completion tolerance. | Only hip test. |
| `hip_test.hold_s` | `2.0` s | non-negative seconds | Hip-test final hold. | Only hip test. |
| `hip_calibration.delta_deg` | `[-3, -3, 3, 3]` | four finite degrees | Sequential outward-calibration displacement. | Only calibration. |
| `hip_calibration.rate_deg_s`, `hip_calibration.hold_s` | `6`, `1` | positive deg/s, non-negative s | Calibration rate and hold. | Only calibration. |
| `wheel_calibration.speed_rad_s` | `0.5` rad/s | finite rad/s | Four-wheel sign-check speed. | Only wheel calibration. |
| `wheel_calibration.pre_drive_hold_s`, `wheel_calibration.drive_s` | `1`, `1` s | non-negative seconds | REST hold then drive duration. | Only wheel calibration. |
| `stance.target_deg` | `[-45, -45, 45, 45]` | four finite degrees | Standalone or automatic nominal stance. | No unless geometry changes. |
| `stance.hip_rate_deg_s` | `15` deg/s | positive deg/s | Standalone stance rate. | No without safety review. |
| `stance.hip_to_wheel_m`, `stance.wheel_radius_m` | `0.260`, `0.0585` m | positive m | Stance wheel-coordination geometry. | No without measurement update. |
| `stance.tolerance_deg`, `stance.hold_s` | `0.5` deg, `1` s | non-negative deg, seconds | Stance completion tolerance and hold. | No without safety review. |

### ROS node: Vicon trigger

| Parameter | Default | Possible values | Description | Set for an experiment? |
| --- | --- | --- | --- | --- |
| `vicon_trigger.enabled` | `false` | `true`, `false` | Owns physical GPIO trigger while timed known-ramp motion is active. | Only with Vicon. |
| `vicon_trigger.chip`, `vicon_trigger.line` | `/dev/gpiochip0`, `112` | existing chip path, non-negative line | Hardware GPIO identity. | No without hardware change. |
| `vicon_trigger.gpiod_site_packages` | `/home/biorola/.local/lib/python3.10/site-packages` | valid Python package path | Local gpiod v2 binding location. | No without environment change. |
| `vicon_trigger.test_mode` | `false` | `true`, `false` | Enables GPIO-only test, which must remain unarmed. | Only trigger preflight. |
| `vicon_trigger.test_duration_s` | `3.0` s | positive seconds | GPIO self-test duration. | No. |

The active PID values are logged when the node starts.  The controller will
not publish a hip position command until fresh feedback has produced its first
feedback-derived command; no parameter disables that startup interlock.

### One-sided angle-difference compensation

The motor-state absolute sensor reports `position_diff`; for new recordings,
`actual_hip_angle = motor_position + position_diff`. With the default
`angle_diff_compensation.gain: 0.0`, this measurement is recorded but does not
change a command. With a positive gain, the controller retains each hip's
greatest observed **outward** difference since node startup (front: most
negative; rear: most positive) and adds `gain × difference` to an outward
target only. For example, a front target of -45 degrees with greatest
`position_diff = -2 degrees` and gain 0.4 becomes -45.8 degrees. Inward
targets receive no bias, and the addition is bounded by
`angle_diff_compensation.maximum_abs_rad`. This is an experimental transmission
compensation, not a force/contact estimate; retain a gain-zero baseline.
