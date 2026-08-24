# Kilin terrain-control operating modes

This reference separates runnable Isaac workflows from future real-robot integration. A profile is an obstacle geometry, not a validated performance result.

## Profiles and suspension

The current assets are the 80 mm and 150 mm profiles (this interprets the recent "8 mm" and "15 mm" shorthand as 80 mm and 150 mm).

| Profile | Geometry | Isaac USD |
| --- | --- | --- |
| `terrain_80mm_two_ramps.yaml` | Two one-sided 80 mm ramp-platform-ramp obstacles | `scenes/kilin_two_one_sided_ramps_experiment.usd` |
| `terrain_150mm_20deg_single.yaml` | One-sided 150 mm, 20 degree ramp; 500 mm deck | `scenes/kilin_one_sided_ramp_150mm_20deg_mapping_test.usd` |

Normal scenes retain the passive 17 mm suspension travel. The rigid comparison is the separate `scenes/kilin_two_one_sided_ramps_rigid_suspension.usd`, with all four suspension joints locked at zero. Stop and reset, then open the desired USD; do not lock/unlock joints while playing. There is no safe GUI toggle or USD variant set yet. A 150 mm rigid variant has not been generated.

## Terrain-input modes and topics

The two names below are deliberately separate: `terrain_profile` is the
controller's preconfigured YAML obstacle geometry, while `terrain_source` is
the local-terrain mapper input selection.

| Mode | Controller configuration | Mapper configuration | LiDAR/FAST-LIO use | Status |
| --- | --- | --- | --- | --- |
| Known analytic terrain | `use_odometry=false`, `use_terrain_window=false`, select `terrain_profile:=...yaml` | Do not run mapper | None. The controller constructs terrain from the selected YAML profile. | Current real-Kilin default and the only approved control mode. |
| Analytic terrain window | `use_terrain_window=true` | `terrain_source:=analytic` with matching ramp parameters | None. The mapper generates a moving window from configured analytic geometry. | Regression/debug only; not live sensing. |
| FAST-LIO point-cloud window | `use_odometry=true`, `use_terrain_window=true` | `terrain_source:=pointcloud`, `pointcloud_topic:=/cloud_registered` | Yes. Mapper consumes FAST-LIO `/cloud_registered`; controller consumes `/Odometry` and `/kilin/terrain/local_window`. | Data collection and offline validation only; do not arm live terrain control yet. |

`terrain_source:=terrain_cloud` is not a valid parameter value; use exactly
`terrain_source:=pointcloud`.

```text
/kilin/isaac/ground_truth/odometry
  -> /Odometry
  -> /kilin/terrain/local_window
  -> /kilin/motor_cmd_raw
```

`/tf` is not a controller or mapper input. Keep it standard. The controller reads odometry and `/kilin/terrain/local_window` directly; do not rename either topic without changing every producer and consumer.

## Fixed speed/time versus live speed

| Choice | Launch settings | Behaviour |
| --- | --- | --- |
| Live speed | `use_speed_command:=true` | Reads `/kilin/control/target_speed_m_s`; stops after 0.5 s without a fresh command. Duration is ignored. |
| Fixed speed/time | `use_speed_command:=false speed_m_s:=0.10 run_duration_s:=22.0 hard_motion_limit_s:=25.0` | Uses fixed speed and stops at the earlier of requested duration and hard limit. |

Maximum live speed is 0.30 m/s. Begin at 0.05--0.10 m/s.

Before every ROS command:

```bash
source /opt/ros/humble/setup.bash
source /home/biorola/kilin_ws/kilin_ros_ws/install/setup.bash
```

After pulling planner or controller changes, rebuild their installed copies
before sourcing the workspace. The planner is a ROS package, so no external
`Documents` checkout is needed at runtime:

```bash
cd /home/biorola/kilin_ws/kilin_ros_ws
env -u COLCON_CURRENT_PREFIX bash -lc '
  source /opt/ros/humble/setup.bash
  colcon build --packages-select kilin_motion_planner kilin_known_terrain_controller
'
```

On Ubuntu/ROS, verify the numerical packages before launching:

```bash
python3 -c 'import numpy, scipy; print("NumPy", numpy.__version__, numpy.__file__); print("SciPy", scipy.__version__, scipy.__file__)'
```

The installed Ubuntu SciPy requires NumPy below 1.25. If this check shows
NumPy 2.x under `~/.local`, it is shadowing the system package. On a machine
where NumPy 2.x is not otherwise needed, repair it with:

```bash
python3 -m pip install --user --upgrade --force-reinstall 'numpy>=1.17.3,<1.25'
```

Alternatively, run the ROS command with `PYTHONNOUSERSITE=1` to use the
system NumPy/SciPy pair without changing the user site:

```bash
PYTHONNOUSERSITE=1 ros2 launch kilin_known_terrain_controller one_sided_ramp_control.launch.py ...
```

## Isaac bridge and FAST-LIO emulator

These are different components with different jobs.

| Component | Job | Use it when | Do not use it when |
| --- | --- | --- | --- |
| `isaac_bridge` | Converts controller commands into the Isaac command topics consumed by `/Graphs/ros2_controller`, and converts Isaac JointState back into canonical `/motor/state`. | Every Isaac controller run in sections 1--3. | Real robot runs. |
| `kilin_fastlio_emulator` | Copies Isaac ground-truth odometry into `/Odometry` (`camera_init` to `body`) with the expected FAST-LIO-like interface. It is not FAST-LIO2 and does not create a point cloud/map. | Isaac terrain-window runs in section 3. | Pre-entered-map Isaac runs (sections 1--2) and all real robot runs. |

`one_sided_ramp_control.launch.py` defaults to `target:=real`, which starts no Isaac component. Every Isaac command below explicitly uses `target:=isaac`; only then does it start `isaac_bridge` with `start_cmd_converter:=false`. Do **not** start a second `isaac_bridge` manually; that could create competing command sources. The Isaac controller launch needs the Isaac stage running and its `/Graphs/ros2_controller` graph present.

The FAST-LIO emulator is separate and requires `/kilin/isaac/ground_truth/odometry`, which comes from `/Graphs/ROS_Odometry` while Isaac is playing. It produces only `/Odometry`; therefore it supports the analytic moving-window regression but cannot by itself support the genuine point-cloud mode.

## 1. Isaac: 80 mm pre-entered map

Open `scenes/kilin_two_one_sided_ramps_experiment.usd`, press Play, then run:

```bash
ros2 launch kilin_known_terrain_controller one_sided_ramp_control.launch.py \
  target:=isaac mode:=known_ramp armed:=true \
  terrain_profile:=terrain_80mm_two_ramps.yaml \
  use_speed_command:=false speed_m_s:=0.10 \
  run_duration_s:=22.0 hard_motion_limit_s:=25.0 \
  target:=isaac
```

Do not start the FAST-LIO emulator or terrain mapper in this mode.

The controller launch starts the Isaac bridge automatically.

## 2. Isaac: 150 mm pre-entered map

Open the 150 mm USD, press Play, then run:

```bash
ros2 launch kilin_known_terrain_controller one_sided_ramp_control.launch.py \
  target:=isaac mode:=known_ramp armed:=true \
  terrain_profile:=terrain_150mm_20deg_single.yaml \
  use_speed_command:=false speed_m_s:=0.05 \
  run_duration_s:=30.0 hard_motion_limit_s:=35.0 \
  target:=isaac
```

The 150 mm obstacle is higher than the configured 115 mm lower-body clearance. Start slowly and monitor body contact.

The controller launch starts the Isaac bridge automatically; FAST-LIO emulator is not needed.

## 3. Isaac: live terrain window, any profile

Make the active Isaac scene publish `/kilin/isaac/ground_truth/odometry`. In separate terminals run:

```bash
# A. Isaac ground-truth odometry -> FAST-LIO-like odometry
ros2 launch kilin_fastlio_emulator fastlio_emulator.launch.py

# B. Genuine live geometry requires this PointCloud2 input to exist.
ros2 run kilin_local_terrain_mapping local_terrain_window --ros-args \
  -p terrain_source:=pointcloud \
  -p pointcloud_topic:=/cloud_registered

# C. Controller, initially unarmed.
ros2 launch kilin_known_terrain_controller one_sided_ramp_control.launch.py \
  target:=isaac mode:=known_ramp armed:=false use_speed_command:=true \
  target:=isaac
```

Verify topics, then enable inputs and arm:

```bash
ros2 topic echo --once /kilin/isaac/ground_truth/odometry
ros2 topic echo --once /Odometry
ros2 topic echo --once /kilin/terrain/local_window
ros2 param set /kilin_known_terrain_controller use_odometry true
ros2 param set /kilin_known_terrain_controller use_terrain_window true
ros2 param set /kilin_known_terrain_controller armed true
```

Until Isaac provides a map cloud, use this 150 mm analytic moving-window regression instead of process B:

```bash
ros2 run kilin_local_terrain_mapping local_terrain_window --ros-args \
  -p terrain_source:=analytic \
  -p analytic_ramp.height_m:=0.15 \
  -p analytic_ramp.up_ramp_length_m:=0.412121216 \
  -p analytic_ramp.deck_length_m:=0.50 \
  -p analytic_ramp.down_ramp_length_m:=0.412121216 \
  -p analytic_second.enabled:=false
```

This is a live window, not live terrain sensing.

## 4. Real Kilin: known analytic terrain (current approved control mode)

The real launch bypasses `isaac_bridge`. `real_kilin_known_ramp.launch.py` publishes planner commands directly to `/motor/command` and reads canonical feedback directly from `/motor/state`. The generic launch's default, `target:=real`, also starts no Isaac component. Module mapping is `A/B/C/D = FL/FR/RL/RR`; motor-state hip positions must use the same radians convention as motor commands, so verify this with `hip_calibration` before any ramp motion.

Prerequisites: physical e-stop is reachable, wheel clearance/test area are ready, the real bridge endpoint is configured (including `CORE_MASTER_ADDR` if your deployment needs it), `kilin_panel` is open, and a second operator is present. The panel launch owns the real ROS/gRPC bridge; do not start a second `kilin_ros2_bridge` process.

```bash
# Terminal A: required real-Kilin panel and its ROS/gRPC bridge.
# This source launch file is named launch.py. Do not start isaac_bridge or a
# second kilin_ros2_bridge process.
cd /home/biorola/kilin_ws/kilin_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kilin_panel launch.py

# Terminal B: calibration controller, disarmed.
cd /home/biorola/kilin_ws/kilin_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 ros2 launch kilin_known_terrain_controller real_kilin_known_ramp.launch.py \
  armed:=false mode:=hip_calibration \
  terrain_profile:=terrain_150mm_20deg_single.yaml
```

Confirm feedback before arming:

```bash
ros2 topic echo --once /motor/state
```

Before arming the terrain controller, put the panel in **Manual** mode so its
UI Send control is disabled. During an armed planner run, the terrain controller
is the only permitted publisher of `/motor/command`; do not use the panel Send
button or start another joystick/converter command path.

With the panel in **Manual** mode, arm the calibration mode from Terminal C:

```bash
ros2 param set /kilin_known_terrain_controller armed true
```

Stop the calibration node, confirm the expected outward motion and radians,
then relaunch Terminal B in known-ramp mode:

```bash
PYTHONNOUSERSITE=1 ros2 launch kilin_known_terrain_controller real_kilin_known_ramp.launch.py \
  armed:=false mode:=known_ramp \
  terrain_profile:=terrain_150mm_20deg_single.yaml \
  use_speed_command:=false speed_m_s:=0.05 \
  run_duration_s:=30.0 hard_motion_limit_s:=35.0
```

Arm only after fresh feedback is still visible:

```bash
ros2 param set /kilin_known_terrain_controller armed true
```

This launch hard-codes `use_odometry=false` and `use_terrain_window=false`.
It uses only the selected YAML `terrain_profile`; pose is integrated from the
initial pose plus applied wheel speed, so it is open-loop rather than measured
feedback. `debug_publish:=true` may be added for RViz plan diagnostics only.

### 4a. Real Kilin: analytic terrain control with FAST-LIO shadow recording

Use this mode to run the approved YAML analytic controller while MID360s and
FAST-LIO2 run independently for RViz and bag recording. It is the recommended
transition experiment before any live-map controller input: the robot motion
remains governed by `terrain_profile`, while you inspect `/Odometry`,
`/cloud_registered`, and `/Laser_map` from the real sensor.

Start the panel/bridge as Terminal A in section 4. In a separate Terminal B,
start the sensor front end:

```bash
cd /home/biorola/kilin_ws/kilin_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kilin_fastlio_bringup mid360s_fastlio.launch.py
```

In Terminal C, start the same known analytic profile controller, initially
disarmed. `debug_publish:=true` is optional and only publishes the controller
path/footprints; it does not consume FAST-LIO data:

```bash
cd /home/biorola/kilin_ws/kilin_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 ros2 launch kilin_known_terrain_controller real_kilin_known_ramp.launch.py \
  armed:=false mode:=known_ramp \
  terrain_profile:=terrain_150mm_20deg_single_600mm.yaml \
  use_speed_command:=false speed_m_s:=0.05 \
  run_duration_s:=30.0 hard_motion_limit_s:=35.0 \
  debug_publish:=true
```

Before arming, verify FAST-LIO is live and view its map in RViz with Fixed
Frame `map`:

```bash
ros2 topic echo --once /Odometry
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
```

Keep the panel in **Manual** mode, then arm only after the usual real-robot
checks are complete:

```bash
ros2 param set /kilin_known_terrain_controller armed true
```

Do not start `kilin_local_terrain_mapping` in this shadow mode: its analytic
or point-cloud window would be unused because this controller launch has
`use_terrain_window=false`. Do not set `use_odometry=true` or
`use_terrain_window=true` for this experiment; record and review the data
first.

## 5. Real Kilin on Orin: MID360s + FAST-LIO2 recording

`kilin_fastlio_bringup` launches the Livox MID360s driver, unmodified
FAST-LIO2, and the static `map -> camera_init -> body -> base_link` TF chain.
The map is suitable for RViz and offline validation. It is **not yet approved
for live terrain control**: ground/obstacle filtering and a base-link odometry
adapter still need validation.

Build the Livox driver with its upstream ROS 2 helper, then build the local
FAST-LIO bringup. Do not use a plain driver-only `colcon build`, which omits
the upstream Humble CMake setup:

```bash
cd /home/biorola/kilin_ws/kilin_ros_ws
source /opt/ros/humble/setup.bash
cd src/livox_ros_driver2 && ./build.sh humble
cd ../..
source install/setup.bash
colcon build --packages-select fast_lio kilin_fastlio_bringup
source install/setup.bash
ros2 launch kilin_fastlio_bringup mid360s_fastlio.launch.py
```

FAST-LIO2 publishes:

```text
FAST-LIO2 odometry    -> /Odometry
map-frame PointCloud2 -> /cloud_registered
local map             -> /Laser_map
terrain window        -> /kilin/terrain/local_window
```

In RViz, use `map` as Fixed Frame. For real experiments, record the raw Livox
topics, FAST-LIO outputs, `/tf`, `/tf_static`, `/motor/command`, and
`/motor/state` before enabling any map-derived controller input.

To inspect only the controller's lightweight plan diagnostics, add
`debug_publish:=true` to a controller launch and display
`/kilin/planner/debug/horizon` and `/kilin/planner/debug/footprints`. This does
not start RViz or publish an additional terrain/FAST-LIO cloud.

## Send a live speed command

```bash
ros2 topic pub -r 5 /kilin/control/target_speed_m_s \
  std_msgs/msg/Float32 "{data: 0.10}"
```

Change the value while publishing to vary speed. `Ctrl+C` stops publishing and the 0.5 s watchdog commands zero; publish `{data: 0.0}` once for an explicit stop.
