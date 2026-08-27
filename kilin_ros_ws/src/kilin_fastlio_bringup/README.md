# Kilin FAST-LIO2 bringup (MID360s)

This bringup replaces the simulator-only `kilin_fastlio_emulator` with live
MID360s odometry and FAST-LIO2, without modifying either vendor package.

Before use, update `config/MID360s_config.json` with the host NIC IP and the
MID360s IP. The configured Livox host address is `192.168.1.11`; it must be
assigned to the NIC connected to the LiDAR. `mapping.extrinsic_T` /
`mapping.extrinsic_R` are only the MID360's internal LiDAR-to-IMU calibration;
the robot mounting pose is configured separately in `config/robot_frames.yaml`.

Build and run:

```bash
cd /home/biorola/kilin_ws/kilin_ros_ws
source /opt/ros/$ROS_DISTRO/setup.bash
cd src/livox_ros_driver2
./build.sh humble
cd ../..
source install/setup.bash
colcon build --packages-select fast_lio kilin_fastlio_bringup
source install/setup.bash
ros2 launch kilin_fastlio_bringup mid360s_fastlio.launch.py
```

Use the upstream `build.sh humble` command for the driver. It creates the
generated ROS 2 manifest, sets `DISTRO_ROS=humble`, and removes its temporary
launch directory after the build. It clears this workspace's `build/` and
`install/` directories first, so run it when no other build is in progress.

Do **not** build the driver with a plain `colcon build --packages-select
livox_ros_driver2`: upstream 1.2.6 requires `-DDISTRO_ROS=humble`, otherwise
CMake can report `LIVOX_INTERFACES_INCLUDE_DIRECTORIES` as `NOTFOUND`.
`build.sh humble` supplies this correctly without modifying the driver
submodule. Always source the workspace-level `install/setup.zsh` (or
`install/setup.bash`) afterwards; never source
`src/livox_ros_driver2/install/local_setup.zsh`, which is a stale path from an
incorrectly inherited prefix.

After that driver-only step, explicitly build `fast_lio` and this bringup
package as shown above. The driver is then resolved from `install/` while its
clean source submodule remains untouched.

FAST-LIO2 retains its default interfaces: `/Odometry` (`camera_init` to
`body`), `/Laser_map`, `/cloud_registered`, and `/path`. Kilin consumers use
the raw topics for diagnostics. This bringup additionally publishes corrected
`/kilin/fastlio/odometry` (`map` to `hip_axis_center`) for the motion planner.
The adapter preserves the raw FAST-LIO topic and refuses unexpected source
frames. `cube_side_length: 30.0` bounds FAST-LIO2's active
ikd-tree map window.

## Robot frames

`config/robot_frames.yaml` is the single source of truth for the static frame
transforms. The launch file publishes the measured MID360 mount as `body` to `base_link`.
`body` is FAST-LIO's MID360-IMU frame; `base_link` is the Kilin vehicle body
origin. The measurement used is the midpoint of the front hip axes plus 200 mm
forward and 70 mm upward, with the MID360 pitched 45 degrees upward. It also
publishes `map` to `camera_init` with a +45 degree pitch, which levels the
otherwise sensor-aligned FAST-LIO local map. In RViz set **Fixed Frame** to
`map` and display `base_link`, `/Odometry`, and `/cloud_registered`.

The static frame file also publishes `base_link -> hip_axis_center` at the
geometric center of the four audited hip axes. The odometry adapter composes
this full TF chain rather than treating the pitched sensor-frame X coordinate
as robot-forward distance.

## Compute budget and Orin deployment

Read [COMPUTE_BUDGET.md](COMPUTE_BUDGET.md) before using the dense terrain
configuration on Orin. It documents compute variables across the MID-360
driver, FAST-LIO2, ROS outputs, local terrain mapper, and planner. In this
FAST-LIO2 fork, `mapping.fov_degree` is not a raw-scan FOV crop and does not
reduce registration load.

If the mechanical mount changes, update `config/robot_frames.yaml`; do not put
this robot-mount transform in FAST-LIO's LiDAR-to-IMU extrinsic parameters.

## Safe server replay and RViz preview

Use the unarmed replay helper to inspect raw-bag terrain perception and the
shadow planner.  It publishes only to `/kilin/terrain_shadow/motor_command`;
it never commands a robot.  Start RViz first:

```bash
source /opt/ros/humble/setup.bash
source ~/kilin_ws/kilin_ros_ws/install/setup.bash
rviz2 -d "$(ros2 pkg prefix kilin_fastlio_bringup)/share/kilin_fastlio_bringup/rviz/terrain_planner_replay.rviz" \
  --ros-args -p use_sim_time:=true
```

Then run the validated dense profile and 0.10 m terrain grid:

```bash
ros2 run kilin_fastlio_bringup run_terrain_shadow_replay.sh \
  --bag ~/kilin_ws/logs/2026-08-26/ramp_test03/bag \
  --label server_rviz_preview_01 \
  --fastlio-config ~/kilin_ws/kilin_ros_ws/src/kilin_fastlio_bringup/config/fastlio_mid360s_terrain_dense_runtime.yaml \
  --terrain-resolution-m 0.10 \
  --visualize-non-nominal-stance
```

The RViz layout uses fixed frame `map` and displays the registered cloud,
filtered terrain cells, terrain-window bounds, corrected hip-centre odometry,
five-knot planned horizon, and planned body footprints.  It does not yet draw
commanded/motor/absolute hip angles as a RobotModel; those require a separate
motor-state-to-joint-state visualizer.

`--visualize-non-nominal-stance` is for RViz replay only.  It relaxes the
initial 45-degree stance gate because replayed feedback cannot move an
unarmed robot into that stance.  The helper remains unarmed and publishes only
to its isolated shadow command topic; never use this flag for a control or
safety validation.
