# Orin compute budget: FAST-LIO2 to terrain planner

## Scope and pipeline

This guide covers the intended live Orin path:

```text
MID-360 raw LiDAR + IMU
  -> Livox driver -> FAST-LIO2 pose and registered cloud
  -> hip-centre odometry adapter -> local terrain mapper
  -> TerrainWindow -> 10 Hz receding-horizon planner
```

It is a measurement plan, not proof that the present 50 mm dense profile is
real-time on Orin. Profile on the Orin with the controller disarmed, change one
variable per run, and record timing, memory, temperature, and terrain coverage.

FAST-LIO2 in this checkout is CPU/OpenMP/PCL code; its registration and ikd-tree
do not use the Orin GPU. GPU load is mainly RViz/display or other processes.
At the current 10 Hz LiDAR rate, the scan-period budget is 100 ms. Mapping and
terrain extraction must leave margin for the 10 Hz planner and all ROS I/O.

## Current profiles

`config/fastlio_mid360s_windowed.yaml` is the odometry baseline:
`point_filter_num: 3`, 250 mm scan/map voxels, and no dense registered cloud.

`config/fastlio_mid360s_terrain_dense.yaml` is the terrain-quality experiment:
`point_filter_num: 1`, 50 mm scan/map voxels, and `dense_publish_en: true`.
It is deliberately expensive and must not be assumed suitable for Orin.

The terrain mapper defaults to a 3 m forward, 1 m rear, +/-1 m window, 50 mm
cells, 5 Hz output, 0.2--3 m forward sensor ROI, and 50 mm retained-scan voxels.
Terrain observed ahead remains cached while within the moving window; 45 s is a
safety cap, not the normal spatial retention limit.

## Compute variables

### Sensor and Livox driver

| Variable | Resource effect | Meaning / trade-off |
| --- | --- | --- |
| Raw points/s and return mode | CPU, RAM, bandwidth | Fundamental input size; depends on sensor configuration and scene returns. More points improve geometry but affect every later stage. |
| Scan rate (`preprocess.scan_rate`, 10 Hz) | CPU and deadline | More scans/s approximately multiplies mapping work and reduces time available per scan. Sensor/driver rate and parameter must agree. |
| Raw sensor FOV | CPU and pose robustness | More coverage means more points but better geometric constraints. Rear sky is costly outdoors, but removing it can harm odometry. |
| `preprocess.blind` | CPU and near-field quality | Rejects close self/body points. Raising it saves modest work but can discard terrain near the robot. |
| Driver `publish_freq` | callback/transport CPU | More publications mean more ROS work. Do not change independently of sensor output without checking timestamp correctness. |
| LiDAR/IMU synchronization | indirect CPU and correctness | Bad timing causes buffering/reinitialization and bad pose. It is not a performance tuning knob. |

### FAST-LIO2 registration and active map

| Variable | Resource effect | Meaning / trade-off |
| --- | --- | --- |
| `point_filter_num` | CPU | Preprocess keeps roughly every Nth raw point. `1` is densest; `3` is about one third of that input. First core-cost knob to test. |
| `filter_size_surf` | CPU and scan detail | Registration scan voxel size. Smaller values create more nearest-neighbour searches; 50 mm is much heavier than 250 mm. |
| `filter_size_map` | RAM, CPU, map detail | Active ikd-tree voxel size. Smaller cells retain more map points and make map update/search heavier. |
| `max_iteration` | worst-case CPU | Maximum iterated update passes per scan. More may help hard registration but raises latency. |
| `cube_side_length` | RAM and CPU | Size of the rolling ikd-tree cube. Larger retains more history; it needs adequate margin over detection range. |
| `mapping.det_range` | RAM and CPU | Local-map edge/movement behavior; it interacts with cube size and retained map volume. |
| `feature_extract_enable` | CPU | Extra preprocessing. It is currently false; keep it false unless measured benefit exists. |
| `mapping.extrinsic_est_en` | small CPU | Online LiDAR-to-IMU extrinsic state. Do not disable for speed before calibration is established. |
| IMU message rate | callback CPU | Usually smaller than point-cloud cost, but queue growth is a timing fault indicator. |

### FOV: what the current setting actually does

`mapping.fov_degree` is **not** a raw point-cloud crop in this FAST-LIO2 fork.
The source reads it and computes an internal cosine, but does not later use that
value to reject points or restrict the ikd-tree. Changing `360` to `180` will
therefore not meaningfully reduce FAST-LIO2 CPU, RAM, or LiDAR bandwidth here.

Keep the present architecture unless a separate experiment proves otherwise:

```text
full 360-degree raw scan -> FAST-LIO2 pose robustness
registered scan -> body-fixed forward ROI -> terrain window -> planner
```

The downstream front ROI saves terrain-mapper/transport work only. A true
front-180 test requires a timestamp-preserving raw-message filter before
FAST-LIO2 (or a supported sensor-side configuration). That changes odometry
input and must be validated separately; do not enable it for a real robot based
only on a CPU-saving expectation.

### ROS output and diagnostics

| Variable | Resource effect | Meaning / trade-off |
| --- | --- | --- |
| `publish.scan_publish_en` | CPU, bandwidth | Enables `/cloud_registered`; it must be true while the terrain mapper uses it. |
| `publish.dense_publish_en` | bandwidth, mapper CPU | `true` publishes undownsampled registered scans; `false` publishes registration-downsampled scans. It does not itself change core registration. |
| `publish.scan_bodyframe_pub_en` | bandwidth | A second registered cloud. Disable on Orin unless consumed. |
| `publish.map_en` | CPU, RAM, bandwidth | Publishes `/Laser_map` at 1 Hz. Useful in RViz, not required by terrain mapping; disable for deployment tests. |
| `publish.path_en` | RAM and bandwidth | Path grows with run duration. Disable unless logging/debugging. |
| `publish.effect_map_en` | bandwidth | Debug cloud; leave disabled. |
| `runtime_pos_log_enable`, `pcd_save.*` | disk I/O | Keep disabled during real-time operation. |
| RViz and bag recording | CPU/GPU/disk/network | Dense clouds can dominate remote RViz or bag writes. They are diagnostics, not runtime requirements. |

### Local terrain mapper

| Variable | Resource effect | Meaning / trade-off |
| --- | --- | --- |
| Input cloud point count | CPU and RAM | Python deserializes every `/cloud_registered` point before cropping. Dense publication is the main mapper cost. |
| Forward ROI x/y/z bounds | memory and coverage | Reduces accepted terrain points. Shrink only after verifying the planner preview remains covered. |
| `retain_observed_terrain`, retention age | RAM | Preserves a previously seen exit ramp through occlusion. Spatial pruning removes passed terrain; age is a hard cap. |
| `retained_terrain.voxel_m` | RAM and detail | One representative per fixed-frame voxel per scan. Larger values reduce memory but hide smaller terrain features. |
| `forward_m`, `rear_m`, `half_width_m` | cells and points | Delivered terrain extent. Use only preview required by the horizon. |
| `resolution_m` | cells and message size | Halving cell size roughly quadruples cells. 50 mm: about 81 x 41 = 3321 cells; 100 mm: 41 x 21 = 861 cells. |
| `minimum_points_per_cell`, percentile | coverage, small CPU | More points rejects sparse artifacts but increases unknown space. Median/percentile improves outlier robustness. |
| Ground-height gate | safety and coverage | Rejects points more than 100 mm above, or 1.5 m below, the measured hip-axis height in the levelled map frame. It removes ceiling/deep outliers; raise the upper limit only after checking the tallest terrain to be traversed. |
| Cell vertical-span gate (350 mm) | safety and coverage | A cell containing widely separated returns, such as floor plus ceiling, becomes unknown instead of being assigned an unsafe height. |
| `rate_hz` | CPU and bandwidth | Window reconstruction/publication frequency; initially it need not exceed planner rate. |
| Cell/bounds/JSON diagnostics | CPU and bandwidth | Useful in RViz/web inspection. Measure their cost before deployment and make optional if needed. |

### Planner and controller

| Variable | Resource effect | Meaning / trade-off |
| --- | --- | --- |
| `planning_rate_hz` (10 Hz) | optimizer deadline | More replans/s increase CPU. Start at 10 Hz. |
| `horizon_steps` (5) | optimizer CPU | More knots increase decision variables, terrain queries, and constraints; strongest planner-side knob. |
| `horizon_knot_spacing_m` (50 mm) | preview distance | With fixed steps, bigger spacing previews farther without more variables but reduces path/terrain resolution. |
| Footprint/clearance sample density | optimizer CPU | More samples improve checking but add constraints and map queries. |
| `publish_rate_hz` (50 Hz) | modest ROS CPU | Re-publishes safe cached motor commands. Do not lower without tracking validation. |
| Planner debug topics | CPU/bandwidth | Disable `debug_publish` for deployment measurements. |

## Orin measurement workflow

1. Measure the actual Orin while disarmed: power mode, CPU core load, RAM,
   temperature, clocks, and topic rates. Server replay is not Orin evidence.
2. Run baseline odometry: full 360-degree input, `fastlio_mid360s_windowed.yaml`,
   no RViz, no recording, no terrain mapper. Confirm stable odometry at LiDAR
   rate with thermal margin.
3. Add terrain mapping with `dense_publish_en: false` and 100 mm cells. Measure
   valid-cell coverage over the ramp and end-to-end latency.
4. Increase only detail required for coverage: scan voxel size, then point
   filtering, then dense publication. Change exactly one setting per run.
5. Remove diagnostics before judging deployability: disable `/Laser_map`, path,
   body-frame cloud, RViz, and bag recording. Keep only odometry, registered
   cloud, and terrain window.
6. Add the planner in shadow mode (`use_terrain_window:=true`, not armed).
   Log unknown/stale terrain windows and planner deadline misses.
7. Only then use armed low-speed tests. Unknown or stale terrain must stop/fall
   back safely; do not fill it with an analytical map silently.

## Commands to measure on Orin

```bash
# Jetson platform telemetry (separate terminal).
sudo tegrastats --interval 1000

# Per-process CPU and memory; install sysstat if pidstat is absent.
pidstat -dur -p "$(pgrep -d, -f 'fastlio_mapping|local_terrain_window|known_terrain_controller')" 1

# Actual topic rates and signs of backlog.
ros2 topic hz /livox/lidar
ros2 topic hz /cloud_registered
ros2 topic hz /kilin/terrain/local_window
ros2 topic hz /kilin/fastlio/odometry
```

For every profile record YAML/overrides, power mode, ambient temperature,
LiDAR/IMU rates, CPU/RAM/temperature, map valid-cell fraction, and planner
deadline misses. Accept a profile only when it meets timing *and* terrain
coverage requirements with thermal margin.

## Initial recommendation

Do not deploy the 50 mm dense profile unchanged. Begin with full 360-degree
odometry and a no-diagnostics profile. Select point filter, scan/map voxel size,
and dense output empirically from Orin measurements; screenshots alone cannot
establish the safe compute budget.
