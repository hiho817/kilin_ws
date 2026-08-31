# Kilin Hip Characterization

One ROS 2 package for low-level hip-transmission characterization, PID and bounded feedforward tuning, and later force/contact experiment support. It replaces `kilin_csv_control` only for these experiments: it directly fills every `MotorCmd` field needed for PID and feedforward torque.

## Safety contract

- It starts **disarmed** and publishes to `/kilin/hip_characterization/command_preview`, never `/kilin/motor_cmd_raw`.
- Real actuation requires both `armed:=true` and an explicit real command topic.
- It sends nothing until `/motor/state` is fresh, then captures current hip and hub feedback as its starting state.
- New-recording convention is `actual_hip_angle_rad = motor_position + position_diff`.
- Strategy **1.1.0** commands the raw motor-position reference directly. `position_diff` is not fed back into the position or FF loop.
- `actual_hip_angle_rad` is still reconstructed and recorded for offline transmission/force analysis. Safety aborts on stale state, non-finite state, motor error code, configured hip-torque bound, or configured raw-motor tracking bound. Abort sends hip rest and hub brake.

The package does not modify `kilin_com_estimator`, the terrain planner, or FAST-LIO2.

## Phase-A campaign

The runner performs a two-state cycle for each repetition:

```text
state-A hold → slow static-release ramp → dynamic move to state B
→ segment hold → return to state A → final state-A hold
```

The static-release segment is intentional: worm-gear breakaway/static friction must be measured separately from the dynamic part. The runner logs phase IDs, therefore offline analysis can compare static-to-dynamic transition behavior directly.

The initial profile is `config/initial_screening.yaml`. For experiments, keep
one direct `profile.yaml` or one master `master.yaml` in the dated log
directory. A master file supplies defaults plus named unit-test overrides; no
cells are generated beforehand. The runner writes the fully resolved profile
only after each unit is executed, as experiment evidence.

Every YAML carries `strategy_name` and numeric `strategy_version`; changing a
control policy requires a new name/version before an experiment is run.

### Wheel-mode status and intended interface

**Strategy 1.1.0 supports `rest` only.** The executable writes hub `rest` on
every startup, test, hold, recovery, and abort command. It therefore cannot be
used to test wheel velocity or wheel torque yet. A YAML `wheel_mode`,
`wheel_torque_*`, or wheel-geometry field is silently ignored by this revision;
do not use such a profile as evidence for a wheel-assisted experiment.

The next wheel-enabled strategy will add one explicit `wheel_mode` and will
record its wheel policy in the manifest. Its modes will mean:

| Future mode | Hub command during hip test | Intended use and safety rule |
| --- | --- | --- |
| `rest` | Hub motor mode `rest`; no active velocity or torque command. | Baseline hip-only test. This remains mandatory for startup, recovery, and abort. |
| `brake` | Hub motor mode `brake`; no velocity or torque setpoint. | Constrain wheel rotation while measuring the transmission. It is not an active wheel-position controller. |
| `speed_ik` | Hub velocity is calculated live from the commanded hip trajectory at every control tick. | The wheel follows hip motion kinematically; it is never a fixed speed. The command field uses the established RPM-times-ten unit: `hub.velocity = wheel_rate_rad_s × 60 × 10 / (2π)`. |
| `torque_assist` | Hub torque magnitude is a fixed configuration value. Only its sign is selected from the live IK wheel-rate direction. | Assist the same direction as `speed_ik` while the hip extends/contracts. The torque is **not** calculated from IK. Outward and inward fixed magnitudes may differ and require a dedicated lower hub-torque clamp. |
| `position_hold` | Hub position is initialized from its measured position before enabling position mode. | Only for a later dedicated test. Starting position mode with a zero command when the wheel is already rotated can cause a violent spin, so a fixed zero reference is prohibited. |

For `speed_ik`, IK supplies both the live wheel-rate magnitude and direction.
For `torque_assist`, IK supplies **only the direction**; the magnitude comes
unchanged from the profile. The direction comes from the Jack hip-test
kinematic convention, evaluated from the live commanded hip angle and hip
rate. In the simple planar model this is
`wheel_rate = -L × cos(commanded_hip_angle) × hip_rate / R`, where `L` is the
hip-to-wheel distance and `R` is wheel radius. The torque sign must equal the
sign of that computed wheel rate; it must not be a constant "forward" sign.
Near zero wheel rate, the implementation will apply a deadband and send zero
torque to prevent sign chatter. Wheel-speed and fixed-torque limits will be separately
bounded and logged with the hip command before any armed wheel-mode campaign.

## Complete runner parameter reference

The direct runner reads parameters below `kilin_hip_characterization.ros__parameters`.
The master runner merges `kilin_hip_batch.defaults` with each test's
`parameters` mapping and writes the resulting values to that unit's
`resolved_profile.yaml`.

### Invocation and provenance

| Parameter | Type / default | Exact effect |
| --- | --- | --- |
| `armed` | bool / `false` | Hardware gate. Nothing is commanded while false. `single_runner.py` and `batch_runner.py` set it true only with their `--armed` flag. |
| `command_topic` | string / preview topic | `MotorCmdStamped` destination. An armed helper overrides it to `/motor/command`. |
| `state_topic` | string / `/motor/state` | Feedback source. The run waits for fresh feedback before starting. |
| `run_dir` | string / empty | Evidence folder. Required for an armed direct controller invocation; the helpers set it automatically. |
| `bag_topics` | string list / six base topics | Topics passed to `ros2 bag record` by `single_runner.py` or `batch_runner.py`. Place it in a direct profile's `ros__parameters`, or master `defaults`; a unit-test override may replace it. The helper removes it before passing the resolved profile to the C++ controller and saves the final list as `bag_topics.txt`. |
| `strategy_name` | string | Human-readable control/analysis strategy name, recorded in the manifest. |
| `strategy_version` | numeric string | Revision of that strategy, e.g. `1.1.0`, recorded in the manifest. Use `1.1.0` or a later new version for the raw-motor controller; do not reuse `1.0.0` compensated-run profiles. |

### Test selection and geometry

| Parameter | Type / default | Exact effect |
| --- | --- | --- |
| `active_modules` | string list / `[A,B]` | Hips selected for the unit. `A,B` are front; `C,D` rear; `A,C` left; `B,D` right. Inactive hips are commanded to physical zero during position phases. |
| `repetitions` | integer / `3` | Number of complete A→B→A cycles before recovery. Must be at least one; use three or more for analysis. |
| `state_a_deg` | degrees / `0` | Initial and return **motor-position reference** magnitude. The runner maps front hips to negative and rear hips to positive references. This is not corrected by `position_diff`. |
| `state_b_deg` | degrees / `45` | Other motor-position reference magnitude for the same mapping. `(0,45)` is baseline; `(15,45)` deliberately starts from a nonzero reference. |
| `startup_move_speed_rad_s` | rad/s / `0.1` | Smooth position move from current measured **motor position** to state A before the unit test. Hip FF is forced to zero and wheels rest. |
| `recovery_position_deg` | degrees / `0` | Active-hip motor-position reference reached after every unit test. |
| `recovery_move_speed_rad_s` | rad/s / `0.1` | Smooth state-A/recovery move speed after the all-motor rest interval. |
| `recovery_rest_s` | seconds / `1.0` | Rest interval between the final cycle and the recovery move; feedback continues to be read. |

### Hip trajectory and PID

| Parameter | Type / default | Exact effect |
| --- | --- | --- |
| `hip_speed_rad_s` | rad/s / `0.2` | Constant-speed dynamic portion of A→B and B→A motion. It must be positive. |
| `start_zero_hold_s` | seconds / `2.0` | Position hold at state A after startup, before static release. The name is historical; it applies even when A is not zero. |
| `static_release_ramp_s` | seconds / `1.0` | Smooth duration of the initial low-distance static-release segment. |
| `static_release_fraction` | fraction / `0.15` | Fraction of the A→B stroke assigned to static release. The remaining fraction is the constant-speed dynamic move. Use `0.15` initially. |
| `segment_hold_s` | seconds / `1.0` | Position hold at state B before returning to A. |
| `kp`, `ki`, `kd` | direct motor gains / `360,0,5` | Hip position-loop gains copied directly into every hip `MotorCmd`. Change only one candidate dimension at a time during PID screening. |

#### Static-release phase in detail

Static release is the deliberately slow first part of every A→B movement. It
is intended to reveal worm-gear breakaway, stiction, and backlash before the
constant-speed tracking result is mixed in.

For an A→B stroke of `D = state_b_deg - state_a_deg`, the runner executes:

```text
1. Hold state A for start_zero_hold_s.
2. Move smoothly through static_release_fraction × D over static_release_ramp_s.
3. Move through the remaining (1 - static_release_fraction) × D at hip_speed_rad_s.
4. Hold B, then return from B to A at hip_speed_rad_s.
```

With the baseline values `A=0°`, `B=45°`, `static_release_fraction=0.15`, and
`static_release_ramp_s=1.0`, the first 6.75° is a smooth one-second breakaway
test. The remaining 38.25° is the dynamic outward move. The return 45° is a
separate dynamic inward/raising measurement; it does not reuse the static
release ramp.

The static release uses a smoothstep position ramp: it starts and ends with
zero commanded velocity, avoiding an artificial velocity step. PID remains
active throughout. During this phase only,
`hip_ff_static_outward_direct` is used for outward movement (or
`hip_ff_static_inward_direct` if an inward static-release strategy is later
introduced). Dynamic movement uses `hip_ff_outward_direct` or
`hip_ff_inward_direct` instead.

To disable static release deliberately, set **both**
`static_release_fraction: 0.0` and `static_release_ramp_s: 0.0`; the runner
then begins the dynamic A→B move immediately after the state-A hold. Otherwise
keep the fraction strictly between `0` and `1` and the ramp duration positive.
Use a small fraction such as `0.10–0.20` so it characterizes breakaway without
turning the whole stroke into a quasi-static test. Change one of fraction, ramp
duration, PID, or static FF at a time. Compare static-release tracking
error/torque with the following dynamic segment; do not average them into one
undifferentiated score.

### Hip feedforward

| Parameter | Type / default | Exact effect |
| --- | --- | --- |
| `hip_ff_outward_direct` | direct command / `0` | Magnitude added during dynamic motion whose planned direction is outward/lowering. Sign is selected from planned front/rear movement. |
| `hip_ff_inward_direct` | direct command / `0` | Magnitude added during dynamic inward/raising movement. It may differ from outward FF. |
| `hip_ff_static_outward_direct` | direct command / `0` | Magnitude used only during the outward static-release ramp. |
| `hip_ff_static_inward_direct` | direct command / `0` | Magnitude used during an inward static-release ramp; current two-state baseline only has an outward initial static-release segment. |
| `max_abs_hip_ff_torque_nm` | direct-command clamp / `200` | Absolute clamp applied after FF direction selection. It is independent of the broader hip safety torque limit. |

### Safety and observability

| Parameter | Type / default | Exact effect |
| --- | --- | --- |
| `max_state_age_s` | seconds / `0.10` | Maximum acceptable age of `/motor/state`. A stale state rests all motors and aborts. |
| `max_abs_hip_error_rad` | radians / `0.35` | Largest allowed measured-motor-minus-commanded-motor error. A breach aborts. It is deliberately not based on reconstructed actual hip angle in strategy 1.1.0. |
| `max_abs_hip_torque_nm` | feedback units / `400` | Largest allowed absolute hip torque feedback value. A breach aborts. Confirm feedback units on hardware before changing it. |

The runner logs phase changes to the terminal and `launch.log`: startup move,
state-A hold, static release, move to B, hold at B, return to A, recovery
rest, recovery move, complete, or aborted. `command_state_trace.csv` records
the controller pair `commanded_motor_rad`/`motor_position_rad`, alongside
`position_diff_rad` and the observation-only reconstruction
`actual_hip_angle_rad = motor_position + position_diff`.

On every abort, the terminal and `launch.log` identify the module, phase,
reason, measured value, and configured limit where applicable. The same reason
is saved as `<run_dir>/abort_reason.txt`. For example, a tracking abort reports
`module=A phase=move_to_state_b reason=hip_motor_tracking_error_limit` plus
commanded motor angle, measured motor angle, signed error, and its absolute
limit.

## Run directory and recording

Follow the 2026-08-26/27 convention: one dated folder, one immutable run folder per unit test, a README table updated before and after each run, the copied/resolved profile, terminal log, and bag.

```text
~/kilin_ws/logs/YYYY-MM-DD/
  README.md
  hip_front_rest_kp350_ff0_speed2_rep_set01/
    profile.yaml
    launch.log
    command_state_trace.csv
    trial_manifest.yaml
    bag/
```

Before arming, create the directory and start full recording. At minimum record the direct state/command contract and power:

```bash
ros2 bag record -o <run_dir>/bag /motor/state /motor/command /power/state /power/command /tf /tf_static
```

Add Vicon, force-plate, and trigger topics for force/contact sessions. Do not rename rosbag files after recording. The runner trace is a compact synchronized command/state analysis file; the bag remains the authoritative raw evidence.

## Dry run and real run

For a direct one-unit profile with automatic bag/manifest handling:

```bash
ros2 run kilin_hip_characterization single_runner.py --armed <profile.yaml> <new-run-directory>
```

For a master batch profile, each named unit test is resolved into its own
evidence folder. The batch stops on the first fault/refusal rather than moving
to the next unit:

```bash
ros2 run kilin_hip_characterization batch_runner.py --armed <master.yaml> <new-batch-run-directory>
```

The unit runner moves to state A at `startup_move_speed_rad_s`, uses zero FF
and wheels rest during this move, then performs the test. It rests, reads
feedback, and moves to global `recovery_position_deg` after each unit.

Build in the normal clean overlay environment:

```bash
cd ~/kilin_ws/kilin_ros_ws
env -u COLCON_CURRENT_PREFIX bash -lc 'source /opt/ros/humble/setup.bash && colcon build --packages-select kilin_hip_characterization'
source install/setup.bash
```

Dry run/default preview:

```bash
ros2 launch kilin_hip_characterization characterization.launch.py
```

For a real run, first copy and review a profile and create the run directory. Then explicitly provide `armed:=true`, `command_topic:=/motor/command`, and `run_dir:=...`. This package must be the only command authority for the test. In strategy 1.1.0, hubs remain in `rest`; the RPM-times-ten velocity conversion described above is a future `speed_ik` interface, not a currently published command.

## Offline analysis

```bash
python3 $(ros2 pkg prefix kilin_hip_characterization)/share/kilin_hip_characterization/scripts/analyze_tracking.py <run_dir>/command_state_trace.csv
```

The first report summarizes actual-angle RMS/bias/peak error, motor torque peak, and fault samples for each trial/module/phase. The next analysis increments will aggregate the three repetitions, quantify hysteresis/backlash, and rank PID/feedforward candidates.

## Force/contact path

Force-plate trials use this same package and the same run-directory structure. A force estimator always emits a corresponding contact output by thresholding its predicted vertical load; contact threshold, hysteresis, and timing must be versioned with the estimator. Estimator versions will be stored with their exact feature set and complete-trial validation split. Force-plate/Vicon data are offline labels, not a planned terrain-controller input.
# Kilin hip characterization

See [STRATEGY_VERSIONING.md](STRATEGY_VERSIONING.md) for the zero-referenced sequence controller, asymmetric guarded feedforward policy, wheel modes, and the required versioning/provenance convention.
