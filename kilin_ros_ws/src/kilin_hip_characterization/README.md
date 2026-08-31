# Kilin Hip Characterization

One ROS 2 package for low-level hip-transmission characterization, PID and bounded feedforward tuning, and later force/contact experiment support. It replaces `kilin_csv_control` only for these experiments: it directly fills every `MotorCmd` field needed for PID and feedforward torque.

## Safety contract

- It starts **disarmed** and publishes to `/kilin/hip_characterization/command_preview`, never `/kilin/motor_cmd_raw`.
- Real actuation requires both `armed:=true` and an explicit real command topic.
- It sends nothing until `/motor/state` is fresh, then captures current hip and hub feedback as its starting state.
- New-recording convention is `actual_hip_angle_rad = motor_position + position_diff`.
- Hip commands target that reconstructed physical angle: the motor-side command is `desired_actual - current_position_diff`.
- Wheel `position_hold` captures current hub feedback as its target. It never commands a zero wheel position at startup.
- Safety aborts on stale state, non-finite state, motor error code, configured hip-torque bound, or configured actual-angle tracking bound. Abort sends hip rest and hub brake.

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

Wheel modes are `rest`, `brake`, `speed`, `torque`, and `position_hold`. In torque mode, `wheel_torque_outward_nm` and `wheel_torque_inward_nm` are explicit direct-command magnitudes; torque is neutral during holds and its sign follows the live-IK wheel-speed direction. Wheel position hold is feedback-initialized.

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

For a real run, first copy and review a profile and create the run directory. Then explicitly provide `armed:=true`, `command_topic:=/motor/command`, and `run_dir:=...`. This package must be the only command authority for the test. The real wheel velocity field uses the existing RPM-times-ten encoding; the runner derives physical wheel speed from the live hip IK and converts it before publishing.

## Offline analysis

```bash
python3 $(ros2 pkg prefix kilin_hip_characterization)/share/kilin_hip_characterization/scripts/analyze_tracking.py <run_dir>/command_state_trace.csv
```

The first report summarizes actual-angle RMS/bias/peak error, motor torque peak, and fault samples for each trial/module/phase. The next analysis increments will aggregate the three repetitions, quantify hysteresis/backlash, and rank PID/feedforward candidates.

## Force/contact path

Force-plate trials use this same package and the same run-directory structure. A force estimator always emits a corresponding contact output by thresholding its predicted vertical load; contact threshold, hysteresis, and timing must be versioned with the estimator. Estimator versions will be stored with their exact feature set and complete-trial validation split. Force-plate/Vicon data are offline labels, not a planned terrain-controller input.
# Kilin hip characterization

See [STRATEGY_VERSIONING.md](STRATEGY_VERSIONING.md) for the zero-referenced sequence controller, asymmetric guarded feedforward policy, wheel modes, and the required versioning/provenance convention.
