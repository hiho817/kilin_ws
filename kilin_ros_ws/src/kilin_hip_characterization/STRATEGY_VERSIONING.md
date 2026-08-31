# Hip characterization strategy and versioning

This package is the shared unit-test package for (1) hip tracking/PID/feedforward and (2) later force/contact estimation.  It does not command the terrain planner or alter the CoM estimator.

## Safety model

- `actual_hip_angle_rad = motor_position + position_diff` is retained as an observed/recorded transmission measurement.
- Strategy `1.1.0` commands and tracks raw `motor_position`; it does not feed `position_diff` into the position target or FF direction. This isolates the worm-gear/PID experiment from angle-difference feedback chatter.
- An armed run performs a controlled move from fresh motor feedback to configured state A at `startup_move_speed_rad_s`, with zero FF and wheels rest.
- Every unit test follows the explicit sequence, then rests all motors, reads feedback, and moves active hips to `recovery_position_deg` before completion.
- Fresh motor state, torque, motor-error and raw-motor tracking limits abort the run. Reconstructed actual angle is recorded for analysis, not used by this controller revision.
- Steering is position-held at zero.  Hub position mode captures the measured hub position at startup, avoiding an unintended move to position zero.

## Motion modes

`two_state_cycle` alternates `state_a_deg` and `state_b_deg`. With three repetitions, `(0, 45)` is `0→45→0→45→0→45→0`, and `(15, 45)` is `15→45→15→45→15→45→15`. `outward_sequence` accepts magnitudes such as `[0, 10, 20, 30, 20, 30]` and returns to its first state. Positive magnitude maps to front hips negative and rear hips positive, so the same profile works for front/rear/left/right selections.

The first outward segment has a smooth, low-distance static-release portion.  All remaining distance then runs at the requested `hip_speed_rad_s`; other segments run at that constant speed.  This separates breakaway friction from dynamic tracking.

## Feedforward policy

There are independent direct motor-command terms: `hip_ff_outward_direct`, `hip_ff_inward_direct`, `hip_ff_static_outward_direct`, and `hip_ff_static_inward_direct`.  Their sign is always computed from the planned motion direction, including front/rear sign reversal.  No gearbox conversion is applied.

FF is disabled during zero/hold states, when desired velocity is too small, when the measured error is opposite to the planned move, and with enable/disable error hysteresis near target.  This avoids sign chatter and unstable assistance when commanded and actual positions are close.  `max_abs_hip_ff_torque_nm` remains a dedicated 200 command clamp.

Wheel speed uses the live IK relation `wheel_rate = -L*cos(commanded_hip)*hip_rate/R`.  In torque mode the torque sign follows that wheel-rate direction.  Rest, brake, speed, torque and captured position-hold are separate wheel conditions.

## Version rule

Every runnable YAML must set both a descriptive immutable `strategy_name` (for example `phase_a_two_state_baseline`) and numeric `strategy_version` (for example `1.1.0`). Change the version whenever sequence shape, guard logic, FF policy, wheel policy, gains, control reference, or analysis definition changes. `1.0.0` was the position-diff-compensated controller; `1.1.0` is the raw-motor controller. Do not overwrite an executed profile: copy it under a new dated run directory.

The runner records the version and control reference in `trial_manifest.yaml`. Each row of `command_state_trace.csv` records phase, trial, commanded/observed motor position, position difference, reconstructed actual hip angle, torque, and error code. The later force/contact estimator must record its estimator version in the same run manifest and produce contact from the force estimate plus an explicitly recorded threshold.
