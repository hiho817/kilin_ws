# Hip characterization strategy and versioning

This package is the shared unit-test package for (1) hip tracking/PID/feedforward and (2) later force/contact estimation.  It does not command the terrain planner or alter the CoM estimator.

## Safety model

- `actual_hip_angle_rad = motor_position + position_diff` is retained as an observed/recorded transmission measurement.
- Strategy `1.1.0` introduced raw-`motor_position` tracking. Strategy `2.6.0` retains that position reference: `position_diff` never changes a motor-position target. It is used only as an observed loading signal by the optional bounded angle-difference lift-assist FF policy and PID schedule.
- An armed run performs a controlled move from fresh motor feedback to configured state A at `startup_move_speed_rad_s`, with zero FF and wheels rest.
- Every unit test follows the explicit sequence, then rests all motors, reads feedback, and moves active hips to `recovery_position_deg` before completion.
- Fresh motor state, torque, motor-error and raw-motor tracking limits abort the run. Reconstructed actual angle is recorded for analysis, not used by this controller revision.
- Steering is position-held at zero.  Hub position mode captures the measured hub position at startup, avoiding an unintended move to position zero.

## Motion modes

`two_state_cycle` alternates `state_a_deg` and `state_b_deg`. With three repetitions, `(0, 45)` is `0→45→0→45→0→45→0`, and `(15, 45)` is `15→45→15→45→15→45→15`. `outward_sequence` accepts magnitudes such as `[0, 10, 20, 30, 20, 30]` and returns to its first state. Positive magnitude maps to front hips negative and rear hips positive, so the same profile works for front/rear/left/right selections.

`per_module_two_state_cycle` uses four-value `module_state_a_deg` and
`module_state_b_deg` lists in fixed `A,B,C,D` order.  It keeps every active
hip under the same position PID, safety checks, and FF policy, but lets only a
selected module move.  For example, A can cycle 40→50→40 while B/C/D receive
continuous 45-degree position commands by setting A-state `[40,45,45,45]`
and B-state `[50,45,45,45]`.  The front-negative/rear-positive mapping is
still applied internally; YAML values are positive physical magnitudes.

All A→B and B→A trajectory distance runs at the requested `hip_speed_rad_s`.
There is no separate static-release trajectory. The optional near-zero
breakaway policy is evaluated only during those normal moving phases.

## Feedforward policy

`feedforward_mode: static_breakaway` preserves the legacy error/velocity-gated
breakaway policy. `feedforward_mode: angle_diff_lift_assist` is the Strategy
2.6.0 loading policy. It ignores all `static_breakaway_*` values, normalizes
each module's `position_diff` so positive means lift, and selects a physical
inward request from the support, blend, or lifted region. Positive physical
inward maps to positive motor torque for A/B and negative motor torque for
C/D. The policy clamps every target to its outward 0–90° motor-frame range and
produces zero lift-assist FF outside that range. It is active only in ordinary
test holds/moves, never startup or recovery. Its two difference thresholds
also form a lift latch, and optional apply/release rate limits bound torque
changes through lift and recontact transitions.

Strategy `2.6.0` can additionally schedule PID gains from that same
instantaneous normalized difference. When
`lift_assist_pid_schedule_enabled: true`, each of Kp, Ki, and Kd is linearly
interpolated between its support and lift values across the two thresholds.
Independent support-to-lift/lift-to-support limits for Kp/Ki/Kd then bound the
commanded-gain change toward those instantaneous values. Their selection follows
the loading-zone transition rather than the numerical gain sign. A zero
directional rate retains a direct change in that transition. This schedule has no dwell, latch, or time accumulation. It operates only during normal test holds/moves; startup,
recovery, rest, completion, and abort retain the global gains. It leaves both
the raw motor-position target and the lift-assist FF calculation unchanged.

Strategy `1.9.1` has no fixed direct hip FF or directional gain parameters.
The selected outward/inward step or exponential schedule is the sole asymmetric
amplitude term in the nonlinear near-zero FF equation. The raw tracking error,
mapped by front/rear module sign, selects the outward/inward schedule; its raw
sign determines corrective torque polarity. A positive schedule value assists
that error sign and a negative value opposes it. No gearbox conversion is
applied.

For `static_breakaway`, FF is disabled during startup/holds/recovery, when the
policy is disabled, when the commanded angle is outside its hard window, or
after the selected-velocity release latch fires. There is no hidden 0.01-rad
error deadband.
The error sign mapped by the module convention selects outward/inward
parameters, and its raw sign selects corrective torque polarity.
`max_abs_hip_ff_torque_nm` remains the dedicated 200-command clamp.

Strategy 1.9.1 retains explicit `filtered`/`raw` velocity-source selection with
`raw` as the default, schedule-only asymmetry, and `none`/`sine` angle shaping.
It supports a three-step or exponential dwell schedule, error
hysteresis, a linear error gate, binary velocity gate, release latch, and hard angle window.
When enabled it is the complete hip FF only in normal moving phases; otherwise
hip FF is zero.
conditions. Hubs remain rest during startup, holds, recovery, completion, and
abort. During a hip-motion phase it uses the live IK relation
`wheel_rate = -L*cos(commanded_hip)*hip_rate/R`; speed mode uses the computed
rate, while torque-assist mode uses a fixed signed configured torque: positive
assists that wheel-rate direction and negative opposes it. Rest, brake, speed, torque, and
feedback-captured position hold will be separately versioned conditions;
brake and position hold are not implemented in 1.9.1.

## Version rule

Every runnable YAML must set both a descriptive immutable `strategy_name` (for example `phase_a_two_state_baseline`) and numeric `strategy_version` (for example `2.6.0`). Change the version whenever sequence shape, guard logic, FF policy, wheel policy, gains, control reference, or analysis definition changes. `1.0.0` was the position-diff-compensated controller; `1.1.0` is the raw-motor controller with hubs forced rest; `1.2.0` adds speed-IK and fixed-torque wheel assist; `1.2.1` permits signed fixed wheel torque; `1.2.2` permits signed hip FF; `1.3.0` added the nonlinear breakaway policy; `1.3.1` removes its separate trajectory phase and adds the hard angle window; `1.4.0` added raw/filtered velocity-source selection; `1.5.0` removed fixed hip FF keys and restricted angle shaping to none or sine; `1.5.1` removes directional gains so asymmetry resides solely in the outward/inward schedules; `1.5.2` selects those schedules from error direction mapped by module convention; `1.6.0` replaces the velocity fade with a binary velocity gate; `1.7.0` replaces the cubic error ramp with a linear error ramp while retaining `static_breakaway_error_enable_rad`, `_full_rad`, and `_disable_rad`; `1.8.0` adds speed-IK hub-travel validity recording and exclusion thresholds; `1.9.0` adds per-module two-state references; `1.9.1` removes the undocumented 0.01-rad early FF return; `2.0.0` adds bounded angle-difference lift assist, normalized front/rear signs, 0–90° outward target trimming, and held-leg eligibility; `2.0.1` interprets a zero lift-policy maximum as the global FF limit, enabling progressive lift-start tuning; `2.1.0` latches lift across the blend region and adds independent applied-FF rise/release rate limits; `2.2.0` adds an optional direct, no-state PID schedule that linearly blends support/lift Kp, Ki, and Kd from the normalized angle difference; `2.3.0` adds optional symmetric Kp/Ki/Kd slew limits; `2.4.0` introduced support-to-lift/lift-to-support semantics; `2.5.0` uses concise `kp/ki/kd_to_lift/support_rate_per_s` profile names for those independent limits; `2.6.0` adds independent upward accumulation and downward decay rates for stored lift strength, while keeping applied FF transition limits separate. Do not overwrite an executed profile: copy it under a new dated run directory.

The runner records the version and control reference in `trial_manifest.yaml`. Each row of `command_state_trace.csv` records phase, trial, commanded/observed motor position, position difference, reconstructed actual hip angle, torque, hub feedback, lift-assist state, scheduled PID targets and commanded gains, and the current speed-IK travel check. `hub_travel_summary.csv` provides the final per-module/per-stroke command travel, feedback travel, ratio, and validity. Any invalid speed-IK stroke invalidates that unit for wheel-condition analysis. The later force/contact estimator must record its estimator version in the same run manifest and produce contact from the force estimate plus an explicitly recorded threshold.
