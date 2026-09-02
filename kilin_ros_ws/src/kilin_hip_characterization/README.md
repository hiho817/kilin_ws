# Kilin Hip Characterization

One ROS 2 package for low-level hip-transmission characterization, PID and bounded feedforward tuning, and later force/contact experiment support. It replaces `kilin_csv_control` only for these experiments: it directly fills every `MotorCmd` field needed for PID and feedforward torque.

## Safety contract

- It starts **disarmed** and publishes to `/kilin/hip_characterization/command_preview`, never `/kilin/motor_cmd_raw`.
- Real actuation requires both `armed:=true` and an explicit real command topic.
- It sends nothing until `/motor/state` is fresh, then captures current hip and hub feedback as its starting state.
- New-recording convention is `actual_hip_angle_rad = motor_position + position_diff`.
- Strategy **1.9.0** commands the raw motor-position reference directly. `position_diff` is not fed back into the position or FF loop.
- `actual_hip_angle_rad` is still reconstructed and recorded for offline transmission/force analysis. Safety aborts on stale state, non-finite state, motor error code, configured hip-torque bound, or configured raw-motor tracking bound. Abort sends all motors to rest.

The package does not modify `kilin_com_estimator`, the terrain planner, or FAST-LIO2.

## Phase-A campaign

The runner performs a two-state cycle for each repetition:

```text
state-A hold → constant-speed move to state B → segment hold
→ constant-speed return to state A → recovery
```

There is no separate static-release phase. Worm-gear breakaway assistance, if
enabled, is evaluated only as a bounded part of the ordinary A→B or B→A move.
The trace records the complete policy-generated FF command and its internal
breakaway contribution.

The initial profile is `config/initial_screening.yaml`. For experiments, keep
one direct `profile.yaml` or one master `master.yaml` in the dated log
directory. A master file supplies defaults plus named unit-test overrides; no
cells are generated beforehand. The runner writes the fully resolved profile
only after each unit is executed, as experiment evidence.

Every YAML carries `strategy_name` and numeric `strategy_version`; changing a
control policy requires a new name/version before an experiment is run.

### Wheel modes

Strategy **1.9.0** has one explicit `wheel_mode`, recorded in the run manifest
and trace. Hubs are always `rest` during startup, both hold phases, recovery,
completion, and abort. Wheel commands are active only during the normal A→B
and B→A moves.

| Future mode | Hub command during hip test | Intended use and safety rule |
| --- | --- | --- |
| `rest` | Hub motor mode `rest`; no active velocity or torque command. | Baseline hip-only test. |
| `speed_ik` (or alias `speed`) | Hub velocity is calculated live from the commanded hip trajectory at every control tick. | The wheel follows hip motion kinematically; it is never a fixed speed. The command field uses the established RPM-times-ten unit: `hub.velocity = wheel_rate_rad_s × 60 × 10 / (2π)`. |
| `torque_assist` (or alias `torque`) | Hub torque magnitude is a fixed signed configuration value. The IK wheel rate chooses the reference direction. | Positive values assist `speed_ik`; negative values oppose it. The torque magnitude is **not** calculated from IK. Outward and inward values may differ and are clamped. |
| `brake` / `position_hold` | Not implemented in 1.9.0. | A profile using either is rejected before commanding hardware. Position hold will require feedback-captured position before it is introduced. |

For `speed_ik`, IK supplies both the live wheel-rate magnitude and direction.
For `torque_assist`, IK supplies **only the reference direction**; the signed
magnitude comes unchanged from the profile. A positive configured torque uses
that direction, while a negative value reverses it. The direction comes from the Jack hip-test
kinematic convention, evaluated from the live commanded hip angle and hip
rate. In the simple planar model this is
`wheel_rate = -L × cos(commanded_hip_angle) × hip_rate / R`, where `L` is the
hip-to-wheel distance and `R` is wheel radius. The torque sign must equal the
sign of that computed wheel rate; it must not be a constant "forward" sign.
Near zero wheel rate, `wheel_rate_deadband_rad_s` sends hub `rest` to prevent
sign chatter. `max_abs_hub_torque_nm` clamps the fixed torque command. Every
trace row records hub mode, commanded hub velocity (RPM×10), and commanded hub
torque, so the bag and compact trace can be cross-checked after every run.

| Parameter | Units / default | Exact effect |
| --- | --- | --- |
| `wheel_mode` | string / `rest` | `rest`, `speed_ik`/`speed`, or `torque_assist`/`torque`. Any other value rejects the run before commands are sent. |
| `wheel_torque_outward_nm` | signed direct hub-torque command / `0` | Fixed torque for an outward/lowering hip movement. Positive assists the IK wheel direction; negative opposes it. Used only by torque mode. |
| `wheel_torque_inward_nm` | signed direct hub-torque command / `0` | Fixed torque for an inward/raising hip movement. Positive assists the IK wheel direction; negative opposes it. Used only by torque mode. |
| `max_abs_hub_torque_nm` | direct hub-torque clamp / `20` | Upper bound applied to either configured torque magnitude. |
| `hip_to_wheel_m` | m / `0.260` | `L` in the live speed-IK relation. |
| `wheel_radius_m` | m / `0.051` | `R` in the live speed-IK relation. |
| `wheel_rate_deadband_rad_s` | rad/s / `0.02` | Below this computed wheel rate, the hub is set to rest. |
| `hub_travel_validation_enabled` | bool / `true` | Enables the speed-IK data-validity check. It records invalid wheel strokes but does not abort the remaining hip unit test. |
| `hub_travel_ratio_min` | ratio / `0.75` | Smallest allowed feedback-travel/commanded-travel ratio. Must be positive. |
| `hub_travel_ratio_max` | ratio / `1.25` | Largest allowed feedback-travel/commanded-travel ratio. Must be at least the minimum. |
| `hub_travel_min_command_rad` | rad / `0.25` | Do not score a segment with less accumulated commanded wheel travel than this. |

#### Speed-IK wheel-travel validity

The hardware feedback currently reports hub velocity as zero even while the hub
is commanded in velocity mode, so velocity feedback is not used to judge a
wheel condition. Strategy 1.9.0 instead compares the feedback hub-position
change with the integrated, sent speed command for every module and every
A→B/B→A stroke:

$$
r = \frac{q_{\mathrm{hub,end}}-q_{\mathrm{hub,start}}}
{\int \dot q_{\mathrm{hub,command}}\,dt}.
$$

For `wheel_mode: speed_ik`, a stroke is valid only when its commanded travel
exceeds `hub_travel_min_command_rad` and
`hub_travel_ratio_min <= r <= hub_travel_ratio_max`. This catches both
under-travel and an opposite-sign response. It is a **data-quality flag**, not
a live closed-loop wheel controller and not a hardware fault abort: the runner
continues the unit test so the evidence remains available, prints a clear
warning, and marks the affected row in `hub_travel_summary.csv` as `valid: 0`.

Exclude the complete unit-test run from any claim about wheel assistance or
wheel motion if any of its scored module-strokes is invalid. A run with
`wheel_mode: rest` or `torque_assist` has no commanded speed integral and is
not scored by this diagnostic.

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
| `strategy_version` | numeric string | Revision of that strategy, e.g. `1.9.0`, recorded in the manifest. Use `1.9.0` for this raw-motor, wheel-mode-capable controller; do not reuse `1.0.0` compensated-run profiles. |

### Test selection and geometry

| Parameter | Type / default | Exact effect |
| --- | --- | --- |
| `active_modules` | string list / `[A,B]` | Hips selected for the unit. `A,B` are front; `C,D` rear; `A,C` left; `B,D` right. Inactive hips are commanded to physical zero during position phases. |
| `motion_mode` | `two_state_cycle` / `two_state_cycle` | `two_state_cycle` gives every active hip the shared `state_a_deg` and `state_b_deg` values. `per_module_two_state_cycle` instead requires both four-value module-state lists below. |
| `repetitions` | integer / `3` | Number of complete A→B→A cycles before recovery. Must be at least one; use three or more for analysis. |
| `state_a_deg` | degrees / `0` | Initial and return **motor-position reference** magnitude. The runner maps front hips to negative and rear hips to positive references. This is not corrected by `position_diff`. |
| `state_b_deg` | degrees / `45` | Other motor-position reference magnitude for the same mapping. `(0,45)` is baseline; `(15,45)` deliberately starts from a nonzero reference. |
| `module_state_a_deg` | four degrees / none | Only for `per_module_two_state_cycle`: state-A magnitudes in exact `[A,B,C,D]` order. All four entries are required even if a module is inactive. |
| `module_state_b_deg` | four degrees / none | Only for `per_module_two_state_cycle`: state-B magnitudes in exact `[A,B,C,D]` order. A module holds its commanded reference when its A and B entries are equal. |
| `startup_move_speed_rad_s` | rad/s / `0.1` | Smooth position move from current measured **motor position** to state A before the unit test. Hip FF is forced to zero and wheels rest. |
| `recovery_position_deg` | degrees / `0` | Active-hip motor-position reference reached after every unit test. |
| `recovery_move_speed_rad_s` | rad/s / `0.1` | Smooth state-A/recovery move speed after the all-motor rest interval. |
| `recovery_rest_s` | seconds / `1.0` | Rest interval between the final cycle and the recovery move; feedback continues to be read. |

#### Example: move A while B/C/D actively hold 45 degrees

Use this when A is the measured hip and the other three hips must remain under
the normal controller, rather than being disabled or sent to zero:

```yaml
strategy_name: a_40_50_with_bcd_45_hold
strategy_version: 1.9.0
active_modules: [A, B, C, D]
motion_mode: per_module_two_state_cycle
module_state_a_deg: [40.0, 45.0, 45.0, 45.0]
module_state_b_deg: [50.0, 45.0, 45.0, 45.0]
repetitions: 5
hip_speed_rad_s: 0.2
```

The fixed lists are always `[A,B,C,D]`, not the order of `active_modules`.
Internally, the positive magnitude convention makes A and B command negative
motor references and C and D positive references.  During each dynamic phase,
A receives the 40→50 or 50→40 reference; B/C/D receive a continuous 45-degree
position reference with the same `kp`, `ki`, `kd`, safety checks, trace
logging, and configured FF policy.  Their commanded hip rate is zero, so
speed-IK or torque-assist wheel commands are also zero/rest for B/C/D.  If a
holding hip develops enough motor-position error to meet the configured FF
error gate, it may receive the same guarded corrective FF as A; that is active
control, not a disabled leg.

### Hip trajectory and PID

| Parameter | Type / default | Exact effect |
| --- | --- | --- |
| `hip_speed_rad_s` | rad/s / `0.2` | Constant trajectory speed for the entire A→B and B→A moves. It must be positive. |
| `start_zero_hold_s` | seconds / `2.0` | Position hold at state A after startup and before every A→B move. The name is historical; it applies even when A is not zero. |
| `segment_hold_s` | seconds / `1.0` | Position hold at state B before returning to A. |
| `kp`, `ki`, `kd` | direct motor gains / `360,0,5` | Hip position-loop gains copied directly into every hip `MotorCmd`. Change only one candidate dimension at a time during PID screening. |

### Near-zero breakaway policy (strategy 1.9.0)

This is the nonlinear worm-gear breakaway policy. It is disabled by default:

    static_breakaway_policy: disabled

Strategy **1.9.0** has no directional gain parameters. Asymmetry comes only
from the separately configured outward and inward schedules $S_d(t_d)$. With
policy `disabled`, hip FF is zero. With policy enabled, this is the complete
hip FF during normal A→B and B→A motion.

For one hip, define the raw motor tracking error

$$e = q_{\mathrm{cmd}} - q_{\mathrm{motor}}.$$

The commanded hip FF is

$$
\tau_{\mathrm{ff}} = \operatorname{clip}_{[-\tau_{\max},\tau_{\max}]}
\left[
\operatorname{sgn}(e)\,A_d(t_d)\,G_e(|e|)\,G_v(|v_s|)\,G_q(|q_{\mathrm{cmd}}|)
\right].
$$

This is exactly the trace field `commanded_hip_ff` and the motor command
torque. There is no second fixed FF term.

| Symbol | Code/profile name | Meaning |
| --- | --- | --- |
| $q_{\mathrm{cmd}}$ | `commanded_motor_rad` | Commanded raw motor position [rad]. |
| $q_{\mathrm{motor}}$ | `motor_position_rad` | Measured raw motor position [rad]. |
| $e$ | `q_cmd - q_motor` | Raw motor tracking error [rad]. `position_diff` is not used. |
| $s_{\mathrm{module}}$ | fixed module convention | `-1` for A/B and `+1` for C/D; maps raw error direction to the physical outward/inward label. |
| $\tau_{\mathrm{ff}}$ | `commanded_hip_ff` | Final bounded hip FF command. |
| $\tau_{\max}$ | `max_abs_hip_ff_torque_nm` | Absolute FF command limit (default 200). |
| $d$ | outward/inward | Physical label selected from error direction: outward when $e\,s_{\mathrm{module}}>0$; A/B use $s=-1$, C/D use $s=+1$. |
| $t_d$ | `static_breakaway_dwell_s` | Per-hip runtime dwell timer [s]. This is a trace/manifest observable, not a YAML setting. It starts at zero when each A→B or B→A move begins and increases by the control-tick interval only while the error and velocity eligibility gates are satisfied. |
| $v_s$ | `breakaway_velocity_used_rad_s` | Selected velocity used by the policy [rad/s]; raw by default, or filtered when configured. |
| $e_{\mathrm{enable}}$ | `static_breakaway_error_enable_rad` | Error magnitude [rad] that arms the hysteretic error state. |
| $e_{\mathrm{full}}$ | `static_breakaway_error_full_rad` | Error magnitude [rad] at which the linear error gate reaches one. |
| $e_{\mathrm{disable}}$ | `static_breakaway_error_disable_rad` | Error magnitude [rad] that disarms the error state. |
| $G_e$ | `error_gate` | Linear error multiplier in $[0,1]$. |
| $G_v$ | `velocity_gate` | Step velocity multiplier: zero or one. |
| $G_q$ | `angle_factor` | Angle multiplier after the hard window check. |
| $A_d$ | `time_amplitude` | Signed direction-dependent amplitude before the gates [direct motor-command units]. |
| $S_d$ | `static_breakaway_steps_*` or `static_breakaway_exp_*` | Direction-dependent schedule [direct motor-command units]. |
| $t_1,t_2$ | `static_breakaway_step_1_s`, `_step_2_s` | Step transition times [s]. |
| $\tau$ | `static_breakaway_exp_tau_s` | Exponential schedule time constant [s]. |
| $v_{\mathrm{gate}}$ | `static_breakaway_dwell_speed_rad_s` | Step velocity-gate threshold [rad/s]. |
| $b_d$ | `static_breakaway_angle_blend_outward/inward` | Direction-dependent angle blend in $[0,1]$. |
| $q_{\min},q_{\max}$ | `static_breakaway_angle_min_deg`, `_max_deg` | Inclusive commanded-angle limits [degrees]. |
| $\operatorname{sgn}(\cdot)$ | implementation operation | Returns $+1$ or $-1$ from its argument; here it uses the sign of $e$, not the motion-direction classification. |
| $\operatorname{clip}$ | implementation operation | Saturates a value to the stated lower and upper bounds. |

The signed asymmetric amplitude is

$$A_d(t_d) = S_d(t_d).$$

The error direction determines both the selected physical schedule and the
corrective torque polarity, but in two different ways. The physical label is
module-dependent:

$$
d=
\begin{cases}
\mathrm{outward}, & e\,s_{\mathrm{module}}>0,\\
\mathrm{inward}, & e\,s_{\mathrm{module}}<0.
\end{cases}
$$

The torque polarity remains the raw error sign:

$$
\operatorname{sgn}(\tau_{\mathrm{ff}})
=\operatorname{sgn}(e)\operatorname{sgn}(S_d),
$$

when the gates are nonzero. Thus outward/inward is **not identical to**
`sign(e)`: the same positive error selects `inward` on a front module but
`outward` on a rear module. This front/rear mapping is why the schedule name is
a physical label rather than simply `positive_error` or `negative_error`.

| Module group | Error sign | Physical label $d$ | Schedule selected | Positive schedule produces |
| --- | --- | --- | --- | --- |
| Front A/B ($s=-1$) | $e>0$ | inward | `*_inward_*` | positive FF |
| Front A/B ($s=-1$) | $e<0$ | outward | `*_outward_*` | negative FF |
| Rear C/D ($s=+1$) | $e>0$ | outward | `*_outward_*` | positive FF |
| Rear C/D ($s=+1$) | $e<0$ | inward | `*_inward_*` | negative FF |

The selected schedule can therefore change whenever error direction crosses
zero. This is intentional in the requested error-direction strategy. The
error-direction reversal also resets the dwell timer and release latch.
`position_diff` participates in neither the physical-label mapping nor torque
polarity.

For `steps`, $S_d$ is the corresponding element of
`static_breakaway_steps_outward_nm` or `_inward_nm`:

$$
S_d(t_d)=
\begin{cases}
S_{d,0}, & t_d < t_1,\\
S_{d,1}, & t_1 \le t_d < t_2,\\
S_{d,2}, & t_d \ge t_2,
\end{cases}
$$

where $t_1$ and $t_2$ are `static_breakaway_step_1_s` and `_step_2_s`. For
`exponential`,

$$S_d(t_d)=S_{d,\mathrm{start}}+
\left(S_{d,\mathrm{peak}}-S_{d,\mathrm{start}}\right)
\left(1-e^{-t_d/\tau}\right),$$

where $\tau$ is `static_breakaway_exp_tau_s`.

Example: with `static_breakaway_exp_start_outward_nm: 40`,
`static_breakaway_exp_peak_outward_nm: 80`, and
`static_breakaway_exp_tau_s: 0.20`, at outward dwell $t_d=0.20$ s:

$$S_{out}(0.20)=40+(80-40)(1-e^{-0.20/0.20})
\approx 65.3.$$

If the error is positive, the gates are all one, and the sine angle factor is
one, the resulting FF is approximately $+65.3$ in the direct motor-command
units. If the same motion has negative error, the result is approximately
$-65.3$; it does not switch from outward to inward merely because the tracking
error changed sign.

The error eligibility arms when $|e|\ge e_{\mathrm{enable}}$ and remains armed
until $|e|\le e_{\mathrm{disable}}$. Its output gate retains the 1.6.0 ramp
range but uses a direct linear slope:

$$
G_e(|e|)=\operatorname{clip}_{[0,1]}
\left(\frac{|e|-e_{\mathrm{enable}}}
{e_{\mathrm{full}}-e_{\mathrm{enable}}}\right).
$$

Thus `static_breakaway_error_enable_rad`, `_full_rad`, and `_disable_rad` all
remain profile parameters. There is no $3u^2-2u^3$ shaping in strategy 1.9.0.
The velocity gate is a step:

$$
G_v(|v_s|)=
\begin{cases}
1, & \texttt{static\_breakaway\_dwell\_speed\_rad\_s}=0\ \text{or}\ |v_s|\le v_{\mathrm{gate}},\\
0, & |v_s|>v_{\mathrm{gate}}.
\end{cases}
$$

Here $v_{\mathrm{gate}}$ is `static_breakaway_dwell_speed_rad_s`. A zero gate
threshold disables this step gate. There is no exponential velocity fade or
velocity-fade power in strategy 1.9.0. $G_q$ is the optional angle factor
described below.

The equation is evaluated only when the commanded raw-motor reference lies in
the inclusive `static_breakaway_angle_min_deg` to `_max_deg` window. Otherwise
$\tau_{\mathrm{ff}}=0$ and dwell does not accumulate. Dwell resets if selected
speed exceeds `static_breakaway_dwell_speed_rad_s` or error direction reverses.
The policy latches off for the remaining move once selected speed reaches
`static_breakaway_release_speed_rad_s`. Finally, `abs(e) < 0.01 rad` forces
$\tau_{\mathrm{ff}}=0$.

#### Gates, dwell, and release latch

- Error hysteresis arms at `static_breakaway_error_enable_rad` and remains
  armed until error falls below `static_breakaway_error_disable_rad`. Between
  `static_breakaway_error_enable_rad` and `_full_rad`, the output rises
  linearly from zero to full scheduled amplitude; there is no cubic shaping.
- A reversal of error sign clears dwell and the release latch. This prevents a
  previous breakaway pulse continuing after overshoot.
- The hard inclusive angle window is checked first:
  `static_breakaway_angle_min_deg <= abs(commanded_motor_deg) <=
  static_breakaway_angle_max_deg`. Outside it, the added policy output is zero
  and dwell does not accumulate. Defaults are 15–90 degrees.
- `static_breakaway_dwell_speed_rad_s` is the step velocity-gate threshold:
  at or below it, $G_v=1$ and dwell can accumulate; above it, $G_v=0$, FF is
  zero, and dwell is cleared. Set it to zero to disable the step gate.
- `static_breakaway_dwell_s` is the resulting per-hip timer, not the speed
  threshold. It is reset at the start of each moving phase, when selected speed
  exceeds the dwell-speed threshold, or when the tracking-error direction
  reverses. It is written to every trace row so the active step or exponential
  amplitude can be reconstructed offline.
- Release speed latches policy output off for the remaining current movement
  phase. Set static_breakaway_release_speed_rad_s to zero to disable it.

Motor velocity comes from `/motor/state`. Set
`static_breakaway_velocity_source: filtered` to use the low-pass value, where
the latest-sample weight is `static_breakaway_velocity_filter_alpha`; set it to
`raw` to use the message velocity directly for the step gate and release.
`raw_hip_velocity_rad_s`, `filtered_hip_velocity_rad_s`, and
`breakaway_velocity_used_rad_s` are all recorded, so the selected source can
be verified after the run.

#### Temporal policies

| `static_breakaway_policy` value | $S_d(t_d)$ schedule | Selection |
| --- | --- | --- |
| `disabled` | Zero hip FF. | Default; no policy influence. |
| `steps` | Three signed plateaus selected by $t_1$ and $t_2$. | Choose this in the profile when you want three timed levels. |
| `exponential` | $S_{start}+(S_{peak}-S_{start})(1-e^{-t_d/\tau})$. | Choose this in the profile for a smooth rise. |

#### Angle shaping inside the hard window

The hard 15–90 degree default window is applied before this optional shaping.
Angle mode `none` gives $G_q=1$. Angle mode `sine` uses
$G_q=(1-b_d)+b_d\sin(|q_{\mathrm{cmd}}|)$, where $b_d$ is the outward or
inward blend. Thus blend zero disables angle scaling and blend one applies the
full sine factor. The two blend parameters let outward and inward motion use
different angle dependence:

    angle_factor = (1-blend) + blend * sin(abs(commanded_motor_angle))

The `static_breakaway_angle_blend_outward` and `_inward` values must be in
$[0,1]$. They select the blend according to the current **error-based physical
label** $d$:

$$
b_d=
\begin{cases}
\texttt{static\_breakaway\_angle\_blend\_outward}, & d=\mathrm{outward},\\
\texttt{static\_breakaway\_angle\_blend\_inward}, & d=\mathrm{inward}.
\end{cases}
$$

They do not select torque sign and do not change the outward/inward schedule.
Those decisions remain respectively $\operatorname{sgn}(e)$ and
$e\,s_{\mathrm{module}}$. The blend only scales the already-selected schedule
amplitude through $G_q$.

| Blend value $b_d$ | Resulting factor $G_q$ | Effect |
| ---: | --- | --- |
| `0.0` | $1$ | No angle dependence; the selected schedule is unchanged. |
| `0<b_d<1` | $(1-b_d)+b_d\sin(|q_{\mathrm{cmd}}|)$ | Partial interpolation between constant amplitude and sine scaling. |
| `1.0` | $\sin(|q_{\mathrm{cmd}}|)$ | Full sine scaling. |

For example, full sine scaling gives approximately $0.259$ at 15°, $0.707$ at
45°, and $1.000$ at 90°. With `blend_outward: 0.5`, outward FF at 45° is
scaled by $(1-0.5)+0.5\times0.707\approx0.854$. With
`blend_inward: 0.0`, inward FF remains unscaled at the same angle. Therefore
the two parameters can represent different gravity/friction dependence for the
two physical labels without altering torque polarity.

The hard angle window is checked before this formula. Below the configured
minimum or above the maximum, the complete policy output is zero regardless of
either blend. Angle mode `none` also forces $G_q=1$, so both blend values have
no effect until `static_breakaway_angle_mode: sine` is selected. Measure torque
versus angle before assuming a gravity model.

#### Static-breakaway parameter reference

| Parameter | Default | Meaning |
| --- | ---: | --- |
| static_breakaway_policy | disabled | disabled, steps, or exponential; invalid values reject the run. |
| static_breakaway_steps_outward_nm | [0,0,0] | Three signed outward step amplitudes. |
| static_breakaway_steps_inward_nm | [0,0,0] | Three signed inward step amplitudes. |
| static_breakaway_step_1_s / step_2_s | 0.10 / 0.30 | Time boundaries; step 2 must be no earlier than step 1. |
| static_breakaway_exp_start_outward_nm / inward_nm | 0 | Signed initial exponential amplitude. |
| static_breakaway_exp_peak_outward_nm / inward_nm | 0 | Signed asymptotic exponential amplitude. |
| static_breakaway_exp_tau_s | 0.20 | Positive exponential time constant. |
| static_breakaway_error_enable_rad | 0.02 | Error enabling breakaway eligibility. |
| static_breakaway_error_full_rad | 0.05 | Error at which the linear FF gate reaches full scheduled amplitude. It must be at least `static_breakaway_error_enable_rad`. |
| static_breakaway_error_disable_rad | 0.01 | Error disabling the gate after enable. |
| static_breakaway_dwell_speed_rad_s | 0.03 | Step velocity-gate threshold. At or below it FF may be nonzero and dwell accumulates; above it FF is zero and dwell clears. Zero disables this gate. |
| static_breakaway_release_speed_rad_s | 0.10 | Speed latching output off; zero disables latch. |
| static_breakaway_velocity_filter_alpha | 0.20 | New motor-speed sample weight, in (0,1]. |
| static_breakaway_velocity_source | raw | `raw` uses `/motor/state` velocity directly; `filtered` uses the low-pass value for the step gate and release latch. |
| static_breakaway_angle_mode | none | `none` or `sine`; sine uses $\sin(|q_{\mathrm{cmd}}|)$. |
| static_breakaway_angle_blend_outward / inward | 0 | Direction-specific blend $b_d$ in $[0,1]$: zero gives no angle scaling, one gives full sine scaling. |
| static_breakaway_angle_min_deg / max_deg | 15 / 90 | Inclusive hard commanded raw-motor reference angle window; max must be at least min. |

#### Progressive safe activation

First confirm the selected PID/wheel controller with policy disabled. Then add
only the step schedule:

    static_breakaway_policy: steps
    static_breakaway_steps_outward_nm: [40.0, 60.0, 80.0]
    static_breakaway_steps_inward_nm: [40.0, 60.0, 80.0]
    static_breakaway_step_1_s: 0.10
    static_breakaway_step_2_s: 0.30
    static_breakaway_angle_mode: none
    static_breakaway_angle_min_deg: 15.0
    static_breakaway_angle_max_deg: 90.0

After measuring a safe plateau range, change one feature at a time: asymmetric
schedule, error hysteresis, step-gate/release thresholds, velocity source, then
angle shaping. Do not change PID or wheel mode in the same campaign. The trace
records commanded hip FF, raw/filtered/selected motor velocity, breakaway FF,
dwell time, and release-latch state; the manifest records policy, schedules,
velocity source, steps, angle mode, and error thresholds.

### Hip FF direction and limit

The near-zero policy is the entire hip FF command. It is issued only in A→B
and B→A motion and is zero in startup, holds, recovery, completion, and abort.
For each hip, the runner uses the raw-motor tracking direction:

```text
e = commanded_motor_position − measured_motor_position
```

If `abs(e) < 0.01 rad`, FF is zero. Otherwise `sign(e)` is the corrective
torque direction. That same error, mapped through the module reference sign
`s` (`−1` for A/B, `+1` for C/D), selects the physical schedule: outward means
`e × s > 0`, otherwise inward. The runner selects the corresponding schedule,
then applies the equation above. Thus positive raw error is inward for a front
module and outward for a rear module; negative raw error reverses those labels.

`max_abs_hip_ff_torque_nm` (default `200`) is the direct-command absolute clamp
on this result. It is independent of the broader measured hip-torque safety
limit. A positive schedule value assists the instantaneous tracking error; a
negative value opposes it. This calculation never uses `position_diff`.

### Safety and observability

| Parameter | Type / default | Exact effect |
| --- | --- | --- |
| `max_state_age_s` | seconds / `0.10` | Maximum acceptable age of `/motor/state`. A stale state rests all motors and aborts. |
| `max_abs_hip_error_rad` | radians / `0.35` | Largest allowed measured-motor-minus-commanded-motor error. A breach aborts. It is deliberately not based on reconstructed actual hip angle in strategy 1.9.0. |
| `max_abs_hip_torque_nm` | feedback units / `400` | Largest allowed absolute hip torque feedback value. A breach aborts. Confirm feedback units on hardware before changing it. |

The runner logs phase changes to the terminal and `launch.log`: startup move,
state-A hold, move to B, hold at B, return to A, recovery rest, recovery move,
complete, or aborted. `command_state_trace.csv` records
the controller pair `commanded_motor_rad`/`motor_position_rad`, alongside
`position_diff_rad` and the observation-only reconstruction
`actual_hip_angle_rad = motor_position + position_diff`.
For speed-IK runs it also records hub position, velocity, torque, mode, error,
and the evolving command/feedback travel ratio. The final per-stroke evidence
is `hub_travel_summary.csv`; use its `valid` column before analysing a wheel
condition.

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

For a real run, first copy and review a profile and create the run directory. Then explicitly provide `armed:=true`, `command_topic:=/motor/command`, and `run_dir:=...`. In strategy 1.9.0, `speed_ik` and `torque_assist` are published only during moving hip phases; hubs are rest otherwise.

## Offline analysis

```bash
python3 $(ros2 pkg prefix kilin_hip_characterization)/share/kilin_hip_characterization/scripts/analyze_tracking.py <run_dir>/command_state_trace.csv
```

The first report summarizes actual-angle RMS/bias/peak error, motor torque peak, and fault samples for each trial/module/phase. The next analysis increments will aggregate the three repetitions, quantify hysteresis/backlash, and rank PID/feedforward candidates.

## Force/contact path

Force-plate trials use this same package and the same run-directory structure. A force estimator always emits a corresponding contact output by thresholding its predicted vertical load; contact threshold, hysteresis, and timing must be versioned with the estimator. Estimator versions will be stored with their exact feature set and complete-trial validation split. Force-plate/Vicon data are offline labels, not a planned terrain-controller input.
# Kilin hip characterization

See [STRATEGY_VERSIONING.md](STRATEGY_VERSIONING.md) for the zero-referenced sequence controller, asymmetric guarded feedforward policy, wheel modes, and the required versioning/provenance convention.
