# Kilin known-terrain controller

This ROS 2 node runs the migrated Version 2 receding-horizon planner online
against two parameterized, already-known one-sided ramps on opposite wheel
tracks. It does not replay CSV
commands and does not use live terrain-map feedback.

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

For `target:=isaac`, the collected Isaac graph intentionally publishes feedback
below the `/kilin/isaac` prefix. The legacy Isaac bridge converts its JointState
into the same `/motor/state` contract used by the real robot. This launch
remaps the controller's simulation clock and the bridge's IMU/JointState
subscriptions to:

- `/kilin/isaac/clock`
- `/kilin/isaac/imu`
- `/kilin/isaac/joint_states` (bridge input)
- `/motor/state` (published by `isaac_bridge` from Isaac JointState)
