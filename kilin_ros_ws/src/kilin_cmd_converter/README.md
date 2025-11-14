# kilin_cmd_converter

Kilin Cmd Converter converts a body Twist (geometry_msgs/Twist) on `/kilin/cmd_vel` into per-module motor commands (`kilin_msgs/MotorCmdStamped`) on `/motor/command`.

This node is intended to be used with the `kilin_joystick` package (which publishes `/kilin/cmd_vel`) or any teleop source. It computes steering angles and hub wheel speeds for a four-module vehicle (A,B,C,D mapped to front-left, front-right, rear-left, rear-right).

## Quick start

Build and source the workspace (from workspace root):

```bash
colcon build --packages-select kilin_cmd_converter
source install/setup.bash
```

Run the node directly:

```bash
ros2 run kilin_cmd_converter kilin_cmd_converter
```

Or include it in a launch file alongside `kilin_joystick` and `joy_node` (see `kilin_joystick/launch/launch.py` for an example).

## Topics

- Subscribes:
  - `/kilin/cmd_vel` (geometry_msgs/Twist) — input Twist: linear.x (forward), linear.y (lateral), angular.z (yaw)
- Publishes:
  - `/motor/command` (kilin_msgs/MotorCmdStamped) — assembled leg commands for modules A..D

## Parameters

The node declares the following parameters (defaults shown):

- `L_base` (double, default 0.60) — wheelbase (m)
- `W_base` (double, default 0.50) — track width (m)
- `R_w` (double, default 0.08) — wheel radius (m)
- `vmax` (double, default 1.0) — maximum linear speed (m/s)
- `wmax` (double, default 2.0) — maximum angular speed (rad/s)

Set parameters via launch or a YAML params file.

## How it works (implementation notes)

- For each module at position r = (Xi, Yi), the node computes the velocity of that module due to the commanded body motion:

  V_module = v_body + ω × r

  or in components used by the node:

  Vx = vx - wz * Yi
  Vy = vy + wz * Xi

- Steering angle is `atan2(Vy, Vx)`.
- Hub (wheel) angular speed is computed as `sqrt(Vx*Vx + Vy*Vy) / R_w`.
- To keep steering within ±90° the implementation normalizes the steering angle into that range by adding/subtracting π when necessary and flips the hub speed sign so the net velocity vector is preserved.

  Note: a more robust approach is to compute hub speed by projecting the module velocity onto the final wheel heading:

  `hub_speed = (Vx*cos(theta) + Vy*sin(theta)) / R_w`

  This projection eliminates the need for an explicit sign flip and is recommended if you refine the node.

## Safety and limitations

- The node clamps incoming vx, vy and wz to `vmax`/`wmax` to avoid generating excessively large commands.
- There is no lowpass filtering or watchdog implemented; if the input Twist stream stops, downstream controllers should handle timeouts or there should be an external safety layer.
- Ensure `R_w` > 0 to avoid division by zero.

## Troubleshooting

- No output on `/motor/command`:
  - Verify the node is running: `ros2 node list`
  - Verify `/kilin/cmd_vel` is being published: `ros2 topic echo /kilin/cmd_vel`

- Steering flips unexpectedly near zero velocities:
  - Consider adding a small deadzone or smoothing.
  - Switch to projection-based hub-speed computation to avoid sign flips.

## Next improvements (suggested)

- Replace the hub speed sign-flip with projection-based computation (see formula above).
- Add runtime parameter updates and validation (e.g., ensure `R_w` > 0 and `vmax` > 0).
- Provide a launch file in this package that demonstrates typical parameters for your robot.

---

Maintainer: see `package.xml` (ian920201)
