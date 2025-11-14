# kilin_joystick

Kilin Joystick is a small ROS 2 node that reads a joystick (via the `joy` package) and publishes a 2D body velocity (`geometry_msgs/Twist`) on `/kilin/cmd_vel` at a fixed 100 Hz rate. It's intended to be used together with `kilin_cmd_converter` which converts `/kilin/cmd_vel` into per-module motor commands.

This package provides:
- `kilin_joystick` node (C++): reads `/joy` and publishes `/kilin/cmd_vel`.
- A composable launch file (`launch/launch.py`) that starts `joy_node`, `kilin_joystick`, and `kilin_cmd_converter` together.

## Features
- Deadzone handling and rescaling so joystick output is mapped smoothly from deadzone..1.0 to 0..1.0
- Fixed-rate publish at 100 Hz (timer-driven)
- Configurable linear/angular maxima and axis mapping for different platforms (PC vs Orin)
- Clean shutdown handling: zero command published on exit

## Topics
- Subscribes:
  - `/joy` (sensor_msgs/Joy) — from `joy_node` or another joystick driver
- Publishes:
  - `/kilin/cmd_vel` (geometry_msgs/Twist) — linear.x, linear.y, angular.z

The `kilin_cmd_converter` (usually launched alongside) subscribes to `/kilin/cmd_vel` and publishes `/motor/command`.

## Parameters
The node declares the following ROS parameters (and sensible defaults):

- `vmax` (double, default 0.8) — maximum linear speed (m/s) mapped from joystick full deflection
- `wmax` (double, default 1.5) — maximum angular speed (rad/s) mapped from joystick full deflection
- `deadzone` (double, default 0.15) — joystick deadzone (0..1). Values inside ±deadzone map to 0. Outside are rescaled.
- `omega_axes` (int, default 2) — index of joystick axis to use for angular velocity (right stick). The `launch` file sets this automatically depending on `device` argument.

You can set parameters either via launch or with a YAML file.

## Common parameters explained

This section explains the most commonly tuned parameters, what they control, units, and recommended values.

- `vmax` (double)
  - What: maps the joystick's full forward/back deflection to a linear speed in m/s for `twist.linear.x` and `twist.linear.y`.
  - Units: meters/second (m/s).
  - Default: 0.8
  - Recommended: start with a conservative value (0.2–0.8) on a real robot and increase once you verify safe behavior in a test area.
  - Notes: If you want different forward vs lateral limits, you can launch multiple node instances or modify the code to handle separate limits.

- `wmax` (double)
  - What: maps the joystick's full rotation deflection to angular yaw rate `twist.angular.z` in rad/s.
  - Units: radians/second (rad/s).
  - Default: 1.5
  - Recommended: for wheeled robots, 0.5–2.0 is typical. Lower `wmax` if the robot is prone to tipping or has slow steering actuators.

- `deadzone` (double)
  - What: defines a symmetric deadzone around zero for joystick axes. Inputs whose absolute value <= deadzone are treated as zero. Values outside the deadzone are rescaled so the output still reaches full range at stick extremes.
  - Units: normalized joystick axis units (0.0–1.0).
  - Default: 0.15
  - Recommended: 0.05–0.2. Increase if your controller has noisy neutral readings or the robot drifts when the sticks are released. Keep it small enough to allow precise slow motion.
  - Example: with `deadzone=0.15`, an axis reading of 0.2 becomes (0.2-0.15)/(1-0.15)=~0.058 after scaling.

- `omega_axes` (int)
  - What: index into the `sensor_msgs/Joy::axes[]` array used to read the right-stick (yaw/omega) axis.
  - Units: integer axis index (0-based)
  - Default: 2 (Orin mapping), 3 when launched with `device:=pc` in the provided launch file.
  - Recommended: inspect `ros2 topic echo /joy` for your controller to find the correct index, then set this parameter via launch if needed.

Tuning advice
- Start with small `vmax`/`wmax` values and a reasonable `deadzone` to ensure the robot is controllable at low speeds.
- Use the launch file `device` argument to choose a mapping close to your hardware, then tweak `omega_axes` if the yaw control is on a different axis.
- For precise slow movement, reduce `vmax` and use smaller deadzone values while testing.


## Build
From your ROS2 workspace root:

```bash
colcon build --packages-select kilin_joystick
source install/setup.bash
```

If you want to build the helper `kilin_cmd_converter` at the same time, include it in the build (or `--packages-select kilin_joystick kilin_cmd_converter`).

## Run

Run the node directly (after building and sourcing):

```bash
ros2 run kilin_joystick kilin_joystick
```

Recommended: use the provided launch file which starts `joy_node`, the joystick node, and the converter together:

```bash
ros2 launch kilin_joystick launch/launch.py
```

The launch file accepts a `device` launch argument to select axis mapping behavior. Examples:

```bash
# Default mapping (Orin mapping)
ros2 launch kilin_joystick launch/launch.py

# PC mapping (different axis index for omega)
ros2 launch kilin_joystick launch/launch.py device:=pc
```

## Axis mapping notes
- By default the launch file sets `omega_axes` to `2` (suitable for Orin controllers). If you set `device:=pc` the launch will set `omega_axes=3`.
- The joystick mapping in the node uses:
  - left stick: axes[1] → vx (forward/back)
  - left stick: axes[0] → vy (left/right)
  - right stick: axes[omega_axes] → ω (yaw)

If your controller uses a different layout, either modify the `device` launch argument or set `omega_axes` directly when launching.

## Behavior and implementation details
- The node applies a deadzone function which keeps small stick motion from causing small outputs. Values outside the deadzone are rescaled so full travel still maps to full speed.
- Publishing is timer-driven at 100 Hz so downstream nodes receive a steady stream of commands.
- On process termination the node publishes a zero Twist to stop the robot.

## Troubleshooting
- No `/joy` messages: make sure `joy_node` (from the `joy` package) is running and that your joystick device is accessible.
- If commands are inverted or axes swapped: check the physical controller mapping with `ros2 topic echo /joy` and adjust `omega_axes` or axis indices in code/launch to match your device.
- If the robot doesn't move: verify that `/kilin/cmd_vel` is being published and that `kilin_cmd_converter` (or your custom controller) subscribes to it:

```bash
ros2 topic echo /kilin/cmd_vel
ros2 topic echo /motor/command  # if converter is running
```

## Next improvements
- Add a YAML parameters file and a launch argument to load it for easy tuning.
- Add button mapping for toggles (enable/disable motors, change speed mode, logging triggers).
- Add a small calibration tool to print and persist axis mappings per controller model.

---

Maintainer: ian920201 (see `package.xml`)
