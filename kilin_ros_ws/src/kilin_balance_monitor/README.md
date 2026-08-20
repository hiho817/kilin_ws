# Kilin Balance Monitor

This package evaluates the simulated robot's static stability without sending
commands to the arm or legs. It is intentionally a shadow-mode component for
validating the COM and support-polygon logic before closed-loop control is
enabled.

## Topics

- Subscribes to `/kilin/balance_state` (`kilin_msgs/BalanceStateStamped`)
- Subscribes to `/kilin/stair_phase` (`kilin_msgs/StairPhaseStamped`)
- Publishes `/kilin/stability_state` (`kilin_msgs/StabilityStateStamped`)

The point order is fixed as front-left, front-right, rear-left, rear-right.
The support calculation uses nominal wheel-bottom points from robot geometry;
contact sensor state does not affect validity or the polygon. For phases 1
through 4, the corresponding future swing leg is removed before the support
polygon is calculated. Phase 0 uses all four finite geometric points.

The published `stability_margin` is the shortest signed XY distance from the
COM projection to a support-polygon edge. A positive value is inside the
polygon. `safe` becomes true when that value is at least `safe_margin_m`.
`correction_direction` is the inward unit normal of the closest or most
violated edge.

## Simulation

Build and source the workspace, then start the monitor:

```bash
colcon build --packages-up-to kilin_balance_monitor kilin_stair_controller
source install/setup.zsh
ros2 launch kilin_balance_monitor balance_monitor.launch.py
```

In Isaac Sim's Script Editor, run:

```python
exec(open(
    "/home/brl--pc5/isaacsim_project/isaac_tools/run/"
    "run_balance_state_publisher.py"
).read())
```

The Isaac script reuses the existing combined Kilin + Kinova mass-weighted COM
calculation. Each nominal support point is the corresponding wheel-link center
minus the wheel radius along world Z. Contact sensor fields are still published
for validation but are not consumed by the monitor. It publishes at 30 Hz by
default.
It may run alongside the Kinova joint-trajectory adapter, but it replaces the
older COM visualizer in the Isaac runtime-manager slot.

Inspect the result with:

```bash
ros2 topic hz /kilin/balance_state
ros2 topic echo /kilin/stair_phase
ros2 topic echo /kilin/stability_state
```

The default safety margin can be changed in
`config/balance_monitor.yaml`. This node remains observation-only regardless
of the configured value.
