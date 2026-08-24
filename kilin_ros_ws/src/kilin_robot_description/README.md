# kilin_robot_description

Self-contained URDF for hardware COM estimation. It retains link inertials and
joint kinematics but intentionally omits meshes, visual geometry, and collision
geometry.

The model combines:

- Kilin AMR: `AMRV2_only_nosupport_0323_URDF`
- Kinova Gen3 7-DOF: no vision module and no gripper

The fixed transform from `base_link` to `arm_base_link` is:

- translation: `[0.16826, 0.0, -0.07970]` m
- rotation: `[0.0, 0.0, pi]` rad (RPY)

The arm transform models its physical installation. It is separate from the
controller's `arm_base_yaw_offset_deg`, which maps requested AMR directions to
Kinova joint-1 commands.

Use `urdf/kilin_gen3_hardware.urdf` as the kinematic/inertial source for the
hardware COM estimator. Base attitude is deliberately not encoded here; it must
come from the estimator's orientation source.

