#!/usr/bin/env python3
"""
Generate validated Kilin stair-gait CSV templates.

The first implementation deliberately supports only the validated 0.10 m rise,
0.35 m run gait.  It can repeat the validated 35--52 s middle-stair cycle while
preserving continuous hip angles, then optionally append the validated final
approach and stage-3 templates through t=75 s.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, Sequence


CSV_HEADER = (
    "time",
    "a_hip_pos", "a_steering_pos", "a_hub_vel", "a_hub_mode",
    "b_hip_pos", "b_steering_pos", "b_hub_vel", "b_hub_mode",
    "c_hip_pos", "c_steering_pos", "c_hub_vel", "c_hub_mode",
    "d_hip_pos", "d_steering_pos", "d_hub_vel", "d_hub_mode",
    "arm_phase",
)

VALIDATED_RISE_M = 0.10
VALIDATED_RUN_M = 0.35
VALIDATED_CENTER_TO_FIRST_RISER_M = 0.63

# Robot dimensions mirrored from /home/brl--pc5/kilin_stairs/cars.py.  Keeping
# them explicit here makes the installed ROS executable self-contained.
BODY_LENGTH_M = 0.555
PIVOT_OFFSET_M = 0.0375
LEG_LENGTH_M = 0.255
WHEEL_RADIUS_M = 0.0525
OUTWARD_ANGLE_DEG = 40.0
# alex_v2 used 90 degrees as a deliberately conservative swing pose, not as a
# measured just-clears-the-riser angle.  Keep that pose until geometry says the
# wheel rim would no longer clear the tread.
MINIMUM_LIFT_ANGLE_DEG = 90.0
LIFT_CLEARANCE_M = 0.010
REFERENCE_TRANSITION_ANGLE_DEG = 20.0
# Successful 0.10/0.35 and 0.12/0.35 runs had about 10--11.5 degrees between
# the front and rear hip stances before the long FR swing.  The generated
# stance uses half the transition angle for that separation, so clamp the
# transition to twice this empirical minimum.
MINIMUM_FRONT_REAR_STANCE_SEPARATION_DEG = 10.0
HUB_COMMAND_TO_RPM = 0.1

# Rise-dependent corrections below are experimental calibrations, not robot
# geometry.  They have only been validated through the successful 0.12/0.35
# gait, so do not extrapolate them beyond that rise.
MAX_CALIBRATED_CORRECTION_RISE_M = 0.12
MAX_CALIBRATED_ENTRY_RUN_M = 0.40
SHORT_RUN_CORRECTION_START_M = 0.35

# Open-loop entry calibration from the 0.12 m-rise / 0.35 m-run rosbag.  At the
# second FR swing, the wheel center was 0.056 m behind the successful 0.10 m
# baseline.  Apply that measured setback to the hub-only approach immediately
# before the second FR lift.  Keeping this as an explicit gain makes the
# calibration auditable and preserves the validated 0.10 m CSV byte-for-byte.
ENTRY_APPROACH_RISE_GAIN = 2.8

# In the 0.08/0.40 rosbag the second FR wheel center settled 36 mm before the
# second riser, leaving only 16 mm of wheel overlap.  A 20 mm wheel-center
# landing reserve requires about 56 mm more travel.  Apply that correction to
# the entry approach (the same safe segment used by the rise correction), and
# scale it from the validated 0.35 m run through the measured 0.40 m run.
# Shorter runs keep their existing open-loop clearance, and longer runs are
# capped until they are measured rather than extrapolating this calibration.
ENTRY_APPROACH_RUN_GAIN = 1.1

# Correct-dimension 0.11/0.33 log02 showed only 11 mm of FR reserve on tread 2;
# the following hub drive was blocked at the edge, leaving RR 66--79 mm short
# of tread 1.  Add up to 15 mm while FR is fully lifted (the hub=125 segment),
# interpolating below the successful 0.35 m baseline and capping at 0.33 m.
# The existing 0.12/0.30 correction remains at the same 15 mm cap.
SHORT_RUN_FR_LANDING_GAIN = 0.75
MAX_SHORT_RUN_FR_LANDING_CORRECTION_M = 0.015

# In the same 0.12/0.35 test, phase 3 reached only 13.86 mm at full arm
# extension because the RR--FL support edge was about 1.14 mm too close to the
# COM.  Moving the planted RR wheel rearward by roughly 2.4 mm is sufficient;
# a +2 degree rear landing correction gives about 6--7 mm of wheel-center
# reserve.  Scale from zero at the validated 0.10 m rise.
REAR_LANDING_RISE_GAIN_DEG_PER_M = 100.0

# The 0.12/0.35 log04 gait succeeded after adding -50 hub command during the
# two-second FL transfer toward the second tread.  This is about 55 mm of ideal
# retreat and keeps the descending FL wheel behind the third riser.  Scale the
# command from zero at the validated 0.10 m rise.
MIDDLE_FL_RETREAT_COMMAND_PER_M = 2500.0

ZERO4 = (0.0, 0.0, 0.0, 0.0)
MODE0 = (0, 0, 0, 0)
MODE5 = (5, 5, 5, 5)


@dataclass(frozen=True)
class GaitRow:
    time: float
    hips: tuple[float, float, float, float]
    steering: tuple[float, float, float, float]
    hub_velocity: tuple[float, float, float, float]
    hub_mode: tuple[int, int, int, int]
    arm_phase: int

    def csv_values(self) -> list[float | int]:
        values: list[float | int] = [self.time]
        for leg in range(4):
            values.extend(
                (
                    self.hips[leg],
                    self.steering[leg],
                    self.hub_velocity[leg],
                    self.hub_mode[leg],
                )
            )
        values.append(self.arm_phase)
        return values


def _row(
    time: float,
    hips: Sequence[float],
    hub_velocity: Sequence[float] = ZERO4,
    hub_mode: Sequence[int] = MODE5,
    arm_phase: int = 0,
    steering: Sequence[float] = ZERO4,
) -> GaitRow:
    return GaitRow(
        time=float(time),
        hips=tuple(float(value) for value in hips),  # type: ignore[arg-type]
        steering=tuple(float(value) for value in steering),  # type: ignore[arg-type]
        hub_velocity=tuple(float(value) for value in hub_velocity),  # type: ignore[arg-type]
        hub_mode=tuple(int(value) for value in hub_mode),  # type: ignore[arg-type]
        arm_phase=int(arm_phase),
    )


@dataclass(frozen=True)
class CyclePoint:
    relative_time: float
    hip_offset: tuple[float, float, float, float]
    hub_velocity: tuple[float, float, float, float] = ZERO4
    hub_mode: tuple[int, int, int, int] = MODE5
    arm_phase: int = 0


@dataclass(frozen=True)
class GeometryAngles:
    """Independent angles from which every continuous hip target is built."""

    outward_deg: float
    lift_deg: float
    transition_deg: float
    rear_landing_correction_deg: float
    middle_fl_retreat_command: float


CYCLE_DURATION_SEC = 20.0


def build_entry_rows(angles: GeometryAngles) -> tuple[GaitRow, ...]:
    """Build state 1 from geometry angles instead of copied hip literals."""
    outward = angles.outward_deg
    lift = angles.lift_deg
    front_landing = 360.0 - outward
    front_next_support = 360.0 + outward
    return (
        _row(0, (0, 0, 0, 0)),
        _row(3, (-outward, -outward, outward, outward), hub_mode=MODE0),
        _row(6, (-outward, -lift, outward, outward), arm_phase=2),
        _row(8, (-outward, -lift, outward, outward), (125, 125, 125, 125), arm_phase=2),
        _row(9, (-outward, -lift, outward, outward), arm_phase=2),
        _row(11, (0, -outward, outward, outward), (8, 8, 8, 8), arm_phase=2),
        _row(12, (0, -outward, outward, outward)),
        _row(15, (front_landing, -outward, outward, outward), arm_phase=1),
        _row(16, (front_landing, -outward, outward, outward), (100, 100, 100, 100)),
        _row(17, (front_landing, -outward, outward, outward)),
        _row(20, (front_landing, -lift, outward, outward), arm_phase=2),
        _row(22, (front_landing, -lift, outward, outward), (125, 125, 125, 125), arm_phase=2),
        _row(23, (front_landing, -lift, outward, outward), arm_phase=2),
        _row(26, (360, -outward, outward, outward), (8, 8, 8, 8), arm_phase=2),
        _row(27, (360, -outward, outward, outward)),
        _row(29, (front_next_support, -outward, outward, outward), arm_phase=1),
        _row(31, (front_next_support, -outward, outward, outward), (100, 100, 100, 100)),
        _row(32, (front_next_support, -outward, outward, outward)),
    )


def build_climb_cycle(angles: GeometryAngles) -> tuple[CyclePoint, ...]:
    """Build the repeatable middle-stair hip offsets geometrically."""
    outward = angles.outward_deg
    transition = angles.transition_deg
    landing = (
        360.0 - 2.0 * outward + angles.rear_landing_correction_deg
    )
    preload = landing + transition
    centered = 360.0 - outward + transition / 2.0
    front_centered = outward + transition / 2.0
    advanced = 360.0 - outward + transition
    retreat = (angles.middle_fl_retreat_command,) * 4
    return (
        CyclePoint(3, (0, 0, 0, landing), hub_mode=(5, 5, 5, 0), arm_phase=4),
        CyclePoint(4, (0, 0, 0, landing)),
        CyclePoint(6, (0, 0, landing, landing), hub_mode=(5, 5, 0, 5), arm_phase=3),
        CyclePoint(8, (0, 0, preload, preload)),
        CyclePoint(10, (preload, transition, preload, preload), hub_velocity=retreat),
        CyclePoint(12, (centered, front_centered, centered, centered)),
        CyclePoint(13, (centered, front_centered, advanced, advanced)),
        CyclePoint(18, (centered, 360, advanced, advanced), hub_mode=(5, 0, 5, 5), arm_phase=2),
        CyclePoint(19, (centered, 360, advanced, advanced), hub_velocity=(100, 100, 100, 100)),
        CyclePoint(20, (360, 360, 360, 360)),
    )


def build_final_approach(angles: GeometryAngles) -> tuple[CyclePoint, ...]:
    outward = angles.outward_deg
    transition = angles.transition_deg
    landing = (
        360.0 - 2.0 * outward + angles.rear_landing_correction_deg
    )
    preload = landing + transition
    return (
        CyclePoint(2, (0, 0, 0, landing), hub_mode=(5, 5, 5, 0), arm_phase=4),
        CyclePoint(3, (0, 0, 0, landing)),
        CyclePoint(5, (0, 0, landing, landing), hub_mode=(5, 5, 0, 5), arm_phase=3),
        CyclePoint(7, (0, 0, preload, preload), hub_velocity=(-20, -20, -20, -40)),
        CyclePoint(9, (preload, transition, preload, preload)),
        CyclePoint(10, (preload, transition, preload, preload), hub_velocity=(100, 100, 100, 100)),
        CyclePoint(11, (preload, transition, preload, preload)),
    )


def build_stage3(angles: GeometryAngles) -> tuple[CyclePoint, ...]:
    """
    Return the validated flat-top targets relative to the generated base.

    Stage 3 is a return to the robot's level zero-degree configuration, so its
    absolute continuous targets must not inherit stair-slope corrections.  The
    relative offsets compensate the dimension-dependent final-approach pose.
    """
    outward = angles.outward_deg
    transition = angles.transition_deg
    rear_landing = 360.0 - 2.0 * outward + angles.rear_landing_correction_deg
    preload = rear_landing + transition
    generated_start = (
        2.0 * 360.0 + outward + preload,
        360.0 - outward + transition,
        360.0 + outward + preload,
        360.0 + outward + preload,
    )
    validated_targets = (
        (1040.0, 320.0, 760.0, 760.0),
        (1090.0, 370.0, 760.0, 760.0),
        (1140.0, 320.0, 760.0, 760.0),
        (1140.0, 320.0, 760.0, 1040.0),
        (1140.0, 320.0, 1040.0, 1040.0),
        (1080.0, 360.0, 1080.0, 1080.0),
    )

    def offset(target: tuple[float, float, float, float]) -> tuple[float, ...]:
        return tuple(target[index] - generated_start[index] for index in range(4))

    return (
        CyclePoint(
            2,
            offset(validated_targets[0]),
            hub_mode=(0, 0, 5, 5),
        ),
        CyclePoint(
            3,
            offset(validated_targets[1]),
            hub_mode=(0, 0, 5, 5),
        ),
        CyclePoint(
            4,
            offset(validated_targets[2]),
            hub_mode=(0, 0, 5, 5),
        ),
        CyclePoint(
            7,
            offset(validated_targets[3]),
            hub_mode=(5, 5, 5, 0),
            arm_phase=4,
        ),
        CyclePoint(
            10,
            offset(validated_targets[4]),
            hub_mode=(5, 5, 0, 5),
            arm_phase=3,
        ),
        CyclePoint(
            12,
            offset(validated_targets[5]),
            hub_mode=MODE0,
        ),
    )


def initial_front_clearance_m(center_to_first_riser_m: float) -> float:
    """Return front-wheel leading-edge clearance in the outward start pose."""
    front_pivot_from_center_m = BODY_LENGTH_M / 2.0 - PIVOT_OFFSET_M
    wheel_center_from_pivot_m = LEG_LENGTH_M * math.sin(
        math.radians(OUTWARD_ANGLE_DEG)
    )
    front_wheel_edge_from_center_m = (
        front_pivot_from_center_m
        + wheel_center_from_pivot_m
        + WHEEL_RADIUS_M
    )
    return center_to_first_riser_m - front_wheel_edge_from_center_m


def ideal_hub_speed_mps(hub_command: float) -> float:
    """Convert the controller's 0.1-RPM hub command to ideal ground speed."""
    rpm = abs(hub_command) * HUB_COMMAND_TO_RPM
    return 2.0 * math.pi * WHEEL_RADIUS_M * rpm / 60.0


def calibrated_rise_excess_m(rise_m: float) -> float:
    """Return the rise excess covered by measured 0.10--0.12 m calibrations."""
    capped_rise_m = min(rise_m, MAX_CALIBRATED_CORRECTION_RISE_M)
    return max(0.0, capped_rise_m - VALIDATED_RISE_M)


def entry_approach_rise_correction_m(rise_m: float) -> float:
    """
    Return extra travel before the second FR lift for a taller stair.

    Shorter-than-reference stairs retain the validated approach rather than
    subtracting clearance from an open-loop gait.
    """
    return calibrated_rise_excess_m(rise_m) * ENTRY_APPROACH_RISE_GAIN


def entry_approach_run_correction_m(run_m: float) -> float:
    """Return measured extra entry travel for a deeper-than-reference tread."""
    capped_run_m = min(run_m, MAX_CALIBRATED_ENTRY_RUN_M)
    return max(0.0, capped_run_m - VALIDATED_RUN_M) * ENTRY_APPROACH_RUN_GAIN


def short_run_fr_landing_correction_m(run_m: float) -> float:
    """Return capped fully-lifted FR travel for a shorter-than-0.35 m tread."""
    run_deficit_m = max(0.0, SHORT_RUN_CORRECTION_START_M - run_m)
    correction_m = run_deficit_m * SHORT_RUN_FR_LANDING_GAIN
    return min(correction_m, MAX_SHORT_RUN_FR_LANDING_CORRECTION_M)


def rear_landing_rise_correction_deg(rise_m: float) -> float:
    """Return the taller-stair rear support correction in degrees."""
    return calibrated_rise_excess_m(rise_m) * REAR_LANDING_RISE_GAIN_DEG_PER_M


def middle_fl_retreat_command(rise_m: float) -> float:
    """Return the hub command used while FL transfers between upper treads."""
    return round(
        -calibrated_rise_excess_m(rise_m) * MIDDLE_FL_RETREAT_COMMAND_PER_M,
        9,
    )


def geometric_lift_angle_deg(rise_m: float) -> float:
    """Return 90 degrees unless tread clearance requires a larger lift."""
    pivot_y_m = WHEEL_RADIUS_M + LEG_LENGTH_M * math.cos(
        math.radians(OUTWARD_ANGLE_DEG)
    )
    target_wheel_center_y_m = rise_m + LIFT_CLEARANCE_M + WHEEL_RADIUS_M
    cosine = (pivot_y_m - target_wheel_center_y_m) / LEG_LENGTH_M
    if cosine < -1.0 or cosine > 1.0:
        raise ValueError("stair rise plus lift clearance exceeds leg reach")
    collision_free_angle_deg = math.degrees(math.acos(cosine))
    return round(max(MINIMUM_LIFT_ANGLE_DEG, collision_free_angle_deg), 9)


def geometric_transition_angle_deg(rise_m: float, run_m: float) -> float:
    """Calculate transition angle while preserving the stable stance minimum."""
    reference_slope_deg = math.degrees(
        math.atan2(VALIDATED_RISE_M, VALIDATED_RUN_M)
    )
    transition_clearance_deg = REFERENCE_TRANSITION_ANGLE_DEG - reference_slope_deg
    stair_slope_deg = math.degrees(math.atan2(rise_m, run_m))
    geometric_angle_deg = stair_slope_deg + transition_clearance_deg
    stable_angle_deg = 2.0 * MINIMUM_FRONT_REAR_STANCE_SEPARATION_DEG
    return round(max(geometric_angle_deg, stable_angle_deg), 9)


def calculate_geometry_angles(rise_m: float, run_m: float) -> GeometryAngles:
    """Return the independent angles used to construct all hip targets."""
    return GeometryAngles(
        outward_deg=OUTWARD_ANGLE_DEG,
        lift_deg=geometric_lift_angle_deg(rise_m),
        transition_deg=geometric_transition_angle_deg(rise_m, run_m),
        rear_landing_correction_deg=rear_landing_rise_correction_deg(rise_m),
        middle_fl_retreat_command=middle_fl_retreat_command(rise_m),
    )


def _is_uniform_hub_only_segment(previous: GaitRow, current: GaitRow) -> bool:
    speeds = current.hub_velocity
    return (
        current.hips == previous.hips
        and all(abs(speed) > 0.0 for speed in speeds)
        and max(speeds) - min(speeds) == 0.0
    )


def _retime_hub_only_segments(
    rows: Sequence[GaitRow],
    rise_m: float,
    run_m: float,
    center_to_first_riser_m: float,
    drive_scale: float,
) -> list[GaitRow]:
    """Calculate hub-only durations from desired distance and wheel speed."""
    if len(rows) < 2:
        return list(rows)

    reference_clearance_m = initial_front_clearance_m(
        VALIDATED_CENTER_TO_FIRST_RISER_M
    )
    reference_initial_drive_m = ideal_hub_speed_mps(125.0) * 2.0
    first_riser_reserve_m = reference_clearance_m - reference_initial_drive_m
    requested_initial_drive_m = (
        initial_front_clearance_m(center_to_first_riser_m) - first_riser_reserve_m
    )
    if requested_initial_drive_m <= 0.0:
        raise ValueError(
            "center-to-first-riser distance leaves no positive initial hub approach"
        )

    result = [rows[0]]
    hub_only_segment_index = 0
    for previous, current in zip(rows[:-1], rows[1:]):
        duration_sec = current.time - previous.time
        if _is_uniform_hub_only_segment(previous, current):
            command = current.hub_velocity[0]
            ideal_speed_mps = ideal_hub_speed_mps(command)
            if hub_only_segment_index == 0:
                desired_distance_m = requested_initial_drive_m
            else:
                reference_distance_m = ideal_speed_mps * duration_sec
                desired_distance_m = reference_distance_m * run_m / VALIDATED_RUN_M
                # Segment 1 is the entry hub motion at reference t=15--16,
                # after both front wheels have crossed the first riser and
                # immediately before FR lifts toward the second tread.  Put
                # the observed rise-dependent setback here; adding it to the
                # initial drive could push the still-grounded FL into riser 1.
                if hub_only_segment_index == 1:
                    desired_distance_m += entry_approach_rise_correction_m(rise_m)
                    desired_distance_m += entry_approach_run_correction_m(run_m)
                # Segment 2 advances the chassis while the second FR swing is
                # fully lifted.  Put the short-tread landing reserve here so a
                # still-grounded FL is not pushed toward riser 2 beforehand.
                if hub_only_segment_index == 2:
                    desired_distance_m += short_run_fr_landing_correction_m(run_m)
            duration_sec = desired_distance_m / (ideal_speed_mps * drive_scale)
            hub_only_segment_index += 1
        new_time = round(result[-1].time + duration_sec, 9)
        result.append(replace(current, time=new_time))
    return result


def validate_dimensions(
    rise_m: float,
    run_m: float,
    center_to_first_riser_m: float = VALIDATED_CENTER_TO_FIRST_RISER_M,
    drive_scale: float = 1.0,
) -> None:
    values = (rise_m, run_m, center_to_first_riser_m, drive_scale)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("stair dimensions and approach distance must be finite")
    if any(value <= 0.0 for value in values):
        raise ValueError(
            "stair dimensions, approach distance, and drive scale must be positive"
        )
    clearance_m = initial_front_clearance_m(center_to_first_riser_m)
    if clearance_m <= 0.0:
        raise ValueError("the outward front wheel intersects the first riser")
    lift_angle_deg = geometric_lift_angle_deg(rise_m)
    if lift_angle_deg > 180.0:
        raise ValueError(
            "stair rise plus the calibrated lift clearance exceeds leg reach"
        )
    transition_angle_deg = geometric_transition_angle_deg(rise_m, run_m)
    if transition_angle_deg >= OUTWARD_ANGLE_DEG:
        raise ValueError(
            "stair slope requires a transition angle outside the validated "
            "outward-leg geometry"
        )


def append_climb_cycle(
    rows: list[GaitRow],
    entry_rows: Sequence[GaitRow],
    climb_cycle: Sequence[CyclePoint],
    cycle_index: int,
) -> None:
    if not rows:
        raise ValueError("entry rows must be present before appending a climb cycle")
    if cycle_index < 0:
        raise ValueError("cycle index cannot be negative")

    start_time = entry_rows[-1].time + cycle_index * CYCLE_DURATION_SEC
    base = tuple(
        entry_rows[-1].hips[leg] + 360.0 * cycle_index for leg in range(4)
    )
    for point in climb_cycle:
        hips = tuple(base[leg] + point.hip_offset[leg] for leg in range(4))
        rows.append(
            _row(
                start_time + point.relative_time,
                hips,
                point.hub_velocity,
                point.hub_mode,
                point.arm_phase,
            )
        )


def _append_relative_template(
    rows: list[GaitRow], template: Sequence[CyclePoint]
) -> None:
    if not rows:
        raise ValueError("a relative template requires an existing base row")
    start_time = rows[-1].time
    base = rows[-1].hips
    for point in template:
        hips = tuple(base[leg] + point.hip_offset[leg] for leg in range(4))
        rows.append(
            _row(
                start_time + point.relative_time,
                hips,
                point.hub_velocity,
                point.hub_mode,
                point.arm_phase,
            )
        )


def append_stage3(
    rows: list[GaitRow],
    final_approach: Sequence[CyclePoint],
    stage3: Sequence[CyclePoint],
) -> None:
    _append_relative_template(rows, final_approach)
    _append_relative_template(rows, stage3)


def generate_gait(
    rise_m: float = VALIDATED_RISE_M,
    run_m: float = VALIDATED_RUN_M,
    center_to_first_riser_m: float = VALIDATED_CENTER_TO_FIRST_RISER_M,
    middle_cycles: int = 1,
    include_stage3: bool = False,
    drive_scale: float = 1.0,
) -> list[GaitRow]:
    validate_dimensions(rise_m, run_m, center_to_first_riser_m, drive_scale)
    if isinstance(middle_cycles, bool) or not isinstance(middle_cycles, int):
        raise ValueError("middle cycle count must be an integer")
    if middle_cycles < 1:
        raise ValueError("middle cycle count must be at least one")

    angles = calculate_geometry_angles(rise_m, run_m)
    entry_rows = build_entry_rows(angles)
    climb_cycle = build_climb_cycle(angles)
    rows = list(entry_rows)
    for cycle_index in range(middle_cycles):
        append_climb_cycle(rows, entry_rows, climb_cycle, cycle_index)
    if include_stage3:
        append_stage3(
            rows, build_final_approach(angles), build_stage3(angles)
        )
    rows = _retime_hub_only_segments(
        rows, rise_m, run_m, center_to_first_riser_m, drive_scale
    )
    validate_gait(rows)
    return rows


def validate_gait(rows: Sequence[GaitRow]) -> None:
    if not rows:
        raise ValueError("gait cannot be empty")
    previous_time = -math.inf
    for index, row in enumerate(rows):
        if not math.isfinite(row.time) or row.time <= previous_time:
            raise ValueError(f"row {index}: time must be finite and strictly increasing")
        previous_time = row.time
        if row.arm_phase < 0 or row.arm_phase > 4:
            raise ValueError(f"row {index}: arm phase must be in [0, 4]")
        for name, values in (
            ("hips", row.hips),
            ("steering", row.steering),
            ("hub velocity", row.hub_velocity),
        ):
            if len(values) != 4 or not all(math.isfinite(value) for value in values):
                raise ValueError(f"row {index}: {name} must contain four finite values")
        if len(row.hub_mode) != 4:
            raise ValueError(f"row {index}: hub mode must contain four values")


def _format_number(value: float | int) -> str:
    numeric = float(value)
    if numeric.is_integer():
        return str(int(numeric))
    return format(numeric, ".12g")


def write_csv(rows: Iterable[GaitRow], output_path: Path, force: bool = False) -> None:
    output_path = output_path.expanduser().resolve()
    if output_path.exists() and not force:
        raise FileExistsError(f"refusing to overwrite existing file: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(CSV_HEADER)
        for row in rows:
            values = row.csv_values()
            formatted = [_format_number(value) for value in values]
            if row.time == 0.0:
                formatted[0] = "0.0"
            writer.writerow(formatted)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a geometry-scaled Kilin stair CSV from the validated "
            "phase sequence."
        )
    )
    parser.add_argument("--rise", type=float, default=VALIDATED_RISE_M, help="stair rise in m")
    parser.add_argument("--run", type=float, default=VALIDATED_RUN_M, help="stair run in m")
    parser.add_argument(
        "--center-to-first-riser",
        type=float,
        default=VALIDATED_CENTER_TO_FIRST_RISER_M,
        help="initial robot-center distance to the first stair riser in m",
    )
    parser.add_argument(
        "--drive-scale",
        type=float,
        default=1.0,
        help="actual/ideal hub travel ratio used to compensate wheel slip",
    )
    parser.add_argument(
        "--middle-cycles",
        type=int,
        default=1,
        help="number of validated 35--52 s middle-stair cycles",
    )
    parser.add_argument(
        "--include-stage3",
        action="store_true",
        help="append the final-approach bridge and validated stage-3 gait",
    )
    parser.add_argument("--output", type=Path, required=True, help="output CSV path")
    parser.add_argument("--force", action="store_true", help="overwrite an existing output file")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        rows = generate_gait(
            args.rise,
            args.run,
            args.center_to_first_riser,
            args.middle_cycles,
            args.include_stage3,
            args.drive_scale,
        )
        write_csv(rows, args.output, args.force)
    except (ValueError, OSError) as error:
        raise SystemExit(f"error: {error}") from error
    angles = calculate_geometry_angles(args.rise, args.run)
    clearance_mm = initial_front_clearance_m(args.center_to_first_riser) * 1000.0
    entry_rise_correction_mm = (
        entry_approach_rise_correction_m(args.rise) * 1000.0
    )
    entry_run_correction_mm = entry_approach_run_correction_m(args.run) * 1000.0
    short_run_landing_correction_mm = (
        short_run_fr_landing_correction_m(args.run) * 1000.0
    )
    print(
        "geometry: "
        f"outward={angles.outward_deg:.3f} deg, "
        f"lift={angles.lift_deg:.3f} deg, "
        f"transition={angles.transition_deg:.3f} deg, "
        f"rear_landing_correction={angles.rear_landing_correction_deg:.3f} deg, "
        f"middle_fl_retreat={angles.middle_fl_retreat_command:.3f}, "
        f"initial_clearance={clearance_mm:.3f} mm, "
        f"entry_rise_correction={entry_rise_correction_mm:.3f} mm, "
        f"entry_run_correction={entry_run_correction_mm:.3f} mm, "
        f"short_run_fr_landing_correction="
        f"{short_run_landing_correction_mm:.3f} mm"
    )
    print(f"wrote {len(rows)} gait rows to {args.output.expanduser().resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
