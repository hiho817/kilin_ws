#!/usr/bin/env python3
"""
Generate validated Kilin stair-gait CSV templates.

The first implementation deliberately supports only the validated 0.35 m rise,
0.10 m run gait.  It can repeat the validated 35--52 s middle-stair cycle while
preserving continuous hip angles, then optionally append the validated final
approach and stage-3 templates through t=75 s.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
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

VALIDATED_RISE_M = 0.35
VALIDATED_RUN_M = 0.10
DIMENSION_TOLERANCE_M = 1e-6

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


# Validated alex_v2 entry, including the transition to the canonical middle-cycle
# base pose at t=32 s.  Logical hip angles remain continuous and are never wrapped.
ENTRY_ROWS = (
    _row(0, (0, 0, 0, 0)),
    _row(3, (-40, -40, 40, 40), hub_mode=MODE0),
    _row(6, (-40, -90, 40, 40), arm_phase=2),
    _row(8, (-40, -90, 40, 40), (125, 125, 125, 125), arm_phase=2),
    _row(9, (-40, -90, 40, 40), arm_phase=2),
    _row(11, (0, -40, 40, 40), (8, 8, 8, 8), arm_phase=2),
    _row(12, (0, -40, 40, 40)),
    _row(15, (320, -40, 40, 40), arm_phase=1),
    _row(16, (320, -40, 40, 40), (100, 100, 100, 100)),
    _row(17, (320, -40, 40, 40)),
    _row(20, (320, -90, 40, 40), arm_phase=2),
    _row(22, (320, -90, 40, 40), (125, 125, 125, 125), arm_phase=2),
    _row(23, (320, -90, 40, 40), arm_phase=2),
    _row(26, (360, -40, 40, 40), (8, 8, 8, 8), arm_phase=2),
    _row(27, (360, -40, 40, 40)),
    _row(29, (400, -40, 40, 40), arm_phase=1),
    _row(31, (400, -40, 40, 40), (100, 100, 100, 100)),
    _row(32, (400, -40, 40, 40)),
)


@dataclass(frozen=True)
class CyclePoint:
    relative_time: float
    hip_offset: tuple[float, float, float, float]
    hub_velocity: tuple[float, float, float, float] = ZERO4
    hub_mode: tuple[int, int, int, int] = MODE5
    arm_phase: int = 0


# Validated alex_v2 t=35--52 middle-stair motion, expressed relative to the
# t=32 base pose [400, -40, 40, 40] deg.  The final offset is +360 deg for all
# legs, making the cycle repeatable without modulo normalization.
CLIMB_CYCLE = (
    CyclePoint(3, (0, 0, 0, 280), hub_mode=(5, 5, 5, 0), arm_phase=4),
    CyclePoint(4, (0, 0, 0, 280)),
    CyclePoint(6, (0, 0, 280, 280), hub_mode=(5, 5, 0, 5), arm_phase=3),
    CyclePoint(8, (0, 0, 300, 300)),
    CyclePoint(10, (300, 20, 300, 300)),
    CyclePoint(12, (330, 50, 330, 330)),
    CyclePoint(13, (330, 50, 340, 340)),
    CyclePoint(18, (330, 360, 340, 340), hub_mode=(5, 0, 5, 5), arm_phase=2),
    CyclePoint(19, (330, 360, 340, 340), hub_velocity=(100, 100, 100, 100)),
    CyclePoint(20, (360, 360, 360, 360)),
)

CYCLE_DURATION_SEC = CLIMB_CYCLE[-1].relative_time


# Bridge from the canonical middle-cycle end pose to the stage-3 start pose.
# Times and offsets are relative to alex_v2 t=52 s.
FINAL_APPROACH = (
    CyclePoint(2, (0, 0, 0, 280), hub_mode=(5, 5, 5, 0), arm_phase=4),
    CyclePoint(3, (0, 0, 0, 280)),
    CyclePoint(5, (0, 0, 280, 280), hub_mode=(5, 5, 0, 5), arm_phase=3),
    CyclePoint(7, (0, 0, 300, 300), hub_velocity=(-20, -20, -20, -40)),
    CyclePoint(9, (300, 20, 300, 300)),
    CyclePoint(10, (300, 20, 300, 300), hub_velocity=(100, 100, 100, 100)),
    CyclePoint(11, (300, 20, 300, 300)),
)


# User-validated final-stair gait. Times and offsets are relative to the stage-3
# start pose at alex_v2 t=63 s [1060, 340, 700, 700] deg.
STAGE3 = (
    CyclePoint(2, (-20, -20, 60, 60), hub_mode=(0, 0, 5, 5)),
    CyclePoint(3, (30, 30, 60, 60), hub_mode=(0, 0, 5, 5)),
    CyclePoint(4, (80, -20, 60, 60), hub_mode=(0, 0, 5, 5)),
    CyclePoint(7, (80, -20, 60, 340), hub_mode=(5, 5, 5, 0), arm_phase=4),
    CyclePoint(10, (80, -20, 340, 340), hub_mode=(5, 5, 0, 5), arm_phase=3),
    CyclePoint(12, (20, 20, 380, 380), hub_mode=MODE0),
)


def validate_dimensions(rise_m: float, run_m: float) -> None:
    if not math.isfinite(rise_m) or not math.isfinite(run_m):
        raise ValueError("stair dimensions must be finite")
    if rise_m <= 0.0 or run_m <= 0.0:
        raise ValueError("stair rise and run must be positive")
    if (
        abs(rise_m - VALIDATED_RISE_M) > DIMENSION_TOLERANCE_M
        or abs(run_m - VALIDATED_RUN_M) > DIMENSION_TOLERANCE_M
    ):
        raise ValueError(
            "only the validated 0.35 m rise / 0.10 m run template is currently "
            "supported; arbitrary dimensions require the future geometry solver"
        )


def append_climb_cycle(rows: list[GaitRow], cycle_index: int) -> None:
    if not rows:
        raise ValueError("entry rows must be present before appending a climb cycle")
    if cycle_index < 0:
        raise ValueError("cycle index cannot be negative")

    start_time = ENTRY_ROWS[-1].time + cycle_index * CYCLE_DURATION_SEC
    base = tuple(
        ENTRY_ROWS[-1].hips[leg] + 360.0 * cycle_index for leg in range(4)
    )
    for point in CLIMB_CYCLE:
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


def append_stage3(rows: list[GaitRow]) -> None:
    _append_relative_template(rows, FINAL_APPROACH)
    _append_relative_template(rows, STAGE3)


def generate_gait(
    rise_m: float = VALIDATED_RISE_M,
    run_m: float = VALIDATED_RUN_M,
    middle_cycles: int = 1,
    include_stage3: bool = False,
) -> list[GaitRow]:
    validate_dimensions(rise_m, run_m)
    if isinstance(middle_cycles, bool) or not isinstance(middle_cycles, int):
        raise ValueError("middle cycle count must be an integer")
    if middle_cycles < 1:
        raise ValueError("middle cycle count must be at least one")

    rows = list(ENTRY_ROWS)
    for cycle_index in range(middle_cycles):
        append_climb_cycle(rows, cycle_index)
    if include_stage3:
        append_stage3(rows)
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
            "Generate a Kilin stair CSV. The current version only supports the "
            "validated 0.35 m rise / 0.10 m run template."
        )
    )
    parser.add_argument("--rise", type=float, default=VALIDATED_RISE_M, help="stair rise in m")
    parser.add_argument("--run", type=float, default=VALIDATED_RUN_M, help="stair run in m")
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
            args.rise, args.run, args.middle_cycles, args.include_stage3
        )
        write_csv(rows, args.output, args.force)
    except (ValueError, OSError) as error:
        raise SystemExit(f"error: {error}") from error
    print(f"wrote {len(rows)} gait rows to {args.output.expanduser().resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
