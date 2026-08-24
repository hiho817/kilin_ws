#!/usr/bin/env python3

import csv
import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "scripts" / "generate_stair_csv.py"
SPEC = importlib.util.spec_from_file_location("generate_stair_csv", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
generator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = generator
SPEC.loader.exec_module(generator)


class GenerateStairCsvTest(unittest.TestCase):
    def test_one_cycle_matches_validated_alex_v2_prefix(self):
        reference_path = (
            PACKAGE_ROOT.parents[2] / "csv" / "stairs_v3" / "alex_v2.csv"
        )
        self.assertTrue(reference_path.exists(), reference_path)

        with reference_path.open(newline="", encoding="utf-8") as stream:
            reference = list(csv.reader(stream))
        expected = [reference[0]]
        expected.extend(row for row in reference[1:] if float(row[0]) <= 52.0)

        rows = generator.generate_gait(middle_cycles=1)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "generated.csv"
            generator.write_csv(rows, output)
            with output.open(newline="", encoding="utf-8") as stream:
                actual = list(csv.reader(stream))

        self.assertEqual(expected, actual)

    def test_second_cycle_adds_360_degrees_without_wrapping(self):
        rows = generator.generate_gait(middle_cycles=2)
        first_end = next(row for row in rows if row.time == 52.0)
        second_end = rows[-1]

        self.assertEqual(72.0, second_end.time)
        self.assertEqual(
            tuple(value + 360.0 for value in first_end.hips), second_end.hips
        )

    def test_stage3_matches_complete_alex_v2(self):
        reference_path = (
            PACKAGE_ROOT.parents[2] / "csv" / "stairs_v3" / "alex_v2.csv"
        )
        with reference_path.open(newline="", encoding="utf-8") as stream:
            expected = list(csv.reader(stream))

        rows = generator.generate_gait(middle_cycles=1, include_stage3=True)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "generated_full.csv"
            generator.write_csv(rows, output)
            with output.open(newline="", encoding="utf-8") as stream:
                actual = list(csv.reader(stream))

        self.assertEqual(expected, actual)

    def test_stage3_shifts_with_an_extra_middle_cycle(self):
        one_cycle = generator.generate_gait(middle_cycles=1, include_stage3=True)
        two_cycles = generator.generate_gait(middle_cycles=2, include_stage3=True)

        self.assertEqual(one_cycle[-1].time + 20.0, two_cycles[-1].time)
        self.assertEqual(
            tuple(value + 360.0 for value in one_cycle[-1].hips),
            two_cycles[-1].hips,
        )

    def test_fr_swing_preserves_continuous_310_degree_motion(self):
        rows = generator.generate_gait(middle_cycles=1)
        start = next(row for row in rows if row.time == 45.0)
        end = next(row for row in rows if row.time == 50.0)

        self.assertEqual(10.0, start.hips[1])
        self.assertEqual(320.0, end.hips[1])
        self.assertEqual(310.0, end.hips[1] - start.hips[1])

    def test_cycle_phase_order_is_fixed(self):
        rows = generator.generate_gait(middle_cycles=1)
        cycle_rows = [row for row in rows if row.time > 32.0]
        transitions = []
        for row in cycle_rows:
            if not transitions or transitions[-1] != row.arm_phase:
                transitions.append(row.arm_phase)
        self.assertEqual([4, 0, 3, 0, 2, 0], transitions)

    def test_15cm_rise_keeps_conservative_90_degree_lift(self):
        rows = generator.generate_gait(rise_m=0.15)
        lift_row = next(row for row in rows if row.arm_phase == 2)
        self.assertEqual(-90.0, lift_row.hips[1])

    def test_lift_increases_only_when_90_degrees_lacks_clearance(self):
        angle = generator.geometric_lift_angle_deg(0.20)
        self.assertGreater(angle, 90.0)

        pivot_height = (
            generator.WHEEL_RADIUS_M
            + generator.LEG_LENGTH_M
            * math.cos(math.radians(generator.OUTWARD_ANGLE_DEG))
        )
        wheel_bottom = (
            pivot_height
            - generator.LEG_LENGTH_M * math.cos(math.radians(angle))
            - generator.WHEEL_RADIUS_M
        )
        self.assertAlmostEqual(0.20 + generator.LIFT_CLEARANCE_M, wheel_bottom)

    def test_validated_geometry_reconstructs_independent_angles(self):
        angles = generator.calculate_geometry_angles(0.10, 0.35)
        self.assertEqual(40.0, angles.outward_deg)
        self.assertEqual(90.0, angles.lift_deg)
        self.assertEqual(20.0, angles.transition_deg)
        self.assertEqual(0.0, angles.rear_landing_correction_deg)
        self.assertEqual(0.0, angles.middle_fl_retreat_command)

    def test_shallow_stair_keeps_ten_degree_front_rear_stance_separation(self):
        angles = generator.calculate_geometry_angles(0.08, 0.40)
        cycle = generator.build_climb_cycle(angles)

        self.assertEqual(20.0, angles.transition_deg)
        # Immediately before the long FR swing, point 6 places both front
        # hips at transition/2 and both rear hips at transition (modulo 360).
        front = cycle[6].hip_offset[0]
        rear = cycle[6].hip_offset[2]
        self.assertAlmostEqual(
            generator.MINIMUM_FRONT_REAR_STANCE_SEPARATION_DEG,
            rear - front,
        )

    def test_dimensions_change_middle_cycle_and_stage3_hip_angles(self):
        reference = generator.calculate_geometry_angles(0.10, 0.35)
        variant = generator.calculate_geometry_angles(0.15, 0.40)
        reference_cycle = generator.build_climb_cycle(reference)
        variant_cycle = generator.build_climb_cycle(variant)
        reference_stage3 = generator.build_stage3(reference)
        variant_stage3 = generator.build_stage3(variant)

        self.assertNotEqual(reference_cycle[3].hip_offset, variant_cycle[3].hip_offset)
        self.assertNotEqual(reference_stage3[0].hip_offset, variant_stage3[0].hip_offset)

    def test_validated_start_clearance_uses_robot_geometry(self):
        clearance = generator.initial_front_clearance_m(0.63)
        self.assertAlmostEqual(0.173589160, clearance, places=9)

    def test_run_scales_entry_duration_and_caps_measured_correction(self):
        rows = generator.generate_gait(run_m=0.70)
        drive_start = next(row for row in rows if row.hips == (320.0, -40.0, 40.0, 40.0))
        following = rows[rows.index(drive_start) + 1]
        correction_duration = (
            generator.entry_approach_run_correction_m(0.70)
            / generator.ideal_hub_speed_mps(100.0)
        )
        self.assertAlmostEqual(
            2.0 + correction_duration,
            following.time - drive_start.time,
            places=8,
        )

    def test_40cm_run_adds_measured_second_fr_landing_correction(self):
        self.assertAlmostEqual(0.055, generator.entry_approach_run_correction_m(0.40))
        self.assertEqual(0.0, generator.entry_approach_run_correction_m(0.35))
        self.assertEqual(0.0, generator.entry_approach_run_correction_m(0.30))
        self.assertAlmostEqual(
            generator.entry_approach_run_correction_m(0.40),
            generator.entry_approach_run_correction_m(0.50),
        )

    def test_short_run_fr_landing_correction_starts_below_35cm_and_caps_at_15mm(self):
        self.assertAlmostEqual(0.015, generator.short_run_fr_landing_correction_m(0.30))
        self.assertAlmostEqual(0.015, generator.short_run_fr_landing_correction_m(0.33))
        self.assertAlmostEqual(0.0075, generator.short_run_fr_landing_correction_m(0.34))
        self.assertEqual(0.0, generator.short_run_fr_landing_correction_m(0.35))
        self.assertEqual(0.0, generator.short_run_fr_landing_correction_m(0.40))
        self.assertAlmostEqual(
            generator.short_run_fr_landing_correction_m(0.30),
            generator.short_run_fr_landing_correction_m(0.20),
        )

    def test_33cm_run_adds_full_correction_to_fully_lifted_fr_drive(self):
        rows = generator.generate_gait(rise_m=0.11, run_m=0.33)
        lift_drive_end = next(
            row for row in rows
            if row.hips == (320.0, -90.0, 40.0, 40.0)
            and row.hub_velocity == (125.0, 125.0, 125.0, 125.0)
        )
        lift_drive_start = rows[rows.index(lift_drive_end) - 1]
        scaled_reference_duration = 2.0 * 0.33 / generator.VALIDATED_RUN_M
        correction_duration = (
            generator.short_run_fr_landing_correction_m(0.33)
            / generator.ideal_hub_speed_mps(125.0)
        )

        self.assertAlmostEqual(
            scaled_reference_duration + correction_duration,
            lift_drive_end.time - lift_drive_start.time,
            places=8,
        )

    def test_30cm_run_adds_correction_only_to_fully_lifted_fr_drive(self):
        rows = generator.generate_gait(rise_m=0.12, run_m=0.30)
        lift_drive_end = next(
            row for row in rows
            if row.hips == (320.0, -90.0, 40.0, 40.0)
            and row.hub_velocity == (125.0, 125.0, 125.0, 125.0)
        )
        lift_drive_start = rows[rows.index(lift_drive_end) - 1]
        scaled_reference_duration = 2.0 * 0.30 / generator.VALIDATED_RUN_M
        correction_duration = (
            generator.short_run_fr_landing_correction_m(0.30)
            / generator.ideal_hub_speed_mps(125.0)
        )

        self.assertAlmostEqual(
            scaled_reference_duration + correction_duration,
            lift_drive_end.time - lift_drive_start.time,
            places=8,
        )

    def test_taller_rise_extends_approach_before_second_fr_lift(self):
        reference = generator.generate_gait(rise_m=0.10)
        taller = generator.generate_gait(rise_m=0.12)

        reference_start = next(
            row for row in reference
            if row.hips == (320.0, -40.0, 40.0, 40.0)
        )
        taller_start = next(
            row for row in taller
            if row.hips == (320.0, -40.0, 40.0, 40.0)
        )
        reference_stop = reference[reference.index(reference_start) + 1]
        taller_stop = taller[taller.index(taller_start) + 1]

        extra_duration = (
            generator.entry_approach_rise_correction_m(0.12)
            / generator.ideal_hub_speed_mps(100.0)
        )
        self.assertAlmostEqual(
            (reference_stop.time - reference_start.time) + extra_duration,
            taller_stop.time - taller_start.time,
            places=8,
        )
        self.assertAlmostEqual(0.056, generator.entry_approach_rise_correction_m(0.12))

    def test_shorter_rise_does_not_remove_entry_clearance(self):
        self.assertEqual(0.0, generator.entry_approach_rise_correction_m(0.08))

    def test_empirical_rise_corrections_are_capped_at_validated_12cm(self):
        self.assertAlmostEqual(
            generator.entry_approach_rise_correction_m(0.12),
            generator.entry_approach_rise_correction_m(0.15),
        )
        self.assertAlmostEqual(
            generator.rear_landing_rise_correction_deg(0.12),
            generator.rear_landing_rise_correction_deg(0.15),
        )
        self.assertAlmostEqual(
            generator.middle_fl_retreat_command(0.12),
            generator.middle_fl_retreat_command(0.15),
        )
        self.assertAlmostEqual(0.056, generator.entry_approach_rise_correction_m(0.15))
        self.assertAlmostEqual(2.0, generator.rear_landing_rise_correction_deg(0.15))
        self.assertAlmostEqual(-50.0, generator.middle_fl_retreat_command(0.15))

    def test_taller_rise_moves_rear_landing_support_rearward(self):
        angles = generator.calculate_geometry_angles(0.12, 0.35)
        cycle = generator.build_climb_cycle(angles)

        self.assertAlmostEqual(2.0, angles.rear_landing_correction_deg)
        self.assertAlmostEqual(282.0, cycle[0].hip_offset[3])
        self.assertAlmostEqual(282.0, cycle[2].hip_offset[2])

    def test_taller_rise_adds_validated_middle_fl_retreat(self):
        reference = generator.build_climb_cycle(
            generator.calculate_geometry_angles(0.10, 0.35)
        )
        taller = generator.build_climb_cycle(
            generator.calculate_geometry_angles(0.12, 0.35)
        )

        self.assertEqual((0.0, 0.0, 0.0, 0.0), reference[4].hub_velocity)
        self.assertEqual((-50.0, -50.0, -50.0, -50.0), taller[4].hub_velocity)

    def test_stage3_returns_to_validated_flat_top_targets(self):
        rows = generator.generate_gait(
            rise_m=0.12, run_m=0.35, include_stage3=True
        )

        self.assertIn((1040.0, 320.0, 760.0, 760.0), [row.hips for row in rows])
        self.assertIn((1140.0, 320.0, 760.0, 760.0), [row.hips for row in rows])
        self.assertEqual((1080.0, 360.0, 1080.0, 1080.0), rows[-1].hips)

    def test_center_distance_changes_only_initial_approach_duration(self):
        reference = generator.generate_gait()
        farther = generator.generate_gait(center_to_first_riser_m=0.70)
        self.assertGreater(farther[3].time - farther[2].time, 2.0)
        first_shift = farther[3].time - reference[3].time
        self.assertAlmostEqual(first_shift, farther[-1].time - reference[-1].time)

    def test_unreachable_rise_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "exceeds leg reach"):
            generator.generate_gait(rise_m=0.50)

    def test_stair_slope_outside_outward_geometry_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "stair slope"):
            generator.generate_gait(rise_m=0.35, run_m=0.10)

    def test_existing_output_requires_force(self):
        rows = generator.generate_gait()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "generated.csv"
            generator.write_csv(rows, output)
            with self.assertRaises(FileExistsError):
                generator.write_csv(rows, output)
            generator.write_csv(rows, output, force=True)


if __name__ == "__main__":
    unittest.main()
