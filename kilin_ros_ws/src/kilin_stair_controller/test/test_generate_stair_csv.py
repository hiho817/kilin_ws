#!/usr/bin/env python3

import csv
import importlib.util
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

    def test_unknown_stair_dimensions_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "only the validated"):
            generator.generate_gait(rise_m=0.30, run_m=0.10)

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
