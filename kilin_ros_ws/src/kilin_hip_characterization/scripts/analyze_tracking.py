#!/usr/bin/env python3
"""Offline Phase-A tracker analysis; input is campaign_runner command_state_trace.csv."""
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    out = args.out or args.trace.with_name("tracking_summary.csv")
    groups = defaultdict(list)
    with args.trace.open(newline="") as handle:
        for row in csv.DictReader(handle):
            # Dynamic phases only: static release (2), extend (3), hold (4), contract (5).
            if int(row["phase"]) not in (2, 3, 4, 5):
                continue
            actual = float(row["hip_actual_rad"])
            command = float(row["hip_cmd_actual_rad"])
            if math.isfinite(actual) and math.isfinite(command):
                groups[(row["trial"], row["module"], row["phase"])].append(
                    (command - actual, float(row["hip_torque_nm"]), int(row["hip_error_code"])))
    with out.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["trial", "module", "phase", "samples", "actual_error_rms_rad", "actual_error_bias_rad", "actual_error_peak_rad", "peak_motor_torque_nm", "fault_samples"])
        for (trial, module, phase), values in sorted(groups.items()):
            errors = [value[0] for value in values]
            writer.writerow([trial, module, phase, len(values), rms(errors), sum(errors) / len(errors), max(map(abs, errors)), max(abs(value[1]) for value in values), sum(value[2] != 0 for value in values)])
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
