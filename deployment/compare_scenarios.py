"""Compare saved external simulator runs without changing their measurements.

Inputs are two independently completed routes per platform. The differences are
descriptive: wind, drag and sensor noise change together in the scenario.
"""

import argparse
import csv
import json
import math
from pathlib import Path


def read_run(directory: Path, platform: str) -> dict:
    """Read route checks and simulation truth comparison, retaining their units."""
    validation = json.loads((directory / "validation.json").read_text(encoding="utf-8"))
    comparison = json.loads((directory / "navigation_truth_comparison.json").read_text(encoding="utf-8"))
    if validation.get("passed") is not True:
        raise ValueError(f"{directory} did not pass route validation")
    aligned_name = "route_truth_aligned_sar.csv" if platform == "sitl" else "route_aligned_sar.csv"
    with (directory / aligned_name).open(encoding="utf-8", newline="") as stream:
        aligned = list(csv.DictReader(stream))
    if not aligned:
        raise ValueError(f"{directory} has no aligned route rows")
    truth_y = [float(row["truth_y_sar_m"]) for row in aligned]
    position_key = (
        "relative_navigation_minus_truth_position_rms_sar_m"
        if platform == "sitl" else "preflight_origin_corrected_rms_sar_m"
    )
    attitude_key = (
        "navigation_minus_truth_attitude_rms_deg"
        if platform == "sitl" else "navigation_minus_gazebo_attitude_rms_deg"
    )
    return {
        "directory": str(directory.resolve()),
        "scenario": validation.get("scenario", "baseline"),
        "route_samples": validation["route_samples"],
        "route_duration_s": validation["route_duration_s"],
        "route_x_span_m": validation["route_x_span_m"],
        "route_altitude_mean_m": validation["route_altitude_mean_m"],
        "route_cross_track_rms_m": validation["route_cross_track_rms_m"],
        "truth_cross_track_rms_m": math.sqrt(sum(y * y for y in truth_y) / len(truth_y)),
        "matched_truth_samples": comparison["samples"],
        "position_difference_rms_sar_m": comparison[position_key],
        "attitude_difference_rms_deg": comparison[attitude_key],
        "velocity_difference_rms_sar_mps": (
            comparison["navigation_minus_truth_velocity_rms_sar_mps"] if platform == "sitl" else None
        ),
    }


def difference(before: object, after: object) -> object:
    if isinstance(before, list) and isinstance(after, list):
        return [difference(a, b) for a, b in zip(before, after, strict=True)]
    if isinstance(before, (int, float)) and isinstance(after, (int, float)):
        return after - before
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for platform in ("sitl", "xtdrone"):
        for label in ("baseline", "scenario"):
            parser.add_argument(f"--{platform}-{label}", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = {"interpretation": (
        "Independent simulator runs, not calibrated aircraft data. Position differences use a pre-capture "
        "relative origin in SITL and a preflight median offset in PX4; these are not the same metric. "
        "Clock/estimator delay is not corrected. The scenario changes wind, drag and IMU noise together, "
        "so differences cannot be assigned to one cause."
    )}
    for platform in ("sitl", "xtdrone"):
        baseline = read_run(getattr(args, f"{platform}_baseline"), platform)
        scenario = read_run(getattr(args, f"{platform}_scenario"), platform)
        if baseline["scenario"] != "baseline" or scenario["scenario"] == "baseline":
            raise ValueError(f"{platform}: expected one baseline and one named scenario")
        changes = {
            key: difference(baseline[key], scenario[key])
            for key in baseline if key not in ("directory", "scenario")
        }
        result[platform] = {"baseline": baseline, "scenario": scenario, "scenario_minus_baseline": changes}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
