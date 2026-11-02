"""Measure sampled collection-route tube around commanded SAR y=0, z=300 m.

Only the 20.48 s horizontal capture window in route-aligned CSV is used.
The tube is a cylinder around the commanded x-axis, not a fitted centreline.
"""

import argparse
import csv
import json
import math
from pathlib import Path


def measure(run: Path, platform: str) -> dict:
    """Return sampled radial tube diameters in metres for truth and navigation."""
    aligned = run / ("route_truth_aligned_sar.csv" if platform == "sitl" else "route_aligned_sar.csv")
    validation = json.loads((run / "validation.json").read_text(encoding="utf-8"))
    with aligned.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or validation.get("passed") is not True:
        raise ValueError(f"missing aligned samples or failed route: {run}")
    truth_time_key = "truth_sim_time_s" if platform == "px4" else "truth_precise_receipt_monotonic_s"
    unique_truth_samples = len({row[truth_time_key] for row in rows})
    truth_rows = list({row[truth_time_key]: row for row in rows}.values())

    def series(prefix: str, samples: list[dict]) -> dict:
        lateral = [float(row[f"{prefix}_y_sar_m"]) for row in samples]
        height = [float(row[f"{prefix}_z_sar_m"]) for row in samples]
        radius = [math.hypot(y, z - 300.0) for y, z in zip(lateral, height, strict=True)]
        if not all(math.isfinite(value) for value in radius):
            raise ValueError(f"nonfinite {prefix} position in {run}")
        ordered = sorted(radius)
        p95_index = math.ceil(0.95 * len(ordered)) - 1
        return {
            "samples": len(radius),
            "radius_rms_m": math.sqrt(sum(value * value for value in radius) / len(radius)),
            "diameter_95_m": 2 * ordered[p95_index],
            "diameter_all_sampled_m": 2 * ordered[-1],
            "radius_median_m": ordered[len(ordered) // 2],
            "lateral_min_max_m": [min(lateral), max(lateral)],
            "height_min_max_m": [min(height), max(height)],
        }

    return {
        "run": str(run.resolve()),
        "platform": platform,
        "scenario": validation.get("scenario", "baseline"),
        "route_duration_s": validation["route_duration_s"],
        "route_samples": validation["route_samples"],
        "matched_samples": len(rows),
        "unique_truth_samples": unique_truth_samples,
        "mean_truth_sample_rate_hz": (unique_truth_samples - 1) / validation["route_duration_s"],
        "match_fraction": len(rows) / validation["route_samples"],
        "truth": series("truth", truth_rows),
        "navigation": series("nav", rows),
        "definition": "2 x radial quantile about commanded y=0,z=300m during capture; x is along-tube axis",
        "limitation": "sampled trajectory only; no bound on unobserved motion between samples or future repeat passes",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sitl-baseline", type=Path, required=True)
    parser.add_argument("--sitl-scenario", type=Path, required=True)
    parser.add_argument("--px4-baseline", type=Path, required=True)
    parser.add_argument("--px4-scenario", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = {
        "sitl_baseline": measure(args.sitl_baseline, "sitl"),
        "sitl_scenario": measure(args.sitl_scenario, "sitl"),
        "px4_baseline": measure(args.px4_baseline, "px4"),
        "px4_scenario": measure(args.px4_scenario, "px4"),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
