"""只读审计外部真值与导航记录的频率、时钟和模块接入条件。"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _column(path: Path, name: str) -> np.ndarray:
    with path.open(encoding="utf-8", newline="") as stream:
        values = [float(row[name]) for row in csv.DictReader(stream)]
    result = np.asarray(values, dtype=np.float64)
    if len(result) < 2 or not np.all(np.isfinite(result)):
        raise ValueError(f"Missing or invalid {name} in {path}")
    return result


def _timing(stamps: np.ndarray) -> dict:
    intervals = np.diff(stamps)
    positive = intervals[intervals > 0]
    return {
        "samples": int(len(stamps)),
        "independent_samples": int(np.count_nonzero(intervals > 0) + 1),
        "nonpositive_intervals": int(np.count_nonzero(intervals <= 0)),
        "duration_s": float(stamps[-1] - stamps[0]),
        "median_interval_s": float(np.median(positive)) if len(positive) else None,
        "maximum_interval_s": float(np.max(positive)) if len(positive) else None,
        "effective_rate_hz": float((len(stamps) - 1) / (stamps[-1] - stamps[0]))
        if stamps[-1] > stamps[0] else None,
    }


def audit(directory: Path, source: str) -> dict:
    """检查实际采集段，不推测通信或估计器延迟；输入目录不得修改。"""
    px4 = source == "px4"
    if source not in ("px4", "sitl"):
        raise ValueError("Source must be px4 or sitl")
    route_clock = "timestamp_s" if px4 else "time_boot_s"
    route = _column(directory / "route_navigation_sar.csv", route_clock)
    nav_path = directory / "navigation_position_sar.csv"
    nav_stamps = _column(nav_path, route_clock)
    nav_receipt = _column(nav_path, "receipt_monotonic_s")
    truth_path = directory / ("gazebo_truth_enu.csv" if px4 else "simulation_truth_precise_wgs84.csv")
    truth_receipt = _column(truth_path, "receipt_monotonic_s")
    truth_stamps = _column(truth_path, "sim_time_s") if px4 else truth_receipt
    if route[-1] <= route[0] or np.any(np.diff(nav_receipt) <= 0) or np.any(np.diff(truth_receipt) <= 0):
        raise ValueError("Route or receipt time must increase")
    begin = int(np.argmin(np.abs(nav_stamps - route[0])))
    end = int(np.argmin(np.abs(nav_stamps - route[-1])))
    if end <= begin:
        raise ValueError("Route cannot be located within navigation records")
    nav_window = (nav_stamps >= route[0]) & (nav_stamps <= route[-1])
    truth_window = (truth_receipt >= nav_receipt[begin]) & (truth_receipt <= nav_receipt[end])
    nav_selected = nav_stamps[nav_window]
    truth_selected = truth_stamps[truth_window]
    if len(nav_selected) < 2 or len(truth_selected) < 2:
        raise ValueError("Insufficient route truth or navigation samples")
    truth_timing = _timing(truth_selected) if px4 else {
        "samples": int(len(truth_selected)),
        "receipt_span_s": float(truth_selected[-1] - truth_selected[0]),
        "native_simulation_interval_s": None,
        "native_simulation_rate_hz": None,
        "route_average_count_rate_hz": float((len(truth_selected) - 1) / (route[-1] - route[0])),
    }
    aligned_directory = directory / ("clock_sync" if px4 else "clock_sync_final")
    alignment_path = aligned_directory / "alignment.json"
    paired_path = aligned_directory / "pair_validation.json"
    alignment = json.loads(alignment_path.read_text(encoding="utf-8")) if alignment_path.exists() else None
    paired = json.loads(paired_path.read_text(encoding="utf-8")) if paired_path.exists() else None
    clock_verified = bool(alignment and paired and paired.get("status") == "shared_simulation_clock_paired")
    return {
        "source": source,
        "directory": str(directory.resolve()),
        "truth_time_source": "Gazebo simulation time" if px4 else "host receipt time only",
        "navigation_time_source": "PX4 synchronized DDS stamp" if px4 else "ArduPilot boot time",
        "route_window_navigation_clock_s": [float(route[0]), float(route[-1])],
        "truth": truth_timing,
        "navigation": _timing(nav_selected),
        "clock_alignment_verified": clock_verified,
        "aligned_dataset": {
            "directory": str(aligned_directory.resolve()),
            "truth_samples": alignment["truth_samples"],
            "navigation_samples": alignment["navigation_samples"],
            "paired_samples": paired["paired_samples"],
        } if clock_verified else None,
        "apc_lever_arm_calibrated": False,
        "motion_segment_ready": False,
        "blocking_reasons": [
            *([] if clock_verified else ["Navigation and truth lack a verified common sample clock"]),
            "External vehicle CG-to-SAR-APC lever arm and frame origin are not calibrated",
            *([] if px4 or clock_verified else ["SITL MAVLink SIMSTATE truth lacks its own simulation timestamp"]),
        ],
        "interpretation": ("Original stream rates above; separately exported data share a verified simulation clock, "
                           "while interpolation and APC limits remain" if clock_verified else
                           "Measured sample counts and source-clock intervals only; no SAR-ready pose pair is asserted"),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=("px4", "sitl"), required=True)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.directory, args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(args.output)


if __name__ == "__main__":
    main()
