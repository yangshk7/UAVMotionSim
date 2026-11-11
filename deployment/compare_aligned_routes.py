"""Compare external baseline captures on the shared along-track interval."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def rms(values):
    return np.sqrt(np.mean(values**2, axis=0))


def load_capture(directory: Path, truth_name: str):
    validation = json.loads((directory.parent / "validation.json").read_text(encoding="utf-8"))
    if validation["passed"] is not True or not validation.get("aligned_route"):
        raise ValueError(f"No passed aligned-route validation: {directory.parent}")
    truth = np.genfromtxt(directory / truth_name, names=True, delimiter=",")
    navigation = np.genfromtxt(directory / "navigation_100hz.csv", names=True, delimiter=",")
    position = np.column_stack([truth[f"cg_{axis}_sar_m"] for axis in "xyz"])
    quaternion = np.column_stack([truth[f"q{axis}_nb"] for axis in "wxyz"])
    time = truth["time_s"]
    endpoints = navigation["time_s"][[0, -1]]
    if (not np.isfinite(position).all() or not np.isfinite(time).all()
            or np.any(np.diff(time) <= 0) or np.any(np.diff(position[:, 0]) <= 0)
            or endpoints[0] < time[0] - 1e-9 or endpoints[1] > time[-1] + 1e-9):
        raise ValueError(f"Invalid or non-monotonic capture: {directory}")
    if abs(np.diff(endpoints).item() - validation["aligned_route"]["duration_s"]) > 0.1:
        raise ValueError(f"Native simulation capture duration differs from the target by over 0.1 s: {directory}")
    return validation, truth, position, quaternion, np.interp(endpoints, time, position[:, 0])


def compare(sitl: Path, px4: Path):
    captures = [load_capture(sitl, "truth_native.csv"), load_capture(px4, "truth_1000hz.csv")]
    profile = captures[0][0]["aligned_route"]
    if profile != captures[1][0]["aligned_route"]:
        raise ValueError("The two runs have different aligned-route profiles")
    lower = max(capture[4][0] for capture in captures)
    upper = min(capture[4][1] for capture in captures)
    # Preserve the original 180 m gate for the 204.8 m baseline; scale only short captures.
    minimum_coverage = min(180.0, (180.0/204.8)*profile["duration_s"]*profile["speed_mps"])
    if upper - lower < minimum_coverage:
        raise ValueError(f"Common physical capture coverage is shorter than {minimum_coverage:g} m")
    step = 0.1
    grid = lower + step * np.arange(math.floor((upper - lower) / step) + 1)
    results = []
    series = []
    for directory, (validation, truth, position, quaternion, _) in zip((sitl, px4), captures, strict=True):
        timing = json.loads((directory / "alignment.json").read_text(encoding="utf-8"))
        start, end = timing["simulation_time_window_s"]
        x = position[:, 0]
        yz = np.column_stack([np.interp(grid, x, column) for column in position[:, 1:].T])
        speed = np.interp(grid, x, truth["cg_vx_sar_mps"])
        attitude = Slerp(x, Rotation.from_quat(quaternion, scalar_first=True))(grid)
        radius = np.linalg.norm(yz - [profile.get("center_y_m", 0), profile["altitude_m"]], axis=1)
        results.append({
            "directory": str(directory.resolve()),
            "capture_start_nav_ned": validation["capture_start_nav_ned"],
            "aligned_native_duration_s": end - start,
            "online_route_duration_s": validation["route_duration_s"],
            "route_x_span_m": validation["route_x_span_m"],
            "truth_y_rms_m": float(rms(yz[:, 0]-profile.get("center_y_m", 0))),
            "truth_height_mean_m": float(yz[:, 1].mean()),
            "truth_height_std_m": float(yz[:, 1].std()),
            "truth_height_error_rms_m": float(rms(yz[:, 1] - profile["altitude_m"])),
            "truth_along_speed_error_rms_mps": float(rms(speed - profile["speed_mps"])),
            "truth_rpy_mean_deg": attitude.as_euler("xyz", degrees=True).mean(axis=0).tolist(),
            "tube_d95_m": float(2 * np.sort(radius)[math.ceil(0.95 * len(radius)) - 1]),
            "tube_max_m": float(2 * radius.max()),
        })
        series.append((yz, speed, attitude))
    (sy, sv, sq), (py, pv, pq) = series
    return {
        "profile": profile,
        "comparison": "Absolute SAR X grid, native physical position; Y/Z left in the original local frames",
        "common_x_window_m": [float(lower), float(upper)],
        "grid_step_m": step,
        "grid_samples": len(grid),
        "sitl": results[0],
        "px4": results[1],
        "px4_minus_sitl_yz_mean_m": (py - sy).mean(axis=0).tolist(),
        "cross_platform_yz_rms_m": rms(py - sy).tolist(),
        "cross_platform_along_speed_rms_mps": float(rms(pv - sv)),
        "cross_platform_attitude_angle_rms_deg": float(np.rad2deg(rms((sq.inv() * pq).magnitude()))),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sitl", type=Path, required=True)
    parser.add_argument("--px4", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.sitl, args.px4)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(args.output)
