"""把已同钟的外部物理真值取到导航时刻并量化配对差。"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

FIELDS = (
    "cg_x_sar_m", "cg_y_sar_m", "cg_z_sar_m",
    "cg_vx_sar_mps", "cg_vy_sar_mps", "cg_vz_sar_mps",
)
Q_FIELDS = ("qx_nb", "qy_nb", "qz_nb", "qw_nb")


def validate(directory: Path, truth_filename: str) -> dict:
    """输出 100 Hz 时刻真值与同钟残差；插值不增加独立物理观测。"""
    truth = np.genfromtxt(directory / truth_filename, delimiter=",", names=True, encoding="utf-8")
    nav = np.genfromtxt(directory / "navigation_100hz.csv", delimiter=",", names=True, encoding="utf-8")
    truth_time, nav_time = truth["time_s"], nav["time_s"]
    if (len(truth_time) < 2 or len(nav_time) < 2
            or np.any(np.diff(truth_time) <= 0) or np.any(np.diff(nav_time) <= 0)):
        raise ValueError("Time must strictly increase")
    if abs(nav_time[0]) > 1e-8 or not np.allclose(np.diff(nav_time), 0.01, rtol=0, atol=1e-7):
        raise ValueError("Navigation must use a zero-origin 100 Hz grid")
    right = np.searchsorted(truth_time, nav_time)
    if np.any(right == len(truth_time)) or np.any(nav_time < truth_time[0] - 1e-8):
        raise ValueError("Navigation extends outside truth coverage")
    exact = np.abs(truth_time[right] - nav_time) < 1e-8
    gaps = np.where(exact, 0.0, truth_time[right] - truth_time[np.maximum(right - 1, 0)])
    if np.max(gaps) > 0.005:
        raise ValueError("Physical truth gap exceeds 5 ms at navigation sample")
    truth_fields = np.column_stack([np.interp(nav_time, truth_time, truth[field]) for field in FIELDS])
    nav_fields = np.column_stack([nav[field] for field in FIELDS])
    truth_rotation = Slerp(truth_time, Rotation.from_quat(
        np.column_stack([truth[field] for field in Q_FIELDS])))(nav_time)
    nav_rotation = Rotation.from_quat(np.column_stack([nav[field] for field in Q_FIELDS]))
    truth_quaternion = truth_rotation.as_quat()
    values = np.column_stack((nav_time, truth_fields, truth_quaternion[:, [3, 0, 1, 2]]))
    path = directory / "truth_at_navigation_100hz.csv"
    report_path = directory / "pair_validation.json"
    if path.exists() or report_path.exists():
        raise FileExistsError("Refusing to overwrite paired data")
    header = "time_s," + ",".join(FIELDS) + ",qw_nb,qx_nb,qy_nb,qz_nb"
    np.savetxt(path, values, delimiter=",", header=header, comments="", fmt="%.16g")
    difference = nav_fields - truth_fields
    attitude_error = np.rad2deg((nav_rotation.inv() * truth_rotation).magnitude())
    report = {
        "status": "shared_simulation_clock_paired",
        "truth_source": truth_filename,
        "navigation_source": "navigation_100hz.csv",
        "paired_samples": len(nav_time),
        "exact_truth_time_matches": int(np.count_nonzero(exact)),
        "maximum_truth_interpolation_gap_s": float(np.max(gaps)),
        "position_nav_minus_truth_mean_sar_m": np.mean(difference[:, :3], axis=0).tolist(),
        "position_nav_minus_truth_rms_sar_m": np.sqrt(np.mean(difference[:, :3] ** 2, axis=0)).tolist(),
        "velocity_nav_minus_truth_rms_sar_mps": np.sqrt(np.mean(difference[:, 3:] ** 2, axis=0)).tolist(),
        "attitude_nav_minus_truth_rms_deg": float(np.sqrt(np.mean(attitude_error**2))),
        "position_difference_first_sample_sar_m": difference[0, :3].tolist(),
        "position_first_sample_offset_corrected_rms_sar_m": np.sqrt(
            np.mean((difference[:, :3] - difference[0, :3]) ** 2, axis=0)).tolist(),
        "interpretation": (
            "Clock pairing verified; position offset, estimator delay and model error remain in residuals. "
            + ("Gazebo truth velocity is a finite difference. " if truth_filename == "truth_1000hz.csv"
               else "SITL truth velocity is native SIM2. ")
            + "This is not calibrated real-flight accuracy."
        ),
        "paired_truth_file": path.name,
    }
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--truth", required=True)
    args = parser.parse_args()
    print(json.dumps(validate(args.directory, args.truth), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
