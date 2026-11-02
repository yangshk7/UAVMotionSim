"""用 PX4 ULog 原始仿真时钟对齐 Gazebo 真值和 100 Hz 导航位姿。"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def _csv(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8", ndmin=1)
    if len(data) < 2:
        raise ValueError(f"Too few rows in {path}")
    return data


def _nearest_route_time(route_row, nav: np.ndarray) -> tuple[float, float]:
    target = np.array([route_row["x_sar_m"], route_row["y_sar_m"], route_row["z_sar_m"]])
    xyz = np.column_stack((nav["x"], nav["y"], -nav["z"]))
    distance = np.linalg.norm(xyz - target, axis=1)
    index = int(np.argmin(distance))
    return float(nav["timestamp_sample"][index]) / 1e6, float(distance[index])


def _bracket_gaps(source_time: np.ndarray, target_time: np.ndarray) -> tuple[np.ndarray, float]:
    if np.any(np.diff(source_time) <= 0):
        raise ValueError("Source sample time is not strictly increasing")
    right = np.searchsorted(source_time, target_time, side="left")
    if np.any(right == 0) or np.any(right == len(source_time)):
        raise ValueError("Target time lies outside source sample coverage")
    left = right - 1
    exact = source_time[right] == target_time
    gap = np.where(exact, 0.0, source_time[right] - source_time[left])
    return right, float(np.max(gap))


def _interpolate_columns(data: np.ndarray, time: np.ndarray, fields: tuple[str, ...]) -> np.ndarray:
    source_time = data["timestamp_sample"].astype(np.float64) / 1e6
    return np.column_stack([np.interp(time, source_time, data[name]) for name in fields])


def align(directory: Path, output: Path) -> dict:
    """匹配同一 Gazebo/PX4 仿真时钟；单位 s、m、m/s，端点含首末样本。"""
    if not output.is_dir():
        raise ValueError("Existing output directory required")
    targets = [output / name for name in ("truth_1000hz.csv", "navigation_100hz.csv", "alignment.json")]
    if any(path.exists() for path in targets):
        raise FileExistsError("Refusing to overwrite an aligned trajectory")
    gazebo = _csv(directory / "gazebo_truth_enu.csv")
    route = _csv(directory / "route_navigation_sar.csv")
    nav = _csv(output / "ulog_vehicle_local_position.csv")
    attitude = _csv(output / "ulog_vehicle_attitude.csv")
    groundtruth = _csv(output / "ulog_vehicle_local_position_groundtruth.csv")
    groundtruth_attitude = _csv(output / "ulog_vehicle_attitude_groundtruth.csv")
    start_raw, start_error = _nearest_route_time(route[0], nav)
    end_raw, end_error = _nearest_route_time(route[-1], nav)
    if max(start_error, end_error) > 1e-3 or end_raw - start_raw < 1:
        raise ValueError("Route endpoints cannot be identified in raw PX4 navigation")
    start = math.ceil((start_raw - 1e-9) * 100) / 100
    end = math.floor((end_raw + 1e-9) * 100) / 100
    count = round((end - start) * 100) + 1
    nav_time = start + np.arange(count, dtype=np.float64) / 100
    truth_time = start + np.arange((count - 1) * 10 + 1, dtype=np.float64) / 1000

    gazebo_time = gazebo["sim_time_s"]
    truth_index = np.searchsorted(gazebo_time, truth_time)
    if (np.any(truth_index >= len(gazebo_time))
            or np.max(np.abs(gazebo_time[truth_index] - truth_time)) > 1e-8):
        raise ValueError("Gazebo 1000 Hz truth has missing simulation-time samples")
    truth_position = np.column_stack((gazebo["y_enu_m"][truth_index], gazebo["x_enu_m"][truth_index],
                                      gazebo["z_up_m"][truth_index]))
    truth_velocity = np.gradient(truth_position, 0.001, axis=0, edge_order=2)
    gazebo_q = np.column_stack([gazebo[key][truth_index] for key in ("qx", "qy", "qz", "qw")])
    ned_from_enu = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    flu_from_frd = np.diag([1, -1, -1])
    truth_rotation = Rotation.from_matrix(ned_from_enu @ Rotation.from_quat(gazebo_q).as_matrix()
                                          @ flu_from_frd)
    truth_q_xyzw = truth_rotation.as_quat()

    valid = (nav["xy_valid"] > 0) & (nav["z_valid"] > 0)
    nav = nav[valid]
    pos_time = nav["timestamp_sample"].astype(np.float64) / 1e6
    attitude_time = attitude["timestamp_sample"].astype(np.float64) / 1e6
    _, position_max_gap = _bracket_gaps(pos_time, nav_time)
    _, attitude_max_gap = _bracket_gaps(attitude_time, nav_time)
    if position_max_gap > 0.04 or attitude_max_gap > 0.02:
        raise ValueError("Navigation source has an excessive sample gap")
    nav_state = _interpolate_columns(nav, nav_time, ("x", "y", "z", "vx", "vy", "vz"))
    nav_position = nav_state[:, :3] * [1, 1, -1]
    nav_velocity = nav_state[:, 3:] * [1, 1, -1]
    attitude_q = np.column_stack([attitude[key] for key in ("q1", "q2", "q3", "q0")])
    nav_rotation = Slerp(attitude_time, Rotation.from_quat(attitude_q))(nav_time)
    nav_q_xyzw = nav_rotation.as_quat()

    gt_time = groundtruth["timestamp_sample"].astype(np.float64) / 1e6
    gt_mask = (gt_time >= start) & (gt_time <= end)
    gt_index = np.searchsorted(gazebo_time, gt_time[gt_mask])
    if (not np.any(gt_mask) or np.any(gt_index >= len(gazebo_time))
            or np.max(np.abs(gazebo_time[gt_index] - gt_time[gt_mask])) > 1e-9):
        raise ValueError("PX4 internal truth and Gazebo do not share exact simulation timestamps")
    gt_position = np.column_stack((groundtruth["x"][gt_mask], groundtruth["y"][gt_mask],
                                    -groundtruth["z"][gt_mask]))
    gz_position = np.column_stack((gazebo["y_enu_m"][gt_index], gazebo["x_enu_m"][gt_index],
                                    gazebo["z_up_m"][gt_index]))
    gt_difference = gt_position - gz_position
    gt_max_m = np.max(np.abs(gt_difference), axis=0)
    if np.max(gt_max_m) > 1e-3:
        raise ValueError("PX4 internal truth disagrees with Gazebo at the same simulation time")
    gt_att_time = groundtruth_attitude["timestamp_sample"].astype(np.float64) / 1e6
    gt_att_mask = (gt_att_time >= start) & (gt_att_time <= end)
    gt_att_index = np.searchsorted(gazebo_time, gt_att_time[gt_att_mask])
    if (not np.any(gt_att_mask) or np.any(gt_att_index >= len(gazebo_time))
            or np.max(np.abs(gazebo_time[gt_att_index] - gt_att_time[gt_att_mask])) > 1e-9):
        raise ValueError("PX4 internal attitude truth lacks exact Gazebo timestamps")
    gt_q = np.column_stack([groundtruth_attitude[key][gt_att_mask]
                            for key in ("q1", "q2", "q3", "q0")])
    gz_q = Rotation.from_matrix(ned_from_enu @ Rotation.from_quat(np.column_stack([
        gazebo[key][gt_att_index] for key in ("qx", "qy", "qz", "qw")
    ])).as_matrix() @ flu_from_frd)
    attitude_error_deg = np.rad2deg((Rotation.from_quat(gt_q).inv() * gz_q).magnitude())
    if np.max(attitude_error_deg) > 0.1:
        raise ValueError("PX4 internal attitude truth disagrees with Gazebo")

    truth_values = np.column_stack((truth_time - start, truth_position, truth_velocity,
                                    truth_q_xyzw[:, [3, 0, 1, 2]]))
    nav_values = np.column_stack((nav_time - start, nav_position, nav_velocity,
                                  nav_q_xyzw[:, [3, 0, 1, 2]]))
    header = "time_s,cg_x_sar_m,cg_y_sar_m,cg_z_sar_m,cg_vx_sar_mps,cg_vy_sar_mps,cg_vz_sar_mps,qw_nb,qx_nb,qy_nb,qz_nb"
    report = {
        "clock_alignment": "exact_shared_gazebo_simulation_clock",
        "clock_evidence": "PX4 GZBridge sets PX4 CLOCK_MONOTONIC from Gazebo /clock; raw ULog timestamp_sample matches Gazebo sim_time",
        "truth_source": "Gazebo PosePublisher native 1000 Hz",
        "navigation_source": "PX4 ULog vehicle_local_position and vehicle_attitude, resampled to 100 Hz",
        "truth_velocity_source": "central finite difference of 1 ms Gazebo position, not native Gazebo velocity",
        "coordinate_frame": "SAR X=ENU Y, Y=ENU X, Z=ENU Z; q body FRD to local NED",
        "apc_status": "not applied; CG-to-APC lever arm remains uncalibrated",
        "route_raw_match_error_m": [start_error, end_error],
        "simulation_time_window_s": [start, end],
        "truth_samples": len(truth_time),
        "navigation_samples": len(nav_time),
        "truth_rate_hz": 1000,
        "navigation_rate_hz": 100,
        "navigation_position_max_interpolation_gap_s": position_max_gap,
        "navigation_attitude_max_interpolation_gap_s": attitude_max_gap,
        "independent_px4_groundtruth_matches": int(np.sum(gt_mask)),
        "groundtruth_position_max_abs_difference_m": gt_max_m.tolist(),
        "groundtruth_position_rms_difference_m": np.sqrt(np.mean(gt_difference**2, axis=0)).tolist(),
        "independent_px4_attitude_matches": int(np.sum(gt_att_mask)),
        "groundtruth_attitude_max_difference_deg": float(np.max(attitude_error_deg)),
        "files": {"truth": targets[0].name, "navigation": targets[1].name},
    }
    np.savetxt(targets[0], truth_values, delimiter=",", header=header, comments="", fmt="%.16g")
    np.savetxt(targets[1], nav_values, delimiter=",", header=header, comments="", fmt="%.16g")
    targets[2].write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = align(args.directory, args.output)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
