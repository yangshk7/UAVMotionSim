"""Evaluate NTU VIRAL FAST-LIO2 positions against independent Leica prism truth.

Inputs are the official eee_01 odometry and ground_truth CSVs. Times are ROS
nanoseconds. Position is metres. Body-to-prism offset is from the NTU tutorial.
Rigid alignment is SE(3), without scale. Ground-truth orientation is unavailable.
"""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import font_manager
from scipy.spatial.transform import Rotation

BODY_TO_PRISM_M = np.array([-0.293656, -0.012288, -0.273095])


def plot_evaluation(time_ns: np.ndarray, estimate: np.ndarray, truth: np.ndarray, path: Path) -> None:
    """Save a Chinese-labeled real-data trajectory and 3D-position error plot."""
    fonts = {font.name for font in font_manager.fontManager.ttflist}
    plt.rcParams["font.sans-serif"] = [name for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC")
                                        if name in fonts] or ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    seconds = (time_ns - time_ns[0]) * 1e-9
    distance = np.linalg.norm(estimate - truth, axis=1)
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    axes[0].plot(truth[:, 0], truth[:, 1], label="激光跟踪真值", linewidth=1.4)
    axes[0].plot(estimate[:, 0], estimate[:, 1], label="FAST-LIO2 估计", linewidth=0.9)
    axes[0].set(xlabel="对齐坐标 X（米）", ylabel="对齐坐标 Y（米）", title="实飞水平航迹")
    axes[0].axis("equal")
    axes[0].legend()
    axes[1].plot(seconds, distance, linewidth=0.8)
    axes[1].set(xlabel="配对后时间（秒）", ylabel="三维位置差（米）", title="逐点位置误差")
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.savefig(path, dpi=160)
    plt.close(figure)


def read_pose(path: Path, *, estimate: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """Read ROS CSV: timestamp ns, xyz m, and optional xyzw body orientation."""
    frame = pd.read_csv(path)
    prefix = "field.pose.pose." if estimate else "field.pose."
    columns = [prefix + f"position.{axis}" for axis in "xyz"]
    if not {"%time", *columns}.issubset(frame.columns):
        raise ValueError(f"missing time or position columns in {path}")
    timestamp_ns = frame["%time"].to_numpy(dtype=np.int64)
    position = frame[columns].to_numpy(dtype=float)
    if len(timestamp_ns) < 100 or np.any(np.diff(timestamp_ns) <= 0) or not np.isfinite(position).all():
        raise ValueError(f"invalid timestamps or positions in {path}")
    quaternion = None
    if estimate:
        q_columns = [prefix + f"orientation.{axis}" for axis in "xyzw"]
        quaternion = frame[q_columns].to_numpy(dtype=float)
        norm = np.linalg.norm(quaternion, axis=1)
        if not np.isfinite(quaternion).all() or np.any(abs(norm - 1) > 0.01):
            raise ValueError("invalid estimate quaternions")
    return timestamp_ns, position, quaternion


def paired_positions(
    truth_time: np.ndarray, truth_position: np.ndarray,
    estimate_time: np.ndarray, estimate_prism: np.ndarray,
    max_bracket_s: float = 0.1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate truth only within two bracketing samples each <= max_bracket_s away."""
    right = np.searchsorted(truth_time, estimate_time, side="right")
    inside = (right > 0) & (right < len(truth_time))
    safe_right = np.clip(right, 1, len(truth_time) - 1)
    left = safe_right - 1
    before = (estimate_time - truth_time[left]) * 1e-9
    after = (truth_time[safe_right] - estimate_time) * 1e-9
    matched = inside & (before >= 0) & (after >= 0) & (before <= max_bracket_s) & (after <= max_bracket_s)
    if np.count_nonzero(matched) < 100:
        raise ValueError("fewer than 100 samples with two truth timestamps within tolerance")
    fraction = (estimate_time[matched] - truth_time[left[matched]]) / (
        truth_time[safe_right[matched]] - truth_time[left[matched]]
    )
    interpolated = (truth_position[left[matched]] * (1 - fraction[:, None])
                    + truth_position[safe_right[matched]] * fraction[:, None])
    return estimate_time[matched], estimate_prism[matched], interpolated


def rigid_alignment(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares proper rotation and translation mapping source xyz to target xyz."""
    source_mean, target_mean = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_mean).T @ (target - target_mean))
    correction = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
    rotation = u @ correction @ vt
    translation = target_mean - source_mean @ rotation
    return rotation, translation


def metrics(estimate: np.ndarray, truth: np.ndarray) -> dict:
    residual = estimate - truth
    distance = np.linalg.norm(residual, axis=1)
    return {
        "samples": len(distance),
        "axis_rmse_m": np.sqrt(np.mean(residual**2, axis=0)).tolist(),
        "ate_rms_m": float(np.sqrt(np.mean(distance**2))),
        "median_3d_error_m": float(np.median(distance)),
        "p95_3d_error_m": float(np.quantile(distance, 0.95)),
        "max_3d_error_m": float(np.max(distance)),
    }


def evaluate(truth_csv: Path, estimate_csv: Path, aligned_csv: Path | None = None) -> dict:
    truth_time, truth_position, _ = read_pose(truth_csv, estimate=False)
    estimate_time, body_position, quaternion = read_pose(estimate_csv, estimate=True)
    prism_estimate = body_position + Rotation.from_quat(quaternion).apply(BODY_TO_PRISM_M)
    matched_time, estimate, truth = paired_positions(truth_time, truth_position, estimate_time, prism_estimate)
    rotation, translation = rigid_alignment(estimate, truth)
    aligned = estimate @ rotation + translation
    midpoint = len(estimate) // 2
    first_rotation, first_translation = rigid_alignment(estimate[:midpoint], truth[:midpoint])
    second_prediction = estimate[midpoint:] @ first_rotation + first_translation
    if aligned_csv:
        frame = pd.DataFrame({"time_ns": matched_time})
        for label, values in (("estimate", aligned), ("truth", truth), ("error", aligned - truth)):
            for index, axis in enumerate("xyz"):
                frame[f"{label}_{axis}_m"] = values[:, index]
        frame.to_csv(aligned_csv, index=False)
        plot_evaluation(matched_time, aligned, truth, aligned_csv.parent / "trajectory_error.png")
    return {
        "dataset": "NTU VIRAL eee_01; official FAST-LIO2 sample vs Leica tracked prism",
        "truth_csv": str(truth_csv.resolve()),
        "truth_sha256": hashlib.sha256(truth_csv.read_bytes()).hexdigest(),
        "estimate_csv": str(estimate_csv.resolve()),
        "estimate_sha256": hashlib.sha256(estimate_csv.read_bytes()).hexdigest(),
        "truth_samples": len(truth_time),
        "estimate_samples": len(estimate_time),
        "matched_samples": len(matched_time),
        "matched_duration_s": float((matched_time[-1] - matched_time[0]) * 1e-9),
        "body_to_prism_m": BODY_TO_PRISM_M.tolist(),
        "match_rule": "linear truth interpolation; both bracketing timestamps within 0.1 s; no extrapolation",
        "alignment": "proper 3D rigid rotation and translation, no scale",
        "full_sequence_aligned": metrics(aligned, truth),
        "first_half_alignment_second_half_test": metrics(second_prediction, truth[midpoint:]),
        "attitude_error": None,
        "attitude_note": "Leica truth CSV orientation is an identity placeholder, not a measured orientation",
        "interpretation": "real-flight method check for this other aircraft and FAST-LIO2; not Hex-X12 accuracy",
        "plot": "trajectory_error.png" if aligned_csv else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("truth_csv", type=Path)
    parser.add_argument("estimate_csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    aligned = args.output.parent / "aligned_positions.csv"
    result = evaluate(args.truth_csv, args.estimate_csv, aligned)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
